"""Eyes Studio HTTP server.

Serves the static page in ./static and a small JSON API over the stdlib
http.server, so it runs on a robot with no extra packages:

    GET  /api/capabilities   eyes.capabilities() plus 'backend'
    GET  /api/animations     eyes.animations()
    GET  /api/state          eyes.state() plus what this server holds ('studio')
    POST /api/eyes           {left, right, color, intensity}, any subset
    POST /api/off            eyes.off()
    POST /api/idle           eyes.idle()
    POST /api/control        {take: bool}
    POST /api/sentry         {active: true}, resume the sentry after a failed release
    POST /api/fake           --fake only: {runstop, battery_soc, sentry_active, drop,
                             lease_holder, sentry_refuse, protocol}

The /api/fake hook sets the fake backend's robot state so the override, sentry,
dropped-command and protocol paths can be exercised without a robot.

All writes go through one worker thread so they reach the PIMU in the order
they were made. Consecutive /api/eyes writes are merged (latest value per
field wins) and sent at most WRITE_HZ times a second, so dragging a colour
wheel or a slider does not flood the PIMU link.

State fields the page relies on but never requires: left and right may be None
(not commanded through this API yet), source may be 'assumed', 'commanded' or
'dropped' (stretch_body_server dropped the push because another client held
its lease; lease_holder names it), capabilities may carry requires_protocol and
a protocol_version of None.

A POST must carry Content-Type application/json and, when the browser adds an
Origin, one that matches the Host header, so a page from another site on the LAN
cannot drive the eyes through the operator's browser. There is no other
authentication.

Ctrl-C, SIGTERM and SIGHUP all release control, resume the sentry and close the
client before exit. kill -9 cannot be caught: after one, run
stretch_eye_animations --release.
"""

import argparse
import collections
import dataclasses
import enum
import json
import os
import signal
import socket
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

# 8765 is the foxglove_bridge default, which a ROS robot may already run.
DEFAULT_PORT = 8770
WRITE_HZ = 20
MAX_BODY = 4096
WRITE_TIMEOUT_S = 5.0   # a release may wait CLOSE_RELEASE_TIMEOUT_S for the sentry to resume

STATIC_DIR = Path(__file__).resolve().parent / 'static'
CONTENT_TYPES = {
    '.html': 'text/html; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.svg': 'image/svg+xml',
    '.png': 'image/png',
}

FAKE_FIELDS = {
    'runstop': (bool,),
    'battery_soc': (int, float),
    'sentry_active': (bool,),
    'drop': (bool,),
    'lease_holder': (str, type(None)),
    'sentry_refuse': (bool,),
    'protocol': (str, type(None)),
}


def to_jsonable(obj):
    """Turn API return values (dataclasses, named tuples, enums) into JSON types."""
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return {f.name: to_jsonable(getattr(obj, f.name)) for f in dataclasses.fields(obj)}
    if isinstance(obj, enum.Enum):
        return obj.name.lower()
    if isinstance(obj, tuple) and hasattr(obj, '_asdict'):
        return {k: to_jsonable(v) for k, v in obj._asdict().items()}
    if isinstance(obj, dict):
        return {str(k): to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, set)):
        return [to_jsonable(v) for v in obj]
    if obj is None or isinstance(obj, (bool, int, float, str)):
        return obj
    return str(obj)


class BadRequest(ValueError):
    pass


def make_fake_backend():
    """
    The API's FakeBackend plus the knobs /api/fake needs: a dropped push (send returns
    False and lease_holder names who had the lease), a refused sentry resume, and a
    protocol override (None stands for a server that does not publish it).
    """
    from stretch4_body.eyes.backends import FakeBackend

    class StudioFakeBackend(FakeBackend):
        def __init__(self):
            super().__init__()
            self.drop = False
            self.lease_holder_name = 'stretch_gamepad_teleop'
            self.sentry_refuse = False
            self.protocol = 'p13'

        def send(self, payload):
            if self.drop:
                return False
            return super().send(payload)

        def lease_holder(self):
            return self.lease_holder_name if self.drop or self.sentry_refuse else super().lease_holder()

        def set_sentry(self, active, *args, **kwargs):
            if self.sentry_refuse:
                return False
            return super().set_sentry(active, *args, **kwargs)

        def protocol_version(self):
            return self.protocol

    return StudioFakeBackend()


class _Op:
    __slots__ = ('kind', 'kwargs', 'done', 'error')

    def __init__(self, kind, kwargs=None):
        self.kind = kind
        self.kwargs = kwargs or {}
        self.done = threading.Event()
        self.error = None


class EyeWriter:
    """Single writer thread in front of an Eyes object.

    Calls run in submission order. A 'set' op still waiting in the queue
    absorbs later set fields, and set ops are spaced by 1/hz seconds.
    """

    def __init__(self, eyes, hz=WRITE_HZ):
        self.eyes = eyes
        self.io = threading.Lock()  # every call into eyes holds this
        self.period = 1.0 / hz
        self.has_control = False
        self.sentry_resume_failed = False   # release() returned False: the sentry stays paused
        self._control = None
        self._queue = collections.deque()
        self._cv = threading.Condition()
        self._last_set = 0.0
        self.writes = 0  # set calls that reached the backend
        threading.Thread(target=self._run, name='eyes-writer', daemon=True).start()

    def submit_set(self, fields):
        with self._cv:
            if self._queue and self._queue[-1].kind == 'set':
                op = self._queue[-1]
                op.kwargs.update(fields)
            else:
                op = _Op('set', dict(fields))
                self._queue.append(op)
            self._cv.notify()
        return op

    def submit(self, kind):
        op = _Op(kind)
        with self._cv:
            self._queue.append(op)
            self._cv.notify()
        return op

    def _run(self):
        while True:
            with self._cv:
                self._cv.wait_for(lambda: self._queue)
                op = self._queue[0]
                if op.kind == 'set':
                    # Leave the op queued while waiting so later writes merge into it.
                    delay = self._last_set + self.period - time.monotonic()
                    if delay > 0:
                        self._cv.wait(delay)
                        continue
                self._queue.popleft()
            try:
                with self.io:
                    self._apply(op)
            except Exception as exc:  # reported to the waiting request
                op.error = exc
            if op.kind == 'set':
                self._last_set = time.monotonic()
            op.done.set()

    def _apply(self, op):
        eyes = self.eyes
        if op.kind == 'set':
            eyes.set(**op.kwargs)
            self.writes += 1
        elif op.kind == 'off':
            eyes.off()
        elif op.kind == 'idle':
            eyes.idle()
        elif op.kind == 'take':
            if not self.has_control:
                self._control = eyes.take_control()
                self.has_control = True
        elif op.kind == 'release':
            if self.has_control:
                ok = eyes.release()
                self._control = None
                self.has_control = False
                # release() returns False when the server refused to resume the sentry.
                self.sentry_resume_failed = ok is False
        elif op.kind == 'resume':
            self.sentry_resume_failed = not eyes.resume_sentry()
            if self.sentry_resume_failed:
                raise RuntimeError('stretch_body_server did not accept the sentry resume; try again once the robot is idle')
        else:
            raise BadRequest(f'unknown op {op.kind}')

    def read(self, fn):
        with self.io:
            return fn()


def _parse_color(value):
    if isinstance(value, str):
        text = value.strip().lstrip('#')
        if len(text) != 6:
            raise BadRequest('color must be #rrggbb')
        try:
            return tuple(int(text[i:i + 2], 16) for i in (0, 2, 4))
        except ValueError:
            raise BadRequest('color must be #rrggbb') from None
    if isinstance(value, (list, tuple)) and len(value) == 3:
        if all(isinstance(c, int) and not isinstance(c, bool) and 0 <= c <= 255 for c in value):
            return tuple(value)
    raise BadRequest('color must be #rrggbb or [r, g, b] with 0..255')


class Studio:
    """Request validation and JSON shaping around an EyeWriter."""

    def __init__(self, eyes, backend_name):
        self.writer = EyeWriter(eyes)
        self.backend_name = backend_name
        self.animations = list(eyes.animations())
        self._anim_keys = {}
        for a in self.animations:
            self._anim_keys[a.id] = a.id
            self._anim_keys[str(a.id)] = a.id
            self._anim_keys[a.name] = a.id

    def capabilities(self):
        caps = to_jsonable(self.writer.read(self.writer.eyes.capabilities))
        if not isinstance(caps, dict):
            caps = {'raw': caps}
        caps['backend'] = self.backend_name
        return caps

    def animations_json(self):
        return [to_jsonable(a) for a in self.animations]

    def state(self):
        raw = self.writer.read(self.writer.eyes.state)
        state = raw.to_dict() if hasattr(raw, 'to_dict') else to_jsonable(raw)
        if not isinstance(state, dict):
            state = {'raw': state}
        state = to_jsonable(state)
        state['studio'] = {
            'control': self.writer.has_control,
            'sentry_resume_failed': self.writer.sentry_resume_failed,
            'backend': self.backend_name,
            'write_hz': WRITE_HZ,
            'writes': self.writer.writes,
        }
        return state

    def parse_set(self, body):
        if not isinstance(body, dict):
            raise BadRequest('body must be a JSON object')
        unknown = set(body) - {'left', 'right', 'color', 'intensity'}
        if unknown:
            raise BadRequest(f'unknown fields: {", ".join(sorted(unknown))}')
        fields = {}
        for eye in ('left', 'right'):
            if body.get(eye) is not None:
                key = body[eye]
                if isinstance(key, bool) or key not in self._anim_keys:
                    raise BadRequest(f'unknown animation for {eye}: {key!r}')
                fields[eye] = self._anim_keys[key]
        if body.get('color') is not None:
            fields['color'] = _parse_color(body['color'])
        if body.get('intensity') is not None:
            x = body['intensity']
            if isinstance(x, bool) or not isinstance(x, int) or not 0 <= x <= 255:
                raise BadRequest('intensity must be an integer 0..255')
            fields['intensity'] = x
        if not fields:
            raise BadRequest('nothing to set')
        return fields

    def set_fake(self, body):
        backend = self.writer.eyes.backend
        if not isinstance(body, dict) or set(body) - set(FAKE_FIELDS):
            raise BadRequest(f'body takes {", ".join(sorted(FAKE_FIELDS))}')
        for key, value in body.items():
            if not isinstance(value, FAKE_FIELDS[key]) or (isinstance(value, bool) and bool not in FAKE_FIELDS[key]):
                raise BadRequest(f'{key}: wrong type')
        with self.writer.io:
            for key, value in body.items():
                setattr(backend, 'lease_holder_name' if key == 'lease_holder' else key, value)

    def wait(self, op):
        if not op.done.wait(WRITE_TIMEOUT_S):
            raise TimeoutError('eyes backend did not answer in time')
        if op.error is not None:
            raise op.error


def make_handler(studio, verbose=False):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'EyesStudio/1'

        def log_message(self, fmt, *args):
            if verbose:
                super().log_message(fmt, *args)

        def _send(self, status, payload=None, body=None, ctype='application/json'):
            if body is None:
                body = json.dumps(payload).encode()
            try:
                self.send_response(status)
                self.send_header('Content-Type', ctype)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):   # the client hung up first
                pass

        def _error(self, status, message):
            self._send(status, {'error': message})

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            try:
                if path == '/api/capabilities':
                    return self._send(HTTPStatus.OK, studio.capabilities())
                if path == '/api/animations':
                    return self._send(HTTPStatus.OK, studio.animations_json())
                if path == '/api/state':
                    return self._send(HTTPStatus.OK, studio.state())
            except Exception as exc:
                return self._error(HTTPStatus.BAD_GATEWAY, f'{type(exc).__name__}: {exc}')
            self._static(path)

        def _static(self, path):
            if path in ('', '/'):
                path = '/index.html'
            try:
                target = (STATIC_DIR / path.lstrip('/')).resolve()
                found = STATIC_DIR in target.parents and target.is_file()
            except (ValueError, OSError):   # a NUL byte or an over-long name in the path
                found = False
            if not found:
                return self._error(HTTPStatus.NOT_FOUND, 'not found')
            ctype = CONTENT_TYPES.get(target.suffix, 'application/octet-stream')
            self._send(HTTPStatus.OK, body=target.read_bytes(), ctype=ctype)

        def do_POST(self):
            path = self.path.split('?', 1)[0]
            try:
                # Only the page's own fetch sends JSON from a matching Origin. A page on
                # another site can POST text/plain without a preflight, and its preflight
                # for JSON gets 501 here: no do_OPTIONS, no Access-Control headers.
                ctype = self.headers.get('Content-Type', '').split(';')[0].strip().lower()
                if ctype != 'application/json':
                    return self._error(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, 'Content-Type must be application/json')
                origin = self.headers.get('Origin')
                if origin and origin != 'null' and urlsplit(origin).netloc.lower() != (self.headers.get('Host') or '').lower():
                    return self._error(HTTPStatus.FORBIDDEN, 'cross-origin request refused')
                length = int(self.headers.get('Content-Length') or 0)
                if not 0 <= length <= MAX_BODY:
                    raise BadRequest('bad Content-Length')
                raw = self.rfile.read(length) if length else b''
                try:
                    body = json.loads(raw) if raw.strip() else {}
                except json.JSONDecodeError:
                    raise BadRequest('body is not JSON') from None

                if path == '/api/eyes':
                    op = studio.writer.submit_set(studio.parse_set(body))
                elif path == '/api/off':
                    op = studio.writer.submit('off')
                elif path == '/api/idle':
                    op = studio.writer.submit('idle')
                elif path == '/api/fake' and studio.backend_name == 'fake':
                    studio.set_fake(body)
                    return self._send(HTTPStatus.OK, studio.state())
                elif path == '/api/control':
                    if not isinstance(body, dict) or not isinstance(body.get('take'), bool):
                        raise BadRequest('body must be {"take": true|false}')
                    op = studio.writer.submit('take' if body['take'] else 'release')
                elif path == '/api/sentry':
                    if not isinstance(body, dict) or body.get('active') is not True:
                        raise BadRequest('body must be {"active": true}')
                    op = studio.writer.submit('resume')
                else:
                    return self._error(HTTPStatus.NOT_FOUND, 'not found')
                studio.wait(op)
                self._send(HTTPStatus.OK, studio.state())
            except BadRequest as exc:
                self._error(HTTPStatus.BAD_REQUEST, str(exc))
            except (ValueError, TypeError) as exc:  # rejected by the eyes API
                self._error(HTTPStatus.BAD_REQUEST, str(exc))
            except Exception as exc:
                self._error(HTTPStatus.BAD_GATEWAY, f'{type(exc).__name__}: {exc}')

    return Handler


def _stop(signum, frame):
    """SIGTERM and SIGHUP take the Ctrl-C path: release control, resume the sentry, close."""
    raise KeyboardInterrupt


def main(argv=None):
    prog = os.path.basename(sys.argv[0])
    if prog in ('__main__.py', ''):
        prog = 'python3 -m stretch4_body.eyes.studio'
    parser = argparse.ArgumentParser(prog=prog, description='Eyes Studio: a web UI for the Stretch 4 eye LEDs.')
    parser.add_argument('--fake', action='store_true', help='use the simulated backend, no robot hardware')
    parser.add_argument('--host', default='0.0.0.0',
                        help='address to listen on (default 0.0.0.0, reachable from the robot network; '
                             'there is no authentication, so use 127.0.0.1 to keep it to this machine)')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help=f'port (default {DEFAULT_PORT})')
    parser.add_argument('--take-control', action='store_true', help='take eye control at startup (pauses sentry animations)')
    parser.add_argument('--verbose', action='store_true', help='log every HTTP request')
    args = parser.parse_args(argv)

    from stretch4_body.eyes import Eyes

    try:
        eyes = Eyes(backend=make_fake_backend()) if args.fake else Eyes()
    except RuntimeError as exc:
        print(f'eyes: {exc}', file=sys.stderr)
        return 1
    studio = Studio(eyes, 'fake' if args.fake else 'robot')
    # Bind before touching the robot, so a port already in use cannot leave the sentry paused.
    try:
        httpd = ThreadingHTTPServer((args.host, args.port), make_handler(studio, args.verbose))
    except OSError as exc:
        print(f'cannot listen on {args.host}:{args.port}: {exc}', file=sys.stderr)
        eyes.close()
        return 1
    httpd.daemon_threads = True
    shown = args.host
    if shown in ('0.0.0.0', '::'):
        shown = socket.gethostname()
        if '.' not in shown:
            shown += '.local'   # what the laptop resolves; the bare name only works on the robot
    # kill, systemd stop and a dropped ssh session take the Ctrl-C path below. kill -9
    # cannot be caught: after one, run stretch_eye_animations --release.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, _stop)
    try:
        if args.take_control:
            studio.wait(studio.writer.submit('take'))
        print(f'Eyes Studio ({studio.backend_name} backend) on http://{shown}:{httpd.server_address[1]}/', flush=True)
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, TimeoutError) as exc:   # the take: the server did not confirm the pause
        print(f'take control failed: {exc}', file=sys.stderr)
        return 1
    finally:
        # A second Ctrl-C or kill during the release wait must not skip eyes.close().
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        httpd.server_close()
        if studio.writer.has_control:
            try:
                studio.wait(studio.writer.submit('release'))
            except Exception as exc:
                print(f'release failed: {exc}', file=sys.stderr)
        if eyes.close() is False:
            print('sentry_eye_animations is still paused: the server refused the resume. '
                  'Run stretch_eye_animations --release once the robot is idle.', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
