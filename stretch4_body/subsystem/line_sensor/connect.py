#!/usr/bin/env python3
"""One way for every line-sensor tool to reach the sensors.

"""

from __future__ import annotations

import os

SERVER = 'server'
DIRECT = 'direct'

PORT = '/dev/hello-pixart-j3'


class LineSensorUnavailable(Exception):
    """No usable route to the sensors. `detail` is already operator-readable."""

    def __init__(self, reason, detail):
        super().__init__(detail)
        self.reason = reason
        self.detail = detail


class LineSensorConnection:
    """A live route to the sensors, plus how it was obtained.

    `loop` is the only thing a tool normally touches. It is a
    LineSensorLoopClient in SERVER mode and a LineSensorLoop in DIRECT mode;
    both carry params/status/pull_status and the tare accessors, so tool code
    does not branch on mode.
    """

    def __init__(self, mode, loop, closer, puller, detail=''):
        self.mode = mode
        self.loop = loop
        self.detail = detail
        self._closer = closer
        self._puller = puller
        self._closed = False

    @property
    def is_direct(self):
        return self.mode == DIRECT

    def pull_status(self):
        """Refresh `self.loop.status`.

        Not the same call in both modes: through the server the whole robot
        status arrives at once and is fanned out to the subsystems, so it is
        the PARENT that must be pulled, while a direct loop drains its own
        queue. Tools would get this wrong; they should call this instead.
        """
        self._puller()

    def describe(self):
        if self.mode == SERVER:
            return 'reading through the body server'
        return f'reading {PORT} directly (no body server on the sensors)'

    def close(self):
        if self._closed:
            return
        self._closed = True
        try:
            self._closer()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def _port_holder():
    """Who has the port open, best effort. Returns a printable string or ''.

    """
    import subprocess
    try:
        out = subprocess.run(['lsof', '-t', '-w', PORT], capture_output=True,
                             text=True, timeout=3.0).stdout.split()
    except (OSError, subprocess.SubprocessError):
        return ''
    names = []
    for pid in out:
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as fh:
                argv = [a.decode('utf-8', 'replace')
                        for a in fh.read().split(b'\0') if a]
        except OSError:
            argv = []
        who = _argv_name(argv)
        names.append(f'pid {pid} ({who})' if who else f'pid {pid}')
    return ', '.join(names)


def _argv_name(argv):
    if not argv:
        return ''
    first = os.path.basename(argv[0])
    if first.startswith('python') and len(argv) > 1:
        for arg in argv[1:]:
            if not arg.startswith('-'):
                return os.path.basename(arg)
    return first


def _probe_port():
    """Checks if the port is free. """
    try:
        import serial
    except ImportError:
        return True, ''       # no pyserial here; let the reader report it
    if not os.path.exists(PORT):
        return False, (f'{PORT} does not exist -- the pixart board is not '
                       f'connected, or its udev rule has not fired')
    try:
        serial.Serial(port=PORT, exclusive=True).close()
        return True, ''
    except Exception as exc:
        holder = _port_holder()
        return False, (f'{PORT} is already open'
                       + (f' by {holder}' if holder else '')
                       + f' ({exc.__class__.__name__}: {exc})')


def open_line_sensors(client_id, allow_direct=True, verbose=True):
    """Connect by whatever route is available. Raises LineSensorUnavailable.
    """
    from stretch4_body.robot.robot_client import RobotClient

    robot = RobotClient(client_id=client_id)
    try:
        up = robot.startup(verbose=False)
    except Exception:
        up = False
    if up and hasattr(robot, 'line_sensor_loop'):
        return LineSensorConnection(SERVER, robot.line_sensor_loop,
                                    robot.stop, robot.pull_status)

    server_up = bool(up)
    try:
        robot.stop()
    except Exception:
        pass

    if not allow_direct:
        raise LineSensorUnavailable(
            'no_subsystem' if server_up else 'no_server',
            'line_sensor_loop is not enabled on the robot server; add it to '
            'the subsystems list in stretch_user_params.yaml'
            if server_up else
            'the body server is not running')

    free, why = _probe_port()
    if not free:
        raise LineSensorUnavailable('port_busy', why)

    if verbose:
        print(f'body server {"is not serving line_sensor_loop" if server_up else "is not running"}'
              f' -- opening {PORT} directly')

    from stretch4_body.subsystem.line_sensor.line_sensor_loop import LineSensorLoop
    loop = LineSensorLoop()
    if not loop.startup():
        try:
            loop.stop()
        except Exception:
            pass
        free, why = _probe_port()
        raise LineSensorUnavailable(
            'no_frames',
            why if not free else
            f'opened {PORT} but no frames arrived -- check the board firmware '
            f'and that hello-pixart-j3 is streaming')
    _settle(loop)
    return LineSensorConnection(DIRECT, loop, loop.stop, loop.pull_status)


def _settle(loop, timeout_s=1.5):
    """Hold until every sensor has produced a frame NEWER than startup's.
    """
    import time
    names = list(loop.params['sensor_names'])
    start = {n: (loop.status.get(n) or {}).get('frame_id', 0) for n in names}
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        loop.pull_status()
        if all((loop.status.get(n) or {}).get('frame_id', 0) != start[n]
               for n in names):
            return True
        time.sleep(0.002)
    return False
