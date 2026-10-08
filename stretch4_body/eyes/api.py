import atexit
import functools
import logging
import numbers
import threading
import time
import weakref
from dataclasses import dataclass, asdict

from stretch4_body.eyes.animations import ANIMATIONS, find_animation
from stretch4_body.eyes.backends import SENTRY_NAME, NOP_ID, FakeBackend, ServerBackend
from stretch4_body.eyes.colors import parse_color, to_hex

logger = logging.getLogger(__name__)

REQUIRES_PROTOCOL = 'p13'  # PIMU protocol that added intensity and color to RPC 29
PIXELS_PER_EYE = 10        # EYE_NUM_LEDS in EyeAnimationManager.h, LED1-LED10 on S4-RING-LED

# What the firmware shows after a PIMU reset (EyeAnimationManager::setup)
IDLE_ANIMATION = 'idle_glow'
IDLE_COLOR = (40, 48, 60)
IDLE_INTENSITY = 255

# Battery SOC at or below which the firmware replaces the eyes with a half ring in
# yellow, then red (LightBarManager::step)
LOW_SOC_YELLOW = 25
LOW_SOC_RED = 12

# How long to wait for the server to confirm a sentry pause or resume. close() waits
# longer: another client's lease frees 1.1 s after its last push.
SENTRY_CONFIRM_S = 1.0
CLOSE_RESUME_S = 3.0
# How long stop() waits for the playback thread. A step blocked in a server push can
# outlast it; that thread sends no more steps once it gets the lock, only stop()'s restore.
STOP_PLAYBACK_S = 2.0
# A step that went out later than this behind its schedule restarts the schedule, so a
# stall skips the missed steps instead of sending them back to back
PLAY_RESYNC_S = 0.05


@dataclass
class EyeState:
    """
    What this API last sent, plus what the robot is known to show over it.

    The firmware has no readback. left and right are the animations last sent through
    this object, or None for an eye it has not commanded yet: that eye is sent the
    firmware's 'no change' code and keeps whatever it was showing. color and intensity
    are the last values sent; before the first command they hold the firmware boot
    defaults. source is 'assumed' until this object has sent a command, 'commanded'
    after a send the server took, and 'dropped' when the latest published status says
    the server rejected it: a routine was running, or another client held a live command
    lease (lease_holder names that other client, and is None when the lease is free,
    expired or this object's own; see ServerBackend). The values are cached either way,
    so the next accepted send carries them. runstop_active and low_soc_override come from
    the server status (None when unknown); while either is set the firmware shows its own
    pattern, not the commanded one. sentry_active is None on a robot whose params do not
    list sentry_eye_animations.
    """
    left: str
    right: str
    color: tuple
    intensity: int
    source: str
    runstop_active: bool = None
    low_soc_override: str = None   # None, 'yellow' or 'red'
    battery_soc: int = None
    sentry_active: bool = None
    in_control: bool = False
    lease_holder: str = None
    playing: dict = None           # Eyes.playing: None, or the sequence being played

    @property
    def color_hex(self):
        return to_hex(self.color)

    def to_dict(self):
        d = asdict(self)
        d['color'] = list(self.color)
        d['color_hex'] = self.color_hex
        return d


def parse_intensity(value):
    """
    Return intensity as an int 0-255. Ints are raw (0-255); floats are a fraction
    (0.0-1.0), so 1 is raw 1 (nearly off) and 1.0 is full.
    """
    if isinstance(value, bool):
        raise ValueError('Intensity must be an int 0-255 or a float 0.0-1.0, not {!r}'.format(value))
    if isinstance(value, numbers.Integral):
        if 0 <= value <= 255:
            return int(value)
        raise ValueError('Intensity {} is outside 0-255'.format(value))
    if isinstance(value, numbers.Real):
        if 0.0 <= value <= 1.0:
            return int(round(value * 255))
        raise ValueError('Intensity {} is a float, so it must be a fraction 0.0-1.0. '
                         'Pass an int for 0-255.'.format(value))
    raise ValueError('Intensity must be an int 0-255 or a float 0.0-1.0, not {!r}'.format(value))


def _close_at_exit(ref):
    # atexit runs before daemon threads are killed, so close() can still stop playback
    eyes = ref()
    if eyes is not None and (eyes._pause_sent or eyes._control_depth):
        eyes.close()


class _Playback:
    """One play(): the events its thread, stop() and wait() share."""
    def __init__(self, restore_to):
        self.stop = threading.Event()
        self.restore = threading.Event()  # set by stop(restore=True): send restore_to on the way out
        # Set last, after the release. Not Thread.join: a Ctrl-C during a join leaves
        # the thread reported as stopped while it still runs
        self.done = threading.Event()
        self.restore_to = restore_to


class _Control:
    def __init__(self, eyes):
        self._eyes = eyes

    def __enter__(self):
        return self._eyes

    def __exit__(self, exc_type, exc, tb):
        try:
            self._eyes.release()
        except RuntimeError as e:  # the server went away; the block's own outcome stands
            logger.warning('Could not release eye control: %s', e)


class Eyes:
    """
    Stretch 4 eye rings: one animation per eye, one color and intensity shared by both.

    Eyes() talks to the robot through stretch_body_server (see ServerBackend for what the
    server's command lease means for eye pushes). Eyes(backend='fake') needs no robot.
    Pass client= to reuse a RobotClient or PowerPeriphClient you already started. The
    server backend raises RuntimeError when stretch_body_server cannot be reached, at
    construction and from any later call.

    The eye sentry (sentry_eye_animations) replaces the eyes every few seconds while it is
    active. Use take_control() to pause it for as long as your eyes should stay put. The
    pause lives in the server: if stretch_body_server restarts while control is held, the
    sentry comes back active although state().in_control is still True; release() and
    take_control() again once the server is back. A robot whose params do not list the
    sentry (capabilities()['sentry_installed'] False, state().sentry_active None) has
    nothing to pause: take_control() and release() are then bookkeeping only.

    Eye animations, color and intensity all need PIMU protocol p13 (hello-pimu2 v0.1.9p13
    or newer; circle_cw and circle_ccw need v0.1.10p13). A released p12 PIMU has no eye
    RPC at all: the board ignores every push, stretch_body_server logs 'Error
    RPC_REPLY_SET_EYE_ANIMATION' for each one, nothing changes on the rings, and
    sentry_eye_animations disables itself. The server does not publish the PIMU protocol,
    so this object cannot tell: capabilities()['protocol_version'] is None through the
    server and capabilities()['requires_protocol'] is 'p13'; stretch_system_check reports
    the board's version.

    Like RobotClient, an Eyes object and the client it wraps are meant for one thread.
    The methods take a lock so bookkeeping stays consistent, but a client= shared with
    another thread is still that thread's problem; Eyes Studio fronts Eyes with a single
    writer thread for this reason. play() runs its own thread, which sends each step under
    that lock; set(), off(), idle() and close() stop it before they send.
    """
    def __init__(self, backend='server', client=None):
        if backend == 'server':
            self._backend = ServerBackend(client=client)
        elif backend == 'fake':
            self._backend = FakeBackend()
        elif isinstance(backend, str):
            raise ValueError("Unknown backend {!r}. Use 'server' or 'fake'.".format(backend))
        else:
            self._backend = backend
        self._lock = threading.RLock()
        self._left = None         # Animation once commanded; None sends NOP (keep) for that eye
        self._right = None
        self._color = IDLE_COLOR
        self._intensity = IDLE_INTENSITY
        self._source = 'assumed'
        self._control_depth = 0
        self._pause_sent = False  # a sentry pause went out and has not been undone
        self._play_lock = threading.Lock()  # the playback swap in play() and stop(); the play thread never takes it
        self._playback = None
        self._playing = None      # replaced, never mutated, so readers need no lock
        self._backend.connect()
        # A process that exits mid-sequence would otherwise leave the sentry paused
        self._atexit = functools.partial(_close_at_exit, weakref.ref(self))
        atexit.register(self._atexit)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    @property
    def backend(self):
        return self._backend

    def animations(self):
        """All animation presets, in firmware id order."""
        return list(ANIMATIONS)

    def set(self, left=None, right=None, color=None, intensity=None):
        """
        Command the eyes. left and right take an animation name or id; color takes
        (r, g, b), '#rrggbb' or a name; intensity takes an int 0-255 (raw) or a float
        0.0-1.0 (fraction), so 1 is raw 1 (nearly off) and 1.0 is full.

        Anything left as None keeps the value last commanded through this object. An eye
        never commanded through this object is sent as NOP (id 0), which the firmware
        reads as "keep what this eye shows", so eyes.set(color='red') recolors whatever
        animation is running rather than replacing it. Color and intensity cannot be kept
        that way: the firmware applies them on every RPC, so until they are commanded the
        firmware defaults (40, 48, 60) at 255 are sent.

        Returns the new EyeState; its source is 'dropped' when the server rejected the
        push (see ServerBackend). The values are cached either way. A sequence playing
        is stopped first, unless the values are invalid.
        """
        command = self._parse(left, right, color, intensity)
        self.stop(restore=False)
        return self._send(*command)

    @staticmethod
    def _parse(left, right, color, intensity):
        return (None if left is None else find_animation(left),
                None if right is None else find_animation(right),
                None if color is None else parse_color(color),
                None if intensity is None else parse_intensity(intensity))

    def _send(self, left, right, color, intensity, stop_event=None):
        with self._lock:
            if stop_event is not None and stop_event.is_set():
                return None  # stop() came while this step waited for the lock
            left = left or self._left
            right = right or self._right
            color = color or self._color
            intensity = self._intensity if intensity is None else intensity
            accepted = self._backend.send((left.id if left else NOP_ID, right.id if right else NOP_ID, intensity) + color)
            self._left, self._right, self._color, self._intensity = left, right, color, intensity
            if accepted is None or accepted:
                self._source = 'commanded'
            else:
                if self._source != 'dropped':  # once per streak, not once per 20 Hz drag
                    holder = self._backend.lease_holder()
                    logger.warning('Eye command dropped by stretch_body_server: %s',
                                   'lease held by ' + holder if holder else 'a routine is running')
                self._source = 'dropped'
            return self._state_locked()

    def set_color(self, color):
        """(r, g, b), '#rrggbb' or a name from NAMED_COLORS. Both eyes keep their animations."""
        return self.set(color=color)

    def set_intensity(self, intensity):
        """An int 0-255 is the raw byte, a float 0.0-1.0 a fraction: 1 is raw 1 (nearly off), 1.0 is full."""
        return self.set(intensity=intensity)

    def off(self):
        return self.set(left='off', right='off')

    def idle(self):
        """The firmware boot default: idle glow in (40, 48, 60) at full intensity."""
        return self.set(left=IDLE_ANIMATION, right=IDLE_ANIMATION, color=IDLE_COLOR, intensity=IDLE_INTENSITY)

    def state(self):
        with self._lock:
            return self._state_locked()

    def _state_locked(self):
        status = self._backend.robot_status()
        soc = status.get('battery_soc')
        low_soc = None
        if soc is not None:
            if soc <= LOW_SOC_RED:
                low_soc = 'red'
            elif soc <= LOW_SOC_YELLOW:
                low_soc = 'yellow'
        return EyeState(left=self._left.name if self._left else None,
                        right=self._right.name if self._right else None,
                        color=self._color, intensity=self._intensity, source=self._source,
                        runstop_active=status.get('runstop'), low_soc_override=low_soc,
                        battery_soc=soc, sentry_active=self._backend.is_sentry_active(),
                        in_control=self._control_depth > 0,
                        lease_holder=self._backend.lease_holder(),
                        playing=self.playing)

    def capabilities(self):
        """
        What this API offers on this robot. protocol_version is the PIMU protocol when the
        backend can read it, else None (the server does not publish it); requires_protocol
        is the protocol the color and intensity bytes need. sentry_installed says whether
        the robot's params list sentry_eye_animations; sentry_active is None when not.
        """
        with self._lock:
            sentry_active = self._backend.is_sentry_active()
            return {
                'pixels_per_eye': PIXELS_PER_EYE,
                'per_eye_animation': True,
                'per_eye_color': False,   # RPC 29 carries one r, g, b for both eyes; no protocol has a per-eye byte
                'per_pixel': False,
                'readback': False,
                'requires_protocol': REQUIRES_PROTOCOL,
                'protocol_version': self._backend.protocol_version(),
                'sentry_installed': sentry_active is not None,
                'sentry_active': sentry_active,
            }

    def take_control(self):
        """
        Pause the eye sentry so commanded eyes stay put. release() (or leaving the
        with block) puts the sentry back the way it was. Calls nest. Raises RuntimeError
        when the server does not confirm the pause; close() still undoes a pause that
        went out. Nothing is sent when the sentry is inactive or not installed.
        """
        with self._lock:
            if self._control_depth == 0 and self._backend.is_sentry_active():
                # Marked before the push, so close() undoes a pause that was interrupted
                # or never confirmed
                self._pause_sent = True
                if not self._backend.set_sentry(False, timeout=SENTRY_CONFIRM_S):
                    raise RuntimeError('Could not pause ' + SENTRY_NAME)
            self._control_depth += 1
        return _Control(self)

    def release(self, timeout=SENTRY_CONFIRM_S):
        """
        Undo one take_control(). Undoing the last one resumes the sentry if this object
        paused it. Returns False, and logs a warning, when the server rejected the resume
        for the whole wait: sentry_eye_animations then stays paused until resume_sentry()
        or `stretch_eye_animations --release` gets through, once the robot is idle.
        Never raises for that case.
        """
        with self._lock:
            if self._control_depth == 0:
                return True
            self._control_depth -= 1
            if self._control_depth > 0 or not self._pause_sent:
                return True
            return self.resume_sentry(timeout=timeout)

    def resume_sentry(self, timeout=SENTRY_CONFIRM_S):
        """
        Unpause sentry_eye_animations, whoever paused it. Returns whether the server
        confirmed; True at once on a robot without the sentry.
        """
        with self._lock:
            if self._backend.is_sentry_active() is None:  # not on this robot, nothing to resume
                self._pause_sent = False
                return True
            if self._backend.set_sentry(True, timeout=timeout):
                self._pause_sent = False
                return True
            holder = self._backend.lease_holder()
            if holder:
                logger.warning('%s is still paused: the server rejected the resume (lease held by %s)', SENTRY_NAME, holder)
            else:
                logger.warning('%s is still paused: the server did not confirm the resume', SENTRY_NAME)
            return False

    def play(self, seq, loop=None):
        """
        Play a sequence in a background thread and return at once. seq is a Sequence, a
        dict in the look file format, or the name of a look in Library(). loop None uses
        the sequence's own loop flag. Each step goes out the way set() sends it, then is
        held for its hold; a playback already running is replaced.

        Control is taken for the playback (take_control(), so it nests with control you
        already hold) and released when it ends or is stopped. A sequence that ends stays
        on its last step; stop() puts back the look from before play() (see stop()). A
        process that exits mid-sequence ends the look early the same way; call wait()
        first if the whole look should play. Raises ValueError or KeyError for a bad
        sequence and RuntimeError when the sentry pause is not confirmed; nothing plays
        then.
        """
        from stretch4_body.eyes.looks import Library, Sequence
        if isinstance(seq, str):
            seq = Library().get(seq)
        elif isinstance(seq, dict):
            seq = Sequence.from_dict(seq)
        elif not isinstance(seq, Sequence):
            raise ValueError('play() takes a Sequence, a look dict or a look name, not {!r}'.format(seq))
        loop = seq.loop if loop is None else bool(loop)
        steps = [(self._parse(s.left, s.right, s.color, s.intensity), s.hold) for s in seq.steps]
        with self._play_lock:
            # Control for the new playback is taken before the old one lets go of its own,
            # so a replaced playback does not resume the sentry only to pause it again
            self.take_control()
            try:
                # A playback that replaces a running one restores what was there before both
                old = self._playback
                replacing = old is not None and not old.done.is_set()
                self._stop_locked(restore=False)
                pb = _Playback(old.restore_to if replacing else self._look_before_play())
            except BaseException:
                self.release()
                raise
            playing = {'name': seq.name, 'step': 0, 'steps': len(steps), 'loop': loop, 'started': time.time()}
            # _playback first: an old thread still exiting clears _playing only while it is its own
            self._playback = pb
            self._playing = playing
            threading.Thread(target=self._play_run, args=(pb, seq.name, steps, loop, playing),
                             name='eyes-play-' + seq.name, daemon=True).start()

    def _look_before_play(self):
        """The command stop() sends to undo a playback: the cached look, or idle if none was sent."""
        with self._lock:
            if self._source == 'assumed':
                return self._parse(IDLE_ANIMATION, IDLE_ANIMATION, IDLE_COLOR, IDLE_INTENSITY)
            # An eye never commanded showed the firmware's own animation; idle is the closest
            idle = find_animation(IDLE_ANIMATION)
            return self._left or idle, self._right or idle, self._color, self._intensity

    def _play_run(self, pb, name, steps, loop, playing):
        try:
            self._play_steps(pb, steps, loop, playing)
            if pb.restore.is_set():
                self._send(*pb.restore_to)
        except Exception:
            logger.exception('Eye sequence %s stopped', name)
        finally:
            if self._playback is pb:
                self._playing = None
            try:
                self.release()
            finally:
                pb.done.set()

    def _play_steps(self, pb, steps, loop, playing):
        due = time.monotonic()
        while not pb.stop.is_set():
            for i, (command, hold) in enumerate(steps):
                self._playing = dict(playing, step=i)
                if self._send(*command, stop_event=pb.stop) is None:
                    return
                # Holds run from a schedule, not from each send, so a loop does not drift.
                # A step that went out late holds from now.
                now = time.monotonic()
                if now - due > PLAY_RESYNC_S:
                    due = now
                due += hold
                if pb.stop.wait(max(0.0, due - time.monotonic())):
                    return
            if not loop:
                return

    @property
    def playing(self):
        """{'name', 'step', 'steps', 'loop', 'started'} for the sequence playing, else None."""
        playing = self._playing
        return None if playing is None else dict(playing)

    def stop(self, restore=True):
        """
        Stop playback and put back the look that was showing before play(): the left,
        right, color and intensity last sent before it, or idle when nothing had been
        sent (an eye never commanded goes to idle). restore=False leaves the eyes on the
        step that was showing. Nothing is sent when no sequence is playing, so a sequence
        that already ended stays on its last step.

        Waits up to STOP_PLAYBACK_S for the playback thread to exit. Returns False when
        it had not exited by then (a step stuck in a server push); it sends no more steps,
        only the restore once that push gets through. True when nothing was playing.
        """
        with self._play_lock:
            return self._stop_locked(restore)

    def _stop_locked(self, restore):
        pb = self._playback
        if pb is None or pb.done.is_set():
            return True
        if restore:
            pb.restore.set()
        else:
            pb.restore.clear()
        pb.stop.set()
        if not pb.done.wait(STOP_PLAYBACK_S):
            logger.warning('Eye sequence thread did not exit within %.1f s; it sends no more steps', STOP_PLAYBACK_S)
            return False
        return True

    def wait(self, timeout=None):
        """Block until playback ends (a looped sequence only ends on stop()). Returns whether it has."""
        pb = self._playback
        return pb is None or pb.done.wait(timeout)

    def close(self):
        """
        Stop playback (restoring the look from before it, as stop() does), release any
        control held, waiting up to CLOSE_RESUME_S for the sentry to resume, and close the
        backend even when that fails or raises. Returns False when the sentry could not be
        resumed. Runs at interpreter exit for an object still holding control.
        """
        atexit.unregister(self._atexit)
        self.stop()
        with self._lock:
            try:
                self._control_depth = 0
                return not self._pause_sent or self.resume_sentry(timeout=CLOSE_RESUME_S)
            finally:
                self._backend.close()
