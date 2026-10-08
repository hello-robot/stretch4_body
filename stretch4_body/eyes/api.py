import logging
import numbers
import threading
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
    writer thread for this reason.
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
        self._backend.connect()

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
        push (see ServerBackend). The values are cached either way.
        """
        left = None if left is None else find_animation(left)
        right = None if right is None else find_animation(right)
        color = None if color is None else parse_color(color)
        intensity = None if intensity is None else parse_intensity(intensity)
        with self._lock:
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
                        lease_holder=self._backend.lease_holder())

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

    def close(self):
        """
        Release any control held, waiting up to CLOSE_RESUME_S for the sentry to resume,
        and close the backend even when that fails or raises. Returns False when the
        sentry could not be resumed.
        """
        with self._lock:
            try:
                self._control_depth = 0
                return not self._pause_sent or self.resume_sentry(timeout=CLOSE_RESUME_S)
            finally:
                self._backend.close()
