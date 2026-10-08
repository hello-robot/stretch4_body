import numbers
from dataclasses import dataclass


@dataclass(frozen=True)
class Animation:
    """
    One eye animation preset rendered by the PIMU firmware.

    id is the byte sent in RPC 29 SET_EYE_ANIMATION. uses_color and uses_intensity
    say whether the shared color and intensity bytes change what the eye shows.
    """
    id: int
    name: str
    label: str
    description: str
    uses_color: bool
    uses_intensity: bool


# Mirrors enum EyeAnimation in stretch_firmware_ii
# arduino/hello_pimu2/EyeAnimationManager.h, descriptions from the render functions in
# EyeAnimationManager.cpp (develop, pimu protocol p13). Id 0 is EYE_ANIM_NOP ("leave this
# eye as it is") and is not an animation, so it is not listed. Keep in sync with
# PowerPeriphDefn.EYE_ANIM_* (test/test_eyes.py checks the two agree).
# Eye animations arrived in hello-pimu2 v0.1.9p13; circle_cw and circle_ccw (13, 14) need
# v0.1.10p13 or newer, v0.1.9p13 ignores them.
# Ring positions are clock positions viewed from the front, 10 pixels per ring: LED1 sits
# just clockwise of the 12 o'clock gap, LED3 at 3, LED8 at 9, the 6 o'clock gap between
# LED5 and LED6 (EyeAnimationManager.h, physical layout).
ANIMATIONS = (
    Animation(1, 'off', 'Off',
              'All ten pixels dark.', False, False),
    Animation(2, 'idle_glow', 'Idle glow',
              'Steady glow that dips to 5% and back over 2 s, every 6 s. '
              'Firmware boot default, color (40, 48, 60).', True, True),
    Animation(3, 'blink', 'Blink',
              'Steady ring that closes and opens in 0.4 s, every 3 s.', True, True),
    Animation(4, 'look_left', 'Look left',
              'Pupil eases from 12 to 9 o\'clock in 0.5 s and holds there.', True, True),
    Animation(5, 'look_right', 'Look right',
              'Pupil eases from 12 to 3 o\'clock in 0.5 s and holds there.', True, True),
    Animation(6, 'rainbow_spin', 'Rainbow spin',
              'Hue wheel chasing round the ring. Ignores color; intensity sets '
              'brightness with a floor of 40.', False, True),
    Animation(7, 'alert', 'Alert',
              'Whole ring pulses on and off every 0.3 s.', True, True),
    Animation(8, 'happy', 'Happy',
              'Burst grows from 12 o\'clock over 0.6 s, then fades. Repeats every 2 s.',
              True, True),
    Animation(9, 'left_half', 'Left half',
              'Steady, the five pixels from 6 to 12 o\'clock on the 9 o\'clock side.',
              True, True),
    Animation(10, 'right_half', 'Right half',
              'Steady, the five pixels from 12 to 6 o\'clock on the 3 o\'clock side.',
              True, True),
    Animation(11, 'top_half', 'Top half',
              'Steady, five pixels from about 10 to 3 o\'clock (LED9, LED10, LED1 to LED3): '
              'the top, shifted one pixel clockwise.', True, True),
    Animation(12, 'bottom_half', 'Bottom half',
              'Steady, five pixels from about 4 to 9 o\'clock (LED4 to LED8): '
              'the bottom, shifted one pixel clockwise.', True, True),
    Animation(13, 'circle_cw', 'Circle clockwise',
              'Three-pixel bar circling clockwise, one turn every 2 s. '
              'Needs hello-pimu2 v0.1.10p13 or newer.', True, True),
    Animation(14, 'circle_ccw', 'Circle anticlockwise',
              'Three-pixel bar circling anticlockwise, one turn every 2 s. '
              'Needs hello-pimu2 v0.1.10p13 or newer.', True, True),
)

ANIMATIONS_BY_ID = {a.id: a for a in ANIMATIONS}
ANIMATIONS_BY_NAME = {a.name: a for a in ANIMATIONS}


def find_animation(value):
    """
    Return the Animation for an id (int or digit string), a name in any case
    ('idle_glow', 'IDLE_GLOW', 'idle-glow') or an Animation. Raises ValueError otherwise.
    """
    if isinstance(value, Animation):
        value = value.id
    if isinstance(value, str) and value.strip().isdigit():
        value = int(value)
    if isinstance(value, numbers.Integral) and not isinstance(value, bool):
        if int(value) in ANIMATIONS_BY_ID:
            return ANIMATIONS_BY_ID[int(value)]
    elif isinstance(value, str):
        key = value.strip().lower().replace('-', '_').replace(' ', '_')
        if key in ANIMATIONS_BY_NAME:
            return ANIMATIONS_BY_NAME[key]
    valid = ', '.join('{} ({})'.format(a.name, a.id) for a in ANIMATIONS)
    raise ValueError('Unknown eye animation {!r}. Valid animations: {}'.format(value, valid))
