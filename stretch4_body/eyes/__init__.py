"""
Python API for the Stretch 4 eye rings.

    from stretch4_body.eyes import Eyes
    with Eyes() as eyes, eyes.take_control():
        eyes.set(left='look_left', right='look_left', color='#00a0ff', intensity=0.5)
"""
from stretch4_body.eyes.animations import Animation, ANIMATIONS, find_animation
from stretch4_body.eyes.api import Eyes, EyeState, parse_intensity
from stretch4_body.eyes.colors import NAMED_COLORS, parse_color
from stretch4_body.eyes.looks import Library, Sequence, Step

__all__ = ['Eyes', 'EyeState', 'Animation', 'ANIMATIONS', 'NAMED_COLORS', 'Library', 'Sequence', 'Step',
           'find_animation', 'parse_color', 'parse_intensity']
