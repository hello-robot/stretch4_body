import numbers
import re

# Named colors. CSS values except orange and purple, which keep the values the eye tool
# has always used on the rings. 'stretch' is the firmware idle glow color.
NAMED_COLORS = {
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'red': (255, 0, 0),
    'green': (0, 255, 0),
    'blue': (0, 0, 255),
    'cyan': (0, 255, 255),
    'magenta': (255, 0, 255),
    'yellow': (255, 255, 0),
    'orange': (255, 80, 0),
    'purple': (128, 0, 255),
    'pink': (255, 192, 203),
    'hot_pink': (255, 105, 180),
    'stretch': (40, 48, 60),
}

_HEX = re.compile(r'^#?([0-9a-f]{6}|[0-9a-f]{3})$')


def parse_color(value):
    """
    Return (r, g, b) ints 0-255 for an (r, g, b) sequence, a hex string ('#ff8000',
    'ff8000', '#f80') or a name from NAMED_COLORS. Raises ValueError otherwise.
    """
    if isinstance(value, str):
        text = value.strip().lower()
        key = text.replace('-', '_').replace(' ', '_')
        if key in NAMED_COLORS:
            return NAMED_COLORS[key]
        m = _HEX.match(text)
        if m:
            digits = m.group(1)
            if len(digits) == 3:
                digits = ''.join(c * 2 for c in digits)
            return tuple(int(digits[i:i + 2], 16) for i in (0, 2, 4))
        raise ValueError('Unknown color {!r}. Use (r, g, b), \'#rrggbb\' or one of: {}'.format(
            value, ', '.join(NAMED_COLORS)))
    try:
        rgb = None if isinstance(value, (bytes, bytearray)) else tuple(value)
    except TypeError:
        rgb = None
    if (rgb is None or len(rgb) != 3
            or not all(isinstance(c, numbers.Integral) and not isinstance(c, bool) and 0 <= c <= 255 for c in rgb)):
        raise ValueError('Color {!r} must be three ints 0-255, \'#rrggbb\' or a name'.format(value))
    return tuple(int(c) for c in rgb)


def to_hex(rgb):
    return '#{:02x}{:02x}{:02x}'.format(*rgb)
