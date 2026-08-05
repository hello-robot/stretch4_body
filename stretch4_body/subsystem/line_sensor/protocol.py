#!/usr/bin/env python3
"""Wire protocol for the hello-pixart-j3 line sensor stream.

The firmware streams newline-delimited JSON over USB serial
(/dev/hello-pixart-j3): one line per sensor, six lines per frame:

    {"frameId": 1234, "distances<bus><dev>": [<report_num> integers, mm]}

<bus> is 1..3 and <dev> is 0..1; the (bus, dev) pair maps to a global sensor
index through the 'bus_sensor_map' robot param (SE4: [[1,0],[3,2],[5,4]]).

The PixArt chip reports distance as 16-bit fixed-point centimeters with 7
fractional bits. Two bit patterns are STATUS CODES, not distances, and the
firmware forwards them verbatim (converted to mm):

    0xFF80 -> 511.0 cm -> 5110 mm -> 5.11 m   NO_RETURN: nothing reflected
              back at all. Ambiguous: dark floor, lighting, or a cliff.
    0xFE80 -> 509.0 cm -> 5090 mm -> 5.09 m   BEYOND_LIMIT: a reflection WAS
              detected, but from past the chip's upper range-limit threshold.
              The sensors point down, so the beam travelled past where the
              floor should be: strong cliff/void evidence.

The chip reports every out-of-range measurement as one of these codes, never
as a plain large distance, so the code band starts at the BEYOND_LIMIT value:
any reading at or above 5090 mm is a code. Unknown patterns in that band are
classified OTHER_INVALID rather than silently treated as distance.

decode_distances_mm() is the ONLY place sentinel classification happens, and
it runs on the raw millimeter values straight off the wire. It returns two aligned arrays:

    ranges  meters, NaN at every bin that is not a distance measurement. A
            code left in this array would be silently averaged into a floor
            estimate; NaN makes that arithmetic fail loudly instead.
    codes   per-bin CODE_* classification. This is where the 5.09-vs-5.11
            distinction lives.

Downstream code must test `codes`, never float-compare the range values.
"""

import numpy as np

# The chip's status codes: raw 16-bit pattern, wire value (mm), and the value
# consumers see after the mm -> m conversion.
RAW_CODE_NO_DETECTION = 0xFF80
RAW_CODE_BEYOND_LIMIT = 0xFE80
MM_NO_DETECTION = 5110.0
MM_BEYOND_LIMIT = 5090.0
RANGE_NO_DETECTION_M = 5.11
RANGE_BEYOND_LIMIT_M = 5.09

# Everything at or above the lowest code value is a status code, not a range.
MM_CODE_BAND_MIN = MM_BEYOND_LIMIT

# Per-bin classification, one uint8 per bin in the status 'codes' array.
CODE_VALID = 0          # a real distance measurement
CODE_BEYOND_LIMIT = 1   # 5.09: return from past the range limit (floor gone)
CODE_NO_RETURN = 2      # 5.11: no reflection (dark floor / lighting / cliff)
CODE_OTHER_INVALID = 3  # zero, negative, non-finite, or an unknown code

CODE_NAMES = {
    CODE_VALID: 'valid',
    CODE_BEYOND_LIMIT: 'beyond_limit_5.09',
    CODE_NO_RETURN: 'no_return_5.11',
    CODE_OTHER_INVALID: 'other_invalid',
}

# What the chip literally reported, per code. `ranges` blanks these bins to
# NaN, so this is how a tool recovers the raw value for display.
CODE_VALUE_M = {
    CODE_BEYOND_LIMIT: RANGE_BEYOND_LIMIT_M,
    CODE_NO_RETURN: RANGE_NO_DETECTION_M,
}


def decode_distances_mm(raw_mm, flip=False):
    """Decode one sensor's wire payload: raw mm values -> (ranges_m, codes).

    `ranges_m` holds distances in meters, with every non-measurement bin set
    to NaN: a status code is not a distance, and leaving 5.11 in the array
    lets it be silently averaged into a floor estimate. `codes` carries the
    per-bin classification, so nothing is lost.

    `flip` reverses bin ordering (the 'flip_range_ordering' robot param) and
    is applied here, once, so ranges and codes always stay aligned.

    """
    raw = np.asarray(raw_mm, dtype=np.float64)
    if flip:
        raw = raw[::-1]

    ranges = raw / 1000.0
    codes = np.zeros(raw.shape, dtype=np.uint8)  # CODE_VALID == 0

    # A real measurement is strictly positive and below the code band. NaN and
    # +inf both fail this, so they fall through to OTHER_INVALID for free.
    suspect = ~((raw > 0.0) & (raw < MM_CODE_BAND_MIN))
    if suspect.any():
        codes[suspect] = CODE_OTHER_INVALID
        codes[raw == MM_BEYOND_LIMIT] = CODE_BEYOND_LIMIT
        codes[raw == MM_NO_DETECTION] = CODE_NO_RETURN
        ranges[suspect] = np.nan  # same mask, no second comparison

    return ranges, codes


def classify_ranges_mm(raw_mm):
    """Per-bin CODE_* classification of raw wire values (mm)."""
    return decode_distances_mm(raw_mm)[1]


def build_key_table(bus_sensor_map):
    """Map JSON payload keys to global sensor indices, from the params bus map.

    bus_sensor_map is indexed [bus - 1][device] -> global sensor index, e.g.
    the SE4 map [[1,0],[3,2],[5,4]] means bus 1 dev 0 is sensor_1. Returns
    {'distances<bus><dev>': index} with the six key strings prebuilt so the
    decode hot path never formats them.
    """
    if len(bus_sensor_map) != 3 or any(len(row) != 2 for row in bus_sensor_map):
        raise ValueError(
            f'bus_sensor_map must be 3 buses x 2 devices, got {bus_sensor_map}')
    indices = [idx for row in bus_sensor_map for idx in row]
    if sorted(indices) != list(range(6)):
        raise ValueError(
            f'bus_sensor_map must contain each sensor index 0..5 exactly '
            f'once, got {bus_sensor_map}')
    return {
        f'distances{bus}{dev}': bus_sensor_map[bus - 1][dev]
        for bus in range(1, 4) for dev in range(2)
    }
