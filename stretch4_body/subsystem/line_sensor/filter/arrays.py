"""Array helpers shared across stages."""

from __future__ import annotations

import numpy as np

from stretch4_body.subsystem.line_sensor import protocol


def as_range_array(ranges_raw) -> np.ndarray:
    """Coerce a raw ranges payload (list / None / ndarray) to float64."""
    if ranges_raw is None:
        return np.array([], dtype=np.float64)
    return np.asarray(ranges_raw, dtype=np.float64)


def as_code_array(codes_raw, n_bins) -> np.ndarray:
    """Coerce a raw codes payload to uint8.

    A missing or wrong-length codes array is NOT filled with CODE_VALID.
    Zero is CODE_VALID, so `np.zeros` would silently declare every bin a good
    measurement -- including the NaN ones -- and the filter would classify a
    blind sensor as clear floor. Absent codes mean 'nothing here is a
    measurement', which is the safe reading.
    """
    if codes_raw is None:
        return np.full(n_bins, protocol.CODE_OTHER_INVALID, dtype=np.uint8)
    codes = np.asarray(codes_raw, dtype=np.uint8)
    if codes.size != n_bins:
        return np.full(n_bins, protocol.CODE_OTHER_INVALID, dtype=np.uint8)
    return codes


def measuring(codes: np.ndarray) -> np.ndarray:
    """Bins carrying a real distance. The single validity test in the package.

    Replaces `isfinite(r) & (r > 0) & (r < 4.0)`. The magnitude test could not
    tell a status code from a distance once the tare had shifted it; this one
    cannot be wrong because the classification happened at decode, before any
    arithmetic touched the value.
    """
    return np.asarray(codes) == protocol.CODE_VALID


def runs(indices: np.ndarray) -> list:
    """Contiguous (start, end) runs in a sorted index array."""
    if len(indices) == 0:
        return []
    breaks = np.flatnonzero(np.diff(indices) > 1)
    starts = np.concatenate([[0], breaks + 1])
    ends = np.concatenate([breaks, [len(indices) - 1]])
    return [(int(indices[a]), int(indices[b])) for a, b in zip(starts, ends)]


def items_to_xy(items, cls, projector=None, near_edge=False) -> np.ndarray:
    """Stack the (x, y) of every item whose class is in `cls`.

    With near_edge=True the point published is the bin's EXPECTED floor
    intersection instead of where the beam landed. Drops and cliffs must use
    it: a long reading means the beam passed over a ledge, so the measured
    point sits beyond the edge and steering to it drives the wheel off.
    """
    wanted = cls if isinstance(cls, tuple) else (cls,)
    points = []
    for sensor_idx, bin_idx, item_cls, pt in items:
        if item_cls not in wanted:
            continue
        if near_edge and projector is not None:
            floor_xy = projector.floor_intersections(sensor_idx)
            if bin_idx < len(floor_xy):
                points.append(floor_xy[bin_idx])
                continue
        points.append(pt[:2])
    return np.vstack(points) if points else np.zeros((0, 2))


def items_to_ids(items, cls) -> np.ndarray:
    """(sensor_idx, bin_idx) for the same items `items_to_xy` would return.

    Published alongside the points so a reading is attributable. Six identical
    60 degree wedges tile the circle, so a rotated, mirrored or permuted bus
    map all produce a complete, plausible ring -- x/y/z alone makes a mounting
    error unfalsifiable, and self-consistency is all software can check.
    """
    wanted = cls if isinstance(cls, tuple) else (cls,)
    ids = [(sensor_idx, bin_idx)
           for sensor_idx, bin_idx, item_cls, _pt in items if item_cls in wanted]
    return np.asarray(ids, dtype=np.int32) if ids else np.zeros((0, 2), np.int32)


def run_ids(sensor_idx: int, start: int, end: int) -> np.ndarray:
    """(sensor_idx, bin_idx) for every bin in an inclusive run."""
    bins = np.arange(start, end + 1, dtype=np.int32)
    return np.column_stack([np.full(bins.size, sensor_idx, np.int32), bins])
