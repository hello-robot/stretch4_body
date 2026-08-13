"""Vocabulary and output types -- the words the rest of the package speaks.

`BinClass` labels every returning bin. `LineSensorHits` is exactly what one
call to `LineSensorSource.process()` returns.

There is deliberately no `max_line_sensor_range` here. The old filter carried a
4.0 m cutoff and asked "is this number a distance?" by magnitude. The chip
never reports a large distance -- every out-of-range measurement comes back as
a status code -- so the reader now classifies each bin once at decode and
publishes a `codes` array. Validity is `codes == protocol.CODE_VALID`: one
integer compare, no threshold to drift, and 5.09 stays distinguishable from
5.11 no matter what the tare did to the number.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np


class BinClass(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OBSTACLE = 2
    SMALL_DROP = 3
    SPRAY = 4
    OBSTACLE_MARGINAL = 5
    DEEP_DROP = 6


# Marginal obstacles group with strong obstacles for run-building and
# publication; they differ only in required run length and confirm frames.
OBSTACLE_FAMILY = (BinClass.OBSTACLE, BinClass.OBSTACLE_MARGINAL)
# Deep drops group with small drops for run-building; they publish on a
# separate output so the hazard layer can treat them as lethal cliffs.
DROP_FAMILY = (BinClass.SMALL_DROP, BinClass.DEEP_DROP)


def family(cls: BinClass) -> BinClass:
    """Collapse a bin class to its family representative.

    History keys use the family, so a bin flapping between strong and marginal
    (or small and deep drop) keeps a single confirmation streak.
    """
    if cls in OBSTACLE_FAMILY:
        return BinClass.OBSTACLE
    if cls in DROP_FAMILY:
        return BinClass.SMALL_DROP
    return cls


@dataclass
class LineSensorHits:
    """One frame's hazards.

    Every drop-family and cliff output is reported at the bin's EXPECTED floor
    intersection, never where the beam actually landed. A bin that reads long
    means the beam flew over a ledge and hit floor further away; publishing
    that far point puts the hazard behind the real edge, and by the time the
    base reaches it the wheel is already over. The ledge is at or nearer than
    the expected-floor intersection, so that is the only conservative choice.
    """

    # --- hazards the consumer acts on ---------------------------------------
    obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    small_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    deep_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    probable_cliff_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    degraded_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))

    # --- measured height ------------------
    # What the beam actually measured, in metres with the floor at 0. 
    obstacle_z: np.ndarray = field(default_factory=lambda: np.zeros(0))
    small_drop_z: np.ndarray = field(default_factory=lambda: np.zeros(0))
    deep_drop_z: np.ndarray = field(default_factory=lambda: np.zeros(0))
    probable_cliff_z: np.ndarray = field(default_factory=lambda: np.zeros(0))
    degraded_z: np.ndarray = field(default_factory=lambda: np.zeros(0))

    # --- identity, row-aligned with the arrays above -------------------------
    # (sensor_idx, bin_idx) for every published point. Publish these as
    # PointFields: six identical 60 degree wedges tile the circle, so a
    # rotated, mirrored or permuted bus map all yield a complete, plausible
    # ring. Without identity a mounting error is not falsifiable from the
    # output, and self-consistency is all software can check.
    obstacle_id: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), np.int32))
    small_drop_id: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), np.int32))
    deep_drop_id: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), np.int32))
    probable_cliff_id: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), np.int32))
    degraded_id: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), np.int32))

    # --- which sensors actually contributed ---------------------------------
    # Absence of points means "clear" AND "not looking" unless you publish
    # this. A consumer computing swept coverage must use it, not the hazard
    # arrays, or it will read a dead sensor's wedge as safe floor.
    observed_sensors: tuple = ()
    skipped_sensors: dict = field(default_factory=dict)   # name -> why

    # --- debug views of the intermediate stages -----------------------------
    raw_obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    raw_small_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    spatial_obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    spatial_small_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    raw_spray_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    raw_marginal_obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    benign_null_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
