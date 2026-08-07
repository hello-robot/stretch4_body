"""Line-sensor hazard filtering: raw ranges -> obstacles, drops, cliffs.
"""

from .arrays import as_code_array, as_range_array, measuring, runs
from .classify import classify_bin
from .config import LineSensorConfig
from .confirm import bin_confirmed, confirm_frames_for_bin
from .geometry import Projector
from .gloss import FlipTracker, quarantine_spray_candidates
from .hits import (
    DROP_FAMILY,
    OBSTACLE_FAMILY,
    BinClass,
    LineSensorHits,
    family,
)
from .nulls import NullEvidenceDetector
from .shape import ShapeGate
from .source import LineSensorSource

__all__ = [
    'BinClass',
    'DROP_FAMILY',
    'FlipTracker',
    'LineSensorConfig',
    'LineSensorHits',
    'LineSensorSource',
    'NullEvidenceDetector',
    'OBSTACLE_FAMILY',
    'Projector',
    'ShapeGate',
    'as_code_array',
    'as_range_array',
    'bin_confirmed',
    'classify_bin',
    'confirm_frames_for_bin',
    'family',
    'measuring',
    'quarantine_spray_candidates',
    'runs',
]
