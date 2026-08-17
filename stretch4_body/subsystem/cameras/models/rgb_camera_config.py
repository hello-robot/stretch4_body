"""Capture configuration for the robot's cameras.

`RGBCameraConfig` describes how to open one camera, and `CAMERA_CONFIGS` holds the configuration of
every camera on the robot. `RGBCameras.<camera>.config` is how the rest of the codebase reads them.
"""

from dataclasses import dataclass
from typing import TYPE_CHECKING

from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels

if TYPE_CHECKING:
    from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras


@dataclass
class RGBCameraConfig:
    camera_device: str
    image_size: tuple[int, int]
    fps: int
    camera_type: "RGBCameras"
    distortion_model:DistortionModels|None = None
    rotate_number_of_times: int = 0
    buffer_size: int = 1
    is_compressed: bool = True
    is_lossless: bool = False # Only used if is_compressed is true
    jpeg_quality: int = 90 # Only used if is_compressed is true and is_lossless is False
    sensor_pixel_size_mm: float|None = None
    use_auto_exposure: bool = True
    limit_max: int | None = None
    exposure_time: int | None = None
    iso: int | None = None
    sync_threshold_ms: int = 15
    stereo_max_range_mm: int = 10000


CAMERA_CONFIGS: dict[str, dict] = {
    "head_left": {
        "camera_device": "OAK-FFC-3P",
        "image_size": (1200, 1920),
        "fps": 30,
        "rotate_number_of_times": 1,
        "buffer_size": 1,
        "is_compressed": False,
        "is_lossless": False,  # Only used if is_compressed is true
        "jpeg_quality": 90,  # Only used if is_compressed is true and is_lossless is False
        "distortion_model": DistortionModels.equidistant_with_recompute_extrinsics,
        "sensor_pixel_size_mm": 3.0 / 1000.0,
        "use_auto_exposure": True,
        "limit_max": None,  # Only used if use_auto_exposure is True
        "exposure_time": None,  # Only used if use_auto_exposure is False
        "iso": None,  # Only used if use_auto_exposure is False
    },
    "head_right": {
        "camera_device": "OAK-FFC-3P",
        "image_size": (1200, 1920),
        "fps": 30,
        "rotate_number_of_times": -1,
        "buffer_size": 1,
        "is_compressed": False,
        "is_lossless": False,
        "jpeg_quality": 90,
        "distortion_model": DistortionModels.equidistant_with_recompute_extrinsics,
        "sensor_pixel_size_mm": 3.0 / 1000.0,
        "use_auto_exposure": True,
        "limit_max": None,
        "exposure_time": None,
        "iso": None,
    },
    "head_center": {
        "camera_device": "OAK-FFC-3P",
        # Full 12MP resolution is (3040, 4056). 24 pixels are subtracted from the width so that
        # it is divisible by 16, which the on-device MJPEG encoder requires.
        "image_size": (3040, 4032),
        "fps": 10,
        "rotate_number_of_times": -1,
        "buffer_size": 1,
        "is_compressed": False,
        "is_lossless": False,
        "jpeg_quality": 90,
        "distortion_model": DistortionModels.wide_angle,
        "sensor_pixel_size_mm": 1.55 / 1000.0,
        "use_auto_exposure": True,
        "limit_max": None,
        "exposure_time": None,
        "iso": None,
    },
    "gripper_left": {
        "camera_device": "OAK-D-SR",
        # Options for full FOV: (400, 640), (500, 800), (600, 960), (640, 1024), (800, 1280)
        "image_size": (400, 640),
        "fps": 30,
        "rotate_number_of_times": 0,
        "buffer_size": 1,
        "is_compressed": True,
        "is_lossless": False,
        "jpeg_quality": 80,
        "distortion_model": None,
        "use_auto_exposure": True,
        "limit_max": None,
        "exposure_time": None,
        "iso": None,
        "sync_threshold_ms": 15,
        "stereo_max_range_mm": 10000,
    },
    "gripper_right": {
        "camera_device": "OAK-D-SR",
        # Options for full FOV: (400, 640), (500, 800), (600, 960), (640, 1024), (800, 1280)
        "image_size": (400, 640),
        "fps": 30,
        "rotate_number_of_times": 0,
        "buffer_size": 1,
        "is_compressed": True,
        "is_lossless": False,
        "jpeg_quality": 80,
        "distortion_model": None,
        "use_auto_exposure": True,
        "limit_max": None,
        "exposure_time": None,
        "iso": None,
        "sync_threshold_ms": 15,
        "stereo_max_range_mm": 10000,
    },
}
"""Capture configuration for every physical camera, keyed by the uncompressed `RGBCameras` member name.

These describe the capture hardware and the streams we ask it for, not per-robot calibration, so they
live here rather than in robot params. Compressed variants (`*_compressed`) reuse the entry of the
camera they derive from with `is_compressed` forced on; see `RGBCameras.get_config()`.
"""
