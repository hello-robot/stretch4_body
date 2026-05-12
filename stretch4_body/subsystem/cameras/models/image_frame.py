from dataclasses import field
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import numpy as np

from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.camera_calibration import (
    RGBCameraCalibration,
)

from stretch4_body.subsystem.cameras.cv_utils import RectifyMaps, rectify

import time

import cv2
import numpy as np


@dataclass
class ImageFrame:
    """
    A container for an image and its timestamp.
    Sometimes the camera module may provide a compressed image. In this case the image np.ndarray will be compressed, and you should use uncompress() to decode it.
    May sometimes also populate a rectified image and its computed camera matrix."""

    timestamp: float
    """The timestamp from the camera module. This is different from timestamp_system, which is the system time of when the image was processed."""

    frame_number: int
    image: np.ndarray
    """The image data. May sometimes be compressed."""
    compression_format: str = ""

    image_raw:np.ndarray = field(init=False)
    """image_raw is assigned if the image is processed by the RGBPipelineController to preserve the original image."""

    timestamp_system: float = field(default_factory=time.time)
    """Unix timestamp of when the image was processed."""
    
    image_rectified: np.ndarray | None = None
    new_K: np.ndarray | None = None

    ai_model_results: list | None = None
    """AI model results are assigned here if an AI model is used in the camera pipeline controller"""

    def is_compressed(self) -> bool:
        return self.compression_format != ""

    def uncompress(self):
        self.image = cv2.imdecode(self.image, cv2.IMREAD_COLOR)
        self.compression_format = ""
        return self.image


@dataclass
class SyncedImageFrame:
    """A container containing the left, right, center, and sometimes the stereo pointcloud from those camera devices."""
    timestamp: float
    left: ImageFrame
    right: ImageFrame
    center: ImageFrame | None = None

    depth: np.ndarray | None = None
    pointcloud: np.ndarray | None = None
    pointcloud_color: np.ndarray | None = None

    def get_frame_by_camera_type(self, camera_type: RGBCameras):
        if camera_type.is_center(): return self.center
        if camera_type.is_right(): return self.right
        if camera_type.is_left(): return self.left

        raise NotImplementedError(f"Unknown {camera_type=}")

    def rectify(
        self,
        left_recify_maps: RectifyMaps,
        right_recify_maps: RectifyMaps,
        left_calibration: RGBCameraCalibration,
        right_calibration: RGBCameraCalibration,
    ):
        tasks = (
            (
                self.left,
                None,
                left_recify_maps.map1,
                left_recify_maps.map2,
            ),
            (
                self.right,
                None,
                right_recify_maps.map1,
                right_recify_maps.map2,
            ),
        )
        with ThreadPoolExecutor(max_workers=2) as executor:
            results = list(executor.map(lambda args: rectify(*args), tasks))

        left_result, right_result = results

        left_rectified, _ = left_result
        right_rectified, _ = right_result

        return SyncedImageFrame(
            timestamp=self.timestamp,
            left=ImageFrame(
                self.timestamp,
                self.left.frame_number,
                self.left.image,
                left_rectified,
                left_recify_maps.new_K,
            ),
            right=ImageFrame(
                self.timestamp,
                self.right.frame_number,
                self.right.image,
                right_rectified,
                right_recify_maps.new_K,
            ),
        )
