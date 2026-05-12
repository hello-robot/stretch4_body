from dataclasses import dataclass
from enum import Enum, auto

from typing import TYPE_CHECKING

import cv2

from stretch4_body.core.device import Device
from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels

if TYPE_CHECKING:
    from stretch4_body.subsystem.cameras.adapters.camera_adapter import CameraAdapter
    from stretch4_body.subsystem.cameras.adapters.synced_camera import SyncedCamera

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

class CameraDevice(Device):
    """Sets up a Stretch Body camera device to pull params from robot params."""
    def __init__(self):
        Device.__init__(self, 'cameras')

    def startup(self): return True
    def stop(self): return True

    def get_config(self, camera_type: "RGBCameras") -> "RGBCameraConfig":
        config_dict = self.params[camera_type.name]["config"]
        config_dict["camera_type"] = camera_type

        config = RGBCameraConfig(**config_dict)
        config.distortion_model = DistortionModels[config_dict["distortion_model"].replace("DistortionModels.", "")] if config_dict["distortion_model"] is not None else None

        return config

class RGBCameras(Enum):
    """
    This enum defines known cameras' capture drivers and configurations (image size, number of rotations, etc..).

    For consistency and quick access to the latest camera configuration across all scripts that use this enum, this enum provides a few static definitions: `RGBCameras.left()`, `RGBCameras.right()`, `RGBCameras.center()`, `RGBCameras.synced_left_right()` and `RGBCameras.synced_left_right_center()`.
    You may forgo using .left(), .right(), .center(), etc.. and use RGBCameras.camera_name directly in any script, of course.
    To update the static definitions, you may edit those methods in this enum below.

    This enum also includes a few helper properties and methods such as:
    1. `RGBCameras.my_camera.config` -> this has information on image size, number of rotations to perform, camera path, etc..
    2. `RGBCameras.my_camera.start()` -> Opens the capture device using the specified driver in the `start()` method.

    To add a new camera, please do the following:
    1. Add the camera's name to the end of this enum, e.g. "my_camera = auto()"
    2. Update the `config()` method with your new enum.
    3. Update the `start()` method with the driver to use. 
        NOTE: if you are adding a stereo camera, you should edit `start_synced()`.
    4. You can now use your new camera with most scripts that use RGBCameras.
    """
    head_left = auto()
    head_center = auto()
    head_right = auto()
    head_left_right = auto()
    head_left_right_center = auto()

    gripper_left = auto()
    gripper_right = auto()
    gripper_rgbd = auto()

    @staticmethod
    def center():
        """A reference to the current center camera definition"""
        return RGBCameras.head_center

    @staticmethod
    def left():
        """A reference to the current left camera definition"""
        return RGBCameras.head_left

    @staticmethod
    def right():
        """A reference to the current right camera definition"""
        return RGBCameras.head_right

    @staticmethod
    def synced_left_right():
        """A reference to the current synced left and right camera definition."""
        return RGBCameras.head_left_right

    @staticmethod
    def synced_left_right_center():
        """A reference to the current synced left and right camera definition, with the center camera included. The center camera may or may not be synced with the left and right feed, depending on the SyncedCamera class implementation. However, this will allow opening capture for the center camera."""
        return RGBCameras.head_left_right_center
    
    @property
    def config(self):
        return CameraDevice().get_config(self)

    def start(self) -> "CameraAdapter":
        """Use `start()` to capture from one camera device. Use `start_synced()` for synced or dual camera setups."""
        if self in [
            RGBCameras.head_left,
            RGBCameras.head_right,
            RGBCameras.head_center,
            RGBCameras.gripper_left,
            RGBCameras.gripper_right,
        ]:
            from stretch4_body.subsystem.cameras.adapters.luxonis_camera_adapter import LuxonisCameraAdapter # import here to avoid circular import
            return LuxonisCameraAdapter(self.config)

        # Handles for other camera types, no need to update or edit these:
        if "synced_left" in self.name or "synced_right" in self.name:
            # There's little reason to stream left/right of a synced module on its own.
            raise ConnectionRefusedError(
                f"There is no need to call start() for {self.name}; call start() for the synced version of your camera and it will be used to stream images for both left and right cameras."
            )
        if "synced" in self.name or self.is_synced_camera_type():
            raise ConnectionRefusedError("Call start_synced() to start a synced camera")

        raise NotImplementedError(f"{self}'s start() method is not implemented.")

    def start_synced(self) -> "SyncedCamera":
        """Use `start_synced()` to start sync'd frame grabbing."""
        from stretch4_body.subsystem.cameras.adapters.luxonis_gripper_camera_adapter import (
            GripperCameraLuxonis # import here to avoid circular import
        )
        from stretch4_body.subsystem.cameras.adapters.luxonis_synced_camera_adapter import (
            SyncedCameraLuxonis, # import here to avoid circular import
        )

        if self == RGBCameras.head_left_right:
            return SyncedCameraLuxonis(
                RGBCameras.head_left.config,
                RGBCameras.head_right.config,
                center=None,
                do_sync_frames=True,
            )

        if self == RGBCameras.head_left_right_center:
            return SyncedCameraLuxonis(
                RGBCameras.head_left.config,
                RGBCameras.head_right.config,
                center=RGBCameras.head_center.config,
                do_sync_frames=True,
            )
        
        if self == RGBCameras.gripper_rgbd:
            return GripperCameraLuxonis(
                RGBCameras.gripper_left.config,
                RGBCameras.gripper_right.config,
            )

        raise NotImplementedError(f"{self}'s start_synced() method is not implemented.")

    def is_left(self):
        """Is the right camera. WARNING: this only works if the RGBCameras.right() static definition is updated with this camera."""
        return self == RGBCameras.left()
    
    def is_right(self):
        """Is the right camera. WARNING: this only works if the RGBCameras.right() static definition is updated with this camera."""
        return self == RGBCameras.right()

    def is_center(self):
        """Is the center camera. WARNING: this only works if the RGBCameras.center() static definition is updated with this camera."""
        return self == RGBCameras.center()

    def is_synced_camera_type(self):
        """Is the synced camera. WARNING: this only works if the `RGBCameras.synced_left_right()` and `RGBCameras.synced_left_right_center()` static definition is updated with this camera."""
        return (
            self == RGBCameras.synced_left_right()
            or self == RGBCameras.synced_left_right_center()
            or self == RGBCameras.gripper_rgbd
        )

    @property
    def recording_folder_name(self) -> str:
        if self == RGBCameras.center():
            return "rgb_camera_center"
        if self == RGBCameras.left():
            return "rgb_camera_left"
        if self == RGBCameras.right():
            return "rgb_camera_right"

        raise NotImplementedError(f"{self}'s recoding folder name is not implemented.")

    @staticmethod
    def active_cameras() -> "list[RGBCameras]":
        return [RGBCameras.left(), RGBCameras.right(), RGBCameras.center()]

    @staticmethod
    def all_recording_folder_names() -> list[str]:
        return [c.recording_folder_name for c in RGBCameras.active_cameras()]

    def load_calibration(self):
        from stretch4_body.subsystem.cameras.models.camera_calibration import RGBCameraCalibration

        return RGBCameraCalibration.load_calibration_from_fleet_path(
            camera_type=self, is_flip_width_and_height=False
        )


if __name__ == "__main__":
    camera = RGBCameras.center().start()
    while True:
        image_frame = camera.get_next()
        if image_frame.image is not None:
            cv2.imshow("Camera Output", image_frame.image)
            cv2.waitKey(1)
