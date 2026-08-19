import threading
from enum import Enum, auto

from typing import TYPE_CHECKING

import cv2

from stretch4_body.core.device import Device
from stretch4_body.subsystem.cameras.models.rgb_camera_config import CAMERA_CONFIGS, RGBCameraConfig

if TYPE_CHECKING:
    from stretch4_body.subsystem.cameras.adapters.camera_adapter import CameraAdapter
    from stretch4_body.subsystem.cameras.adapters.synced_camera import SyncedCamera

COMPRESSED_SUFFIX = "_compressed"
"""Names of the compressed camera variants end with this, e.g. RGBCameras.head_left_compressed."""


class CameraDevice(Device):
    """Deprecated. Camera configuration now lives in `CAMERA_CONFIGS` / `RGBCameras.get_config()`.

    Kept so that existing callers keep working, and so that importing this module still applies the
    fleet logging configuration that `Device` sets up.
    """
    def __init__(self):
        Device.__init__(self, 'cameras', req_params=False)

    def startup(self): return True
    def stop(self): return True

    def get_config(self, camera_type: "RGBCameras") -> "RGBCameraConfig":
        return camera_type.config

class RGBCameras(Enum):
    """
    This enum defines known cameras' capture drivers and configurations (image size, number of rotations, etc..).

    For consistency and quick access to the latest camera configuration across all scripts that use this enum, this enum provides a few static definitions: `RGBCameras.left()`, `RGBCameras.right()`, `RGBCameras.center()`, `RGBCameras.synced_left_right()` and `RGBCameras.synced_left_right_center()`.
    You may forgo using .left(), .right(), .center(), etc.. and use RGBCameras.camera_name directly in any script, of course.
    To update the static definitions, you may edit those methods in this enum below.

    This enum also includes a few helper properties and methods such as:
    1. `RGBCameras.my_camera.config` -> this has information on image size, number of rotations to perform, camera path, etc..
    2. `RGBCameras.my_camera.start()` -> Opens the capture device using the specified driver in the `start()` method.

    Every camera also has a `*_compressed` variant. It is the same camera with the same optics and
    calibration, but the device encodes the stream to MJPEG on-chip, so frames arrive as JPEG
    bitstreams (`ImageFrame.is_compressed()`) instead of raw BGR. Use them when the frames have to
    cross a process boundary (e.g. ROS 2 topics) where moving raw megabytes is the bottleneck.

    To add a new camera, please do the following:
    1. Add the camera's name to the end of this enum, e.g. "my_camera = auto()", and its compressed
       variant, e.g. "my_camera_compressed = auto()".
    2. Add its configuration to `CAMERA_CONFIGS` (keyed by the uncompressed name).
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

    # On-device MJPEG variants of the cameras above. See the class docstring.
    head_left_compressed = auto()
    head_center_compressed = auto()
    head_right_compressed = auto()
    head_left_right_compressed = auto()
    head_left_right_center_compressed = auto()

    gripper_left_compressed = auto()
    gripper_right_compressed = auto()
    gripper_rgbd_compressed = auto()

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
    def is_compressed_variant(self) -> bool:
        """True for the `*_compressed` members, whose frames arrive as JPEG bitstreams."""
        return self.name.endswith(COMPRESSED_SUFFIX)

    @property
    def base(self) -> "RGBCameras":
        """The uncompressed camera this member derives from. Returns itself when already uncompressed.

        Use this whenever you need the camera's identity rather than its transport, e.g. to look up
        calibration, a DepthAI board socket, or a recording folder.
        """
        if self.is_compressed_variant:
            return RGBCameras[self.name[: -len(COMPRESSED_SUFFIX)]]
        return self

    @property
    def compressed(self) -> "RGBCameras":
        """The MJPEG variant of this camera. Returns itself when already compressed."""
        if self.is_compressed_variant:
            return self
        return RGBCameras[self.name + COMPRESSED_SUFFIX]

    def matching_variant_of(self, camera_type: "RGBCameras") -> "RGBCameras":
        """`camera_type` in the same transport (compressed or not) as this camera.

        Lets a synced camera hand its own compressedness down to the cameras it is composed of.
        """
        return camera_type.compressed if self.is_compressed_variant else camera_type.base

    def get_config(self) -> "RGBCameraConfig":
        """The capture configuration for this camera, from `CAMERA_CONFIGS`.

        Synced camera types have no configuration of their own; ask the cameras they are composed of.
        """
        config_dict = CAMERA_CONFIGS.get(self.base.name)
        if config_dict is None:
            raise NotImplementedError(
                f"{self.name} has no configuration in CAMERA_CONFIGS. Synced camera types read the "
                f"configuration of the cameras they are composed of; see start_synced()."
            )

        config = RGBCameraConfig(**config_dict, camera_type=self)
        if self.is_compressed_variant:
            config.is_compressed = True

        return config

    @property
    def config(self):
        return self.get_config()

    def start(self, stop_event: threading.Event | None = None) -> "CameraAdapter":
        """Use `start()` to capture from one camera device. Use `start_synced()` for synced or dual camera setups."""
        if self.base in [
            RGBCameras.head_left,
            RGBCameras.head_right,
            RGBCameras.head_center,
            RGBCameras.gripper_left,
            RGBCameras.gripper_right,
        ]:
            from stretch4_body.subsystem.cameras.adapters.luxonis_camera_adapter import LuxonisCameraAdapter # import here to avoid circular import
            return LuxonisCameraAdapter(self.config, stop_event=stop_event)

        # Handles for other camera types, no need to update or edit these:
        if "synced_left" in self.name or "synced_right" in self.name:
            # There's little reason to stream left/right of a synced module on its own.
            raise ConnectionRefusedError(
                f"There is no need to call start() for {self.name}; call start() for the synced version of your camera and it will be used to stream images for both left and right cameras."
            )
        if "synced" in self.name or self.is_synced_camera_type():
            raise ConnectionRefusedError("Call start_synced() to start a synced camera")

        raise NotImplementedError(f"{self}'s start() method is not implemented.")

    def start_synced(self, stop_event: threading.Event | None = None, enable_pointcloud: bool = False) -> "SyncedCamera":
        """Use `start_synced()` to start sync'd frame grabbing."""
        from stretch4_body.subsystem.cameras.adapters.luxonis_gripper_camera_adapter import (
            GripperCameraLuxonis # import here to avoid circular import
        )
        from stretch4_body.subsystem.cameras.adapters.luxonis_synced_camera_adapter import (
            SyncedCameraLuxonis, # import here to avoid circular import
        )

        if self.base == RGBCameras.head_left_right:
            return SyncedCameraLuxonis(
                self.matching_variant_of(RGBCameras.head_left).config,
                self.matching_variant_of(RGBCameras.head_right).config,
                center=None,
                do_sync_frames=True,
                stop_event=stop_event,
            )

        if self.base == RGBCameras.head_left_right_center:
            return SyncedCameraLuxonis(
                self.matching_variant_of(RGBCameras.head_left).config,
                self.matching_variant_of(RGBCameras.head_right).config,
                center=self.matching_variant_of(RGBCameras.head_center).config,
                do_sync_frames=True,
                stop_event=stop_event,
            )

        if self.base == RGBCameras.gripper_rgbd:
            return GripperCameraLuxonis(
                self.matching_variant_of(RGBCameras.gripper_left).config,
                self.matching_variant_of(RGBCameras.gripper_right).config,
                enable_pointcloud=enable_pointcloud,
            )

        raise NotImplementedError(f"{self}'s start_synced() method is not implemented.")

    def is_left(self):
        """Is the right camera. WARNING: this only works if the RGBCameras.right() static definition is updated with this camera."""
        return self.base == RGBCameras.left()

    def is_right(self):
        """Is the right camera. WARNING: this only works if the RGBCameras.right() static definition is updated with this camera."""
        return self.base == RGBCameras.right()

    def is_center(self):
        """Is the center camera. WARNING: this only works if the RGBCameras.center() static definition is updated with this camera."""
        return self.base == RGBCameras.center()

    def is_synced_camera_type(self):
        """Is the synced camera. WARNING: this only works if the `RGBCameras.synced_left_right()` and `RGBCameras.synced_left_right_center()` static definition is updated with this camera."""
        return self.base in (
            RGBCameras.synced_left_right(),
            RGBCameras.synced_left_right_center(),
            RGBCameras.gripper_rgbd,
        )

    @property
    def recording_folder_name(self) -> str:
        if self.base == RGBCameras.gripper_left:
            return "rgb_camera_gripper_left"
        if self.base == RGBCameras.gripper_right:
            return "rgb_camera_gripper_right"
        if self.is_center():
            return "rgb_camera_center"
        if self.is_left():
            return "rgb_camera_left"
        if self.is_right():
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

    def start_camera_stream(self, is_rotate:bool, use_ros_for_cameras:bool=False):
        from stretch4_body.subsystem.cameras import (
    stream_left_camera,
    stream_right_camera,
    stream_center_camera,
    stream_left_right_camera,
    stream_left_right_center_camera,
    stream_gripper_camera,
    stream_left_camera_compressed,
    stream_right_camera_compressed,
    stream_center_camera_compressed,
    stream_left_right_camera_compressed,
    stream_left_right_center_camera_compressed,
    stream_gripper_camera_compressed,
)
        is_compressed = self.is_compressed_variant
        if self.base == RGBCameras.synced_left_right():
            gen_fn = stream_left_right_camera_compressed if is_compressed else stream_left_right_camera
        elif self.base == RGBCameras.synced_left_right_center():
            gen_fn = stream_left_right_center_camera_compressed if is_compressed else stream_left_right_center_camera
        elif self.is_left():
            gen_fn = stream_left_camera_compressed if is_compressed else stream_left_camera
        elif self.is_right():
            gen_fn = stream_right_camera_compressed if is_compressed else stream_right_camera
        elif self.is_center():
            gen_fn = stream_center_camera_compressed if is_compressed else stream_center_camera
        elif self.base == RGBCameras.gripper_rgbd:
            gen_fn = stream_gripper_camera_compressed if is_compressed else stream_gripper_camera
        else:
            raise ValueError(f"Unknown camera type: {self}")

        return gen_fn(
            is_rotate=is_rotate,
            use_ros_for_cameras=use_ros_for_cameras
        )


if __name__ == "__main__":
    camera = RGBCameras.center().start()
    while True:
        image_frame = camera.get_next()
        if image_frame.image is not None:
            cv2.imshow("Camera Output", image_frame.image)
            cv2.waitKey(1)
