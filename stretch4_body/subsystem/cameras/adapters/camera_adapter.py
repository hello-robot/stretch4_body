from collections.abc import Generator

from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameraConfig, RGBCameras
from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame
from stretch4_body.subsystem.cameras.adapters.camera_controls_mixin import CameraControlsMixin


class CameraAdapter(CameraControlsMixin):

    def __init__(self, camera_config: RGBCameraConfig):
        self.camera_config = camera_config

        self.open()

    def is_open(self): raise NotImplementedError()

    def open(self): raise NotImplementedError()

    def stop(self): raise NotImplementedError()

    def get_frames(self) -> Generator[ImageFrame,None,None]: raise NotImplementedError()
    
    def get_next(self) -> ImageFrame:
        return next(self.get_frames())

