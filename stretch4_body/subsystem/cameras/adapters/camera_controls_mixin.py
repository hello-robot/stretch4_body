from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

class CameraControlsMixin:

    def focus_roi(self, roi: list[int], camera_type: RGBCameras | None = None): raise NotImplementedError()
    
    def set_manual_exposure(self, exposure_time: int, iso: int, camera_type: RGBCameras | None = None): raise NotImplementedError()
    
    def set_auto_exposure(self, limit_max: int | None = None, camera_type: RGBCameras | None = None): raise NotImplementedError()
    
    def set_manual_white_balance(self, color_temperature: int, camera_type: RGBCameras | None = None): 
        """
        Set manual white balance.
        
        Args:
            color_temperature: Value between 1000 and 12000.
            camera_type: The camera to apply this to.
        """
        raise NotImplementedError()
    
    def set_auto_white_balance(self, camera_type: RGBCameras | None = None): raise NotImplementedError()

    def set_brightness(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image brightness.
        
        Args:
            value: Brightness, range -10..10, default 0
            camera_type: The camera to apply this to.
        """
        raise NotImplementedError()

    def set_contrast(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image contrast.
        
        Args:
            value: Contrast, range -10..10, default 0
            camera_type: The camera to apply this to.
        """
        raise NotImplementedError()

    def set_saturation(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image saturation.
        
        Args:
            value: Saturation, range -10..10, default 0
            camera_type: The camera to apply this to.
        """
        raise NotImplementedError()

    def set_sharpness(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image sharpness.
        
        Args:
            value: Sharpness, range 0..4, default 1
            camera_type: The camera to apply this to.
        """
        raise NotImplementedError()