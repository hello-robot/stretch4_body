import time
import cv2
import numpy as np

from stretch4_body.core.hello_utils import LoopTimer
from stretch4_body.subsystem.cameras.cv_utils import get_recify_maps
from stretch4_body.subsystem.cameras.cv_utils import RectifyMaps
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameraConfig, RGBCameras
from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame, SyncedImageFrame
from stretch4_body.subsystem.cameras.adapters.camera_controls_mixin import CameraControlsMixin


class SyncedCamera(CameraControlsMixin):
    """
    A synced camera module with h-stack'd left and right camera frames.
    The Left camera is assumed to be the main device, and will be used for video capturing.
    """
    def __init__(self, left: RGBCameraConfig, right:RGBCameraConfig, center:RGBCameraConfig|None, do_sync_frames:bool) -> None:

        self.left = left
        self.right = right
        self.center = center

        self.do_sync_frames = do_sync_frames

        self.left_rectify_maps: RectifyMaps | None = None
        self.right_rectify_maps: RectifyMaps | None = None

        self.left_camera = left.camera_type.start()
    
    def stop(self):
        self.left_camera.stop()

    def is_open(self):
        return self.left_camera.is_open()

    def get_frames(self):
        if not self.is_open():
            raise RuntimeError("Camera is not running.")
            
        for image_frame in self.left_camera.get_frames():
            if not isinstance(image_frame, ImageFrame): raise ValueError("Expected an ImageFrame type")
            timestamp = image_frame.timestamp
            right_image, left_image = np.hsplit(image_frame.image, 2)

            yield SyncedImageFrame(timestamp, ImageFrame(timestamp, left_image), ImageFrame(timestamp, right_image))

    def get_next(self) -> SyncedImageFrame:
        return next(self.get_frames())

    def get_next_rectified(self) -> SyncedImageFrame:
        synced_image = self.get_next()

        return self.rectify(synced_image)
        
    def rectify(self, synced_image:SyncedImageFrame):
        if self.left_rectify_maps is None or self.right_rectify_maps is None:
            raise Exception("Rectify maps are not set. Please call create_rectify_maps() before rectify().")
        
        return synced_image.rectify(
            left_recify_maps=self.left_rectify_maps,
            right_recify_maps=self.right_rectify_maps,
            left_calibration=self.left_calibration,
            right_calibration=self.right_calibration
        )
    
    def create_rectify_maps(self, synced_image: SyncedImageFrame, balance: float, fov_scale: float):
        """Call this before get_next_rectified() to generate the rectification maps."""
        self.left_calibration = self.left.camera_type.load_calibration()
        self.right_calibration = self.right.camera_type.load_calibration()

        if self.left_rectify_maps is None:
            self.left_rectify_maps = get_recify_maps(
                synced_image.left.image,
                sim_cam_matrix=self.left_calibration.camera_matrix,
                sim_cam_distortion_coeffs=self.left_calibration.distortion_coefficients,
                balance=balance,
                fov_scale=fov_scale,
            )
        if self.right_rectify_maps is None:
            self.right_rectify_maps = get_recify_maps(
                synced_image.right.image,
                sim_cam_matrix=self.right_calibration.camera_matrix,
                sim_cam_distortion_coeffs=self.right_calibration.distortion_coefficients,
                balance=balance,
                fov_scale=fov_scale,
            )


class SyncedEmulatedCameras(SyncedCamera):
    """Emulated sync that uses two cameras and attempts to software sync their frames.
    TODO: create a buffer and match the closest timestamp between a reference frame and a buffer frame as CK suggests.
    """

    def __init__(self, left: RGBCameraConfig, right:RGBCameraConfig, center:RGBCameraConfig|None, do_sync_frames:bool):
        super().__init__(left=left, right=right, center=center, do_sync_frames=do_sync_frames)

        self.right_camera = right.camera_type.start()

    def get_frames(self):
        while True:
            left_callback = self.left_camera.get_next()
            right_callback = self.right_camera.get_next()
            
            if abs(left_callback.timestamp - right_callback.timestamp) < 0.05:
                yield SyncedImageFrame(left_callback.timestamp, left_callback, right_callback, )
    



def main():
    import rerun as rr

    rr.init("synced_cameras", spawn=False)
    rr.spawn(memory_limit="1GB")

    camera = RGBCameras.synced_left_right().start_synced()

    loop_timer = LoopTimer()
    while True:
        loop_timer.start_of_iteration()
        # synced_image = camera.get_next_rectified(balance=1.0, fov_scale=0.2)
        # synced_image = camera.get_next_rectified(balance=0.0, fov_scale=0.8)
        synced_image = camera.get_next()
        # synced_image, timestamp = camera.camera.get_next()

        if synced_image is None:
            print("synced_image is None")
            time.sleep(0.5)
            continue
        def log_to_rerun(synced_image: SyncedImageFrame):
            rr.log("left_image", rr.Image(synced_image.left.image, color_model=rr.ColorModel.BGR))
            rr.log("right_image", rr.Image(synced_image.right.image, color_model=rr.ColorModel.BGR))
            if synced_image.center:
                rr.log("center_image", rr.Image(synced_image.center.image, color_model=rr.ColorModel.BGR))



        if not isinstance(synced_image, np.ndarray):
            log_to_rerun(synced_image)
        else:
            # synced_image = cv2.resize(synced_image, (0,0), fx=0.25, fy=0.25)
            # rr.log("image", rr.Image(synced_image, color_model=rr.ColorModel.BGR))
            cv2.imshow("image", synced_image)
            cv2.waitKey(1)

        loop_timer.end_of_iteration()
        loop_timer.pretty_print()



if __name__ == "__main__":
    main()