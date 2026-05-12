"""
Adapter for connecting and controlling the Luxonis Short-Range stereo camera pair used in Stretch's wrist.
"""

import logging
import depthai as dai

from stretch4_body.subsystem.cameras.cv_utils import RectifyMaps
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameraConfig, RGBCameras
from stretch4_body.subsystem.cameras.adapters.luxonis_camera_adapter import LuxonisCameraAdapter, clear_device_cache
from stretch4_body.subsystem.cameras.adapters.synced_camera import SyncedCamera
from stretch4_body.subsystem.cameras.models.image_frame import SyncedImageFrame


class GripperCameraLuxonis(SyncedCamera):
    """Start a stream with the gripper left/right stereo cameras and the point cloud pipeline."""
    def __init__(self, left: RGBCameraConfig, right: RGBCameraConfig):
        self.do_sync_frames = True

        self.left = left
        self.right = right

        self.left_rectify_maps: RectifyMaps | None = None
        self.right_rectify_maps: RectifyMaps | None = None

        self.pipeline, self.device = LuxonisCameraAdapter.create_pipeline(left.camera_device)
        self.camera = self.pipeline

        self.left_camera_node, node_left = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=left)
        self.right_camera_node, node_right = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=right)

        stereo, rgbd = LuxonisCameraAdapter.create_rgbd_node(self.pipeline, node_left, node_right)
        
        self.left_output = stereo.syncedLeft.createOutputQueue(maxSize=1)
        self.right_output =  stereo.syncedRight.createOutputQueue(maxSize=1)
        self.depth_output = stereo.depth.createOutputQueue(maxSize=1)
        self.pointcloud_output = rgbd.pcl.createOutputQueue(maxSize=1)

        self.left_input_queue = self.left_camera_node.inputControl.createInputQueue()
        self.right_input_queue = self.right_camera_node.inputControl.createInputQueue()

        try:
            self.pipeline.start()
        except Exception:
            clear_device_cache()
            raise
        
    def is_open(self):
        return self.pipeline is not None and self.device is not None and self.pipeline.isRunning() and not self.device.isClosed()

    def get_frames(self):
        if not self.is_open():
            raise RuntimeError("Camera is not running.")
            
        while True:

            left_callback = next(LuxonisCameraAdapter.get_frame_from_output_queue(self.left_output))
            right_callback = next(LuxonisCameraAdapter.get_frame_from_output_queue(self.right_output))
            depth_callback = next(LuxonisCameraAdapter.get_frame_from_output_queue(self.depth_output))

            points, points_rgb, points_sequence_number = next(LuxonisCameraAdapter.get_pointcloud_from_output_queue(self.pointcloud_output))

            synced_image = SyncedImageFrame(timestamp=left_callback.timestamp, left=left_callback, right=right_callback, center=None, pointcloud=points, pointcloud_color=points_rgb, depth=depth_callback.image)

            yield synced_image

    def stop(self):
        self.pipeline.stop()
        self.device.close()


    def focus_roi(self, roi: list[int], camera_type: RGBCameras | None = None):
        ctrl = dai.CameraControl()
        ctrl.setAutoExposureRegion(*roi)
        ctrl.setAutoFocusRegion(*roi)

        if camera_type == self.left.camera_type and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        elif camera_type == self.right.camera_type and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)

    def set_manual_exposure(self, exposure_time: int, iso: int, camera_type: RGBCameras | None = None):
        ctrl = dai.CameraControl()
        ctrl.setManualExposure(exposure_time, iso)
        logging.info(f"Setting runtime manual exposure for {camera_type.name if camera_type else 'all'} to {exposure_time=} and {iso=}")

        if camera_type == self.left.camera_type and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        elif camera_type == self.right.camera_type and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)

    def set_auto_exposure(self, limit_max: int | None = None, camera_type: RGBCameras | None = None):
        ctrl = dai.CameraControl()
        ctrl.setAutoExposureEnable()
        if limit_max is not None:
            ctrl.setAutoExposureLimit(limit_max)
        logging.info(f"Setting runtime auto exposure for {camera_type.name if camera_type else 'all'} with {limit_max=}")

        if camera_type == self.left.camera_type and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        elif camera_type == self.right.camera_type and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)

    def set_manual_white_balance(self, color_temperature: int, camera_type: RGBCameras | None = None):
        """
        Set manual white balance.
        
        Args:
            color_temperature: Value between 1000 and 12000.
            camera_type: The camera to apply this to.
        """
        ctrl = dai.CameraControl()
        ctrl.setManualWhiteBalance(color_temperature)
        logging.info(f"Setting runtime manual white balance for {camera_type.name if camera_type else 'all'} to {color_temperature=}K")

        if camera_type == self.left.camera_type and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        elif camera_type == self.right.camera_type and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)

    def set_auto_white_balance(self, camera_type: RGBCameras | None = None):
        ctrl = dai.CameraControl()
        ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
        logging.info(f"Setting runtime auto white balance for {camera_type.name if camera_type else 'all'}")

        if camera_type == self.left.camera_type and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        elif camera_type == self.right.camera_type and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)
