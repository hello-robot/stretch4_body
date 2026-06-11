"""
Adapter for connecting and controlling the Luxonis Short-Range stereo camera pair used in Stretch's wrist.
"""

import logging
import depthai as dai

from stretch4_body.subsystem.cameras.cv_utils import RectifyMaps
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameraConfig, RGBCameras
from stretch4_body.subsystem.cameras.adapters.luxonis_camera_adapter import LuxonisCameraAdapter, clear_device_cache
from stretch4_body.subsystem.cameras.adapters.synced_camera import SyncedCamera
from stretch4_body.subsystem.cameras.models.image_frame import SyncedImageFrame, ImageFrame
import dataclasses
import numpy as np


class GripperCameraLuxonis(SyncedCamera):
    """Start a stream with the gripper left/right stereo cameras and the point cloud pipeline."""
    def __init__(self, left: RGBCameraConfig, right: RGBCameraConfig, enable_pointcloud: bool = False):
        self.do_sync_frames = True
        self.enable_pointcloud = enable_pointcloud

        self.left = left
        self.right = right

        self.left_rectify_maps: RectifyMaps | None = None
        self.right_rectify_maps: RectifyMaps | None = None

        self.pipeline, self.device = LuxonisCameraAdapter.create_pipeline(left.camera_device)
        self.camera = self.pipeline

        left_cfg = dataclasses.replace(left, is_compressed=False)

        self.left_camera_node, node_left = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=left_cfg)
        self.right_camera_node, node_right = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=right)

        if not self.enable_pointcloud:
            stereo = self.pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_C)
            stereo.initialConfig.postProcessing.thresholdFilter.maxRange = 10000 
            
            node_left.link(stereo.left)
            node_right.link(stereo.right)

            import datetime
            sync = self.pipeline.create(dai.node.Sync)
            sync.setSyncThreshold(datetime.timedelta(milliseconds=15))

            node_right.link(sync.inputs["right"])
            stereo.depth.link(sync.inputs["depth"])

            self.q_sync = sync.out.createOutputQueue(maxSize=1, blocking=False)
        else:
            stereo, rgbd = LuxonisCameraAdapter.create_rgbd_node(self.pipeline, node_left, node_right)
            
            self.right_output = node_right.createOutputQueue(maxSize=1)
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
            if not self.enable_pointcloud:
                msgGroup = self.q_sync.get()
                if msgGroup is not None:
                    msgNames = msgGroup.getMessageNames()
                    frame_right_msg = msgGroup["right"] if "right" in msgNames else None
                    frame_depth_msg = msgGroup["depth"] if "depth" in msgNames else None
                    
                    if frame_right_msg and frame_depth_msg:
                        timestamp = frame_right_msg.getTimestamp().total_seconds()
                        sequence_num = frame_right_msg.getSequenceNum()
                        
                        if self.right.is_compressed:
                            img_right = frame_right_msg.getData()
                            right_frame = ImageFrame(image=img_right, timestamp=timestamp, frame_number=sequence_num, compression_format="jpeg")
                        else:
                            img_right = frame_right_msg.getCvFrame()
                            right_frame = ImageFrame(image=img_right, timestamp=timestamp, frame_number=sequence_num)
                        
                        depth_frame = frame_depth_msg.getFrame()
                        
                        left_frame = ImageFrame(image=np.zeros((1, 1, 3), dtype=np.uint8), timestamp=timestamp, frame_number=sequence_num)
                        
                        synced_image = SyncedImageFrame(
                            timestamp=timestamp,
                            left=left_frame,
                            right=right_frame,
                            center=None,
                            depth=depth_frame
                        )
                        yield synced_image
            else:
                right_callback = next(LuxonisCameraAdapter.get_frame_from_output_queue(self.right_output))
                depth_callback = next(LuxonisCameraAdapter.get_frame_from_output_queue(self.depth_output))

                points, points_rgb, points_sequence_number = next(LuxonisCameraAdapter.get_pointcloud_from_output_queue(self.pointcloud_output))

                left_callback = ImageFrame(image=np.zeros((1, 1, 3), dtype=np.uint8), timestamp=right_callback.timestamp, frame_number=right_callback.frame_number)

                synced_image = SyncedImageFrame(timestamp=right_callback.timestamp, left=left_callback, right=right_callback, center=None, pointcloud=points, pointcloud_color=points_rgb, depth=depth_callback.image)

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
