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
import datetime


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

        self.left_camera_node, node_left, node_left_compressed = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=left)
        self.right_camera_node, node_right, node_right_compressed = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=right)

        stereo = self.pipeline.create(dai.node.StereoDepth)
        # stereo.setRectifyEdgeFillColor(0)
        # stereo.enableDistortionCorrection(True)
        # https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        stereo.setDepthAlign(LuxonisCameraAdapter.get_depthai_camera_socket(self.right.camera_type)) # Align to right camera
        stereo.initialConfig.postProcessing.thresholdFilter.maxRange = int(self.right.stereo_max_range_mm)
        
        node_left.link(stereo.left)
        node_right.link(stereo.right)

        sync = self.pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(datetime.timedelta(milliseconds=int(self.right.sync_threshold_ms)))

        output_node_left = node_left_compressed if node_left_compressed is not None else node_left
        output_node_right = node_right_compressed if node_right_compressed is not None else node_right

        sync.inputs["left"].setMaxSize(self.left.buffer_size)
        sync.inputs["left"].setBlocking(False)
        output_node_left.link(sync.inputs["left"])

        sync.inputs["right"].setMaxSize(self.right.buffer_size)
        sync.inputs["right"].setBlocking(False)
        output_node_right.link(sync.inputs["right"])

        sync.inputs["depth"].setMaxSize(self.right.buffer_size)
        sync.inputs["depth"].setBlocking(False)
        stereo.depth.link(sync.inputs["depth"])

        if self.enable_pointcloud:
            rgbd = self.pipeline.create(dai.node.RGBD)
            rgbd.setDepthUnits(dai.StereoDepthConfig.AlgorithmControl.DepthUnit.METER)

            manip = self.pipeline.create(dai.node.ImageManip)
            manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)
            manip.setMaxOutputFrameSize(self.right.image_size[0] * self.right.image_size[1] * 3)
            manip.inputImage.setBlocking(False)
            manip.inputImage.setMaxSize(self.right.buffer_size)
            node_right.link(manip.inputImage)

            rgbd.inColor.setBlocking(False)
            rgbd.inColor.setMaxSize(self.right.buffer_size)
            manip.out.link(rgbd.inColor)

            rgbd.inDepth.setBlocking(False)
            rgbd.inDepth.setMaxSize(self.right.buffer_size)
            stereo.depth.link(rgbd.inDepth)
            self.q_pointcloud = rgbd.pcl.createOutputQueue(maxSize=self.right.buffer_size, blocking=False)

        self.q_sync = sync.out.createOutputQueue(maxSize=self.right.buffer_size, blocking=False)

        self.left_input_queue = self.left_camera_node.inputControl.createInputQueue()
        self.right_input_queue = self.right_camera_node.inputControl.createInputQueue()

        try:
            self.pipeline.start()
        except Exception:
            clear_device_cache()
            raise

    def get_gripper_intrinsics(self, camera_type: RGBCameras):
        """Returns M and D from the hardware factory calibration if available."""
        try:
            calib = self.device.readCalibration()
            M = np.array(calib.getCameraIntrinsics(LuxonisCameraAdapter.get_depthai_camera_socket(camera_type), camera_type.config.image_size[0], camera_type.config.image_size[1]), dtype=np.float64)
            D = np.array(calib.getDistortionCoefficients(LuxonisCameraAdapter.get_depthai_camera_socket(camera_type)), dtype=np.float64)
            return M, D
        except Exception as e:
            print(f"Warning: could not read calibration from OAK-D: {e}")
        return None, None
        
    def is_open(self):
        return self.pipeline is not None and self.device is not None and self.pipeline.isRunning() and not self.device.isClosed()

    def get_frames(self):
        if not self.is_open():
            raise RuntimeError("Camera is not running.")

        empty_left_or_right_frame = ImageFrame(image=np.zeros((self.left.image_size[1], self.left.image_size[0], 3), dtype=np.uint8), timestamp=0, frame_number=0)
            
        while True:
            msgGroup = self.q_sync.get()
            if msgGroup is not None:
                msgNames = msgGroup.getMessageNames()
                frame_left_msg = msgGroup["left"] if "left" in msgNames else None
                frame_right_msg = msgGroup["right"] if "right" in msgNames else None
                frame_depth_msg = msgGroup["depth"] if "depth" in msgNames else None

                pointcloud = None
                pointcloud_color = None
                if self.enable_pointcloud:
                    frame_pointcloud_msg = None
                    while self.q_pointcloud.has():
                        frame_pointcloud_msg = self.q_pointcloud.tryGet()
                    if frame_pointcloud_msg is None:
                        frame_pointcloud_msg = self.q_pointcloud.get()
                    if frame_pointcloud_msg is not None:
                        pointcloud, pointcloud_color = frame_pointcloud_msg.getPointsRGB()
                
                if frame_depth_msg:
                    timestamp = frame_depth_msg.getTimestamp().total_seconds()
                    sequence_num = frame_depth_msg.getSequenceNum()
                    if frame_left_msg:
                        left_frame = LuxonisCameraAdapter.dai_message_to_image_frame(frame_left_msg)
                    else:
                        left_frame = empty_left_or_right_frame
                    if frame_right_msg:
                        right_frame = LuxonisCameraAdapter.dai_message_to_image_frame(frame_right_msg)
                    else:
                        right_frame = empty_left_or_right_frame
                    
                    depth_frame = frame_depth_msg.getFrame()
                    
                    synced_image = SyncedImageFrame(
                        timestamp=timestamp,
                        left=left_frame,
                        right=right_frame,
                        center=None,
                        depth=depth_frame,
                        pointcloud=pointcloud,
                        pointcloud_color=pointcloud_color
                    )
                    yield synced_image
                else:
                    print("No depth frame received.")

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
