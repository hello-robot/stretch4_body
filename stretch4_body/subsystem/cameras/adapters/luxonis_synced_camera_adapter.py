"""
Adapter for connecting to and controlling the Luxonis head cameras using the DepthAI API for the Luxonis OAK-FFC 3P board.
"""
import datetime
import time
import threading
import logging

logger = logging.getLogger(__name__)
import depthai as dai
from stretch4_body.subsystem.cameras.cv_utils import RectifyMaps
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.rgb_camera_config import RGBCameraConfig
from stretch4_body.subsystem.cameras.adapters.synced_camera import SyncedCamera
from stretch4_body.subsystem.cameras.adapters.luxonis_camera_adapter import LuxonisCameraAdapter, clear_device_cache
from stretch4_body.subsystem.cameras.models.image_frame import SyncedImageFrame


class SyncedCameraLuxonis(SyncedCamera):
    """Starts a stream with the left and right cameras synced, and an option to use the center camera as well."""

    def __init__(self, left: RGBCameraConfig, right:RGBCameraConfig, center:RGBCameraConfig|None, do_sync_frames:bool, stop_event: threading.Event = None):
        self.do_sync_frames = do_sync_frames
        self.stop_event = stop_event

        self.left = left
        self.right = right
        self.center = center

        self.left_rectify_maps: RectifyMaps | None = None
        self.right_rectify_maps: RectifyMaps | None = None

        self.pipeline, self.device = LuxonisCameraAdapter.create_pipeline(left.camera_device or right.camera_device)
        self.camera = self.pipeline

        self.left_camera_node, node_left, node_left_compressed = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=left)
        self.right_camera_node, node_right, node_right_compressed = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=right)
   
        node_center = None
        node_center_compressed = None
        if center is not None:
            self.center_camera_node, node_center, node_center_compressed = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=center)
            self.center_input_queue = self.center_camera_node.inputControl.createInputQueue()
        
        self.left_input_queue = self.left_camera_node.inputControl.createInputQueue()
        self.right_input_queue = self.right_camera_node.inputControl.createInputQueue()

        output_node_left = node_left_compressed if node_left_compressed is not None else node_left
        output_node_right = node_right_compressed if node_right_compressed is not None else node_right
        output_node_center = node_center_compressed if node_center_compressed is not None else node_center

        self.center_output = None
        if self.do_sync_frames:
            sync = self.pipeline.create(dai.node.Sync)
            sync.setSyncThreshold(datetime.timedelta(milliseconds=int(self.left.sync_threshold_ms)))

            sync.inputs["left"].setMaxSize(self.left.buffer_size)
            sync.inputs["left"].setBlocking(False)
            output_node_left.link(sync.inputs["left"])

            sync.inputs["right"].setMaxSize(self.right.buffer_size)
            sync.inputs["right"].setBlocking(False)
            output_node_right.link(sync.inputs["right"])

            # sync.inputs["center"].setMaxSize(self.center.buffer_size)
            # sync.inputs["center"].setBlocking(False)
            
            # output_node_center.link(sync.inputs["center"])

            # CENTER is NOT in the sync pipeline to keep it at its own 10fps while left/right are at 30fps
            if center is not None:
                self.center_output = output_node_center.createOutputQueue(maxSize=self.center.buffer_size, blocking=False)

            self.q_sync = sync.out.createOutputQueue(maxSize= self.left.buffer_size, blocking=False)
        else:
            if center is not None:
                self.center_output = output_node_center.createOutputQueue(maxSize=self.center.buffer_size, blocking=False)
            self.left_output = output_node_left.createOutputQueue(maxSize=self.left.buffer_size, blocking=False)
            self.right_output = output_node_right.createOutputQueue(maxSize=self.right.buffer_size, blocking=False)

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

        if self.do_sync_frames:
            while True:
                if self.stop_event is not None and self.stop_event.is_set():
                    return
                msg_group = self.q_sync.get()
                left_msg = msg_group["left"]
                right_msg = msg_group["right"]
                center_msg = msg_group["center"] if "center" in msg_group else None

                left_frame = LuxonisCameraAdapter.dai_message_to_image_frame(left_msg)
                right_frame = LuxonisCameraAdapter.dai_message_to_image_frame(right_msg)

                center_frame = None
                # if center_msg is not None:
                #     c_frame = LuxonisCameraAdapter.dai_message_to_image_frame(center_msg)
                if self.center_output:
                    c_frame = next(LuxonisCameraAdapter.get_frame_from_output_queue_no_block(self.center_output))
                    if c_frame:
                        lr_timestamp = left_msg.getTimestamp().total_seconds()
                        diff = abs(c_frame.timestamp - lr_timestamp)
                        if diff < 1/self.center.fps:
                            center_frame = c_frame

                yield SyncedImageFrame(timestamp=time.time(), left=left_frame, right=right_frame, center=center_frame)
        else:
            while True:
                left_frame = next(LuxonisCameraAdapter.get_frame_from_output_queue(self.left_output))
                right_frame = next(LuxonisCameraAdapter.get_frame_from_output_queue(self.right_output))

                center_frame = None
                if self.center is not None:
                    center_frame = next(LuxonisCameraAdapter.get_frame_from_output_queue_no_block(self.center_output))

                yield SyncedImageFrame(timestamp=time.time(), left=left_frame, right=right_frame, center=center_frame)

    def stop(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass

    def focus_roi(self, roi: list[int], camera_type: RGBCameras | None = None):
        logger.info(f"Setting roi {roi} for {camera_type.name if camera_type else 'all'}")
        ctrl = dai.CameraControl()
        ctrl.setAutoExposureRegion(*roi)
        ctrl.setAutoFocusRegion(*roi)

        if (camera_type is None or camera_type == self.left.camera_type) and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        if (camera_type is None or camera_type == self.right.camera_type) and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)
        if self.center is not None and (camera_type is None or camera_type == self.center.camera_type) and hasattr(self, 'center_input_queue'):
            self.center_input_queue.send(ctrl)

    def set_manual_exposure(self, exposure_time: int, iso: int, camera_type: RGBCameras | None = None):
        ctrl = dai.CameraControl()
        ctrl.setManualExposure(exposure_time, iso)
        logging.info(f"Setting runtime manual exposure for {camera_type.name if camera_type else 'all'} to {exposure_time=} and {iso=}")

        if iso < 100 or iso > 1600:
            raise ValueError("iso value has to be between 100 and 1600")
            
        if (camera_type is None or camera_type == self.left.camera_type) and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        if (camera_type is None or camera_type == self.right.camera_type) and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)
        if self.center is not None and (camera_type is None or camera_type == self.center.camera_type) and hasattr(self, 'center_input_queue'):
            self.center_input_queue.send(ctrl)

    def set_auto_exposure(self, limit_max: int | None = None, camera_type: RGBCameras | None = None):
        ctrl = dai.CameraControl()
        ctrl.setAutoExposureEnable()
        if limit_max is not None:
            ctrl.setAutoExposureLimit(limit_max)
        logging.info(f"Setting runtime auto exposure for {camera_type.name if camera_type else 'all'} with {limit_max=}")

        if (camera_type is None or camera_type == self.left.camera_type) and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        if (camera_type is None or camera_type == self.right.camera_type) and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)
        if self.center is not None and (camera_type is None or camera_type == self.center.camera_type) and hasattr(self, 'center_input_queue'):
            self.center_input_queue.send(ctrl)

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

        if color_temperature < 1000 or color_temperature > 12000:
            raise ValueError("color_temperature value has to be between 1000 and 12000")

        if (camera_type is None or camera_type == self.left.camera_type) and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        if (camera_type is None or camera_type == self.right.camera_type) and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)
        if self.center is not None and (camera_type is None or camera_type == self.center.camera_type) and hasattr(self, 'center_input_queue'):
            self.center_input_queue.send(ctrl)

    def set_auto_white_balance(self, camera_type: RGBCameras | None = None):
        ctrl = dai.CameraControl()
        ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
        logging.info(f"Setting runtime auto white balance for {camera_type.name if camera_type else 'all'}")

        if (camera_type is None or camera_type == self.left.camera_type) and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        if (camera_type is None or camera_type == self.right.camera_type) and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)
        if self.center is not None and (camera_type is None or camera_type == self.center.camera_type) and hasattr(self, 'center_input_queue'):
            self.center_input_queue.send(ctrl)

    def set_brightness(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image brightness.
        
        Args:
            value: Brightness, range -10..10, default 0
            camera_type: The camera to apply this to.
        """
        if value < -10 or value > 10:
            raise ValueError("brightness value has to be between -10 and 10")
            
        ctrl = dai.CameraControl()
        ctrl.setBrightness(value)
        logging.info(f"Setting runtime brightness for {camera_type.name if camera_type else 'all'} to {value}")

        if (camera_type is None or camera_type == self.left.camera_type) and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        if (camera_type is None or camera_type == self.right.camera_type) and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)
        if self.center is not None and (camera_type is None or camera_type == self.center.camera_type) and hasattr(self, 'center_input_queue'):
            self.center_input_queue.send(ctrl)

    def set_contrast(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image contrast.
        
        Args:
            value: Contrast, range -10..10, default 0
            camera_type: The camera to apply this to.
        """
        if value < -10 or value > 10:
            raise ValueError("contrast value has to be between -10 and 10")
            
        ctrl = dai.CameraControl()
        ctrl.setContrast(value)
        logging.info(f"Setting runtime contrast for {camera_type.name if camera_type else 'all'} to {value}")

        if (camera_type is None or camera_type == self.left.camera_type) and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        if (camera_type is None or camera_type == self.right.camera_type) and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)
        if self.center is not None and (camera_type is None or camera_type == self.center.camera_type) and hasattr(self, 'center_input_queue'):
            self.center_input_queue.send(ctrl)

    def set_saturation(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image saturation.
        
        Args:
            value: Saturation, range -10..10, default 0
            camera_type: The camera to apply this to.
        """
        if value < -10 or value > 10:
            raise ValueError("saturation value has to be between -10 and 10")
            
        ctrl = dai.CameraControl()
        ctrl.setSaturation(value)
        logging.info(f"Setting runtime saturation for {camera_type.name if camera_type else 'all'} to {value}")

        if (camera_type is None or camera_type == self.left.camera_type) and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        if (camera_type is None or camera_type == self.right.camera_type) and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)
        if self.center is not None and (camera_type is None or camera_type == self.center.camera_type) and hasattr(self, 'center_input_queue'):
            self.center_input_queue.send(ctrl)

    def set_sharpness(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image sharpness.
        
        Args:
            value: Sharpness, range 0..4, default 1
            camera_type: The camera to apply this to.
        """
        if value < 0 or value > 4:
            raise ValueError("sharpness value has to be between 0 and 4")
            
        ctrl = dai.CameraControl()
        ctrl.setSharpness(value)
        logging.info(f"Setting runtime sharpness for {camera_type.name if camera_type else 'all'} to {value}")

        if (camera_type is None or camera_type == self.left.camera_type) and hasattr(self, 'left_input_queue'):
            self.left_input_queue.send(ctrl)
        if (camera_type is None or camera_type == self.right.camera_type) and hasattr(self, 'right_input_queue'):
            self.right_input_queue.send(ctrl)
        if self.center is not None and (camera_type is None or camera_type == self.center.camera_type) and hasattr(self, 'center_input_queue'):
            self.center_input_queue.send(ctrl)