"""
Adapter for connecting to and controlling the Luxonis head cameras using the DepthAI API for the Luxonis OAK-FFC 3P board.
"""

import time
import threading
import logging
import collections
import depthai as dai
from stretch4_body.subsystem.cameras.cv_utils import RectifyMaps
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameraConfig, RGBCameras
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

        self.left_camera_node, node_left = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=left)
        self.right_camera_node, node_right = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=right)
   
        if center is not None:
            self.center_camera_node, node_center = LuxonisCameraAdapter.create_camera_node(pipeline=self.pipeline, camera_config=center)
            self.center_input_queue = self.center_camera_node.inputControl.createInputQueue()
        
        self.left_input_queue = self.left_camera_node.inputControl.createInputQueue()
        self.right_input_queue = self.right_camera_node.inputControl.createInputQueue()
        self.center_buffer = collections.deque(maxlen=10)

        if self.do_sync_frames:
            sync = self.pipeline.create(dai.node.Sync)
            buffer_size = left.buffer_size

            sync.inputs["left"].setMaxSize(1)
            sync.inputs["left"].setBlocking(False)
            node_left.link(sync.inputs["left"])

            sync.inputs["right"].setMaxSize(1)
            sync.inputs["right"].setBlocking(False)
            node_right.link(sync.inputs["right"])

            # CENTER is NOT in sync to keep it at its own 10fps while left/right are at 30fps
            if center is not None:
                self.center_output = node_center.createOutputQueue(maxSize=1, blocking=False)

            self.q_sync = sync.out.createOutputQueue(maxSize=buffer_size, blocking=False)
        else:
            if center is not None:
                self.center_output = node_center.createOutputQueue(maxSize=1, blocking=False)
            self.left_output = node_left.createOutputQueue(maxSize=1, blocking=False)
            self.right_output = node_right.createOutputQueue(maxSize=1, blocking=False)

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

                left_frame = LuxonisCameraAdapter.dai_message_to_image_frame(left_msg)
                right_frame = LuxonisCameraAdapter.dai_message_to_image_frame(right_msg)

                center_frame = None
                if self.center is not None:
                    # Drain center queue into buffer
                    while True:
                        c_frame = next(LuxonisCameraAdapter.get_frame_from_output_queue_no_block(self.center_output))
                        if c_frame is None:
                            break
                        self.center_buffer.append(c_frame)
                    
                    # Find best match in center_buffer to keep center synced with left/right frames
                    if self.center_buffer:
                        lr_timestamp = left_msg.getTimestamp().total_seconds()
                        
                        # Find the frame with the closest timestamp
                        # We also discard frames that are significantly older than the current LR frame
                        best_diff = float('inf')
                        best_idx = -1
                        for i, c_f in enumerate(self.center_buffer):
                            diff = abs(c_f.timestamp - lr_timestamp)
                            if diff < best_diff:
                                best_diff = diff
                                best_idx = i
                        
                        # If a match is found within a reasonable window
                        if best_idx != -1 and best_diff < 1/self.center.fps:
                            center_frame = self.center_buffer[best_idx]
                            # Discard the used frame and all older ones
                            for _ in range(best_idx + 1):
                                self.center_buffer.popleft()
                        elif best_idx != -1 and self.center_buffer[0].timestamp < lr_timestamp - 0.1:
                            # Discard frames that are way too old
                            while self.center_buffer and self.center_buffer[0].timestamp < lr_timestamp - 0.1:
                                self.center_buffer.popleft()

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
        print(f"Setting roi {roi} for {camera_type.name if camera_type else 'all'}")
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