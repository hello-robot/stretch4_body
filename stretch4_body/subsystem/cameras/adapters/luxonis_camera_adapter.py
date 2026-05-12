"""
Adapter for connecting and controlling a single Luxonis camera.
"""
import time

import numpy as np
import depthai as dai
import logging

from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameraConfig, RGBCameras
from stretch4_body.subsystem.cameras.adapters.camera_adapter import CameraAdapter
from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame

import tempfile
import json
import os

CACHE_FILE_PATH = os.path.join(tempfile.gettempdir(), f"luxonis_device_cache_{os.getuid()}.json")
CACHE_EXPIRY_SECONDS = 24 * 60 * 60  # Invalidate after 24 hours

def _load_cache():
    try:
        if os.path.exists(CACHE_FILE_PATH):
            with open(CACHE_FILE_PATH, 'r') as f:
                return json.load(f)
    except Exception as e:
        print(f"Warning: Failed to load device cache: {e}")
    return {}

def _save_cache(cache_data):
    try:
        with open(CACHE_FILE_PATH, 'w') as f:
            json.dump(cache_data, f)
    except Exception as e:
        print(f"Warning: Failed to save device cache: {e}")

def clear_device_cache():
    try:
        if os.path.exists(CACHE_FILE_PATH):
            os.remove(CACHE_FILE_PATH)
            print(f"Cleared cache at {CACHE_FILE_PATH}")
    except Exception as e:
        print(f"Warning: Failed to clear device cache: {e}")

def get_device_port_by_product_name(product_name:str):
    """When multiple Luxonis cameras are connected, we should query their serial number to use them."""
    devices = dai.Device.getAllAvailableDevices()

    cache_data = _load_cache()
    current_time = time.time()

    # Check cache first
    for info in devices:
        cached_device = cache_data.get(info.deviceId)
        if cached_device:
            # Check expiry
            if current_time - cached_device.get("timestamp", 0) <= CACHE_EXPIRY_SECONDS:
                if cached_device.get("product_name") == product_name:
                    print(f"Found cached device for {product_name=}")
                    return info.name

    # If not found in cache, fallback and update cache
    for info in devices:
        # Skip if we already have valid cache for this specific device
        cached_device = cache_data.get(info.deviceId)
        if cached_device and current_time - cached_device.get("timestamp", 0) <= CACHE_EXPIRY_SECONDS:
            continue

        with dai.Device(maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS, nameOrDeviceId=info.deviceId) as device:
            actual_product_name = device.getProductName()
            print(f"Found device {actual_product_name}. Looking for {product_name=}")
            
            cache_data[info.deviceId] = {
                "product_name": actual_product_name,
                "timestamp": current_time,
                "name": info.name
            }
            _save_cache(cache_data)

            if product_name == actual_product_name:
                # return info.deviceId
                device.close()
                time.sleep(1) # Sleep is needed so that device goes back to ready state, it's quite slow. TODO: cache this value.
                return info.name
            
    raise RuntimeError(f"Could not find the {product_name} device. Is it connected?")


class LuxonisCameraAdapter(CameraAdapter):
    """This adapter has the logic to interface with the Luxonis DepthAI api to build and open camera streams."""


    @staticmethod
    def get_depthai_camera_socket(camera_type: RGBCameras):
        if camera_type == RGBCameras.head_left:
            return dai.CameraBoardSocket.CAM_C
        if camera_type == RGBCameras.head_right:
            return dai.CameraBoardSocket.CAM_B
        if camera_type == RGBCameras.head_center:
            return dai.CameraBoardSocket.CAM_A
        
        if camera_type == RGBCameras.gripper_left:
            return dai.CameraBoardSocket.CAM_B
        if camera_type == RGBCameras.gripper_right:
            return dai.CameraBoardSocket.CAM_C

        raise Exception(f"{camera_type} is not supported as a Luxonis device.")

    @staticmethod
    def create_camera_node(pipeline: dai.Pipeline, camera_config: RGBCameraConfig) -> tuple[dai.node.Camera, dai.Node.Output]:
        """
        Takes a dai.Pipeline reference and adds a camera node to it.
        """
        board_socket = LuxonisCameraAdapter.get_depthai_camera_socket(
            camera_type=camera_config.camera_type
        )

        buffer_size = camera_config.buffer_size
        fps = camera_config.fps
        node = pipeline.create(dai.node.Camera)
        node.setNumFramesPools(isp=buffer_size, raw=buffer_size, imgmanip=buffer_size)
        node.setSensorType(dai.CameraSensorType.COLOR)
        node.build(boardSocket=board_socket, sensorFps=fps)

        if camera_config.use_auto_exposure:
            node.initialControl.setAutoExposureEnable()
            if camera_config.limit_max is not None:
                node.initialControl.setAutoExposureLimit(camera_config.limit_max)
            logging.info(f"Setting auto exposure for {camera_config.camera_type} with {camera_config.limit_max=}")
        else:
            if camera_config.exposure_time is None or camera_config.iso is None:
                raise ValueError("exposure_time and iso must be set when use_auto_exposure is False")
            logging.info(f"Setting manual exposure for {camera_config.camera_type} to {camera_config.exposure_time=} and {camera_config.iso=}")
            node.initialControl.setManualExposure(camera_config.exposure_time, camera_config.iso)
            

        camera_output = node.requestOutput(
            size=camera_config.image_size[::-1],
            fps=fps,
            type=dai.ImgFrame.Type.NV12,
            resizeMode=dai.ImgResizeMode.CROP,
            enableUndistortion=False,
        )

        print("camera_config:", camera_config)
        if camera_config.is_compressed:
            videoEncoder = pipeline.create(dai.node.VideoEncoder)
            videoEncoder.setNumFramesPool(buffer_size)
            videoEncoder.build(
                camera_output, 
                frameRate=fps, 
                profile=dai.VideoEncoderProperties.Profile.MJPEG, 
                lossless=camera_config.is_lossless, 
                quality=camera_config.jpeg_quality
            )
            camera_output = videoEncoder.bitstream

        return node, camera_output

    @staticmethod
    def create_pipeline(luxonis_device_product_name:str) -> tuple[dai.Pipeline, dai.Device]:
        device_port = get_device_port_by_product_name(luxonis_device_product_name)
        device = dai.Device(maxUsbSpeed=dai.UsbSpeed.SUPER_PLUS, nameOrDeviceId=device_port)
        pipeline = dai.Pipeline(defaultDevice=device)

        print("DeviceID:", device.getDeviceInfo().getDeviceId())
        print("USB speed:", device.getUsbSpeed())
        print("Connected cameras:", device.getConnectedCameras())

        pipeline.setXLinkChunkSize(0)

        return pipeline, device
    
    @staticmethod
    def create_rgbd_node(pipeline: dai.Pipeline, left_rgb_output: dai.Node.Output, right_rgb_out: dai.Node.Output):

        stereo = pipeline.create(dai.node.StereoDepth).build(left_rgb_output, right_rgb_out)
        rgbd = pipeline.create(dai.node.RGBD)

        stereo.setRectifyEdgeFillColor(0)
        stereo.enableDistortionCorrection(True)
        # https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/stereo_depth
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
        stereo.initialConfig.postProcessing.thresholdFilter.maxRange = 1000 * 5 # in mm
        rgbd.setDepthUnits(dai.StereoDepthConfig.AlgorithmControl.DepthUnit.METER)

        stereo.syncedLeft.link(rgbd.inColor)
        stereo.depth.link(rgbd.inDepth)
        left_rgb_output.link(stereo.inputAlignTo)

        return stereo, rgbd

    def is_open(self):
        return self.pipeline is not None and self.device is not None and self.pipeline.isRunning() and not self.device.isClosed()

    def open(self):
        self.pipeline, self.device = LuxonisCameraAdapter.create_pipeline(self.camera_config.camera_device)
        self.camera = self.pipeline

        self.camera_node, node_output = LuxonisCameraAdapter.create_camera_node(
            pipeline=self.pipeline, camera_config=self.camera_config
        )

        self.output_queue = node_output.createOutputQueue(maxSize=1)
        self.input_queue = self.camera_node.inputControl.createInputQueue()

        try:
            self.pipeline.start()
        except Exception:
            clear_device_cache()
            raise

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


    @staticmethod
    def dai_message_to_image_frame(
        message: dai.ImgFrame,
    ):
        # https://docs.luxonis.com/hardware/platform/deploy/frame-sync/
        timestamp = message.getTimestamp().total_seconds() # Timestamp synced with the host computer clock
        sequence_number = message.getSequenceNum()
        
        if message.getType() == dai.ImgFrame.Type.BITSTREAM:
            """If the luxonis pipeline returns a compressed image, it will not be compressed here, just passed as is."""
            return ImageFrame(image = message.getData(), timestamp=timestamp, frame_number=sequence_number, compression_format="jpeg")
        else:
            color_image = message.getCvFrame()
            return ImageFrame( image=color_image, timestamp=timestamp, frame_number=sequence_number)

    @staticmethod
    def get_frame_from_output_queue(
        output_queue: dai.MessageQueue
    ):
        while True:
            try:
                message: dai.ImgFrame = output_queue.get()
                yield LuxonisCameraAdapter.dai_message_to_image_frame(message)
            except Exception as e:
                logging.error(f"Error getting frame from output queue: {e}")

    @staticmethod
    def get_frame_from_output_queue_no_block(
        output_queue: dai.MessageQueue,
        timeout: float = 1/120
    ):
        while True:
            message: dai.ImgFrame | None = output_queue.get(timeout=timeout)
            if message:
                yield LuxonisCameraAdapter.dai_message_to_image_frame(message)
            else:
                yield None


    @staticmethod
    def get_pointcloud_from_output_queue(
        output_queue: dai.MessageQueue
    ):
        while True:
            message = output_queue.get()
            if message:
                # time_stamp = time.monotonic()
                # https://docs.luxonis.com/hardware/platform/deploy/frame-sync/
                # time_stamp = message.getTimestamp().total_seconds() # Timestamp synced with the host computer clock
                sequence_number = message.getSequenceNum()
                points, colors = message.getPointsRGB()

                # latencyMs = (dai.Clock.now() - message.getTimestamp()).total_seconds() * 1000
                # diffs = np.append(diffs, latencyMs)
                # print(f"Latency: {latencyMs} ms")

                # yield color_image, time_stamp
                yield points, colors, sequence_number

            # print(f"Dropped frame {output_queue.getName()}")

    def get_frames(self):
        if not self.is_open():
            raise RuntimeError("Camera is not running.")
            
        return LuxonisCameraAdapter.get_frame_from_output_queue(
            self.output_queue
        )
    
    def get_next(self):
        return next(self.get_frames())

    def focus_roi(self, roi: list[int], camera_type: RGBCameras | None = None):
        if not hasattr(self, 'input_queue'):
            return
        print(f"Setting roi {roi} for {camera_type.name}")
        ctrl = dai.CameraControl()
        ctrl.setAutoExposureRegion(*roi)
        ctrl.setAutoFocusRegion(*roi)
        self.input_queue.send(ctrl)

    def set_manual_exposure(self, exposure_time: int, iso: int, camera_type: RGBCameras | None = None):
        """
        exposure_time: Exposure time
        iso: Sensitivity as ISO value, usual range 100..1600
        """
        if not hasattr(self, 'input_queue'):
            return
        if iso < 100 or iso > 1600:
            raise ValueError("iso value has to be between 100 and 1600")
        logging.info(f"Setting runtime manual exposure for {self.camera_config.camera_type} to {exposure_time=} and {iso=}")
        ctrl = dai.CameraControl()
        ctrl.setManualExposure(exposure_time, iso)
        self.input_queue.send(ctrl)

    def set_auto_exposure(self, limit_max: int | None = None, camera_type: RGBCameras | None = None):
        if not hasattr(self, 'input_queue'):
            return
        logging.info(f"Setting runtime auto exposure for {self.camera_config.camera_type} with {limit_max=}")
        ctrl = dai.CameraControl()
        ctrl.setAutoExposureEnable()
        if limit_max is not None:
            ctrl.setAutoExposureLimit(limit_max)
        self.input_queue.send(ctrl)

    def set_manual_white_balance(self, color_temperature: int, camera_type: RGBCameras | None = None):
        """
        Set manual white balance.
        
        Args:
            color_temperature: Light source color temperature in kelvins, between 1000 and 12000.
            camera_type: The camera to apply this to.
        """
        if not hasattr(self, 'input_queue'):
            return
        if color_temperature < 1000 or color_temperature > 12000:
            raise ValueError("color_temperature value has to be between 1000 and 12000")
        logging.info(f"Setting runtime manual white balance for {self.camera_config.camera_type} to {color_temperature=}K")
        ctrl = dai.CameraControl()
        ctrl.setManualWhiteBalance(color_temperature)
        self.input_queue.send(ctrl)

    def set_auto_white_balance(self, camera_type: RGBCameras | None = None):
        if not hasattr(self, 'input_queue'):
            return
        logging.info(f"Setting runtime auto white balance for {self.camera_config.camera_type}")
        ctrl = dai.CameraControl()
        ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
        self.input_queue.send(ctrl)

    def set_brightness(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image brightness.
        
        Args:
            value: Brightness, range -10..10, default 0
            camera_type: The camera to apply this to.
        """
        if not hasattr(self, 'input_queue'):
            return
        if value < -10 or value > 10:
            raise ValueError("brightness value has to be between -10 and 10")
        logging.info(f"Setting runtime brightness for {self.camera_config.camera_type} to {value}")
        ctrl = dai.CameraControl()
        ctrl.setBrightness(value)
        self.input_queue.send(ctrl)

    def set_contrast(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image contrast.
        
        Args:
            value: Contrast, range -10..10, default 0
            camera_type: The camera to apply this to.
        """
        if not hasattr(self, 'input_queue'):
            return
        if value < -10 or value > 10:
            raise ValueError("contrast value has to be between -10 and 10")
        logging.info(f"Setting runtime contrast for {self.camera_config.camera_type} to {value}")
        ctrl = dai.CameraControl()
        ctrl.setContrast(value)
        self.input_queue.send(ctrl)

    def set_saturation(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image saturation.
        
        Args:
            value: Saturation, range -10..10, default 0
            camera_type: The camera to apply this to.
        """
        if not hasattr(self, 'input_queue'):
            return
        if value < -10 or value > 10:
            raise ValueError("saturation value has to be between -10 and 10")
        logging.info(f"Setting runtime saturation for {self.camera_config.camera_type} to {value}")
        ctrl = dai.CameraControl()
        ctrl.setSaturation(value)
        self.input_queue.send(ctrl)

    def set_sharpness(self, value: int, camera_type: RGBCameras | None = None):
        """
        Set image sharpness.
        
        Args:
            value: Sharpness, range 0..4, default 1
            camera_type: The camera to apply this to.
        """
        if not hasattr(self, 'input_queue'):
            return
        if value < 0 or value > 4:
            raise ValueError("sharpness value has to be between 0 and 4")
        logging.info(f"Setting runtime sharpness for {self.camera_config.camera_type} to {value}")
        ctrl = dai.CameraControl()
        ctrl.setSharpness(value)
        self.input_queue.send(ctrl)