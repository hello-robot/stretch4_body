from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels
from stretch4_body.subsystem.cameras.adapters.synced_camera import SyncedCamera
from stretch4_body.subsystem.cameras.adapters.camera_adapter import CameraAdapter
from enum import Enum
import queue
import threading
from collections.abc import Generator
import numpy as np
from stretch4_body.subsystem.cameras.detectors.detector_ai_models import AIModelWrapper, do_object_detection
from stretch4_body.subsystem.cameras.detectors.detector_aruco import do_aruco_detection
import cv2
import rerun as rr

from stretch4_body.subsystem.cameras.enums.aruco_dictionary import ArucoDictionary
from stretch4_body.subsystem.cameras.models.camera_calibration import (
    RGBCameraCalibration,
)
from stretch4_body.subsystem.cameras.cv_utils import (
    RectifyMaps,
    get_recify_maps,
    rectify,
)
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.image_write_to_disk import RgbImageToWriteToDisk, add_image_to_save_queue, create_directory_if_it_does_not_exist, saver_thread
from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame, SyncedImageFrame


class RecordRgbShowImageIn(Enum):
    """Decide the program to show images in: Rerun or OpenCV Imshow"""
    RERUN = 0
    CVIMSHOW = 1


class RGBPipelineController:
    """Initialized with the info required to setup a pipeline to capture and post-process an image captured from a camera device
    to do image saving, cropping, rotating, object detection, aruco detection, and more."""

    def __init__(
        self,
        camera_type: "RGBCameras",
        recording_directory: str | None,
        show_image_in: "RecordRgbShowImageIn | None",
        is_rotate: bool,
        is_rectify: bool,
        is_crop: bool,
        ai_models_to_use: list[AIModelWrapper],
        detect_aruco_marker_size: float|None,
        is_open_camera: bool = True
    ):
        """
        `detect_aruco_marker_size`: Runs ArUco detection if a float >= 0.0 is provided. If length is 0.0, the ArUco markers will be detected, but distance will not be printed. If length > 0.0 and calibration is available, ArUco pose and L2 distance to the marker will be displayed.
        """
        self.recording_directory = recording_directory
        self.camera_type = camera_type
        self.show_image_in = show_image_in
        self.is_rectify = is_rectify
        self.is_crop = is_crop
        self.ai_models_to_use = ai_models_to_use
        self.detect_aruco_marker_size = detect_aruco_marker_size
        self.detect_aruco_dictionaries_to_detect = ArucoDictionary.all_1000()
        self.is_rotate = is_rotate

        self.ai_models: list[AIModelWrapper] = []
        self.rectify_maps: "RectifyMaps | None" = None
        self._camera: CameraAdapter|SyncedCamera|None= None

        self.frame_number = 0

        self.camera_calibration:RGBCameraCalibration|None = None
        try:
            self.camera_calibration = RGBCameraCalibration.load_calibration_from_fleet_path(
                camera_type=self.camera_type, is_flip_width_and_height=False
            )
        except Exception as e: ...

        self.save_rgb_queue: queue.Queue[RgbImageToWriteToDisk] = queue.Queue(maxsize=500)

        self.stop_event = threading.Event()

        self.save_thread = threading.Thread(
            target=saver_thread, args=(self.stop_event, self.save_rgb_queue), daemon=True
        )

        self.save_directory = None
        if self.recording_directory:
             if not self.camera_type.is_synced_camera_type():
                # We don't need to create folders for is_synced_camera_type, 
                # they get their own instances of left/right/center in the pipeline, 
                # and their folders get created on init.
                self.save_directory, time_string = create_directory_if_it_does_not_exist(
                    self.recording_directory, self.camera_type
                )

        if is_open_camera:
            self.open_camera()

    def open_camera(self):
        if self.camera_type.is_synced_camera_type():
            self._camera = self.camera_type.start_synced()
        else:
            self._camera = self.camera_type.start()

    @property
    def camera(self):
        if self._camera is None:
            raise Exception("Camera is not initialized. Did you call start()?")
        return self._camera

    def _show_rererun(self, color_image: np.ndarray):
        rr.log(
            f"{self.camera_type.name.upper()} Camera",
            rr.Image(color_image, color_model="BGR").compress(),
        )

    def _show_cvimshow(self, color_image: np.ndarray):
        cv2.namedWindow(self.camera_type.name, cv2.WINDOW_NORMAL)
        cv2.imshow(self.camera_type.name, color_image)
        if cv2.getWindowProperty(self.camera_type.name, cv2.WND_PROP_VISIBLE) < 1:
            self.stop_event.set()

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop_event.set()


    def show_image(self, color_image: np.ndarray):
        if self.show_image_in is RecordRgbShowImageIn.RERUN:
            self._show_rererun(color_image)
        elif self.show_image_in is RecordRgbShowImageIn.CVIMSHOW:
            self._show_cvimshow(color_image)

    def _run_object_detection(self, color_image: np.ndarray, frame: ImageFrame):        
        output_image, results = do_object_detection(
            color_image=color_image,
            ai_models=self.ai_models_to_use,
            is_visualize=self.show_image_in is not None
        )
        frame.ai_model_results = results
        return output_image

    def _run_aruco_detection(self, color_image: np.ndarray):
        color_image = do_aruco_detection(color_image, camera_calibration=self.camera_calibration, marker_length=self.detect_aruco_marker_size, dictionaries_to_detect=self.detect_aruco_dictionaries_to_detect)

        return color_image

    def _run_rectify(self, color_image: np.ndarray):        
        if self.camera_calibration is None:
            raise Exception("Camera calibration is None.")
        
        if self.camera_calibration.distortion_model.is_fisheye() and not self.camera_calibration.distortion_model is DistortionModels.omnidir:
            if self.rectify_maps is None:
                # Compute rectify maps once
                balance = 0.0
                fov_scale = 0.8
                self.rectify_maps = get_recify_maps(
                    color_image,
                    sim_cam_matrix=self.camera_calibration.camera_matrix,
                    sim_cam_distortion_coeffs=self.camera_calibration.distortion_coefficients,
                    balance=balance,
                    fov_scale=fov_scale,
                )

        return rectify(color_image, self.camera_calibration.camera_matrix, self.camera_calibration.distortion_coefficients,self.camera_calibration.distortion_model, rectify_maps=self.rectify_maps)

    def _run_crop(self, color_image: np.ndarray):
        h, w = color_image.shape[:2]
        width_crop_percentage_left = 0.2
        width_crop_percentage_right = 0.1
        if self.camera_type.is_right():
            width_crop_percentage_left = 0.15
            width_crop_percentage_right = 0.15
        height_crop_percentage_up = 0.20
        height_crop_percentage_down = 0.1
        crop_x = int(w * width_crop_percentage_left)
        crop_y = int(h * height_crop_percentage_up)
        crop_x_end = w - int(w * width_crop_percentage_right)
        crop_y_end = h - int(h * height_crop_percentage_down)
        color_image = color_image[crop_y:crop_y_end, crop_x:crop_x_end]

        return color_image

    def save_frame(self, color_image:np.ndarray, rgb_timestamp:float,):
        if self.save_directory is None: return

        add_image_to_save_queue(
            directory=self.save_directory,
            camera_type=self.camera_type,
            color_image=color_image,
            rgb_timestamp=rgb_timestamp,
            frame_number=self.frame_number,
            save_rgb_queue=self.save_rgb_queue,
        )

    def run_pipeline(self, frame: ImageFrame):
        """Runs the image pipeline that handles uncompressing, cropping, rectifying, calibration, object detection, ArUco detection, etc.."""

        color_image = frame.uncompress() if frame.is_compressed() else frame.image

        frame.image_raw = color_image.copy()
        
        if self.is_rectify:
            return self._run_rectify(
            color_image=frame.image,
        )
        
        if self.is_rotate and self.camera_type.config.rotate_number_of_times:
            color_image = np.rot90(color_image, k=self.camera_type.config.rotate_number_of_times)

        # Note: Running crop after rotation would mean crop params are relative to the rotated image.
        if self.is_crop:
            return self._run_crop(color_image=color_image)


        if self.detect_aruco_marker_size is not None:
            return self._run_aruco_detection(color_image)
        

        if self.ai_models_to_use:
            return self._run_object_detection(color_image, frame)

        self.save_frame(color_image=color_image, rgb_timestamp=frame.timestamp)

        frame.image = color_image
    
    def _start_rerun(self):
        rr.init(self.camera_type.name, spawn=False)
        rr.spawn(memory_limit="4GB")

        import rerun.blueprint as rrb

        if self.camera_type.is_synced_camera_type():
            blueprint = rrb.Blueprint(
                rrb.Horizontal(
                    rrb.Spatial2DView(name="Left Camera", origin="HEAD_LEFT Camera"),
                    rrb.Spatial2DView(name="Center Camera", origin="HEAD_CENTER Camera"),
                    rrb.Spatial2DView(name="Right Camera", origin="HEAD_RIGHT Camera"),
                ),
                collapse_panels=True
            )
        else:
            blueprint = rrb.Blueprint(
                rrb.Spatial2DView(name=f"{self.camera_type.name} Camera", origin=f"{self.camera_type.name.upper()} Camera"),
                collapse_panels=True
            )
        rr.send_blueprint(blueprint)
        # rr.init(camera_type.name, spawn=True)
    
    def get_frame_synced(self, is_run_pipeline: bool) -> Generator[SyncedImageFrame, None, None]:
        """
        Starts the capture loop for synced imagery. This means that left/right cameras are being captured and synced. Sometimes the center camera is being recorded as well.
        Yields the captured frames.
        """
        if self._camera is None:
            raise RuntimeError("Camera is not opened. Did you call rgb_pipeline_controller.camera_open?")

        if not self._camera.is_open():
            raise RuntimeError("Camera is not open. Did you call rgb_pipeline_controller.camera_open()?")

        if self.stop_event.is_set():
            raise RuntimeError("The stop event is set, the pipeline should have stopped.")
        
        if not self.camera_type.is_synced_camera_type():
            raise Exception("get_frame_synced() can only be called for a synced camera type. Use get_frame() instead.")
        
        if not isinstance(self._camera, SyncedCamera):
            raise RuntimeError(f"The opened camera is of the wrong type. Expected a SyncedCamera instance, but got {type(self._camera)}")
        
        if self.show_image_in is RecordRgbShowImageIn.RERUN:
            self._start_rerun()

        left = RGBCameras.left()
        right = RGBCameras.right()
        center = RGBCameras.center()

        if self.camera_type is RGBCameras.gripper_rgbd:
            left = RGBCameras.gripper_left
            right = RGBCameras.gripper_right

        left_pipeline_controller = self.copy_for(left, is_open_camera=False)
        right_pipeline_controller = self.copy_for(right, is_open_camera=False)
        use_center = self.camera_type == RGBCameras.head_left_right_center
        center_pipeline_controller = None
        if use_center:
            center_pipeline_controller = self.copy_for(center, is_open_camera=False)
            center_pipeline_controller.is_crop = False # Hack / baked in logic - we don't normally want to crop center image.

        if self.recording_directory is not None:
            left_pipeline_controller.save_thread.start()
            right_pipeline_controller.save_thread.start()
            if center_pipeline_controller is not None:
                center_pipeline_controller.save_thread.start()

        for frame in self._camera.get_frames():

            if self.stop_event.is_set():
                break

            if is_run_pipeline and frame is not None:
                left_pipeline_controller.run_pipeline(frame.left)
                right_pipeline_controller.run_pipeline(frame.right)
                if frame.center is not None:
                    center_pipeline_controller.run_pipeline(frame.center)

            left_pipeline_controller.show_image(frame.left.image)
            right_pipeline_controller.show_image(frame.right.image)

            if frame.center is not None:
                center_pipeline_controller.show_image(frame.center.image)

            if frame.depth is not None and self.show_image_in is RecordRgbShowImageIn.RERUN:
                rr.log(
                    f"{self.camera_type.name}/depth",
                    rr.DepthImage(frame.depth),
                )

            if frame.pointcloud is not None and self.show_image_in is RecordRgbShowImageIn.RERUN:
                rr.log(
                    f"{self.camera_type.name}/pcl",
                    rr.Transform3D(
                        rotation=rr.RotationAxisAngle(axis=[0, 0, 1], angle=rr.Angle(rad=np.pi)),
                    )
                )
                rr.log(f"{self.camera_type.name}/pcl", rr.Points3D(frame.pointcloud, colors=frame.pointcloud_color, radii=[0.002]))

            yield frame

            self.frame_number += 1


    def get_frame(self, is_run_pipeline: bool) -> Generator[ImageFrame, None, None]:
        """
        if `is_run_pipeline` is true, the captured image will be fed into the pipeline. Otherwise, the raw image is returned.
        Yields the captured frames.
        """
        if self._camera is None:
            raise RuntimeError("Camera is not opened. Did you call rgb_pipeline_controller.camera_open?")

        if self.camera_type.is_synced_camera_type():
            raise RuntimeError(f"{self.camera_type} is a synced camera, use get_frame_synced() instead.")
        
        if self.stop_event.is_set():
            raise RuntimeError("The stop event is set, the pipeline should have stopped.")
        
        if not isinstance(self._camera, CameraAdapter):
            raise RuntimeError(f"The opened camera is of the wrong type. Expected a CameraAdapter instance, but got {type(self._camera)}")
        
        if self.show_image_in is RecordRgbShowImageIn.RERUN:
            self._start_rerun()

        if self.recording_directory is not None:
            self.save_thread.start()

        for frame in self._camera.get_frames():

            if self.stop_event.is_set():
                break

            if is_run_pipeline and frame is not None:
                self.run_pipeline(frame)

            self.show_image(frame.image)

            yield frame

            self.frame_number += 1
            
    def copy_for(self, camera_type: RGBCameras, is_open_camera: bool):
        """Create a copy with this camera type"""
        copy = RGBPipelineController(
            camera_type=camera_type,
            recording_directory=self.recording_directory,
            show_image_in=self.show_image_in,
            is_rotate=self.is_rotate,
            is_rectify=self.is_rectify,
            is_crop=self.is_crop,
            ai_models_to_use=self.ai_models_to_use,
            detect_aruco_marker_size=self.detect_aruco_marker_size,
            is_open_camera=is_open_camera
        )
        return copy


    def focus_roi(self, roi: list[int] ):
        self.camera.focus_roi(roi)

    def set_calibration_exposure_preset(self):
        """
        These exposure settings work best for 450-650 lux ambient lighting.
        This was tested by using the max brightness and the white light setting on the 
        2800-6500K Dimmable Photography Light Panels set 2ft horizontally from the mast on either side of the robot.
        The height of the bottom of each light panel is 5ft from the floor.
        The panels are angled 45 degrees toward the charuco board vertically, about 10 degrees toward the floor.
        """
        self.set_manual_exposure_left_right(int(4000), 100)
        self.set_manual_exposure_center(int(10000), 200)

    
    def set_manual_exposure(self, exposure_time: int, iso: int):
        """
        exposure_time: Exposure time
        iso: Sensitivity as ISO value, usual range 100..1600
        """
        self.camera.set_manual_exposure(exposure_time, iso)

    def set_manual_exposure_left_right(self, exposure_time: int, iso: int):
        """
        exposure_time: Exposure time
        iso: Sensitivity as ISO value, usual range 100..1600
        """
        self.camera.set_manual_exposure(exposure_time, iso, RGBCameras.left())
        self.camera.set_manual_exposure(exposure_time, iso, RGBCameras.right())

    def set_manual_exposure_center(self, exposure_time: int, iso: int):
        """
        exposure_time: Exposure time
        iso: Sensitivity as ISO value, usual range 100..1600
        """
        self.camera.set_manual_exposure(exposure_time, iso, RGBCameras.center())
    
    def set_auto_exposure(self, limit_max: int | None = None):
        self.camera.set_auto_exposure(limit_max)
    
    def set_manual_white_balance(self, color_temperature: int):
        """
        Set manual white balance.
        
        Args:
            color_temperature: Light source color temperature in kelvins, between 1000 and 12000.
            camera_type: The camera to apply this to.
        """
        self.camera.set_manual_white_balance(color_temperature)
    
    def set_auto_white_balance(self):
        self.camera.set_auto_white_balance()

    def set_brightness(self, value: int):
        """
        Set image brightness.
        
        Args:
            value: Brightness, range -10..10, default 0
        """
        self.camera.set_brightness(value)

    def set_contrast(self, value: int):
        """
        Set image contrast.
        
        Args:
            value: Contrast, range -10..10, default 0
        """
        self.camera.set_contrast(value)

    def set_saturation(self, value: int):
        """
        Set image saturation.
        
        Args:
            value: Saturation, range -10..10, default 0
        """
        self.camera.set_saturation(value)

    def set_sharpness(self, value: int):
        """
        Set image sharpness.
        
        Args:
            value: Sharpness, range 0..4, default 1
        """
        self.camera.set_sharpness(value)

    def stop(self):
        self.stop_event.set()
        if self._camera is not None:
            self._camera.stop()
        if self.save_thread.is_alive():
            self.save_thread.join(timeout=5)

class RGBPipelineControllerROS(RGBPipelineController):
    """
    A specialized controller that leverages stretch_python_bridge's StreamManager for camera streams.
    This adapter allows using camera tools with ROS2 camera nodes.
    """
    def __init__(self, camera_type: "RGBCameras", recording_directory: str | None, show_image_in: "RecordRgbShowImageIn | None", is_rotate: bool, is_rectify: bool, is_crop: bool, ai_models_to_use: list[AIModelWrapper], detect_aruco_marker_size: float|None, is_open_camera: bool = True):
        super().__init__(
            camera_type=camera_type,
            recording_directory=recording_directory,
            show_image_in=show_image_in,
            is_rotate=is_rotate,
            is_rectify=is_rectify,
            is_crop=is_crop,
            ai_models_to_use=ai_models_to_use,
            detect_aruco_marker_size=detect_aruco_marker_size,
            is_open_camera=False
        )
        
        try:
            from stretch_python_bridge.stream_manager import StreamManager
        except ImportError:
            raise ImportError("stretch_python_bridge not found. Did you colcon build? Please source ROS 2 workspace.")
            
        self.stream_manager = StreamManager()

        self.generator = None

    def copy_for(self, camera_type: "RGBCameras", is_open_camera: bool):
        copy = RGBPipelineControllerROS(
            camera_type=camera_type,
            recording_directory=self.recording_directory,
            show_image_in=self.show_image_in,
            is_rotate=self.is_rotate,
            is_rectify=self.is_rectify,
            is_crop=self.is_crop,
            ai_models_to_use=self.ai_models_to_use,
            detect_aruco_marker_size=self.detect_aruco_marker_size,
            is_open_camera=is_open_camera
        )
        return copy

    def get_frame(self, is_run_pipeline: bool) -> Generator[ImageFrame, None, None]:
        if self.camera_type.is_synced_camera_type():
            raise RuntimeError(f"{self.camera_type} is a synced camera, use get_frame_synced() instead.")
        
        try:
            from stretch_python_bridge import stream_camera_left, stream_camera_right, stream_camera_center
        except ImportError:
            raise ImportError("stretch_python_bridge not found. Did you colcon build? Please source ROS 2 workspace.")
        
        if self.show_image_in is RecordRgbShowImageIn.RERUN:
            self._start_rerun()

        if self.recording_directory is not None:
            self.save_thread.start()

        camera_generator = None
        if self.camera_type.is_left():
            camera_generator = stream_camera_left(stream_manager=self.stream_manager)
        elif self.camera_type.is_right():
            camera_generator = stream_camera_right(stream_manager=self.stream_manager)
        elif self.camera_type.is_center():
            camera_generator = stream_camera_center(stream_manager=self.stream_manager)
        else:
            raise NotImplementedError(f"Camera {self.camera_type} is not supported.")

        self.generator = self.stream_manager.stream()

        try:
            for _ in self.generator:
                if self.stop_event.is_set():
                    break

                ros_frame = self.stream_manager.get(camera_generator)
                if ros_frame is None:
                    print("ros_frame is None in get_frame")
                    continue

                frame = ImageFrame(timestamp=ros_frame.timestamp, frame_number=self.frame_number, image=ros_frame.image, timestamp_system=ros_frame.timestamp_system)

                if is_run_pipeline:
                    self.run_pipeline(frame)

                self.show_image(frame.image)
                yield frame

                self.frame_number += 1
        finally:
            self.stop()
            if self.save_thread.is_alive():
                self.save_thread.join(timeout=5)

    def get_frame_synced(self, is_run_pipeline: bool) -> Generator[SyncedImageFrame, None, None]:
        if not self.camera_type.is_synced_camera_type():
            raise Exception("get_frame_synced() can only be called for a synced camera type. Use get_frame() instead.")
        
        try:
            from stretch_python_bridge import stream_camera_left, stream_camera_right, stream_camera_center
        except ImportError:
            raise ImportError("stretch_python_bridge not found. Did you colcon build? Please source ROS 2 workspace.")

        if self.show_image_in is RecordRgbShowImageIn.RERUN:
            self._start_rerun()

        left_pipeline_controller = self.copy_for(RGBCameras.left(), is_open_camera=False)
        right_pipeline_controller = self.copy_for(RGBCameras.right(), is_open_camera=False)
        
        use_center = self.camera_type == RGBCameras.head_left_right_center
        center_pipeline_controller = None
        if use_center:
            center_pipeline_controller = self.copy_for(RGBCameras.center(), is_open_camera=False)
            center_pipeline_controller.is_crop = False 

        if self.recording_directory is not None:
            left_pipeline_controller.save_thread.start()
            right_pipeline_controller.save_thread.start()
            if center_pipeline_controller is not None:
                center_pipeline_controller.save_thread.start()
        
        left_generator = None
        right_generator = None
        center_generator = None

        left_generator = stream_camera_left(stream_manager=self.stream_manager)
        right_generator = stream_camera_right(stream_manager=self.stream_manager)
        if use_center:
            center_generator = stream_camera_center(stream_manager=self.stream_manager)

        self.generator = self.stream_manager.stream()

        try:
            for _ in self.generator:
                if self.stop_event.is_set():
                    break

                left_ros_frame = self.stream_manager.get(left_generator)
                right_ros_frame = self.stream_manager.get(right_generator)
                center_ros_frame = self.stream_manager.get(center_generator)

                if left_ros_frame is None or right_ros_frame is None:
                    continue

                frame = SyncedImageFrame(
                    timestamp=left_ros_frame.timestamp, 
                    left=ImageFrame(timestamp=left_ros_frame.timestamp, frame_number=self.frame_number, image=left_ros_frame.image),
                    right=ImageFrame(timestamp=right_ros_frame.timestamp, frame_number=self.frame_number, image=right_ros_frame.image),
                )

                if self.camera_type == RGBCameras.head_left_right_center:
                    if center_ros_frame is not None:
                        frame.center = ImageFrame(timestamp=center_ros_frame.timestamp, frame_number=self.frame_number, image=center_ros_frame.image)
                    else:
                        continue 

                if is_run_pipeline:
                    left_pipeline_controller.run_pipeline(frame.left)
                    right_pipeline_controller.run_pipeline(frame.right)
                    if frame.center is not None and center_pipeline_controller is not None:
                        center_pipeline_controller.run_pipeline(frame.center)

                left_pipeline_controller.show_image(frame.left.image)
                right_pipeline_controller.show_image(frame.right.image)

                if frame.center is not None and center_pipeline_controller is not None:
                    center_pipeline_controller.show_image(frame.center.image)

                yield frame

                self.frame_number += 1
                left_pipeline_controller.frame_number += 1
                right_pipeline_controller.frame_number += 1
                if center_pipeline_controller is not None:
                    center_pipeline_controller.frame_number += 1

        finally:
            self.stop()
            left_pipeline_controller.stop()
            right_pipeline_controller.stop()
            if center_pipeline_controller is not None:
                center_pipeline_controller.stop()
            if left_pipeline_controller.save_thread.is_alive():
                left_pipeline_controller.save_thread.join(timeout=5)
            if right_pipeline_controller.save_thread.is_alive():
                right_pipeline_controller.save_thread.join(timeout=5)
            if center_pipeline_controller is not None and center_pipeline_controller.save_thread.is_alive():
                center_pipeline_controller.save_thread.join(timeout=5)

    def stop(self):
        super().stop()
        if self.stream_manager is not None:
            self.stream_manager.close()
        if self.generator is not None:
            try:
                self.generator.close()
            except Exception:
                pass

    def open_camera(self): pass
    def focus_roi(self, roi: list[int] ): raise NotImplementedError
    def set_manual_exposure(self, exposure_time: int, iso: int): raise NotImplementedError
    def set_manual_exposure_left_right(self, exposure_time: int, iso: int): raise NotImplementedError
    def set_manual_exposure_center(self, exposure_time: int, iso: int): raise NotImplementedError
    def set_auto_exposure(self, limit_max: int | None = None): raise NotImplementedError
    def set_manual_white_balance(self, color_temperature: int): raise NotImplementedError
    def set_auto_white_balance(self): raise NotImplementedError
    def set_brightness(self, value: int): raise NotImplementedError
    def set_contrast(self, value: int): raise NotImplementedError
    def set_saturation(self, value: int): raise NotImplementedError
    def set_sharpness(self, value: int): raise NotImplementedError