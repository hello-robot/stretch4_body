import threading
from dataclasses import dataclass
import numpy as np
import os
import yaml
from typing import Any
from collections.abc import Generator


from stretch4_body.subsystem.cameras.models.camera_calibration import RGBCameraCalibration
from stretch4_body.subsystem.cameras.detectors.detector_ai_models import AIModelWrapper
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras import (
    stream_left_camera,
    stream_right_camera,
    stream_center_camera,
    stream_left_right_camera,
    stream_left_right_center_camera,
)
from stretch4_body.subsystem.cameras.models.image_frame import (
    ImageFrame,
)
from stretch4_body.subsystem.cameras.calibrate_extrinsics_lidars import (
    DualLidarCalibration,
)
from stretch4_body.subsystem.cameras.cv_utils import project_points
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import RGBPipelineControllerROS



@dataclass
class RGBDFrame:
    timestamp: float
    image_frame: ImageFrame
    camera_type: RGBCameras
    pointcloud: np.ndarray
    """Point Cloud in the camera frame"""
    pointcloud_base: np.ndarray
    """Point Cloud in the base frame"""
    pointcloud_colors: np.ndarray
    """An array of colors corresponding to the point cloud points"""
    depth_image: np.ndarray

@dataclass
class SyncedRGBDFrame:
    timestamp: float
    left: RGBDFrame|None = None
    right: RGBDFrame|None = None
    center: RGBDFrame|None = None


class EmulatedRGBDStreamer:
    _instance = None

    def __init__(self, use_left_lidar:bool=True, use_right_lidar:bool=True, use_ros_for_lidars:bool=False):
        self.fleet_path = os.environ.get("HELLO_FLEET_PATH", "")
        self.fleet_id = os.environ.get("HELLO_FLEET_ID", "")

        if not self.fleet_path or not self.fleet_id:
            raise RuntimeError(
                "HELLO_FLEET_PATH or HELLO_FLEET_ID environment variables are missing."
            )

        from stretch4_body.subsystem.cameras.calibrate_extrinsics_cameras import CAMERA_EXTRINSICS_YAML_PATH
        
        self.lidar_calib = DualLidarCalibration()

        self.camera_extrinsics = {}
        if os.path.exists(CAMERA_EXTRINSICS_YAML_PATH):
            with open(CAMERA_EXTRINSICS_YAML_PATH, "r") as f:
                self.camera_extrinsics = yaml.safe_load(f) or {}

        self.T_left_to_center = np.array(
            self.camera_extrinsics.get("left_to_center", np.eye(4))
        )
        self.T_right_to_center = np.array(
            self.camera_extrinsics.get("right_to_center", np.eye(4))
        )

        self.stop_event = threading.Event()

        self.lidars = {}
        if use_ros_for_lidars:
            try:
                from stretch_python_bridge import stream_lidar_left, stream_lidar_right, StreamManager # This is a ros2 package, requires a colcon build
            except ImportError:
                raise ImportError("stretch_python_bridge not found. Did you colcon build? Please source ROS 2 workspace.")

            stream_manager = StreamManager()
            if use_left_lidar:
                self.lidars["left"] = stream_lidar_left(stream_manager=stream_manager)
            if use_right_lidar:
                self.lidars["right"] = stream_lidar_right(stream_manager=stream_manager)
            
            def _stream_lidars():
                for _ in stream_manager.stream():
                    if self.stop_event.is_set():
                        break

            threading.Thread(target=_stream_lidars, daemon=True).start()
        else:
            try:
                from pyhesai_wrapper import stream_lidar_left, stream_lidar_right
            except ImportError:
                raise ImportError("pyhesai_wrapper not found. Please install it or use the `--use_ros_for_lidars` flag.")

            if use_left_lidar:
                self.lidars["left"] = stream_lidar_left()

            if use_right_lidar:
                self.lidars["right"] = stream_lidar_right()

        self.latest_lidar_pts: dict[str, np.ndarray] = {}
        self.latest_pts_base = np.zeros((0, 3))
        self.latest_lidar_timestamps = {}

        self.T_base_to_center = np.eye(4)
        key = "transform_right_lidar_to_head_center"
        if key in self.camera_extrinsics:
            T_l_to_c = np.array(self.camera_extrinsics[key]["data"])
            T_base_to_l = self.lidar_calib.get_lidar_to_base_transform(is_right_lidar=True)
            self.T_base_to_center = T_l_to_c @ np.linalg.inv(T_base_to_l)

        self.T_base_to_cam = {
            RGBCameras.left(): np.linalg.inv(self.T_left_to_center) @ self.T_base_to_center,
            RGBCameras.right(): np.linalg.inv(self.T_right_to_center) @ self.T_base_to_center,
            RGBCameras.center(): np.linalg.inv(np.eye(4)) @ self.T_base_to_center,
        }

        # Preload calibrations
        self.calibs: dict[RGBCameras, Any] = {}
        self.calibs[RGBCameras.left()] = RGBCameras.left().load_calibration()
        self.calibs[RGBCameras.right()] = RGBCameras.right().load_calibration()
        self.calibs[RGBCameras.center()] = RGBCameras.center().load_calibration()

    @classmethod
    def get_instance(cls, use_left_lidar:bool=True, use_right_lidar:bool=True, use_ros_for_lidars:bool=False):
        if cls._instance is None:
            cls._instance = cls(use_left_lidar, use_right_lidar, use_ros_for_lidars)
        return cls._instance

    def process_lidars(self):
        did_update = False

        for l_name, l_sensor in self.lidars.items():
            lidar_frame = next(l_sensor)
            if lidar_frame is not None:
                self.latest_lidar_pts[l_name] = lidar_frame.points
                self.latest_lidar_timestamps[l_name] = lidar_frame.timestamp_system
                did_update = True
                
        if did_update or len(self.latest_pts_base) == 0:
            left_pts=self.latest_lidar_pts.get("left")
            right_pts=self.latest_lidar_pts.get("right")

            if left_pts is None or right_pts is None:
                return
            
            self.latest_pts_base = self.lidar_calib.unify_clouds(
                left_pts=left_pts,
                right_pts=right_pts,
            )

    @staticmethod
    def create_rgbd_frame(camera_type:RGBCameras, frame:ImageFrame, pts_base:np.ndarray, T_base_to_cam:dict[RGBCameras, np.ndarray], calib:RGBCameraCalibration) -> RGBDFrame:

        T_base_to_this_cam = T_base_to_cam[camera_type]

        # Transform to camera frame once
        ones = np.ones((len(pts_base), 1))
        pts_cam_all = (T_base_to_this_cam @ np.hstack([pts_base, ones]).T).T[:, :3]

        # Filter points behind camera
        valid_idx = pts_cam_all[:, 2] > 0
        pts_cam_valid = pts_cam_all[valid_idx]
        pts_base_valid = pts_base[valid_idx]

        depth_img = np.zeros(frame.image_raw.shape[:2], dtype=np.float32)

        if len(pts_cam_valid) > 0:
            rvec = np.zeros(3)
            tvec = np.zeros(3)
            img_pts = project_points(
                pts_cam_valid, rvec, tvec, calib.camera_matrix, calib.distortion_coefficients, calib.distortion_model
            ).reshape(-1, 2)

            h, w = frame.image_raw.shape[:2]

            img_pts_int = np.round(img_pts).astype(int)
            u = img_pts_int[:, 0]
            v = img_pts_int[:, 1]

            valid_uv = (u >= 0) & (u < w) & (v >= 0) & (v < h)

            u_valid = u[valid_uv]
            v_valid = v[valid_uv]

            colors_bgr = frame.image_raw[v_valid, u_valid]
            cols = colors_bgr[:, ::-1]  # BGR to RGB

            pts_cam = pts_cam_valid[valid_uv]
            pts_world = pts_base_valid[valid_uv]

            if len(v_valid) > 0:
                z_vals = pts_cam_valid[valid_uv, 2]
                # buffer = np.full((h, w), np.inf, dtype=np.float32)
                # np.minimum.at(buffer, (v_valid, u_valid), z_vals)
                # depth_img[buffer != np.inf] = buffer[buffer != np.inf]
                
                sort_idx = np.argsort(z_vals)[::-1]
                v_sorted = v_valid[sort_idx]
                u_sorted = u_valid[sort_idx]
                z_sorted = z_vals[sort_idx]
                depth_img[v_sorted, u_sorted] = z_sorted
        else:
            pts_cam = np.zeros((0, 3))
            pts_world = np.zeros((0, 3))
            cols = np.zeros((0, 3))

        return RGBDFrame(
            timestamp=frame.timestamp,
            image_frame=frame,
            camera_type=camera_type,
            pointcloud=pts_cam,
            pointcloud_base=pts_world,
            pointcloud_colors=cols,
            depth_image=depth_img,
        )
        

    def process_camera_rgbd(
        self, frame: ImageFrame, camera_type: RGBCameras
    ) -> RGBDFrame|None:
        calib = self.calibs[camera_type]

        if len(self.latest_pts_base) == 0:
            print(f"[{camera_type.name}] Dropping frame: no unified lidar cloud.")
            return None

        for lidar_name, lidar_timestamp in self.latest_lidar_timestamps.items():
            diff = abs(frame.timestamp_system - lidar_timestamp)
            if diff > 2.0 / 10.0:
                print(f"Camera {camera_type.name} timestamp {frame.timestamp} is too far from {lidar_name=} timestamp {lidar_timestamp}, {diff=}")
                return None

        pts_base = self.latest_pts_base
        return self.create_rgbd_frame(camera_type, frame, pts_base, self.T_base_to_cam, calib)

    def stop(self):
        for l_sensor in self.lidars.values():
            if hasattr(l_sensor, "stop"):
                l_sensor.stop()
        self.stop_event.set()

    

class EmulatedRGBDStreamerROS(EmulatedRGBDStreamer):
    """
    A specialized streamer that leverages RGBPipelineControllerROS's internal 
    StreamManager for concurrent lidar and camera streams.
    """
    def __init__(self, camera_type: RGBCameras, is_rotate: bool, is_rectify: bool, is_crop: bool, ai_models_to_use: list[AIModelWrapper]|None, detect_aruco_marker_size: float|None, use_left_lidar:bool=True, use_right_lidar:bool=True):
        self.fleet_path = os.environ.get("HELLO_FLEET_PATH", "")
        self.fleet_id = os.environ.get("HELLO_FLEET_ID", "")

        if not self.fleet_path or not self.fleet_id:
            raise RuntimeError(
                "HELLO_FLEET_PATH or HELLO_FLEET_ID environment variables are missing."
            )


        from stretch4_body.subsystem.cameras.calibrate_extrinsics_cameras import CAMERA_EXTRINSICS_YAML_PATH
        self.stop_event = threading.Event()

        self.lidar_calib = DualLidarCalibration()

        self.camera_extrinsics = {}
        if os.path.exists(CAMERA_EXTRINSICS_YAML_PATH):
            with open(CAMERA_EXTRINSICS_YAML_PATH, "r") as f:
                self.camera_extrinsics = yaml.safe_load(f) or {}

        self.T_left_to_center = np.array(
            self.camera_extrinsics.get("left_to_center", np.eye(4))
        )
        self.T_right_to_center = np.array(
            self.camera_extrinsics.get("right_to_center", np.eye(4))
        )
        
        self.T_base_to_center = np.eye(4)
        key = "transform_right_lidar_to_head_center"
        if key in self.camera_extrinsics:
            T_l_to_c = np.array(self.camera_extrinsics[key]["data"])
            T_base_to_l = self.lidar_calib.get_lidar_to_base_transform(is_right_lidar=True)
            self.T_base_to_center = T_l_to_c @ np.linalg.inv(T_base_to_l)

        self.T_base_to_cam = {
            RGBCameras.left(): np.linalg.inv(self.T_left_to_center) @ self.T_base_to_center,
            RGBCameras.right(): np.linalg.inv(self.T_right_to_center) @ self.T_base_to_center,
            RGBCameras.center(): np.linalg.inv(np.eye(4)) @ self.T_base_to_center,
        }

        self.calibs: dict[RGBCameras, Any] = {}
        try:
            self.calibs[RGBCameras.left()] = RGBCameras.left().load_calibration()
        except: pass
        try:
            self.calibs[RGBCameras.right()] = RGBCameras.right().load_calibration()
        except: pass
        try:
            self.calibs[RGBCameras.center()] = RGBCameras.center().load_calibration()
        except: pass

        self.latest_lidar_pts: dict[str, np.ndarray] = {}
        self.latest_pts_base = np.zeros((0, 3))
        self.latest_lidar_timestamps:dict[str, float] = {}
        
        self.camera_type = camera_type
        self.pipeline = RGBPipelineControllerROS(
            camera_type=camera_type,
            recording_directory=None,
            show_image_in=None,
            is_rotate=is_rotate,
            is_rectify=is_rectify,
            is_crop=is_crop,
            ai_models_to_use=ai_models_to_use or [],
            detect_aruco_marker_size=detect_aruco_marker_size
        )
        
        self.use_left_lidar = use_left_lidar
        self.use_right_lidar = use_right_lidar
        
        try:
            from stretch_python_bridge import stream_lidar_left, stream_lidar_right
        except ImportError:
            raise ImportError("stretch_python_bridge not found. Did you colcon build? Please source ROS 2 workspace.")
        
        self.lidars = {}
        if self.use_left_lidar:
             self.lidars["left"] = stream_lidar_left(stream_manager=self.pipeline.stream_manager)
        if self.use_right_lidar:
             self.lidars["right"] = stream_lidar_right(stream_manager=self.pipeline.stream_manager)


    def process_lidars(self):
        left_pts_frame = self.pipeline.stream_manager.get(self.lidars.get("left"), block=False) if self.use_left_lidar else None
        right_pts_frame = self.pipeline.stream_manager.get(self.lidars.get("right"), block=False) if self.use_right_lidar else None
        
        did_update = False
        if left_pts_frame is not None:
            self.latest_lidar_pts["left"] = left_pts_frame.points
            self.latest_lidar_timestamps["left"] = left_pts_frame.timestamp_system
            did_update = True
        if right_pts_frame is not None:
            self.latest_lidar_pts["right"] = right_pts_frame.points
            self.latest_lidar_timestamps["right"] = right_pts_frame.timestamp_system
            did_update = True
            
        if did_update or len(self.latest_pts_base) == 0:

            left_pts=self.latest_lidar_pts.get("left")
            right_pts=self.latest_lidar_pts.get("right")
            if left_pts is None or right_pts is None:
                return
            
            self.latest_pts_base = self.lidar_calib.unify_clouds(
                left_pts=left_pts,
                right_pts=right_pts
            )

    def get_rgbd_frame(self) -> Generator[RGBDFrame, None, None]:
        for frame in self.pipeline.get_frame(is_run_pipeline=True):
            if frame is None:
                continue
                
            self.process_lidars()
            rgbd_frame = self.process_camera_rgbd(frame, self.camera_type)
            if rgbd_frame is None:
                continue
            
            yield rgbd_frame

    def get_rgbd_frame_synced(self) -> Generator[SyncedRGBDFrame, None, None]:
        for synced_frame in self.pipeline.get_frame_synced(is_run_pipeline=True):
            if synced_frame is None:
                continue
                
            self.process_lidars()
            
            ret = SyncedRGBDFrame(timestamp=synced_frame.timestamp)
            if synced_frame.left:
                ret.left = self.process_camera_rgbd(synced_frame.left, RGBCameras.left())
            if synced_frame.right:
                ret.right = self.process_camera_rgbd(synced_frame.right, RGBCameras.right())
            if synced_frame.center:
                ret.center = self.process_camera_rgbd(synced_frame.center, RGBCameras.center())
            
            if ret.left is None and ret.right is None and ret.center is None:
                continue
            
            yield ret

    def stop(self):
        super().stop()
        self.stop_event.set()
        self.pipeline.stop()


def stream_left_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, ai_models_to_use: list[AIModelWrapper]|None=None , detect_aruco_marker_size: float|None = None, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
    try:
        if use_ros_for_cameras and use_ros_for_lidars:
            streamer = EmulatedRGBDStreamerROS(camera_type=RGBCameras.head_left, is_rotate=is_rotate, is_rectify=False, is_crop=False, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar)
            yield from streamer.get_rgbd_frame()
            return
        
        streamer = EmulatedRGBDStreamer.get_instance(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_lidars=use_ros_for_lidars)
        for image_frame in stream_left_camera(is_rotate=is_rotate, ai_models_to_use=ai_models_to_use , detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras):
            if image_frame is None:
                continue
            streamer.process_lidars()
            rgbd_frame = streamer.process_camera_rgbd(image_frame, RGBCameras.left()
            )
            if rgbd_frame is None:
                continue
            yield rgbd_frame
    finally:
        streamer.stop() 


def stream_right_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, ai_models_to_use: list[AIModelWrapper]|None=None , detect_aruco_marker_size: float|None = None, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
    try:
        if use_ros_for_cameras and use_ros_for_lidars:
            streamer = EmulatedRGBDStreamerROS(camera_type=RGBCameras.head_right, is_rotate=is_rotate, is_rectify=False, is_crop=False, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar)
            yield from streamer.get_rgbd_frame()
            return
        
        streamer = EmulatedRGBDStreamer.get_instance(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_lidars=use_ros_for_lidars)
        for image_frame in stream_right_camera(is_rotate=is_rotate, ai_models_to_use=ai_models_to_use , detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras):
            if image_frame is None:
                continue
            streamer.process_lidars()
            rgbd_frame = streamer.process_camera_rgbd(image_frame,  RGBCameras.right()
            )
            if rgbd_frame is None:
                continue
            yield rgbd_frame
    finally:
        streamer.stop()


def stream_center_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, ai_models_to_use: list[AIModelWrapper]|None=None , detect_aruco_marker_size: float|None = None, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
    try:
        if use_ros_for_cameras and use_ros_for_lidars:
            streamer = EmulatedRGBDStreamerROS(camera_type=RGBCameras.head_center, is_rotate=is_rotate, is_rectify=False, is_crop=False, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar)
            yield from streamer.get_rgbd_frame()
            return
        
        streamer = EmulatedRGBDStreamer.get_instance(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_lidars=use_ros_for_lidars)
        for image_frame in stream_center_camera(is_rotate=is_rotate, ai_models_to_use=ai_models_to_use , detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras):
            if image_frame is None:
                continue
            streamer.process_lidars()
            rgbd_frame = streamer.process_camera_rgbd(image_frame, RGBCameras.center())
            if rgbd_frame is None:
                continue
            yield rgbd_frame
    finally:
        streamer.stop()


def stream_left_right_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, ai_models_to_use: list[AIModelWrapper]|None=None , detect_aruco_marker_size: float|None = None, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[SyncedRGBDFrame, None, None]:
    try:
        if use_ros_for_cameras and use_ros_for_lidars:
            streamer = EmulatedRGBDStreamerROS(camera_type=RGBCameras.head_left_right, is_rotate=is_rotate, is_rectify=False, is_crop=False, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar)
            yield from streamer.get_rgbd_frame_synced()
            return
        
        streamer = EmulatedRGBDStreamer.get_instance(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_lidars=use_ros_for_lidars)
        for synced_frame in stream_left_right_camera(is_rotate=is_rotate, ai_models_to_use=ai_models_to_use , detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras):
            if synced_frame is None:
                continue
            streamer.process_lidars()
            ret = SyncedRGBDFrame(timestamp=synced_frame.timestamp)
            if synced_frame.left:
                ret.left = streamer.process_camera_rgbd(synced_frame.left, RGBCameras.left())
            if synced_frame.right:
                ret.right = streamer.process_camera_rgbd(synced_frame.right, RGBCameras.right())
            if ret.left is None and ret.right is None:
                continue
            yield ret
    finally:
        streamer.stop()


def stream_left_right_center_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, ai_models_to_use: list[AIModelWrapper]|None=None , detect_aruco_marker_size: float|None = None, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[SyncedRGBDFrame, None, None]:
    try:  
        if use_ros_for_cameras and use_ros_for_lidars: 
                streamer = EmulatedRGBDStreamerROS(camera_type=RGBCameras.head_left_right_center, is_rotate=is_rotate, is_rectify=False, is_crop=False, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar)
                yield from streamer.get_rgbd_frame_synced()
                return
            
        streamer = EmulatedRGBDStreamer.get_instance(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_lidars=use_ros_for_lidars)
        for synced_frame in stream_left_right_center_camera(is_rotate=is_rotate, ai_models_to_use=ai_models_to_use , detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras):
            if synced_frame is None:
                continue
            streamer.process_lidars()
            ret = SyncedRGBDFrame(timestamp=synced_frame.timestamp)
            if synced_frame.left:
                ret.left = streamer.process_camera_rgbd(synced_frame.left, RGBCameras.left())
            if synced_frame.right:
                ret.right = streamer.process_camera_rgbd(synced_frame.right, RGBCameras.right())
            if synced_frame.center:
                ret.center = streamer.process_camera_rgbd(synced_frame.center, RGBCameras.center())
            if ret.left is None and ret.right is None and ret.center is None:
                continue
            yield ret
    finally:
        streamer.stop()
