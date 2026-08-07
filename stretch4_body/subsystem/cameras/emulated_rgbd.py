import logging

logger = logging.getLogger(__name__)
import threading
from dataclasses import dataclass
import numpy as np
import os
import yaml
from typing import Any
from collections.abc import Generator
import collections
import time
import cv2

from stretch4_body.subsystem.cameras.models.camera_calibration import RGBCameraCalibration
from stretch4_body.subsystem.cameras.detectors.detector_ai_models import AIModelWrapper
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras import (
    stream_left_camera,
    stream_right_camera,
    stream_center_camera,
    stream_left_right_camera,
    stream_left_right_center_camera,
    stream_gripper_camera,
)
from stretch4_body.subsystem.cameras.models.image_frame import (
    ImageFrame,
)
from stretch4_urdf import get_urdf_from_robot_params
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

    def __init__(self, use_left_lidar:bool=True, use_right_lidar:bool=True, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False, is_rotate:bool=True, ai_models_to_use:list[AIModelWrapper]|None=None, detect_aruco_marker_size:float|None=None):
        self.fleet_path = os.environ.get("HELLO_FLEET_PATH", "")
        self.fleet_id = os.environ.get("HELLO_FLEET_ID", "")

        if not self.fleet_path or not self.fleet_id:
            raise RuntimeError(
                "HELLO_FLEET_PATH or HELLO_FLEET_ID environment variables are missing."
            )

        from stretch4_body.subsystem.cameras.calibrate_extrinsics_cameras import CAMERA_EXTRINSICS_YAML_PATH
        
        urdf_contents = get_urdf_from_robot_params(apply_calibration=True)
        from yourdfpy import URDF
        import io
        self.urdf = URDF.load(io.StringIO(urdf_contents))

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

        self.use_ros_for_cameras = use_ros_for_cameras
        self.is_rotate = is_rotate
        self.ai_models_to_use = ai_models_to_use
        self.detect_aruco_marker_size = detect_aruco_marker_size

        self.lidars = {}
        if use_ros_for_lidars:
            try:
                from stretch_python_bridge import stream_lidar_points_left as stream_lidar_left, stream_lidar_points_right as stream_lidar_right, StreamManager
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

        if not self.lidars:
            raise RuntimeError("No LiDAR is connected or used. Emulated RGB-D requires at least one active LiDAR.")

        self.T_base_to_center = np.eye(4)
        key = "transform_right_lidar_to_head_center"
        if key in self.camera_extrinsics:
            T_l_to_c = np.array(self.camera_extrinsics[key]["data"])
            T_base_to_l = self.get_lidar_to_base_transform(is_right_lidar=True)
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

        # Generator structures
        self.camera_generators = {}

        from concurrent.futures import ThreadPoolExecutor
        self.executor = ThreadPoolExecutor(max_workers=4)

    def get_nominal_transform(self, joint_name: str) -> np.ndarray:
        for joint in self.urdf.robot.joints:
            if joint.name == joint_name:
                return np.eye(4) if joint.origin is None else joint.origin
        return np.eye(4)

    def get_lidar_to_base_transform(self, is_right_lidar: bool) -> np.ndarray:
        T_joint_head = self.get_nominal_transform("joint_head")
        if is_right_lidar:
            T_lidar_joint = self.get_nominal_transform("lidar_right_joint")
        else:
            T_lidar_joint = self.get_nominal_transform("lidar_left_joint")
        return T_joint_head @ T_lidar_joint

    def unify_clouds(self, left_pts: np.ndarray, right_pts: np.ndarray) -> np.ndarray:
        merged = []
        if left_pts is not None and len(left_pts) > 0:
            T_lidar_to_base = self.get_lidar_to_base_transform(is_right_lidar=False)
            ones = np.ones((len(left_pts), 1))
            left_base = (T_lidar_to_base @ np.hstack([left_pts[:, :3], ones]).T).T[:, :3]
            merged.append(left_base)

        if right_pts is not None and len(right_pts) > 0:
            T_lidar_to_base = self.get_lidar_to_base_transform(is_right_lidar=True)
            ones = np.ones((len(right_pts), 1))
            right_base = (T_lidar_to_base @ np.hstack([right_pts[:, :3], ones]).T).T[:, :3]
            merged.append(right_base)

        if not merged:
            return np.zeros((0, 3))
        return np.vstack(merged)

    @classmethod
    def get_instance(cls, use_left_lidar:bool=True, use_right_lidar:bool=True, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False, is_rotate:bool=True, ai_models_to_use:list[AIModelWrapper]|None=None, detect_aruco_marker_size:float|None=None):
        if cls._instance is None:
            cls._instance = cls(
                use_left_lidar=use_left_lidar,
                use_right_lidar=use_right_lidar,
                use_ros_for_lidars=use_ros_for_lidars,
                use_ros_for_cameras=use_ros_for_cameras,
                is_rotate=is_rotate,
                ai_models_to_use=ai_models_to_use,
                detect_aruco_marker_size=detect_aruco_marker_size
            )
        return cls._instance

    def start_camera_stream(self, camera_type: RGBCameras):
        if camera_type in self.camera_generators:
            return

        if camera_type == RGBCameras.synced_left_right():
            gen_fn = stream_left_right_camera
        elif camera_type == RGBCameras.synced_left_right_center():
            gen_fn = stream_left_right_center_camera
        elif camera_type == RGBCameras.left():
            gen_fn = stream_left_camera
        elif camera_type == RGBCameras.right():
            gen_fn = stream_right_camera
        elif camera_type == RGBCameras.center():
            gen_fn = stream_center_camera
        elif camera_type == RGBCameras.gripper_rgbd:
            gen_fn = stream_gripper_camera
        else:
            raise ValueError(f"Unknown camera type: {camera_type}")

        self.camera_generators[camera_type] = gen_fn(
            is_rotate=self.is_rotate,
            ai_models_to_use=self.ai_models_to_use,
            detect_aruco_marker_size=self.detect_aruco_marker_size,
            use_ros_for_cameras=self.use_ros_for_cameras
        )

    @staticmethod
    def apply_shadow_filter(sparse_depth_image: np.ndarray, window_size: int=5, depth_threshold: float=0.3):
        if window_size <= 1:
            return sparse_depth_image, np.zeros_like(sparse_depth_image, dtype=bool)
        depth_inf = sparse_depth_image.copy()
        depth_inf[depth_inf == 0] = np.inf
        kernel = np.ones((window_size, window_size), np.uint8)
        min_depth = cv2.erode(depth_inf, kernel)
        shadowed = (sparse_depth_image > 0) & (sparse_depth_image - min_depth > depth_threshold)
        filtered_depth = sparse_depth_image.copy()
        filtered_depth[shadowed] = 0.0
        return filtered_depth, shadowed

    @staticmethod
    def create_rgbd_frame(camera_type:RGBCameras, frame:ImageFrame, pts_base:np.ndarray, T_base_to_cam:dict[RGBCameras, np.ndarray], calib:RGBCameraCalibration) -> RGBDFrame:
        T_base_to_this_cam = T_base_to_cam[camera_type]

        # Highly optimized 3D projection mapping matching fast_emulated_rgbd.py
        pts_cam_all = pts_base @ T_base_to_this_cam[:3, :3].T + T_base_to_this_cam[:3, 3]

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

            if len(v_valid) > 0:
                z_vals = pts_cam_valid[valid_uv, 2]
                sort_idx = np.argsort(z_vals)[::-1]
                v_sorted = v_valid[sort_idx]
                u_sorted = u_valid[sort_idx]
                z_sorted = z_vals[sort_idx]
                
                orig_indices = np.arange(len(v_valid))[sort_idx]
                
                depth_img[v_sorted, u_sorted] = z_sorted
                
                index_img = np.full((h, w), -1, dtype=np.int32)
                index_img[v_sorted, u_sorted] = orig_indices
                
                # Apply high-speed sparsity shadow filter
                depth_img, shadowed = EmulatedRGBDStreamer.apply_shadow_filter(depth_img, window_size=5, depth_threshold=0.3)
                
                valid_mask = depth_img > 0
                surviving_indices = index_img[valid_mask]
                
                pts_cam = pts_cam_valid[valid_uv][surviving_indices]
                pts_world = pts_base_valid[valid_uv][surviving_indices]
                
                v_filtered, u_filtered = np.where(valid_mask)
                colors_bgr = frame.image_raw[v_filtered, u_filtered]
                cols = colors_bgr[:, ::-1]  # BGR to RGB
            else:
                pts_cam = np.zeros((0, 3))
                pts_world = np.zeros((0, 3))
                cols = np.zeros((0, 3))
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

    def _get_next_camera_frames(self) -> dict[RGBCameras, Any]:
        frames = {}
        for cam_type, gen in list(self.camera_generators.items()):
            try:
                frame_or_synced = next(gen)
            except StopIteration:
                continue
            if frame_or_synced is None:
                continue
            if cam_type == RGBCameras.synced_left_right():
                if getattr(frame_or_synced, "left", None) is not None:
                    frames[RGBCameras.left()] = frame_or_synced.left
                if getattr(frame_or_synced, "right", None) is not None:
                    frames[RGBCameras.right()] = frame_or_synced.right
            elif cam_type == RGBCameras.synced_left_right_center():
                if getattr(frame_or_synced, "left", None) is not None:
                    frames[RGBCameras.left()] = frame_or_synced.left
                if getattr(frame_or_synced, "right", None) is not None:
                    frames[RGBCameras.right()] = frame_or_synced.right
                if getattr(frame_or_synced, "center", None) is not None:
                    frames[RGBCameras.center()] = frame_or_synced.center
            else:
                frames[cam_type] = frame_or_synced
        return frames

    def stream_rgbd(self, camera_types: list[RGBCameras]) -> Generator[RGBDFrame, None, None]:
        master_lidar_name = "left" if "left" in self.lidars else ("right" if "right" in self.lidars else next(iter(self.lidars.keys())))

        while not self.stop_event.is_set():
            try:
                master_lidar_frame = next(self.lidars[master_lidar_name])
            except StopIteration:
                break

            if master_lidar_frame is None:
                continue

            mid_ts = getattr(master_lidar_frame, 'timestamp_system', time.monotonic())

            cam_frames = self._get_next_camera_frames()
            if not cam_frames:
                continue

            synced_lidar_frames = {master_lidar_name: master_lidar_frame}
            for l_name in self.lidars:
                if l_name != master_lidar_name:
                    try:
                        synced_lidar_frames[l_name] = next(self.lidars[l_name])
                    except StopIteration:
                        pass

            left_frame = synced_lidar_frames.get("left")
            right_frame = synced_lidar_frames.get("right")

            left_pts = left_frame.points if left_frame is not None else None
            right_pts = right_frame.points if right_frame is not None else None

            if left_pts is None and right_pts is None:
                continue

            pts_base = self.unify_clouds(
                left_pts=left_pts if left_pts is not None else np.zeros((0, 3)),
                right_pts=right_pts if right_pts is not None else np.zeros((0, 3)),
            )

            futures = {}
            for camera_type in camera_types:
                cam_frame = cam_frames.get(camera_type)
                if cam_frame is None:
                    continue

                calib = self.calibs[camera_type]
                futures[camera_type] = self.executor.submit(
                    self.create_rgbd_frame,
                    camera_type,
                    cam_frame,
                    pts_base,
                    self.T_base_to_cam,
                    calib
                )

            for camera_type, future in futures.items():
                try:
                    rgbd_frame = future.result()
                except Exception as e:
                    logger.error(f"Error creating RGB-D frame for {camera_type}: {e}")
                    continue

                if rgbd_frame is not None:
                    yield rgbd_frame

    def stream_rgbd_synced(self, camera_types: list[RGBCameras]) -> Generator[SyncedRGBDFrame, None, None]:
        master_lidar_name = "left" if "left" in self.lidars else ("right" if "right" in self.lidars else next(iter(self.lidars.keys())))

        while not self.stop_event.is_set():
            try:
                master_lidar_frame = next(self.lidars[master_lidar_name])
            except StopIteration:
                break

            if master_lidar_frame is None:
                continue

            mid_ts = getattr(master_lidar_frame, 'timestamp_system', time.monotonic())

            cam_frames = self._get_next_camera_frames()
            if not cam_frames:
                continue

            synced_lidar_frames = {master_lidar_name: master_lidar_frame}
            for l_name in self.lidars:
                if l_name != master_lidar_name:
                    try:
                        synced_lidar_frames[l_name] = next(self.lidars[l_name])
                    except StopIteration:
                        pass

            left_frame = synced_lidar_frames.get("left")
            right_frame = synced_lidar_frames.get("right")

            left_pts = left_frame.points if left_frame is not None else None
            right_pts = right_frame.points if right_frame is not None else None

            if left_pts is None and right_pts is None:
                continue

            pts_base = self.unify_clouds(
                left_pts=left_pts if left_pts is not None else np.zeros((0, 3)),
                right_pts=right_pts if right_pts is not None else np.zeros((0, 3)),
            )

            futures = {}
            for camera_type in camera_types:
                cam_frame = cam_frames.get(camera_type)
                if cam_frame is None:
                    continue

                calib = self.calibs[camera_type]
                futures[camera_type] = self.executor.submit(
                    self.create_rgbd_frame,
                    camera_type,
                    cam_frame,
                    pts_base,
                    self.T_base_to_cam,
                    calib
                )

            synced_rgbd = SyncedRGBDFrame(timestamp=mid_ts)
            has_any = False
            for camera_type, future in futures.items():
                try:
                    rgbd_frame = future.result()
                except Exception as e:
                    logger.error(f"Error creating RGB-D frame for {camera_type}: {e}")
                    continue

                if rgbd_frame is not None:
                    if camera_type == RGBCameras.left():
                        synced_rgbd.left = rgbd_frame
                        has_any = True
                    elif camera_type == RGBCameras.right():
                        synced_rgbd.right = rgbd_frame
                        has_any = True
                    elif camera_type == RGBCameras.center():
                        synced_rgbd.center = rgbd_frame
                        has_any = True

            if has_any:
                yield synced_rgbd

    def stop(self):
        self.stop_event.set()
        for l_sensor in self.lidars.values():
            if hasattr(l_sensor, "stop"):
                l_sensor.stop()
        if hasattr(self, "executor"):
            self.executor.shutdown(wait=False)
        if EmulatedRGBDStreamer._instance == self:
            EmulatedRGBDStreamer._instance = None


class EmulatedRGBDStreamerROS(EmulatedRGBDStreamer):
    def __init__(self, camera_type: RGBCameras, is_rotate: bool, is_rectify: bool, is_crop: bool, ai_models_to_use: list[AIModelWrapper]|None, detect_aruco_marker_size: float|None, use_left_lidar:bool=True, use_right_lidar:bool=True):
        super().__init__(
            use_left_lidar=use_left_lidar,
            use_right_lidar=use_right_lidar,
            use_ros_for_lidars=True,
            use_ros_for_cameras=True,
            is_rotate=is_rotate,
            ai_models_to_use=ai_models_to_use,
            detect_aruco_marker_size=detect_aruco_marker_size
        )


def stream_left_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None = None, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
    try:
        streamer = EmulatedRGBDStreamer.get_instance(
            use_left_lidar=use_left_lidar,
            use_right_lidar=use_right_lidar,
            use_ros_for_lidars=use_ros_for_lidars,
            use_ros_for_cameras=use_ros_for_cameras,
            is_rotate=is_rotate,
            ai_models_to_use=ai_models_to_use,
            detect_aruco_marker_size=detect_aruco_marker_size
        )
        streamer.start_camera_stream(RGBCameras.left())
        yield from streamer.stream_rgbd([RGBCameras.left()])
    finally:
        streamer.stop()


def stream_right_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None = None, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
    try:
        streamer = EmulatedRGBDStreamer.get_instance(
            use_left_lidar=use_left_lidar,
            use_right_lidar=use_right_lidar,
            use_ros_for_lidars=use_ros_for_lidars,
            use_ros_for_cameras=use_ros_for_cameras,
            is_rotate=is_rotate,
            ai_models_to_use=ai_models_to_use,
            detect_aruco_marker_size=detect_aruco_marker_size
        )
        streamer.start_camera_stream(RGBCameras.right())
        yield from streamer.stream_rgbd([RGBCameras.right()])
    finally:
        streamer.stop()


def stream_center_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None = None, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
    try:
        streamer = EmulatedRGBDStreamer.get_instance(
            use_left_lidar=use_left_lidar,
            use_right_lidar=use_right_lidar,
            use_ros_for_lidars=use_ros_for_lidars,
            use_ros_for_cameras=use_ros_for_cameras,
            is_rotate=is_rotate,
            ai_models_to_use=ai_models_to_use,
            detect_aruco_marker_size=detect_aruco_marker_size
        )
        streamer.start_camera_stream(RGBCameras.center())
        yield from streamer.stream_rgbd([RGBCameras.center()])
    finally:
        streamer.stop()


def stream_left_right_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None = None, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[SyncedRGBDFrame, None, None]:
    try:
        streamer = EmulatedRGBDStreamer.get_instance(
            use_left_lidar=use_left_lidar,
            use_right_lidar=use_right_lidar,
            use_ros_for_lidars=use_ros_for_lidars,
            use_ros_for_cameras=use_ros_for_cameras,
            is_rotate=is_rotate,
            ai_models_to_use=ai_models_to_use,
            detect_aruco_marker_size=detect_aruco_marker_size
        )
        streamer.start_camera_stream(RGBCameras.synced_left_right())
        yield from streamer.stream_rgbd_synced([RGBCameras.left(), RGBCameras.right()])
    finally:
        streamer.stop()


def stream_left_right_center_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None = None, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[SyncedRGBDFrame, None, None]:
    try:
        streamer = EmulatedRGBDStreamer.get_instance(
            use_left_lidar=use_left_lidar,
            use_right_lidar=use_right_lidar,
            use_ros_for_lidars=use_ros_for_lidars,
            use_ros_for_cameras=use_ros_for_cameras,
            is_rotate=is_rotate,
            ai_models_to_use=ai_models_to_use,
            detect_aruco_marker_size=detect_aruco_marker_size
        )
        streamer.start_camera_stream(RGBCameras.synced_left_right_center())
        yield from streamer.stream_rgbd_synced([RGBCameras.left(), RGBCameras.right(), RGBCameras.center()])
    finally:
        streamer.stop()


def stream_gripper_rgbd(*, is_rotate=True, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
    try:
        calib = RGBCameraCalibration.load_calibration_from_fleet_path(
            RGBCameras.gripper_right, is_flip_width_and_height=False
        )
        camera_matrix = calib.camera_matrix
    except Exception as e:
        import logging
        logging.warning(f"Could not load calibration for gripper_right: {e}. Using fallback default intrinsics.")
        camera_matrix = np.array([
            [251.0758, 0.0, 201.3269],
            [0.0, 250.9491, 319.2119],
            [0.0, 0.0, 1.0]
        ])

    for synced_frame in stream_gripper_camera(is_rotate=is_rotate, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, enable_pointcloud=False):
        if synced_frame is None:
            continue
        image_frame = synced_frame.right
        if image_frame is None:
            continue
        
        depth_image = synced_frame.depth if synced_frame.depth is not None else np.zeros((0, 0))
        if synced_frame.pointcloud is not None:
            pointcloud = synced_frame.pointcloud
            pointcloud_colors = synced_frame.pointcloud_color
        elif depth_image.size > 0:
            fx = camera_matrix[0, 0]
            fy = camera_matrix[1, 1]
            cx = camera_matrix[0, 2]
            cy = camera_matrix[1, 2]
            
            h, w = depth_image.shape
            v, u = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
            
            valid_mask = depth_image > 0
            z = depth_image[valid_mask].astype(np.float32) / 1000.0  # mm to meters
            
            u_valid = u[valid_mask]
            v_valid = v[valid_mask]
            
            x = (u_valid - cx) * z / fx
            y = (v_valid - cy) * z / fy
            
            pointcloud = np.stack((x, y, z), axis=-1)
            
            rgb_image = image_frame.image
            if rgb_image is not None and rgb_image.ndim == 3:
                pointcloud_colors = rgb_image[valid_mask][:, ::-1] # BGR to RGB
            else:
                pointcloud_colors = np.zeros((len(pointcloud), 3))
        else:
            pointcloud = np.zeros((0, 3))
            pointcloud_colors = np.zeros((0, 3))
        
        yield RGBDFrame(
            timestamp=synced_frame.timestamp,
            image_frame=image_frame,
            camera_type=RGBCameras.gripper_rgbd,
            pointcloud=pointcloud,
            pointcloud_base=np.zeros((0, 3)),
            pointcloud_colors=pointcloud_colors,
            depth_image=depth_image
        )
