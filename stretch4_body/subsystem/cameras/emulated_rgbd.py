import logging

logger = logging.getLogger(__name__)
from dataclasses import dataclass
import numpy as np
from collections.abc import Generator
import time
import cv2

from stretch4_body.subsystem.cameras.models.camera_calibration import RGBCameraCalibration
from stretch4_body.subsystem.cameras.detectors.detector_ai_models import AIModelWrapper
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras import (
    stream_gripper_camera,
)
from stretch4_body.subsystem.cameras.models.image_frame import (
    ImageFrame,
)
from stretch4_urdf import get_urdf_from_robot_params, get_transform
from stretch4_body.subsystem.cameras.cv_utils import project_points


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

def get_lidar_stream(use_left_lidar:bool=True, use_right_lidar:bool=True, use_ros_for_lidars:bool=False):

    if use_ros_for_lidars:
        try:
            from stretch_python_bridge import stream_lidar_points_left as stream_lidar_left, stream_lidar_points_right as stream_lidar_right, StreamManager
        except ImportError:
            raise ImportError("stretch_python_bridge not found. Did you colcon build? Please source ROS 2 workspace.")

        stream_manager = StreamManager()
        if use_left_lidar and use_right_lidar:
            def _stream_lidar_left_right():
                yield from zip(stream_lidar_left(stream_manager=stream_manager), stream_lidar_right(stream_manager=stream_manager))
            return _stream_lidar_left_right()
        elif use_left_lidar:
            return stream_lidar_left(stream_manager=stream_manager)
        elif use_right_lidar:
            return stream_lidar_right(stream_manager=stream_manager)
        
        raise ValueError("Must specify use_right and/or use_left.")
        
    from pyhesai_wrapper import stream_lidar_left, stream_lidar_right, stream_lidar_left_right

    if use_left_lidar and use_right_lidar:
        return stream_lidar_left_right()
    elif use_left_lidar:
        return stream_lidar_left()
    elif use_right_lidar:
        return stream_lidar_right()
    raise ValueError("Must specify use_right and/or use_left.")


def transform_and_optionally_unify_clouds(left_pts: np.ndarray|None, right_pts: np.ndarray|None, T_left_lidar_to_base: np.ndarray, T_right_lidar_to_base: np.ndarray) -> np.ndarray:
    merged = []
    
    if left_pts is not None and len(left_pts) > 0:
        # Extract 3x3 Rotation and 3-element Translation
        R_left = T_left_lidar_to_base[:3, :3]
        t_left = T_left_lidar_to_base[:3, 3]
        
        # Apply transformation: P * R^T + t
        left_base = left_pts[:, :3] @ R_left.T + t_left
        merged.append(left_base)

    if right_pts is not None and len(right_pts) > 0:
        R_right = T_right_lidar_to_base[:3, :3]
        t_right = T_right_lidar_to_base[:3, 3]
        
        right_base = right_pts[:, :3] @ R_right.T + t_right
        merged.append(right_base)

    if not merged:
        return np.zeros((0, 3))
        
    return np.vstack(merged)


def apply_shadow_filter(sparse_depth_image: np.ndarray, window_size: int=5, depth_threshold: float=0.3):
    # From stretch4_rgbd
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

def create_rgbd_frame(camera_type:RGBCameras, frame:ImageFrame, pts_base:np.ndarray, T_base_to_cam:np.ndarray, calib:RGBCameraCalibration) -> RGBDFrame:
    pts_cam_all = pts_base @ T_base_to_cam[:3, :3].T + T_base_to_cam[:3, 3]

    valid_idx = pts_cam_all[:, 2] > 0
    pts_cam_valid = pts_cam_all[valid_idx]
    pts_base_valid = pts_base[valid_idx]

    depth_img = np.zeros(frame.image_raw.shape[:2], dtype=np.float32)

    pts_cam = np.zeros((0, 3))
    pts_world = np.zeros((0, 3))
    cols = np.zeros((0, 3))

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
            depth_img, shadowed = apply_shadow_filter(depth_img, window_size=5, depth_threshold=0.3)
            
            valid_mask = depth_img > 0
            surviving_indices = index_img[valid_mask]
            
            pts_cam = pts_cam_valid[valid_uv][surviving_indices]
            pts_world = pts_base_valid[valid_uv][surviving_indices]
            
            v_filtered, u_filtered = np.where(valid_mask)
            colors_bgr = frame.image_raw[v_filtered, u_filtered]
            cols = colors_bgr[:, ::-1]  # BGR to RGB

    return RGBDFrame(
        timestamp=frame.timestamp,
        image_frame=frame,
        camera_type=camera_type,
        pointcloud=pts_cam,
        pointcloud_base=pts_world,
        pointcloud_colors=cols,
        depth_image=depth_img,
    )

def stream_rgbd(camera_type: RGBCameras, use_left_lidar:bool=True, use_right_lidar:bool=True, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False, is_rotate:bool=True,) -> Generator[RGBDFrame, None, None]:
    if camera_type.is_synced_camera_type():
        raise RuntimeError(f"{camera_type} is a synced camera, use stream_rgbd_synced()")

    def _lidar_camera_stream():
        lidar_stream = get_lidar_stream(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_lidars=use_ros_for_lidars)

        camera_stream = camera_type.start_camera_stream(use_ros_for_cameras=use_ros_for_cameras, is_rotate=is_rotate)

        yield from zip(lidar_stream, camera_stream)


    camera_calibration = camera_type.load_calibration()


    urdf_contents = get_urdf_from_robot_params(apply_calibration=True)

    T_left_lidar_to_base = get_transform(urdf_contents, "lidar_left_link", "base_footprint")
    T_right_lidar_to_base = get_transform(urdf_contents, "lidar_right_link", "base_footprint")
    
    camera_name = "center"
    if camera_type.is_right():
        camera_name = "right"
    elif camera_type.is_left():
        camera_name = "left"
    T_cam_to_base = get_transform(urdf_contents, f"camera_{camera_name}_optical_link", "base_footprint")
    T_base_to_cam = np.linalg.inv(T_cam_to_base)

    for lidar_frame, camera_frame in _lidar_camera_stream():
        left_points = None
        right_points = None
        if use_left_lidar and use_right_lidar:
            left_points = lidar_frame[0].points if lidar_frame[0] is not None else None
            right_points = lidar_frame[1].points if lidar_frame[1] is not None else None
        elif use_left_lidar:
            left_points = lidar_frame.points
        elif use_right_lidar:
            right_points = lidar_frame.points

        if left_points is None and right_points is None:
            continue

        pts_base = transform_and_optionally_unify_clouds(
            left_points,
            right_points,
            T_left_lidar_to_base=T_left_lidar_to_base,
            T_right_lidar_to_base=T_right_lidar_to_base
        )

        yield create_rgbd_frame(
            camera_type=camera_type,
            frame=camera_frame,
            pts_base=pts_base,
            T_base_to_cam=T_base_to_cam,
            calib=camera_calibration
        )


def stream_rgbd_synced(camera_type: RGBCameras, use_left_lidar:bool=True, use_right_lidar:bool=True, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False, is_rotate:bool=True) -> Generator[SyncedRGBDFrame, None, None]:
    
    if not camera_type.is_synced_camera_type():
        raise RuntimeError(f"{camera_type} is not a synced camera, use stream_rgbd()")
    
    def _lidar_camera_stream():
        lidar_stream = get_lidar_stream(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_lidars=use_ros_for_lidars)

        camera_stream = camera_type.start_camera_stream(use_ros_for_cameras=use_ros_for_cameras, is_rotate=is_rotate)

        yield from zip(lidar_stream, camera_stream)


    camera_calibration_left = RGBCameras.left().load_calibration()
    camera_calibration_center = RGBCameras.center().load_calibration()
    camera_calibration_right = RGBCameras.right().load_calibration()


    urdf_contents = get_urdf_from_robot_params(apply_calibration=True)

    T_left_lidar_to_base = get_transform(urdf_contents, "lidar_left_link", "base_footprint")
    T_right_lidar_to_base = get_transform(urdf_contents, "lidar_right_link", "base_footprint")
    
    T_base_to_cam_left = np.linalg.inv(get_transform(urdf_contents, "camera_left_optical_link", "base_footprint"))
    T_base_to_cam_center = np.linalg.inv(get_transform(urdf_contents, "camera_center_optical_link", "base_footprint"))
    T_base_to_cam_right = np.linalg.inv(get_transform(urdf_contents, "camera_right_optical_link", "base_footprint"))

    from concurrent.futures import ThreadPoolExecutor
    executor = ThreadPoolExecutor(max_workers=4)

    for lidar_frame, camera_frame in _lidar_camera_stream():
        left_points = None
        right_points = None
        if use_left_lidar and use_right_lidar:
            left_points = lidar_frame[0].points if lidar_frame[0] is not None else None
            right_points = lidar_frame[1].points if lidar_frame[1] is not None else None
        elif use_left_lidar:
            left_points = lidar_frame.points
        elif use_right_lidar:
            right_points = lidar_frame.points

        if left_points is None and right_points is None:
            continue

        pts_base = transform_and_optionally_unify_clouds(
            left_points,
            right_points,
            T_left_lidar_to_base=T_left_lidar_to_base,
            T_right_lidar_to_base=T_right_lidar_to_base
        )


        if camera_frame.left is None:
            continue

        left_future = executor.submit(
            create_rgbd_frame,
            RGBCameras.left(),
            camera_frame.left,
            pts_base,
            T_base_to_cam_left,
            camera_calibration_left
        )
        right_future = executor.submit(
            create_rgbd_frame,
            RGBCameras.right(),
            camera_frame.right,
            pts_base,
            T_base_to_cam_right,
            camera_calibration_right
        )

        if camera_frame.center is not None:
            center_future = executor.submit(
                create_rgbd_frame,
                RGBCameras.center(),
                camera_frame.center,
                pts_base,
                T_base_to_cam_center,
                camera_calibration_center
            )

        synced_rgbd = SyncedRGBDFrame(timestamp=camera_frame.left.timestamp)

        synced_rgbd.left = left_future.result()
        if camera_frame.center is not None:
            synced_rgbd.center = center_future.result()
        synced_rgbd.right = right_future.result()
        
        yield synced_rgbd



def stream_left_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
    yield from stream_rgbd(RGBCameras.left(), 
        use_left_lidar=use_left_lidar,
        use_right_lidar=use_right_lidar,
        use_ros_for_lidars=use_ros_for_lidars,
        use_ros_for_cameras=use_ros_for_cameras,
        is_rotate=is_rotate,)


def stream_right_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
    yield from stream_rgbd(RGBCameras.right(),
    use_left_lidar=use_left_lidar,
    use_right_lidar=use_right_lidar,
    use_ros_for_lidars=use_ros_for_lidars,
    use_ros_for_cameras=use_ros_for_cameras,
    is_rotate=is_rotate,)


def stream_center_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
    yield from stream_rgbd(RGBCameras.center(),
    use_left_lidar=use_left_lidar,
    use_right_lidar=use_right_lidar,
    use_ros_for_lidars=use_ros_for_lidars,
    use_ros_for_cameras=use_ros_for_cameras,
    is_rotate=is_rotate,)


def stream_left_right_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[SyncedRGBDFrame, None, None]:
    yield from stream_rgbd_synced(RGBCameras.synced_left_right(),
    use_left_lidar=use_left_lidar,
    use_right_lidar=use_right_lidar,
    use_ros_for_lidars=use_ros_for_lidars,
    use_ros_for_cameras=use_ros_for_cameras,
    is_rotate=is_rotate,)


def stream_left_right_center_rgbd(*, is_rotate=True, use_left_lidar=True, use_right_lidar=True, use_ros_for_lidars:bool=False, use_ros_for_cameras:bool=False) -> Generator[SyncedRGBDFrame, None, None]:
    yield from stream_rgbd_synced(RGBCameras.synced_left_right_center(),
    use_left_lidar=use_left_lidar,
    use_right_lidar=use_right_lidar,
    use_ros_for_lidars=use_ros_for_lidars,
    use_ros_for_cameras=use_ros_for_cameras,
    is_rotate=is_rotate,)


def stream_gripper_rgbd(*, is_rotate=True, use_ros_for_cameras:bool=False) -> Generator[RGBDFrame, None, None]:
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

    for synced_frame in stream_gripper_camera(is_rotate=is_rotate, use_ros_for_cameras=use_ros_for_cameras, enable_pointcloud=False):
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
