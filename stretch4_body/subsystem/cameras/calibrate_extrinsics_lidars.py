import argparse
import time
import cv2
import rerun as rr
import numpy as np
import threading
import sys
import yaml
import os

from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.enums.charuco_dictionary import (
    CharucoBoards,
)
from stretch4_body.subsystem.cameras.cv_utils import solve_pnp, draw_frame_axes
from stretch4_body.subsystem.cameras.models.camera_calibration import (
    RGBCameraCalibration,
    DEFAULT_CALIBRATION_FOLDER_PATH,
)

from stretch4_body.subsystem.cameras.detectors.detector_frame_settled import (
    DetectFrameSettled,
)
from dataclasses import dataclass
import datetime
import yaml


from stretch4_body.core.gamepad_teleop import GamePadTeleop
from stretch4_body.core.gamepad_controller import ButtonPressCounter
from stretch4_body.subsystem.cameras.calibrate_intrinsics_robot_move import (
    MoveRobotMode,
)
from scipy.spatial.transform import Rotation
from stretch4_body.subsystem.cameras.cv_utils import project_points
from pathlib import Path


from stretch_animate.keyframes.record_keyframes import KeyframeRecorder
from stretch_animate.keyframes.play_keyframes import KeyframePlayer
from stretch_animate.keyframes.models import RobotJoints
from stretch4_body.core.gamepad_enums import MotionProfile
from stretch4_body.robot.robot_client import RobotClient

class DualLidarCalibration:
    """
    This is a helper class to read the dual lidar calibration file at 
    HELLO_FLEET_PATH/HELLO_FLEET_ID/calibration_dual_lidar/dual_lidar_calibration.yaml
    """
    def __init__(self, filepath=None):
        if not filepath:
            fleet_path = os.environ.get("HELLO_FLEET_PATH", "")
            fleet_id = os.environ.get("HELLO_FLEET_ID", "")
            if not fleet_path or not fleet_id:
                raise ValueError(
                    "Calibration file not provided using --calib_file, and HELLO_FLEET_PATH/HELLO_FLEET_ID environment variables are missing."
                )
            self.filepath = os.path.join(
                fleet_path,
                fleet_id,
                "calibration_dual_lidar",
                "dual_lidar_calibration.yaml",
            )
        else:
            self.filepath = filepath

        self.data = {}
        self.robot_id = os.environ.get("HELLO_FLEET_ID", "unknown")
        self.load()
        self._cached_lidar_transforms = {}

    def load(self):
        if os.path.exists(self.filepath):
            with open(self.filepath, "r") as f:
                self.data = yaml.safe_load(f) or {}

    def save(self):
        if os.path.exists(self.filepath):
            import shutil
            mod_time = int(os.path.getmtime(self.filepath))
            p = Path(self.filepath)
            backup_path = p.with_name(f"{p.stem}_backup_{mod_time}{p.suffix}")
            shutil.copy2(self.filepath, backup_path)
            print(f"Backed up {self.filepath} to {backup_path}")

        with open(self.filepath, "w") as f:
            yaml.dump(self.data, f, default_flow_style=None)

    def get_transform(self, key):
        return np.array(self.data.get(key, {}).get("data", np.eye(4)))

    def set_transform(self, key, T: np.ndarray):
        timestamp = datetime.datetime.now().isoformat()
        self.data[key] = {
            "data": T.tolist(),
            "robot_id": self.robot_id,
            "timestamp": timestamp,
        }

    @property
    def right_to_left_transform(self):
        if "right_to_left_transform" in self.data:
            return self.get_transform("right_to_left_transform")
        return None

    def apply(self, points: np.ndarray, transform: np.ndarray = None) -> np.ndarray:
        if transform is None:
            transform = self.right_to_left_transform
            if transform is None:
                return points
        ones = np.ones((points.shape[0], 1))
        pts_new = (transform @ np.hstack([points[:, :3], ones]).T).T[:, :3]
        if points.shape[1] > 3:  # carry over intensity/ring if present
            return np.hstack([pts_new, points[:, 3:]])
        return pts_new

    def get_lidar_to_base_transform(self, is_right_lidar: bool):      
        lidar_link = "lidar_right_link" if is_right_lidar else "lidar_left_link"

        if lidar_link in self._cached_lidar_transforms:
            return self._cached_lidar_transforms[lidar_link]
            

        from stretch4_urdf.utils.urdf_utils_generate_from_base_xacro import (
            get_urdf_from_robot_params,
        )
        from yourdfpy import URDF
        import io

        try:
            robot = URDF.load(io.StringIO(get_urdf_from_robot_params()))
        except Exception as e:
            print(f"Failed to load URDF: {e}")
            return np.eye(4)

        link_to_parent = {}
        for joint in robot.robot.joints:
            link_to_parent[joint.child] = (joint.parent, joint.origin)

        current = lidar_link
        chain = []
        while current != "base_link":
            if current not in link_to_parent:
                print(f"Lidar link {lidar_link} not connected to base_link")
                return np.eye(4)
            parent, origin = link_to_parent[current]
            chain.append(origin)
            current = parent

        T_base_to_lidar = np.eye(4)
        for origin in reversed(chain):
            T_j = np.eye(4) if origin is None else origin
            T_base_to_lidar = T_base_to_lidar @ T_j

        self._cached_lidar_transforms[lidar_link] = T_base_to_lidar
        return T_base_to_lidar

    def unify_clouds(
        self, left_pts: np.ndarray, right_pts: np.ndarray
    ) -> np.ndarray:
        merged = []
        if left_pts is not None and len(left_pts) > 0:
            T_lidar_to_base = self.get_lidar_to_base_transform(is_right_lidar=False)
            ones = np.ones((len(left_pts), 1))
            left_base = (T_lidar_to_base @ np.hstack([left_pts[:, :3], ones]).T).T[
                :, :3
            ]
            merged.append(left_base)

        if right_pts is not None and len(right_pts) > 0:
            T_lidar_to_base = self.get_lidar_to_base_transform(is_right_lidar=True)
            ones = np.ones((len(right_pts), 1))
            right_base = (T_lidar_to_base @ np.hstack([right_pts[:, :3], ones]).T).T[
                :, :3
            ]
            merged.append(right_base)

        if not merged:
            return np.array([])
        return np.vstack(merged)

    def get_world_transform_for_lidar(self, is_right_lidar: bool):
        T_floor_to_base = self.get_transform("floor_to_base_link_transform")
        T_base_to_lidar = self.get_lidar_to_base_transform(is_right_lidar)
        return T_floor_to_base @ T_base_to_lidar


@dataclass
class CameraDetection:
    image: np.ndarray
    camera_calibration: RGBCameraCalibration
    transform_camera: np.ndarray|None = None


@dataclass
class LidarDetection:
    transform_lidar: np.ndarray|None = None
    centroids: np.ndarray|None= None


def project_lidar_to_camera_image(
    image: np.ndarray,
    lidar_points_3d: np.ndarray,
    T_lidar_to_camera: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    distortion_model: DistortionModels,
):
    """
    Projects 3D lidar points onto the camera image and colors them by depth.
    lidar_points_3d: (N, 3)
    T_lidar_to_camera: 4x4 transform
    """
    if len(lidar_points_3d) == 0:
        return image.copy()

    # Transform points to camera frame
    ones = np.ones((lidar_points_3d.shape[0], 1))
    pts_homo = np.hstack([lidar_points_3d, ones])
    pts_cam = (T_lidar_to_camera @ pts_homo.T).T[:, :3]

    # Filter points behind camera
    valid_idx = pts_cam[:, 2] > 0
    pts_cam = pts_cam[valid_idx]

    if len(pts_cam) == 0:
        return image.copy()

    rvec = np.zeros(3)
    tvec = np.zeros(3)

    img_pts = project_points(
        pts_cam, rvec, tvec, camera_matrix, dist_coeffs, distortion_model=distortion_model
    )
    img_pts = img_pts.reshape(-1, 2)

    out_img = image.copy()
    h, w = out_img.shape[:2]

    # Color map by depth (Z in camera frame)
    z_vals = pts_cam[:, 2]
    z_min, z_max = np.percentile(z_vals, 5), np.percentile(z_vals, 95)
    if z_max == z_min:
        z_max = z_min + 0.1

    norm_z = np.clip((z_vals - z_min) / (z_max - z_min), 0, 1)

    u = np.round(img_pts[:, 0]).astype(int)
    v = np.round(img_pts[:, 1]).astype(int)

    valid_uv = (u >= 0) & (u < w) & (v >= 0) & (v < h)

    u_valid = u[valid_uv]
    v_valid = v[valid_uv]
    norm_z_valid = norm_z[valid_uv]

    if len(u_valid) > 0:
        colors_mapped = cv2.applyColorMap(
            (norm_z_valid * 255).astype(np.uint8), cv2.COLORMAP_JET
        ).reshape(-1, 3)

        for i in range(len(u_valid)):
            cv2.circle(
                out_img, (u_valid[i], v_valid[i]), 2, colors_mapped[i].tolist(), -1
            )

    return out_img


def create_colored_pointcloud(
    image: np.ndarray,
    lidar_points_3d: np.ndarray,
    T_lidar_to_camera: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    distortion_model: DistortionModels,
):
    """
    Returns (filtered_lidar_points, colors) by sampling the image.
    """
    if len(lidar_points_3d) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3))

    ones = np.ones((lidar_points_3d.shape[0], 1))
    pts_homo = np.hstack([lidar_points_3d, ones])
    pts_cam = (T_lidar_to_camera @ pts_homo.T).T[:, :3]

    valid_idx = pts_cam[:, 2] > 0
    if not np.any(valid_idx):
        return np.zeros((0, 3)), np.zeros((0, 3))

    pts_cam_valid = pts_cam[valid_idx]
    orig_pts_valid = lidar_points_3d[valid_idx]

    rvec = np.zeros(3)
    tvec = np.zeros(3)
    img_pts = project_points(
        pts_cam_valid, rvec, tvec, camera_matrix, dist_coeffs, distortion_model
    ).reshape(-1, 2)

    h, w = image.shape[:2]

    u = np.round(img_pts[:, 0]).astype(int)
    v = np.round(img_pts[:, 1]).astype(int)

    valid_uv = (u >= 0) & (u < w) & (v >= 0) & (v < h)

    u_valid = u[valid_uv]
    v_valid = v[valid_uv]

    colors_bgr = image[v_valid, u_valid]
    colors_rgb = colors_bgr[:, ::-1]  # BGR to RGB

    final_pts = orig_pts_valid[valid_uv]

    return final_pts, colors_rgb


def cluster_points(
    points: np.ndarray,
    expected_width: float,
    expected_height: float,
    tolerance: float,
    threshold: float = 0.05,
    expected_clusters: int = 4,
):
    """
    Clusters high-intensity points based on distance.
    Returns the centroids that match the expected rectangle dimensions.
    """
    import itertools

    clusters = []
    for p in points:
        found = False
        for c in clusters:
            if np.linalg.norm(c[0] - p[:3]) < threshold:
                c[1].append(p[:3])
                c[0] = np.mean(c[1], axis=0)
                found = True
                break
        if not found:
            clusters.append([p[:3], [p[:3]]])

    # Try combinations of the largest clusters first
    clusters.sort(key=lambda x: len(x[1]), reverse=True)
    if len(clusters) < expected_clusters:
        return (
            None,
            f"Only found {len(clusters)} clusters, expected {expected_clusters}",
        )

    for combo in itertools.combinations(clusters, expected_clusters):
        centroids = np.array([c[0] for c in combo])
        valid, msg = evaluate_lidar_rectangle(
            centroids, expected_width, expected_height, tolerance
        )
        if valid:
            return centroids, msg

    # If no valid combo, return the first one with its failure message
    for combo in itertools.combinations(clusters, expected_clusters):
        centroids = np.array([c[0] for c in combo])
        _, msg = evaluate_lidar_rectangle(
            centroids, expected_width, expected_height, tolerance
        )
        return centroids, msg

    return None, "Not enough points to form 4 clusters"


def compute_lidar_rectangle_pose(
    centroids: np.ndarray,
    expected_width: float,
    expected_height: float,
    transform_camera: np.ndarray | None = None,
):
    """
    Computes the 4x4 matrix transform_lidar tracking the lidar rectangle pose in lidar frame.
    To resolve the X/Y axes sign ambiguity:
    - We match the width to the Charuco X axis.
    - We match the height to the Charuco Y axis.
    """
    center = np.mean(centroids, axis=0)
    centered = centroids - center
    cov = centered.T @ centered
    direction_principal_components, variance, _ = np.linalg.svd(cov)

    origin = direction_principal_components[:, 0]
    tangent = direction_principal_components[:, 1]
    normal = direction_principal_components[:, 2]

    # Ensure normal points towards origin (Lidar is at 0,0,0)
    if np.dot(normal, center) > 0:
        normal = -normal

    # Match axes based on dimensions
    if expected_width > expected_height:
        e_width = origin
        e_height = tangent
    else:
        e_width = tangent
        e_height = origin

    x_ax = e_width
    y_ax = e_height

    z_ax = np.cross(x_ax, y_ax)

    transform_lidar = np.eye(4)
    transform_lidar[:3, :3] = np.column_stack((x_ax, y_ax, z_ax))
    transform_lidar[:3, 3] = center
    return transform_lidar


def evaluate_lidar_rectangle(
    centroids: np.ndarray,
    expected_width: float,
    expected_height: float,
    tolerance: float,
) -> tuple[bool, str]:
    if centroids is None or len(centroids) != 4:
        return False, "Not exactly 4 centroids"

    center = np.mean(centroids, axis=0)
    centered = centroids - center
    cov = centered.T @ centered
    direction_principal_components, variance, _ = np.linalg.svd(cov)

    # Check planarity: RMS out of plane distance
    z_var = variance[2] / 4.0
    if np.sqrt(z_var) > 0.10:  # 10cm tolerance for planarity
        return False, f"Not planar enough. RMS z: {np.sqrt(z_var):.3f}m > 0.10m"

    # Project onto principal axes
    proj1 = [np.dot(p, direction_principal_components[:, 0]) for p in centered]
    proj2 = [np.dot(p, direction_principal_components[:, 1]) for p in centered]

    span1 = np.ptp(proj1)
    span2 = np.ptp(proj2)

    actual_long_side = max(span1, span2)
    actual_short_side = min(span1, span2)

    if expected_width > expected_height:
        expected_long_side = expected_width
        expected_short_side = expected_height
    else:
        expected_long_side = expected_height
        expected_short_side = expected_width

    msg = (
        f"Target: {expected_long_side:.3f}x{expected_short_side:.3f} (±{tolerance:.3f})\\n"
        f"Guess: {actual_long_side:.3f}x{actual_short_side:.3f}\\n"
        f"Off by: {actual_long_side - expected_long_side:.3f}, {actual_short_side - expected_short_side:.3f}"
    )

    if (
        actual_long_side > expected_long_side + tolerance
        or actual_short_side > expected_short_side + tolerance
    ):
        return False, msg + "\\nStatus: Too large"

    if (
        actual_long_side < expected_long_side - tolerance
        or actual_short_side < expected_short_side - tolerance
    ):
        return False, msg + "\\nStatus: Too small"

    return True, msg + "\\nStatus: Valid"


def get_high_intensity_points(
    points, intensities: np.ndarray, intensity_threshold: float = 240.0
):
    """
    Returns the high intensity points from the lidar points.
    """
    return points[intensities > intensity_threshold]


def detect_lidar_rectangle(
    high_intensity_points: np.ndarray,
    expected_width: float,
    expected_height: float,
    tolerance: float,
    transform_camera: np.ndarray | None = None,
):
    """
    Returns transform_lidar matrix and centroids.
    points: Nx4 array (x, y, z, intensity)
    """
    if len(high_intensity_points) < 10:
        return None, None, "Not enough high intensity points"

    centroids, msg = cluster_points(
        high_intensity_points, expected_width, expected_height, tolerance
    )
    if centroids is None:
        return None, None, msg

    # rr.log(
    #     "debug/lidar_rectangle_centroids",
    #     rr.Points3D(centroids, colors=[0, 255, 0]),
    # )

    valid, _ = evaluate_lidar_rectangle(
        centroids, expected_width, expected_height, tolerance
    )
    if not valid:
        return None, None, msg

    transform_lidar = compute_lidar_rectangle_pose(
        centroids, expected_width, expected_height, transform_camera
    )
    return transform_lidar, centroids, msg


def average_transforms(T_list):
    if not T_list:
        return None
    t_avg = np.mean([T[:3, 3] for T in T_list], axis=0)
    quats = [Rotation.from_matrix(T[:3, :3]).as_quat() for T in T_list]
    mean_rot = Rotation.from_quat(quats).mean().as_matrix()
    T_avg = np.eye(4)
    T_avg[:3, :3] = mean_rot
    T_avg[:3, 3] = t_avg
    return T_avg


def log_to_rerun(
    camera_name: str,
    lidar_name: str,
    image: np.ndarray,
    projected_image: np.ndarray,
    colored_points: np.ndarray,
    pointcloud_colors: np.ndarray,
    T_lidar_to_camera: np.ndarray,
    T_world: np.ndarray | None = None,
):
    """
    Logs the images and point cloud to Rerun.
    T_world (optional): 4x4 transform from floor to lidar.
    """
    rr.log(f"{camera_name}/image", rr.Image(image, color_model="BGR").compress())
    rr.log(
        f"{camera_name}/projected_image",
        rr.Image(projected_image, color_model="BGR").compress(),
    )

    # Log point clouds by manually transforming them without using rerun transforms
    if len(colored_points) > 0:
        # 1. Colored point cloud natively in the Lidar frame
        rr.log(
            f"Lidars/{lidar_name}/colored_cloud",
            rr.Points3D(colored_points, colors=pointcloud_colors, radii=[0.005]),
        )

        # 2. Colored point cloud transformed into the Camera frame
        ones = np.ones((colored_points.shape[0], 1))
        pts_cam = (T_lidar_to_camera @ np.hstack([colored_points, ones]).T).T[:, :3]
        rr.log(
            f"Cameras/{camera_name}/colored_cloud",
            rr.Points3D(pts_cam, colors=pointcloud_colors, radii=[0.005]),
        )

        # 3. Colored point cloud transformed into the World frame
        if T_world is not None:
            pts_world = (T_world @ np.hstack([colored_points, ones]).T).T[:, :3]
            rr.log(
                f"world/{lidar_name}/world_cloud",
                rr.Points3D(pts_world, colors=pointcloud_colors, radii=[0.005]),
            )


class CalibrateLidarToCamera:
    def __init__(
        self,
        camera: str,
        lidar: str,
        use_ros_for_lidars: bool,
        charuco_board_name: str,
        expected_width: float,
        expected_height: float,
        tolerance: float,
        use_gamepad: bool,
        calib_file: str | None = None,
        skip_user_prompt: bool = False,
        replay_from_folder: str | None = None,
        replay_last: bool = False,
    ):
        self.captured_transforms: list[np.ndarray] = []
        self.current_average_transform: np.ndarray | None = None

        self.camera_name = camera
        self.lidar_name = lidar
        self.skip_user_prompt = skip_user_prompt

        # Assign camera
        self.camera:RGBCameras
        if self.camera_name == "left":
            self.camera = RGBCameras.left()
        elif self.camera_name == "right":
            self.camera = RGBCameras.right()
        else:
            self.camera = RGBCameras.center()

        self.camera_calibration = self.camera.load_calibration()

        self.replay_folder = None
        base_dir = os.path.join(DEFAULT_CALIBRATION_FOLDER_PATH, "calibration_lidar_points")
        if replay_from_folder:
            self.replay_folder = os.path.join(base_dir, replay_from_folder)
        elif replay_last:
            if os.path.exists(base_dir):
                folders = sorted([f for f in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, f))])
                if folders:
                    self.replay_folder = os.path.join(base_dir, folders[-1])
        
        self.is_replaying = self.replay_folder is not None and os.path.exists(self.replay_folder)
        self.use_right_lidar = self.lidar_name == "right"
        self.session_timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_base_dir = os.path.join(base_dir, self.session_timestamp)
        self.saved_poses = 0

        if self.is_replaying:
            print(f"Replaying from folder: {self.replay_folder}")
            self.replay_poses = sorted([int(d) for d in os.listdir(self.replay_folder) if d.isdigit()])
            self.replay_idx = 0
            
            class ReplayCameraAdapter:
                def ReplayCameraAdapter(self, manager):
                    self.manager = manager
                def get_frames(self):
                    while True:
                        if self.manager.replay_idx < len(self.manager.replay_poses):
                            pose_dir = os.path.join(self.manager.replay_folder, str(self.manager.replay_poses[self.manager.replay_idx]))
                            img = cv2.imread(os.path.join(pose_dir, "rgb.png"))
                            if img is not None:
                                from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame
                                yield ImageFrame(timestamp=0.0, frame_number=self.manager.replay_idx, image=img)
                                continue
                        yield None
                def stop(self):
                    pass
            self.camera_adapter = ReplayCameraAdapter(self)

            class ReplayLidarFrame:
                def __init__(self, p, i):
                    self.points = p
                    self.intensity = i
            def replay_lidar_stream():
                while True:
                    if self.replay_idx < len(self.replay_poses):
                        pose_dir = os.path.join(self.replay_folder, str(self.replay_poses[self.replay_idx]))
                        pts_path = os.path.join(pose_dir, "points.npy")
                        int_path = os.path.join(pose_dir, "intensity.npy")
                        if os.path.exists(pts_path) and os.path.exists(int_path):
                            pts = np.load(pts_path)
                            intensity = np.load(int_path)
                            yield ReplayLidarFrame(pts, intensity)
                            continue
                    yield None
            self.lidar_stream = replay_lidar_stream()
        else:
            # Start camera adapter
            self.camera_adapter = self.camera.start()

            # Start lidars
            if use_ros_for_lidars:
                try:
                    from stretch_python_bridge import stream_lidar_left, stream_lidar_right # This is a ros2 package, requires a colcon build
                except ImportError:
                    raise ImportError("stretch_python_bridge not found. Did you colcon build?")
            else:
                try:
                    from pyhesai_wrapper import stream_lidar_left, stream_lidar_right
                except ImportError:
                    raise ImportError("pyhesai_wrapper not found. Please install it or use the `--use_ros_for_lidars` flag.")
            
            self.lidar_stream = (
                stream_lidar_right()
                if self.use_right_lidar
                else stream_lidar_left()
            )

        print(f"Using Camera: {self.camera_name}, Lidar: {self.lidar_name}")

        # Ensure output file path
        self.calib_file = calib_file
        if not self.calib_file:
            fleet_path = os.environ.get("HELLO_FLEET_PATH", "")
            fleet_id = os.environ.get("HELLO_FLEET_ID", "")
            if fleet_path and fleet_id:
                self.calib_file = os.path.join(
                    fleet_path,
                    fleet_id,
                    "calibration_dual_lidar",
                    "camera_lidar_extrinsic_transform.yaml",
                )
            else:
                raise ValueError(
                    "Calibration file not provided using --calib_file, and HELLO_FLEET_PATH/HELLO_FLEET_ID environment variables are missing."
                )

        print(f"Calibration results will be saved to: {self.calib_file}")

        # Load Calibration Class
        self.base_calibration = DualLidarCalibration() # Base for floor_to_base and lidar_to_base
        self.calibration = DualLidarCalibration(self.calib_file)

        # World transform
        self.T_world = None
        try:
            self.T_world = self.base_calibration.get_world_transform_for_lidar(
                self.use_right_lidar
            )
        except Exception as e:
            print(f"Warning: could not get T_world: {e}")

        key = f"transform_{self.lidar_name}_lidar_to_{self.camera.name}"
        if key in self.calibration.data and "data" in self.calibration.data[key]:
            self.current_average_transform = np.array(
                self.calibration.data[key]["data"]
            )

        self.charuco_board = CharucoBoards[
            charuco_board_name
        ].get_board_config(use_high_MP_corner_refinement=self.camera.is_center())

        self.expected_width = expected_width
        self.expected_height = expected_height
        self.tolerance = tolerance

        if self.is_replaying:
            self.move_robot_mode = MoveRobotMode.ARM_POSES
        elif use_gamepad:
            self.move_robot_mode = MoveRobotMode.GAMEPAD_MODE
        else:
            self.move_robot_mode = MoveRobotMode.ARM_POSES

        self.is_capture_requested = False
        self.save_requested = False
        self.quit_requested = threading.Event()

        self.gamepad_teleop = GamePadTeleop(use_server=True, cb_loop=None)
        self.gamepad_teleop.sleep = 0
        self.gamepad_teleop.startup()
        self.left_button_counter = ButtonPressCounter("left_button_pressed")

        self.keyframe_recorder = KeyframeRecorder()

        self.robot = RobotClient()
        self.robot.startup()
        joints_allowed_to_move = [
            j
            for j in RobotJoints
            if j not in [RobotJoints.base, RobotJoints.stretch_gripper]
        ]
        self.keyframe_player = KeyframePlayer(
            joints_allowed_to_move=joints_allowed_to_move,
            motion_profile=MotionProfile.SLOW,
            robot=self.robot,
        )
        poses_file = (
            Path(__file__).parent.absolute()
            / "models/calibration_poses_extrinsics.json"
        )
        if poses_file.exists():
            self.keyframe_player.load_from_file(poses_file)
        else:
            print(f"Warning: poses file {poses_file} does not exist!")

        self.frame_settled_detector = DetectFrameSettled()

    def get_latest_camera_frame(self):
        for frame in self.camera_adapter.get_frames():
            return frame
        return None

    def _process_camera_frame(self, frame) -> CameraDetection | None:
        """
        Process a camera frame to detect the ChArUco board and return the charuco board's pose.
        """
        detection = CameraDetection(image=frame.image.copy(), camera_calibration=self.camera_calibration)

        img = detection.image
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            self.charuco_board.charuco_detector.detectBoard(gray)
        )

        board = self.charuco_board.charuco_detector.getBoard()

        if marker_ids is not None:
            valid_board_ids = set(np.array(board.getIds()).flatten())
            filtered_marker_corners = []
            filtered_marker_ids = []
            for corner, m_id in zip(marker_corners, marker_ids.flatten()):
                if m_id in valid_board_ids:
                    filtered_marker_corners.append(corner)
                    filtered_marker_ids.append([m_id])
            # marker_corners = tuple(filtered_marker_corners)
            marker_ids = (
                np.array(filtered_marker_ids, dtype=np.int32)
                if filtered_marker_ids
                else None
            )

        object_points = []
        image_points = []
        if charuco_ids is not None and len(charuco_ids) > 0:
            (
                object_points,
                image_points,
            ) = board.matchImagePoints(charuco_corners, charuco_ids, None, None)

        object_points = np.array(object_points, dtype=np.float32)
        image_points = np.array(image_points, dtype=np.float32)

        if len(object_points) < 6:
            return None

        if charuco_corners is not None and len(charuco_corners) > 0:
            cv2.aruco.drawDetectedCornersCharuco(
                img, charuco_corners, charuco_ids, (0, 255, 0)
            )

        success, rvec, tvec = solve_pnp(
            object_points=object_points,
            image_points=image_points,
            camera_matrix=self.camera_calibration.camera_matrix,
            distortion_coefficients=self.camera_calibration.distortion_coefficients,
            distortion_model=self.camera_calibration.distortion_model,
        )

        if success:
            R, _ = cv2.Rodrigues(rvec)

            board_width = self.charuco_board.size[0] * self.charuco_board.square_length
            board_height = self.charuco_board.size[1] * self.charuco_board.square_length
            center_offset = np.array([[board_width / 2.0], [board_height / 2.0], [0.0]])

            tvec_centered = (R @ center_offset) + tvec.reshape((3, 1))

            transform_camera = np.eye(4)
            transform_camera[:3, :3] = R
            transform_camera[:3, 3] = tvec_centered.flatten()

            detection.transform_camera = transform_camera

            draw_frame_axes(
                img,
                self.camera_calibration.camera_matrix,
                self.camera_calibration.distortion_coefficients,
                rvec,
                tvec_centered,
                0.1,
                distortion_model=self.camera_calibration.distortion_model,
            )

        return detection

    def log_camera_detection(self, img, detection: CameraDetection):
        """
        Log the camera detection to Rerun.
        """
        transform = detection.transform_camera
        rr.log(
            f"Cameras/{self.camera.name}", rr.Image(img, color_model="BGR").compress()
        )
        if transform is not None:
            rr.log(
                f"Cameras/{self.camera.name}/charuco_pose",
                rr.Transform3D(
                    translation=transform[:3, 3],
                    mat3x3=transform[:3, :3],
                ),
                rr.TransformAxes3D(
                    axis_length=0.1
                ),  # You can also just pass the float positionally: rr.TransformAxes3D(0.1)
            )
        else:
            rr.log(
                f"Cameras/{self.camera.name}/charuco_pose", rr.Clear(recursive=False)
            )

    def _process_lidar_frame(
        self, lidar_frame, transform_camera: np.ndarray | None
    ) -> LidarDetection:
        """
        Process a lidar frame to detect the lidar rectangle and return the pose of that virtual rectangle.

        The virtual lidar rectangle is a rectangle formed by connecting the four reflective markers on the corners of the Calibration Board tool. 
        """
        detection = LidarDetection()

        rr.log(
            f"Lidars/{self.lidar_name}", rr.Points3D(lidar_frame.points, radii=[0.001])
        )

        high_intensity_points = get_high_intensity_points(
            lidar_frame.points, lidar_frame.intensity
        )
        rr.log(
            f"Lidars/{self.lidar_name}/high_intensity_points",
            rr.Points3D(
                high_intensity_points[:, :3], radii=[0.003], colors=[255, 0, 0]
            ),
        )
        transform_lidar, centroids, msg = detect_lidar_rectangle(
            high_intensity_points,
            expected_width=self.expected_width,
            expected_height=self.expected_height,
            tolerance=self.tolerance,
            transform_camera=transform_camera,
        )

        if msg:
            rr.log(
                f"Lidars/{self.lidar_name}/dimensions_debug",
                rr.TextDocument(msg, media_type="text/markdown"),
            )

        detection.transform_lidar = transform_lidar
        detection.centroids = centroids

        if centroids is not None:
            rr.log(
                f"Lidars/{self.lidar_name}/circles",
                rr.Points3D(centroids, colors=[255, 0, 0], radii=[0.01]),
            )
            c_center = np.mean(centroids, axis=0)
            rr.log(
                f"Lidars/{self.lidar_name}/dimensions_label",
                rr.Points3D(
                    [c_center],
                    labels=[msg.replace("\\n", " | ")],
                    radii=[0.001],
                    colors=[255, 255, 255],
                ),
            )

            # Order centroids to draw a proper rectangle
            c_center = np.mean(centroids, axis=0)
            c_centered = centroids - c_center
            c_cov = c_centered.T @ c_centered
            c_U, _, _ = np.linalg.svd(c_cov)
            c_coords_2d = np.array(
                [[np.dot(p, c_U[:, 0]), np.dot(p, c_U[:, 1])] for p in c_centered]
            )
            c_angles = np.arctan2(c_coords_2d[:, 1], c_coords_2d[:, 0])
            ordered_centroids = centroids[np.argsort(c_angles)]

            # Draw a box looping through ordered centroids
            lines = [
                (ordered_centroids[i], ordered_centroids[(i + 1) % 4]) for i in range(4)
            ]
            rr.log(
                f"Lidars/{self.lidar_name}/virtual_rectangle",
                rr.LineStrips3D([np.array(l) for l in lines], colors=[0, 255, 0]),
            )

        if transform_lidar is not None:
            rr.log(
                f"Lidars/{self.lidar_name}/rectangle_pose",
                rr.Transform3D(
                    translation=transform_lidar[:3, 3], mat3x3=transform_lidar[:3, :3]
                ),
                rr.TransformAxes3D(
                    axis_length=0.1
                ),  # You can also just pass the float positionally: rr.TransformAxes3D(0.1)
            )
        else:
            # Clear stale visualizations if tracking is lost
            rr.log(f"Lidars/{self.lidar_name}/circles", rr.Clear(recursive=False))
            rr.log(
                f"Lidars/{self.lidar_name}/virtual_rectangle", rr.Clear(recursive=False)
            )
            rr.log(
                f"Lidars/{self.lidar_name}/rectangle_pose", rr.Clear(recursive=False)
            )
            rr.log(
                f"Lidars/{self.lidar_name}/dimensions_label", rr.Clear(recursive=False)
            )

        return detection

    def _spawn_rerun_with_blueprint(self):
        rr.spawn(memory_limit="1GB")

        import rerun.blueprint as rrb

        blueprint = rrb.Blueprint(
            rrb.Vertical(
                rrb.Spatial3DView(
                    name="Lidar Frame Visualization",
                    origin="Lidars",
                    contents=[
                        "+ Lidars/**",
                    ],
                    eye_controls=rrb.EyeControls3D(
                        tracking_entity=f"Lidars/{self.lidar_name}/virtual_rectangle",
                    ),
                ),
                rrb.Horizontal(
                    rrb.Spatial2DView(
                        name="Camera Live Stream",
                        origin="Cameras",
                    ),
                    rrb.Spatial2DView(
                        name="Projected Live Stream",
                        origin=self.camera.name,
                    ),
                    rrb.TextLogView(
                        name="Logs",
                        origin="Logs",
                    ),
                ),
            ),
            collapse_panels=True,
        )
        rr.send_blueprint(blueprint)


    def run(self, is_interactive:bool):
        rr.init("Extrinsics Interactive Calibration", spawn=False)
        if is_interactive:
            self._spawn_rerun_with_blueprint()

        def request_capture():
            if self.move_robot_mode == MoveRobotMode.GAMEPAD_MODE:
                rr.log(
                    "Logs/action",
                    rr.TextLog("Gamepad X Tap: Capture Requested", level="INFO"),
                )
                self.keyframe_recorder.capture_pose()
            self.is_capture_requested = True

        def request_save():
            self.save_requested = True

        is_paused = threading.Event()
        if not self.skip_user_prompt:
            is_paused.set()
        def gamepad_poller():
            def trigger_pause(wait_for_x):
                if is_paused.is_set():
                    rr.log(
                        "Logs/action",
                        rr.TextLog("Unpaused. Automatic movement will start!", level="INFO"),
                    )
                    wait_for_x.clear()
                else:
                    rr.log(
                        "Logs/action",
                        rr.TextLog("Pausing automatic movement.", level="INFO"),
                    )
                    wait_for_x.set()

            while not self.quit_requested.is_set():
                if self.gamepad_teleop is not None:
                    self.gamepad_teleop.step_mainloop()

                    if self.gamepad_teleop.controller_state is not None:
                        self.left_button_counter.step(
                            controller_state=self.gamepad_teleop.controller_state
                        )
                        if self.move_robot_mode == MoveRobotMode.GAMEPAD_MODE:
                            self.left_button_counter.trigger_on_tap(
                                callback=request_capture
                            )
                            self.left_button_counter.trigger_on_hold(
                                4, callback=request_save
                            )
                        else:
                            def _unpause_and_capture():
                                trigger_pause(is_paused)
                                request_capture()
                            self.left_button_counter.trigger_on_tap(
                                callback=_unpause_and_capture
                            )
                            self.left_button_counter.trigger_on_hold(
                                4, callback=request_save
                            )

                if self.robot.power_periph.status["runstop_event"]:
                    rr.log(
                        "Logs/error",
                        rr.TextLog("Runstop event triggered, pausing automatic movement.", level="ERROR"),
                    )
                    is_paused.set()
                    continue

                time.sleep(1 / 30)

        threading.Thread(target=gamepad_poller, daemon=True).start()

        if self.move_robot_mode == MoveRobotMode.GAMEPAD_MODE:
            rr.log(
                "Logs/action",
                rr.TextLog("Started manual gamepad mode. Use Gamepad to move. Tap X to capture. Hold X to save average.", level="INFO"),
            )
        else:
            rr.log(
                "Logs/action",
                rr.TextLog("Started automatic replay mode. Press X to start moving to poses.", level="INFO"),
            )

        latest_cam_frame = None
        latest_lidar_frame = None
        is_frame_settled = False
        is_capture_frame = False

        def move_to_next_pose():
            if is_paused.is_set():
                return
            if self.is_replaying:
                self.replay_idx += 1
                request_capture()
            else:
                self.keyframe_player.play_next(loop=False)
                request_capture()

        if self.move_robot_mode == MoveRobotMode.ARM_POSES:
            # Go to first pose
            move_to_next_pose()

        
        while True:
            time.sleep(0.01)

            if self.save_requested:
                rr.log(
                    "Logs/action",
                    rr.TextLog("Save requested via gamepad X hold!", level="INFO"),
                )
                self.save_requested = False
                self.save()

            if self.move_robot_mode == MoveRobotMode.ARM_POSES:
                if self.is_replaying:
                    if self.replay_idx >= len(self.replay_poses):
                        print("Finished replaying all poses. Saving and exiting.")
                        self.save()
                        break
                else:
                    if self.keyframe_player.current_pose_index >= len(self.keyframe_player.poses):
                        print("Finished replaying all poses. Saving and exiting.")
                        self.save()
                        break
                    
            cam_frame_tmp = self.get_latest_camera_frame()
            if cam_frame_tmp is not None:
                latest_cam_frame = cam_frame_tmp
                is_frame_settled = self.frame_settled_detector.check_stability_diff(
                    latest_cam_frame.image, threshold=3
                )

            lidar_frame_tmp = next(self.lidar_stream)
            if lidar_frame_tmp is not None:
                latest_lidar_frame = lidar_frame_tmp

            if latest_cam_frame is None or latest_lidar_frame is None:
                print("lidar and camera frames are both None")
                continue

            camera_detection = self._process_camera_frame(latest_cam_frame)
            display_img = (
                latest_cam_frame.image.copy() if camera_detection is None else camera_detection.image
            )

            if camera_detection is None:
                rr.log(
                    f"Cameras/{self.camera.name}",
                    rr.Image(display_img, color_model="BGR").compress(),
                )
            is_capture_frame = False
            if self.is_capture_requested:
                if is_frame_settled:  
                    if camera_detection is None:
                        rr.log(
                            "Logs/error",
                            rr.TextLog(
                                "Capture Failed: Charuco board not fully visible in Camera",
                                level="ERROR",
                            ),
                        )
                        self.is_capture_requested = False

                        if self.move_robot_mode == MoveRobotMode.ARM_POSES:
                            print("This pose did not have a valid frame! Skipping to next pose.")
                            move_to_next_pose()
                        continue

                    rr.log(
                        "Logs/action",
                        rr.TextLog("Capturing this frame!", level="INFO"),
                    )
                    self.is_capture_requested = False
                    is_capture_frame = True

                    if self.move_robot_mode == MoveRobotMode.ARM_POSES:
                        move_to_next_pose()

                else:
                    if self.move_robot_mode == MoveRobotMode.GAMEPAD_MODE:
                        rr.log(
                            "Logs/error",
                            rr.TextLog(
                                "Capture Failed: Frame not settled", level="ERROR"
                            ),
                        )

            if camera_detection is None:
                continue

            self.log_camera_detection(display_img, camera_detection)

            lidar_det = self._process_lidar_frame(
                latest_lidar_frame, camera_detection.transform_camera
            )

            if not is_capture_frame:
                continue

            did_capture_this_frame = False
            lidar_pts = latest_lidar_frame.points

            T_lidar_to_camera = None
            if (
                camera_detection.transform_camera is not None
                and lidar_det.transform_lidar is not None
            ):
                T_lidar_to_camera = camera_detection.transform_camera @ np.linalg.inv(
                    lidar_det.transform_lidar
                )
                if is_capture_frame:
                    self.captured_transforms.append(T_lidar_to_camera)

                    T_avg = average_transforms(self.captured_transforms)

                    self.current_average_transform = T_avg

                    t_str = np.array2string(
                        T_lidar_to_camera,
                        formatter={"float_kind": lambda x: "%.4f" % x},
                    )

                    l_key = f"{self.lidar_name}_lidar"
                    c_key = self.camera.name
                    msg = f"""Captured transform for {c_key} <-> {l_key}! 
                    Current Transform: {t_str}
                    Average Transform: {np.array2string(T_avg, formatter={"float_kind": lambda x: "%.4f" % x})}
                    Total: {len(self.captured_transforms)}"""
                    print(msg)
                    rr.log(
                        f"Logs/capture",
                        rr.TextLog(msg, level="INFO"),
                    )

                    did_capture_this_frame = True

                    if not self.is_replaying:
                        pose_dir = os.path.join(self.save_base_dir, str(self.saved_poses))
                        os.makedirs(pose_dir, exist_ok=True)
                        cv2.imwrite(os.path.join(pose_dir, "rgb.png"), display_img)
                        np.save(os.path.join(pose_dir, "points.npy"), latest_lidar_frame.points)
                        if hasattr(latest_lidar_frame, "intensity"):
                            np.save(os.path.join(pose_dir, "intensity.npy"), latest_lidar_frame.intensity)
                        self.saved_poses += 1

            T_display = (
                self.current_average_transform
                if self.current_average_transform is not None
                else T_lidar_to_camera
            )

            if T_display is not None:
                proj_img = project_lidar_to_camera_image(
                    camera_detection.image,
                    lidar_pts,
                    T_display,
                    camera_detection.camera_calibration.camera_matrix,
                    camera_detection.camera_calibration.distortion_coefficients,
                    distortion_model=camera_detection.camera_calibration.distortion_model,
                )
                c_pts, c_cols = create_colored_pointcloud(
                    camera_detection.image,
                    lidar_pts,
                    T_display,
                    camera_detection.camera_calibration.camera_matrix,
                    camera_detection.camera_calibration.distortion_coefficients,
                    distortion_model=camera_detection.camera_calibration.distortion_model,
                )
                l_key = f"{self.lidar_name}_lidar"
                c_key = self.camera.name
                log_to_rerun(
                    camera_name=c_key,
                    lidar_name=l_key,
                    image=camera_detection.image,
                    projected_image=proj_img,
                    colored_points=c_pts,
                    pointcloud_colors=c_cols,
                    T_lidar_to_camera=T_display,
                    T_world=self.T_world,
                )
                rr.log(
                    f"Transform/{l_key}_to_{c_key}",
                    rr.Transform3D(
                        translation=T_display[:3, 3],
                        mat3x3=T_display[:3, :3],
                    ),
                )

            if is_capture_frame and not did_capture_this_frame:
                msg = "Capture requested, but failed! (Board not fully visible to both Camera and Lidar)"
                print(msg)
                rr.log("Logs/error", rr.TextLog(msg, level="WARN"))



        print("Quitting...")

        self.quit_requested.set()
        
        try:
            self.lidar_stream.close()
        except Exception:
            pass

        self.camera_adapter.stop()
        self.robot.stop()

    def save(self):
        if len(self.captured_transforms) == 0:
            return

        l_key = f"{self.lidar_name}_lidar"
        c_key = self.camera.name

        t_str = np.array2string(
            self.current_average_transform,
            formatter={"float_kind": lambda x: "%.4f" % x},
        )
        rr.log(
            f"Logs/average",
            rr.TextLog(
                f"New Average Transform for {c_key} <-> {l_key}:\n{t_str}",
                level="INFO",
            ),
        )

        from stretch4_body.subsystem.cameras.calibrate_extrinsics_cameras import CAMERA_EXTRINSICS_YAML_PATH
        import yaml
        import os
        from pathlib import Path
        import datetime
        import shutil

        key = f"transform_{self.lidar_name}_lidar_to_{self.camera.name}"
        
        out_yaml = CAMERA_EXTRINSICS_YAML_PATH
        out_dir = os.path.dirname(out_yaml)
        os.makedirs(out_dir, exist_ok=True)

        if os.path.exists(out_yaml):
            mod_time = int(os.path.getmtime(out_yaml))
            p = Path(out_yaml)
            backup_path = p.with_name(f"{p.stem}_backup_{mod_time}{p.suffix}")
            shutil.copy2(out_yaml, backup_path)
            print(f"Backed up {out_yaml} to {backup_path}")

        if os.path.exists(out_yaml):
            with open(out_yaml, "r") as f:
                existing_data = yaml.safe_load(f) or {}
        else:
            existing_data = {}

        # Use the format expected by DualLidarCalibration just in case, but store it in camera_extrinsics
        existing_data[key] = {
            "data": self.current_average_transform.tolist(),
            "robot_id": os.environ.get("HELLO_FLEET_ID", ""),
            "timestamp": datetime.datetime.now().isoformat(),
        }

        with open(out_yaml, "w") as f:
            yaml.dump(existing_data, f, default_flow_style=False)
            
        print(f"Saved transform {l_key} <-> {c_key} to {out_yaml}")

        if self.move_robot_mode == MoveRobotMode.GAMEPAD_MODE:
            poses_file = str(
                Path(__file__).parent.absolute()
                / "models/calibration_poses_extrinsics.json"
            )
            self.keyframe_recorder.save_to_file(poses_file)
            print(f"Saved poses to {poses_file}")
            rr.log(
                "Logs/action",
                rr.TextLog(
                    "Gamepad X Hold: Extrinsics Poses & Transform Saved!",
                    level="INFO",
                ),
            )


def calibrate_extrinsics_camera_lidar():
    parser = argparse.ArgumentParser(
        "Interactive Extrinsics calibration using Gamepad."
    )

    # Camera/Lidar single args
    parser.add_argument(
        "-c",
        "--camera",
        type=str,
        choices=["left", "right", "center"],
        default="center",
        help="Name of the RGB camera to use.",
    )
    parser.add_argument(
        "-l",
        "--lidar",
        type=str,
        choices=["left", "right"],
        default="right",
        help="Name of the Hesai lidar to use.",
    )

    parser.add_argument(
        "--gamepad",
        action="store_true",
        help="Manual gamepad teleop and calibration frame capture",
    )
    parser.add_argument(
        "--replay",
        action="store_true",
        help="Automatic arm movement and calibration frame capture (Default)",
    )
    # Other args
    parser.add_argument(
        "--charuco_board_name",
        type=str,
        default="BOARD_5x7_37mm_27mm_4x4_start_id_0",
        help=f"Name of the CharucoBoards enum to use. One of {[c.name for c in CharucoBoards]}",
    )
    parser.add_argument(
        "--calib_file",
        type=str,
        default=None,
        help="Path to lidar calibration YAML file. Also where the output is saved.",
    )
    parser.add_argument(
        "--expected_width",
        type=float,
        default=280.72 / 1000,  # 11.05 inches - reflective marker to reflective marker
        help="Expected width of the lidar-detected rectangle in meters.",
    )
    parser.add_argument(
        "--expected_height",
        type=float,
        default=208.33 / 1000,  # 8.2 inches - reflective marker to reflective marker
        help="Expected height of the lidar-detected rectangle in meters.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=15 / 1000,
        help="Tolerance for the lidar rectangle size validation.",
    )

    parser.add_argument(
        "--skip_user_prompt",
        action="store_true",
        help="Skip user prompt before automatic robot movements",
    )
    parser.add_argument(
        "--not_interactive",
        action="store_true",
        help="Do not open rerun visualization windows",
    )
    parser.add_argument(
        "--use_ros_for_lidars",
        action="store_true",
        help="Use ros2 to subscribe to lidar points. (Default: False)",
    )
    parser.add_argument(
        "--replay_from_folder",
        type=str,
        default=None,
        help="Replay from a specific timestamp folder inside calibration_lidar_points",
    )
    parser.add_argument(
        "--replay_last",
        action="store_true",
        help="Replay from the last recorded timestamp folder inside calibration_lidar_points",
    )

    args, _ = parser.parse_known_args()

    manager = CalibrateLidarToCamera(
        camera=args.camera,
        lidar=args.lidar,
        use_ros_for_lidars=args.use_ros_for_lidars,
        charuco_board_name=args.charuco_board_name,
        expected_width=args.expected_width,
        expected_height=args.expected_height,
        tolerance=args.tolerance,
        use_gamepad=args.gamepad,
        calib_file=args.calib_file,
        skip_user_prompt=args.skip_user_prompt,
        replay_from_folder=args.replay_from_folder,
        replay_last=args.replay_last
    )
    manager.run(is_interactive=not args.not_interactive)


def REx_calibrate_extrinsics_lidars(interactive: bool):
    import sys

    if not interactive:
        if "--not_interactive" not in sys.argv:
            sys.argv.append("--not_interactive")
        if "--skip_user_prompt" not in sys.argv:
            sys.argv.append("--skip_user_prompt")
    else:
        if "--not_interactive" in sys.argv:
            sys.argv.remove("--not_interactive")
        if "--skip_user_prompt" in sys.argv:
            sys.argv.remove("--skip_user_prompt")

    calibrate_extrinsics_camera_lidar()


if __name__ == "__main__":
    calibrate_extrinsics_camera_lidar()
