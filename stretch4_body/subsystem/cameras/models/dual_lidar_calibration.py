import logging

logger = logging.getLogger(__name__)
import os
import yaml
import datetime
import numpy as np
from pathlib import Path

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
            logger.info(f"Backed up {self.filepath} to {backup_path}")

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
        from stretch4_body.utils.stretch_pose_models import RobotJoints # to keep original dependencies
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
            logger.info(f"Failed to load URDF: {e}")
            return np.eye(4)

        link_to_parent = {}
        for joint in robot.robot.joints:
            link_to_parent[joint.child] = (joint.parent, joint.origin)

        current = lidar_link
        chain = []
        while current != "base_link":
            if current not in link_to_parent:
                logger.info(f"Lidar link {lidar_link} not connected to base_link")
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
