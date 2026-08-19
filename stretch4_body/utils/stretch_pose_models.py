#!/usr/bin/env python3

from dataclasses import asdict, dataclass, field
from enum import Enum, auto
from functools import cache, cached_property
from typing import Dict, List

from stretch4_body.core.gamepad_enums import MotionProfile
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.robot.robot_client import ParallelGripperClient, StretchGripperClient
from stretch4_body.utils.gripper_metadata import GRIPPER_MODELS, GripperMetadata


@dataclass
class JointPose:
    name: str
    position: float
    velocity: float
    effort: float


@dataclass
class BasePose:
    x: float
    y: float
    theta: float


@dataclass
class RobotPose:
    name: str
    timestamp: float
    joints: Dict[str, JointPose] = field(default_factory=dict)
    base: BasePose | None = None
    delay_before_start: float = 0.0

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        pose = cls(name=data['name'], timestamp=data['timestamp'])
        if 'delay_before_start' in data:
            pose.delay_before_start = data['delay_before_start']
        if 'base' in data and data['base']:
            pose.base = BasePose(**data['base'])
        if 'joints' in data:
            for k, v in data['joints'].items():
                joint = RobotJoints.get_joint_by_name(k)
                normalized_key = joint.name if joint is not None else k
                v_copy = dict(v)
                v_copy.setdefault('name', normalized_key)
                pose.joints[normalized_key] = JointPose(**v_copy)
        return pose

    @classmethod
    def load_tool_pose_models(cls, tool_name=None) -> Dict[str, 'RobotPose']:
        """
        Dynamically load pre-defined pose models from the custom tool directory.
        """
        import os

        import yaml

        from stretch4_body.core.robot_params import RobotParams
        
        if tool_name is None:
            _, robot_params = RobotParams.get_params()
            tool_name = robot_params.get('robot', {}).get('tool')
            
        if not tool_name or not RobotParams.is_user_defined_tool(tool_name):
            return {}
            
        tool_path = RobotParams.get_user_defined_tool_path(tool_name)
        if not tool_path:
            return {}
            
        pose_yaml_path = os.path.join(tool_path, 'pose_models.yaml')
        if not os.path.exists(pose_yaml_path):
            return {}
            
        try:
            with open(pose_yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            poses = {}
            for p_dict in data:
                p = cls.from_dict(p_dict)
                poses[p.name] = p
            return poses
        except Exception as e:
            print(f"Warning: Failed to load pose models from {pose_yaml_path}: {e}")
            return {}


class RobotJoints(Enum):
    base = auto()
    lift = auto()
    arm = auto()
    wrist_yaw = auto()
    wrist_pitch = auto()
    wrist_roll = auto()
    gripper = auto()


    @classmethod
    def get_joint_by_name(cls, name: str) -> 'RobotJoints | None':
        if name in cls.__members__:
            return cls[name]
        for joint in cls:
            if joint.value == name:
                return joint
        return None

    @classmethod
    def get_end_of_arm_joints(cls) -> List['RobotJoints']:
        joints = [cls.wrist_pitch, cls.wrist_roll, cls.wrist_yaw]
        if cls.gripper.value is not None:
            joints.append(cls.gripper)
        return joints

    @property
    def value(self) -> str | None:
        if self.name == 'gripper':
            return self.gripper_name
        else:
            return self.name

    @property
    def gripper_model(self) -> GripperMetadata | None:
        if self.name == 'gripper' and self.value is not None:
            return GRIPPER_MODELS.get(self.value)
        return None

    @property
    def finger_joints(self) -> List[str]:
        model = self.gripper_model
        return model.finger_joints if model else []

    @property
    def finger_links(self) -> List[str]:
        model = self.gripper_model
        return model.finger_links if model else []

    @property
    def gripper_client(self) -> ParallelGripperClient | StretchGripperClient | None:
        model = self.gripper_model
        return model.client_class() if model else None

    @cached_property
    def gripper_name(self) -> str | None:
        _, robot_params = RobotParams.get_params()
        for name in GRIPPER_MODELS:
            if name in robot_params:
                return name
        return None

    @property
    def poses(self) -> Dict[str, float] | None:
        if self.name == 'gripper' and self.gripper_model:
            client = self.gripper_client
            if client and hasattr(client, 'poses'):
                return {k: self.subsystem_to_urdf(v) for k, v in client.poses.items()}
        return None

    @property
    def subsystem_range(self) -> tuple[float, float] | None:
        model = self.gripper_model
        return model.subsystem_range if model else None

    @property
    def urdf_range(self) -> tuple[float, float] | None:
        model = self.gripper_model
        return model.urdf_range if model else None

    @property
    def aperture_range_m(self) -> tuple[float, float] | None:
        model = self.gripper_model
        return model.aperture_range_m if model else None


    def to_subsystem_units(self, position: float) -> float:
        """Alias for `urdf_to_subsystem`, kept for non-gripper joints where "subsystem"/"standard" are the more familiar terms."""
        return self.urdf_to_subsystem(position)

    def to_standard_units(self, position: float) -> float:
        """Alias for `subsystem_to_urdf`, kept for non-gripper joints where "subsystem"/"standard" are the more familiar terms."""
        return self.subsystem_to_urdf(position)

    def urdf_to_subsystem(self, urdf: float) -> float:
        """Converts URDF units (radians/meters) to subsystem units (percentage/meters); unchanged if no gripper model."""
        model = self.gripper_model
        return model.urdf_to_subsystem(urdf) if model else urdf

    def subsystem_to_urdf(self, subsystem: float) -> float:
        """Converts subsystem units (percentage/meters) to URDF units (radians/meters); unchanged if no gripper model."""
        model = self.gripper_model
        return model.subsystem_to_urdf(subsystem) if model else subsystem

    def normalized_to_subsystem(self, normalized: float) -> float | None:
        """Converts a normalized scale (0.0=closed, 1.0=open) to subsystem units, or None if no gripper model."""
        model = self.gripper_model
        return model.normalized_to_subsystem(normalized) if model else None

    def subsystem_to_normalized(self, subsystem: float) -> float | None:
        """Converts subsystem units to a normalized scale (0.0=closed, 1.0=open), or None if no gripper model."""
        model = self.gripper_model
        return model.subsystem_to_normalized(subsystem) if model else None

    def urdf_to_normalized(self, urdf: float) -> float | None:
        """Converts URDF units to a normalized scale (0.0=closed, 1.0=open), or None if no gripper model."""
        model = self.gripper_model
        return model.urdf_to_normalized(urdf) if model else None

    def normalized_to_urdf(self, normalized: float) -> float | None:
        """Converts a normalized scale (0.0=closed, 1.0=open) to URDF units, or None if no gripper model."""
        model = self.gripper_model
        return model.normalized_to_urdf(normalized) if model else None

    def aperture_to_normalized(self, aperture_m: float) -> float | None:
        """Converts fingertip aperture (meters) to a normalized scale (0.0=closed, 1.0=open), or None if no gripper model."""
        model = self.gripper_model
        return model.aperture_to_normalized(aperture_m) if model else None

    def normalized_to_aperture(self, normalized: float) -> float | None:
        """Converts a normalized scale (0.0=closed, 1.0=open) to fingertip aperture (meters), or None if no gripper model."""
        model = self.gripper_model
        return model.normalized_to_aperture(normalized) if model else None

    def aperture_to_subsystem(self, aperture_m: float) -> float | None:
        """Converts fingertip aperture (meters) to subsystem units, or None if no gripper model."""
        model = self.gripper_model
        return model.aperture_to_subsystem(aperture_m) if model else None

    def subsystem_to_aperture(self, subsystem: float) -> float | None:
        """Converts subsystem units to fingertip aperture (meters), or None if no gripper model."""
        model = self.gripper_model
        return model.subsystem_to_aperture(subsystem) if model else None

    def urdf_to_aperture(self, urdf: float) -> float | None:
        """Converts URDF units to fingertip aperture (meters), or None if no gripper model."""
        model = self.gripper_model
        return model.urdf_to_aperture(urdf) if model else None

    def aperture_to_urdf(self, aperture_m: float) -> float | None:
        """Converts fingertip aperture (meters) to URDF units, or None if no gripper model."""
        model = self.gripper_model
        return model.aperture_to_urdf(aperture_m) if model else None

    @cache
    def get_joint_params(self, profile: MotionProfile) -> tuple[float, float]:
        if self.value is None:
            raise ValueError(f"{self.name} has no joint/device configured in robot params (e.g. no gripper attached).")

        _, robot_params = RobotParams.get_params()
        params = robot_params.get(self.value)
        if params is None:
            raise ValueError(f"No robot params found for joint '{self.value}'.")

        motion_params = params.get('motion')
        if motion_params is None:
            raise ValueError(f"Robot params for joint '{self.value}' are missing a 'motion' section.")

        profile_name = profile.get_name()
        joint_params = motion_params.get(profile_name)
        if joint_params is None:
            raise ValueError(f"Joint '{self.value}' has no '{profile_name}' motion profile defined.")

        v = joint_params.get('vel', joint_params.get('vel_m'))
        a = joint_params.get('accel', joint_params.get('accel_m'))
        if v is None or a is None:
            raise ValueError(
                f"Motion profile '{profile_name}' for joint '{self.value}' is missing "
                f"'vel'/'vel_m' or 'accel'/'accel_m' keys."
            )
        return v, a

    @cache
    def get_base_params(self, profile: MotionProfile) -> tuple[float, float, float, float]:
        params = RobotParams().get_params()[1]['omnibase']
        base_params = params['motion'][profile.get_name()]
        accel_w_r = base_params['accel_w_r']
        vel_w_r = base_params['vel_w_r']
        accel_xy_m = base_params['accel_xy_m']
        vel_xy_m = base_params['vel_xy_m']
        return vel_xy_m, accel_xy_m, vel_w_r, accel_w_r

