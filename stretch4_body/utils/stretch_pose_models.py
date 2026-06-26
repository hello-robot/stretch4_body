#!/usr/bin/env python3

from dataclasses import asdict, dataclass, field
from enum import Enum, auto
from functools import cache, cached_property
from typing import Dict, List, Optional

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
    base: Optional[BasePose] = None
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
    def get_joint_by_name(cls, name: str) -> Optional['RobotJoints']:
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
    def value(self) -> str:
        if self.name == 'gripper':
            return self.gripper_name
        else:
            return self.name

    @property
    def gripper_model(self) -> Optional[GripperMetadata]:
        if self.name == 'gripper':
            return GRIPPER_MODELS.get(self.value)
        return None

    @property
    def finger_joints(self) -> List[str]:
        model = self.gripper_model
        return model.finger_joints if model else []

    @property
    def arm_joints(self) -> List[str]:
        if self.name == 'arm':
            return ['arm_l1_joint', 'arm_l2_joint', 'arm_l3_joint', 'arm_l4_joint']
        return []

    @property
    def finger_links(self) -> List[str]:
        model = self.gripper_model
        return model.finger_links if model else []

    @property
    def gripper_client(self) -> Optional[ParallelGripperClient | StretchGripperClient]:
        model = self.gripper_model
        return model.client_class() if model else None

    @cached_property
    def gripper_name(self) -> Optional[str]:
        _, robot_params = RobotParams.get_params()
        for name in GRIPPER_MODELS:
            if name in robot_params:
                return name
        return None

    def to_subsystem_units(self, position: float) -> float:
        return self.urdf_to_subsystem(position)

    def to_standard_units(self, position: float) -> float:
        return self.subsystem_to_urdf(position)

    def urdf_to_subsystem(self, urdf: float) -> float:
        model = self.gripper_model
        return model.urdf_to_subsystem(urdf) if model else urdf

    def subsystem_to_urdf(self, subsystem: float) -> float:
        model = self.gripper_model
        return model.subsystem_to_urdf(subsystem) if model else subsystem

    @property
    def poses(self) -> Optional[Dict[str, float]]:
        if self.name == 'gripper' and self.gripper_model:
            client = self.gripper_client
            if client and hasattr(client, 'poses'):
                return {k: self.subsystem_to_urdf(v) for k, v in client.poses.items()}
        return None

    @property
    def subsystem_range(self) -> Optional[tuple[float, float]]:
        model = self.gripper_model
        return model.subsystem_range if model else None

    @property
    def urdf_range(self) -> Optional[tuple[float, float]]:
        model = self.gripper_model
        return model.urdf_range if model else None

    @property
    def aperture_range_m(self) -> Optional[tuple[float, float]]:
        model = self.gripper_model
        return model.aperture_range_m if model else None

    def normalized_to_subsystem(self, normalized: float) -> Optional[float]:
        model = self.gripper_model
        return model.normalized_to_subsystem(normalized) if model else None

    def subsystem_to_normalized(self, subsystem: float) -> Optional[float]:
        model = self.gripper_model
        return model.subsystem_to_normalized(subsystem) if model else None

    def urdf_to_normalized(self, urdf: float) -> Optional[float]:
        model = self.gripper_model
        return model.urdf_to_normalized(urdf) if model else None

    def normalized_to_urdf(self, normalized: float) -> Optional[float]:
        model = self.gripper_model
        return model.normalized_to_urdf(normalized) if model else None

    def aperture_to_normalized(self, aperture_m: float) -> Optional[float]:
        model = self.gripper_model
        return model.aperture_to_normalized(aperture_m) if model else None

    def normalized_to_aperture(self, normalized: float) -> Optional[float]:
        model = self.gripper_model
        return model.normalized_to_aperture(normalized) if model else None

    def aperture_to_subsystem(self, aperture_m: float) -> Optional[float]:
        model = self.gripper_model
        return model.aperture_to_subsystem(aperture_m) if model else None

    def subsystem_to_aperture(self, subsystem: float) -> Optional[float]:
        model = self.gripper_model
        return model.subsystem_to_aperture(subsystem) if model else None

    def urdf_to_aperture(self, urdf: float) -> Optional[float]:
        model = self.gripper_model
        return model.urdf_to_aperture(urdf) if model else None

    def aperture_to_urdf(self, aperture_m: float) -> Optional[float]:
        model = self.gripper_model
        return model.aperture_to_urdf(aperture_m) if model else None

    @cache
    def get_joint_params(self, profile: MotionProfile) -> tuple[float, float]:
        _, robot_params = RobotParams.get_params()
        params = robot_params[self.value]
        joint_params = params['motion'][profile.get_name()]
        v = joint_params['vel'] if 'vel' in joint_params else joint_params['vel_m']
        a = joint_params['accel'] if 'accel' in joint_params else joint_params['accel_m']
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
