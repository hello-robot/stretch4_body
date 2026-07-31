#!/usr/bin/env python3

from dataclasses import dataclass, asdict, field
from enum import Enum, auto
from functools import cache
from typing import Dict, Optional
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.core.gamepad_enums import MotionProfile
from stretch4_body.core.hello_utils import deg_to_rad

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

    @property
    def value(self):
        if self.name == 'gripper':
            return self.get_gripper()
        return self.name

    @property
    def finger_joints(self):
        if self.name == 'gripper':
            if self.value == 'parallel_gripper' or (self.value and ('parallel' in self.value or 'jaw' in self.value)):
                return ['finger_left_joint', 'finger_right_joint']
            elif self.value == 'stretch_gripper':
                return ['gripper_finger_left_joint', 'gripper_finger_right_joint']
            else:
                return []
        return []

    @property
    def finger_links(self):
        if self.name == 'gripper':
            if self.value == 'parallel_gripper' or (self.value and ('parallel' in self.value or 'jaw' in self.value)):
                return ['finger_left_link', 'finger_right_link']
            elif self.value == 'stretch_gripper':
                return ['gripper_finger_left_link', 'gripper_finger_right_link']
            else:
                return []
        return []

    def to_subsystem_units(self, position):
        if self.name == 'gripper':
            from stretch4_body.core.robot_params import RobotParams
            _, robot_params = RobotParams.get_params()
            tool_name = robot_params.get('robot', {}).get('tool')
            if tool_name and RobotParams.is_user_defined_tool(tool_name):
                try:
                    import re
                    sanitized = re.sub(r'[^a-zA-Z0-9_]', '_', tool_name)
                    if sanitized and sanitized[0].isdigit():
                        sanitized = "_" + sanitized
                    mod = RobotParams.import_user_tool_module(tool_name, 'gripper_conversion', is_server=True)
                    conv_func = getattr(mod, f"{sanitized}_urdf_to_subsystem", None)
                    if conv_func:
                        return conv_func(position, robot_params.get(tool_name, {}))
                except Exception:
                    pass

            if self.value == 'parallel_gripper' or (self.value and ('parallel' in self.value or 'jaw' in self.value)):
                return position
            elif self.value == 'stretch_gripper':
                sg_params = robot_params.get('stretch_gripper', {})
                range_deg_0 = sg_params.get('range_deg', [-100.0, 0.0])[0]
                return -100.0 * position / deg_to_rad(range_deg_0)
        return position

    @classmethod
    def get_joint_by_name(cls, name):
        if name in ('stretch_gripper', 'parallel_gripper', 'gripper'):
            return cls.gripper
        if name in cls.__members__:
            return cls[name]
        return None

    @staticmethod
    def get_end_of_arm_joints():
        joints = [RobotJoints.wrist_pitch, RobotJoints.wrist_roll, RobotJoints.wrist_yaw]
        if RobotJoints.gripper.value is not None:
            joints.append(RobotJoints.gripper)
        return joints
    
    @cache
    def get_joint_params(self, profile: MotionProfile):
        _, robot_params = RobotParams.get_params()
        params = robot_params[self.value]
        joint_params = params['motion'][profile.get_name()]
        v = joint_params['vel'] if 'vel' in joint_params else joint_params['vel_m']
        a = joint_params['accel'] if 'accel' in joint_params else joint_params['accel_m']
        return v, a
    
    @cache
    def get_base_params(self, profile: MotionProfile):
        params = RobotParams().get_params()[1]['omnibase']
        base_params = params['motion'][profile.get_name()]
        accel_w_r = base_params['accel_w_r']
        vel_w_r = base_params['vel_w_r']
        accel_xy_m = base_params['accel_xy_m']
        vel_xy_m = base_params['vel_xy_m']
        return vel_xy_m, accel_xy_m, vel_w_r, accel_w_r

    @cache
    def get_gripper(self):
        _, robot_params = RobotParams.get_params()
        if 'stretch_gripper' in robot_params:
            return 'stretch_gripper'
        elif 'parallel_gripper' in robot_params:
            return 'parallel_gripper'
        
        # Check if the active tool is a custom tool with a gripper device
        tool_name = robot_params.get('robot', {}).get('tool')
        if tool_name and tool_name in robot_params:
            tool_params = robot_params[tool_name]
            for d_name, d_params in tool_params.get('devices', {}).items():
                py_class = d_params.get('py_class_name', '')
                if 'gripper' in d_name.lower() or 'jaw' in d_name.lower() or 'gripper' in py_class.lower() or 'jaw' in py_class.lower():
                    return d_name
        return None
