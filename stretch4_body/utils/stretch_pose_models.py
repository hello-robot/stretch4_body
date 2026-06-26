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
                pose.joints[normalized_key] = JointPose(**v)
        return pose


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
            if self.value == 'parallel_gripper':
                return ['finger_left_joint', 'finger_right_joint']
            elif self.value == 'stretch_gripper':
                return ['gripper_finger_left_joint', 'gripper_finger_right_joint']
            else:
                return []
        return []

    @property
    def finger_links(self):
        if self.name == 'gripper':
            if self.value == 'parallel_gripper':
                return ['finger_left_link', 'finger_right_link']
            elif self.value == 'stretch_gripper':
                return ['gripper_finger_left_link', 'gripper_finger_right_link']
            else:
                return []
        return []

    def to_subsystem_units(self, position):
        if self.name == 'gripper':
            if self.value == 'parallel_gripper':
                return position
            elif self.value == 'stretch_gripper':
                _, robot_params = RobotParams.get_params()
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
        return None
