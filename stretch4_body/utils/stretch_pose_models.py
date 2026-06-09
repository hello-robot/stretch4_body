#!/usr/bin/env python3

from dataclasses import dataclass, asdict, field
from enum import Enum, auto
from functools import cache
from typing import List, Dict, Optional
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.core.gamepad_enums import MotionProfile

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
                pose.joints[k] = JointPose(**v)
        return pose


class RobotJoints(Enum):
    base = auto()
    lift = auto()
    arm = auto()
    wrist_yaw = auto()
    wrist_pitch = auto()
    wrist_roll = auto()
    stretch_gripper = auto()

    @staticmethod
    def get_end_of_arm_joints():
        return [RobotJoints.wrist_pitch, RobotJoints.wrist_roll, RobotJoints.wrist_yaw, RobotJoints.stretch_gripper]
    
    @cache
    def get_joint_params(self, profile: MotionProfile):
        params = RobotParams().get_params()[1][self.name]
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
