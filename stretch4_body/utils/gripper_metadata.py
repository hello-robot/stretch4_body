#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import List, Type
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.core.hello_utils import deg_to_rad
from stretch4_body.robot.robot_client import ParallelGripperClient, StretchGripperClient


class GripperMetadata(ABC):
    @property
    @abstractmethod
    def finger_joints(self) -> List[str]:
        pass

    @property
    @abstractmethod
    def finger_links(self) -> List[str]:
        pass

    @property
    @abstractmethod
    def client_class(self) -> Type:
        pass

    @abstractmethod
    def to_subsystem_units(self, position: float) -> float:
        # Converts move_to and move_by commanded positions to subsystem command units
        pass


class ParallelGripperMetadata(GripperMetadata):
    @property
    def finger_joints(self) -> List[str]:
        return ['finger_left_joint', 'finger_right_joint']

    @property
    def finger_links(self) -> List[str]:
        return ['finger_left_link', 'finger_right_link']

    @property
    def client_class(self) -> Type:
        return ParallelGripperClient

    def to_subsystem_units(self, position: float) -> float:
        # Commanded position and subsystem units are both the finger gap width in meters
        return position


class StretchGripperMetadata(GripperMetadata):
    @property
    def finger_joints(self) -> List[str]:
        return ['gripper_finger_left_joint', 'gripper_finger_right_joint']

    @property
    def finger_links(self) -> List[str]:
        return ['gripper_finger_left_link', 'gripper_finger_right_link']

    @property
    def client_class(self) -> Type:
        return StretchGripperClient

    def to_subsystem_units(self, position: float) -> float:
        # Commanded position is aperture radians, and subsystem units are a range percentage
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get('stretch_gripper', {})
        range_deg_0 = sg_params.get('range_deg', [-100.0, 0.0])[0]
        return -100.0 * position / deg_to_rad(range_deg_0)


GRIPPER_MODELS = {
    'parallel_gripper': ParallelGripperMetadata(),
    'stretch_gripper': StretchGripperMetadata(),
}
