#!/usr/bin/env python3

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Type

from stretch4_body.core.hello_utils import deg_to_rad
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.robot.robot_client import ParallelGripperClient, StretchGripperClient


class GripperMetadata(ABC):
    @property
    @abstractmethod
    def finger_joints(self) -> List[str]:
        """List of finger joint names in URDF"""
        pass

    @property
    @abstractmethod
    def finger_links(self) -> List[str]:
        """List of finger link names in URDF"""
        pass

    @property
    @abstractmethod
    def client_class(self) -> Type:
        """RobotClient subclass for controlling this gripper"""
        pass

    @property
    @abstractmethod
    def subsystem_range(self) -> tuple[float, float]:
        """(close_val, open_val) in native subsystem/hardware units expected by the RobotClient (percentage for SG4, meters for PG4)"""
        pass

    @property
    def urdf_range(self) -> tuple[float, float]:
        """(close_val, open_val) in URDF/ROS coordinate units (radians for revolute SG4, meters for prismatic PG4)"""
        low, high = self.subsystem_range
        return self.subsystem_to_urdf(low), self.subsystem_to_urdf(high)

    @property
    def aperture_range_m(self) -> tuple[float, float]:
        """(min_aperture, max_aperture) in physical meters representing fingertip separation distance"""
        low, high = self.subsystem_range
        return self.subsystem_to_aperture(low), self.subsystem_to_aperture(high)

    # --- Abstract Base Layer Conversions ---
    @abstractmethod
    def urdf_to_subsystem(self, urdf: float) -> float:
        """Converts from URDF units (radians or meters) to native subsystem units expected by the RobotClient (percentage or meters)"""
        pass

    @abstractmethod
    def subsystem_to_urdf(self, subsystem: float) -> float:
        """Converts from native subsystem units expected by the RobotClient (percentage or meters) to URDF units (radians or meters)"""
        pass

    @abstractmethod
    def aperture_to_subsystem(self, aperture_m: float) -> float:
        """Converts from physical fingertip aperture distance in meters to native subsystem units expected by the RobotClient"""
        pass

    @abstractmethod
    def subsystem_to_aperture(self, subsystem: float) -> float:
        """Converts from native subsystem units to physical fingertip aperture distance in meters"""
        pass

    # --- Chained Layer Conversions ---
    
    # Normalized <-> Subsystem
    def normalized_to_subsystem(self, normalized: float) -> float:
        """Converts a normalized scale value (0.0=closed, 1.0=open) to native subsystem units"""
        low, high = self.subsystem_range
        return low + normalized * (high - low)

    def subsystem_to_normalized(self, subsystem: float) -> float:
        """Converts native subsystem units to a normalized scale value (0.0=closed, 1.0=open)"""
        low, high = self.subsystem_range
        return (subsystem - low) / (high - low)

    # URDF <-> Normalized
    def urdf_to_normalized(self, urdf: float) -> float:
        """Converts URDF units (radians or meters) to a normalized scale value (0.0=closed, 1.0=open)"""
        sub = self.urdf_to_subsystem(urdf)
        return self.subsystem_to_normalized(sub)

    def normalized_to_urdf(self, normalized: float) -> float:
        """Converts a normalized scale value (0.0=closed, 1.0=open) to URDF units (radians or meters)"""
        sub = self.normalized_to_subsystem(normalized)
        return self.subsystem_to_urdf(sub)

    # Aperture <-> Normalized
    def aperture_to_normalized(self, aperture_m: float) -> float:
        """Converts physical fingertip aperture distance in meters to a normalized scale value (0.0=closed, 1.0=open)"""
        sub = self.aperture_to_subsystem(aperture_m)
        return self.subsystem_to_normalized(sub)

    def normalized_to_aperture(self, normalized: float) -> float:
        """Converts a normalized scale value (0.0=closed, 1.0=open) to physical fingertip aperture distance in meters"""
        sub = self.normalized_to_subsystem(normalized)
        return self.subsystem_to_aperture(sub)

    # URDF <-> Aperture
    def urdf_to_aperture(self, urdf: float) -> float:
        """Converts URDF units (radians or meters) to physical fingertip aperture distance in meters"""
        sub = self.urdf_to_subsystem(urdf)
        return self.subsystem_to_aperture(sub)

    def aperture_to_urdf(self, aperture_m: float) -> float:
        """Converts physical fingertip aperture distance in meters to URDF units (radians or meters)"""
        sub = self.aperture_to_subsystem(aperture_m)
        return self.subsystem_to_urdf(sub)


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

    @property
    def subsystem_range(self) -> tuple[float, float]:
        client = self.client_class()
        return client.poses['close'], client.poses['open']

    def urdf_to_subsystem(self, urdf: float) -> float:
        return urdf

    def subsystem_to_urdf(self, subsystem: float) -> float:
        return subsystem

    def aperture_to_subsystem(self, aperture_m: float) -> float:
        return aperture_m

    def subsystem_to_aperture(self, subsystem: float) -> float:
        return subsystem


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

    @property
    def subsystem_range(self) -> tuple[float, float]:
        client = self.client_class()
        return client.poses['close'], client.poses['open']

    def urdf_to_subsystem(self, urdf: float) -> float:
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get('stretch_gripper', {})
        range_deg_0 = sg_params.get('range_deg', [-100.0, 0.0])[0]
        return -100.0 * urdf / deg_to_rad(range_deg_0)

    def subsystem_to_urdf(self, subsystem: float) -> float:
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get('stretch_gripper', {})
        range_deg_0 = sg_params.get('range_deg', [-100.0, 0.0])[0]
        return subsystem * deg_to_rad(range_deg_0) / -100.0

    def aperture_to_subsystem(self, aperture_m: float) -> float:
        client = self.client_class()
        return client.gripper_conversion.aperture_to_servo(aperture_m)

    def subsystem_to_aperture(self, subsystem: float) -> float:
        client = self.client_class()
        return client.gripper_conversion.servo_to_aperture(subsystem)


GRIPPER_MODELS = {
    'parallel_gripper': ParallelGripperMetadata(),
    'stretch_gripper': StretchGripperMetadata(),
}
