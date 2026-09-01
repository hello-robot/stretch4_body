#!/usr/bin/env python3

from abc import ABC, abstractmethod

from stretch4_body.core.hello_utils import deg_to_rad
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.robot.robot_client import ParallelGripperClient, StretchGripperClient
from stretch4_body.utils.user_tool_utils import add_user_tool_to_sys_path


class ToolConfigurationError(ValueError):
    """Raised when an end-of-arm tool parameter or metadata configuration is missing or invalid."""


class ToolMetadata(ABC):
    """
    Abstract base class defining kinematic, hardware command, and physical unit conversions
    for Stretch 4 end-of-arm tools and grippers.
    """

    @property
    def joint_name(self) -> str:
        """Name of the joint/device in robot_params used for motion params lookup."""
        return self.primary_joint

    @property
    @abstractmethod
    def tool_joints(self) -> list[str]:
        """List of all URDF joint names controlled by this tool (for ROS JointState publishing)."""

    @property
    def finger_joints(self) -> list[str]:
        """Alias for tool_joints for backward compatibility."""
        return self.tool_joints

    @property
    def actuated_joints(self) -> list[str]:
        """List of URDF joints backed by physical motors (excluding mimic joints). Defaults to [primary_joint]."""
        return [self.primary_joint]

    @property
    def primary_joint(self) -> str:
        """Designated primary joint mapped to single-axis teleop triggers/sliders. Defaults to first tool joint."""
        joints = self.tool_joints
        if not joints:
            raise ToolConfigurationError("tool_joints list is empty.")
        return joints[0]

    @property
    @abstractmethod
    def tool_links(self) -> list[str]:
        """List of visual/collision URDF link names comprising the tool."""

    @property
    def finger_links(self) -> list[str]:
        """Alias for tool_links for backward compatibility."""
        return self.tool_links

    @property
    @abstractmethod
    def client_class(self) -> type:
        """RobotClient subclass for controlling this tool."""

    @property
    @abstractmethod
    def actuator_command_range(self) -> tuple[float, float]:
        """(min_val, max_val) bounds in raw actuator/hardware command units (e.g. % for SG4, meters for PG4)."""

    @property
    def subsystem_range(self) -> tuple[float, float]:
        """Alias for actuator_command_range for backward compatibility."""
        return self.actuator_command_range

    @property
    def urdf_range(self) -> tuple[float, float]:
        """(min_val, max_val) bounds in URDF/ROS coordinate units (radians or meters)."""
        low, high = self.actuator_command_range
        return self.actuator_to_urdf(low), self.actuator_to_urdf(high)

    @property
    def aperture_range(self) -> tuple[float, float]:
        """(min_aperture, max_aperture) physical opening bounds (meters or angle)."""
        low, high = self.actuator_command_range
        return self.actuator_to_aperture(low), self.actuator_to_aperture(high)

    @property
    def aperture_range_m(self) -> tuple[float, float]:
        """Alias for aperture_range for backward compatibility."""
        return self.aperture_range

    # --- Abstract Base Conversions ---

    @abstractmethod
    def urdf_to_actuator(self, urdf: float) -> float:
        """Converts from URDF units (radians/meters) to native actuator command units."""

    def urdf_to_subsystem(self, urdf: float) -> float:
        """Alias for urdf_to_actuator."""
        return self.urdf_to_actuator(urdf)

    @abstractmethod
    def actuator_to_urdf(self, actuator: float) -> float:
        """Converts from native actuator command units to URDF units (radians/meters)."""

    def subsystem_to_urdf(self, subsystem: float) -> float:
        """Alias for actuator_to_urdf."""
        return self.actuator_to_urdf(subsystem)

    @abstractmethod
    def aperture_to_actuator(self, aperture: float) -> float:
        """Converts from physical opening aperture to native actuator command units."""

    def aperture_to_subsystem(self, aperture_m: float) -> float:
        """Alias for aperture_to_actuator."""
        return self.aperture_to_actuator(aperture_m)

    @abstractmethod
    def actuator_to_aperture(self, actuator: float) -> float:
        """Converts from native actuator command units to physical opening aperture."""

    def subsystem_to_aperture(self, subsystem: float) -> float:
        """Alias for actuator_to_aperture."""
        return self.actuator_to_aperture(subsystem)

    # --- Normalized <-> Actuator Conversions ---

    def normalized_to_actuator(self, normalized: float) -> float:
        """Converts a normalized scale value (0.0=closed/min, 1.0=open/max) to native actuator units."""
        low, high = self.actuator_command_range
        return low + normalized * (high - low)

    def normalized_to_subsystem(self, normalized: float) -> float:
        """Alias for normalized_to_actuator."""
        return self.normalized_to_actuator(normalized)

    def actuator_to_normalized(self, actuator: float) -> float:
        """Converts native actuator units to a normalized scale value (0.0=closed/min, 1.0=open/max)."""
        low, high = self.actuator_command_range
        if high == low:
            return 0.0
        return (actuator - low) / (high - low)

    def subsystem_to_normalized(self, subsystem: float) -> float:
        """Alias for actuator_to_normalized."""
        return self.actuator_to_normalized(subsystem)

    # --- Chained Layer Conversions ---

    def urdf_to_normalized(self, urdf: float) -> float:
        act = self.urdf_to_actuator(urdf)
        return self.actuator_to_normalized(act)

    def normalized_to_urdf(self, normalized: float) -> float:
        act = self.normalized_to_actuator(normalized)
        return self.actuator_to_urdf(act)

    def aperture_to_normalized(self, aperture: float) -> float:
        act = self.aperture_to_actuator(aperture)
        return self.actuator_to_normalized(act)

    def normalized_to_aperture(self, normalized: float) -> float:
        act = self.normalized_to_actuator(normalized)
        return self.actuator_to_aperture(act)

    def urdf_to_aperture(self, urdf: float) -> float:
        act = self.urdf_to_actuator(urdf)
        return self.actuator_to_aperture(act)

    def aperture_to_urdf(self, aperture: float) -> float:
        act = self.aperture_to_actuator(aperture)
        return self.actuator_to_urdf(act)


class ParallelGripperMetadata(ToolMetadata):
    @property
    def joint_name(self) -> str:
        return 'parallel_gripper'

    @property
    def tool_joints(self) -> list[str]:
        return ['finger_left_joint', 'finger_right_joint']

    @property
    def primary_joint(self) -> str:
        return 'finger_left_joint'

    @property
    def tool_links(self) -> list[str]:
        return ['finger_left_link', 'finger_right_link']

    @property
    def client_class(self) -> type:
        return ParallelGripperClient

    @property
    def actuator_command_range(self) -> tuple[float, float]:
        client = self.client_class()
        return client.poses['close'], client.poses['open']

    def urdf_to_actuator(self, urdf: float) -> float:
        return urdf

    def actuator_to_urdf(self, actuator: float) -> float:
        return actuator

    def aperture_to_actuator(self, aperture: float) -> float:
        return aperture

    def actuator_to_aperture(self, actuator: float) -> float:
        return actuator


class StretchGripperMetadata(ToolMetadata):
    @property
    def joint_name(self) -> str:
        return 'stretch_gripper'

    @property
    def tool_joints(self) -> list[str]:
        return ['gripper_finger_left_joint', 'gripper_finger_right_joint']

    @property
    def primary_joint(self) -> str:
        return 'gripper_finger_left_joint'

    @property
    def tool_links(self) -> list[str]:
        return ['gripper_finger_left_link', 'gripper_finger_right_link']

    @property
    def client_class(self) -> type:
        return StretchGripperClient

    @property
    def actuator_command_range(self) -> tuple[float, float]:
        client = self.client_class()
        return client.poses['close'], client.poses['open']

    def urdf_to_actuator(self, urdf: float) -> float:
        from stretch4_body.core.robot_params import RobotParams
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get('stretch_gripper', {})
        range_deg_0 = sg_params.get('range_deg', [-100.0, 0.0])[0]
        return -100.0 * urdf / deg_to_rad(range_deg_0)

    def actuator_to_urdf(self, actuator: float) -> float:
        from stretch4_body.core.robot_params import RobotParams
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get('stretch_gripper', {})
        range_deg_0 = sg_params.get('range_deg', [-100.0, 0.0])[0]
        return actuator * deg_to_rad(range_deg_0) / -100.0

    def aperture_to_actuator(self, aperture: float) -> float:
        client = self.client_class()
        return client.gripper_conversion.aperture_to_servo(aperture)

    def actuator_to_aperture(self, actuator: float) -> float:
        client = self.client_class()
        return client.gripper_conversion.servo_to_aperture(actuator)


class UserToolMetadata(ToolMetadata):
    """
    Metadata representation for custom user tools loaded strictly from YAML parameters.
    Fails fast if any required configuration key is missing.
    """

    def __init__(self, tool_name: str):
        from stretch4_body.core.robot_params import RobotParams
        self.tool_name = tool_name
        _, self.robot_params = RobotParams.get_params()
        
        if tool_name not in self.robot_params:
            raise ToolConfigurationError(
                f"Tool '{tool_name}' not found in robot_params. Ensure it is registered in stretch_user_params.yaml."
            )
            
        self.tool_params = self.robot_params[tool_name]
        self._validate_and_load_parameters()

    def _validate_and_load_parameters(self) -> None:
        """Strictly validates all required YAML keys for user tools."""
        from stretch4_body.core.robot_params import RobotParams

        # 1. Joints and Links
        joints = self.tool_params.get('tool_joints', self.tool_params.get('finger_joints'))
        if not joints:
            raise ToolConfigurationError(
                f"Missing required key 'tool_joints' in robot_params['{self.tool_name}']."
            )
        self._tool_joints = list(joints)

        self._primary_joint = self.tool_params.get('primary_joint', self._tool_joints[0])

        links = self.tool_params.get('tool_links', self.tool_params.get('finger_links'))
        if not links:
            raise ToolConfigurationError(
                f"Missing required key 'tool_links' in robot_params['{self.tool_name}']."
            )
        self._tool_links = list(links)

        # 2. Client Classes
        client_module = self.tool_params.get('client_module_name')
        client_class_name = self.tool_params.get('client_class_name')
        if not client_module or not client_class_name:
            raise ToolConfigurationError(
                f"Missing required keys 'client_module_name' or 'client_class_name' in robot_params['{self.tool_name}']."
            )

        add_user_tool_to_sys_path(self.tool_name)
        try:
            module = RobotParams.import_user_tool_module(self.tool_name, client_module, is_server=False)
            self._client_class = getattr(module, client_class_name)
        except Exception as e:
            raise ToolConfigurationError(
                f"Failed to import client class '{client_class_name}' from module '{client_module}' "
                f"for user tool '{self.tool_name}': {e}"
            )

        # 3. Ranges
        act_range = self.tool_params.get('actuator_command_range', self.tool_params.get('subsystem_range'))
        if not act_range or len(act_range) != 2:
            raise ToolConfigurationError(
                f"Missing or invalid required key 'actuator_command_range' [min, max] in robot_params['{self.tool_name}']."
            )
        self._actuator_command_range = (float(act_range[0]), float(act_range[1]))

        ap_range = self.tool_params.get('aperture_range', self.tool_params.get('aperture_range_m'))
        if not ap_range or len(ap_range) != 2:
            raise ToolConfigurationError(
                f"Missing or invalid required key 'aperture_range' [min, max] in robot_params['{self.tool_name}']."
            )
        self._aperture_range = (float(ap_range[0]), float(ap_range[1]))

        self._urdf_scale = float(self.tool_params.get('urdf_to_actuator_scale', 1.0))

    @property
    def tool_joints(self) -> list[str]:
        return self._tool_joints

    @property
    def primary_joint(self) -> str:
        return self._primary_joint

    @property
    def tool_links(self) -> list[str]:
        return self._tool_links

    @property
    def client_class(self) -> type:
        return self._client_class

    @property
    def actuator_command_range(self) -> tuple[float, float]:
        return self._actuator_command_range

    @property
    def aperture_range(self) -> tuple[float, float]:
        return self._aperture_range

    def urdf_to_actuator(self, urdf: float) -> float:
        return urdf * self._urdf_scale

    def actuator_to_urdf(self, actuator: float) -> float:
        return actuator / self._urdf_scale if self._urdf_scale != 0 else actuator

    def aperture_to_actuator(self, aperture: float) -> float:
        ap_low, ap_high = self._aperture_range
        act_low, act_high = self._actuator_command_range
        if ap_high == ap_low:
            return act_low
        norm = (aperture - ap_low) / (ap_high - ap_low)
        return act_low + norm * (act_high - act_low)

    def actuator_to_aperture(self, actuator: float) -> float:
        ap_low, ap_high = self._aperture_range
        act_low, act_high = self._actuator_command_range
        if act_high == act_low:
            return ap_low
        norm = (actuator - act_low) / (act_high - act_low)
        return ap_low + norm * (ap_high - ap_low)


BUILTIN_TOOL_MODELS: dict[str, ToolMetadata] = {
    'parallel_gripper': ParallelGripperMetadata(),
    'stretch_gripper': StretchGripperMetadata(),
}


def get_tool_metadata(tool_name: str | None = None) -> ToolMetadata:
    """
    Factory function to resolve and return the ToolMetadata instance for the active tool.

    1. Checks built-in grippers ('stretch_gripper', 'parallel_gripper').
    2. Checks for custom metadata class in user_tools (metadata_module_name/metadata_class_name).
    3. Uses explicit UserToolMetadata for YAML-configured tools (failing fast if required keys are missing).
    """
    _, robot_params = RobotParams.get_params()

    if tool_name is None:
        tool_name = robot_params.get('robot', {}).get('tool')

    if not tool_name:
        raise ToolConfigurationError("No active tool configured in robot_params['robot']['tool'].")

    # 1. Built-in Tool Check
    if tool_name in BUILTIN_TOOL_MODELS:
        return BUILTIN_TOOL_MODELS[tool_name]
    if 'sg4' in tool_name or tool_name == 'stretch_gripper' or 'stretch_gripper' in robot_params.get(tool_name, {}).get('devices', {}):
        return BUILTIN_TOOL_MODELS['stretch_gripper']
    if 'pg4' in tool_name or tool_name == 'parallel_gripper' or 'parallel_gripper' in robot_params.get(tool_name, {}).get('devices', {}):
        return BUILTIN_TOOL_MODELS['parallel_gripper']

    tool_params = robot_params.get(tool_name, {})
    if not tool_params:
        raise ToolConfigurationError(f"Tool '{tool_name}' is not defined in robot_params.")

    # 2. Check for explicit custom ToolMetadata class in user tool directory
    meta_module = tool_params.get('metadata_module_name')
    meta_class = tool_params.get('metadata_class_name')

    if meta_module and meta_class:
        add_user_tool_to_sys_path(tool_name)
        try:
            module = RobotParams.import_user_tool_module(tool_name, meta_module, is_server=False)
            MetadataClass = getattr(module, meta_class)
            return MetadataClass()
        except Exception as e:
            raise ToolConfigurationError(
                f"Failed to import custom metadata class '{meta_class}' from '{meta_module}' for tool '{tool_name}': {e}"
            )

    # 3. Explicit UserToolMetadata parser (fails fast on missing YAML parameters)
    return UserToolMetadata(tool_name)
