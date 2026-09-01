from dataclasses import asdict, dataclass, field
from enum import Enum, auto
from functools import cache, cached_property

from stretch4_body.core.gamepad_enums import MotionProfile
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.robot.robot_client import WristJointClient
from stretch4_body.utils.tool_metadata import (
    BUILTIN_TOOL_MODELS,
    ToolConfigurationError,
    ToolMetadata,
    get_tool_metadata,
)

# Backwards compatibility alias for tests patching GRIPPER_MODELS
GRIPPER_MODELS = BUILTIN_TOOL_MODELS


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
    joints: dict[str, JointPose] = field(default_factory=dict)
    base: BasePose | None = None
    delay_before_start: float = 0.0

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, data):
        pose = cls(name=data["name"], timestamp=data["timestamp"])
        if "delay_before_start" in data:
            pose.delay_before_start = data["delay_before_start"]
        if "base" in data and data["base"]:
            pose.base = BasePose(**data["base"])
        if "joints" in data:
            for k, v in data["joints"].items():
                joint = RobotJoints.get_joint_by_name(k)
                normalized_key = joint.name if joint is not None else k
                v_copy = dict(v)
                v_copy.setdefault("name", normalized_key)
                pose.joints[normalized_key] = JointPose(**v_copy)
        return pose

    @classmethod
    def load_tool_pose_models(cls, tool_name=None) -> dict[str, "RobotPose"]:
        """
        Dynamically load pre-defined pose models from the custom tool directory.
        """

        if tool_name is None:
            _, robot_params = RobotParams.get_params()
            tool_name = robot_params.get("robot", {}).get("tool")

        if not tool_name or not RobotParams.is_user_defined_tool(tool_name):
            return {}

        tool_path = RobotParams.get_user_defined_tool_path(tool_name)
        if not tool_path:
            return {}

        pose_yaml_path = os.path.join(tool_path, "pose_models.yaml")
        if not os.path.exists(pose_yaml_path):
            return {}

        try:
            with open(pose_yaml_path, "r") as f:
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
    def get_joint_by_name(cls, name: str) -> "RobotJoints | None":
        """Looks up a joint by its enum name, alias, configured tool name, or URDF tool joints."""
        if name in cls.__members__:
            return cls[name]
        if name in ["gripper", "tool"]:
            return cls.gripper
        for joint in cls:
            if joint.value == name:
                return joint
            if joint == cls.gripper:
                if (
                    joint.gripper_name and name == joint.gripper_name
                ) or name in joint.tool_joints:
                    return joint
                try:
                    if get_tool_metadata(name) is not None:
                        return joint
                except Exception:
                    pass
        return None

    @classmethod
    def get_end_of_arm_joints(cls) -> list["RobotJoints"]:
        """Returns the wrist joints, plus the gripper if one is configured."""
        joints = [cls.wrist_pitch, cls.wrist_roll, cls.wrist_yaw]
        if cls.gripper.value is not None:
            joints.append(cls.gripper)
        return joints

    @property
    def value(self) -> str | None:
        """Returns the robot_params key for this joint, or the configured gripper joint name for the gripper joint (None if unconfigured)."""
        if self.name == "gripper":
            if self.gripper_model:
                return self.gripper_model.joint_name
            return self.gripper_name
        else:
            return self.name

    @property
    def gripper_model(self) -> ToolMetadata | None:
        """Returns the ToolMetadata for this joint, or None if this isn't the gripper joint or none is configured."""
        if self.name == "gripper":
            try:
                return get_tool_metadata(self.gripper_name)
            except Exception:
                return None
        return None

    @property
    def tool_model(self) -> ToolMetadata | None:
        """Alias for gripper_model."""
        return self.gripper_model

    @property
    def tool_joints(self) -> list[str]:
        """Returns the URDF joint names for this tool, or [] if no tool model."""
        model = self.tool_model
        return model.tool_joints if model else []

    @property
    def finger_joints(self) -> list[str]:
        """Returns the URDF finger joint names for this joint, or [] if no gripper model."""
        return self.tool_joints

    @property
    def tool_links(self) -> list[str]:
        """Returns the URDF link names for this tool, or [] if no tool model."""
        model = self.tool_model
        return model.tool_links if model else []

    @property
    def finger_links(self) -> list[str]:
        """Returns the URDF finger link names for this joint, or [] if no gripper model."""
        return self.tool_links

    @property
    def gripper_client(self) -> WristJointClient | None:
        """Returns a RobotClient instance for this joint's gripper, or None if no gripper model."""
        model = self.gripper_model
        return model.client_class() if model else None

    @cached_property
    def gripper_name(self) -> str | None:
        """Returns the configured tool's robot_params name, or None if no tool is configured."""
        _, robot_params = RobotParams.get_params()
        return robot_params.get("robot", {}).get("tool")

    @property
    def poses(self) -> dict[str, float] | None:
        """Returns this joint's named gripper client poses in URDF units, or None if no gripper model."""
        if self.name == "gripper" and self.gripper_model:
            client = self.gripper_client
            if client and hasattr(client, "poses"):
                # client.poses is in command units (move_to()'s own parameter), not true actuator.
                return {k: self.command_to_urdf(v) for k, v in client.poses.items()}
        return None

    @property
    def actuator_range(self) -> tuple[float, float] | None:
        """Returns (min, max) in true raw actuator units (radians) for this joint, or None if no tool model."""
        model = self.tool_model
        return model.actuator_range if model else None

    @property
    def command_range(self) -> tuple[float, float] | None:
        """Returns (min, max) in this tool's own move_to()/move_by() command units, or None if no tool model."""
        model = self.tool_model
        return model.command_range if model else None

    @property
    def urdf_range(self) -> tuple[float, float] | None:
        """Returns (close, open) in URDF units for this joint, or None if no gripper model."""
        model = self.tool_model
        return model.urdf_range if model else None

    @property
    def position_tolerance(self) -> float | None:
        """Returns this joint's "close enough" tolerance in URDF units, or None if no tool model."""
        model = self.tool_model
        return model.position_tolerance if model else None

    @property
    def aperture_range(self) -> tuple[float, float] | None:
        """Returns (min, max) fingertip aperture for this joint, or None if no gripper model."""
        model = self.tool_model
        return model.aperture_range if model else None


    @cache
    def get_joint_params(self, profile: MotionProfile) -> tuple[float, float]:
        """Returns (vel, accel) for this joint under the given motion profile, from robot_params."""
        if self.value is None:
            raise ValueError(
                f"{self.name} has no joint/device configured in robot params (e.g. no gripper attached)."
            )

        _, robot_params = RobotParams.get_params()
        params = robot_params.get(self.value)
        if params is None:
            raise ValueError(f"No robot params found for joint '{self.value}'.")

        motion_params = params.get("motion")
        if motion_params is None:
            raise ValueError(
                f"Robot params for joint '{self.value}' are missing a 'motion' section."
            )

        profile_name = profile.get_name()
        joint_params = motion_params.get(profile_name)
        if joint_params is None:
            raise ValueError(
                f"Joint '{self.value}' has no '{profile_name}' motion profile defined."
            )

        v = joint_params.get("vel", joint_params.get("vel_m"))
        a = joint_params.get("accel", joint_params.get("accel_m"))
        if v is None or a is None:
            raise ValueError(
                f"Motion profile '{profile_name}' for joint '{self.value}' is missing "
                f"'vel'/'vel_m' or 'accel'/'accel_m' keys."
            )
        return v, a

    @cache
    def get_base_params(
        self, profile: MotionProfile
    ) -> tuple[float, float, float, float]:
        """Returns (vel_xy_m, accel_xy_m, vel_w_r, accel_w_r) for the base under the given motion profile, from robot_params."""
        params = RobotParams().get_params()[1]["omnibase"]
        base_params = params["motion"][profile.get_name()]
        accel_w_r = base_params["accel_w_r"]
        vel_w_r = base_params["vel_w_r"]
        accel_xy_m = base_params["accel_xy_m"]
        vel_xy_m = base_params["vel_xy_m"]
        return vel_xy_m, accel_xy_m, vel_w_r, accel_w_r

    def raise_joint_specific_warning(
        self, method: str, expected_joints: list[str]
    ) -> None:
        """Raises NotImplementedError if this joint isn't one of `expected_joints`."""
        if self.name not in expected_joints:
            raise NotImplementedError(
                f"Method {method} is not implemented for joint {self.name}"
            )

    def get_gripper_model(self, method: str | None = None) -> ToolMetadata:
        """Returns this joint's GripperMetadata, or raises if this isn't the gripper joint or none is configured."""
        method = method if method is not None else "get_gripper_model"
        self.raise_joint_specific_warning(method=method, expected_joints=["gripper"])
        model = self.gripper_model
        if model is None:
            raise ValueError(
                f"No gripper is configured for joint '{self.name}' (needed by '{method}')."
            )
        return model

    # Only relevant for the gripper joint

    def urdf_to_command(self, urdf_units: float) -> float:
        """Converts URDF units (radians/meters) to this tool's own move_to()/move_by() command units."""
        return self.get_gripper_model("urdf_to_command").urdf_to_command(urdf_units)

    def command_to_urdf(self, command: float) -> float:
        """Converts this tool's own move_to()/move_by() command units to URDF units (radians/meters)."""
        return self.get_gripper_model("command_to_urdf").command_to_urdf(command)

    def urdf_to_actuator(self, urdf_units: float) -> float:
        """Converts URDF units (radians/meters) to true raw actuator units (radians)."""
        return self.get_gripper_model("urdf_to_actuator").urdf_to_actuator(urdf_units)

    def actuator_to_urdf(self, actuator: float) -> float:
        """Converts true raw actuator units (radians) to URDF units (radians/meters)."""
        return self.get_gripper_model("actuator_to_urdf").actuator_to_urdf(actuator)

    def command_to_actuator(self, command: float) -> float:
        """Converts this tool's own move_to()/move_by() command units to true raw actuator units"""
        return self.get_gripper_model("command_to_actuator").command_to_actuator(
            command
        )

    def actuator_to_command(self, actuator: float) -> float:
        """Converts true raw actuator units (radians) to this tool's own move_to()/move_by() command units."""
        return self.get_gripper_model("actuator_to_command").actuator_to_command(
            actuator
        )

    def normalized_to_actuator(self, normalized: float) -> float:
        """Converts a normalized scale (0.0=closed, 1.0=open) to actuator units."""
        return self.get_gripper_model("normalized_to_actuator").normalized_to_actuator(
            normalized
        )

    def actuator_to_normalized(self, actuator: float) -> float:
        """Converts actuator units to a normalized scale (0.0=closed, 1.0=open)."""
        return self.get_gripper_model("actuator_to_normalized").actuator_to_normalized(
            actuator
        )

    def urdf_to_normalized(self, urdf: float) -> float:
        """Converts URDF units to a normalized scale (0.0=closed, 1.0=open)."""
        return self.get_gripper_model("urdf_to_normalized").urdf_to_normalized(urdf)

    def normalized_to_urdf(self, normalized: float) -> float:
        """Converts a normalized scale (0.0=closed, 1.0=open) to URDF units."""
        return self.get_gripper_model("normalized_to_urdf").normalized_to_urdf(
            normalized
        )

    def aperture_to_normalized(self, aperture_m: float) -> float:
        """Converts fingertip aperture (meters) to a normalized scale (0.0=closed, 1.0=open)."""
        return self.get_gripper_model("aperture_to_normalized").aperture_to_normalized(
            aperture_m
        )

    def normalized_to_aperture(self, normalized: float) -> float:
        """Converts a normalized scale (0.0=closed, 1.0=open) to fingertip aperture (meters)."""
        return self.get_gripper_model("normalized_to_aperture").normalized_to_aperture(
            normalized
        )

    def aperture_to_actuator(self, aperture_m: float) -> float:
        """Converts fingertip aperture (meters) to actuator units."""
        return self.get_gripper_model("aperture_to_actuator").aperture_to_actuator(
            aperture_m
        )

    def actuator_to_aperture(self, actuator: float) -> float:
        """Converts actuator units to fingertip aperture (meters)."""
        return self.get_gripper_model("actuator_to_aperture").actuator_to_aperture(
            actuator
        )

    def urdf_to_aperture(self, urdf: float) -> float:
        """Converts URDF units to fingertip aperture (meters)."""
        return self.get_gripper_model("urdf_to_aperture").urdf_to_aperture(urdf)

    def aperture_to_urdf(self, aperture_m: float) -> float:
        """Converts fingertip aperture (meters) to URDF units."""
        return self.get_gripper_model("aperture_to_urdf").aperture_to_urdf(aperture_m)
