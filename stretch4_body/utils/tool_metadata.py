import math
from abc import ABC, abstractmethod
from collections.abc import Callable
from functools import cached_property, partial

from stretch4_urdf import get_joint_limits, get_urdf

from stretch4_body.core.hello_utils import deg_to_rad, rad_to_deg
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.robot.robot_client import ToolJointClient, WristJointClient
from stretch4_body.subsystem.end_of_arm.parallel_gripper import ParallelGripper
from stretch4_body.subsystem.end_of_arm.stretch_gripper import StretchGripper
from stretch4_body.utils.user_tool_utils import add_user_tool_to_sys_path


class ToolConfigurationError(ValueError):
    """Raised when an end-of-arm tool parameter or metadata configuration is missing or invalid."""


class ToolMetadata(ABC):
    """
    Abstract base class defining kinematic, hardware command, and physical unit conversions
    for Stretch 4 end-of-arm tools and grippers.

    Five unit tiers, ROS-facing to hardware-facing:
      - urdf: the ROS/URDF joint value (radians or meters), as seen on JointTrajectory/JointState.
      - command: the value this tool's own move_to()/move_by()/pose() take directly (e.g. Pct for
        SG4, fingertip aperture in meters for PG4). This is what ROS-facing code should convert
        into (via urdf_to_command) before calling move_to()/move_by(), and convert out of (via
        command_to_urdf) when reading status back.
      - actuator: the true raw servo/motor register value (radians). This is the boundary every
        Feetech-driven joint bottoms out at -- FeetechSMHello.move_to()'s own argument -- the
        same for every tool, gripper or not (e.g. WristYaw has no ToolMetadata and passes URDF
        radians straight through, because for a direct-drive joint urdf IS actuator).
      - aperture: physical fingertip opening (meters) -- client convenience.
      - normalized: 0.0 (closed) .. 1.0 (open) -- client convenience.
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
    def client_class(self) -> Callable[..., WristJointClient]:
        """
        Constructs a RobotClient for controlling this tool remotely, callable as
        `client_class(parent=None, ip_address=None)`. Either a bespoke RobotClient subclass, or
        `partial(ToolJointClient, self)` binding this metadata into the generic `ToolJointClient`.

        Typed as `WristJointClient` by convention, but this is a duck-typed contract, not an
        enforced one: a user tool's bespoke class isn't validated against that shape, so it may
        omit or diverge from methods (`move_to`, `poses`, etc.) that some consumers assume exist.
        """

    @property
    @abstractmethod
    def driver_class(self) -> type:
        """Subsystem driver subclass for controlling this tool directly."""

    @property
    @abstractmethod
    def actuator_range(self) -> tuple[float, float]:
        """(min_val, max_val) bounds in true raw actuator/servo units (radians, for every tool)."""

    @property
    @abstractmethod
    def command_range(self) -> tuple[float, float]:
        """(min_val, max_val) bounds in this tool's own move_to()/move_by() command units (e.g. % for SG4, aperture meters for PG4)."""

    @property
    def urdf_range(self) -> tuple[float, float]:
        """(min_val, max_val) bounds in URDF/ROS coordinate units (radians or meters)."""
        low, high = self.actuator_range
        return self.actuator_to_urdf(low), self.actuator_to_urdf(high)

    @property
    def aperture_range(self) -> tuple[float, float]:
        """(min_aperture, max_aperture) physical opening bounds (meters or angle)."""
        low, high = self.actuator_range
        return self.actuator_to_aperture(low), self.actuator_to_aperture(high)

    @property
    def aperture_range_m(self) -> tuple[float, float]:
        """Alias for aperture_range for backward compatibility."""
        return self.aperture_range

    # Default fraction of urdf_range treated as "close enough" by position_tolerance below.
    # urdf_range is a real physical quantity (radians/meters, from the joint's own URDF limits)
    # for every tool, unlike command_range -- which is e.g. a percent scale for SG4 but aperture
    # meters for PG4 -- so a fraction of it means the same thing (physical closeness) regardless
    # of how a given tool chose to define its own command units. Subclasses may override this
    # constant, or the property itself, with a hardware-tuned value.
    _POSITION_TOLERANCE_FRACTION: float = 0.02

    @property
    def position_tolerance(self) -> float:
        """
        Absolute error, in URDF units (radians/meters), within which a move to this tool's goal
        position is considered complete. Defaults to a fraction of the tool's full urdf_range so
        callers aren't required to supply a tool-specific value; override for hardware that needs
        a tighter or looser tolerance than the default fraction gives.
        """
        low, high = self.urdf_range
        return self._POSITION_TOLERANCE_FRACTION * abs(high - low)

    # --- Abstract Base Conversions ---

    @abstractmethod
    def urdf_to_command(self, urdf: float) -> float:
        """Converts from URDF units (radians/meters) to this tool's own move_to()/move_by() command units."""

    @abstractmethod
    def command_to_urdf(self, command: float) -> float:
        """Converts from this tool's own move_to()/move_by() command units to URDF units (radians/meters)."""

    @abstractmethod
    def command_to_actuator(self, command: float) -> float:
        """Converts from this tool's own move_to()/move_by() command units to true raw actuator units (radians)."""

    @abstractmethod
    def actuator_to_command(self, actuator: float) -> float:
        """Converts from true raw actuator units (radians) to this tool's own move_to()/move_by() command units."""

    @abstractmethod
    def aperture_to_actuator(self, aperture: float) -> float:
        """Converts from physical opening aperture to true raw actuator units (radians)."""

    @abstractmethod
    def actuator_to_aperture(self, actuator: float) -> float:
        """Converts from true raw actuator units (radians) to physical opening aperture."""

    @abstractmethod
    def status_to_metadata(self, status: dict) -> dict:
        """
        Derives physical/URDF-relevant fields from a raw hardware status dict.

        Returns a dict with keys 'aperture_m', 'finger_rad', 'finger_effort', and 'finger_vel',
        used to populate status['gripper_conversion'] for downstream consumers (ROS JointState
        publishing, self-collision checking, pose recording).
        """

    @staticmethod
    def _map_range(
        value: float, in_min: float, in_max: float, out_min: float, out_max: float
    ) -> float:
        """Linearly maps `value` from the range [in_min, in_max] to the range [out_min, out_max]."""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # --- Normalized <-> Actuator Conversions ---

    def normalized_to_actuator(self, normalized: float) -> float:
        """Converts a normalized scale value (0.0=closed/min, 1.0=open/max) to true raw actuator units (radians)."""
        low, high = self.actuator_range
        return low + normalized * (high - low)

    def actuator_to_normalized(self, actuator: float) -> float:
        """Converts true raw actuator units (radians) to a normalized scale value (0.0=closed/min, 1.0=open/max)."""
        low, high = self.actuator_range
        if high == low:
            return 0.0
        return (actuator - low) / (high - low)

    # --- Chained Layer Conversions ---

    def urdf_to_actuator(self, urdf: float) -> float:
        """Converts URDF units to true raw actuator units (radians), via this tool's command units."""
        return self.command_to_actuator(self.urdf_to_command(urdf))

    def actuator_to_urdf(self, actuator: float) -> float:
        """Converts true raw actuator units (radians) to URDF units, via this tool's command units."""
        return self.command_to_urdf(self.actuator_to_command(actuator))

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

    # --- Client-facing defaults ---

    @property
    def poses(self) -> dict[str, float]:
        """
        Named command positions ('close', 'open', 'mid') in this tool's command_range units --
        i.e. move_to()/move_by()'s own expected units. Subclasses may override to add
        tool-specific poses (e.g. a 'zero' pose).
        """
        low, high = self.command_range
        return {"close": low, "open": high, "mid": (low + high) / 2.0}

    @property
    def status(self) -> dict[str, float]:
        """Default zeroed 'gripper_conversion' status fields, used to seed a fresh client's status dict."""
        return {"aperture_m": 0.0, "finger_rad": 0.0, "finger_effort": 0.0, "finger_vel": 0.0}


class ParallelGripperMetadata(ToolMetadata):
    @property
    def joint_name(self) -> str:
        return "parallel_gripper"

    @property
    def tool_joints(self) -> list[str]:
        return ["finger_left_joint", "finger_right_joint"]

    @property
    def primary_joint(self) -> str:
        return "finger_left_joint"

    @property
    def tool_links(self) -> list[str]:
        return ["finger_left_link", "finger_right_link"]

    @cached_property
    def client_class(self) -> Callable[..., WristJointClient]:
        return partial(ToolJointClient, self)

    @property
    def driver_class(self) -> type:
        return ParallelGripper

    @property
    def poses(self) -> dict[str, float]:
        """
        Named command positions in meters (aperture units) — the PG4 driver/client's move_to()
        takes aperture directly, so command_range coincides with aperture_range for this tool.
        """
        low, high = self.command_range
        return {"zero": 0.0, "close": low, "open": high, "mid": (low + high) / 2.0}

    @property
    def actuator_range(self) -> tuple[float, float]:
        """(closed, open) bounds in true raw servo angle (radians)."""
        range_deg = self._params.get("range_deg", [0.0, 116.5])
        return deg_to_rad(range_deg[0]), deg_to_rad(range_deg[1])

    @property
    def position_tolerance(self) -> float:
        """User-supplied 'position_tolerance' (URDF units, meters) from robot_params if set, else the default fraction of urdf_range."""
        user_value = self._params.get("position_tolerance")
        return float(user_value) if user_value is not None else super().position_tolerance

    @property
    def command_range(self) -> tuple[float, float]:
        """
        (closed, open) bounds in fingertip aperture (meters) — PG4's command units, matching
        `move_to()`/`move_by()`'s own public parameter directly.
        """
        return self.aperture_range

    def urdf_to_command(self, urdf: float) -> float:
        """
        Converts the URDF finger slide-joint value (meters) to fingertip aperture (meters) —
        PG4's command units, matching what this tool's own `move_to()`/`move_by()` take directly.
        """
        lower, upper = self._finger_joint_limits
        range_m = self._params.get("range_mm", 80.0) / 1000.0
        if lower == upper:
            return 0.0
        pct = (urdf - upper) / (lower - upper)
        return pct * range_m

    def command_to_urdf(self, command: float) -> float:
        """
        Converts fingertip aperture (meters) — PG4's command units, see `urdf_to_command` — to
        the URDF finger slide-joint value (meters).
        """
        aperture_m = command
        lower, upper = self._finger_joint_limits
        range_m = self._params.get("range_mm", 80.0) / 1000.0
        if range_m == 0:
            return upper
        pct = aperture_m / range_m
        return upper + pct * (lower - upper)

    def command_to_actuator(self, command: float) -> float:
        """PG4's command units (aperture, meters) coincide with aperture, so this is aperture_to_actuator."""
        return self.aperture_to_actuator(command)

    def actuator_to_command(self, actuator: float) -> float:
        """PG4's command units (aperture, meters) coincide with aperture, so this is actuator_to_aperture."""
        return self.actuator_to_aperture(actuator)

    def aperture_to_actuator(self, aperture: float) -> float:
        """
        Converts fingertip aperture (meters) to raw servo angle (radians), accounting for the
        nonlinear four-bar linkage geometry connecting the servo horn to the finger slider.
        """
        x_mm = (
            aperture * 1000.0
        )  # Calibration constants below (kL/kR/kX0) are specified in mm
        L = self._params.get("kL", 30.25)  # Length of the connecting linkage rod (mm)
        r = self._params.get(
            "kR", 22.0
        )  # Radius of rotation of the servo horn pivot (mm)
        finger_offset = self._params.get(
            "kX0", 10.5
        )  # Horizontal distance from slider pivot to fingertip contact face (mm)
        kT0_rad = math.radians(
            self._params.get("kT0", 44.0)
        )  # Angular offset aligning servo zero with the kinematic reference frame

        # A: The horizontal position of the slider pivot relative to the motor axis center (mm)
        A = -(x_mm / 2.0 + finger_offset)
        # numerator/denominator: Derived from squaring the linkage geometry equation to isolate sin(q_eff)
        numerator = A**2 + r**2 - L**2
        denominator = 2 * A * r
        # Clamp to [-1.0, 1.0] to prevent floating point out-of-bounds domain errors in arcsin
        sin_q_eff = max(-1.0, min(1.0, numerator / denominator))
        q_eff = math.asin(sin_q_eff)
        return kT0_rad - q_eff

    def actuator_to_aperture(self, actuator: float) -> float:
        """Converts raw servo angle (radians) to fingertip aperture (meters), the inverse of `aperture_to_actuator`."""
        L = self._params.get("kL", 30.25)  # Length of the connecting linkage rod (mm)
        r = self._params.get(
            "kR", 22.0
        )  # Radius of rotation of the servo horn pivot (mm)
        finger_offset = self._params.get(
            "kX0", 10.5
        )  # Horizontal distance from slider pivot to fingertip contact face (mm)
        kT0 = self._params.get(
            "kT0", 44.0
        )  # Angular offset aligning servo zero with the kinematic reference frame (deg)

        # q_eff: Effective angle of the servo arm relative to the vertical axis
        q_eff = -1 * actuator + math.radians(kT0)
        # term: The squared horizontal distance spanned by the connecting rod (derived via Pythagorean theorem)
        term = L**2 - (r * math.cos(q_eff)) ** 2
        # x_pivot: Horizontal position of the slider pivot relative to the motor axis center (mm)
        x_pivot = r * math.sin(q_eff) - math.sqrt(term)
        # x_mm: Combined gap width between both fingers (twice the distance from slider to contact face)
        x_mm = 2 * (-x_pivot - finger_offset)
        return round(x_mm, 3) / 1000.0

    @property
    def _params(self) -> dict:
        _, robot_params = RobotParams.get_params()
        return robot_params.get("parallel_gripper", {})

    @cached_property
    def _finger_joint_limits(self) -> tuple[float, float]:
        """Cached (lower, upper) limits of finger_left_joint, loaded from the URDF."""
        _, robot_params = RobotParams.get_params()
        model_name = robot_params["robot"]["model_name"]
        batch_name = robot_params["robot"]["batch_name"]
        eoa_name = robot_params["robot"]["tool"]
        urdf_contents = get_urdf(
            model_name, batch_name, eoa_name, do_add_file_prefix_to_absolute_paths=False
        )
        limits = get_joint_limits(urdf_contents)
        return limits.get("finger_left_joint", (-0.04, 0.0))

    def status_to_metadata(self, status: dict) -> dict:
        pos_mm = status.get("pos_mm")
        if pos_mm is None:
            pos_mm = self.actuator_to_aperture(status.get("pos", 0.0)) * 1000.0
        return {
            "aperture_m": pos_mm / 1000.0,
            "finger_rad": self.aperture_to_urdf(pos_mm / 1000.0),
            "finger_effort": status.get("effort", 0.0),
            "finger_vel": status.get("vel", 0.0),
        }


class StretchGripperMetadata(ToolMetadata):
    @property
    def joint_name(self) -> str:
        return "stretch_gripper"

    @property
    def tool_joints(self) -> list[str]:
        return ["gripper_finger_left_joint", "gripper_finger_right_joint"]

    @property
    def primary_joint(self) -> str:
        return "gripper_finger_left_joint"

    @property
    def tool_links(self) -> list[str]:
        return ["gripper_finger_left_link", "gripper_finger_right_link"]

    @cached_property
    def client_class(self) -> Callable[..., WristJointClient]:
        return partial(ToolJointClient, self)

    @property
    def driver_class(self) -> type:
        return StretchGripper

    @property
    def poses(self) -> dict[str, float]:
        """Named command positions in pct (command units): 'zero' (fingertips just touching), plus
        'close'/'open' bounding the full range."""
        low_deg, high_deg = self._range_deg
        pct_max_open = 100.0 * abs(high_deg / low_deg) if low_deg else 100.0
        return {"zero": 0.0, "close": -100.0, "open": pct_max_open}

    @property
    def command_range(self) -> tuple[float, float]:
        """(closed, open) bounds in Pct — SG4's command units, matching move_to()/move_by()'s own public parameter."""
        return self.poses["close"], self.poses["open"]

    @property
    def actuator_range(self) -> tuple[float, float]:
        """(closed, open) bounds in true raw servo angle (radians)."""
        low, high = self.command_range
        return self.command_to_actuator(low), self.command_to_actuator(high)

    @property
    def position_tolerance(self) -> float:
        """User-supplied 'position_tolerance' (URDF units, radians) from robot_params if set, else the default fraction of urdf_range."""
        _, robot_params = RobotParams.get_params()
        user_value = robot_params.get("stretch_gripper", {}).get("position_tolerance")
        return float(user_value) if user_value is not None else super().position_tolerance

    def urdf_to_command(self, urdf: float) -> float:
        """Converts the URDF finger joint value (radians) to Pct — SG4's command units."""
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get("stretch_gripper", {})
        range_deg_0 = sg_params.get("range_deg", [-100.0, 0.0])[0]
        return -100.0 * urdf / deg_to_rad(range_deg_0)

    def command_to_urdf(self, command: float) -> float:
        """Converts Pct — SG4's command units — to the URDF finger joint value (radians)."""
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get("stretch_gripper", {})
        range_deg_0 = sg_params.get("range_deg", [-100.0, 0.0])[0]
        return command * deg_to_rad(range_deg_0) / -100.0

    def command_to_actuator(self, command: float) -> float:
        """
        Converts Pct — SG4's command units — to true raw servo angle (radians). Promoted from
        StretchGripper.pct_to_world_rad() so ToolMetadata owns this conversion the same way PG4
        does via aperture_to_actuator(), instead of leaving it only on the driver.
        """
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get("stretch_gripper", {})
        range_deg_0 = sg_params.get("range_deg", [-100.0, 0.0])[0]
        return deg_to_rad(range_deg_0) * command / -100.0

    def actuator_to_command(self, actuator: float) -> float:
        """Converts true raw servo angle (radians) to Pct — SG4's command units. Promoted from StretchGripper.world_rad_to_pct()."""
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get("stretch_gripper", {})
        range_deg_0 = sg_params.get("range_deg", [-100.0, 0.0])[0]
        return -100.0 * actuator / deg_to_rad(range_deg_0)

    @property
    def _range_deg(self) -> tuple[float, float]:
        _, robot_params = RobotParams.get_params()
        range_deg = robot_params.get("stretch_gripper", {}).get(
            "range_deg", [-100.0, 0.0]
        )
        return float(range_deg[0]), float(range_deg[1])

    @property
    def _gripper_conversion_params(self) -> dict:
        _, robot_params = RobotParams.get_params()
        return robot_params.get("stretch_gripper", {}).get("gripper_conversion", {})

    @staticmethod
    def _angle_from_chord_length_and_radius(radius_m: float, chord_m: float) -> float:
        """Angle (radians) subtended by a chord of length `chord_m` on a circle of radius `radius_m`."""
        return 2 * math.asin(chord_m / (2 * radius_m))

    @staticmethod
    def _chord_from_radius_and_angle(radius_m: float, angle_rad: float) -> float:
        """Chord length (meters) subtended by `angle_rad` on a circle of radius `radius_m`."""
        return 2 * radius_m * math.sin(angle_rad / 2)

    @property
    def _finger_length_m(self) -> float:
        return self._gripper_conversion_params["finger_length_m"]

    @property
    def _aperture_open_deg(self) -> float:
        """Aperture opening angle (degrees) corresponding to the fully-open finger chord length."""
        params = self._gripper_conversion_params
        aperture_open_rad = self._angle_from_chord_length_and_radius(
            self._finger_length_m, params["aperture_open_m"]
        )
        return math.degrees(aperture_open_rad)

    @cached_property
    def _servo_to_aperture_slope(self) -> float:
        params = self._gripper_conversion_params
        return (
            params["aperture_open_m"] - params["aperture_closed_m"]
        ) / self._aperture_open_deg

    def _aperture_m_to_aperture_angle_degrees(self, aperture_m: float) -> float:
        return math.degrees(
            self._angle_from_chord_length_and_radius(self._finger_length_m, aperture_m)
        )

    def _aperture_angle_degrees_to_aperture_m(
        self, aperture_angle_degrees: float
    ) -> float:
        return self._chord_from_radius_and_angle(
            self._finger_length_m, math.radians(aperture_angle_degrees)
        )

    def aperture_to_actuator(self, aperture: float) -> float:
        """
        Models the SG4 gripper's finger as a circular arc to map an aperture (chord length,
        meters) to true raw servo angle (radians). Note: this is a simplified model, not
        accurate to the gripper's real motion.
        """
        aperture_angle_deg = self._aperture_m_to_aperture_angle_degrees(aperture)
        servo_closed_deg, servo_open_deg = self._range_deg
        servo_angle_deg = self._map_range(
            aperture_angle_deg, 0.0, self._aperture_open_deg, servo_closed_deg, servo_open_deg
        )
        return deg_to_rad(servo_angle_deg)

    def actuator_to_aperture(self, actuator: float) -> float:
        """Converts true raw servo angle (radians) to fingertip aperture (meters), the inverse of `aperture_to_actuator`."""
        servo_closed_deg, servo_open_deg = self._range_deg
        aperture_angle_deg = self._map_range(
            rad_to_deg(actuator), servo_closed_deg, servo_open_deg, 0.0, self._aperture_open_deg
        )
        return self._aperture_angle_degrees_to_aperture_m(aperture_angle_deg)

    def status_to_metadata(self, status: dict) -> dict:
        aperture_m = self.actuator_to_aperture(status.get("pos", 0.0))
        finger_rad = (
            math.radians(self._aperture_m_to_aperture_angle_degrees(aperture_m)) / 2.0
        )
        return {
            "aperture_m": aperture_m,
            "finger_rad": finger_rad,
            "finger_effort": status["effort"],
            "finger_vel": (self._servo_to_aperture_slope * status["vel"]) / 2.0,
        }


class LinearToolMetadata(ToolMetadata):
    """
    Metadata representation for custom user tools loaded strictly from YAML parameters.
    Fails fast if any required configuration key is missing.
    """

    def __init__(self, tool_name: str):
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

        # 1. Joints and Links
        joints = self.tool_params.get("tool_joints", [])
        if not joints:
            raise ToolConfigurationError(
                f"Missing required key 'tool_joints' in robot_params['{self.tool_name}']."
            )
        self._tool_joints = list(joints)

        self._primary_joint = self.tool_params.get(
            "primary_joint", self._tool_joints[0]
        )

        links = self.tool_params.get("tool_links", self.tool_params.get("finger_links"))
        if not links:
            raise ToolConfigurationError(
                f"Missing required key 'tool_links' in robot_params['{self.tool_name}']."
            )
        self._tool_links = list(links)

        # 2. Client Class (optional: a single-joint tool can omit this and fall back to the
        # generic ToolJointClient instead of a bespoke class)
        client_module = self.tool_params.get("client_module_name")
        client_class_name = self.tool_params.get("client_class_name")
        if client_module or client_class_name:
            if not client_module or not client_class_name:
                raise ToolConfigurationError(
                    f"Both 'client_module_name' and 'client_class_name' must be set together in robot_params['{self.tool_name}']."
                )
            add_user_tool_to_sys_path(self.tool_name)
            try:
                module = RobotParams.import_user_tool_module(
                    self.tool_name, client_module, is_server=False
                )
                self._client_class = getattr(module, client_class_name)
            except Exception as e:
                raise ToolConfigurationError(
                    f"Failed to import client class '{client_class_name}' from module '{client_module}' "
                    f"for user tool '{self.tool_name}': {e}"
                )
        else:
            self._client_class = None

        # 3. Ranges
        act_range = self.tool_params.get("actuator_command_range")
        if not act_range or len(act_range) != 2:
            raise ToolConfigurationError(
                f"Missing or invalid required key 'actuator_command_range' [min, max] in robot_params['{self.tool_name}']."
            )
        self._command_range = (float(act_range[0]), float(act_range[1]))

        ap_range = self.tool_params.get(
            "aperture_range", self.tool_params.get("aperture_range_m")
        )
        if not ap_range or len(ap_range) != 2:
            raise ToolConfigurationError(
                f"Missing or invalid required key 'aperture_range' [min, max] in robot_params['{self.tool_name}']."
            )
        self._aperture_range = (float(ap_range[0]), float(ap_range[1]))

        self._urdf_scale = float(self.tool_params.get("urdf_to_actuator_scale", 1.0))

        # Optional: 'position_tolerance', in URDF units (meters/radians). Left unset (None) if
        # the user doesn't supply one, so position_tolerance falls back to the base class's
        # fraction-of-urdf_range default instead.
        user_tolerance = self.tool_params.get("position_tolerance")
        self._position_tolerance = (
            float(user_tolerance) if user_tolerance is not None else None
        )

    @property
    def tool_joints(self) -> list[str]:
        return self._tool_joints

    @property
    def primary_joint(self) -> str:
        return self._primary_joint

    @property
    def position_tolerance(self) -> float:
        """User-supplied 'position_tolerance' (URDF units) from robot_params if set, else the default fraction of urdf_range."""
        if self._position_tolerance is not None:
            return self._position_tolerance
        return super().position_tolerance

    @property
    def tool_links(self) -> list[str]:
        return self._tool_links

    @cached_property
    def client_class(self) -> Callable[..., WristJointClient]:
        return self._client_class or partial(ToolJointClient, self)

    @property
    def driver_class(self) -> type:
        device_params = self.tool_params.get("devices", {}).get(self.joint_name, {})
        py_module = (
            device_params.get("py_module_name")
            or self.tool_params.get("server_module_name")
            or self.tool_params.get("py_module_name")
        )
        py_class = (
            device_params.get("py_class_name")
            or self.tool_params.get("server_class_name")
            or self.tool_params.get("py_class_name")
        )

        if not py_module or not py_class:
            raise ToolConfigurationError(
                f"Direct driver configuration for tool '{self.tool_name}' must specify 'py_module_name' and 'py_class_name'."
            )
        add_user_tool_to_sys_path(self.tool_name)
        try:
            module = RobotParams.import_user_tool_module(
                self.tool_name, py_module, is_server=True
            )
            return getattr(module, py_class)
        except Exception as e:
            raise ToolConfigurationError(
                f"Failed to import driver class '{py_class}' from module '{py_module}' for tool '{self.tool_name}': {e}"
            )

    @property
    def command_range(self) -> tuple[float, float]:
        """(min_val, max_val) in this tool's own move_to()/move_by() command units, from the 'actuator_command_range' YAML key."""
        return self._command_range

    @property
    def actuator_range(self) -> tuple[float, float]:
        """
        User tools have no YAML mechanism to describe a true actuator/servo scale distinct from
        move_to()/move_by()'s own command units, so actuator is assumed to coincide with command
        -- see command_to_actuator/actuator_to_command.
        """
        return self._command_range

    @property
    def aperture_range(self) -> tuple[float, float]:
        return self._aperture_range

    def urdf_to_command(self, urdf: float) -> float:
        return urdf * self._urdf_scale

    def command_to_urdf(self, command: float) -> float:
        return command / self._urdf_scale if self._urdf_scale != 0 else command

    def command_to_actuator(self, command: float) -> float:
        """Identity: user tools assume the true actuator range coincides with command (see actuator_range)."""
        return command

    def actuator_to_command(self, actuator: float) -> float:
        """Identity: user tools assume the true actuator range coincides with command (see actuator_range)."""
        return actuator

    def aperture_to_actuator(self, aperture: float) -> float:
        ap_low, ap_high = self._aperture_range
        act_low, act_high = self._command_range
        if ap_high == ap_low:
            return act_low
        norm = (aperture - ap_low) / (ap_high - ap_low)
        return act_low + norm * (act_high - act_low)

    def actuator_to_aperture(self, actuator: float) -> float:
        ap_low, ap_high = self._aperture_range
        act_low, act_high = self._command_range
        if act_high == act_low:
            return ap_low
        norm = (actuator - act_low) / (act_high - act_low)
        return ap_low + norm * (ap_high - ap_low)

    def status_to_metadata(self, status: dict) -> dict:
        """
        Generic default: derives aperture/URDF/effort/velocity fields from a raw 'pos' status
        value using this tool's own actuator conversions. Custom tools needing bespoke status
        handling should provide their own ToolMetadata subclass via metadata_module_name /
        metadata_class_name instead of relying on this default.
        """
        actuator = status.get("pos", 0.0)
        return {
            "aperture_m": self.actuator_to_aperture(actuator),
            "finger_rad": self.actuator_to_urdf(actuator),
            "finger_effort": status.get("effort", 0.0),
            "finger_vel": status.get("vel", 0.0),
        }


_sg_meta = StretchGripperMetadata()
_pg_meta = ParallelGripperMetadata()

BUILTIN_TOOL_MODELS: dict[str, ToolMetadata] = {
    "parallel_gripper": _pg_meta,
    "stretch_gripper": _sg_meta,
    "eoa_wrist_dw4_tool_sg4": _sg_meta,
    "eoa_wrist_dw4_tool_pg4": _pg_meta,
}


def is_tool_joint(name: str) -> bool:
    """
    True if `name` is configured as a tool with ToolMetadata (a gripper or other end-effector),
    as opposed to a plain joint (e.g. wrist_yaw/wrist_pitch/wrist_roll) that has no ToolMetadata
    concept. A cheap membership/key check only, not full validation — so a joint that IS a tool
    but is misconfigured still passes here and should raise from `get_tool_metadata` instead of
    being silently treated as "not a tool".
    """
    if name in BUILTIN_TOOL_MODELS:
        return True
    _, robot_params = RobotParams.get_params()
    tool_params = robot_params.get(name, {})
    return bool(tool_params.get("tool_joints")) or bool(
        tool_params.get("metadata_module_name") and tool_params.get("metadata_class_name")
    )


def get_tool_metadata(tool_name: str | None = None) -> ToolMetadata:
    """
    Factory function to resolve and return the ToolMetadata instance for the active tool.

    1. Checks built-in grippers ('stretch_gripper', 'parallel_gripper') and standard tool aliases.
    2. Checks for custom metadata class in user_tools (metadata_module_name/metadata_class_name).
    3. Uses explicit LinearToolMetadata for YAML-configured tools (failing fast if required keys are missing).
    """
    _, robot_params = RobotParams.get_params()

    if tool_name is None:
        tool_name = robot_params.get("robot", {}).get("tool")

    if not tool_name:
        raise ToolConfigurationError(
            "No active tool configured in robot_params['robot']['tool']."
        )

    # 1. Built-in Tool Check
    if tool_name in BUILTIN_TOOL_MODELS:
        return BUILTIN_TOOL_MODELS[tool_name]

    tool_params = robot_params.get(tool_name, {})
    for device_name in tool_params.get("devices", {}):
        if device_name in BUILTIN_TOOL_MODELS:
            return BUILTIN_TOOL_MODELS[device_name]

    if not tool_params:
        raise ToolConfigurationError(
            f"Tool '{tool_name}' is not defined in robot_params."
        )

    # 2. Check for explicit custom ToolMetadata class in user tool directory
    meta_module = tool_params.get("metadata_module_name")
    meta_class = tool_params.get("metadata_class_name")

    if meta_module and meta_class:
        add_user_tool_to_sys_path(tool_name)
        try:
            module = RobotParams.import_user_tool_module(
                tool_name, meta_module, is_server=False
            )
            MetadataClass = getattr(module, meta_class)
            return MetadataClass()
        except Exception as e:
            raise ToolConfigurationError(
                f"Failed to import custom metadata class '{meta_class}' from '{meta_module}' for tool '{tool_name}': {e}"
            )

    # 3. Explicit LinearToolMetadata parser (fails fast on missing YAML parameters)
    return LinearToolMetadata(tool_name)
