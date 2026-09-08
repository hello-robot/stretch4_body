from __future__ import annotations

import math
from abc import ABC, abstractmethod
from collections.abc import Callable
from functools import cached_property, partial
from typing import TYPE_CHECKING, Any

from stretch4_urdf import get_joint_limits, get_urdf

from stretch4_body.core.hello_utils import deg_to_rad, rad_to_deg
from stretch4_body.core.robot_params import RobotParams

if TYPE_CHECKING:
    from stretch4_body.robot.robot_client import WristJointClient


class ToolConfigurationError(ValueError):
    """Raised when an end-of-arm tool parameter or metadata configuration is missing or invalid."""


class ToolMetadata(ABC):
    """
    Abstract base class defining kinematic, hardware command, and physical unit conversions
    for Stretch 4 end-of-arm tools and grippers.

    Five unit types, ROS-facing to hardware-facing:
      - urdf: the ROS/URDF joint value (radians or meters), as seen on JointTrajectory/JointState.
      - command: the value this tool's own move_to()/move_by()/pose() take directly (e.g. Pct for
        SG4, fingertip aperture in meters for PG4). This is what ROS-facing code should convert
        into (via urdf_to_command) before calling move_to()/move_by(), and convert out of (via
        command_to_urdf) when reading status back.
      - actuator: the servo/motor register value (radians). This is the boundary every
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
        """(min_val, max_val) bounds in actuator units (radians, for every tool)."""

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
        """Converts from this tool's own move_to()/move_by() command units to actuator units (radians)."""

    @abstractmethod
    def actuator_to_command(self, actuator: float) -> float:
        """Converts from actuator units (radians) to this tool's own move_to()/move_by() command units."""

    @abstractmethod
    def aperture_to_actuator(self, aperture: float) -> float:
        """Converts from physical opening aperture to actuator units (radians)."""

    @abstractmethod
    def actuator_to_aperture(self, actuator: float) -> float:
        """Converts from actuator units (radians) to physical opening aperture."""

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
        """Converts a normalized scale value (0.0=closed/min, 1.0=open/max) to actuator units (radians)."""
        low, high = self.actuator_range
        return low + normalized * (high - low)

    def actuator_to_normalized(self, actuator: float) -> float:
        """Converts actuator units (radians) to a normalized scale value (0.0=closed/min, 1.0=open/max)."""
        low, high = self.actuator_range
        if high == low:
            return 0.0
        return (actuator - low) / (high - low)

    # --- Chained Layer Conversions ---

    def urdf_to_actuator(self, urdf: float) -> float:
        """Converts URDF units to actuator units (radians), via this tool's command units."""
        return self.command_to_actuator(self.urdf_to_command(urdf))

    def actuator_to_urdf(self, actuator: float) -> float:
        """Converts actuator units (radians) to URDF units, via this tool's command units."""
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

    def aperture_to_command(self, aperture: float) -> float:
        act = self.aperture_to_actuator(aperture)
        return self.actuator_to_command(act)

    def command_to_aperture(self, command: float) -> float:
        act = self.command_to_actuator(command)
        return self.actuator_to_aperture(act)

    def normalized_to_command(self, normalized: float) -> float:
        act = self.normalized_to_actuator(normalized)
        return self.actuator_to_command(act)

    def command_to_normalized(self, command: float) -> float:
        act = self.command_to_actuator(command)
        return self.actuator_to_normalized(act)

    # --- Differential (velocity / delta) Conversions (y' = f'(x) * x')---

    _UNIT_TYPE_PARENT: dict[str, str] = {
        "urdf": "command",
        "command": "actuator",
        "aperture": "actuator",
        "normalized": "actuator",
    }

    #: Edges known to be affine, whose gain is exactly the full-range secant. Subclasses extend
    #: this; unlisted edges use an analytic override, else central differencing.
    _LINEAR_CONVERSIONS: frozenset[tuple[str, str]] = frozenset(
        {("normalized", "actuator"), ("actuator", "normalized")}
    )

    @classmethod
    def _unit_type_path(cls, frm: str, to: str) -> list[str]:
        """Ordered list of unit types to traverse from `frm` to `to`, via their lowest common ancestor."""
        valid = set(cls._UNIT_TYPE_PARENT) | {"actuator"}
        for unit_type in (frm, to):
            if unit_type not in valid:
                raise ToolConfigurationError(
                    f"Unknown unit type '{unit_type}'. Expected one of {sorted(valid)}."
                )
        if frm == to:
            return [frm]

        def ancestors(unit_type: str) -> list[str]:
            chain = [unit_type]
            while unit_type in cls._UNIT_TYPE_PARENT:
                unit_type = cls._UNIT_TYPE_PARENT[unit_type]
                chain.append(unit_type)
            return chain

        up_from = ancestors(frm)
        up_to = ancestors(to)
        depth_in_to = {unit_type: i for i, unit_type in enumerate(up_to)}
        for i, unit_type in enumerate(up_from):
            if unit_type in depth_in_to:
                return up_from[: i + 1] + list(reversed(up_to[: depth_in_to[unit_type]]))
        raise ToolConfigurationError(f"No conversion path from '{frm}' to '{to}'.")

    def unit_type_range(self, unit_type: str) -> tuple[float, float]:
        """(min, max) bounds of this tool in the given unit type."""
        if unit_type == "urdf":
            return self.urdf_range
        if unit_type == "command":
            return self.command_range
        if unit_type == "actuator":
            return self.actuator_range
        if unit_type == "aperture":
            return self.aperture_range
        if unit_type == "normalized":
            return 0.0, 1.0
        raise ToolConfigurationError(f"Unknown unit type '{unit_type}'.")

    def _analytic_gain(self, frm: str, to: str, at: float) -> float | None:
        """
        d(to)/d(frm) in closed form for one edge, or None if unavailable. Subclasses override
        this for nonlinear transmissions.
        """
        return None

    def _edge_gain(self, frm: str, to: str, at: float) -> float:
        """d(to)/d(frm) across one primitive edge, with `at` expressed in `frm` units."""
        convert = getattr(self, f"{frm}_to_{to}")
        low, high = sorted(self.unit_type_range(frm))

        if (frm, to) in self._LINEAR_CONVERSIONS and high > low:
            return (convert(high) - convert(low)) / (high - low)

        analytic = self._analytic_gain(frm, to, at)
        if analytic is not None:
            return analytic

        at = min(max(at, low), high)
        span = high - low
        step = span * 1e-4 if span > 0 else 1e-6
        lower, upper = at - step, at + step
        # Keep the stencil inside the valid range: a conversion evaluated past its own limits
        # can saturate or raise.
        if lower < low:
            lower, upper = low, min(low + 2 * step, high)
        elif upper > high:
            lower, upper = max(high - 2 * step, low), high
        if upper == lower:
            return 0.0
        return (convert(upper) - convert(lower)) / (upper - lower)

    def conversion_gain(self, frm: str, to: str, at: float) -> float:
        """
        Jacobian of the `frm` -> `to` conversion: d(to)/d(frm) at `at`, in `frm` units.

        Scalar, as a tool has one actuated degree of freedom. `at` is required because the
        derivative is constant only for an affine conversion; PG4's actuator->aperture
        derivative spans 0.0129 to 0.0544 m/rad. Composite pairs apply the chain rule.
        """
        path = self._unit_type_path(frm, to)
        gain = 1.0
        position = at
        for source, target in zip(path, path[1:]):
            gain *= self._edge_gain(source, target, position)
            position = getattr(self, f"{source}_to_{target}")(position)
        return gain

    def convert_velocity(self, velocity: float, frm: str, to: str, at: float) -> float:
        """Converts a rate from `frm` units per second to `to` units per second, at position `at`."""
        return velocity * self.conversion_gain(frm, to, at)

    def convert_acceleration(self, accel: float, frm: str, to: str, at: float) -> float:
        """
        Converts an acceleration between unit types using the first-order gain only.

        The exact transform is y'' = f'(x)*x'' + f''(x)*x'^2; the second term is dropped. Valid
        for a motion-profile limit, not for tracking an acceleration trajectory.
        """
        return accel * self.conversion_gain(frm, to, at)

    def convert_delta(self, delta: float, frm: str, to: str, at: float) -> float:
        """
        Converts a finite displacement exactly, as f(at + delta) - f(at).

        Prefer this over `convert_velocity` for a displacement (a move_by amount): it is exact
        across a nonlinear transmission, where the Jacobian is only first-order.
        """
        if frm == to:
            return delta
        self._unit_type_path(frm, to)  # validates both unit type names
        convert = getattr(self, f"{frm}_to_{to}")
        return convert(at + delta) - convert(at)

    # --- Named velocity wrappers, mirroring the position conversions above ---

    def urdf_to_command_velocity(self, velocity: float, at_urdf: float) -> float:
        return self.convert_velocity(velocity, "urdf", "command", at_urdf)

    def command_to_urdf_velocity(self, velocity: float, at_command: float) -> float:
        return self.convert_velocity(velocity, "command", "urdf", at_command)

    def urdf_to_actuator_velocity(self, velocity: float, at_urdf: float) -> float:
        return self.convert_velocity(velocity, "urdf", "actuator", at_urdf)

    def actuator_to_urdf_velocity(self, velocity: float, at_actuator: float) -> float:
        return self.convert_velocity(velocity, "actuator", "urdf", at_actuator)

    def command_to_actuator_velocity(self, velocity: float, at_command: float) -> float:
        return self.convert_velocity(velocity, "command", "actuator", at_command)

    def actuator_to_command_velocity(
        self, velocity: float, at_actuator: float
    ) -> float:
        return self.convert_velocity(velocity, "actuator", "command", at_actuator)

    def actuator_to_aperture_velocity(
        self, velocity: float, at_actuator: float
    ) -> float:
        return self.convert_velocity(velocity, "actuator", "aperture", at_actuator)

    def aperture_to_actuator_velocity(
        self, velocity: float, at_aperture: float
    ) -> float:
        return self.convert_velocity(velocity, "aperture", "actuator", at_aperture)

    # --- Velocity limits ---

    def actuator_velocity_limit(self, profile: str = "default") -> float:
        """
        This tool's motion-profile velocity limit, in actuator units (rad/s).

        Raises if the tool has no motion params, which includes any tool that is not the
        configured one.
        """
        _, robot_params = RobotParams.get_params()
        motion = robot_params.get(self.joint_name, {}).get("motion", {})
        prof = motion.get(profile) or motion.get("default")
        if not prof or "vel" not in prof:
            raise ToolConfigurationError(
                f"No motion velocity limit for tool '{self.joint_name}' "
                f"(looked for robot_params['{self.joint_name}']['motion']['{profile}']['vel']). "
                "Is this the configured tool?"
            )
        return float(prof["vel"])

    def velocity_limit(self, unit_type: str, at: float, profile: str = "default") -> float:
        """
        The actuator velocity limit in `unit_type` units:
        |d(unit_type)/d(actuator)| * limit_actuator, at position `at` (in `unit_type` units).

        Position-dependent wherever that derivative is.
        """
        limit = self.actuator_velocity_limit(profile)
        at_actuator = (
            at if unit_type == "actuator" else getattr(self, f"{unit_type}_to_actuator")(at)
        )
        return abs(self.convert_velocity(limit, "actuator", unit_type, at_actuator))

    def position_independent_velocity_limit(
        self, unit_type: str, profile: str = "default", samples: int = 33
    ) -> float:
        """
        The minimum of `velocity_limit` over the actuator range, so it needs no position: a rate
        achievable everywhere. Use where a single scalar is required, such as a ROS parameter.
        """
        limit = self.actuator_velocity_limit(profile)
        low, high = sorted(self.actuator_range)
        if high == low:
            return abs(self.convert_velocity(limit, "actuator", unit_type, low))
        step = (high - low) / (samples - 1)
        return min(
            abs(self.convert_velocity(limit, "actuator", unit_type, low + i * step))
            for i in range(samples)
        )

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
        return {
            "aperture_m": 0.0,
            "finger_rad": 0.0,
            "finger_effort": 0.0,
            "finger_vel": 0.0,
        }


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
        # Import here to avoid circular dependencies
        from stretch4_body.robot.robot_client import ToolJointClient

        return partial(ToolJointClient, self)

    @property
    def driver_class(self) -> type:
        # Import here to avoid circular dependencies
        from stretch4_body.subsystem.end_of_arm.parallel_gripper import ParallelGripper

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
        """(closed, open) bounds in servo angle (radians)."""
        range_deg = self._params.get("range_deg", [0.0, 116.5])
        return deg_to_rad(range_deg[0]), deg_to_rad(range_deg[1])

    @property
    def position_tolerance(self) -> float:
        """User-supplied 'position_tolerance' (URDF units, meters) from robot_params if set, else the default fraction of urdf_range."""
        user_value = self._params.get("position_tolerance")
        return (
            float(user_value) if user_value is not None else super().position_tolerance
        )

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
        Converts fingertip aperture (meters) to servo angle (radians) across the nonlinear
        slider-crank linkage: the servo horn is the crank (kR), kL the connecting rod, and the
        finger carrier the slider.
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
        # Not rounded: quantizing a conversion makes its numeric derivative unusable.
        return x_mm / 1000.0

    # urdf <-> command (aperture) is affine. command <-> actuator is the slider-crank linkage,
    # since PG4's command unit is aperture; see _analytic_gain.
    _LINEAR_CONVERSIONS = ToolMetadata._LINEAR_CONVERSIONS | frozenset(
        {("urdf", "command"), ("command", "urdf")}
    )

    def _analytic_gain(self, frm: str, to: str, at: float) -> float | None:
        """Closed-form derivative of the slider-crank linkage; see `actuator_to_aperture`."""
        # PG4's command units are aperture meters, so the command edge is the same linkage.
        pair = (
            frm.replace("command", "aperture"),
            to.replace("command", "aperture"),
        )
        if pair == ("actuator", "aperture"):
            return self._aperture_gain_at_actuator(at)
        if pair == ("aperture", "actuator"):
            actuator = self.aperture_to_actuator(at)
            gain = self._aperture_gain_at_actuator(actuator)
            return None if gain == 0.0 else 1.0 / gain
        return None

    def _aperture_gain_at_actuator(self, actuator: float) -> float:
        """
        d(aperture_m)/d(actuator_rad), differentiating `actuator_to_aperture` in closed form.

        With q = kT0 - actuator and term = L^2 - (r*cos q)^2:
            d(x_mm)/d(actuator) = 2 * (r*cos q - r^2*sin q*cos q / sqrt(term))
        """
        L = self._params.get("kL", 30.25)
        r = self._params.get("kR", 22.0)
        kT0_rad = math.radians(self._params.get("kT0", 44.0))

        q = kT0_rad - actuator
        term = L**2 - (r * math.cos(q)) ** 2
        if term <= 0.0:
            # Unbounded at the linkage singularity.
            return 0.0
        d_x_mm = 2 * (
            r * math.cos(q) - (r**2 * math.sin(q) * math.cos(q)) / math.sqrt(term)
        )
        return d_x_mm / 1000.0

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
            # Time derivative of finger_rad above.
            "finger_vel": self.actuator_to_urdf_velocity(
                status.get("vel", 0.0), status.get("pos", 0.0)
            ),
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
        # Import here to avoid circular dependencies
        from stretch4_body.robot.robot_client import ToolJointClient

        return partial(ToolJointClient, self)

    @property
    def driver_class(self) -> type:
        # Import here to avoid circular dependencies
        from stretch4_body.subsystem.end_of_arm.stretch_gripper import StretchGripper

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
        """(closed, open) bounds in servo angle (radians)."""
        low, high = self.command_range
        return self.command_to_actuator(low), self.command_to_actuator(high)

    @property
    def position_tolerance(self) -> float:
        """User-supplied 'position_tolerance' (URDF units, radians) from robot_params if set, else the default fraction of urdf_range."""
        _, robot_params = RobotParams.get_params()
        user_value = robot_params.get("stretch_gripper", {}).get("position_tolerance")
        return (
            float(user_value) if user_value is not None else super().position_tolerance
        )

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
        Converts Pct — SG4's command units — to servo angle (radians). Promoted from
        StretchGripper.pct_to_world_rad() so ToolMetadata owns this conversion the same way PG4
        does via aperture_to_actuator(), instead of leaving it only on the driver.
        """
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get("stretch_gripper", {})
        range_deg_0 = sg_params.get("range_deg", [-100.0, 0.0])[0]
        return deg_to_rad(range_deg_0) * command / -100.0

    def actuator_to_command(self, actuator: float) -> float:
        """Converts servo angle (radians) to Pct — SG4's command units"""
        _, robot_params = RobotParams.get_params()
        sg_params = robot_params.get("stretch_gripper", {})
        range_deg_0 = sg_params.get("range_deg", [-100.0, 0.0])[0]
        return -100.0 * actuator / deg_to_rad(range_deg_0)

    # SG4's urdf/command/actuator conversions are pure scalings through the origin, so a rate
    # converts like a position there. Only the aperture edge is nonlinear.
    _LINEAR_CONVERSIONS = ToolMetadata._LINEAR_CONVERSIONS | frozenset(
        {
            ("urdf", "command"),
            ("command", "urdf"),
            ("command", "actuator"),
            ("actuator", "command"),
        }
    )

    def _analytic_gain(self, frm: str, to: str, at: float) -> float | None:
        """Closed-form derivative of the circular-arc chord model; see `actuator_to_aperture`."""
        if (frm, to) == ("actuator", "aperture"):
            return self._aperture_gain_at_actuator(at)
        if (frm, to) == ("aperture", "actuator"):
            gain = self._aperture_gain_at_actuator(self.aperture_to_actuator(at))
            return None if gain == 0.0 else 1.0 / gain
        return None

    @property
    def _aperture_angle_per_actuator(self) -> float:
        """
        d(aperture_angle)/d(actuator_angle), dimensionless.

        `actuator_to_aperture` maps the servo span onto the aperture-angle span by a constant
        `_map_range` ratio. Both sides are angles, so the ratio is unit-independent.
        """
        servo_closed_deg, servo_open_deg = self._range_deg
        servo_span_deg = servo_open_deg - servo_closed_deg
        if servo_span_deg == 0:
            return 0.0
        return self._aperture_open_deg / servo_span_deg

    def _aperture_gain_at_actuator(self, actuator: float) -> float:
        """
        d(aperture_m)/d(actuator_rad) for aperture = 2*R*sin(theta/2).

        d(theta)/d(actuator) is the constant `_map_range` ratio; the chord contributes
        d(aperture)/d(theta) = R*cos(theta/2).
        """
        theta_per_actuator = self._aperture_angle_per_actuator
        if theta_per_actuator == 0.0:
            return 0.0
        servo_closed_deg, servo_open_deg = self._range_deg
        aperture_angle_deg = self._map_range(
            rad_to_deg(actuator),
            servo_closed_deg,
            servo_open_deg,
            0.0,
            self._aperture_open_deg,
        )
        theta_rad = math.radians(aperture_angle_deg)
        return self._finger_length_m * math.cos(theta_rad / 2.0) * theta_per_actuator

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
        meters) to servo angle (radians). Note: this is a simplified model, not
        accurate to the gripper's real motion.
        """
        aperture_angle_deg = self._aperture_m_to_aperture_angle_degrees(aperture)
        servo_closed_deg, servo_open_deg = self._range_deg
        servo_angle_deg = self._map_range(
            aperture_angle_deg,
            0.0,
            self._aperture_open_deg,
            servo_closed_deg,
            servo_open_deg,
        )
        return deg_to_rad(servo_angle_deg)

    def actuator_to_aperture(self, actuator: float) -> float:
        """Converts servo angle (radians) to fingertip aperture (meters), the inverse of `aperture_to_actuator`."""
        servo_closed_deg, servo_open_deg = self._range_deg
        aperture_angle_deg = self._map_range(
            rad_to_deg(actuator),
            servo_closed_deg,
            servo_open_deg,
            0.0,
            self._aperture_open_deg,
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
            # Time derivative of finger_rad above. finger_rad is half the chord-model aperture
            # angle, not this tool's `urdf` unit type (actuator_to_urdf is the identity for SG4),
            # so it cannot route through the urdf conversions.
            "finger_vel": self._aperture_angle_per_actuator
            * status.get("vel", 0.0)
            / 2.0,
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
            RobotParams.add_user_tool_to_sys_path(self.tool_name)
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
        if self._client_class is None:
            raise ToolConfigurationError(
                f"No client class available for user tool '{self.tool_name}': set "
                "'client_module_name' and 'client_class_name' in robot_params."
            )
        return self._client_class

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
        RobotParams.add_user_tool_to_sys_path(self.tool_name)
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
        User tools have no YAML mechanism to describe an actuator/servo scale distinct from
        move_to()/move_by()'s own command units, so actuator is assumed to coincide with command
        -- see command_to_actuator/actuator_to_command.
        """
        return self._command_range

    @property
    def aperture_range(self) -> tuple[float, float]:
        return self._aperture_range

    # Every conversion this class defines is affine, so every edge has a constant gain.
    _LINEAR_CONVERSIONS = ToolMetadata._LINEAR_CONVERSIONS | frozenset(
        {
            ("urdf", "command"),
            ("command", "urdf"),
            ("command", "actuator"),
            ("actuator", "command"),
            ("aperture", "actuator"),
            ("actuator", "aperture"),
        }
    )

    def urdf_to_command(self, urdf: float) -> float:
        return urdf * self._urdf_scale

    def command_to_urdf(self, command: float) -> float:
        return command / self._urdf_scale if self._urdf_scale != 0 else command

    def command_to_actuator(self, command: float) -> float:
        """Identity: user tools assume the actuator range coincides with command (see actuator_range)."""
        return command

    def actuator_to_command(self, actuator: float) -> float:
        """Identity: user tools assume the actuator range coincides with command (see actuator_range)."""
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
            # Time derivative of finger_rad above.
            "finger_vel": self.actuator_to_urdf_velocity(
                status.get("vel", 0.0), actuator
            ),
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
        tool_params.get("metadata_module_name")
        and tool_params.get("metadata_class_name")
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
        RobotParams.add_user_tool_to_sys_path(tool_name)
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


def get_gripper_instance(
    direct: bool = False, ip_address: str | None = None
) -> tuple[Any, str] | tuple[None, None]:
    """
    Constructs and returns a tool instance (the driver for direct, client otherwise) along with its type name.
    """
    try:
        meta = get_tool_metadata()
    except Exception:
        return None, None

    gripper_type = meta.joint_name

    try:
        if direct:
            DriverClass = meta.driver_class
            g = DriverClass(is_direct=True)
        else:
            ClientClass = meta.client_class
            try:
                g = ClientClass(ip_address=ip_address)
            except TypeError:
                g = ClientClass()
    except Exception as e:
        raise ValueError(
            f"Failed to instantiate {'driver' if direct else 'client'} for tool '{gripper_type}': {e}"
        )

    return g, gripper_type
