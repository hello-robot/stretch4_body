#!/usr/bin/env python3
"""
Validates that a user tool's `tool_params.yaml` resolves to the classes it names.

Checks configuration only: nothing here talks to hardware, so it is safe to run with no robot
attached. Exits 0 when every check passes, 1 otherwise.
"""

import argparse
import os
import sys
import textwrap

import yaml

from stretch4_body.core.device import Device
from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.core.user_tool_paths import list_user_tools, user_tool_dirs
from stretch4_body.utils.stretch_pose_models import RobotPose
from stretch4_body.utils.tool_metadata import get_tool_metadata

_COLOR = sys.stdout.isatty() and not os.environ.get("NO_COLOR")
_RESET = "\033[0m" if _COLOR else ""
_BOLD = "\033[1m" if _COLOR else ""
_DIM = "\033[2m" if _COLOR else ""
_GREEN = "\033[32m" if _COLOR else ""
_RED = "\033[31m" if _COLOR else ""
_YELLOW = "\033[33m" if _COLOR else ""
_CYAN = "\033[36m" if _COLOR else ""


def _tag(color, label):
    """A fixed-width `[ LABEL ]` badge, colored and bolded when the terminal supports it."""
    return f"{_BOLD}{color}[ {label:<4} ]{_RESET}"


class _Report:
    """Collects pass/fail/skip lines for one tool."""

    def __init__(self):
        self.passed = True
        self.counts = {"PASS": 0, "FAIL": 0, "SKIP": 0}

    def _line(self, tag, title, description):
        print(f"  {tag}  {_BOLD}{title}{_RESET}: {_DIM}{description}{_RESET}")

    def ok(self, title, description):
        self.counts["PASS"] += 1
        self._line(_tag(_GREEN, "PASS"), title, description)

    def fail(self, title, description):
        self.counts["FAIL"] += 1
        self._line(_tag(_RED, "FAIL"), title, description)
        self.passed = False

    def skip(self, title, description):
        self.counts["SKIP"] += 1
        self._line(_tag(_YELLOW, "SKIP"), title, description)

    def note(self, message):
        print(f"            {message}")

    def para(self, message):
        """A note wrapped to the terminal, for explanations too long for one line."""
        for line in textwrap.wrap(" ".join(message.split()), width=92):
            print(f"            {line}")

    def summary(self):
        """One colored `N passed, N failed, N skipped` tally line for this tool."""
        parts = []
        if self.counts["PASS"]:
            parts.append(f"{_GREEN}{self.counts['PASS']} passed{_RESET}")
        if self.counts["FAIL"]:
            parts.append(f"{_RED}{self.counts['FAIL']} failed{_RESET}")
        if self.counts["SKIP"]:
            parts.append(f"{_YELLOW}{self.counts['SKIP']} skipped{_RESET}")
        print(f"  {', '.join(parts)}")


def check_tool(tool_name):
    """Runs every check against `tool_name`. Returns True when all of them pass."""
    print(f"\n{_BOLD}{_CYAN}{tool_name}{_RESET}")
    report = _Report()

    tool_path = RobotParams.get_user_defined_tool_path(tool_name)
    if not tool_path:
        report.fail(
            "Tool Directory",
            f"No directory named '{tool_name}' under {user_tool_dirs()}. Are "
            "HELLO_FLEET_PATH and HELLO_FLEET_ID set correctly?",
        )
        report.summary()
        return False
    report.ok("Tool Directory", f"found at {tool_path}")

    params_path = os.path.join(tool_path, "tool_params.yaml")
    if not os.path.exists(params_path):
        report.fail("Tool Configuration", f"not found in {tool_path}")
        report.summary()
        return False
    try:
        with open(params_path, "r") as f:
            params = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        report.fail("Tool Configuration", f"could not be read: {e}")
        report.summary()
        return False
    report.ok("Tool Configuration", "parses as valid YAML")

    RobotParams.reload()
    _, robot_params = RobotParams.get_params()
    if tool_name in robot_params.get("robot", {}).get("supported_eoa", []) or tool_name in robot_params:
        report.ok("Robot Registration", "tool was successfully registered")
    else:
        report.fail(
            "Robot Registration",
            "Tool is NOT registered after a reload. Directory not found in "
            f"user_tools: {user_tool_dirs()}.",
        )

    try:
        meta = get_tool_metadata(tool_name)
    except Exception as e:
        report.fail("Tool Metadata", f"could not resolve for '{tool_name}': {e}")
        report.summary()
        return False
    report.ok("Tool Metadata", f"resolved as {type(meta).__name__}")
    report.note(f"primary_joint={meta.primary_joint}")

    _check_tool_name(meta, params, tool_name, robot_params, report)
    try:
        report.note(
            f"command_range={meta.command_range}  aperture_range={meta.aperture_range}"
        )
    except Exception as e:
        report.fail("Command and aperture ranges", str(e))

    _report_resolved_modules(meta, params, report)

    _check_metadata_components(meta, report)
    _check_driver_components(meta, tool_name, report)
    _check_eoa_subsystem_class(tool_name, robot_params, report)
    _check_client_components(meta, report)

    _check_subsystem_client(params, tool_name, report)
    _check_pose_models(params, tool_name, report)
    _check_stow_position(tool_name, robot_params, report)
    _check_home_position(tool_name, robot_params, report)
    report.summary()
    return report.passed


def _callable_name(obj):
    """Display name for a class or a `functools.partial` wrapping one (e.g. `partial(ToolJointClient, self)`)."""
    name = getattr(obj, "__name__", None)
    if name:
        return name
    inner = getattr(obj, "func", None)
    return getattr(inner, "__name__", str(obj))


def _report_resolved_line(label, params, module_key, class_key, default_name, resolve, report):
    """
    Prints one line showing which module and class were used for a role (metadata, driver, or
    client), noting whether it came from tool_params.yaml or a built-in default.
    """
    declared_module = params.get(module_key)
    declared_class = params.get(class_key)
    if declared_module and declared_class:
        report.note(f"  {label}: {declared_module}.{declared_class} (user-defined)")
        return
    try:
        resolved = resolve()
        resolved = resolved if isinstance(resolved, type) else getattr(resolved, "func", resolved)
        name = getattr(resolved, "__name__", None) or str(resolved)
        module = getattr(resolved, "__module__", default_name)
        report.note(f"  {label}: {module}.{name} (default)")
    except Exception as e:
        report.note(f"  {label}: unresolved: {e}")


def _report_resolved_modules(meta, params, report):
    """
    Prints which module/class is actually used for metadata, driver, and client.
    """
    report.note("Resolved modules:")
    _report_resolved_line(
        "metadata", params, "metadata_module_name", "metadata_class_name",
        "stretch4_body.utils.tool_metadata", lambda: type(meta), report,
    )
    _report_resolved_line(
        "driver", params, "driver_module_name", "driver_class_name",
        "(none)", lambda: meta.driver_class, report,
    )
    _report_resolved_line(
        "client", params, "client_module_name", "client_class_name",
        "stretch4_body.robot.robot_client", lambda: meta.client_class, report,
    )


def _check_metadata_components(meta, report):
    """
    Exercises each of `ToolMetadata`'s required conversion methods with a real value.
    """
    try:
        urdf_mid = sum(meta.urdf_range) / 2.0
        command_mid = sum(meta.command_range) / 2.0
        actuator_mid = sum(meta.actuator_range) / 2.0
        aperture_mid = sum(meta.aperture_range) / 2.0
    except Exception as e:
        report.fail("Metadata conversion", f"a required range property raised: {e}")
        return

    checks = (
        ("urdf_to_command", lambda: meta.urdf_to_command(urdf_mid)),
        ("command_to_urdf", lambda: meta.command_to_urdf(command_mid)),
        ("command_to_actuator", lambda: meta.command_to_actuator(command_mid)),
        ("actuator_to_command", lambda: meta.actuator_to_command(actuator_mid)),
        ("aperture_to_actuator", lambda: meta.aperture_to_actuator(aperture_mid)),
        ("actuator_to_aperture", lambda: meta.actuator_to_aperture(actuator_mid)),
        (
            "status_to_metadata",
            lambda: meta.status_to_metadata(
                {"pos": actuator_mid, "vel": 0.0, "effort": 0.0}
            ),
        ),
    )
    failures = []
    for name, fn in checks:
        try:
            fn()
        except Exception as e:
            failures.append(f"{name}: {e}")

    if failures:
        report.fail("Metadata Conversion", "; ".join(failures))
    else:
        report.ok("Metadata Conversion", f"{len(checks)} methods run cleanly")


_REQUIRED_DRIVER_METHODS = ("move_to", "move_by", "home", "quick_stop")


def _check_driver_components(meta, tool_name, report):
    """
    The driver must subclass `Device` and provide move_to/move_by/home/quick_stop. Standard Hello Robot tools
    use Feetech motors with the custom FeetechSMHello superclass.
    """
    try:
        driver = meta.driver_class
    except Exception as e:
        report.fail("Driver", str(e))
        return

    if not (isinstance(driver, type) and issubclass(driver, Device)):
        report.fail(
            "Driver",
            f"'{driver.__module__}.{driver.__name__}' does not subclass Device. The "
            "status, params, and logger plumbing every subsystem relies on is not "
            "guaranteed.",
        )
    else:
        missing = [m for m in _REQUIRED_DRIVER_METHODS if not callable(getattr(driver, m, None))]
        if missing:
            report.fail(
                "Driver",
                f"'{driver.__module__}.{driver.__name__}' is missing required method(s) "
                f"{missing}. Gamepad, ROS command groups, and the collision-stop sentry "
                "call these on every end-of-arm joint.",
            )
        elif issubclass(driver, FeetechSMHello):
            report.ok("Driver", "subclasses FeetechSMHello")
        else:
            report.ok(
                "Driver",
                "subclasses Device and provides move_to/move_by/home/quick_stop "
                "(non-Feetech servo)",
            )

    _check_driver_instantiates(driver, tool_name, report)


def _check_driver_instantiates(driver, tool_name, report):
    """
    Constructs the driver to catch any unset tool_params.yaml keys or other errors that will crash the
    the whole `EndOfArmLoop` worker process on the real robot.
    """
    try:
        driver(chain=None)
    except Exception as e:
        report.fail("Driver Instantiation", f"{tool_name}(chain=None): {e}")
        return
    report.ok("Driver Instantiation", "succeeds with no hardware attached")


def _check_eoa_subsystem_class(tool_name, robot_params, report):
    """
    Check the top-level 'py_module_name'/'py_class_name' module. It should be an EndOfArm subclass
    constructed as `SomeClass(tool_name)`.
    """
    tool_params = robot_params.get(tool_name, {})
    module_name = tool_params.get("py_module_name")
    class_name = tool_params.get("py_class_name")
    if not module_name or not class_name:
        report.fail(
            "End-of-arm subsystem class",
            f"robot_params['{tool_name}'] has no driver class ('py_module_name'/"
            "'py_class_name'). Was one defined in tool_params.yaml?",
        )
        return

    try:
        module = RobotParams.import_user_tool_module(tool_name, module_name, is_server=True)
        EoaClass = getattr(module, class_name)
    except Exception as e:
        report.fail(
            "End-of-arm subsystem class", f"could not import '{class_name}' from '{module_name}': {e}"
        )
        return

    from stretch4_body.subsystem.end_of_arm.end_of_arm import EndOfArm

    if not (isinstance(EoaClass, type) and issubclass(EoaClass, EndOfArm)):
        report.fail(
            "End-of-arm subsystem class",
            f"Top-level '{class_name}' does not subclass EndOfArm. It will be "
            f"constructed as {class_name}('{tool_name}') and must manage a chain of "
            "servos, one per 'devices' entry, not act as a single servo's own driver. "
            "Leave 'py_module_name'/'py_class_name' unset to inherit the default "
            "EOA_Wrist_DW4_Tool_NIL, and name your driver only under a 'devices' entry "
            "(or 'driver_module_name'/'driver_class_name').",
        )
        return

    try:
        EoaClass(tool_name)
    except Exception as e:
        report.fail(
            "End-of-arm subsystem class", f"instantiation failed: {class_name}('{tool_name}'): {e}"
        )
        return
    report.ok("End-of-arm subsystem class", f"instantiates: {class_name}('{tool_name}')")


def _check_client_components(meta, report):
    """
    The resolved client must subclass `WristJointClient`, which guarantees move_to, move_by,
    pose, and status exist for application code and the gamepad to call.
    """
    from stretch4_body.robot.robot_client import WristJointClient

    try:
        client = meta.client_class
    except Exception as e:
        report.fail("Client", str(e))
        return

    client_type = client if isinstance(client, type) else getattr(client, "func", None)
    if isinstance(client_type, type) and issubclass(client_type, WristJointClient):
        report.ok("Client", f"subclasses WristJointClient ({client_type.__name__})")
    else:
        report.fail(
            "Client",
            f"'{_callable_name(client)}' does not subclass WristJointClient. move_to, "
            "move_by, pose, and status are not guaranteed.",
        )


def _check_tool_name(meta, params, tool_name, robot_params, report):
    """`ToolMetadata.tool_name` has to name one of the tool's servos, not one of its URDF joints."""
    devices = robot_params.get(tool_name, {}).get("devices", {})
    try:
        declared = meta.tool_name
    except Exception as e:
        report.fail("Tool name", str(e))
        _report_servos(devices, report)
        return

    if declared in devices:
        report.ok("Tool name", f"'{declared}' names a servo on the wrist bus")
        return

    report.fail("Tool name", f"'{declared}' does not name a servo on the wrist bus")
    _report_servos(devices, report)
    if declared == meta.primary_joint:
        report.para(
            f"'{declared}' is a URDF joint name, this tool's primary_joint."
        )


def _report_servos(devices, report):
    report.para(
        f"No 'devices' entry in tool_params.yaml matches tool_name (only "
        f"{', '.join(sorted(devices))} are defined). Fix: add a devices entry keyed to "
        "tool_name's value, naming this tool's own servo."
    )


def _check_subsystem_client(params, tool_name, report):
    """`client_class_name` may name the EndOfArmClient subclass RobotClient installs."""
    from stretch4_body.robot.robot_client import EndOfArmClient

    module_name = params.get("client_module_name")
    class_name = params.get("client_class_name")
    if not (module_name or class_name):
        report.ok("Subsystem Client", "EndOfArmClient (generic)")
        return
    if not (module_name and class_name):
        report.fail(
            "Subsystem Client", "'client_module_name' and 'client_class_name' must be set together"
        )
        return
    try:
        module = RobotParams.import_user_tool_module(
            tool_name, module_name, is_server=False
        )
        declared = getattr(module, class_name)
    except Exception as e:
        report.fail("Subsystem Client", f"could not import '{class_name}' from '{module_name}': {e}")
        return
    if isinstance(declared, type) and issubclass(declared, EndOfArmClient):
        report.ok("Subsystem Client", class_name)
    else:
        report.ok("Subsystem Client", "EndOfArmClient (generic)")


def _check_pose_models(params, tool_name, report):
    """`pose_models` is optional. Most tools have none, so an absent key is a SKIP, not a FAIL."""
    if "pose_models" not in params:
        report.skip("Pose models", "none declared in tool_params.yaml (optional)")
        return
    try:
        poses = RobotPose.load_tool_pose_models(tool_name)
    except Exception as e:
        report.fail("Pose models", f"failed to load: {e}")
        return
    report.ok("Pose models", f"{len(poses)} pose(s) loaded: {', '.join(sorted(poses))}")


def _check_stow_position(tool_name, robot_params, report):
    """
    The default end-of-arm subsystem never moves the gripper during stow. Only a custom
    EndOfArm subclass with its own stow() override can stow a gripper.
    """
    tool_params = robot_params.get(tool_name, {})
    module_name = tool_params.get("py_module_name")
    class_name = tool_params.get("py_class_name")
    if not module_name or not class_name:
        return

    try:
        module = RobotParams.import_user_tool_module(tool_name, module_name, is_server=True)
        EoaClass = getattr(module, class_name)
    except Exception:
        return

    from stretch4_body.subsystem.end_of_arm.end_of_arm_tools import (
        EOA_Wrist_DW4_Tool_NIL,
    )

    if getattr(EoaClass, "stow", None) is EOA_Wrist_DW4_Tool_NIL.stow:
        report.skip(
            "Stow Position",
            "the gripper is not given a stow pose, it will not change position during the stow action",
        )
    else:
        report.ok(
            "Stow Position", f"{class_name} specifies a stow pose"
        )


def _check_home_position(tool_name, robot_params, report):
    """Without a 'homing' entry, the tool's own joint homes to command=0, not fully open or closed."""
    homing = robot_params.get(tool_name, {}).get("homing", {})
    if tool_name in homing:
        report.ok("Home Position", f"homes to {homing[tool_name]}")
    else:
        report.skip(
            "Home Position",
            "the gripper is not given a home pose, defaults to a 0 command",
        )


def main():
    parser = argparse.ArgumentParser(
        description="Validate a user tool's configuration."
    )
    parser.add_argument(
        "tool_name",
        nargs="?",
        help="User tool to check. Defaults to the configured tool.",
    )
    parser.add_argument(
        "--all", action="store_true", help="Check every installed user tool."
    )
    args = parser.parse_args()

    if args.all:
        tools = list_user_tools()
        if not tools:
            print("No user tools are installed.")
            return 0
    elif args.tool_name:
        tools = [args.tool_name]
    else:
        _, robot_params = RobotParams.get_params()
        configured = robot_params.get("robot", {}).get("tool")
        if not RobotParams.get_user_defined_tool_path(configured):
            print(f"The configured tool '{configured}' is a built-in tool.")
            return 0
        tools = [configured]

    failed = [name for name in tools if not check_tool(name)]
    print()
    if failed:
        print(
            f"{_BOLD}{_RED}{len(failed)} of {len(tools)} tool(s) failed validation: "
            f"{', '.join(failed)}{_RESET}"
        )
        return 1
    print(f"{_BOLD}{_GREEN}All checks passed ({len(tools)} tool(s)).{_RESET}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
