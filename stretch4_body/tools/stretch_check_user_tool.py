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

from stretch4_body.core.robot_params import RobotParams
from stretch4_body.core.user_tool_paths import list_user_tools, user_tool_dirs
from stretch4_body.utils.stretch_pose_models import RobotPose
from stretch4_body.utils.tool_metadata import get_tool_metadata


class _Report:
    """Collects pass/fail lines for one tool."""

    def __init__(self):
        self.passed = True

    def ok(self, message):
        print(f"  PASS  {message}")

    def fail(self, message):
        print(f"  FAIL  {message}")
        self.passed = False

    def note(self, message):
        print(f"        {message}")

    def para(self, message):
        """A note wrapped to the terminal, for explanations too long for one line."""
        for line in textwrap.wrap(" ".join(message.split()), width=92):
            print(f"        {line}")


def check_tool(tool_name):
    """Runs every check against `tool_name`. Returns True when all of them pass."""
    print(f"\n{tool_name}")
    report = _Report()

    tool_path = RobotParams.get_user_defined_tool_path(tool_name)
    if not tool_path:
        report.fail(
            f"no directory named '{tool_name}' under {user_tool_dirs()}"
        )
        return False
    report.ok(f"tool directory: {tool_path}")

    params_path = os.path.join(tool_path, "tool_params.yaml")
    if not os.path.exists(params_path):
        report.fail("tool_params.yaml not found")
        return False
    try:
        with open(params_path, "r") as f:
            params = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as e:
        report.fail(f"tool_params.yaml could not be read: {e}")
        return False
    report.ok("tool_params.yaml parses")

    RobotParams.reload()
    _, robot_params = RobotParams.get_params()
    if tool_name in robot_params.get("robot", {}).get("supported_eoa", []) or tool_name in robot_params:
        report.ok("registered in robot params")
    else:
        report.fail("not registered in robot params after reload")

    try:
        meta = get_tool_metadata(tool_name)
    except Exception as e:
        report.fail(f"get_tool_metadata: {e}")
        return False
    report.note(f"primary_joint={meta.primary_joint}")

    _check_tool_name(meta, params, tool_name, robot_params, report)
    report.note(
        f"command_range={meta.command_range}  aperture_range={meta.aperture_range}"
    )

    _report_resolved_modules(meta, params, report)

    _check_metadata_components(meta, report)
    _check_driver_components(meta, tool_name, report)
    _check_client_components(meta, report)

    _check_subsystem_client(params, tool_name, report)
    _check_pose_models(params, tool_name, report)
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
    Prints one `label: module.Class (user-defined|default)` line. When `tool_params.yaml`
    declared the module/class pair, that's printed verbatim -- `resolve()`'s own `__module__`
    would instead show the synthesized `sys.modules` key `import_user_tool_module` loads it
    under (e.g. `user_tool_client_nyu_gripper_nyu_gripper_metadata`), not the file it's actually
    in. For the default case there's no declared name to fall back on, so `resolve()` is used.
    """
    declared_module = params.get(module_key)
    declared_class = params.get(class_key)
    if declared_module and declared_class:
        report.note(f"  {label}: {declared_module}.{declared_class} (user-defined)")
        return
    try:
        resolved = resolve()
        # Unwrap a `functools.partial(SomeClass, ...)` (e.g. the default tool joint client) to
        # the class itself, so __module__/__name__ describe SomeClass, not `functools`.
        resolved = resolved if isinstance(resolved, type) else getattr(resolved, "func", resolved)
        name = getattr(resolved, "__name__", None) or str(resolved)
        module = getattr(resolved, "__module__", default_name)
        report.note(f"  {label}: {module}.{name} (default)")
    except Exception as e:
        report.note(f"  {label}: unresolved -- {e}")


def _report_resolved_modules(meta, params, report):
    """
    Prints which module/class is actually used for metadata, driver, and client -- what
    tool_params.yaml named (`metadata_module_name`/`driver_module_name`/`client_module_name`
    and their `_class_name` pairs), or the built-in fallback (`LinearToolMetadata`, none, and
    `ToolJointClient` respectively) when it named nothing.
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
    Exercises each of `ToolMetadata`'s required conversion methods with a real value from this
    tool's own ranges. Python's ABC machinery already guarantees these methods exist once `meta`
    constructs; this catches one that exists but reaches for a robot_params key tool_params.yaml
    never set and raises only when actually called.
    """
    try:
        urdf_mid = sum(meta.urdf_range) / 2.0
        command_mid = sum(meta.command_range) / 2.0
        actuator_mid = sum(meta.actuator_range) / 2.0
        aperture_mid = sum(meta.aperture_range) / 2.0
    except Exception as e:
        report.fail(f"metadata: a required range property raised -- {e}")
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
        report.fail("metadata: " + "; ".join(failures))
    else:
        report.ok(f"metadata conversion methods run cleanly ({len(checks)} checked)")


def _check_driver_components(meta, tool_name, report):
    """
    The driver must subclass `FeetechSMHello` -- the base class every joint on the wrist chain
    is built from, which is what actually guarantees `move_to`/`move_by`/`home`/`quick_stop`/
    `pull_status` exist -- and must construct with no hardware attached.
    """
    try:
        driver = meta.driver_class
    except Exception as e:
        report.fail(f"driver_class: {e}")
        return

    from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello

    if isinstance(driver, type) and issubclass(driver, FeetechSMHello):
        report.ok("driver subclasses FeetechSMHello")
    else:
        report.fail(
            f"driver '{driver.__module__}.{driver.__name__}' does not subclass FeetechSMHello -- "
            "move_to/move_by/home/quick_stop/pull_status are not guaranteed"
        )

    _check_driver_instantiates(driver, tool_name, report)


def _check_driver_instantiates(driver, tool_name, report):
    """
    Constructs the driver exactly as `FeetechSMChain.startup()` does -- `driver(chain=...)`, no
    other arguments -- with `chain=None` since this check runs with no hardware attached. Catches
    a driver whose `__init__` reaches for a robot_params key `tool_params.yaml` never set -- e.g.
    a driver that expects `self.params['gripper_conversion']` -- which otherwise only surfaces
    when `FeetechSMChain.startup()` hits it inside the `EndOfArmLoop` worker process and takes
    the whole end-of-arm chain down with it.
    """
    try:
        driver(chain=None)
    except Exception as e:
        report.fail(f"driver instantiation ({tool_name}(chain=None)): {e}")
        return
    report.ok("driver instantiates with no hardware attached")


def _check_client_components(meta, report):
    """
    The resolved per-joint client -- the generic `ToolJointClient` default, or a bespoke
    override -- must subclass `WristJointClient`, which is what guarantees `move_to`/`move_by`/
    `pose`/`status` exist for application code and the gamepad to call.
    """
    from stretch4_body.robot.robot_client import WristJointClient

    try:
        client = meta.client_class
    except Exception as e:
        report.fail(f"client_class: {e}")
        return

    client_type = client if isinstance(client, type) else getattr(client, "func", None)
    if isinstance(client_type, type) and issubclass(client_type, WristJointClient):
        report.ok(f"client subclasses WristJointClient ({client_type.__name__})")
    else:
        report.fail(
            f"client '{_callable_name(client)}' does not subclass WristJointClient -- "
            "move_to/move_by/pose/status are not guaranteed"
        )


def _check_tool_name(meta, params, tool_name, robot_params, report):
    """`ToolMetadata.tool_name` has to name one of the tool's servos, not one of its URDF joints."""
    devices = robot_params.get(tool_name, {}).get("devices", {})
    try:
        declared = meta.tool_name
    except Exception as e:
        report.fail(str(e))
        _report_servos(devices, report)
        return

    if declared in devices:
        report.ok(f"tool_name '{declared}' names a servo on the wrist bus")
        return

    report.fail(f"tool_name '{declared}' does not name a servo on the wrist bus")
    _report_servos(devices, report)
    if declared == meta.primary_joint:
        report.para(
            f"'{declared}' is a URDF joint name, this tool's primary_joint."
        )


def _report_servos(devices, report):
    report.para(
        f"tool_name must be one of this tool's servos: {', '.join(sorted(devices))}. Those are "
        "the keys of the 'devices' block in tool_params.yaml, merged over the three wrist joints "
        "every tool inherits. The name is how the rest of the stack reaches this tool: it keys "
        "the tool's entry in status['end_of_arm'], and names the joint ToolJointClient sends "
        "move_to/move_by to."
    )


def _check_subsystem_client(params, tool_name, report):
    """`client_class_name` may name the EndOfArmClient subclass RobotClient installs."""
    from stretch4_body.robot.robot_client import EndOfArmClient

    module_name = params.get("client_module_name")
    class_name = params.get("client_class_name")
    if not (module_name or class_name):
        report.ok("subsystem client: EndOfArmClient (generic)")
        return
    if not (module_name and class_name):
        report.fail(
            "'client_module_name' and 'client_class_name' must be set together"
        )
        return
    try:
        module = RobotParams.import_user_tool_module(
            tool_name, module_name, is_server=False
        )
        declared = getattr(module, class_name)
    except Exception as e:
        report.fail(f"could not import '{class_name}' from '{module_name}': {e}")
        return
    if isinstance(declared, type) and issubclass(declared, EndOfArmClient):
        report.ok(f"subsystem client: {class_name}")
    else:
        report.ok("subsystem client: EndOfArmClient (generic)")


def _check_pose_models(params, tool_name, report):
    if "pose_models" not in params:
        return
    try:
        poses = RobotPose.load_tool_pose_models(tool_name)
    except Exception as e:
        report.fail(f"pose_models: {e}")
        return
    report.ok(f"pose_models: {len(poses)} pose(s) -- {', '.join(sorted(poses))}")


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
        print(f"FAILED: {', '.join(failed)}")
        return 1
    print(f"All checks passed ({len(tools)} tool(s)).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
