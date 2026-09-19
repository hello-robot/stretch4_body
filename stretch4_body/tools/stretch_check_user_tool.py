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
    report.ok(f"metadata: {type(meta).__name__}")
    report.note(f"primary_joint={meta.primary_joint}")

    _check_tool_name(meta, params, tool_name, robot_params, report)
    report.note(
        f"command_range={meta.command_range}  aperture_range={meta.aperture_range}"
    )

    try:
        driver = meta.driver_class
        report.ok(f"driver class: {driver.__module__}.{driver.__name__}")
    except Exception as e:
        report.fail(f"driver_class: {e}")

    try:
        client = meta.client_class
        name = getattr(client, "__name__", None)
        if name:
            report.ok(f"tool joint client: {name}")
        else:
            report.ok("tool joint client: ToolJointClient (generic)")
    except Exception as e:
        report.fail(f"client_class: {e}")

    _check_subsystem_client(params, tool_name, report)
    _check_pose_models(tool_path, tool_name, report)
    return report.passed

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


def _check_pose_models(tool_path, tool_name, report):
    if not os.path.exists(os.path.join(tool_path, "pose_models.yaml")):
        return
    try:
        poses = RobotPose.load_tool_pose_models(tool_name)
    except Exception as e:
        report.fail(f"pose_models.yaml: {e}")
        return
    report.ok(f"pose_models.yaml: {len(poses)} pose(s) -- {', '.join(sorted(poses))}")


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
        if not RobotParams.is_user_defined_tool(configured):
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
