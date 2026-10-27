"""Helpers for making sure the lift is high enough for the end-of-arm tool to move freely, for tools like stretch_dex_wrist_home and stretch_configure_tool."""

import time

import click

LIFT_CLEARANCE_MIN_M = 0.35


def get_lift_clearance(robot_params, tool):
    """Lift height (m) the tool needs to clear the base: the tool's stow lift_prestow, but never below LIFT_CLEARANCE_MIN_M."""
    lift_prestow = robot_params.get(tool, {}).get('stow', {}).get('lift_prestow', LIFT_CLEARANCE_MIN_M)
    return max(lift_prestow, LIFT_CLEARANCE_MIN_M)


def ensure_lift_clearance(lift, root=None, tool=None, timeout=15.0):
    """
    If the lift is below the clearance the tool needs, ask to raise it there.

    `lift`: a Lift, LiftClient, or a robot's lift subsystem.
    `root`: the object to pull status / push commands through (eg, the Robot or RobotClient that owns `lift`). Defaults to `lift`.
    `tool`: the tool to get the clearance for. Defaults to the configured tool.
    Returns True if the lift is (now) high enough, False if the user declined or the lift could not get there.
    """
    if root is None:
        root = lift
    if tool is None:
        tool = lift.robot_params['robot']['tool']
    lift_min = get_lift_clearance(lift.robot_params, tool)

    root.pull_status()
    if not lift.is_homed():
        print(f'The lift is not homed, so it cannot be raised to the {lift_min:.3f} m needed to clear the tool. Run stretch_lift_home first.')
        return False

    lift_pos = lift.status['pos']
    if lift_pos >= lift_min:
        return True

    print(f'The lift is at {lift_pos:.3f} m, below the {lift_min:.3f} m needed for the tool to clear the base.')
    if not click.confirm(f'Raise the lift to {lift_min:.3f} m automatically?', default=True):
        print('Aborted. Raise the lift manually and try again.')
        return False

    print('Raising lift...')
    if lift.move_to(lift_min) is False:
        print('Failed to command the lift.')
        return False
    root.push_command()

    ts = time.time()
    while time.time() - ts < timeout:
        time.sleep(0.1)
        root.pull_status()
        if lift.status['pos'] >= lift_min - 0.01:
            return True
    print(f'The lift did not reach {lift_min:.3f} m (at {lift.status["pos"]:.3f} m). Aborting.')
    return False
