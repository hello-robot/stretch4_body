"""Helpers for keeping the lift safe around end-of-arm tool moves and swaps, for tools like stretch_dex_wrist_home and stretch_configure_tool."""

import time

import click

LIFT_CLEARANCE_MIN_M = 0.35


def get_lift_clearance(robot_params, tool):
    """Lift height (m) the tool needs to clear the base: the tool's stow lift_prestow, but never below LIFT_CLEARANCE_MIN_M."""
    lift_prestow = robot_params.get(tool, {}).get('stow', {}).get('lift_prestow', LIFT_CLEARANCE_MIN_M)
    return max(lift_prestow, LIFT_CLEARANCE_MIN_M)


def hold_lift_stiff(stiffness=1.0):
    """
    Make the lift hold its position stiffly while in safety mode, over the direct API (the server must be stopped).

    Normally the lift only counterbalances in safety mode, using a feedforward sized for the configured tool, so it droops
    when a heavier tool is attached until the server restarts and applies the new tool's feedforward.
    This lasts until the lift gains are next pushed, eg, when the server or a direct Robot starts up.
    Returns True on success.
    """
    from stretch4_body.core.robot_params import RobotParams
    from stretch4_body.subsystem.lift import Lift

    if 'lift' not in RobotParams.get_params()[1]['robot']['subsystems']:
        return True
    l = Lift()
    # Set before startup, so the first gains pushed already hold
    l.motor.gains['safety_hold'] = 1
    l.motor.gains['safety_stiffness'] = stiffness
    if not l.startup():
        print('Could not connect to the lift to hold it in place.')
        return False
    l.stop() # Leaves the lift in safety mode, now holding position
    return True


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
