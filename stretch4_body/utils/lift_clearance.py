"""Helpers for keeping the lift safe around end-of-arm tool moves and swaps, for tools like stretch_dex_wrist_home and stretch_configure_tool."""

import time

import click

LIFT_CLEARANCE_MIN_M = 0.35


def get_lift_clearance(robot_params, tool):
    """Lift height (m) the tool needs to clear the base: the tool's stow lift_prestow, but never below LIFT_CLEARANCE_MIN_M."""
    lift_prestow = robot_params.get(tool, {}).get('stow', {}).get('lift_prestow', LIFT_CLEARANCE_MIN_M)
    return max(lift_prestow, LIFT_CLEARANCE_MIN_M)


def set_lift_safety_hold(lift, stiffness=1.0):
    """
    Make a direct API lift hold its position stiffly once it is in safety mode, eg, after lift.stop().

    Normally the lift only counterbalances in safety mode, using a feedforward sized for the configured tool, so it droops
    when the tool is heavier than that, or when nothing has applied the tool's feedforward yet (eg, a stopped server).
    The gains are pushed on the lift's next push_command(), which stop() does, and last until the lift gains are next
    pushed, eg, when the server or a direct Robot starts up.
    """
    lift.motor.gains['safety_hold'] = 1
    lift.motor.gains['safety_stiffness'] = stiffness
    lift.motor.set_gains()


def hold_lift_stiff(stiffness=1.0):
    """
    Make the lift hold its position stiffly while in safety mode, over the direct API (the server must be stopped).
    See set_lift_safety_hold(). Returns True on success.
    """
    from stretch4_body.core.robot_params import RobotParams
    from stretch4_body.subsystem.lift import Lift

    if 'lift' not in RobotParams.get_params()[1]['robot']['subsystems']:
        return True
    l = Lift()
    set_lift_safety_hold(l, stiffness) # Set before startup, so the first gains pushed already hold
    if not l.startup():
        print('Could not connect to the lift to hold it in place.')
        return False
    l.stop() # Leaves the lift in safety mode, now holding position
    return True


def _refresh_status(root):
    # A direct Robot's status thread already keeps the lift status fresh, and pulling
    # from this thread as well would compete with it for the serial ports
    body_thread = getattr(root, 'body_thread', None)
    if body_thread is not None and body_thread.is_alive():
        return
    root.pull_status()


def ensure_lift_clearance(lift, root, tool=None, timeout=15.0):
    """
    Make sure the lift is high enough for the tool to move without hitting the base, asking to raise it if not.
    When it is, the lift is left actively holding its position, so it doesn't droop under the tool while the tool moves.

    `lift`: a robot's lift subsystem.
    `root`: the Robot or RobotClient that owns `lift`, to pull status / push commands through. A Robot is needed rather than
    a standalone direct Lift, as the lift runs in sync mode and only moves on the motor sync that Robot.push_command() sends.
    `tool`: the tool to get the clearance for. Defaults to the configured tool.
    Returns True if the lift is (now) high enough, False if the user declined or the lift could not get there.
    """
    if tool is None:
        tool = lift.robot_params['robot']['tool']
    lift_min = get_lift_clearance(lift.robot_params, tool)

    _refresh_status(root)
    if not lift.is_homed():
        print(f'The lift is not homed, so it cannot be raised to the {lift_min:.3f} m needed to clear the tool. Run stretch_lift_home first.')
        return False

    lift_pos = lift.status['pos']
    if lift_pos >= lift_min:
        lift.move_to(lift_pos)
        root.push_command()
        return True

    print(f'The lift is at {lift_pos:.3f} m, below the {lift_min:.3f} m needed for the tool to clear the base.')
    if not click.confirm(f'Raise the lift to {lift_min:.3f} m automatically?', default=True):
        print('Not raising the lift. Raise it manually and try again.')
        return False

    print('Raising lift...')
    if lift.move_to(lift_min) is False:
        print('Failed to command the lift.')
        return False
    root.push_command()

    ts = time.time()
    while time.time() - ts < timeout:
        time.sleep(0.1)
        _refresh_status(root)
        if lift.status['pos'] >= lift_min - 0.01:
            return True
    print(f'The lift did not reach {lift_min:.3f} m (at {lift.status["pos"]:.3f} m). Aborting.')
    return False
