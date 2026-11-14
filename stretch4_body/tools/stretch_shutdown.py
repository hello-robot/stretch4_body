#!/usr/bin/env python3
"""Sleep, reboot or power off the robot, as the power button does.

Stops the Stretch Body Server first, then talks to the power board directly,
so a server in a bad state can't get in the way.
"""
import sys
import subprocess
import argparse
import click
import stretch4_body.core.hello_utils as hu

hu.print_stretch_re_use()

# option: (power_periph method, what it does)
ACTIONS = {
    'sleep': ('trigger_robot_sleep',
              'This computer shuts down and the robot goes to sleep. The power button\n'
              'pulses yellow, then red. Tap the power button to wake the robot.'),
    'reboot': ('trigger_robot_reboot',
               'This computer shuts down, the robot sleeps for a moment, then it wakes by\n'
               'itself and this computer starts again. The power button pulses yellow\n'
               'until the robot is back up.'),
    'power_off': ('trigger_robot_shutdown',
                  'This computer shuts down, then the robot turns off fully, as after a long\n'
                  'press of the power button. With the charger connected it stays off and\n'
                  'charging. Press the power button to turn it on again.'),
}

KILL_TIMEOUT_S = 30

parser = argparse.ArgumentParser(description='Sleep, reboot or power off the robot')
action = parser.add_mutually_exclusive_group(required=True)
action.add_argument('--sleep', action='store_true', help='Shut down this computer and put the robot to sleep')
action.add_argument('--reboot', action='store_true', help='Shut down this computer, then wake the robot and start it again')
action.add_argument('--power_off', action='store_true', help='Shut down this computer, then turn the robot off fully')
parser.add_argument('-y', '--yes', action='store_true', help="Don't ask for confirmation")
args = parser.parse_args()

name = next(a for a in ACTIONS if getattr(args, a))
method, description = ACTIONS[name]


def stop_server():
    """Stop the Stretch Body Server so the power board's port is free. If a
    normal stop fails or hangs, force kill its processes."""
    print('Stopping the Stretch Body Server...')
    try:
        if subprocess.run(['stretch_body_server', '--kill'], timeout=KILL_TIMEOUT_S).returncode == 0:
            return
    except subprocess.TimeoutExpired:
        print('Stopping the Stretch Body Server timed out.')
    except FileNotFoundError:
        print('stretch_body_server was not found. Continuing.')
        return
    print('Force killing the Stretch Body Server...')
    try:
        subprocess.run(['stretch_body_server', '--cleanup'], timeout=KILL_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        print('Force killing the Stretch Body Server timed out. Trying the power board anyway.')


def send_direct():
    from stretch4_body.subsystem.power_periph import PowerPeriph
    p = PowerPeriph()
    if not p.startup():
        return False
    try:
        getattr(p, method)()
        p.push_command()
    finally:
        p.stop()
    return True


print(description)
print('The Stretch Body Server is stopped first.')
print('')
if not args.yes and not click.confirm('%s the robot now?' % name.replace('_', ' ').capitalize(), default=False):
    print('Cancelled. Nothing was changed.')
    sys.exit(1)

stop_server()
print('Sending to the power board...')
if not send_direct():
    print('Failed to reach the power board. Please run `stretch_system_check`.')
    sys.exit(1)
print('Done. This computer will shut down in a few seconds.')
