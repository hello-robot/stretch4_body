#!/usr/bin/env python3
"""Sleep, reboot or power off the robot, as the power button does.

Goes through the Stretch Body Server when it is running, and talks to the
power board directly when it is not.
"""
import sys
import time
import argparse
import click
import stretch4_body.core.hello_utils as hu
from stretch4_body.utils.server_power import is_server_running

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

parser = argparse.ArgumentParser(description='Sleep, reboot or power off the robot')
action = parser.add_mutually_exclusive_group(required=True)
action.add_argument('--sleep', action='store_true', help='Shut down this computer and put the robot to sleep')
action.add_argument('--reboot', action='store_true', help='Shut down this computer, then wake the robot and start it again')
action.add_argument('--power_off', action='store_true', help='Shut down this computer, then turn the robot off fully')
parser.add_argument('-y', '--yes', action='store_true', help="Don't ask for confirmation")
args = parser.parse_args()

name = next(a for a in ACTIONS if getattr(args, a))
method, description = ACTIONS[name]


def send_via_server():
    from stretch4_body.robot.robot_client import RobotClient
    r = RobotClient()
    if not r.startup(verbose=False):
        return False
    try:
        getattr(r.power_periph, method)()
        # A power change goes through whichever program is driving the robot,
        # as the power button does
        r.push_command(ignore_control_lock=True)
        time.sleep(0.5)  # Let the server pick the command up before disconnecting
    finally:
        r.stop()
    return True


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
print('')
if not args.yes and not click.confirm('%s the robot now?' % name.replace('_', ' ').capitalize(), default=False):
    print('Cancelled. Nothing was changed.')
    sys.exit(1)

if is_server_running():
    print('Sending through the Stretch Body Server...')
    ok = send_via_server()
else:
    print('Stretch Body Server is not running. Sending directly to the power board...')
    ok = send_direct()

if not ok:
    print('Failed to reach the power board. Please run `stretch_system_check`.')
    sys.exit(1)
print('Done. This computer will shut down in a few seconds.')
