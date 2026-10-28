#!/usr/bin/env python3
from __future__ import print_function
import os
import sys
import time
import argparse
import click
import stretch4_body.core.hello_utils as hu
from stretch4_body.utils.server_power import stop_server_for_power_change, resume_server
hu.print_stretch_re_use()


parser=argparse.ArgumentParser(description='Control the power of an actuator device')

parser.add_argument("--lift", help="Power cycle lift", action="store_true")
parser.add_argument("--arm", help="Power cycle arm", action="store_true")
parser.add_argument("--omni_0", help="Power cycle omni wheel 0", action="store_true")
parser.add_argument("--omni_1", help="Power cycle omni wheel 1", action="store_true")
parser.add_argument("--omni_2", help="Power cycle omni wheel 2", action="store_true")
parser.add_argument("--eoa", help="Power cycle eoa servos", action="store_true")
parser.add_argument("--all", help="Power cycle all actuators", action="store_true")
parser.add_argument("--action", type=str, default='cycle', help='Action to take: on / off / [cycle]')
parser.add_argument("--off_time", type=float, default=0.5, help='Seconds to stay powered off during a cycle [0.5]. Increase if a board does not re-enumerate on USB.')
parser.add_argument("-y", "--yes", help="Don't prompt before stopping and starting the stretch_body_server", action="store_true")

args, _ = parser.parse_known_args()

# (cli flag, name used by actuator_control, key in status['periph_power_state'], udev device node)
ACTUATORS = [('lift',   'lift',   'power_to_lift',          '/dev/hello-motor-lift'),
             ('arm',    'arm',    'power_to_arm',           '/dev/hello-motor-arm'),
             ('omni_0', 'omni-0', 'power_to_omni_0_motor',  '/dev/hello-motor-omni-0'),
             ('omni_1', 'omni-1', 'power_to_omni_1_motor',  '/dev/hello-motor-omni-1'),
             ('omni_2', 'omni-2', 'power_to_omni_2_motor',  '/dev/hello-motor-omni-2'),
             ('eoa',    'eoa',    'power_to_eoa',           '/dev/hello-feetech-wrist')]


def read_power_state(p, key):
    """Return the board's reported power state for `key`, or None if unavailable."""
    return p.status.get('periph_power_state', {}).get(key, None)


def wait_on_power_state(p, key, enable, timeout=2.0):
    ts = time.time()
    while time.time() - ts < timeout:
        p.pull_status()
        if read_power_state(p, key) == enable:
            return True
        time.sleep(0.05)
    return False


def wait_on_device(device, present, timeout=8.0):
    """Wait for a udev device node to appear (or disappear) after a power change."""
    ts = time.time()
    while time.time() - ts < timeout:
        if os.path.exists(device) == present:
            return True
        time.sleep(0.1)
    return False


def set_power(p, name, key, enable, n_retry=3):
    """Command power on/off for a single actuator and confirm it took effect."""
    verb = 'on' if enable else 'off'
    p.pull_status()
    was = read_power_state(p, key)
    if was == enable:
        # Powering on a rail that is already on is not a power cycle: a board
        # hung with its rail up never loses power, so it never reboots.
        print('Powering %s %s... (already %s)' % (verb, name, verb))
    else:
        print('Powering %s %s...' % (verb, name))

    if was is None:
        # Board doesn't report this actuator's power state, so send it blind.
        p.actuator_control(name, enable=enable)
        time.sleep(0.2)
        return True

    for i in range(n_retry):
        p.actuator_control(name, enable=enable)
        if wait_on_power_state(p, key, enable):
            return True
        print('  ...command did not take, retrying (%d/%d)' % (i + 1, n_retry))
    print('  WARNING: unable to confirm power %s for %s' % (verb, name))
    return False


selected = [a for a in ACTUATORS if args.all or getattr(args, a[0])]
if not selected:
    print('No actuator selected. Pick --all or one of %s.' % ', '.join('--' + a[0] for a in ACTUATORS))
    sys.exit(1)

if args.action not in ('cycle', 'on', 'off'):
    print('Unrecognized action "%s". Expected one of: on / off / cycle.' % args.action)
    sys.exit(1)

# Power is always changed through the direct API with the server stopped. See
# stretch4_body/utils/server_power.py for why the server cannot do this.
ok, server_was_running = stop_server_for_power_change(assume_yes=args.yes)
if not ok:
    print('Aborted. Actuator power was not changed.')
    sys.exit(1)

from stretch4_body.subsystem.power_periph import PowerPeriph

p = PowerPeriph()
if not p.startup():
    print('Failed to connect to the power management board. Please run `stretch_system_check`.')
    resume_server(server_was_running, assume_yes=args.yes)
    sys.exit(1)

did_fail = False
try:
    print('---------------------')
    if args.action == 'cycle' or args.action == 'off':
        for flag, name, key, device in selected:
            did_fail |= not set_power(p, name, key, enable=False)
    if args.action == 'cycle':
        print('---------------------')
        time.sleep(args.off_time)
    if args.action == 'cycle' or args.action == 'on':
        for flag, name, key, device in selected:
            did_fail |= not set_power(p, name, key, enable=True)
    print('---------------------')
    time.sleep(0.5)

    # The rail being on is not the same as the board having rebooted onto the USB
    # bus. A hung board holds its rail on, so "on" is a no-op and only a cycle
    # with a long enough off time brings it back.
    if args.action == 'cycle' or args.action == 'on':
        for flag, name, key, device in selected:
            if not wait_on_device(device, present=True):
                did_fail = True
                print('WARNING: %s is powered but %s has not come back on the USB bus.' % (name, device))
                if args.action == 'on':
                    print('         Try: REx_actuator_control --%s --action cycle --off_time 3' % flag)
                else:
                    print('         Try a longer off time: --off_time %g' % max(3.0, args.off_time * 2))
except (KeyboardInterrupt, click.Abort):
    print('\nAborted.')
finally:
    p.stop()  # Release the board before the server tries to open it

if args.action == 'off':
    # Don't offer to start it: the server cannot open a motor whose board is
    # unpowered, so it would just fail and drop into a systemd restart loop.
    print('\nNote: the stretch_body_server will not start while these actuators are powered off.')
    print('Run `stretch_body_server --restart` once you power them back on.')
else:
    resume_server(server_was_running, assume_yes=args.yes, always_prompt=True)

sys.exit(1 if did_fail else 0)
