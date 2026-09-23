#!/usr/bin/env python3
from __future__ import print_function
import sys
import time
import argparse
import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.power_periph import PowerPeriph
from stretch4_body.utils.server_power import (is_server_running,
                                              stop_server_for_power_change,
                                              resume_server)
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Control the power of the end-of-arm')
parser.add_argument("--cycle", help="Power cycle", action="store_true")
parser.add_argument("--on", help="Power on", action="store_true")
parser.add_argument("--off", help="Power off", action="store_true")
parser.add_argument("--status", help="Print current status", action="store_true")
parser.add_argument("-y", "--yes", help="Don't prompt before stopping and starting the stretch_body_server", action="store_true")
parser.add_argument("-d", "--direct", help=argparse.SUPPRESS, action="store_true")  # deprecated: always direct now
args, _ = parser.parse_known_args()

if args.status:
    # Read-only, so it can share the board with a running server.
    if is_server_running():
        from stretch4_body.robot.robot_client import PowerPeriphClient
        p = PowerPeriphClient()
        if not p.startup(verbose=False):
            sys.exit(1)
    else:
        p = PowerPeriph()
        if not p.startup():
            sys.exit(1)
    print('---------------------')
    p.pull_status()
    print('Power on: ', p.status['periph_power_state']['power_to_eoa'])
    print('Current EOA', p.status['current_eoa'])
    print('High Current EOA Alert', p.status['high_current_eoa_alert'])
    print('---------------------')
    p.stop()
    sys.exit(0)

if not (args.cycle or args.on or args.off):
    print('Nothing to do. Pick one of --cycle, --on, --off or --status.')
    sys.exit(1)

# Power is always changed through the direct API with the server stopped. See
# stretch4_body/utils/server_power.py for why the server cannot do this.
ok, server_was_running = stop_server_for_power_change(assume_yes=args.yes)
if not ok:
    print('Aborted. End-of-arm power was not changed.')
    sys.exit(1)

p = PowerPeriph()
if not p.startup():
    print('Failed to connect to the power management board. Please run `stretch_system_check`.')
    resume_server(server_was_running, assume_yes=args.yes)
    sys.exit(1)

try:
    if args.cycle:
        print('---------------------')
        print('Powering off eoa...')
        p.actuator_control( 'eoa', enable=False)
        time.sleep(0.5)
        print('Powering on eoa...')
        p.actuator_control('eoa', enable=True)

    if args.off:
        print('---------------------')
        print('Powering off eoa...')
        p.actuator_control( 'eoa', enable=False)

    if args.on:
        print('Powering on eoa...')
        p.actuator_control( 'eoa', enable=True)

    print('---------------------')
    time.sleep(0.5)
except KeyboardInterrupt:
    print('\nAborted.')
finally:
    p.stop()  # Release the board before the server tries to open it

if args.off and server_was_running:
    print('\nNote: the stretch_body_server will not start while the end-of-arm is powered off.')
    print('Run `stretch_body_server --restart` once you power it back on.')
else:
    resume_server(server_was_running, assume_yes=args.yes)
