#!/usr/bin/env python3
from __future__ import print_function
from stretch4_body.subsystem.power_periph import PowerPeriph
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()
import time
import argparse

parser=argparse.ArgumentParser(description='Control the power of the end-of-arm')
parser.add_argument("--cycle", help="Power cycle", action="store_true")
parser.add_argument("--on", help="Power on", action="store_true")
parser.add_argument("--off", help="Power off", action="store_true")
parser.add_argument("--status", help="Print current status", action="store_true")
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args, _ = parser.parse_known_args()

if not args.direct:
    from stretch4_body.robot.robot_client import PowerPeriphClient as PowerPeriph
else:
    from stretch4_body.subsystem.power_periph import PowerPeriph

p=PowerPeriph()
if not p.startup():
    exit()

if args.status:
    print('---------------------')
    p.pull_status()
    print('Power on: ', p.status['periph_power_state']['power_to_eoa'])
    print('Current EOA', p.status['current_eoa'])
    print('High Current EOA Alert', p.status['high_current_eoa_alert'])
    print('---------------------')
    exit()

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
p.stop()