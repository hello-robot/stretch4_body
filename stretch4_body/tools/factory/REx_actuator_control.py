#!/usr/bin/env python3
from __future__ import print_function
import sys
from stretch4_body.subsystem.power_periph import PowerPeriph
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()
import time
import argparse


parser=argparse.ArgumentParser(description='Control the power of an actuator device')

parser.add_argument("--lift", help="Power cycle lift", action="store_true")
parser.add_argument("--arm", help="Power cycle arm", action="store_true")
parser.add_argument("--omni_0", help="Power cycle omni wheel 0", action="store_true")
parser.add_argument("--omni_1", help="Power cycle omni wheel 1", action="store_true")
parser.add_argument("--omni_2", help="Power cycle omni wheel 2", action="store_true")
parser.add_argument("--eoa", help="Power cycle eoa servos", action="store_true")
parser.add_argument("--all", help="Power cycle all actuators", action="store_true")
parser.add_argument('--action', type=str, default='cycle', help='Action to take: on / off / [cycle]')
args, _ = parser.parse_known_args()


p=PowerPeriph()
if not p.startup():
    exit()
print('---------------------')
if args.action=='cycle' or args.action=='off':
    if args.lift or args.all:
        print('Powering off lift...')
        p.actuator_control( 'lift', enable=False)
    if args.arm or args.all:
        print('Powering off arm...')
        p.actuator_control( 'arm', enable=False)
    if args.omni_0 or args.all:
        print('Powering off omni-0...')
        p.actuator_control( 'omni-0', enable=False)
    if args.omni_1 or args.all:
        print('Powering off omni-1...')
        p.actuator_control( 'omni-1', enable=False)
    if args.omni_2 or args.all:
        print('Powering off omni-2...')
        p.actuator_control( 'omni-2', enable=False)
    if args.eoa or args.all:
        print('Powering off eoa...')
        p.actuator_control( 'eoa', enable=False)
print('---------------------')
time.sleep(0.5)
if args.action=='cycle' or args.action=='on':

    if args.lift or args.all:
        print('Powering on lift...')
        p.actuator_control( 'lift', enable=True)
    if args.arm or args.all:
        print('Powering on arm...')
        p.actuator_control( 'arm', enable=True)
    if args.omni_0 or args.all:
        print('Powering on omni-0...')
        p.actuator_control( 'omni-0', enable=True)
    if args.omni_1 or args.all:
        print('Powering on omni-1...')
        p.actuator_control( 'omni-1', enable=True)
    if args.omni_2 or args.all:
        print('Powering on omni-2...')
        p.actuator_control( 'omni-2', enable=True)
    if args.eoa or args.all:
        print('Powering on eoa...')
        p.actuator_control( 'eoa', enable=True)
print('---------------------')
time.sleep(0.5)
p.stop()