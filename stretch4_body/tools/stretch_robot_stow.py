#!/usr/bin/env python3
from __future__ import print_function
import stretch4_body.robot as rb
import stretch4_body.core.hello_utils as hu
import sys
import argparse
parser=argparse.ArgumentParser(description='Move robot to stow position')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()


hu.print_stretch_re_use()

if not args.direct:
    from stretch4_body.robot.robot_client import RobotClient as Robot
else:
    from  stretch4_body.robot.robot import Robot

r = Robot()
if r.startup():
    r.pull_status()
    if not r.power_periph.status['runstop_event']:
        r.stow()
    else:
        r.logger.warning('Cannot stow while run-stopped')
    r.stop()

