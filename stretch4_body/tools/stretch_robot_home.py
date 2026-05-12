#!/usr/bin/env python3
from __future__ import print_function

import argparse

import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Find zeros for all robot joints')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

if not args.direct:
    from stretch4_body.robot.robot_client import RobotClient as Robot
else:
    from stretch4_body.robot.robot import Robot


def main():
        r = Robot()
        if r.startup():
            r.pull_status()
            if not r.power_periph.status['runstop_event']:
                r.home()
            else:
                r.logger.error('Cannot home while run-stopped')
        r.stop()

if __name__ == '__main__':
    main()
