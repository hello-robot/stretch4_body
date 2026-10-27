#!/usr/bin/env python3

import argparse
from stretch4_body.core.hello_utils import *

from stretch4_body.robot.robot_client import WristJointClient
from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello

from stretch4_body.robot.robot_client import WristYawClient
from stretch4_body.robot.robot_client import WristRollClient
from stretch4_body.robot.robot_client import WristPitchClient
from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.robot.robot_client import LiftClient

from stretch4_body.subsystem.end_of_arm.wrist_yaw import WristYaw
from stretch4_body.subsystem.end_of_arm.wrist_pitch import WristPitch
from stretch4_body.subsystem.end_of_arm.wrist_roll import WristRoll
from stretch4_body.subsystem.lift import Lift
from stretch4_body.robot.robot import Robot
from stretch4_body.utils.lift_clearance import ensure_lift_clearance

def _check_lift_clearance(direct):
    l = Lift() if direct else LiftClient()
    if 'lift' not in l.robot_params['robot']['subsystems']:
        return True
    if not l.startup():
        print('Failed to connect to the lift.')
        return False
    try:
        return ensure_lift_clearance(l)
    finally:
        l.stop()

def _home_joint(r:FeetechSMHello|WristJointClient):
    if r.startup():
        success = r.home()
        r.stop()
        if success:
            print('Homing complete')
        else:
            print('Homing failed')

if __name__ == "__main__":
    print_stretch_re_use()

    parser=argparse.ArgumentParser(description='Home the dexterous wrist joints')
    parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument("--yaw", help="Home yaw joint",action="store_true")
    group.add_argument("--pitch", help="Home pitch joint",action="store_true")
    group.add_argument("--roll", help="Home roll joint",action="store_true")
    group.add_argument("--all", help="Home all joints",action="store_true")
    args, _ = parser.parse_known_args()

    if not (args.roll or args.yaw or args.pitch or args.all):
        args.all = True

    if not _check_lift_clearance(args.direct):
        exit(1)

    if args.roll:
        print("Homing Roll Joint")
        r = WristRoll() if args.direct else WristRollClient()
        _home_joint(r)
    elif args.yaw:
        print("Homing Yaw Joint")
        y=WristYaw() if args.direct else WristYawClient()
        _home_joint(y)
    elif args.pitch:
        print("Homing Pitch Joint")
        p=WristPitch() if args.direct else WristPitchClient()
        _home_joint(p)
    elif args.all:
        print("Homing Wrist Joints")
        r = Robot() if args.direct else RobotClient()
        if r.startup():
            success = r.end_of_arm.home()
            r.stop()
            if success:
                print('Homing complete')
            else:
                print('Homing failed')
