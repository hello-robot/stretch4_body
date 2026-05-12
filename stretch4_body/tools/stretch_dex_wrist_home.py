#!/usr/bin/env python3

import argparse
from stretch4_body.core.hello_utils import *
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

if not args.direct:
    from stretch4_body.robot.robot_client import WristYawClient as WristYaw
    from stretch4_body.robot.robot_client import WristRollClient as WristRoll
    from stretch4_body.robot.robot_client import WristPitchClient as WristPitch
    from stretch4_body.robot.robot_client import EndOfArmClient as EndOfArm
    from stretch4_body.robot.robot_client import RobotClient as Robot
else:
    from stretch4_body.subsystem.end_of_arm.wrist_yaw import WristYaw
    from stretch4_body.subsystem.end_of_arm.wrist_pitch import WristPitch
    from stretch4_body.subsystem.end_of_arm.wrist_roll import WristRoll
    from stretch4_body.robot.robot import Robot

if args.roll:
    print("Homing Roll Joint")
    r = WristRoll()
    if r.startup():
        success = r.home()
        r.stop()
        if success:
            print('Homing complete')
elif args.yaw:
    print("Homing Yaw Joint")
    y=WristYaw()
    if y.startup():
        success = y.home()
        y.stop()
        if success:
            print('Homing complete')
elif args.pitch:
    print("Homing Pitch Joint")
    p=WristPitch()
    if p.startup():
        success = p.home()
        p.stop()
        if success:
            print('Homing complete')
elif args.all:
    print("Homing Wrist Joints")
    r = Robot()
    if r.startup():
        success = r.end_of_arm.home()
        r.stop()
        if success:
            print('Homing complete')
