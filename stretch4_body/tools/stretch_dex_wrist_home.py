#!/usr/bin/env python3

import argparse
from stretch4_body.core.hello_utils import *

from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.robot.robot import Robot
from stretch4_body.utils.lift_clearance import ensure_lift_clearance, set_lift_safety_hold

def _home(r:Robot|RobotClient, joint_name:str|None, direct:bool) -> bool:
    if joint_name is None:
        return r.end_of_arm.home()
    if joint_name not in r.end_of_arm.joints:
        print(f'The current tool has no {joint_name} joint.')
        return False
    if direct:
        return r.end_of_arm.home_joint(joint_name)
    return getattr(r.end_of_arm, joint_name).home()

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

    if args.roll:
        joint_name = 'wrist_roll'
    elif args.yaw:
        joint_name = 'wrist_yaw'
    elif args.pitch:
        joint_name = 'wrist_pitch'
    else:
        joint_name = None

    # The lift and wrist go through the same Robot, so the lift can be raised
    # and held in place while the wrist homes
    r = Robot() if args.direct else RobotClient()
    if not r.startup():
        print('Failed to connect to the robot.')
        exit(1)

    has_lift = r.get_subsystem('lift') is not None
    success = False
    try:
        if not has_lift or ensure_lift_clearance(r.lift, root=r):
            print(f"Homing {joint_name.replace('_', ' ').title() if joint_name else 'Wrist Joints'}")
            success = _home(r, joint_name, args.direct)
            print('Homing complete' if success else 'Homing failed')
    finally:
        if args.direct and has_lift:
            # Stopping the direct Robot puts the lift in safety mode, which only
            # counterbalances it, so make that hold position too
            set_lift_safety_hold(r.lift)
        r.stop()

    if not success:
        exit(1)
