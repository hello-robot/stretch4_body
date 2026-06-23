#!/usr/bin/env python3
import threading
import time
import argparse
import stretch4_body.core.hello_utils as hu
from stretch4_body.utils.stretch_pose_models import RobotJoints
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the gripper position by closing until motion stops')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()


gripper_type = RobotJoints.gripper.value
if gripper_type is None:
    print("No gripper is configured on this robot.")
    exit(1)

if not args.direct:
    if gripper_type == 'parallel_gripper':
        from stretch4_body.robot.robot_client import ParallelGripperClient as Gripper
    else:
        from stretch4_body.robot.robot_client import StretchGripperClient as Gripper
else:
    if gripper_type == 'parallel_gripper':
        from stretch4_body.subsystem.end_of_arm.parallel_gripper import ParallelGripper as Gripper
    else:
        from stretch4_body.subsystem.end_of_arm.stretch_gripper import StretchGripper as Gripper

g=Gripper()
if not g.startup():
    exit()
g.home()
time.sleep(3.0)
g.stop()