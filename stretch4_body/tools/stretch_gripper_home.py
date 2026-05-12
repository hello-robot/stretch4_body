#!/usr/bin/env python3
import threading
import time
import argparse
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the gripper position by closing until motion stops')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
parser.add_argument("-p","--use_parallel", help="Use the Parallel Gripper",action="store_true")
args=parser.parse_args()

if not args.direct:
    if args.use_parallel:
        from stretch4_body.robot.robot_client import ParallelGripperClient as StretchGripper
    else:
        from stretch4_body.robot.robot_client import StretchGripperClient as StretchGripper
else:
    if args.use_parallel:
        from stretch4_body.subsystem.end_of_arm.parallel_gripper import ParallelGripper as StretchGripper
    else:
        from stretch4_body.subsystem.end_of_arm.stretch_gripper import StretchGripper

cancel_homing_event = threading.Event()
g=StretchGripper()
if not g.startup():
    exit()
g.home()
time.sleep(3.0)
g.stop()