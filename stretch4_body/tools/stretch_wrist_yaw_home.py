#!/usr/bin/env python3
import threading
import argparse
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the wrist_yaw position by moving to a hardstop')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

if not args.direct:
    from stretch4_body.robot.robot_client import WristYawClient as WristYaw
else:
    from stretch4_body.subsystem.end_of_arm.wrist_yaw import WristYaw

w=WristYaw()
if not w.startup():
    exit()
w.home()
w.stop()
