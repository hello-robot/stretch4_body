#!/usr/bin/env python3
import argparse
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the arm position by moving to the upper hardstop')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

if not args.direct:
    from stretch4_body.robot.robot_client import ArmClient as Arm
else:
    from stretch4_body.subsystem.arm import Arm

a=Arm()
if not a.startup():
    exit()
a.home()
a.stop()


