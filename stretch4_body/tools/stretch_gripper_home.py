#!/usr/bin/env python3
import time
import argparse
import stretch4_body.core.hello_utils as hu
from stretch4_body.utils.tool_metadata import get_gripper_instance

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the gripper position by closing until motion stops')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

g, gripper_type, is_parallel = get_gripper_instance(direct=args.direct)
if g is None:
    print("No gripper is configured on this robot.")
    exit(1)
if not g.startup():
    exit()
g.home()
time.sleep(3.0)
g.stop()