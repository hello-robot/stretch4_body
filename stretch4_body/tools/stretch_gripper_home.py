#!/usr/bin/env python3
import argparse
import time

import stretch4_body.core.hello_utils as hu
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.utils.tool_metadata import get_gripper_instance

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Calibrate the gripper position by closing until motion stops')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

g, gripper_type = get_gripper_instance(direct=args.direct)
if g is None:
    print("No gripper is configured on this robot.")
    exit(1)
if not g.startup():
    exit()
if args.direct:
    # The server reads the tool's 'homing' end_pos itself; the bare driver does not
    _, robot_params = RobotParams.get_params()
    end_pos = robot_params[robot_params['robot']['tool']].get('homing', {}).get(gripper_type)
    if end_pos is not None:
        g.home(end_pos=end_pos)
    else:
        g.home()
else:
    g.home()
time.sleep(3.0)
g.stop()