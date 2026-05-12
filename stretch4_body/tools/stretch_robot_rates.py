#!/usr/bin/env python3
import stretch4_body.core.hello_utils as hu
from stretch4_body.robot.robot_client import RobotClient
import argparse
import time

hu.print_stretch_re_use()
parser=argparse.ArgumentParser(description='Measure rates of RobotClient and server')
parser.add_argument("-v","--viz", help="Visualize the latest rates from server",action="store_true")
parser.add_argument("--static_ip", help="IP address of the robot server", type=str, default=None)

args=parser.parse_args()

if args.viz:
    hu.display_most_recent_robot_rates()

r = RobotClient(ip_address=args.static_ip)
if r.startup():
    print('Measuring RobotClient update rates...')
    r.power_periph.trigger_runstop()
    r.push_command()
    ts = time.time()
    for i in range(100):
        r.pull_status()
        r.lift.move_by(0.0)
        r.arm.move_by(0.0)
        r.omnibase.translate_by(0.0,0.0)
        r.end_of_arm.move_to('wrist_pitch',0)
        r.end_of_arm.move_to('wrist_yaw', 0)
        r.end_of_arm.move_to('wrist_roll', 0)
        r.push_command()
        #print('--- ITR %d | Server Rate (Hz) %f- ----------'%(i,r.status['server']['control_loop']['curr_rate_hz']))
        #time.sleep(0.1)
    print('Maximum update rate of RobotClient (Hz)', 100 / (time.time() - ts))
    r.stop()

