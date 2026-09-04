#!/usr/bin/env python3
import argparse
import sys

import stretch4_body.core.hello_utils as hu
import stretch4_body.core.robot_params
from stretch4_body.utils.tool_metadata import get_gripper_instance, get_tool_metadata

stretch4_body.core.robot_params.RobotParams.set_logging_level("DEBUG")


hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the gripper from the keyboard')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
parser.add_argument("-i","--ip", help="IP address to remote server", type=str, default=None)
args=parser.parse_args()

g, gripper_type = get_gripper_instance(direct=args.direct, ip_address=args.ip)
if g is None:
    print("No gripper is configured on this robot.")
    exit(1)

try:
    meta = get_tool_metadata(gripper_type)
    # g.move_to()/move_by() below take this tool's own command units (e.g. aperture meters
    # for PG4), not true raw actuator units -- use command_range, not actuator_range.
    act_range = meta.command_range
except Exception:
    act_range = (0.0, 100.0)

low, high = min(act_range), max(act_range)
step_size = (high - low) / 10.0 if high != low else 1.0

# For custom client end-of-arm tool grippers, bind joint-specific methods/properties directly
is_custom_client = not args.direct and gripper_type not in ['parallel_gripper', 'stretch_gripper']
if is_custom_client:
    if not hasattr(g, 'poses'):
        g.poses = {
            'zero': 0.0,
            'open': high,
            'mid': (low + high) / 2.0,
            'close': low,
        }

    g.move_to_joint = g.move_to
    g.move_by_joint = g.move_by
    g.move_to = lambda x, v=None, a=None: g.move_to_joint(gripper_type, x, v, a)
    g.move_by = lambda x, v=None, a=None: g.move_by_joint(gripper_type, x, v, a)

    def custom_pretty_print():
        print(f"--- {gripper_type} ---")
        status = g.status.get(gripper_type, {})
        for k, val in status.items():
            if 'pos' in k:
                print(f"{k}: {val}")

    g.pretty_print = custom_pretty_print

if not g.startup():
    exit()

g.pull_status()
v_des = g.params['motion']['default']['vel']
a_des = g.params['motion']['default']['accel']


def menu_top():
    print('------ MENU -------')
    print('m: menu')
    print('h: home')
    print('x: close by 10% step')
    print('y: open by 10% step')
    print(f'p: go to position ({low:.3f} to {high:.3f})')
    print('r: reboot')
    print('-----')
    print('a: open')
    print('b: zero')
    print('c: close')
    print('-----')
    print('1: speed slow')
    print('2: speed default')
    print('3: speed fast')
    print('4: speed max')
    print('-------------------')


def step_interaction():
    global v_des, a_des
    menu_top()
    x = sys.stdin.readline()
    if not x:
        exit()
    if len(x) > 1:
        if x[0] == 'm':
            menu_top()
        if x[0] == 'h':
            g.home()
        if x[0] == 'x':
            g.move_by(-step_size, v_des, a_des)
        if x[0] == 'y':
            g.move_by(step_size, v_des, a_des)
        if x[0] == 'p':
            print(f"Enter position ({low:.3f} to {high:.3f}): ")
            try:
                ff = float(sys.stdin.readline())
                ff = min(max(low, ff), high)
                g.move_to(ff, v_des, a_des)
            except ValueError:
                print("Invalid input position.")
        if x[0] == 'a':
            g.move_to(g.poses['open'], v_des, a_des)
        if x[0] == 'b':
            g.move_to(g.poses['zero'], v_des, a_des)
        if x[0] == 'c':
            g.move_to(g.poses['close'], v_des, a_des)
        if x[0] == 'r':
            g.motor.do_reboot()
            print('Exiting after reboot.')
            exit()

        if x[0] == '1':
            v_des = g.params['motion']['slow']['vel']
            a_des = g.params['motion']['slow']['accel']

        if x[0] == '2':
            v_des = g.params['motion']['default']['vel']
            a_des = g.params['motion']['default']['accel']

        if x[0] == '3':
            v_des = g.params['motion']['fast']['vel']
            a_des = g.params['motion']['fast']['accel']

        if x[0] == '4':
            v_des = g.params['motion']['max']['vel']
            a_des = g.params['motion']['max']['accel']
        g.push_command()
    else:
        g.pretty_print()


try:
    while True:
        try:
            step_interaction()
        except ValueError:
            print('Bad input...')
        g.pull_status()
except KeyboardInterrupt:
    g.stop()
