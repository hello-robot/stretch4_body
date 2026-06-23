#!/usr/bin/env python3
import threading
import stretch4_body.core.robot_params
from stretch4_body.utils.stretch_pose_models import RobotJoints
stretch4_body.core.robot_params.RobotParams.set_logging_level("DEBUG")
import sys
import argparse
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the gripper from the keyboard')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
parser.add_argument("-i","--ip", help="IP address to remote server", type=str, default=None)
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
    g = Gripper(ip_address=args.ip)
else:
    if gripper_type == 'parallel_gripper':
        from stretch4_body.subsystem.end_of_arm.parallel_gripper import ParallelGripper as Gripper
    else:
        from stretch4_body.subsystem.end_of_arm.stretch_gripper import StretchGripper as Gripper
    g = Gripper(is_direct=True)

if not g.startup():
    exit()

g.pull_status()
v_des=g.params['motion']['default']['vel']
a_des=g.params['motion']['default']['accel']

def menu_top():
    print('------ MENU -------')
    print('m: menu')
    print('h: home')
    if gripper_type == 'parallel_gripper':
        print('x: close by 10mm')
        print('y: open by 10mm')
        print('p: go to position (mm)')
    else:
        print('x: close by 10%')
        print('y: open by 10%')
        print('p: go to position (%6.2f to -100)'%g.pct_max_open)
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
    x=sys.stdin.readline()
    if not x:
        exit()
    if len(x)>1:
        if x[0]=='m':
            menu_top()
        if x[0]=='h':
            g.home()
        if x[0]=='x':
            g.move_by(-10.0, v_des, a_des)
        if x[0]=='y':
            g.move_by(10.0, v_des, a_des)
        if x[0]=='p':
            print("Enter position: ")
            ff = float(sys.stdin.readline())
            if gripper_type != 'parallel_gripper':
                ff=min(max(-100,ff),g.pct_max_open)
            g.move_to(ff, v_des, a_des)
        if x[0] == 'a':
            g.move_to(g.poses['open'], v_des, a_des)
        if x[0] == 'b':
            g.move_to(g.poses['zero'], v_des, a_des)
        if x[0] == 'c':
            g.move_to(g.poses['close'], v_des, a_des)
        if x[0]=='r':
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
        except (ValueError):
            print('Bad input...')
        g.pull_status()
except (KeyboardInterrupt):
    g.stop()

