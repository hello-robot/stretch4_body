#!/usr/bin/env python3
import threading
import stretch4_body.core.robot_params
from stretch4_body.utils.stretch_pose_models import RobotJoints
from stretch4_body.utils.user_tool_utils import add_user_tool_to_sys_path
stretch4_body.core.robot_params.RobotParams.set_logging_level("DEBUG")
import sys
import argparse
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the gripper from the keyboard')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
parser.add_argument("-i","--ip", help="IP address to remote server", type=str, default=None)
args=parser.parse_args()


from stretch4_body.utils.user_tool_utils import get_gripper_instance

g, gripper_type, is_parallel = get_gripper_instance(direct=args.direct, ip_address=args.ip)
if g is None:
    print("No gripper is configured on this robot.")
    exit(1)

# For custom client end-of-arm tool grippers, bind joint-specific methods/properties directly
is_custom_client = not args.direct and gripper_type not in ['parallel_gripper', 'stretch_gripper']
if is_custom_client:
    g_params = g.params.get('devices', {}).get(gripper_type, g.params)
    if is_parallel:
        from stretch4_body.subsystem.end_of_arm.gripper_conversion import parallel_gripper_servo_rad_to_mm
        from stretch4_body.core.hello_utils import deg_to_rad
        open_m = parallel_gripper_servo_rad_to_mm(deg_to_rad(g_params['range_deg'][1]), g_params) / 1000.0
        g.poses = {'zero': 0.0,
                   'open': open_m,
                   'mid': open_m / 2.0,
                   'close': 0.0}
    else:
        # A custom client defines its own pct convention; fall back to the
        # Stretch Gripper one only when it does not. range_deg[0] is 0 for
        # tools that put their closed hardstop at pct 0
        if not hasattr(g, 'pct_max_open'):
            range_deg = g_params.get('range_deg', [0.0, 0.0])
            g.pct_max_open = 100 * abs(range_deg[1] / range_deg[0]) if range_deg[0] else 100.0
        if not hasattr(g, 'poses'):
            g.poses = {'zero': 0,
                       'open': g.pct_max_open,
                       'close': -100}
    
    g.move_to_joint = g.move_to
    g.move_by_joint = g.move_by
    g.move_to = lambda x, v=None, a=None: g.move_to_joint(gripper_type, x, v, a)
    g.move_by = lambda x, v=None, a=None: g.move_by_joint(gripper_type, x, v, a)
    
    def custom_pretty_print():
        print(f"--- {gripper_type} ---")
        status = g.status.get(gripper_type, {})
        if 'pos_mm' in status:
            print(f"Position (mm): {status['pos_mm']}")
        elif 'pos' in status:
            print(f"Position: {status['pos']}")
    g.pretty_print = custom_pretty_print

if not g.startup():
    exit()

g.pull_status()
v_des=g.params['motion']['default']['vel']
a_des=g.params['motion']['default']['accel']
pct_min = min(g.poses.values()) if not is_parallel else None

def menu_top():
    print('------ MENU -------')
    print('m: menu')
    print('h: home')
    if is_parallel:
        print('x: close by 10mm')
        print('y: open by 10mm')
        print('p: go to position (m)')
    else:
        print('x: close by 10%')
        print('y: open by 10%')
        print('p: go to position (%6.2f to %6.2f)'%(g.pct_max_open, pct_min))
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
            if is_parallel:
                g.move_by(-0.01, v_des, a_des)
            else:
                g.move_by(-10.0, v_des, a_des)
        if x[0]=='y':
            if is_parallel:
                g.move_by(0.01, v_des, a_des)
            else:
                g.move_by(10.0, v_des, a_des)
        if x[0]=='p':
            if is_parallel:
                print("Enter position (m): ")
                ff = float(sys.stdin.readline())
            else:
                print("Enter position (%): ")
                ff = float(sys.stdin.readline())
                ff=min(max(pct_min,ff),g.pct_max_open)
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

