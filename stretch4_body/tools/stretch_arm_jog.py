#!/usr/bin/env python3
from __future__ import print_function
import sys, tty, termios
import time
import argparse
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the arm motion from the keyboard')
parser.add_argument("--no_rs", help="No runstop required",action="store_true")
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

if not args.direct:
    from stretch4_body.robot.robot_client import ArmClient as Arm
else:
    from stretch4_body.subsystem.arm import Arm

small_move_m=.01
large_move_m=0.1

a=Arm()
if not a.startup():
    exit()
a.disable_sync_mode()
if args.no_rs:
    a.disable_runstop()
a.push_command()

def get_keystroke():

    fd=sys.stdin.fileno()
    old_settings=termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch=sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)
    return ch

def menu():
    print('--------------')
    print('m: menu')
    print('i / o : small in out')
    print('I / O : large in out')
    print('f: stiffness float')
    print('s: stiffness soft')
    print('h: stiffness hard')
    print('1: rate slow')
    print('2: rate default')
    print('3: rate fast')
    print('4: rate max')
    print('5: Set contact sensitivity low')
    print('6: Set contact sensitivity default')
    print('7: Set contact sensitivity high')
    print('8: Disable guarded contacts')
    print('q: quit')
    print('')
    print('Input?')

rate='default'
req_calibration=False
stiffness=1.0
try:

    while True:
        menu()
        c=get_keystroke()
        a.pull_status()
        a.pretty_print()

        if c=='1':
            rate='slow'
        if c == '2':
            rate = 'default'
        if c == '3':
            rate = 'fast'
        if c == '4':
            rate = 'max'
        if c == '5':
            a.set_guarded_contact_sensitivity('sensitivity_low')
        if c == '6':
            a.set_guarded_contact_sensitivity('sensitivity_default')
        if c == '7':
            a.set_guarded_contact_sensitivity('sensitivity_high')
        if c == '8':
            a.set_guarded_contact_sensitivity('off')
        if c == 'f':
            stiffness=0.0
            a.move_by(x_m=-0, v_m=a.params['motion'][rate]['vel_m'],
                      a_m=a.params['motion'][rate]['accel_m'], stiffness=stiffness, req_calibration=req_calibration)
        if c == 's':
            stiffness=0.3
            a.move_by(x_m=0, v_m=a.params['motion'][rate]['vel_m'],
                      a_m=a.params['motion'][rate]['accel_m'], stiffness=stiffness, req_calibration=req_calibration)
        if c == 'h':
            stiffness=1.0
            a.move_by(x_m=0, v_m=a.params['motion'][rate]['vel_m'],
                      a_m=a.params['motion'][rate]['accel_m'], stiffness=stiffness, req_calibration=req_calibration)

        if c=='m':
            menu()
        if c=="Q" or c=='q':
            break
        if c == 'i':
            a.move_by(x_m= -1*small_move_m, v_m=a.params['motion'][rate]['vel_m'], a_m=a.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=req_calibration)
        if c == 'o':
            a.move_by(x_m=small_move_m, v_m=a.params['motion'][rate]['vel_m'], a_m=a.params['motion'][rate]['accel_m'], stiffness=stiffness,req_calibration=req_calibration)
        if c == 'I':
            a.move_by(x_m=-1*large_move_m, v_m=a.params['motion'][rate]['vel_m'], a_m=a.params['motion'][rate]['accel_m'],stiffness=stiffness,req_calibration=req_calibration)
        if c == 'O':
            a.move_by(x_m=large_move_m, v_m=a.params['motion'][rate]['vel_m'], a_m=a.params['motion'][rate]['accel_m'],stiffness=stiffness,req_calibration=req_calibration)
        a.push_command()
        time.sleep(0.1)
except (KeyboardInterrupt, SystemExit):
    pass
a.stop()
