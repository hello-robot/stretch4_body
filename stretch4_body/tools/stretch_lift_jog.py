#!/usr/bin/env python3
import sys, tty, termios
import time
import stretch4_body.subsystem.lift as lift
import argparse
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the lift motion from the keyboard')
parser.add_argument("--no_rs", help="No runstop required",action="store_true")
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

small_move_m=.01
large_move_m=0.3


if not args.direct:
    from stretch4_body.robot.robot_client import LiftClient as Lift
else:
    from stretch4_body.subsystem.lift import Lift

l = Lift()

if not l.startup():
    exit()
l.disable_sync_mode()
if args.no_rs:
    l.disable_runstop()
l.push_command()

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
    print('u / d : small up down')
    print('U / D : large up down')
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
    menu()
    while True:
        c=get_keystroke()
        l.pull_status()
        l.pretty_print()
        if c == 'f':
            stiffness=0.0
            l.move_by(x_m=0, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],
                      stiffness=stiffness, req_calibration=req_calibration)
        if c == 's':
            stiffness=0.3
            l.move_by(x_m=0, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],
                      stiffness=stiffness, req_calibration=req_calibration)
        if c == 'h':
            stiffness=1.0
            l.move_by(x_m=0, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],
                      stiffness=stiffness, req_calibration=req_calibration)

        if c=='1':
            rate='slow'
        if c == '2':
            rate = 'default'
        if c == '3':
            rate = 'fast'
        if c == '4':
            rate = 'max'
        if c == '5':
            l.set_guarded_contact_sensitivity('sensitivity_low')
        if c == '6':
            l.set_guarded_contact_sensitivity('sensitivity_default')
        if c == '7':
            l.set_guarded_contact_sensitivity('sensitivity_high')
        if c == '8':
            l.set_guarded_contact_sensitivity('off')
        if c=='m':
            menu()
        if c=="Q" or c=='q':
            break
        if c == 'u':
            l.move_by(x_m= small_move_m, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=req_calibration)
        if c == 'd':
            l.move_by(x_m=-1*small_move_m, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=req_calibration)
        if c == 'U':
            l.move_by(x_m=large_move_m, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=req_calibration)
        if c == 'D':
            l.move_by(x_m=-1*large_move_m, v_m=l.params['motion'][rate]['vel_m'], a_m=l.params['motion'][rate]['accel_m'],stiffness=stiffness, req_calibration=req_calibration)
        l.push_command()
        time.sleep(0.1)
except (KeyboardInterrupt, SystemExit):
    pass
l.stop()

