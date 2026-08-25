#!/usr/bin/env python3
from stretch4_body.core.hello_utils import *
import argparse
import select

print_stretch_re_use()

parser=argparse.ArgumentParser(description='Jog the base motion from the keyboard')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

# pKd_d=100.0
# pKi_d=0.0#.01
# pKi_limit=50.0
# pKp_d=24.0 #12.0
#
# for i in range(3):
#     b.wheels[i].gains['pKd_d']=pKd_d
#     b.wheels[i].gains['pKi_d']=pKi_d
#     b.wheels[i].gains['pKi_limit']=pKi_limit
#     b.wheels[i].gains['pKp_d']=pKp_d




large_move_m=0.1
small_move_m=large_move_m/8
small_rotate_rad=deg_to_rad(1.0)
large_rotate_rad=deg_to_rad(10.0)

# v_r_slow = b.translation_to_rotation(b.params['motion']['slow']['vel_xy_m'])
# v_r_def = b.translation_to_rotation(b.params['motion']['default']['vel_xy_m'])
# v_r_fast=b.translation_to_rotation(b.params['motion']['fast']['vel_xy_m'])
# v_r_max = b.translation_to_rotation(b.params['motion']['max']['vel_xy_m'])
# a_r_slow = b.translation_to_rotation(b.params['motion']['slow']['accel_xy_m'])
# a_r_def = b.translation_to_rotation(b.params['motion']['default']['accel_xy_m'])
# a_r_fast=b.translation_to_rotation(b.params['motion']['fast']['accel_xy_m'])
# a_r_max = b.translation_to_rotation(b.params['motion']['max']['accel_xy_m'])
# v_r={'fast':v_r_fast,'default':v_r_def,'slow':v_r_slow,'max':v_r_max}
# a_r={'fast':a_r_fast,'default':a_r_def,'slow':a_r_slow,'max':a_r_max}



def get_keystroke():

    fd=sys.stdin.fileno()
    old_settings=termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch=sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)
    return ch

def prompt_float(msg,default,limit):
    """Prompt until a value within +/-limit is entered. Blank / invalid input keeps the default."""
    while True:
        print('%s, max +/-%f [%f]'%(msg,limit,default))
        try:
            v = float(input())
        except ValueError:
            return default
        if abs(v) > limit:
            print('Rejected: %f exceeds the max of %f in params'%(v,limit))
        else:
            return v

def command_velocity(vx,vy,vw):
    b.set_velocity(vx,vy,vw,a_m=b.params['motion'][rate]['accel_xy_m'],a_r=b.params['motion'][rate]['accel_w_r'])
    b.push_command()
    p.trigger_motor_sync()

def set_base_velocity():
    """Prompt for a base velocity and hold it until a key is pressed. The wheel firmware runs a
    velocity watchdog (gains['enable_vel_watchdog']) that stops the motors ~1s after the last
    command, so the command is re-sent at 10Hz for as long as the motion should continue."""
    limits = b.params['motion']['max']
    vx = prompt_float('Enter Vx (m/s), forward',0.0,limits['vel_xy_m'])
    vy = prompt_float('Enter Vy (m/s), left',0.0,limits['vel_xy_m'])
    vw = prompt_float('Enter Vtheta (rad/s), CCW',0.0,limits['vel_w_r'])
    print('Commanding Vx: %.3f m/s | Vy: %.3f m/s | Vtheta: %.3f rad/s. Press any key to stop'%(vx,vy,vw))
    fd=sys.stdin.fileno()
    old_settings=termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            command_velocity(vx,vy,vw)
            b.pull_status()
            print_base_status()
            print('Commanding Vx: %.3f m/s | Vy: %.3f m/s | Vtheta: %.3f rad/s. Press any key to stop'%(vx,vy,vw))
            if select.select([sys.stdin],[],[],0.1)[0]:
                sys.stdin.read(1)
                break
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)
        command_velocity(0,0,0)

def print_base_status():
    """Pose and per-wheel velocity. Stepper 'vel' is motor-shaft rad/s, so divide by
    the gear ratio for wheel rad/s and multiply by the wheel radius for ground speed."""
    r_wheel = b.params['wheel_diameter_m']/2.0
    print('---------- Base ----------')
    print('X (m): %.4f | Y (m): %.4f | Theta (rad): %.4f'%(b.status['x'],b.status['y'],b.status['theta']))
    print('Vx (m/s): %.4f | Vy (m/s): %.4f | Vtheta (rad/s): %.4f'%(b.status['x_vel'],b.status['y_vel'],b.status['theta_vel']))
    for i in range(3):
        v_motor = b.status['wheel_%d'%i]['vel']
        v_wheel = v_motor/b.params['gr']
        print('Wheel %d | motor %.3f rad/s | wheel %.3f rad/s | ground %.4f m/s'%(i,v_motor,v_wheel,v_wheel*r_wheel))

def menu():
    print('--------------')
    print('m: menu')
    print('')
    print('1: rate slow')
    print('2: rate default')
    print('3: rate fast')
    print('4: rate max')
    print('5: contact sensitivity low')
    print('6: contact sensitivity default')
    print('7: contact sensitivity high')
    print('8: Disable guarded contacts')
    print('o: freewheel')
    print('h: hold')
    print('')
    print('f / b / l / r : small forward / back / left / right')
    print('F / B / L / R : large forward / back / left / right')
    print('u / v : small rotate CCW/CW')
    print('U / V : large rotate CCW/CW')
    print('c: hold base velocity (Vx, Vy, Vtheta) until a key is pressed')
    print('k: stop (zero velocity)')
    print('')
    print('')
    print('w: cycle CCW/CW 90 deg')
    print('x: cycle right-> left 0.5m')
    print('y: cycle forward-> back 0.5m')
    print('s: cycle square of 0.5m')
    print('z: spin at 90deg/s')
    print('')

    print('p: pretty print')
    print('q: quit')
    print('')
    print('Input?')

rate ='default'

if not args.direct:
    from stretch4_body.robot.robot_client import OmniBaseClient as OmniBase
    from stretch4_body.robot.robot_client import PowerPeriphClient as PowerPeriph
else:
    from stretch4_body.subsystem.omnibase import OmniBase
    from stretch4_body.subsystem.power_periph import PowerPeriph

b=OmniBase()
p=PowerPeriph()
p.startup()
if not b.startup():
    exit()
try:
    while True:
        if True:
            menu()
            print('---------')
            c=get_keystroke()

            #Read current motor positions when in sync mode
            #p.trigger_motor_sync()
            #time.sleep(0.1)
            b.pull_status()

            # ################################################################################################
            if c=='p':
                print_base_status()
                b.pretty_print()
            if c == 'o':
                b.enable_freewheel_mode()
            if c == 'h':
                b.enable_hold_mode()
            if c == 'm':
                menu()
            if c == "Q" or c == 'q':
                break
            # ################################################################################################
            if c == 'f':
                b.translate_by(x_m=small_move_m, y_m=0, v_m=b.params['motion'][rate]['vel_xy_m'],
                            a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'b':
                b.translate_by(x_m=-1 * small_move_m, y_m=0, v_m=b.params['motion'][rate]['vel_xy_m'],
                            a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'F':
                b.translate_by(x_m=large_move_m, y_m=0, v_m=b.params['motion'][rate]['vel_xy_m'],
                            a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'B':
                b.translate_by(x_m=-1 * large_move_m, y_m=0, v_m=b.params['motion'][rate]['vel_xy_m'],
                            a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'l':
                b.translate_by(x_m=0, y_m=small_move_m, v_m=b.params['motion'][rate]['vel_xy_m'],
                            a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'r':
                b.translate_by(x_m=0, y_m=-1 * small_move_m, v_m=b.params['motion'][rate]['vel_xy_m'],
                            a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'L':
                b.translate_by(x_m=0, y_m=large_move_m, v_m=b.params['motion'][rate]['vel_xy_m'],
                            a_m=b.params['motion'][rate]['accel_xy_m'])
            if c == 'R':
                b.translate_by(x_m=0, y_m=-1 * large_move_m, v_m=b.params['motion'][rate]['vel_xy_m'],
                            a_m=b.params['motion'][rate]['accel_xy_m'])

            if c == 'u':
                b.rotate_by(w_r=small_rotate_rad, v_r=b.params['motion'][rate]['vel_w_r'],
                            a_r=b.params['motion'][rate]['accel_w_r'])
            if c == 'v':
                b.rotate_by(w_r=-1*small_rotate_rad, v_r=b.params['motion'][rate]['vel_w_r'],
                            a_r=b.params['motion'][rate]['accel_w_r'])

            if c == 'U':
                b.rotate_by(w_r=large_rotate_rad, v_r=b.params['motion'][rate]['vel_w_r'],
                            a_r=b.params['motion'][rate]['accel_w_r'])
            if c == 'V':
                b.rotate_by(w_r=-1*large_rotate_rad, v_r=b.params['motion'][rate]['vel_w_r'],
                            a_r=b.params['motion'][rate]['accel_w_r'])
            if c == 'c':
                set_base_velocity()
            if c == 'k':
                command_velocity(0,0,0)
            # ################################################################################################
            if c == '1':
                rate = 'slow'
            if c == '2':
                rate = 'default'
            if c == '3':
                rate = 'fast'
            if c == '4':
                rate = 'max'
            if c == '5':
                b.set_guarded_contact_sensitivity('sensitivity_low')
            if c == '6':
                b.set_guarded_contact_sensitivity('sensitivity_default')
            if c == '7':
                b.set_guarded_contact_sensitivity('sensitivity_high')
            if c == '8':
                b.set_guarded_contact_sensitivity('off')

            if c=='z':
                print('Enter num revolutions [1]')
                try:
                    x = int(input())
                except ValueError:
                    x = 1
                ts=time.time()
                b.set_omni_velocity('w',v_des=deg_to_rad(90),a_des=b.params['motion'][rate]['accel_w_r'])
                b.push_command()
                p.trigger_motor_sync()
                while time.time()-ts<(x*4+0.5):
                    time.sleep(0.1)
                b.set_omni_velocity('w',v_des=0,a_des=b.params['motion'][rate]['accel_w_r'])
                b.push_command()
                p.trigger_motor_sync()

            if c =='s':
                    print('Enter num cycles [10]')

                    try:
                        x=int(input())
                    except ValueError:
                        x=10
                    qq=[[0.5, 0], [0,0.5], [-0.5,0],[0,-0.5]]
                    for i in range(x):
                        for q in qq:
                            b.translate_by(q[0], y_m=q[1],v_m=b.params['motion'][rate]['vel_xy_m'], a_m=b.params['motion'][rate]['accel_xy_m'])
                            b.push_command()
                            p.trigger_motor_sync()
                            time.sleep(2.5)
            if c =='x':
                    print('Enter num cycles [10]')
                    try:
                        x=int(input())
                    except ValueError:
                        x=10
                    for i in range(x):
                        #time.sleep(x)
                        b.translate_by(x_m=-0.5, y_m=0.0,v_m=b.params['motion'][rate]['vel_xy_m'], a_m=b.params['motion'][rate]['accel_xy_m'])
                        b.push_command()
                        p.trigger_motor_sync()
                        for k in range(40):
                            b.pull_status()
                            print('Vx',b.status['x_vel'])
                            print('Vy', b.status['y_vel'])
                            print('Vw', b.status['theta_vel'])
                            time.sleep(0.1)
                        #time.sleep(4.0)
                        b.translate_by(x_m=0.5,y_m=0.0,v_m=b.params['motion'][rate]['vel_xy_m'], a_m=b.params['motion'][rate]['accel_xy_m'])
                        b.push_command()
                        p.trigger_motor_sync()
                        for k in range(40):
                            b.pull_status()
                            print('Vx', b.status['x_vel'])
                            print('Vy', b.status['y_vel'])
                            print('Vw', b.status['theta_vel'])
                            time.sleep(0.1)
            if c =='y':
                    print('Enter num cycles [10]')
                    try:
                        x=int(input())
                    except ValueError:
                        x=10
                    for i in range(x):
                        #time.sleep(x)
                        b.translate_by(x_m=0.0, y_m=0.5,v_m=b.params['motion'][rate]['vel_xy_m'], a_m=b.params['motion'][rate]['accel_xy_m'])
                        b.push_command()
                        p.trigger_motor_sync()
                        time.sleep(4.0)
                        b.translate_by(x_m=0.0,y_m=-0.5,v_m=b.params['motion'][rate]['vel_xy_m'], a_m=b.params['motion'][rate]['accel_xy_m'])
                        b.push_command()
                        p.trigger_motor_sync()
                        time.sleep(4.0)
            if c =='w':
                print('Enter num cycles [10]')
                try:
                    x = int(input())
                except ValueError:
                    x = 10
                for i in range(x):
                    b.rotate_by(w_r=deg_to_rad(90.0), v_r=b.params['motion'][rate]['vel_w_r'],a_r=b.params['motion'][rate]['accel_w_r'])
                    b.push_command()
                    p.trigger_motor_sync()
                    time.sleep(4.0)
                    b.rotate_by(w_r=deg_to_rad(-90.0), v_r=b.params['motion'][rate]['vel_w_r'],a_r=b.params['motion'][rate]['accel_w_r'])
                    b.push_command()
                    p.trigger_motor_sync()
                    time.sleep(4.0)

            b.push_command()
            p.trigger_motor_sync()
            time.sleep(0.1)
except (KeyboardInterrupt, SystemExit):
    pass
b.stop()
