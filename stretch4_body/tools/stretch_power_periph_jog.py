#!/usr/bin/env python3
from __future__ import print_function
import sys
from stretch4_body.subsystem.power_periph import PowerPeriph
import stretch4_body.core.hello_utils as hu
import time

hu.print_stretch_re_use()

import argparse
parser=argparse.ArgumentParser(description='Comnmand and query the Power-Periph board from the keyboard')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

if not args.direct:
    from stretch4_body.robot.robot_client import PowerPeriphClient as PowerPeriph
else:
    from stretch4_body.subsystem.power_periph import PowerPeriph

p=PowerPeriph()

if not p.startup():
    exit()

def menu():
    print('------ MENU -------')
    print('m: menu')
    print('i: reset imu')
    print('f: toggle fan')
    print('p: beep')
    print('t: trigger motor sync')
    print('r: reset board')
    print('x: reset runstop event')
    print('o: trigger runstop event')
    print('s: trigger sleep')
    print('a: set lidar off')
    print('b: set lidar on')
    print('c: set aux cpu off')
    print('d: set aux cpu on')

    print('--- ESP32 ---')
    print('e: place esp32 into bootloader')
    print('g: reset esp32')
    print('n: set network info')
    print('f: set firebase info')
    print('----------')
    print('q: quit')
    print('-------------------')

def get_val_default(prompt,default):
    x= input(prompt+' ['+default+']: ')
    if len(x)==0:
        x=default
    return x

def step_interaction():
    menu()
    x=sys.stdin.readline()
    p.pull_status()
    if len(x)>1:
        if x[0]=='a':
            p.set_lidar_off()
        if x[0] == 'b':
            p.set_lidar_on()
        if x[0] == 'c':
            p.set_aux_cpu_off()
        if x[0] == 'd':
            p.set_aux_cpu_on()
        if x[0]=='m':
            menu()
        if x[0]=='x':
            print('Clear Runstop Event')
            p.clear_runstop()
        if x[0]=='o':
            print('Triggering Runstop Event')
            p.trigger_runstop()
        if x[0]=='s':
            if input('Warning: About to sleep robot. Type yes to proceed. ')=='yes':
                p.trigger_sleep()
        if x[0]=='r':
            print('Resetting Board!!!')
            p.board_reset()
            p.push_command()
            time.sleep(2)
            exit(0)
        if x[0]=='i':
            p.imu_reset()
        if x[0]=='e':
            p.set_esp_fw_update()
        if x[0]=='g':
            p.set_esp_reset()
        if x[0]=='n':
            ssid=get_val_default('Enter Wifi SSID ',p.params['firebase']['network_ssid'])
            password=get_val_default('Enter Wifi Password ',p.params['firebase']['network_password'])
            p.send_network_info(ssid,password)


        if x[0]=='f':
            url=get_val_default('Enter URL ',p.params['firebase']['network_ssid'])
            api_key=get_val_default('Enter API Key ',p.params['firebase']['api_key'])
            user_email = get_val_default('Enter User Email ', p.params['firebase']['user_email'])
            user_password = get_val_default('Enter User Password ', p.params['firebase']['user_password'])
            p.send_firebase_info(url,api_key,user_email,user_password)
        if x[0]=='f':
            if p.status['fan_on']:
                p.set_fan_off()
            else:
                p.set_fan_on()
        if x[0] == 'p':
            p.trigger_beep()
        if x[0] == 't':
            p.trigger_motor_sync()
        if x[0]=='q':
            exit()
        p.push_command()
    else:
        p.pretty_print()

try:
    while True:
        try:
            step_interaction()
        except (ValueError):
            print('Bad input...')
except (KeyboardInterrupt, SystemExit):
    p.stop()
