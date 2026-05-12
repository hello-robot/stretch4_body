#!/usr/bin/env python3
import sys
import time
import serial
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

import argparse
parser=argparse.ArgumentParser(description='Query and configure the Firebase system')
args=parser.parse_args()

from stretch4_body.subsystem.power_periph import PowerPeriph

p=PowerPeriph()

if not p.startup():
    exit()
s=None
try:
    s = serial.Serial(port='/dev/hello-esp32', baudrate=115200, timeout=0.1)
except serial.SerialException as e:
    print(f"Error opening serial port /dev/hello-esp32: {e}")
    s = None

def print_esp_output(timeout=1.0,idle_time=1.0):
    global s
    print('')
    print('--- ESP Output ---')
    if s is not None:
        idle_t0 = time.time()
        max_t0 = time.time()
        while time.time() - max_t0 < timeout and time.time() - idle_t0 < idle_time:
            if s.in_waiting > 0:
                try:
                    line = s.readline().decode('utf-8').strip()
                    if line:
                        print(line)
                        idle_t0 = time.time()
                except UnicodeDecodeError:
                    print("Error decoding line")
            else:
                time.sleep(0.01)
    print('-----------')
    print('')


def menu():
    print('------ MENU -------')
    print('e: place esp32 into bootloader')
    print('g: reset esp32')
    print('n: set network info')
    print('f: set firebase info')
    print('s: print status')

    print('q: quit')
    print('-------------------')

def get_val_default(prompt,default):
    x= input(prompt+' ['+default+']: ')
    if len(x)==0:
        x=default
    return x

def step_interaction():
    global s
    menu()
    x=sys.stdin.readline()
    p.pull_status()
    if len(x)>1:
        if x[0]=='m':
            menu()
        if x[0] == 'e':
            p.set_esp_fw_update()
        if x[0]=='g':
            p.set_esp_reset()
            p.push_command()
            time.sleep(1.0)
            try:
                s = serial.Serial(port='/dev/hello-esp32', baudrate=115200, timeout=0.1)
            except serial.SerialException as e:
                print(f"Error opening serial port /dev/hello-esp32: {e}")
                s = None

        if x[0]=='s':
            p.set_esp_status_print()
            p.push_command()
            print_esp_output(10.0,1.0)

        if x[0]=='n':
            ssid=get_val_default('Enter Wifi SSID ',p.params['firebase']['network_ssid'])
            password=get_val_default('Enter Wifi Password ',p.params['firebase']['network_password'])
            p.send_network_info(ssid,password)
            p.push_command()
            print_esp_output(1.0)
        if x[0]=='f':
            url=get_val_default('Enter URL ',p.params['firebase']['network_ssid'])
            api_key=get_val_default('Enter API Key ',p.params['firebase']['api_key'])
            user_email = get_val_default('Enter User Email ', p.params['firebase']['user_email'])
            user_password = get_val_default('Enter User Password ', p.params['firebase']['user_password'])
            p.send_firebase_info(url,api_key,user_email,user_password)
            p.push_command()
            print_esp_output(1.0)
        if x[0]=='q':
            exit()
        
        if s is not None:
            s.reset_input_buffer()
            

    else:
        print('Connected to Network',p.status['connected_to_network'])
        print('Connected to Firebase', p.status['connected_to_firebase'])
        print('dbg',p.status['debug'])
        #print_esp_output(1.0)


try:
    while True:
        try:
            step_interaction()
        except (ValueError):
            print('Bad input...')
except (KeyboardInterrupt, SystemExit):
    p.stop()
    if s is not None:
        s.close()
