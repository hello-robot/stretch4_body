#!/usr/bin/env python3

import stretch4_body.core.stepper as stepper
import stretch4_body.subsystem.power_periph as power_periph
import stretch4_body.core.hello_utils as hu
import argparse
import time


parser = argparse.ArgumentParser(description='Read or Write Stepper Type to Flash Memory of Stepper Boards \n eg:(REx_stepper_type --write --arm)')

group = parser.add_mutually_exclusive_group()
group.add_argument("--write", help="Write Stepper Type to Flash Memory", action="store_true")
group.add_argument("--read", help="Read Stepper Type from Flash Memory", action="store_true")


parser.add_argument("--arm", help="Read/Write stepper type from arm flash memory", action="store_true")
parser.add_argument("--lift", help="Read/Write stepper type from lift flash memory", action="store_true")
parser.add_argument("--wheel_0", help="Read/Write stepper type from wheel_0 flash memory", action="store_true")
parser.add_argument("--wheel_1", help="Read/Write stepper type from wheel_1 flash memory", action="store_true")
parser.add_argument("--wheel_2", help="Read/Write stepper type from wheel_2 flash memory", action="store_true")

args = parser.parse_args()

def stepper_type():
    if args.arm or args.lift or args.wheel_0 or args.wheel_1 or args.wheel_2:
        use_device={'hello-motor-lift':args.lift,'hello-motor-arm':args.arm,
                    'hello-motor-omni-0':args.wheel_0,
                    'hello-motor-omni-1':args.wheel_1,
                    'hello-motor-omni-2':args.wheel_2}
    else:
        use_device = {'hello-motor-lift': True, 'hello-motor-arm': True,
                      'hello-motor-omni-0': True,
                      'hello-motor-omni-1': True,
                      'hello-motor-omni-2': True}

    for i in use_device:
        if use_device[i]:
            motor = stepper.Stepper(f'/dev/{i}')
            if not motor.startup():
                print(f"Error with communication to {i}")
                exit(1)
            if args.write:
                print(f"Now setting {i} stepper type to flash...")
                motor.write_stepper_type_to_flash(i)
                time.sleep(1)
                motor.read_stepper_type_from_flash()
                if i == motor.board_info['stepper_type']:
                    print(f"Success {i} stepper type to flash\n")
                else:
                    print(f"Error setting stepper type to {i}, please try again\n")
                motor.stop()
                time.sleep(1)
            if args.read:
                print(f"Now reading stepper_type from {i}....")
                motor.read_stepper_type_from_flash()
                time.sleep(1)

                print(f"stepper_type == {motor.board_info['stepper_type']}\n")
                time.sleep(1)
                motor.stop()
stepper_type()

