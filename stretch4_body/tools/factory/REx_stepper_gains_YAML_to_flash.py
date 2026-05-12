#!/usr/bin/env python3

import stretch4_body.core.stepper as stepper
import stretch4_body.core.hello_utils as hu
import argparse

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Push stepper gains from YAML to flash memory')
parser.add_argument('stepper_name', metavar='stepper_name', type=str, nargs=1,help='Provide the stepper name e.g.: hello-motor-lift')
args=parser.parse_args()

motor = stepper.Stepper('/dev/'+args.stepper_name[0])
if not motor.startup():
    exit(1)

motor.write_gains_to_flash()
motor.push_command()
print('Gains written to flash')
print(motor.gains)
motor.stop()
