#!/usr/bin/env python3

import stretch4_body.core.stepper as stepper
import stretch4_body.core.hello_utils as hu
import argparse

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Pull gains from flash and print to console')
parser.add_argument('stepper_name', metavar='stepper_name', type=str, nargs=1,help='Provide the stepper name e.g.: hello-motor-lift')
args=parser.parse_args()

motor = stepper.Stepper('/dev/'+args.stepper_name[0])
if not motor.startup():
    exit(1)

print('Reading gains data from stepper...')
motor.read_gains_from_flash()
motor.pull_status()
print(motor.gains_flash)
motor.stop()





