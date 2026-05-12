#!/usr/bin/env python3

import stretch4_body.core.stepper as stepper
import stretch4_body.core.hello_utils as hu
import argparse

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Push encoder calibration from YAML to stepper flash memory')
parser.add_argument('stepper_name', metavar='stepper_name', type=str, nargs=1,help='Provide the stepper name e.g.: hello-motor-lift')
args=parser.parse_args()

motor = stepper.Stepper('/dev/'+args.stepper_name[0],backend=0) #use python backend
if not motor.startup():
    exit(1)

motor.write_gains_to_flash()
motor.push_command()
print('Gains written to flash')

print('Reading calibration data from YAML...')
data=motor.read_encoder_calibration_from_YAML()
if len(data)==16384:
    all_zeros=True
    for i in range(16384):
        if data[i]!=0:
            all_zeros=False
    if not all_zeros:
        print('Writing calibration data to Flash...')
        motor.write_encoder_calibration_to_flash(data)
        print('Successful write of FLASH. Resetting board now.')
        motor.board_reset()
        motor.push_command()
    else:
        print('Read all zeros from YAML. Aborting write to Flash.')
else:
    print('Invalid read from YAML. Aborting write to Flash. Data len of %d'%len(data))


