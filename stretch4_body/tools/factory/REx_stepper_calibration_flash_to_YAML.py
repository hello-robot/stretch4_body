#!/usr/bin/env python3

import stretch4_body.core.stepper as stepper
import stretch4_body.core.hello_utils as hu
import argparse

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Pull encoder calibration from stepper flash and write to YAML')
parser.add_argument('stepper_name', metavar='stepper_name', type=str, nargs=1,help='Provide the stepper name e.g.: hello-motor-lift')
args=parser.parse_args()

motor = stepper.Stepper('/dev/'+args.stepper_name[0],backend=0) #use python backend)
if not motor.startup():
    exit(1)

print('Reading calibration data from stepper...')
data = motor.read_encoder_calibration_from_flash()
print('Read data of len',len(data))
if len(data)==16384:
    all_zeros=True
    for i in range(16384):
        if data[i]!=0:
            all_zeros=False
    if not all_zeros:
        print('Writing calibration data to YAML...')
        motor.write_encoder_calibration_to_YAML(data)
    else:
        print('Read all zeros from Flash. Aborting write to YAML.')
else:
    print('Invalid read from flash. Aborting write to YAML. Data len of %d'%len(data))




