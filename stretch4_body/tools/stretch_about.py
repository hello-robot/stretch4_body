#!/usr/bin/env python3

from stretch4_body.core import robot_params
import argparse
parser=argparse.ArgumentParser(description='Display model and serial number information')
args=parser.parse_args()


r=robot_params.RobotParams
robot_info = r._robot_params['robot']
batch_name = robot_info['batch_name']
serial_number = robot_info['serial_no']
model=robot_info['model_name']

batch_name_string = 'Stretch Batch Name: {0}'.format(batch_name)
serial_number_string = 'Stretch Serial Number: {0}'.format(serial_number)
model_string = 'Stretch Model: {0}'.format(model)

print(batch_name_string)
print(serial_number_string)
print(model_string)
