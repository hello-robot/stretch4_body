#!/usr/bin/env python3

import argparse
import time
import numpy as np
from scipy.optimize import curve_fit
import stretch4_body.subsystem.omnibase as base
import stretch4_body.subsystem.power_periph as pimu
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Collect data from base stepper motors')
parser.add_argument("--no_rs", help="No runstop required",action="store_true")

args=parser.parse_args()


b = base.OmniBase()
p = pimu.PowerPeriph()
p.startup()
if not b.startup():
    exit(1)
skew_pos = 400
skew_neg = -400
print('Omnibase selected')
b.set_guarded_contact_sensitivity('off')
b.push_command()

print("The robot will spin at different speeds to perform the calibration. Please ensure the surroundings are clear and the arm is completely extended.")
c = input("Enter 'Y' to begin or 'N' to quit: ")

if c == 'y' or c == 'Y':
    pass
else:
    raise SystemExit("Calibration cancelled")

def switch_motion_params(mode):
    if mode == 0:
        xdes = 3.14
        vdes = b.params['motion']['slow']['vel_w_r']
        ades = b.params['motion']['slow']['accel_w_r'] * 0.8
    
    elif mode == 1:
        xdes = 3.14
        vdes = b.params['motion']['default']['vel_w_r']
        ades = b.params['motion']['default']['accel_w_r'] * 0.8

    elif mode == 2:
        xdes = 3.14
        vdes = b.params['motion']['fast']['vel_w_r']
        ades = b.params['motion']['fast']['accel_w_r'] * 0.8

    elif mode == 3:
        xdes = 3.14
        vdes = b.params['motion']['max']['vel_w_r']
        ades = b.params['motion']['max']['accel_w_r'] * 0.8

    else:
        xdes = 3.14
        vdes = b.params['motion']['default']['vel_w_r']
        ades = b.params['motion']['default']['accel_w_r'] * 0.8

    return xdes, vdes, ades


def collect_data(mode):
    acc0 = []
    eff0 = []
    acc1 = []
    eff1 = []
    acc2 = []
    eff2 = []
    xdes,vdes,ades = switch_motion_params(mode)
    time.sleep(1)
    b.rotate_by(-1*xdes, vdes, ades)
    p.trigger_motor_sync()
    b.push_command()
    print('Iteration number: ', mode+1)
    time.sleep(0.1)
    b.pull_status()
    while abs(b.wheels[0].status['vel']) > 0.1 and abs(b.wheels[1].status['vel']) > 0.1 and abs(b.wheels[2].status['vel']) > 0.1:
        b.pull_status()
        acc0.append(ades*57.29577951308232)
        eff0.append(b.wheels[0].status['effort_ticks'])
        acc1.append(ades*57.29577951308232)
        eff1.append(b.wheels[1].status['effort_ticks'])
        acc2.append(ades*57.29577951308232)
        eff2.append(b.wheels[2].status['effort_ticks'])
    time.sleep(1)
    b.rotate_by(1*xdes, vdes, ades)
    p.trigger_motor_sync()
    b.push_command()
    time.sleep(0.1)
    b.pull_status()
    while abs(b.wheels[0].status['vel']) > 0.1 and abs(b.wheels[1].status['vel']) > 0.1 and abs(b.wheels[2].status['vel']) > 0.1:
        b.pull_status()
        acc0.append(ades*57.29577951308232)
        eff0.append(b.wheels[0].status['effort_ticks'])
        acc1.append(ades*57.29577951308232)
        eff1.append(b.wheels[1].status['effort_ticks'])
        acc2.append(ades*57.29577951308232)
        eff2.append(b.wheels[2].status['effort_ticks'])

    acc = np.zeros(3)
    eff_pos = np.zeros(3)
    eff_neg = np.zeros(3)

    acc[0] = max(acc0)
    eff_pos[0] = max(eff0)
    eff_neg[0] = min(eff0)

    acc[1] = max(acc1)
    eff_pos[1] = max(eff1)
    eff_neg[1] = min(eff1)

    acc[2] = max(acc2)
    eff_pos[2] = max(eff2)
    eff_neg[2] = min(eff2)

    return acc, eff_pos, eff_neg

def preprocess_data(eff_pos, eff_neg, skew_pos, skew_neg):
    for i in range(0, len(eff_pos)):
        eff_pos[i] += skew_pos
        eff_neg[i] += skew_neg
    return eff_pos, eff_neg

def model(acc, a, b):
    return a*acc + b;    

acc0 = []
eff_pos0 = []
eff_neg0 = []

acc1 = []
eff_pos1 = []
eff_neg1 = []

acc2 = []
eff_pos2 = []
eff_neg2 = []
i = 0
while i < 4:
    a, ep, en = collect_data(i)
    acc0.append(a[0])
    eff_pos0.append(ep[0])
    eff_neg0.append(en[0])
    acc1.append(a[1])
    eff_pos1.append(ep[1])
    eff_neg1.append(en[1])
    acc2.append(a[2])
    eff_pos2.append(ep[2])
    eff_neg2.append(en[2])
    i += 1
xdes,vdes,ades = switch_motion_params(1)
b.rotate_by(0, vdes, ades)
b.push_command()
print('Data collection ended')
eff_pos0, eff_neg0 = preprocess_data(eff_pos0, eff_neg0, skew_pos, skew_neg)
eff_pos1, eff_neg1 = preprocess_data(eff_pos1, eff_neg1, skew_pos, skew_neg)
eff_pos2, eff_neg2 = preprocess_data(eff_pos2, eff_neg2, skew_pos, skew_neg)

coeff_pos0, _ = curve_fit(model, acc0, eff_pos0)
coeff_neg0, _ = curve_fit(model, acc0, eff_neg0)
coeff_pos1, _ = curve_fit(model, acc1, eff_pos1)
coeff_neg1, _ = curve_fit(model, acc1, eff_neg1)
coeff_pos2, _ = curve_fit(model, acc2, eff_pos2)
coeff_neg2, _ = curve_fit(model, acc2, eff_neg2)
print("Acc: ", acc0)
print("EffP: ", eff_pos0)
print("EffN: ", eff_neg0)
print("Wheel 0: ")
print("Old Coeff were : ")
print("Pos Coeff: ", b.wheels[0].params['gains']['coeff_acc_pos'], b.wheels[0].params['gains']['coeff_intercept_pos'])
print("Neg Coeff: ", b.wheels[0].params['gains']['coeff_acc_neg'], b.wheels[0].params['gains']['coeff_intercept_neg'])
print("New Coeff are : ")
print('Pos Coeff = ', coeff_pos0)
print("Neg Coeff = ", coeff_neg0)

print("Wheel 1: ")
print("Old Coeff were : ")
print("Pos Coeff: ", b.wheels[1].params['gains']['coeff_acc_pos'], b.wheels[1].params['gains']['coeff_intercept_pos'])
print("Neg Coeff: ", b.wheels[1].params['gains']['coeff_acc_neg'], b.wheels[1].params['gains']['coeff_intercept_neg'])
print("New Coeff are : ")
print('Pos Coeff = ', coeff_pos1)
print("Neg Coeff = ", coeff_neg1)

print("Wheel 2: ")
print("Old Coeff were : ")
print("Pos Coeff: ", b.wheels[2].params['gains']['coeff_acc_pos'], b.wheels[2].params['gains']['coeff_intercept_pos'])
print("Neg Coeff: ", b.wheels[2].params['gains']['coeff_acc_neg'], b.wheels[2].params['gains']['coeff_intercept_neg'])
print("New Coeff are : ")
print('Pos Coeff = ', coeff_pos2)
print("Neg Coeff = ", coeff_neg2)

i = input("Please enter y to save the calibration parameters : ")
if i == 'y' or i == 'Y':
    b.write_configuration_param_to_YAML(b.wheels[0].name + '.gains.coeff_acc_pos', coeff_pos0[0].item(), force_creation=True)
    b.write_configuration_param_to_YAML(b.wheels[0].name + '.gains.coeff_intercept_pos', coeff_pos0[1].item(), force_creation=True)
    b.write_configuration_param_to_YAML(b.wheels[0].name + '.gains.coeff_acc_neg', coeff_neg0[0].item(), force_creation=True)
    b.write_configuration_param_to_YAML(b.wheels[0].name + '.gains.coeff_intercept_neg', coeff_neg0[1].item(), force_creation=True )

    b.write_configuration_param_to_YAML(b.wheels[1].name + '.gains.coeff_acc_pos', coeff_pos1[0].item(), force_creation=True)
    b.write_configuration_param_to_YAML(b.wheels[1].name + '.gains.coeff_intercept_pos', coeff_pos1[1].item(), force_creation=True)
    b.write_configuration_param_to_YAML(b.wheels[1].name + '.gains.coeff_acc_neg', coeff_neg1[0].item(), force_creation=True)
    b.write_configuration_param_to_YAML(b.wheels[1].name + '.gains.coeff_intercept_neg', coeff_neg1[1].item(), force_creation=True )
    
    b.write_configuration_param_to_YAML(b.wheels[2].name + '.gains.coeff_acc_pos', coeff_pos2[0].item(), force_creation=True)
    b.write_configuration_param_to_YAML(b.wheels[2].name + '.gains.coeff_intercept_pos', coeff_pos2[1].item(), force_creation=True)
    b.write_configuration_param_to_YAML(b.wheels[2].name + '.gains.coeff_acc_neg', coeff_neg2[0].item(), force_creation=True)
    b.write_configuration_param_to_YAML(b.wheels[2].name + '.gains.coeff_intercept_neg', coeff_neg2[1].item(), force_creation=True )
    
b.set_guarded_contact_sensitivity('sensitivity_default')
b.stop()
p.stop()
b.push_command()