#!/usr/bin/env python3

import argparse
import time
import numpy as np
import stretch4_body.core.hello_utils as hu
from stretch4_body.core.contact_fit import fit_contact_line, ContactFitError
hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Collect data from stepper motor')
parser.add_argument("--no_rs", help="No runstop required",action="store_true")

group1 = parser.add_mutually_exclusive_group(required=True)
group1.add_argument("--arm", help="Test trajectories on the arm joint", action="store_true")
group1.add_argument("--lift", help="Test trajectories on the lift joint", action="store_true")


args=parser.parse_args()

if args.arm:
    import stretch4_body.subsystem.arm as arm
    device = arm.Arm()
    r = device
    if not device.startup():
        exit(1)
    skew_pos = 100
    skew_neg = -100
    print('Arm selected')

if args.lift:
    import stretch4_body.subsystem.lift as lift
    device = lift.Lift()
    r = device
    if not device.startup():
        exit(1)
    skew_pos = 50
    skew_neg = -250
    print('Lift selected')


device.motor.disable_sync_mode()
device.motor.disable_guarded_mode()
device.pull_status()
r.push_command()
xtop = 0.9*device.params['range_m'][1]
xbottom = 0.05*device.params['range_m'][1]
device.set_soft_motion_limit_max(x=xtop)
device.set_soft_motion_limit_min(x=xbottom)
stiffness = 1.0
req_calibration = False
r.push_command()

def switch_motion_params(mode):
    if mode == 0:
        xdes = 0.3
        vdes = device.params['motion']['slow']['vel_m']
        ades = device.params['motion']['slow']['accel_m']
    
    elif mode == 1:
        xdes = 0.5
        vdes = device.params['motion']['default']['vel_m']
        ades = device.params['motion']['default']['accel_m']

    elif mode == 2:
        xdes = 1
        vdes = device.params['motion']['fast']['vel_m']
        ades = device.params['motion']['fast']['accel_m']

    elif mode == 3:
        xdes = 1.5
        vdes = device.params['motion']['max']['vel_m']
        ades = device.params['motion']['max']['accel_m']

    else:
        xdes = 0.5
        vdes = device.params['motion']['default']['vel_m']
        ades = device.params['motion']['default']['accel_m']

    return xdes, vdes, ades


def collect_data(device, mode, r=device):
    cnt = 0
    goUp = True
    xdes,vdes,ades = switch_motion_params(mode)
    device.move_to(x_m=xbottom, v_m=vdes, a_m=ades, stiffness=stiffness, req_calibration=req_calibration)
    r.push_command()
    while abs(device.status['pos']-xbottom) > 0.005:
        device.pull_status()
    print('Iteration number: ', mode+1)
    time.sleep(0.2)
    acc = []
    eff = []
    # The firmware threshold is coeff_acc * cmd.a_des with a_des in motor
    # rad/s^2, so the fit uses the same. (The sample used to mix accel_m x
    # 57.3 and the commanded a_des, which can read 0 while coasting.)
    a_motor = device.translate_m_to_motor_rad(ades)
    while cnt < 2:
        eff.append(device.status['motor']['effort_ticks'])
        acc.append(a_motor)
        device.pull_status()
        eff.append(device.motor.status['effort_ticks'])
        if device.status['pos'] <= 1.1*xbottom and not goUp:
            goUp = True
            cnt = cnt + 1

        if device.status['pos'] >= 0.9*xtop and goUp:
            goUp = False
            cnt = cnt + 1
        
        if goUp:
            device.move_by(x_m=xdes, v_m=vdes, a_m=ades, stiffness=stiffness, req_calibration=req_calibration)

        if not goUp:
            device.move_by(x_m=-xdes, v_m=vdes, a_m=ades, stiffness=stiffness, req_calibration=req_calibration)

        r.push_command()

    device.move_to(x_m=xbottom, v_m=vdes, a_m=ades, stiffness=stiffness, req_calibration=req_calibration)
    r.push_command()

    return max(acc), max(eff), min(eff)

def preprocess_data(eff_pos, eff_neg, skew_pos, skew_neg):
    for i in range(0, len(eff_pos)):
        eff_pos[i] += skew_pos
        eff_neg[i] += skew_neg
    return eff_pos, eff_neg

acc = []
eff_pos = []
eff_neg = []
i = 0
while i < 4:
    a, ep, en = collect_data(device, i, r)
    acc.append(a)
    eff_pos.append(ep)
    eff_neg.append(en)
    i += 1
xdes,vdes,ades = switch_motion_params(1)
device.move_to(x_m=xbottom, v_m=vdes, a_m=ades, stiffness=stiffness, req_calibration=req_calibration)
r.push_command()
while abs(device.status['pos']-xbottom) > 0.005:
    device.pull_status()
print('Data collection ended')
eff_pos, eff_neg = preprocess_data(eff_pos, eff_neg, skew_pos, skew_neg)
# A profile whose peak effort sat at the current clamp says nothing about the
# effort it needs; it is left out of the fit (core/contact_fit.py).
clamp_pos = device.motor.current_to_effort_ticks(abs(device.motor.params['gains']['iMax_pos']))
clamp_neg = device.motor.current_to_effort_ticks(abs(device.motor.params['gains']['iMax_neg']))
try:
    a_p, b_p, kept_p = fit_contact_line(acc, eff_pos, clamp_pos)
    a_n, b_n, kept_n = fit_contact_line(acc, eff_neg, clamp_neg)
except ContactFitError as e:
    print('Calibration not saved:', e)
    device.stop()
    exit(1)
for name, kept in (('pos', kept_p), ('neg', kept_n)):
    for i, k in enumerate(kept):
        if not k:
            print('Profile %d %s effort at the current clamp, left out of the fit' % (i + 1, name))
coeff_pos = np.array([a_p, b_p])
coeff_neg = np.array([a_n, b_n])
print("Old Coeff were : ")
print("Pos Coeff: ", device.motor.params['gains']['coeff_acc_pos'], device.motor.params['gains']['coeff_intercept_pos'])
print("Neg Coeff: ", device.motor.params['gains']['coeff_acc_neg'], device.motor.params['gains']['coeff_intercept_neg'])
print("New Coeff are : ")
print('Pos Coeff = ', coeff_pos)
print("Neg Coeff = ", coeff_neg)
i = input("Please enter y to save the calibration parameters : ")
if i == 'y' or i == 'Y':
    device.write_configuration_param_to_YAML(device.motor.name + '.gains.coeff_acc_pos', coeff_pos[0].item(), force_creation=True)
    device.write_configuration_param_to_YAML(device.motor.name + '.gains.coeff_intercept_pos', coeff_pos[1].item(), force_creation=True)
    device.write_configuration_param_to_YAML(device.motor.name + '.gains.coeff_acc_neg', coeff_neg[0].item(), force_creation=True)
    device.write_configuration_param_to_YAML(device.motor.name + '.gains.coeff_intercept_neg', coeff_neg[1].item(), force_creation=True )
    r.push_command()
device.set_guarded_contact_sensitivity('sensitivity_default')
device.enable_sync_mode()
device.set_soft_motion_limit_max(x=device.params['range_m'][1])
device.set_soft_motion_limit_min(x=device.params['range_m'][0])
device.stop()
r.push_command()