#!/usr/bin/env python3

import argparse
import time
import math
import sys
from stretch4_body.core.hello_utils import *

print_stretch_re_use()

parser = argparse.ArgumentParser(description='Measure the mechanical backlash for a Feetech wrist joint and calculate the homing offset bias.')
group = parser.add_mutually_exclusive_group(required=True)
group.add_argument("--wrist_yaw", help="Measure wrist yaw backlash", action="store_true")
group.add_argument("--wrist_pitch", help="Measure wrist pitch backlash", action="store_true")
group.add_argument("--wrist_roll", help="Measure wrist roll backlash", action="store_true")

args = parser.parse_args()

if args.wrist_yaw:
    from stretch4_body.subsystem.end_of_arm.wrist_yaw import WristYaw
    joint = WristYaw()
elif args.wrist_pitch:
    from stretch4_body.subsystem.end_of_arm.wrist_pitch import WristPitch
    joint = WristPitch()
elif args.wrist_roll:
    from stretch4_body.subsystem.end_of_arm.wrist_roll import WristRoll
    joint = WristRoll()

if not joint.startup() or not joint.do_ping():
    print('Failed to start joint or ping feetech motor.')
    sys.exit(1)

unwrapped_pos = 0
last_pos = joint.motor.get_pos()

def update_unwrapped():
    global unwrapped_pos, last_pos
    curr_pos = joint.motor.get_pos()
    delta = curr_pos - last_pos
    if delta > 2048:
        delta -= 4096
    elif delta < -2048:
        delta += 4096
    unwrapped_pos += delta
    last_pos = curr_pos

def measure_hardstop(pwm):
    print('Moving to hardstop with PWM %d...' % pwm)
    joint.enable_pwm()
    joint.set_pwm(pwm)
    ts = time.time()
    time.sleep(0.5)
    timeout = False
    
    # Wait until it settles against the hardstop
    while not timeout:
        update_unwrapped()
        if abs(joint.motor.get_vel()) < 100:
            break
        timeout = time.time() - ts > 15.0
        time.sleep(0.05)
    
    time.sleep(0.5)
    update_unwrapped()
    print('Contact at unwrapped pos (ticks):', unwrapped_pos)
    return unwrapped_pos

homing_pwm = joint.params['homing_pwm']
joint.status['is_homing'] = True
joint.bubble_up_comm_exception = True

try:
    print('--- Finding Hardstop 1 (Homing direction) ---')
    T1 = measure_hardstop(homing_pwm)
    
    print('--- Finding Hardstop 2 (Opposite direction) ---')
    # Drive into the opposite hardstop
    T2 = measure_hardstop(-homing_pwm)
    
    measured_span_t = abs(T2 - T1)
    
    range_deg = joint.params['range_deg']
    # Total range output shaft is expected to move in DEGREES
    expected_span_deg = abs(range_deg[1] - range_deg[0])
    
    # Motor encoder expected ticks (from deg_to_rad back to ticks without offset)
    expected_span_rad = deg_to_rad(expected_span_deg)
    
    # Multiply by gear ratio and convert to ticks
    expected_span_t = abs(joint.rad_to_ticks(expected_span_rad * joint.params['gr']))
    
    backlash_t = measured_span_t - expected_span_t

    # homing_pwm dictates the direction we hit T1. 
    # To fix T1 such that it anchors the 'true' output shaft zero,
    # we need to shift T1 towards T2 by backlash_t.
    bias = int(math.copysign(backlash_t, T2 - T1))
    
    print('\n=======================================')
    print('Hardstop 1 (Ticks): %d' % T1)
    print('Hardstop 2 (Ticks): %d' % T2)
    print('Measured Motor Span (Ticks): %d' % measured_span_t)
    print('Expected Output Span (Ticks): %d' % expected_span_t)
    print('---------------------------------------')
    print('Computed Backlash: %d ticks' % backlash_t)
    print('Computed homing_offset_bias_t parameter: %d' % bias)
    print('=======================================')
    print('To apply this bias, update your robot_params_SE4.py or YAML user configuration:')
    print(f"'{joint.name}': {{")
    print(f"    'homing_offset_bias_t': {bias}")
    print("}")
    
finally:
    joint.status['is_homing'] = False
    joint.disable_torque()
    joint.stop()
