#!/usr/bin/env python3
from __future__ import print_function
import sys
import argparse
import time
import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.power_periph import PowerPeriphDefn

hu.print_stretch_re_use()

parser=argparse.ArgumentParser(description='Push eye animations to the robot')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

if not args.direct:
    from stretch4_body.robot.robot_client import PowerPeriphClient as PowerPeriph
else:
    from stretch4_body.subsystem.power_periph import PowerPeriph

p=PowerPeriph()

if not p.startup():
    print("Failed to start PowerPeriph")
    sys.exit(1)

def main():
    print('Available Eye Animations:')
    # Sort them by idx for nicer printing
    animations = sorted(PowerPeriphDefn.EYE_ANIM_NAME_TO_IDX.items(), key=lambda item: item[1])
    for name, idx in animations:
        print('  {}: {}'.format(idx, name))
    print('')

    left_input = input('Enter Left Eye Animation IDX (or press Enter to skip): ')
    right_input = input('Enter Right Eye Animation IDX (or press Enter to skip): ')
    intensity_input = input('Enter Intensity [0-255] (or press Enter for 255): ')
    r_input = input('Enter Red [0-255] (or press Enter for 255): ')
    g_input = input('Enter Green [0-255] (or press Enter for 255): ')
    b_input = input('Enter Blue [0-255] (or press Enter for 255): ')

    left_idx = int(left_input) if left_input.strip() else None
    right_idx = int(right_input) if right_input.strip() else None
    intensity = int(intensity_input) if intensity_input.strip() else 255
    r = int(r_input) if r_input.strip() else 255
    g = int(g_input) if g_input.strip() else 255
    b = int(b_input) if b_input.strip() else 255

    print('Sending Left: {}, Right: {}, Intensity: {}, RGB: ({}, {}, {})'.format(left_idx, right_idx, intensity, r, g, b))

    p.set_eye_animation(left_idx=left_idx, right_idx=right_idx, intensity=intensity, r=r, g=g, b=b)
    p.push_command()
    time.sleep(0.5)

if __name__ == '__main__':
    try:
        while True:
            try:
                main()
            except Exception as e:
                print("Error:", e)
    except (KeyboardInterrupt):
        pass
    finally:
        p.stop()
