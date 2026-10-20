#!/usr/bin/env python3
"""
Print the current position of a single Feetech servo in ticks and degrees.

Torque is not touched, so the joint can be positioned by hand beforehand.
Stop stretch_body_server first, it holds the servo bus open.
"""
import argparse
import sys

from stretch4_body.core.feetech.feetech_SM_servo import FeetechSMServo, FeetechCommError

parser = argparse.ArgumentParser(description='Print current servo position in ticks and degrees')
parser.add_argument('--id', type=int, default=26, help='Servo ID (default: 26)')
parser.add_argument('--usb', type=str, default='/dev/hello-feetech-wrist', help='Servo bus')
parser.add_argument('--baud', type=int, default=1000000, help='Baud rate')
args = parser.parse_args()

servo = FeetechSMServo(id=args.id, usb=args.usb, baud=args.baud)
if not servo.create_port_handler():
    print('Unable to open %s. Is stretch_body_server stopped?' % args.usb)
    sys.exit(1)

try:
    angular_res = servo.get_angular_res()
    pos = servo.get_pos()
    deg_per_tick = 360.0 * angular_res / 4096.0
    print('Servo %d: %d ticks = %.2f deg  (angular_resolution=%d, %.5f deg/tick)'
          % (args.id, pos, pos * deg_per_tick, angular_res, deg_per_tick))
except FeetechCommError:
    print('No response from servo %d on %s. Check the ID and baud.' % (args.id, args.usb))
finally:
    servo.stop()
