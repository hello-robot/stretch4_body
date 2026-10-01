#!/usr/bin/env python3
import argparse
import struct
import time
import stretch4_body.core.hello_utils as hu

hu.print_stretch_re_use()

parser = argparse.ArgumentParser(description='Turn the head lidar power on or off via the Pimu')
parser.add_argument('action', nargs='?', choices=['on', 'off'], help='Power the lidars on or off')
parser.add_argument('--status', help='Report lidar spin hours and remaining life', action='store_true')
parser.add_argument('-d', '--direct', help='Use direct API (no server)', action='store_true')
args = parser.parse_args()

if args.action is None and not args.status:
    parser.error('specify on, off, or --status')

LIDAR_LIFE_HOURS = 10000
PTC_GET_LIDAR_STATUS = 0x09
# JT128 PTC 0x09 (Get LiDAR Status) layout, big-endian
STATUS_UPTIME_OFFSET = 0          # uint32, seconds since power-up
STATUS_MOTOR_RPM_OFFSET = 4       # uint16
STATUS_TOTAL_OP_HOURS_OFFSET = 44 # uint32, lifetime operating hours


def print_status():
    from stretch4_pyhesai_wrapper.ptc_client import _run, LEFT_LIDAR_IP, RIGHT_LIDAR_IP, HesaiPtcError
    for name, ip in [('Left', LEFT_LIDAR_IP), ('Right', RIGHT_LIDAR_IP)]:
        try:
            raw = _run(lambda c: c.query_command(PTC_GET_LIDAR_STATUS, ''), ip)
        except HesaiPtcError as e:
            print('%s lidar (%s): not reachable (powered off?) - %s' % (name, ip, e))
            continue
        uptime_s = struct.unpack_from('>I', raw, STATUS_UPTIME_OFFSET)[0]
        rpm = struct.unpack_from('>H', raw, STATUS_MOTOR_RPM_OFFSET)[0]
        hours = struct.unpack_from('>I', raw, STATUS_TOTAL_OP_HOURS_OFFSET)[0]
        left = max(LIDAR_LIFE_HOURS - hours, 0)
        print('%s lidar (%s):' % (name, ip))
        print('    Motor speed:     %d RPM' % rpm)
        print('    Uptime:          %.1f min' % (uptime_s / 60.0))
        print('    Total spin time: %d h' % hours)
        print('    Life remaining:  %d h of %d h (%.1f%% used)' % (left, LIDAR_LIFE_HOURS, 100.0 * hours / LIDAR_LIFE_HOURS))


if args.action is None:
    print_status()
    exit(0)

if not args.direct:
    from stretch4_body.robot.robot_client import PowerPeriphClient as PowerPeriph
else:
    from stretch4_body.subsystem.power_periph import PowerPeriph

p = PowerPeriph()
if not p.startup():
    exit(1)

try:
    if args.action == 'on':
        p.set_lidar_on()
    else:
        p.set_lidar_off()
    p.push_command()
    time.sleep(0.25)
    print('Lidars turned %s' % args.action)
finally:
    p.stop()

if args.status:
    print_status()
