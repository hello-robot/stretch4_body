#!/usr/bin/env python3

import argparse
from stretch4_body.core.factory.firmware_available import FirmwareAvailable
from stretch4_body.core.factory.firmware_recommended import FirmwareRecommended
from stretch4_body.core.factory.firmware_installed import FirmwareInstalled
from stretch4_body.core.factory.firmware_updater import FirmwareUpdater
import os
import click
import stretch4_body.core.factory.hello_device_utils as hdu
import stretch4_body.core.factory.firmware_utils as fwu

parser = argparse.ArgumentParser(description='Upload Stretch firmware to microcontrollers')

group = parser.add_mutually_exclusive_group()
group.add_argument("--current", help="Display the currently installed firmware versions", action="store_true")
group.add_argument("--available", help="Display the available firmware versions", action="store_true")
group.add_argument("--recommended", help="Display the recommended firmware", action="store_true")
group.add_argument("--install", help="Install the recommended firmware", action="store_true")
group.add_argument("--install_version", help="Install a specific firmware version", action="store_true")
parser.add_argument("--map", help="Print mapping from ttyACMx to Hello device", action="store_true")

parser.add_argument("--pimu", help="Upload Pimu (Power Periph) firmware", action="store_true")
parser.add_argument("--arm", help="Upload Arm Stepper firmware", action="store_true")
parser.add_argument("--lift", help="Upload Lift Stepper firmware", action="store_true")
parser.add_argument("--wheel_0", help="Upload Omni Wheel 0 Stepper firmware", action="store_true")
parser.add_argument("--wheel_1", help="Upload Omni Wheel 1 Stepper firmware", action="store_true")
parser.add_argument("--wheel_2", help="Upload Omni Wheel 2 Stepper firmware", action="store_true")
parser.add_argument("--pixart", help="Upload Pixart J3 firmware", action="store_true")
parser.add_argument("--esp32", help="Upload ESP32 firmware", action="store_true")
parser.add_argument("--no_prompts", help="Proceed without prompts", action="store_true")
parser.add_argument("--verbose", help="Verbose output", action="store_true")
parser.add_argument("--dummy", help="Simulate flashing without arduino-cli", action="store_true")
args = parser.parse_args()

import sys
from stretch4_body.core.device import Device

d = Device(req_params=False)
is_unh = d.robot_params.get('robot', {}).get('model_name') == 'SE4UNH'

if is_unh and args.arm:
    print("Error: The arm device is not supported on the SE4UNH robot.", file=sys.stderr)
    sys.exit(1)

if args.arm or args.lift or args.wheel_0 or args.wheel_1 or args.wheel_2 or args.pimu or args.pixart or args.esp32:
    use_device={'hello-esp32':args.esp32,'hello-motor-lift':args.lift,'hello-motor-arm':args.arm, 'hello-motor-omni-0':args.wheel_0, 'hello-motor-omni-1':args.wheel_1, 'hello-motor-omni-2':args.wheel_2,'hello-power-periph':args.pimu, 'hello-pixart-j3':args.pixart}
else:
    use_device = {'hello-esp32':True,'hello-motor-arm': not is_unh, 'hello-motor-omni-0': True, 'hello-motor-omni-1': True, 'hello-motor-omni-2': True, 'hello-power-periph': True, 'hello-pixart-j3': True,'hello-motor-lift': True}


if args.map:
    fwu.print_tty_mapping()
    exit()

if args.current:
    c = FirmwareInstalled(use_device)
    c.pretty_print()
    exit()

if args.recommended:
    r = FirmwareRecommended(use_device)
    r.pretty_print()
    r.print_recommended_args()
    exit()

if args.available:
    a = FirmwareAvailable(use_device)
    a.pretty_print()
    exit()


if args.install or args.install_version:
    u = FirmwareUpdater(use_device, args)
    success = u.run()
    exit(0 if success else 1)
else:
    parser.print_help()

