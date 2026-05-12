#!/usr/bin/env python3

import os.path
import sys
import argparse
import time
import click
import glob
import requests
from colorama import Fore, Style
from stretch4_body.core.factory.firmware_available import FirmwareAvailable
import stretch4_body.core.factory.hello_device_utils as hdu
import stretch4_body.core.factory.firmware_utils as fwu
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

parser = argparse.ArgumentParser(description='Tool to directly flash Stretch firmware to a ttyACM device', )
parser.add_argument("--verbose", help="Print verbose output of arduino-cli", action="store_true")
group = parser.add_mutually_exclusive_group()
group.add_argument("--map", help="Print mapping from ttyACMx to Hello devices", action="store_true")
group.add_argument("--available", help="Print available firmware versions for download", action="store_true")
group.add_argument('--device', nargs=1, type=str, help='Specify the usb path to the device to flash (e.g. /dev/ttyACM0)')
group.add_argument('--power_cycle', nargs=1, type=str, help='Power cycle a board. E.g, --power_cycle hello-motor-arm')
args = parser.parse_args()

if args.power_cycle:
    device_name = args.power_cycle[0]
    mapping = {
        'hello-motor-arm': 'arm',
        'hello-motor-lift': 'lift',
        'hello-motor-omni-0': 'omni-0',
        'hello-motor-omni-1': 'omni-1',
        'hello-motor-omni-2': 'omni-2'
    }
    if device_name in mapping:
        from stretch4_body.subsystem.power_periph import PowerPeriph
        p = PowerPeriph()
        if p.startup():
            actuator = mapping[device_name]
            print(Fore.CYAN + f'Powering off {actuator}...' + Style.RESET_ALL)
            p.actuator_control(actuator, enable=False)
            time.sleep(1.0)
            print(Fore.CYAN + f'Powering on {actuator}...' + Style.RESET_ALL)
            p.actuator_control(actuator, enable=True)
            time.sleep(1.0)
            p.stop()
            print(Fore.GREEN + f"Successfully power cycled {device_name}." + Style.RESET_ALL)
    else:
        print(Fore.RED + f"Cannot power cycle {device_name}. Only stepper motors and ESP32 can be power cycled." + Style.RESET_ALL)

#arduino-cli upload -p /dev/ttyACM1 --fqbn hello-robot:samd:hello_robot_hello_stepper2 -i /tmp/hello_stepper2_v0.1.0p8.uf2
if args.map:
    fwu.print_tty_mapping()

if args.available:
    devices = [
        'hello-motor-arm',
        'hello-motor-lift',
        'hello-motor-omni-0',
        'hello-motor-omni-1',
        'hello-motor-omni-2',
        'hello-pixart-j3',
        'hello-power-periph',
        'hello-esp32'
    ]
    use_device = {d: True for d in devices}
    fa = FirmwareAvailable(use_device)
    fa.pretty_print()

if args.device:
    import subprocess
    port = args.device[0]
    
    # Identify device from port
    tty_name = port.split('/')[-1]
    mapping = hdu.get_hello_ttyACMx_mapping()
    device_name = None
    
    if tty_name in mapping['hello']:
        device_name = tty_name
        port = f"/dev/{mapping['hello']
        [tty_name]}"
    elif tty_name in mapping['ACMx']:
        device_name = mapping['ACMx'][tty_name]
        
    if not device_name:
        print(Fore.RED + f"Could not identify Hello device at port {port}." + Style.RESET_ALL)
        sys.exit(1)
        
    print(Fore.CYAN + f"Identified device as {device_name}" + Style.RESET_ALL)
    
    a = FirmwareAvailable({device_name: True})
    versions = a.versions.get(device_name, [])
    if not versions:
        print(Fore.RED + f"No firmware versions found for {device_name}" + Style.RESET_ALL)
        sys.exit(1)
        
    versions.sort(reverse=True)
    
    print(Fore.CYAN + f"Available firmware versions for {device_name}:" + Style.RESET_ALL)
    for i, v in enumerate(versions):
        default_str = " [Default]" if i == 0 else ""
        print(f"  {i}) {v.to_string()}{default_str}")
        
    try:
        selection = input(Fore.CYAN + "Select firmware index [0]: " + Style.RESET_ALL).strip()
        idx = 0 if not selection else int(selection)
        if idx < 0 or idx >= len(versions):
            raise ValueError
    except ValueError:
        print(Fore.RED + "Invalid selection." + Style.RESET_ALL)
        sys.exit(1)
    except KeyboardInterrupt:
        print("")
        sys.exit(1)
        
    version_str = versions[idx].to_string()
    
    if not fwu.flash_firmware_update(device_name, version_str, port, args.verbose):
        sys.exit(1)
