#!/usr/bin/env python3

import os
import sys

# Ensure the local src directory is in the python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from stretch4_body.core.factory.firmware_available import FirmwareAvailable

def main():
    devices = [
        'hello-motor-arm',
        'hello-motor-lift',
        'hello-motor-omni-0',
        'hello-motor-omni-1',
        'hello-motor-omni-2',
        'hello-pixart-j3',
        'hello-power-periph',
        'hello-esp'
    ]

    use_device = {d: True for d in devices}

    print("Initializing FirmwareAvailable for 8 devices...")
    fa = FirmwareAvailable(use_device)

    print("\nExecuting pretty_print()...")
    fa.pretty_print()

    print("\nTesting get_most_recent_version() for each device...")
    for d in devices:
        recent = fa.get_most_recent_version(d, None)
        if recent is not None:
            print(f"Device: {d:20} -> Most recent version: {recent.to_string()}")
        else:
            print(f"Device: {d:20} -> Most recent version: None found")

if __name__ == '__main__':
    main()
