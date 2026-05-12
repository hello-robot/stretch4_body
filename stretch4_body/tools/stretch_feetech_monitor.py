#!/usr/bin/env python3

import time
import sys
import argparse
from stretch4_body.core.feetech.feetech_SM_servo import FeetechSMServo
from stretch4_body.core.feetech.port_handler import PortHandler
from stretch4_body.core.feetech.sms_sts import sms_sts
import stretch4_body.core.hello_utils as hu

hu.print_stretch_re_use()

def main():
    parser = argparse.ArgumentParser(description='Monitor Feetech motors status')
    parser.add_argument("--usb", help="The full path to Feetech USB bus", default='/dev/hello-feetech-wrist')
    parser.add_argument("--ids", help="Comma separated list of IDs", default='20,21,22,23')
    parser.add_argument("--rate", help="Update rate in Hz", type=float, default=1.0)
    args = parser.parse_args()

    usb = args.usb
    try:
        motor_ids = [int(x) for x in args.ids.split(',')]
    except ValueError:
        print("Invalid IDs format. Use comma separated numbers like 20,21,22,23")
        sys.exit(1)

    print(f"Opening port {usb}...")
    port_handler = PortHandler(usb)
    if not port_handler.openPort():
        print(f"Failed to open port {usb}")
        sys.exit(1)
    
    baud = 1000000
    if not port_handler.setBaudRate(baud):
        print(f"Failed to set baud rate {baud}")
        port_handler.closePort()
        sys.exit(1)

    servos = []
    print("Connecting to motors...")
    for motor_id in motor_ids:
        s = FeetechSMServo(motor_id, usb, port_handler=port_handler, baud=baud)
        # Manually set packet handler as we are managing the port externally
        s.packet_handler = sms_sts(s.port_handler)
        s.hw_valid = True
        servos.append(s)

    print("\nStarting monitor (Ctrl-C to exit)...")
    
    header = f"{'ID':<5} | {'Temp(C)':<8} | {'Pos':<10} | {'Voltage(V)':<10} | {'Load(%)':<10} | {'Error':<10}"

    try:
        while True:
            # Clear screen and home cursor
            print("\033[2J\033[H", end="") 
            print(f"Monitoring Feetech Motors on {usb} (baud {baud})")
            print(header)
            print("-" * len(header))

            for m in servos:
                try:
                    # We are already connected, just read values
                    # Note: These calls typically read registers. 
                    temp = m.get_temp()
                    pos = m.get_pos()
                    volts = m.get_voltage()/10.0
                    load = m.get_load_pct()
                    err = m.get_hardware_error()
                    
                    print(f"{m.id:<5} | {temp:<8} | {pos:<10} | {volts:<10.1f} | {load:<10.1f} | {err:<10}")
                except Exception as e:
                    print(f"{m.id:<5} | {'Error':<8} | {'-':<10} | {'-':<10} | {'-':<10} | {str(e):<10}")

            time.sleep(1.0/args.rate)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        port_handler.closePort()

if __name__ == '__main__':
    main()
