#!/usr/bin/env python
from stretch4_body.core.feetech.feetech_SM_servo import *
from stretch4_body.core.hello_utils import *
import argparse

import stretch4_body.core.device
d = stretch4_body.core.device.Device(name='dummy_device',req_params=False) # to initialize logging config

print_stretch_re_use()


parser=argparse.ArgumentParser(description='Reboot all of the Feetech servos on a bus')
parser.add_argument("usb_full_path", help="The full path to the Feetecch USB bus e.g.: /dev/hello-feetech-wrist")

args = parser.parse_args()

m=None
num_reboots=0


try:
    print('Scanning bus...')
    for id in range(25):
        print('ID %d'%id)
        baud = FeetechSMServo.identify_baud_rate(id, args.usb_full_path)
        if baud != -1:
            m = FeetechSMServo(id, args.usb_full_path,baud=baud)
            m.startup()
            #m.startup() #Don't startup as may be in error state
            if (m.do_ping(verbose=False)):
                print('Found device %d on bus %s'%(id,args.usb_full_path))
                m.do_reboot()
                num_reboots=num_reboots+1
            else:
                m.stop()
    if num_reboots==0:
        print('Unable to detect Feetech devices on bus %s for reboot.'%args.usb_full_path)
    else:
        print('Rebooted %d devices'%num_reboots)
except (KeyboardInterrupt, SystemExit):
    if m is not None:
        m.stop()
