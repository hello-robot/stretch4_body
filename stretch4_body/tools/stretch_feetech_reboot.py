#!/usr/bin/env python3

import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()
import time

from stretch4_body.subsystem.power_periph import PowerPeriph

p=PowerPeriph()
if not p.startup():
    exit()

print('Powering off Feetech devics...')
p.actuator_control( 'eoa', enable=False)
time.sleep(0.5)
print('Powering on Feetech devices...')
p.actuator_control( 'eoa', enable=True)
time.sleep(0.5)
p.stop()

print('Feetech servo reboot complete. You will need to re-home servos now.')
