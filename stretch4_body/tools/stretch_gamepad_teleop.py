#!/usr/bin/env python3
from __future__ import print_function
from stretch4_body.core.gamepad_teleop import GamePadTeleop
from stretch4_body.core.hello_utils import print_stretch_re_use
import argparse

print_stretch_re_use()
parser=argparse.ArgumentParser(description='Control Stretch from a GamePad')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

if __name__ == "__main__":
   gamepad_teleop = GamePadTeleop(use_server=not args.direct)
   gamepad_teleop.startup()
   gamepad_teleop.mainloop()