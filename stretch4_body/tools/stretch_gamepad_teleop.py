#!/usr/bin/env python3
import argparse
from stretch4_body.core.gamepad_control_mappings import ControlMapping
from stretch4_body.core.gamepad_controller import check_gamepad_teleop_singleton
from stretch4_body.core.gamepad_teleop import GamePadTeleop
from stretch4_body.core.hello_utils import print_stretch_re_use


if __name__ == "__main__":
   
   print_stretch_re_use()
   parser=argparse.ArgumentParser(description='Control Stretch from a GamePad')
   parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
   args=parser.parse_args()

   if not check_gamepad_teleop_singleton(acquire=False):
      print("Gamepad teleop is already running!")
      exit(1)
   
   # Provide a helpful description of the controls for each mapping
   mappings = ControlMapping._get_cycleable_options()
   for mapping in mappings:
      print(mapping.description())

   gamepad_teleop = GamePadTeleop(use_server=not args.direct)
   gamepad_teleop.startup()
   gamepad_teleop.mainloop()

