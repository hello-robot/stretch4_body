#!/usr/bin/env python3
from pathlib import Path
import argparse
from stretch4_body.utils.file_access_utils import setup_shared_directory, acquire_lock_if_available
from stretch4_body.core.gamepad_teleop import GamePadTeleop
from stretch4_body.core.hello_utils import print_stretch_re_use

print_stretch_re_use()
parser=argparse.ArgumentParser(description='Control Stretch from a GamePad')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
args=parser.parse_args()

def _check_singleton():
   tmp_file = "/tmp/stretch_gamepad_teleop/gamepad_teleop_singleton.lock"

   setup_shared_directory(Path(tmp_file).parent)
   
   if not acquire_lock_if_available(tmp_file, remove_if_exists_and_unused=True):
      return False
   return True


if __name__ == "__main__":

   if not _check_singleton():
      print("Gamepad teleop is already running!")
      exit(1)
   
   gamepad_teleop = GamePadTeleop(use_server=not args.direct)
   gamepad_teleop.startup()
   gamepad_teleop.mainloop()
