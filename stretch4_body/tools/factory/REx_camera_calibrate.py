#!/usr/bin/env python3
from stretch4_body.subsystem.cameras.calibrate_extrinsics_lidars import calibrate_extrinsics_camera_lidar
import argparse

from stretch4_body.core.hello_utils import print_stretch_re_use

from stretch4_body.subsystem.cameras.calibrate_extrinsics_cameras import REx_calibrate_extrinsics_cameras
from stretch4_body.subsystem.cameras.calibrate_extrinsics_lidars import REx_calibrate_extrinsics_lidars
from stretch4_body.subsystem.cameras.calibrate_intrinsics_robot_move import REx_calibrate_intrinsics_robot_move
from stretch4_body.subsystem.cameras.calibrate_intrinsics_and_extrinsics import calibrate_intrinsics_and_extrinsics_not_interactive
from stretch4_body.subsystem.cameras.camera_intrinsics_validate_l2_distance import REx_validate_intrinsics
from stretch4_body.subsystem.cameras.calibrate_intrinsics import REx_calibrate_intrinsics

print_stretch_re_use()

def main():
   parser = argparse.ArgumentParser(
      description="Calibrate the robot cameras (intrinsics and extrinsics).",
      add_help=False,
   )
   parser.add_argument(
      "--not_interactive",
      action="store_true",
      help="If this is true, a --replay_from_folder or --replay_last must be passed in. The camera device will not be used, and calibration will be done on previously recorded images.",
   )
   parser.add_argument(
      "--intrinsics",
      action="store_true",
      help="Run only the intrinsics calibration.",
   )
   parser.add_argument(
      "--extrinsics_lidar",
      action="store_true",
      help="Run only the extrinsics camera-lidar calibration.",
   )
   parser.add_argument(
      "--extrinsics_camera",
      action="store_true",
      help="Run only the extrinsics camera-camera calibration.",
   )
   parser.add_argument(
      "--intrinsics_and_extrinsics",
      action="store_true",
      help="Run both the intrinsics and extrinsics calibration.",
   )
   parser.add_argument(
      "--verify",
      action="store_true",
      help="Run the camera intrinsics calibration validation (non-interactive mode). Same as --validate.",
   )
   parser.add_argument(
      "--validate",
      action="store_true",
      help="Run the camera intrinsics calibration validation (non-interactive mode). Same as --verify.",
   )

   parser.add_argument(
      "--replay_from_folder", help="Timestamp of the recording to process"
   )
   parser.add_argument(
      "--replay_last",
      action="store_true",
      help="Use the last recorded folder timestamp inside the provided recording dir. This will load existing images and 'append' new saves to this folder.",
   )

   args, unknown = parser.parse_known_args()

   global_args = ['not_interactive', 'replay_from_folder', 'replay_last']
   has_sub_mode = any(getattr(args, dest) for dest in vars(args) if dest not in global_args and getattr(args, dest))

   wants_help = "-h" in unknown or "--help" in unknown

   if wants_help and not has_sub_mode:
      parser.add_argument("-h", "--help", action="help", default=argparse.SUPPRESS, help="Show this help message and exit. You can also do --<calibration_option> --help to see the help message for a specific calibration option.")
      parser.print_help()
      return

   interactive = not args.not_interactive
   replay = args.replay_from_folder or args.replay_last


   if args.verify or args.validate:
      return REx_validate_intrinsics(interactive=interactive)
   elif args.extrinsics_lidar:
      if replay:
         return calibrate_extrinsics_camera_lidar()
         
      return REx_calibrate_extrinsics_lidars(interactive=interactive)
   elif args.extrinsics_camera:
      return REx_calibrate_extrinsics_cameras(interactive=interactive)
   elif args.intrinsics:
      if replay:
         return REx_calibrate_intrinsics(interactive=interactive)
      
      return REx_calibrate_intrinsics_robot_move(interactive=interactive)
   else:
      print("No calibration type specified. Defaulting to both intrinsics and extrinsics calibration, non-interactive (no rerun windows will appear).")
      return calibrate_intrinsics_and_extrinsics_not_interactive()

if __name__ == "__main__":
   main()