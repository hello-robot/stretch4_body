"""
This script performs the full camera calibration pipeline for the robot.
The user answers the first prompt, and leaves the robot for 20 minutes to perform calibration.
At the end of this, the camera-camera and camera-lidar extrinsics are calibrated.

Requires lidar-lidar calibration to be performed first.
Requires the camera calibration tool to be mounted on the robot.
"""
import logging

logger = logging.getLogger(__name__)
import sys
import time

from stretch4_body.subsystem.cameras.calibrate_intrinsics_robot_move import REx_calibrate_intrinsics_robot_move
from stretch4_body.subsystem.cameras.calibrate_extrinsics_lidars import REx_calibrate_extrinsics_lidars
from stretch4_body.subsystem.cameras.calibrate_extrinsics_cameras import REx_calibrate_extrinsics_cameras
from stretch4_body.subsystem.cameras.camera_intrinsics_validate_l2_distance import REx_validate_intrinsics

def calibrate_intrinsics_and_extrinsics_not_interactive(loggin_level = logging.WARNING):
    # Device (imported above) configures the root logger via logging.config.dictConfig()
    # at import time, which installs handlers. logging.basicConfig() is then a no-op
    # (it only takes effect when the root logger has no handlers), so the level must be
    # set directly on the root logger instead.
    logging.getLogger().setLevel(loggin_level)

    print("""
    This script performs the full camera calibration pipeline for the robot.
    The user answers the first prompt, and leaves the robot for 20 minutes to perform calibration.
    At the end of this, the camera-camera and camera-lidar extrinsics are calibrated.

    Please make sure lidar-lidar calibration has been performed first.
    Please make sure the camera calibration tool is mounted on the robot.

    To mount the camera calibration tool, please run:
    ```
    stretch_configure_tool # select the calibration tool
    ```

    After selecting the calibation tool, run:
    ```
    REx_actuator_control --eoa --action off 
    # Take off the end effector
    REx_actuator_control --eoa --action on
    stretch_body_server --restart
    ```
    """)
    ans = input("The robot will move for 20 minutes to perform calibration. Proceed? [y/N]: ")
    if ans.lower() != 'y':
        print("Calibration cancelled.")
        return

    def _print_title(title:str):
        print(f"""
====================================
{title}
====================================
""")

    try:
        _print_title("Starting Intrinsics Calibration")
        # REx_calibrate_intrinsics_robot_move(interactive=False)

        _print_title("Starting Intrinsics Validation")
        validation_passed = False
        errors = None
        for retry in range(3):
            time.sleep(3) # wait for the camera device to come back on the USB bus after we closed it at the end of the last step
            try:
                errors = REx_validate_intrinsics(interactive=False)
                if not any(e > 0.1 or e is None or e == float('inf') for e in errors):
                    validation_passed = True
                    break
            except Exception as e:
                logger.error(f"Intrinsics validation failed: {e=}")
            
            logger.warning(f"Retrying intrinsics validation. Try {retry+1}/3.")

        if not validation_passed:
            raise Exception(f"Intrinsic calibration failed! Distance errors ({errors}) are above 0.1m. (inf = no detection)")

        _print_title("Starting Extrinsics Camera-Camera Calibration")
        # REx_calibrate_extrinsics_cameras(interactive=False)
        
        _print_title("Starting Extrinsics Camera-Lidar Calibration")
        REx_calibrate_extrinsics_lidars(interactive=False)
        
        _print_title("Finished Intrinsics and Extrinsics Calibration")

        exit(0)
    except KeyboardInterrupt:
        logger.error("\nCalibration sequence aborted by user.")
        raise

if __name__ == "__main__":
    calibrate_intrinsics_and_extrinsics_not_interactive()

