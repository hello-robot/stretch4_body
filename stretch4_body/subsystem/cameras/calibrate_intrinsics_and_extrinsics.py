"""
This script performs the full camera calibration pipeline for the robot.
The user answers the first prompt, and leaves the robot for 20 minutes to perform calibration.
At the end of this, the camera-camera and camera-lidar extrinsics are calibrated.

Requires lidar-lidar calibration to be performed first.
Requires the camera calibration tool to be mounted on the robot.
"""
import sys
import time

from stretch4_body.subsystem.cameras.calibrate_intrinsics_robot_move import REx_calibrate_intrinsics_robot_move
from stretch4_body.subsystem.cameras.calibrate_extrinsics_lidars import REx_calibrate_extrinsics_lidars
from stretch4_body.subsystem.cameras.calibrate_extrinsics_cameras import REx_calibrate_extrinsics_cameras
from stretch4_body.subsystem.cameras.camera_intrinsics_validate_l2_distance import REx_validate_intrinsics

def calibrate_intrinsics_and_extrinsics_not_interactive():
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

    try:
        print("====================================")
        print("Starting Intrinsics Calibration")
        print("====================================")
        REx_calibrate_intrinsics_robot_move(interactive=False)

        print("====================================")
        print("Starting Intrinsics Validation")
        print("====================================")
        time.sleep(3) # wait for the camera device to come back on the USB bus after we closed it at the end of the last step
        errors = REx_validate_intrinsics(interactive=False)
        if any(e > 0.1 or e is None or e == float('inf') for e in errors):
            raise Exception(f"Intrinsic calibration failed! Distance errors ({errors}) are above 0.1m. (inf = no detection)")

        print("====================================")
        print("Starting Extrinsics Camera-Camera Calibration")
        print("====================================")
        REx_calibrate_extrinsics_cameras(interactive=False)
        
        print("====================================")
        print("Starting Extrinsics Camera-Lidar Calibration")
        print("====================================")
        REx_calibrate_extrinsics_lidars(interactive=False)
        
        print("====================================")
        print("Finished Intrinsics and Extrinsics Calibration")
        print("====================================")

        exit(0)
    except KeyboardInterrupt:
        print("\nCalibration sequence aborted by user.")
        raise

if __name__ == "__main__":
    calibrate_intrinsics_and_extrinsics_not_interactive()

