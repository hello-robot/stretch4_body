from stretch4_body.subsystem.cameras.controllers.robot_movement_controller import MoveRobotMode
from stretch4_body.subsystem.cameras.controllers.robot_movement_controller import RobotMovementController
f"""
This script allows the user to control the robot with gamepad teleop while doing camera intrinsic calibration.

This script does not handle any calibration logic; that is done in `calibrate_intrinsics.py`.

This script focuses on robot control and triggering frame capture requests via the CalibrateIntrinsics class.

See `MoveRobotMode` for more information about the available --move_robot_mode flags.

`REx_camera_calibrate -lrc --gamepad` can be used to do manual calibration with gamepad teleop and capture new poses to save into calibration_poses_intrinsics.json.

`REx_camera_calibrate -lrc --replay` can be used to do automatic calibration with pre-recorded arm poses previously saved in calibration_poses_intrinsics.json.
"""
import argparse

from stretch4_body.subsystem.cameras.calibrate_intrinsics import (
    RECOMMENDED_MINIMUM_NUMBER_OF_IMAGES_TO_CAPTURE,
    RECOMMENDED_REPROJECTION_ERROR,
    CalibrateIntrinsics,
    CalibrateIntrinsicsThreeHeadCameras,
)
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import (
    RGBPipelineController,
)
from stretch4_body.subsystem.cameras.enums.charuco_dictionary import CharucoBoards
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.camera_calibration import (
    DEFAULT_IMAGES_SAVE_PATH,
)


def _calibrate_intrinsics_robot_move(
        mode,
        recording_directory,
        is_use_last_recording,
        charuco_board_names,
        camera_type,
        skip_user_prompt,
        not_interactive,
):

    if camera_type.is_synced_camera_type():
        calibration = CalibrateIntrinsicsThreeHeadCameras(
            recording_directory=recording_directory,
            timestamp=None,
            is_use_last_recording=is_use_last_recording,
            charuco_board_names=charuco_board_names,
            use_center_camera=camera_type == RGBCameras.synced_left_right_center(),
            time_between_image_captures=None,
            not_interactive=not_interactive,
        )
    else:
        calibration = CalibrateIntrinsics(
            recording_directory=recording_directory,
            camera_type=camera_type,
            timestamp=None,
            is_use_last_recording=is_use_last_recording,
            charuco_board_names=charuco_board_names,
            time_between_image_captures=None,
            not_interactive=not_interactive,
        )

    rgb_pipeline_controller = RGBPipelineController(
        camera_type=camera_type,
        recording_directory=None,
        show_image_in=None,
        is_rotate=False,
        is_rectify=False,
        is_crop=False,
        ai_models_to_use=[],
        detect_aruco_marker_size=None,
    )

    robot_controller = RobotMovementController(
        move_robot_mode=mode, calibration=calibration, stop_event=rgb_pipeline_controller.stop_event, skip_user_prompt=skip_user_prompt
    )

    if not not_interactive:
        calibration.show_interactive_windows()

    instructions = f"""
===============================================

You are about to perform camera calibration using {mode.description}.

A rerun.io window should open up showing you the live feed from the camera(s).

Please read the following instructions on how to perform a good calibration.

Before you begin, please consider the following:
1. Make sure your lens is focused. You can use the `REx_camera_focus` script to check if the lens is focused. 
    A good focus will allow you to read text up to ~3ft away with the fisheye lenses.
2. Lighting is very important; avoid direct sunlight on the lens or the ChArUco board. 
    Balanced and diffuse ambient lighting yields the best results.
3. Capture at least {RECOMMENDED_MINIMUM_NUMBER_OF_IMAGES_TO_CAPTURE} images 
    and produce a Project Error of less than {RECOMMENDED_REPROJECTION_ERROR} for a good calibration.
4. It might be helpful to do calibration in a room with no movement around the robot. 
    This calibration uses motion detection to decide if the image is stable enough before capturing, to minimize blurry images. 
    Warning: If there is motion in the camera frame, you may notice that the auto-capture is not triggering for a long time.

This program is expecting the following ChArUco board(s):
{[CharucoBoards[charuco_board_name].get_board_config(use_high_MP_corner_refinement=camera_type.is_center()) for charuco_board_name in charuco_board_names]}
* Note: You can change the charuco board(s) by passing comma separated values to the --charuco_board_names flag.
If you use multiple boards, make sure the id's of the ArUco markers do not overlap on any of the boards being used in tandem.


After calibration is finished, you can verify the calibration by using an aruco marker of a known size, 
    and running `stretch_camera_show --detect_aruco_marker_size KNOWN_SIZE --left_right_center --opencv`.

One yaml file will be saved to your recording directory, and copies of it will be saved in the HELLO_FLEET_PATH/HELLO_FLEET_ID directory to 
    be used by ROS2 and other scripts that require camera calibration.

All your images are saved in the recording directory: {recording_directory}.

You can rerun the calibration on those images by using the --not_interactive flag, 
    and passing either the timestamp of the recordings or the --use_last_recording flag. 

===============================================
"""
    calibration.log_instructions(instructions)

    calibration.log_instructions("""
    These exposure settings work best for 450-650 lux ambient lighting.
    This was tested by using the max brightness and the white light setting on the 
    2800-6500K Dimmable Photography Light Panels set 2ft horizontally from the mast on either side of the robot.
    The height of the bottom of each light panel is 5ft from the floor.
    The panels are angled 45 degrees toward the charuco board vertically, about 10 degrees toward the floor.
    """)
    rgb_pipeline_controller.set_calibration_exposure_preset()

    robot_controller.start_movement()

    try:
        if isinstance(calibration, CalibrateIntrinsicsThreeHeadCameras):
            for frame in rgb_pipeline_controller.get_frame_synced(is_run_pipeline=True):
                calibration.process_synced_image_frame(
                    frame, save_image_to_disk=True,
                    use_stable_frames_only=True
                )

        else:
            for frame in rgb_pipeline_controller.get_frame(is_run_pipeline=True):
                calibration.process_image_frame(
                    frame, save_image_to_disk=True,
                    use_stable_frames_only=True
                )
    except KeyboardInterrupt:
        print("\nProcess interrupted by user. Stopping robot.")
        raise
    finally:
        print("Stopping robot and camera pipeline...")
        try:
            robot_controller.stop()
        except Exception as e:
            print(f"Error stopping robot controller: {e}")
        try:
            rgb_pipeline_controller.stop()
        except Exception as e:
            print(f"Error stopping camera pipeline: {e}")


def _parse_args() -> tuple[MoveRobotMode, str, bool, list[str], RGBCameras, bool, bool]:
    parser = argparse.ArgumentParser(
        description="Allows the user to control the robot while doing camera intrinsic calibration."
    )
    parser.add_argument(
        "--gamepad", action="store_true", help="Manual gamepad teleop and calibration frame capture"
    )
    parser.add_argument(
        "--replay", action="store_true", help="Automatic arm movement and calibration frame capture (Default)"
    )

    parser.add_argument(
        "-d",
        "--recording_directory",
        type=str,
        default=DEFAULT_IMAGES_SAVE_PATH,
        help=f"Directory used to record the data. Otherwise {DEFAULT_IMAGES_SAVE_PATH} is used.",
    )

    parser.add_argument(
        "-last",
        "--use_last_recording",
        action="store_true",
        help="Use the last recorded folder timestamp inside the provided recording dir. This will load existing images and 'append' new saves to this folder.",
    )

    parser.add_argument(
        "--charuco_board_names",
        type=str,
        default="BOARD_5x7_37mm_27mm_4x4_start_id_0,BOARD_5x7_37mm_27mm_4x4_start_id_20,BOARD_5x7_37mm_27mm_4x4_start_id_40",
        help=f"Name of the CharucoBoards enum to use for calibration.",
    )

    parser.add_argument(
        "-l", "--left", action="store_true", help="Use the left RGB camera."
    )
    parser.add_argument(
        "-r", "--right", action="store_true", help="Use the right RGB camera."
    )
    parser.add_argument(
        "-c", "--center", action="store_true", help="Use the center RGB."
    )
    parser.add_argument(
        "-lr",
        "--left_right",
        action="store_true",
        help="Use the left and right RGB cameras.",
    )
    parser.add_argument(
        "-lrc",
        "--left_right_center",
        action="store_true",
        help="Use all three RGB cameras.",
    )

    parser.add_argument("--skip_user_prompt", action="store_true", help="Skip user prompt before automatic robot movements")
    parser.add_argument("--not_interactive", action="store_true", help="Do not open rerun visualization windows")

    args, _ = parser.parse_known_args()

    camera_type = None
    if args.left:
        camera_type = RGBCameras.left()
    elif args.right:
        camera_type = RGBCameras.right()
    elif args.center:
        camera_type = RGBCameras.center()
    elif args.left_right:
        camera_type = RGBCameras.synced_left_right()
    elif args.left_right_center:
        camera_type = RGBCameras.synced_left_right_center()
    else:
        print("No camera type specified. Defaulting to synced_left_right_center.")
        camera_type = RGBCameras.synced_left_right_center()

    if args.gamepad:
        move_robot_mode = MoveRobotMode.GAMEPAD_MODE
    else:
        move_robot_mode = MoveRobotMode.ARM_POSES

    return (
        move_robot_mode,
        args.recording_directory,
        args.use_last_recording,
        args.charuco_board_names.split(","),
        camera_type,
        args.skip_user_prompt,
        args.not_interactive,
    )

def REx_calibrate_intrinsics_robot_move(interactive: bool):
    (
        mode,
        recording_directory,
        is_use_last_recording,
        charuco_board_names,
        camera_type,
        skip_user_prompt,
        not_interactive,
    ) = _parse_args()

    _calibrate_intrinsics_robot_move(
        mode=mode,
        recording_directory=recording_directory,
        is_use_last_recording=is_use_last_recording,
        charuco_board_names=charuco_board_names,
        camera_type=camera_type,
        skip_user_prompt=not interactive,
        not_interactive=not interactive,
    )


if __name__ == "__main__":
    params = _parse_args()
    _calibrate_intrinsics_robot_move(*params)
