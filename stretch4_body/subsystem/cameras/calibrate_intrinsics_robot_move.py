f"""
This script allows the user to control the robot with gamepad teleop while doing camera intrinsic calibration.

This script does not handle any calibration logic; that is done in `calibrate_intrinsics.py`.

This script focuses on robot control and triggering frame capture requests via the CalibrateIntrinsics class.

See `MoveRobotMode` for more information about the available --move_robot_mode flags.

`REx_camera_calibrate -lrc --gamepad` can be used to do manual calibration with gamepad teleop and capture new poses to save into calibration_poses_intrinsics.json.

`REx_camera_calibrate -lrc --replay` can be used to do automatic calibration with pre-recorded arm poses previously saved in calibration_poses_intrinsics.json.
"""
from enum import Enum, auto
from pathlib import Path
import time
import math
import argparse
import threading
from dataclasses import dataclass
from typing import Callable

from stretch4_body.core.gamepad_controller import ButtonPressCounter
from stretch4_body.core.gamepad_teleop import GamePadTeleop
from stretch4_body.core.hello_utils import get_fleet_directory
from stretch4_body.core.gamepad_enums import MotionProfile
from stretch4_body.robot.robot_client import RobotClient

from stretch4_body.subsystem.cameras.calibrate_intrinsics import (
    RECOMMENDED_MINIMUM_NUMBER_OF_IMAGES_TO_CAPTURE,
    RECOMMENDED_REPROJECTION_ERROR,
    CalibrateIntrinsics,
    CalibrateIntrinsicsThreeHeadCameras,
)
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import (
    RGBPipelineController,
)
from stretch4_body.subsystem.cameras.detectors.detector_frame_settled import (
    DetectFrameSettled,
)
from stretch4_body.subsystem.cameras.enums.charuco_dictionary import CharucoBoards
from stretch4_body.subsystem.cameras.enums.log_levels import LogLevels
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.camera_calibration import (
    DEFAULT_IMAGES_SAVE_PATH,
)

import threading
import time
import math

from stretch_animate.keyframes.record_keyframes import KeyframeRecorder

from stretch_animate.keyframes.play_keyframes import KeyframePlayer
from stretch_animate.keyframes.models import RobotJoints



class RobotMovementController:
    """
    Handles automated robot translation, rotation, and gamepad teleop
    independently of the camera calibration logic.
    """

    def __init__(
        self, move_robot_mode: "MoveRobotMode", calibration: CalibrateIntrinsics, stop_event: threading.Event, skip_user_prompt: bool = False
    ) -> None:

        self.move_robot_mode = move_robot_mode
        self.calibration = calibration

        self.robot = RobotClient()
        self.robot.startup()
        self.skip_user_prompt = skip_user_prompt

        self.delay = 0.5

        self._stop_event = stop_event
        self.move_robot_frame_settled_detector = DetectFrameSettled()
        self.is_doing_motion = False

        # Independent threads for movement and gamepad polling
        self.movement_thread = threading.Thread(target=self._movement_loop, daemon=True)
        self.teleop_thread = threading.Thread(target=self._teleop_loop, daemon=True)

        self.gamepad_teleop = None

        self.keyframe_recorder = KeyframeRecorder()

    def start_movement(self):
        """Initializes the gamepad and begins the background movement/teleop threads."""
        self.gamepad_teleop = GamePadTeleop(use_server=True, cb_loop=None)
        self.gamepad_teleop.sleep = 0
        self.gamepad_teleop.startup()

        self.teleop_thread.start()
        self.movement_thread.start()

    def stop(self):
        """Safely shuts down the background loops and robot base."""
        print("Stopping Robot Movement Controller")
        self._stop_event.set()
        self.robot.stop()

    def _teleop_loop(self):
        """Replaces the need to step the gamepad inside the camera's image callback."""
        while not self._stop_event.is_set():
            if self.gamepad_teleop is not None:
                self.gamepad_teleop.step_mainloop()
            # Polling delay roughly equivalent to standard camera framerates (e.g., 30fps)
            time.sleep(1 / 30)

    def _wait_camera_to_stabilize(self):
        self.calibration.log_message(
            "Waiting for the camera to stabilize.", LogLevels.INFO
        )
        while not self.calibration.has_frame_been_stable():
            time.sleep(0.1)

    def _movement_loop(self):
        """Background thread sequence containing the grid mapping logic."""
        self.left_button_counter = ButtonPressCounter("left_button_pressed")

        self._wait_camera_to_stabilize()

        if self.move_robot_mode == MoveRobotMode.GAMEPAD_MODE:
            self._movement_gamepad()
        elif self.move_robot_mode == MoveRobotMode.ARM_POSES:
            self._movement_poses_play()
        else:
            raise NotImplementedError(
                f"{self.move_robot_mode=} is not implemented"
            )

    def _movement_gamepad(self):
        self.calibration.log_message(
            MoveRobotMode.GAMEPAD_MODE.instructions, LogLevels.INFO
        )

        def request_capture():
            if self.keyframe_recorder is not None:
                self.keyframe_recorder.capture_pose()
            self.calibration.request_capture()

        def save_calibration():
            if self.keyframe_recorder is not None:
                poses_file = f"{get_fleet_directory()}poses_{time.time()}.json"
                self.keyframe_recorder.save_to_file(poses_file)
                self.calibration.log_message(f"Saved to {poses_file}", LogLevels.INFO)
            try:
                self.calibration.save_calibration()
            except Exception as e:
                self.calibration.log_message(
                    f"Could not save calibration: {e}", LogLevels.ERROR
                )

        while not self._stop_event.is_set():
            if (
                self.gamepad_teleop is not None
                and self.gamepad_teleop.controller_state is not None
            ):
                self.left_button_counter.step(
                    controller_state=self.gamepad_teleop.controller_state
                )
                self.left_button_counter.trigger_on_tap(callback=request_capture)
                self.left_button_counter.trigger_on_hold(3, callback=save_calibration)
            time.sleep(1 / 15)

    def _movement_poses_play(self):
        self.calibration.log_message(
            MoveRobotMode.ARM_POSES.instructions, LogLevels.INFO
        )

        # As a safety precaution, do not allow the base to move:
        # Also no gripper so it doesn't accidentally let go:
        joints_allowed_to_move = [
            j
            for j in RobotJoints
            if j not in [RobotJoints.base, RobotJoints.stretch_gripper] and j.name != "parallel_gripper"
        ]
        self.keyframe_player = KeyframePlayer(
            joints_allowed_to_move=joints_allowed_to_move,
            motion_profile=MotionProfile.SLOW,
            robot=self.robot,
        )

        self.keyframe_player.load_from_file(
            Path(__file__).parent.absolute() / "models/calibration_poses_intrinsics.json"
        )

        def double_beep():
            for _ in range(2):
                self.robot.power_periph.trigger_beep()
                self.robot.push_command()
                time.sleep(0.5)

        is_paused = threading.Event()
        if not self.skip_user_prompt:
            is_paused.set()  # Pause by default until the user presses 'x' on the gamepad

        def trigger_pause(wait_for_x: threading.Event):
            if is_paused.is_set():
                self.calibration.log_message(
                    f"Unpaused. Automatic movement will start!", LogLevels.INFO
                )
                wait_for_x.clear()
            else:
                self.calibration.log_message(
                    f"Pausing automatic movement.", LogLevels.INFO
                )
                wait_for_x.set()

        # Play through all the keyframe_player poses:
        while not self._stop_event.is_set():
            if (
                self.gamepad_teleop is not None
                and self.gamepad_teleop.controller_state is not None
            ):
                self.left_button_counter.step(
                    controller_state=self.gamepad_teleop.controller_state
                )
                self.left_button_counter.trigger_on_tap(
                    callback=lambda: trigger_pause(is_paused)
                )

            if is_paused.is_set() or self.calibration.is_capture_request_pending():
                time.sleep(1 / 10)
                continue
                
            if self.robot.power_periph.status['runstop_event']:
                self.calibration.log_message(
                    f"Runstop event triggered, pausing automatic movement.", LogLevels.INFO
                )
                is_paused.set()
                continue

            if not self.keyframe_player.play_next(loop=False):
                break

            # if self.keyframe_player.current_pose_index == 6:
            #     break

            self.calibration.log_message(
                f"Moving to pose {self.keyframe_player.current_pose_index}/{len(self.keyframe_player.poses)}",
                LogLevels.INFO,
            )

            time.sleep(self.delay)

            self._wait_camera_to_stabilize()

            self.calibration.request_capture()

        # double_beep()

        self.calibration.log_message(
            MoveRobotMode.ARM_POSES.post_calibration_instructions, LogLevels.INFO
        )
        self.calibration.save_calibration()

        self.stop()


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

    print("Finished moving the robot and calibrating.")
    rgb_pipeline_controller.stop()


class MoveRobotMode(Enum):
    GAMEPAD_MODE = auto()
    """Allows the user to control the robot using the gamepad. Pressing X will capture calibration frame and the robot's pose."""
    ARM_POSES = auto()
    """Replays ./models/calibration_poses_intrinsics.json and captures frames while following those poses."""

    @property
    def description(self):
        if self is MoveRobotMode.ARM_POSES:
            return "automatic arm movement and calibration frame capture"
        if self is MoveRobotMode.GAMEPAD_MODE:
            return "manual gamepad teleop and calibration frame capture"

        raise NotImplementedError(f"No description provided for {self.name}")

    @property
    def instructions(self):
        if self is MoveRobotMode.ARM_POSES:
            return f"""
===============================================
                                     
Starting camera calibration by {self.description} 
The robot arm will move in various poses automatically. 
This process will take around 15 minutes.

Press 'x' on the gamepad to start movement.

Note: Press 'x' again at any time to pause automatic movement.

===============================================                   
"""
        elif self is MoveRobotMode.GAMEPAD_MODE:
            return """
===============================================

Started gamepad calibration mode.

Press 'x' on the gamepad to capture an image for calibration.

Hold 'x' on the gamepad to save the calibration to disk.

Robot poses will also be captured using `stretch_animate` and saved to disk.

===============================================
"""
        return ""

    @property
    def post_calibration_instructions(self):
        if self is MoveRobotMode.ARM_POSES:
            return f"""
===============================================

Calibration is finished!
                                     
You can verify your calibration by doing the following:
1. Exit this script
2. Run `stretch_camera_show --detect_aruco_marker_size 0.027 --left_right_center --opencv` 
3. Use a tape measure to measure the distance from the lens of each camera to 
    the center ArUco marker using the current pose of the board.
    Please be careful not to scratch the camera lenses!

Expected values:
Left Camera: 0.67 +- 9mm
Right Camera: 0.60 +- 9mm
Center Camera: 0.65 +- 3mm


Changing mode to gamepad mode. You may control the robot using the gamepad until you exit this script.

===============================================
"""
        return ""


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
