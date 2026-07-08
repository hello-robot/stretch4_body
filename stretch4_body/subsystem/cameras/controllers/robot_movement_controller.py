
from enum import auto
from enum import Enum
from pathlib import Path
import time
import threading

from stretch4_body.core.gamepad_controller import ButtonPressCounter
from stretch4_body.core.gamepad_teleop import GamePadTeleop
from stretch4_body.core.hello_utils import get_fleet_directory
from stretch4_body.core.gamepad_enums import MotionProfile
from stretch4_body.robot.robot_client import RobotClient

from stretch4_body.subsystem.cameras.detectors.detector_frame_settled import (
    DetectFrameSettled,
)
from stretch4_body.subsystem.cameras.enums.log_levels import LogLevels
from stretch4_body.subsystem.cameras.calibrate_intrinsics import CalibrateIntrinsics

import threading
import time

from stretch4_body.tools.stretch_pose_record import KeyframeRecorder

from stretch4_body.tools.stretch_pose_play import KeyframePlayer
from stretch4_body.utils.stretch_pose_models import RobotJoints


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
                                     
You can validate your calibration by doing the following:
1. Exit this script
2. Run `REx_camera_calibrate --validate`


Changing mode to gamepad mode. You may control the robot using the gamepad until you exit this script.

===============================================
"""
        return ""



class RobotMovementController:
    """
    Handles automated robot translation, rotation, and gamepad teleop
    independently of the camera calibration logic.
    """

    def __init__(
        self, move_robot_mode: MoveRobotMode, calibration: CalibrateIntrinsics, stop_event: threading.Event, skip_user_prompt: bool = False
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
        if self._stop_event.is_set():
            return
        print("Stopping Robot Movement Controller")
        self._stop_event.set()
        if self.movement_thread.is_alive() and self.movement_thread != threading.current_thread():
            self.movement_thread.join(timeout=10)
        if self.teleop_thread.is_alive() and self.teleop_thread != threading.current_thread():
            self.teleop_thread.join(timeout=10)
        self.robot.stop()

    def _teleop_loop(self):
        """Replaces the need to step the gamepad inside the camera's image callback."""
        while not self._stop_event.is_set():
            if self.gamepad_teleop is not None:
                self.gamepad_teleop.step_mainloop()
            # Polling delay roughly equivalent to standard camera framerates (e.g., 30fps)
            time.sleep(1 / 30)

    def _wait_camera_to_stabilize(self):
        self.calibration.reset_stability()
        self.calibration.log_message(
            "Waiting for the camera to stabilize.", LogLevels.INFO
        )
        while not self.calibration.has_frame_been_stable() and not self._stop_event.is_set():
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
                poses_file = f"{get_fleet_directory()}poses_{time.time()}.yaml"
                self.keyframe_recorder.save_to_file(poses_file)
                self.calibration.log_message(f"Saved to {poses_file}", LogLevels.INFO)
            try:
                self.calibration.save_calibration()
            except Exception as e:
                self.calibration.log_message(
                    f"Could not save calibration: {e}", LogLevels.ERROR
                )
        
        self.calibration.register_callback_save_calibration(save_calibration)

        import select
        import sys
        def keyboard_poller():
            print("\n" + "="*50)
            print("Keyboard commands enabled:")
            print("  Press 'x' + Enter to capture a frame and pose")
            print("  Press 's' + Enter to save calibration and exit")
            print("  Press 'q' + Enter to quit")
            print("="*50 + "\n")
            while not self._stop_event.is_set():
                try:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if rlist:
                        line = sys.stdin.readline().strip()
                        if not line:
                            continue
                        for char in line:
                            if char.lower() == 'x':
                                request_capture()
                            elif char.lower() == 's':
                                print("Keyboard 's': saving calibration...")
                                save_calibration()
                                self._stop_event.set()
                            elif char.lower() == 'q':
                                print("Keyboard 'q': quitting...")
                                self._stop_event.set()
                except Exception:
                    time.sleep(0.1)

        threading.Thread(target=keyboard_poller, daemon=True).start()

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
            if j not in (RobotJoints.base, RobotJoints.gripper)
        ]
        self.keyframe_player = KeyframePlayer(
            joints_allowed_to_move=joints_allowed_to_move,
            motion_profile=MotionProfile.SLOW,
            robot=self.robot,
        )

        self.keyframe_player.load_from_file(
            Path(__file__).parent.absolute() / "models/calibration_poses_intrinsics.yaml"
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

        import select
        import sys
        def keyboard_poller():
            print("\n" + "="*50)
            print("Keyboard commands enabled:")
            print("  Press 'x' + Enter to unpause/proceed to next pose")
            print("  Press 'q' + Enter to quit")
            print("="*50 + "\n")
            while not self._stop_event.is_set():
                try:
                    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if rlist:
                        line = sys.stdin.readline().strip()
                        if not line:
                            continue
                        for char in line:
                            if char.lower() == 'x':
                                if is_paused.is_set():
                                    trigger_pause(is_paused)
                            elif char.lower() == 'q':
                                print("Keyboard 'q': quitting...")
                                self._stop_event.set()
                except Exception:
                    time.sleep(0.1)

        if not self.skip_user_prompt:
            threading.Thread(target=keyboard_poller, daemon=True).start()

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

            # Ensure the robot has finished its movement and has had a moment to physically settle
            self.robot.wait_command(timeout=60)
            time.sleep(self.delay)

            self._wait_camera_to_stabilize()

            self.calibration.request_capture()

        # double_beep()

        self.calibration.log_message(
            MoveRobotMode.ARM_POSES.post_calibration_instructions, LogLevels.INFO
        )
        self.calibration.save_calibration()

        self.stop()

