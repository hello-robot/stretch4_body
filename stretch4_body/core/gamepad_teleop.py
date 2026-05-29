#!/usr/bin/env python3

from pathlib import Path

from stretch4_body.core.gamepad_control_mappings import ControlMapping
import stretch4_body.core.gamepad_controller as gc
from stretch4_body.core.device import Device
from stretch4_body.core.gamepad_enums import *
from stretch4_body.core.hello_utils import *
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.core.feetech.feetech_SM_hello import FeetechCommError
from stretch4_body.core import gamepad_joints
import os
import time
import threading
import sys
import click
import subprocess
import threading

from stretch4_body.utils.file_access_utils import acquire_lock_if_available, setup_shared_directory

# Header constants
STEP_SLEEP = 1/15
CALIBRATION_MSG_INTERVAL = 100
TRIGGER_THRESHOLD = 0.9

# Button Hold Durations
TOP_BUTTON_HOLD_TIME_S = 2
START_BUTTON_HOLD_TIME_S = 3
SELECT_BUTTON_SHUTDOWN_TIME_S = 10
SELECT_BUTTON_MAPPING_TIME_S = 2
FN_BUTTON_DETECT_SPAN_S = 0.5

# Sound Delays
BEEP_DELAY_S = 0.5
SOUND_DELAY_MEDIUM_S = 1.0
SOUND_DELAY_LONG_S = 1.7

"""
The GamePadTeleop runs the Stretch's main gamepad controller that ships with 
the robot. The GamePadController is used to listen to the gamepad's inputs 
(button presses,analog stick, trigger) and convert them into robot motions
using the gamepad_joints library's motion command classes.

The gamepad controller key mappings can be customized by modifying `gamepad_control_mappings.py` to add or edit mappings. 

Additionally this class provides other robot function through the gamepad to be 
customized such as manage_shutdown(), manage_fn_button() and setting precision_mode.x`
"""



class GamePadTeleop(Device):
    def __init__(self, robot = None, print_dongle_status = True, lock=None, use_server=False,cb_loop=None):
        """
        Main controller for Stretch's gamepad that ships with the robot.

        Args:
            robot (robot.Robot, optional): A robot instance. If None, one will be created.
            print_dongle_status (bool, optional): Print Dongle status when not plugged into.
            lock (_thread.lock, optional): Pass on lock object to be used while calling robot instance methods.
            use_server (bool, optional): If True, use RobotClient instead of Robot.
            cb_loop (function, optional): A callback function to be called in the main loop.
        """
        Device.__init__(self, 'stretch_gamepad')

        self.motion_profile = MotionProfile.MEDIUM
        self.gripper_handedness = GripperHandedness.RIGHT
        self.control_mapping = ControlMapping.MANIPULATION
        self.contact_sensitivity_profile = GuardedContactSensitivity.HIGH_SENSITIVITY_MANIPULATION

        self.gamepad_controller = gc.GamePadController(print_dongle_status=print_dongle_status)
        self.precision_mode = 0.0
        self.use_arm_lift_mode = False
        self.robot = robot
        self.use_server=use_server
        self.cb_loop=cb_loop
        if self.robot is None:
            if use_server:
                import stretch4_body.robot.robot_client as rc
                self.robot = rc.RobotClient(client_id='gamepad_teleop')
            else:
                import stretch4_body.robot.robot as rb
                self.robot = rb.Robot()
        self.controller_state = self.gamepad_controller.get_state()

        self.end_of_arm_tool =RobotParams().get_params()[1]['robot']['tool']

        self.sleep = STEP_SLEEP
        self.print_mode = False
        self._i = 0
        
        self.fn_button_command = self.params['function_cmd'] # command to execute on pressing X(left button) for N seconds
        self.fn_button_detect_span = self.params['press_time_span'] #s
        
        self._last_fn_btn_press = None
        self.start_button_counter = gc.ButtonPressCounter("start_button_pressed")
        self.top_button_counter = gc.ButtonPressCounter("top_button_pressed")
        self.select_button_counter = gc.ButtonPressCounter("select_button_pressed")
        self.is_gamepad_active = False
        self.gripper = None

        self.gripper_name = 'parallel_gripper' if self.end_of_arm_tool == 'eoa_wrist_dw4_tool_pg4' else 'stretch_gripper'
        self.use_devices={'arm':'arm' in self.robot.subsystems,
                          'eoa':'end_of_arm' in self.robot.subsystems,
                          'lift':'lift' in self.robot.subsystems,
                          'base':'omnibase' in self.robot.subsystems,
                          'gripper':'end_of_arm' in self.robot.subsystems and self.end_of_arm_tool in ['eoa_wrist_dw4_tool_sg4', 'eoa_wrist_dw4_tool_pg4'] }


        self.set_joint_command()
        
        self.effort_trackers = {
            'lift': gc.JointEffortTracker('lift', pos_thresholds=[34.0, 45.0], neg_thresholds= [25.0, 35.0]),
            'arm': gc.JointEffortTracker('arm', pos_thresholds=[10.0, 20.0], neg_thresholds=[10.0, 20.0]),
            'wrist_yaw_joint': gc.JointEffortTracker('eoa', pos_thresholds=[3.0, 10.0], neg_thresholds=[3.0, 10.0], joint_name='wrist_yaw'),
            'wrist_pitch_joint': gc.JointEffortTracker('eoa', pos_thresholds=[3.0, 10.0], neg_thresholds=[3.0, 10.0], joint_name='wrist_pitch'),
            'wrist_roll_joint': gc.JointEffortTracker('eoa', pos_thresholds=[3.0, 10.0], neg_thresholds=[3.0, 10.0], joint_name='wrist_roll'),
            self.gripper_name: gc.JointEffortTracker('eoa', pos_thresholds=[5.0, 20.0], neg_thresholds=[5.0, 20.0], joint_name=self.gripper_name),
        }

            
        print(f"Key mapped to End-Of-Arm Tool: {self.end_of_arm_tool}")
        self.lock = lock or threading.Lock()
        
        self.skip_x_button = False
        self.left_stick_button_fn = None
        self.right_stick_button_fn = None
        self.currently_stowing = False

        self.contact_sensitivity_profile.apply(self.robot)

    def set_joint_command(self):
        self.base_command = gamepad_joints.CommandBase(motion_profile=self.motion_profile.get_name(), motion_profile_angular=self.motion_profile.get_one_lower_speed().get_name())
        self.lift_command = gamepad_joints.CommandLift(motion_profile=self.motion_profile.get_name() )
        if self.use_devices['arm']:
            self.arm_command = gamepad_joints.CommandArm(motion_profile=self.motion_profile.get_name() )
        if self.use_devices['eoa']:
            self.wrist_yaw_command = gamepad_joints.CommandWristYaw(motion_profile=self.motion_profile.get_name() )
            self.wrist_pitch_command = gamepad_joints.CommandWristPitch(motion_profile=self.motion_profile.get_name() )
            self.wrist_roll_command = gamepad_joints.CommandWristRoll(motion_profile=self.motion_profile.get_name() )
        if self.use_devices['gripper']:
            if self.gripper_name == 'parallel_gripper':
                self.gripper = gamepad_joints.CommandParallelGripperPosition(motion_profile=self.motion_profile.get_name() )
            else:
                self.gripper = gamepad_joints.CommandStretchGripperPosition(motion_profile=self.motion_profile.get_name() )


    def cycle_motion_profile(self):
        self.motion_profile = self.motion_profile.cycle(is_forward=True)

        print(f'Switched to {self.motion_profile.name} motion_profile.')
        
        self.motion_profile.play_sound_file()
        duration = 150 * self.motion_profile.value
        self.gamepad_controller.vibrate(duration_ms=duration, strong_magnitude=1.0, weak_magnitude=1.0)
        self.set_joint_command()

    def cycle_mapping(self):
        self.control_mapping = self.control_mapping.cycle(is_forward=True)

        print(f'Switched to {self.control_mapping.name} gamepad mapping.')
        
        self.control_mapping.play_sound_file()
        duration = 150 * self.control_mapping.value
        self.gamepad_controller.vibrate(duration_ms=duration, strong_magnitude=1.0, weak_magnitude=1.0)

    def cycle_contact_sensitivity_profile(self):

        self.contact_sensitivity_profile = self.contact_sensitivity_profile.cycle(is_forward=True)

        print(f'Switched to {self.contact_sensitivity_profile.name} contact_sensitivity_profile.')
        
        self.contact_sensitivity_profile.play_sound_file()
        duration = 150 * self.contact_sensitivity_profile.value
        self.gamepad_controller.vibrate(duration_ms=duration, strong_magnitude=1.0, weak_magnitude=1.0)
        self.contact_sensitivity_profile.apply(self.robot)

    def _handle_vibration(self, actuated_joints):
        """
        Handle vibration feedback for the gamepad controller.
        
        Parameters
        ----------
        actuated_joints : Dict[str, JointState]
            Dictionary of actuated joints and their commands.
        """
        for joint_id, tracker in self.effort_trackers.items():
            is_actuated = joint_id in actuated_joints
            tracker.step(self.robot, is_actuated, actuated_joints.get(joint_id, 0))

            if not is_actuated: continue
            
            def trigger_vibrate(effort, j_id=joint_id, t=tracker):
                strong_mag = 1.0
                weak_mag = 1.0
                try:
                    thresholds = t.pos_thresholds if t.last_direction >= 0 else t.neg_thresholds
                    min_e, max_e = thresholds
                    abs_effort = abs(effort)
                    if max_e > min_e:
                        fraction = min(1.0, max(0.0, (abs_effort - min_e) / (max_e - min_e)))
                        strong_mag = 0.2 + 0.8 * fraction
                        weak_mag = strong_mag
                except Exception:
                    pass
                
                self.gamepad_controller.vibrate_sequence(
                    sequence_ms=[100, 50, 100], 
                    strong_magnitude=strong_mag, 
                    weak_magnitude=weak_mag, 
                    tag=f"effort_{j_id}", 
                    cooldown=0.1
                )
            tracker.trigger_on_hold(0.25, trigger_vibrate)

    def do_motion(self, state = None, robot = None):
        """
        This method should called in the control loop (mainloop())
    
        Parameters
        ----------
        state : Dict
            Override the gamepad controller state providing custom state, Checkout method GamePadController.get_state()
        robot : robot.Robot 
            Valid robot instance

        Returns
        -------
        Whether the robot was commanded to do some motion
        """
        if not robot:
            robot = self.robot
        self._i = self._i + 1 
        self._update_state(state)
        self._update_modes()
        with self.lock:
            if self.currently_stowing: # No control during stowing
                return False
            if not robot.is_homed():
                qprint('press the start button to calibrate the robot')
                
                # Vibrate if trying to move unhomed
                if self.controller_state:
                    state = self.controller_state
                    is_movement_attempt = (
                        abs(state['left_stick_x']) > 0.1 or
                        abs(state['left_stick_y']) > 0.1 or
                        abs(state['right_stick_x']) > 0.1 or
                        abs(state['right_stick_y']) > 0.1 or
                        state['left_shoulder_button_pressed'] or
                        state['right_shoulder_button_pressed'] or
                        state['bottom_button_pressed'] or
                        state['right_button_pressed'] or
                        state['left_pad_pressed'] or
                        state['right_pad_pressed'] or
                        state['top_pad_pressed'] or
                        state['bottom_pad_pressed']
                    )
                    if is_movement_attempt:
                        self.gamepad_controller.vibrate(duration_ms=400, strong_magnitude=1.0, weak_magnitude=1.0)
                
            if self.controller_state is None: # No control if gamepad not being controlled
                return False

            self.manage_start_button(robot)

            if robot.is_homed():
                if self.robot.power_periph.status['runstop_event']:
                    # If the robot is runstopped, vibrate
                    self.gamepad_controller.vibrate(duration_ms=100, strong_magnitude=1.0, weak_magnitude=1.0)
                    return False
                
                # Regular control
                if self.gamepad_controller.is_gamepad_active or state:
                    self.manage_fn_button(robot, self.controller_state['left_button_pressed'])

                    self.precision_mode = self.controller_state['left_trigger_pulled']
                    self.use_arm_lift_mode = self.controller_state['right_trigger_pulled'] > TRIGGER_THRESHOLD
                    
                    actuated_joints = self.control_mapping.do_motion(robot, self)

                    if actuated_joints:
                        try:
                            collisions = robot.status['safety_layer']['sentry_manager']['sentry_self_collision']['collisions']
                            if collisions:
                                self.gamepad_controller.vibrate_sequence(sequence_ms=[150, 100, 200], strong_magnitude=1.0, weak_magnitude=1.0, tag="collision", cooldown=1.0)
                        except Exception:
                            pass
                    
                        if self.precision_mode:
                            self._handle_vibration(actuated_joints)

                    self.manage_top_button(robot) # Stow the robot on Y/top_button long 2s press
                    self.manage_select_button(robot) # Stows the robot and performs a PC shutdown when the Back/SELECT_BUTTON is long pressed for 10s. Comment to turn off

                    self.manage_left_stick_fn_button(self.controller_state['left_stick_button_pressed'])
                    self.manage_right_stick_fn_button(self.controller_state['right_stick_button_pressed'])
                else:
                    self._safety_stop(robot)
        return True

    def _update_state(self, state = None):
        with self.lock:
            self.controller_state = state if state else self.gamepad_controller.get_state()

    def startup(self, robot = None):
        """Start the gamepad controller thread and robot thread if required.

        Args:
            robot (robot.Robot, optional): Valid robot instance if required.
        """
        if self.robot:
            robot = self.robot
        self.gamepad_controller.startup()
        if self.robot:
            if self.robot.startup():
                pass #self.do_double_beep()
            else:
                print('Exiting...')
                sys.exit(0)
        else:
            self.do_double_beep(robot)

    
    def do_single_beep(self, robot=None):
        if self.robot:
            robot = self.robot
        robot.power_periph.trigger_beep()
        robot.push_command()
          
    def do_double_beep(self, robot = None):
        if self.robot:
            robot = self.robot
        robot.power_periph.trigger_beep()
        robot.push_command()
        time.sleep(0.5)
        robot.power_periph.trigger_beep()
        robot.push_command()
        time.sleep(0.5)

    
    def do_four_beep(self, robot = None):
        if self.robot:
            robot = self.robot
        robot.power_periph.trigger_beep()
        robot.push_command()
        time.sleep(0.5)
        robot.power_periph.trigger_beep()
        robot.push_command()
        time.sleep(0.5)
        robot.power_periph.trigger_beep()
        robot.push_command()
        time.sleep(0.5)
        robot.power_periph.trigger_beep()
        robot.push_command()
        time.sleep(0.5)
                    
    def _update_modes(self):
        if self.use_devices['arm']:
            self.arm_command.precision_mode = self.precision_mode
        self.lift_command.precision_mode = self.precision_mode
        self.base_command.precision_mode = self.precision_mode
        if self.use_devices['gripper']:
            self.gripper.precision_mode = self.precision_mode
        if self.use_devices['eoa']:
            self.wrist_pitch_command.precision_mode = self.precision_mode
            self.wrist_roll_command.precision_mode = self.precision_mode
            self.wrist_yaw_command.precision_mode = self.precision_mode

    def manage_top_button(self, robot):
        """
        Manage the state of the top button (Y button).
        
        If the button is held for more than TOP_BUTTON_HOLD_TIME_S (2s), it cycles the motion profile.
        Otherwise, it plays a sequence of sounds indicating the current state of gripper handedness, 
        motion profile, and contact sensitivity.
        """

        self.top_button_counter.step(self.controller_state)

        self.top_button_counter.trigger_on_hold(TOP_BUTTON_HOLD_TIME_S,self.cycle_motion_profile)

        def on_tap():
            self.gripper_handedness.play_sound_file()
            time.sleep(SOUND_DELAY_MEDIUM_S)
            self.motion_profile.play_sound_file()
            time.sleep(SOUND_DELAY_LONG_S)
            self.contact_sensitivity_profile.play_sound_file()

        self.top_button_counter.trigger_on_tap(on_tap)
            

    def change_gripper_handedness(self, robot, *, do_motion:bool):
        """
        Change the gripper handedness (Left/Right).

        Args:
            robot (robot.Robot): Valid robot instance.
            do_motion (bool): If True, the robot will physically move the gripper to the new orientation.
        """
        if not robot.is_homed():
           return
        if not self.use_devices['eoa']:
            print("No eoa device")
            return

        print('Switching gripper handedness')

        if self.gripper_handedness == GripperHandedness.RIGHT:
            self.gripper_handedness = GripperHandedness.LEFT
        else:
            self.gripper_handedness = GripperHandedness.RIGHT

        self.gripper_handedness.play_sound_file()
        duration = 150 * (self.gripper_handedness.value + 1)
        self.gamepad_controller.vibrate(duration_ms=duration, strong_magnitude=1.0, weak_magnitude=1.0)

        if do_motion:
            self.gripper_handedness.move_to(robot)


    def manage_left_stick_fn_button(self, button_state):
        """
        Trigger custom user function for left stick button press.

        The function is executed if the button is held for FN_BUTTON_DETECT_SPAN_S.

        Args:
            button_state (bool): derived from controller_state['left_stick_button_pressed'].
        """
        if self.left_stick_button_fn == None:
            return

        if button_state:
            if not self._last_left_stick_fn_btn_press:
                self._last_left_stick_fn_btn_press = time.time()

            if time.time() - self._last_left_stick_fn_btn_press >= self.fn_button_detect_span:
                click.secho("Executing Left Stick Custom Function", fg="green", bold=True)
                self.left_stick_button_fn()
                self._last_left_stick_fn_btn_press = None
        else:
            self._last_left_stick_fn_btn_press = None

    def manage_right_stick_fn_button(self, button_state):
        """
        Trigger custom user function for right stick button press.

        The function is executed if the button is held for FN_BUTTON_DETECT_SPAN_S.

        Args:
            button_state (bool): derived from controller_state['right_stick_button_pressed'].
        """
        if self.right_stick_button_fn == None:
            return

        if button_state:
            if not self._last_right_stick_fn_btn_press:
                self._last_right_stick_fn_btn_press = time.time()

            if time.time() - self._last_right_stick_fn_btn_press >= self.fn_button_detect_span:
                click.secho("Executing right Stick Custom Function", fg="green", bold=True)
                self.right_stick_button_fn()
                self._last_right_stick_fn_btn_press = None
        else:
            self._last_right_stick_fn_btn_press = None

    def manage_fn_button(self, robot, button_state):
        """
        Detect function button press (Xbox button / Left button).

        Executes a localized shell command (params['function_cmd']) if the button is held 
        for FN_BUTTON_DETECT_SPAN_S.

        Args:
            robot (robot.Robot): Valid robot instance.
            button_state (bool): derived from controller_state['left_button_pressed'].
        """    
        if self.params['enable_fn_button']: 
            if button_state:
                if not self._last_fn_btn_press:
                    self._last_fn_btn_press = time.time()

                if time.time() - self._last_fn_btn_press >= FN_BUTTON_DETECT_SPAN_S: #self.fn_button_detect_span
                    self._last_fn_btn_press = None
                    click.secho(f"Executing Function command: {self.fn_button_command}", fg="green", bold=True)
                    self.do_four_beep(robot)
                    self._execute_fn_cmd()
            else:
                self._last_fn_btn_press = None
    
    def _execute_fn_cmd(self):
        if self.fn_button_command:
            execute_command_non_blocking(self.fn_button_command)
    
    def _safety_stop(self, robot):
        """
        Stop all robot motions.

        This is called when the gamepad is inactive or no input is detected to ensure
        the robot doesn't drift or continue moving.

        Args:
            robot (robot.Robot): Valid robot instance.
        """
        if self.use_devices['eoa']:
            self.wrist_yaw_command.command_button_to_motion(0, robot)
            self.wrist_pitch_command.command_button_to_motion(0, robot)
            self.wrist_roll_command.command_button_to_motion(0, robot)
        if self.use_devices['arm']:
            self.arm_command.command_stick_to_motion(0, robot)
        if self.use_devices['lift']:
            self.lift_command.command_stick_to_motion(0, robot)
        if self.use_devices['base']:
            self.base_command.command_stick_to_motion(0,0,0,robot)

    def stow_robot(self):
        """
        Stow the robot to a safe position.

        Sets the wrist yaw motion parameters to default values before stowing.

        Args:
            robot (robot.Robot): Valid robot instance.
        """
        if self.robot.is_homed():
            # Reset motion params as fast for xbox
            self.currently_stowing = True
            params = RobotParams().get_params()[1]['wrist_yaw']
            v = params['motion']['default']['vel']
            a = params['motion']['default']['accel']
            # robot.end_of_arm.motors['wrist_yaw'].set_motion_params(v, a)
            self.wrist_yaw_command.max_vel = v
            self.wrist_yaw_command.acc = a
            self.robot.stow()
            self.do_single_beep(self.robot)
            self.currently_stowing = False
    
    def stop(self):
        """
        Stop the gamepad controller and the robot.
        """
        self.robot.stop()
        self.gamepad_controller.stop()

    
    def manage_start_button(self, robot):
        """
        Manage the state of the Start button.

        - If the robot is NOT homed, pressing Start triggers homing.
        - If the robot IS homed, holding Start for START_BUTTON_HOLD_TIME_S (3s) changes gripper handedness with motion.

        Args:
            robot (robot.Robot): Valid robot instance.
        """
        self.start_button_counter.step(self.controller_state)
            
        if not robot.is_homed():
            def do_home():
                self.do_single_beep(robot)
                play_sound(get_sounds_dir()+f'/homing.wav')
                robot.home()
            self.start_button_counter.trigger_on_tap(do_home)
            return


        if robot.is_homed() and not self.currently_stowing:
            """If the user holds the start button, it will do the automatic handedness change motion"""
            self.start_button_counter.trigger_on_hold(START_BUTTON_HOLD_TIME_S, lambda:self.change_gripper_handedness(robot, do_motion=True))
            self.start_button_counter.trigger_on_tap( lambda:self.change_gripper_handedness(robot, do_motion=False))
    
    def manage_select_button(self, robot):
        """
        Manage the state of the Select button (Back button).

        - Short press: Cycles contact sensitivity profile.
        - Hold > SELECT_BUTTON_MAPPING_TIME_S (2s): Cycles control mapping (Default/Analog Wrist/Manipulation).
        - Hold > SELECT_BUTTON_SHUTDOWN_TIME_S (10s): Stows the robot and shuts down the PC.

        Args:
            robot (robot.Robot): Valid robot instance.
        """
        self.select_button_counter.step(self.controller_state)

        # def shutdown():
        #     print("Shutting Down the Robot...")
        #     self.do_four_beep(robot)
        #     self._last_select_btn_press = None
        #     robot.power_periph.trigger_beep()
        #     robot.stow()
        #     self.gamepad_controller.stop()
        #     robot.stop()
        #     time.sleep(SOUND_DELAY_MEDIUM_S)
        #     os.system(
        #         'paplay --device=alsa_output.pci-0000_00_1f.3.analog-stereo /usr/share/sounds/ubuntu/stereo/desktop-logout.ogg')
        #     os.system('sudo shutdown now')  # sudoers should be set up to not need a password

        # self.select_button_counter.trigger_on_hold(SELECT_BUTTON_SHUTDOWN_TIME_S, shutdown)

        self.select_button_counter.trigger_on_hold(SELECT_BUTTON_MAPPING_TIME_S, self.cycle_mapping)

        self.select_button_counter.trigger_on_tap(self.cycle_contact_sensitivity_profile)

            
        

    def step_mainloop(self,robot=None):
        """
        Execute a single step of the main control loop.
        
        This method:
        1. Calculates and sends motion commands based on gamepad input.
        2. Pushes commands to the robot.
        3. Sleeps for a short duration (STEP_SLEEP).
        
        Args:
            robot (robot.Robot, optional): Valid robot instance.
        """
        if not robot:
            robot = self.robot
        did_queue_motion = self.do_motion(robot=robot)
        if did_queue_motion:
            if self.use_server:
                robot.push_command(ignore_control_lock=True,priority=1)
            else:
                robot.push_command()
        time.sleep(self.sleep)
        if self.use_server:
            robot.pull_status()

        if self.cb_loop is not None:
            self.cb_loop()

    def mainloop(self):
        """
        Run the main control loop.

        This method runs indefinitely until a KeyboardInterrupt or SystemExit is received.
        It handles signal registration for graceful shutdown.
        """
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        try:
            while True:
                self.step_mainloop()
        except (ThreadServiceExit, KeyboardInterrupt, SystemExit, FeetechCommError):
            self.gamepad_controller.stop()
            self.robot.stop()

def signal_handler(signal_received, frame):
    time.sleep(0.5)
    sys.exit(0)

def execute_command_non_blocking(command):
    try:
        # Use subprocess.Popen to start the command in a separate process that wont get killed
        # when the main self process is killed
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setpgrp  # Detach the child process from the parent
        )
        
        # Optionally, you can save the process ID (PID) for later management if needed
        tmp_file = "/tmp/stretch_gamepad_teleop/gamepad_fn_command_process.pid"

        setup_shared_directory(Path(tmp_file).parent)
        
        if not acquire_lock_if_available(tmp_file, remove_if_exists_and_unused=True):
            raise Exception("Could not acquire lock file for gamepad teleop.")
            
        with open(tmp_file, "w") as pid_file:
            print(f"Process PID ID saved to `/tmp/gamepad_fn_command_process.pid`")
            pid_file.write(str(process.pid))

    except Exception as e:
        print(f"An error occurred: {e}")
        
if __name__ == "__main__":
   gamepad_teleop = GamePadTeleop()
   gamepad_teleop.startup()
   gamepad_teleop.mainloop()





