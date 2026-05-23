from enum import Enum
from typing import TYPE_CHECKING
from stretch4_body.core.gamepad_enums import GripperHandedness, MotionProfile
from stretch4_body.core.hello_utils import *

import pinocchio as pin

if TYPE_CHECKING:
    from stretch4_body.core.gamepad_teleop import GamePadTeleop

class ControlMapping(Enum):
    """
    These mappings are defined as control callbacks in gamepad_teleop.
    """
    OMNIBASE = 1
    """Omnibase controller mapping that mixes both manipulation controls and omnibase movement."""
    MANIPULATION = 2
    """MANIPULATION separates base motion and arm/wrist motion with a manipulation mode (Holding down right trigger)"""
    FLYING_GRIPPER_IK = 3
    """FLYING_GRIPPER_IK provides IK-based Cartesian control of the gripper."""
    EASY_MODE = 4
    """EASY_MODE provides simplified control of the gripper."""
    IMPROVED_MANIPULATION = 5
    """IMPROVED_MANIPULATION mapping with navigation and manipulation modes."""

    def _get_cycleable_options(self):
        # return list(type(self))
        return [ControlMapping.IMPROVED_MANIPULATION,ControlMapping.FLYING_GRIPPER_IK]
        
    def cycle(self, is_forward:bool):
        """
        Cycle through the available control mappings.
        
        Args:
            is_forward (bool): If True, cycle forward. If False, cycle backward.
            
        Returns:
            ControlMapping: The next control mapping.
        """
        index_offset = 1 if is_forward else -1
        members = self._get_cycleable_options()
        index = members.index(self)
        return members[(index + index_offset) % len(members)]

    def play_sound_file(self):
        """
        Play the sound file associated with the current control mapping.
        """
        file_name:str
        if self is ControlMapping.OMNIBASE:
            file_name = "gamepad_teleop_mapping_omnibase.wav"
        elif self is ControlMapping.MANIPULATION:
            file_name = "gamepad_teleop_mapping_manipulation.wav"
        elif self is ControlMapping.FLYING_GRIPPER_IK:
            file_name = "gamepad_teleop_mapping_flying_gripper_ik.wav"
        elif self is ControlMapping.EASY_MODE:
            file_name = "gamepad_teleop_mapping_omnibase.wav"
        elif self is ControlMapping.IMPROVED_MANIPULATION:
            file_name = "gamepad_teleop_mapping_manipulation.wav"
        else:
            raise NotImplementedError(f"No sound file for {self}")
        
        play_sound(get_sounds_dir()+f'/{file_name}')

    def do_motion(self, robot, gamepad_teleop: "GamePadTeleop" ):
        """
        Execute motion commands based on the current mapping.
        
        Args:
            robot (robot.Robot): Valid robot instance.
            gamepad_teleop (GamePadTeleop): The gamepad teleop instance containing controller state and command objects.
        """
        if self == ControlMapping.OMNIBASE:
            return self._map_omnibase(robot, gamepad_teleop)
        elif self == ControlMapping.MANIPULATION:
            return self._map_manipulation(robot, gamepad_teleop)
        elif self == ControlMapping.FLYING_GRIPPER_IK:
            return self._map_flying_gripper_ik(robot, gamepad_teleop)
        elif self == ControlMapping.EASY_MODE:
            return self._map_easy_mode(robot, gamepad_teleop)
        elif self == ControlMapping.IMPROVED_MANIPULATION:
            return self._map_manip_improved(robot, gamepad_teleop)
        else: raise NotImplementedError(f"No controls callback for {self}")

    def _map_omnibase(self, robot, gamepad_teleop: "GamePadTeleop"):
        """
        Default mapping:
        - Wrist Yaw: Bumpers (LB/RB)
        - Wrist Pitch: D-Pad Up/Down
        - Wrist Roll: D-Pad Left/Right
        - Arm/Lift/Base: Sticks
        - Gripper: A/B (Bottom/Right) buttons
        """

        # Set control modes flags
        dxl_zero_vel_set_division_factor = 3 
        # Note: Coninuously commanding stop_motion()(set zero velocities) to chained Dxls above 15 Hz might cause thread blocking issues 
        # while used in multithreaded executors (E.g. ROS2). So using a division factor to downscale the stop_motion() call rate.

        actuated_joints = {}
        if gamepad_teleop.use_devices['eoa']:
            # Wrist Yaw Control
            if gamepad_teleop.controller_state['right_shoulder_button_pressed']:
                gamepad_teleop.wrist_yaw_command.command_button_to_motion(-1,robot)
                actuated_joints['wrist_yaw_joint'] = -1

            elif gamepad_teleop.controller_state['left_shoulder_button_pressed']:
                gamepad_teleop.wrist_yaw_command.command_button_to_motion(1,robot)
                actuated_joints['wrist_yaw_joint'] = 1
            else:
                if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                    gamepad_teleop.wrist_yaw_command.stop_motion(robot)
            if gamepad_teleop.controller_state['top_pad_pressed']:
                cmd = 1 if gamepad_teleop.gripper_handedness is GripperHandedness.LEFT else -1
                gamepad_teleop.wrist_pitch_command.command_button_to_motion(cmd,robot)
                actuated_joints['wrist_pitch_joint'] = cmd
            elif gamepad_teleop.controller_state['bottom_pad_pressed']:
                cmd = -1 if gamepad_teleop.gripper_handedness is GripperHandedness.LEFT else 1
                gamepad_teleop.wrist_pitch_command.command_button_to_motion(cmd, robot)
                actuated_joints['wrist_pitch_joint'] = cmd
            else:
                if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                    gamepad_teleop.wrist_pitch_command.stop_motion(robot)

            if gamepad_teleop.controller_state['left_pad_pressed']:
                gamepad_teleop.wrist_roll_command.command_button_to_motion(1,robot)
                actuated_joints['wrist_roll_joint'] = 1
            elif gamepad_teleop.controller_state['right_pad_pressed']:
                gamepad_teleop.wrist_roll_command.command_button_to_motion(-1,robot)
                actuated_joints['wrist_roll_joint'] = -1
            else:
                if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                    gamepad_teleop.wrist_roll_command.stop_motion(robot)


        if gamepad_teleop.use_devices['arm']:
            cmd = gamepad_teleop.controller_state['right_stick_x'] if gamepad_teleop.use_arm_lift_mode else 0
            gamepad_teleop.arm_command.command_stick_to_motion(cmd, robot)
            if abs(cmd) > 0.1:
                actuated_joints['arm'] = cmd
        if gamepad_teleop.use_devices['lift']:
            cmd = gamepad_teleop.controller_state['right_stick_y'] if gamepad_teleop.use_arm_lift_mode else 0
            gamepad_teleop.lift_command.command_stick_to_motion(cmd, robot)
            if abs(cmd) > 0.1:
                actuated_joints['lift'] = cmd
        if gamepad_teleop.use_devices['base']:
            # Base frame | Joystick frame
            #      X ^   |       Y ^
            #        |   |         |
            # Y <----*   | -X <----*
            ## This is why (x,y,t) is mapped to (y,-lx,-rx) here
            cmd_y = gamepad_teleop.controller_state['left_stick_y']
            cmd_x = -gamepad_teleop.controller_state['left_stick_x']
            cmd_t = -gamepad_teleop.controller_state['right_stick_x'] if not gamepad_teleop.use_arm_lift_mode else 0
            gamepad_teleop.base_command.command_stick_to_motion(cmd_y, cmd_x, cmd_t, robot)
            if abs(cmd_y) > 0.1 or abs(cmd_x) > 0.1 or abs(cmd_t) > 0.1:
                actuated_joints['base'] = cmd_x + cmd_y + cmd_t

        if gamepad_teleop.use_devices['gripper']:
            if gamepad_teleop.controller_state['right_button_pressed']:
                gamepad_teleop.gripper.open_gripper(robot)
                actuated_joints[gamepad_teleop.gripper.name] = 1
            elif gamepad_teleop.controller_state['bottom_button_pressed']:
                gamepad_teleop.gripper.close_gripper(robot)
                actuated_joints[gamepad_teleop.gripper.name] = -1
            else:
                gamepad_teleop.gripper.stop_gripper(robot)
                
        return actuated_joints
            
    def _map_manipulation(self, robot, gamepad_teleop: "GamePadTeleop"):
        """
        Analog Wrist mapping:
        - When Trigger pulled (Manipulation Mode):
            - Right Stick controls Wrist Yaw and Pitch
            - D-Pad Left/Right controls Wrist Roll
        - Otherwise standard arm/lift/base control.
        """
        # Set control modes flags
        gamepad_teleop.precision_mode = gamepad_teleop.controller_state['left_trigger_pulled'] > 0.9
        gamepad_teleop.use_arm_lift_mode = gamepad_teleop.controller_state['right_trigger_pulled'] > 0.9

        dxl_zero_vel_set_division_factor = 3 

        right_stick_x = gamepad_teleop.controller_state['right_stick_x']
        right_stick_y = gamepad_teleop.controller_state['right_stick_y']

        actuated_joints = {}
        if gamepad_teleop.use_devices['lift']:
            if gamepad_teleop.controller_state['top_pad_pressed']:
                gamepad_teleop.lift_command.command_button_to_motion(0.5,robot)
                actuated_joints['lift'] = 0.5
            elif gamepad_teleop.controller_state['bottom_pad_pressed']:
                gamepad_teleop.lift_command.command_button_to_motion(-0.5,robot)
                actuated_joints['lift'] = -0.5
            else:
                if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                    gamepad_teleop.lift_command.stop_motion(robot)


        if gamepad_teleop.use_devices['eoa'] and gamepad_teleop.use_arm_lift_mode:
            gamepad_teleop.base_command.stop_motion(robot)

            # Wrist Yaw Control
            if abs(right_stick_x) > 0.1:
                gamepad_teleop.wrist_yaw_command.command_stick_to_motion(-right_stick_x, robot)
                actuated_joints['wrist_yaw_joint'] = -right_stick_x

            # Wrist Pitch Control
            if abs(right_stick_y) > 0.1:
                handedness_inversion = -1 if gamepad_teleop.gripper_handedness is GripperHandedness.RIGHT else 1
                cmd = handedness_inversion * right_stick_y
                gamepad_teleop.wrist_pitch_command.command_stick_to_motion(cmd, robot)
                actuated_joints['wrist_pitch_joint'] = right_stick_y

            # Wrist Roll Control
            if gamepad_teleop.controller_state['left_pad_pressed']:
                gamepad_teleop.wrist_roll_command.command_button_to_motion(-1,robot)
                actuated_joints['wrist_roll_joint'] = -1
            elif gamepad_teleop.controller_state['right_pad_pressed']:
                gamepad_teleop.wrist_roll_command.command_button_to_motion(1,robot)
                actuated_joints['wrist_roll_joint'] = 1
            else:
                if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                    gamepad_teleop.wrist_roll_command.stop_motion(robot)


            if gamepad_teleop.use_devices['arm']:
                cmd = gamepad_teleop.controller_state['left_stick_y'] if gamepad_teleop.use_arm_lift_mode else 0
                gamepad_teleop.arm_command.command_stick_to_motion(cmd, robot)
                if abs(cmd) > 0.1:
                    actuated_joints['arm'] = cmd


        else:
            if gamepad_teleop.use_devices['arm']:
                # Stop motion for the arm immediately if the manip button is released. This was added intentionally at some point, unsure if it's stil needed.
                gamepad_teleop.arm_command.stop_motion(robot)
            if gamepad_teleop.use_devices['eoa']:
                # Stop motion for the wrist immediately if the manip button is released. This was added intentionally at some point, unsure if it's stil needed.
                gamepad_teleop.wrist_yaw_command.stop_motion(robot)
                gamepad_teleop.wrist_pitch_command.stop_motion(robot)
                gamepad_teleop.wrist_roll_command.stop_motion(robot)

            if gamepad_teleop.use_devices['base']:
                # Base frame | Joystick frame
                #      X ^   |       Y ^
                #        |   |         |
                # Y <----*   | -X <----*
                ## This is why (x,y,t) is mapped to (y,-lx,-rx) here
                cmd_y = gamepad_teleop.controller_state['left_stick_y'] if not gamepad_teleop.use_arm_lift_mode else 0
                cmd_x = -gamepad_teleop.controller_state['left_stick_x'] if not gamepad_teleop.use_arm_lift_mode else 0
                cmd_t = -gamepad_teleop.controller_state['right_stick_x'] if not gamepad_teleop.use_arm_lift_mode else 0
                gamepad_teleop.base_command.command_stick_to_motion(cmd_y, cmd_x, cmd_t, robot)
                if abs(cmd_y) > 0.1 or abs(cmd_x) > 0.1 or abs(cmd_t) > 0.1:
                    actuated_joints['base'] = cmd_x + cmd_y + cmd_t


        if gamepad_teleop.use_devices['gripper']:
            if gamepad_teleop.controller_state['right_button_pressed']:
                gamepad_teleop.gripper.open_gripper(robot)
                actuated_joints[gamepad_teleop.gripper.name] = 1
            elif gamepad_teleop.controller_state['bottom_button_pressed']:
                gamepad_teleop.gripper.close_gripper(robot)
                actuated_joints[gamepad_teleop.gripper.name] = -1
            else:
                gamepad_teleop.gripper.stop_gripper(robot)

        return actuated_joints

    def _map_easy_mode(self, robot, gamepad_teleop: "GamePadTeleop"):
        """
        Direct Control Mapping:
        - Left Trigger for precision mode (handled externally)
        - D-pad up/down for lift
        - D-pad left/right for arm
        - Left Stick for Omnibase translation
        - Right Trigger + Left Stick: Move in straight line
        - Right Stick for Pitch and Yaw
        - Right Trigger + D-pad for Pitch and Yaw
        - Shoulder buttons for Omnibase rotate
        - Right Trigger + Shoulder buttons for Roll
        - A and B buttons for open close gripper
        """
        import math
        
        dxl_zero_vel_set_division_factor = 3
        actuated_joints = {}
        
        state = gamepad_teleop.controller_state
        rt_pulled = state.get('right_trigger_pulled', 0.0) > 0.9 # TRIGGER_THRESHOLD

        if gamepad_teleop.use_devices['gripper']:
            if rt_pulled:
                gamepad_teleop.gripper.stop_gripper(robot)
            else:
                if state.get('right_button_pressed'):
                    gamepad_teleop.gripper.open_gripper(robot)
                    actuated_joints[gamepad_teleop.gripper.name] = 1
                elif state.get('bottom_button_pressed'):
                    gamepad_teleop.gripper.close_gripper(robot)
                    actuated_joints[gamepad_teleop.gripper.name] = -1
                else:
                    gamepad_teleop.gripper.stop_gripper(robot)
        
        if gamepad_teleop.use_devices['base']:
            ls_x = state.get('left_stick_x', 0.0)
            ls_y = state.get('left_stick_y', 0.0)
            if rt_pulled:
                if abs(ls_x) > 0.1 or abs(ls_y) > 0.1:
                    if abs(ls_y) > abs(ls_x):
                        cmd_y = math.copysign(1.0, ls_y)
                        cmd_x = 0.0
                    else:
                        cmd_y = 0.0
                        cmd_x = math.copysign(1.0, -ls_x)
                else:
                    cmd_y = 0.0
                    cmd_x = 0.0
            else:
                cmd_y = ls_y if abs(ls_y) > 0.1 else 0.0
                cmd_x = -ls_x if abs(ls_x) > 0.1 else 0.0
            
            cmd_t = 0.0
            if not rt_pulled:
                if state.get('left_shoulder_button_pressed'):
                    cmd_t = 1.0
                elif state.get('right_shoulder_button_pressed'):
                    cmd_t = -1.0

            gamepad_teleop.base_command.command_stick_to_motion(cmd_y, cmd_x, cmd_t, robot)
            if abs(cmd_y) > 0.1 or abs(cmd_x) > 0.1 or abs(cmd_t) > 0.1:
                actuated_joints['base'] = cmd_x + cmd_y + cmd_t
                
        cmd_lift = 0.0
        cmd_arm = 0.0
        dpad_pitch = 0.0
        dpad_yaw = 0.0
        
        if state.get('top_pad_pressed'):
            if rt_pulled:
                pass
            else:
                cmd_lift = 1.0
        elif state.get('bottom_pad_pressed'):
            if rt_pulled:
                pass
            else:
                cmd_lift = -1.0
                
        if state.get('left_pad_pressed'):
            if rt_pulled:
                pass
            else:
                cmd_arm = -1.0
        elif state.get('right_pad_pressed'):
            if rt_pulled:
                pass
            else:
                cmd_arm = 1.0
                
        if gamepad_teleop.use_devices['lift']:
            if cmd_lift != 0:
                gamepad_teleop.lift_command.command_button_to_motion(cmd_lift, robot)
                actuated_joints['lift'] = cmd_lift
            else:
                if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                    gamepad_teleop.lift_command.stop_motion(robot)
                    
        if gamepad_teleop.use_devices['arm']:
            if cmd_arm != 0:
                gamepad_teleop.arm_command.command_button_to_motion(cmd_arm, robot)
                actuated_joints['arm'] = cmd_arm
            else:
                if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                    gamepad_teleop.arm_command.stop_motion(robot)
                    
        if gamepad_teleop.use_devices['eoa']:

            if rt_pulled:
                cmd_roll = 0.0
                if rt_pulled:
                    if state.get('left_shoulder_button_pressed'):
                        cmd_roll = -1.0
                    elif state.get('right_shoulder_button_pressed'):
                        cmd_roll = 1.0
                        
                if cmd_roll != 0:
                    gamepad_teleop.wrist_roll_command.command_button_to_motion(cmd_roll, robot)
                    actuated_joints['wrist_roll_joint'] = cmd_roll
                else:
                    if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                        gamepad_teleop.wrist_roll_command.stop_motion(robot)
            else:
                handedness_inversion = -1 if gamepad_teleop.gripper_handedness is GripperHandedness.RIGHT else 1
                
                rs_y = state.get('right_stick_y', 0.0)
                cmd_pitch = 0.0
                if abs(rs_y) > 0.1:
                    cmd_pitch = rs_y
                elif dpad_pitch != 0:
                    cmd_pitch = dpad_pitch
                    
                if cmd_pitch != 0:
                    gamepad_teleop.wrist_pitch_command.command_stick_to_motion(cmd_pitch * handedness_inversion, robot)
                    actuated_joints['wrist_pitch_joint'] = cmd_pitch
                else:
                    if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                        gamepad_teleop.wrist_pitch_command.stop_motion(robot)

                rs_x = state.get('right_stick_x', 0.0)
                cmd_yaw = 0.0
                if abs(rs_x) > 0.1:
                    cmd_yaw = -rs_x
                elif dpad_yaw != 0:
                    cmd_yaw = dpad_yaw
                    
                if cmd_yaw != 0:
                    gamepad_teleop.wrist_yaw_command.command_stick_to_motion(cmd_yaw, robot)
                    actuated_joints['wrist_yaw_joint'] = cmd_yaw
                else:
                    if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                        gamepad_teleop.wrist_yaw_command.stop_motion(robot)
         
        return actuated_joints

    def _map_manip_improved(self, robot, gamepad_teleop: "GamePadTeleop"):
        """
        Improved Manip Mapping suggested by BM:
        - Left trigger - Precision mode
        - Right trigger - Navigation / End effector control switch
        
        Navigation mode (RT released):
        - Left stick - omnibase translate
        - Right stick - omnibase rotate
        - D-pad - Lift (Up/Down) and Arm (Left/Right)
        
        Manipulation mode (RT pulled):
        - Left stick Y - Arm
        - Left stick X - Lift (deadzone 0.2)
        - D-pad - Lift (Up/Down)
        - Right stick - pitch and yaw
        - Bumpers - roll
        - A-B - gripper open/close
        """
        dxl_zero_vel_set_division_factor = 3
        actuated_joints = {}
        
        state = gamepad_teleop.controller_state
        gamepad_teleop.precision_mode = state.get('left_trigger_pulled', 0.0) > 0.9
        gamepad_teleop.use_arm_lift_mode = state.get('right_trigger_pulled', 0.0) > 0.9
        rt_pulled = gamepad_teleop.use_arm_lift_mode

        # Gripper (Manipulation Mode only)
        if gamepad_teleop.use_devices['gripper']:
            if state.get('right_button_pressed'):
                gamepad_teleop.gripper.open_gripper(robot)
                actuated_joints[gamepad_teleop.gripper.name] = 1
            elif state.get('bottom_button_pressed'):
                gamepad_teleop.gripper.close_gripper(robot)
                actuated_joints[gamepad_teleop.gripper.name] = -1
            else:
                gamepad_teleop.gripper.stop_gripper(robot)
        
        # Omnibase translation & rotation (Navigation Mode)
        if gamepad_teleop.use_devices['base']:
            if not rt_pulled:
                ls_x = state.get('left_stick_x', 0.0)
                ls_y = state.get('left_stick_y', 0.0)
                cmd_y = ls_y if abs(ls_y) > 0.1 else 0.0
                cmd_x = -ls_x if abs(ls_x) > 0.1 else 0.0
                
                rs_x = state.get('right_stick_x', 0.0)
                cmd_t = -rs_x if abs(rs_x) > 0.1 else 0.0
                
                gamepad_teleop.base_command.command_stick_to_motion(cmd_y, cmd_x, cmd_t, robot)
                if abs(cmd_y) > 0.1 or abs(cmd_x) > 0.1 or abs(cmd_t) > 0.1:
                    actuated_joints['base'] = cmd_x + cmd_y + cmd_t
            else:
                gamepad_teleop.base_command.stop_motion(robot)
                
        cmd_lift = 0.0
        cmd_arm = 0.0
        
        # Lift and Arm (D-pad & Sticks)
        if state.get('top_pad_pressed'):
            cmd_lift = 1.0
        elif state.get('bottom_pad_pressed'):
            cmd_lift = -1.0
        if state.get('left_pad_pressed'):
            cmd_arm = -1.0 # retract
        elif state.get('right_pad_pressed'):
            cmd_arm = 1.0 # extend
            
        if rt_pulled:
            # Manipulation mode
            ls_y = state.get('left_stick_y', 0.0)
            if abs(ls_y) > 0.1:
                cmd_lift = ls_y
                
            ls_x = state.get('left_stick_x', 0.0)
            if abs(ls_x) > 0.2:
                cmd_arm = ls_x
                
        if gamepad_teleop.use_devices['lift']:
            if abs(cmd_lift) > 0.1:
                gamepad_teleop.lift_command.command_stick_to_motion(cmd_lift, robot)
                actuated_joints['lift'] = cmd_lift
            else:
                if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                    gamepad_teleop.lift_command.stop_motion(robot)
                    
        if gamepad_teleop.use_devices['arm']:
            if abs(cmd_arm) > 0.1:
                gamepad_teleop.arm_command.command_stick_to_motion(cmd_arm, robot)
                actuated_joints['arm'] = cmd_arm
            else:
                if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                    gamepad_teleop.arm_command.stop_motion(robot)
                    
        # Wrist Pitch, Yaw, Roll (Manipulation Mode)
        if gamepad_teleop.use_devices['eoa']:
            if rt_pulled:
                handedness_inversion = -1 if gamepad_teleop.gripper_handedness is GripperHandedness.RIGHT else 1
                
                # Pitch
                rs_y = state.get('right_stick_y', 0.0)
                cmd_pitch = rs_y if abs(rs_y) > 0.1 else 0.0
                if cmd_pitch != 0:
                    gamepad_teleop.wrist_pitch_command.command_stick_to_motion(cmd_pitch * handedness_inversion, robot)
                    actuated_joints['wrist_pitch_joint'] = cmd_pitch
                else:
                    if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                        gamepad_teleop.wrist_pitch_command.stop_motion(robot)

                # Yaw
                rs_x = state.get('right_stick_x', 0.0)
                cmd_yaw = -rs_x if abs(rs_x) > 0.1 else 0.0
                if cmd_yaw != 0:
                    gamepad_teleop.wrist_yaw_command.command_stick_to_motion(cmd_yaw, robot)
                    actuated_joints['wrist_yaw_joint'] = cmd_yaw
                else:
                    if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                        gamepad_teleop.wrist_yaw_command.stop_motion(robot)

                # Roll
                cmd_roll = 0.0
                if state.get('left_shoulder_button_pressed'):
                    cmd_roll = -1.0
                elif state.get('right_shoulder_button_pressed'):
                    cmd_roll = 1.0
                    
                if cmd_roll != 0:
                    gamepad_teleop.wrist_roll_command.command_button_to_motion(cmd_roll, robot)
                    actuated_joints['wrist_roll_joint'] = cmd_roll
                else:
                    if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                        gamepad_teleop.wrist_roll_command.stop_motion(robot)
            else:
                if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                    gamepad_teleop.wrist_pitch_command.stop_motion(robot)
                    gamepad_teleop.wrist_yaw_command.stop_motion(robot)
                    gamepad_teleop.wrist_roll_command.stop_motion(robot)
                    
        return actuated_joints

    def _map_flying_gripper_ik(self, robot, gamepad_teleop: "GamePadTeleop") -> dict:
        ikin = gamepad_teleop.flying_gripper_controller

        ikin.q[0] = robot.base.status['x']
        ikin.q[1] = robot.base.status['y']
        ikin.q[2] = np.cos(robot.base.status['theta'])
        ikin.q[3] = np.sin(robot.base.status['theta'])
        ikin.q[4] = robot.lift.status['pos']
        ikin.q[5] = robot.arm.status['pos']
        ikin.q[6] = robot.end_of_arm.status['wrist_yaw']['pos']
        ikin.q[7] = robot.end_of_arm.status['wrist_pitch']['pos'] 
        ikin.q[8] = robot.end_of_arm.status['wrist_roll']['pos']
        
        pin.forwardKinematics(ikin.model, ikin.data, ikin.q)
        pin.updateFramePlacements(ikin.model, ikin.data)


        v_desired = np.zeros(3)
        rot_change = np.zeros(3)

        def deadzone(val, thresh=0.15): return val if abs(val) > thresh else 0.0

        v_desired[0] = deadzone(gamepad_teleop.controller_state.get('left_stick_y', 0.0))
        v_desired[1] = deadzone(-gamepad_teleop.controller_state.get('left_stick_x', 0.0))
        rot_change[1] = deadzone(gamepad_teleop.controller_state.get('right_stick_y', 0.0))
        rot_change[0] = deadzone(-gamepad_teleop.controller_state.get('right_stick_x', 0.0))

        if gamepad_teleop.controller_state.get('top_pad_pressed'): v_desired[2] = 1.0
        elif gamepad_teleop.controller_state.get('bottom_pad_pressed'): v_desired[2] = -1.0

        if gamepad_teleop.controller_state.get('left_pad_pressed'): rot_change[2] = -1.0
        elif gamepad_teleop.controller_state.get('right_pad_pressed'): rot_change[2] = 1.0

        control_mode = 1

        dt = gamepad_teleop.sleep

        if gamepad_teleop.motion_profile == MotionProfile.FAST:
            gamepad_speed_trans = 0.25
            gamepad_speed_rot = 1.0
        elif gamepad_teleop.motion_profile == MotionProfile.MEDIUM:
            gamepad_speed_trans = 0.15
            gamepad_speed_rot = 0.5
        elif gamepad_teleop.motion_profile == MotionProfile.SLOW:
            gamepad_speed_trans = 0.05
            gamepad_speed_rot = 0.4
        else:
            raise ValueError(f"Unknown motion profile: {gamepad_teleop.motion_profile}")

        v_desired_vel = v_desired *gamepad_speed_trans * dt
        rot_change_vel = rot_change *gamepad_speed_rot * dt
        v, _ = ikin.compute_ik_step(v_desired_vel, rot_change_vel, control_mode)
        v_vel = v / dt

        actuated_joints = {}

        right_trigger_pulled = gamepad_teleop.controller_state['right_trigger_pulled'] > 0.9

        if not right_trigger_pulled and np.any(v != 0):
            
            gamepad_teleop.base_command._move(v_vel[0], v_vel[1], v_vel[2], robot)
            gamepad_teleop.lift_command._move(v_vel[3], robot)
            gamepad_teleop.arm_command._move(v_vel[4], robot)
            
            # Smoothing move_by control commands using a high lookahead targeting horizon
            lookahead = 5.0
            
            # Yaw
            yaw_cmd_rad = v[5] * lookahead
            gamepad_teleop.wrist_yaw_command._move(np.degrees(yaw_cmd_rad), robot, velocity=abs(v_vel[5]))
            # Pitch
            handedness_inversion = 1 if gamepad_teleop.gripper_handedness is GripperHandedness.LEFT else -1
            pitch_cmd_rad = v[6] * lookahead * handedness_inversion
            gamepad_teleop.wrist_pitch_command._move(np.degrees(pitch_cmd_rad), robot, velocity=abs(v_vel[6]))
            # Roll
            roll_cmd_rad = v[7] * lookahead * handedness_inversion * -1 
            gamepad_teleop.wrist_roll_command._move(np.degrees(roll_cmd_rad), robot, velocity=abs(v_vel[7]))

            if abs(v_vel[0]) > 0 or abs(v_vel[1]) > 0 or abs(v_vel[2]) > 0:
                actuated_joints['base'] = v_vel[0] + v_vel[1] + v_vel[2]
            if abs(v_vel[3]) > 0:
                actuated_joints['lift'] = v_vel[3]
            if abs(v_vel[4]) > 0:
                actuated_joints['arm'] = v_vel[4]
            if abs(yaw_cmd_rad) > 0:
                actuated_joints['joint_wrist_yaw'] = yaw_cmd_rad
            if abs(pitch_cmd_rad) > 0:
                actuated_joints['joint_wrist_pitch'] = pitch_cmd_rad
            if abs(roll_cmd_rad) > 0:
                actuated_joints['joint_wrist_roll'] = roll_cmd_rad
        else:
            dxl_zero_vel_set_division_factor = 3
            if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                gamepad_teleop.wrist_yaw_command.stop_motion(robot)
                gamepad_teleop.wrist_pitch_command.stop_motion(robot)
                gamepad_teleop.wrist_roll_command.stop_motion(robot)
            gamepad_teleop.base_command.stop_motion(robot)
            gamepad_teleop.lift_command.stop_motion(robot)
            gamepad_teleop.arm_command.stop_motion(robot)

        if gamepad_teleop.use_devices['gripper']:
            if right_trigger_pulled:
                gamepad_teleop.gripper.stop_gripper(robot)
            else:
                if gamepad_teleop.controller_state.get('right_button_pressed'):
                    gamepad_teleop.gripper.open_gripper(robot)
                    actuated_joints['stretch_gripper'] = 1
                elif gamepad_teleop.controller_state.get('bottom_button_pressed'):
                    gamepad_teleop.gripper.close_gripper(robot)
                    actuated_joints['stretch_gripper'] = -1
                else:
                    gamepad_teleop.gripper.stop_gripper(robot)

        return actuated_joints