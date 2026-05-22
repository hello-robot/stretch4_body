
from enum import Enum
from typing import TYPE_CHECKING
from stretch4_body.core.gamepad_enums import GripperHandedness
from stretch4_body.core.hello_utils import *
from stretch4_body.core.gamepad_enums import MotionProfile

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

    def cycle(self, is_forward:bool):
        """
        Cycle through the available control mappings.
        
        Args:
            is_forward (bool): If True, cycle forward. If False, cycle backward.
            
        Returns:
            ControlMapping: The next control mapping.
        """
        index_offset = 1 if is_forward else -1

        members = list(type(self))
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

        if np.any(v != 0):
            
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
            if gamepad_teleop.controller_state['right_button_pressed']:
                gamepad_teleop.gripper.open_gripper(robot)
                actuated_joints['stretch_gripper'] = 1
            elif gamepad_teleop.controller_state['bottom_button_pressed']:
                gamepad_teleop.gripper.close_gripper(robot)
                actuated_joints['stretch_gripper'] = -1
            else:
                gamepad_teleop.gripper.stop_gripper(robot)

        return actuated_joints
