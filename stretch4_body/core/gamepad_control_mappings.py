
from enum import auto
from enum import Enum
from typing import TYPE_CHECKING
from stretch4_body.core.gamepad_enums import GripperHandedness
from stretch4_body.core.hello_utils import *
from stretch4_body.core.gamepad_enums import MotionProfile

import coal # do not remove this unused import; it is needed by pinocchio
import pinocchio as pin

if TYPE_CHECKING:
    from stretch4_body.core.gamepad_teleop import GamePadTeleop

class ControlMapping(Enum):
    """
    These mappings are defined as control callbacks in gamepad_teleop.
    """
    FLYING_GRIPPER_IK = auto()
    """FLYING_GRIPPER_IK provides IK-based Cartesian control of the gripper."""
    JOINT_SPACE = auto()
    """JOINT_SPACE provides control of the robot in joint space."""

    @staticmethod
    def _get_cycleable_options():
        # return list(type(self))
        return [ControlMapping.JOINT_SPACE,ControlMapping.FLYING_GRIPPER_IK]

    def description(self) -> str:
        """
        Returns a helpful explanation of the controls for this mapping.
        """
        def format_table(title, description, rows):
            s = f"\n{Fore.CYAN}{Style.BRIGHT}{'=' * 60}\n"
            s += f" {title.center(58)}\n"
            s += f"{'=' * 60}{Style.RESET_ALL}\n"
            s += f"{Fore.CYAN}{description}\n"
            s += f"{'=' * 60}{Style.RESET_ALL}\n"
            s += f"{Fore.YELLOW}{'Control':<25} | {'Action':<30}{Style.RESET_ALL}\n"
            s += f"{'-' * 25}-+-{'-' * 30}\n"
            for control, action in rows:
                s += f"{control:<25} | {action:<30}\n"
            s += f"{Fore.CYAN}{'=' * 60}{Style.RESET_ALL}\n"
            return s

        if self == ControlMapping.JOINT_SPACE:
            title = "Joint Space Control"
            description = "Control robot joints directly. The controls are summarized in the table below:"
            rows = [
                ("Base Controls", ""),
                ("  Left Stick", "Translate Base"),
                ("  LB / RB", "Rotate Base"),
                ("  Hold LB + RB, Right Stick", "Rotate Base"),
                ("Arm Controls", ""),
                ("  Right Stick", "Wrist Pitch & Yaw"),
                ("  D-Pad Up / Down", "Lift Up / Down"),
                ("  D-Pad Left / Right", "Arm In / Out"),
                ("  A / B Buttons", "Close / Open Gripper"),
                ("  RT + Left Stick", "Straight Line Base Move"),
                ("  RT + LB / RB", "Wrist Roll"),
                ("Modifiers", ""),
                ("  LT", "Reduce Speed"),
                ("  Hold Start", "Change Handedness"),
                ("  RT + A", "Change Speed Profile"),
                ("  RT + B", "Change Strength Profile"),
                ("  Y", "Switch controller mode"),
            ]
            return format_table(title, description, rows)
        
        elif self == ControlMapping.FLYING_GRIPPER_IK:
            title = "Flying Gripper IK Control"
            description = "To move the robot, first point the gripper toward the object you wish to manipulate with the Right Stick\nand then move forward with the Left Stick to go toward it."
            rows = [
                ("Left Stick", "Move toward object"),
                ("Right Stick", "Point to object"),
                ("D-Pad Up / Down", "Lift"),
                ("D-Pad Left / Right", "Wrist Roll"),
                ("A / B Buttons", "Close / Open Gripper"),
                ("Modifiers", ""),
                ("  LT", "Reduce Speed"),
                ("  Hold Start", "Change Handedness"),
                ("  RT + A", "Change Speed Profile"),
                ("  RT + B", "Change Strength Profile"),
                ("  Y", "Switch controller mode"),
            ]
            return format_table(title, description, rows)

        return f"No description available for {self.name}"

        
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
        if self is ControlMapping.FLYING_GRIPPER_IK:
            file_name = "gamepad_teleop_mapping_flying_gripper_ik.wav"
        elif self is ControlMapping.JOINT_SPACE:
            file_name = "gamepad_teleop_mapping_joint_space.wav"
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
        if self == ControlMapping.FLYING_GRIPPER_IK:
            return self._map_flying_gripper_ik(robot, gamepad_teleop)
        elif self == ControlMapping.JOINT_SPACE:
            return self._map_joint_space(robot, gamepad_teleop)
        else: raise NotImplementedError(f"No controls callback for {self}")

    def _map_joint_space(self, robot, gamepad_teleop: "GamePadTeleop"):
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


        left_shoulder_pressed = state.get('left_shoulder_button_pressed')
        right_shoulder_pressed = state.get('right_shoulder_button_pressed')

        right_stick_for_base_rotate = left_shoulder_pressed and right_shoulder_pressed

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
            rs_x = state.get('right_stick_x', 0.0)
            rs_y = state.get('right_stick_y', 0.0)
            if rt_pulled:
                # If Right-Trigger is pulled, make the left-stick move the robot in straight lines.
                # This is great for moving in tight corners! 
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
                # If the left and right shoulder buttons are pressed, use the right stick for base rotation
                if right_stick_for_base_rotate:
                    cmd_t = -rs_x
                elif left_shoulder_pressed:
                    cmd_t = 1.0
                elif right_shoulder_pressed:
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
            def _control_eoa():
                # Use the right stick for controlling pitch and yaw as long as the right and left bumpers and not depressed
                # Use the bumpers for roll as long as Right Trigger is held AND the right and left bumpers are not depressed 
                if right_stick_for_base_rotate:
                    return

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
            _control_eoa()
         
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

        right_trigger_pulled = gamepad_teleop.controller_state['right_trigger_pulled'] > 0.9

        if right_trigger_pulled:
            if gamepad_teleop.controller_state.get('left_shoulder_button_pressed'):
                rot_change[2] = -1.0
            elif gamepad_teleop.controller_state.get('right_shoulder_button_pressed'):
                rot_change[2] = 1.0

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


        # Smoothing move_by control commands using a high lookahead targeting horizon
        lookahead = 5.0
        handedness_inversion = 1 if gamepad_teleop.gripper_handedness is GripperHandedness.LEFT else -1
        
        if not right_trigger_pulled and np.any(v != 0):
            
            vel_xy, accel_xy, vel_w, accel_w = gamepad_teleop.base_command._get_motion_params(is_rotating=abs(v_vel[2])>=0.1)
            gamepad_teleop.base_command._move(v_vel[0], v_vel[1], v_vel[2], accel_xy, accel_w, robot)
            gamepad_teleop.lift_command._move(v_vel[3], robot)
            gamepad_teleop.arm_command._move(v_vel[4], robot)
            
            
            # Yaw
            yaw_cmd_rad = v[5] * lookahead
            gamepad_teleop.wrist_yaw_command._move(np.degrees(yaw_cmd_rad), robot, velocity=abs(v_vel[5]))
            # Pitch
            pitch_cmd_rad = v[6] * lookahead * handedness_inversion
            gamepad_teleop.wrist_pitch_command._move(np.degrees(pitch_cmd_rad), robot, velocity=abs(v_vel[6]))

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
        else:
            roll_cmd_rad = v[7] * lookahead * handedness_inversion * -1
            if abs(roll_cmd_rad)> 0:
                gamepad_teleop.wrist_roll_command._move(np.degrees(roll_cmd_rad), robot, velocity=abs(v_vel[7]))

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
                    actuated_joints[gamepad_teleop.gripper.name] = 1
                elif gamepad_teleop.controller_state.get('bottom_button_pressed'):
                    gamepad_teleop.gripper.close_gripper(robot)
                    actuated_joints[gamepad_teleop.gripper.name] = -1
                else:
                    gamepad_teleop.gripper.stop_gripper(robot)

        return actuated_joints
