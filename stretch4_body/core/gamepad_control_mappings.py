
from enum import (
    auto,
    Enum,
)
import math
from typing import (
    Any,
    TYPE_CHECKING,
)

import numpy as np
from scipy.spatial.transform import Rotation as R

from stretch4_body.core.hello_utils import *
from stretch4_kinematics.state import StretchJointPositions

if TYPE_CHECKING:
    from stretch4_body.core.gamepad_teleop import GamePadTeleop

FLYING_GRIPPER_FRAME = "tool_attachment_site_link"
"""Frame controlled by the FLYING_GRIPPER_IK mapping."""


def _compute_ee_err_local(current_pose: Any, anchor_pose: Any) -> np.ndarray:
    """
    Compute the 6D local end-effector error vector in SE(3).

    Calculates the spatial twist error [v, w] in the current local tool frame
    from current and anchor poses without relying on Pinocchio.

    Args:
        current_pose: Current end-effector pose object with rotation (3x3)
            and translation (3,) attributes.
        anchor_pose: Anchor end-effector pose object with rotation (3x3)
            and translation (3,) attributes.

    Returns:
        6-element numpy array containing 3D linear velocity error followed by
        3D angular velocity error in the current local frame.
    """
    r_cur = current_pose.rotation
    p_cur = current_pose.translation
    r_anc = anchor_pose.rotation
    p_anc = anchor_pose.translation

    r_rel = r_cur.T @ r_anc
    p_rel = r_cur.T @ (p_anc - p_cur)

    # Angular error via SO(3) logarithm (rotation vector)
    rot = R.from_matrix(r_rel)
    w = rot.as_rotvec()
    theta = float(np.linalg.norm(w))

    # Linear error via SE(3) left Jacobian inverse: v = V^-1 * p_rel
    w_cross = np.array([
        [0.0, -w[2], w[1]],
        [w[2], 0.0, -w[0]],
        [-w[1], w[0], 0.0],
    ])
    if theta < 1e-6:
        alpha = 1.0 / 12.0 + theta**2 / 720.0
    else:
        alpha = (1.0 - (theta / (2.0 * np.tan(theta / 2.0)))) / (theta**2)

    v_inv = np.eye(3) - 0.5 * w_cross + alpha * (w_cross @ w_cross)
    v = v_inv @ p_rel

    return np.concatenate([v, w])

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
                ("  RT + Right Stick", "Wrist Pitch or Yaw Only"),
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
                ("RT + Left Stick / D-Pad", "Move Base, Gripper Holds Still"),
                ("RT + LB / RB", "Wrist Roll"),
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
        - Right Trigger + Right Stick: Pitch or Yaw only (locked to dominant axis)
        - Right Trigger + D-pad for Pitch and Yaw
        - Shoulder buttons for Omnibase rotate
        - Right Trigger + Shoulder buttons for Roll
        - A and B buttons for open close gripper
        """
        
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
                # With Right Trigger held, the right stick is locked to its dominant axis
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

                inversion = 1 if gamepad_teleop.invert_pitch else -1

                rs_x = state.get('right_stick_x', 0.0)
                rs_y = state.get('right_stick_y', 0.0)
                if rt_pulled:
                    # If Right-Trigger is pulled, lock the right-stick to its dominant axis (like the left-stick),
                    # so pitch or yaw can be moved alone without drifting the other.
                    if abs(rs_y) > abs(rs_x):
                        rs_x = 0.0
                    else:
                        rs_y = 0.0

                cmd_pitch = 0.0
                if abs(rs_y) > 0.1:
                    cmd_pitch = rs_y
                elif dpad_pitch != 0:
                    cmd_pitch = dpad_pitch
                    
                if cmd_pitch != 0:
                    gamepad_teleop.wrist_pitch_command.command_stick_to_motion(cmd_pitch * inversion, robot)
                    actuated_joints['wrist_pitch_joint'] = cmd_pitch
                else:
                    if gamepad_teleop._i % dxl_zero_vel_set_division_factor == 0:
                        gamepad_teleop.wrist_pitch_command.stop_motion(robot)

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

    def _map_flying_gripper_ik(self, robot: Any, gamepad_teleop: "GamePadTeleop") -> dict[str, float]:
        """
        Map gamepad input to Cartesian flying gripper inverse kinematics and base motion.

        Computes velocity commands for the arm, lift, wrist, and mobile base using differential
        inverse kinematics or nullspace control depending on trigger and stick inputs.

        Args:
            robot: Robot hardware client or instance.
            gamepad_teleop: Gamepad teleoperation controller providing current joystick state
                and command objects.

        Returns:
            Dictionary mapping actuated joint names to commanded velocities.
        """
        kinematics = gamepad_teleop.flying_gripper_kinematics

        current_pos = StretchJointPositions(
            base_x=robot.base.status['x'],
            base_y=robot.base.status['y'],
            base_theta=robot.base.status['theta'],
            lift=robot.lift.status['pos'],
            arm=robot.arm.status['pos'],
            wrist_yaw=robot.end_of_arm.status['wrist_yaw']['pos'],
            wrist_pitch=robot.end_of_arm.status['wrist_pitch']['pos'],
            wrist_roll=robot.end_of_arm.status['wrist_roll']['pos'],
        )

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

        # RT + Left Stick / D-Pad drives the base (forward, left) in its own frame while the gripper holds still
        base_input = v_desired[:2].copy()
        if gamepad_teleop.controller_state.get('top_pad_pressed'): base_input[0] += 1.0
        elif gamepad_teleop.controller_state.get('bottom_pad_pressed'): base_input[0] -= 1.0
        if gamepad_teleop.controller_state.get('left_pad_pressed'): base_input[1] += 1.0
        elif gamepad_teleop.controller_state.get('right_pad_pressed'): base_input[1] -= 1.0
        base_input = np.clip(base_input, -1.0, 1.0)
        do_nullspace_motion = right_trigger_pulled and np.any(base_input != 0)
        if not do_nullspace_motion:
            gamepad_teleop.nullspace_anchor = None

        dt = gamepad_teleop.sleep

        # Joint velocity limits of the current motion profile, matching JOINT_SPACE mode
        vel_xy, _, _, _ = gamepad_teleop.base_command._get_motion_params(is_rotating=False)
        _, _, vel_w, _ = gamepad_teleop.base_command._get_motion_params(is_rotating=True)
        v_max = np.array([
            vel_xy,
            vel_xy,
            vel_w,
            gamepad_teleop.lift_command.max_linear_vel,
            gamepad_teleop.arm_command.max_linear_vel,
            gamepad_teleop.wrist_yaw_command.max_vel,
            gamepad_teleop.wrist_pitch_command.max_vel,
            gamepad_teleop.wrist_roll_command.max_vel,
        ])

        def scale_to_limits(v_vel):
            # Scale all joints uniformly so none exceeds its JOINT_SPACE speed while the gripper keeps its commanded direction
            speeds = np.abs(v_vel)
            limits = v_max[:len(v_vel)]
            if np.any(speeds > limits):
                v_vel = v_vel * np.min(limits[speeds > 0] / speeds[speeds > 0])
            return v_vel

        # Translation is solved by the tool-frame IK (which also compensates wrist yaw for base rotation).
        # Request the fastest of lift/arm and let scale_to_limits() slow it to JOINT_SPACE speeds.
        target_vel = np.zeros(6)
        if not right_trigger_pulled:
            target_vel[:3] = v_desired * max(v_max[3], v_max[4])
        v_vel = scale_to_limits(kinematics.differential_ik(current_pos, FLYING_GRIPPER_FRAME, target_vel).to_numpy()[:6])
        v = v_vel * dt

        actuated_joints = {}


        # Smoothing move_by control commands using a high lookahead targeting horizon
        lookahead = 5.0
        inversion = 1 if gamepad_teleop.invert_pitch else -1

        if do_nullspace_motion:
            # Hold the gripper at the pose it had when the motion started, correcting drift with proportional feedback.
            # The base moves relative to its heading at the start, since the base may also rotate to hold the gripper.
            if gamepad_teleop.nullspace_anchor is None:
                gamepad_teleop.nullspace_anchor = (kinematics.forward(current_pos, FLYING_GRIPPER_FRAME).copy(), current_pos.base_theta)
            anchor_pose, anchor_theta = gamepad_teleop.nullspace_anchor
            current_pose = kinematics.forward(current_pos, FLYING_GRIPPER_FRAME)
            ee_err_local = _compute_ee_err_local(current_pose, anchor_pose)
            kp_ee = 2.0

            base_to_world = np.array([[np.cos(anchor_theta), -np.sin(anchor_theta)], [np.sin(anchor_theta), np.cos(anchor_theta)]])
            v_ns = kinematics.nullspace_base_control(
                q=current_pos,
                v_ee_local_desired=kp_ee * ee_err_local,
                v_base_world_desired=base_to_world @ (base_input * vel_xy),
                target_frame=FLYING_GRIPPER_FRAME,
            ).to_numpy()
            v_ns = scale_to_limits(v_ns)

            # Stop at the lift/arm/wrist position limits; slowing every joint together keeps the gripper still
            q_pin = current_pos.to_pinocchio_q()
            q_joints, v_joints = q_pin[4:], v_ns[3:]
            room = np.where(v_joints > 0, kinematics.model.upperPositionLimit[4:] - q_joints, kinematics.model.lowerPositionLimit[4:] - q_joints)
            moving = v_joints != 0
            if np.any(moving):
                v_ns *= np.clip(np.min(room[moving] / (v_joints[moving] * dt)), 0.0, 1.0)

            _, accel_xy, _, accel_w = gamepad_teleop.base_command._get_motion_params(is_rotating=abs(v_ns[2])>=0.1)
            gamepad_teleop.base_command._move(v_ns[0], v_ns[1], v_ns[2], accel_xy, accel_w, robot)
            gamepad_teleop.lift_command._move(v_ns[3], robot)
            gamepad_teleop.arm_command._move(v_ns[4], robot)
            wrist_commands = [gamepad_teleop.wrist_yaw_command, gamepad_teleop.wrist_pitch_command, gamepad_teleop.wrist_roll_command]
            for wrist_command, wrist_vel in zip(wrist_commands, v_ns[5:]):
                if abs(wrist_vel) > 0:
                    wrist_command._move(np.degrees(wrist_vel * dt * lookahead), robot, velocity=abs(wrist_vel))

            actuated_joints['base'] = v_ns[0] + v_ns[1] + v_ns[2]
            for joint, joint_vel in zip(['lift', 'arm', 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll'], v_ns[3:]):
                if abs(joint_vel) > 0:
                    actuated_joints[joint] = joint_vel
        elif not right_trigger_pulled and (np.any(v != 0) or np.any(rot_change[:2] != 0)):

            _, accel_xy, _, accel_w = gamepad_teleop.base_command._get_motion_params(is_rotating=abs(v_vel[2])>=0.1)
            gamepad_teleop.base_command._move(v_vel[0], v_vel[1], v_vel[2], accel_xy, accel_w, robot)
            gamepad_teleop.lift_command._move(v_vel[3], robot)
            gamepad_teleop.arm_command._move(v_vel[4], robot)

            # Wrist rotation steps the same as JOINT_SPACE mode, on top of the IK's yaw compensation
            # Yaw
            yaw_cmd_rad = v[5] * lookahead + np.radians(gamepad_teleop.wrist_yaw_command.dx_deg * rot_change[0])
            yaw_vel = None if rot_change[0] != 0 else abs(v_vel[5])
            gamepad_teleop.wrist_yaw_command._move(np.degrees(yaw_cmd_rad), robot, velocity=yaw_vel)
            # Pitch
            pitch_cmd_rad = np.radians(gamepad_teleop.wrist_pitch_command.dx_deg * rot_change[1]) * inversion
            if abs(pitch_cmd_rad) > 0:
                gamepad_teleop.wrist_pitch_command._move(np.degrees(pitch_cmd_rad), robot)

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
            roll_cmd_rad = np.radians(gamepad_teleop.wrist_roll_command.dx_deg * rot_change[2]) * inversion * -1
            if abs(roll_cmd_rad)> 0:
                gamepad_teleop.wrist_roll_command._move(np.degrees(roll_cmd_rad), robot)

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
