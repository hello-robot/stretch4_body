#!/usr/bin/env python3
import sys
import time
import argparse
import sys
import numpy as np
from stretch4_body.core.hello_utils import H0_from_driving_dir, inverse_3x3_matrix
import stretch4_body.core.hello_utils as hu
from stretch4_body.robot.robot_client import RobotClient
import serial
import time
import math

class GripperSlider:
    def __init__(self, port='/dev/hello-gripper-pistol', baudrate=115200):
        """
        Initializes the slider reader.
        :param port: The device path (e.g., '/dev/hello-gripper-pistol')
        :param baudrate: Communication speed (115200 is standard for CircuitPython)
        """
        self.port = port
        self.baudrate = baudrate
        self.connection = None

    def connect(self):
        """Establishes the serial connection."""
        try:
            self.connection = serial.Serial(self.port, self.baudrate, timeout=1)
            # Give the board a moment to initialize after the connection opens
            time.sleep(0.1)
            print(f"Successfully connected to {self.port}")
            self.hw_valid=True
            self._buffer = ""
            self._last_val = None
        except serial.SerialException as e:
            print(f"Could not open serial port {self.port}: {e}")
            self.hw_valid=False

    def get_value(self):
        """Reads the latest slider potentiometer value from the Trinkey.
        
        The Trinkey sends alternating lines:
          Touch: <cap_touch_value>
          Slider: <pot_value>
        We want the Slider (potentiometer) value.
        """
        if not self.connection or not self.connection.is_open:
            print("No connection")
            return None

        try:
            if self.connection.in_waiting > 0:
                self._buffer += self.connection.read(self.connection.in_waiting).decode('utf-8', errors='ignore')
                if '\n' in self._buffer:
                    lines = self._buffer.split('\n')
                    self._buffer = lines[-1]
                    for line in lines[:-1]:
                        line = line.strip()
                        if "Slider:" in line:
                            try:
                                self._last_val = float(line.split(":")[-1].strip())
                            except ValueError:
                                pass
        except Exception:
            pass
            
        return getattr(self, '_last_val', None)


    def close(self):
        """Closes the serial connection."""
        if self.connection:
            self.connection.close()
            #print("Serial connection closed.")

# def main():
#     slider = GripperSlider()
#     slider.connect()
#     try:
#         print("Reading slider values (Press Ctrl+C to stop)...")
#         while True:
#             val = slider.get_value()
#             if val is not None:
#                 # Normalize 16-bit value to 0.0 - 1.0 (if your Trinkey sends 0-65535)
#                 # normalized_val = val / 65535.0
#                 print(f"Slider Position: {val}")
            
#             time.sleep(0.1)  # 0.1s interval
#     except KeyboardInterrupt:
#         slider.close()


def main():
    hu.print_stretch_re_use()
    parser = argparse.ArgumentParser(description='Controller-Puppet Teleop over IP')
    parser.add_argument("--puppet_ip", type=str, default=None, help="IP address of the puppet robot running Stretch Body Server (e.g. 192.168.1.10)")
    parser.add_argument("--joints", nargs='+', default=["omnibase", "lift", "arm", "wrist", "gripper"], help="List of joints to mimic. Example: --joints lift arm wrist_yaw. Default: omnibase lift arm gripper wrist_yaw pitch_roll")
    parser.add_argument("--no_puppet", action='store_true', help="Run without a puppet robot. Prints controller joint positions only.")
    parser.add_argument("--no_pistol", action='store_true',help="Run without the pistol installed.")
    parser.add_argument("--pg4", action='store_true',help="Run a PG4 on the puppet side.")
    parser.add_argument("--pg4c", action='store_true',help="Run a PG4 on the controller side.")
    parser.add_argument("--print_only", action='store_true', help="Print controller and puppet joint positions without commanding motion.")
    parser.add_argument("--base_rotate_only", action='store_true', help="Controller base motion will only generate pure rotation commands on the puppet base.")
    parser.add_argument("--tool_nil_controller", action='store_true', help="Run when the controller has eoa_wrist_dw4_tool_nil (no gripper on controller side).")
    args = parser.parse_args()

    if not args.no_puppet and args.puppet_ip is None:
        parser.error("--puppet_ip is required unless --no_puppet is set")

    print(f"Connecting to controller (local)...")
    controller = RobotClient()
    if not controller.startup():
        print("Failed to start controller RobotClient. Is the local robot server running?")
        sys.exit(1)

    if args.pg4c and controller.end_of_arm is not None:
        if 'parallel_gripper' not in controller.end_of_arm.joints:
            controller.end_of_arm.joints.append('parallel_gripper')
        if 'stretch_gripper' in controller.end_of_arm.joints:
            controller.end_of_arm.joints.remove('stretch_gripper')


    # Validate that wrist joints have enable_torque_after_runstop set to 0
    invalid_params = []
    for joint in ['wrist_yaw', 'wrist_pitch', 'wrist_roll']:
        if joint in controller.robot_params:
            if controller.robot_params[joint].get('enable_torque_after_runstop', 1) != 0:
                invalid_params.append(joint)
    
    if invalid_params:
        print(f"WARNING: The following joints do not have 'enable_torque_after_runstop' set to 0: {', '.join(invalid_params)}")
        print("Please update your stretch_user_params.yaml to set 'enable_torque_after_runstop: 0' for these joints.")
        print("This is required to prevent the controller robot from snapping to positions after runstop.")
        controller.stop()
        sys.exit(1)


    puppet = None
    if not args.no_puppet:
        print(f"Connecting to puppet at tcp://{args.puppet_ip}...")
        puppet = RobotClient(ip_address=args.puppet_ip)
        if not puppet.startup():
            print("Failed to start puppet RobotClient. Is the remote robot server running on IP:", args.puppet_ip)
            controller.stop()
            sys.exit(1)
            
        if puppet.end_of_arm is not None:
            if args.pg4:
                if 'parallel_gripper' not in puppet.end_of_arm.joints:
                    puppet.end_of_arm.joints.append('parallel_gripper')
                if 'stretch_gripper' in puppet.end_of_arm.joints:
                    puppet.end_of_arm.joints.remove('stretch_gripper')
            else:
                if 'stretch_gripper' not in puppet.end_of_arm.joints:
                    puppet.end_of_arm.joints.append('stretch_gripper')
                if 'parallel_gripper' in puppet.end_of_arm.joints:
                    puppet.end_of_arm.joints.remove('parallel_gripper')

    # Validate homing
    print("Checking if controller is homed...")
    controller.pull_status(blocking=True)
    if not controller.is_homed():
        print("Controller robot is not fully homed! Please home the controller robot first. Exiting gracefully.")
        controller.stop()
        if puppet:
            puppet.stop()
        sys.exit(1)

    if not args.no_puppet:
        print("Checking if puppet is homed...")
        puppet.pull_status(blocking=True)
        if not puppet.is_homed():
            print("Puppet robot is not fully homed! Please home the puppet robot first. Exiting gracefully.")
            print("Also check that the tool type is correctly configured (eg pg4 vs stretch_gripper)")
            controller.stop()
            puppet.stop()
            sys.exit(1)

    # Setup the requested joints
    joints = args.joints

    def parse_joints(joint_list):
        # Maps the requested generic joint names to the actual subsystem keys
        active = {}
        for j in joint_list:
            if j == "omnibase":
                active['omnibase'] = True
            elif j == "lift":
                active['lift'] = True
            elif j == "arm":
                active['arm'] = True
            elif j == "wrist":
                active['wrist_yaw'] = True
                active['wrist_pitch'] = True
                active['wrist_roll'] = True
            elif j == "wrist_yaw":
                active['wrist_yaw'] = True
            elif j == "wrist_pitch":
                active['wrist_pitch'] = True
            elif j == "wrist_roll":
                active['wrist_roll'] = True
            elif j in ["stretch_gripper", "parallel_gripper", "gripper"]:
                active['gripper'] = True
        return active

    active_joints = parse_joints(joints)
    print('Running puppet teleop on joints', active_joints)
    
    slider = GripperSlider()
    if not args.no_pistol:
        slider.connect()
    else:
        slider.hw_valid = False
    current_slider_val = None
    
    # For tracking omnibase deltas
    prev_theta_controller = None
    prev_x_controller = None
    prev_y_controller = None

    hz =80.0
    rate = 1.0 / hz

    if args.no_puppet:
        print("No-puppet mode: displaying controller joint positions. Press Ctrl-C to exit.")
    else:
        print("Puppet teleop started! Press Ctrl-C to exit.")

    if not args.no_puppet and puppet is not None and not args.print_only:
        print("Synchronizing puppet joints to controller...")
        input('Hit enter to begin...')
        controller.pull_status()

        if 'lift' in active_joints and controller.lift is not None:
            puppet_val = controller.lift.status.get('pos', 0.2)
            puppet.lift.move_to(puppet_val)

        if 'arm' in active_joints and controller.arm is not None:
            puppet_val = controller.arm.status.get('pos', 0.0)
            puppet.arm.move_to(puppet_val)

        if controller.end_of_arm is not None and ('wrist_yaw' in active_joints or 'wrist_pitch' in active_joints or 'gripper' in active_joints):
            if hasattr(controller.end_of_arm, 'joints'):
                for eoa_j in controller.end_of_arm.joints:
                    if eoa_j == 'wrist_yaw' and 'wrist_yaw' not in active_joints: continue
                    if eoa_j == 'wrist_pitch' and 'wrist_pitch' not in active_joints: continue
                    if eoa_j == 'wrist_roll' and 'wrist_roll' not in active_joints: continue
                    
                    try:
                        if eoa_j == 'stretch_gripper':
                            controller_val = controller.status['end_of_arm'][eoa_j].get('pos_pct', controller.status['end_of_arm'][eoa_j]['pos'])
                            params = controller.robot_params.get(eoa_j, {})
                            pct_max_open=100*abs(params['range_deg'][1]/params['range_deg'][0]) if 'range_deg' in params and params['range_deg'][0] != 0 else 100.0
                            normalized_pct = min(max((controller_val + 100.0) / (pct_max_open + 100.0), 0.0), 1.0)
                        elif eoa_j == 'parallel_gripper':
                            controller_val = controller.status['end_of_arm'][eoa_j].get('pos_mm', 0.0)
                            c_range = controller.robot_params.get('parallel_gripper', {}).get('range_mm', 80.0)
                            normalized_pct = min(max(controller_val / c_range, 0.0), 1.0) if c_range != 0 else 0.0
                        else:
                            controller_val = controller.status['end_of_arm'][eoa_j]['pos']
                            
                        if args.pg4 and eoa_j in ['stretch_gripper', 'parallel_gripper']:
                            pg4_range = puppet.robot_params.get('parallel_gripper', {}).get('range_mm', 80.0) # default 80
                            
                            sl_val = slider.get_value() if getattr(slider, 'hw_valid', False) else None
                            if sl_val is not None:
                                slider_min, slider_max = 0.1, 1.0
                                denom = (slider_max - slider_min) if slider_max != slider_min else 1.0
                                mapped_val = min(max((sl_val - slider_min) / denom, 0.0), 1.0)
                                puppet.end_of_arm.move_to_mm('parallel_gripper', mapped_val * pg4_range)
                            else:
                                puppet.end_of_arm.move_to_mm('parallel_gripper', normalized_pct * pg4_range)
                        elif eoa_j in ['stretch_gripper', 'parallel_gripper']:
                            params = puppet.robot_params.get('stretch_gripper', {})
                            pct_max_open=100*abs(params['range_deg'][1]/params['range_deg'][0]) if 'range_deg' in params and params['range_deg'][0] != 0 else 100.0
                            target_pct = normalized_pct * (pct_max_open + 100.0) - 100.0
                            puppet.end_of_arm.move_to('stretch_gripper', target_pct)
                        else:
                            puppet.end_of_arm.move_to(eoa_j, controller_val)
                    except KeyError:
                        pass

        puppet.push_command()
        # print("Waiting 5s for motion to complete...")
        # time.sleep(5.0)

    print("Making controller backdrivable...")
    if 'omnibase' in active_joints and controller.omnibase is not None:
        controller.omnibase.enable_freewheel_mode()
    if 'arm' in active_joints and controller.arm is not None:
        controller.arm.enable_safety()
    if 'lift' in active_joints and controller.lift is not None:
        controller.lift.enable_safety()
    if controller.end_of_arm is not None and hasattr(controller.end_of_arm, 'joints'):
        for eoa_j in controller.end_of_arm.joints:
            # Check if this EOA joint is active before making backdrivable
            is_active = True
            if eoa_j == 'wrist_yaw' and 'wrist_yaw' not in active_joints: is_active = False
            elif eoa_j == 'wrist_pitch' and 'wrist_pitch' not in active_joints: is_active = False
            elif eoa_j == 'wrist_roll' and 'wrist_roll' not in active_joints: is_active = False
            elif eoa_j in ['stretch_gripper', 'parallel_gripper'] and 'gripper' not in active_joints: is_active = False
            
            if is_active:
                if args.pg4c and eoa_j == 'parallel_gripper':
                    pass
                elif args.tool_nil_controller and eoa_j in ['stretch_gripper', 'parallel_gripper']:
                    pass
                else:
                    print('Disabling controller torque on',eoa_j)
                    controller.end_of_arm.disable_torque(eoa_j)
    controller.push_command()

    num_lines_printed = 0
    pg4_cmd_log_mm = None
    last_print_time = 0.0
    
    last_loop_time = time.time()
    loop_rate_hz = 0.0
    alpha = 0.1
    
    try:
        while True:
            t_now = time.time()
            dt_loop = t_now - last_loop_time
            last_loop_time = t_now
            if dt_loop > 0:
                current_hz = 1.0 / dt_loop
                if loop_rate_hz == 0.0:
                    loop_rate_hz = current_hz
                else:
                    loop_rate_hz = (1.0 - alpha) * loop_rate_hz + alpha * current_hz

            t_start = time.time()
            controller.pull_status()
            if getattr(slider, 'hw_valid', False):
                try:
                    val = slider.get_value()
                except Exception:
                    val = None
                if val is not None:
                    slider_min, slider_max = 0.1, 1.0
                    denom = (slider_max - slider_min) if slider_max != slider_min else 1.0
                    current_slider_val = min(max((val - slider_min) / denom, 0.0), 1.0)

            if not args.no_puppet:
                # ---- Normal teleop mode ----
                puppet.pull_status(blocking=False) #Dont wait to avoid race condition on push_command timing

                eoa_vel_r = 12.0
                eoa_accel_r=10.0
                lift_vel_m=controller.lift.params['motion']['max']['vel_m']
                lift_accel_m=controller.lift.params['motion']['max']['accel_m']*0.7
                arm_vel_m=controller.arm.params['motion']['max']['vel_m']
                arm_accel_m=controller.arm.params['motion']['max']['accel_m']

                if not args.print_only:
                    if 'lift' in active_joints and controller.lift is not None:
                        puppet_val = controller.lift.status['pos']
                        puppet.lift.move_to(puppet_val, v_m=lift_vel_m, a_m=lift_accel_m)

                    if 'arm' in active_joints and controller.arm is not None:
                        puppet_val = controller.arm.status['pos']
                        puppet.arm.move_to(puppet_val, v_m=arm_vel_m, a_m=arm_accel_m)

                    if controller.end_of_arm is not None and ('wrist_yaw' in active_joints or 'wrist_pitch' in active_joints or 'gripper' in active_joints):
                        if hasattr(controller.end_of_arm, 'joints'):
                            for eoa_j in controller.end_of_arm.joints:
                                if eoa_j == 'wrist_yaw' and 'wrist_yaw' not in active_joints: continue
                                if eoa_j == 'wrist_pitch' and 'wrist_pitch' not in active_joints: continue
                                if eoa_j == 'wrist_roll' and 'wrist_roll' not in active_joints: continue
                                if eoa_j in ['stretch_gripper', 'parallel_gripper'] and getattr(slider, 'hw_valid', False): continue
                                
                                # Only target supported joints. If end_of_arm is named differently, it'll still pass.
                                try:
                                    if eoa_j == 'stretch_gripper':
                                        controller_val = controller.status['end_of_arm'][eoa_j].get('pos_pct', controller.status['end_of_arm'][eoa_j]['pos'])
                                        params = controller.robot_params.get(eoa_j, {})
                                        pct_max_open=100*abs(params['range_deg'][1]/params['range_deg'][0]) if 'range_deg' in params and params['range_deg'][0] != 0 else 100.0
                                        normalized_pct = min(max((controller_val + 100.0) / (pct_max_open + 100.0), 0.0), 1.0)
                                    elif eoa_j == 'parallel_gripper':
                                        controller_val = controller.status['end_of_arm'][eoa_j].get('pos_mm', 0.0)
                                        c_range = controller.robot_params.get('parallel_gripper', {}).get('range_mm', 80.0)
                                        normalized_pct = min(max(controller_val / c_range, 0.0), 1.0) if c_range != 0 else 0.0
                                    else:
                                        controller_val = controller.status['end_of_arm'][eoa_j]['pos']
                                        
                                    if args.pg4 and eoa_j in ['stretch_gripper', 'parallel_gripper']:
                                        pg4_range = puppet.robot_params.get('parallel_gripper', {}).get('range_mm', 80.0)
                                        
                                        if current_slider_val is not None:
                                            pg4_cmd_log_mm = current_slider_val * pg4_range
                                        else:
                                            pg4_cmd_log_mm = normalized_pct * pg4_range
                                            
                                        puppet.end_of_arm.move_to_mm('parallel_gripper', pg4_cmd_log_mm, v_r=eoa_vel_r, a_r=eoa_accel_r)
                                    elif eoa_j in ['stretch_gripper', 'parallel_gripper']:
                                        params = puppet.robot_params.get('stretch_gripper', {})
                                        pct_max_open=100*abs(params['range_deg'][1]/params['range_deg'][0]) if 'range_deg' in params and params['range_deg'][0] != 0 else 100.0
                                        target_pct = normalized_pct * (pct_max_open + 100.0) - 100.0
                                        puppet.end_of_arm.move_to('stretch_gripper', target_pct, v_r=eoa_vel_r, a_r=eoa_accel_r)
                                    else:
                                        puppet.end_of_arm.move_to(eoa_j, controller_val, v_r=eoa_vel_r, a_r=eoa_accel_r)

                                except KeyError:
                                    pass # Some joints might not report pos

                    if 'omnibase' in active_joints and controller.omnibase is not None:
                        is_runstop = False
                        if hasattr(controller, 'power_periph') and controller.power_periph is not None:
                            is_runstop = is_runstop or controller.power_periph.status.get('runstop_event', False)
                        if not args.no_puppet and hasattr(puppet, 'power_periph') and puppet.power_periph is not None:
                            is_runstop = is_runstop or puppet.power_periph.status.get('runstop_event', False)

                        was_runstop = getattr(controller, '_was_runstop', False)
                        #print('RUNStOP is',is_runstop,'was',was_runstop)
                        if not is_runstop and was_runstop:
                            # System recovered from runstop, loosen EOA joints again for backdrivability
                            if controller.end_of_arm is not None and hasattr(controller.end_of_arm, 'joints'):
                                for eoa_j in controller.end_of_arm.joints:
                                    if args.pg4c and eoa_j == 'parallel_gripper':
                                        continue
                                    if args.tool_nil_controller and eoa_j in ['stretch_gripper', 'parallel_gripper']:
                                        continue
                                    print('Torque diesable',eoa_j)
                                    controller.end_of_arm.disable_torque(eoa_j)
                            controller.push_command()
                        controller._was_runstop = is_runstop

                        if not hasattr(controller, '_wheel_start_pos'):
                            controller._wheel_start_pos = {}
                            if not args.no_puppet and puppet.omnibase is not None:
                                puppet._wheel_start_pos = {}
                                
                            for i in range(3):
                                wn = f'wheel_{i}'
                                controller._wheel_start_pos[wn] = controller.omnibase.status[wn]['pos']
                                if not args.no_puppet and puppet.omnibase is not None:
                                    puppet._wheel_start_pos[wn] = puppet.omnibase.status[wn]['pos']

                        if is_runstop:
                            # Re-anchor the tracking base to discard any delta that occurs while run-stopped
                            for i in range(3):
                                wn = f'wheel_{i}'
                                controller._wheel_start_pos[wn] = controller.omnibase.status[wn]['pos']
                                if not args.no_puppet and puppet.omnibase is not None:
                                    puppet._wheel_start_pos[wn] = puppet.omnibase.status[wn]['pos']
                        else:
                            if args.base_rotate_only:
                                # Mathematically filter pure rotation from controller wheel encoders
                                dw = np.array([controller.omnibase.status[f'wheel_{i}']['pos'] - controller._wheel_start_pos[f'wheel_{i}'] for i in range(3)])
                                m_rp = controller.robot_params['omnibase']
                                controller_H0 = H0_from_driving_dir(m_rp['wheel_diameter_m'], m_rp['base_radius_m'], m_rp.get('forward_dir', 'calder'))
                                controller_H0_inv = inverse_3x3_matrix(controller_H0)
                                dx, dy, dtheta = controller_H0_inv @ (dw / m_rp['gr'])
                                
                                p_rp = puppet.robot_params['omnibase'] if (not args.no_puppet and puppet is not None) else m_rp
                                puppet_H0 = H0_from_driving_dir(p_rp['wheel_diameter_m'], p_rp['base_radius_m'], p_rp.get('forward_dir', 'calder'))
                                u_w = puppet_H0 @ np.array([0, 0, dtheta])
                                u = u_w * p_rp['gr']
                                
                                for i in range(3):
                                    wn = f'wheel_{i}'
                                    if not args.no_puppet and puppet.omnibase is not None:
                                        p_des = puppet._wheel_start_pos[wn] + u[i]
                                        puppet.omnibase.wheel_move_to(wn, p_des)
                            else:
                                for i in range(3):
                                    wn = f'wheel_{i}'
                                    controller_cur = controller.omnibase.status[wn]['pos']
                                    if not args.no_puppet and puppet.omnibase is not None:
                                        p_des = puppet._wheel_start_pos[wn] + (controller_cur - controller._wheel_start_pos[wn])
                                        puppet.omnibase.wheel_move_to(wn, p_des)

            if slider.hw_valid and current_slider_val is not None:
                if not args.print_only:
                    if controller.end_of_arm is not None and hasattr(controller.end_of_arm, 'joints'):
         
                        for eoa_j in controller.end_of_arm.joints:
                
                            if eoa_j in ['stretch_gripper', 'parallel_gripper']:
                                if args.tool_nil_controller:
                                    pass
                                elif args.pg4c and eoa_j == 'parallel_gripper':
                                    pg4_range = controller.robot_params.get('parallel_gripper', {}).get('range_mm', 80.0)
                                    controller.end_of_arm.move_to_mm('parallel_gripper', current_slider_val * pg4_range)
                                else:
                                    params = controller.robot_params.get(eoa_j, {})
                                    pct_max_open=100*abs(params['range_deg'][1]/params['range_deg'][0]) if 'range_deg' in params and params['range_deg'][0] != 0 else 100.0
                                    target_pct = current_slider_val * (pct_max_open + 100.0) - 100.0
                                    controller.end_of_arm.move_to(eoa_j, target_pct)
                                    
                    if not args.no_puppet and puppet is not None and hasattr(puppet, 'end_of_arm') and puppet.end_of_arm is not None:
                        if 'gripper' in active_joints:
                            if args.pg4 and 'parallel_gripper' in puppet.end_of_arm.joints:
                                pg4_range = puppet.robot_params.get('parallel_gripper', {}).get('range_mm', 80.0)
                                pg4_cmd_log_mm = current_slider_val * pg4_range
                                puppet.end_of_arm.move_to_mm('parallel_gripper', pg4_cmd_log_mm)
                            elif 'stretch_gripper' in puppet.end_of_arm.joints:
                                params = puppet.robot_params.get('stretch_gripper', {})
                                pct_max_open=100*abs(params['range_deg'][1]/params['range_deg'][0]) if 'range_deg' in params and params['range_deg'][0] != 0 else 100.0
                                target_pct = current_slider_val * (pct_max_open + 100.0) - 100.0
                                puppet.end_of_arm.move_to('stretch_gripper', target_pct)
                
            controller.push_command()
            if not args.no_puppet and puppet is not None:
                puppet.push_command()

            # ---- Build table of joint positions ----
            print_table=False
            if print_table:
                lines = []
                if args.no_puppet:
                    lines.append("┌──────────────────────┬──────────────────────┐")
                    lines.append("│ Joint                │ Controller               │")
                    lines.append("├──────────────────────┼──────────────────────┤")
                else:
                    lines.append("┌──────────────────────┬──────────────────────┬──────────────────────┐")
                    lines.append("│ Joint                │ Controller               │ Puppet                │")
                    lines.append("├──────────────────────┼──────────────────────┼──────────────────────┤")

                if 'lift' in active_joints and controller.lift is not None:
                    pos = controller.lift.status.get('pos', 0.0)
                    if args.no_puppet:
                        lines.append(f"│ {'Lift':<20} │ {pos:>18.4f} m │")
                    else:
                        p_pos = puppet.lift.status.get('pos', 0.0) if puppet.lift is not None else 0.0
                        lines.append(f"│ {'Lift':<20} │ {pos:>18.4f} m │ {p_pos:>18.4f} m │")

                if 'arm' in active_joints and controller.arm is not None:
                    pos = controller.arm.status.get('pos', 0.0)
                    if args.no_puppet:
                        lines.append(f"│ {'Arm':<20} │ {pos:>18.4f} m │")
                    else:
                        p_pos = puppet.arm.status.get('pos', 0.0) if puppet.arm is not None else 0.0
                        lines.append(f"│ {'Arm':<20} │ {pos:>18.4f} m │ {p_pos:>18.4f} m │")

                if 'omnibase' in active_joints and controller.omnibase is not None:
                    x = controller.omnibase.status.get('x', 0.0)
                    y = controller.omnibase.status.get('y', 0.0)
                    theta = controller.omnibase.status.get('theta', 0.0)

                    if args.no_puppet:
                        lines.append(f"│ {'Base X':<20} │ {x:>18.4f} m │")
                        lines.append(f"│ {'Base Y':<20} │ {y:>18.4f} m │")
                        lines.append(f"│ {'Base Theta':<20} │ {math.degrees(theta):>17.2f} deg│")
                    else:
                        p_x = puppet.omnibase.status.get('x', 0.0) if puppet.omnibase is not None else 0.0
                        p_y = puppet.omnibase.status.get('y', 0.0) if puppet.omnibase is not None else 0.0
                        p_theta = puppet.omnibase.status.get('theta', 0.0) if puppet.omnibase is not None else 0.0
                        lines.append(f"│ {'Base X':<20} │ {x:>18.4f} m │ {p_x:>18.4f} m │")
                        lines.append(f"│ {'Base Y':<20} │ {y:>18.4f} m │ {p_y:>18.4f} m │")
                        lines.append(f"│ {'Base Theta':<20} │ {math.degrees(theta):>17.2f} deg│ {math.degrees(p_theta):>17.2f} deg│")

                if controller.end_of_arm is not None and hasattr(controller.end_of_arm, 'joints'):
                    eoa_status = controller.status.get('end_of_arm', {})
                    p_eoa_status = puppet.status.get('end_of_arm', {}) if not args.no_puppet and puppet.end_of_arm is not None else {}
                    for eoa_j in controller.end_of_arm.joints:
                        if eoa_j in ('wrist_yaw', 'wrist_pitch', 'wrist_roll'):
                            if eoa_j not in active_joints:
                                continue

                        joint_status = eoa_status.get(eoa_j, {})
                        pos = joint_status.get('pos', 0.0)
                        label = eoa_j.replace('_', ' ').title()
                        
                        if args.pg4c and eoa_j in ['stretch_gripper', 'parallel_gripper']:
                            c_pos_str = f"{joint_status.get('pos_mm', 0.0):>18.2f} mm"
                        else:
                            c_pos_str = f"{math.degrees(pos):>17.2f} deg"

                        if args.no_puppet:
                            lines.append(f"│ {label:<20} │ {c_pos_str}│")
                        else:
                            if args.pg4 and eoa_j in ['stretch_gripper', 'parallel_gripper']:
                                p_joint_status = p_eoa_status.get('parallel_gripper', {})
                                p_pos_mm = p_joint_status.get('pos_mm', 0.0)
                                p_pos_str = f"{p_pos_mm:>18.2f} mm"
                            else:
                                trg_j = 'stretch_gripper' if eoa_j in ['stretch_gripper', 'parallel_gripper'] else eoa_j
                                p_joint_status = p_eoa_status.get(trg_j, {})
                                p_pos = p_joint_status.get('pos', 0.0)
                                p_pos_str = f"{math.degrees(p_pos):>17.2f} deg"
                                
                            lines.append(f"│ {label:<20} │ {c_pos_str}│ {p_pos_str}│")

                if current_slider_val is not None:
                    val_str = f"{current_slider_val:.4f}"
                    if args.no_puppet:
                        lines.append(f"│ {'Gripper Slider':<20} │ {val_str:>20} │")
                    else:
                        lines.append(f"│ {'Gripper Slider':<20} │ {val_str:>20} │ {'-':>20} │")

                if pg4_cmd_log_mm is not None:
                    cmd_str = f"{pg4_cmd_log_mm:.2f} mm"
                    lines.append(f"│ {'Cmd to PG4':<20} │ {'-':>20} │ {cmd_str:>20} │")

                if args.no_puppet:
                    lines.append("└──────────────────────┴──────────────────────┘")
                else:
                    lines.append("├──────────────────────┼──────────────────────┼──────────────────────┤")
                    lines.append(f"│ {'Update Rate':<20} │ {'-':>20} │ {loop_rate_hz:>17.2f} Hz │")
                    lines.append("└──────────────────────┴──────────────────────┴──────────────────────┘")

                if time.time() - last_print_time >= 0.1:
                    if num_lines_printed > 0:
                        print(f"\033[{num_lines_printed}A", end="")

                    output = "\n".join(lines)
                    print(output, flush=True)
                    num_lines_printed = len(lines)
                    last_print_time = time.time()
            
            t_diff = time.time() - t_start
            t_sleep = max(0, rate - t_diff)
            time.sleep(t_sleep)
            
    except (KeyboardInterrupt, SystemExit):
        print("\nExiting puppet teleop...")
    finally:
        controller.stop()
        if puppet:
            puppet.stop()

if __name__ == '__main__':
    main()

