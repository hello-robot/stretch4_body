#!/usr/bin/env python3

import sys
import time
from stretch4_body.robot.robot_client import RobotClient

def test_api(name, func, *args, **kwargs):
    print(f"[TEST] {name} ... ", end='', flush=True)
    try:
        func(*args, **kwargs)
        print("PASS")
    except Exception as e:
        print(f"FAIL (Exception): {e}")

def main():
    print("Initializing RobotClient...")
    try:
        r = RobotClient()
        success = r.startup()
        if not success:
            print("WARNING: RobotClient.startup() returned False. Is stretch_body_server running?")
            print("Continuing to test APIs to ensure they handle the state gracefully or raise expected errors.")
    except Exception as e:
        print(f"CRITICAL: Failed to instantiate or startup RobotClient: {e}")
        return

    # --- RobotClient APIs ---
    print("\n--- Testing RobotClient APIs ---")
    test_api("robot.is_homed", r.is_homed)
    test_api("robot.get_guarded_contact_modes", r.get_guarded_contact_modes)
    test_api("robot.set_guarded_contact_sensitivity('default')", r.set_guarded_contact_sensitivity, 'default')
    # These might block or wait, so use with caution in a test script. Passing timeout to return quickly if possible.
    # Note: wait_on_motion_start/finish might hang if robot is not actually moving. 
    # checking they don't crash is the goal.
    test_api("robot.wait_on_motion_start(['arm'], timeout=0.1)", r.wait_on_motion_start, ['arm'], timeout=0.1)
    test_api("robot.wait_on_motion_finish(['arm'], timeout=0.1)", r.wait_on_motion_finish, ['arm'], timeout=0.1)
    
    # --- RoutinesClient APIs ---
    print("\n--- Testing RoutinesClient APIs ---")
    if hasattr(r, 'routines'):
        # Many of these are blocking, so we pass wait_on_completion=False or small timeout where applicable
        # The API definition says wait_on_completion=True by default. 
        # We'll test queuing the commands.
        test_api("routines.run('routine_nop', wait_on_completion=False)", r.routines.run, 'routine_nop', wait_on_completion=False)
        test_api("routines.routine_robot_stow(wait_on_completion=False)", r.routines.routine_robot_stow, wait_on_completion=False)
        test_api("routines.routine_robot_home(wait_on_completion=False)", r.routines.routine_robot_home, wait_on_completion=False)
        test_api("routines.routine_lift_home(wait_on_completion=False)", r.routines.routine_lift_home, wait_on_completion=False)
        test_api("routines.routine_arm_home(wait_on_completion=False)", r.routines.routine_arm_home, wait_on_completion=False)
        test_api("routines.routine_blind_dock(wait_on_completion=False)", r.routines.routine_blind_dock, wait_on_completion=False)
        if hasattr(r, 'end_of_arm') and hasattr(r.end_of_arm, 'motors'):
             # Try to find a wrist joint name
             joints = list(r.end_of_arm.motors.keys())
             if joints:
                 test_api(f"routines.routine_wrist_joint_home('{joints[0]}', wait=False)", r.routines.routine_wrist_joint_home, joints[0], wait_on_completion=False)
    else:
        print("FAIL: RobotClient has no 'routines' attribute")

    # --- PowerPeriphClient APIs ---
    print("\n--- Testing PowerPeriphClient APIs ---")
    if hasattr(r, 'power_periph'):
        test_api("power_periph.trigger_beep", r.power_periph.trigger_beep)
        test_api("power_periph.set_charger_on", r.power_periph.set_charger_on)
        test_api("power_periph.set_charger_off", r.power_periph.set_charger_off)
        test_api("power_periph.set_fan_on", r.power_periph.set_fan_on)
        test_api("power_periph.set_fan_off", r.power_periph.set_fan_off)
        # actuator_control(motor_type, enable)
        test_api("power_periph.actuator_control('arm', True)", r.power_periph.actuator_control, 'arm', True)
    else:
        print("FAIL: RobotClient has no 'power_periph' attribute")

    # --- OmniBaseClient APIs ---
    print("\n--- Testing OmniBaseClient APIs ---")
    if hasattr(r, 'omnibase'):
        test_api("omnibase.translate_by(0.1, 0.0)", r.omnibase.translate_by, 0.1, 0.0)
        test_api("omnibase.rotate_by(0.1)", r.omnibase.rotate_by, 0.1)
        test_api("omnibase.set_velocity(0.1, 0.0, 0.1)", r.omnibase.set_velocity, 0.1, 0.0, 0.1)
        test_api("omnibase.move_by(0.1, 0.0, 0.1)", r.omnibase.move_by, 0.1, 0.0, 0.1)
        test_api("omnibase.enable_freewheel_mode", r.omnibase.enable_freewheel_mode)
        test_api("omnibase.enable_hold_mode", r.omnibase.enable_hold_mode)
        test_api("omnibase.set_guarded_contact_sensitivity('default')", r.omnibase.set_guarded_contact_sensitivity, 'default')
        test_api("omnibase.get_guarded_contact_modes", r.omnibase.get_guarded_contact_modes)
        test_api("omnibase.hard_stop", r.omnibase.hard_stop)
        # Stop performs a shutdown of the client partially, so do it last for this subsystem
        test_api("omnibase.stop", r.omnibase.stop)
    else:
        print("FAIL: RobotClient has no 'omnibase' attribute")

    # --- PrismaticJointClient APIs (Arm/Lift) ---
    for joint_name in ['arm', 'lift']:
        print(f"\n--- Testing {joint_name.capitalize()}Client APIs ---")
        if hasattr(r, joint_name):
            j = getattr(r, joint_name)
            test_api(f"{joint_name}.enable_safety", j.enable_safety)
            test_api(f"{joint_name}.is_homed", j.is_homed)
            test_api(f"{joint_name}.disable_sync_mode", j.disable_sync_mode)
            test_api(f"{joint_name}.enable_sync_mode", j.enable_sync_mode)
            test_api(f"{joint_name}.move_by(0.01)", j.move_by, 0.01)
            test_api(f"{joint_name}.move_to(0.1)", j.move_to, 0.1)
            test_api(f"{joint_name}.set_velocity(0.01)", j.set_velocity, 0.01)
            test_api(f"{joint_name}.set_guarded_contact_sensitivity('default')", j.set_guarded_contact_sensitivity, 'default')
            # Home blocks, so skip or use with caution. Current impl prints and waits.
            # We can try to mock the wait or just skip it to avoid hanging if no server.
            print(f"[INFO] Skipping {joint_name}.home() as it blocks indefinitely if no server response.")
            # test_api(f"{joint_name}.home", j.home) 
            test_api(f"{joint_name}.stop", j.stop)
        else:
            print(f"FAIL: RobotClient has no '{joint_name}' attribute")

    # --- EndOfArm (Wrist/Gripper) ---
    print("\n--- Testing EndOfArm APIs ---")
    if hasattr(r, 'end_of_arm'):
        # Iterate over motors in end_of_arm
        if hasattr(r.end_of_arm, 'motors'):
            for m_name, m_client in r.end_of_arm.motors.items():
                print(f"  Testing motor: {m_name}")
                test_api(f"end_of_arm.motors['{m_name}'].move_by(0.1)", m_client.move_by, 0.1)
                test_api(f"end_of_arm.motors['{m_name}'].move_to(0.1)", m_client.move_to, 0.1)
                test_api(f"end_of_arm.motors['{m_name}'].set_velocity(0.1)", m_client.set_velocity, 0.1)
                test_api(f"end_of_arm.motors['{m_name}'].do_ping()", m_client.do_ping)
                test_api(f"end_of_arm.motors['{m_name}'].stop()", m_client.stop)
    else:
        print("FAIL: RobotClient has no 'end_of_arm' attribute")


    print("\n[TEST COMPLETED]")
    try:
        r.stop()
    except Exception as e:
        print(f"WARNING: r.stop() failed (expected if startup failed): {e}")

if __name__ == "__main__":
    main()
