#!/usr/bin/env python3
import math
import subprocess
from stretch4_body.subsystem.end_of_arm.gripper_conversion import (
    parallel_gripper_pos_mm_to_urdf_m,
    parallel_gripper_rad_to_urdf_m
)
from stretch4_body.utils.stretch_pose_models import RobotJoints
from stretch4_body.core.gamepad_enums import MotionProfile

def test_conversions():
    params = {
        'kL': 30.25,
        'kR': 22.0,
        'kT0': 44.0,
        'kX0': 10.5,
        'range_deg': [0, 116.5],
        'range_mm': 80.0
    }
    
    # Test pos_mm to URDF meters
    val_closed = parallel_gripper_pos_mm_to_urdf_m(0.0, params)
    val_open = parallel_gripper_pos_mm_to_urdf_m(80.0, params)
    print(f"pos_mm to URDF: closed={val_closed}m, open={val_open}m")
    assert math.isclose(val_closed, 0.0, abs_tol=1e-5), f"Expected 0.0, got {val_closed}"
    assert math.isclose(val_open, -0.04, abs_tol=1e-5), f"Expected -0.04, got {val_open}"
    
    # Test rad to URDF meters
    val_rad_closed = parallel_gripper_rad_to_urdf_m(0.0, params)
    val_rad_open = parallel_gripper_rad_to_urdf_m(math.radians(116.5), params)
    print(f"rad to URDF: closed={val_rad_closed}m, open={val_rad_open}m")
    assert math.isclose(val_rad_closed, 0.0, abs_tol=0.005), f"Expected close to 0.0, got {val_rad_closed}"
    assert math.isclose(val_rad_open, -0.04, abs_tol=0.005), f"Expected close to -0.04, got {val_rad_open}"
    print("Conversions tests passed!")

def test_param_lookup():
    v, a = RobotJoints.gripper.get_joint_params(MotionProfile.SLOW)
    print(f"Joint params for gripper: vel={v}, accel={a}")
    assert v is not None and a is not None
    print("Param lookup test passed!")

def test_robot_joints_properties():
    # Test finger joints and links based on active gripper configuration
    joints = RobotJoints.gripper.finger_joints
    links = RobotJoints.gripper.finger_links
    print("RobotJoints.gripper.finger_joints:", joints)
    print("RobotJoints.gripper.finger_links:", links)
    
    # Active gripper is parallel_gripper in robot_params
    assert "finger_left_joint" in joints
    assert "finger_left_link" in links
    
    # Test to_subsystem_units conversion
    sub_val = RobotJoints.gripper.to_subsystem_units(0.08)
    print("0.08m to subsystem units:", sub_val)
    assert math.isclose(sub_val, 80.0, abs_tol=0.005)

    # Test stretch_gripper conversion
    from unittest.mock import patch
    with patch.object(RobotJoints.gripper, 'get_gripper', return_value='stretch_gripper'):
        # For stretch_gripper, to_subsystem_units converts radians to percent.
        # -100 deg is -1.745329... rad.
        # If position is -1.745329... rad, expected percent is -100.0% (closed).
        val_pct = RobotJoints.gripper.to_subsystem_units(-1.7453292519943295)
        print("stretch_gripper rad to subsystem units:", val_pct)
        assert math.isclose(val_pct, -100.0, abs_tol=0.01)
    
    print("RobotJoints properties test passed!")

def test_scripts_auto_detect():
    print("Testing auto-detection on scripts...")
    
    # Run stretch_gripper_home, expect clean exit (returncode 0) because startup fails offline
    res_home = subprocess.run(["python3", "-m", "stretch4_body.tools.stretch_gripper_home"], capture_output=True, text=True)
    print("stretch_gripper_home exit code:", res_home.returncode)
    assert res_home.returncode == 0, f"Expected 0, got:\n{res_home.stderr}"
    
    # Run stretch_gripper_jog, passing empty input to stdin so it prints menu and exits
    res_jog = subprocess.run(["python3", "-m", "stretch4_body.tools.stretch_gripper_jog"], input="", capture_output=True, text=True)
    print("stretch_gripper_jog exit code:", res_jog.returncode)
    assert "close by 10mm" in res_jog.stdout, f"Expected parallel gripper menu, got:\n{res_jog.stdout}"
    
    print("Scripts auto-detect checks passed!")

def test_parallel_gripper_direct_commands():
    from stretch4_body.subsystem.end_of_arm.parallel_gripper import ParallelGripper
    from unittest.mock import MagicMock
    
    gripper = ParallelGripper()
    gripper.params = {
        'kL': 30.25,
        'kR': 22.0,
        'kT0': 44.0,
        'kX0': 10.5,
        'range_deg': [0, 116.5],
        'range_mm': 80.0
    }
    
    # Mock FeetechSMHello.move_to (the parent call)
    mock_move_to = MagicMock()
    from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
    FeetechSMHello.move_to = mock_move_to
    
    # Call move_to with 80.0 mm
    gripper.move_to(80.0)
    
    # Ensure it translated 80.0 mm into servo radians using parallel_gripper_mm_to_servo_rad
    from stretch4_body.subsystem.end_of_arm.gripper_conversion import parallel_gripper_mm_to_servo_rad
    expected_rad = parallel_gripper_mm_to_servo_rad(80.0, gripper.params)
    mock_move_to.assert_called_once_with(gripper, x_des=expected_rad, v_des=None, a_des=None)
    print("ParallelGripper direct move_to test passed!")

if __name__ == "__main__":
    test_conversions()
    test_param_lookup()
    test_robot_joints_properties()
    test_parallel_gripper_direct_commands()
    test_scripts_auto_detect()
    print("All tests passed successfully!")
