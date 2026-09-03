#!/usr/bin/env python3
import math
import subprocess
from unittest.mock import PropertyMock, patch
from stretch4_body.utils.tool_metadata import ParallelGripperMetadata
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
    metadata = ParallelGripperMetadata()

    # gripper_conversion.py's free functions were folded into ToolMetadata; _params and
    # _finger_joint_limits are patched directly so this exercises the same geometry with the
    # same inputs, independent of whatever tool this test environment's robot_params configure.
    with patch.object(ParallelGripperMetadata, '_params', new_callable=PropertyMock, return_value=params), \
         patch.object(ParallelGripperMetadata, '_finger_joint_limits', new_callable=PropertyMock, return_value=(-0.04, 0.0)):

        # Test aperture (meters) to URDF meters -- PG4's command units are aperture directly.
        val_closed = metadata.command_to_urdf(0.0)
        val_open = metadata.command_to_urdf(80.0 / 1000.0)
        print(f"aperture to URDF: closed={val_closed}m, open={val_open}m")
        assert math.isclose(val_closed, 0.0, abs_tol=1e-5), f"Expected 0.0, got {val_closed}"
        assert math.isclose(val_open, -0.04, abs_tol=1e-5), f"Expected -0.04, got {val_open}"

        # Test servo radians to URDF meters
        val_rad_closed = metadata.actuator_to_urdf(0.0)
        val_rad_open = metadata.actuator_to_urdf(math.radians(116.5))
        print(f"servo rad to URDF: closed={val_rad_closed}m, open={val_rad_open}m")
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
    assert math.isclose(sub_val, 0.08, abs_tol=0.005)

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

    # Mock RobotParams.get_params to provide parallel_gripper parameters during instantiation
    dummy_params = {
        'parallel_gripper': {
            'kL': 30.25,
            'kR': 22.0,
            'kT0': 44.0,
            'kX0': 10.5,
            'range_deg': [0, 116.5],
            'range_mm': 80.0
        }
    }
    with patch('stretch4_body.core.device.RobotParams.get_params', return_value=({}, dummy_params)):
        gripper = ParallelGripper()

    # Mock FeetechSMHello.move_to (the parent call)
    mock_move_to = MagicMock()
    from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
    FeetechSMHello.move_to = mock_move_to

    with patch('stretch4_body.core.device.RobotParams.get_params', return_value=({}, dummy_params)):
        # Call move_to with 0.05 m (comfortably inside command_range, away from the 'open' bound --
        # the geometry model rounds to the nearest mm, so command_range's upper bound lands a hair
        # under the nominal 0.08 m and would otherwise get silently clamped here)
        gripper.move_to(0.05)

        # Ensure it translated 0.05 m into servo radians using the tool metadata's own conversion
        expected_rad = gripper.tool_metadata.aperture_to_actuator(0.05)
    mock_move_to.assert_called_once_with(gripper, x_des=expected_rad, v_des=None, a_des=None)
    print("ParallelGripper direct move_to test passed!")

if __name__ == "__main__":
    test_conversions()
    test_param_lookup()
    test_robot_joints_properties()
    test_parallel_gripper_direct_commands()
    test_scripts_auto_detect()
    print("All tests passed successfully!")
