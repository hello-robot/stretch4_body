#!/usr/bin/env python3
import math
import subprocess
import os
import tempfile
import yaml
from stretch4_body.utils.stretch_pose_models import RobotJoints
from stretch4_body.subsystem.end_of_arm.stretch_gripper import StretchGripper
from unittest.mock import MagicMock, patch

def test_stretch_gripper_direct_commands():
    # Mock RobotParams.get_params to provide stretch_gripper parameters during instantiation
    dummy_params = {
        'stretch_gripper': {
            'range_deg': [-100.0, 0.0],
            'usb_uri': 'dummy_uri',
            'gripper_conversion': {
                'finger_length_m': 0.18205,
                'aperture_open_m': 0.150,
                'aperture_closed_m': 0.0
            }
        }
    }
    # command_to_actuator/actuator_to_command (StretchGripperMetadata, promoted from the
    # driver's old pct_to_world_rad/world_rad_to_pct, matching PG4's aperture_to_actuator/
    # actuator_to_aperture pattern) read RobotParams.get_params() live rather than a snapshot,
    # so keep the patch active for the whole test rather than just construction.
    with patch('stretch4_body.core.device.RobotParams.get_params', return_value=({}, dummy_params)), \
         patch('stretch4_body.utils.tool_metadata.RobotParams.get_params', return_value=({}, dummy_params)):
        gripper = StretchGripper()

        # Mock parent calls
        mock_move_to = MagicMock()
        from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
        FeetechSMHello.move_to = mock_move_to

        # Test conversions
        assert math.isclose(gripper.tool_metadata.command_to_actuator(-100.0), math.radians(-100.0), abs_tol=1e-5)
        assert math.isclose(gripper.tool_metadata.actuator_to_command(math.radians(-100.0)), -100.0, abs_tol=1e-5)

        # Test move_to
        gripper.move_to(50.0)
        mock_move_to.assert_called_once_with(gripper, x_des=gripper.tool_metadata.command_to_actuator(50.0), v_des=None, a_des=None)
    print("StretchGripper direct move_to test passed!")

def test_robot_joints_properties_stretch():
    from unittest.mock import patch, PropertyMock
    with patch.object(RobotJoints, 'gripper_name', new_callable=PropertyMock, return_value='stretch_gripper'):
        joints = RobotJoints.gripper.finger_joints
        links = RobotJoints.gripper.finger_links
        print("stretch_gripper finger_joints:", joints)
        print("stretch_gripper finger_links:", links)
        
        assert "gripper_finger_left_joint" in joints
        assert "gripper_finger_left_link" in links
        
        # -100 deg is -1.745329... rad.
        # If position is -1.745329... rad, expected percent is -100.0% (closed).
        val_pct = RobotJoints.gripper.urdf_to_command(-1.7453292519943295)
        print("stretch_gripper rad to command units:", val_pct)
        assert math.isclose(val_pct, -100.0, abs_tol=0.01)

def test_scripts_auto_detect_stretch():
    print("Testing auto-detection on scripts for stretch_gripper...")
    
    with tempfile.TemporaryDirectory() as tmpdir:
        # Create fake stretch_user_params.yaml and stretch_configuration_params.yaml
        fleet_id = "stretch-test-sg"
        fleet_dir = os.path.join(tmpdir, fleet_id)
        os.makedirs(fleet_dir, exist_ok=True)
        
        # Minimal configurations to load nominal parameters for SE4/sg4
        user_params = {
            'robot': {
                'model_name': 'SE4',
                'tool': 'eoa_wrist_dw4_tool_sg4'
            }
        }
        config_params = {
            'robot': {
                'model_name': 'SE4',
                'tool': 'eoa_wrist_dw4_tool_sg4'
            }
        }
        
        with open(os.path.join(fleet_dir, 'stretch_user_params.yaml'), 'w') as f:
            yaml.dump(user_params, f)
        with open(os.path.join(fleet_dir, 'stretch_configuration_params.yaml'), 'w') as f:
            yaml.dump(config_params, f)
            
        env = os.environ.copy()
        env['HELLO_FLEET_PATH'] = tmpdir
        env['HELLO_FLEET_ID'] = fleet_id
        
        # Run stretch_gripper_home, expect clean exit
        res_home = subprocess.run(["python3", "-m", "stretch4_body.tools.stretch_gripper_home"], capture_output=True, text=True, env=env)
        print("stretch_gripper_home exit code:", res_home.returncode)
        assert res_home.returncode == 0, f"Expected 0, got:\n{res_home.stderr}"
        
        # Run stretch_gripper_jog, passing empty input
        res_jog = subprocess.run(["python3", "-m", "stretch4_body.tools.stretch_gripper_jog"], input="", capture_output=True, text=True, env=env)
        print("stretch_gripper_jog exit code:", res_jog.returncode)
        assert res_jog.returncode == 0
        
        print("Scripts auto-detect checks passed!")

if __name__ == "__main__":
    test_stretch_gripper_direct_commands()
    test_robot_joints_properties_stretch()
    test_scripts_auto_detect_stretch()
    print("All stretch_gripper tests passed successfully!")
