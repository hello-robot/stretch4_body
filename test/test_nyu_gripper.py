#!/usr/bin/env python3
import math
import subprocess
import os
import tempfile
import yaml
from stretch4_body.utils.stretch_pose_models import RobotJoints
from stretch4_body.subsystem.end_of_arm.nyu_gripper import NYUGripper
from stretch4_body.core.robot_params import RobotParams
from unittest.mock import MagicMock, patch

DUMMY_GRIPPER_PARAMS = {
    'range_deg': [0.0, 420.0],
    'usb_uri': 'dummy_uri',
    'gripper_conversion': {
        'finger_length_m': 0.10,
        'aperture_open_m': 0.08,
        'aperture_closed_m': 0.0
    }
}

def _make_gripper():
    dummy_params = {'nyu_gripper': dict(DUMMY_GRIPPER_PARAMS)}
    with patch('stretch4_body.core.device.RobotParams.get_params', return_value=({}, dummy_params)):
        gripper = NYUGripper()
    gripper.params = dict(DUMMY_GRIPPER_PARAMS)
    return gripper

def test_nyu_gripper_direct_commands():
    gripper = _make_gripper()

    # Mock parent calls
    mock_move_to = MagicMock()
    from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
    FeetechSMHello.move_to = mock_move_to

    # Test conversions: 0 pct = closed hardstop (0 rad), 100 pct = range_deg[1]
    assert math.isclose(gripper.pct_to_world_rad(0.0), 0.0, abs_tol=1e-6)
    assert math.isclose(gripper.pct_to_world_rad(100.0), math.radians(420.0), abs_tol=1e-5)
    assert math.isclose(gripper.world_rad_to_pct(math.radians(420.0)), 100.0, abs_tol=1e-5)
    assert math.isclose(gripper.world_rad_to_pct(math.radians(210.0)), 50.0, abs_tol=1e-5)
    assert gripper.pct_max_open == 100.0
    assert gripper.poses['close'] == 0.0
    assert gripper.poses['zero'] == 0.0
    assert math.isclose(gripper.poses['open'], 100.0)

    # Test move_to
    gripper.move_to(50.0)
    mock_move_to.assert_called_once_with(gripper, x_des=gripper.pct_to_world_rad(50.0), v_des=None, a_des=None)
    print("NYUGripper direct move_to test passed!")

def test_nyu_gripper_conversion_status():
    gripper = _make_gripper()

    gripper.status['pos_pct'] = 0.0
    gc0 = gripper.get_conversion_status()
    assert math.isclose(gc0['aperture_m'], 0.0, abs_tol=1e-6)
    assert math.isclose(gc0['finger_rad'], 0.0, abs_tol=1e-6)

    gripper.status['pos_pct'] = 100.0
    gc100 = gripper.get_conversion_status()
    assert math.isclose(gc100['aperture_m'], 0.08, abs_tol=1e-6)
    expected_finger_rad = math.asin(0.08 / (2 * 0.10))
    assert math.isclose(gc100['finger_rad'], expected_finger_rad, abs_tol=1e-6)
    print("NYUGripper conversion status test passed!")

def test_robot_joints_properties_nyu():
    with patch.object(RobotJoints.gripper, 'get_gripper', return_value='nyu_gripper'):
        joints = RobotJoints.gripper.finger_joints
        links = RobotJoints.gripper.finger_links
        print("nyu_gripper finger_joints:", joints)
        print("nyu_gripper finger_links:", links)

        assert "ng_finger_left_joint" in joints
        assert "ng_finger_left_link" in links

        # Round trip: finger_rad at fully open must map back to 100 pct.
        # Source the same params to_subsystem_units will read (falls back to
        # defaults when the ng4 tool is not the active tool on this machine).
        _, robot_params = RobotParams.get_params()
        gc = robot_params.get('nyu_gripper', {}).get('gripper_conversion', {})
        aperture_open_m = gc.get('aperture_open_m', 0.08)
        finger_length_m = gc.get('finger_length_m', 0.10)
        finger_rad_open = math.asin(aperture_open_m / (2 * finger_length_m))
        val_pct = RobotJoints.gripper.to_subsystem_units(finger_rad_open)
        print("nyu_gripper finger_rad to subsystem units:", val_pct)
        assert math.isclose(val_pct, 100.0, abs_tol=0.5)

def test_scripts_auto_detect_nyu():
    print("Testing auto-detection on scripts for nyu_gripper...")

    with tempfile.TemporaryDirectory() as tmpdir:
        fleet_id = "stretch-test-ng"
        fleet_dir = os.path.join(tmpdir, fleet_id)
        os.makedirs(fleet_dir, exist_ok=True)

        # Minimal configurations to load nominal parameters for SE4/ng4.
        # This also smoke-tests every robot_params_SE4.py registration:
        # RobotParams sys.exit(1)s if the tool is not fully registered.
        user_params = {
            'robot': {
                'model_name': 'SE4',
                'tool': 'eoa_wrist_dw4_tool_ng4'
            }
        }
        config_params = {
            'robot': {
                'model_name': 'SE4',
                'tool': 'eoa_wrist_dw4_tool_ng4'
            }
        }

        with open(os.path.join(fleet_dir, 'stretch_user_params.yaml'), 'w') as f:
            yaml.dump(user_params, f)
        with open(os.path.join(fleet_dir, 'stretch_configuration_params.yaml'), 'w') as f:
            yaml.dump(config_params, f)

        env = os.environ.copy()
        env['HELLO_FLEET_PATH'] = tmpdir
        env['HELLO_FLEET_ID'] = fleet_id

        res_home = subprocess.run(["python3", "-m", "stretch4_body.tools.stretch_gripper_home"], capture_output=True, text=True, env=env)
        print("stretch_gripper_home exit code:", res_home.returncode)
        assert res_home.returncode == 0, f"Expected 0, got:\n{res_home.stderr}"

        res_jog = subprocess.run(["python3", "-m", "stretch4_body.tools.stretch_gripper_jog"], input="", capture_output=True, text=True, env=env)
        print("stretch_gripper_jog exit code:", res_jog.returncode)
        assert res_jog.returncode == 0
        assert "close by 10%" in res_jog.stdout or "close by 10" in res_jog.stdout

        print("Scripts auto-detect checks passed!")

if __name__ == "__main__":
    test_nyu_gripper_direct_commands()
    test_nyu_gripper_conversion_status()
    test_robot_joints_properties_nyu()
    test_scripts_auto_detect_nyu()
    print("All nyu_gripper tests passed successfully!")
