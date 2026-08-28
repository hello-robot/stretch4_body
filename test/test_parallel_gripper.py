#!/usr/bin/env python3
import math
import subprocess
from unittest.mock import patch
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.utils.tool_metadata import ParallelGripperMetadata
from stretch4_body.utils.stretch_pose_models import RobotJoints
from stretch4_body.core.gamepad_enums import MotionProfile

def _patched_parallel_gripper_params(params):
    """
    Context manager that makes RobotParams.get_params() report `params` under
    robot_params['parallel_gripper'], leaving everything else (including robot/model_name,
    needed by ParallelGripperMetadata._finger_joint_limits to load the real URDF) untouched.
    """
    _user, real_robot_params = RobotParams.get_params()
    patched_robot_params = dict(real_robot_params)
    patched_robot_params['parallel_gripper'] = params
    return patch.object(RobotParams, 'get_params', return_value=(_user, patched_robot_params))

def test_conversions():
    params = {
        'kL': 30.25,
        'kR': 22.0,
        'kT0': 44.0,
        'kX0': 10.5,
        'range_deg': [0, 116.5],
        'range_mm': 80.0
    }

    with _patched_parallel_gripper_params(params):
        meta = ParallelGripperMetadata()

        # Test aperture (meters) to URDF meters
        val_closed = meta.aperture_to_urdf(0.0)
        val_open = meta.aperture_to_urdf(0.08)
        print(f"aperture to URDF: closed={val_closed}m, open={val_open}m")
        assert math.isclose(val_closed, 0.0, abs_tol=1e-5), f"Expected 0.0, got {val_closed}"
        assert math.isclose(val_open, -0.04, abs_tol=1e-5), f"Expected -0.04, got {val_open}"

        # Test command (== aperture, in meters) to URDF meters. command_to_urdf shares units
        # with aperture_to_urdf: PG4's command tier is defined in aperture space so it matches
        # what this tool's own move_to()/move_by() take directly.
        val_command_closed = meta.command_to_urdf(0.0)
        val_command_open = meta.command_to_urdf(0.08)
        print(f"command to URDF: closed={val_command_closed}m, open={val_command_open}m")
        assert math.isclose(val_command_closed, 0.0, abs_tol=1e-5), f"Expected 0.0, got {val_command_closed}"
        assert math.isclose(val_command_open, -0.04, abs_tol=1e-5), f"Expected -0.04, got {val_command_open}"

        # Test true actuator (raw servo angle, radians) to URDF meters -- a different unit space
        # than command (aperture, meters), unlike SG4 where command and actuator differ only by
        # a linear scale. Round-trip rather than asserting a fixed pair.
        actuator_val = meta.urdf_to_actuator(0.0)
        round_trip = meta.actuator_to_urdf(actuator_val)
        print("PG4 urdf->actuator->urdf round trip:", 0.0, "->", actuator_val, "->", round_trip)
        assert math.isclose(round_trip, 0.0, abs_tol=1e-6)

        # command_to_actuator/actuator_to_command coincide with aperture_to_actuator/
        # actuator_to_aperture for PG4, since PG4's command tier IS aperture.
        assert math.isclose(meta.command_to_actuator(0.08), meta.aperture_to_actuator(0.08))
        assert math.isclose(meta.actuator_to_command(actuator_val), meta.actuator_to_aperture(actuator_val))
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
    
    # Verify joints and links match active tool metadata
    assert len(joints) > 0
    assert len(links) > 0
    if "parallel" in RobotJoints.gripper.gripper_name.lower() or "pg4" in RobotJoints.gripper.gripper_name.lower():
        assert "finger_left_joint" in joints
        assert "finger_left_link" in links
    else:
        assert "gripper_finger_left_joint" in joints
        assert "gripper_finger_left_link" in links
    
    # Test urdf_to_command / command_to_urdf conversions -- the ROS-facing bridge to
    # move_to()/move_by()'s own units.
    if "parallel" in RobotJoints.gripper.gripper_name.lower() or "pg4" in RobotJoints.gripper.gripper_name.lower():
        # PG4's command unit is fingertip aperture in meters, matching what this tool's own
        # move_to()/move_by() take directly -- so round-trip urdf -> command -> urdf rather
        # than asserting a fixed pair tied to a specific URDF calibration.
        command_val = RobotJoints.gripper.urdf_to_command(0.0)
        round_trip = RobotJoints.gripper.command_to_urdf(command_val)
        print("PG4 urdf->command->urdf round trip:", 0.0, "->", command_val, "->", round_trip)
        assert math.isclose(round_trip, 0.0, abs_tol=1e-6)
    else:
        sub_val = RobotJoints.gripper.urdf_to_command(0.08)
        print("0.08 to command units:", sub_val)
        # Stretch gripper converts radians to percent
        assert math.isclose(sub_val, 4.58, abs_tol=0.1)

    # Test true actuator (raw servo angle, radians) round trip, distinct from command above.
    actuator_val = RobotJoints.gripper.urdf_to_actuator(0.0)
    round_trip = RobotJoints.gripper.actuator_to_urdf(actuator_val)
    print("urdf->actuator->urdf round trip:", 0.0, "->", actuator_val, "->", round_trip)
    assert math.isclose(round_trip, 0.0, abs_tol=1e-6)

    # Test stretch_gripper conversion
    from unittest.mock import patch, MagicMock, PropertyMock
    with patch.object(RobotJoints, 'gripper_name', new_callable=PropertyMock, return_value='stretch_gripper'):
        # For stretch_gripper, urdf_to_command converts radians to percent.
        # -100 deg is -1.745329... rad.
        # If position is -1.745329... rad, expected percent is -100.0% (closed).
        val_pct = RobotJoints.gripper.urdf_to_command(-1.7453292519943295)
        print("stretch_gripper rad to command units:", val_pct)
        assert math.isclose(val_pct, -100.0, abs_tol=0.01)

        # command_to_actuator/actuator_to_command are SG4's promoted pct_to_world_rad/
        # world_rad_to_pct: at the fully-closed reference point, Pct=-100 maps to exactly
        # deg_to_rad(range_deg[0]), the same value as the urdf input above.
        actuator_sg = RobotJoints.gripper.command_to_actuator(val_pct)
        assert math.isclose(actuator_sg, -1.7453292519943295, abs_tol=1e-6)
        assert math.isclose(RobotJoints.gripper.actuator_to_command(actuator_sg), val_pct, abs_tol=1e-6)

    # Test gripper_client property: any built-in gripper resolves to a generic
    # WristJointClient wired up by ToolMetadata.client_class (make_joint_client_class),
    # not a bespoke per-gripper class.
    client = RobotJoints.gripper.gripper_client
    from stretch4_body.robot.robot_client import WristJointClient
    assert isinstance(client, WristJointClient)
    assert client.tool_metadata is RobotJoints.gripper.gripper_model

    with patch.object(RobotJoints, 'gripper_name', new_callable=PropertyMock, return_value='stretch_gripper'):
        with patch.dict('stretch4_body.utils.stretch_pose_models.GRIPPER_MODELS') as mock_models:
            mock_meta = MagicMock()
            mock_client = MagicMock()
            mock_meta.client_class.return_value = mock_client
            mock_models['stretch_gripper'] = mock_meta
            client_sg = RobotJoints.gripper.gripper_client
            assert client_sg == mock_client
    # Test get_joint_by_name generic lookup
    assert RobotJoints.get_joint_by_name('gripper') == RobotJoints.gripper
    assert RobotJoints.get_joint_by_name('parallel_gripper') == RobotJoints.gripper
    assert RobotJoints.get_joint_by_name('lift') == RobotJoints.lift
    assert RobotJoints.get_joint_by_name('non_existent') is None

    with patch.object(RobotJoints, 'gripper_name', new_callable=PropertyMock, return_value='stretch_gripper'):
        assert RobotJoints.get_joint_by_name('stretch_gripper') == RobotJoints.gripper
    
    print("RobotJoints properties test passed!")

def test_scripts_auto_detect():
    print("Testing auto-detection on scripts...")
    
    # Run stretch_gripper_home, expect clean exit (returncode 0) because startup fails offline
    res_home = subprocess.run(["python3", "-m", "stretch4_body.tools.stretch_gripper_home"], capture_output=True, text=True)
    print("stretch_gripper_home exit code:", res_home.returncode)
    assert res_home.returncode == 0, f"Expected 0, got:\n{res_home.stderr}"
    
    # Run stretch_gripper_jog, passing empty input to stdin so it exits cleanly
    res_jog = subprocess.run(["python3", "-m", "stretch4_body.tools.stretch_gripper_jog"], input="", capture_output=True, text=True)
    print("stretch_gripper_jog exit code:", res_jog.returncode)
    assert res_jog.returncode == 0, f"Expected clean exit code 0, got:\n{res_jog.stderr}"
    
    print("Scripts auto-detect checks passed!")

def test_parallel_gripper_direct_commands():
    from stretch4_body.subsystem.end_of_arm.parallel_gripper import ParallelGripper
    from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
    from unittest.mock import MagicMock, patch

    def mock_feetech_init(self_obj, *args, **kwargs):
        self_obj.status = {'pos_mm': 0.0}
        self_obj.params = {'range_deg': [0, 116.5]}

    params = {
        'kL': 30.25,
        'kR': 22.0,
        'kT0': 44.0,
        'kX0': 10.5,
        'range_deg': [0, 116.5],
        'range_mm': 80.0
    }

    # ParallelGripperMetadata always reads the live global robot_params (no per-call override,
    # to match the ToolMetadata ABC's uniform conversion signatures), so patch that instead of
    # the gripper's own .params to control the conversion this test checks.
    with _patched_parallel_gripper_params(params):
        with patch.object(FeetechSMHello, '__init__', side_effect=mock_feetech_init):
            gripper = ParallelGripper()

        # Mock FeetechSMHello.move_to (the parent call)
        mock_move_to = MagicMock()
        FeetechSMHello.move_to = mock_move_to

        # Call move_to with 0.08 m
        gripper.move_to(0.08)

        # Ensure it translated 0.08 m into servo radians using the gripper's tool_metadata conversion
        expected_rad = gripper.tool_metadata.aperture_to_actuator(0.08)
        mock_move_to.assert_called_once_with(gripper, x_des=expected_rad, v_des=None, a_des=None)
    print("ParallelGripper direct move_to test passed!")

if __name__ == "__main__":
    test_conversions()
    test_param_lookup()
    test_robot_joints_properties()
    test_parallel_gripper_direct_commands()
    test_scripts_auto_detect()
    print("All tests passed successfully!")
