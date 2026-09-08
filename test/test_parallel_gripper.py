#!/usr/bin/env python3
import math
import subprocess
from unittest.mock import PropertyMock, patch

from stretch4_body.core.gamepad_enums import MotionProfile
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.utils.stretch_pose_models import RobotJoints
from stretch4_body.utils.tool_metadata import ParallelGripperMetadata


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
        # with aperture_to_urdf: PG4's command unit type is defined in aperture space so it matches
        # what this tool's own move_to()/move_by() take directly.
        val_command_closed = meta.command_to_urdf(0.0)
        val_command_open = meta.command_to_urdf(0.08)
        print(f"command to URDF: closed={val_command_closed}m, open={val_command_open}m")
        assert math.isclose(val_command_closed, 0.0, abs_tol=1e-5), f"Expected 0.0, got {val_command_closed}"
        assert math.isclose(val_command_open, -0.04, abs_tol=1e-5), f"Expected -0.04, got {val_command_open}"

        # Test actuator (servo angle, radians) to URDF meters -- a different unit space
        # than command (aperture, meters), unlike SG4 where command and actuator differ only by
        # a linear scale. Round-trip rather than asserting a fixed pair.
        actuator_val = meta.urdf_to_actuator(0.0)
        round_trip = meta.actuator_to_urdf(actuator_val)
        print("PG4 urdf->actuator->urdf round trip:", 0.0, "->", actuator_val, "->", round_trip)
        assert math.isclose(round_trip, 0.0, abs_tol=1e-6)

        # command_to_actuator/actuator_to_command coincide with aperture_to_actuator/
        # actuator_to_aperture for PG4, since PG4's command unit type IS aperture.
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

    # Test actuator (servo angle, radians) round trip, distinct from command above.
    actuator_val = RobotJoints.gripper.urdf_to_actuator(0.0)
    round_trip = RobotJoints.gripper.actuator_to_urdf(actuator_val)
    print("urdf->actuator->urdf round trip:", 0.0, "->", actuator_val, "->", round_trip)
    assert math.isclose(round_trip, 0.0, abs_tol=1e-6)

    # Test stretch_gripper conversion
    from unittest.mock import MagicMock, PropertyMock, patch
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
        with patch.dict('stretch4_body.utils.tool_metadata.BUILTIN_TOOL_MODELS') as mock_models:
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
    from unittest.mock import MagicMock, patch

    from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello

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

        # Ensure it translated 0.08 m into servo radians using the gripper's tool_metadata
        # conversion. move_to() clamps to command_range first, so clamp here too -- 0.08 is the
        # nominal full-open aperture and sits a hair outside the range the linkage actually
        # reaches, so an unclamped expectation never matches.
        low, high = gripper.tool_metadata.command_range
        expected_rad = gripper.tool_metadata.aperture_to_actuator(min(max(0.08, low), high))
        mock_move_to.assert_called_once_with(gripper, x_des=expected_rad, v_des=None, a_des=None)
    print("ParallelGripper direct move_to test passed!")

if __name__ == "__main__":
    test_conversions()
    test_param_lookup()
    test_robot_joints_properties()
    test_parallel_gripper_direct_commands()
    test_scripts_auto_detect()
    print("All tests passed successfully!")


# ---------------------------------------------------------------------------
# Velocity / differential conversions
#
# A rate does not convert like a position: for y = f(x), y' = f'(x) * x'. Reusing a position
# conversion on a rate adds the map's offset (affine) or uses the wrong gain (nonlinear).
# ---------------------------------------------------------------------------

def _numeric_derivative(f, x, h=1e-6):
    return (f(x + h) - f(x - h)) / (2 * h)


def test_pg4_gain_matches_numeric_derivative():
    """The closed-form slider-crank gain must agree with differentiating the conversion itself."""
    from stretch4_body.utils.tool_metadata import ParallelGripperMetadata
    meta = ParallelGripperMetadata()
    low, high = meta.actuator_range
    for i in range(1, 12):
        at = low + (high - low) * i / 12
        analytic = meta.conversion_gain("actuator", "aperture", at)
        numeric = _numeric_derivative(meta.actuator_to_aperture, at, 1e-5)
        assert math.isclose(analytic, numeric, rel_tol=1e-6), (
            f"at={at}: analytic {analytic} != numeric {numeric}"
        )


def test_pg4_gain_varies_across_range():
    """
    Characterisation guard. PG4's linkage gain swings ~4x from closed to mid-range, so any
    regression to a single constant slope (the thing this whole API exists to prevent) fails here.
    """
    from stretch4_body.utils.tool_metadata import ParallelGripperMetadata
    meta = ParallelGripperMetadata()
    low, high = meta.actuator_range
    gains = [
        abs(meta.conversion_gain("actuator", "aperture", low + (high - low) * i / 10))
        for i in range(11)
    ]
    spread = max(gains) / min(gains)
    print(f"PG4 actuator->aperture gain spread: {spread:.2f}x  (min {min(gains):.5f}, max {max(gains):.5f})")
    assert spread > 3.0, f"expected a strongly position-dependent gain, got {spread:.2f}x"


def test_gain_is_reciprocal_in_both_directions():
    from stretch4_body.utils.tool_metadata import ParallelGripperMetadata
    meta = ParallelGripperMetadata()
    low, high = meta.aperture_range
    for i in range(1, 5):
        aperture = low + (high - low) * i / 5
        fwd = meta.conversion_gain("aperture", "actuator", aperture)
        back = meta.conversion_gain("actuator", "aperture", meta.aperture_to_actuator(aperture))
        assert math.isclose(fwd * back, 1.0, abs_tol=1e-9)


def test_velocity_conversion_drops_the_affine_offset():
    """
    The bug this API exists to prevent, made visible.

    PG4's urdf->command map is affine; its offset is zero only because the shipped
    finger_left_joint limits happen to have upper == 0. Force a non-zero upper limit and a
    position conversion applied to a rate acquires a spurious offset, while the rate conversion
    stays correct.
    """
    from unittest.mock import patch
    from stretch4_body.utils.tool_metadata import ParallelGripperMetadata
    meta = ParallelGripperMetadata()
    # (lower, upper) with a deliberately non-zero upper limit.
    with patch.object(ParallelGripperMetadata, "_finger_joint_limits", (-0.05, 0.01)):
        offset = meta.urdf_to_command(0.0)
        assert not math.isclose(offset, 0.0, abs_tol=1e-9), "test setup failed to create an offset"
        # A zero rate must stay a zero rate, whatever the offset is.
        assert math.isclose(meta.urdf_to_command_velocity(0.0, -0.02), 0.0, abs_tol=1e-12)
        # And a non-zero rate must be pure gain, with no offset added.
        gain = meta.conversion_gain("urdf", "command", -0.02)
        assert math.isclose(meta.urdf_to_command_velocity(2.0, -0.02), 2.0 * gain, rel_tol=1e-12)
        assert not math.isclose(meta.urdf_to_command_velocity(2.0, -0.02),
                                meta.urdf_to_command(2.0), rel_tol=1e-6)


def test_pg4_velocity_sign_flips_urdf_to_command():
    """PG4's URDF finger joint closes as the aperture opens, so the rate gain is negative."""
    from stretch4_body.utils.tool_metadata import ParallelGripperMetadata
    meta = ParallelGripperMetadata()
    assert meta.conversion_gain("urdf", "command", -0.02) < 0.0


def test_convert_delta_is_exact_across_the_linkage():
    """A finite displacement converts exactly; the first-order rate conversion only approximates."""
    from stretch4_body.utils.tool_metadata import ParallelGripperMetadata
    meta = ParallelGripperMetadata()
    at, delta = 0.02, 0.01
    exact = meta.aperture_to_actuator(at + delta) - meta.aperture_to_actuator(at)
    assert math.isclose(meta.convert_delta(delta, "aperture", "actuator", at), exact, rel_tol=1e-15)
    jacobian = meta.convert_velocity(delta, "aperture", "actuator", at)
    assert abs(jacobian - exact) > abs(meta.convert_delta(delta, "aperture", "actuator", at) - exact)


def test_finger_vel_is_the_derivative_of_finger_rad():
    """
    status_to_metadata publishes finger_rad/finger_vel as a ROS JointState position/velocity
    pair, so the velocity must be the time derivative of that exact position expression.
    """
    from stretch4_body.utils.tool_metadata import (
        ParallelGripperMetadata,
        StretchGripperMetadata,
    )
    for meta in (ParallelGripperMetadata(), StretchGripperMetadata()):
        low, high = meta.actuator_range
        for i in range(1, 6):
            at = low + (high - low) * i / 6
            position_of = lambda x: meta.status_to_metadata(
                {"pos": x, "vel": 0.0, "effort": 0.0}
            )["finger_rad"]
            expected = _numeric_derivative(position_of, at)
            reported = meta.status_to_metadata({"pos": at, "vel": 1.0, "effort": 0.0})["finger_vel"]
            assert math.isclose(reported, expected, rel_tol=1e-5), (
                f"{type(meta).__name__} at {at}: finger_vel {reported} != d(finger_rad)/dt {expected}"
            )


def test_same_unit_type_conversions_are_identities():
    """A conversion from a unit type to itself has no `<unit_type>_to_<unit_type>` method to call."""
    from stretch4_body.utils.tool_metadata import (
        ParallelGripperMetadata,
        ToolConfigurationError,
    )
    meta = ParallelGripperMetadata()
    assert meta.conversion_gain("actuator", "actuator", 0.5) == 1.0
    assert meta.convert_velocity(2.5, "urdf", "urdf", 0.0) == 2.5
    assert meta.convert_delta(0.1, "aperture", "aperture", 0.02) == 0.1
    # An unknown unit type must still be rejected, not silently treated as an identity.
    for call in (
        lambda: meta.conversion_gain("bogus", "urdf", 0.0),
        lambda: meta.convert_delta(0.1, "urdf", "bogus", 0.0),
    ):
        try:
            call()
            raise AssertionError("expected ToolConfigurationError for an unknown unit type")
        except ToolConfigurationError:
            pass
