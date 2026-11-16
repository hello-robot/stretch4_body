"""Converting the wrist camera's DepthAI IMU packets into samples: freshness, frames and gravity."""

import datetime
from types import SimpleNamespace

import numpy as np
import pytest

from stretch4_body.subsystem.cameras.adapters.luxonis_imu import (
    STANDARD_GRAVITY,
    ImuPacketConverter,
    gravity_in_sensor_frame,
    has_sensor_fusion,
    is_rotation_matrix,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
)

# Camera optical frame (x right, y down, z forward) from an IMU whose x is the camera's z, y is the
# camera's -x and z is the camera's -y (forward-left-up).
R_CAMERA_FROM_FLU = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
])


def report(seconds: float, sequence: int = 0, **fields):
    """A stand-in for a DepthAI IMU report. seconds == 0 is a report the IMU never filled in."""
    return SimpleNamespace(
        sequence=sequence,
        getTimestamp=lambda: datetime.timedelta(seconds=seconds),
        **fields,
    )


def packet(t: float, sequence: int, acceleration=(0.0, 0.0, STANDARD_GRAVITY), gyro=(0.0, 0.0, 0.0),
           quaternion_xyzw=(0.0, 0.0, 0.0, 1.0), gyro_t: float | None = None, rotation_t: float | None = None):
    x, y, z = acceleration
    gx, gy, gz = gyro
    qx, qy, qz, qw = quaternion_xyzw
    return SimpleNamespace(
        acceleroMeter=report(t, sequence, x=x, y=y, z=z),
        gyroscope=report(t if gyro_t is None else gyro_t, sequence, x=gx, y=gy, z=gz),
        rotationVector=report(t if rotation_t is None else rotation_t, sequence, i=qx, j=qy, k=qz, real=qw),
        magneticField=report(0.0),
    )


def test_level_and_at_rest_has_no_linear_acceleration():
    sample = ImuPacketConverter().convert(packet(10.0, 1))
    assert sample.timestamp == 10.0
    assert sample.sequence == 1
    np.testing.assert_allclose(sample.acceleration, [0, 0, STANDARD_GRAVITY])
    np.testing.assert_allclose(sample.linear_acceleration, [0, 0, 0], atol=1e-9)
    np.testing.assert_allclose(sample.orientation_xyzw, [0, 0, 0, 1], atol=1e-9)


def test_tilted_at_rest_has_no_linear_acceleration():
    # Rolled 90 deg about x: the sensor's y axis points up, so it reads +g on y.
    half = np.sqrt(0.5)
    q_roll_90 = (half, 0.0, 0.0, half)
    np.testing.assert_allclose(gravity_in_sensor_frame(np.array(q_roll_90)), [0, STANDARD_GRAVITY, 0], atol=1e-9)

    sample = ImuPacketConverter().convert(packet(1.0, 1, acceleration=(0.0, STANDARD_GRAVITY, 0.0), quaternion_xyzw=q_roll_90))
    np.testing.assert_allclose(sample.linear_acceleration, [0, 0, 0], atol=1e-9)


def test_a_push_shows_in_linear_acceleration():
    sample = ImuPacketConverter().convert(packet(1.0, 1, acceleration=(3.0, 0.0, STANDARD_GRAVITY)))
    np.testing.assert_allclose(sample.linear_acceleration, [3.0, 0, 0], atol=1e-9)


def test_vectors_are_rotated_into_the_camera_frame():
    converter = ImuPacketConverter(R_CAMERA_FROM_FLU)
    # IMU level, pushed forward (its +x) and yawing left (its +z).
    sample = converter.convert(packet(1.0, 1, acceleration=(2.0, 0.0, STANDARD_GRAVITY), gyro=(0.0, 0.0, 0.5)))

    # Up is the camera's -y, forward its +z.
    np.testing.assert_allclose(sample.acceleration, [0.0, -STANDARD_GRAVITY, 2.0], atol=1e-9)
    np.testing.assert_allclose(sample.linear_acceleration, [0.0, 0.0, 2.0], atol=1e-9)
    np.testing.assert_allclose(sample.angular_velocity, [0.0, -0.5, 0.0], atol=1e-9)

    # The camera's orientation maps its own -y (up) onto the world's +z.
    R_world_from_camera = quaternion_xyzw_to_matrix(sample.orientation_xyzw)
    np.testing.assert_allclose(R_world_from_camera @ [0.0, -1.0, 0.0], [0, 0, 1], atol=1e-9)


def test_repeated_accelerometer_report_is_not_a_new_sample():
    converter = ImuPacketConverter()
    assert converter.convert(packet(1.0, 1)) is not None
    assert converter.convert(packet(1.0, 1)) is None
    assert converter.convert(packet(1.01, 2)) is not None
    assert converter.lost_samples == 0


def test_lost_samples_are_counted():
    converter = ImuPacketConverter()
    converter.convert(packet(1.00, 1))
    converter.convert(packet(1.04, 5))
    assert converter.lost_samples == 3


def test_missing_reports():
    converter = ImuPacketConverter()
    # No accelerometer report yet: nothing to publish.
    assert converter.convert(packet(0.0, 0, gyro_t=1.0, rotation_t=1.0)) is None

    # The gyroscope and rotation vector have not reported yet: those fields are unknown.
    sample = ImuPacketConverter().convert(packet(1.0, 1, gyro_t=0.0, rotation_t=0.0))
    assert sample.angular_velocity is None
    assert sample.orientation_xyzw is None
    assert sample.linear_acceleration is None


def test_latest_gyroscope_is_held_between_its_reports():
    converter = ImuPacketConverter()
    converter.convert(packet(1.00, 1, gyro=(0.1, 0.2, 0.3)))
    sample = converter.convert(packet(1.01, 2, gyro=(9.0, 9.0, 9.0), gyro_t=0.0))
    np.testing.assert_allclose(sample.angular_velocity, [0.1, 0.2, 0.3])


def test_without_fusion_there_is_no_orientation():
    sample = ImuPacketConverter(has_orientation=False).convert(packet(1.0, 1))
    assert sample.orientation_xyzw is None
    assert sample.linear_acceleration is None
    assert sample.angular_velocity is not None


@pytest.mark.parametrize("imu_type, fused", [("BNO086", True), ("BNO085", True), ("BMI270", False), ("NONE", False)])
def test_sensor_fusion_by_imu_type(imu_type, fused):
    assert has_sensor_fusion(imu_type) == fused


def test_quaternion_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        q = -q if q[3] < 0 else q
        R = quaternion_xyzw_to_matrix(q)
        assert is_rotation_matrix(R)
        np.testing.assert_allclose(matrix_to_quaternion_xyzw(R), q, atol=1e-9)


def test_rotation_matrix_check():
    assert is_rotation_matrix(R_CAMERA_FROM_FLU)
    assert not is_rotation_matrix(np.zeros((3, 3)))
    assert not is_rotation_matrix(-np.eye(3))  # a reflection
