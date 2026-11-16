"""
The IMU inside a Luxonis camera, read through that camera's DepthAI pipeline.

Stretch's wrist camera (OAK-D-SR) carries one. The IMU is part of the camera device, so it can only be
read by the process that owns the camera: `GripperCameraLuxonis` adds it to its pipeline when its
configuration asks for it (`RGBCameraConfig.imu_rate_hz`), and exposes it as `.imu`.

What it reports, per accelerometer sample (`ImuSample`):
  - acceleration, gravity included (m/s^2), and angular velocity (rad/s);
  - on IMUs with on-chip fusion (BNO08x): orientation and gravity-free linear acceleration.
All vectors are rotated into the camera's optical frame (x right, y down, z forward), using the IMU
extrinsics stored on the camera.

The accelerometer can only report one of ACCELEROMETER, LINEAR_ACCELERATION or GRAVITY at a time
(DepthAI's IMUPacket has a single accelerometer field), so the IMU is asked for the accelerometer with
gravity, plus the game rotation vector, and linear acceleration is computed here: the measured
acceleration minus gravity as seen through the fused orientation. That is what the chip's own
LINEAR_ACCELERATION report computes as well.
"""

import datetime
import logging
import math
from dataclasses import dataclass

import depthai as dai
import numpy as np

logger = logging.getLogger(__name__)

STANDARD_GRAVITY = 9.80665
"""m/s^2"""

DEFAULT_IMU_RATE_HZ = 100

IMU_QUEUE_SIZE = 64
"""IMU messages held on the host before the oldest is dropped: ~0.6 s at 100 Hz. Non-blocking, so a
slow reader loses samples instead of stalling the camera."""

IMU_MAX_BATCH_REPORTS = 10
"""At most this many samples per message. The device sends as soon as it has one (batch threshold 1),
so this only matters when the link falls behind: samples are then sent together instead of dropped."""

NO_IMU_NAMES = ("", "NONE")
"""What `dai.Device.getConnectedIMU()` returns when the device has no IMU."""


@dataclass
class ImuSample:
    """One accelerometer sample, with the latest gyroscope and orientation reports at that time.

    Vectors are in the camera's optical frame (x right, y down, z forward).
    """

    timestamp: float
    """Seconds on the host's monotonic clock (DepthAI's steady clock, synced to the device clock by
    DepthAI), when the accelerometer sample was taken."""

    sequence: int
    """The accelerometer report's sequence number. Gaps mean samples were lost."""

    acceleration: np.ndarray
    """m/s^2, gravity included: ~9.8 upward at rest."""

    angular_velocity: np.ndarray | None
    """rad/s. None until the gyroscope has reported."""

    orientation_xyzw: np.ndarray | None
    """The camera frame's orientation in a gravity-aligned frame (z up) whose heading is arbitrary and
    drifts (the game rotation vector, no magnetometer). None without on-chip fusion, or until it reports."""

    linear_acceleration: np.ndarray | None
    """m/s^2, gravity removed: ~0 at rest. None whenever `orientation_xyzw` is."""


def has_sensor_fusion(imu_type: str) -> bool:
    """Whether the IMU fuses orientation on-chip: the BNO08x does, the BMI270 does not."""
    return imu_type.upper().startswith("BNO")


def quaternion_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    """The rotation matrix of a quaternion (x, y, z, w), normalized first."""
    x, y, z, w = np.asarray(q, dtype=np.float64) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quaternion_xyzw(R: np.ndarray) -> np.ndarray:
    """The quaternion (x, y, z, w, with w >= 0) of a rotation matrix."""
    trace = np.trace(R)
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def gravity_in_sensor_frame(orientation_xyzw: np.ndarray) -> np.ndarray:
    """What an accelerometer at rest reads, in the sensor frame: gravity's reaction, STANDARD_GRAVITY
    along the world's +z (up).

    `orientation_xyzw` takes sensor-frame vectors into a world frame with z up, which is how the
    BNO08x reports its rotation vectors (ENU, or z-up with arbitrary heading for the game rotation vector).
    """
    R_world_from_sensor = quaternion_xyzw_to_matrix(orientation_xyzw)
    return R_world_from_sensor.T @ np.array([0.0, 0.0, STANDARD_GRAVITY])


def is_rotation_matrix(R: np.ndarray, tolerance: float = 1e-3) -> bool:
    R = np.asarray(R, dtype=np.float64)
    return R.shape == (3, 3) and np.allclose(R @ R.T, np.eye(3), atol=tolerance) and abs(np.linalg.det(R) - 1.0) < tolerance


def imu_to_camera_rotation(calibration: dai.CalibrationHandler, camera_socket: dai.CameraBoardSocket) -> np.ndarray | None:
    """The rotation that takes IMU-frame vectors into the camera's optical frame, from the IMU
    extrinsics on the camera's EEPROM. None when the EEPROM has none, or not a valid rotation."""
    try:
        transform = np.array(calibration.getImuToCameraExtrinsics(camera_socket), dtype=np.float64)
    except Exception as e:
        logger.debug(f"No IMU extrinsics on the camera: {e}")
        return None

    R = transform[:3, :3] if transform.shape == (4, 4) else None
    if R is None or not is_rotation_matrix(R):
        return None
    return R


def _report_seconds(report) -> float:
    """A report's host-synced timestamp in seconds; 0 for a report the IMU never filled in."""
    return report.getTimestamp().total_seconds()


class ImuPacketConverter:
    """Turns DepthAI IMUPackets into ImuSamples, one per new accelerometer report.

    Sensors report at their own times, and a packet carries the latest report of each, so a report is
    only taken as new when its sequence number or timestamp changed. Gyroscope and orientation reports
    are held and attached to the next accelerometer sample.
    """

    def __init__(self, R_camera_from_imu: np.ndarray | None = None, has_orientation: bool = True):
        self.R_camera_from_imu = np.eye(3) if R_camera_from_imu is None else np.asarray(R_camera_from_imu, dtype=np.float64)
        self.has_orientation = has_orientation

        self.last_acceleration_key: tuple[int, float] | None = None
        self.angular_velocity: np.ndarray | None = None
        self.orientation_xyzw_imu: np.ndarray | None = None

        self.lost_samples = 0
        """Accelerometer samples missing from the sequence so far."""

    def convert(self, packet) -> ImuSample | None:
        gyroscope = packet.gyroscope
        if _report_seconds(gyroscope) > 0:
            self.angular_velocity = self.R_camera_from_imu @ np.array([gyroscope.x, gyroscope.y, gyroscope.z], dtype=np.float64)

        if self.has_orientation:
            rotation = packet.rotationVector
            if _report_seconds(rotation) > 0:
                q = np.array([rotation.i, rotation.j, rotation.k, rotation.real], dtype=np.float64)
                if np.linalg.norm(q) > 0.5:
                    self.orientation_xyzw_imu = q

        accelerometer = packet.acceleroMeter
        timestamp = _report_seconds(accelerometer)
        if timestamp <= 0:
            return None
        key = (int(accelerometer.sequence), timestamp)
        if key == self.last_acceleration_key:
            return None
        if self.last_acceleration_key is not None:
            gap = key[0] - self.last_acceleration_key[0] - 1
            if 0 < gap < 10000:
                self.lost_samples += gap
        self.last_acceleration_key = key

        acceleration_imu = np.array([accelerometer.x, accelerometer.y, accelerometer.z], dtype=np.float64)

        orientation_xyzw = None
        linear_acceleration = None
        if self.orientation_xyzw_imu is not None:
            linear_acceleration = self.R_camera_from_imu @ (acceleration_imu - gravity_in_sensor_frame(self.orientation_xyzw_imu))
            # World-from-camera = world-from-IMU @ IMU-from-camera.
            R_world_from_camera = quaternion_xyzw_to_matrix(self.orientation_xyzw_imu) @ self.R_camera_from_imu.T
            orientation_xyzw = matrix_to_quaternion_xyzw(R_world_from_camera)

        return ImuSample(
            timestamp=timestamp,
            sequence=key[0],
            acceleration=self.R_camera_from_imu @ acceleration_imu,
            angular_velocity=self.angular_velocity,
            orientation_xyzw=orientation_xyzw,
            linear_acceleration=linear_acceleration,
        )


class LuxonisImu:
    """A Luxonis device's IMU, added to its pipeline before the pipeline starts.

    Use `try_create()`, which returns None (after one warning) when the device has no IMU, so the
    camera streams on without it.
    """

    def __init__(self, pipeline: dai.Pipeline, device: dai.Device, rate_hz: int, camera_socket: dai.CameraBoardSocket):
        self.imu_type = device.getConnectedIMU()
        if self.imu_type.upper() in NO_IMU_NAMES:
            raise RuntimeError(f"the camera reports no IMU ({self.imu_type!r}).")
        self.rate_hz = int(rate_hz)
        self.has_orientation = has_sensor_fusion(self.imu_type)

        sensors = [dai.IMUSensor.ACCELEROMETER, dai.IMUSensor.GYROSCOPE_CALIBRATED]
        if self.has_orientation:
            sensors.append(dai.IMUSensor.GAME_ROTATION_VECTOR)

        imu = pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(sensors, self.rate_hz)
        # Send every sample as soon as it exists: latency over throughput.
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(IMU_MAX_BATCH_REPORTS)
        self.queue = imu.out.createOutputQueue(maxSize=IMU_QUEUE_SIZE, blocking=False)

        self.camera_socket = camera_socket
        self.R_camera_from_imu = imu_to_camera_rotation(device.readCalibration(), camera_socket)
        """From the camera's EEPROM. None when it has none: the IMU axes are then taken as the camera's."""
        if self.R_camera_from_imu is None:
            logger.warning(
                f"No IMU extrinsics on the camera; taking the {self.imu_type}'s axes as the {camera_socket.name} "
                f"camera's optical axes (x right, y down, z forward), unrotated."
            )
        self.converter = ImuPacketConverter(self.R_camera_from_imu, has_orientation=self.has_orientation)
        logger.info(self.describe())

    def describe(self) -> str:
        """What is read, at what rate, and how it is rotated into the camera frame."""
        reports = "acceleration, angular velocity"
        reports += (", orientation and linear acceleration" if self.has_orientation
                    else " (no on-chip fusion: no orientation or linear acceleration)")
        if self.R_camera_from_imu is None:
            rotation = "IMU axes taken as the camera's (no IMU extrinsics on the EEPROM)"
        else:
            rotation = f"rotated by the EEPROM's IMU extrinsics, rows {self.R_camera_from_imu.round(3).tolist()}"
        frame = f"the {self.camera_socket.name} camera's optical frame"
        return f"{self.imu_type} at {self.rate_hz} Hz: {reports}; into {frame}, {rotation}."

    @staticmethod
    def try_create(pipeline: dai.Pipeline, device: dai.Device, rate_hz: int,
                   camera_socket: dai.CameraBoardSocket) -> "LuxonisImu | None":
        """The device's IMU, or None if `rate_hz` is 0 or the device has none (warns once)."""
        if rate_hz <= 0:
            return None
        try:
            return LuxonisImu(pipeline, device, rate_hz, camera_socket)
        except Exception as e:
            logger.warning(f"Not reading the camera's IMU, streaming images without it: {e}")
            return None

    @property
    def lost_samples(self) -> int:
        """Accelerometer samples missing from the sequence so far."""
        return self.converter.lost_samples

    def get_samples(self, timeout_seconds: float = 0.1) -> list[ImuSample]:
        """Every sample that arrived since the last call. Waits up to `timeout_seconds` when there are none."""
        messages = []
        message = self.queue.get(datetime.timedelta(seconds=timeout_seconds))
        if message is not None:
            messages.append(message)
            messages.extend(self.queue.tryGetAll())

        samples = []
        for message in messages:
            if message is None:
                continue
            for packet in message.packets:
                sample = self.converter.convert(packet)
                if sample is not None:
                    samples.append(sample)
        return samples
