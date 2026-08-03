#!/usr/bin/env python3
"""
Lidar-Lidar Comparison Script for Stretch 4.

Compares extrinsics and measurement consistency between left and right lidars
using the reflective virtual rectangle target on the calibration tool.

Replays the same validation poses as `camera_intrinsics_validate_l2_distance.py`,
detects the virtual rectangle corners and center for both lidars, transforms them to `base_link`,
matches corresponding corners, computes L2 errors, and exports a detailed YAML report.
"""

import argparse
import time
import sys
import os
import yaml
import datetime
import threading
import queue
import numpy as np
from scipy.optimize import linear_sum_assignment

from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.subsystem.cameras.models.dual_lidar_calibration import DualLidarCalibration
from stretch4_body.subsystem.cameras.detectors.detector_frame_settled import DetectFrameSettled
from stretch4_body.subsystem.cameras.calibrate_extrinsics_lidars import (
    get_high_intensity_points,
    detect_lidar_rectangle,
    evaluate_lidar_rectangle,
)
from stretch4_body.subsystem.cameras.models.camera_calibration import DEFAULT_CALIBRATION_FOLDER_PATH

import rerun as rr
import rerun.blueprint as rrb


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Lidar-Lidar Virtual Rectangle Comparison Script for Stretch 4."
    )
    parser.add_argument(
        "--skip_user_prompt",
        action="store_true",
        help="Skip confirmation prompt before moving robot.",
    )
    parser.add_argument(
        "--not_interactive",
        action="store_true",
        help="Disable Rerun GUI visualization.",
    )
    parser.add_argument(
        "--use_ros_for_lidars",
        action="store_true",
        help="Use ROS 2 python bridge to subscribe to lidar streams.",
    )
    parser.add_argument(
        "--expected_width",
        type=float,
        default=280.72 / 1000.0,
        help="Expected width of virtual rectangle in meters.",
    )
    parser.add_argument(
        "--expected_height",
        type=float,
        default=208.33 / 1000.0,
        help="Expected height of virtual rectangle in meters.",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=15.0 / 1000.0,
        help="Tolerance for rectangle dimension validation in meters.",
    )
    parser.add_argument(
        "--output_file",
        type=str,
        default=None,
        help="Path to save output YAML file. Default is inside calibration_dual_lidar folder.",
    )
    parser.add_argument(
        "--manual",
        action="store_true",
        help="Enable manual mode to press Enter to capture, displaying joint positions on capture.",
    )
    return parser.parse_known_args()[0]


def get_angular_sort_indices(centroids_base: np.ndarray) -> np.ndarray:
    """Sorts 4 rectangle corner points deterministically by angle in principal plane."""
    center = np.mean(centroids_base, axis=0)
    centered = centroids_base - center
    cov = centered.T @ centered
    direction_principal_components, _, _ = np.linalg.svd(cov)
    coords_2d = np.array(
        [[np.dot(p, direction_principal_components[:, 0]), np.dot(p, direction_principal_components[:, 1])] for p in centered]
    )
    angles = np.arctan2(coords_2d[:, 1], coords_2d[:, 0])
    return np.argsort(angles)


class ThreadedStreamWrapper:
    def __init__(self, stream):
        self.stream = stream
        self.queue = queue.Queue(maxsize=100)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                frame = next(self.stream)
                if frame is None:
                    time.sleep(0.01)
                    continue
                try:
                    self.queue.put(frame, timeout=0.1)
                except queue.Full:
                    # Discard oldest frame to keep the queue fresh and bounded
                    try:
                        self.queue.get_nowait()
                    except queue.Empty:
                        pass
                    try:
                        self.queue.put_nowait(frame)
                    except queue.Full:
                        pass
            except StopIteration:
                break
            except Exception as e:
                print(f"Error in stream thread: {e}")
                break

    def __next__(self):
        while not self.stop_event.is_set():
            try:
                return self.queue.get(timeout=0.1)
            except queue.Empty:
                continue
        raise StopIteration()

    def __iter__(self):
        return self

    def close(self):
        self.stop_event.set()
        # Clean up queue to unblock any putting thread
        try:
            while not self.queue.empty():
                self.queue.get_nowait()
        except:
            pass
        if hasattr(self.stream, 'close'):
            try:
                self.stream.close()
            except:
                pass


class LidarLidarCompare:
    def __init__(
        self,
        use_ros_for_lidars: bool = False,
        expected_width: float = 0.28072,
        expected_height: float = 0.20833,
        tolerance: float = 0.015,
        output_file: str | None = None,
        manual: bool = False,
    ):
        self.use_ros_for_lidars = use_ros_for_lidars
        self.expected_width = expected_width
        self.expected_height = expected_height
        self.tolerance = tolerance
        self.output_file = output_file
        self.manual = manual
        self.stop_event = threading.Event()

        self.poses = [
            {'lift': 0.7422438515712114, 'arm': 0.49702133496485, 'wrist_yaw': 0.37045636027438233, 'wrist_pitch': -0.8736020587008726, 'wrist_roll': -0.08513593372765309},
            {'lift': 0.9462642491572507, 'arm': 0.49703237617656476, 'wrist_yaw': 0.37045636027438233, 'wrist_pitch': -0.8736020587008726, 'wrist_roll': -0.08513593372765309},
            {'lift': 1.1516953266937076, 'arm': 0.5463595285612547, 'wrist_yaw': 0.21782527187976106, 'wrist_pitch': -1.0569127628532067, 'wrist_roll': -0.08743690490948154},
            {'lift': 1.0651885827001875, 'arm': 0.4368502999959916, 'wrist_yaw': 3.748282055198564, 'wrist_pitch': 3.425379099348637, 'wrist_roll': 3.14082566319585},
            {'lift': 0.4317325114572012, 'arm': 0.43669278614402063, 'wrist_yaw': 3.748282055198564, 'wrist_pitch': 3.425379099348637, 'wrist_roll': 3.14082566319585},
            {'lift': 0.4317325114572012, 'arm': 0.11970237670445559, 'wrist_yaw': 3.948466548017641, 'wrist_pitch': 2.941408160770717, 'wrist_roll': 3.14082566319585},
            {'lift': 0.4317587575974519, 'arm': 0.11961223469063292, 'wrist_yaw': 1.9412526870692788, 'wrist_pitch': 2.655320743830045, 'wrist_roll': 3.14082566319585},
            {'lift': 0.23195447186152143, 'arm': 0.11961580356714678, 'wrist_yaw': 2.6039323874358757, 'wrist_pitch': 2.6177382145268466, 'wrist_roll': 3.14082566319585},
            {'lift': 0.2320348297960165, 'arm': 0.11961937244366064, 'wrist_yaw': 3.4023693875303525, 'wrist_pitch': 2.7504275526789543, 'wrist_roll': 3.14082566319585},
            {'lift': 0.23206843888019246, 'arm': 0.11961937244366064, 'wrist_yaw': 4.273670475049396, 'wrist_pitch': 2.8148547457701514, 'wrist_roll': 3.142359643983736},
            {'lift': 0.2320818791746592, 'arm': 0.1196266867817242, 'wrist_yaw': 0.8099418560036186, 'wrist_pitch': 2.6384469551633027, 'wrist_roll': 3.14082566319585},
            {'lift': 0.01253318833188502, 'arm': 0.3393704530060471, 'wrist_yaw': 3.4085053106818943, 'wrist_pitch': 3.228262568105332, 'wrist_roll': 3.14082566319585},
            {'lift': 0.22383738550050916, 'arm': 0.5546602520471264, 'wrist_yaw': 3.557301447106802, 'wrist_pitch': 3.0633596334076256, 'wrist_roll': 3.141592653589793},
        ]

        # Robot startup
        print("Starting robot client, please wait...", flush=True)
        self.robot = RobotClient()
        if not self.robot.startup():
            print("Failed to start robot client.", flush=True)
            sys.exit(1)

        if self.robot.params.get('tool') != 'eoa_wrist_dw4_tool_calibration':
            print("WARNING: This script is intended to be run with the 'eoa_wrist_dw4_tool_calibration' tool.")
            print("Make sure your tool parameter in your robot geometry is correctly set.")

        # Load Dual Lidar Calibration transforms
        self.dual_lidar_calib = DualLidarCalibration()
        self.T_base_from_left_calibrated = self.dual_lidar_calib.get_lidar_to_base_transform(is_right_lidar=False, apply_calibration=True)
        self.T_base_from_right_calibrated = self.dual_lidar_calib.get_lidar_to_base_transform(is_right_lidar=True, apply_calibration=True)
        self.T_base_from_left_uncalibrated = self.dual_lidar_calib.get_lidar_to_base_transform(is_right_lidar=False, apply_calibration=False)
        self.T_base_from_right_uncalibrated = self.dual_lidar_calib.get_lidar_to_base_transform(is_right_lidar=True, apply_calibration=False)
        # Keep old aliases for backward compatibility
        self.T_base_from_left = self.T_base_from_left_uncalibrated
        self.T_base_from_right = self.T_base_from_right_uncalibrated

        # Start lidar streams
        print("Connecting to Lidar Streams...", flush=True)
        self.left_stream = None
        self.right_stream = None
        self._init_lidar_streams()
        print("Lidar streams connected.", flush=True)

    def _init_lidar_streams(self):
        if self.use_ros_for_lidars:
            try:
                from stretch_python_bridge import stream_lidar_left, stream_lidar_right, StreamManager
                self.stream_manager = StreamManager()
                self.left_stream = stream_lidar_left(stream_manager=self.stream_manager)
                self.right_stream = stream_lidar_right(stream_manager=self.stream_manager)

                def _spin_ros():
                    for _ in self.stream_manager.stream():
                        if self.stop_event.is_set():
                            break

                self.ros_thread = threading.Thread(target=_spin_ros, daemon=True)
                self.ros_thread.start()
            except ImportError:
                raise ImportError("stretch_python_bridge not found. Did you colcon build? Please source ROS 2 workspace.")
        else:
            try:
                from pyhesai_wrapper import stream_lidar_left, stream_lidar_right
                self.left_stream = ThreadedStreamWrapper(stream_lidar_left())
                self.right_stream = ThreadedStreamWrapper(stream_lidar_right())
            except ImportError:
                raise ImportError("pyhesai_wrapper not found. Please install it or pass `--use_ros_for_lidars`.")

    def cleanup(self):
        print("Cleaning up LidarLidarCompare...")
        self.stop_event.set()
        if hasattr(self, 'left_stream') and self.left_stream is not None:
            try:
                self.left_stream.close()
            except Exception as e:
                print(f"Warning: error closing left_stream: {e}")

        if hasattr(self, 'right_stream') and self.right_stream is not None:
            try:
                self.right_stream.close()
            except Exception as e:
                print(f"Warning: error closing right_stream: {e}")

        if hasattr(self, 'robot') and self.robot is not None:
            try:
                self.robot.stop()
            except Exception as e:
                print(f"Warning: error stopping robot: {e}")

    def setup_rerun_blueprint(self):
        rr.spawn()
        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    name="Calibrated View",
                    origin="calibrated/base_link",
                ),
                rrb.Spatial3DView(
                    name="Uncalibrated View",
                    origin="uncalibrated/base_link",
                ),
                rrb.TextDocumentView(name="Comparison Status", origin="Validation/Instructions"),
                column_shares=[2, 2, 1],
            ),
            rrb.BlueprintPanel(state="collapsed"),
            rrb.SelectionPanel(state="collapsed"),
        )
        try:
            rr.send_blueprint(blueprint)
        except Exception as e:
            print(f"Failed to send rerun blueprint: {e}")

    def update_instructions(self, message: str):
        rr.log("Validation/Instructions", rr.TextDocument(message, media_type=rr.MediaType.MARKDOWN))

    def move_to_pose(self, pose):
        print(f"Moving to pose: {pose}")
        self.robot.lift.move_to(pose['lift'])
        self.robot.arm.move_to(pose['arm'])
        self.robot.end_of_arm.move_to('wrist_pitch', pose['wrist_pitch'])
        self.robot.end_of_arm.move_to('wrist_roll', pose.get('wrist_roll', 0.0))
        self.robot.end_of_arm.move_to('wrist_yaw', pose.get('wrist_yaw', 0.0))
        self.robot.push_command()
        self.robot.wait_command()
        time.sleep(2.5)  # Allow robot and lidars to settle

    def flush_lidar_streams(self, max_discard=150):
        """Flushes stale buffered frames from both left and right lidar streams to get fresh, settled data."""
        settled_time = time.time()
        print("Flushing stale lidar frames...")
        
        # Flush left stream
        left_flushed = 0
        while left_flushed < max_discard:
            t0 = time.perf_counter()
            frame = next(self.left_stream)
            dt = time.perf_counter() - t0
            left_flushed += 1
            if frame is None:
                break
            
            # Check system timestamp for freshness
            ts = getattr(frame, 'timestamp_system', None)
            if ts is not None:
                # Discard frames older than settled_time
                if ts >= settled_time - 0.05:
                    break
            
            # If retrieving took more than 15ms, it blocked waiting for a fresh physical frame
            if dt > 0.015:
                break
                
        # Flush right stream
        right_flushed = 0
        while right_flushed < max_discard:
            t0 = time.perf_counter()
            frame = next(self.right_stream)
            dt = time.perf_counter() - t0
            right_flushed += 1
            if frame is None:
                break
                
            # Check system timestamp for freshness
            ts = getattr(frame, 'timestamp_system', None)
            if ts is not None:
                # Discard frames older than settled_time
                if ts >= settled_time - 0.05:
                    break
                    
            # If retrieving took more than 15ms, it blocked waiting for a fresh physical frame
            if dt > 0.015:
                break
                
        print(f"Flushed {left_flushed} left frames and {right_flushed} right frames.")

    def _preview_loop(self):
        while not self.stop_event.is_set():
            self.preview_active.wait(timeout=0.1)
            if not self.preview_active.is_set():
                continue

            try:
                # Pull robot status
                self.robot.pull_status()
                current_lift = self.robot.lift.status['pos']
                current_arm = self.robot.arm.status['pos']
                current_wrist_yaw = self.robot.end_of_arm.status['wrist_yaw']['pos']
                current_wrist_pitch = self.robot.end_of_arm.status['wrist_pitch']['pos']
                current_wrist_roll = self.robot.end_of_arm.status['wrist_roll']['pos']

                # Capture a single frame pair (returns dict of calibrated and uncalibrated data)
                sample = self.capture_single_frame_pair()

                if sample is not None:
                    # Calibrated errors
                    left_corners_cal = sample['calibrated']['left_base_corners']
                    right_corners_cal = sample['calibrated']['right_base_corners']
                    left_center_cal = sample['calibrated']['left_base_center']
                    right_center_cal = sample['calibrated']['right_base_center']

                    corner_errors_cal = [float(np.linalg.norm(left_corners_cal[k] - right_corners_cal[k])) for k in range(4)]
                    mean_corner_error_cal = float(np.mean(corner_errors_cal))
                    center_error_cal = float(np.linalg.norm(left_center_cal - right_center_cal))

                    # Uncalibrated errors
                    left_corners_uncal = sample['uncalibrated']['left_base_corners']
                    right_corners_uncal = sample['uncalibrated']['right_base_corners']
                    left_center_uncal = sample['uncalibrated']['left_base_center']
                    right_center_uncal = sample['uncalibrated']['right_base_center']

                    corner_errors_uncal = [float(np.linalg.norm(left_corners_uncal[k] - right_corners_uncal[k])) for k in range(4)]
                    mean_corner_error_uncal = float(np.mean(corner_errors_uncal))
                    center_error_uncal = float(np.linalg.norm(left_center_uncal - right_center_uncal))

                    # Log calibrated detections
                    rr.log(
                        "calibrated/base_link/left_corners",
                        rr.Points3D(left_corners_cal, colors=[255, 0, 0], radii=[0.008], labels=[f"L_C{k+1}" for k in range(4)]),
                    )
                    rr.log(
                        "calibrated/base_link/right_corners",
                        rr.Points3D(right_corners_cal, colors=[0, 255, 0], radii=[0.008], labels=[f"R_C{k+1}" for k in range(4)]),
                    )
                    rr.log(
                        "calibrated/base_link/left_center",
                        rr.Points3D([left_center_cal], colors=[255, 100, 100], radii=[0.012], labels=["L_Center"]),
                    )
                    rr.log(
                        "calibrated/base_link/right_center",
                        rr.Points3D([right_center_cal], colors=[100, 255, 100], radii=[0.012], labels=["R_Center"]),
                    )

                    # Log uncalibrated detections
                    rr.log(
                        "uncalibrated/base_link/left_corners",
                        rr.Points3D(left_corners_uncal, colors=[255, 0, 0], radii=[0.008], labels=[f"L_C{k+1}" for k in range(4)]),
                    )
                    rr.log(
                        "uncalibrated/base_link/right_corners",
                        rr.Points3D(right_corners_uncal, colors=[0, 255, 0], radii=[0.008], labels=[f"R_C{k+1}" for k in range(4)]),
                    )
                    rr.log(
                        "uncalibrated/base_link/left_center",
                        rr.Points3D([left_center_uncal], colors=[255, 100, 100], radii=[0.012], labels=["L_Center"]),
                    )
                    rr.log(
                        "uncalibrated/base_link/right_center",
                        rr.Points3D([right_center_uncal], colors=[100, 255, 100], radii=[0.012], labels=["R_Center"]),
                    )

                    live_instructions = f"""
# Lidar-Lidar Live Preview (Manual Mode)

### Robot Pose
- **Lift**: {current_lift:.5f} m
- **Arm**: {current_arm:.5f} m
- **Wrist Yaw**: {current_wrist_yaw:.5f} rad
- **Wrist Pitch**: {current_wrist_pitch:.5f} rad
- **Wrist Roll**: {current_wrist_roll:.5f} rad

### Real-time Calibrated Errors (apply_calibration=True)
- **Corner Errors**: {", ".join([f"{e*1000.0:.1f}" for e in corner_errors_cal])} mm
- **Mean Corner Error**: **{mean_corner_error_cal*1000.0:.2f} mm**
- **Center Error**: **{center_error_cal*1000.0:.2f} mm**

### Real-time Uncalibrated Errors (apply_calibration=False)
- **Corner Errors**: {", ".join([f"{e*1000.0:.1f}" for e in corner_errors_uncal])} mm
- **Mean Corner Error**: **{mean_corner_error_uncal*1000.0:.2f} mm**
- **Center Error**: **{center_error_uncal*1000.0:.2f} mm**

> [!TIP]
> Adjust the physical robot joints to minimize the errors shown above. 
> Press **[Enter]** in the terminal to save this pose's averaged calibration measurements.
"""
                    self.update_instructions(live_instructions)
                    print(f"\r[LIVE PREVIEW] Lift: {current_lift:.3f}m | Arm: {current_arm:.3f}m | Yaw: {current_wrist_yaw:.3f}rad | Pitch: {current_wrist_pitch:.3f}rad | Roll: {current_wrist_roll:.3f}rad | Calibrated Mean Corner Err: {mean_corner_error_cal*1000.0:.2f}mm | Uncalibrated Mean Corner Err: {mean_corner_error_uncal*1000.0:.2f}mm   ", end="", flush=True)
                else:
                    live_instructions = f"""
# Lidar-Lidar Live Preview (Manual Mode)

### Robot Pose
- **Lift**: {current_lift:.5f} m
- **Arm**: {current_arm:.5f} m
- **Wrist Yaw**: {current_wrist_yaw:.5f} rad
- **Wrist Pitch**: {current_wrist_pitch:.5f} rad
- **Wrist Roll**: {current_wrist_roll:.5f} rad

### Real-time Calibration Errors
- **Target Status**: **Virtual rectangle NOT detected on both lidars**

> [!WARNING]
> Ensure both lidars have a clear line of sight to the calibration target.
"""
                    self.update_instructions(live_instructions)
                    print(f"\r[LIVE PREVIEW] Lift: {current_lift:.3f}m | Arm: {current_arm:.3f}m | Yaw: {current_wrist_yaw:.3f}rad | Pitch: {current_wrist_pitch:.3f}rad | Roll: {current_wrist_roll:.3f}rad | Target Status: NOT DETECTED   ", end="", flush=True)

                time.sleep(0.05)
            except Exception as e:
                time.sleep(0.1)

    def capture_single_frame_pair(self):
        """Captures a frame from left and right lidars and processes rectangle centroids in base_link."""
        left_frame = next(self.left_stream)
        right_frame = next(self.right_stream)

        if left_frame is None or right_frame is None:
            return None

        # Extract high-intensity points
        left_high = get_high_intensity_points(left_frame.points, left_frame.intensity, intensity_threshold=240.0)
        right_high = get_high_intensity_points(right_frame.points, right_frame.intensity, intensity_threshold=240.0)

        # ----------------------------------------------------
        # 1. Logging raw points and high-intensity to Rerun
        # ----------------------------------------------------
        # Calibrated View
        if len(left_frame.points) > 0:
            ones_l = np.ones((len(left_frame.points), 1))
            left_pts_base_cal = (self.T_base_from_left_calibrated @ np.hstack([left_frame.points, ones_l]).T).T[:, :3]
            rr.log(
                "calibrated/base_link/left_lidar/points",
                rr.Points3D(left_pts_base_cal, colors=[120, 80, 80], radii=[0.003]),
            )
        if len(left_high) > 0:
            ones_lh = np.ones((len(left_high), 1))
            left_high_base_cal = (self.T_base_from_left_calibrated @ np.hstack([left_high, ones_lh]).T).T[:, :3]
            rr.log(
                "calibrated/base_link/left_lidar/high_intensity",
                rr.Points3D(left_high_base_cal, colors=[255, 69, 0], radii=[0.006]),
            )

        if len(right_frame.points) > 0:
            ones_r = np.ones((len(right_frame.points), 1))
            right_pts_base_cal = (self.T_base_from_right_calibrated @ np.hstack([right_frame.points, ones_r]).T).T[:, :3]
            rr.log(
                "calibrated/base_link/right_lidar/points",
                rr.Points3D(right_pts_base_cal, colors=[80, 80, 120], radii=[0.003]),
            )
        if len(right_high) > 0:
            ones_rh = np.ones((len(right_high), 1))
            right_high_base_cal = (self.T_base_from_right_calibrated @ np.hstack([right_high, ones_rh]).T).T[:, :3]
            rr.log(
                "calibrated/base_link/right_lidar/high_intensity",
                rr.Points3D(right_high_base_cal, colors=[0, 255, 150], radii=[0.006]),
            )

        # Uncalibrated View
        if len(left_frame.points) > 0:
            ones_l = np.ones((len(left_frame.points), 1))
            left_pts_base_uncal = (self.T_base_from_left_uncalibrated @ np.hstack([left_frame.points, ones_l]).T).T[:, :3]
            rr.log(
                "uncalibrated/base_link/left_lidar/points",
                rr.Points3D(left_pts_base_uncal, colors=[120, 80, 80], radii=[0.003]),
            )
        if len(left_high) > 0:
            ones_lh = np.ones((len(left_high), 1))
            left_high_base_uncal = (self.T_base_from_left_uncalibrated @ np.hstack([left_high, ones_lh]).T).T[:, :3]
            rr.log(
                "uncalibrated/base_link/left_lidar/high_intensity",
                rr.Points3D(left_high_base_uncal, colors=[255, 69, 0], radii=[0.006]),
            )

        if len(right_frame.points) > 0:
            ones_r = np.ones((len(right_frame.points), 1))
            right_pts_base_uncal = (self.T_base_from_right_uncalibrated @ np.hstack([right_frame.points, ones_r]).T).T[:, :3]
            rr.log(
                "uncalibrated/base_link/right_lidar/points",
                rr.Points3D(right_pts_base_uncal, colors=[80, 80, 120], radii=[0.003]),
            )
        if len(right_high) > 0:
            ones_rh = np.ones((len(right_high), 1))
            right_high_base_uncal = (self.T_base_from_right_uncalibrated @ np.hstack([right_high, ones_rh]).T).T[:, :3]
            rr.log(
                "uncalibrated/base_link/right_lidar/high_intensity",
                rr.Points3D(right_high_base_uncal, colors=[0, 255, 150], radii=[0.006]),
            )

        # ----------------------------------------------------
        # 2. Detect rectangle corners on raw sensor centroids
        # ----------------------------------------------------
        _, left_centroids, _ = detect_lidar_rectangle(
            left_high, expected_width=self.expected_width, expected_height=self.expected_height, tolerance=self.tolerance
        )
        _, right_centroids, _ = detect_lidar_rectangle(
            right_high, expected_width=self.expected_width, expected_height=self.expected_height, tolerance=self.tolerance
        )

        if left_centroids is None or len(left_centroids) != 4 or right_centroids is None or len(right_centroids) != 4:
            return None

        # ----------------------------------------------------
        # 3. Process Calibrated Coordinates
        # ----------------------------------------------------
        ones = np.ones((4, 1))
        left_base_cal = (self.T_base_from_left_calibrated @ np.hstack([left_centroids, ones]).T).T[:, :3]
        right_base_cal = (self.T_base_from_right_calibrated @ np.hstack([right_centroids, ones]).T).T[:, :3]
        
        left_center_cal = np.mean(left_base_cal, axis=0)
        right_center_cal = np.mean(right_base_cal, axis=0)

        dist_matrix_cal = np.linalg.norm(left_base_cal[:, None, :] - right_base_cal[None, :, :], axis=2)
        row_ind_cal, col_ind_cal = linear_sum_assignment(dist_matrix_cal)
        right_base_matched_cal = right_base_cal[col_ind_cal]

        sort_idx_cal = get_angular_sort_indices(left_base_cal)
        left_base_ordered_cal = left_base_cal[sort_idx_cal]
        right_base_ordered_cal = right_base_matched_cal[sort_idx_cal]

        # ----------------------------------------------------
        # 4. Process Uncalibrated Coordinates
        # ----------------------------------------------------
        left_base_uncal = (self.T_base_from_left_uncalibrated @ np.hstack([left_centroids, ones]).T).T[:, :3]
        right_base_uncal = (self.T_base_from_right_uncalibrated @ np.hstack([right_centroids, ones]).T).T[:, :3]
        
        left_center_uncal = np.mean(left_base_uncal, axis=0)
        right_center_uncal = np.mean(right_base_uncal, axis=0)

        dist_matrix_uncal = np.linalg.norm(left_base_uncal[:, None, :] - right_base_uncal[None, :, :], axis=2)
        row_ind_uncal, col_ind_uncal = linear_sum_assignment(dist_matrix_uncal)
        right_base_matched_uncal = right_base_uncal[col_ind_uncal]

        sort_idx_uncal = get_angular_sort_indices(left_base_uncal)
        left_base_ordered_uncal = left_base_uncal[sort_idx_uncal]
        right_base_ordered_uncal = right_base_matched_uncal[sort_idx_uncal]

        return {
            'calibrated': {
                'left_base_corners': left_base_ordered_cal,
                'right_base_corners': right_base_ordered_cal,
                'left_base_center': left_center_cal,
                'right_base_center': right_center_cal,
            },
            'uncalibrated': {
                'left_base_corners': left_base_ordered_uncal,
                'right_base_corners': right_base_ordered_uncal,
                'left_base_center': left_center_uncal,
                'right_base_center': right_center_uncal,
            }
        }

    def run(self, skip_user_prompt: bool, interactive: bool):
        if not skip_user_prompt and not self.manual:
            print(f"\n\nThis script will move the robot to validation poses. Do you wish to proceed? [y/N]: \n\n ", end="", flush=True)
            ans = input()
            if ans.lower() != 'y':
                print("Exiting.", flush=True)
                raise Exception("User aborted comparison.")

        rr.init("Lidar_Lidar_Comparison", spawn=interactive)
        if interactive:
            self.setup_rerun_blueprint()

        print("Starting Lidar-Lidar Comparison...", flush=True)
        pose_results = []

        overall_corner_errors_cal = []
        overall_center_errors_cal = []
        overall_corner_errors_uncal = []
        overall_center_errors_uncal = []

        if self.manual:
            idx = 0
            self.preview_active = threading.Event()
            self.preview_thread = threading.Thread(target=self._preview_loop, daemon=True)
            self.preview_thread.start()

            while True:
                print(f"\n====================================", flush=True)
                print(f"[MANUAL MODE] Pose {idx+1}: Please position the robot.", flush=True)
                print(f"====================================", flush=True)

                # Enable live preview
                self.preview_active.set()

                print("Press [Enter] to capture this pose, or type 'q' and press Enter to finish: ", end="", flush=True)
                ans = input()

                # Disable live preview during final frame averaging capture
                self.preview_active.clear()
                print()  # Finalize the \r live preview console line cleanly

                if ans.lower() == 'q':
                    break

                self.robot.pull_status()
                current_lift = self.robot.lift.status['pos']
                current_arm = self.robot.arm.status['pos']
                current_wrist_yaw = self.robot.end_of_arm.status['wrist_yaw']['pos']
                current_wrist_pitch = self.robot.end_of_arm.status['wrist_pitch']['pos']
                current_wrist_roll = self.robot.end_of_arm.status['wrist_roll']['pos']
                pose = {
                    'lift': float(current_lift),
                    'arm': float(current_arm),
                    'wrist_yaw': float(current_wrist_yaw),
                    'wrist_pitch': float(current_wrist_pitch),
                    'wrist_roll': float(current_wrist_roll),
                }
                print(f"\nCapturing Pose {idx+1} (averaging 15 frames)...", flush=True)

                self.flush_lidar_streams()

                frame_measurements = []
                for attempt in range(15):
                    sample = self.capture_single_frame_pair()
                    if sample is not None:
                        frame_measurements.append(sample)
                    time.sleep(0.05)

                if len(frame_measurements) == 0:
                    print(f"Pose {idx+1}: Could not detect virtual rectangle on both lidars.", flush=True)
                    pose_dict = {
                        'pose_number': idx + 1,
                        'joint_targets': pose,
                        'status': 'FAILED_NO_DETECTION',
                    }
                    pose_results.append(pose_dict)
                    idx += 1
                    continue

                # Average calibrated measurements
                avg_left_corners_cal = np.mean([m['calibrated']['left_base_corners'] for m in frame_measurements], axis=0)
                avg_right_corners_cal = np.mean([m['calibrated']['right_base_corners'] for m in frame_measurements], axis=0)
                avg_left_center_cal = np.mean([m['calibrated']['left_base_center'] for m in frame_measurements], axis=0)
                avg_right_center_cal = np.mean([m['calibrated']['right_base_center'] for m in frame_measurements], axis=0)

                corner_errors_cal = [float(np.linalg.norm(avg_left_corners_cal[k] - avg_right_corners_cal[k])) for k in range(4)]
                mean_corner_error_cal = float(np.mean(corner_errors_cal))
                center_error_cal = float(np.linalg.norm(avg_left_center_cal - avg_right_center_cal))

                # Average uncalibrated measurements
                avg_left_corners_uncal = np.mean([m['uncalibrated']['left_base_corners'] for m in frame_measurements], axis=0)
                avg_right_corners_uncal = np.mean([m['uncalibrated']['right_base_corners'] for m in frame_measurements], axis=0)
                avg_left_center_uncal = np.mean([m['uncalibrated']['left_base_center'] for m in frame_measurements], axis=0)
                avg_right_center_uncal = np.mean([m['uncalibrated']['right_base_center'] for m in frame_measurements], axis=0)

                corner_errors_uncal = [float(np.linalg.norm(avg_left_corners_uncal[k] - avg_right_corners_uncal[k])) for k in range(4)]
                mean_corner_error_uncal = float(np.mean(corner_errors_uncal))
                center_error_uncal = float(np.linalg.norm(avg_left_center_uncal - avg_right_center_uncal))

                overall_corner_errors_cal.extend(corner_errors_cal)
                overall_center_errors_cal.append(center_error_cal)
                overall_corner_errors_uncal.extend(corner_errors_uncal)
                overall_center_errors_uncal.append(center_error_uncal)

                print(f"\n--- Results for Manual Pose {idx+1} ---", flush=True)
                print(f"  [Calibrated]   Mean Corner Error: {mean_corner_error_cal*1000.0:.2f} mm | Center Error: {center_error_cal*1000.0:.2f} mm")
                print(f"  [Uncalibrated] Mean Corner Error: {mean_corner_error_uncal*1000.0:.2f} mm | Center Error: {center_error_uncal*1000.0:.2f} mm", flush=True)

                # Rerun logging - Calibrated
                rr.log(
                    "calibrated/base_link/left_corners",
                    rr.Points3D(avg_left_corners_cal, colors=[255, 0, 0], radii=[0.008], labels=[f"L_C{k+1}" for k in range(4)]),
                )
                rr.log(
                    "calibrated/base_link/right_corners",
                    rr.Points3D(avg_right_corners_cal, colors=[0, 255, 0], radii=[0.008], labels=[f"R_C{k+1}" for k in range(4)]),
                )
                rr.log(
                    "calibrated/base_link/left_center",
                    rr.Points3D([avg_left_center_cal], colors=[255, 100, 100], radii=[0.012], labels=["L_Center"]),
                )
                rr.log(
                    "calibrated/base_link/right_center",
                    rr.Points3D([avg_right_center_cal], colors=[100, 255, 100], radii=[0.012], labels=["R_Center"]),
                )

                # Rerun logging - Uncalibrated
                rr.log(
                    "uncalibrated/base_link/left_corners",
                    rr.Points3D(avg_left_corners_uncal, colors=[255, 0, 0], radii=[0.008], labels=[f"L_C{k+1}" for k in range(4)]),
                )
                rr.log(
                    "uncalibrated/base_link/right_corners",
                    rr.Points3D(avg_right_corners_uncal, colors=[0, 255, 0], radii=[0.008], labels=[f"R_C{k+1}" for k in range(4)]),
                )
                rr.log(
                    "uncalibrated/base_link/left_center",
                    rr.Points3D([avg_left_center_uncal], colors=[255, 100, 100], radii=[0.012], labels=["L_Center"]),
                )
                rr.log(
                    "uncalibrated/base_link/right_center",
                    rr.Points3D([avg_right_center_uncal], colors=[100, 255, 100], radii=[0.012], labels=["R_Center"]),
                )

                instructions = f"""
# Lidar-Lidar Comparison Status (Manual Mode)

### Pose {idx+1}
- **Joint Targets**: {pose}

#### Calibrated (apply_calibration=True)
- **Corner Errors**: {", ".join([f"{e*1000.0:.2f}" for e in corner_errors_cal])} mm
- **Mean Corner Error**: {mean_corner_error_cal*1000.0:.2f} mm
- **Center Error**: {center_error_cal*1000.0:.2f} mm

#### Uncalibrated (apply_calibration=False)
- **Corner Errors**: {", ".join([f"{e*1000.0:.2f}" for e in corner_errors_uncal])} mm
- **Mean Corner Error**: {mean_corner_error_uncal*1000.0:.2f} mm
- **Center Error**: {center_error_uncal*1000.0:.2f} mm
"""
                self.update_instructions(instructions)

                pose_dict = {
                    'pose_number': idx + 1,
                    'joint_targets': pose,
                    'status': 'SUCCESS',
                    'calibrated': {
                        'left_lidar_base': {
                            'corner_1': avg_left_corners_cal[0].tolist(),
                            'corner_2': avg_left_corners_cal[1].tolist(),
                            'corner_3': avg_left_corners_cal[2].tolist(),
                            'corner_4': avg_left_corners_cal[3].tolist(),
                            'center': avg_left_center_cal.tolist(),
                        },
                        'right_lidar_base': {
                            'corner_1': avg_right_corners_cal[0].tolist(),
                            'corner_2': avg_right_corners_cal[1].tolist(),
                            'corner_3': avg_right_corners_cal[2].tolist(),
                            'corner_4': avg_right_corners_cal[3].tolist(),
                            'center': avg_right_center_cal.tolist(),
                        },
                        'errors_m': {
                            'corner_1_error': corner_errors_cal[0],
                            'corner_2_error': corner_errors_cal[1],
                            'corner_3_error': corner_errors_cal[2],
                            'corner_4_error': corner_errors_cal[3],
                            'center_error': center_error_cal,
                            'mean_corner_error': mean_corner_error_cal,
                        },
                    },
                    'uncalibrated': {
                        'left_lidar_base': {
                            'corner_1': avg_left_corners_uncal[0].tolist(),
                            'corner_2': avg_left_corners_uncal[1].tolist(),
                            'corner_3': avg_left_corners_uncal[2].tolist(),
                            'corner_4': avg_left_corners_uncal[3].tolist(),
                            'center': avg_left_center_uncal.tolist(),
                        },
                        'right_lidar_base': {
                            'corner_1': avg_right_corners_uncal[0].tolist(),
                            'corner_2': avg_right_corners_uncal[1].tolist(),
                            'corner_3': avg_right_corners_uncal[2].tolist(),
                            'corner_4': avg_right_corners_uncal[3].tolist(),
                            'center': avg_right_center_uncal.tolist(),
                        },
                        'errors_m': {
                            'corner_1_error': corner_errors_uncal[0],
                            'corner_2_error': corner_errors_uncal[1],
                            'corner_3_error': corner_errors_uncal[2],
                            'corner_4_error': corner_errors_uncal[3],
                            'center_error': center_error_uncal,
                            'mean_corner_error': mean_corner_error_uncal,
                        },
                    }
                }
                pose_results.append(pose_dict)
                idx += 1
        else:
            for idx, target_pose in enumerate(self.poses):
                self.move_to_pose(target_pose)
                print(f"Pose {idx+1}/{len(self.poses)} reached. Capturing lidar frames...", flush=True)

                self.flush_lidar_streams()

                frame_measurements = []
                for attempt in range(15):
                    sample = self.capture_single_frame_pair()
                    if sample is not None:
                        frame_measurements.append(sample)
                    time.sleep(0.05)

                self.robot.pull_status()
                current_lift = self.robot.lift.status['pos']
                current_arm = self.robot.arm.status['pos']
                current_wrist_yaw = self.robot.end_of_arm.status['wrist_yaw']['pos']
                current_wrist_pitch = self.robot.end_of_arm.status['wrist_pitch']['pos']
                current_wrist_roll = self.robot.end_of_arm.status['wrist_roll']['pos']
                pose = {
                    'lift': float(current_lift),
                    'arm': float(current_arm),
                    'wrist_yaw': float(current_wrist_yaw),
                    'wrist_pitch': float(current_wrist_pitch),
                    'wrist_roll': float(current_wrist_roll),
                }

                if len(frame_measurements) == 0:
                    print(f"Pose {idx+1}: Could not detect virtual rectangle on both lidars.", flush=True)
                    pose_dict = {
                        'pose_number': idx + 1,
                        'joint_targets': pose,
                        'status': 'FAILED_NO_DETECTION',
                    }
                    pose_results.append(pose_dict)
                    continue

                # Average calibrated measurements
                avg_left_corners_cal = np.mean([m['calibrated']['left_base_corners'] for m in frame_measurements], axis=0)
                avg_right_corners_cal = np.mean([m['calibrated']['right_base_corners'] for m in frame_measurements], axis=0)
                avg_left_center_cal = np.mean([m['calibrated']['left_base_center'] for m in frame_measurements], axis=0)
                avg_right_center_cal = np.mean([m['calibrated']['right_base_center'] for m in frame_measurements], axis=0)

                corner_errors_cal = [float(np.linalg.norm(avg_left_corners_cal[k] - avg_right_corners_cal[k])) for k in range(4)]
                mean_corner_error_cal = float(np.mean(corner_errors_cal))
                center_error_cal = float(np.linalg.norm(avg_left_center_cal - avg_right_center_cal))

                # Average uncalibrated measurements
                avg_left_corners_uncal = np.mean([m['uncalibrated']['left_base_corners'] for m in frame_measurements], axis=0)
                avg_right_corners_uncal = np.mean([m['uncalibrated']['right_base_corners'] for m in frame_measurements], axis=0)
                avg_left_center_uncal = np.mean([m['uncalibrated']['left_base_center'] for m in frame_measurements], axis=0)
                avg_right_center_uncal = np.mean([m['uncalibrated']['right_base_center'] for m in frame_measurements], axis=0)

                corner_errors_uncal = [float(np.linalg.norm(avg_left_corners_uncal[k] - avg_right_corners_uncal[k])) for k in range(4)]
                mean_corner_error_uncal = float(np.mean(corner_errors_uncal))
                center_error_uncal = float(np.linalg.norm(avg_left_center_uncal - avg_right_center_uncal))

                overall_corner_errors_cal.extend(corner_errors_cal)
                overall_center_errors_cal.append(center_error_cal)
                overall_corner_errors_uncal.extend(corner_errors_uncal)
                overall_center_errors_uncal.append(center_error_uncal)

                print(f"\n--- Results for Pose {idx+1} ---", flush=True)
                print(f"  [Calibrated]   Mean Corner Error: {mean_corner_error_cal*1000.0:.2f} mm | Center Error: {center_error_cal*1000.0:.2f} mm")
                print(f"  [Uncalibrated] Mean Corner Error: {mean_corner_error_uncal*1000.0:.2f} mm | Center Error: {center_error_uncal*1000.0:.2f} mm", flush=True)

                # Rerun logging - Calibrated
                rr.log(
                    "calibrated/base_link/left_corners",
                    rr.Points3D(avg_left_corners_cal, colors=[255, 0, 0], radii=[0.008], labels=[f"L_C{k+1}" for k in range(4)]),
                )
                rr.log(
                    "calibrated/base_link/right_corners",
                    rr.Points3D(avg_right_corners_cal, colors=[0, 255, 0], radii=[0.008], labels=[f"R_C{k+1}" for k in range(4)]),
                )
                rr.log(
                    "calibrated/base_link/left_center",
                    rr.Points3D([avg_left_center_cal], colors=[255, 100, 100], radii=[0.012], labels=["L_Center"]),
                )
                rr.log(
                    "calibrated/base_link/right_center",
                    rr.Points3D([avg_right_center_cal], colors=[100, 255, 100], radii=[0.012], labels=["R_Center"]),
                )

                # Rerun logging - Uncalibrated
                rr.log(
                    "uncalibrated/base_link/left_corners",
                    rr.Points3D(avg_left_corners_uncal, colors=[255, 0, 0], radii=[0.008], labels=[f"L_C{k+1}" for k in range(4)]),
                )
                rr.log(
                    "uncalibrated/base_link/right_corners",
                    rr.Points3D(avg_right_corners_uncal, colors=[0, 255, 0], radii=[0.008], labels=[f"R_C{k+1}" for k in range(4)]),
                )
                rr.log(
                    "uncalibrated/base_link/left_center",
                    rr.Points3D([avg_left_center_uncal], colors=[255, 100, 100], radii=[0.012], labels=["L_Center"]),
                )
                rr.log(
                    "uncalibrated/base_link/right_center",
                    rr.Points3D([avg_right_center_uncal], colors=[100, 255, 100], radii=[0.012], labels=["R_Center"]),
                )

                instructions = f"""
# Lidar-Lidar Comparison Status

### Pose {idx+1} / {len(self.poses)}
- **Joint Targets**: {pose}

#### Calibrated (apply_calibration=True)
- **Corner Errors**: {", ".join([f"{e*1000.0:.2f}" for e in corner_errors_cal])} mm
- **Mean Corner Error**: {mean_corner_error_cal*1000.0:.2f} mm
- **Center Error**: {center_error_cal*1000.0:.2f} mm

#### Uncalibrated (apply_calibration=False)
- **Corner Errors**: {", ".join([f"{e*1000.0:.2f}" for e in corner_errors_uncal])} mm
- **Mean Corner Error**: {mean_corner_error_uncal*1000.0:.2f} mm
- **Center Error**: {center_error_uncal*1000.0:.2f} mm
"""
                self.update_instructions(instructions)

                pose_dict = {
                    'pose_number': idx + 1,
                    'joint_targets': pose,
                    'status': 'SUCCESS',
                    'calibrated': {
                        'left_lidar_base': {
                            'corner_1': avg_left_corners_cal[0].tolist(),
                            'corner_2': avg_left_corners_cal[1].tolist(),
                            'corner_3': avg_left_corners_cal[2].tolist(),
                            'corner_4': avg_left_corners_cal[3].tolist(),
                            'center': avg_left_center_cal.tolist(),
                        },
                        'right_lidar_base': {
                            'corner_1': avg_right_corners_cal[0].tolist(),
                            'corner_2': avg_right_corners_cal[1].tolist(),
                            'corner_3': avg_right_corners_cal[2].tolist(),
                            'corner_4': avg_right_corners_cal[3].tolist(),
                            'center': avg_right_center_cal.tolist(),
                        },
                        'errors_m': {
                            'corner_1_error': corner_errors_cal[0],
                            'corner_2_error': corner_errors_cal[1],
                            'corner_3_error': corner_errors_cal[2],
                            'corner_4_error': corner_errors_cal[3],
                            'center_error': center_error_cal,
                            'mean_corner_error': mean_corner_error_cal,
                        },
                    },
                    'uncalibrated': {
                        'left_lidar_base': {
                            'corner_1': avg_left_corners_uncal[0].tolist(),
                            'corner_2': avg_left_corners_uncal[1].tolist(),
                            'corner_3': avg_left_corners_uncal[2].tolist(),
                            'corner_4': avg_left_corners_uncal[3].tolist(),
                            'center': avg_left_center_uncal.tolist(),
                        },
                        'right_lidar_base': {
                            'corner_1': avg_right_corners_uncal[0].tolist(),
                            'corner_2': avg_right_corners_uncal[1].tolist(),
                            'corner_3': avg_right_corners_uncal[2].tolist(),
                            'corner_4': avg_right_corners_uncal[3].tolist(),
                            'center': avg_right_center_uncal.tolist(),
                        },
                        'errors_m': {
                            'corner_1_error': corner_errors_uncal[0],
                            'corner_2_error': corner_errors_uncal[1],
                            'corner_3_error': corner_errors_uncal[2],
                            'corner_4_error': corner_errors_uncal[3],
                            'center_error': center_error_uncal,
                            'mean_corner_error': mean_corner_error_uncal,
                        },
                    }
                }
                pose_results.append(pose_dict)

        # Output YAML summary
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fleet_id = os.environ.get("HELLO_FLEET_ID", "unknown")

        overall_summary = {
            'calibrated': {
                'overall_mean_corner_error_m': float(np.mean(overall_corner_errors_cal)) if len(overall_corner_errors_cal) > 0 else None,
                'overall_mean_center_error_m': float(np.mean(overall_center_errors_cal)) if len(overall_center_errors_cal) > 0 else None,
                'max_corner_error_m': float(np.max(overall_corner_errors_cal)) if len(overall_corner_errors_cal) > 0 else None,
            },
            'uncalibrated': {
                'overall_mean_corner_error_m': float(np.mean(overall_corner_errors_uncal)) if len(overall_corner_errors_uncal) > 0 else None,
                'overall_mean_center_error_m': float(np.mean(overall_center_errors_uncal)) if len(overall_center_errors_uncal) > 0 else None,
                'max_corner_error_m': float(np.max(overall_corner_errors_uncal)) if len(overall_corner_errors_uncal) > 0 else None,
            }
        }

        output_data = {
            'timestamp': timestamp,
            'fleet_id': fleet_id,
            'poses': pose_results,
            'summary': overall_summary,
        }

        if self.output_file is None:
            fleet_path = os.environ.get("HELLO_FLEET_PATH", "")
            if fleet_path and fleet_id:
                save_dir = os.path.join(fleet_path, fleet_id, "calibration_dual_lidar", "lidar_lidar_comparison")
            else:
                save_dir = os.path.join(DEFAULT_CALIBRATION_FOLDER_PATH, "lidar_lidar_comparison")
            os.makedirs(save_dir, exist_ok=True)
            self.output_file = os.path.join(save_dir, f"lidar_lidar_comparison_{timestamp}.yaml")
        else:
            os.makedirs(os.path.dirname(os.path.abspath(self.output_file)), exist_ok=True)

        with open(self.output_file, 'w') as f:
            yaml.dump(output_data, f, default_flow_style=False)

        print(f"\n====================================")
        print(f"Lidar-Lidar Comparison Complete!")
        print(f"--- Calibrated Results (apply_calibration=True) ---")
        print(f"Overall Mean Corner Error: {overall_summary['calibrated']['overall_mean_corner_error_m']*1000.0 if overall_summary['calibrated']['overall_mean_corner_error_m'] else 0.0:.2f} mm")
        print(f"Overall Mean Center Error: {overall_summary['calibrated']['overall_mean_center_error_m']*1000.0 if overall_summary['calibrated']['overall_mean_center_error_m'] else 0.0:.2f} mm")
        print(f"--- Uncalibrated Results (apply_calibration=False) ---")
        print(f"Overall Mean Corner Error: {overall_summary['uncalibrated']['overall_mean_corner_error_m']*1000.0 if overall_summary['uncalibrated']['overall_mean_corner_error_m'] else 0.0:.2f} mm")
        print(f"Overall Mean Center Error: {overall_summary['uncalibrated']['overall_mean_center_error_m']*1000.0 if overall_summary['uncalibrated']['overall_mean_center_error_m'] else 0.0:.2f} mm")
        print(f"Saved results to: {self.output_file}")
        print(f"====================================\n")

        return output_data


def REx_lidar_lidar_compare(interactive: bool = True):
    args = _parse_args()
    comparator = None
    try:
        comparator = LidarLidarCompare(
            use_ros_for_lidars=args.use_ros_for_lidars,
            expected_width=args.expected_width,
            expected_height=args.expected_height,
            tolerance=args.tolerance,
            output_file=args.output_file,
            manual=args.manual,
        )
        return comparator.run(skip_user_prompt=not interactive or args.skip_user_prompt, interactive=interactive and not args.not_interactive)
    except KeyboardInterrupt:
        print("\nInterrupted by user (Ctrl+C). Cleaning up and exiting...")
        if comparator is not None:
            if hasattr(comparator, 'robot') and comparator.robot is not None:
                try:
                    comparator.robot.stop()
                except Exception as e:
                    print(f"Warning: error stopping robot: {e}")
        os._exit(1)
    finally:
        if comparator is not None:
            try:
                comparator.cleanup()
            except Exception as e:
                print(f"Warning: error during cleanup: {e}")


if __name__ == "__main__":
    REx_lidar_lidar_compare(interactive=True)
    os._exit(0)
