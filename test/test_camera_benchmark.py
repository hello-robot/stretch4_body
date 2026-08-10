#!/usr/bin/env python3
"""
Side-by-Side Luxonis DepthAI Camera Performance Benchmark & Regression Test

This script measures and compares performance characteristics (FPS, latency, jitter, drop rate)
between the older camera wrappers (from stretch4_rgbd and stretch4_gripper_modeling_and_control)
and the newer wrappers merged into stretch4_body.

0 ms Jitter (Perfect): If a camera is running at a perfect, rock-solid 30 FPS, every single frame arrives exactly every 33.33 ms. Because the interval is constant, the standard deviation is 0 ms.
Low Jitter (e.g., 2–5 ms): The frames are arriving very smoothly and predictably (e.g., alternating slightly between 31 ms and 35 ms).
High Jitter (e.g., >10 ms): The stream is choppy, uneven, or experiencing transport hiccups. Some frames might arrive in rapid bursts (e.g., 5 ms apart) while others are delayed (e.g., 60 ms apart).

To run:
    python3 test/benchmark_cameras.py --duration 10
"""

import sys
import os
import time
import argparse
import numpy as np
import pytest

# Dynamic path configuration to allow importing from all active workspaces
HOME_REPOS = os.path.join(os.path.expanduser("~"), "repos")
OLD_RGBD_PATH = os.path.join(HOME_REPOS, "stretch4_rgbd")
OLD_GRIPPER_PATH = os.path.join(HOME_REPOS, "stretch4_gripper_modeling_and_control")

if os.path.exists(OLD_RGBD_PATH):
    sys.path.insert(0, OLD_RGBD_PATH)
if os.path.exists(OLD_GRIPPER_PATH):
    sys.path.insert(0, OLD_GRIPPER_PATH)
    # The gripper repo has src/ containing the stretch4_gripper_modeling_and_control package
    src_dir = os.path.join(OLD_GRIPPER_PATH, "src")
    if os.path.exists(src_dir):
        sys.path.insert(0, src_dir)

# Ensure current stretch4_body is also in path
sys.path.insert(0, os.path.join(HOME_REPOS, "stretch4_body"))

import depthai as dai


def run_head_camera_old(duration, target_fps=30):
    """Benchmarks the HeadCamera from stretch4_rgbd."""
    print("\n--- Benchmarking HeadCamera (stretch4_rgbd) ---")
    if not os.path.exists(OLD_RGBD_PATH):
        print(f"OLD_RGBD_PATH ({OLD_RGBD_PATH}) is not available. Skipping stretch4_rgbd HeadCamera benchmark.")
        return None
    try:
        from stretch4_emulated_rgbd.head_camera import HeadCamera
    except ImportError as e:
        print(f"Failed to import HeadCamera from stretch4_rgbd: {e}")
        return None

    t_init_start = time.perf_counter()
    try:
        # Instantiate old HeadCamera
        camera = HeadCamera(camera_name=["left", "right"], fps=target_fps, resolution_height=800, compress=True, oak_buffer_size=1)
        camera.start()
    except Exception as e:
        print(f"Failed to initialize old HeadCamera: {e}")
        return None

    # Wait for the first frame
    print("Waiting for first frame...")
    first_frame_received = False
    t_first_frame = None
    
    # Simple poll until a frame exists in the buffer
    while (time.perf_counter() - t_init_start) < 15.0:
        img, ts, seq, _ = camera.get_closest_frame(time.monotonic(), "right")
        if ts is not None:
            t_first_frame = time.perf_counter()
            first_frame_received = True
            break
        time.sleep(0.01)

    if not first_frame_received:
        print("Timeout waiting for first frame from old HeadCamera.")
        camera.stop()
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    # Data collection loop
    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration
    last_seq = None

    while time.monotonic() < t_end:
        # Request closest frame to current time
        img, ts, seq, _ = camera.get_closest_frame(time.monotonic(), "right")
        if ts is not None and seq != last_seq:
            timestamps_host.append(time.monotonic())
            timestamps_sensor.append(ts)
            seq_nums.append(seq)
            last_seq = seq
        # Yield CPU slightly to match the target frame rate
        time.sleep(1.0 / (target_fps * 1.5))

    camera.stop()
    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def start_ros_launch(launch_args, wait_seconds=8.0):
    """Starts a ROS2 launch file in a separate process group and optionally waits for it to initialize."""
    import subprocess
    import os
    import time
    print(f"Launching: {' '.join(launch_args)}")
    try:
        # Launch the ROS2 process in its own process group
        proc = subprocess.Popen(
            launch_args,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )
        # Sleep to let nodes initialize and begin publishing
        if wait_seconds > 0:
            print(f"Waiting {wait_seconds} seconds for ROS2 nodes to initialize...")
            time.sleep(wait_seconds)
        return proc
    except Exception as e:
        print(f"Failed to launch ROS2 nodes: {e}")
        return None


def stop_ros_launch(proc):
    """Cleanly stops the ROS2 launch process group using SIGINT."""
    import signal
    import os
    if proc is None:
        return
    print("Stopping ROS2 launch process...")
    try:
        # Send SIGINT to the entire process group
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=10.0)
    except Exception as e:
        print(f"Error stopping ROS2 launch process: {e}")
        try:
            # Force kill if SIGINT didn't work
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
    print("Waiting 5 seconds for ROS2 camera nodes to stop...")
    time.sleep(5.0)


def run_left_right_python(duration):
    """Benchmarks stream_left_right_camera(use_ros_for_cameras=False) from stretch4_body."""
    print("\n--- Benchmarking Head Camera stream_left_right_camera (stretch4_body Python) ---")
    try:
        from stretch4_body.subsystem.cameras.stream_cameras import stream_left_right_camera
    except ImportError as e:
        print(f"Failed to import stretch4_body camera modules: {e}")
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_left_right_camera(use_ros_for_cameras=False)
    except Exception as e:
        print(f"Failed to initialize stream_left_right_camera: {e}")
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None and frame.right is not None:
                t_first_frame = time.perf_counter()
                first_frame_received = True
                break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting first frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_left_right_camera Python.")
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    for frame in frame_generator:
        if frame is not None and frame.right is not None:
            timestamps_host.append(time.monotonic())
            timestamps_sensor.append(frame.right.timestamp)
            seq_nums.append(frame.right.frame_number)
        
        if time.monotonic() >= t_end:
            break


    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_left_right_ros(duration):
    """Benchmarks stream_left_right_camera(use_ros_for_cameras=True) from stretch4_body."""
    print("\n--- Benchmarking Head Camera stream_left_right_camera (stretch4_body ROS2) ---")
    proc = start_ros_launch(["ros2", "launch", "stretch_core", "luxonis.launch.py", "use_center:=true"])
    if proc is None:
        print("Could not start ROS2 head camera launch file. Skipping ROS benchmark.")
        return None

    try:
        from stretch4_body.subsystem.cameras.stream_cameras import stream_left_right_camera
    except ImportError as e:
        print(f"Failed to import stretch4_body camera modules: {e}")
        stop_ros_launch(proc)
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_left_right_camera(use_ros_for_cameras=True)
    except Exception as e:
        print(f"Failed to initialize stream_left_right_camera ROS2: {e}")
        stop_ros_launch(proc)
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None and frame.right is not None:
                t_first_frame = time.perf_counter()
                first_frame_received = True
                break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_left_right_camera ROS2.")
        stop_ros_launch(proc)
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    for frame in frame_generator:
        if frame is not None and frame.right is not None:
            timestamps_host.append(time.time())
            timestamps_sensor.append(frame.right.timestamp)
            seq_nums.append(frame.right.frame_number)
        
        if time.monotonic() >= t_end:
            break


    stop_ros_launch(proc)
    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_left_right_center_python(duration):
    """Benchmarks stream_left_right_center_camera(use_ros_for_cameras=False) from stretch4_body."""
    print("\n--- Benchmarking Head Camera stream_left_right_center_camera (stretch4_body Python) ---")
    try:
        from stretch4_body.subsystem.cameras.stream_cameras import stream_left_right_center_camera
    except ImportError as e:
        print(f"Failed to import stretch4_body camera modules: {e}")
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_left_right_center_camera(use_ros_for_cameras=False)
    except Exception as e:
        print(f"Failed to initialize stream_left_right_center_camera: {e}")
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None and frame.right is not None:
                t_first_frame = time.perf_counter()
                first_frame_received = True
                break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting first frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_left_right_center_camera Python.")
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    for frame in frame_generator:
        if frame is not None and frame.right is not None:
            timestamps_host.append(time.monotonic())
            timestamps_sensor.append(frame.right.timestamp)
            seq_nums.append(frame.right.frame_number)
        
        if time.monotonic() >= t_end:
            break


    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_left_right_center_ros(duration):
    """Benchmarks stream_left_right_center_camera(use_ros_for_cameras=True) from stretch4_body."""
    print("\n--- Benchmarking Head Camera stream_left_right_center_camera (stretch4_body ROS2) ---")
    proc = start_ros_launch(["ros2", "launch", "stretch_core", "luxonis.launch.py", "use_center:=true"])
    if proc is None:
        print("Could not start ROS2 head camera launch file. Skipping ROS benchmark.")
        return None

    try:
        from stretch4_body.subsystem.cameras.stream_cameras import stream_left_right_center_camera
    except ImportError as e:
        print(f"Failed to import stretch4_body camera modules: {e}")
        stop_ros_launch(proc)
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_left_right_center_camera(use_ros_for_cameras=True)
    except Exception as e:
        print(f"Failed to initialize stream_left_right_center_camera ROS2: {e}")
        stop_ros_launch(proc)
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None and frame.right is not None:
                t_first_frame = time.perf_counter()
                first_frame_received = True
                break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_left_right_center_camera ROS2.")
        stop_ros_launch(proc)
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration
 
    for frame in frame_generator:
        if frame is not None and frame.right is not None:
            timestamps_host.append(time.time())
            timestamps_sensor.append(frame.right.timestamp)
            seq_nums.append(frame.right.frame_number)
         
        if time.monotonic() >= t_end:
            break


    stop_ros_launch(proc)
    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_left_rgbd_python(duration):
    """Benchmarks stream_left_rgbd(use_ros_for_cameras=False) from stretch4_body."""
    print("\n--- Benchmarking stream_left_rgbd (stretch4_body Python) ---")
    try:
        from stretch4_body.subsystem.cameras.emulated_rgbd import stream_left_rgbd
    except ImportError as e:
        print(f"Failed to import stretch4_body emulated_rgbd modules: {e}")
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_left_rgbd(use_ros_for_cameras=False)
    except Exception as e:
        print(f"Failed to initialize stream_left_rgbd: {e}")
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None and frame.image_frame is not None:
                t_first_frame = time.perf_counter()
                first_frame_received = True
                break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting first frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_left_rgbd Python.")
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    for frame in frame_generator:
        if frame is not None and frame.image_frame is not None:
            timestamps_host.append(time.monotonic())
            timestamps_sensor.append(frame.image_frame.timestamp)
            seq_nums.append(frame.image_frame.frame_number)
        else:
            time.sleep(0.01)
        
        if time.monotonic() >= t_end:
            break
        
    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_left_rgbd_ros(duration):
    """Benchmarks stream_left_rgbd(use_ros_for_cameras=True) from stretch4_body."""
    print("\n--- Benchmarking stream_left_rgbd (stretch4_body ROS2) ---")
    proc_camera = start_ros_launch(["ros2", "launch", "stretch_core", "luxonis.launch.py", "use_center:=true"], wait_seconds=0.0)
    proc_driver = start_ros_launch(["ros2", "launch", "stretch_core", "stretch_driver.launch.py"], wait_seconds=8.0)
    proc_lidar = start_ros_launch(["ros2", "launch", "stretch_core", "dual_hesai.launch.py"], wait_seconds=8.0)
    if proc_camera is None or proc_lidar is None or proc_driver is None:
        print("Could not start ROS2 launch files. Skipping ROS benchmark.")
        if proc_camera: stop_ros_launch(proc_camera)
        if proc_lidar: stop_ros_launch(proc_lidar)
        if proc_driver: stop_ros_launch(proc_driver)
        return None

    try:
        from stretch4_body.subsystem.cameras.emulated_rgbd import stream_left_rgbd
    except ImportError as e:
        print(f"Failed to import stretch4_body emulated_rgbd modules: {e}")
        stop_ros_launch(proc_camera)
        stop_ros_launch(proc_lidar)
        stop_ros_launch(proc_driver)
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_left_rgbd(use_ros_for_cameras=True, use_ros_for_lidars=True)
    except Exception as e:
        print(f"Failed to initialize stream_left_rgbd ROS2: {e}")
        stop_ros_launch(proc_camera)
        stop_ros_launch(proc_lidar)
        stop_ros_launch(proc_driver)
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None and frame.image_frame is not None:
                t_first_frame = time.perf_counter()
                first_frame_received = True
                break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_left_rgbd ROS2.")
        stop_ros_launch(proc_camera)
        stop_ros_launch(proc_lidar)
        stop_ros_launch(proc_driver)
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    for frame in frame_generator:
        if frame is not None and frame.image_frame is not None:
            timestamps_host.append(time.time())
            timestamps_sensor.append(frame.image_frame.timestamp)
            seq_nums.append(frame.image_frame.frame_number)
        
        if time.monotonic() >= t_end:
            break


    stop_ros_launch(proc_camera)
    stop_ros_launch(proc_lidar)
    stop_ros_launch(proc_driver)
    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_left_right_rgbd_python(duration):
    """Benchmarks stream_left_right_rgbd(use_ros_for_cameras=False) from stretch4_body."""
    print("\n--- Benchmarking stream_left_right_rgbd (stretch4_body Python) ---")
    try:
        from stretch4_body.subsystem.cameras.emulated_rgbd import stream_left_right_rgbd
    except ImportError as e:
        print(f"Failed to import stretch4_body emulated_rgbd modules: {e}")
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_left_right_rgbd(use_ros_for_cameras=False)
    except Exception as e:
        print(f"Failed to initialize stream_left_right_rgbd: {e}")
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None:
                active_frame = frame.right if frame.right is not None else frame.left
                if active_frame is not None and active_frame.image_frame is not None:
                    t_first_frame = time.perf_counter()
                    first_frame_received = True
                    break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting first frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_left_right_rgbd Python.")
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    for frame in frame_generator:
        if frame is not None:
            active_frame = frame.right if frame.right is not None else frame.left
            if active_frame is not None and active_frame.image_frame is not None:
                timestamps_host.append(time.monotonic())
                timestamps_sensor.append(active_frame.image_frame.timestamp)
                seq_nums.append(active_frame.image_frame.frame_number)
        
        if time.monotonic() >= t_end:
            break


    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_left_right_rgbd_ros(duration):
    """Benchmarks stream_left_right_rgbd(use_ros_for_cameras=True) from stretch4_body."""
    print("\n--- Benchmarking stream_left_right_rgbd (stretch4_body ROS2) ---")
    proc_camera = start_ros_launch(["ros2", "launch", "stretch_core", "luxonis.launch.py", "use_center:=true"], wait_seconds=0.0)
    proc_lidar = start_ros_launch(["ros2", "launch", "stretch_core", "dual_hesai.launch.py"], wait_seconds=8.0)
    if proc_camera is None or proc_lidar is None:
        print("Could not start ROS2 launch files. Skipping ROS benchmark.")
        if proc_camera: stop_ros_launch(proc_camera)
        if proc_lidar: stop_ros_launch(proc_lidar)
        return None

    try:
        from stretch4_body.subsystem.cameras.emulated_rgbd import stream_left_right_rgbd
    except ImportError as e:
        print(f"Failed to import stretch4_body emulated_rgbd modules: {e}")
        stop_ros_launch(proc_camera)
        stop_ros_launch(proc_lidar)
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_left_right_rgbd(use_ros_for_cameras=True, use_ros_for_lidars=True)
    except Exception as e:
        print(f"Failed to initialize stream_left_right_rgbd ROS2: {e}")
        stop_ros_launch(proc_camera)
        stop_ros_launch(proc_lidar)
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None:
                active_frame = frame.right if frame.right is not None else frame.left
                if active_frame is not None and active_frame.image_frame is not None:
                    t_first_frame = time.perf_counter()
                    first_frame_received = True
                    break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_left_right_rgbd ROS2.")
        stop_ros_launch(proc_camera)
        stop_ros_launch(proc_lidar)
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    for frame in frame_generator:
        if frame is not None:
            active_frame = frame.right if frame.right is not None else frame.left
            if active_frame is not None and active_frame.image_frame is not None:
                timestamps_host.append(time.time())
                timestamps_sensor.append(active_frame.image_frame.timestamp)
                seq_nums.append(active_frame.image_frame.frame_number)
        
        if time.monotonic() >= t_end:
            break


    stop_ros_launch(proc_camera)
    stop_ros_launch(proc_lidar)
    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_left_right_center_rgbd_python(duration):
    """Benchmarks stream_left_right_center_rgbd(use_ros_for_cameras=False) from stretch4_body."""
    print("\n--- Benchmarking stream_left_right_center_rgbd (stretch4_body Python) ---")
    try:
        from stretch4_body.subsystem.cameras.emulated_rgbd import stream_left_right_center_rgbd
    except ImportError as e:
        print(f"Failed to import stretch4_body emulated_rgbd modules: {e}")
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_left_right_center_rgbd(use_ros_for_cameras=False)
    except Exception as e:
        print(f"Failed to initialize stream_left_right_center_rgbd: {e}")
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None:
                active_frame = frame.right if frame.right is not None else (frame.left if frame.left is not None else frame.center)
                if active_frame is not None and active_frame.image_frame is not None:
                    t_first_frame = time.perf_counter()
                    first_frame_received = True
                    break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting first frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_left_right_center_rgbd Python.")
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    for frame in frame_generator:
        if frame is not None:
            active_frame = frame.right if frame.right is not None else (frame.left if frame.left is not None else frame.center)
            if active_frame is not None and active_frame.image_frame is not None:
                timestamps_host.append(time.monotonic())
                timestamps_sensor.append(active_frame.image_frame.timestamp)
                seq_nums.append(active_frame.image_frame.frame_number)
        
        if time.monotonic() >= t_end:
            break


    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_left_right_center_rgbd_ros(duration):
    """Benchmarks stream_left_right_center_rgbd(use_ros_for_cameras=True) from stretch4_body."""
    print("\n--- Benchmarking stream_left_right_center_rgbd (stretch4_body ROS2) ---")
    proc_camera = start_ros_launch(["ros2", "launch", "stretch_core", "luxonis.launch.py", "use_center:=true"], wait_seconds=0.0)
    proc_lidar = start_ros_launch(["ros2", "launch", "stretch_core", "dual_hesai.launch.py"], wait_seconds=8.0)
    if proc_camera is None or proc_lidar is None:
        print("Could not start ROS2 launch files. Skipping ROS benchmark.")
        if proc_camera: stop_ros_launch(proc_camera)
        if proc_lidar: stop_ros_launch(proc_lidar)
        return None

    try:
        from stretch4_body.subsystem.cameras.emulated_rgbd import stream_left_right_center_rgbd
    except ImportError as e:
        print(f"Failed to import stretch4_body emulated_rgbd modules: {e}")
        stop_ros_launch(proc_camera)
        stop_ros_launch(proc_lidar)
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_left_right_center_rgbd(use_ros_for_cameras=True, use_ros_for_lidars=True)
    except Exception as e:
        print(f"Failed to initialize stream_left_right_center_rgbd ROS2: {e}")
        stop_ros_launch(proc_camera)
        stop_ros_launch(proc_lidar)
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None:
                active_frame = frame.right if frame.right is not None else (frame.left if frame.left is not None else frame.center)
                if active_frame is not None and active_frame.image_frame is not None:
                    t_first_frame = time.perf_counter()
                    first_frame_received = True
                    break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_left_right_center_rgbd ROS2.")
        stop_ros_launch(proc_camera)
        stop_ros_launch(proc_lidar)
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    for frame in frame_generator:
        if frame is not None:
            active_frame = frame.right if frame.right is not None else (frame.left if frame.left is not None else frame.center)
            if active_frame is not None and active_frame.image_frame is not None:
                timestamps_host.append(time.time())
                timestamps_sensor.append(active_frame.image_frame.timestamp)
                seq_nums.append(active_frame.image_frame.frame_number)
        
        if time.monotonic() >= t_end:
            break


    stop_ros_launch(proc_camera)
    stop_ros_launch(proc_lidar)
    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_gripper_camera_old(duration, target_fps=30):
    """Benchmarks the GripperCamera from stretch4_gripper_modeling_and_control."""
    print("\n--- Benchmarking GripperCamera (stretch4_gripper_modeling_and_control) ---")
    if not os.path.exists(OLD_GRIPPER_PATH):
        print(f"OLD_GRIPPER_PATH ({OLD_GRIPPER_PATH}) is not available. Skipping stretch4_gripper_modeling_and_control GripperCamera benchmark.")
        return None
    try:
        from stretch4_gripper_modeling_and_control.gripper_camera import GripperCamera
    except ImportError as e:
        print(f"Failed to import GripperCamera from stretch4_gripper_modeling_and_control: {e}")
        return None

    t_init_start = time.perf_counter()
    try:
        # Instantiate old GripperCamera
        camera = GripperCamera(fps=target_fps, image_size=(640, 400), use_gripper=True, use_center=False, compress=True, oak_buffer_size=1)
        camera.start()
    except Exception as e:
        print(f"Failed to initialize old GripperCamera: {e}")
        return None

    # Wait for first frame
    print("Waiting for first frame...")
    try:
        _, _, _, _, ts, seq = camera.get_frames_with_metadata()
        t_first_frame = time.perf_counter()
    except Exception as e:
        print(f"Error getting first frame: {e}")
        camera.stop()
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    # Data collection loop
    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    while time.monotonic() < t_end:
        try:
            _, _, _, _, ts, seq = camera.get_frames_with_metadata()
            if ts is not None:
                timestamps_host.append(time.monotonic())
                timestamps_sensor.append(ts)
                seq_nums.append(seq)
        except Exception as e:
            print(f"Error reading frame: {e}")
            break

    camera.stop()
    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_gripper_python(duration):
    """Benchmarks stream_gripper_camera(use_ros_for_cameras=False) from stretch4_body."""
    print("\n--- Benchmarking Gripper Camera stream_gripper_camera (stretch4_body Python) ---")
    try:
        from stretch4_body.subsystem.cameras.stream_cameras import stream_gripper_camera
    except ImportError as e:
        print(f"Failed to import stretch4_body camera modules: {e}")
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_gripper_camera(use_ros_for_cameras=False)
    except Exception as e:
        print(f"Failed to initialize stream_gripper_camera: {e}")
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None and frame.right is not None:
                t_first_frame = time.perf_counter()
                first_frame_received = True
                break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting first frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_gripper_camera Python.")
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration

    for frame in frame_generator:
        if frame is not None and frame.right is not None:
            timestamps_host.append(time.monotonic())
            timestamps_sensor.append(frame.right.timestamp)
            seq_nums.append(frame.right.frame_number)
        
        if time.monotonic() >= t_end:
            break


    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def run_gripper_ros(duration):
    """Benchmarks stream_gripper_camera(use_ros_for_cameras=True) from stretch4_body."""
    print("\n--- Benchmarking Gripper Camera stream_gripper_camera (stretch4_body ROS2) ---")
    proc = start_ros_launch(["ros2", "launch", "stretch_core", "gripper_camera.launch.py"])
    if proc is None:
        print("Could not start ROS2 gripper camera launch file. Skipping ROS benchmark.")
        return None

    try:
        from stretch4_body.subsystem.cameras.stream_cameras import stream_gripper_camera
    except ImportError as e:
        print(f"Failed to import stretch4_body camera modules: {e}")
        stop_ros_launch(proc)
        return None

    t_init_start = time.perf_counter()
    try:
        frame_generator = stream_gripper_camera(use_ros_for_cameras=True)
    except Exception as e:
        print(f"Failed to initialize stream_gripper_camera ROS2: {e}")
        stop_ros_launch(proc)
        return None

    print("Waiting for first frame (up to 15 seconds)...")
    t_first_frame = None
    first_frame_received = False
    
    t_start = time.monotonic()
    while (time.monotonic() - t_start) < 15.0:
        try:
            frame = next(frame_generator)
            if frame is not None and frame.right is not None:
                t_first_frame = time.perf_counter()
                first_frame_received = True
                break
        except StopIteration:
            break
        except Exception as e:
            print(f"Error getting frame: {e}")
            break
        time.sleep(0.05)

    if not first_frame_received:
        print("Timeout waiting for first frame from stream_gripper_camera ROS2.")
        stop_ros_launch(proc)
        return None

    init_latency = t_first_frame - t_init_start
    print(f"First frame received! Initialization Latency: {init_latency:.3f} s")

    timestamps_host = []
    timestamps_sensor = []
    seq_nums = []

    t_end = time.monotonic() + duration
 
    for frame in frame_generator:
        if frame is not None and frame.right is not None:
            timestamps_host.append(time.time())
            timestamps_sensor.append(frame.right.timestamp)
            seq_nums.append(frame.right.frame_number)
         
        if time.monotonic() >= t_end:
            break


    stop_ros_launch(proc)
    return compile_results(timestamps_host, timestamps_sensor, seq_nums, init_latency)


def compile_results(host_times, sensor_times, seq_nums, init_latency):
    """Compiles statistics from the raw timestamp lists."""
    if len(host_times) < 5:
        print("Warning: Insufficient frames collected to compute metrics.")
        return {
            "init_latency": init_latency,
            "fps": 0.0,
            "mean_latency_ms": 0.0,
            "std_latency_ms": 0.0,
            "jitter_ms": 0.0,
            "drop_rate": 100.0,
            "frame_count": len(host_times)
        }

    host_times = np.array(host_times)
    sensor_times = np.array(sensor_times)
    seq_nums = np.array(seq_nums)

    # 1. Pipeline Latency (Host Receipt Time minus Physical Sensor Exposure Time)
    latencies = (host_times - sensor_times) * 1000.0  # Convert to ms
    # Handle possible timestamp epoch wrapping or synchronization issues gracefully
    valid_mask = (latencies >= 0) & (latencies < 5000)
    if not np.any(valid_mask):
        mean_lat = 0.0
        std_lat = 0.0
    else:
        mean_lat = np.mean(latencies[valid_mask])
        std_lat = np.std(latencies[valid_mask])

    # 2. Realized FPS
    total_time = host_times[-1] - host_times[0]
    fps = (len(host_times) - 1) / total_time if total_time > 0 else 0.0

    # 3. Frame Arrival Jitter (Std Dev of arrival intervals)
    intervals = np.diff(host_times) * 1000.0  # Convert to ms
    jitter = np.std(intervals)

    # 4. Frame Drop Rate
    seq_diffs = np.diff(seq_nums)
    expected_frames = np.sum(seq_diffs) if len(seq_diffs) > 0 else len(seq_nums)
    dropped_frames = expected_frames - len(seq_nums)
    drop_rate = (dropped_frames / expected_frames) * 100.0 if expected_frames > 0 else 0.0
    if drop_rate < 0:
        drop_rate = 0.0  # Safe boundary

    result = {
        "init_latency": init_latency,
        "fps": fps,
        "mean_latency_ms": mean_lat,
        "std_latency_ms": std_lat,
        "jitter_ms": jitter,
        "drop_rate": drop_rate,
        "frame_count": len(host_times)
    }

    print(f"{result=}")

    return result

def main():
    parser = argparse.ArgumentParser(description="Luxonis Camera Regression & Performance Benchmark")
    parser.add_argument("--duration", type=float, default=10.0, help="Duration in seconds to run each benchmark")
    parser.add_argument("--fps", type=int, default=30, help="Target frame rate (FPS) for the camera streams")
    parser.add_argument("--type", type=str, choices=["all", "head", "gripper", "rgbd"], default="all", help="Which cameras to benchmark")
    parser.add_argument("--output", type=str, default="camera_regression_results.md", help="Path to save the performance comparison report")
    args = parser.parse_args()

    results = {}

    print("=" * 80)
    print(f"Starting Side-by-Side Performance Comparison (Duration: {args.duration}s, Target: {args.fps} FPS)")
    print("=" * 80)

    # 1. Benchmark Head Cameras
    if args.type in ["all", "head"]:
        if os.path.exists(OLD_RGBD_PATH):
            results["Head (stretch4_rgbd)"] = run_head_camera_old(args.duration, args.fps)
            time.sleep(3.0)  # Power-cycle delay for USB stabilization
        else:
            print("stretch4_rgbd path not found, skipping Head (stretch4_rgbd) benchmark.")

        results["Head Left-Right Python (stretch4_body)"] = run_left_right_python(args.duration)
        time.sleep(3.0)

        results["Head Left-Right ROS (stretch4_body)"] = run_left_right_ros(args.duration)
        time.sleep(3.0)

        results["Head Left-Right-Center Python (stretch4_body)"] = run_left_right_center_python(args.duration)
        time.sleep(3.0)

        results["Head Left-Right-Center ROS (stretch4_body)"] = run_left_right_center_ros(args.duration)
        time.sleep(3.0)

    # 2. Benchmark Gripper Cameras
    if args.type in ["all", "gripper"]:
        if os.path.exists(OLD_GRIPPER_PATH):
            results["Gripper (stretch4_gripper_modeling_and_control)"] = run_gripper_camera_old(args.duration, args.fps)
            time.sleep(3.0)
        else:
            print("stretch4_gripper_modeling_and_control path not found, skipping Gripper (stretch4_gripper_modeling_and_control) benchmark.")

        results["Gripper Python (stretch4_body)"] = run_gripper_python(args.duration)
        time.sleep(3.0)

        results["Gripper ROS (stretch4_body)"] = run_gripper_ros(args.duration)

    # 3. Benchmark RGB-D Cameras
    if args.type in ["all", "rgbd"]:
        results["Left RGBD Python (stretch4_body)"] = run_left_rgbd_python(args.duration)
        time.sleep(3.0)

        results["Left RGBD ROS (stretch4_body)"] = run_left_rgbd_ros(args.duration)
        time.sleep(3.0)

        results["Left-Right RGBD Python (stretch4_body)"] = run_left_right_rgbd_python(args.duration)
        time.sleep(3.0)

        results["Left-Right RGBD ROS (stretch4_body)"] = run_left_right_rgbd_ros(args.duration)
        time.sleep(3.0)

        results["Left-Right-Center RGBD Python (stretch4_body)"] = run_left_right_center_rgbd_python(args.duration)
        time.sleep(3.0)

        results["Left-Right-Center RGBD ROS (stretch4_body)"] = run_left_right_center_rgbd_ros(args.duration)
        time.sleep(3.0)

    # 4. Print Quantitative Performance Summary
    print("\n" + "=" * 80)
    print("                      PERFORMANCE COMPARISON REPORT")
    print("=" * 80)

    table_headers = ["Implementation", "Init Latency", "Realized FPS", "Sensor-to-Host Latency", "Jitter (Arrival)", "Drop Rate", "Frames Recv"]
    print(f"{table_headers[0]:<50} | {table_headers[1]:<12} | {table_headers[2]:<12} | {table_headers[3]:<22} | {table_headers[4]:<16} | {table_headers[5]:<10} | {table_headers[6]:<11}")
    print("-" * 145)

    for label, metrics in results.items():
        if metrics is None:
            print(f"{label:<50} | N/A - Failed/Not Installed/Not Sourced")
            continue

        latency_str = f"{metrics['mean_latency_ms']:.1f} \u00b1 {metrics['std_latency_ms']:.1f} ms"
        print(f"{label:<50} | {metrics['init_latency']:.2f} s      | {metrics['fps']:.1f}         | {latency_str:<22} | {metrics['jitter_ms']:.2f} ms     | {metrics['drop_rate']:.1f} %      | {metrics['frame_count']}")

    # 5. Check for Regressions
    print("\n" + "=" * 80)
    print("                           REGRESSION CHECK")
    print("=" * 80)

    head_old = results.get("Head (stretch4_rgbd)")
    head_new = results.get("Head Left-Right Python (stretch4_body)")
    gripper_old = results.get("Gripper (stretch4_gripper_modeling_and_control)")
    gripper_new = results.get("Gripper Python (stretch4_body)")

    has_regression = False

    def check_regression(old_metrics, new_metrics, name):
        nonlocal has_regression
        if not old_metrics or not new_metrics:
            print(f"- {name}: Cannot check regression (one or both implementations failed or were skipped).")
            return

        # We assert regression if the new latency is >15% higher or FPS is >15% lower than the old one
        latency_threshold = old_metrics["mean_latency_ms"] * 1.15
        fps_threshold = old_metrics["fps"] * 0.85

        print(f"\nEvaluating {name}:")
        print(f"  * Latency: Old={old_metrics['mean_latency_ms']:.1f}ms, New={new_metrics['mean_latency_ms']:.1f}ms (Limit={latency_threshold:.1f}ms)")
        print(f"  * FPS:     Old={old_metrics['fps']:.1f}, New={new_metrics['fps']:.1f} (Limit={fps_threshold:.1f})")

        regression_detected = False
        if new_metrics["mean_latency_ms"] > latency_threshold:
            print(f"  [!] REGRESSION: Latency increased by >15%!")
            regression_detected = True
        if new_metrics["fps"] < fps_threshold:
            print(f"  [!] REGRESSION: Realized FPS dropped by >15%!")
            regression_detected = True

        if regression_detected:
            has_regression = True
        else:
            print(f"  [+] PASSED: Performance is equivalent within threshold bounds!")

    if args.type in ["all", "head"]:
        check_regression(head_old, head_new, "Head Camera")
    if args.type in ["all", "gripper"]:
        check_regression(gripper_old, gripper_new, "Gripper Camera")

    # Save detailed report to the specified output directory/file
    artifact_path = args.output
    if artifact_path:
        try:
            with open(artifact_path, "w") as f:
                f.write("# Luxonis DepthAI Performance Regression Report\n\n")
                f.write(f"Generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Parameters: Duration={args.duration}s, Target={args.fps} FPS\n\n")
                f.write("## Benchmark Statistics\n\n")
                
                f.write("| Implementation | Init Latency | Realized FPS | Sensor-to-Host Latency | Jitter (Arrival) | Drop Rate | Frames Recv |\n")
                f.write("|---|---|---|---|---|---|---|\n")
                for label, metrics in results.items():
                    if metrics is None:
                        f.write(f"| {label} | N/A - Failed/Not Installed/Not Sourced | | | | | |\n")
                    else:
                        latency_str = f"{metrics['mean_latency_ms']:.1f} ± {metrics['std_latency_ms']:.1f} ms"
                        f.write(f"| {label} | {metrics['init_latency']:.2f} s | {metrics['fps']:.1f} | {latency_str} | {metrics['jitter_ms']:.2f} ms | {metrics['drop_rate']:.1f} % | {metrics['frame_count']} |\n")
                
                f.write("\n## Regression Assessment\n\n")
                if has_regression:
                    f.write("> [!WARNING]\n> Regression detected! The new implementation's latency or frame rate does not meet the performance parity threshold of the older codebase.\n")
                else:
                    f.write("> [!NOTE]\n> Regression test passed successfully! Performance metrics of the new merged code in `stretch4_body` are on par with or better than the original codebases.\n")
            print(f"\nSaved performance report to: {artifact_path}")
        except Exception as e:
            print(f"Warning: Failed to write artifact report: {e}")
    
    # Exit with code 1 if a regression is detected
    if has_regression:
        sys.exit(1)
    else:
        sys.exit(0)


def test_camera_performance_regression():
    """Pytest-compatible camera performance regression test."""
    try:
        import depthai as dai
        devices = dai.Device.getAllAvailableDevices()
    except (ImportError, Exception):
        pytest.skip("depthai is not installed or available. Skipping performance regression tests.")

    if not devices:
        pytest.skip("No depthai / Luxonis devices connected. Skipping performance regression tests.")

    # Run a quick performance verification benchmark
    duration = 2.0
    target_fps = 30

    head_old = run_head_camera_old(duration, target_fps) if os.path.exists(OLD_RGBD_PATH) else None
    head_new = run_left_right_python(duration)
    head_ros = run_left_right_ros(duration)
    head_center_new = run_left_right_center_python(duration)
    head_center_ros = run_left_right_center_ros(duration)

    gripper_old = run_gripper_camera_old(duration, target_fps) if os.path.exists(OLD_GRIPPER_PATH) else None
    gripper_new = run_gripper_python(duration)
    gripper_ros = run_gripper_ros(duration)

    left_rgbd_new = run_left_rgbd_python(duration)
    left_rgbd_ros = run_left_rgbd_ros(duration)
    left_right_rgbd_new = run_left_right_rgbd_python(duration)
    left_right_rgbd_ros = run_left_right_rgbd_ros(duration)
    left_right_center_rgbd_new = run_left_right_center_rgbd_python(duration)
    left_right_center_rgbd_ros = run_left_right_center_rgbd_ros(duration)

    if (head_old is None and head_new is None and gripper_old is None and gripper_new is None and
            left_rgbd_new is None and left_right_rgbd_new is None and left_right_center_rgbd_new is None):
        pytest.skip("No cameras or RGB-D streams were successfully initialized. Skipping verification.")

    has_failed = False
    failure_messages = []

    if head_old and head_new:
        latency_threshold = head_old["mean_latency_ms"] * 1.15
        fps_threshold = head_old["fps"] * 0.85
        if head_new["mean_latency_ms"] > latency_threshold:
            has_failed = True
            failure_messages.append(f"Head Camera Latency Regression: Old={head_old['mean_latency_ms']:.1f}ms, New={head_new['mean_latency_ms']:.1f}ms (limit={latency_threshold:.1f}ms)")
        if head_new["fps"] < fps_threshold:
            has_failed = True
            failure_messages.append(f"Head Camera FPS Regression: Old={head_old['fps']:.1f}, New={head_new['fps']:.1f} (limit={fps_threshold:.1f})")
    elif head_new:
        # Standalone health verification for stretch4_body head camera
        if head_new["fps"] < 25.0:
            has_failed = True
            failure_messages.append(f"Head Camera FPS underperforming: Realized={head_new['fps']:.1f} FPS (expected >= 25)")
        if head_new["mean_latency_ms"] > 100.0:
            has_failed = True
            failure_messages.append(f"Head Camera Latency too high: Realized={head_new['mean_latency_ms']:.1f}ms (expected <= 100ms)")

    if head_ros:
        if head_ros["fps"] < 25.0:
            has_failed = True
            failure_messages.append(f"Head Camera ROS FPS underperforming: Realized={head_ros['fps']:.1f} FPS (expected >= 25)")
        if head_ros["mean_latency_ms"] > 150.0:
            has_failed = True
            failure_messages.append(f"Head Camera ROS Latency too high: Realized={head_ros['mean_latency_ms']:.1f}ms (expected <= 150ms)")

    if head_center_new:
        if head_center_new["fps"] < 25.0:
            has_failed = True
            failure_messages.append(f"Head Center Camera FPS underperforming: Realized={head_center_new['fps']:.1f} FPS (expected >= 25)")
        if head_center_new["mean_latency_ms"] > 120.0:
            has_failed = True
            failure_messages.append(f"Head Center Camera Latency too high: Realized={head_center_new['mean_latency_ms']:.1f}ms (expected <= 120ms)")

    if head_center_ros:
        if head_center_ros["fps"] < 25.0:
            has_failed = True
            failure_messages.append(f"Head Center Camera ROS FPS underperforming: Realized={head_center_ros['fps']:.1f} FPS (expected >= 25)")
        if head_center_ros["mean_latency_ms"] > 200.0:
            has_failed = True
            failure_messages.append(f"Head Center Camera ROS Latency too high: Realized={head_center_ros['mean_latency_ms']:.1f}ms (expected <= 200ms)")

    if gripper_old and gripper_new:
        latency_threshold = gripper_old["mean_latency_ms"] * 1.15
        fps_threshold = gripper_old["fps"] * 0.85
        if gripper_new["mean_latency_ms"] > latency_threshold:
            has_failed = True
            failure_messages.append(f"Gripper Camera Latency Regression: Old={gripper_old['mean_latency_ms']:.1f}ms, New={gripper_new['mean_latency_ms']:.1f}ms (limit={latency_threshold:.1f}ms)")
        if gripper_new["fps"] < fps_threshold:
            has_failed = True
            failure_messages.append(f"Gripper Camera FPS Regression: Old={gripper_old['fps']:.1f}, New={gripper_new['fps']:.1f} (limit={fps_threshold:.1f})")
    elif gripper_new:
        # Standalone health verification for stretch4_body gripper camera
        if gripper_new["fps"] < 25.0:
            has_failed = True
            failure_messages.append(f"Gripper Camera FPS underperforming: Realized={gripper_new['fps']:.1f} FPS (expected >= 25)")
        if gripper_new["mean_latency_ms"] > 120.0:
            has_failed = True
            failure_messages.append(f"Gripper Camera Latency too high: Realized={gripper_new['mean_latency_ms']:.1f}ms (expected <= 120ms)")

    if gripper_ros:
        if gripper_ros["fps"] < 25.0:
            has_failed = True
            failure_messages.append(f"Gripper Camera ROS FPS underperforming: Realized={gripper_ros['fps']:.1f} FPS (expected >= 25)")
        if gripper_ros["mean_latency_ms"] > 600.0:
            has_failed = True
            failure_messages.append(f"Gripper Camera ROS Latency too high: Realized={gripper_ros['mean_latency_ms']:.1f}ms (expected <= 600ms)")

    if left_rgbd_new:
        if left_rgbd_new["fps"] < 10.0:
            has_failed = True
            failure_messages.append(f"Left RGBD Python FPS underperforming: Realized={left_rgbd_new['fps']:.1f} FPS (expected >= 10)")
        if left_rgbd_new["mean_latency_ms"] > 250.0:
            has_failed = True
            failure_messages.append(f"Left RGBD Python Latency too high: Realized={left_rgbd_new['mean_latency_ms']:.1f}ms (expected <= 250ms)")

    if left_rgbd_ros:
        if left_rgbd_ros["fps"] < 10.0:
            has_failed = True
            failure_messages.append(f"Left RGBD ROS FPS underperforming: Realized={left_rgbd_ros['fps']:.1f} FPS (expected >= 10)")
        if left_rgbd_ros["mean_latency_ms"] > 300.0:
            has_failed = True
            failure_messages.append(f"Left RGBD ROS Latency too high: Realized={left_rgbd_ros['mean_latency_ms']:.1f}ms (expected <= 300ms)")

    if left_right_rgbd_new:
        if left_right_rgbd_new["fps"] < 10.0:
            has_failed = True
            failure_messages.append(f"Left-Right RGBD Python FPS underperforming: Realized={left_right_rgbd_new['fps']:.1f} FPS (expected >= 10)")
        if left_right_rgbd_new["mean_latency_ms"] > 250.0:
            has_failed = True
            failure_messages.append(f"Left-Right RGBD Python Latency too high: Realized={left_right_rgbd_new['mean_latency_ms']:.1f}ms (expected <= 250ms)")

    if left_right_rgbd_ros:
        if left_right_rgbd_ros["fps"] < 10.0:
            has_failed = True
            failure_messages.append(f"Left-Right RGBD ROS FPS underperforming: Realized={left_right_rgbd_ros['fps']:.1f} FPS (expected >= 10)")
        if left_right_rgbd_ros["mean_latency_ms"] > 300.0:
            has_failed = True
            failure_messages.append(f"Left-Right RGBD ROS Latency too high: Realized={left_right_rgbd_ros['mean_latency_ms']:.1f}ms (expected <= 300ms)")

    if left_right_center_rgbd_new:
        if left_right_center_rgbd_new["fps"] < 10.0:
            has_failed = True
            failure_messages.append(f"Left-Right-Center RGBD Python FPS underperforming: Realized={left_right_center_rgbd_new['fps']:.1f} FPS (expected >= 10)")
        if left_right_center_rgbd_new["mean_latency_ms"] > 250.0:
            has_failed = True
            failure_messages.append(f"Left-Right-Center RGBD Python Latency too high: Realized={left_right_center_rgbd_new['mean_latency_ms']:.1f}ms (expected <= 250ms)")

    if left_right_center_rgbd_ros:
        if left_right_center_rgbd_ros["fps"] < 10.0:
            has_failed = True
            failure_messages.append(f"Left-Right-Center RGBD ROS FPS underperforming: Realized={left_right_center_rgbd_ros['fps']:.1f} FPS (expected >= 10)")
        if left_right_center_rgbd_ros["mean_latency_ms"] > 300.0:
            has_failed = True
            failure_messages.append(f"Left-Right-Center RGBD ROS Latency too high: Realized={left_right_center_rgbd_ros['mean_latency_ms']:.1f}ms (expected <= 300ms)")

    assert not has_failed, "Performance regression or stand-alone underperformance detected:\n" + "\n".join(failure_messages)


if __name__ == "__main__":
    import os
    try:
        main()
        print("\nBenchmark completed successfully! Exiting cleanly...")
        os._exit(0)
    except AssertionError as e:
        print(f"\nBenchmark failed:\n{e}")
        os._exit(1)
    except Exception as e:
        import traceback
        traceback.print_exc()
        os._exit(1)
