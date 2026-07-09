#!/usr/bin/env python3
"""
Stretch Monitor (Continuous Connectivity Monitor)
Author: Hello Robot Inc.

Monitors the connection status of:
1. /dev/tty* devices (Motors, Arduino)
2. Head Cameras (OAK-D / Luxonis, 3 sensors: RGB, Left, Right)
3. Gripper Cameras (OAK-D / Luxonis, 2 sensors: Left, Right)
4. Head Lidars (Hesai)
5. Line Sensors (Pixart J3, 6 sensors)
6. Kernel Logs (dmesg) for USB disconnects

Modes:
- Active (Default): Claims devices, checks streams/FPS.
- Passive (--passive): Checks device presence only. Used when running side-by-side visualization.
- Visualize (--visualize): Shows camera feed in Active mode.
"""

import time
import shutil
import sys
import os
import signal
import subprocess
import threading
import queue
import argparse
import logging
import csv
from datetime import datetime
import select
import socket
import glob

import click
from rich.console import Console
from rich.live import Live
from rich.table import Table
from rich.layout import Layout
from rich.panel import Panel
from rich.text import Text

from stretch4_body.core.factory.hello_device_utils import find_tty_devices
# Try importing Feetech
try:
    from stretch4_body.core.feetech.feetech_SM_servo import FeetechSMServo
    from stretch4_body.core.feetech.port_handler import PortHandler
    from stretch4_body.core.feetech.sms_sts import sms_sts
except ImportError:
    FeetechSMServo = None
    PortHandler = None
    sms_sts = None

# Try importing DepthAI
try:
    import depthai as dai
except ImportError:
    dai = None
    if os.geteuid() == 0:
        print("WARNING: 'depthai' module not found. If installed as user, try running with 'sudo -E'.")

# Try importing Hesai lidar decoder
try:
    from stretch4_body.utils.hesai_lidar_utils import HesaiJT128Decoder
except ImportError:
    HesaiJT128Decoder = None

# Try importing rerun for lidar visualization
try:
    import rerun as rr
    RERUN_AVAILABLE = True
except ImportError:
    rr = None
    RERUN_AVAILABLE = False

# Try importing Line Sensor reader
try:
    from stretch4_body.subsystem.line_sensor.pixart_j3_reader import PixartJ3Reader
except ImportError:
    PixartJ3Reader = None

LINE_SENSOR_COUNT = 6

# Constants
LIDAR_DEVICES = {
    'Left Lidar': '192.168.1.202',
    'Right Lidar': '192.168.1.201'
}
LIDAR_PORTS = [2368, 2378]
FEETECH_IDS = [20, 21, 22, 23]
FEETECH_BAUD = 1000000

# Lidar decoder configs (matches test_BRI_head_lidars.py)
_fleet_dir = os.environ.get('HELLO_FLEET_PATH', '')
_fleet_id  = os.environ.get('HELLO_FLEET_ID', '')
_cal_dir   = os.path.join(_fleet_dir, _fleet_id, 'calibration_hesais')
LIDAR_CONFIGS = [
    {
        'name':             'Left Lidar',
        'ip':               '192.168.1.202',
        'udp_port':         2378,
        'correction_file':  os.path.join(_cal_dir, 'left_lidar_calibration.dat'),
        'rr_label':         'world/left_lidar',
        'color':            [0, 180, 255],
    },
    {
        'name':             'Right Lidar',
        'ip':               '192.168.1.201',
        'udp_port':         2368,
        'correction_file':  os.path.join(_cal_dir, 'right_lidar_calibration.dat'),
        'rr_label':         'world/right_lidar',
        'color':            [255, 120, 0],
    },
]

# Gripper Camera Configuration (2-sensor OAK-D, matches test_FAB_gripper_cameras.py)
GRIPPER_CAM_CONFIGS = [
    {
        'name': 'left', 'dev_key': 'GRIP_Left',
        'socket': dai.CameraBoardSocket.CAM_C,
        'width': 1280, 'height': 800,
        'fps': 12,
        'resize_target': (640, 400)
    },
    {
        'name': 'right', 'dev_key': 'GRIP_Right',
        'socket': dai.CameraBoardSocket.CAM_B,
        'width': 1280, 'height': 800,
        'fps': 12,
        'resize_target': (640, 400)
    }
] if dai else []

class DmesgMonitor:
    def __init__(self):
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.log_lines = []

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()

    def _monitor_loop(self):
        # Clear buffer first? sudo dmesg -c? No, just follow from now.
        # We use Popen to follow dmesg
        try:
            # We use -w (follow) if available, or just tail?
            # 'dmesg -w' follows.
            proc = subprocess.Popen(['dmesg', '-w'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            poll_obj = select.poll()
            poll_obj.register(proc.stdout, select.POLLIN)
            
            while not self.stop_event.is_set():
                if poll_obj.poll(500): # 0.5s timeout
                    line = proc.stdout.readline()
                    if line:
                        self._process_line(line)
        except Exception as e:
            self.q.put(f"[Monitor Error] {e}")

    def _process_line(self, line):
        # Filter for relevant keywords
        keywords = ['usb', 'tty', 'disconnect', 'error', 'failed', 'device']
        if any(k in line.lower() for k in keywords):
            # Clean timestamp if possible, but raw is fine
            msg = line.strip()
            self.q.put(msg)
            # Keep defined buffer
            if len(self.log_lines) > 50:
                self.log_lines.pop(0)
            self.log_lines.append(msg)

    def get_recent_logs(self):
        # Empty queue to local list
        while not self.q.empty():
            try:
                line = self.q.get_nowait()
                if len(self.log_lines) > 50:
                     self.log_lines.pop(0)
                self.log_lines.append(line)
            except queue.Empty:
                break
        return self.log_lines


# -----------------------------------------------------------------------------
# Device Monitor Class
# -----------------------------------------------------------------------------
class DeviceMonitor:
    def __init__(self, passive=False, visualize=False):
        self.passive = passive
        self.visualize = visualize
        self.devices = {}  # {name: {'type': str, 'status': str, 'details': str, 'errors': int}}
        self.devices = {}  # {name: {'type': str, 'status': str, 'details': str, 'errors': int, 'prev_status': str}}
        self.start_time = time.time()
        
        # Output Directory
        self.output_dir = os.path.join(
            os.path.expanduser('~'), 'stretch_user', 'log', 'stretch_monitor_logs'
        )
        os.makedirs(self.output_dir, exist_ok=True)

        filename = f"stretch_monitor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.log_file = os.path.join(self.output_dir, filename)
        
        self.csv_file = open(self.log_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp', 'Event', 'Details'])
        
        # Camera Pipeline - Head (Active only)
        self.pipeline = None
        self.cam_queues = {}
        self.cam_device = None
        
        # Camera Pipeline - Gripper (Active only)
        self.grip_pipeline = None
        self.grip_cam_queues = {}
        self.grip_cam_device = None
        
        # Lidar Sockets (Active only)
        self.lidar_sockets = []
        
        # Lidar Decoders + Visualization (Active only)
        self.lidar_decoders = []       # list of HesaiJT128Decoder instances
        self.lidar_threads = []        # background receive threads
        self.lidar_stop_event = threading.Event()
        self.lidar_mutex = threading.Lock()
        # {name: {'points': np.ndarray or None, 'frame_count': int}}
        self.lidar_state = {}
        self.rerun_initialized = False
        self._lidar_start_time = None  # time when lidar decoders first started
        
        # Feetech Servos (Active only)
        # {id: FeetechSMServo_Instance}
        self.feetech_servos = {}
        self.feetech_port_handler = None
        
        # Line Sensor (Active only)
        self.line_sensor_reader = None
        self.line_sensor_thread = None
        self.line_sensor_stop = threading.Event()

    def log(self, event, details):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.csv_writer.writerow([timestamp, event, details])
        self.csv_file.flush()

    def discover_devices(self):
        """Initial discovery of devices to monitor."""
        try:
            # 1. TTY Devices via /dev/hello-* symlinks
            print("Scanning /dev/hello-* devices...")
            
            # We prefer checking the symlinks because they represent the logical devices
            hello_devs = glob.glob('/dev/hello-*')
            
            # Also get detailed info to cross-reference if needed
            tty_info_map = find_tty_devices() # path -> info
            
            for symlink_path in hello_devs:
                if 'camera' in symlink_path: continue # Skip camera USB symlinks (handled by DepthAI)
                
                try:
                    real_path = os.path.realpath(symlink_path)
                    
                    # Check if it looks like a TTY or known device type
                    if 'tty' in real_path or 'usb' in real_path:
                        name = os.path.basename(symlink_path)
                        self.devices[symlink_path] = {
                            'name': name,
                            'type': 'TTY',
                            'status': 'CONNECTED',
                            'details': real_path,
                            'errors': 0,
                            'details': real_path,
                            'errors': 0,
                            'last_seen': time.time(),
                            'prev_status': 'UNKNOWN'
                        }
                except Exception:
                    pass
            
            # 1.1 Feetech Wrist Special Handling
            feetech_path = '/dev/hello-feetech-wrist'
            if os.path.exists(feetech_path) and FeetechSMServo:
                # Add individual servo entries if active
                if not self.passive:
                    for fid in FEETECH_IDS:
                        self.devices[f"Feetech_ID{fid}"] = {
                            'name': f"Feetech Wrist {fid}",
                            'type': 'MOTOR',
                            'status': 'WAITING',
                            'details': feetech_path,
                            'type': 'MOTOR',
                            'status': 'WAITING',
                            'details': feetech_path,
                            'errors': 0,
                            'prev_status': 'UNKNOWN'
                        }

        except Exception as e:
            self.log("ERROR", f"TTY Discovery Failed: {e}")

        # 2. Cameras (DepthAI)
        if dai:
            print("Scanning Cameras...")
            try:
                device_infos = dai.Device.getAllAvailableDevices()
                found = len(device_infos) > 0
                
                # Identify head vs gripper cameras by sensor count
                head_info = None
                grip_info = None
                for info in device_infos:
                    try:
                        temp_dev = dai.Device(info)
                        sensor_count = len(temp_dev.getConnectedCameras())
                        if sensor_count == 3 and head_info is None:
                            head_info = info
                        elif sensor_count == 2 and grip_info is None:
                            grip_info = info
                        temp_dev.close()
                    except Exception:
                        pass
                
                self._head_cam_info = head_info
                self._grip_cam_info = grip_info
                
                # Active mode: We will track individual streams
                # Passive mode: We track the main USB device
                if self.passive:
                    self.devices['OAK-D'] = {
                        'name': 'OAK-D Head',
                        'type': 'CAMERA',
                        'status': 'CONNECTED' if head_info else 'MISSING',
                        'details': f"{len(device_infos)} Found",
                        'errors': 0 if head_info else 1,
                        'prev_status': 'UNKNOWN'
                    }
                    self.devices['OAK-D-Grip'] = {
                        'name': 'OAK-D Gripper',
                        'type': 'CAMERA',
                        'status': 'CONNECTED' if grip_info else 'MISSING',
                        'details': f"{len(device_infos)} Found",
                        'errors': 0 if grip_info else 1,
                        'prev_status': 'UNKNOWN'
                    }
                else:
                    # Active: Pre-populate the 3 head camera sensors
                    for cam_name in ['RGB', 'Left', 'Right']:
                        self.devices[f'CAM_{cam_name}'] = {
                            'name': f'Head {cam_name}',
                            'type': 'CAMERA',
                            'status': 'WAITING' if head_info else 'MISSING',
                            'details': 'Stream',
                            'errors': 0 if head_info else 1,
                            'fps': 0.0,
                            'frame_count': 0,
                            'last_fps_time': time.time(),
                            'prev_status': 'UNKNOWN'
                        }
                    # Active: Pre-populate the 2 gripper camera sensors
                    for conf in GRIPPER_CAM_CONFIGS:
                        dev_key = conf['dev_key']
                        self.devices[dev_key] = {
                            'name': f'Gripper {conf["name"].title()}',
                            'type': 'CAMERA',
                            'status': 'WAITING' if grip_info else 'MISSING',
                            'details': 'Stream',
                            'errors': 0 if grip_info else 1,
                            'fps': 0.0,
                            'frame_count': 0,
                            'last_fps_time': time.time(),
                            'prev_status': 'UNKNOWN'
                        }
            except Exception as e:
                 self.log("ERROR", f"Camera Discovery Failed: {e}")

        # 3. Lidars (Hesai)
        print("Ping Lidars...")
        for name, ip in LIDAR_DEVICES.items():
            self.devices[name] = {
                'name': name,
                'type': 'LIDAR',
                'status': 'UNKNOWN',
                'details': ip,
                'errors': 0,
                'packets': 0,
                'packet_count': 0,
                'last_packet_time': time.time(),
                'prev_status': 'UNKNOWN'
            }

        # 4. Line Sensors (Pixart J3)
        if PixartJ3Reader and not self.passive:
            print("Scanning Line Sensors...")
            for i in range(LINE_SENSOR_COUNT):
                self.devices[f'LineSensor_{i}'] = {
                    'name': f'Line Sensor {i}',
                    'type': 'LINE_SENSOR',
                    'status': 'WAITING',
                    'details': 'Pixart J3',
                    'errors': 0,
                    'prev_status': 'UNKNOWN'
                }

    def start_active_monitoring(self):
        """Start active streams (Cameras, Lidar Sockets, Motors)."""
        if self.passive:
            return
            
        # Start Feetech Motors
        feetech_path = '/dev/hello-feetech-wrist'
        if os.path.exists(feetech_path) and FeetechSMServo:
            try:
                print(f"Opening Feetech port {feetech_path}...")
                self.feetech_port_handler = PortHandler(feetech_path)
                if self.feetech_port_handler.openPort():
                    if self.feetech_port_handler.setBaudRate(FEETECH_BAUD):
                        for fid in FEETECH_IDS:
                            try:
                                # Use shared port handler
                                m = FeetechSMServo(fid, feetech_path, port_handler=self.feetech_port_handler, baud=FEETECH_BAUD)
                                # Manual setup as per stretch_feetech_monitor.py
                                m.packet_handler = sms_sts(m.port_handler)
                                m.hw_valid = True # Assume valid if port is open, verification happens on read
                                
                                # Try a ping or read to verify?
                                # stretch_feetech_monitor just appends and reads later.
                                # Let's try to ping to be sure it's there?
                                # stretch_feetech_monitor does ping in main but not in the loop.
                                # Let's just add it on faith and let the loop check it.
                                
                                self.feetech_servos[fid] = m
                                if f"Feetech_ID{fid}" in self.devices:
                                     self.devices[f"Feetech_ID{fid}"]['status'] = 'ALIVE'
                            except Exception as e:
                                 self.log("ERROR", f"Feetech Init ID {fid}: {e}")
                    else:
                        self.log("ERROR", f"Failed to set baud rate {FEETECH_BAUD} for Feetech")
                else:
                    self.log("ERROR", f"Failed to open Feetech port {feetech_path}")
            except Exception as e:
                self.log("ERROR", f"Feetech Port Init Failed: {e}")

        # Start Head Camera Pipeline
        if dai and any(d['status'] != 'MISSING' for k, d in self.devices.items() if 'CAM_' in k):
            try:
                head_info = getattr(self, '_head_cam_info', None)
                if not head_info:
                    raise RuntimeError("No head camera (3-sensor) device found")
                
                self.cam_device = dai.Device(head_info)
                
                # 2. Create Pipeline attached to device
                self.pipeline = dai.Pipeline(defaultDevice=self.cam_device)
                
                # 3. Create Nodes for each camera
                # Configuration from visualize_head_cameras.py
                cam_configs = [
                    {
                        'name': 'rgb', 'dev_key': 'CAM_RGB',
                        'socket': dai.CameraBoardSocket.CAM_A,
                        'width': 4032, 'height': 3040,
                        'resize_target': (1024, 768) # Aspect ratio match roughly (4:3)
                    },
                    {
                        'name': 'left', 'dev_key': 'CAM_Left',
                        'socket': dai.CameraBoardSocket.CAM_C,
                        'width': 1920, 'height': 1200,
                        'resize_target': (800, 500) # Aspect ratio match (16:10)
                    },
                    {
                        'name': 'right', 'dev_key': 'CAM_Right',
                        'socket': dai.CameraBoardSocket.CAM_B,
                        'width': 1920, 'height': 1200,
                        'resize_target': (800, 500)
                    }
                ]

                for conf in cam_configs:
                    dev_key = conf['dev_key']
                    # Check if tracked
                    if dev_key not in self.devices: continue
                    
                    try:
                        # Use high-level Camera node
                        cam_node = self.pipeline.create(dai.node.Camera)
                        cam_node.setSensorType(dai.CameraSensorType.COLOR)
                        
                        # Use build() as per visualize_head_cameras.py
                        cam_node.build(boardSocket=conf['socket'], sensorFps=10)
                        
                        # Request logic matching visualize_head_cameras.py but requesting smaller size directly
                        # We use the resize_target as the output size to keep bandwidth low
                        cam_out = cam_node.requestOutput(
                            size=conf['resize_target'],
                            fps=10,
                            type=dai.ImgFrame.Type.NV12,
                            resizeMode=dai.ImgResizeMode.CROP,
                            enableUndistortion=False
                        )
                        
                        # Create Queue directly from output
                        self.cam_queues[conf['name']] = cam_out.createOutputQueue(maxSize=4, blocking=False)
                        
                    except Exception as node_ex:
                         self.log("WARN", f"Failed to create node for {dev_key}: {node_ex}")
                         self.devices[dev_key]['status'] = 'ERROR'

                # 4. Start Pipeline
                self.pipeline.start()
                self.log("INFO", "Head Camera Pipeline Started")
                
            except Exception as e:
                self.log("ERROR", f"Failed to start head camera pipeline: {e}")
                for k in ['CAM_RGB', 'CAM_Left', 'CAM_Right']:
                     if k in self.devices:
                        self.devices[k]['status'] = 'ERROR'
                        self.devices[k]['details'] = str(e)

        # Start Gripper Camera Pipeline
        if dai and any(d['status'] != 'MISSING' for k, d in self.devices.items() if 'GRIP_' in k):
            try:
                grip_info = getattr(self, '_grip_cam_info', None)
                if not grip_info:
                    raise RuntimeError("No gripper camera (2-sensor) device found")
                
                self.grip_cam_device = dai.Device(grip_info)
                self.grip_pipeline = dai.Pipeline(defaultDevice=self.grip_cam_device)

                for conf in GRIPPER_CAM_CONFIGS:
                    dev_key = conf['dev_key']
                    if dev_key not in self.devices:
                        continue
                    try:
                        cam_node = self.grip_pipeline.create(dai.node.Camera)
                        cam_node.setSensorType(dai.CameraSensorType.COLOR)
                        cam_node.build(boardSocket=conf['socket'], sensorFps=conf['fps'])

                        cam_out = cam_node.requestOutput(
                            size=(conf['width'], conf['height']),
                            fps=conf['fps'],
                            type=dai.ImgFrame.Type.NV12,
                            resizeMode=dai.ImgResizeMode.CROP,
                            enableUndistortion=False
                        )

                        self.grip_cam_queues[conf['name']] = cam_out.createOutputQueue(maxSize=4, blocking=False)
                    except Exception as node_ex:
                        self.log("WARN", f"Failed to create gripper node for {dev_key}: {node_ex}")
                        self.devices[dev_key]['status'] = 'ERROR'

                self.grip_pipeline.start()
                self.log("INFO", "Gripper Camera Pipeline Started")

            except Exception as e:
                self.log("ERROR", f"Failed to start gripper camera pipeline: {e}")
                for conf in GRIPPER_CAM_CONFIGS:
                    dev_key = conf['dev_key']
                    if dev_key in self.devices:
                        self.devices[dev_key]['status'] = 'ERROR'
                        self.devices[dev_key]['details'] = str(e)

        # Start Lidar Decoders (for proper point cloud decoding + visualization)
        if HesaiJT128Decoder:
            for cfg in LIDAR_CONFIGS:
                try:
                    dec = HesaiJT128Decoder(
                        udp_port=cfg['udp_port'],
                        lidar_ip=cfg['ip'],
                        correction_file=cfg['correction_file'],
                    )
                    dec.open()
                    self.lidar_decoders.append(dec)
                    self.lidar_state[cfg['name']] = {'points': None, 'frame_count': 0}

                    # Background receive thread (same pattern as hesai_lidar_utils)
                    def recv_worker(decoder, config, state, mutex, stop_event):
                        while not stop_event.is_set():
                            xyz = decoder.recv_frame(timeout=0.1)
                            if xyz is not None and len(xyz) > 0:
                                with mutex:
                                    state[config['name']]['points'] = xyz
                                    state[config['name']]['frame_count'] += 1

                    t = threading.Thread(
                        target=recv_worker,
                        args=(dec, cfg, self.lidar_state, self.lidar_mutex, self.lidar_stop_event),
                        daemon=True,
                    )
                    t.start()
                    self.lidar_threads.append(t)
                    self.log("INFO", f"Lidar decoder started for {cfg['name']} ({cfg['ip']}, port {cfg['udp_port']})")
                except Exception as e:
                    self.log("ERROR", f"Failed to start lidar decoder for {cfg['name']}: {e}")
            # Record when decoders first started for the NO DATA grace window
            if self.lidar_decoders and self._lidar_start_time is None:
                self._lidar_start_time = time.time()
        else:
            # Fallback: raw socket packet counting (original behavior)
            for port in LIDAR_PORTS:
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.setblocking(0)
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    s.bind(('0.0.0.0', port))
                    self.lidar_sockets.append(s)
                except Exception as e:
                    self.log("ERROR", f"Failed to bind Lidar port {port}: {e}")

        # Init Rerun for lidar visualization
        if self.visualize and RERUN_AVAILABLE and self.lidar_decoders:
            try:
                rr.init('stretch_monitor_lidars', spawn=True)
                rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
                rr.log(
                    'world/xyz',
                    rr.Arrows3D(
                        vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                    ),
                    static=True,
                )
                self.rerun_initialized = True
                self.log("INFO", "Rerun lidar viewer launched")
            except Exception as e:
                self.log("WARN", f"Failed to initialize rerun: {e}")

        # Start Line Sensor reader with background polling thread
        if PixartJ3Reader and any(d['type'] == 'LINE_SENSOR' for d in self.devices.values()):
            try:
                self.line_sensor_reader = PixartJ3Reader(verbose=False)
                if self.line_sensor_reader.startup():
                    self.log("INFO", "Line Sensor reader started")

                    def _line_sensor_poll(reader, stop_event):
                        while not stop_event.is_set():
                            try:
                                if not reader.is_valid:
                                    # Reader became invalid (JSON parse error, out-of-order
                                    # frames, serial error, etc). Attempt recovery.
                                    try:
                                        if hasattr(reader, 'ser') and reader.ser.is_open:
                                            reader.ser.close()
                                    except Exception:
                                        pass
                                    time.sleep(1.0)  # back off before retry
                                    if stop_event.is_set():
                                        break
                                    try:
                                        reader.startup()
                                    except Exception:
                                        pass
                                    continue
                                reader.step()
                            except Exception:
                                pass
                            time.sleep(0.004)  # ~250 Hz poll rate

                    self.line_sensor_thread = threading.Thread(
                        target=_line_sensor_poll,
                        args=(self.line_sensor_reader, self.line_sensor_stop),
                        daemon=True,
                    )
                    self.line_sensor_thread.start()
                else:
                    self.log("ERROR", "Line Sensor reader startup failed")
                    self.line_sensor_reader = None
                    for i in range(LINE_SENSOR_COUNT):
                        key = f'LineSensor_{i}'
                        if key in self.devices:
                            self.devices[key]['status'] = 'ERROR'
            except Exception as e:
                self.log("ERROR", f"Line Sensor init failed: {e}")
                self.line_sensor_reader = None

    print("Starting Main Loop Check")

    def update_status(self, dev, new_status, error_msg=None):
        """Helper to update status and log transitions."""
        old_status = dev.get('prev_status', 'UNKNOWN')
        
        # Determine if we should log
        # 1. Status Changed
        if new_status != old_status:
            # Filter somewhat noisy "STREAMING (X FPS)" updates if they are just value changes
            # But here we want to see if it STOPS streaming
            
            is_streaming_update = "STREAMING" in new_status and "STREAMING" in old_status
            
            if not is_streaming_update:
                if "ERROR" in new_status or "MISSING" in new_status or "DISCONNECTED" in new_status or "NO DATA" in new_status or "LOST" in new_status:
                    log_type = "ERROR"
                    if "DISCONNECTED" in new_status: log_type = "DISCONNECT"
                    elif "NO DATA" in new_status: log_type = "NO_DATA"
                    
                    self.log(log_type, f"{dev['name']}: {old_status} -> {new_status} {error_msg if error_msg else ''}")
                elif "CONNECTED" in new_status or "ALIVE" in new_status or "STREAMING" in new_status or "PING OK" in new_status or "DETECTED" in new_status:
                    self.log("RECONNECT", f"{dev['name']}: {old_status} -> {new_status}")
                else:
                    self.log("INFO", f"{dev['name']}: {old_status} -> {new_status}")
        
        # Update state
        dev['status'] = new_status
        dev['prev_status'] = new_status
        if error_msg:
             dev['errors'] += 1

    def check_devices(self):
        """Main loop check function."""
        current_time = time.time()

        # 1. TTY Checks
        for path, dev in self.devices.items():
            if dev['type'] == 'TTY':
                try:
                    if os.path.exists(path):
                        self.update_status(dev, 'CONNECTED')
                        dev['last_seen'] = current_time
                    else: 
                        self.update_status(dev, 'DISCONNECTED', error_msg="Device path not found")
                except Exception as e:
                    self.update_status(dev, 'ERROR', error_msg=str(e))

        # 1.5 Feetech Checks
        if not self.passive:
            for fid, motor in self.feetech_servos.items():
                dev_key = f"Feetech_ID{fid}"
                if dev_key not in self.devices: continue
                dev = self.devices[dev_key]
                try:
                    # Lightweight check: get position
                    pos = motor.get_pos()
                    if pos is not None:
                         self.update_status(dev, 'ALIVE')
                    else:
                         self.update_status(dev, 'LOST', error_msg="Read returned None")
                except Exception as e:
                    self.update_status(dev, 'ERROR', error_msg=str(e))


        # 2. Camera Checks
        if dai:
            if self.passive:
                for dev_key in ['OAK-D', 'OAK-D-Grip']:
                    dev = self.devices.get(dev_key)
                    if dev:
                        try:
                            device_infos = dai.Device.getAllAvailableDevices()
                            if len(device_infos) > 0:
                                self.update_status(dev, 'CONNECTED')
                            else:
                                self.update_status(dev, 'DISCONNECTED')
                        except Exception as e:
                            self.update_status(dev, 'ERROR', error_msg=str(e))
            else:
                 # Active: Check head camera queues for RGB, Left, Right
                 try:
                     if self.cam_device and not self.cam_device.isClosed():
                         for stream_name, dev_key in [('rgb', 'CAM_RGB'), ('left', 'CAM_Left'), ('right', 'CAM_Right')]:
                             if dev_key not in self.devices: continue
                             
                             dev = self.devices[dev_key]
                             q = self.cam_queues.get(stream_name)
                             
                             if q and q.has():
                                frames = 0
                                while q.has():
                                    frame = q.get()
                                    frames += 1
                                    # Visualize only RGB for now to keep it simple, or iterate if needed
                                    if self.visualize:
                                        import cv2
                                        cv2.imshow(f"Head {stream_name.title()}", frame.getCvFrame())
                                        cv2.waitKey(1)
                                
                                dev['frame_count'] += frames
                                
                                # Update FPS
                                dt = current_time - dev['last_fps_time']
                                if dt >= 1.0:
                                    dev['fps'] = dev['frame_count'] / dt
                                    dev['frame_count'] = 0
                                    dev['last_fps_time'] = current_time
                                    
                                self.update_status(dev, f"STREAMING ({dev['fps']:.1f} FPS)")
                             else:
                                 # No frames
                                 dt = current_time - dev['last_fps_time']
                                 if dt >= 2.0:
                                     dev['fps'] = 0.0
                                     self.update_status(dev, "NO DATA")
                     else:
                         for k in ['CAM_RGB', 'CAM_Left', 'CAM_Right']:
                             if k in self.devices:
                                 self.update_status(self.devices[k], 'ERROR (Closed)')
                 except Exception as e:
                     self.log("ERROR", f"Head Camera Check: {e}")
                     for k in ['CAM_RGB', 'CAM_Left', 'CAM_Right']:
                         if k in self.devices:
                              self.update_status(self.devices[k], 'ERROR', error_msg=str(e))

                 # Active: Check gripper camera queues
                 try:
                     if self.grip_cam_device and not self.grip_cam_device.isClosed():
                         for conf in GRIPPER_CAM_CONFIGS:
                             stream_name = conf['name']
                             dev_key = conf['dev_key']
                             if dev_key not in self.devices: continue

                             dev = self.devices[dev_key]
                             q = self.grip_cam_queues.get(stream_name)

                             if q and q.has():
                                 frames = 0
                                 while q.has():
                                     frame = q.get()
                                     frames += 1
                                     if self.visualize:
                                         import cv2
                                         img = frame.getCvFrame()
                                         resized = cv2.resize(img, conf['resize_target'])
                                         cv2.imshow(f"Gripper {conf['name'].title()}", resized)
                                         cv2.waitKey(1)

                                 dev['frame_count'] += frames

                                 dt = current_time - dev['last_fps_time']
                                 if dt >= 1.0:
                                     dev['fps'] = dev['frame_count'] / dt
                                     dev['frame_count'] = 0
                                     dev['last_fps_time'] = current_time

                                 self.update_status(dev, f"STREAMING ({dev['fps']:.1f} FPS)")
                             else:
                                 dt = current_time - dev['last_fps_time']
                                 if dt >= 2.0:
                                     dev['fps'] = 0.0
                                     self.update_status(dev, "NO DATA")
                     else:
                         for conf in GRIPPER_CAM_CONFIGS:
                             dev_key = conf['dev_key']
                             if dev_key in self.devices:
                                 self.update_status(self.devices[dev_key], 'ERROR (Closed)')
                 except Exception as e:
                     self.log("ERROR", f"Gripper Camera Check: {e}")
                     for conf in GRIPPER_CAM_CONFIGS:
                         dev_key = conf['dev_key']
                         if dev_key in self.devices:
                             self.update_status(self.devices[dev_key], 'ERROR', error_msg=str(e))

        # 3. Lidar Checks (Hesai)
        if not self.passive and self.lidar_decoders:
            # Decoder-based: get frame counts from background threads
            with self.lidar_mutex:
                snapshot = {name: dict(state) for name, state in self.lidar_state.items()}

            for name, dev in self.devices.items():
                if dev['type'] != 'LIDAR':
                    continue
                lidar_name = dev['name']
                state = snapshot.get(lidar_name)
                if state is None:
                    continue

                fc = state['frame_count']
                pts = state['points']

                # Track decoded frame rate
                dt = current_time - dev['last_packet_time']
                new_frames = fc - dev.get('_prev_frame_count', 0)

                if dt >= 1.0:
                    fps = new_frames / dt
                    dev['packets'] = fps
                    dev['_prev_frame_count'] = fc
                    dev['last_packet_time'] = current_time

                    if fps > 0:
                        n_pts = len(pts) if pts is not None else 0
                        self.update_status(dev, f"STREAMING ({fps:.0f} FPS, {n_pts} pts)")
                    else:
                        # Only report NO DATA after a 5s grace window — decoders
                        # take a moment to start receiving packets after startup.
                        grace = 5.0
                        elapsed = current_time - (self._lidar_start_time or current_time)
                        if elapsed >= grace:
                            self.update_status(dev, "NO DATA")
                        else:
                            self.update_status(dev, f"STARTING... ({grace - elapsed:.0f}s)")

                # Rerun visualization
                if self.rerun_initialized and pts is not None and len(pts) > 0:
                    try:
                        cfg = next((c for c in LIDAR_CONFIGS if c['name'] == lidar_name), None)
                        if cfg:
                            show_pts = pts
                            max_points = 8000
                            if len(show_pts) > max_points:
                                stride = len(show_pts) // max_points
                                show_pts = show_pts[::stride]
                            rr.log(
                                cfg['rr_label'],
                                rr.Points3D(
                                    positions=show_pts,
                                    colors=[cfg['color']] * len(show_pts),
                                    radii=0.01,
                                ),
                            )
                    except Exception:
                        pass

        elif not self.passive and self.lidar_sockets:
            # Fallback: raw socket packet counting (no decoder available)
            packet_increments = {ip: 0 for ip in LIDAR_DEVICES.values()}
            for s in self.lidar_sockets:
                try:
                    for _ in range(200):
                        try:
                            data, addr = s.recvfrom(2048)
                            ip = addr[0]
                            if ip in packet_increments:
                                packet_increments[ip] += 1
                        except BlockingIOError:
                            break
                except Exception:
                    pass

            for name, dev in self.devices.items():
                if dev['type'] != 'LIDAR':
                    continue
                ip = dev['details']
                count = packet_increments.get(ip, 0)
                dev['packet_count'] += count

                dt = current_time - dev['last_packet_time']
                if dt >= 1.0:
                    pps = dev['packet_count'] / dt
                    dev['packets'] = pps
                    dev['packet_count'] = 0
                    dev['last_packet_time'] = current_time

                    if pps > 0:
                        self.update_status(dev, f"STREAMING ({int(pps)} PPS)")
                    else:
                        self.update_status(dev, "NO DATA")

        else:
            # Passive: ping check
            for name, dev in self.devices.items():
                if dev['type'] != 'LIDAR':
                    continue
                try:
                    response = os.system(f"ping -c 1 -W 0.2 {dev['details']} > /dev/null 2>&1")
                    if response == 0:
                        self.update_status(dev, 'PING OK')
                    else:
                        self.update_status(dev, 'UNREACHABLE', error_msg="Ping Timeout")
                except Exception as e:
                    self.update_status(dev, 'ERROR', error_msg=str(e))

        # 4. Line Sensor Checks
        if self.line_sensor_reader and not self.passive:
            for i in range(LINE_SENSOR_COUNT):
                dev_key = f'LineSensor_{i}'
                if dev_key not in self.devices:
                    continue
                dev = self.devices[dev_key]
                try:
                    sensor_status = self.line_sensor_reader.status[f'sensor_{i}']
                    rate = sensor_status['rate_hz']
                    ts_last = sensor_status.get('ts_last_read', 0)
                    # Check if data is stale (no update in >3 seconds)
                    stale = (current_time - ts_last) > 3.0 if ts_last > 0 else False
                    if not self.line_sensor_reader.is_valid:
                        self.update_status(dev, 'RECOVERING')
                    elif stale:
                        self.update_status(dev, 'NO DATA')
                    elif rate > 0:
                        self.update_status(dev, f'{rate:.1f} Hz')
                    else:
                        self.update_status(dev, 'NO DATA')
                except Exception as e:
                    self.update_status(dev, 'ERROR', error_msg=str(e))
                         
    def close(self):
        # 1. Tear down DepthAI head pipeline — stop pipeline FIRST so node threads
        #    finish cleanly, then close the device. Skipping stop() causes X_LINK_ERROR.
        self.cam_queues.clear()
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
        self.pipeline = None
        if self.cam_device:
            try:
                self.cam_device.close()
            except:
                pass
        self.cam_device = None

        # 2. Tear down DepthAI gripper pipeline
        self.grip_cam_queues.clear()
        if self.grip_pipeline:
            try:
                self.grip_pipeline.stop()
            except:
                pass
        self.grip_pipeline = None
        if self.grip_cam_device:
            try:
                self.grip_cam_device.close()
            except:
                pass
        self.grip_cam_device = None
        
        if self.feetech_port_handler:
            try:
                self.feetech_port_handler.closePort()
            except:
                pass
                
        for s in self.lidar_sockets:
            try:
                s.close()
            except:
                pass
        
        # Stop lidar decoder threads and close decoders
        self.lidar_stop_event.set()
        for t in self.lidar_threads:
            t.join(timeout=1.0)
        for dec in self.lidar_decoders:
            try:
                dec.close()
            except:
                pass
        
        try:
            self.csv_file.close()
        except:
            pass
        
        # Stop line sensor
        if self.line_sensor_reader:
            self.line_sensor_stop.set()
            if self.line_sensor_thread:
                self.line_sensor_thread.join(timeout=1.0)
            try:
                self.line_sensor_reader.stop()
            except:
                pass
        
        if self.visualize:
            try:
                import cv2
                cv2.destroyAllWindows()
            except:
                pass


def generate_table(monitor):
    table = Table(title="Device Status")
    table.add_column("Device", style="cyan")
    table.add_column("Type", style="magenta")
    table.add_column("Details", style="white")
    table.add_column("Status", style="bold")
    table.add_column("Errors", justify="right", style="red")

    for name, dev in monitor.devices.items():
        status = dev['status']
        status_style = "green"
        
        if "DISCONNECTED" in status or "MISSING" in status or "UNREACHABLE" in status or "NO DATA" in status or "ERROR" in status:
            status_style = "red"
        elif "STREAMING" in status or "PING OK" in status or "CONNECTED" in status:
            status_style = "green"
            
        table.add_row(
            dev['name'],
            dev['type'],
            dev['details'],
            f"[{status_style}]{status}[/{status_style}]",
            str(dev['errors'])
        )
    return table

def _is_server_active() -> bool:
    """Return True if stretch_body_server is currently running.

    Tries the programmatic RobotClient API first (zero-latency, no subprocess).
    Falls back to a one-shot `stretch_body_server --ping` subprocess if the
    stretch4_body package is not importable in the current environment.
    """
    # --- Programmatic path (preferred) ---
    try:
        from stretch4_body.robot.robot_client import RobotClient
        client = RobotClient()
        active = client.startup(verbose=False, allow_different_user_connection=True) and client.is_server_active()
        client.stop()
        return active
    except Exception:
        pass

    # --- Subprocess fallback ---
    if shutil.which('stretch_body_server') is None:
        return False
    try:
        result = subprocess.run(
            ['stretch_body_server', '--ping'],
            capture_output=True, text=True, timeout=5
        )
        # --ping prints "Successful server ping" on success
        return 'Successful server ping' in result.stdout
    except Exception:
        return False


def _kill_server() -> bool:
    """Issue a clean `stretch_body_server --kill` and wait for the process
    and its transport file locks to be fully released.

    Returns True if the server was successfully stopped, False otherwise.
    """
    click.secho("\n  Sending kill signal to stretch_body_server...", fg='yellow')
    try:
        result = subprocess.run(
            ['stretch_body_server', '--kill'],
            capture_output=True, text=True, timeout=15
        )
        if result.returncode != 0:
            click.secho(f"  [WARN] --kill returned non-zero exit code: {result.returncode}", fg='yellow')
    except subprocess.TimeoutExpired:
        click.secho("  [WARN] stretch_body_server --kill timed out.", fg='yellow')
    except Exception as e:
        click.secho(f"  [WARN] Could not run stretch_body_server --kill: {e}", fg='yellow')

    # Poll until the server is confirmed dead (up to 15 s)
    click.secho("  Waiting for server to shut down and release transport locks...", fg='cyan')
    deadline = time.time() + 15.0
    while time.time() < deadline:
        if not _is_server_active():
            click.secho("  ✓ Server is no longer active. Transport locks are free.", fg='green', bold=True)
            # Give udev / OS a moment to release any remaining file descriptors
            time.sleep(0.5)
            return True
        time.sleep(0.75)

    click.secho("  [ERROR] Server did not shut down within 15 s.", fg='red', bold=True)
    return False


def _check_server_and_prompt() -> bool:
    """Check if stretch_body_server is running and prompt the user to kill it.

    Returns True if it is safe to continue (server was not running, or was
    successfully killed), False if the user declined or the kill failed.
    """
    click.secho("\nChecking stretch_body_server status...", fg='cyan')

    if not _is_server_active():
        click.secho("  ✓ No active stretch_body_server detected.", fg='green')
        return True

    # Server IS running — show status summary
    click.secho("\n  ⚠ stretch_body_server is currently running in the background.", fg='yellow', bold=True)
    try:
        status_result = subprocess.run(
            ['stretch_body_server', '--status'],
            capture_output=True, text=True, timeout=5
        )
        if status_result.stdout.strip():
            click.secho("\n" + status_result.stdout.strip(), fg='white')
    except Exception:
        pass

    click.secho(
        "\n  To run this tool, the stretch_body_server must be stopped.\n",
        fg='yellow'
    )

    proceed = click.confirm(
        click.style("  Do you want to kill the server and proceed?", fg='cyan', bold=True),
        default=False
    )
    if not proceed:
        click.secho("  Aborted. Stretch Body Server is still running.", fg='red')
        return False

    return _kill_server()


def main():
    parser = argparse.ArgumentParser(description="Stretch Monitor")
    parser.add_argument("--passive", action="store_true", help="Passive checking only (no stream claims)")
    parser.add_argument("--visualize", action="store_true", help="Show camera feeds + lidar point clouds in rerun (Active mode only)")
    parser.add_argument("--duration", type=float, default=0.0, help="Test duration in seconds (0=infinite)")
    args = parser.parse_args()

    # Check for sudo/root permissions (needed for dmesg)
    if os.geteuid() != 0:
        print("WARNING: This script should be run with 'sudo' to fully capture dmesg/kernel logs.")
        # We continue, but dmesg might be empty.
    
    # Check dependencies
    if not args.passive and dai is None:
        print("WARNING: DepthAI not installed. Cameras will be skipped in Active mode.")
        # return  <-- Removed to allow testing other components

    # ── Server pre-flight check ────────────────────────────────────────────────
    # In Active mode (default) this tool claims serial ports, cameras, and the
    # line sensor — all of which are exclusively held by stretch_body_server.
    # In Passive mode we only check device presence (symlinks / pings) and the
    # check is purely advisory, so we don't force a kill.
    if not args.passive:
        if not _check_server_and_prompt():
            sys.exit(1)
    else:
        # Passive mode: warn but do not block.
        if _is_server_active():
            click.secho(
                "\n  [INFO] stretch_body_server is running. Passive mode is safe to use "
                "alongside the server (no device locks are claimed).\n",
                fg='cyan'
            )
    # ── End server pre-flight check ───────────────────────────────────────────

    monitor = DeviceMonitor(passive=args.passive, visualize=args.visualize)
    monitor.discover_devices()
    monitor.start_active_monitoring()
    
    dmesg = DmesgMonitor()
    dmesg.start()
    
    start_time = time.time()
    
    _stop = threading.Event()

    def _sigint_handler(sig, frame):
        _stop.set()

    signal.signal(signal.SIGINT, _sigint_handler)

    try:
        with Live(generate_table(monitor), refresh_per_second=4) as live:
            while not _stop.is_set():
                # Update Watchdogs
                monitor.check_devices()

                # Check Duration
                if args.duration > 0 and (time.time() - start_time) > args.duration:
                    break

                # Layout Construction
                layout = Layout()
                layout.split_column(
                    Layout(name="upper", ratio=3),
                    Layout(name="lower", ratio=1, minimum_size=5)
                )

                # Upper: Device Table
                layout["upper"].update(generate_table(monitor))

                # Lower: Log Panel (compact — last 5 lines)
                logs = dmesg.get_recent_logs()
                log_text = Text("\n".join(logs[-5:]))
                panel = Panel(log_text, title="dmesg / Kernel Log monitor", border_style="blue")
                layout["lower"].update(panel)

                live.update(layout)
                time.sleep(0.25)

    except KeyboardInterrupt:
        pass
    finally:
        monitor.close()
        dmesg.stop()
        print(f"Log saved to {monitor.log_file}")


if __name__ == "__main__":
    main()

