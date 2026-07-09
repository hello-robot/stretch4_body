#!/usr/bin/env python3
"""
Hesai QT128 Lidar Utilities
============================
Shared utilities for Hesai QT128 lidar tests.

Provides:
  - HesaiJT128Decoder : pure-Python UDP packet decoder + rerun visualizer
  - run_lidar_visualization : ready-made test_04 helper used by EOL/BRI tests
"""

import os
import csv
import math
import struct
import socket
import select
import threading
import time

import click
import numpy as np

try:
    import rerun as rr
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False


class HesaiJT128Decoder:
    """
    Pure-Python decoder for Hesai QT128 UDP point cloud packets.

    Packet layout (little-endian):
      Pre-header : 6 bytes  (0xEE 0xFF, version x2, reserved x2)
      Main header: 6 bytes  (laser_num, block_num, first_return, reserved,
                             dis_unit_mm, return_mode)
      Data blocks: block_num × (2B azimuth + laser_num × 4B channel)
                   channel = 2B distance + 1B intensity + 1B reserved
      Tail       : ignored

    Standard QT128 values: laser_num=128, block_num=2, dis_unit=4 mm.
    """

    PRE_HEADER_FMT  = '<BBBBBB'   # 6 bytes
    MAIN_HEADER_FMT = '<BBBBBB'   # 6 bytes
    AZIMUTH_FMT     = '<H'        # 2 bytes per block
    CHANNEL_FMT     = '<HBB'      # 4 bytes per channel: dist(u16), intensity(u8), reserved(u8)

    def __init__(self, udp_port, lidar_ip, correction_file=None):
        self.udp_port = udp_port
        self.lidar_ip = lidar_ip
        self.sock = None
        self._az_offsets_rad = None   # per-channel azimuth offsets [128]
        self._el_rad = None           # per-channel elevation [128]
        self._load_calibration(correction_file)

    # ── Calibration ──────────────────────────────────────────────────────────

    def _load_calibration(self, correction_file):
        """Load per-channel elevation/azimuth offsets from a .dat CSV file.

        Falls back to zero offsets if the file is missing, binary, or malformed.
        """
        n = 128
        if correction_file and os.path.exists(correction_file):
            elevations, azimuths = [], []
            try:
                # Read with latin-1 so any byte value is valid (no UnicodeDecodeError)
                with open(correction_file, 'r', encoding='latin-1') as f:
                    first_line = f.readline()
                    # Validate it looks like a CSV with the expected headers
                    if 'Elevation' not in first_line or 'Azimuth' not in first_line:
                        click.secho(
                            f"    NOTE: Calibration file is not a CSV "
                            f"(expected 'Elevation,Azimuth' header) — "
                            f"using zero offsets. Run stretch_fetch_lidar_calibration to fetch it.",
                            fg="yellow",
                        )
                    else:
                        # Valid CSV — parse remaining rows
                        reader = csv.DictReader(f, fieldnames=first_line.strip().split(','))
                        for row in reader:
                            elevations.append(math.radians(float(row['Elevation'])))
                            azimuths.append(math.radians(float(row['Azimuth'])))
                        if elevations:
                            self._el_rad         = np.array(elevations, dtype=np.float32)
                            self._az_offsets_rad = np.array(azimuths,   dtype=np.float32)
                            click.secho(
                                f"    Calibration loaded: {correction_file} ({len(elevations)} channels)",
                                fg="cyan",
                            )
                            return
            except Exception as exc:
                click.secho(f"    WARNING: Failed to parse calibration file: {exc}", fg="yellow")

        elif correction_file:
            click.secho(
                f"    NOTE: Calibration file not found: {correction_file} — "
                f"using zero offsets. Run stretch_fetch_lidar_calibration to fetch it.",
                fg="yellow",
            )

        self._el_rad         = np.zeros(n, dtype=np.float32)
        self._az_offsets_rad = np.zeros(n, dtype=np.float32)


    # ── Socket lifecycle ─────────────────────────────────────────────────────

    def open(self):
        """Bind a non-blocking UDP socket on self.udp_port."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setblocking(False)
        self.sock.bind(('0.0.0.0', self.udp_port))

    def close(self):
        """Close the UDP socket."""
        if self.sock:
            self.sock.close()
            self.sock = None

    # ── Packet decoding ───────────────────────────────────────────────────────

    def decode_packet(self, data):
        """
        Decode one UDP payload into an (N, 3) float32 XYZ array (metres).
        Returns None if the packet is not a valid QT128 data packet.
        """
        PRE  = struct.calcsize(self.PRE_HEADER_FMT)   # 6
        MAIN = struct.calcsize(self.MAIN_HEADER_FMT)  # 6
        AZ   = struct.calcsize(self.AZIMUTH_FMT)      # 2
        CH   = struct.calcsize(self.CHANNEL_FMT)      # 4

        if len(data) < PRE + MAIN:
            return None

        pre = struct.unpack_from(self.PRE_HEADER_FMT, data, 0)
        if pre[0] != 0xEE or pre[1] != 0xFF:
            return None  # Not a Hesai data packet

        main = struct.unpack_from(self.MAIN_HEADER_FMT, data, PRE)
        laser_num = main[0]   # e.g. 128
        block_num = main[1]   # e.g. 2
        dis_unit  = main[4]   # e.g. 4 mm

        if laser_num == 0 or block_num == 0:
            return None

        block_size = AZ + laser_num * CH
        if len(data) < PRE + MAIN + block_num * block_size:
            return None

        dis_unit_m = dis_unit * 1e-3           # mm → m
        n_channels = min(laser_num, len(self._el_rad))

        all_xyz = []
        offset = PRE + MAIN
        for _ in range(block_num):
            (az_raw,) = struct.unpack_from(self.AZIMUTH_FMT, data, offset)
            az_block_rad = math.radians(az_raw * 0.01)  # 0.01° per LSB
            offset += AZ

            for ch_idx in range(n_channels):
                dist_raw, _intensity, _ = struct.unpack_from(self.CHANNEL_FMT, data, offset)
                offset += CH

                if dist_raw == 0:
                    continue  # Invalid return

                d  = dist_raw * dis_unit_m
                az = az_block_rad + self._az_offsets_rad[ch_idx]
                el = self._el_rad[ch_idx]

                cos_el = math.cos(el)
                all_xyz.append((
                    d * cos_el * math.cos(az),
                    d * cos_el * math.sin(az),
                    d * math.sin(el),
                ))

            # Skip channels beyond what calibration covers
            if laser_num > n_channels:
                offset += (laser_num - n_channels) * CH

        if not all_xyz:
            return None
        return np.array(all_xyz, dtype=np.float32)

    def recv_frame(self, timeout=0.1):
        """
        Drain the socket for `timeout` seconds, accumulate and return a single
        merged (N, 3) XYZ array from all valid packets. Returns None if no
        valid data arrived from self.lidar_ip within the timeout.
        """
        deadline = time.time() + timeout
        chunks = []
        while time.time() < deadline:
            ready, _, _ = select.select([self.sock], [], [], 0.05)
            for s in ready:
                try:
                    pkt, addr = s.recvfrom(2048)
                    if addr[0] != self.lidar_ip:
                        continue
                    xyz = self.decode_packet(pkt)
                    if xyz is not None:
                        chunks.append(xyz)
                except socket.error:
                    pass
        if not chunks:
            return None
        return np.concatenate(chunks, axis=0)


# ── Rerun visualization helper ────────────────────────────────────────────────

def run_lidar_visualization(
    lidar_configs,
    session_name,
    display_duration=15.0,
    frames_needed=3,
    max_points=8000,
):
    """
    Open one HesaiJT128Decoder per entry in `lidar_configs`, stream decoded
    point clouds into a rerun viewer, then return a dict of results.

    Args:
        lidar_configs (list[dict]): Each entry must have:
            - name            : str   display name
            - ip              : str   lidar IP address
            - udp_port        : int   UDP data port
            - correction_file : str   path to .dat calibration CSV
            - rr_label        : str   rerun entity path  (e.g. 'world/left_lidar')
            - color           : list  [R, G, B]
        session_name (str):     rerun init name (e.g. 'EOL_head_lidar_pointcloud')
        display_duration (float): seconds to stream
        frames_needed (int):    minimum frame count to consider a lidar healthy

    Returns:
        dict: {name: frame_count}   for each lidar
    """
    if not RERUN_AVAILABLE:
        raise RuntimeError("rerun-sdk not installed. Run: pip install rerun-sdk")

    try:
        rr.init(session_name, spawn=True)
    except Exception as exc:
        # A previous rerun session may have left an orphaned gRPC connection.
        # The viewer from that session may still be open, so try connecting to it.
        click.secho(f"  NOTE: rerun spawn warning (previous session): {exc}", fg="yellow")
        try:
            rr.init(session_name, spawn=False)
        except Exception:
            pass
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log(
        "world/xyz",
        rr.Arrows3D(
            vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
        ),
        static=True,
    )
    click.secho("  Rerun viewer launched. Watch for point clouds from both lidars.", fg="cyan")

    # ── Open decoders ──────────────────────────────────────────────────────
    lidar_mutex = threading.Lock()
    lidar_state = {cfg['name']: {'points': None, 'frame_count': 0} for cfg in lidar_configs}
    decoders = []

    for cfg in lidar_configs:
        dec = HesaiJT128Decoder(
            udp_port=cfg['udp_port'],
            lidar_ip=cfg['ip'],
            correction_file=cfg['correction_file'],
        )
        dec.open()
        decoders.append(dec)
        click.secho(f"  Decoder opened for {cfg['name']} ({cfg['ip']}, port {cfg['udp_port']})", fg="cyan")

    # ── Background receive threads ─────────────────────────────────────────
    stop_event = threading.Event()

    def recv_worker(dec, cfg):
        while not stop_event.is_set():
            xyz = dec.recv_frame(timeout=0.1)
            if xyz is not None and len(xyz) > 0:
                with lidar_mutex:
                    lidar_state[cfg['name']]['points'] = xyz
                    lidar_state[cfg['name']]['frame_count'] += 1

    threads = []
    for dec, cfg in zip(decoders, lidar_configs):
        t = threading.Thread(target=recv_worker, args=(dec, cfg), daemon=True)
        t.start()
        threads.append(t)

    # ── Rerun render loop ──────────────────────────────────────────────────
    click.echo(f"  Streaming for {display_duration:.0f} seconds... (rerun window should be open)")
    start = time.time()
    try:
        while time.time() - start < display_duration:
            time.sleep(0.05)

            # Snapshot state under the lock — keep the lock as brief as possible
            # so recv threads are never stalled by a slow rr.log() call.
            snapshot = {}
            with lidar_mutex:
                for cfg in lidar_configs:
                    name = cfg['name']
                    pts  = lidar_state[name]['points']
                    fc   = lidar_state[name]['frame_count']
                    if pts is not None and len(pts) > 0:
                        snapshot[name] = (pts.copy(), fc)

            # Log outside the mutex — rr.log() may block when the batcher is full
            for cfg in lidar_configs:
                name = cfg['name']
                if name not in snapshot:
                    continue
                pts, fc = snapshot[name]

                # Subsample to max_points to keep the rerun batcher from stalling
                if len(pts) > max_points:
                    stride = len(pts) // max_points
                    pts = pts[::stride]

                rr.log(
                    cfg['rr_label'],
                    rr.Points3D(
                        positions=pts,
                        colors=[cfg['color']] * len(pts),
                        radii=0.01,
                    ),
                )
                if fc % 20 == 1:
                    click.echo(f"    {name}: frame {fc}, {len(pts)} points (shown)")
    finally:
        stop_event.set()
        for t in threads:
            t.join(timeout=1.0)
        for dec in decoders:
            dec.close()
        # Cleanly disconnect rerun to suppress gRPC shutdown errors
        try:
            rr.disconnect()
        except Exception:
            pass

    return {cfg['name']: lidar_state[cfg['name']]['frame_count'] for cfg in lidar_configs}
