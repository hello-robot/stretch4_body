#!/usr/bin/env python3
"""
stretch_fetch_lidar_calibration
================================
Fetches the per-channel angle-correction (.dat) calibration file directly
from a Hesai QT128 lidar over its PTC (Pandar TCP Command) interface.

The lidar exposes this data on TCP port 9347.
PTC command 0x05  →  PTC_COMMAND_GET_LIDAR_CALIBRATION
Response payload  →  raw CSV bytes (Channel,Elevation,Azimuth,...).

Usage
-----
Connect just one lidar at a time and run:

    # Fetch left lidar calibration  (IP 192.168.1.202)
    ./stretch_fetch_lidar_calibration --ip 192.168.1.202 --out ~/stretch_user/$(hostname)/calibration_hesais/left_lidar_calibration.dat

    # Fetch right lidar calibration  (IP 192.168.1.201)
    ./stretch_fetch_lidar_calibration --ip 192.168.1.201 --out ~/stretch_user/$(hostname)/calibration_hesais/right_lidar_calibration.dat

Or use the --left / --right convenience flags to use the default IPs and
write to the correct paths automatically:

    ./stretch_fetch_lidar_calibration --left
    ./stretch_fetch_lidar_calibration --right
"""

import os
import shutil
import socket
import struct
import subprocess
import sys
import argparse

# ── PTC protocol constants ───────────────────────────────────────────────────
PTC_PORT             = 9347
PTC_MAGIC_1          = 0x47
PTC_MAGIC_2          = 0x74
PTC_COMMAND_GET_CAL  = 0x05   # PTC_COMMAND_GET_LIDAR_CALIBRATION
PTC_HEADER_FMT       = '>BBBBII'   # magic1 magic2 cmd ver payloadLen(BE) reserved(BE)
PTC_HEADER_SIZE      = struct.calcsize(PTC_HEADER_FMT)  # 12 bytes

LEFT_IP  = '192.168.1.202'
RIGHT_IP = '192.168.1.201'

STRETCH_USER = os.path.expanduser('~/stretch_user')


def _default_out_path(lidar_side: str) -> str:
    """Return the canonical calibration_hesais path for this robot."""
    hostname = os.uname().nodename  # e.g. stretch-se4-4001
    cal_dir = os.path.join(STRETCH_USER, hostname, 'calibration_hesais')
    return os.path.join(cal_dir, f'{lidar_side}_lidar_calibration.dat')


def build_request(cmd: int) -> bytes:
    """Build a PTC request packet (12-byte header, zero payload)."""
    return struct.pack(PTC_HEADER_FMT,
                       PTC_MAGIC_1, PTC_MAGIC_2,  # magic
                       cmd,                         # command id
                       0x00,                        # protocol version
                       0,                           # payload length
                       0)                           # reserved


def parse_response(data: bytes) -> bytes:
    """
    Strip the PTC response header and return the raw payload.
    Raises ValueError on protocol errors.
    """
    if len(data) < PTC_HEADER_SIZE:
        raise ValueError(f'Response too short: {len(data)} bytes')

    magic1, magic2, cmd, ver, payload_len, _ = struct.unpack_from(PTC_HEADER_FMT, data, 0)
    if magic1 != PTC_MAGIC_1 or magic2 != PTC_MAGIC_2:
        raise ValueError(f'Bad magic bytes: 0x{magic1:02X} 0x{magic2:02X} (expected 0x74 0x47)')

    payload = data[PTC_HEADER_SIZE:]
    # payload_len may be 0 for some firmware – fall back to whatever arrived
    if payload_len > 0:
        payload = payload[:payload_len]
    return payload


def fetch_calibration(ip: str, timeout: float = 5.0) -> bytes:
    """
    Open a TCP connection to the lidar, send the GetLidarCalibration command,
    and return the raw CSV payload bytes.
    """
    req = build_request(PTC_COMMAND_GET_CAL)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(timeout)
        print(f'  Connecting to {ip}:{PTC_PORT} …', flush=True)
        s.connect((ip, PTC_PORT))
        s.sendall(req)

        # Read until connection closes (calibration payload can be ~10 KB)
        chunks = []
        while True:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            chunks.append(chunk)

    raw = b''.join(chunks)
    if not raw:
        raise RuntimeError('No data received from lidar – is it powered on and reachable?')
    return parse_response(raw)


def save_calibration(payload: bytes, out_path: str) -> None:
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'wb') as f:
        f.write(payload)


def push_to_fleet(local_path: str) -> None:
    """
    Copy the calibration file into the fleet repo under
    ~/repos/stretch_fleet_ii/robots/<HELLO_FLEET_ID>/calibration_hesais/
    then commit and push.
    """
    fleet_id = os.environ.get('HELLO_FLEET_ID')
    if not fleet_id:
        print('  WARNING: HELLO_FLEET_ID not set – skipping fleet push.')
        return

    fleet_repo = os.path.expanduser('~/repos/stretch_fleet_ii')
    if not os.path.isdir(fleet_repo):
        print(f'  WARNING: Fleet repo not found at {fleet_repo} – skipping fleet push.')
        return

    dest_dir = os.path.join(fleet_repo, 'robots', fleet_id, 'calibration_hesais')
    os.makedirs(dest_dir, exist_ok=True)

    dest_path = os.path.join(dest_dir, os.path.basename(local_path))
    shutil.copy2(local_path, dest_path)
    print(f'  Copied calibration to fleet repo: {dest_path}')

    def _run(cmd, **kwargs):
        result = subprocess.run(cmd, cwd=fleet_repo, capture_output=True, text=True, **kwargs)
        if result.returncode != 0:
            raise RuntimeError(f'`{" ".join(cmd)}` failed:\n{result.stderr.strip()}')
        return result.stdout.strip()

    print('  Pushing to stretch_fleet_ii …')
    _run(['git', 'pull'])
    _run(['git', 'add', dest_path])
    # Only commit if there is something staged
    staged = subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=fleet_repo).returncode
    if staged != 0:
        _run(['git', 'commit', '-m', f'{fleet_id}: update lidar calibration'])
        _run(['git', 'push'])
        print('  ✓ Fleet repo updated and pushed.')
    else:
        print('  Fleet file unchanged – nothing to commit.')


def validate_csv(payload: bytes, out_path: str) -> bool:
    """
    Quick sanity check: the payload should look like a CSV with
    'Channel', 'Elevation', 'Azimuth' columns.
    """
    try:
        text = payload.decode('ascii', errors='replace')
        lines = [l for l in text.splitlines() if l.strip()]
        if not lines:
            return False
        header = lines[0].lower()
        return 'elevation' in header and 'azimuth' in header
    except Exception:
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Fetch Hesai QT128 angle-correction calibration file over PTC.')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--left',  action='store_true',
                       help=f'Fetch left lidar ({LEFT_IP}) to default path')
    group.add_argument('--right', action='store_true',
                       help=f'Fetch right lidar ({RIGHT_IP}) to default path')
    parser.add_argument('--ip',  default=None,
                        help='Lidar IP address (overrides --left/--right)')
    parser.add_argument('--out', default=None,
                        help='Output .dat file path (overrides default)')
    parser.add_argument('--timeout', type=float, default=5.0,
                        help='TCP timeout in seconds (default 5)')
    parser.add_argument('--no-fleet', action='store_true',
                        help='Skip copying and pushing to stretch_fleet_ii')
    args = parser.parse_args()

    if args.ip:
        ip = args.ip
        side = 'left' if ip == LEFT_IP else ('right' if ip == RIGHT_IP else 'lidar')
    elif args.left:
        ip = LEFT_IP
        side = 'left'
    elif args.right:
        ip = RIGHT_IP
        side = 'right'
    else:
        parser.error('Specify --left, --right, or --ip <addr>')

    out_path = args.out or _default_out_path(side)

    print(f'\n=== Hesai Calibration Fetcher ===')
    print(f'  Lidar  : {ip}  ({side})')
    print(f'  Output : {out_path}')

    try:
        payload = fetch_calibration(ip, timeout=args.timeout)
    except (ConnectionRefusedError, OSError) as e:
        print(f'\nERROR: Cannot connect to lidar at {ip}:{PTC_PORT}')
        print(f'  {e}')
        print('\nCheck:')
        print('  1. Lidar is powered on')
        print('  2. Ethernet cable is connected')
        print(f'  3. This computer has an IP on the 192.168.1.x subnet')
        sys.exit(1)
    except RuntimeError as e:
        print(f'\nERROR: {e}')
        sys.exit(1)

    if not validate_csv(payload, out_path):
        print(f'\nWARNING: Response does not look like a CSV calibration file.')
        print(f'  Raw prefix: {payload[:80]!r}')
        print(f'  Saving anyway – please inspect the file.')

    save_calibration(payload, out_path)

    n_lines = max(0, payload.count(b'\n') - 1)  # minus header
    print(f'\n  ✓ Saved calibration ({n_lines} channels, {len(payload)} bytes)')
    print(f'    → {out_path}')

    if not args.no_fleet:
        try:
            push_to_fleet(out_path)
        except RuntimeError as e:
            print(f'\nWARNING: Fleet push failed: {e}')
    else:
        print('  (Fleet push skipped via --no-fleet)')


if __name__ == '__main__':
    main()
