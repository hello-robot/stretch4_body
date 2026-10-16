#!/usr/bin/env python3
"""
stretch_firmware_versions: the firmware version of every board on the robot.

Boards on a USB port (PIMU, ESP behind it, steppers, end of arm servos, line sensor) come
from status['firmware_versions'], which stretch_body_server fills from code that already
owns each port; this command never opens a device. The head lidars are asked over the
network with one read-only PTC inventory query, unless the lidar rail is off (the query
counts as a lidar consumer, so it restarts sentry_lidar_idle's idle countdown). Cameras
come from the versions file each camera writes when it is opened; no camera is opened here.
"""
import argparse
import glob
import json
import os
import socket
import struct
import sys
import tempfile
import time

from stretch4_body.robot.firmware_versions import row

PTC_PORT = 9347
LIDARS = (('lidar right', '192.168.1.201'), ('lidar left', '192.168.1.202'))
PTC_GET_INVENTORY_INFO = 0x07
PTC_MIN_INVENTORY_BYTES = 136
CAMERA_VERSIONS_GLOB = os.path.join(tempfile.gettempdir(), 'luxonis_device_versions_*.json')

NO_SERVER = 'stretch_body_server not running'


class PtcError(Exception):
    pass


# ------------------------------------------------------------------------ rows

def server_rows(status):
    """Rows the server published, or None when it predates status['firmware_versions']."""
    fv = (status or {}).get('firmware_versions')
    if not isinstance(fv, dict):
        return None
    return list(fv.get('boards') or [])


# ------------------------------------------------------------------------ lidars

def ptc_request(cmd, payload=b''):
    return struct.pack('>BBBBI', 0x47, 0x74, cmd, 0, len(payload)) + payload


def decode_ptc_reply(header, cmd):
    """Payload length of a PTC reply header; raises PtcError on a bad header or return code."""
    if len(header) != 8:
        raise PtcError('short reply header')
    h0, h1, rcmd, rc, length = struct.unpack('>BBBBI', header)
    if (h0, h1) != (0x47, 0x74) or rcmd != cmd:
        raise PtcError('not a PTC reply to command 0x%02x' % cmd)
    if rc != 0:
        raise PtcError('lidar returned code %d' % rc)
    return length


def decode_inventory(raw):
    """GetInventoryInfo payload: serial, model, and the three firmware versions."""
    if len(raw) < PTC_MIN_INVENTORY_BYTES:
        raise PtcError('inventory reply too short (%d < %d bytes)' % (len(raw), PTC_MIN_INVENTORY_BYTES))

    def text(b):
        return b.split(b'\x00', 1)[0].decode('ascii', errors='replace').strip()
    out = {'serial_number': text(raw[0:18]), 'model': text(raw[18:50]),
           'app_version': text(raw[72:104]), 'sensor_version': text(raw[104:120]),
           'controller_version': text(raw[120:136])}
    if len(raw) >= 194:
        out['build_id'] = text(raw[184:194])
    # The APP field ends in a letter (15.AF.B0.00.02.L); the byte after it is not always
    # NUL, so the raw text can read 15.AF.B0.00.02.L0. Keep the raw text beside it.
    head, _, last = out['app_version'].rpartition('.')
    if head and last[:1].isalpha() and last[1:].isdigit():
        out['app_version_raw'] = out['app_version']
        out['app_version'] = head + '.' + last[0]
    return out


def _recv_exact(sock, n):
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise PtcError('lidar closed the connection')
        data += chunk
    return data


def query_inventory(ip, port=PTC_PORT, timeout=2.0):
    with socket.create_connection((ip, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(ptc_request(PTC_GET_INVENTORY_INFO))
        length = decode_ptc_reply(_recv_exact(sock, 8), PTC_GET_INVENTORY_INFO)
        return decode_inventory(_recv_exact(sock, length))


def lidar_rail_off_reason(status):
    """Why the lidars must not be asked, or None. A PTC connection counts as a lidar
    consumer for sentry_lidar_idle, so asking would switch a rail it has cut back on."""
    sentry = (((status or {}).get('safety_layer') or {}).get('sentry_manager') or {}).get(
        'sentry_lidar_idle') or {}
    if sentry.get('rail_cmd') == 'off':
        return ('lidar rail is off (sentry_lidar_idle, nothing consumes lidar data); not asked, '
                'since a query would power it on')
    return None


def lidar_rows(status, query=None, now=None):
    query = query or query_inventory
    now = now or time.time
    source = 'PTC inventory query'
    rows = []
    skip = lidar_rail_off_reason(status)
    for board, ip in LIDARS:
        port = '%s:%d' % (ip, PTC_PORT)
        if skip:
            rows.append(row(board, port, None, source, reason=skip))
            continue
        try:
            inv = query(ip)
        except (OSError, PtcError) as e:
            rows.append(row(board, port, None, source, reason='no answer (%s)' % (str(e) or type(e).__name__)))
            continue
        detail = {k: inv[k] for k in ('sensor_version', 'controller_version', 'build_id',
                                      'serial_number', 'model', 'app_version_raw') if inv.get(k)}
        rows.append(row(board, port, inv.get('app_version'), source, now(), detail,
                        'lidar reported no APP version'))
    return rows


# ------------------------------------------------------------------------ cameras

def camera_rows(paths=None):
    source = 'versions written when the camera was last opened'
    paths = sorted(glob.glob(CAMERA_VERSIONS_GLOB)) if paths is None else paths
    records = {}
    for path in paths:
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError):
            continue
        for key, rec in (data.items() if isinstance(data, dict) else ()):
            if isinstance(rec, dict) and (key not in records or
                                          (rec.get('time') or 0) > (records[key].get('time') or 0)):
                records[key] = rec
    if not records:
        return [row('cameras', None, None, source,
                    reason='no camera opened since the computer started')]
    rows = []
    for key, rec in sorted(records.items(), key=lambda kv: str(kv[1].get('device_name') or kv[0])):
        name = rec.get('product_name') or rec.get('device_name') or 'camera'
        detail = {'bootloader_version': rec.get('bootloader_version'),
                  'device_id': rec.get('device_id') or key, 'usb_speed': rec.get('usb_speed')}
        rows.append(row('camera %s' % name, rec.get('device_port'), rec.get('depthai_version'),
                        source + ' (device firmware comes with depthai)', rec.get('time'),
                        {k: v for k, v in detail.items() if v}))
    return rows


# ------------------------------------------------------------------------ report

def age_text(t, now):
    if not t:
        return '-'
    s = max(0.0, now - t)
    if s < 120:
        return '%ds' % s
    if s < 7200:
        return '%dm' % (s // 60)
    if s < 172800:
        return '%dh' % (s // 3600)
    return '%dd' % (s // 86400)


def render(rows, now):
    """One line per board; a board without a version gets its reason on the next line."""
    head = ('board', 'port / address', 'version', 'age', 'source')
    table = [(r['board'], r['port'] or '-', r['version'] or '-', age_text(r['time'], now),
              r['source']) for r in rows]
    widths = [max(len(t[i]) for t in [head] + table) for i in range(4)]

    def line(cells):
        return '  '.join([c.ljust(w) for c, w in zip(cells, widths)] + [cells[4]]).rstrip()
    out = [line(head)]
    for r, t in zip(rows, table):
        out.append(line(t))
        if not r['version']:
            out.append(' ' * (widths[0] + 2) + 'no version: ' + r['reason'])
    return '\n'.join(out) + '\n'


def build_parser():
    parser = argparse.ArgumentParser(
        prog='stretch_firmware_versions',
        description='Firmware version of every board: PIMU, ESP, steppers, end of arm servos and '
                    'line sensor from stretch_body_server, the head lidars over the network, and '
                    'the cameras from the versions recorded when each was last opened. Never '
                    'opens a robot device.',
        epilog='Exit status: 0 every board reported; 1 some board has no version; 2 no server.')
    parser.add_argument('--json', action='store_true', help='print the rows as JSON')
    parser.add_argument('--no-lidar', action='store_true', help='do not ask the lidars')
    return parser


def connect():
    """A started RobotClient, or None. Never opens a device."""
    from stretch4_body.robot.robot_client import RobotClient
    robot = RobotClient()
    if not robot.startup(verbose=False):
        return None
    return robot


def collect(args):
    status = None
    robot = connect()
    if robot is None:
        rows = [row('usb boards', None, None, 'stretch_body_server status',
                    reason=NO_SERVER + ' (start it with: stretch_body_server --launch)')]
    else:
        try:
            robot.pull_status()
            status = robot.status
            rows = server_rows(status)
            if rows is None:
                rows = [row('usb boards', None, None, 'stretch_body_server status',
                            reason='this stretch_body_server does not publish firmware_versions '
                                   '(older stretch4_body)')]
        finally:
            robot.stop()
    if not args.no_lidar:
        rows += lidar_rows(status)
    rows += camera_rows()
    return rows, robot is not None


def main(argv=None):
    args = build_parser().parse_args(argv)
    rows, server_ok = collect(args)
    now = time.time()
    if args.json:
        print(json.dumps({'time': now, 'boards': rows}, indent=2))
    else:
        sys.stdout.write(render(rows, now))
    if not server_ok:
        return 2
    return 0 if all(r['version'] for r in rows) else 1


if __name__ == '__main__':
    sys.exit(main())
