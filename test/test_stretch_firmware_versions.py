"""stretch_firmware_versions: PTC inventory decode, lidar rail guard, camera file, report."""
import json
import socket
import struct
import threading
import types

import pytest

from stretch4_body.tools import stretch_firmware_versions as cmd

T0 = 1_760_000_000.0


def field(text, size):
    return text.encode('ascii').ljust(size, b'\x00')


def inventory_payload(build_id=True):
    raw = (field('2430A1B2C3D4E5F6G7', 18) + field('JT128', 32) + field('2025-03-01', 16)
           + bytes([0x00, 0x11, 0x22, 0x33, 0x44, 0x55]) + field('15.AF.B0.00.02.Y', 32)
           + field('1.b.0028', 16) + field('1.b.0031', 16))
    if build_id:
        raw += b'\x00' * 48 + field('B240715', 10)
    return raw


def reply(payload, cmd_id=0x07, rc=0):
    return struct.pack('>BBBBI', 0x47, 0x74, cmd_id, rc, len(payload)) + payload


# ------------------------------------------------------------------------ PTC

def test_request_is_the_inventory_command_with_no_payload():
    assert cmd.ptc_request(0x07) == bytes([0x47, 0x74, 0x07, 0, 0, 0, 0, 0])


def test_decode_inventory_fields():
    inv = cmd.decode_inventory(inventory_payload())
    assert inv == {'serial_number': '2430A1B2C3D4E5F6G7', 'model': 'JT128',
                   'app_version': '15.AF.B0.00.02.Y', 'sensor_version': '1.b.0028',
                   'controller_version': '1.b.0031', 'build_id': 'B240715'}
    assert 'build_id' not in cmd.decode_inventory(inventory_payload(build_id=False))


def test_app_version_trailing_byte_is_dropped():
    # Some lidars report APP as 15.AF.B0.00.02.L0.
    raw = bytearray(inventory_payload())
    raw[72:104] = field('15.AF.B0.00.02.L0', 32)
    inv = cmd.decode_inventory(bytes(raw))
    assert inv['app_version'] == '15.AF.B0.00.02.L'
    assert inv['app_version_raw'] == '15.AF.B0.00.02.L0'


def test_decode_rejects_short_and_bad_replies():
    with pytest.raises(cmd.PtcError, match='too short'):
        cmd.decode_inventory(inventory_payload()[:135])
    with pytest.raises(cmd.PtcError, match='code 3'):
        cmd.decode_ptc_reply(reply(b'', rc=3)[:8], 0x07)
    with pytest.raises(cmd.PtcError, match='not a PTC reply'):
        cmd.decode_ptc_reply(b'\x00' * 8, 0x07)
    assert cmd.decode_ptc_reply(reply(b'abc')[:8], 0x07) == 3


def test_query_inventory_against_a_local_ptc_server():
    srv = socket.socket()
    srv.bind(('127.0.0.1', 0))
    srv.listen(1)
    got = {}

    def serve():
        conn, _ = srv.accept()
        with conn:
            got['request'] = conn.recv(8)
            data = reply(inventory_payload())
            conn.sendall(data[:5])      # split across segments
            conn.sendall(data[5:])
    t = threading.Thread(target=serve)
    t.start()
    try:
        inv = cmd.query_inventory('127.0.0.1', port=srv.getsockname()[1], timeout=2.0)
    finally:
        t.join(2)
        srv.close()
    assert got['request'] == cmd.ptc_request(0x07)
    assert inv['app_version'] == '15.AF.B0.00.02.Y'


# ------------------------------------------------------------------------ lidars

def sentry_status(rail_cmd):
    return {'safety_layer': {'sentry_manager': {'sentry_lidar_idle': {'rail_cmd': rail_cmd}}}}


def test_lidars_not_asked_when_the_rail_is_off():
    def query(ip):
        raise AssertionError('asked a lidar with the rail off')
    rows = cmd.lidar_rows(sentry_status('off'), query=query)
    assert [r['board'] for r in rows] == ['lidar right', 'lidar left']
    assert all('lidar rail is off' in r['reason'] for r in rows)


def test_lidars_asked_when_rail_on_or_no_server():
    asked = []

    def query(ip):
        asked.append(ip)
        if ip == '192.168.1.202':
            raise socket.timeout('timed out')
        return cmd.decode_inventory(inventory_payload())
    for status in (sentry_status('on'), None):
        asked.clear()
        rows = cmd.lidar_rows(status, query=query, now=lambda: T0)
        assert asked == ['192.168.1.201', '192.168.1.202']
        right, left = rows
        assert right['version'] == '15.AF.B0.00.02.Y' and right['time'] == T0
        assert right['detail']['sensor_version'] == '1.b.0028'
        assert right['port'] == '192.168.1.201:9347'
        assert left['version'] is None and left['reason'] == 'no answer (timed out)'


# ------------------------------------------------------------------------ cameras

def test_cameras_from_the_versions_files(tmp_path):
    a = tmp_path / 'luxonis_device_versions_1000.json'
    b = tmp_path / 'luxonis_device_versions_1001.json'
    rec = {'device_port': '1.2.3', 'time': T0, 'depthai_version': '3.1.0',
           'device_id': 'ID1', 'product_name': 'OAK-D-W', 'device_name': '1.2.3',
           'bootloader_version': '0.0.26', 'usb_speed': 'SUPER'}
    a.write_text(json.dumps({'ID1': rec}))
    b.write_text(json.dumps({'ID1': dict(rec, time=T0 + 5, depthai_version='3.1.1')}))
    (tmp_path / 'luxonis_device_versions_9.json').write_text('{torn')
    rows = cmd.camera_rows(sorted(str(p) for p in tmp_path.iterdir()))
    assert len(rows) == 1
    r = rows[0]
    assert r['board'] == 'camera OAK-D-W' and r['version'] == '3.1.1' and r['time'] == T0 + 5
    assert r['detail']['bootloader_version'] == '0.0.26'


def test_no_camera_opened():
    rows = cmd.camera_rows([])
    assert rows[0]['version'] is None
    assert 'no camera opened' in rows[0]['reason']


# ------------------------------------------------------------------------ command

def server_status():
    boards = [
        {'board': 'pimu', 'port': '/dev/hello-power-periph', 'version': 'hello-pimu2.v0.1.53p14',
         'detail': {}, 'source': 'board_info at server startup', 'time': T0, 'reason': None},
        {'board': 'esp', 'port': 'via /dev/hello-power-periph', 'version': None, 'detail': {},
         'source': 'PIMU diagnostics snapshot', 'time': None,
         'reason': 'ESP firmware older than hello-esp.v0.1.11p0 does not report its version'},
    ]
    return dict(sentry_status('off'), firmware_versions={'boards': boards, 'time': T0})


def run(argv, status, capsys, monkeypatch):
    def query(ip):
        raise AssertionError('asked a lidar with the rail off')
    monkeypatch.setattr(cmd, 'read_server_status', lambda: status)
    monkeypatch.setattr(cmd, 'camera_rows', lambda: [])
    monkeypatch.setattr(cmd, 'query_inventory', query)
    monkeypatch.setattr(cmd.time, 'time', lambda: T0 + 90)
    rc = cmd.main(argv)
    return rc, capsys.readouterr()


def test_command_json(capsys, monkeypatch):
    rc, out = run(['--json'], server_status(), capsys, monkeypatch)
    data = json.loads(out.out)
    assert [r['board'] for r in data['boards']] == ['pimu', 'esp', 'lidar right', 'lidar left']
    assert rc == 1                       # the ESP and the lidars have no version


def test_command_table(capsys, monkeypatch):
    rc, out = run([], server_status(), capsys, monkeypatch)
    lines = out.out.splitlines()
    assert lines[0].split()[:2] == ['board', 'port']
    pimu = next(l for l in lines if l.startswith('pimu'))
    assert 'hello-pimu2.v0.1.53p14' in pimu and '90s' in pimu
    assert any(l.strip().startswith('no version: ESP firmware older') for l in lines)
    assert any('no version: lidar rail is off' in l for l in lines)


def test_command_without_server(capsys, monkeypatch):
    rc, out = run(['--json', '--no-lidar'], None, capsys, monkeypatch)
    assert rc == 2
    boards = json.loads(out.out)['boards']
    assert boards[0]['reason'].startswith('stretch_body_server not running')


def test_command_with_an_older_server(capsys, monkeypatch):
    rc, out = run(['--json', '--no-lidar'], {'server': {}}, capsys, monkeypatch)
    assert 'does not publish firmware_versions' in json.loads(out.out)['boards'][0]['reason']
    assert rc == 1


# ------------------------------------------------------------------------ read only

class RecordingSocket:
    def __init__(self, kind, status):
        self.kind, self.status, self.sent, self.connected = kind, status, [], []

    def setsockopt(self, *a):
        pass

    def connect(self, addr):
        self.connected.append(addr)

    def poll(self, *a):
        return 1

    def recv_pyobj(self, flags=0):
        return self.status

    def close(self, linger=None):
        pass

    def __getattr__(self, name):
        if name.startswith('send'):
            def send(*a, **k):
                self.sent.append((name, a))
            return send
        raise AttributeError(name)


def test_command_only_subscribes_to_status_and_sends_nothing(capsys, monkeypatch):
    import zmq
    from stretch4_body.core import client_server
    from stretch4_body.core.subsystem_client import SubsystemClient
    sockets = []

    class Context:
        def socket(self, kind):
            sockets.append(RecordingSocket(kind, server_status()))
            return sockets[-1]

    def no_command(*a, **k):
        raise AssertionError('a command was sent')
    monkeypatch.setattr(zmq.Context, 'instance', staticmethod(lambda: Context()))
    monkeypatch.setattr(client_server.StretchBodyClient, '_do_send_cmd', no_command)
    monkeypatch.setattr(client_server.StretchBodyClient, 'startup', no_command)
    monkeypatch.setattr(SubsystemClient, 'push_command', no_command)
    real_exists = cmd.os.path.exists
    monkeypatch.setattr(cmd.os.path, 'exists',
                        lambda p: True if p == client_server.PORT_STATUS else real_exists(p))
    monkeypatch.setattr(cmd, 'camera_rows', lambda: [])
    rc = cmd.main(['--json', '--no-lidar'])
    boards = json.loads(capsys.readouterr().out)['boards']
    assert [b['board'] for b in boards] == ['pimu', 'esp']
    assert rc == 1
    assert [s.kind for s in sockets] == [zmq.SUB]
    assert sockets[0].connected == ['ipc://%s' % client_server.PORT_STATUS]
    assert sockets[0].sent == []


def test_no_status_socket_means_no_server(monkeypatch):
    from stretch4_body.core import client_server
    real_exists = cmd.os.path.exists
    monkeypatch.setattr(cmd.os.path, 'exists',
                        lambda p: False if p == client_server.PORT_STATUS else real_exists(p))
    assert cmd.read_server_status() is None


def test_age_text():
    assert cmd.age_text(None, T0) == '-'
    assert cmd.age_text(T0 - 5, T0) == '5s'
    assert cmd.age_text(T0 - 600, T0) == '10m'
    assert cmd.age_text(T0 - 3 * 3600, T0) == '3h'
    assert cmd.age_text(T0 - 3 * 86400, T0) == '3d'
