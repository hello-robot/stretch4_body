"""sentry_lidar_idle: /proc/net parsing, consumer naming and the rail state machine.

No hardware and no real sockets: /proc text comes from fixtures, the clock is passed in,
and power_periph is a fake that records set_lidar_on/off the way the real one does.
"""
import os

import pytest

from stretch4_body.behavior.sentries.sentry_lidar_idle import (
    ProcNetScanner, SentryLidarIdle, parse_proc_net)
from stretch4_body.core.robot_params import RobotParams

UDP_HDR = '  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode ref pointer drops\n'
UDP = UDP_HDR + (
    ' 100: 00000000:0940 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 5001 2 0000000000000000 0\n'
    ' 101: 00000000:094A 00000000:0000 07 00000000:00000000 00:00000000 00000000     0        0 5002 2 0000000000000000 0\n'
    ' 102: 00000000:A2A9 00000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 5003 2 0000000000000000 0\n')
UDP6 = UDP_HDR + (
    ' 200: 0000000000000000FFFF00000200A8C0:0940 00000000000000000000000000000000:0000 07 00000000:00000000 00:00000000 00000000  1000        0 6001 2 0000000000000000 0\n')
TCP = UDP_HDR + (
    # 192.168.1.2:40000 -> 192.168.1.201:9347 ESTABLISHED
    '   0: 0201A8C0:9C40 C901A8C0:2483 01 00000000:00000000 00:00000000 00000000  1000        0 7001 1 0000000000000000 20 4 0 10 -1\n'
    # SYN_SENT to the left lidar (a tool trying an unpowered lidar)
    '   1: 0201A8C0:9C41 CA01A8C0:2483 02 00000000:00000000 00:00000000 00000000  1000        0 7002 1 0000000000000000 20 4 0 10 -1\n'
    # TIME_WAIT to a lidar: not a consumer
    '   2: 0201A8C0:9C42 C901A8C0:2483 06 00000000:00000000 03:00000F9E 00000000     0        0 0 3 0000000000000000\n'
    # port 9347 on some other host: not a lidar
    '   3: 0201A8C0:9C43 0101A8C0:2483 01 00000000:00000000 00:00000000 00000000  1000        0 7004 1 0000000000000000 20 4 0 10 -1\n')

PORTS = [2368, 2378]
IPS = ['192.168.1.201', '192.168.1.202']


# --- parser ---------------------------------------------------------------------------------

def test_udp_rows_on_lidar_ports_any_user():
    socks = parse_proc_net(UDP, 'udp', PORTS)
    assert [(s['port'], s['inode'], s['uid']) for s in socks] == [(2368, 5001, 1000), (2378, 5002, 0)]


def test_udp6_ipv4_mapped_row():
    socks = parse_proc_net(UDP6, 'udp', PORTS)
    assert [(s['port'], s['inode']) for s in socks] == [(2368, 6001)]


def test_tcp_ptc_sessions_established_and_syn_sent_only():
    socks = parse_proc_net(TCP, 'tcp', (), 9347, IPS)
    assert [s['inode'] for s in socks] == [7001, 7002]


def test_tcp_without_ip_filter_counts_any_ptc_peer():
    socks = parse_proc_net(TCP, 'tcp', (), 9347, [])
    assert [s['inode'] for s in socks] == [7001, 7002, 7004]


def test_header_and_junk_lines_are_ignored():
    assert parse_proc_net(UDP_HDR + 'garbage\n\n', 'udp', PORTS) == []


# --- scanner against a fake /proc ------------------------------------------------------------

def make_proc(tmp_path, udp=UDP, tcp=TCP):
    net = tmp_path / 'net'
    net.mkdir()
    (net / 'udp').write_text(udp)
    (net / 'udp6').write_text(UDP_HDR)
    (net / 'tcp').write_text(tcp)
    pid = tmp_path / '4242'
    (pid / 'fd').mkdir(parents=True)
    (pid / 'comm').write_text('hesai_ros_drive\n')
    os.symlink('socket:[5001]', str(pid / 'fd' / '7'))
    os.symlink('/dev/null', str(pid / 'fd' / '0'))
    return tmp_path


def test_scanner_names_consumers_and_tolerates_missing_files(tmp_path):
    root = make_proc(tmp_path)
    sc = ProcNetScanner(PORTS, 9347, IPS, proc_root=str(root))
    found = sc.scan()
    assert {'proto': 'udp', 'port': 2368, 'pid': 4242, 'name': 'hesai_ros_drive'} in found
    assert {'proto': 'udp', 'port': 2378, 'pid': None, 'name': '?'} in found
    assert sum(1 for c in found if c['proto'] == 'tcp') == 2


def test_scanner_resolves_names_only_when_sockets_change(tmp_path, monkeypatch):
    root = make_proc(tmp_path)
    sc = ProcNetScanner(PORTS, None, IPS, proc_root=str(root))
    calls = []
    real = sc._resolve
    monkeypatch.setattr(sc, '_resolve', lambda inodes: calls.append(inodes) or real(inodes))
    sc.scan()
    sc.scan()
    assert len(calls) == 1
    (root / 'net' / 'udp').write_text(UDP_HDR)
    assert sc.scan() == []
    assert len(calls) == 1  # empty set needs no lookup


# --- state machine ---------------------------------------------------------------------------

class FakePowerPeriph:
    def __init__(self):
        self.status_aux = {'current_pwr_state_name': 'STATE_ACTIVE'}
        self.lidar_cmd = {'action': None, 'source': None, 'ts': None, 'seq': 0}
        self.sent = []

    def _note(self, action, source):
        self.lidar_cmd.update(action=action, source=source, ts=0.0, seq=self.lidar_cmd['seq'] + 1)

    def set_lidar_on(self, source='client'):
        self.sent.append(('on', source))
        self._note('on', source)

    def set_lidar_off(self, source='client'):
        self.sent.append(('off', source))
        self._note('off', source)

    def release_lidar_hold(self, source='client'):
        self._note('release', source)


class FakeRobot:
    def __init__(self):
        self.power_periph = FakePowerPeriph()
        self.subsystems = {'power_periph': self.power_periph}
        self._power_aux_meta = {'available': True, 'sample_monotonic_s': 0.0}


CONSUMER = [{'proto': 'udp', 'port': 2368, 'pid': 1234, 'name': 'hesai_ros_driver_node'}]


class Rig:
    """Drives the sentry at 1 s steps with a fresh ACTIVE sample unless told otherwise."""

    def __init__(self, monkeypatch, **params):
        base = dict(RobotParams.get_params()[1]['sentry_lidar_idle'])
        base.update(params)
        monkeypatch.setitem(RobotParams.get_params()[1], 'sentry_lidar_idle', base)
        self.robot = FakeRobot()
        self.s = SentryLidarIdle(self.robot)
        self.now = 1000.0
        self.s.clock = lambda: self.now
        self.s.scanner.scan = lambda: []
        assert self.s.startup()
        self.s._stop_evt.set()  # no background thread in tests; snapshots are passed in
        self.s._thread.join()
        self.s._thread = None
        self.fresh = True

    @property
    def pp(self):
        return self.robot.power_periph

    def run(self, seconds, consumers=()):
        end = self.now + seconds
        while self.now < end:
            self.now += 1.0
            if self.fresh:
                self.robot._power_aux_meta['sample_monotonic_s'] = self.now
            self.s.update(self.now, (self.now, list(consumers)))
        return self.s.status


@pytest.fixture
def rig(monkeypatch):
    return Rig(monkeypatch)


def test_params_defaults():
    p = RobotParams.get_params()[1]['sentry_lidar_idle']
    assert p['enabled'] == 1 and p['idle_timeout_s'] == 300 and p['hold_timeout_s'] == 0
    assert 'sentry_lidar_idle' in RobotParams.get_params()[1]['sentry_manager']['controllers']


def test_startup_counts_down_then_cuts_at_idle_timeout(rig):
    st = rig.run(299)
    assert rig.pp.sent == [] and st['rail_cmd'] == 'on' and st['seconds_to_off'] == 1.0
    st = rig.run(1)
    assert rig.pp.sent == [('off', 'sentry')]
    assert st['rail_cmd'] == 'off' and st['rail_cmd_source'] == 'sentry' and st['off_events'] == 1
    assert 'idle 300 s' in st['last_reason']
    rig.run(600)
    assert rig.pp.sent == [('off', 'sentry')]  # one command per transition


def test_consumer_turns_rail_on_within_one_poll(rig):
    rig.run(300)
    st = rig.run(1, CONSUMER)
    assert rig.pp.sent[-1] == ('on', 'sentry')
    assert st['rail_cmd'] == 'on' and st['consumers'] == CONSUMER
    assert 'hesai_ros_driver_node pid 1234' in st['last_reason']


def test_consumer_present_keeps_rail_on_then_idle_after_it_leaves(rig):
    rig.run(1000, CONSUMER)
    assert rig.pp.sent == []
    rig.run(299)
    assert rig.pp.sent == []
    rig.run(1)
    assert rig.pp.sent == [('off', 'sentry')]


def test_min_on_s_respected(monkeypatch):
    r = Rig(monkeypatch, idle_timeout_s=10, min_on_s=60)
    r.run(60)
    assert r.pp.sent == [('off', 'sentry')]
    r.run(1, CONSUMER)
    r.run(10)
    assert r.pp.sent[-1] == ('on', 'sentry')  # idle 10 s but on for only 11 s
    r.run(50)
    assert r.pp.sent[-1] == ('off', 'sentry')


def test_client_on_is_a_hold_never_cut(rig):
    rig.pp.set_lidar_on()
    st = rig.run(5000)
    assert rig.pp.sent == [('on', 'client')]
    assert st['held'] and st['rail_cmd_source'] == 'client' and st['seconds_to_off'] is None


def test_release_restarts_countdown(rig):
    rig.pp.set_lidar_on()
    rig.run(1000)
    rig.pp.release_lidar_hold()
    rig.run(1)  # seen on the next step; the countdown starts there
    st = rig.run(299)
    assert not st['held'] and rig.pp.sent == [('on', 'client')]
    rig.run(1)
    assert rig.pp.sent[-1] == ('off', 'sentry')


def test_hold_timeout_param(monkeypatch):
    r = Rig(monkeypatch, hold_timeout_s=600)
    r.pp.set_lidar_on()
    st = r.run(1)
    assert st['held'] and st['hold_remaining_s'] == 600
    st = r.run(599)
    assert st['held']
    st = r.run(1)
    assert not st['held']
    r.run(299)
    assert r.pp.sent == [('on', 'client')]
    r.run(1)
    assert r.pp.sent[-1] == ('off', 'sentry')


def test_client_off_obeyed_and_a_later_consumer_turns_on(rig):
    rig.pp.set_lidar_off()
    st = rig.run(100)
    assert st['rail_cmd'] == 'off' and st['rail_cmd_source'] == 'client'
    assert rig.pp.sent == [('off', 'client')]
    rig.run(1, CONSUMER)
    assert rig.pp.sent[-1] == ('on', 'sentry')


def test_on_suppressed_when_pimu_not_active_or_stale(rig):
    rig.run(300)
    rig.pp.status_aux['current_pwr_state_name'] = 'STATE_SLEEP'
    st = rig.run(5, CONSUMER)
    assert rig.pp.sent == [('off', 'sentry')] and 'PIMU state inactive' in st['last_reason']
    rig.pp.status_aux['current_pwr_state_name'] = 'STATE_ACTIVE'
    rig.fresh = False
    rig.now += 3
    st = rig.run(5, CONSUMER)
    assert rig.pp.sent == [('off', 'sentry')] and st['pimu_state'] == 'unknown'


def test_no_cut_without_fresh_active_state(rig):
    rig.fresh = False
    rig.now += 3
    rig.run(1000)
    assert rig.pp.sent == []


def test_wake_resets_to_on_with_full_countdown(rig):
    rig.run(300)
    rig.pp.status_aux['current_pwr_state_name'] = 'STATE_SLEEP_CHRG'
    rig.run(10)
    rig.pp.status_aux['current_pwr_state_name'] = 'STATE_ACTIVE'
    st = rig.run(1)
    assert st['rail_cmd'] == 'on' and st['rail_cmd_source'] == 'firmware_wake'
    rig.run(299)
    assert rig.pp.sent == [('off', 'sentry')]
    rig.run(1)
    assert rig.pp.sent == [('off', 'sentry'), ('off', 'sentry')]


def test_stop_restores_only_a_sentry_cut_while_active(rig):
    rig.run(300)
    rig.s.stop()
    assert rig.pp.sent == [('off', 'sentry'), ('on', 'sentry')]


def test_stop_leaves_a_client_off_alone(rig):
    rig.pp.set_lidar_off()
    rig.run(1)
    rig.s.stop()
    assert rig.pp.sent == [('off', 'client')]


def test_stop_does_not_restore_outside_active(rig):
    rig.run(300)
    rig.pp.status_aux['current_pwr_state_name'] = 'STATE_SLEEP'
    rig.run(1)
    rig.s.stop()
    assert rig.pp.sent == [('off', 'sentry')]


def test_no_decision_before_first_scan(rig):
    rig.now += 1000
    rig.robot._power_aux_meta['sample_monotonic_s'] = rig.now
    rig.s.update(rig.now, None)
    assert rig.pp.sent == [] and rig.s.status['scan_age_s'] is None


def test_enabled_0_sentry_not_built(monkeypatch):
    from stretch4_body.behavior.sentries.sentry_manager import SentryManager
    params = RobotParams.get_params()[1]
    monkeypatch.setitem(params, 'sentry_manager', {'controllers': ['sentry_lidar_idle']})
    monkeypatch.setitem(params, 'sentry_lidar_idle', dict(params['sentry_lidar_idle'], enabled=0))
    assert 'sentry_lidar_idle' not in SentryManager(FakeRobot()).sentries
    monkeypatch.setitem(params, 'sentry_lidar_idle', dict(params['sentry_lidar_idle'], enabled=1))
    assert 'sentry_lidar_idle' in SentryManager(FakeRobot()).sentries
