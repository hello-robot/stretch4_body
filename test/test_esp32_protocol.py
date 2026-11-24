"""
ESP32 (stretch4_body.subsystem.esp32) without hardware: the RPC protocol (packed layouts,
unpacking, packing, validation, the log format), and the Firebase set-up (Esp32Firebase:
the rules, the database URL check, and the Firebase CLI wrapper with subprocess mocked;
the rules also in the Realtime Database emulator, where it is installed).
"""
import base64
import glob
import json
import math
import os
import re
import shutil
import socket
import struct
import subprocess
import time
import urllib.error
import urllib.request
from unittest import mock

import pytest

import stretch4_body.subsystem.esp32 as esp32
from stretch4_body.core.device import DeviceTimestamp
from stretch4_body.subsystem.esp32 import (Esp32Defn, Esp32Base, Esp32_Protocol_P1, esp32_status_template,
                                           firmware_version_tuple, generate_robot_key)

D = Esp32Defn

# Sizes asserted in stretch_firmware_ii/arduino/hello_esp/EspRpc.h
FIRMWARE_SIZES = {
    'FMT_BOARD_INFO': 50,
    'FMT_STATUS': 91,           # Up to hello-esp v0.2.3
    'FMT_STATUS_BATTERY': 92,   # hello-esp v0.2.4 on: + the Pimu's battery status
    'FMT_WIFI_STATUS': 112,
    'FMT_FIREBASE_STATUS': 151,
    'FMT_WIFI_CONFIG': 99,
    'FMT_FIREBASE_CONFIG': 827,      # hello-esp v0.2.6 on (with fleet_id and robot_key)
    'FMT_CONFIG': 11,
    'FMT_TRIGGER': 4,
}

ESP_RPC_H = os.path.join(os.path.dirname(__file__), '..', '..', 'stretch_firmware_ii', 'arduino', 'hello_esp',
                         'EspRpc.h')


KEY = 'AbCdEfGhIjKlMnOpQrStUvWxYz012345'  # 32 characters, as generate_robot_key makes
FLEET = 'stretch-se4-4024'


def p1():
    """An Esp32_Protocol_P1 with just what the unpackers use: no params, no port."""
    e = object.__new__(Esp32_Protocol_P1)
    e.status = esp32_status_template()
    e.timestamp = DeviceTimestamp()
    e.board_info = {'board_variant': None, 'firmware_version': None, 'protocol_version': None, 'hardware_id': 0}
    return e


class TestLayouts:
    @pytest.mark.parametrize('fmt,size', FIRMWARE_SIZES.items())
    def test_sizes_match_firmware(self, fmt, size):
        assert struct.calcsize(getattr(D, fmt)) == size

    def test_scan_size(self):
        assert struct.calcsize(D.FMT_SCAN_HEADER) + D.SCAN_MAX * struct.calcsize(D.FMT_SCAN_ENTRY) == 722

    def test_replies_fit_the_transport(self):
        # A pull reply is at most 1024 bytes after the RPC ID
        for fmt in ('FMT_STATUS', 'FMT_WIFI_STATUS', 'FMT_FIREBASE_STATUS', 'FMT_FIREBASE_CONFIG'):
            assert struct.calcsize(getattr(D, fmt)) <= 1024
        # A push reply rides in one 58 byte frame: the SET replies are a byte, the trigger echo four
        assert struct.calcsize(D.FMT_TRIGGER) <= 57

    @pytest.mark.skipif(not os.path.exists(ESP_RPC_H), reason='stretch_firmware_ii not checked out alongside')
    def test_firmware_header_agrees(self):
        src = open(ESP_RPC_H).read()
        asserted = dict(re.findall(r'static_assert\(sizeof\((\w+)\) == (\d+),', src))
        assert int(asserted['Esp32_Status']) == struct.calcsize(D.FMT_STATUS_BATTERY)
        assert int(asserted['Esp32_Wifi_Status']) == struct.calcsize(D.FMT_WIFI_STATUS)
        assert int(asserted['Esp32_Firebase_Status']) == struct.calcsize(D.FMT_FIREBASE_STATUS)
        assert int(asserted['Esp32_Wifi_Config']) == struct.calcsize(D.FMT_WIFI_CONFIG)
        assert int(asserted['Esp32_Firebase_Config']) == struct.calcsize(D.FMT_FIREBASE_CONFIG)
        assert int(asserted['Esp32_Config']) == struct.calcsize(D.FMT_CONFIG)
        ids = dict((k, int(v)) for k, v in re.findall(r'#define (RPC_\w+)\s+(\d+)', src))
        for name, value in ids.items():
            assert getattr(D, name) == value, name
        assert int(re.search(r'#define ESP32_FB_KEY_LEN\s+(\d+)', src).group(1)) == D.FB_KEY_LEN
        assert int(re.search(r'#define ESP32_FB_KEY_MIN_LEN\s+(\d+)', src).group(1)) == D.FB_KEY_MIN_LEN
        for name, value in re.findall(r'#define ESP32_FB_(\w+)\s+(\d+)', src):
            if name.endswith('_LEN'):
                continue
            assert D.FIREBASE_STATE_NAMES[int(value)] == name, name
        for name, value in re.findall(r'#define ESP32_CMD_RESULT_(\w+)\s+(\d+)', src):
            assert D.CMD_RESULT_NAMES[int(value)] == name, name
        assert int(re.search(r'#define ESP32_CMD_ID_LEN\s+(\d+)', src).group(1)) == D.CMD_ID_LEN
        triggers = dict((k, int(v)) for k, v in re.findall(r'#define ESP32_(TRIGGER_\w+)\s+\(1U << (\d+)\)', src))
        for name, bit in triggers.items():
            assert getattr(D, name) == 1 << bit, name


class TestUnpack:
    def test_status(self):
        values = [123456789, 3, 1, 80, 200000, 150000, 1234, 56, 7, 2, 12.1, 19.8,
                  1, 0, 1,                     # adapter connected, fault, NUC USB 5V
                  1, 0, 1, 1, 0, 1, 0, 0, 2,   # outputs, sd_pulse_count
                  24.5, 19.9, 35.0,            # Pimu battery, 20V0, temperature
                  87, 1, 0, 2, 0, 1,           # soc, charging, runstop, cpu power, rail off, fresh
                  12, 1000, 3, 0, 900, 1, 42]  # age, rx, crc, overflow, tx, drops, counter
        e = p1()
        e._unpack_status(struct.pack(D.FMT_STATUS, *values))
        s = e.status
        assert s['pwr_state_name'] == 'SLEEP_CHRG'
        assert s['reset_reason_name'] == 'POWERON'
        assert s['cpu_freq_mhz'] == 80
        assert s['voltage_12v0'] == pytest.approx(12.1, abs=1e-5)
        assert s['adapter_connected'] and not s['adapter_fault'] and s['cpu_5v_usb_on']
        assert s['outputs']['en_12v0'] and not s['outputs']['en_aux_20v0'] and s['outputs']['robot_active']
        assert s['outputs']['sd_pulse_count'] == 2
        p = s['pimu_link']
        assert p['battery_soc'] == 87 and p['charger_charging'] and not p['runstop_event']
        assert p['cpu_power_state_name'] == 'ON' and p['status_fresh']
        assert p['status_age_ms'] == 12 and p['uart_crc_errors'] == 3 and p['esp_status_counter'] == 42

    def test_status_battery_status(self):
        base = [123456789, 3, 1, 80, 200000, 150000, 1234, 56, 7, 2, 12.1, 19.8,
                1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 0, 2,
                24.5, 19.9, 35.0, 87, 1, 0, 2, 0, 1,
                12, 1000, 3, 0, 900, 1, 42]
        e = p1()
        # Older firmware: no status byte, values used as before
        e._unpack_status(struct.pack(D.FMT_STATUS, *base))
        p = e.status['pimu_link']
        assert p['battery_status'] == 'UNKNOWN' and p['battery_valid'] and p['battery_soc'] == 87
        e._unpack_status(struct.pack(D.FMT_STATUS_BATTERY, *base, 1))
        assert p['battery_status'] == 'OK' and p['battery_valid'] and p['battery_soc'] == 87
        # Lost or not detected: the Pimu sends NaN and 255, both read as NaN
        lost = list(base)
        lost[24] = float('nan')
        lost[27] = 255
        for code, name in ((2, 'BMS_LOST'), (3, 'NOT_DETECTED')):
            e._unpack_status(struct.pack(D.FMT_STATUS_BATTERY, *lost, code))
            assert p['battery_status'] == name and not p['battery_valid']
            assert math.isnan(p['battery_soc']) and math.isnan(p['voltage_battery'])

    def test_status_never_received(self):
        values = [0] * 33 + [D.UNKNOWN_AGE] + [0] * 6
        values[10:12] = [0.0, 0.0]
        values[24:27] = [0.0, 0.0, 0.0]
        e = p1()
        e._unpack_status(struct.pack(D.FMT_STATUS, *values))
        assert e.status['pimu_link']['status_age_ms'] is None

    def test_wifi_status(self):
        packed = struct.pack(D.FMT_WIFI_STATUS, 3, 3, 1, 0, b'HelloNet', -61, 6, bytes([1, 2, 3, 4, 5, 6]),
                             bytes([0xAA] * 6), bytes([192, 168, 1, 20]), bytes([192, 168, 1, 1]),
                             bytes([255, 255, 255, 0]), bytes([8, 8, 8, 8]), b'stretch-esp', 4, 2, 201, 65000)
        e = p1()
        e._unpack_wifi_status(packed)
        w = e.status['wifi']
        assert w['state_name'] == 'CONNECTED' and w['wl_status_name'] == 'CONNECTED'
        assert w['ssid'] == 'HelloNet' and w['rssi'] == -61 and w['channel'] == 6
        assert w['bssid'] == '01:02:03:04:05:06' and w['ip'] == '192.168.1.20' and w['netmask'] == '255.255.255.0'
        assert w['hostname'] == 'stretch-esp'
        assert w['last_disconnect_reason_name'] == 'NO_AP_FOUND'
        assert w['connected_s'] == pytest.approx(65.0)

    def test_firebase_status(self):
        packed = struct.pack(D.FMT_FIREBASE_STATUS, 4, 1, 1, 1, 2, 100, 3, 1500, 5000, -118, b'connection refused',
                             9, 12000, 1, 1, 2, 2, 3500, b'-NxAbc123')
        e = p1()
        e._unpack_firebase_status(packed)
        f = e.status['firebase']
        assert f['state_name'] == 'READY' and f['publish_ok'] == 100 and f['publish_errors'] == 3
        assert f['last_publish_age_s'] == pytest.approx(1.5) and f['publish_period_s'] == pytest.approx(5.0)
        assert f['last_error_code'] == -118 and f['last_error'] == 'connection refused'
        assert f['stream_events'] == 9 and f['stream_age_s'] == pytest.approx(12.0) and f['stream_restarts'] == 1
        assert f['time_synced']
        assert f['cmd_action_name'] == 'WAKE' and f['cmd_result_name'] == 'DONE'
        assert f['cmd_age_s'] == pytest.approx(3.5) and f['cmd_id'] == '-NxAbc123'

    def test_firebase_status_no_stream_no_command(self):
        packed = struct.pack(D.FMT_FIREBASE_STATUS, 5, 1, 1, 1, 1, 0, 0, D.UNKNOWN_AGE, 5000, 0, b'',
                             0, D.UNKNOWN_AGE, 0, 0, 0, 0, D.UNKNOWN_AGE, b'')
        e = p1()
        e._unpack_firebase_status(packed)
        f = e.status['firebase']
        assert f['state_name'] == 'STREAM_CONNECTING'
        assert f['stream_age_s'] is None and f['cmd_age_s'] is None and f['cmd_action_name'] == 'NONE'

    def test_board_info(self):
        e = p1()
        e._unpack_board_info(struct.pack(D.FMT_BOARD_INFO, b'hello-esp32.0', b'hello-esp.v0.2.0p1'))
        assert e.board_info['firmware_version'] == 'hello-esp.v0.2.0p1'
        assert e.board_info['protocol_version'] == 'p1'
        assert e.board_info['hardware_id'] == 0


class TestPack:
    def test_wifi_config(self):
        e = p1()
        e.set_wifi_config('HelloNet', 'password123', persist=True, apply_now=False)
        ssid, password, flags = struct.unpack(D.FMT_WIFI_CONFIG, e._pending_wifi_config)
        assert ssid.rstrip(b'\0') == b'HelloNet' and password.rstrip(b'\0') == b'password123'
        assert flags == D.CFG_FLAG_PERSIST

    def test_open_network(self):
        e = p1()
        e.set_wifi_config('Cafe', '')
        assert struct.unpack(D.FMT_WIFI_CONFIG, e._pending_wifi_config)[2] == D.CFG_FLAG_PERSIST | D.CFG_FLAG_APPLY_NOW

    def test_firebase_config_robot_key_only(self):
        e = p1()
        e.set_firebase_config('https://x.firebaseio.com', FLEET, KEY, apply_now=False)
        url, fleet, key, api, email, pw, flags = struct.unpack(D.FMT_FIREBASE_CONFIG, e._pending_firebase_config)
        assert url.rstrip(b'\0') == b'https://x.firebaseio.com' and fleet.rstrip(b'\0') == FLEET.encode()
        assert key.rstrip(b'\0') == KEY.encode()
        assert api.rstrip(b'\0') == email.rstrip(b'\0') == pw.rstrip(b'\0') == b''
        assert flags == D.CFG_FLAG_PERSIST

    def test_firebase_config_sign_in(self):
        e = p1()
        e.set_firebase_config('https://x.firebaseio.com', FLEET, KEY, 'api', 'a@b.c', 'pw')
        assert e._unpack_firebase_config(e._pending_firebase_config) == {
            'url': 'https://x.firebaseio.com', 'fleet_id': FLEET, 'robot_key': KEY, 'api_key': 'api',
            'user_email': 'a@b.c', 'user_password': 'pw', 'configured': False}

    def test_firebase_config_clear(self):
        e = p1()
        e.set_firebase_config('', '', '')
        assert not any(e._unpack_firebase_config(e._pending_firebase_config).values())

    def test_supports_firebase(self):
        # Earlier p1 firmware has another Firebase config layout
        e = p1()
        for version, ok in ((None, False), ('hello-esp.v0.2.0p1', False), ('hello-esp.v0.2.5p1', False),
                            ('hello-esp.v0.2.6p1', True), ('hello-esp.v0.2.10p1', True), ('hello-esp.v1.0.0p1', True)):
            e.board_info['firmware_version'] = version
            assert e.supports_firebase() == ok, version
        assert firmware_version_tuple('hello-esp.v0.2.10p1') == (0, 2, 10) and firmware_version_tuple('x') == ()

    def test_firmware_supports_firebase(self):
        # The firmware's own version passes the check
        common_h = os.path.join(os.path.dirname(ESP_RPC_H), 'Common.h')
        if not os.path.exists(common_h):
            pytest.skip('stretch_firmware_ii is not next to stretch4_body')
        version = re.search(r'#define FIRMWARE_VERSION_HR "([^"]+)"', open(common_h).read()).group(1)
        assert version.endswith('p1')
        assert firmware_version_tuple(version) >= firmware_version_tuple(D.FIRST_FIREBASE_FIRMWARE)

    def test_config(self):
        e = p1()
        packed = e._pack_config({'firebase_publish_period_ms': 2000, 'wifi_retry_ms': 7000, 'wifi_enable': 1,
                                 'firebase_enable': 0, 'log_level': 2})
        assert struct.unpack(D.FMT_CONFIG, packed) == (2000, 7000, 1, 0, 2)


class TestValidate:
    @pytest.mark.parametrize('ssid,password', [('', 'password123'), ('net', 'short'), ('x' * 33, ''),
                                               ('net', 'p' * 65)])
    def test_bad_wifi(self, ssid, password):
        with pytest.raises(ValueError):
            Esp32Base.validate_wifi_config(ssid, password)

    def test_good_wifi(self):
        Esp32Base.validate_wifi_config('net', '')
        Esp32Base.validate_wifi_config('net', 'p' * 64)

    def test_firebase_good(self):
        Esp32Base.validate_firebase_config('', '', '')                          # Clears Firebase
        Esp32Base.validate_firebase_config('u', FLEET, KEY)                     # Robot key only
        Esp32Base.validate_firebase_config('u', FLEET, KEY, 'k', 'e', 'p')      # And signs in
        Esp32Base.validate_firebase_config('u', 'f' * 64, 'a-_' + 'x' * 61)     # 64, the longest

    @pytest.mark.parametrize('fields', [
        ('u', FLEET, ''), ('', FLEET, KEY), ('u', '', KEY),  # The URL, fleet ID and key go together
        ('u', FLEET, KEY, 'k', '', 'p'),                     # The sign-in is all three or none
        ('u', FLEET, 'x' * 21),                              # Too short
        ('u', FLEET, 'x' * 65),                              # Too long
        ('u', FLEET, KEY[:-1] + '/'), ('u', FLEET, KEY[:-1] + '"'),  # Not in a path, or in JSON
        ('u', 'stretch/se4', KEY), ('u', 'stretch.se4', KEY), ('u', 'f' * 65, KEY),  # Not a path segment
        ('u' * 251, FLEET, KEY),
        ('', '', '', 'k', 'e', 'p'),
    ])
    def test_firebase_bad(self, fields):
        with pytest.raises(ValueError):
            Esp32Base.validate_firebase_config(*fields)

    def test_generated_keys(self):
        keys = {generate_robot_key() for _ in range(50)}
        assert len(keys) == 50
        for k in keys:
            assert len(k) == 32
            Esp32Base.validate_robot_key(k)


class TestLog:
    def test_unpack(self):
        entries = [(1000, D.LOG_INFO, b'Wi-Fi connected'), (2500, D.LOG_IDF, b'E (123) wifi: oops')]
        buf = struct.pack('<HB', 3, len(entries))
        for ms, level, text in entries:
            buf += struct.pack('<IBB', ms, level, len(text)) + text
        out = Esp32Base.unpack_log(buf)
        assert out[0]['level_name'] == 'DROPPED' and '3' in out[0]['text']
        assert out[1] == {'ms': 1000, 'level': D.LOG_INFO, 'level_name': 'INFO', 'text': 'Wi-Fi connected'}
        assert out[2]['level_name'] == 'IDF'

    def test_empty(self):
        assert Esp32Base.unpack_log(struct.pack('<HB', 0, 0)) == []


# ############################# Firebase set-up (Esp32Firebase) #############################

fb = esp32.Esp32Firebase


class TestRules:
    def test_paths_and_key_check(self):
        for sign_in in (False, True):
            rules = fb.rules(sign_in)['rules']
            # Nothing at the root, on robots/, on robots/<fleet> or on robot_key: no listing,
            # and the key can be read or written only from the console or the CLI
            assert list(rules) == ['robots'] and list(rules['robots']) == ['$fleetId']
            fleet = rules['robots']['$fleetId']
            assert sorted(fleet) == ['power_system_requests', 'power_system_status']
            req = fleet['power_system_requests']
            writes = [fleet['power_system_status']['.write'], req['power_command']['.write'],
                      req['power_command_ack']['.write']]
            assert all(fb.KEY_CHECK in w for w in writes) and '.write' not in req
            assert ("root.child('robots').child($fleetId).child('robot_key').val()" in fb.KEY_CHECK and
                    "newData.child('robot_key')" in fb.KEY_CHECK)
            read = fleet['power_system_status']['.read']
            assert read == req['.read'] == (fb.ASSIGNED if sign_in else True)
            assert all(w.startswith(fb.ASSIGNED + ' && ') == sign_in for w in writes)
        # Assigned the way web teleop assigns users
        for part in ("child('robots').child($fleetId).val() === true", "child('role').val() === 'robot'",
                     "child('name').val() === $fleetId", "root.child('uids').child(auth.uid)"):
            assert part in fb.ASSIGNED

    LOCKED = '{\n  "rules": {\n    ".read": false,\n    ".write": false\n  }\n}'
    TEST_MODE = '{"rules": {".read": "now < 1735689600000",  // 2025-1-1\n ".write": "now < 1735689600000"}}'
    OTHER = '{"rules": {"robots": {"$fleetId": {".read": "auth != null", "uid": {".write": true}}}, "maps": {}}}'

    def test_classify(self):
        assert fb.classify_rules(self.LOCKED) == fb.classify_rules(self.TEST_MODE) == 'default'
        assert fb.classify_rules('{"rules": {".read": "auth != null", ".write": "auth != null"}}') == 'default'
        assert fb.classify_rules(fb.rules_json()) == fb.classify_rules(fb.rules_json(True)) == 'ours'
        assert fb.classify_rules('/* a comment */\n' + fb.rules_json()) == 'ours'
        assert fb.classify_rules(json.dumps({'rules': {'robots': {'$robot_key': {}}}})) == 'ours'  # The earlier rules
        earlier_sign_in = {'rules': {'robots': {'$fleetId': {'power_system_status': {'.read': 'auth != null'},
                                                             'power_system_requests': {'.read': 'auth != null'}}}}}
        assert fb.classify_rules(json.dumps(earlier_sign_in)) == 'ours'
        assert fb.classify_rules(json.dumps({'rules': {'robots': {'$fleetId': {'status': {}}}}})) == 'other'
        assert fb.classify_rules(self.OTHER) == 'other'
        assert fb.classify_rules('{"rules": {".read": true, ".write": true}}') == 'other'  # Open to all: not ours to take
        assert fb.classify_rules('not json') == fb.classify_rules(None) == 'other'

    def test_strip_comments_keeps_strings(self):
        src = '{"a": "x // y /* z */", /* c */ "b": 1 // d\n}'
        assert json.loads(fb.strip_comments(src)) == {'a': 'x // y /* z */', 'b': 1}

    WEB_TELEOP_RULES = os.path.expanduser('~/ament_ws/src/stretch4_web_teleop/database.rules.json')

    def test_merge_keeps_everything_else(self):
        theirs = {'rules': {'robots': {'$fleetId': {'.read': 'auth != null', 'status': {'.write': 'auth != null'},
                                                    '$other': {'.validate': False}}},
                            'maps': {'.read': True}}}
        merged, changes = fb.merge_rules(json.dumps(theirs))
        node = merged['rules']['robots']['$fleetId']
        assert node['power_system_status'] == fb.rules()['rules']['robots']['$fleetId']['power_system_status']
        for key in ('power_system_status', 'power_system_requests'):
            del node[key]
        assert merged == theirs and len(changes) == 2 and all(c.startswith('added') for c in changes)

    def test_merge_replaces_its_own_and_renames_the_wildcard(self):
        theirs = {'rules': {'robots': {'$robot': {'power_system_status': {'.read': 'auth != null'}, 'x': {}}}}}
        merged, changes = fb.merge_rules(json.dumps(theirs), sign_in=True)
        node = merged['rules']['robots']['$robot']
        assert set(node) == {'power_system_status', 'power_system_requests', 'x'}
        text = json.dumps(node)
        assert '$robot' in text and '$fleetId' not in text  # Their wildcard's name in the expressions
        assert changes[0].startswith('replaced') and changes[1].startswith('added')

    def test_merge_adds_robots(self):
        merged, _ = fb.merge_rules('// theirs\n{"rules": {"maps": {".read": true}}}')
        assert merged == {'rules': {'maps': {'.read': True}, 'robots': fb.rules()['rules']['robots']}}
        merged, _ = fb.merge_rules('{"rules": {"robots": {"public": {".read": true}}}}')  # No wildcard there
        assert set(merged['rules']['robots']) == {'public', '$fleetId'}

    @pytest.mark.parametrize('bad', ['not json', '{"x": 1}', '{"rules": {"robots": 3}}',
                                     '{"rules": {"robots": {"$id": true}}}'])
    def test_merge_refuses(self, bad):
        with pytest.raises(ValueError):
            fb.merge_rules(bad)

    def test_merge_web_teleop_rules(self):
        if not os.path.exists(self.WEB_TELEOP_RULES):
            pytest.skip('stretch4_web_teleop is not checked out')
        source = open(self.WEB_TELEOP_RULES).read()
        merged, changes = fb.merge_rules(source)
        node = merged['rules']['robots']['$fleetId']
        for key in ('power_system_status', 'power_system_requests'):
            del node[key]
        assert merged == json.loads(source) and len(changes) == 2


class TestDefaults:
    def test_from_fleet_id(self):
        assert fb.default_project_id('stretch-se4-4024') == 'stretch-se4-4024-pwr-mgmt'
        assert fb.default_database_url('stretch-se4-4024') == \
            'https://stretch-se4-4024-pwr-mgmt-default-rtdb.firebaseio.com'
        # Project IDs are lowercase letters, digits and hyphens
        assert fb.default_project_id('Stretch_SE4.4024') == 'stretch-se4-4024-pwr-mgmt'
        assert fb.normalize_database_url(fb.default_database_url('stretch-se4-4024')) == \
            fb.default_database_url('stretch-se4-4024')

    def test_environment(self):
        with mock.patch.dict(os.environ, {'HELLO_FLEET_ID': 'stretch-se4-1001'}):
            assert fb.default_project_id() == 'stretch-se4-1001-pwr-mgmt'
        with mock.patch.dict(os.environ, {}, clear=True):
            assert fb.default_project_id() is None and fb.default_database_url() is None


class TestDatabaseUrl:
    @pytest.mark.parametrize('typed,url', [
        ('https://proj-default-rtdb.firebaseio.com', 'https://proj-default-rtdb.firebaseio.com'),
        ('https://proj-default-rtdb.firebaseio.com/', 'https://proj-default-rtdb.firebaseio.com'),
        (' proj-default-rtdb.firebaseio.com ', 'https://proj-default-rtdb.firebaseio.com'),
        ('https://proj-default-rtdb.europe-west1.firebasedatabase.app/',
         'https://proj-default-rtdb.europe-west1.firebasedatabase.app'),
        ('HTTPS://Proj.FirebaseIO.com', 'https://proj.firebaseio.com'),
    ])
    def test_good(self, typed, url):
        assert fb.normalize_database_url(typed) == url

    @pytest.mark.parametrize('typed', [
        '', 'https://console.firebase.google.com/project/proj/database/proj-default-rtdb/data',
        'http://proj.firebaseio.com', 'https://proj.firebaseio.com/robots', 'https://example.com',
        'https://.firebaseio.com',
    ])
    def test_bad(self, typed):
        with pytest.raises(ValueError):
            fb.normalize_database_url(typed)


def completed(stdout, returncode=0, stderr=''):
    return subprocess.CompletedProcess([], returncode, stdout=stdout, stderr=stderr)


class TestCli:
    def test_result(self):
        out = '(node:1) DeprecationWarning\n' + json.dumps({'status': 'success', 'result': [{'projectId': 'p'}]})
        with mock.patch.object(fb, 'cli_path', return_value='/usr/bin/firebase'), \
                mock.patch.object(esp32.subprocess, 'run', return_value=completed(out)) as run:
            assert fb.list_projects() == [{'projectId': 'p'}]
        cmd = run.call_args[0][0]
        assert cmd[:2] == ['/usr/bin/firebase', 'projects:list'] and '--json' in cmd and '--non-interactive' in cmd

    def test_error(self):
        out = json.dumps({'status': 'error', 'error': 'Failed to list Firebase projects.'})
        with mock.patch.object(esp32.subprocess, 'run', return_value=completed(out, 1)):
            with pytest.raises(fb.CliError, match='Failed to list'):
                fb.list_projects()

    def test_no_json(self):
        with mock.patch.object(esp32.subprocess, 'run', return_value=completed('', 1, 'command not found')):
            with pytest.raises(fb.CliError, match='command not found'):
                fb.run_cli(['projects:list'])

    def test_expired_login(self):
        # --json says only "Failed to ...": the reason is in firebase-debug.log, in the directory it runs in
        def run(cmd, cwd=None, **kw):
            with open(os.path.join(cwd, 'firebase-debug.log'), 'w') as f:
                f.write('[error] Authentication Error: Your credentials are no longer valid. '
                        'Please run firebase login --reauth\n')
            return completed(json.dumps({'status': 'error', 'error': 'Failed to list Firebase projects. '
                                                                     'See firebase-debug.log for more info.'}), 1)
        with mock.patch.object(esp32.subprocess, 'run', side_effect=run):
            with pytest.raises(fb.AuthError):
                fb.list_projects()

    def test_not_logged_in(self):
        err = json.dumps({'status': 'error', 'error': 'Failed to authenticate, have you run firebase login?'})
        with mock.patch.object(esp32.subprocess, 'run', return_value=completed(err, 1)):
            with pytest.raises(fb.AuthError):
                fb.list_projects()

    def test_other_errors_are_not_auth(self):
        err = json.dumps({'status': 'error', 'error': 'Failed to list Firebase projects.'})
        with mock.patch.object(esp32.subprocess, 'run', return_value=completed(err, 1)):
            with pytest.raises(fb.CliError) as e:
                fb.list_projects()
        assert not isinstance(e.value, fb.AuthError)

    def test_runs_in_a_temporary_directory(self):
        # So firebase-debug.log isn't left where the user ran the tool
        ok = json.dumps({'status': 'success', 'result': []})
        with mock.patch.object(esp32.subprocess, 'run', return_value=completed(ok)) as run:
            fb.list_projects()
        cwd = run.call_args[1]['cwd']
        assert cwd and cwd != os.getcwd() and not os.path.exists(cwd)

    @staticmethod
    def _get_returning(value):
        def run(cmd, cwd=None, **kw):
            if cmd[1] == 'database:get':
                with open(cmd[cmd.index('--output') + 1], 'w') as f:
                    f.write(json.dumps(value))
            return completed(json.dumps({'status': 'success'}))
        return run

    def test_get_value(self):
        with mock.patch.object(esp32.subprocess, 'run', side_effect=self._get_returning({'a': 1})) as r:
            assert fb.get_value('proj', 'inst', '/x/y') == {'a': 1}
        assert r.call_args[0][0][1:3] == ['database:get', '/x/y']
        with mock.patch.object(esp32.subprocess, 'run', side_effect=self._get_returning(None)):
            assert fb.get_value('proj', 'inst', '/x/y') is None

    def test_assignment_needed_only_reads(self):
        for current, needed in ((None, True), ({'robots': {'other': True}}, True), ({'role': 'robot', 'name': 'x'}, True),
                                ({'robots': {FLEET: True}}, False), ({'role': 'robot', 'name': FLEET}, False)):
            with mock.patch.object(esp32.subprocess, 'run', side_effect=self._get_returning(current)) as r:
                got = fb.assignment_needed('proj', 'inst', 'UID9', FLEET)
            assert got == ('/assignments/UID9/robots/%s' % FLEET if needed else None), current
            assert [c[0][0][1] for c in r.call_args_list] == ['database:get']  # Nothing written

    def test_set_true(self):
        ok = json.dumps({'status': 'success'})
        with mock.patch.object(esp32.subprocess, 'run', return_value=completed(ok)) as r:
            fb.set_true('proj', 'inst', '/assignments/UID9/robots/%s' % FLEET)
        assert r.call_args[0][0][1:5] == ['database:set', '/assignments/UID9/robots/%s' % FLEET, '--data', 'true']

    def test_set_robot_key(self):
        ok = json.dumps({'status': 'success', 'result': None})
        with mock.patch.object(esp32.subprocess, 'run', return_value=completed(ok)) as run:
            fb.set_robot_key('proj', 'proj-default-rtdb', FLEET, 'KEY')
        a = run.call_args[0][0]
        assert a[1:5] == ['database:set', '/robots/%s/robot_key' % FLEET, '--data', '"KEY"']
        assert a[a.index('--instance') + 1] == 'proj-default-rtdb' and a[a.index('--project') + 1] == 'proj'

    def test_web_api_key(self):
        ok = lambda r: completed(json.dumps({'status': 'success', 'result': r}))
        config = ok({'fileName': 'firebase-config.json', 'sdkConfig': {'apiKey': 'AIzaKEY', 'projectId': 'proj'}})
        with mock.patch.object(esp32.subprocess, 'run', side_effect=[ok([{'appId': '1:2:web:3'}]), config]) as r:
            assert fb.web_api_key('proj') == 'AIzaKEY'
        assert [c[0][0][1:4] for c in r.call_args_list] == [['apps:list', 'WEB', '--project'],
                                                           ['apps:sdkconfig', 'WEB', '1:2:web:3']]
        with mock.patch.object(esp32.subprocess, 'run', side_effect=[ok([])]):
            assert fb.web_api_key('proj') is None  # No web app, and not asked to make one
        with mock.patch.object(esp32.subprocess, 'run', side_effect=[ok([]), ok({'appId': '1:2:web:9'}), config]) as r:
            assert fb.web_api_key('proj', create=True) == 'AIzaKEY'
        assert r.call_args_list[1][0][0][1:4] == ['apps:create', 'WEB', 'Stretch power']

    def test_current_rules(self):
        # Read from /.settings/rules into a file, comments and all
        def run(cmd, cwd=None, **kw):
            with open(cmd[cmd.index('--output') + 1], 'w') as f:
                f.write('// mine\n' + TestRules.OTHER + '\n')
            return completed(json.dumps({'status': 'success'}))
        with mock.patch.object(esp32.subprocess, 'run', side_effect=run) as r:
            assert fb.current_rules('proj', 'inst').startswith('// mine\n')
        assert r.call_args[0][0][1:3] == ['database:get', '/.settings/rules']

    def test_deploy_writes_rules(self):
        seen = {}

        def run(cmd, cwd=None, **kw):
            seen['firebase.json'] = json.load(open(cwd + '/firebase.json'))
            seen['rules'] = open(cwd + '/database.rules.json').read()
            seen['cmd'] = cmd
            return completed(json.dumps({'status': 'success', 'result': {}}))
        with mock.patch.object(esp32.subprocess, 'run', side_effect=run):
            fb.deploy_rules('proj', 'inst', '// as given\n' + TestRules.OTHER)
        assert seen['firebase.json'] == {'database': [{'instance': 'inst', 'rules': 'database.rules.json'}]}
        assert seen['rules'].startswith('// as given\n')  # The text given, comments and all
        assert seen['cmd'][1:4] == ['deploy', '--only', 'database']

    def test_rules_work(self):
        # Checked by what the rules do: the key unreadable, a wrong key refused, the right one
        # let in, the commands readable
        url = 'https://x.firebaseio.com'
        for statuses, ok in (([401, 401, 200, 200], True), ([200], False), ([401, 200], False),
                             ([401, 401, 401], False), ([401, 401, 200, 401], False), ([0], False)):
            with mock.patch.object(fb, 'http_status', side_effect=statuses) as st:
                assert fb.rules_work(url, FLEET, KEY)[0] == ok, statuses
        with mock.patch.object(fb, 'http_status', side_effect=[401, 401, 200, 200]) as st:
            fb.rules_work(url, FLEET, KEY)
        asked = [c[0] for c in st.call_args_list]
        base = url + '/robots/' + FLEET
        assert asked[0] == (base + '/robot_key.json',)
        assert asked[1][:2] == (base + '/power_system_status.json', 'PATCH') and asked[1][2]['robot_key'] != KEY
        assert asked[2] == (base + '/power_system_status.json', 'PATCH', {'robot_key': KEY})
        assert asked[3] == (base + '/power_system_requests.json',)
        # With sign-in, the last check reads without the token, and must be refused
        with mock.patch.object(fb, 'http_status', side_effect=[401, 401, 200, 200, 401]) as st:
            assert fb.rules_work(url, FLEET, KEY, id_token='TOK') == (True, '')
        # The key is checked without signing in (users let in may read it); the rest as the robot
        assert st.call_args_list[0][0] == (base + '/robot_key.json',)
        assert all(c[0][0].endswith('?auth=TOK') for c in st.call_args_list[1:4])
        assert st.call_args_list[4][0] == (base + '/power_system_requests.json',)
        with mock.patch.object(fb, 'http_status', side_effect=[401, 401, 200, 200, 200]):
            ok, why = fb.rules_work(url, FLEET, KEY, id_token='TOK')  # Rules without sign-in
        assert not ok and 'without signing in' in why

    def test_sign_in(self):
        class Reply:
            def __init__(self, body):
                self.body = body
            def __enter__(self):
                return self
            def __exit__(self, *a):
                return False
            def read(self, *a):
                return self.body
        with mock.patch.object(esp32.urllib.request, 'urlopen',
                               return_value=Reply(b'{"idToken": "TOK", "localId": "UID9"}')) as u:
            assert fb.sign_in('API', 'a@b.c', 'pw') == 'TOK'
        with mock.patch.object(esp32.urllib.request, 'urlopen',
                               return_value=Reply(b'{"idToken": "TOK", "localId": "UID9"}')):
            assert fb.sign_in_account('API', 'a@b.c', 'pw')['localId'] == 'UID9'
        req = u.call_args[0][0]
        assert req.full_url.endswith('signInWithPassword?key=API')
        assert json.loads(req.data) == {'email': 'a@b.c', 'password': 'pw', 'returnSecureToken': True}
        refused = esp32.urllib.error.HTTPError('u', 400, 'Bad', {}, None)
        refused.read = lambda *a: b'{"error": {"message": "INVALID_LOGIN_CREDENTIALS"}}'
        with mock.patch.object(esp32.urllib.request, 'urlopen', side_effect=refused):
            with pytest.raises(ValueError, match='INVALID_LOGIN_CREDENTIALS'):
                fb.sign_in('API', 'a@b.c', 'bad')

    def test_no_database_yet(self):
        err = json.dumps({'status': 'error', 'error': "It looks like you haven't created a Realtime Database "
                                                     "instance in this project before."})
        with mock.patch.object(esp32.subprocess, 'run', return_value=completed(err, 1)):
            assert fb.list_databases('proj') == []


# ############################# The rules, in the database emulator #############################
# Runs where Java and the Realtime Database emulator are installed (firebase setup:emulators:database),
# and skips elsewhere. It sends requests the way the ESP32 and the app do: a user's ID token in ?auth=.

EMULATOR_JARS = sorted(glob.glob(os.path.expanduser('~/.cache/firebase/emulators/firebase-database-emulator-*.jar')))
NS = 'demo-robots'


@pytest.fixture(scope='module')
def emulator():
    if not EMULATOR_JARS or not shutil.which('java'):
        pytest.skip('needs Java and the Realtime Database emulator (firebase setup:emulators:database)')
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    p = subprocess.Popen(['java', '-jar', EMULATOR_JARS[-1], '--port', str(port)], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    try:
        for _ in range(150):
            try:
                socket.create_connection(('127.0.0.1', port), 0.2).close()
                break
            except OSError:
                time.sleep(0.2)
        yield 'http://127.0.0.1:%d' % port
    finally:
        p.terminate()
        p.wait()


def id_token(uid):
    """An unsigned ID token, which the emulator takes."""
    def enc(d):
        return base64.urlsafe_b64encode(json.dumps(d).encode()).rstrip(b'=').decode()
    now = int(time.time())
    return enc({'alg': 'none', 'typ': 'JWT'}) + '.' + enc({
        'sub': uid, 'user_id': uid, 'iat': now, 'exp': now + 3600, 'auth_time': now, 'aud': NS,
        'iss': 'https://securetoken.google.com/' + NS}) + '.'


def request(base, method, path, data=None, uid=None, admin=False, raw=False):
    """The HTTP status. admin: as the console or the CLI, which bypass the rules. raw: data is text, sent as is."""
    url = '%s/%s.json?ns=%s' % (base, path.strip('/'), NS) + ('&auth=' + id_token(uid) if uid else '')
    body = None if data is None else (data if raw else json.dumps(data)).encode()
    r = urllib.request.Request(url, method=method, data=body)
    if admin:
        r.add_header('Authorization', 'Bearer owner')
    try:
        with urllib.request.urlopen(r) as resp:
            return resp.status
    except urllib.error.HTTPError as e:
        return e.code


K1, K2 = 'K' * 32, 'J' * 32
F1, F2 = 'stretch-se4-1001', 'stretch-se4-1002'
STATUS = {'power_state': 'active', 'online': True}


def command(key, cid='c1'):
    return {'robot_key': key, 'action': 'sleep', 'id': cid, 'acknowledged': False, 'ts': {'.sv': 'timestamp'}}


def make_db(base, sign_in):
    assert request(base, 'PUT', '.settings/rules', fb.rules(sign_in), admin=True) == 200
    assert request(base, 'PUT', '', {'robots': {F1: {'robot_key': K1}, F2: {'robot_key': K2}}}, admin=True) == 200
    return base


@pytest.fixture
def db(emulator):
    return make_db(emulator, sign_in=False)


@pytest.fixture
def db_sign_in(emulator):
    return make_db(emulator, sign_in=True)


class TestRulesInEmulator:
    def test_robot_and_app_with_the_key(self, db):
        assert request(db, 'PUT', 'robots/%s/power_system_status' % F1, dict(STATUS, robot_key=K1)) == 200
        assert request(db, 'PUT', 'robots/%s/power_system_requests/power_command' % F1, command(K1)) == 200
        assert request(db, 'GET', 'robots/%s/power_system_requests/power_command' % F1) == 200
        # The robot marks the command, writing that field alone: the command carries the key
        assert request(db, 'PUT', 'robots/%s/power_system_requests/power_command/acknowledged' % F1, True) == 200
        ack = {'robot_key': K1, 'id': 'c1', 'action': 'sleep', 'result': 'done'}
        assert request(db, 'PUT', 'robots/%s/power_system_requests/power_command_ack' % F1, ack) == 200
        # Two robots, one database, each with its own key
        assert request(db, 'PUT', 'robots/%s/power_system_status' % F2, dict(STATUS, robot_key=K2)) == 200
        assert request(db, 'GET', 'robots/%s/power_system_status' % F1) == 200  # Reads are open

    @pytest.mark.parametrize('path,data', [
        ('robots/%s/power_system_status' % F1, STATUS),                                # No key
        ('robots/%s/power_system_status' % F1, dict(STATUS, robot_key=K2)),            # Another robot's key
        ('robots/%s/power_system_requests/power_command' % F1, command(None)),
        ('robots/%s/power_system_requests/power_command' % F1, command(K2)),
        ('robots/%s/power_system_requests/power_command_ack' % F1, {'result': 'done', 'robot_key': 'x' * 32}),
        ('robots/%s/robot_key' % F1, K2),                                              # Can't change the key
        ('robots/%s/something_else' % F1, {'robot_key': K1}),                          # Nothing else
        ('robots/stretch-se4-9999/power_system_status', dict(STATUS, robot_key=K1)),   # A fleet without a key
    ])
    def test_writes_refused(self, db, path, data):
        assert request(db, 'PUT', path, data) == 401

    @pytest.mark.parametrize('path', ['robots/%s/robot_key' % F1, 'robots/%s' % F1, 'robots', ''])
    def test_key_and_listing_unreadable(self, db, path):
        assert request(db, 'GET', path) == 401
        assert request(db, 'GET', path, uid='UID1') == 401

    def test_sign_in(self, db_sign_in):
        db = db_sign_in
        # UID1 is assigned to F1, as web teleop assigns operators; UID2 is a user of the project, not assigned
        assert request(db, 'PUT', 'assignments/UID1/robots/%s' % F1, True, admin=True) == 200
        status = 'robots/%s/power_system_status' % F1
        cmd_path = 'robots/%s/power_system_requests/power_command' % F1
        assert request(db, 'GET', status) == 401                                    # Not signed in
        assert request(db, 'GET', status, uid='UID2') == 401                        # Signed in, not assigned
        assert request(db, 'GET', status, uid='UID1') == 200
        assert request(db, 'GET', 'robots/%s/power_system_status' % F2, uid='UID1') == 401  # Not this robot
        assert request(db, 'PUT', cmd_path, command(K1)) == 401
        assert request(db, 'PUT', cmd_path, command(K1), uid='UID2') == 401
        assert request(db, 'PUT', cmd_path, command(K1), uid='UID1') == 200
        assert request(db, 'PUT', cmd_path, command(K2, 'c2'), uid='UID1') == 401  # Assigned, wrong key
        assert request(db, 'GET', 'robots/%s/robot_key' % F1, uid='UID1') == 401
        assert request(db, 'GET', 'assignments', uid='UID1') == 401
        assert request(db, 'PUT', 'assignments/UID2/robots/%s' % F1, True, uid='UID2') == 401  # Can't assign oneself

    def test_sign_in_web_teleop_forms(self, db_sign_in):
        db = db_sign_in
        status = 'robots/%s/power_system_status' % F1
        # Web teleop's robot account for F1 (role robot, name F1)
        assert request(db, 'PUT', 'assignments/ROBOT1', {'role': 'robot', 'name': F1}, admin=True) == 200
        assert request(db, 'PUT', status, dict(STATUS, robot_key=K1), uid='ROBOT1') == 200
        assert request(db, 'PUT', 'robots/%s/power_system_status' % F2, dict(STATUS, robot_key=K2), uid='ROBOT1') == 401
        # An operator through web teleop's uids alias
        assert request(db, 'PUT', 'uids/UID3', 'alice', admin=True) == 200
        assert request(db, 'PUT', 'assignments/alice/robots/%s' % F1, True, admin=True) == 200
        assert request(db, 'GET', status, uid='UID3') == 200

    def test_change_key(self, db):
        assert request(db, 'PUT', 'robots/%s/robot_key' % F1, 'N' * 32, admin=True) == 200  # In the console
        assert request(db, 'PUT', 'robots/%s/power_system_status' % F1, dict(STATUS, robot_key=K1)) == 401
        assert request(db, 'PUT', 'robots/%s/power_system_status' % F1, dict(STATUS, robot_key='N' * 32)) == 200

    def test_merged_rules_work(self, emulator):
        # Someone's rules (shaped like web teleop's), with the robot rules merged in
        theirs = {'rules': {'robots': {'$fleetId': {'.read': 'auth != null', 'status': {'.write': 'auth != null'},
                                                    '$other': {'.validate': False}}}}}
        merged, _ = fb.merge_rules(json.dumps(theirs))
        assert request(emulator, 'PUT', '.settings/rules', merged, admin=True) == 200
        assert request(emulator, 'PUT', 'robots/%s/robot_key' % F1, K1, admin=True) == 200
        assert request(emulator, 'PUT', 'robots/%s/power_system_status' % F1, dict(STATUS, robot_key=K1)) == 200
        assert request(emulator, 'PUT', 'robots/%s/power_system_status' % F1, dict(STATUS, robot_key=K2)) == 401
        assert request(emulator, 'PUT', 'robots/%s/power_system_requests/power_command' % F1, command(K1)) == 200
        # Theirs still apply
        assert request(emulator, 'PUT', 'robots/%s/status' % F1, 'online') == 401
        assert request(emulator, 'PUT', 'robots/%s/status' % F1, 'online', uid='UID1') == 200
        assert request(emulator, 'PUT', 'robots/%s/other' % F1, 1, uid='UID1') == 401
        assert request(emulator, 'GET', 'robots/%s' % F1, uid='UID1') == 200

    def test_check_passes_on_merged_web_teleop_rules(self, emulator):
        # Web teleop lets assigned users read all of robots/$fleetId, robot_key included: the
        # tool's check must still pass for the robot's assigned user
        if not os.path.exists(TestRules.WEB_TELEOP_RULES):
            pytest.skip('stretch4_web_teleop is not checked out')
        merged, _ = fb.merge_rules(open(TestRules.WEB_TELEOP_RULES).read(), sign_in=True)
        assert request(emulator, 'PUT', '.settings/rules', merged, admin=True) == 200
        assert request(emulator, 'PUT', 'robots/%s/robot_key' % F1, K1, admin=True) == 200
        assert request(emulator, 'PUT', 'assignments/ROBOTUSER/robots/%s' % F1, True, admin=True) == 200
        real = fb.http_status
        with mock.patch.object(fb, 'http_status', side_effect=lambda url, method='GET', data=None, timeout=10:
                               real(url + ('&' if '?' in url else '?') + 'ns=' + NS, method, data, timeout)):
            assert fb.rules_work(emulator, F1, K1, id_token=id_token('ROBOTUSER')) == (True, '')
            assert not fb.rules_work(emulator, F1, K1, id_token=id_token('STRANGER'))[0]
        assert request(emulator, 'GET', 'robots/%s/robot_key' % F1) == 401
        assert request(emulator, 'GET', 'robots/%s/robot_key' % F1, uid='STRANGER') == 401
