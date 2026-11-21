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
    'FMT_FIREBASE_CONFIG': 762,      # hello-esp v0.2.6 on (with robot_key)
    'FMT_CONFIG': 11,
    'FMT_TRIGGER': 4,
}

ESP_RPC_H = os.path.join(os.path.dirname(__file__), '..', '..', 'stretch_firmware_ii', 'arduino', 'hello_esp',
                         'EspRpc.h')


KEY = 'AbCdEfGhIjKlMnOpQrStUvWxYz012345'  # 32 characters, as generate_robot_key makes


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
        e.set_firebase_config('https://x.firebaseio.com', KEY, apply_now=False)
        url, key, api, email, pw, flags = struct.unpack(D.FMT_FIREBASE_CONFIG, e._pending_firebase_config)
        assert url.rstrip(b'\0') == b'https://x.firebaseio.com' and key.rstrip(b'\0') == KEY.encode()
        assert api.rstrip(b'\0') == email.rstrip(b'\0') == pw.rstrip(b'\0') == b''
        assert flags == D.CFG_FLAG_PERSIST

    def test_firebase_config_sign_in(self):
        e = p1()
        e.set_firebase_config('https://x.firebaseio.com', KEY, 'api', 'a@b.c', 'pw')
        assert e._unpack_firebase_config(e._pending_firebase_config) == {
            'url': 'https://x.firebaseio.com', 'robot_key': KEY, 'api_key': 'api', 'user_email': 'a@b.c',
            'user_password': 'pw', 'configured': False}

    def test_firebase_config_clear(self):
        e = p1()
        e.set_firebase_config('', '')
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
        Esp32Base.validate_firebase_config('', '')                       # Clears Firebase
        Esp32Base.validate_firebase_config('u', KEY)                     # Robot key only
        Esp32Base.validate_firebase_config('u', KEY, 'k', 'e', 'p')      # And signs in
        Esp32Base.validate_firebase_config('u', 'a-_' + 'x' * 61)        # 64, the longest

    @pytest.mark.parametrize('fields', [
        ('u', ''), ('', KEY),                          # The URL and the key go together
        ('u', KEY, 'k', '', 'p'),                       # The sign-in is all three or none
        ('u', 'x' * 21),                                # Too short
        ('u', 'x' * 65),                                # Too long
        ('u', KEY[:-1] + '/'), ('u', KEY[:-1] + '.'),   # Not allowed in a database path
        ('u' * 251, KEY),
        ('', '', 'k', 'e', 'p'),
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
    def test_only_robots_by_key(self):
        rules = fb.RULES['rules']
        # Nothing at the root or on robots/ itself: no listing, and robot_keys is unreadable
        assert list(rules) == ['robots'] and list(rules['robots']) == ['$robot_key']
        node = rules['robots']['$robot_key']
        assert node['.read'] == node['.write'] == fb.ACCESS
        assert "child($robot_key).val() === true" in fb.ACCESS
        assert 'auth != null' in fb.ACCESS and 'auth.uid' in fb.ACCESS

    def test_text_round_trips(self):
        assert fb.rules_are_ours(fb.rules_json())
        assert fb.rules_are_ours(json.dumps(fb.RULES))
        assert not fb.rules_are_ours('{"rules": {".read": true, ".write": true}}')
        assert not fb.rules_are_ours('// a comment\n' + fb.rules_json())
        assert not fb.rules_are_ours(None)

    def test_locked(self):
        assert fb.rules_are_locked('{\n  "rules": {\n    ".read": false,\n    ".write": false\n  }\n}')
        assert not fb.rules_are_locked(fb.rules_json())


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

    def test_add_robot_key(self):
        ok = json.dumps({'status': 'success', 'result': None})
        with mock.patch.object(esp32.subprocess, 'run', return_value=completed(ok)) as run:
            fb.add_robot_key('proj', 'proj-default-rtdb', 'KEY')
            fb.add_robot_key('proj', 'proj-default-rtdb', 'KEY', uid='UID1')
        a, b = (c[0][0] for c in run.call_args_list)
        assert a[1:5] == ['database:set', '/robot_keys/KEY', '--data', 'true']
        assert b[4] == '"UID1"'
        assert a[a.index('--instance') + 1] == 'proj-default-rtdb' and a[a.index('--project') + 1] == 'proj'

    def test_deploy_writes_rules(self):
        seen = {}

        def run(cmd, cwd=None, **kw):
            seen['firebase.json'] = json.load(open(cwd + '/firebase.json'))
            seen['rules'] = open(cwd + '/database.rules.json').read()
            seen['cmd'] = cmd
            return completed(json.dumps({'status': 'success', 'result': {}}))
        with mock.patch.object(esp32.subprocess, 'run', side_effect=run):
            fb.deploy_rules('proj', 'inst')
        assert seen['firebase.json'] == {'database': [{'instance': 'inst', 'rules': 'database.rules.json'}]}
        assert fb.rules_are_ours(seen['rules'])
        assert seen['cmd'][1:4] == ['deploy', '--only', 'database']

    def test_current_rules(self):
        replies = [json.dumps({'status': 'success', 'result': {'labeled': {'stable': 'r1'}, 'rulesets': []}}),
                   json.dumps({'status': 'success', 'result': {'id': 'r1', 'source': fb.rules_json()}})]
        with mock.patch.object(esp32.subprocess, 'run', side_effect=[completed(r) for r in replies]):
            assert fb.rules_are_ours(fb.current_rules('proj', 'inst'))

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


def request(base, method, path, data=None, uid=None, admin=False):
    """The HTTP status. admin: as the console or the CLI, which bypass the rules."""
    url = '%s/%s.json?ns=%s' % (base, path.strip('/'), NS) + ('&auth=' + id_token(uid) if uid else '')
    r = urllib.request.Request(url, method=method, data=None if data is None else json.dumps(data).encode())
    if admin:
        r.add_header('Authorization', 'Bearer owner')
    try:
        with urllib.request.urlopen(r) as resp:
            return resp.status
    except urllib.error.HTTPError as e:
        return e.code


K1, K2, KU, KX = 'K' * 32, 'J' * 32, 'U' * 32, 'X' * 32
STATUS = {'power_state': 'active', 'online': True}


@pytest.fixture
def db(emulator):
    assert request(emulator, 'PUT', '.settings/rules', fb.RULES, admin=True) == 200
    assert request(emulator, 'PUT', '', {'robot_keys': {K1: True, K2: True, KU: 'UID1'}}, admin=True) == 200
    return emulator


class TestRulesInEmulator:
    def test_robot_and_app_with_the_key(self, db):
        assert request(db, 'PUT', 'robots/%s/system_status' % K1, STATUS) == 200
        cmd = {'action': 'sleep', 'id': 'c1', 'ts': {'.sv': 'timestamp'}}
        assert request(db, 'PUT', 'robots/%s/requests/power_command' % K1, cmd) == 200
        assert request(db, 'GET', 'robots/%s/requests/power_command' % K1) == 200
        assert request(db, 'PUT', 'robots/%s/system_status' % K2, STATUS) == 200  # Two robots, one database

    @pytest.mark.parametrize('method,path', [
        ('PUT', 'robots/%s/system_status' % KX), ('GET', 'robots/%s' % KX),  # A key robot_keys doesn't list
        ('GET', 'robots'), ('GET', 'robot_keys'), ('GET', ''),                # No listing
        ('PUT', 'robot_keys/%s' % KX),                                        # Can't add a key
    ])
    def test_denied_without_a_listed_key(self, db, method, path):
        assert request(db, method, path, True if method == 'PUT' else None) == 401
        assert request(db, method, path, True if method == 'PUT' else None, uid='UID1') == 401

    def test_key_bound_to_a_user(self, db):
        assert request(db, 'GET', 'robots/%s' % KU) == 401
        assert request(db, 'GET', 'robots/%s' % KU, uid='UID2') == 401
        assert request(db, 'PUT', 'robots/%s/system_status' % KU, STATUS, uid='UID1') == 200

    def test_revoke(self, db):
        assert request(db, 'GET', 'robots/%s' % K1) == 200
        assert request(db, 'DELETE', 'robot_keys/%s' % K1, admin=True) == 200
        assert request(db, 'GET', 'robots/%s' % K1) == 401
