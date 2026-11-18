"""
ESP32 RPC protocol (stretch4_body.subsystem.esp32) without hardware: the packed
layouts, unpacking, packing, validation, the log format and the credentials helpers.
"""
import os
import re
import struct

import pytest

from stretch4_body.core.device import DeviceTimestamp
from stretch4_body.subsystem.esp32 import (Esp32Defn, Esp32Base, Esp32_Protocol_P1, esp32_status_template,
                                           params_credentials)

D = Esp32Defn

# Sizes asserted in stretch_firmware_ii/arduino/hello_esp/EspRpc.h
FIRMWARE_SIZES = {
    'FMT_BOARD_INFO': 50,
    'FMT_STATUS': 91,
    'FMT_WIFI_STATUS': 112,
    'FMT_FIREBASE_STATUS': 151,
    'FMT_WIFI_CONFIG': 99,
    'FMT_FIREBASE_CONFIG': 697,
    'FMT_CONFIG': 11,
    'FMT_TRIGGER': 4,
}

ESP_RPC_H = os.path.join(os.path.dirname(__file__), '..', '..', 'stretch_firmware_ii', 'arduino', 'hello_esp',
                         'EspRpc.h')


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
        assert int(asserted['Esp32_Status']) == struct.calcsize(D.FMT_STATUS)
        assert int(asserted['Esp32_Wifi_Status']) == struct.calcsize(D.FMT_WIFI_STATUS)
        assert int(asserted['Esp32_Firebase_Status']) == struct.calcsize(D.FMT_FIREBASE_STATUS)
        assert int(asserted['Esp32_Wifi_Config']) == struct.calcsize(D.FMT_WIFI_CONFIG)
        assert int(asserted['Esp32_Firebase_Config']) == struct.calcsize(D.FMT_FIREBASE_CONFIG)
        assert int(asserted['Esp32_Config']) == struct.calcsize(D.FMT_CONFIG)
        ids = dict((k, int(v)) for k, v in re.findall(r'#define (RPC_\w+)\s+(\d+)', src))
        for name, value in ids.items():
            assert getattr(D, name) == value, name
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

    def test_firebase_config(self):
        e = p1()
        e.set_firebase_config('https://x.firebaseio.com', 'key', 'a@b.c', 'pw')
        url, key, email, pw, flags = struct.unpack(D.FMT_FIREBASE_CONFIG, e._pending_firebase_config)
        assert url.rstrip(b'\0') == b'https://x.firebaseio.com' and email.rstrip(b'\0') == b'a@b.c'

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

    def test_firebase_all_or_none(self):
        Esp32Base.validate_firebase_config('', '', '', '')
        Esp32Base.validate_firebase_config('u', 'k', 'e', 'p')
        with pytest.raises(ValueError):
            Esp32Base.validate_firebase_config('u', 'k', '', 'p')
        with pytest.raises(ValueError):
            Esp32Base.validate_firebase_config('u' * 251, 'k', 'e', 'p')


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


class TestCredentials:
    def test_esp32_params_first(self):
        rp = {'esp32': {'wifi': {'ssid': 'New', 'password': 'newpassword'},
                        'firebase': {'url': 'u2', 'api_key': 'NA', 'user_email': 'NA', 'user_password': 'NA'}},
              'power-periph': {'firebase': {'network_ssid': 'Old', 'network_password': 'oldpassword', 'url': 'u1',
                                            'api_key': 'k1', 'user_email': 'e1', 'user_password': 'p1'}}}
        wifi, fb = params_credentials(rp)
        assert wifi == {'ssid': 'New', 'password': 'newpassword'}
        assert fb == {'url': 'u2', 'api_key': 'k1', 'user_email': 'e1', 'user_password': 'p1'}

    def test_unset(self):
        wifi, fb = params_credentials({'esp32': {'wifi': {'ssid': 'NA', 'password': 'NA'}}})
        assert wifi == {'ssid': None, 'password': None}
        assert all(v is None for v in fb.values())
