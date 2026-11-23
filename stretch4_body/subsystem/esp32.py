#!/usr/bin/env python3
"""
API to the ESP32 (hello_esp firmware) over its USB CDC port, /dev/hello-esp32.

Same pattern as PowerPeriph and Stepper: pull status, push config, push triggers,
over the Hello Robot RPC transport. Needs hello-esp firmware v0.2.0p1 or later;
older firmware has no RPC (configure it through PowerPeriph.send_network_info /
send_firebase_info, which go through the Pimu).

The ESP32 is not part of the robot server: this direct API (and stretch_esp32_jog,
which uses it) is the only way in. One process at a time can have the port.
The robot server still sees the ESP32's rails, adapter and connection flags through
PowerPeriph, which the Pimu fills from its UART link to the ESP32.

The packed layouts below must match stretch_firmware_ii/arduino/hello_esp/EspRpc.h.
"""
import array as arr
import json
import os
import re
import secrets
import shutil
import struct
import subprocess
import tempfile
import textwrap
import time
import urllib.error
import urllib.request

from stretch4_body.core.device import Device
from stretch4_body.core.transport.transport import Transport


ESP32_USB_NAME = '/dev/hello-esp32'
# The firmware sketch and its release binaries are named hello_esp / hello-esp
ESP32_SKETCH_NAME = 'hello_esp'
ESP32_FIRMWARE_NAMES = ('hello-esp', 'hello_esp')


def _cstr(b):
    """Bytes of a fixed width C string to str."""
    return bytes(b).split(b'\x00', 1)[0].decode('utf-8', errors='replace')


def _pack_cstr(s, width, field):
    """str to bytes for a fixed width C string, leaving room for the NUL."""
    b = (s or '').encode('utf-8')
    if len(b) > width - 1:
        raise ValueError('%s is %d bytes, at most %d allowed' % (field, len(b), width - 1))
    return b


def _ip(b):
    return '.'.join(str(x) for x in bytes(b))


def _mac(b):
    return ':'.join('%02X' % x for x in bytes(b))


def firmware_version_tuple(version):
    """'hello-esp.v0.2.6p1' to (0, 2, 6), () if it isn't a version."""
    m = re.search(r'\.v(\d+)\.(\d+)\.(\d+)', version or '')
    return tuple(int(x) for x in m.groups()) if m else ()


def generate_robot_key():
    """A new Firebase robot key: 32 URL-safe characters (192 random bits)."""
    return secrets.token_urlsafe(24)


def esp32_status_template():
    """The Esp32 status dict, before the first read."""
    return {
        'timestamp': 0, 'pwr_state': 0, 'pwr_state_name': '', 'reset_reason': 0, 'reset_reason_name': '',
        'cpu_freq_mhz': 0, 'free_heap': 0, 'min_free_heap': 0, 'loop_time_us': 0, 'power_task_time_us': 0,
        'log_pending': 0, 'log_dropped': 0,
        'voltage_12v0': 0.0, 'voltage_20v0_aux': 0.0, 'adapter_connected': False, 'adapter_fault': False,
        'cpu_5v_usb_on': False,
        'outputs': {'en_12v0': False, 'en_aux_20v0': False, 'en_lidar': False, 'en_status_leds': False,
                    'dcm_mode': False, 'robot_active': False, 'cpu_sd': False, 'system_shutdown': False,
                    'sd_pulse_count': 0},
        'pimu_link': {'voltage_battery': 0.0, 'voltage_20v0': 0.0, 'temperature': 0.0, 'battery_soc': 0,
                      'battery_status': 'UNKNOWN', 'battery_valid': True,
                      'charger_charging': False, 'runstop_event': False, 'cpu_power_state': 0,
                      'cpu_power_state_name': '', 'nuc_rail_off': False, 'status_fresh': False,
                      'status_age_ms': None, 'uart_rx_packets': 0, 'uart_crc_errors': 0,
                      'uart_rx_overflows': 0, 'uart_tx_packets': 0, 'uart_tx_drops': 0,
                      'esp_status_counter': 0},
        'wifi': {'state': 0, 'state_name': '', 'wl_status': 0, 'wl_status_name': '', 'enabled': False,
                 'low_power': False, 'ssid': '', 'rssi': 0, 'channel': 0, 'bssid': '', 'mac': '', 'ip': '',
                 'gateway': '', 'netmask': '', 'dns': '', 'hostname': '', 'connect_attempts': 0,
                 'disconnects': 0, 'last_disconnect_reason': 0, 'last_disconnect_reason_name': '',
                 'connected_s': 0.0},
        'firebase': {'state': 0, 'state_name': '', 'enabled': False, 'app_ready': False, 'db_setup': False,
                     'init_count': 0, 'publish_ok': 0, 'publish_errors': 0, 'last_publish_age_s': None,
                     'publish_period_s': 0.0, 'last_error_code': 0, 'last_error': '',
                     'stream_events': 0, 'stream_age_s': None, 'stream_restarts': 0, 'time_synced': False,
                     'cmd_action': 0, 'cmd_action_name': '', 'cmd_result': 0, 'cmd_result_name': '',
                     'cmd_age_s': None, 'cmd_id': ''},
    }


class Esp32Defn:
    RPC_GET_BOARD_INFO = 1
    RPC_REPLY_BOARD_INFO = 2
    RPC_GET_STATUS = 3
    RPC_REPLY_STATUS = 4
    RPC_GET_WIFI_STATUS = 5
    RPC_REPLY_WIFI_STATUS = 6
    RPC_GET_FIREBASE_STATUS = 7
    RPC_REPLY_FIREBASE_STATUS = 8
    RPC_GET_WIFI_CONFIG = 9
    RPC_REPLY_WIFI_CONFIG = 10
    RPC_SET_WIFI_CONFIG = 11
    RPC_REPLY_SET_WIFI_CONFIG = 12
    RPC_GET_FIREBASE_CONFIG = 13
    RPC_REPLY_FIREBASE_CONFIG = 14
    RPC_SET_FIREBASE_CONFIG = 15
    RPC_REPLY_SET_FIREBASE_CONFIG = 16
    RPC_SET_CONFIG = 17
    RPC_REPLY_SET_CONFIG = 18
    RPC_SET_TRIGGER = 19
    RPC_REPLY_SET_TRIGGER = 20
    RPC_GET_WIFI_SCAN = 21
    RPC_REPLY_WIFI_SCAN = 22
    RPC_READ_LOG = 23
    RPC_REPLY_READ_LOG = 24

    RPC_RESULT_OK = 0
    RPC_RESULT_INVALID = 1

    TRIGGER_WIFI_RECONNECT = 1 << 0
    TRIGGER_WIFI_DISCONNECT = 1 << 1
    TRIGGER_WIFI_FORGET = 1 << 2
    TRIGGER_FIREBASE_RESTART = 1 << 3
    TRIGGER_FIREBASE_FORGET = 1 << 4
    TRIGGER_WIFI_SCAN = 1 << 5
    TRIGGER_REBOOT = 1 << 6
    TRIGGER_LOG_CLEAR = 1 << 7

    CFG_FLAG_PERSIST = 1 << 0
    CFG_FLAG_APPLY_NOW = 1 << 1
    CFG_FLAG_CONFIGURED = 1 << 7

    LOG_DEBUG = 0
    LOG_INFO = 1
    LOG_WARN = 2
    LOG_ERROR = 3
    LOG_IDF = 4
    LOG_LEVEL_NAMES = {0: 'DEBUG', 1: 'INFO', 2: 'WARN', 3: 'ERROR', 4: 'IDF'}

    PWR_STATE_NAMES = {0: 'ACTIVE', 1: 'SLEEP', 2: 'SHUTDOWN_CHRG', 3: 'SLEEP_CHRG', 4: 'SHUTDOWN'}
    CPU_POWER_STATE_NAMES = {0: 'UNKNOWN', 1: 'OFF', 2: 'ON'}
    RESET_REASON_NAMES = {0: 'UNKNOWN', 1: 'POWERON', 2: 'EXT', 3: 'SW', 4: 'PANIC', 5: 'INT_WDT',
                          6: 'TASK_WDT', 7: 'WDT', 8: 'DEEPSLEEP', 9: 'BROWNOUT', 10: 'SDIO',
                          11: 'USB', 12: 'JTAG', 13: 'EFUSE', 14: 'PWR_GLITCH', 15: 'CPU_LOCKUP'}
    WIFI_STATE_NAMES = {0: 'NOT_CONFIGURED', 1: 'DISABLED', 2: 'CONNECTING', 3: 'CONNECTED',
                        4: 'SCANNING', 5: 'POWER_OFF', 6: 'DISCONNECTED'}
    WL_STATUS_NAMES = {0: 'IDLE', 1: 'NO_SSID_AVAIL', 2: 'SCAN_COMPLETED', 3: 'CONNECTED',
                       4: 'CONNECT_FAILED', 5: 'CONNECTION_LOST', 6: 'DISCONNECTED', 255: 'NO_SHIELD'}
    WIFI_DISCONNECT_REASON_NAMES = {0: '', 1: 'UNSPECIFIED', 2: 'AUTH_EXPIRE', 3: 'AUTH_LEAVE',
                                    4: 'ASSOC_EXPIRE', 8: 'ASSOC_LEAVE', 15: '4WAY_HANDSHAKE_TIMEOUT',
                                    200: 'BEACON_TIMEOUT', 201: 'NO_AP_FOUND', 202: 'AUTH_FAIL',
                                    203: 'ASSOC_FAIL', 204: 'HANDSHAKE_TIMEOUT', 205: 'CONNECTION_FAIL',
                                    206: 'AP_TSF_RESET', 207: 'ROAMING', 210: 'NO_AP_FOUND_W_COMPATIBLE_SECURITY',
                                    211: 'NO_AP_FOUND_IN_AUTHMODE_THRESHOLD', 212: 'NO_AP_FOUND_IN_RSSI_THRESHOLD'}
    FIREBASE_STATE_NAMES = {0: 'NOT_CONFIGURED', 1: 'DISABLED', 2: 'WAITING_FOR_WIFI', 3: 'AUTHENTICATING',
                            4: 'READY', 5: 'STREAM_CONNECTING'}
    # The last power command from the app (power_system_requests/power_command) and what came of it
    CMD_ACTION_NAMES = {0: 'NONE', 1: 'SLEEP', 2: 'WAKE'}
    CMD_RESULT_NAMES = {0: 'NONE', 1: 'PENDING', 2: 'DONE', 3: 'FAILED', 4: 'REJECTED', 5: 'EXPIRED',
                        6: 'INVALID'}
    CMD_ID_LEN = 40
    SCAN_STATE_NAMES = {0: 'IDLE', 1: 'RUNNING', 2: 'DONE', 3: 'FAILED'}
    AUTH_MODE_NAMES = {0: 'OPEN', 1: 'WEP', 2: 'WPA_PSK', 3: 'WPA2_PSK', 4: 'WPA_WPA2_PSK',
                       5: 'WPA2_ENTERPRISE', 6: 'WPA3_PSK', 7: 'WPA2_WPA3_PSK', 8: 'WAPI_PSK',
                       9: 'OWE', 10: 'WPA3_ENT_192'}

    # Field widths, including the NUL
    SSID_LEN = 33
    WIFI_PASS_LEN = 65
    FB_URL_LEN = 251
    FB_FLEET_LEN = 65
    FB_KEY_LEN = 65
    FB_KEY_MIN_LEN = 22
    FB_API_LEN = 129
    FB_EMAIL_LEN = 251
    FB_PASS_LEN = 65
    SCAN_MAX = 20

    # Protocol p1 layouts (little endian, packed)
    FMT_BOARD_INFO = '<20s30s'
    FMT_STATUS = '<QBBHIIHHHHff' + 'B' * 12 + 'fff' + 'B' * 6 + 'IIIIII' + 'B'
    FMT_STATUS_BATTERY = FMT_STATUS + 'B'  # hello-esp v0.2.4 on: the Pimu's battery status, last
    # The Pimu's battery status (pimu_link battery_status): whether its battery values can be used
    BATTERY_STATUS_NAMES = {0: 'UNKNOWN', 1: 'OK', 2: 'BMS_LOST', 3: 'NOT_DETECTED'}
    FMT_WIFI_STATUS = '<BBBB33sbB6s6s4s4s4s4s32sIIBI'
    FMT_FIREBASE_STATUS = '<BBBBIIIIIi64sIIIBBBI40s'
    FMT_WIFI_CONFIG = '<33s65sB'
    FMT_FIREBASE_CONFIG = '<251s65s65s129s251s65sB'  # url, fleet_id, robot_key, api_key, user_email, user_password, flags
    FMT_CONFIG = '<IIBBB'
    FMT_TRIGGER = '<I'
    FMT_SCAN_HEADER = '<BB'
    FMT_SCAN_ENTRY = '<33sbBB'

    # Firmware older than this has no RPC (it only prints)
    FIRST_RPC_FIRMWARE = 'hello-esp.v0.2.0p1'
    # Firebase set-up (the robot key, FMT_FIREBASE_CONFIG) needs this firmware or later. Earlier
    # p1 firmware has a different Firebase config layout, from before Firebase was used
    FIRST_FIREBASE_FIRMWARE = 'hello-esp.v0.2.6p1'
    # Protocols the firmware tools may install: p0 is the firmware before RPC
    FIRMWARE_PROTOCOLS = ['p0', 'p1']

    UNKNOWN_AGE = 0xFFFFFFFF


class Esp32Base(Esp32Defn, Device):
    """
    API to the ESP32 (power sequencing, Wi-Fi and Firebase)
    """

    def __init__(self, usb=None):
        Device.__init__(self, 'esp32')
        if usb is None:
            usb = self.params.get('usb_name', ESP32_USB_NAME)
        # Low rate, so the Python backend: no C transport queue (qid) needed
        self.transport = Transport(port_name=usb, logger=self.logger,
                                   default_backend=Transport.BACKEND_PY_SERIAL)
        self.hw_valid = False
        self.board_info = {'board_variant': None, 'firmware_version': None, 'protocol_version': None,
                           'hardware_id': 0}
        self.config = dict(self.params.get('config', {}))
        self._dirty_config = True
        self._trigger = 0
        self._pending_wifi_config = None
        self._pending_firebase_config = None
        self._rpc_ok = False
        self._last_set_result = None
        self.status = esp32_status_template()
        self.status['transport'] = self.transport.status

    # ###########  Device Methods #############

    def startup(self, quiet=False):
        """
        Open the port, read the board info and push the config. Returns True if the ESP32 answers.
        quiet: log a missing ESP32 at debug level (for retries)
        """
        try:
            self.logger.debug('Starting Esp32...')
            Device.startup(self)
            if not self.transport.startup():
                self.transport.stop()
                (self.logger.debug if quiet else self.logger.warning)('Unable to open %s' % self.transport.port_name)
                return False
            # Drop anything left in the port (boot ROM banner, old firmware output)
            self.transport.ser.reset_input_buffer()
            for _ in range(3):
                if self._pull_board_info():
                    break
                time.sleep(0.1)
                self.transport.ser.reset_input_buffer()
            if self.board_info['firmware_version'] is None:
                (self.logger.debug if quiet else self.logger.warning)('No board info from the ESP32 on %s. Firmware older than %s has no RPC.'
                                    % (self.transport.port_name, self.FIRST_RPC_FIRMWARE))
                self.transport.stop()
                return False
            self.transport.configure_version(self.board_info['firmware_version'])
            self.hw_valid = True
            return True
        except Exception as e:
            self.hw_valid = False
            self.logger.error('Failed to start Esp32: %s', e)
            self.transport.stop()
            return False

    def stop(self):
        Device.stop(self)
        self.transport.stop()
        self.hw_valid = False

    def pull_status(self):
        """Read the system, Wi-Fi and Firebase status. Returns True if all three were read."""
        if not self.hw_valid:
            return False
        ok = self._rpc_pull(self.RPC_GET_STATUS, self.RPC_REPLY_STATUS, self._unpack_status)
        ok = self._rpc_pull(self.RPC_GET_WIFI_STATUS, self.RPC_REPLY_WIFI_STATUS, self._unpack_wifi_status) and ok
        ok = self._rpc_pull(self.RPC_GET_FIREBASE_STATUS, self.RPC_REPLY_FIREBASE_STATUS,
                            self._unpack_firebase_status) and ok
        return ok

    def push_command(self):
        """Send what has been set since the last push: config, credentials, triggers. Returns True on success."""
        if not self.hw_valid:
            return False
        ok = True
        if self._dirty_config:
            payload = arr.array('B', [self.RPC_SET_CONFIG]) + arr.array('B', self._pack_config(self.config))
            ok = self._rpc_push(payload, self.RPC_REPLY_SET_CONFIG) and ok
            self._dirty_config = False
        if self._pending_wifi_config is not None:
            payload = arr.array('B', [self.RPC_SET_WIFI_CONFIG]) + arr.array('B', self._pending_wifi_config)
            self._pending_wifi_config = None
            ok = self._rpc_push(payload, self.RPC_REPLY_SET_WIFI_CONFIG, self._check_set_result) and ok
        if self._pending_firebase_config is not None:
            payload = arr.array('B', [self.RPC_SET_FIREBASE_CONFIG]) + arr.array('B', self._pending_firebase_config)
            self._pending_firebase_config = None
            ok = self._rpc_push(payload, self.RPC_REPLY_SET_FIREBASE_CONFIG, self._check_set_result) and ok
        if self._trigger:
            payload = arr.array('B', [self.RPC_SET_TRIGGER]) + arr.array('B', struct.pack(self.FMT_TRIGGER, self._trigger))
            self._trigger = 0
            ok = self._rpc_push(payload, self.RPC_REPLY_SET_TRIGGER) and ok
        return ok

    # ###########  Configuration #############

    def set_config(self, **kwargs):
        """
        Change runtime settings (sent on the next push_command, not saved on the ESP32):
        firebase_publish_period_ms, wifi_retry_ms, wifi_enable, firebase_enable, log_level
        """
        for k, v in kwargs.items():
            if k not in ('firebase_publish_period_ms', 'wifi_retry_ms', 'wifi_enable', 'firebase_enable', 'log_level'):
                raise ValueError('Unknown ESP32 config %s' % k)
            self.config[k] = v
        self._dirty_config = True

    def set_wifi_config(self, ssid, password, persist=True, apply_now=True):
        """
        Set the Wi-Fi network (sent on the next push_command). An empty password is an open network.
        persist: save to the ESP32's flash. apply_now: reconnect now.
        """
        password = password or ''
        self.validate_wifi_config(ssid, password)
        flags = (self.CFG_FLAG_PERSIST if persist else 0) | (self.CFG_FLAG_APPLY_NOW if apply_now else 0)
        self._pending_wifi_config = struct.pack(
            self.FMT_WIFI_CONFIG, _pack_cstr(ssid, self.SSID_LEN, 'ssid'),
            _pack_cstr(password, self.WIFI_PASS_LEN, 'password'), flags)

    def set_firebase_config(self, url, fleet_id, robot_key, api_key='', user_email='', user_password='',
                            persist=True, apply_now=True):
        """
        Set the Firebase database, the robot's fleet ID (its data is under robots/<fleet_id>)
        and its robot key (sent on the next push_command). All empty clears Firebase. The
        sign-in (api_key, user_email, user_password) is optional: all three, or none.
        persist: save to the ESP32's flash. apply_now: restart Firebase now.
        Needs firmware hello-esp.v0.2.6p1 or later (supports_firebase).
        """
        fields = (url or '', fleet_id or '', robot_key or '', api_key or '', user_email or '', user_password or '')
        self.validate_firebase_config(*fields)
        flags = (self.CFG_FLAG_PERSIST if persist else 0) | (self.CFG_FLAG_APPLY_NOW if apply_now else 0)
        self._pending_firebase_config = self._pack_firebase_config(fields, flags)

    @classmethod
    def validate_wifi_config(cls, ssid, password):
        """Raise ValueError if the ESP32 would reject these Wi-Fi credentials."""
        password = password or ''
        if not ssid:
            raise ValueError('The Wi-Fi SSID is empty')
        _pack_cstr(ssid, cls.SSID_LEN, 'ssid')
        _pack_cstr(password, cls.WIFI_PASS_LEN, 'password')
        if len(password) and not 8 <= len(password) <= 64:
            raise ValueError('A Wi-Fi password is 8 to 64 characters, or empty for an open network')

    @classmethod
    def validate_robot_key(cls, robot_key):
        """Raise ValueError if robot_key can't be a robot key."""
        if not re.fullmatch(r'[A-Za-z0-9_-]{%d,%d}' % (cls.FB_KEY_MIN_LEN, cls.FB_KEY_LEN - 1), robot_key or ''):
            raise ValueError('A robot key is %d to %d of A-Z a-z 0-9 - _' % (cls.FB_KEY_MIN_LEN, cls.FB_KEY_LEN - 1))

    @classmethod
    def validate_fleet_id(cls, fleet_id):
        """Raise ValueError if fleet_id can't be a fleet ID (a path in the database)."""
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,%d}' % (cls.FB_FLEET_LEN - 1), fleet_id or ''):
            raise ValueError('A fleet ID is 1 to %d of A-Z a-z 0-9 - _' % (cls.FB_FLEET_LEN - 1))

    @classmethod
    def validate_firebase_config(cls, url, fleet_id, robot_key, api_key='', user_email='', user_password=''):
        """Raise ValueError if the ESP32 would reject these Firebase settings."""
        fields = (url or '', fleet_id or '', robot_key or '', api_key or '', user_email or '', user_password or '')
        for value, width, name in zip(fields, (cls.FB_URL_LEN, cls.FB_FLEET_LEN, cls.FB_KEY_LEN, cls.FB_API_LEN,
                                               cls.FB_EMAIL_LEN, cls.FB_PASS_LEN),
                                      ('url', 'fleet_id', 'robot_key', 'api_key', 'user_email', 'user_password')):
            _pack_cstr(value, width, name)
        if not any(fields):
            return  # Clears Firebase
        if not fields[0] or not fields[1] or not fields[2]:
            raise ValueError('Firebase needs a database URL, a fleet ID and a robot key')
        cls.validate_fleet_id(fields[1])
        cls.validate_robot_key(fields[2])
        if sum(1 for f in fields[3:] if f) not in (0, 3):
            raise ValueError('Signing in needs the API key, the email and the password, or none of them')

    def get_wifi_config(self):
        """Read the Wi-Fi credentials the ESP32 is using. Returns a dict, or None on failure."""
        out = {}

        def cb(s):
            ssid, password, flags = struct.unpack_from(self.FMT_WIFI_CONFIG, s)
            out.update({'ssid': _cstr(ssid), 'password': _cstr(password),
                        'configured': bool(flags & self.CFG_FLAG_CONFIGURED)})
        if not self._rpc_pull(self.RPC_GET_WIFI_CONFIG, self.RPC_REPLY_WIFI_CONFIG, cb):
            return None
        return out

    def get_firebase_config(self):
        """
        Read the Firebase settings the ESP32 is using: url, fleet_id, robot_key, api_key,
        user_email, user_password, configured. Returns a dict, or None on failure.
        """
        out = {}

        def cb(s):
            out.update(self._unpack_firebase_config(s))
        if not self._rpc_pull(self.RPC_GET_FIREBASE_CONFIG, self.RPC_REPLY_FIREBASE_CONFIG, cb):
            return None
        return out

    def supports_firebase(self):
        """Whether the firmware takes this Firebase config: FIRST_FIREBASE_FIRMWARE or later."""
        return firmware_version_tuple(self.board_info.get('firmware_version')) >= \
            firmware_version_tuple(self.FIRST_FIREBASE_FIRMWARE)

    # ###########  Triggers (sent on the next push_command) #############

    def trigger_wifi_reconnect(self):
        self._trigger |= self.TRIGGER_WIFI_RECONNECT

    def trigger_wifi_disconnect(self):
        self._trigger |= self.TRIGGER_WIFI_DISCONNECT

    def forget_wifi(self):
        """Erase the Wi-Fi credentials saved on the ESP32 and disconnect."""
        self._trigger |= self.TRIGGER_WIFI_FORGET

    def trigger_firebase_restart(self):
        self._trigger |= self.TRIGGER_FIREBASE_RESTART

    def forget_firebase(self):
        """Erase the Firebase credentials saved on the ESP32 and stop Firebase."""
        self._trigger |= self.TRIGGER_FIREBASE_FORGET

    def start_wifi_scan(self):
        """Start a Wi-Fi scan; read it with get_wifi_scan() once status['wifi']['state_name'] is no longer SCANNING."""
        self._trigger |= self.TRIGGER_WIFI_SCAN

    def reboot(self):
        """Restart the ESP32. Known issue: an ESP32 reset cuts the robot's power (no pull-down on the power-off line)."""
        self._trigger |= self.TRIGGER_REBOOT

    def clear_log(self):
        self._trigger |= self.TRIGGER_LOG_CLEAR

    # ###########  Reads #############

    def get_wifi_scan(self):
        """Result of the last scan: {'state_name': ..., 'networks': [{'ssid', 'rssi', 'channel', 'auth'}]} or None."""
        out = {}

        def cb(s):
            state, n = struct.unpack_from(self.FMT_SCAN_HEADER, s)
            entry_size = struct.calcsize(self.FMT_SCAN_ENTRY)
            networks = []
            for i in range(min(n, self.SCAN_MAX)):
                ssid, rssi, channel, auth = struct.unpack_from(self.FMT_SCAN_ENTRY, s, 2 + i * entry_size)
                networks.append({'ssid': _cstr(ssid), 'rssi': rssi, 'channel': channel,
                                 'auth': self.AUTH_MODE_NAMES.get(auth, str(auth))})
            out.update({'state': state, 'state_name': self.SCAN_STATE_NAMES.get(state, str(state)),
                        'networks': networks})
        if not self._rpc_pull(self.RPC_GET_WIFI_SCAN, self.RPC_REPLY_WIFI_SCAN, cb):
            return None
        return out

    def read_log(self):
        """
        Drain the ESP32's log. Returns a list of {'ms', 'level', 'level_name', 'text'} (oldest first),
        with an entry noting any lost to overflow, or None on failure.
        """
        entries = []
        for _ in range(16):  # The ring holds 64 entries: a few reads empty it
            batch = []
            if not self._rpc_pull(self.RPC_READ_LOG, self.RPC_REPLY_READ_LOG,
                                  lambda s: batch.extend(self.unpack_log(s))):
                return entries or None
            entries.extend(batch)
            if not any(e['level'] is not None for e in batch):
                break
        return entries

    @classmethod
    def unpack_log(cls, s):
        dropped, n = struct.unpack_from('<HB', s)
        out = []
        if dropped:
            out.append({'ms': None, 'level': None, 'level_name': 'DROPPED',
                        'text': '%d log entries lost to overflow' % dropped})
        idx = 3
        for _ in range(n):
            ms, level, length = struct.unpack_from('<IBB', s, idx)
            idx += 6
            text = bytes(s[idx:idx + length]).decode('utf-8', errors='replace')
            idx += length
            out.append({'ms': ms, 'level': level, 'level_name': cls.LOG_LEVEL_NAMES.get(level, str(level)),
                        'text': text})
        return out

    # ###########  Transport #############

    def _rpc_pull(self, rpc_id, reply_id, unpack, quiet=False):
        return self._rpc(arr.array('B', [rpc_id]), False, reply_id, unpack, quiet)

    def _rpc_push(self, payload, reply_id, unpack=None):
        return self._rpc(payload, True, reply_id, unpack)

    def _rpc(self, payload, is_push, reply_id, unpack, quiet=False):
        """Run one RPC. Returns True if the expected reply came back and unpacked."""
        self._rpc_ok = False
        log = self.logger.debug if quiet else self.logger.warning

        def cb(reply):
            if len(reply) > 0 and reply[0] == reply_id:
                if unpack is not None:
                    unpack(reply[1:])
                self._rpc_ok = True
            else:
                log('Esp32: RPC %d: expected reply %d, got %s'
                    % (payload[0], reply_id, reply[0] if len(reply) else 'nothing'))
        try:
            self.transport.do_rpc(blocking=True, is_push=is_push, payload=payload, rpc_callback=cb)
        except IndexError:
            # How the transport reports a timeout: it indexes the empty reply
            log('Esp32: RPC %d: no reply' % payload[0])
            return False
        except Exception as e:
            log('Esp32: RPC %d failed: %s: %s' % (payload[0], type(e).__name__, e))
            return False
        return self._rpc_ok

    def _check_set_result(self, s):
        self._last_set_result = s[0] if len(s) else None
        if self._last_set_result != self.RPC_RESULT_OK:
            self.logger.warning('Esp32 rejected the configuration (result %s)' % self._last_set_result)

    def _pull_board_info(self):
        # Quiet: startup probes, and firmware without RPC never answers
        return self._rpc_pull(self.RPC_GET_BOARD_INFO, self.RPC_REPLY_BOARD_INFO, self._unpack_board_info, quiet=True)

    def _unpack_board_info(self, s):
        variant, version = struct.unpack_from(self.FMT_BOARD_INFO, s)
        self.board_info['board_variant'] = _cstr(variant)
        self.board_info['firmware_version'] = _cstr(version)
        self.board_info['protocol_version'] = self.board_info['firmware_version'][
                                              self.board_info['firmware_version'].rfind('p'):]
        try:
            self.board_info['hardware_id'] = int(self.board_info['board_variant'][-1])
        except (ValueError, IndexError):
            self.board_info['hardware_id'] = 0

    def _pack_config(self, c):
        return struct.pack(self.FMT_CONFIG, int(c.get('firebase_publish_period_ms', 5000)),
                           int(c.get('wifi_retry_ms', 5000)), int(c.get('wifi_enable', 1)),
                           int(c.get('firebase_enable', 1)), int(c.get('log_level', self.LOG_INFO)))

    # Protocol specific, in Esp32_Protocol_P1
    def _unpack_status(self, s):
        raise NotImplementedError('Esp32 protocol not set')

    def _unpack_wifi_status(self, s):
        raise NotImplementedError('Esp32 protocol not set')

    def _unpack_firebase_status(self, s):
        raise NotImplementedError('Esp32 protocol not set')

    def _pack_firebase_config(self, fields, flags):
        raise NotImplementedError('Esp32 protocol not set')

    def _unpack_firebase_config(self, s):
        raise NotImplementedError('Esp32 protocol not set')

    # ###########  Printing #############

    def pretty_print(self):
        s = self.status
        print('----------- Esp32 ----------')
        print('Firmware', self.board_info['firmware_version'], '| Board', self.board_info['board_variant'])
        print('Power state %s | Reset reason %s | CPU %d MHz | Heap %d (min %d) | Loop max %d us | Power task max %d us'
              % (s['pwr_state_name'], s['reset_reason_name'], s['cpu_freq_mhz'], s['free_heap'], s['min_free_heap'],
                 s['loop_time_us'], s['power_task_time_us']))
        print('12V0 %.2f V | Aux 20V0 %.2f V | Adapter connected %s | Adapter fault %s | NUC USB 5V %s'
              % (s['voltage_12v0'], s['voltage_20v0_aux'], s['adapter_connected'], s['adapter_fault'],
                 s['cpu_5v_usb_on']))
        print('Outputs', s['outputs'])
        p = s['pimu_link']
        age = 'never' if p['status_age_ms'] is None else '%d ms' % p['status_age_ms']
        battery = ('%.2f V %d%%' % (p['voltage_battery'], p['battery_soc'])) if p['battery_valid'] else '-'
        print('Pimu link: fresh %s (age %s) | Battery %s (%s) | Charging %s | Runstop %s | NUC %s | NUC rail off %s'
              % (p['status_fresh'], age, battery, p['battery_status'], p['charger_charging'],
                 p['runstop_event'], p['cpu_power_state_name'], p['nuc_rail_off']))
        print('  UART rx %d | crc errors %d | overflows %d | tx %d | tx drops %d'
              % (p['uart_rx_packets'], p['uart_crc_errors'], p['uart_rx_overflows'], p['uart_tx_packets'],
                 p['uart_tx_drops']))
        w = s['wifi']
        print('Wi-Fi: %s (%s) | SSID "%s" | RSSI %d | Ch %d | IP %s | MAC %s | Host %s'
              % (w['state_name'], w['wl_status_name'], w['ssid'], w['rssi'], w['channel'], w['ip'], w['mac'],
                 w['hostname']))
        print('  Attempts %d | Disconnects %d | Last reason %s | Connected %.0f s | Low power %s'
              % (w['connect_attempts'], w['disconnects'], w['last_disconnect_reason_name'] or '-',
                 w['connected_s'], w['low_power']))
        f = s['firebase']
        age = 'never' if f['last_publish_age_s'] is None else '%.1f s ago' % f['last_publish_age_s']
        print('Firebase: %s | Inits %d | Published ok %d, errors %d, last %s | Last error %s %s | Stream events %d'
              % (f['state_name'], f['init_count'], f['publish_ok'], f['publish_errors'], age, f['last_error_code'],
                 f['last_error'], f['stream_events']))
        if f['cmd_action']:
            print('  Last power command: %s "%s" %s, %.0f s ago'
                  % (f['cmd_action_name'], f['cmd_id'], f['cmd_result_name'], f['cmd_age_s'] or 0))


class Esp32_Protocol_P1(Esp32Base):
    def _unpack_status(self, s):
        # Older firmware ends a byte sooner, without the battery status
        if len(s) >= struct.calcsize(self.FMT_STATUS_BATTERY):
            v = struct.unpack_from(self.FMT_STATUS_BATTERY, s)
        else:
            v = struct.unpack_from(self.FMT_STATUS, s) + (0,)
        st = self.status
        st['timestamp'] = self.timestamp.set(v[0])
        st['pwr_state'] = v[1]
        st['pwr_state_name'] = self.PWR_STATE_NAMES.get(v[1], str(v[1]))
        st['reset_reason'] = v[2]
        st['reset_reason_name'] = self.RESET_REASON_NAMES.get(v[2], str(v[2]))
        st['cpu_freq_mhz'] = v[3]
        st['free_heap'] = v[4]
        st['min_free_heap'] = v[5]
        st['loop_time_us'] = v[6]
        st['power_task_time_us'] = v[7]
        st['log_pending'] = v[8]
        st['log_dropped'] = v[9]
        st['voltage_12v0'] = v[10]
        st['voltage_20v0_aux'] = v[11]
        st['adapter_connected'] = bool(v[12])
        st['adapter_fault'] = bool(v[13])
        st['cpu_5v_usb_on'] = bool(v[14])
        o = st['outputs']
        for i, k in enumerate(('en_12v0', 'en_aux_20v0', 'en_lidar', 'en_status_leds', 'dcm_mode',
                               'robot_active', 'cpu_sd', 'system_shutdown')):
            o[k] = bool(v[15 + i])
        o['sd_pulse_count'] = v[23]
        p = st['pimu_link']
        p['voltage_battery'] = v[24]
        p['voltage_20v0'] = v[25]
        p['temperature'] = v[26]
        p['battery_soc'] = v[27]
        p['charger_charging'] = bool(v[28])
        p['runstop_event'] = bool(v[29])
        p['cpu_power_state'] = v[30]
        p['cpu_power_state_name'] = self.CPU_POWER_STATE_NAMES.get(v[30], str(v[30]))
        p['nuc_rail_off'] = bool(v[31])
        p['status_fresh'] = bool(v[32])
        p['status_age_ms'] = None if v[33] == self.UNKNOWN_AGE else v[33]
        p['uart_rx_packets'] = v[34]
        p['uart_crc_errors'] = v[35]
        p['uart_rx_overflows'] = v[36]
        p['uart_tx_packets'] = v[37]
        p['uart_tx_drops'] = v[38]
        p['esp_status_counter'] = v[39]
        # Not OK: the Pimu sends NaN and 255 in place of its battery values. Both are
        # NaN here, so they can't pass for readings. UNKNOWN (older firmware): as before.
        p['battery_status'] = self.BATTERY_STATUS_NAMES.get(v[40], str(v[40]))
        p['battery_valid'] = v[40] in (0, 1)
        if not p['battery_valid']:
            p['voltage_battery'] = float('nan')
            p['battery_soc'] = float('nan')

    def _unpack_wifi_status(self, s):
        v = struct.unpack_from(self.FMT_WIFI_STATUS, s)
        w = self.status['wifi']
        w['state'] = v[0]
        w['state_name'] = self.WIFI_STATE_NAMES.get(v[0], str(v[0]))
        w['wl_status'] = v[1]
        w['wl_status_name'] = self.WL_STATUS_NAMES.get(v[1], str(v[1]))
        w['enabled'] = bool(v[2])
        w['low_power'] = bool(v[3])
        w['ssid'] = _cstr(v[4])
        w['rssi'] = v[5]
        w['channel'] = v[6]
        w['bssid'] = _mac(v[7])
        w['mac'] = _mac(v[8])
        w['ip'] = _ip(v[9])
        w['gateway'] = _ip(v[10])
        w['netmask'] = _ip(v[11])
        w['dns'] = _ip(v[12])
        w['hostname'] = _cstr(v[13])
        w['connect_attempts'] = v[14]
        w['disconnects'] = v[15]
        w['last_disconnect_reason'] = v[16]
        w['last_disconnect_reason_name'] = self.WIFI_DISCONNECT_REASON_NAMES.get(v[16], str(v[16]))
        w['connected_s'] = v[17] / 1000.0

    def _unpack_firebase_status(self, s):
        v = struct.unpack_from(self.FMT_FIREBASE_STATUS, s)
        f = self.status['firebase']
        f['state'] = v[0]
        f['state_name'] = self.FIREBASE_STATE_NAMES.get(v[0], str(v[0]))
        f['enabled'] = bool(v[1])
        f['app_ready'] = bool(v[2])
        f['db_setup'] = bool(v[3])
        f['init_count'] = v[4]
        f['publish_ok'] = v[5]
        f['publish_errors'] = v[6]
        f['last_publish_age_s'] = None if v[7] == self.UNKNOWN_AGE else v[7] / 1000.0
        f['publish_period_s'] = v[8] / 1000.0
        f['last_error_code'] = v[9]
        f['last_error'] = _cstr(v[10])
        f['stream_events'] = v[11]
        f['stream_age_s'] = None if v[12] == self.UNKNOWN_AGE else v[12] / 1000.0
        f['stream_restarts'] = v[13]
        f['time_synced'] = bool(v[14])
        f['cmd_action'] = v[15]
        f['cmd_action_name'] = self.CMD_ACTION_NAMES.get(v[15], str(v[15]))
        f['cmd_result'] = v[16]
        f['cmd_result_name'] = self.CMD_RESULT_NAMES.get(v[16], str(v[16]))
        f['cmd_age_s'] = None if v[17] == self.UNKNOWN_AGE else v[17] / 1000.0
        f['cmd_id'] = _cstr(v[18])

    def _pack_firebase_config(self, fields, flags):
        url, fleet_id, robot_key, api_key, user_email, user_password = fields
        return struct.pack(self.FMT_FIREBASE_CONFIG, _pack_cstr(url, self.FB_URL_LEN, 'url'),
                           _pack_cstr(fleet_id, self.FB_FLEET_LEN, 'fleet_id'),
                           _pack_cstr(robot_key, self.FB_KEY_LEN, 'robot_key'),
                           _pack_cstr(api_key, self.FB_API_LEN, 'api_key'),
                           _pack_cstr(user_email, self.FB_EMAIL_LEN, 'user_email'),
                           _pack_cstr(user_password, self.FB_PASS_LEN, 'user_password'), flags)

    def _unpack_firebase_config(self, s):
        url, fleet_id, robot_key, api_key, email, password, flags = struct.unpack_from(self.FMT_FIREBASE_CONFIG, s)
        return {'url': _cstr(url), 'fleet_id': _cstr(fleet_id), 'robot_key': _cstr(robot_key), 'api_key': _cstr(api_key),
                'user_email': _cstr(email), 'user_password': _cstr(password),
                'configured': bool(flags & self.CFG_FLAG_CONFIGURED)}


class Esp32(Esp32Base):
    """
    API to the ESP32. Picks the protocol from the firmware's board info at startup.
    """

    def __init__(self, usb=None):
        Esp32Base.__init__(self, usb=usb)
        self._supported_protocols = {'p1': (Esp32_Protocol_P1,)}

    def startup(self, quiet=False):
        if not Esp32Base.startup(self, quiet=quiet):
            return False
        protocol = self.board_info['protocol_version']
        if protocol not in self._supported_protocols:
            self.logger.warning(textwrap.dedent("""
                ----------------
                Firmware protocol mismatch on {0}.
                Protocol on board is {1}.
                Valid protocols are: {2}.
                Disabling device.
                Please upgrade the firmware and/or version of Stretch Body.
                ----------------
                """.format(self.name, protocol, list(self._supported_protocols.keys()))))
            self.stop()
            return False
        Esp32.__bases__ = self._supported_protocols[protocol]
        self._dirty_config = True
        self.push_command()
        self.pull_status()
        return True


# ############################# Firebase set-up #############################

class FirebaseCliError(Exception):
    pass


class FirebaseAuthError(FirebaseCliError):
    """The Firebase CLI is not logged in, or its login has expired: firebase login (--reauth)."""


class Esp32Firebase:
    """
    Firebase set-up for the ESP32's remote sleep and wake: the database rules, and the
    Firebase CLI steps that stretch_esp32_jog can run for the user.

    Each robot keeps its data in a Realtime Database under robots/<fleet ID> (its
    $HELLO_FLEET_ID; hello_esp firmware v0.2.6p1 on; the layout is in
    stretch_firmware_ii/arduino/hello_esp/README.md, "Firebase"):

        robots/<fleet ID>/robot_key              the key, set in the console or with the CLI
        robots/<fleet ID>/power_system_status    written by the robot
        robots/<fleet ID>/power_system_requests  power_command (the app), power_command_ack (the robot)

    Every write carries robot_key, and the rules (rules()) let it in only if that matches
    robots/<fleet ID>/robot_key, which the rules let no one read. Reads are open, or need a
    signed-in user (rules(sign_in=True)).
    """
    # A write is let in if it carries the robot's key
    KEY_CHECK = "newData.child('robot_key').val() === root.child('robots').child($fleetId).child('robot_key').val()"
    DATABASE_HOST_SUFFIXES = ('.firebaseio.com', '.firebasedatabase.app')
    # How the CLI says it can't authenticate. With --json it often says only "Failed to ...",
    # and the reason is in firebase-debug.log
    AUTH_ERROR_MARKERS = ('authentication error', 'failed to authenticate', 'have you run firebase login',
                          'credentials are no longer valid', 'login --reauth')
    # Marks the robot rules put into someone else's rules as a comment, to reconcile
    COMMENT_BEGIN = '/* stretch_esp32_jog: robot power rules, NOT ACTIVE.'
    COMMENT_END = 'end of the robot power rules */'
    CliError = FirebaseCliError
    AuthError = FirebaseAuthError

    # ###########  Rules #############

    @staticmethod
    def rules(sign_in=False):
        """
        The rules for the robots' power data. Without sign_in, reads are open; with it, they
        need a signed-in user, and so do writes. robot_key itself has no rule, so no one can
        read or write it through the rules (the console and the CLI can).
        """
        read = 'auth != null' if sign_in else True
        write = ('auth != null && ' if sign_in else '') + Esp32Firebase.KEY_CHECK
        return {'rules': {'robots': {'$fleetId': {
            'power_system_status': {'.read': read, '.write': write},
            'power_system_requests': {'.read': read,
                                      'power_command': {'.write': write},
                                      'power_command_ack': {'.write': write}}}}}}

    @staticmethod
    def rules_json(sign_in=False):
        """The rules as the text to paste into the console's Rules tab."""
        return json.dumps(Esp32Firebase.rules(sign_in), indent=2)

    @staticmethod
    def strip_comments(source):
        """Rules source without its // and /* */ comments (the rules language allows them)."""
        out, i, n = [], 0, len(source or '')
        while i < n:
            c = source[i]
            if c == '"':
                j = i + 1
                while j < n and source[j] != '"':
                    j += 2 if source[j] == '\\' else 1
                out.append(source[i:j + 1])
                i = j + 1
            elif source.startswith('//', i):
                j = source.find('\n', i)
                i = n if j < 0 else j
            elif source.startswith('/*', i):
                j = source.find('*/', i + 2)
                i = n if j < 0 else j + 2
            else:
                out.append(c)
                i += 1
        return ''.join(out)

    @staticmethod
    def classify_rules(source):
        """
        'default' (a new database's: locked or test mode, which nothing relies on), 'ours'
        (these rules, or this tool's earlier ones), or 'other' (someone's rules, not to be
        replaced). Comments don't count.
        """
        try:
            parsed = json.loads(Esp32Firebase.strip_comments(source))
        except (TypeError, ValueError):
            return 'other'
        if parsed in (Esp32Firebase.rules(False), Esp32Firebase.rules(True)):
            return 'ours'
        inner = parsed.get('rules') if isinstance(parsed, dict) else None
        if not isinstance(inner, dict):
            return 'other'
        if set(inner) == {'robots'} and '$robot_key' in (inner['robots'] or {}):
            return 'ours'  # The earlier robots/<key> rules
        if set(inner) <= {'.read', '.write'} and all(
                v is False or v == 'auth != null' or (isinstance(v, str) and v.replace(' ', '').startswith('now<'))
                for v in inner.values()):
            return 'default'
        return 'other'

    @staticmethod
    def rules_with_comments(source, sign_in=False):
        """
        source (someone's rules) with the robot rules added as a comment, inside "rules", to
        reconcile by hand. Rules that change nothing: what was in force stays in force. A
        comment from an earlier run is replaced.
        """
        src = source or ''
        b = src.find(Esp32Firebase.COMMENT_BEGIN)
        if b >= 0:
            e = src.find(Esp32Firebase.COMMENT_END, b)
            if e >= 0:
                src = src[:b].rstrip(' ') + src[e + len(Esp32Firebase.COMMENT_END):].lstrip('\n')
        robots = json.dumps(Esp32Firebase.rules(sign_in)['rules'], indent=2)
        comment = '\n'.join(
            ['    ' + Esp32Firebase.COMMENT_BEGIN,
             '       Merge them into these rules: add the "robots" entry below, or, if there is a',
             '       "robots" entry already, add "power_system_status" and "power_system_requests" to',
             '       its "$fleetId" entry (whatever it is named). Then publish, and delete this comment.',
             '       Until then the robot can\'t publish its status or read commands.'] +
            ['       ' + line for line in robots[1:-1].strip('\n').splitlines()] +
            ['       ' + Esp32Firebase.COMMENT_END, ''])
        m = re.search(r'"rules"\s*:\s*\{[ \t]*\n?', src)
        if not m:
            return comment + src
        return src[:m.end()] + ('' if src[m.end() - 1] == '\n' else '\n') + comment + src[m.end():]

    @staticmethod
    def default_project_id(fleet_id=None):
        """
        This robot's suggested Firebase project ID: $HELLO_FLEET_ID + '-PWR-MGMT', lowercase
        (project IDs are), e.g. stretch-se4-4024-pwr-mgmt. None without a fleet ID.
        """
        fleet_id = os.environ.get('HELLO_FLEET_ID') if fleet_id is None else fleet_id
        if not fleet_id:
            return None
        return re.sub(r'[^a-z0-9-]', '-', fleet_id.lower()) + '-pwr-mgmt'

    @staticmethod
    def default_database_url(fleet_id=None):
        """
        The database URL of default_project_id's default database, as the console creates it in
        us-central1: https://<project>-default-rtdb.firebaseio.com. None without a fleet ID.
        """
        project = Esp32Firebase.default_project_id(fleet_id)
        return 'https://%s-default-rtdb.firebaseio.com' % project if project else None

    @staticmethod
    def normalize_database_url(url):
        """
        The database URL as the ESP32 wants it: https://<name>.firebaseio.com or
        https://<name>.<region>.firebasedatabase.app, with no path. Raises ValueError for
        something else, such as a console link.
        """
        u = (url or '').strip()
        if '://' not in u:
            u = 'https://' + u
        scheme, rest = u.split('://', 1)
        host = rest.split('/', 1)[0].lower()
        if scheme.lower() != 'https' or not host.endswith(Esp32Firebase.DATABASE_HOST_SUFFIXES) or \
                host.startswith('.'):
            raise ValueError('Not a Realtime Database URL: it looks like https://<name>.firebaseio.com or '
                             'https://<name>.<region>.firebasedatabase.app (at the top of the console\'s Data tab)')
        if rest[len(host):].strip('/'):
            raise ValueError('A database URL has no path: https://%s' % host)
        return 'https://' + host

    # ###########  Firebase CLI #############

    @staticmethod
    def cli_path():
        """The firebase command, or None if the Firebase CLI isn't installed."""
        return shutil.which('firebase')

    @staticmethod
    def run_cli(args, project=None, cwd=None, timeout=180):
        """
        Run firebase <args> --json and return its result. Raises FirebaseAuthError if the CLI
        can't authenticate, else FirebaseCliError with the CLI's message on failure.
        cwd: where to run it. By default a temporary directory, so firebase-debug.log isn't left
        in the user's directory.
        """
        if cwd is None:
            with tempfile.TemporaryDirectory(prefix='stretch_firebase_') as d:
                return Esp32Firebase.run_cli(args, project=project, cwd=d, timeout=timeout)
        cmd = [Esp32Firebase.cli_path() or 'firebase'] + list(args) + \
            (['--project', project] if project else []) + ['--json', '--non-interactive']
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=cwd)
        except (OSError, subprocess.TimeoutExpired) as e:
            raise FirebaseCliError('firebase %s: %s' % (args[0], e))
        out = r.stdout
        try:
            d = json.loads(out[out.index('{'):])
            error = None if d.get('status') == 'success' else str(d.get('error') or 'firebase %s failed' % args[0])
        except ValueError:
            tail = (r.stderr or out).strip().splitlines()[-3:]
            error = 'firebase %s failed: %s' % (args[0], ' '.join(tail) or 'no output')
        if error is None:
            return d.get('result')
        # Searched, never shown: the debug log can hold tokens
        try:
            with open(os.path.join(cwd, 'firebase-debug.log')) as f:
                debug_log = f.read()
        except OSError:
            debug_log = ''
        if any(m in (error + r.stderr + debug_log).lower() for m in Esp32Firebase.AUTH_ERROR_MARKERS):
            raise FirebaseAuthError(error)
        raise FirebaseCliError(error)

    @staticmethod
    def logged_in():
        """
        Whether the CLI has an account on record. Its login may still have expired: the
        commands then raise FirebaseAuthError.
        """
        try:
            return bool(Esp32Firebase.run_cli(['login:list'], timeout=30))
        except FirebaseCliError:
            return False

    @staticmethod
    def list_projects():
        """[{'projectId', 'displayName', ...}] the account can use."""
        return Esp32Firebase.run_cli(['projects:list'], timeout=60) or []

    @staticmethod
    def create_project(project_id, display_name=None):
        """Create a Google Cloud project with Firebase. project_id: 6-30 of a-z 0-9 -, starting with a letter."""
        return Esp32Firebase.run_cli(['projects:create', project_id] +
                                     (['--display-name', display_name] if display_name else []), timeout=300)

    @staticmethod
    def list_databases(project):
        """The project's Realtime Databases: [{'name', 'databaseUrl', 'type', 'location', ...}]."""
        try:
            return Esp32Firebase.run_cli(['database:instances:list'], project=project, timeout=60) or []
        except FirebaseAuthError:
            raise
        except FirebaseCliError as e:
            if 'default' in str(e).lower() or 'haven\'t created' in str(e).lower():
                return []
            raise

    @staticmethod
    def database_console_url(project):
        """Where to create the project's first Realtime Database (the CLI can only add more)."""
        return 'https://console.firebase.google.com/project/%s/database' % project

    # ###########  Checking access as the ESP32 has it #############

    @staticmethod
    def http_status(url, method='GET', data=None, timeout=10):
        """The HTTP status of a request to url (data: JSON to send). 0 if the server can't be reached."""
        req = urllib.request.Request(url, method=method,
                                     data=None if data is None else json.dumps(data).encode())
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code
        except (urllib.error.URLError, OSError):
            return 0

    @staticmethod
    def sign_in(api_key, email, password, timeout=10):
        """A Firebase ID token for email and password (the ESP32's optional sign-in). Raises ValueError if refused."""
        req = urllib.request.Request(
            'https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key=' + api_key,
            data=json.dumps({'email': email, 'password': password, 'returnSecureToken': True}).encode(),
            headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return json.load(r)['idToken']
        except urllib.error.HTTPError as e:
            try:
                message = json.load(e)['error']['message']
            except (ValueError, KeyError, TypeError):
                message = 'HTTP %d' % e.code
            raise ValueError('Firebase sign-in refused: %s' % message)
        except (urllib.error.URLError, OSError, KeyError, ValueError) as e:
            raise ValueError('Firebase sign-in failed: %s' % e)

    @staticmethod
    def rules_work(url, fleet_id, robot_key, id_token=None):
        """
        Whether the rules in force do what the robot needs, checked by what they do, from this
        computer, the way the ESP32 connects (signed in with id_token, with a user):
        robots/<fleet_id>/robot_key can't be read, a write with another key is refused, a write
        with robot_key is let in, and the commands can be read.
        Returns (True, '') or (False, what failed).
        """
        base = '%s/robots/%s' % (url, fleet_id)
        auth = '?auth=' + id_token if id_token else ''
        status_url = base + '/power_system_status.json' + auth
        wrong = 'not-' + robot_key if len(robot_key) < 60 else robot_key[::-1]
        if Esp32Firebase.http_status(base + '/robot_key.json' + auth) != 401:
            return False, 'robots/%s/robot_key can be read' % fleet_id
        if Esp32Firebase.http_status(status_url, 'PATCH', {'robot_key': wrong}) != 401:
            return False, 'a write without the robot key is let in'
        code = Esp32Firebase.http_status(status_url, 'PATCH', {'robot_key': robot_key})
        if code != 200:
            return False, 'a write with the robot key is refused (HTTP %d)' % code
        code = Esp32Firebase.http_status(base + '/power_system_requests.json' + auth)
        if code != 200:
            return False, 'the commands can\'t be read (HTTP %d)' % code
        return True, ''

    @staticmethod
    def current_rules(project, instance):
        """The database's rules in force, as text (comments included)."""
        with tempfile.TemporaryDirectory(prefix='stretch_firebase_') as d:
            out = os.path.join(d, 'rules.txt')
            Esp32Firebase.run_cli(['database:get', '/.settings/rules', '--output', out, '--instance', instance],
                                  project=project, cwd=d, timeout=60)
            with open(out) as f:
                return f.read()

    @staticmethod
    def deploy_rules(project, instance, source=None):
        """Publish source (rules text; by default rules_json()) as the database's rules: it replaces them."""
        with tempfile.TemporaryDirectory(prefix='stretch_firebase_') as d:
            with open(os.path.join(d, 'database.rules.json'), 'w') as f:
                f.write((source if source is not None else Esp32Firebase.rules_json()).rstrip('\n') + '\n')
            with open(os.path.join(d, 'firebase.json'), 'w') as f:
                json.dump({'database': [{'instance': instance, 'rules': 'database.rules.json'}]}, f)
            Esp32Firebase.run_cli(['deploy', '--only', 'database'], project=project, cwd=d, timeout=300)

    @staticmethod
    def set_robot_key(project, instance, fleet_id, robot_key):
        """Write robots/<fleet_id>/robot_key (the CLI isn't held to the rules)."""
        Esp32Firebase.run_cli(['database:set', '/robots/%s/robot_key' % fleet_id, '--data', json.dumps(robot_key),
                               '--instance', instance, '--force'], project=project, timeout=60)

if __name__ == '__main__':
    e = Esp32()
    if e.startup():
        e.pretty_print()
        for entry in e.read_log() or []:
            print(entry)
        e.stop()
