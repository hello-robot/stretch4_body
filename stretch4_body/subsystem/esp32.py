#!/usr/bin/env python3
"""
API to the ESP32 (hello_esp firmware) over its USB CDC port, /dev/hello-esp32.

Same pattern as PowerPeriph and Stepper: pull status, push config, push triggers,
over the Hello Robot RPC transport. Needs hello-esp firmware v0.2.0p1 or later;
older firmware has no RPC (configure it through PowerPeriph.send_network_info /
send_firebase_info, which go through the Pimu).

The robot server runs this class in its own process (Esp32Loop). Use it directly
only with the server stopped, as stretch_esp32_jog -d does.

The packed layouts below must match stretch_firmware_ii/arduino/hello_esp/EspRpc.h.
"""
import array as arr
import struct
import textwrap
import time

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


def params_credentials(robot_params):
    """
    The Wi-Fi and Firebase credentials in the robot parameters: esp32.wifi and esp32.firebase,
    falling back to the older power-periph.firebase (stretch_configuration_params.yaml) and
    power_periph.firebase blocks. 'NA' counts as unset (None).
    Returns ({'ssid', 'password'}, {'url', 'api_key', 'user_email', 'user_password'}).
    """
    def val(d, k):
        v = (d or {}).get(k)
        return None if v in (None, 'NA', '') else str(v)

    esp32 = robot_params.get('esp32') or {}
    legacy = [(robot_params.get(name) or {}).get('firebase') or {} for name in ('power-periph', 'power_periph')]

    def first(d, k, legacy_key):
        return val(d, k) or next((val(l, legacy_key) for l in legacy if val(l, legacy_key)), None)

    wifi = {'ssid': first(esp32.get('wifi'), 'ssid', 'network_ssid'),
            'password': first(esp32.get('wifi'), 'password', 'network_password')}
    firebase = {k: first(esp32.get('firebase'), k, k) for k in ('url', 'api_key', 'user_email', 'user_password')}
    return wifi, firebase


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
                     'last_power_trigger': 0, 'last_trigger_age_s': None, 'stream_events': 0},
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
                            4: 'READY'}
    SCAN_STATE_NAMES = {0: 'IDLE', 1: 'RUNNING', 2: 'DONE', 3: 'FAILED'}
    AUTH_MODE_NAMES = {0: 'OPEN', 1: 'WEP', 2: 'WPA_PSK', 3: 'WPA2_PSK', 4: 'WPA_WPA2_PSK',
                       5: 'WPA2_ENTERPRISE', 6: 'WPA3_PSK', 7: 'WPA2_WPA3_PSK', 8: 'WAPI_PSK',
                       9: 'OWE', 10: 'WPA3_ENT_192'}

    # Field widths, including the NUL
    SSID_LEN = 33
    WIFI_PASS_LEN = 65
    FB_URL_LEN = 251
    FB_API_LEN = 129
    FB_EMAIL_LEN = 251
    FB_PASS_LEN = 65
    SCAN_MAX = 20

    # Protocol p1 layouts (little endian, packed)
    FMT_BOARD_INFO = '<20s30s'
    FMT_STATUS = '<QBBHIIHHHHff' + 'B' * 12 + 'fff' + 'B' * 6 + 'IIIIII' + 'B'
    FMT_WIFI_STATUS = '<BBBB33sbB6s6s4s4s4s4s32sIIBI'
    FMT_FIREBASE_STATUS = '<BBBBIIIIIi64siII'
    FMT_WIFI_CONFIG = '<33s65sB'
    FMT_FIREBASE_CONFIG = '<251s129s251s65sB'
    FMT_CONFIG = '<IIBBB'
    FMT_TRIGGER = '<I'
    FMT_SCAN_HEADER = '<BB'
    FMT_SCAN_ENTRY = '<33sbBB'

    # Firmware older than this has no RPC (it only prints)
    FIRST_RPC_FIRMWARE = 'hello-esp.v0.2.0p1'
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
        self.status_aux = {}
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

    def pause_transport(self):
        self.transport.pause()

    def unpause_transport(self):
        self.transport.unpause()

    def pull_status(self, blocking=True):
        """
        Read the system, Wi-Fi and Firebase status. The ESP32 is always read
        synchronously, whatever `blocking` says. Returns True if all three were read.
        """
        if not self.hw_valid:
            return False
        ok = self._rpc_pull(self.RPC_GET_STATUS, self.RPC_REPLY_STATUS, self._unpack_status)
        ok = self._rpc_pull(self.RPC_GET_WIFI_STATUS, self.RPC_REPLY_WIFI_STATUS, self._unpack_wifi_status) and ok
        ok = self._rpc_pull(self.RPC_GET_FIREBASE_STATUS, self.RPC_REPLY_FIREBASE_STATUS,
                            self._unpack_firebase_status) and ok
        return ok

    def pull_status_aux(self, blocking=True):
        return True

    def push_command(self, blocking=True):
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

    def load_rpc_results(self, wait_on_result=True):
        return []

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

    def set_firebase_config(self, url, api_key, user_email, user_password, persist=True, apply_now=True):
        """
        Set the Firebase project and user (sent on the next push_command). All four, or all
        empty to clear Firebase. persist: save to the ESP32's flash. apply_now: restart Firebase now.
        """
        fields = (url or '', api_key or '', user_email or '', user_password or '')
        self.validate_firebase_config(*fields)
        flags = (self.CFG_FLAG_PERSIST if persist else 0) | (self.CFG_FLAG_APPLY_NOW if apply_now else 0)
        self._pending_firebase_config = struct.pack(
            self.FMT_FIREBASE_CONFIG, _pack_cstr(fields[0], self.FB_URL_LEN, 'url'),
            _pack_cstr(fields[1], self.FB_API_LEN, 'api_key'),
            _pack_cstr(fields[2], self.FB_EMAIL_LEN, 'user_email'),
            _pack_cstr(fields[3], self.FB_PASS_LEN, 'user_password'), flags)

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
    def validate_firebase_config(cls, url, api_key, user_email, user_password):
        """Raise ValueError if the ESP32 would reject these Firebase credentials."""
        fields = (url or '', api_key or '', user_email or '', user_password or '')
        if sum(1 for f in fields if len(f)) not in (0, 4):
            raise ValueError('Set all four Firebase fields, or none')
        for value, width, name in zip(fields, (cls.FB_URL_LEN, cls.FB_API_LEN, cls.FB_EMAIL_LEN, cls.FB_PASS_LEN),
                                      ('url', 'api_key', 'user_email', 'user_password')):
            _pack_cstr(value, width, name)

    def get_params_credentials(self):
        """Credentials from the robot parameters (see params_credentials). Returns (wifi, firebase) dicts."""
        return params_credentials(self.robot_params)

    def send_credentials_from_params(self, persist=True, apply_now=True):
        """Queue the Wi-Fi and Firebase credentials from the robot parameters. Returns what was queued."""
        wifi, firebase = self.get_params_credentials()
        sent = []
        if wifi['ssid']:
            self.set_wifi_config(wifi['ssid'], wifi['password'] or '', persist, apply_now)
            sent.append('wifi')
        if all(firebase.values()):
            self.set_firebase_config(firebase['url'], firebase['api_key'], firebase['user_email'],
                                     firebase['user_password'], persist, apply_now)
            sent.append('firebase')
        return sent

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
        """Read the Firebase credentials the ESP32 is using. Returns a dict, or None on failure."""
        out = {}

        def cb(s):
            url, api_key, email, password, flags = struct.unpack_from(self.FMT_FIREBASE_CONFIG, s)
            out.update({'url': _cstr(url), 'api_key': _cstr(api_key), 'user_email': _cstr(email),
                        'user_password': _cstr(password), 'configured': bool(flags & self.CFG_FLAG_CONFIGURED)})
        if not self._rpc_pull(self.RPC_GET_FIREBASE_CONFIG, self.RPC_REPLY_FIREBASE_CONFIG, cb):
            return None
        return out

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
        """Restart the ESP32. Its power outputs reset while it boots."""
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
        print('Pimu link: fresh %s (age %s) | Battery %.2f V %d%% | Charging %s | Runstop %s | NUC %s | NUC rail off %s'
              % (p['status_fresh'], age, p['voltage_battery'], p['battery_soc'], p['charger_charging'],
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


class Esp32_Protocol_P1(Esp32Base):
    def _unpack_status(self, s):
        v = struct.unpack_from(self.FMT_STATUS, s)
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
        f['last_power_trigger'] = v[11]
        f['last_trigger_age_s'] = None if v[12] == self.UNKNOWN_AGE else v[12] / 1000.0
        f['stream_events'] = v[13]


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


if __name__ == '__main__':
    e = Esp32()
    if e.startup():
        e.pretty_print()
        for entry in e.read_log() or []:
            print(entry)
        e.stop()
