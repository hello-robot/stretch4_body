#!/usr/bin/env python3
"""
Esp32Loop: the robot server's ESP32 subsystem.

The ESP32 is read in its own process at a low rate (esp32.loop_rate_Hz), so its
RPCs (a few ms each, up to a second on a timeout) never touch the control loop.
Commands from clients go to the process through a queue; status, including the
ESP32's log, comes back through another.

The ESP32 is not needed for the robot to run, so a missing ESP32, or one on
firmware older than hello-esp.v0.2.0p1 (no RPC), does not stop the server: its
status reports hw_valid False and the process keeps trying to connect.
"""
import collections
import copy
import queue
import signal
import time
import uuid
from multiprocessing import Process, Event

from stretch4_body.core.device import Device
from stretch4_body.core.worker_loop import worker_loop
import stretch4_body.core.hello_utils as hello_utils
from stretch4_body.subsystem.esp32 import Esp32, Esp32Base, esp32_status_template

# Commands a client may send. Each runs on the Esp32 instance in the loop process.
ESP32_LOOP_COMMANDS = (
    'set_wifi_config', 'set_firebase_config', 'set_config', 'send_credentials_from_params',
    'trigger_wifi_reconnect', 'trigger_wifi_disconnect', 'forget_wifi',
    'trigger_firebase_restart', 'forget_firebase', 'start_wifi_scan', 'reboot', 'clear_log',
)
# Commands after which the credentials are read back
_CONFIG_COMMANDS = ('set_wifi_config', 'set_firebase_config', 'send_credentials_from_params', 'forget_wifi',
                    'forget_firebase')


def sanitize_credentials(wifi, firebase, publish_secrets):
    """The credentials as published in the server status: secrets only if publish_secrets."""
    out_wifi, out_fb = None, None
    if wifi is not None:
        out_wifi = {'ssid': wifi['ssid'], 'configured': wifi['configured'],
                    'password_set': len(wifi['password']) > 0}
        if publish_secrets:
            out_wifi['password'] = wifi['password']
    if firebase is not None:
        out_fb = {'url': firebase['url'], 'user_email': firebase['user_email'],
                  'configured': firebase['configured'],
                  'api_key_set': len(firebase['api_key']) > 0,
                  'user_password_set': len(firebase['user_password']) > 0}
        if publish_secrets:
            out_fb['api_key'] = firebase['api_key']
            out_fb['user_password'] = firebase['user_password']
    return out_wifi, out_fb


class _Esp32Worker:
    """State of the loop process: the Esp32 and what is read less often than status."""
    LOG_LEVELS = {Esp32Base.LOG_DEBUG: 'debug', Esp32Base.LOG_INFO: 'info', Esp32Base.LOG_WARN: 'warning',
                  Esp32Base.LOG_ERROR: 'error', Esp32Base.LOG_IDF: 'warning'}

    def __init__(self, params):
        self.esp = Esp32()
        self.logger = self.esp.logger
        self.params = params
        self.log = collections.deque(maxlen=int(params.get('log_history', 50)))
        self.wifi_config = None
        self.firebase_config = None
        self.wifi_scan = None
        self.wifi_scan_ts = 0.0     # When wifi_scan was read
        self.ts_scan_request = 0.0
        self.ts_config_read = 0.0
        self.ts_connect_attempt = 0.0
        self.scan_pending = False
        self.config_pending = True
        self.was_valid = False

    def connect(self):
        quiet = self.ts_connect_attempt > 0  # Say it once, not on every retry
        self.ts_connect_attempt = time.time()
        if self.esp.startup(quiet=quiet):
            self.logger.info('ESP32 connected: %s' % self.esp.board_info['firmware_version'])
            self.config_pending = True
            self.was_valid = True
            return True
        return False

    def read_log(self):
        for e in self.esp.read_log() or []:
            if e['level'] is None:
                self.logger.warning('ESP32: %s' % e['text'])
            else:
                getattr(self.logger, self.LOG_LEVELS.get(e['level'], 'info'))('ESP32: %s' % e['text'])
            self.log.append(e)

    def read_config(self):
        self.wifi_config = self.esp.get_wifi_config()
        self.firebase_config = self.esp.get_firebase_config()
        self.ts_config_read = time.time()
        self.config_pending = False


def _cb_esp32_loop_step(w, q_cmd_in, status_out):
    # Commands, even while the ESP32 is away: they are sent once it is back
    commanded = False
    while q_cmd_in.qsize():
        try:
            cmd = q_cmd_in.get_nowait()
            subsystem, method, cmd_id, args, kwargs = cmd
            if method not in ESP32_LOOP_COMMANDS:
                w.logger.error('Esp32Loop: no such command %s' % method)
                continue
            getattr(w.esp, method)(*args, **kwargs)
            commanded = True
            if method in _CONFIG_COMMANDS:
                w.config_pending = True
            if method == 'start_wifi_scan':
                w.scan_pending = True
                w.ts_scan_request = time.time()
        except queue.Empty:
            break
        except Exception as e:
            # A bad command must not take the loop down with it
            w.logger.error('Esp32Loop: command failed: %s' % e)

    if not w.esp.hw_valid:
        if w.was_valid:
            w.logger.warning('ESP32 connection lost')
            w.was_valid = False
        if time.time() - w.ts_connect_attempt > float(w.params.get('reconnect_period_s', 10.0)):
            w.connect()
    if w.esp.hw_valid:
        if commanded:
            w.esp.push_command()
        if not w.esp.pull_status():
            if not w.esp.pull_status():  # One retry before calling it lost
                w.esp.stop()
        if w.esp.hw_valid:
            w.read_log()
            if w.config_pending or time.time() - w.ts_config_read > float(w.params.get('config_read_period_s', 10.0)):
                w.read_config()
            # A scan takes seconds. Wait a moment before looking, so the previous
            # result isn't taken for this one before the ESP32 has started scanning
            if w.scan_pending and time.time() - w.ts_scan_request > 1.5 and \
                    w.esp.status['wifi']['state_name'] != 'SCANNING':
                scan = w.esp.get_wifi_scan()
                if scan is not None and scan['state_name'] in ('DONE', 'FAILED'):
                    w.wifi_scan = scan
                    w.wifi_scan_ts = time.time()
                    w.scan_pending = False
                elif time.time() - w.ts_scan_request > 30.0:
                    w.scan_pending = False

    status_out.update(copy.deepcopy(w.esp.status))
    status_out.pop('transport', None)
    status_out['hw_valid'] = w.esp.hw_valid
    status_out['board_info'] = dict(w.esp.board_info)
    wifi_cfg, fb_cfg = sanitize_credentials(w.wifi_config, w.firebase_config,
                                            bool(w.params.get('publish_credentials', 0)))
    status_out['wifi_config'] = wifi_cfg
    status_out['firebase_config'] = fb_cfg
    status_out['wifi_scan'] = w.wifi_scan
    status_out['wifi_scan_ts'] = w.wifi_scan_ts
    status_out['log'] = list(w.log)
    return True


def _cb_esp32_loop_pause(w):
    # Close the port, so a tool (or the firmware updater) can have it
    w.esp.stop()
    return True


def _cb_esp32_loop_unpause(w):
    w.connect()
    return True


def _cb_esp32_loop_exit(w):
    w.esp.stop()
    return True


def esp32_loop(do_exit, rate_hz, q_admin, q_cmd, q_status, params):
    w = _Esp32Worker(params)
    if not w.connect():
        w.logger.warning('ESP32 not available yet on %s (firmware %s or later has RPC). Retrying every %.0f s.'
                         % (w.esp.transport.port_name, Esp32Base.FIRST_RPC_FIRMWARE,
                            float(params.get('reconnect_period_s', 10.0))))
    worker_loop(
        loop_name='esp32_loop',
        rate_hz=rate_hz,
        worker_instance=w,
        q_admin=q_admin,
        q_status=q_status,
        q_cmd=q_cmd,
        do_exit=do_exit,
        callback_step=_cb_esp32_loop_step,
        callback_pause=_cb_esp32_loop_pause,
        callback_unpause=_cb_esp32_loop_unpause,
        callback_exit=_cb_esp32_loop_exit,
    )
    w.esp.stop()
    return True


class Esp32Loop(Device):
    """
    The robot server's ESP32 subsystem (server.subsystems: esp32).

    Methods called through the server (RobotClient.esp32) queue a command for the
    loop process and return at once. Results show up in status:
      status['wifi'], ['firebase'], ['pimu_link'], ... : as Esp32.status
      status['wifi_config'], ['firebase_config']: the credentials the ESP32 uses. Passwords
          and the API key only if esp32.publish_credentials is 1; otherwise *_set flags.
      status['wifi_scan']: the last scan, after start_wifi_scan()
      status['log']: the ESP32's recent log entries
    """
    def __init__(self):
        Device.__init__(self, 'esp32')
        self.esp32_process = None
        self.q_cmd = hello_utils.CircularMultiprocessingQueue(100)
        self.q_status = hello_utils.CircularMultiprocessingQueue(3)
        self.q_admin = hello_utils.CircularMultiprocessingQueue(3)
        self.do_exit = Event()
        self.status = esp32_status_template()
        self.status.update({'hw_valid': False, 'board_info': {}, 'wifi_config': None, 'firebase_config': None,
                            'wifi_scan': None, 'wifi_scan_ts': 0.0, 'log': [], 'loop': {}})
        self.status_aux = {}

    def startup(self):
        """Start the loop process. Returns True even without an ESP32: the robot runs without one."""
        self.logger.info('Starting Esp32Loop...')
        if self.esp32_process is None:
            self.esp32_process = Process(
                target=esp32_loop,
                args=(self.do_exit, float(self.params.get('loop_rate_Hz', 2.0)), self.q_admin, self.q_cmd,
                      self.q_status, dict(self.params)))
            self.esp32_process.start()
            # Wait for the first status: the process posts it after its first connect attempt
            ts = time.time()
            while time.time() - ts < float(self.params.get('startup_timeout_s', 5.0)):
                self.pull_status()
                if self.status['loop']:
                    break
                time.sleep(0.05)
            if not self.status['hw_valid']:
                self.logger.warning('Esp32Loop running, but no ESP32 yet. Its status will report hw_valid False.')
        return True

    JOIN_TIMEOUT_S = 3.0

    def stop(self):
        original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, lambda *args: self.do_exit.set())
        self.q_admin.put('exit')
        p = self.esp32_process
        if p is not None:
            p.join(timeout=self.JOIN_TIMEOUT_S)
            if p.is_alive():
                self.logger.warning('Esp32Loop did not exit on request -- terminating')
                p.terminate()
                p.join(timeout=1.0)
            if p.is_alive():
                p.kill()
                p.join(timeout=1.0)
            self.esp32_process = None
        signal.signal(signal.SIGINT, original_sigint)

    def pause_transport(self):
        self.q_admin.put('pause')

    def unpause_transport(self):
        self.q_admin.put('unpause')

    def pull_status(self, blocking=False):
        """Take the latest status from the loop process. Non blocking."""
        while self.q_status.qsize():
            try:
                self.status.update(self.q_status.get(block=False))
            except queue.Empty:
                pass
        return True

    def pull_status_aux(self, blocking=False):
        return True

    def push_command(self, blocking=False):
        """No-op: commands reach the loop process through q_cmd as they are dispatched."""
        pass

    def load_rpc_results(self, wait_on_result=True):
        pass

    # -- commands (dispatched by the server) ----------------------------------
    #
    # The server's dispatch lets only AttributeError through, so these never
    # raise: a bad argument is logged and the call returns False.

    def _queue(self, method, *args, **kwargs):
        self.q_cmd.put(['esp32', method, uuid.uuid1(), args, kwargs])
        return True

    def set_wifi_config(self, ssid, password, persist=True, apply_now=True):
        try:
            Esp32Base.validate_wifi_config(ssid, password)
        except ValueError as e:
            self.logger.error('esp32.set_wifi_config: %s' % e)
            return False
        return self._queue('set_wifi_config', ssid, password, persist=persist, apply_now=apply_now)

    def set_firebase_config(self, url, api_key, user_email, user_password, persist=True, apply_now=True):
        try:
            Esp32Base.validate_firebase_config(url, api_key, user_email, user_password)
        except ValueError as e:
            self.logger.error('esp32.set_firebase_config: %s' % e)
            return False
        return self._queue('set_firebase_config', url, api_key, user_email, user_password, persist=persist,
                           apply_now=apply_now)

    def set_config(self, **kwargs):
        return self._queue('set_config', **kwargs)

    def send_credentials_from_params(self):
        return self._queue('send_credentials_from_params')

    def trigger_wifi_reconnect(self):
        return self._queue('trigger_wifi_reconnect')

    def trigger_wifi_disconnect(self):
        return self._queue('trigger_wifi_disconnect')

    def forget_wifi(self):
        return self._queue('forget_wifi')

    def trigger_firebase_restart(self):
        return self._queue('trigger_firebase_restart')

    def forget_firebase(self):
        return self._queue('forget_firebase')

    def start_wifi_scan(self):
        return self._queue('start_wifi_scan')

    def reboot(self):
        return self._queue('reboot')

    def clear_log(self):
        return self._queue('clear_log')


if __name__ == '__main__':
    el = Esp32Loop()
    el.startup()
    try:
        while True:
            el.pull_status()
            print('hw_valid %s | Wi-Fi %s | Firebase %s | 12V0 %.2f V'
                  % (el.status['hw_valid'], el.status['wifi']['state_name'], el.status['firebase']['state_name'],
                     el.status['voltage_12v0']))
            time.sleep(1.0)
    except KeyboardInterrupt:
        el.stop()
