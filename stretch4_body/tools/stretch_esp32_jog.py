#!/usr/bin/env python3
"""
Status, Wi-Fi and Firebase of the ESP32.

By default this goes through the robot server (its 'esp32' subsystem), with the
RobotClient interface (Esp32Client). With -d/--direct it opens /dev/hello-esp32
itself, which needs the server stopped. It talks only to the ESP32.

Needs ESP32 firmware hello-esp.v0.2.0p1 or later (earlier firmware has no RPC).
"""
import argparse
import getpass
import sys
import time

import click

import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.esp32 import Esp32Base

hu.print_stretch_re_use()

parser = argparse.ArgumentParser(description='Status, Wi-Fi and Firebase of the ESP32')
parser.add_argument('-d', '--direct', help='Use the direct API on /dev/hello-esp32 (server must be stopped)',
                    action='store_true')
parser.add_argument('--status', help='Print the full status and exit', action='store_true')
parser.add_argument('--wifi', help='Print the Wi-Fi status and exit', action='store_true')
parser.add_argument('--firebase', help='Print the Firebase status and exit', action='store_true')
parser.add_argument('--config', help='Print the credentials in use, secrets masked, and exit', action='store_true')
parser.add_argument('--show-credentials', help='Print the credentials in use, unmasked, and exit',
                    action='store_true')
parser.add_argument('--scan', help='Scan for Wi-Fi networks and exit', action='store_true')
parser.add_argument('--log', help='Print the ESP32 log and exit', action='store_true')
args = parser.parse_args()

LABEL_WIDTH = 24


# ############################# Backends #############################

class DirectBackend:
    """The ESP32 through the direct API (stretch4_body.subsystem.esp32.Esp32)."""
    name = 'direct'
    settle_s = 0.3

    def __init__(self):
        from stretch4_body.subsystem.esp32 import Esp32
        self.esp = Esp32()

    def startup(self):
        if self.esp.startup():
            return True
        click.secho('Unable to talk to the ESP32 on %s.' % self.esp.transport.port_name, fg='red')
        click.secho('  Is the server running? Stop it (stretch_body_server --kill), or drop -d to go through it.\n'
                    '  Is the ESP32 firmware older than hello-esp.v0.2.0p1? It has no RPC: update it with\n'
                    '  REx_firmware_updater --install --esp32', fg='yellow')
        return False

    def stop(self):
        self.esp.stop()

    def pull(self):
        self.esp.pull_status()

    def snapshot(self):
        s = dict(self.esp.status)
        s['hw_valid'] = self.esp.hw_valid
        s['board_info'] = self.esp.board_info
        return s

    def credentials(self):
        """(wifi, firebase) with every field, or None where unreadable."""
        return self.esp.get_wifi_config(), self.esp.get_firebase_config()

    def command(self, method, *a, **kw):
        getattr(self.esp, method)(*a, **kw)
        return self.esp.push_command()

    def start_scan_and_wait(self, timeout=20.0):
        self.command('start_wifi_scan')
        ts = time.time()
        time.sleep(1.5)  # Let the ESP32 start it, so the previous result isn't taken for this one
        while time.time() - ts < timeout:
            self.pull()
            if self.esp.status['wifi']['state_name'] != 'SCANNING':
                scan = self.esp.get_wifi_scan()
                if scan is not None and scan['state_name'] in ('DONE', 'FAILED'):
                    return scan
            time.sleep(0.5)
        return None

    def log(self):
        return self.esp.read_log() or []

    def params_credentials(self):
        return self.esp.get_params_credentials()


class ServerBackend:
    """The ESP32 through the robot server's esp32 subsystem (Esp32Client)."""
    name = 'server'
    settle_s = 1.5  # The server reads the ESP32 at esp32.loop_rate_Hz

    def __init__(self):
        from stretch4_body.robot.robot_client import Esp32Client
        self.esp = Esp32Client()
        self._last_log_ms = None

    def startup(self):
        if not self.esp.startup():
            click.secho('Unable to connect to the robot server. Start it (stretch_body_server --launch), '
                        'or use -d with the server stopped.', fg='red')
            return False
        if 'hw_valid' not in self.esp.status:
            click.secho("The server has no esp32 subsystem. Add 'esp32' to robot.server.subsystems in "
                        "stretch_user_params.yaml and restart it, or use -d with the server stopped.", fg='red')
            self.esp.stop()
            return False
        if not self.esp.status['hw_valid']:
            click.secho('The server is not talking to the ESP32 (firmware older than hello-esp.v0.2.0p1?).',
                        fg='yellow')
        return True

    def stop(self):
        self.esp.stop()

    def pull(self):
        self.esp.pull_status()

    def snapshot(self):
        return self.esp.status

    def credentials(self):
        return self.esp.status.get('wifi_config'), self.esp.status.get('firebase_config')

    def command(self, method, *a, **kw):
        getattr(self.esp, method)(*a, **kw)
        # ESP32 settings don't move the robot: don't take control from whoever has it
        return self.esp.push_command(ignore_control_lock=True)

    def start_scan_and_wait(self, timeout=25.0):
        ts = time.time()
        self.command('start_wifi_scan')
        while time.time() - ts < timeout:
            time.sleep(0.5)
            self.pull()
            if self.esp.status.get('wifi_scan_ts', 0) > ts:
                return self.esp.status['wifi_scan']
        return None

    def log(self):
        # The server keeps the recent entries: show the ones not shown yet
        entries = self.esp.status.get('log', [])
        if self._last_log_ms is not None:
            entries = [e for e in entries if e['ms'] is None or e['ms'] > self._last_log_ms]
        ms = [e['ms'] for e in entries if e['ms'] is not None]
        if ms:
            self._last_log_ms = max(ms)
        return entries

    def params_credentials(self):
        from stretch4_body.subsystem.esp32 import params_credentials
        return params_credentials(self.esp.robot_params)


# ############################# Formatting #############################

def header(title):
    click.secho('\n%s %s' % (title, '-' * max(4, 60 - len(title))), fg='cyan', bold=True)


def row(label, value, fg=None):
    click.echo('  %s %s' % (('%s' % label).ljust(LABEL_WIDTH), click.style(str(value), fg=fg)))


def yes_no(b, good=True):
    """YES/NO, green when it is the good value."""
    return click.style('YES' if b else 'NO', fg='green' if bool(b) == good else 'yellow')


def on_off(b):
    return click.style('ON' if b else 'OFF', fg='green' if b else 'white')


def state_color(name, good, bad=()):
    if name in good:
        return 'green'
    if name in bad:
        return 'red'
    return 'yellow'


def age(s):
    if s is None:
        return 'never'
    if s < 120:
        return '%.1f s ago' % s
    return '%.0f min ago' % (s / 60.0)


def duration(s):
    s = int(s)
    h, rem = divmod(s, 3600)
    m, sec = divmod(rem, 60)
    return '%dh %02dm %02ds' % (h, m, sec) if h else ('%dm %02ds' % (m, sec) if m else '%ds' % sec)


def rssi_text(rssi, width=0):
    if rssi >= -55:
        q, fg = 'excellent', 'green'
    elif rssi >= -67:
        q, fg = 'good', 'green'
    elif rssi >= -75:
        q, fg = 'fair', 'yellow'
    else:
        q, fg = 'weak', 'red'
    return click.style(('%d dBm (%s)' % (rssi, q)).ljust(width), fg=fg)


def mask(s):
    if not s:
        return '(empty)'
    return s[:2] + '*' * max(len(s) - 2, 4)


def print_system(s):
    header('ESP32')
    bi = s.get('board_info') or {}
    row('Connection', yes_no(s.get('hw_valid')) + '  (%s)' % backend.name)
    row('Firmware', bi.get('firmware_version'))
    row('Board', bi.get('board_variant'))
    if not s.get('hw_valid'):
        return False
    row('Power state', s['pwr_state_name'], fg=state_color(s['pwr_state_name'], ('ACTIVE',)))
    row('Last reset', s['reset_reason_name'],
        fg=state_color(s['reset_reason_name'], ('POWERON', 'SW', 'EXT', 'USB'), ('PANIC', 'INT_WDT', 'TASK_WDT', 'WDT',
                                                                             'BROWNOUT', 'CPU_LOCKUP')))
    row('CPU', '%d MHz' % s['cpu_freq_mhz'])
    row('Free heap', '%d bytes (lowest %d)' % (s['free_heap'], s['min_free_heap']))
    row('Longest pass (1 s)', 'loop %d us, power task %d us' % (s['loop_time_us'], s['power_task_time_us']))
    row('Log', '%d waiting, %d dropped' % (s['log_pending'], s['log_dropped']))
    return True


def print_power(s):
    header('Power')
    row('12V0 rail', '%.2f V' % s['voltage_12v0'])
    row('Aux 20V0 rail', '%.2f V' % s['voltage_20v0_aux'])
    row('Adapter connected', yes_no(s['adapter_connected']))
    row('Adapter fault', yes_no(s['adapter_fault'], good=False))
    row('NUC USB 5V', on_off(s['cpu_5v_usb_on']))
    o = s['outputs']
    row('12V0 / Aux 20V0 / Lidar', '%s / %s / %s' % (on_off(o['en_12v0']), on_off(o['en_aux_20v0']),
                                                    on_off(o['en_lidar'])))
    row('Status LEDs / DCM mode', '%s / %s' % (on_off(o['en_status_leds']), on_off(o['dcm_mode'])))
    row('Robot active', on_off(o['robot_active']))
    row('NUC shutdown line', '%s (pulse %d of 2)' % (on_off(o['cpu_sd']), o['sd_pulse_count']))
    row('System shutdown line', on_off(o['system_shutdown']))

    header('Pimu link (UART)')
    p = s['pimu_link']
    fresh = 'live' if p['status_fresh'] else 'STALE'
    row('Pimu status', '%s, %s' % (click.style(fresh, fg='green' if p['status_fresh'] else 'red'),
                                   'never received' if p['status_age_ms'] is None else '%d ms old' % p['status_age_ms']))
    row('Battery', '%.2f V, %d%%' % (p['voltage_battery'], p['battery_soc']))
    row('Charging', yes_no(p['charger_charging']))
    row('Runstop', yes_no(p['runstop_event'], good=False))
    row('NUC', p['cpu_power_state_name'])
    row('NUC rail cut', yes_no(p['nuc_rail_off'], good=False))
    row('Board temperature', '%.1f C' % p['temperature'])
    errors = p['uart_crc_errors'] + p['uart_rx_overflows'] + p['uart_tx_drops']
    row('UART', 'rx %d, tx %d | CRC errors %d, overflows %d, tx drops %d'
        % (p['uart_rx_packets'], p['uart_tx_packets'], p['uart_crc_errors'], p['uart_rx_overflows'],
           p['uart_tx_drops']), fg='yellow' if errors else None)


def print_wifi(s):
    header('Wi-Fi')
    w = s['wifi']
    row('State', w['state_name'], fg=state_color(w['state_name'], ('CONNECTED',), ('NOT_CONFIGURED',)))
    row('Enabled', yes_no(w['enabled']))
    row('Driver status', w['wl_status_name'])
    if w['state_name'] == 'CONNECTED':
        row('Network', '"%s"' % w['ssid'])
        row('Signal', rssi_text(w['rssi']))
        row('Channel', w['channel'])
        row('Access point', w['bssid'])
        row('IP address', w['ip'])
        row('Gateway / netmask', '%s / %s' % (w['gateway'], w['netmask']))
        row('DNS', w['dns'])
        row('Connected for', duration(w['connected_s']))
    row('MAC address', w['mac'])
    row('Hostname', w['hostname'])
    row('Connect attempts', w['connect_attempts'])
    row('Disconnects', '%d (last reason: %s)' % (w['disconnects'], w['last_disconnect_reason_name'] or '-'))
    row('Low power (modem sleep)', yes_no(w['low_power'], good=False))


def print_firebase(s):
    header('Firebase')
    f = s['firebase']
    row('State', f['state_name'], fg=state_color(f['state_name'], ('READY',), ('NOT_CONFIGURED',)))
    row('Enabled', yes_no(f['enabled']))
    row('Signed in / DB ready', '%s / %s' % (yes_no(f['app_ready']), yes_no(f['db_setup'])))
    row('Publish period', '%.1f s' % f['publish_period_s'])
    row('Last publish', age(f['last_publish_age_s']))
    row('Publishes', '%d ok, %d errors' % (f['publish_ok'], f['publish_errors']),
        fg='yellow' if f['publish_errors'] else None)
    row('Last error', '%d %s' % (f['last_error_code'], f['last_error']) if f['last_error'] else '-')
    trigger = {0: 'none', 1: 'sleep', 2: 'wake'}.get(f['last_power_trigger'], str(f['last_power_trigger']))
    row('Last power request', '%s (%s)' % (trigger, age(f['last_trigger_age_s'])))
    row('Stream events', f['stream_events'])
    row('Initializations', f['init_count'])


def print_credentials(unmask):
    wifi, fb = backend.credentials()

    def secret(d, key):
        if key in d:
            return d[key] if unmask else mask(d[key])
        return '(not published by the server: %s)' % ('set' if d.get(key + '_set') else 'empty')

    header('Wi-Fi credentials')
    if wifi is None:
        row('', 'unavailable', fg='yellow')
    else:
        row('Configured', yes_no(wifi.get('configured')))
        row('SSID', '"%s"' % wifi.get('ssid', ''))
        row('Password', secret(wifi, 'password'))
    header('Firebase credentials')
    if fb is None:
        row('', 'unavailable', fg='yellow')
    else:
        row('Configured', yes_no(fb.get('configured')))
        row('URL', fb.get('url', ''))
        row('User email', fb.get('user_email', ''))
        row('API key', secret(fb, 'api_key'))
        row('User password', secret(fb, 'user_password'))
    if backend.name == 'server' and wifi is not None and 'password' not in wifi:
        click.secho('  Secrets are shown with -d (server stopped), or with esp32.publish_credentials: 1.', fg='white')


def print_status(sections=('system', 'power', 'wifi', 'firebase')):
    backend.pull()
    s = backend.snapshot()
    if 'system' in sections and not print_system(s):
        return
    if not s.get('hw_valid'):
        click.secho('No ESP32 status.', fg='yellow')
        return
    if 'power' in sections:
        print_power(s)
    if 'wifi' in sections:
        print_wifi(s)
    if 'firebase' in sections:
        print_firebase(s)


def print_log():
    backend.pull()
    header('ESP32 log')
    entries = backend.log()
    if not entries:
        click.echo('  (nothing new)')
    colors = {'WARN': 'yellow', 'ERROR': 'red', 'IDF': 'magenta', 'DROPPED': 'red', 'DEBUG': 'white'}
    for e in entries:
        t = '%10.3f' % (e['ms'] / 1000.0) if e['ms'] is not None else ' ' * 10
        click.echo('  %s %s %s' % (t, click.style(e['level_name'].ljust(7), fg=colors.get(e['level_name'])), e['text']))


def print_scan():
    click.echo('Scanning (a few seconds)...')
    scan = backend.start_scan_and_wait()
    header('Wi-Fi networks')
    if scan is None or scan['state_name'] != 'DONE':
        click.secho('  Scan %s' % ('timed out' if scan is None else scan['state_name'].lower()), fg='yellow')
        return
    click.secho('  %-33s %-18s %4s  %s' % ('SSID', 'SIGNAL', 'CH', 'SECURITY'), bold=True)
    for net in scan['networks']:
        click.echo('  %-33s %s %4d  %s' % (net['ssid'] or '(hidden)', rssi_text(net['rssi'], 18), net['channel'],
                                          net['auth']))
    if not scan['networks']:
        click.echo('  (none found)')


# ############################# Input #############################

def ask(prompt, default=None, secret=False):
    shown = '' if default in (None, '') else ' [%s]' % (mask(default) if secret else default)
    x = (getpass.getpass if secret else input)('  %s%s: ' % (prompt, shown))
    return x if len(x) else (default if default is not None else '')


def ask_int(prompt, default):
    """An integer, or None if the input isn't one."""
    x = ask(prompt, str(default))
    try:
        return int(x)
    except ValueError:
        click.secho('  Not a number: %s' % x, fg='red')
        return None


def confirm(msg):
    return input('  %s [y/N]: ' % msg).strip().lower() in ('y', 'yes')


def confirm_yes(msg):
    return input('  %s [Y/n]: ' % msg).strip().lower() not in ('n', 'no')


def after_command(ok, what):
    if ok:
        click.secho('  %s: sent' % what, fg='green')
        time.sleep(backend.settle_s)
    else:
        click.secho('  %s: failed' % what, fg='red')


def set_wifi_credentials():
    current, _ = backend.credentials()
    current = current or {}
    params, _ = backend.params_credentials()
    ssid = ask('SSID', current.get('ssid') or params['ssid'])
    known_pw = current.get('password', params['password'] if params['ssid'] == ssid else None)
    password = ask('Password (empty for an open network)', known_pw, secret=True)
    if not password and not confirm('No password: is "%s" an open network?' % ssid):
        return
    try:
        Esp32Base.validate_wifi_config(ssid, password)
    except ValueError as e:
        click.secho('  %s' % e, fg='red')
        return
    persist = confirm_yes('Save to the ESP32 flash (survives a reboot)?')
    after_command(backend.command('set_wifi_config', ssid, password, persist=persist, apply_now=True),
                  'Wi-Fi credentials')


def set_firebase_credentials():
    _, current = backend.credentials()
    current = current or {}
    _, params = backend.params_credentials()
    url = ask('Database URL', current.get('url') or params['url'])
    email = ask('User email', current.get('user_email') or params['user_email'])
    api_key = ask('API key', current.get('api_key') or params['api_key'], secret=True)
    password = ask('User password', current.get('user_password') or params['user_password'], secret=True)
    if not all((url, email, api_key, password)):
        click.secho('  All four are needed (use "forget" to clear Firebase).', fg='red')
        return
    try:
        Esp32Base.validate_firebase_config(url, api_key, email, password)
    except ValueError as e:
        click.secho('  %s' % e, fg='red')
        return
    persist = confirm_yes('Save to the ESP32 flash (survives a reboot)?')
    after_command(backend.command('set_firebase_config', url, api_key, email, password, persist=persist,
                                  apply_now=True), 'Firebase credentials')


def send_params_credentials():
    wifi, fb = backend.params_credentials()
    header('Credentials in the robot parameters')
    row('Wi-Fi SSID', wifi['ssid'] or '(not set)')
    row('Firebase URL', fb['url'] or '(not set)')
    if not wifi['ssid'] and not all(fb.values()):
        click.secho('  Nothing to send: set esp32.wifi / esp32.firebase in stretch_configuration_params.yaml',
                    fg='yellow')
        return
    if confirm('Send these to the ESP32 and save them to its flash?'):
        after_command(backend.command('send_credentials_from_params'), 'Credentials from params')


# ############################# Menus #############################

def wifi_menu():
    while True:
        click.secho('\n--- Wi-Fi ---', bold=True)
        click.echo('  s: status            c: credentials (masked)   C: credentials (unmasked)\n'
                   '  w: set credentials   n: scan networks\n'
                   '  r: reconnect         d: disconnect\n'
                   '  e: enable            x: disable                t: retry period\n'
                   '  F: forget (erase the credentials from the ESP32 flash)\n'
                   '  b: back')
        k = input('> ').strip()
        if k in ('b', 'q', ''):
            return
        if k == 's':
            print_status(('wifi',))
        elif k == 'c':
            print_credentials(unmask=False)
        elif k == 'C':
            print_credentials(unmask=True)
        elif k == 'w':
            set_wifi_credentials()
        elif k == 'n':
            print_scan()
        elif k == 'r':
            after_command(backend.command('trigger_wifi_reconnect'), 'Reconnect')
        elif k == 'd':
            after_command(backend.command('trigger_wifi_disconnect'), 'Disconnect')
        elif k == 'e':
            after_command(backend.command('set_config', wifi_enable=1), 'Wi-Fi enable')
        elif k == 'x':
            if confirm('Disable Wi-Fi (until enabled, or the ESP32 gets its config at the next startup)?'):
                after_command(backend.command('set_config', wifi_enable=0), 'Wi-Fi disable')
        elif k == 't':
            ms = ask_int('Retry period while disconnected (ms, 1000 or more)', 5000)
            if ms is not None:
                after_command(backend.command('set_config', wifi_retry_ms=ms), 'Retry period')
        elif k == 'F':
            if confirm('Erase the Wi-Fi credentials from the ESP32 and disconnect?'):
                after_command(backend.command('forget_wifi'), 'Forget Wi-Fi')
        else:
            click.echo('Unknown choice')


def firebase_menu():
    while True:
        click.secho('\n--- Firebase ---', bold=True)
        click.echo('  s: status            c: credentials (masked)   C: credentials (unmasked)\n'
                   '  f: set credentials   r: restart\n'
                   '  e: enable            x: disable                t: publish period\n'
                   '  F: forget (erase the credentials from the ESP32 flash)\n'
                   '  b: back')
        k = input('> ').strip()
        if k in ('b', 'q', ''):
            return
        if k == 's':
            print_status(('firebase',))
        elif k == 'c':
            print_credentials(unmask=False)
        elif k == 'C':
            print_credentials(unmask=True)
        elif k == 'f':
            set_firebase_credentials()
        elif k == 'r':
            after_command(backend.command('trigger_firebase_restart'), 'Restart')
        elif k == 'e':
            after_command(backend.command('set_config', firebase_enable=1), 'Firebase enable')
        elif k == 'x':
            if confirm('Disable Firebase (until enabled, or the ESP32 gets its config at the next startup)?'):
                after_command(backend.command('set_config', firebase_enable=0), 'Firebase disable')
        elif k == 't':
            ms = ask_int('Publish period (ms, 1000 or more)', 5000)
            if ms is not None:
                after_command(backend.command('set_config', firebase_publish_period_ms=ms), 'Publish period')
        elif k == 'F':
            if confirm('Erase the Firebase credentials from the ESP32 and stop Firebase?'):
                after_command(backend.command('forget_firebase'), 'Forget Firebase')
        else:
            click.echo('Unknown choice')


def main_menu():
    while True:
        click.secho('\n------ ESP32 (%s) ------' % backend.name, bold=True)
        click.echo('  s: full status       p: power and Pimu link\n'
                   '  w: Wi-Fi ...         f: Firebase ...\n'
                   '  c: credentials (masked)\n'
                   '  P: send the credentials in the robot params\n'
                   '  l: log               v: log level\n'
                   '  R: reboot the ESP32\n'
                   '  q: quit')
        k = input('> ').strip()
        if k == 'q':
            return
        if k in ('s', ''):
            print_status()
        elif k == 'p':
            print_status(('power',))
        elif k == 'w':
            wifi_menu()
        elif k == 'f':
            firebase_menu()
        elif k == 'c':
            print_credentials(unmask=False)
        elif k == 'P':
            send_params_credentials()
        elif k == 'l':
            print_log()
        elif k == 'v':
            level = ask_int('Log level (0 DEBUG, 1 INFO, 2 WARN, 3 ERROR)', 1)
            if level is not None and 0 <= level <= 3:
                after_command(backend.command('set_config', log_level=level), 'Log level')
            elif level is not None:
                click.secho('  The log level is 0 to 3', fg='red')
        elif k == 'R':
            if confirm('Reboot the ESP32? Its power outputs reset while it boots.'):
                after_command(backend.command('reboot'), 'Reboot')
        else:
            click.echo('Unknown choice')


# ############################# Main #############################

backend = DirectBackend() if args.direct else ServerBackend()
if not backend.startup():
    sys.exit(1)

try:
    one_shot = [args.status, args.wifi, args.firebase, args.config, args.show_credentials, args.scan, args.log]
    if any(one_shot):
        if args.status:
            print_status()
        if args.wifi:
            print_status(('wifi',))
        if args.firebase:
            print_status(('firebase',))
        if args.config or args.show_credentials:
            print_credentials(unmask=args.show_credentials)
        if args.scan:
            print_scan()
        if args.log:
            print_log()
    else:
        print_status()
        main_menu()
except (KeyboardInterrupt, EOFError):
    click.echo('')
finally:
    backend.stop()
