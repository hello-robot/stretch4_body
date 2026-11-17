#!/usr/bin/env python3
"""
ESP32 status, and set-up of its Wi-Fi and Firebase.

Prints the status, then offers a menu:
  w  configure Wi-Fi: pick a network and connect, as in a desktop Wi-Fi menu
  f  configure Firebase
  s  print the status
  q  quit

By default this goes through the robot server (its 'esp32' subsystem). With -d it
opens /dev/hello-esp32 itself, which needs the server stopped. It talks only to
the ESP32.

Settings that work are saved to the ESP32's flash and to the robot's
stretch_configuration_params.yaml (esp32.wifi, esp32.firebase).

Needs ESP32 firmware hello-esp.v0.2.0p1 or later (earlier firmware has no RPC).
"""
import argparse
import getpass
import subprocess
import sys
import time

import click

import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.esp32 import Esp32Base

hu.print_stretch_re_use()

parser = argparse.ArgumentParser(description='ESP32 status, and set-up of its Wi-Fi and Firebase')
parser.add_argument('-d', '--direct', help='Use the direct API on /dev/hello-esp32 (server must be stopped)',
                    action='store_true')
args = parser.parse_args()

LABEL_WIDTH = 24
WIFI_CONNECT_TIMEOUT_S = 30.0
FIREBASE_READY_TIMEOUT_S = 45.0


# ############################# Backends #############################

class DirectBackend:
    """The ESP32 through the direct API (stretch4_body.subsystem.esp32.Esp32)."""
    name = 'direct'

    def __init__(self):
        from stretch4_body.subsystem.esp32 import Esp32
        self.esp = Esp32()
        self.device = self.esp

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
        """(wifi, firebase) as the ESP32 has them, secrets included. None where unreadable."""
        return self.esp.get_wifi_config(), self.esp.get_firebase_config()

    def command(self, method, *a, **kw):
        getattr(self.esp, method)(*a, **kw)
        ok = self.esp.push_command()
        time.sleep(0.3)  # Let the ESP32 act on it before the next status read
        return ok

    def scan(self, timeout=20.0):
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


class ServerBackend:
    """The ESP32 through the robot server's esp32 subsystem (Esp32Client)."""
    name = 'server'

    def __init__(self):
        from stretch4_body.robot.robot_client import Esp32Client
        self.esp = Esp32Client()
        self.device = self.esp

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
        return True

    def stop(self):
        self.esp.stop()

    def pull(self):
        self.esp.pull_status()

    def snapshot(self):
        return self.esp.status

    def credentials(self):
        """(wifi, firebase) as the server publishes them: secrets only if esp32.publish_credentials is 1."""
        return self.esp.status.get('wifi_config'), self.esp.status.get('firebase_config')

    def command(self, method, *a, **kw):
        getattr(self.esp, method)(*a, **kw)
        # ESP32 settings don't move the robot: don't take control from whoever has it
        return self.esp.push_command(ignore_control_lock=True)

    def scan(self, timeout=25.0):
        ts = time.time()
        self.command('start_wifi_scan')
        while time.time() - ts < timeout:
            time.sleep(0.5)
            self.pull()
            if self.esp.status.get('wifi_scan_ts', 0) > ts:
                return self.esp.status['wifi_scan']
        return None


# ############################# Robot configuration #############################

def saved_credentials():
    """(wifi, firebase) saved in the robot configuration (esp32.*, or the older power-periph.firebase)."""
    from stretch4_body.subsystem.esp32 import params_credentials
    return params_credentials(backend.device.robot_params)


def save_to_robot_config(section, values):
    """Write esp32.<section>.<key> to stretch_configuration_params.yaml."""
    try:
        for k, v in values.items():
            backend.device.write_configuration_param_to_YAML('esp32.%s.%s' % (section, k), v, force_creation=True)
        # And in memory, so this session sees it as saved
        backend.device.robot_params.setdefault('esp32', {}).setdefault(section, {}).update(values)
        click.secho('  Saved to stretch_configuration_params.yaml (esp32.%s)' % section, fg='green')
    except Exception as e:
        click.secho('  Could not save to stretch_configuration_params.yaml: %s' % e, fg='red')


def robot_computer_wifi():
    """
    The Wi-Fi network the robot's computer is on, from NetworkManager:
    {'ssid', 'password' (None if unreadable), 'key_mgmt'}, or None if it isn't on Wi-Fi.
    """
    def nmcli(*a):
        r = subprocess.run(['nmcli'] + list(a), capture_output=True, text=True, timeout=5)
        return r.stdout.strip() if r.returncode == 0 else ''

    def unescape(s):
        return s.replace('\\:', ':').replace('\\\\', '\\')

    try:
        for line in nmcli('-t', '-f', 'NAME,TYPE', 'connection', 'show', '--active').splitlines():
            name, _, ctype = line.rpartition(':')
            if ctype != '802-11-wireless':
                continue
            name = unescape(name)
            ssid = unescape(nmcli('-g', '802-11-wireless.ssid', 'connection', 'show', name))
            key_mgmt = nmcli('-g', '802-11-wireless-security.key-mgmt', 'connection', 'show', name)
            password = unescape(nmcli('-s', '-g', '802-11-wireless-security.psk', 'connection', 'show', name))
            return {'ssid': ssid, 'password': password or None, 'key_mgmt': key_mgmt}
    except (OSError, subprocess.TimeoutExpired):
        pass
    return None


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


def rssi_quality(rssi):
    if rssi >= -55:
        return 'excellent', 'green', 4
    if rssi >= -67:
        return 'good', 'green', 3
    if rssi >= -75:
        return 'fair', 'yellow', 2
    return 'weak', 'red', 1


def rssi_text(rssi):
    q, fg, _ = rssi_quality(rssi)
    return click.style('%d dBm (%s)' % (rssi, q), fg=fg)


def signal_bars(rssi):
    _, fg, n = rssi_quality(rssi)
    return click.style('▂▄▆█'[:n].ljust(4), fg=fg)


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


def print_status():
    backend.pull()
    s = backend.snapshot()
    if not print_system(s):
        click.secho('No ESP32 status.', fg='yellow')
        return
    print_power(s)
    print_wifi(s)
    print_firebase(s)


# ############################# Input #############################

def ask(prompt, default=None, secret=False):
    shown = '' if default in (None, '') else ' [%s]' % (mask(default) if secret else default)
    x = (getpass.getpass if secret else input)('  %s%s: ' % (prompt, shown))
    return x if len(x) else (default if default is not None else '')


def confirm(msg, default=False):
    x = input('  %s %s: ' % (msg, '[Y/n]' if default else '[y/N]')).strip().lower()
    return default if not x else x in ('y', 'yes')


def require_esp32():
    backend.pull()
    if not backend.snapshot().get('hw_valid'):
        click.secho('The ESP32 is not connected (%s).' % backend.name, fg='red')
        return False
    return True


# ############################# Wi-Fi wizard #############################

# What a failed connection most likely means
WIFI_FAILURE_HINTS = {
    'AUTH_FAIL': 'wrong password?',
    '4WAY_HANDSHAKE_TIMEOUT': 'wrong password?',
    'HANDSHAKE_TIMEOUT': 'wrong password?',
    'NO_AP_FOUND': 'network not in range, or 5 GHz only (the ESP32 is 2.4 GHz only)',
    'NO_AP_FOUND_W_COMPATIBLE_SECURITY': 'the network uses security the ESP32 does not support',
    'ASSOC_FAIL': 'the access point refused the ESP32',
    'BEACON_TIMEOUT': 'signal too weak',
}


def print_wifi_connection():
    backend.pull()
    w = backend.snapshot()['wifi']
    header('Current connection')
    if w['state_name'] == 'CONNECTED':
        row('Network', '"%s"' % w['ssid'], fg='green')
        row('Signal', rssi_text(w['rssi']))
        row('IP address', w['ip'])
        row('Connected for', duration(w['connected_s']))
    else:
        row('State', w['state_name'], fg=state_color(w['state_name'], (), ('NOT_CONFIGURED',)))
        wifi, _ = backend.credentials()
        if wifi and wifi.get('ssid'):
            row('Trying', '"%s" (%d attempts, last reason %s)'
                % (wifi['ssid'], w['connect_attempts'], w['last_disconnect_reason_name'] or '-'))
    return w


def list_networks(scan, current_ssid, robot_ssid, saved_ssid):
    """Print the scan as a numbered list. Returns the networks, one per SSID, strongest first."""
    networks, seen = [], set()
    for net in scan['networks']:
        if net['ssid'] and net['ssid'] not in seen:
            seen.add(net['ssid'])
            networks.append(net)
    header('Available networks')
    if not networks:
        click.echo('  (none found)')
    for i, net in enumerate(networks):
        tags = []
        if net['ssid'] == current_ssid:
            tags.append(click.style('connected', fg='green'))
        if net['ssid'] == robot_ssid:
            tags.append('robot computer')
        if net['ssid'] == saved_ssid:
            tags.append('saved')
        security = 'open' if net['auth'] == 'OPEN' else net['auth']
        click.echo('  %2d  %s %-32s %-14s ch %-3d %s' % (i + 1, signal_bars(net['rssi']), net['ssid'], security,
                                                       net['channel'], ', '.join(tags)))
    return networks


def wifi_attempts():
    backend.pull()
    return backend.snapshot()['wifi']['connect_attempts']


def wait_for_wifi(ssid, attempts0, ts):
    """
    Wait for the ESP32 to join ssid, after a command sent at ts when it had made attempts0
    connection attempts. Returns (connected, failure reason name).
    """
    click.echo('  Connecting to "%s" ' % ssid, nl=False)
    while time.time() - ts < WIFI_CONNECT_TIMEOUT_S:
        time.sleep(1.0)
        click.echo('.', nl=False)
        backend.pull()
        w = backend.snapshot()['wifi']
        # A connection made since the command: not the one it had before
        if w['state_name'] == 'CONNECTED' and w['ssid'] == ssid and w['connected_s'] < time.time() - ts + 0.5:
            click.echo('')
            return True, None
        # The ESP32 retries every few seconds: three failed tries is an answer
        if w['connect_attempts'] - attempts0 >= 3:
            break
    click.echo('')
    return False, backend.snapshot()['wifi']['last_disconnect_reason_name'] or None


def stop_trying(ssid):
    """Stop the ESP32 retrying a network that failed, so it is free to scan (firmware v0.2.1p1 or later)."""
    backend.command('trigger_wifi_disconnect')
    click.secho('  Stopped trying "%s". The ESP32 goes back to its saved network at its next reboot, or pick '
                'one here.' % ssid, fg='yellow')


def connect_wifi(ssid, password, previous):
    """
    Join the network. On success, save it to the ESP32 flash and the robot configuration. On failure,
    put the ESP32 back as it was: on the previous network if that can be rejoined, with no network if
    it had none ({'ssid': ''}), else stopped. Returns True on success.
    """
    try:
        Esp32Base.validate_wifi_config(ssid, password)
    except ValueError as e:
        click.secho('  %s' % e, fg='red')
        return False
    # Try it first without saving, so a wrong password doesn't stick after a reboot
    attempts0, ts = wifi_attempts(), time.time()
    if not backend.command('set_wifi_config', ssid, password, persist=False, apply_now=True):
        click.secho('  Could not send the network to the ESP32', fg='red')
        return False
    ok, reason = wait_for_wifi(ssid, attempts0, ts)
    if ok:
        backend.command('set_wifi_config', ssid, password, persist=True, apply_now=False)
        w = backend.snapshot()['wifi']
        click.secho('  Connected to "%s": IP %s, signal %s' % (ssid, w['ip'], rssi_text(w['rssi'])), fg='green')
        save_to_robot_config('wifi', {'ssid': ssid, 'password': password})
        return True
    hint = WIFI_FAILURE_HINTS.get(reason, '')
    click.secho('  Could not connect to "%s"%s%s' % (ssid, ': %s' % reason if reason else '',
                                                       ' (%s)' % hint if hint else ''), fg='red')
    if previous is not None and previous['ssid'] == '':
        # It had no network: clear the one that failed, rather than leave it retrying
        backend.command('forget_wifi')
        click.secho('  The ESP32 is back to no network.', fg='yellow')
    elif previous and previous['ssid'] != ssid and previous.get('password') is not None and \
            confirm('Go back to "%s"?' % previous['ssid'], default=True):
        attempts0, ts = wifi_attempts(), time.time()
        backend.command('set_wifi_config', previous['ssid'], previous['password'], persist=False,
                        apply_now=True)
        if wait_for_wifi(previous['ssid'], attempts0, ts)[0]:
            click.secho('  Back on "%s"' % previous['ssid'], fg='green')
        else:
            click.secho('  Could not get back to "%s" either.' % previous['ssid'], fg='red')
            stop_trying(previous['ssid'])
    else:
        stop_trying(ssid)
    return False


def similar_networks(ssid, visible):
    """
    Visible networks that may be the 2.4 GHz side of ssid (e.g. "Home" and "Home 2.4"), likeliest first.
    Compares names without case, spaces or punctuation.
    """
    def norm(s):
        return ''.join(c for c in s.lower() if c.isalnum())
    base = norm(ssid)
    if not base:
        return []
    found = [v for v in visible if v != ssid and base in norm(v)]
    marked = lambda v: any(m in norm(v)[len(base):] for m in ('24', '2g', '24g', '24ghz'))
    return sorted(found, key=lambda v: (not marked(v), len(v)))


def password_for(ssid, security, known):
    """Ask for the password of ssid, offering a known one. None means cancel."""
    if security == 'OPEN':
        return '' if confirm('"%s" is an open network (no password). Connect?' % ssid, default=True) else None
    if known:
        click.echo('  A saved password for "%s" is available.' % ssid)
        x = getpass.getpass('  Password (Enter to use the saved one): ')
        return x if x else known
    x = getpass.getpass('  Password for "%s": ' % ssid)
    return x if x else None


def config_wifi():
    if not require_esp32():
        return
    click.secho('ESP32 Wi-Fi set-up. The ESP32 is 2.4 GHz only.', bold=True)
    while True:
        w = print_wifi_connection()
        current_ssid = w['ssid'] if w['state_name'] == 'CONNECTED' else None
        esp_wifi, _ = backend.credentials()
        saved_wifi, _ = saved_credentials()
        robot = robot_computer_wifi()

        # Passwords we know, by SSID, to offer as defaults
        known = {}
        if saved_wifi['ssid'] and saved_wifi['password']:
            known[saved_wifi['ssid']] = saved_wifi['password']
        if robot and robot['ssid'] and robot['password']:
            known.setdefault(robot['ssid'], robot['password'])
        if esp_wifi and esp_wifi.get('ssid') and esp_wifi.get('password'):
            known[esp_wifi['ssid']] = esp_wifi['password']
        # What to put back on failure. {'ssid': ''}: the ESP32 had no network. None: unknown
        previous = None
        if esp_wifi is not None:
            previous = {'ssid': esp_wifi.get('ssid') or '', 'password': known.get(esp_wifi.get('ssid'))}

        click.echo('\n  Scanning...')
        scan = backend.scan()
        if scan is None or scan['state_name'] != 'DONE':
            click.secho('  Scan %s' % ('timed out' if scan is None else scan['state_name'].lower()), fg='yellow')
            if backend.snapshot()['wifi']['state_name'] == 'CONNECTING':
                click.secho('  The ESP32 cannot scan while it keeps trying to join a network. d stops it, then r '
                            'scans again. (Firmware hello-esp.v0.2.1p1 or later stops it by itself.)', fg='yellow')
            networks = []
        else:
            networks = list_networks(scan, current_ssid, robot['ssid'] if robot else None, saved_wifi['ssid'])

        click.echo('')
        if networks:
            click.echo('  1-%d: connect to that network' % len(networks))
        if robot and robot['ssid']:
            click.echo('  i:   use the robot computer\'s network ("%s")' % robot['ssid'])
        if saved_wifi['ssid']:
            click.echo('  s:   use the saved network ("%s")' % saved_wifi['ssid'])
        click.echo('  h:   hidden network (type its name)\n'
                   '  r:   scan again\n'
                   '  d:   disconnect\n'
                   '  f:   forget the network saved on the ESP32\n'
                   '  q:   back to the main menu')
        k = input('> ').strip()

        if k == 'q' or k == '':
            return
        if k == 'r':
            continue
        if k.isdigit() and 1 <= int(k) <= len(networks):
            net = networks[int(k) - 1]
            password = password_for(net['ssid'], net['auth'], known.get(net['ssid']))
            if password is not None and connect_wifi(net['ssid'], password, previous):
                return
        elif k == 'i' and robot and robot['ssid']:
            if 'eap' in (robot['key_mgmt'] or ''):
                click.secho('  The robot computer is on an enterprise (802.1X) network. The ESP32 cannot join it.',
                            fg='red')
                continue
            target = robot['ssid']
            visible = [n['ssid'] for n in networks]
            if networks and target not in visible:
                click.secho('  The ESP32 does not see "%s": it is probably 5 GHz only.' % target, fg='yellow')
                similar = similar_networks(target, visible)
                if len(similar) == 1:
                    if confirm('It sees "%s", which may be the same router\'s 2.4 GHz network. Try it with the '
                               'robot computer\'s password?' % similar[0], default=True):
                        target = similar[0]
                elif similar:
                    click.echo('  It sees these, which may be the same router\'s 2.4 GHz network:')
                    for i, v in enumerate(similar):
                        click.echo('    %d  %s' % (i + 1, v))
                    x = input('  Try which one with the robot computer\'s password (Enter for none)? ').strip()
                    if x.isdigit() and 1 <= int(x) <= len(similar):
                        target = similar[int(x) - 1]
                if target == robot['ssid'] and not confirm('Try "%s" anyway?' % target):
                    continue
            password = robot['password']
            if password is None:
                password = password_for(target, 'WPA2_PSK', None)
            if password is not None and connect_wifi(target, password, previous):
                return
        elif k == 's' and saved_wifi['ssid']:
            if connect_wifi(saved_wifi['ssid'], saved_wifi['password'] or '', previous):
                return
        elif k == 'h':
            ssid = ask('Network name (SSID)')
            if ssid:
                password = getpass.getpass('  Password (Enter for an open network): ')
                if connect_wifi(ssid, password, previous):
                    return
        elif k == 'd':
            backend.command('trigger_wifi_disconnect')
            click.echo('  Disconnected. The ESP32 stays off Wi-Fi until you pick a network here, or it reboots.')
            time.sleep(1.0)
        elif k == 'f':
            if confirm('Erase the network saved on the ESP32 and disconnect?'):
                backend.command('forget_wifi')
                if saved_wifi['ssid'] and confirm('Also remove it from the robot configuration?'):
                    save_to_robot_config('wifi', {'ssid': 'NA', 'password': 'NA'})
                time.sleep(1.0)
        else:
            click.echo('  Unknown choice')


# ############################# Firebase wizard #############################

def print_firebase_connection(esp_fb):
    backend.pull()
    s = backend.snapshot()
    f = s['firebase']
    header('Current Firebase connection')
    row('State', f['state_name'], fg=state_color(f['state_name'], ('READY',), ('NOT_CONFIGURED',)))
    if esp_fb:
        row('Database URL', esp_fb.get('url') or '-')
        row('User email', esp_fb.get('user_email') or '-')
    row('Last publish', age(f['last_publish_age_s']))
    if f['last_error']:
        row('Last error', '%d %s' % (f['last_error_code'], f['last_error']), fg='yellow')
    if s['wifi']['state_name'] != 'CONNECTED':
        click.secho('  Wi-Fi is not connected: Firebase needs it. Configure Wi-Fi (w in the main menu) first.', fg='yellow')
    return f


def firebase_inits():
    backend.pull()
    return backend.snapshot()['firebase']['init_count']


def wait_for_firebase(init0):
    """Wait for Firebase to be ready after a restart, sent when it had init0 initializations. Returns (ready, error)."""
    ts = time.time()
    click.echo('  Connecting to Firebase ', nl=False)
    while time.time() - ts < FIREBASE_READY_TIMEOUT_S:
        time.sleep(1.0)
        click.echo('.', nl=False)
        backend.pull()
        f = backend.snapshot()['firebase']
        if f['state_name'] == 'READY' and f['init_count'] > init0:
            click.echo('')
            return True, None
    click.echo('')
    f = backend.snapshot()['firebase']
    return False, ('%d %s' % (f['last_error_code'], f['last_error'])) if f['last_error'] else f['state_name']


def connect_firebase(cfg, previous):
    """Use cfg (url, api_key, user_email, user_password). Save it if Firebase comes up. Returns True on success."""
    try:
        Esp32Base.validate_firebase_config(cfg['url'], cfg['api_key'], cfg['user_email'], cfg['user_password'])
    except ValueError as e:
        click.secho('  %s' % e, fg='red')
        return False
    fields = (cfg['url'], cfg['api_key'], cfg['user_email'], cfg['user_password'])
    init0 = firebase_inits()
    if not backend.command('set_firebase_config', *fields, persist=False, apply_now=True):
        click.secho('  Could not send the settings to the ESP32', fg='red')
        return False
    ok, err = wait_for_firebase(init0)
    if ok:
        backend.command('set_firebase_config', *fields, persist=True, apply_now=False)
        click.secho('  Firebase ready', fg='green')
        save_to_robot_config('firebase', dict(cfg))
        return True
    click.secho('  Firebase did not come up: %s' % err, fg='red')
    if previous and all(previous.get(k) for k in ('url', 'api_key', 'user_email', 'user_password')):
        if confirm('Go back to the previous settings (%s)?' % previous['url'], default=True):
            init0 = firebase_inits()
            backend.command('set_firebase_config', previous['url'], previous['api_key'], previous['user_email'],
                            previous['user_password'], persist=False, apply_now=True)
            if wait_for_firebase(init0)[0]:
                click.secho('  Back on the previous settings', fg='green')
            else:
                click.secho('  The previous settings did not come up either. The ESP32 goes back to its saved '
                            'settings at its next reboot.', fg='red')
    else:
        click.secho('  The ESP32 goes back to its saved settings at its next reboot.', fg='yellow')
    return False


def merge_known(esp_fb, saved_fb):
    """Best known value of each field: the ESP32's (when readable), else the robot configuration's."""
    out = {}
    for k in ('url', 'api_key', 'user_email', 'user_password'):
        v = (esp_fb or {}).get(k)
        out[k] = v if v else saved_fb.get(k)
    return out


def config_firebase():
    if not require_esp32():
        return
    click.secho('ESP32 Firebase set-up.', bold=True)
    while True:
        _, esp_fb = backend.credentials()
        _, saved_fb = saved_credentials()
        print_firebase_connection(esp_fb)
        known = merge_known(esp_fb, saved_fb)
        # What to go back to on failure: the ESP32's settings, with secrets from the robot
        # configuration only if that is the same project and user
        previous = None
        if esp_fb and esp_fb.get('url'):
            same = saved_fb['url'] == esp_fb['url'] and saved_fb['user_email'] == esp_fb.get('user_email')
            if esp_fb.get('user_password') or same:
                previous = known

        saved_complete = all(saved_fb.values())
        click.echo('')
        if saved_complete:
            click.echo('  i: use the robot configuration\'s settings (%s, %s)' % (saved_fb['url'],
                                                                                 saved_fb['user_email']))
        click.echo('  e: enter the settings\n'
                   '  r: reconnect\n'
                   '  f: forget the settings saved on the ESP32\n'
                   '  q: back to the main menu')
        k = input('> ').strip()

        if k == 'q' or k == '':
            return
        if k == 'i' and saved_complete:
            if connect_firebase(dict(saved_fb), previous):
                return
        elif k == 'e':
            click.echo('  Press Enter to keep a value in [brackets].')
            cfg = {'url': ask('Database URL', known['url']),
                   'user_email': ask('User email', known['user_email']),
                   'api_key': ask('Web API key', known['api_key'], secret=True),
                   'user_password': ask('User password', known['user_password'], secret=True)}
            if not all(cfg.values()):
                click.secho('  All four are needed.', fg='red')
                continue
            if connect_firebase(cfg, previous):
                return
        elif k == 'r':
            init0 = firebase_inits()
            backend.command('trigger_firebase_restart')
            ok, err = wait_for_firebase(init0)
            click.secho('  Firebase ready' if ok else '  Firebase did not come up: %s' % err,
                        fg='green' if ok else 'red')
        elif k == 'f':
            if confirm('Erase the Firebase settings saved on the ESP32 and stop Firebase?'):
                backend.command('forget_firebase')
                if saved_complete and confirm('Also remove them from the robot configuration?'):
                    save_to_robot_config('firebase', {'url': 'NA', 'api_key': 'NA', 'user_email': 'NA',
                                                      'user_password': 'NA'})
                time.sleep(1.0)
        else:
            click.echo('  Unknown choice')


# ############################# Main #############################

backend = DirectBackend() if args.direct else ServerBackend()
if not backend.startup():
    sys.exit(1)


def main_menu():
    """Status, then the menu. Enter, or anything not on the menu, prints both again."""
    print_status()
    while True:
        click.secho('\n------ ESP32 (%s) ------' % backend.name, bold=True)
        click.echo('  w: configure Wi-Fi\n'
                   '  f: configure Firebase\n'
                   '  s: print status\n'
                   '  q: quit')
        k = input('> ').strip()
        if k == 'q':
            return
        if k == 'w':
            config_wifi()
        elif k == 'f':
            config_firebase()
        else:
            print_status()


try:
    main_menu()
except (KeyboardInterrupt, EOFError):
    click.echo('')
finally:
    backend.stop()
