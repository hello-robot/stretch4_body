#!/usr/bin/env python3
"""
ESP32 status, and set-up of its Wi-Fi and Firebase.

Prints the status, then offers a menu:
  w  configure Wi-Fi: pick a network and connect, as in a desktop Wi-Fi menu
  f  configure Firebase (remote sleep and wake): makes a robot key, and sets up the
     database under robots/$HELLO_FLEET_ID, with the Firebase CLI if it is installed
     (Esp32Firebase, in subsystem/esp32.py)
  s  print the status
  q  quit

It opens /dev/hello-esp32 directly; the ESP32 is not part of the robot server.
It talks only to the ESP32.

Settings that work are saved to the ESP32's flash, and only there: they are secrets, so
they are not written to the robot's configuration files.

Needs ESP32 firmware hello-esp.v0.2.0p1 or later (earlier firmware has no RPC), and
hello-esp.v0.2.6p1 or later for Firebase.
"""
import argparse
import getpass
import os
import subprocess
import sys
import time

import click

import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.esp32 import Esp32Base, Esp32Firebase, generate_robot_key

hu.print_stretch_re_use()

parser = argparse.ArgumentParser(description='ESP32 status, and set-up of its Wi-Fi and Firebase')
# The tool is always direct now; -d is still accepted so existing habits and scripts work
parser.add_argument('-d', '--direct', help=argparse.SUPPRESS, action='store_true')
args = parser.parse_args()

LABEL_WIDTH = 24
WIFI_CONNECT_TIMEOUT_S = 30.0
FIREBASE_READY_TIMEOUT_S = 45.0


# ############################# ESP32 access #############################

class Esp32Backend:
    """The ESP32 through the direct API (stretch4_body.subsystem.esp32.Esp32)."""

    def __init__(self):
        from stretch4_body.subsystem.esp32 import Esp32
        self.esp = Esp32()
        self.device = self.esp
        self.name = self.esp.transport.port_name

    def startup(self):
        if self.esp.startup():
            return True
        click.secho('Unable to talk to the ESP32 on %s.' % self.esp.transport.port_name, fg='red')
        click.secho('  Is another program using it (another stretch_esp32_jog, or the firmware updater)?\n'
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


# ############################# Robot computer #############################

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
    if p['battery_status'] == 'BMS_LOST':
        row('Battery', "BMS NOT RESPONDING: values can't be trusted", fg='red')
    elif p['battery_status'] == 'NOT_DETECTED':
        row('Battery', 'NOT DETECTED: running from the adapter', fg='yellow')
    else:
        row('Battery', '%.2f V, %d%%' % (p['voltage_battery'], p['battery_soc']))
    # charger_charging means the charger is running, which with no battery powers the robot
    if p['battery_status'] == 'NOT_DETECTED':
        row('Charging', 'NO: the charger powers the robot (no battery)', fg='yellow')
    elif p['battery_status'] == 'BMS_LOST' and p['charger_charging']:
        row('Charging', 'UNKNOWN: the charger runs, the BMS is not answering', fg='yellow')
    else:
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
    row('Command stream', '%d events, last %s, %d restarts'
        % (f['stream_events'], age(f['stream_age_s']), f['stream_restarts']),
        fg='yellow' if f['state_name'] == 'STREAM_CONNECTING' else None)
    row('Clock set (SNTP)', yes_no(f['time_synced']))
    if f['cmd_action'] or f['cmd_result']:
        row('Last power command', '%s "%s": %s (%s)'
            % (f['cmd_action_name'].lower(), f['cmd_id'], f['cmd_result_name'].lower(), age(f['cmd_age_s'])),
            fg=state_color(f['cmd_result_name'], ('DONE', 'PENDING'), ('FAILED', 'REJECTED', 'INVALID')))
    else:
        row('Last power command', 'none')
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


def list_networks(scan, current_ssid, robot_ssid):
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
    Join the network. On success, save it to the ESP32's flash. On failure,
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
        click.secho('  Connected to "%s": IP %s, signal %s. Saved on the ESP32.'
                    % (ssid, w['ip'], rssi_text(w['rssi'])), fg='green')
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
        click.echo('  The password for "%s" is known (from the ESP32 or the robot computer).' % ssid)
        x = getpass.getpass('  Password (Enter to use the known one): ')
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
        robot = robot_computer_wifi()

        # Passwords we know, by SSID, to offer as defaults
        known = {}
        if robot and robot['ssid'] and robot['password']:
            known[robot['ssid']] = robot['password']
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
            networks = list_networks(scan, current_ssid, robot['ssid'] if robot else None)

        click.echo('')
        if networks:
            click.echo('  1-%d: connect to that network' % len(networks))
        if robot and robot['ssid']:
            click.echo('  i:   use the robot computer\'s network ("%s")' % robot['ssid'])
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
                time.sleep(1.0)
        else:
            click.echo('  Unknown choice')


# ############################# Firebase wizard #############################

FIREBASE_FIELDS = ('url', 'fleet_id', 'robot_key', 'api_key', 'user_email', 'user_password')
SIGN_IN_FIELDS = ('api_key', 'user_email', 'user_password')


def print_firebase_connection(esp_fb):
    backend.pull()
    s = backend.snapshot()
    f = s['firebase']
    header('Current Firebase connection')
    row('State', f['state_name'], fg=state_color(f['state_name'], ('READY',), ('NOT_CONFIGURED',)))
    if esp_fb:
        row('Database URL', esp_fb.get('url') or '-')
        row('Fleet ID', esp_fb.get('fleet_id') or '-')
        row('Robot key', esp_fb.get('robot_key') or '-')
        row('Signs in as', esp_fb.get('user_email') or 'no one (robot key only)')
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


def firebase_failure_hint(err):
    e = (err or '').lower()
    if '401' in e or 'unauthorized' in e or 'permission' in e:
        return ('the database rules refused the robot. Are the robot rules in force (p prints them), and does '
                'robots/<fleet ID>/robot_key hold this robot\'s key?')
    if any(w in e for w in ('credential', 'password', 'email', 'api key', 'api_key')):
        return 'the sign-in was refused: check the Web API key, the email and the password'
    return ''


def connect_firebase(cfg, previous):
    """
    Use cfg (url, fleet_id, robot_key, and api_key, user_email, user_password, empty for no
    sign-in). Save it if Firebase comes up. Returns True on success.
    """
    fields = tuple(cfg.get(k) or '' for k in FIREBASE_FIELDS)
    try:
        Esp32Base.validate_firebase_config(*fields)
    except ValueError as e:
        click.secho('  %s' % e, fg='red')
        return False
    init0 = firebase_inits()
    if not backend.command('set_firebase_config', *fields, persist=False, apply_now=True):
        click.secho('  Could not send the settings to the ESP32', fg='red')
        return False
    ok, err = wait_for_firebase(init0)
    if ok:
        backend.command('set_firebase_config', *fields, persist=True, apply_now=False)
        click.secho('  Firebase ready. Saved on the ESP32.', fg='green')
        return True
    hint = firebase_failure_hint(err)
    click.secho('  Firebase did not come up: %s%s' % (err, ' (%s)' % hint if hint else ''), fg='red')
    if is_configured(previous):
        if confirm('Go back to the previous settings (%s)?' % previous['url'], default=True):
            init0 = firebase_inits()
            backend.command('set_firebase_config', *(previous.get(k) or '' for k in FIREBASE_FIELDS),
                            persist=False, apply_now=True)
            if wait_for_firebase(init0)[0]:
                click.secho('  Back on the previous settings', fg='green')
            else:
                click.secho('  The previous settings did not come up either. The ESP32 goes back to its saved '
                            'settings at its next reboot.', fg='red')
        else:
            click.secho('  The ESP32 keeps trying the new settings, but they are not saved: it goes back to its '
                        'saved settings at its next reboot.', fg='yellow')
    else:
        click.secho('  The ESP32 goes back to its saved settings at its next reboot.', fg='yellow')
    return False


def is_configured(cfg):
    return bool(cfg and cfg.get('url') and cfg.get('fleet_id') and cfg.get('robot_key'))


def known_firebase(esp_fb):
    """
    The settings the ESP32 has (the only place they are kept), to offer as defaults. The
    fleet ID defaults to $HELLO_FLEET_ID, and the URL to this robot's own project
    ($HELLO_FLEET_ID-pwr-mgmt) default database.
    """
    out = {k: (esp_fb or {}).get(k) or None for k in FIREBASE_FIELDS}
    out['fleet_id'] = out['fleet_id'] or os.environ.get('HELLO_FLEET_ID')
    out['url'] = out['url'] or Esp32Firebase.default_database_url()
    return out


def ask_database_url(default=None):
    """The database URL, typed. None if left empty."""
    while True:
        url = ask('Database URL', default).strip()
        if not url:
            return None
        try:
            return Esp32Firebase.normalize_database_url(url)
        except ValueError as e:
            click.secho('  %s' % e, fg='red')


def ask_fleet_id(default=None):
    """The robot's fleet ID (its data is under robots/<fleet ID>), typed. None if left empty."""
    while True:
        fleet_id = ask('Fleet ID', default).strip()
        if not fleet_id:
            return None
        try:
            Esp32Base.validate_fleet_id(fleet_id)
            return fleet_id
        except ValueError as e:
            click.secho('  %s' % e, fg='red')


def ask_sign_in(known, project=None, api_key=None):
    """
    The optional Firebase user: {'api_key', 'user_email', 'user_password'}, or None to cancel.
    project: the Firebase project, when known, for direct links. api_key: the project's Web
    API key when the CLI read it; else it is asked for (sign-in goes through Google's Identity
    Toolkit, which needs the key to know the project).
    """
    where = lambda page: Esp32Firebase.auth_console_url(project, page) if project else ''
    click.echo('  The robot signs in as a Firebase user of the project. In the Firebase console:')
    click.echo('    Authentication > Sign-in method: enable Email/Password%s' % ('\n      ' + where('providers') if project else ''))
    click.echo('    Authentication > Users: add a user for the robot%s' % ('\n      ' + where('users') if project else ''))
    if not api_key:
        click.echo('    Project settings > General: copy the Web API key')
    click.echo('  The app must sign in too, as any user of the project.')
    cfg = {'api_key': api_key or ask('Web API key', known['api_key'], secret=True),
           'user_email': ask('User email', known['user_email']),
           'user_password': ask('User password', known['user_password'], secret=True)}
    if not all(cfg.values()):
        click.secho('  The email and the password are needed%s.' % ('' if api_key else ', and the API key'), fg='red')
        return None
    return cfg


def cli_sign_in(project, known):
    """
    With the CLI: the project's Web API key read from its web app (one made if it has none),
    then the user to sign in as. Returns the sign-in dict, or None to cancel.
    """
    api_key = Esp32Firebase.web_api_key(project)
    if not api_key:
        click.echo('  Sign-in needs the project\'s Web API key, which comes with a web app, and "%s" has none.'
                   % project)
        if not confirm('Add a web app named "Stretch power" to the project, for its API key?', default=True):
            return None
        api_key = Esp32Firebase.web_api_key(project, create=True)
    if not api_key:
        click.secho('  Could not read the Web API key: copy it from Project settings > General.', fg='yellow')
    else:
        click.secho('  Web API key read from the project.', fg='green')
    return ask_sign_in(known, project, api_key)


def print_rules(sign_in):
    for line in Esp32Firebase.rules_json(sign_in).splitlines():
        click.secho('       ' + line, fg='cyan')


def setup_database_by_hand(fleet_id, robot_key, sign_in, default_url):
    """Walk the user through the console. Returns the database URL, or None."""
    header('Set up the database in the Firebase console')
    project = Esp32Firebase.default_project_id()
    click.echo('  1. Open https://console.firebase.google.com and create a project, or pick one.%s\n'
               % ('\n     For this robot\'s own project, use the project ID %s.' % project if project else ''))
    click.echo('  2. Build > Realtime Database > Create Database: any location, start in locked mode.\n'
               '  3. Rules tab. If it has only the default rules (".read" and ".write" false, or a\n'
               '     "now < ..." test-mode date), replace them with these, then Publish. If it has other\n'
               '     rules, merge these into them instead: add the "robots" entry, or add\n'
               '     "power_system_status" and "power_system_requests" to an existing "robots" > "$fleetId".\n'
               '     Skip this if another robot set up this database already.\n')
    print_rules(bool(sign_in))
    click.echo('\n  4. Data tab: add robots > %s > robot_key, with the value (in quotes):' % fleet_id)
    row('     robot_key', '"%s"' % robot_key, fg='cyan')
    click.echo('  5. Copy the database URL from the top of the Data tab.\n')
    return ask_database_url(default_url)


def pick(items, label, extra=(), enter='set up by hand instead'):
    """A numbered choice. Returns the item, an extra key typed, or None for Enter."""
    for i, item in enumerate(items):
        click.echo('  %2d  %s' % (i + 1, label(item)))
    for k, text in extra:
        click.echo('  %2s  %s' % (k, text))
    click.echo('  Enter: %s' % enter)
    while True:
        x = input('> ').strip()
        if not x:
            return None
        if x.isdigit() and 1 <= int(x) <= len(items):
            return items[int(x) - 1]
        if x in dict(extra):
            return x
        click.echo('  Unknown choice')


# setup_database_with_cli: back to the Firebase menu, rather than on by hand
BACK_TO_MENU = 'back to menu'

FIREBASE_CLI_NOT_INSTALLED = (
    '  The Firebase CLI is not installed. It is recommended: with it, this tool sets up the database\n'
    '  for you (project, rules and robot key). To install it, in a terminal:\n'
    '    curl -sL https://firebase.tools | bash        (or, with Node.js 20+: sudo npm install -g firebase-tools)\n'
    '    firebase login                                (over SSH: firebase login --no-localhost)\n'
    '  then choose n here again. More: https://firebase.google.com/docs/cli')


def firebase_cli_sign_in_error(expired):
    """The CLI can't authenticate: say how to sign in. expired: an account is on record, but its login expired."""
    command = 'firebase login --reauth' if expired else 'firebase login'
    click.secho('  Error: the Firebase CLI is %s. In a terminal, run\n'
                '    %s\n'
                '  (over SSH: %s --no-localhost), then choose n here again.'
                % ('signed in, but its login has expired' if expired else 'not signed in', command, command),
                fg='red')


def robot_sign_in(sign_in):
    """An ID token for the optional user (None without one), as the ESP32 signs in. False if refused."""
    if not sign_in:
        return None
    try:
        return Esp32Firebase.sign_in(sign_in['api_key'], sign_in['user_email'], sign_in['user_password'])
    except ValueError as e:
        click.secho('  %s. Check the Web API key, the email and the password.' % e, fg='red')
        return False


def wait_for_rules(url, fleet_id, robot_key, id_token, timeout=60.0):
    """Wait until the rules in force do what the robot needs (Esp32Firebase.rules_work). Returns (ok, why not)."""
    click.echo('  Checking the rules, as the robot uses them ', nl=False)
    ts = time.time()
    while True:
        ok, why = Esp32Firebase.rules_work(url, fleet_id, robot_key, id_token)
        if ok:
            click.secho(' ok', fg='green')
            return True, ''
        if time.time() - ts > timeout:
            click.echo('')
            return False, why
        click.echo('.', nl=False)
        time.sleep(2.0)


def check_rules_by_hand(url, fleet_id, robot_key, sign_in):
    """After the console steps: whether the rules do what the robot needs. Lets the user fix them and check again."""
    id_token = robot_sign_in(sign_in)
    if id_token is False:
        return False
    while True:
        ok, why = wait_for_rules(url, fleet_id, robot_key, id_token, timeout=10.0)
        if ok:
            return True
        click.secho('  Not yet: %s. Check steps 3 and 4: the robot rules in force, and\n'
                    '  robots/%s/robot_key = "%s".' % (why, fleet_id, robot_key), fg='yellow')
        if not confirm('Check again?', default=True):
            return False


def reconcile_rules(project, instance, url, fleet_id, robot_key, id_token, sign_in, source):
    """
    The database has its own rules: don't replace them. Publish them with the robot rules added
    as a comment, and wait for the user to merge them. Returns True once the rules work.
    """
    click.secho('  This database has its own rules, so they are not replaced. The robot rules are added to\n'
                '  them as a comment (the rules in force don\'t change), for you to merge.', fg='yellow')
    if not confirm('Publish the rules with the robot rules as a comment?', default=True):
        return False
    Esp32Firebase.deploy_rules(project, instance, Esp32Firebase.rules_with_comments(source, sign_in))
    click.secho('  Published.', fg='green')
    click.echo('  Open %s/%s/rules. Merge the robot rules from the\n'
               '  comment at the top into the rules, delete the comment, and click Publish.'
               % (Esp32Firebase.database_console_url(project), instance))
    while True:
        if input('  Enter when the rules are merged and published, q to stop: ').strip().lower() == 'q':
            return False
        ok, why = wait_for_rules(url, fleet_id, robot_key, id_token, timeout=10.0)
        if ok:
            return True
        click.secho('  Not yet: %s.' % why, fg='yellow')


def setup_database_with_cli(fleet_id, robot_key, want_sign_in, known):
    """
    With the Firebase CLI: pick or create a project, find its database, write the robot key,
    take the user to sign in as (with the project's Web API key, if want_sign_in), and get
    the rules in force (published over a new database's defaults, or merged by the user into
    rules that are there). Returns (database URL, sign-in dict or None), None to go on by
    hand, or BACK_TO_MENU.
    """
    fb = Esp32Firebase
    try:
        if not fb.logged_in():
            firebase_cli_sign_in_error(expired=False)
            return BACK_TO_MENU
        projects = fb.list_projects()  # The first call that needs a valid login

        header('Firebase projects')
        own = fb.default_project_id()
        choice = pick(projects, lambda p: '%s (%s)%s' % (p.get('displayName') or p['projectId'], p['projectId'],
                                                        ', this robot\'s' if p['projectId'] == own else ''),
                      extra=(('c', 'create a project%s' % (' (%s)' % own if own else '')),))
        if choice is None:
            return None
        if choice == 'c':
            project = ask('Project ID (6-30 of a-z 0-9 -, starting with a letter)', own).strip()
            if not project:
                return None
            click.echo('  Creating "%s" (this takes a minute)...' % project)
            fb.create_project(project, ask('Display name', project))
        else:
            project = choice['projectId']

        while True:
            databases = fb.list_databases(project)
            if databases:
                break
            click.secho('  "%s" has no Realtime Database yet, and the CLI can\'t create a project\'s first one.'
                        % project, fg='yellow')
            click.echo('  Open %s, click Create Database (any location, locked mode), then come back.'
                       % fb.database_console_url(project))
            if input('  Enter when it is created, q to set up by hand: ').strip().lower() == 'q':
                return None
        if len(databases) == 1:
            db = databases[0]
        else:
            header('Realtime Databases in %s' % project)
            db = pick(databases, lambda d: '%s (%s)' % (d['databaseUrl'], d.get('location', '?')))
            if db is None:
                return None
        instance, url = db['name'], fb.normalize_database_url(db['databaseUrl'])

        # The key first (the CLI isn't held to the rules), then see whether the rules work with it
        fb.set_robot_key(project, instance, fleet_id, robot_key)
        click.secho('  Robot key written to robots/%s/robot_key' % fleet_id, fg='green')
        sign_in = None
        if want_sign_in:
            sign_in = cli_sign_in(project, known)
            if sign_in is None:
                return BACK_TO_MENU
        id_token = robot_sign_in(sign_in)
        if id_token is False:
            return BACK_TO_MENU
        ok, why = fb.rules_work(url, fleet_id, robot_key, id_token)
        if ok:
            click.secho('  %s already has the robot rules.' % url, fg='green')
            return url, sign_in
        click.echo('  The rules %s has now don\'t do what the robot needs (%s).' % (url, why))
        source = fb.current_rules(project, instance)
        if fb.classify_rules(source) == 'other':
            return (url, sign_in) if reconcile_rules(project, instance, url, fleet_id, robot_key, id_token,
                                                     bool(sign_in), source) else BACK_TO_MENU
        click.echo('  It has %s rules: the robot rules%s replace them.'
                   % ('a new database\'s default' if fb.classify_rules(source) == 'default' else 'this tool\'s earlier',
                      ' with sign-in' if sign_in else ''))
        if sign_in and fb.classify_rules(source) == 'ours':
            # Rules cover the whole database: a robot here that doesn't sign in stops working
            click.secho('  With these rules, every robot on this database must sign in: one set up without a\n'
                        '  Firebase user stops publishing and taking commands until it is set up again with one.',
                        fg='yellow')
        if not confirm('Publish the robot rules%s?' % (' with sign-in' if sign_in else ''), default=True):
            return None
        fb.deploy_rules(project, instance, fb.rules_json(bool(sign_in)))
        click.secho('  Rules published', fg='green')
        # Before the ESP32 tries them: its first connection would fail while the rules take effect
        ok, why = wait_for_rules(url, fleet_id, robot_key, id_token)
        if not ok:
            click.secho('  The rules still don\'t work (%s). Check the Rules tab and robots/%s/robot_key in the '
                        'console.' % (why, fleet_id), fg='red')
            return BACK_TO_MENU
        return url, sign_in
    except fb.AuthError:
        firebase_cli_sign_in_error(expired=True)
        return BACK_TO_MENU
    except (fb.CliError, ValueError) as e:
        click.secho('  Firebase CLI: %s' % e, fg='red')
        return None


def print_app_settings(cfg):
    header('For the app')
    row('Database URL', cfg['url'], fg='cyan')
    row('Fleet ID', cfg['fleet_id'], fg='cyan')
    row('Robot key', cfg['robot_key'], fg='cyan')
    if cfg.get('user_email'):
        row('Sign in', 'as any user of the project', fg='cyan')
    click.echo('  The robot\'s data is under robots/%s. Every command the app writes must carry this robot_key.\n'
               '  To revoke the key, change robots/%s/robot_key in the Firebase console, then set the new\n'
               '  key on the robot (e) and in the app.' % (cfg['fleet_id'], cfg['fleet_id']))


def command_snippets(cfg):
    """
    curl and Python to sleep and wake the robot through its database, from cfg (the ESP32's
    Firebase settings). Every command carries the robot key. With a user to sign in as, they
    sign in first (the password left out).
    """
    url = '%s/robots/%s' % (cfg['url'], cfg['fleet_id'])
    sign_in = bool(cfg.get('user_email'))
    sign_in_url = 'https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key=%s' % cfg.get('api_key')
    curl = ['URL=%s' % url, 'KEY=%s' % cfg['robot_key']]
    if sign_in:
        curl += ['# Sign in (the rules want a signed-in user)',
                 'TOKEN=$(curl -s "%s" -H "Content-Type: application/json" \\' % sign_in_url,
                 '  -d \'{"email": "%s", "password": "<password>", "returnSecureToken": true}\' \\' % cfg['user_email'],
                 '  | python3 -c \'import json, sys; print(json.load(sys.stdin)["idToken"])\')']
    auth = '?auth=$TOKEN' if sign_in else ''
    for action in ('sleep', 'wake'):
        curl += ['', '# %s (a new id for every command)' % action.capitalize(),
                 'curl -X PUT "$URL/power_system_requests/power_command.json%s" \\' % auth,
                 '  -d \'{"robot_key": "\'$KEY\'", "action": "%s", "id": "%s-\'$(date +%%s)\'", '
                 '"acknowledged": false, "ts": {".sv": "timestamp"}}\'' % (action, action)]
    curl += ['', '# The robot\'s answer (pending, then done; the command\'s acknowledged turns true), and its status',
             'curl "$URL/power_system_requests/power_command_ack.json%s"' % auth,
             'curl "$URL/power_system_status.json%s"' % auth]

    py = ['import json', 'import time', 'import urllib.request', '', "URL = '%s'" % url,
          "KEY = '%s'" % cfg['robot_key']]
    if sign_in:
        py += ['', '# Sign in (the rules want a signed-in user)',
               "req = urllib.request.Request('%s'," % sign_in_url,
               "                             data=json.dumps({'email': '%s', 'password': '<password>',"
               % cfg['user_email'],
               "                                              'returnSecureToken': True}).encode(),",
               "                             headers={'Content-Type': 'application/json'})",
               "AUTH = '?auth=' + json.load(urllib.request.urlopen(req))['idToken']"]
    else:
        py += ["AUTH = ''"]
    py += ['', '', 'def get(path):',
           "    return json.load(urllib.request.urlopen(URL + '/' + path + '.json' + AUTH))",
           '', '',
           'def power(action):',
           '    """Sleep or wake the robot: action is \'sleep\' or \'wake\'. Returns the robot\'s answer."""',
           "    cmd = {'robot_key': KEY, 'action': action, 'id': '%s-%d' % (action, time.time() * 1000),",
           "           'acknowledged': False, 'ts': {'.sv': 'timestamp'}}",
           "    urllib.request.urlopen(urllib.request.Request(URL + '/power_system_requests/power_command.json' + AUTH,",
           "                                                  data=json.dumps(cmd).encode(), method='PUT'))",
           '    for _ in range(150):  # A wake while going to sleep waits for the sleep to finish',
           "        ack = get('power_system_requests/power_command_ack') or {}",
           "        if ack.get('id') == cmd['id'] and ack.get('result') != 'pending':",
           '            return ack',
           '        time.sleep(1)',
           '    return ack',
           '', '',
           "print(power('sleep'))  # Or power('wake')",
           "print(get('power_system_status'))"]
    return curl, py


def print_command_snippets(cfg):
    curl, py = command_snippets(cfg)
    header('Sleep and wake with curl')
    for line in curl:
        click.secho('  ' + line, fg='cyan')
    header('Sleep and wake with Python')
    for line in py:
        click.secho('  ' + line, fg='cyan')
    click.echo('\n  Sleep shuts down the robot\'s computer. A robot that is shut down can\'t be woken this way:\n'
               '  press its power button. The robot key in these lets whoever has them sleep and wake it.')


def setup_firebase(known, previous):
    """Set up Firebase from scratch: fleet ID, robot key, optional sign-in, database, then connect. Returns True on success."""
    fleet_id = known['fleet_id'] if known['fleet_id'] else ask_fleet_id()
    if not fleet_id:
        click.secho('  The fleet ID is needed ($HELLO_FLEET_ID is not set).', fg='red')
        return False
    robot_key = known['robot_key']
    if robot_key and confirm('Keep this robot\'s key (%s)? With a new key, the app needs the new one too.'
                             % robot_key, default=True):
        pass
    else:
        robot_key = generate_robot_key()
        click.secho('  New robot key made.', fg='green')

    click.echo('  Writes need the robot key. Signing in as a Firebase user as well is optional: it also\n'
               '  keeps people who aren\'t signed in from reading the robot\'s data.')
    # Only whether, here: the user and the project's Web API key are taken once the project is known
    want_sign_in = confirm('Also sign in as a Firebase user? This is optional, prevents unauthenticated reads, and '
                           'requires extra setup in Firebase Console. Note: Writes are already secured by the robot '
                           'key without auth.', default=False)

    url, sign_in = None, None
    if Esp32Firebase.cli_path():
        if confirm('The Firebase CLI is installed. Use it to set up the database (pick or create a project, '
                   'write the robot key, get the rules in force)? It must be logged in (firebase login)', default=True):
            done = setup_database_with_cli(fleet_id, robot_key, want_sign_in, known)
            if done == BACK_TO_MENU:
                return False
            if done is None:
                click.echo('  Going on by hand.')
            else:
                url, sign_in = done
    else:
        click.secho(FIREBASE_CLI_NOT_INSTALLED, fg='yellow')
        if not confirm('Set up the database by hand in the Firebase console instead?', default=False):
            return False
    if url is None:
        url = setup_database_by_hand(fleet_id, robot_key, want_sign_in, known['url'])
        if url is None:
            return False
        if want_sign_in:
            sign_in = ask_sign_in(known)
            if sign_in is None:
                return False
        if not check_rules_by_hand(url, fleet_id, robot_key, sign_in):
            return False

    cfg = dict(url=url, fleet_id=fleet_id, robot_key=robot_key,
               **{k: (sign_in or {}).get(k, '') for k in SIGN_IN_FIELDS})
    if connect_firebase(cfg, previous):
        print_app_settings(cfg)
        return True
    return False


def enter_firebase(known, previous):
    """Use a database set up already: its URL, the fleet ID and the robot key. Returns True on success."""
    click.echo('  Press Enter to keep a value in [brackets].')
    url = ask_database_url(known['url'])
    fleet_id = ask_fleet_id(known['fleet_id'])
    robot_key = ask('Robot key', known['robot_key'], secret=True).strip()
    if not url or not fleet_id or not robot_key:
        click.secho('  The database URL, the fleet ID and the robot key are needed.', fg='red')
        return False
    cfg = {'url': url, 'fleet_id': fleet_id, 'robot_key': robot_key, 'api_key': '', 'user_email': '',
           'user_password': ''}
    if confirm('Sign in as a Firebase user as well?', default=bool(known['user_email'])):
        cfg['api_key'] = ask('Web API key', known['api_key'], secret=True)
        cfg['user_email'] = ask('User email', known['user_email'])
        cfg['user_password'] = ask('User password', known['user_password'], secret=True)
    return connect_firebase(cfg, previous)


def config_firebase():
    if not require_esp32():
        return
    if not backend.esp.supports_firebase():
        click.secho('Firebase set-up needs ESP32 firmware %s or later (this one is %s). Update it with\n'
                    '  REx_firmware_updater --install --esp32'
                    % (Esp32Base.FIRST_FIREBASE_FIRMWARE, backend.esp.board_info['firmware_version']), fg='red')
        return
    click.secho('ESP32 Firebase set-up (remote sleep and wake).', bold=True)
    while True:
        _, esp_fb = backend.credentials()
        print_firebase_connection(esp_fb)
        known = known_firebase(esp_fb)
        # What to go back to on failure
        previous = esp_fb if is_configured(esp_fb) else None

        click.echo('')
        click.echo('  n: set up Firebase (makes a robot key, and sets up the database%s)'
                   % (' with the Firebase CLI' if Esp32Firebase.cli_path() else
                      '; installing the Firebase CLI first is recommended'))
        click.echo('  e: enter the settings (a database set up already, the fleet ID and the robot key)')
        if previous:
            click.echo('  a: show what the app needs (database URL, fleet ID and robot key)\n'
                       '  c: commands to sleep and wake the robot (curl and Python)')
        click.echo('  p: print the database rules\n'
                   '  r: reconnect\n'
                   '  f: forget the settings saved on the ESP32\n'
                   '  q: back to the main menu')
        k = input('> ').strip()

        if k == 'q' or k == '':
            return
        if k == 'n':
            if setup_firebase(known, previous):
                return
        elif k == 'e':
            if enter_firebase(known, previous):
                return
        elif k == 'c' and previous:
            print_command_snippets(previous)
        elif k == 'a' and previous:
            print_app_settings(previous)
        elif k == 'p':
            sign_in = bool(previous and previous.get('user_email'))
            header('Database rules%s' % (' (with sign-in)' if sign_in else ''))
            click.echo(Esp32Firebase.rules_json(sign_in))
            click.echo('  Over a new database\'s default rules, publish these as they are. If the database has\n'
                       '  other rules, merge these into them.')
        elif k == 'r':
            init0 = firebase_inits()
            backend.command('trigger_firebase_restart')
            ok, err = wait_for_firebase(init0)
            hint = '' if ok else firebase_failure_hint(err)
            click.secho('  Firebase ready' if ok else '  Firebase did not come up: %s%s'
                        % (err, ' (%s)' % hint if hint else ''), fg='green' if ok else 'red')
        elif k == 'f':
            if confirm('Erase the Firebase settings saved on the ESP32 and stop Firebase?'):
                backend.command('forget_firebase')
                if previous:
                    click.echo('  The robot key still works until you change robots/%s/robot_key in the Firebase '
                               'console.' % previous['fleet_id'])
                time.sleep(1.0)
        else:
            click.echo('  Unknown choice')


# ############################# Main #############################

backend = Esp32Backend()
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
