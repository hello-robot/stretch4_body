#!/usr/bin/env python3
"""
stretch_doctor: how the robot is feeling, in plain words.

It gathers readings from the running stretch_body_server (its status socket only), the
systemd journal (the server's user unit and the kernel), the /dev/hello-* links and
Linux, then judges every reading against RULES: one row per symptom, with its normal
range, the window it covers, what it means and what to do. The report gives one
headline and the findings, worst first.

Read-only: it never opens a robot device, never stops the server and never moves a joint.
Each source is a function in SOURCES that returns readings and the raw text behind them.
"""
import argparse
import collections
import datetime
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time

from stretch4_body.core.server_status_reader import read_server_status
from stretch4_body.subsystem.power_health import describe_power_event

OK, WATCH, ATTENTION, UNKNOWN = 'ok', 'watch', 'attention', 'unknown'
RANK = {ATTENTION: 0, WATCH: 1, UNKNOWN: 2, OK: 3}
HEADLINES = {OK: 'Healthy', WATCH: 'Watch', ATTENTION: 'Needs attention'}

JOURNAL_SINCE = '24 hours ago'
SERVER_UNIT = 'stretch_body_server.service'
BUTTON_FAULTS_EVENT = 55
BMS_FAILURE_EVENTS = (10, 11, 12, 25)
BUTTON_RED_FAILURES = 5          # ButtonFaults.h: 5 failed BMS replies in 60 s latch BMS_UNRELIABLE

Rule = collections.namedtuple('Rule', 'area reading low high level window symptom evidence cause action')

RULES = [
    Rule('server', 'server_up', 1, None, ATTENTION, 'now',
         'The robot server is not running',
         'No status on the server status socket.',
         'stretch_body_server stopped or failed to start; nothing can drive the robot.',
         'Run stretch_body_server --launch and read the errors it prints.'),
    Rule('server', 'server_failed_starts', None, 0, ATTENTION, '24 h',
         'The robot server failed to start or crashed',
         '{server_failed_starts} failed exits and {server_starts} starts. Last error: {server_last_error}',
         'An exception during start or while running; the traceback names the part.',
         'Read journalctl --user -u stretch_body_server; send the export if the cause is not obvious.'),
    Rule('server', 'server_tracebacks', None, 0, WATCH, '24 h',
         'The robot server logged Python errors',
         '{server_tracebacks} tracebacks. Last: {server_last_error}',
         'A code path raised an exception; the server may have recovered on its own.',
         'Check the traceback in the export; a repeat after a clean restart is a bug to report.'),
    Rule('server', 'server_missed_pct', None, 1.0, WATCH, 'since the server started',
         'The robot server is missing its 100 Hz control loop',
         '{server_missed_loops} of {server_loops} loops missed ({server_missed_pct:.2f} %), '
         'average {server_avg_hz:.0f} Hz.',
         'The NUC is busy (CPU load, a slow device) and the loop overruns.',
         'Check the CPU load finding; stop heavy processes and see whether it settles.'),
    Rule('power', 'lamp_held', None, 0, ATTENTION, 'since the last power state change',
         'Power button light red: {lamp_names}',
         'The power board latched it at {lamp_since}.',
         'The power board saw the fault in its own data; the findings below name the reading.',
         'Fix the cause, then sleep and wake the robot to clear the light.'),
    Rule('power', 'lamp_unknown', None, 0, WATCH, 'held events',
         'Cannot tell whether the power button light is red',
         'The last button change is no longer in the {events_held} events the server holds.',
         'The event log turned over; this firmware does not report the light directly.',
         'Look at the button; if it is red, run with --export and send it.'),
    Rule('power', 'bms_fail_pct', None, 0.1, ATTENTION, 'since the power board started',
         'Battery data unreliable',
         '{bms_failures} bad of {bms_polls} battery replies since the power board started '
         '{pimu_up} ago ({bms_fail_pct:.2f} %).',
         'Noise or a poor contact on the battery data line, or a failing battery monitor board.',
         'Check the battery data cable and connector; if it follows the pack to another robot, '
         'retire the pack.'),
    Rule('power', 'bms_worst_minute', None, BUTTON_RED_FAILURES - 1, ATTENTION, 'held events',
         'Battery data failed in bursts',
         '{bms_worst_minute} failed battery replies within one minute (5 turn the button red).',
         'Bursts point to noise, for example while charging.',
         'Note what the robot was doing at {bms_worst_at}; compare charging and not charging.'),
    Rule('power', 'power_faults', None, 0, ATTENTION, 'now',
         'Power board check failing: {power_fault_names}',
         '{power_fault_messages}',
         'See the power check message.',
         'Run stretch_power_diagnostics for the detail.'),
    Rule('power', 'power_warnings', None, 0, WATCH, 'now',
         'Power board check warning: {power_warning_names}',
         '{power_warning_messages}',
         'See the power check message.',
         'Run stretch_power_diagnostics for the detail.'),
    Rule('power', 'battery_soc', 15, None, WATCH, 'now',
         'Battery low',
         'Charge {battery_soc} %.',
         'The pack needs charging.',
         'Put the robot on the charger.'),
    Rule('power', 'battery_soh', 80, None, WATCH, 'now',
         'Battery worn',
         'Battery health {battery_soh} % of new.',
         'The pack has aged; run time is shorter.',
         'Plan a replacement pack.'),
    Rule('usb', 'devices_missing', None, 0, ATTENTION, 'now',
         'Robot device missing: {devices_missing_names}',
         'The server knows these boards, their /dev links are gone.',
         'A USB cable, hub or the board itself dropped off.',
         'Reseat the cable; check the USB findings for disconnects.'),
    Rule('usb', 'usb_overcurrent', None, 0, ATTENTION, '24 h',
         'A USB port reported over-current',
         '{usb_overcurrent} kernel over-current messages. Last: {usb_overcurrent_last}',
         'A shorted cable or a device drawing too much from the port.',
         'Find the port in the message and check the cable and the device.'),
    Rule('usb', 'usb_resets', None, 0, WATCH, '24 h',
         'USB devices had to be reset',
         '{usb_resets} USB resets. Last: {usb_resets_last}',
         'A device stopped answering and the kernel reset it; cables, hubs and power are suspects.',
         'If it repeats on one port, swap the cable or hub.'),
    Rule('usb', 'usb_disconnects', None, 0, WATCH, '24 h',
         'USB devices disconnected',
         '{usb_disconnects} disconnects. Last: {usb_disconnects_last}',
         'Normal after a firmware flash or a power state change; otherwise a loose cable.',
         'If no one flashed or slept the robot, check the cable of the device named.'),
    Rule('linux', 'cpu_temp_c', None, 90, WATCH, 'now',
         'NUC running hot',
         'CPU package at {cpu_temp_c:.0f} C over 1 s, {cpu_throttle} heat throttle events since boot.',
         'Blocked airflow or a fan problem.',
         'Check the NUC vents and fan.'),
    Rule('linux', 'load_per_cpu', None, 1.0, WATCH, '5 min',
         'NUC overloaded',
         'Load {load_per_cpu:.2f} per CPU.',
         'More work than CPUs; the control loop can slip.',
         'Find the busy process with top.'),
    Rule('linux', 'mem_available_pct', 10, None, WATCH, 'now',
         'NUC low on memory',
         '{mem_available_pct:.0f} % of memory available.',
         'A process is holding a lot of memory.',
         'Find it with top, sorted by memory.'),
    Rule('linux', 'disk_free_pct', 15, None, ATTENTION, 'now',
         'Disk nearly full',
         '{disk_free_pct:.0f} % of the system disk free.',
         'Logs or recordings filled the disk; logging and updates fail when it is full.',
         'Clear old recordings or logs under the fleet folder.'),
    Rule('linux', 'clock_synced', 1, None, WATCH, 'now',
         'NUC clock not synchronised',
         'timedatectl reports the clock is not synchronised.',
         'No network time; log times from different sources may not line up.',
         'Connect the robot to a network with internet access.'),
]


# ------------------------------------------------------------------------ sources

def _run(cmd, timeout=20):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None


def _duration(seconds):
    if seconds >= 3600:
        return '%.1f h' % (seconds / 3600.0)
    return '%.0f min' % (seconds / 60.0)


def _clock(wall):
    return datetime.datetime.fromtimestamp(wall).strftime('%H:%M:%S') if wall else 'unknown'


def lamp_readings(pp):
    """Held button faults from the newest event 55, or lamp_unknown when it has turned over."""
    log = pp.get('power_event_log') or {}
    events = list(log.get('events') or [])
    out = {'events_held': len(events)}
    changes = [e for e in events if e.get('event_id') == BUTTON_FAULTS_EVENT]
    if changes:
        last = changes[-1]
        error, charging = describe_power_event(last).split('error ', 1)[1].split(', charging ')
        names = [n for part in (error, charging) for n in part.split(' + ') if n != 'none']
        out['lamp_held'] = len(names)
        out['lamp_names'] = ' + '.join(names)
        out['lamp_since'] = _clock(last.get('estimated_wall_time_s'))
        out['lamp_unknown'] = 0
    elif events and events[0].get('event_id') == 1 and not log.get('firmware_dropped_count'):
        out['lamp_held'] = 0
        out['lamp_unknown'] = 0
    else:
        out['lamp_unknown'] = 1
    return out


def bms_readings(pp):
    diag = pp.get('power_diagnostics') or {}
    out = {}
    failures = sum(diag.get(k) or 0 for k in
                   ('bms_crc_errors', 'bms_timeouts', 'bms_rx_overflows', 'bms_protocol_errors'))
    valid = diag.get('bms_valid_replies')
    if valid is not None:
        polls = valid + failures
        out.update(bms_failures=failures, bms_polls=polls,
                   bms_fail_pct=100.0 * failures / polls if polls else 0.0,
                   pimu_up=_duration((diag.get('uptime_ms') or 0) / 1000.0))
    times = [e.get('estimated_wall_time_s') for e in (pp.get('power_event_log') or {}).get('events') or []
             if e.get('event_id') in BMS_FAILURE_EVENTS and e.get('estimated_wall_time_s')]
    worst, worst_at = 0, None
    for i, t in enumerate(times):
        n = sum(1 for u in times[i:] if u < t + 60.0)
        if n > worst:
            worst, worst_at = n, t
    out['bms_worst_minute'] = worst
    out['bms_worst_at'] = _clock(worst_at)
    return out


def status_readings(status):
    """Readings from one server status, or server_up 0 when there is none."""
    if not status:
        return {'server_up': 0}
    out = {'server_up': 1}
    loop = (status.get('server') or {}).get('control_loop') or {}
    if loop.get('num_loops'):
        out.update(server_loops=loop['num_loops'], server_missed_loops=loop.get('missed_loops', 0),
                   server_missed_pct=100.0 * loop.get('missed_loops', 0) / loop['num_loops'],
                   server_avg_hz=loop.get('avg_rate_hz') or 0.0)
    pp = status.get('power_periph') or status.get('pimu') or {}
    if pp:
        out.update(lamp_readings(pp))
        out.update(bms_readings(pp))
        checks = (pp.get('power_health') or {}).get('checks') or []
        for level, key in (('fault', 'power_fault'), ('warn', 'power_warning')):
            hit = [c for c in checks if c.get('level') == level]
            out[key + 's'] = len(hit)
            out[key + '_names'] = ', '.join(c['id'].replace('_', ' ') for c in hit)
            out[key + '_messages'] = ' '.join(c.get('message', '') for c in hit)
        for key in ('battery_soc', 'battery_soh'):
            if pp.get(key) is not None:
                out[key] = pp[key]
    boards = (status.get('firmware_versions') or {}).get('boards') or []
    ports = sorted({b['port'].split()[0] for b in boards if str(b.get('port', '')).startswith('/dev/hello-')})
    if ports:
        missing = [p for p in ports if not os.path.exists(p)]
        out['devices_missing'] = len(missing)
        out['devices_missing_names'] = ', '.join(missing)
    return out


ANSI = re.compile(r'\x1b\[[0-9;]*m')


def server_journal_readings(text):
    if text is None:
        return {}
    lines = [ANSI.sub('', l) for l in text.splitlines()]
    errors = [l for l in lines if '[ERROR]' in l or re.search(r'^\w+(Error|Exception):', l.strip())]
    last = errors[-1].split('[ERROR]:', 1)[-1].strip() if errors else 'none'
    return {
        'server_starts': sum('Started ' + SERVER_UNIT in l for l in lines),
        'server_failed_starts': sum(SERVER_UNIT + ': Failed with result' in l for l in lines),
        'server_tracebacks': sum('Traceback (most recent call last)' in l for l in lines),
        'server_last_error': last[:160],
    }


KERNEL_PATTERNS = {
    'usb_disconnects': re.compile(r'USB disconnect'),
    'usb_resets': re.compile(r'reset \S+ USB device'),
    'usb_overcurrent': re.compile(r'over-current', re.I),
}


def kernel_readings(text):
    if text is None:
        return {}
    out = {}
    lines = text.splitlines()
    for key, pattern in KERNEL_PATTERNS.items():
        hit = [l for l in lines if pattern.search(l)]
        out[key] = len(hit)
        out[key + '_last'] = hit[-1].strip()[:160] if hit else 'none'
    return out


def _package_temp():
    """The CPU package temperature in C, from the x86_pkg_temp thermal zone."""
    root = '/sys/class/thermal'
    for zone in sorted(os.listdir(root)) if os.path.isdir(root) else []:
        try:
            with open(os.path.join(root, zone, 'type')) as f:
                if f.read().strip() != 'x86_pkg_temp':
                    continue
            with open(os.path.join(root, zone, 'temp')) as f:
                return int(f.read()) / 1000.0
        except (OSError, ValueError):
            pass
    return None


def linux_readings():
    out = {}
    temps = []
    for _ in range(3):
        temps.append(_package_temp())
        time.sleep(0.3)
    if None not in temps:
        out['cpu_temp_c'] = min(temps)
    try:
        with open('/sys/devices/system/cpu/cpu0/thermal_throttle/package_throttle_count') as f:
            out['cpu_throttle'] = int(f.read())
    except (OSError, ValueError):
        out['cpu_throttle'] = 'unknown'
    out['load_per_cpu'] = os.getloadavg()[1] / (os.cpu_count() or 1)
    try:
        with open('/proc/meminfo') as f:
            mem = dict(l.split(':', 1) for l in f)
        out['mem_available_pct'] = 100.0 * int(mem['MemAvailable'].split()[0]) / int(mem['MemTotal'].split()[0])
    except (OSError, KeyError, ValueError):
        pass
    disk = shutil.disk_usage('/')
    out['disk_free_pct'] = 100.0 * disk.free / disk.total
    synced = _run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'])
    if synced:
        out['clock_synced'] = 1 if synced.strip() == 'yes' else 0
    return out


def gather():
    """(readings, evidence): every source's readings merged, and the raw input behind them."""
    status = read_server_status(timeout=3.0)
    server_log = _run(['journalctl', '--user', '-u', SERVER_UNIT, '--since', JOURNAL_SINCE,
                       '--no-pager', '-o', 'short-iso'])
    kernel_log = _run(['journalctl', '-k', '--since', JOURNAL_SINCE, '--no-pager', '-o', 'short-iso'])
    readings = {}
    for part in (status_readings(status), server_journal_readings(server_log),
                 kernel_readings(kernel_log), linux_readings()):
        readings.update(part)
    evidence = {'status.json': json.dumps(status, indent=1, default=str),
                'journal_server.txt': server_log or '', 'journal_kernel.txt': kernel_log or ''}
    return readings, evidence


# ------------------------------------------------------------------------ judging

class _Readings(dict):
    def __missing__(self, key):
        return '?'


def judge(rule, readings):
    value = readings.get(rule.reading)
    if value is None:
        return UNKNOWN
    if (rule.low is not None and value < rule.low) or (rule.high is not None and value > rule.high):
        return rule.level
    return OK


def _fill(template, readings):
    try:
        return template.format_map(_Readings(readings))
    except (ValueError, TypeError):
        return template


def examine(readings):
    """Every rule's finding, worst first."""
    findings = []
    for rule in RULES:
        level = judge(rule, readings)
        findings.append({
            'level': level, 'area': rule.area, 'reading': rule.reading,
            'value': readings.get(rule.reading), 'window': rule.window,
            'symptom': _fill(rule.symptom, readings), 'evidence': _fill(rule.evidence, readings),
            'cause': rule.cause, 'action': rule.action})
    findings.sort(key=lambda f: RANK[f['level']])
    return findings


def headline(findings):
    levels = {f['level'] for f in findings}
    for level in (ATTENTION, WATCH):
        if level in levels:
            return level
    return OK


# ------------------------------------------------------------------------ report

def render(findings, robot, verbose=False, now=None):
    level = headline(findings)
    when = datetime.datetime.fromtimestamp(now or time.time()).strftime('%Y-%m-%d %H:%M')
    out = ['%s, %s: %s' % (robot, when, HEADLINES[level])]
    shown = [f for f in findings if verbose or f['level'] in (ATTENTION, WATCH)]
    for f in shown:
        out.append('')
        out.append('[%s] %s' % (f['level'].upper(), f['symptom']))
        if f['level'] in (ATTENTION, WATCH):
            out.append('  Evidence (%s): %s' % (f['window'], f['evidence']))
            out.append('  Likely cause: %s' % f['cause'])
            out.append('  Action: %s' % f['action'])
        elif f['level'] == UNKNOWN:
            out.append('  Not judged: no %s reading.' % f['reading'])
    if not shown:
        out.append('Every rule within its normal range. --verbose lists them.')
    return '\n'.join(out) + '\n'


def export(directory, text, findings, readings, evidence):
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    path = os.path.join(directory, 'doctor_%s' % stamp)
    os.makedirs(path)
    files = dict(evidence)
    files['report.txt'] = text
    files['report.json'] = json.dumps({'findings': findings, 'readings': readings}, indent=1, default=str)
    for name, body in files.items():
        with open(os.path.join(path, name), 'w') as f:
            f.write(body)
    return path


def build_parser():
    parser = argparse.ArgumentParser(
        prog='stretch_doctor',
        description='How the robot is feeling: power board, button light, robot server, USB '
                    'and Linux, judged against normal ranges. Read-only.',
        epilog='Exit status: 0 healthy or watch, 1 needs attention.')
    parser.add_argument('-v', '--verbose', action='store_true', help='show every rule, not only findings')
    parser.add_argument('--json', action='store_true', help='print findings and readings as JSON')
    parser.add_argument('--export', nargs='?', metavar='DIR',
                        const=os.path.join(os.environ.get('HELLO_FLEET_PATH', os.path.expanduser('~/stretch_user')),
                                           'log', 'exports'),
                        help='also save the report and the raw evidence to DIR '
                             '(default $HELLO_FLEET_PATH/log/exports)')
    parser.add_argument('--exam', action='store_true',
                        help='also run stretch_system_check (slow) and append its output')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    readings, evidence = gather()
    findings = examine(readings)
    robot = os.environ.get('HELLO_FLEET_ID') or socket.gethostname()
    text = render(findings, robot, args.verbose)
    if args.exam:
        exam = _run(['stretch_system_check'], timeout=600)
        evidence['system_check.txt'] = exam or ''
        text += '\nstretch_system_check\n' + (exam or 'did not run\n')
    if args.json:
        print(json.dumps({'headline': headline(findings), 'findings': findings, 'readings': readings},
                         indent=2, default=str))
    else:
        sys.stdout.write(text)
    if args.export:
        print('Saved %s' % export(args.export, text, findings, readings, evidence), file=sys.stderr)
    return 1 if headline(findings) == ATTENTION else 0


if __name__ == '__main__':
    sys.exit(main())
