#!/usr/bin/env python3
"""
stretch_power_diagnostics: what the PIMU says about the power path, in plain language.

Live, it reads the status stretch_body_server publishes (RobotClient over ZMQ); it never
opens a device, because the server is the only process allowed to talk to the PIMU.
With --from-log it rebuilds the same report from a SentryStatusLogger file
($HELLO_FLEET_PATH/log/stretch_status/status_*.json, or a stretch_status --export zip),
which is the post-mortem path for a robot that comes back from the field.

Reference: docs/primer_power_diagnostics.md.
"""
import argparse
import datetime
import json
import os
import socket
import sys
import time
import zipfile

from stretch4_body.core.status_log import StatusLogExpander
from stretch4_body.subsystem.power_health import (
    CHECK_TITLES, PowerHealthEvaluator, describe_power_event)

LAMPS = {'ok': '[ OK  ]', 'warn': '[WARN ]', 'fault': '[FAULT]', 'unknown': '[  ?  ]'}
COLORS = {'ok': '\033[32m', 'warn': '\033[33m', 'fault': '\033[31m', 'unknown': '\033[37m'}
RESET = '\033[0m'


# ------------------------------------------------------------------------ reading

def power_periph_of(status):
    """status['power_periph'] from a RobotClient status or a logged status sample."""
    if not isinstance(status, dict):
        return None
    return status.get('power_periph') or status.get('pimu')


def _iter_json_array(text):
    """
    Yield the elements of a logged JSON array one at a time. A damaged file (a write
    cut short, a file that starts mid-sample, bytes spliced in) does not stop the read:
    at each damaged spot the reader skips to the next '}, {' element boundary and goes
    on. The generator's return value (StopIteration.value) says how many spots it
    skipped, or None for a clean file.
    """
    decoder = json.JSONDecoder()
    n = len(text)
    skipped = 0
    stripped = text.lstrip()
    if stripped.startswith('['):
        i = n - len(stripped) + 1
    else:
        skipped += 1
        i = text.find('}, {')
        i = n if i < 0 else i + 3
    while True:
        while i < n and text[i] in ' \t\r\n,':
            i += 1
        if i >= n:
            break
        if text[i] == ']':
            if not text[i + 1:].strip():
                break
            # The end of a list nested in a sample: the last resync landed inside one.
            # Same damaged spot, so not counted again.
            nxt = text.find('}, {', i + 1)
            if nxt < 0:
                break
            i = nxt + 3
            continue
        try:
            element, i = decoder.raw_decode(text, i)
        except ValueError:
            skipped += 1
            nxt = text.find('}, {', i + 1)
            if nxt < 0:
                break
            i = nxt + 3
            continue
        yield element
    return ('skipped %d damaged spot%s' % (skipped, '' if skipped == 1 else 's')
            if skipped else None)


def _log_texts(path):
    """(name, text) of each status JSON file in path: a .json file or an export .zip."""
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as zf:
            for name in sorted(n for n in zf.namelist() if n.endswith('.json')):
                yield name, zf.read(name).decode('utf-8', errors='replace')
    else:
        with open(path, encoding='utf-8', errors='replace') as f:
            yield os.path.basename(path), f.read()


def load_log(path):
    """
    Read the power part of every logged status sample. Keeps only the timestamp and
    the power_periph dict of each sample: a full minute file is about 200 MB of
    status, its power part a small fraction. Elements without power data (subsystem
    dicts met while resynchronizing after damage) are dropped. Power keys the logger left
    out because they had not changed are carried forward from the last sample that has
    them (stretch4_body/core/status_log.py). Returns (samples, notes).
    """
    samples, notes = [], []
    for name, text in _log_texts(path):
        if not text.strip():
            notes.append('%s is empty' % name)
            continue
        gen = _iter_json_array(text)
        expander = StatusLogExpander()
        count = 0
        while True:
            try:
                element = expander.expand(next(gen))
            except StopIteration as stop:
                if stop.value:
                    notes.append('%s is damaged: %s, kept %d samples' % (name, stop.value, count))
                if expander.unresolved:
                    notes.append('%s: %d carried power keys had no earlier value in the file '
                                 '(its first sample is lost), left empty'
                                 % (name, expander.unresolved))
                break
            pp = power_periph_of(element)
            if pp is None or 'timestamp' not in element:
                continue
            samples.append({'timestamp': element.get('timestamp'), 'power_periph': pp})
            count += 1
    samples.sort(key=lambda s: s['timestamp'] or 0.0)
    return samples, notes


def parse_at(spec, samples):
    """
    Timestamp the --at option names: '+SECONDS' from the first sample, '-SECONDS' from
    the last, 'HH:MM[:SS]' local time on the first sample's date, or epoch seconds.
    """
    first = samples[0]['timestamp'] or 0.0
    last = samples[-1]['timestamp'] or 0.0
    spec = spec.strip()
    if spec.startswith('+'):
        return first + float(spec[1:])
    if spec.startswith('-'):
        return last - float(spec[1:])
    if ':' in spec:
        parts = [int(p) for p in spec.split(':')]
        while len(parts) < 3:
            parts.append(0)
        day = datetime.datetime.fromtimestamp(first)
        return day.replace(hour=parts[0], minute=parts[1], second=parts[2],
                           microsecond=0).timestamp()
    return float(spec)


def select_sample(samples, at_s=None):
    """Index of the last sample at or before at_s (the last sample when at_s is None)."""
    if at_s is None:
        return len(samples) - 1
    index = 0
    for i, s in enumerate(samples):
        if (s['timestamp'] or 0.0) <= at_s:
            index = i
        else:
            break
    return index


def health_for_log_sample(samples, index):
    """
    The power_health the server logged with that sample, or, for a log from a body
    without it, the same evaluation replayed over the samples up to that one (so the
    one-minute rates are real). Returns (health, how).
    """
    pp = samples[index]['power_periph']
    if pp.get('power_health', {}).get('checks'):
        return pp['power_health'], 'as logged by the server'
    evaluator = PowerHealthEvaluator()
    start_t = (samples[index]['timestamp'] or 0.0) - 2 * evaluator.t.RATE_WINDOW_S
    health = None
    for s in samples[:index + 1]:
        t = s['timestamp'] or 0.0
        if t < start_t:
            continue
        health = evaluator.evaluate(s['power_periph'], t, wall_time_s=t)
    return health, 'recomputed from the logged diagnostics'


# ------------------------------------------------------------------------ rendering

def _lamp(level, color):
    text = LAMPS.get(level, LAMPS['unknown'])
    return COLORS.get(level, '') + text + RESET if color else text


def _fmt(value, unit='', scale=1.0, digits=2):
    if value is None:
        return 'n/a'
    return ('%.' + str(digits) + 'f%s') % (value * scale, unit)


def _yes(value):
    return 'n/a' if value is None else ('yes' if value else 'no')


def key_values(pp):
    """Short labelled lines of the numbers a support engineer asks for first."""
    d = pp.get('power_diagnostics') or {}
    meta = pp.get('power_diagnostics_meta') or {}
    if not d.get('_valid'):
        return []
    lines = [
        ('Power state', '%s (%s), last transition %s (%s)' % (
            d.get('power_state_name'), d.get('power_state'),
            d.get('power_transition_reason'), d.get('power_transition_reason_raw'))),
        ('Battery (BMS)', '%s, %s, SoC %s %%, SoH %s %%, sample %s old' % (
            _fmt(d.get('battery_voltage_mv'), ' V', 0.001), _fmt(d.get('battery_current_ma'), ' A', 0.001),
            d.get('battery_soc'), d.get('battery_soh'), _fmt(meta.get('bms_age_s'), ' s', digits=1))),
    ]
    if d.get('ina_vbat_mv') is not None:
        lines.append(('INA228 witness', '%s, %s, flags %s' % (
            _fmt(d.get('ina_vbat_mv'), ' V', 0.001), _fmt(d.get('ina_current_ma'), ' A', 0.001),
            d.get('ina_flags'))))
    adapter = 'present %s, ESP connected %s' % (_yes(d.get('adapter_voltage_present')),
                                                _yes(d.get('esp_adapter_connected')))
    if d.get('adapter_presence_source') is not None:
        adapter += ', source %s (%s), agreement %s (%s)' % (
            d.get('adapter_presence_source_name'), d.get('adapter_presence_source'),
            d.get('esp_adapter_disagree_name'), d.get('esp_adapter_disagree'))
    lines.append(('Adapter', adapter))
    charger = 'command %s, expected active %s, charger current %s' % (
        'on' if d.get('charger_enable_command') else 'off', _yes(d.get('charger_expected_active')),
        _fmt(d.get('charger_current_ma'), ' A', 0.001))
    if d.get('charge_stop_state') is not None:
        charger += ', charge stop %s (0x%02X%s)' % (
            d.get('charge_stop_state_name'), d['charge_stop_state'],
            ', policy on' if d.get('charge_stop_enabled') else ', policy compiled out')
    lines.append(('Charger', charger))
    lines.append(('Rails', 'NUC %s %s, 5V0 %s, 36VDD %s, 12V0 %s, 20V0 aux %s' % (
        _fmt(d.get('voltage_cpu_mv'), ' V', 0.001, 1), _fmt(d.get('current_cpu_ma'), ' A', 0.001),
        _fmt(d.get('voltage_5v0_mv'), ' V', 0.001), _fmt(d.get('voltage_36v0_mv'), ' V', 0.001, 1),
        _fmt(d.get('voltage_12v0_mv'), ' V', 0.001, 1),
        _fmt(d.get('voltage_20v0_aux_mv'), ' V', 0.001, 1))))
    if d.get('shutdown_wait_last_end') is not None:
        lines.append(('NUC shutdown wait', '%s, last end %s (%s), %s waits since boot, cap %s' % (
            'active %s' % _fmt(d.get('shutdown_wait_elapsed_ms'), ' s', 0.001, 1)
            if d.get('shutdown_wait_active') else 'idle',
            d.get('shutdown_wait_last_end_name'), d.get('shutdown_wait_last_end'),
            d.get('shutdown_wait_count'), _fmt(d.get('shutdown_wait_cap_ms'), ' s', 0.001, 0))))
        lines.append(('Shutdown command', 'wait_sd_ack %s, %s sends' % (
            _yes(d.get('wait_sd_ack')), d.get('esp_shutdown_cmd_sends'))))
    if d.get('adc_stale') is not None:
        reinit = d.get('adc_reinit_cnt') or [0, 0]
        lines.append(('ADC', 'stale mask 0x%X, rebuilds since boot ADC0 %d, ADC1 %d' % (
            d['adc_stale'], reinit[0], reinit[1])))
    lines.append(('PIMU', 'up %s, reset %s, loop max %s us, overruns %s, board %s' % (
        _fmt((d.get('uptime_ms') or 0) / 3600000.0, ' h', digits=1),
        ', '.join(d.get('reset_causes') or []) or 'n/a', d.get('loop_time_max_us'),
        d.get('loop_overrun_count'), _fmt(d.get('pimu_temp_c'), ' C', digits=1))))
    return lines


def recent_events(pp, count):
    log = pp.get('power_event_log') or {}
    events = list(log.get('events') or [])[-count:] if count > 0 else []
    out = []
    for e in events:
        item = dict(e)
        item['detail'] = describe_power_event(e)
        out.append(item)
    return out


def _event_time(e):
    wall = e.get('estimated_wall_time_s')
    if wall is not None:
        return datetime.datetime.fromtimestamp(wall).strftime('%H:%M:%S.%f')[:-3]
    return 'PIMU +%.3f s' % ((e.get('timestamp_ms') or 0) / 1000.0)


def render_report(pp, health, source, events=10, color=False, now_wall=None, robot=None):
    """The text report. pp is status['power_periph'], health its power_health."""
    meta = pp.get('power_diagnostics_meta') or {}
    diag = pp.get('power_diagnostics') or {}
    now_wall = time.time() if now_wall is None else now_wall
    head = [robot] if robot else []
    head += [source, datetime.datetime.fromtimestamp(now_wall).strftime('%Y-%m-%d %H:%M:%S')]
    out = ['Stretch power diagnostics, ' + ', '.join(head)]
    schema = diag.get('schema_version') if diag.get('_valid') else None
    out.append('PIMU firmware %s (%s), diagnostics schema %s, sample %s old' % (
        meta.get('firmware_version') or 'unknown', meta.get('protocol_version') or '?',
        schema if schema is not None else 'none', _fmt(meta.get('sample_age_s'), ' s', digits=1)))
    out.append('')
    level = health.get('level', 'unknown')
    out.append('Overall %s %s' % (_lamp(level, color), health.get('summary', '')))
    out.append('')
    width = max(len(t) for t in CHECK_TITLES.values())
    for check in health.get('checks', []):
        title = CHECK_TITLES.get(check['id'], check['id'])
        out.append('  %s %s  %s' % (_lamp(check['level'], color), title.ljust(width),
                                   check['message']))
    lines = key_values(pp)
    if lines:
        out.append('')
        out.append('Key values')
        label_width = max(len(k) for k, _ in lines)
        for label, text in lines:
            out.append('  %s  %s' % (label.ljust(label_width), text))
    log = pp.get('power_event_log') or {}
    ev = recent_events(pp, events)
    if events > 0:
        out.append('')
        held = len(log.get('events') or [])
        out.append('Last %d power events (of %d held, cursor %s, %s gaps, firmware drops %s)' % (
            len(ev), held, log.get('cursor_sequence'), log.get('coverage_gap_count', 0),
            log.get('firmware_dropped_count', 0)))
        if not ev:
            out.append('  none')
        for e in ev:
            out.append('  #%s  %s  %s  %s (%s): %s' % (
                e.get('sequence'), _event_time(e), e.get('power_state_name'),
                e.get('event_name'), e.get('event_id'), e['detail']))
    return '\n'.join(out) + '\n'


def report_dict(pp, health, source, events=10):
    """The --json form: health, key values, decoded recent events and the link meta."""
    return {
        'source': source,
        'health': health,
        'key_values': dict(key_values(pp)),
        'events': recent_events(pp, events),
        'power_diagnostics_meta': pp.get('power_diagnostics_meta') or {},
        'power_diagnostics': pp.get('power_diagnostics') or {},
    }


def this_robot():
    return os.environ.get('HELLO_FLEET_ID') or socket.gethostname()


def emit(pp, health, source, args, color):
    if args.json:
        print(json.dumps(report_dict(pp, health, source, args.events), indent=2, default=str))
    else:
        sys.stdout.write(render_report(pp, health, source, args.events, color,
                                       robot=this_robot()))


# ------------------------------------------------------------------------ modes

def run_from_log(args, color):
    samples, notes = load_log(args.from_log)
    for note in notes:
        print('note: %s' % note, file=sys.stderr)
    if not samples:
        print('No status sample with power_periph data in %s.' % args.from_log, file=sys.stderr)
        return 1
    at_s = parse_at(args.at, samples) if args.at else None
    index = select_sample(samples, at_s)
    health, how = health_for_log_sample(samples, index)
    t = samples[index]['timestamp']
    source = 'from %s, sample %d of %d, health %s' % (
        os.path.basename(args.from_log), index + 1, len(samples), how)
    if args.json:
        emit(samples[index]['power_periph'], health, source, args, color)
    else:
        sys.stdout.write(render_report(samples[index]['power_periph'], health, source,
                                       args.events, color, now_wall=t))
    return 0


def connect():
    """A started RobotClient, or None with the reason printed. Never opens a device."""
    from stretch4_body.robot.robot_client import RobotClient
    robot = RobotClient()
    if not robot.startup(verbose=False):
        print('stretch_body_server is not running (no answer on its admin socket), so '
              'there is no power status to read.\nStart it with: stretch_body_server --launch\n'
              'For a robot that is off, read a status log instead: '
              'stretch_power_diagnostics --from-log <status_*.json>', file=sys.stderr)
        return None
    return robot


def run_live(args, color):
    robot = connect()
    if robot is None:
        return 2
    evaluator = PowerHealthEvaluator()   # only used if the server predates power_health
    try:
        while True:
            robot.pull_status()
            pp = power_periph_of(robot.status) or {}
            if not pp:
                print('The server publishes no power_periph status (no PIMU configured).',
                      file=sys.stderr)
                return 2
            health = pp.get('power_health')
            source = 'live from stretch_body_server'
            if not health or not health.get('checks'):
                health = evaluator.evaluate(pp, time.monotonic())
                source += ' (health computed here: this server does not publish power_health)'
            if args.watch:
                sys.stdout.write('\033[2J\033[H')
            emit(pp, health, source, args, color)
            if not args.watch:
                return 0 if health.get('level') != 'fault' else 1
            sys.stdout.flush()
            time.sleep(args.interval)
    except KeyboardInterrupt:
        return 0
    finally:
        robot.stop()


def build_parser():
    parser = argparse.ArgumentParser(
        prog='stretch_power_diagnostics',
        description='Plain-language health of the PIMU power path (P14 diagnostics), read '
                    'from stretch_body_server or from a status log. Never opens a device.',
        epilog='Exit status: 0 ok, warn or unknown; 1 fault (single report); 2 no server.')
    parser.add_argument('--json', action='store_true', help='print the report as JSON')
    parser.add_argument('--watch', action='store_true', help='refresh until Ctrl-C')
    parser.add_argument('--interval', type=float, default=1.0,
                        help='seconds between refreshes with --watch (default 1)')
    parser.add_argument('-n', '--events', type=int, default=10,
                        help='how many recent power events to show (default 10, 0 for none)')
    parser.add_argument('--from-log', metavar='FILE',
                        help='read a SentryStatusLogger status_*.json or a stretch_status '
                             '--export zip instead of the live server')
    parser.add_argument('--at', metavar='TIME',
                        help='with --from-log: the sample at or before TIME, given as '
                             'HH:MM[:SS], +SECONDS from the start, -SECONDS from the end, or '
                             'epoch seconds (default: the last sample)')
    parser.add_argument('--no-color', action='store_true', help='plain text lamps')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.at and not args.from_log:
        build_parser().error('--at needs --from-log')
    color = sys.stdout.isatty() and not args.no_color and not args.json
    if args.from_log:
        return run_from_log(args, color)
    return run_live(args, color)


if __name__ == '__main__':
    sys.exit(main())
