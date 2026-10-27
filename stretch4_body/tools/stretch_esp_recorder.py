#!/usr/bin/env python3
"""
stretch_esp_recorder: the PIMU's flight recorder, read back from the ESP32.

The PIMU sends the ESP a 16-byte power record 20 times a second (200 for 2 s after a
button edge, a state transition, a runstop or a fault change) and one for every power
event; the ESP keeps the last 2 MB of them in flash (hello_esp FlightRecorder). This tool
asks the ESP console on /dev/hello-esp32 for the last N seconds and writes them as a
status log that stretch_power_diagnostics --from-log replays, one sample per record. It
uses the ESP's own USB port at the console baud rate only, so the robot stack keeps
running and the PIMU is never opened. Never put esptool or a 1200 baud touch on this
port: that reboots the ESP into its ROM loader, its pins float and the PIMU power path
follows them.
"""
import argparse
import datetime
import json
import struct
import sys
import time

from stretch4_body.core.status_log import StatusLogTrimmer
from stretch4_body.subsystem.power_health import describe_power_event
from stretch4_body.subsystem.power_periph import PowerPeriphAux

PORT = '/dev/hello-esp32'
BAUD = 115200
RECORD_SIZE = 16
# FlightRecord.h: ms, sequence, state, kind, event_id, faults, five union bytes, crc
HEAD = struct.Struct('<IBBBBH')
SAMPLE = struct.Struct('<Hh')
EVENT = struct.Struct('<hh')
KIND_NAMES = {0: 'sample', 1: 'burst', 2: 'event'}
KIND_EVENT = 2
STATE_MASK = 0x07
PWR_EN = 0x08
BUTTON = 0x10
ROBOT_ACTIVE = 0x20
NUC_5V = 0x40
RUNSTOP = 0x80
RUNSTOP_CAUSES = ('none', 'button', 'python', 'low_soc', 'high_current', 'high_current_eoa')


def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def decode(raw):
    """One record as a dict, or None for a torn one (bad CRC) or an erased slot."""
    if raw == b'\xff' * RECORD_SIZE or crc8(raw[:15]) != raw[15]:
        return None
    ms, sequence, state, kind, event_id, faults = HEAD.unpack_from(raw)
    r = {
        'ms': ms, 'sequence': sequence, 'kind': kind & 0x0F,
        'power_state': state & STATE_MASK, 'pwr_en': bool(state & PWR_EN),
        'button': bool(state & BUTTON), 'robot_active': bool(state & ROBOT_ACTIVE),
        'nuc_5v': bool(state & NUC_5V), 'runstop': bool(state & RUNSTOP),
        'runstop_cause': kind >> 4, 'faults': faults,
    }
    if r['kind'] == KIND_EVENT:
        r['event_id'] = event_id
        r['data0'], r['data1'] = EVENT.unpack_from(raw, HEAD.size)
    else:
        dv, da = SAMPLE.unpack_from(raw, HEAD.size)
        r['bus_v'] = dv / 10.0
        r['bus_a'] = da / 10.0
    return r


# ------------------------------------------------------------------------ the console

def read_dump(port):
    """
    The records of a dump already asked for, oldest first, torn ones left out. The ESP
    writes "FREC <n>" and then n raw records, and "FREC 0" at the end; any other line is
    console chatter and is skipped.
    """
    records = []
    while True:
        line = port.read_until(b'\n')
        if not line:
            raise IOError('the ESP stopped answering mid dump (%d records so far)' % len(records))
        if not line.startswith(b'FREC '):
            continue
        n = int(line[5:].strip())
        if n == 0:
            return records
        raw = port.read(n * RECORD_SIZE)
        if len(raw) != n * RECORD_SIZE:
            raise IOError('short read inside a dump block')
        for i in range(n):
            r = decode(raw[i * RECORD_SIZE:(i + 1) * RECORD_SIZE])
            if r is not None:
                records.append(r)


def read_line(port, prefix):
    while True:
        line = port.read_until(b'\n')
        if not line:
            raise IOError('no answer from the ESP console')
        if line.startswith(prefix):
            return line.decode('ascii', errors='replace').strip()


def open_port(path):
    import serial
    return serial.Serial(path, BAUD, timeout=5)


# ------------------------------------------------------------------------ the log

def wall_times(records, dump_wall_s):
    """
    Estimated wall time of each record, counting back from the dump: PIMU milliseconds
    between records, and nothing across a PIMU reboot (the ms count falls).
    """
    out = [0.0] * len(records)
    elapsed = 0.0
    for i in range(len(records) - 1, -1, -1):
        if i + 1 < len(records) and records[i + 1]['ms'] >= records[i]['ms']:
            elapsed += (records[i + 1]['ms'] - records[i]['ms']) / 1000.0
        out[i] = dump_wall_s - elapsed
    return out


def event_dict(r, sequence, wall_s):
    return {
        'sequence': sequence,
        'timestamp_ms': r['ms'],
        'timestamp_s': r['ms'] / 1000.0,
        'event_id': r['event_id'],
        'event_name': PowerPeriphAux.POWER_EVENT_NAMES.get(r['event_id'], 'unknown_%d' % r['event_id']),
        'power_state': r['power_state'],
        'power_state_name': PowerPeriphAux.get_power_state_name(r['power_state']),
        'data0': r['data0'],
        'data1': r['data1'],
        'estimated_wall_time_s': wall_s,
    }


def samples_of(records, dump_wall_s, port=PORT):
    """
    One logged status sample per record, in the SentryStatusLogger shape that
    stretch_power_diagnostics --from-log reads. The bus readings and the state bits go in
    power_diagnostics; the power events accumulate into power_event_log as the PIMU's
    own log would hold them.
    """
    times = wall_times(records, dump_wall_s)
    events = []
    bus_v = bus_a = None
    trimmer = StatusLogTrimmer()
    samples = []
    for i, r in enumerate(records):
        if r['kind'] == KIND_EVENT:
            events.append(event_dict(r, len(events) + 1, times[i]))
        else:
            bus_v, bus_a = r['bus_v'], r['bus_a']
        diag = {
            '_valid': True,
            'schema_version': None,
            'uptime_ms': r['ms'],
            'power_state': r['power_state'],
            'power_state_name': PowerPeriphAux.get_power_state_name(r['power_state']),
            'power_button_raw': int(r['pwr_en']),
            'power_button_debounced': int(r['button']),
            'robot_active': int(r['robot_active']),
            'cpu_on_status': int(r['nuc_5v']),
            'runstop_event': int(r['runstop']),
            'runstop_cause_raw': r['runstop_cause'],
            'runstop_cause_name': cause_name(r['runstop_cause']),
            'button_fault_mask': r['faults'],
            'battery_voltage_mv': None if bus_v is None else int(round(bus_v * 1000)),
            'battery_current_ma': None if bus_a is None else int(round(bus_a * 1000)),
            'record_kind': KIND_NAMES.get(r['kind'], 'unknown_%d' % r['kind']),
            'record_sequence': r['sequence'],
        }
        meta = {
            'available': True, 'sample_seq': i + 1, 'sample_age_s': 0.0,
            'firmware_version': None, 'protocol_version': None,
            'source': 'esp flight recorder on %s' % port,
        }
        log = {
            'available': True, 'events': events[-64:], 'cursor_sequence': len(events),
            'firmware_dropped_count': 0, 'coverage_gap_count': 0, 'coverage_gaps': [],
        }
        status = {'timestamp': times[i],
                  'power_periph': {'power_diagnostics': diag, 'power_diagnostics_meta': meta,
                                   'power_event_log': log}}
        samples.append(trimmer.trim(status))
    return samples


def cause_name(cause):
    return RUNSTOP_CAUSES[cause] if cause < len(RUNSTOP_CAUSES) else 'unknown_%d' % cause


def record_line(r, wall_s):
    when = datetime.datetime.fromtimestamp(wall_s).strftime('%H:%M:%S.%f')[:-3]
    head = '%s  PIMU +%10.3f s  #%3d  %-6s %-19s' % (
        when, r['ms'] / 1000.0, r['sequence'], KIND_NAMES.get(r['kind'], '?'),
        PowerPeriphAux.get_power_state_name(r['power_state']))
    bits = ' '.join(name for key, name in (('pwr_en', 'PWR_EN'), ('button', 'BUTTON'),
                                           ('robot_active', 'ROBOT_ACTIVE'), ('nuc_5v', 'NUC_5V'))
                    if r[key])
    tail = 'faults 0x%04X' % r['faults']
    if r['runstop']:
        tail = 'runstop %s, ' % cause_name(r['runstop_cause']) + tail
    if r['kind'] == KIND_EVENT:
        e = event_dict(r, 0, wall_s)
        return '%s  %s (%d): %s  [%s; %s]' % (head, e['event_name'], e['event_id'],
                                             describe_power_event(e), bits, tail)
    return '%s  %5.1f V %6.1f A  [%s; %s]' % (head, r['bus_v'], r['bus_a'], bits, tail)


# ------------------------------------------------------------------------ main

def build_parser():
    parser = argparse.ArgumentParser(
        prog='stretch_esp_recorder',
        description='Read the PIMU flight recorder held by the ESP32 into a status log '
                    'for stretch_power_diagnostics --from-log.')
    parser.add_argument('--port', default=PORT, help='the ESP USB console (default %s)' % PORT)
    parser.add_argument('--seconds', type=int, default=600,
                        help='how far back, in PIMU time (default 600, 0 for everything held)')
    parser.add_argument('--out', metavar='FILE',
                        help='the log to write (default esp_recorder_<date>.json)')
    parser.add_argument('--print', action='store_true', help='list the records as text too')
    parser.add_argument('--stat', action='store_true', help="print the ESP's recorder status only")
    parser.add_argument('--clear', action='store_true', help='erase the recorder and exit')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    port = open_port(args.port)
    try:
        if args.stat:
            port.write(b'frec stat\n')
            print(read_line(port, b'frec:'))
            return 0
        if args.clear:
            port.write(b'frec clear\n')
            port.timeout = 60
            print(read_line(port, b'frec: cleared'))
            return 0
        port.write(b'frec dump %d\n' % args.seconds)
        dump_wall_s = time.time()
        records = read_dump(port)
    finally:
        port.close()
    if not records:
        print('The ESP holds no records.', file=sys.stderr)
        return 1
    out = args.out or datetime.datetime.fromtimestamp(dump_wall_s).strftime(
        'esp_recorder_%Y%m%d_%H%M%S.json')
    samples = samples_of(records, dump_wall_s, args.port)
    with open(out, 'w') as f:
        json.dump(samples, f)
    events = sum(1 for r in records if r['kind'] == KIND_EVENT)
    if args.print:
        times = wall_times(records, dump_wall_s)
        for r, t in zip(records, times):
            print(record_line(r, t))
    print('%d records (%d power events) spanning %.1f s of PIMU time written to %s' % (
        len(records), events, dump_wall_s - wall_times(records, dump_wall_s)[0], out))
    print('Replay: stretch_power_diagnostics --from-log %s' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
