"""
The black box: every number in the RobotClient status, kept 10 days on the NUC.

Each status sample is flattened into channels (dotted paths). Fast channels (motion,
currents, IMU, runstop) are written at 20 Hz; slow ones at most once a second and only
when they move past a deadband, plus once a minute so a flat channel still shows up.
Power events, journal lines and fault changes are events. Every 10 s the rows go to an
append-only chunk file; every hour the chunks of a closed hour are compacted into one
zstd Parquet file per table. Hour files past 10 days or past the size cap go first.

A ring holds the last 10 s of full-rate samples in memory, pickled, and only
channels due for writing are flattened. A trigger (a fault change,
an error event, a runstop, a server exception, a USB disconnect, a loop overrun or a
hand mark) writes the ring plus the next 10 s as a capture. One capture per trigger
kind per minute; repeats inside the minute are counted and written with the next one.
Captures have their own size cap.
"""
import collections
import json
import os
import pickle
import re
import time

FAST_RATE_HZ = 20.0
SLOW_RATE_HZ = 1.0
SLOW_KEEPALIVE_S = 60.0
DEADBAND_REL = 0.005
DEADBAND_ABS = 1e-3
CHUNK_PERIOD_S = 10.0
RING_S = 10.0
CAPTURE_AFTER_S = 10.0
CAPTURE_MAX_S = 60.0
TRIGGER_HOLDOFF_S = 60.0
KEEP_DAYS = 10
HOURS_MAX_BYTES = 4000 * 1024 * 1024
CAPTURES_MAX_BYTES = 1000 * 1024 * 1024

FAST_KEYS = ('pos', 'vel', 'effort', 'current', 'imu', 'accel', 'gyro', 'odom', 'runstop', 'force', 'torque')
SKIP_KEYS = ('power_event_log', 'transitions', 'firmware_versions', 'eye_frame')
SKIP_TOP = ('pimu', 'base')
SHARED = ('server', 'routines', 'safety_layer')
ERROR_EVENTS = ('bms_timeout', 'bms_crc_error', 'bms_rx_overflow', 'bms_alarm_changed', 'bms_protocol_error',
                'bms_decision_blocked', 'esp_status_stale', 'esp_crc_error', 'esp_rx_overflow', 'esp_tx_queue_drop',
                'esp_request_rejected', 'esp_rx_length_error', 'esp_trigger_dropped', 'runstop_raised',
                'charge_stop', 'adc_stale_changed', 'adapter_disagreement', 'eoa_trip', 'nuc_boot_watch',
                'low_battery_qualified')
BUTTON_FAULT_EVENT = 'button_faults'
SERVER_EXCEPTION = re.compile(r'Traceback|Exception|\bERROR\b|CRITICAL')
USB_TROUBLE = re.compile(r'USB disconnect|usb \S+: reset|cdc_acm.*(error|disconnect)|device descriptor read|'
                         r'over-current|unable to enumerate', re.I)
ANSI = re.compile(r'\x1b\[[0-9;]*m')


def flatten(status):
    """Dotted path -> float for every number and bool in a RobotClient status."""
    out = {}
    stack = []
    for k, v in status.items():
        if k in SKIP_TOP or k in SKIP_KEYS:
            continue
        if type(v) is dict:
            stack.extend((f'{k}.{kk}', vv) for kk, vv in v.items()
                         if kk not in SKIP_KEYS and (k in SHARED or kk not in SHARED))
        else:
            stack.append((k, v))
    while stack:
        path, o = stack.pop()
        t = type(o)
        if t is float or t is int:
            out[path] = float(o)
        elif t is bool:
            out[path] = 1.0 if o else 0.0
        elif t is dict:
            stack.extend((f'{path}.{k}', v) for k, v in o.items() if k not in SKIP_KEYS)
        elif (t is list or t is tuple) and len(o) <= 32:
            stack.extend((f'{path}.{i}', v) for i, v in enumerate(o))
    return out


def is_fast(channel):
    leaf = channel.lower().split('.')
    return any(key in part for part in leaf[1:] for key in FAST_KEYS)


def journal_text(entry):
    msg = entry.get('MESSAGE', '')
    if isinstance(msg, list):
        msg = bytes(msg).decode('utf-8', 'replace')
    return ANSI.sub('', msg)


def due(t, last, period):
    """(due, new last): keeps to the period without drifting, and catches up after a stall."""
    if t - last < period - 1e-6:
        return False, last
    return True, (last + period if t - last < 2 * period else t)


class Sampler:
    """Decides which channel values are written: fast at FAST_RATE_HZ, slow on change."""

    def __init__(self):
        self.last_fast = -1e9
        self.last_slow = -1e9
        self.written = {}
        self.written_t = {}
        self.fast = {}
        self.fast_list = []

    def rows(self, t, flat):
        fast_due, self.last_fast = due(t, self.last_fast, 1.0 / FAST_RATE_HZ)
        slow_due, self.last_slow = due(t, self.last_slow, 1.0 / SLOW_RATE_HZ)
        rows = []
        if slow_due:
            fast_list = []
            for ch, v in flat.items():
                if ch not in self.fast:
                    self.fast[ch] = is_fast(ch)
                if self.fast[ch]:
                    fast_list.append(ch)
                    continue
                old = self.written.get(ch)
                if (old is None or abs(v - old) > max(DEADBAND_ABS, DEADBAND_REL * abs(old))
                        or t - self.written_t[ch] >= SLOW_KEEPALIVE_S):
                    self.written[ch] = v
                    self.written_t[ch] = t
                    rows.append((t, ch, v))
            self.fast_list = fast_list
        if fast_due:
            rows.extend((t, ch, flat[ch]) for ch in self.fast_list if ch in flat)
        return rows


class TriggerGate:
    """One capture per trigger kind per TRIGGER_HOLDOFF_S; repeats are counted."""

    def __init__(self):
        self.last = {}
        self.repeats = collections.Counter()

    def offer(self, kind, t):
        if t - self.last.get(kind, -1e9) < TRIGGER_HOLDOFF_S:
            self.repeats[kind] += 1
            return None
        self.last[kind] = t
        return self.repeats.pop(kind, 0)


class Capture:
    def __init__(self, t, kind, detail, repeats, ring):
        self.t0 = t
        self.end = t + CAPTURE_AFTER_S
        self.triggers = [{'t': t, 'kind': kind, 'detail': detail, 'repeats_before': repeats}]
        self.samples = list(ring)

    def extend(self, t, kind, detail, repeats):
        self.end = min(max(self.end, t + CAPTURE_AFTER_S), self.t0 + CAPTURE_MAX_S)
        self.triggers.append({'t': t, 'kind': kind, 'detail': detail, 'repeats_before': repeats})


class Recorder:
    """Everything except the RobotClient and the journal readers, so tests drive it directly."""

    def __init__(self, root, writer):
        self.root = root
        self.writer = writer
        for d in ('chunks', 'hours', 'captures', 'marks'):
            os.makedirs(os.path.join(root, d), exist_ok=True)
        self.sampler = Sampler()
        self.gate = TriggerGate()
        self.ring = collections.deque()
        self.capture = None
        self.rows = []
        self.events = []
        self.last_chunk = None
        self.seen_events = collections.OrderedDict()
        self.last_flat = {}
        self.started = None

    def event(self, t, source, kind, detail, trigger=None):
        self.events.append((t, source, kind, detail))
        if trigger:
            self.trigger(t, trigger, f'{kind}: {detail}')

    def trigger(self, t, kind, detail=''):
        repeats = self.gate.offer(kind, t)
        if repeats is None:
            return
        if self.capture is not None and t <= self.capture.end:
            self.capture.extend(t, kind, detail, repeats)
        else:
            self.capture = Capture(t, kind, detail, repeats, self.ring)
        self.events.append((t, 'black_box', 'trigger', json.dumps({'kind': kind, 'detail': detail,
                                                                  'repeats_before': repeats})))

    def sample(self, t, status):
        if self.started is None:
            self.started = t
        packed = (t, pickle.dumps(status, protocol=pickle.HIGHEST_PROTOCOL))
        self.ring.append(packed)
        while self.ring and t - self.ring[0][0] > RING_S:
            self.ring.popleft()
        if self.capture is not None:
            self.capture.samples.append(packed)
        if t - self.sampler.last_fast < 1.0 / FAST_RATE_HZ - 1e-6:
            return
        flat = flatten(status)
        self.rows.extend(self.sampler.rows(t, flat))
        self._power_events(t, status)
        self._edges(t, flat)
        self.last_flat = flat

    def _power_events(self, t, status):
        log = status.get('power_periph', {}).get('power_event_log') or {}
        for e in log.get('events') or []:
            key = (e.get('sequence'), e.get('timestamp_ms'), e.get('event_id'))
            if key in self.seen_events:
                continue
            self.seen_events[key] = True
            if len(self.seen_events) > 512:
                self.seen_events.popitem(last=False)
            name = e.get('event_name', str(e.get('event_id')))
            et = e.get('estimated_wall_time_s') or t
            if et < self.started - 5:
                trigger = None
            elif name == BUTTON_FAULT_EVENT:
                trigger = 'fault_change'
            elif name in ERROR_EVENTS:
                trigger = name
            else:
                trigger = None
            self.event(et, 'pimu', name, json.dumps(e, default=str), trigger)

    def _edges(self, t, flat):
        old = self.last_flat
        for ch in ('power_periph.runstop_event', 'power_periph.power_diagnostics.loop_overrun_count'):
            if ch in flat and ch in old and flat[ch] > old[ch]:
                self.trigger(t, 'runstop' if 'runstop' in ch else 'pimu_loop_overrun', f'{ch} {flat[ch]:g}')
        rate = flat.get('server.control_loop.curr_rate_hz')
        if rate is not None and 0 < rate < 50:
            self.trigger(t, 'server_loop_overrun', f'control loop at {rate:.1f} Hz')

    def journal(self, t, source, entry):
        text = journal_text(entry)
        if source == 'kernel':
            if USB_TROUBLE.search(text):
                self.event(t, 'kernel', 'usb', text, 'usb_trouble')
            elif int(entry.get('PRIORITY', 6)) <= 3:
                self.event(t, 'kernel', 'error', text, 'kernel_error')
        elif SERVER_EXCEPTION.search(text):
            self.event(t, 'server', 'error', text, 'server_exception')
        elif int(entry.get('PRIORITY', 6)) <= 4:
            self.event(t, 'server', 'warning', text)

    def poll_marks(self, t):
        d = os.path.join(self.root, 'marks')
        for f in sorted(os.listdir(d)):
            p = os.path.join(d, f)
            try:
                text = open(p).read().strip()
                os.remove(p)
            except OSError:
                continue
            self.event(t, 'host', 'mark', text, 'hand_mark')

    def tick(self, t):
        """Flushes chunks, closes captures and compacts closed hours; call about once a second."""
        if self.last_chunk is None:
            self.last_chunk = t
        if t - self.last_chunk >= CHUNK_PERIOD_S:
            self.flush(t)
        if self.capture is not None and t > self.capture.end:
            self.writer.write_capture(os.path.join(self.root, 'captures'), self.capture)
            self.capture = None
            prune(os.path.join(self.root, 'captures'), CAPTURES_MAX_BYTES, None, t)

    def flush(self, t):
        self.last_chunk = t
        if self.rows or self.events:
            self.writer.write_chunk(os.path.join(self.root, 'chunks'), t, self.rows, self.events)
        self.rows = []
        self.events = []
        self.writer.compact(os.path.join(self.root, 'chunks'), os.path.join(self.root, 'hours'), t)
        prune(os.path.join(self.root, 'hours'), HOURS_MAX_BYTES, KEEP_DAYS * 86400, t)


def hour_of(t):
    return time.strftime('%Y%m%d_%H', time.localtime(t))


def prune(directory, max_bytes, max_age_s, now):
    """Deletes the oldest files (by name, which starts with the time) past the age or size cap."""
    files = sorted(f for f in os.listdir(directory) if not f.startswith('.'))
    sizes = {f: os.path.getsize(os.path.join(directory, f)) for f in files}
    total = sum(sizes.values())
    for f in files:
        too_old = max_age_s is not None and now - os.path.getmtime(os.path.join(directory, f)) > max_age_s
        if not too_old and total <= max_bytes:
            break
        os.remove(os.path.join(directory, f))
        total -= sizes[f]


class ParquetWriter:
    """Chunks, hour files and captures as zstd Parquet, each written to a temp name then renamed."""

    def __init__(self):
        import pyarrow
        import pyarrow.parquet
        self.pa = pyarrow
        self.pq = pyarrow.parquet

    def _write(self, table, path, metadata=None):
        if metadata:
            table = table.replace_schema_metadata(metadata)
        tmp = os.path.join(os.path.dirname(path), '.' + os.path.basename(path))
        self.pq.write_table(table, tmp, compression='zstd')
        os.replace(tmp, path)

    def _samples(self, rows):
        t, ch, v = zip(*rows) if rows else ((), (), ())
        return self.pa.table({'t': self.pa.array(t, self.pa.float64()),
                              'channel': self.pa.array(ch, self.pa.string()),
                              'value': self.pa.array(v, self.pa.float64())})

    def _events(self, events):
        t, src, kind, detail = zip(*events) if events else ((), (), (), ())
        return self.pa.table({'t': self.pa.array(t, self.pa.float64()), 'source': self.pa.array(src, self.pa.string()),
                              'kind': self.pa.array(kind, self.pa.string()),
                              'detail': self.pa.array(detail, self.pa.string())})

    def write_chunk(self, directory, t, rows, events):
        stem = f'{hour_of(t)}_{t:.3f}'
        samples = self._samples(rows).sort_by([('channel', 'ascending'), ('t', 'ascending')])
        self._write(samples, os.path.join(directory, stem + '.samples.parquet'))
        self._write(self._events(events), os.path.join(directory, stem + '.events.parquet'))

    def _stream(self, tables, path, schema, metadata=None):
        """Writes one row group per table, so memory stays at one table, not the whole file."""
        if metadata:
            schema = schema.with_metadata(metadata)
        tmp = os.path.join(os.path.dirname(path), '.' + os.path.basename(path))
        with self.pq.ParquetWriter(tmp, schema, compression='zstd') as w:
            for table in tables:
                w.write_table(table.cast(schema) if table.schema != schema else table)
        os.replace(tmp, path)
        self.pa.default_memory_pool().release_unused()

    def compact(self, chunks, hours, now):
        current = hour_of(now)
        for kind in ('samples', 'events'):
            by_hour = collections.defaultdict(list)
            for f in sorted(os.listdir(chunks)):
                if f.endswith(f'.{kind}.parquet') and not f.startswith('.') and f[:11] < current:
                    by_hour[f[:11]].append(os.path.join(chunks, f))
            for hour, files in by_hour.items():
                out = os.path.join(hours, f'{hour}.{kind}.parquet')
                n = 1
                while os.path.exists(out):
                    out = os.path.join(hours, f'{hour}_{n}.{kind}.parquet')
                    n += 1
                schema = (self._samples([]) if kind == 'samples' else self._events([])).schema
                self._stream((self.pq.read_table(f) for f in files), out, schema)
                for f in files:
                    os.remove(f)

    def write_capture(self, directory, capture):
        def batches(size=100):
            for i in range(0, len(capture.samples), size):
                yield self._samples([(t, ch, v) for t, raw in capture.samples[i:i + size]
                                     for ch, v in flatten(pickle.loads(raw)).items()])
        kind = capture.triggers[0]['kind']
        name = f"{time.strftime('%Y%m%d_%H%M%S', time.localtime(capture.t0))}_{kind}.parquet"
        self._stream(batches(), os.path.join(directory, name), self._samples([]).schema,
                     {'triggers': json.dumps(capture.triggers)})
