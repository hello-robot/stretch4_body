"""Black box recorder: sampling, triggers and captures, chunks, compaction and rotation.

No server and no device: the recorder is driven with status dicts, and the Parquet
writer works in a temp directory.
"""
import json
import os
import time

import pyarrow.parquet as pq
import pytest

from stretch4_body.core import black_box as bb

T0 = time.mktime((2026, 11, 5, 20, 59, 0, 0, 0, -1))


def status(current=1.0, voltage=24.0, runstop=False, overruns=0, rate=100.0, events=()):
    return {
        'server': {'control_loop': {'curr_rate_hz': rate}},
        'pimu': {'voltage': voltage},
        'power_periph': {
            'voltage': voltage, 'current': current, 'runstop_event': runstop,
            'server': {'control_loop': {'curr_rate_hz': rate}},
            'power_diagnostics': {'loop_overrun_count': overruns},
            'power_event_log': {'events': list(events)},
        },
    }


def power_event(seq, name, t):
    return {'sequence': seq, 'timestamp_ms': seq * 10, 'event_id': seq, 'event_name': name,
            'estimated_wall_time_s': t, 'detail': name}


class FakeWriter:
    def __init__(self):
        self.chunks = []
        self.captures = []

    def write_chunk(self, directory, t, rows, events):
        self.chunks.append((t, list(rows), list(events)))

    def compact(self, chunks, hours, now):
        pass

    def write_capture(self, directory, capture):
        self.captures.append(capture)


@pytest.fixture
def rec(tmp_path):
    return bb.Recorder(str(tmp_path), FakeWriter())


def run(rec, t0, seconds, hz=100, **kw):
    for i in range(int(seconds * hz)):
        t = t0 + i / hz
        rec.sample(t, status(**kw))
        rec.tick(t)
    return t0 + seconds


def test_flatten_drops_aliases_shared_dicts_and_event_log():
    flat = bb.flatten(status(events=[power_event(1, 'boot', T0)]))
    assert flat['power_periph.current'] == 1.0
    assert flat['server.control_loop.curr_rate_hz'] == 100.0
    assert flat['power_periph.runstop_event'] == 0.0
    assert not any(k.startswith('pimu.') for k in flat)
    assert not any(k.startswith('power_periph.server') for k in flat)
    assert not any('power_event_log' in k for k in flat)


def test_fast_channels_at_20_hz_slow_channels_on_change_and_keepalive():
    s = bb.Sampler()
    rows = []
    for i in range(200):
        t = T0 + i / 100
        rows += s.rows(t, {'power_periph.current': float(i), 'power_periph.voltage': 24.0})
    fast = [r for r in rows if r[1] == 'power_periph.current']
    slow = [r for r in rows if r[1] == 'power_periph.voltage']
    assert len(fast) == 40
    assert len(slow) == 1
    assert s.rows(T0 + 3.0, {'power_periph.voltage': 24.0001}) == []
    assert s.rows(T0 + 4.0, {'power_periph.voltage': 25.0}) == [(T0 + 4.0, 'power_periph.voltage', 25.0)]
    assert s.rows(T0 + 4.0 + bb.SLOW_KEEPALIVE_S, {'power_periph.voltage': 25.0})


def test_gate_allows_one_trigger_per_kind_per_minute_and_counts_repeats():
    g = bb.TriggerGate()
    assert g.offer('bms_crc_error', T0) == 0
    assert g.offer('bms_crc_error', T0 + 5) is None
    assert g.offer('bms_crc_error', T0 + 30) is None
    assert g.offer('usb_trouble', T0 + 31) == 0
    assert g.offer('bms_crc_error', T0 + 61) == 2


def test_capture_holds_ring_before_and_window_after_trigger(rec):
    t = run(rec, T0, 15)
    rec.trigger(t, 'hand_mark', 'look')
    assert rec.writer.captures == []
    run(rec, t, 11)
    cap, = rec.writer.captures
    times = [s[0] for s in cap.samples]
    assert t - bb.RING_S - 0.02 <= times[0] <= t - bb.RING_S + 0.02
    assert times[-1] >= t + bb.CAPTURE_AFTER_S
    assert cap.triggers[0]['kind'] == 'hand_mark'


def test_overlapping_triggers_merge_into_one_capture(rec):
    run(rec, T0, 2)
    rec.trigger(T0 + 2, 'hand_mark')
    run(rec, T0 + 2, 5)
    rec.trigger(T0 + 7, 'usb_trouble', 'USB disconnect')
    run(rec, T0 + 7, 11)
    cap, = rec.writer.captures
    assert [x['kind'] for x in cap.triggers] == ['hand_mark', 'usb_trouble']
    assert cap.end == T0 + 17


def test_power_events_dedupe_and_trigger_on_errors_and_fault_changes(rec):
    evs = [power_event(1, 'boot', T0), power_event(2, 'bms_crc_error', T0 + 1),
           power_event(3, 'bms_recovered', T0 + 2), power_event(4, 'button_faults', T0 + 3)]
    rec.sample(T0, status(events=evs))
    rec.sample(T0 + 0.01, status(events=evs))
    kinds = [e[2] for e in rec.events if e[1] == 'pimu']
    assert kinds == ['boot', 'bms_crc_error', 'bms_recovered', 'button_faults']
    trig = [json.loads(e[3])['kind'] for e in rec.events if e[2] == 'trigger']
    assert trig == ['bms_crc_error', 'fault_change']


def test_runstop_overrun_journal_and_mark_triggers(rec):
    rec.sample(T0, status())
    rec.sample(T0 + 0.05, status(runstop=True, overruns=1, rate=30.0))
    rec.journal(T0 + 1, 'kernel', {'MESSAGE': 'usb 3-2: USB disconnect, device number 7', 'PRIORITY': '6'})
    rec.journal(T0 + 1, 'server', {'MESSAGE': list(b'\x1b[31mTraceback (most recent call last):'), 'PRIORITY': '3'})
    with open(os.path.join(rec.root, 'marks', '1'), 'w') as f:
        f.write('lamp went red')
    rec.poll_marks(T0 + 2)
    trig = [json.loads(e[3])['kind'] for e in rec.events if e[2] == 'trigger']
    assert trig == ['runstop', 'pimu_loop_overrun', 'server_loop_overrun', 'usb_trouble',
                    'server_exception', 'hand_mark']
    assert os.listdir(os.path.join(rec.root, 'marks')) == []


def test_chunks_every_10_s(rec):
    run(rec, T0, 25)
    assert [round(c[0] - T0) for c in rec.writer.chunks] == [10, 20]


def test_parquet_chunks_compact_by_hour_and_capture_carries_triggers(tmp_path):
    r = bb.Recorder(str(tmp_path), bb.ParquetWriter())
    run(r, T0, 50, hz=20)
    r.trigger(T0 + 50, 'hand_mark', 'look')
    run(r, T0 + 50, 70, hz=20)
    r.flush(T0 + 120)
    hours = sorted(os.listdir(tmp_path / 'hours'))
    assert hours == ['20261105_20.events.parquet', '20261105_20.samples.parquet']
    assert all(f.startswith('20261105_21') for f in os.listdir(tmp_path / 'chunks'))
    samples = pq.read_table(tmp_path / 'hours' / '20261105_20.samples.parquet')
    keys = list(zip(samples['channel'].to_pylist(), samples['t'].to_pylist()))
    assert keys == sorted(keys) and all(t < T0 + 60 for _, t in keys)
    cap, = os.listdir(tmp_path / 'captures')
    meta = json.loads(pq.read_schema(tmp_path / 'captures' / cap).metadata[b'triggers'])
    assert meta[0]['kind'] == 'hand_mark'


def test_prune_by_size_then_age(tmp_path):
    for i in range(5):
        p = tmp_path / f'2026110{i}_00.samples.parquet'
        p.write_bytes(b'x' * 100)
        os.utime(p, (T0 - (5 - i) * 86400,) * 2)
    bb.prune(str(tmp_path), 300, None, T0)
    assert len(os.listdir(tmp_path)) == 3
    bb.prune(str(tmp_path), 10 ** 9, 2.5 * 86400, T0)
    assert sorted(os.listdir(tmp_path)) == ['20261103_00.samples.parquet', '20261104_00.samples.parquet']


def test_events_already_in_the_log_at_start_are_recorded_without_a_capture(rec):
    rec.sample(T0 + 100, status(events=[power_event(1, 'bms_crc_error', T0)]))
    assert [e[2] for e in rec.events] == ['bms_crc_error']
