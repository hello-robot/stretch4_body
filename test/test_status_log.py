"""SentryStatusLogger file format: trimming in the logger, reconstruction in the readers.

Nothing here opens a device or needs a server: the logger daemon runs against a fake
RobotClient that updates its status dicts in place, the way the real one does.
"""
import json
import logging
import multiprocessing
import os
import sys
import zipfile
from pathlib import Path

import pytest

from stretch4_body.core import status_log
from stretch4_body.core.status_log import StatusLogExpander, StatusLogTrimmer
from stretch4_body.behavior.sentries import sentry_status_logger as sl
from stretch4_body.tools import stretch_power_diagnostics as cli
from test_stretch_power_diagnostics import BOOT, LENGTH_ERROR, REALISTIC, T0
from test_power_health import evaluate, status


HEALTH = {}
JUNE = Path(__file__).parent / 'fixtures' / 'status_sample_4032_20260602.json'


def transition(seq, to_state):
    return {'seq': seq, 'observed_wall_time_s': T0 + seq, 'observed_monotonic_s': 100.0 + seq,
            'from_state': None if seq == 1 else 0, 'to_state': to_state,
            'to_state_name': 'STATE_ACTIVE', 'runstop_cause_raw': 0}


def share_server_dicts(st):
    """Give every subsystem the top-level server, routines and safety_layer objects,
    as RobotClient.pull_status does."""
    for sub, value in st.items():
        if isinstance(value, dict) and sub not in status_log.SHARED and sub not in status_log.ALIASES:
            for key in status_log.SHARED:
                value[key] = st[key]
    return st


def robot_status(i, cursor=9, seq=1, errors=0, transitions=1):
    """A RobotClient-shaped status: power keys, the legacy aliases and the per-subsystem
    server dicts, sharing objects as RobotClient does. power_health is evaluated once per
    diagnostics sample, as the server does at 2 Hz."""
    pp = status(meta={'sample_seq': seq, 'request_errors': errors},
                log={'events': [BOOT, LENGTH_ERROR] if cursor >= 10 else [BOOT],
                     'cursor_sequence': cursor}, **REALISTIC)
    if (seq, errors) not in HEALTH:
        HEALTH[(seq, errors)] = evaluate(pp)
    pp['power_health'] = json.loads(json.dumps(HEALTH[(seq, errors)]))
    pp['voltage'] = 27.0 + 0.01 * i
    pp['status_aux_meta'] = {'available': True, 'sample_seq': seq, 'sample_age_s': 0.01 * i,
                             'transitions': [transition(n + 1, n % 2) for n in range(transitions)]}
    omnibase = {'x': 0.001 * i, 'y': 0.0}
    st = {'timestamp': T0 + 0.01 * i,
          'server': {'control_loop': {'num_loops': 1000 + i}, 'status_id': i},
          'routines': {'last_routine_id': 3},
          'safety_layer': {'in_safety_event': False, 'n': i},
          'power_periph': pp, 'omnibase': omnibase, 'lift': {'pos': 0.5 + 0.001 * i}}
    share_server_dicts(st)
    st['pimu'] = pp
    st['base'] = omnibase
    return st


def sequence():
    """Twelve statuses at 100 Hz: one new 2 Hz diagnostics sample, one new event, and one
    request error with no new sample (the meta changes while sample_seq does not)."""
    out = []
    for i in range(12):
        out.append(robot_status(i, cursor=10 if i >= 7 else 9, seq=2 if i >= 4 else 1,
                                errors=1 if i >= 9 else 0, transitions=2 if i >= 5 else 1))
    return out


def as_logged(sample):
    return json.loads(json.dumps(sample, default=str))



# --- the logger side ----------------------------------------------------------------------------

def test_first_sample_has_every_value_once():
    s = robot_status(0)
    out = StatusLogTrimmer().trim(s)
    assert 'pimu' not in out and 'base' not in out
    assert status_log.FORMAT_KEY not in out
    for key in status_log.SHARED:
        assert out[key] == s[key]
        assert key not in out['lift'] and key not in out['power_periph']
    assert sorted(out[status_log.SHARED_KEY]) == ['lift', 'omnibase', 'power_periph']
    pp = {k: v for k, v in s['power_periph'].items() if k not in status_log.SHARED}
    assert out['power_periph'] == pp


def test_unchanged_power_keys_are_left_out_and_listed():
    t = StatusLogTrimmer()
    t.trim(robot_status(0))
    out = t.trim(robot_status(1))
    pp = out['power_periph']
    for key in ('power_event_log', 'power_diagnostics', 'power_diagnostics_meta', 'power_health'):
        assert key not in pp
    assert 'transitions' not in pp['status_aux_meta']
    assert pp['status_aux_meta']['sample_age_s'] == pytest.approx(0.01)
    assert sorted(out[status_log.FORMAT_KEY]['power_periph']) == sorted(
        ['power_event_log', 'power_diagnostics', 'power_diagnostics_meta', 'power_health',
         'status_aux_meta.transitions'])
    assert pp['voltage'] == pytest.approx(27.01)       # fast keys always written
    assert out['lift'] == {'pos': pytest.approx(0.501)}


def test_changed_keys_are_written_again():
    t = StatusLogTrimmer()
    t.trim(robot_status(0, cursor=9, seq=1))
    out = t.trim(robot_status(1, cursor=10, seq=1))
    assert out['power_periph']['power_event_log']['cursor_sequence'] == 10
    assert 'power_event_log' not in out[status_log.FORMAT_KEY]['power_periph']
    out = t.trim(robot_status(2, cursor=10, seq=1, errors=3))
    assert out['power_periph']['power_diagnostics_meta']['request_errors'] == 3
    out = t.trim(robot_status(3, cursor=10, seq=1, errors=3, transitions=2))
    assert len(out['power_periph']['status_aux_meta']['transitions']) == 2


def test_a_subsystem_server_copy_that_differs_is_kept():
    s = robot_status(0)
    s['lift']['server'] = {'control_loop': {'num_loops': 1}, 'status_id': 0}
    out = StatusLogTrimmer().trim(s)
    assert out['lift']['server'] == {'control_loop': {'num_loops': 1}, 'status_id': 0}
    assert 'lift' not in out[status_log.SHARED_KEY]
    assert StatusLogExpander().expand(json.loads(json.dumps(out))) == as_logged(s)


def test_trim_never_touches_the_published_status():
    t = StatusLogTrimmer()
    t.trim(robot_status(0))
    s = robot_status(1)
    before = as_logged(s)
    t.trim(s)
    assert as_logged(s) == before
    assert s['pimu'] is s['power_periph'] and s['base'] is s['omnibase']


def test_reset_makes_the_next_sample_complete():
    t = StatusLogTrimmer()
    t.trim(robot_status(0))
    t.reset()
    out = t.trim(robot_status(1))
    assert status_log.FORMAT_KEY not in out
    assert 'power_event_log' in out['power_periph']


def test_a_real_key_named_base_is_kept():
    s = robot_status(0)
    s['base'] = {'x': 1.0}           # not an alias: a different object
    assert 'base' in StatusLogTrimmer().trim(s)


# --- the reader side ----------------------------------------------------------------------------

def logged_file(statuses):
    t = StatusLogTrimmer()
    return '[' + ', '.join(json.dumps(t.trim(s), default=str) for s in statuses) + ']'


def test_expander_rebuilds_every_sample_exactly():
    statuses = sequence()
    ex = StatusLogExpander()
    rebuilt = [ex.expand(e) for e in json.loads(logged_file(statuses))]
    assert rebuilt == [as_logged(s) for s in statuses]
    assert ex.unresolved == 0


def test_expander_leaves_old_files_alone():
    old = [as_logged(s) for s in sequence()]
    ex = StatusLogExpander()
    assert [ex.expand(json.loads(json.dumps(e))) for e in old] == old


def test_expander_counts_keys_it_cannot_fill():
    """A damaged file can lose its first, complete sample."""
    elements = json.loads(logged_file(sequence()))[1:]
    ex = StatusLogExpander()
    rebuilt = [ex.expand(e) for e in elements]
    assert ex.unresolved > 0
    assert 'power_event_log' not in rebuilt[0]['power_periph']
    assert rebuilt[-1]['power_periph']['power_event_log']['cursor_sequence'] == 10


def june_sample():
    return json.loads(JUNE.read_text())


def june_like_live(i, n_events=64):
    """The June sample as RobotClient holds it now: subsystem server dicts are the
    top-level ones (the June logger wrote them stale), aliases are the same objects, and
    the P14 power keys are present with a full event log. i ticks the fast values."""
    st = june_sample()
    st['timestamp'] += 0.01 * i
    st['server']['status_id'] += i
    st['lift']['pos'] += 0.0001 * i
    p14 = robot_status(i, cursor=9 + n_events, seq=1 + i // 50,
                       transitions=1 + i // 200)['power_periph']
    p14['power_event_log']['events'] = [dict(BOOT, sequence=10 + n) for n in range(n_events)]
    for key in status_log.SHARED:
        p14.pop(key)
    st['power_periph'].update(p14)
    share_server_dicts(st)
    st['pimu'] = st['power_periph']
    st['base'] = st['omnibase']
    return st


def test_june_sample_round_trips_exactly_as_logged():
    """The real June sample as written then: stale subsystem server copies stay written."""
    st = june_sample()
    st['pimu'] = st['power_periph']
    st['base'] = st['omnibase']
    out = json.loads(json.dumps(StatusLogTrimmer().trim(st), default=str))
    assert StatusLogExpander().expand(out) == june_sample()


def test_june_sample_as_live_round_trips_exactly_and_shrinks():
    statuses = [june_like_live(i) for i in range(0, 600, 7)]
    text = logged_file(statuses)
    ex = StatusLogExpander()
    rebuilt = [ex.expand(e) for e in json.loads(text)]
    assert rebuilt == [as_logged(s) for s in statuses]
    assert ex.unresolved == 0
    elements = json.loads(text)
    later = len(json.dumps(elements[-1]))
    assert later < len(json.dumps(as_logged(statuses[-1]))) / 5
    assert 'server' not in elements[-1]['lift']


def test_trimmed_sample_is_much_smaller():
    t = StatusLogTrimmer()
    full = len(json.dumps(robot_status(0), default=str))
    t.trim(robot_status(0))
    trimmed = len(json.dumps(t.trim(robot_status(1)), default=str))
    assert trimmed < full / 4


def test_power_diagnostics_reads_a_trimmed_log(tmp_path):
    statuses = sequence()
    path = tmp_path / 'status_20261016_120000.json'
    path.write_text(logged_file(statuses))
    samples, notes = cli.load_log(str(path))
    assert notes == []
    assert len(samples) == 12
    for s, orig in zip(samples, statuses):
        assert s['power_periph'] == as_logged(orig['power_periph'])
    assert samples[-1]['power_periph']['power_event_log']['cursor_sequence'] == 10


def test_power_diagnostics_reports_a_trimmed_log_that_lost_its_first_sample(tmp_path):
    text = logged_file(sequence())
    second = text.index('{"timestamp"', 10)
    path = tmp_path / 'status_mid.json'
    path.write_text(text[second + 5:])
    samples, notes = cli.load_log(str(path))
    assert samples
    assert any('carried' in n for n in notes)


def test_stretch_status_import_replays_full_samples(tmp_path, monkeypatch, capsys):
    from stretch4_body.tools import stretch_status
    zpath = tmp_path / 'export.zip'
    statuses = sequence()
    with zipfile.ZipFile(zpath, 'w') as zf:
        zf.writestr('status_20261016_120000.json', logged_file(statuses[:6]))
        zf.writestr('status_20261016_120100.json', logged_file(statuses[6:]))
    monkeypatch.setattr(sys, 'argv', [
        'stretch_status', '--import', str(zpath), '--fields',
        'robot.pimu.power_event_log.cursor_sequence',
        'robot.power_periph.power_diagnostics_meta.request_errors'])
    stretch_status.main()
    out = capsys.readouterr().out
    assert out.count('=== Status ===') == 12
    assert out.count('cursor_sequence: 9') == 7 and out.count('cursor_sequence: 10') == 5
    assert out.count('request_errors: 1') == 3
    assert 'Error reading' not in out


# --- the logger daemon --------------------------------------------------------------------------

class FakeClient:
    """Updates nested status dicts in place on every pull, like RobotClient."""

    def __init__(self, stop_after, do_exit):
        self.stop_after, self.do_exit, self.pulls = stop_after, do_exit, 0
        first = robot_status(0)
        self.status = {k: first[k] for k in ('timestamp', 'server', 'routines', 'safety_layer',
                                             'power_periph', 'omnibase', 'lift')}
        self.status['pimu'] = self.status['power_periph']
        self.status['base'] = self.status['omnibase']

    def startup(self):
        return True

    def pull_status(self):
        i = self.pulls
        new = robot_status(i, cursor=10 if i >= 7 else 9, seq=2 if i >= 4 else 1)
        for key in ('timestamp',) + status_log.SHARED:
            self.status[key] = new[key]
        for sub in ('power_periph', 'omnibase', 'lift'):
            self.status[sub].update(new[sub])
        self.pulls += 1
        if self.pulls >= self.stop_after:
            self.do_exit.set()

    def stop(self):
        pass


def run_daemon(tmp_path, pulls, batch_period_s):
    do_exit = multiprocessing.Event()
    client = FakeClient(pulls, do_exit)
    sl.writer_daemon(str(tmp_path), 1000.0, do_exit, 5000, logging.getLogger('test'),
                     batch_period_s=batch_period_s, client_factory=lambda: client)
    return sorted(p for p in os.listdir(tmp_path))


def read_all(tmp_path, names):
    out = []
    for name in names:
        ex = StatusLogExpander()
        out.append([ex.expand(e) for e in json.loads((tmp_path / name).read_text())])
    return out


def test_daemon_logs_each_sample_as_it_was_polled(tmp_path):
    """The old logger kept references to the live dicts and wrote every sample of a batch
    with the values of the last one."""
    names = run_daemon(tmp_path, 12, 60.0)
    assert len(names) == 1 and names[0].startswith('status_') and names[0].endswith('.json')
    (samples,) = read_all(tmp_path, names)
    assert len(samples) == 12
    assert [s['lift']['pos'] for s in samples] == pytest.approx([0.5 + 0.001 * i for i in range(12)])
    assert [s['pimu']['power_event_log']['cursor_sequence'] for s in samples] == [9] * 7 + [10] * 5


def test_daemon_starts_every_file_complete_and_never_overwrites(tmp_path):
    names = run_daemon(tmp_path, 5, 0.0)
    assert not [n for n in names if not n.endswith('.json')]
    files = read_all(tmp_path, names)
    assert sum(len(f) for f in files) == 5
    for name in names:
        first = json.loads((tmp_path / name).read_text())[0]
        assert status_log.FORMAT_KEY not in first
        assert 'power_event_log' in first['power_periph']


def test_write_batch_is_atomic_and_unique(tmp_path):
    a = sl.write_batch(str(tmp_path), ['{"a": 1}'], stamp='20261016_120000')
    b = sl.write_batch(str(tmp_path), ['{"b": 2}', '{"c": 3}'], stamp='20261016_120000')
    assert os.path.basename(a) == 'status_20261016_120000.json'
    assert os.path.basename(b) == 'status_20261016_120000_1.json'
    assert json.loads(open(a).read()) == [{'a': 1}]
    assert json.loads(open(b).read()) == [{'b': 2}, {'c': 3}]
    assert sorted(os.listdir(tmp_path)) == ['status_20261016_120000.json',
                                             'status_20261016_120000_1.json']


def test_failed_write_leaves_no_partial_file(tmp_path, monkeypatch):
    def boom(*a, **k):
        raise OSError('disk full')
    monkeypatch.setattr(sl.os, 'fsync', boom)
    with pytest.raises(OSError):
        sl.write_batch(str(tmp_path), ['{"a": 1}'], stamp='20261016_120000')
    assert os.listdir(tmp_path) == []


def test_stale_temp_files_are_removed_at_start(tmp_path):
    stale = tmp_path / 'status_20260602_191038.json.1234.tmp'
    stale.write_text('[{"partial')
    os.utime(stale, (0, 0))
    run_daemon(tmp_path, 2, 60.0)
    assert not stale.exists()
