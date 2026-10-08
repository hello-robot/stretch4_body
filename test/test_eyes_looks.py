import copy
import json
import os
import signal
import subprocess
import sys
import threading
import time

import pytest

import stretch4_body.eyes.api as eyes_api
from stretch4_body.eyes import Eyes, Library, Sequence, Step
from stretch4_body.eyes.backends import FakeBackend, SENTRY_NAME
from stretch4_body.eyes.looks import BUILTIN_DIR, FORMAT, user_dir

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILTINS = ['attention', 'glance', 'happy', 'idle', 'off', 'sleepy', 'thinking']
TOLERANCE_S = 0.05


def look(name='quick', steps=None, **kwargs):
    d = {'format': FORMAT, 'name': name,
         'steps': steps or [{'left': 'look_left', 'right': 'look_left', 'color': '#00a0ff', 'intensity': 0.5, 'hold': 0.1},
                            {'left': 'look_right', 'right': 'look_right', 'hold': 0.1}]}
    d.update(kwargs)
    return d


def write_look(directory, d):
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, d['name'] + '.json')
    with open(path, 'w') as f:
        json.dump(d, f)
    return path


@pytest.fixture
def dirs(tmp_path, monkeypatch):
    """A user dir under a temp fleet and two shared dirs, never the real fleet dir."""
    d = {'fleet': tmp_path / 'fleet', 'user': str(tmp_path / 'fleet' / 'stretch-se4-0000' / 'eyes'),
         'shared1': str(tmp_path / 'shared1'), 'shared2': str(tmp_path / 'shared2')}
    monkeypatch.setenv('HELLO_FLEET_PATH', str(d['fleet']))
    monkeypatch.setenv('HELLO_FLEET_ID', 'stretch-se4-0000')
    monkeypatch.setenv('STRETCH_EYES_LIBRARY', os.pathsep.join([d['shared1'], d['shared2']]))
    return d


# ---------------- File format ----------------

def test_step_normalises_values():
    s = Step(left='LOOK-LEFT', right=5, color=[1, 2, 3], intensity=0.5, hold=2)
    assert (s.left, s.right, s.color, s.intensity, s.hold) == ('look_left', 'look_right', (1, 2, 3), 128, 2.0)
    assert s.to_dict() == {'left': 'look_left', 'right': 'look_right', 'color': '#010203', 'intensity': 128, 'hold': 2.0}
    assert Step(hold=0.05).to_dict() == {'hold': 0.05}


@pytest.mark.parametrize('kwargs,field', [
    (dict(hold=0.01), 'hold'), (dict(hold=601), 'hold'), (dict(hold=True), 'hold'), (dict(hold='1'), 'hold'),
    (dict(left='wink'), 'left'), (dict(right=0), 'right'), (dict(color='mauve'), 'color'),
    (dict(intensity=1.5), 'intensity'), (dict(intensity=256), 'intensity'),
])
def test_step_errors_name_the_field(kwargs, field):
    with pytest.raises(ValueError, match='^' + field + ': '):
        Step(**kwargs)


DELETE = object()


def _mutate(path, value):
    """look() with the value at path ('steps.1.hold') replaced, or deleted when value is DELETE."""
    d = copy.deepcopy(look(steps=[{'left': 'blink', 'hold': 0.1}, {'right': 'blink', 'hold': 0.1},
                                  {'color': 'red', 'hold': 0.1}]))
    keys = [int(k) if k.isdigit() else k for k in path.split('.')]
    parent = d
    for k in keys[:-1]:
        parent = parent[k]
    if value is DELETE:
        del parent[keys[-1]]
    else:
        parent[keys[-1]] = value
    return d


@pytest.mark.parametrize('path,value,field', [
    ('format', DELETE, 'format'), ('format', 'stretch-eyes/2', 'format'),
    ('name', DELETE, 'name'), ('name', 'Glance!', 'name'), ('name', 'x' * 49, 'name'), ('name', '-x', 'name'),
    ('name', 'nl\n', 'name'),
    ('title', 5, 'title'), ('author', None, 'author'), ('note', ['a'], 'note'), ('loop', 'yes', 'loop'),
    ('steps', DELETE, 'steps'), ('steps', [], 'steps'), ('steps', 'blink', 'steps'),
    ('steps', [{'hold': 0.1}] * 201, 'steps'),
    ('colour', 'red', 'colour'),
    ('steps.1', 'blink', r'steps\[1\]'),
    ('steps.1.hold', DELETE, r'steps\[1\]\.hold'), ('steps.2.hold', 0.01, r'steps\[2\]\.hold'),
    ('steps.0.left', 'wink', r'steps\[0\]\.left'), ('steps.1.right', 99, r'steps\[1\]\.right'),
    ('steps.2.color', 'mauve', r'steps\[2\]\.color'), ('steps.0.intensity', 1.5, r'steps\[0\]\.intensity'),
    ('steps.0.colour', 'red', r'steps\[0\]\.colour'),
])
def test_sequence_errors_name_the_field(path, value, field):
    with pytest.raises(ValueError, match='^' + field + ': '):
        Sequence.from_dict(_mutate(path, value))


def test_sequence_defaults_and_duration():
    seq = Sequence('quick', [Step(left='blink', hold=0.5), {'hold': 1.25}])
    assert (seq.title, seq.author, seq.note, seq.loop) == ('quick', '', '', False)
    assert seq.duration == 1.75
    assert seq.to_dict()['steps'] == [{'left': 'blink', 'hold': 0.5}, {'hold': 1.25}]


def test_x_keys_are_kept():
    d = look(**{'x-studio': {'grid': 4}})
    d['steps'][1]['x-label'] = 'right'
    seq = Sequence.from_dict(d)
    out = seq.to_dict()
    assert out['x-studio'] == {'grid': 4}
    assert out['steps'][1]['x-label'] == 'right'
    assert Sequence.from_dict(out) == seq


def test_extra_takes_only_x_keys():
    with pytest.raises(ValueError, match="^extra: 'name'"):
        Sequence('ok', [Step(hold=1)], extra={'name': '../../evil'})
    with pytest.raises(ValueError, match="^extra: 'hold'"):
        Step(hold=1, extra={'hold': 0})
    assert Step(hold=1, extra={'x-a': 1}).to_dict() == {'hold': 1.0, 'x-a': 1}


def test_save_is_canonical_and_round_trips(tmp_path):
    loose = look(steps=[{'left': 'LOOK_LEFT', 'color': 'hot pink', 'intensity': 0.5, 'hold': 1},
                        {'color': [1, 2, 3], 'intensity': 7, 'hold': 0.25}], loop=True, author='me')
    src = write_look(str(tmp_path / 'in'), loose)
    seq = Sequence.load(src)
    out = str(tmp_path / 'quick.json')
    assert seq.save(out) == out
    with open(out) as f:
        text = f.read()
    assert text == seq.to_json()
    saved = json.loads(text)
    assert saved['steps'] == [{'left': 'look_left', 'color': '#ff69b4', 'intensity': 128, 'hold': 1.0},
                              {'color': '#010203', 'intensity': 7, 'hold': 0.25}]
    assert list(saved) == ['format', 'name', 'title', 'author', 'note', 'loop', 'steps']
    again = Sequence.load(out)
    assert again == seq
    again.save(out)
    with open(out) as f:
        assert f.read() == text
    assert sorted(os.listdir(str(tmp_path))) == ['in', 'quick.json']  # no temp file left behind
    assert os.stat(out).st_mode & 0o777 == 0o644


def test_a_failed_save_leaves_no_temp_file(tmp_path, monkeypatch):
    seq = Sequence.from_dict(look())
    monkeypatch.setattr(Sequence, 'to_json', lambda self: 1 / 0)
    with pytest.raises(ZeroDivisionError):
        seq.save(str(tmp_path / 'quick.json'))
    assert os.listdir(str(tmp_path)) == []


def test_saves_from_threads_do_not_collide(dirs):
    lib, errors = Library(), []

    def save(title):
        for _ in range(50):
            try:
                lib.save(look('race', title=title), overwrite=True)
            except Exception as e:
                errors.append(e)
    threads = [threading.Thread(target=save, args=('t{}'.format(i),)) for i in range(4)]
    [t.start() for t in threads]
    [t.join() for t in threads]
    assert errors == []
    assert lib.get('race').title in ('t0', 't1', 't2', 't3')
    assert os.listdir(dirs['user']) == ['race.json']


def test_save_without_overwrite_is_one_step(dirs):
    """Two saves of a new name at once without overwrite: one wins, the other raises."""
    lib, results = Library(), []
    barrier = threading.Barrier(2)

    def save(title):
        barrier.wait()
        try:
            results.append(lib.save(look('new', title=title)))
        except FileExistsError as e:
            results.append(e)
    threads = [threading.Thread(target=save, args=(t,)) for t in ('a', 'b')]
    [t.start() for t in threads]
    [t.join() for t in threads]
    assert sorted(type(r).__name__ for r in results) == ['FileExistsError', 'str']
    err = [r for r in results if isinstance(r, FileExistsError)][0]
    assert err.filename == os.path.join(dirs['user'], 'new.json') and 'overwrite' in str(err)
    assert os.listdir(dirs['user']) == ['new.json']


def test_load_names_the_file(tmp_path):
    bad = tmp_path / 'bad.json'
    bad.write_text('{"format": ')
    with pytest.raises(ValueError, match='bad.json: not JSON'):
        Sequence.load(str(bad))
    bad.write_text(json.dumps(look(steps=[{'hold': 0}])))
    with pytest.raises(ValueError, match=r'bad.json: steps\[0\]\.hold'):
        Sequence.load(str(bad))


# ---------------- Library ----------------

def test_user_dir_follows_the_fleet_env(tmp_path, monkeypatch):
    monkeypatch.setenv('HELLO_FLEET_PATH', '/fleet')
    monkeypatch.setenv('HELLO_FLEET_ID', 'stretch-se4-0001')
    assert user_dir() == os.path.join('/fleet', 'stretch-se4-0001', 'eyes')
    monkeypatch.delenv('HELLO_FLEET_ID')
    monkeypatch.setenv('HOME', str(tmp_path))
    assert user_dir() == os.path.join(str(tmp_path), 'stretch_user', 'eyes')


def test_builtins_are_listed_with_no_user_or_shared_looks(dirs):
    entries = Library().list()
    assert [e['name'] for e in entries] == BUILTINS
    assert {e['source'] for e in entries} == {'builtin'}
    assert not any(e['shadowed'] for e in entries)
    glance = entries[BUILTINS.index('glance')]
    assert (glance['steps'], glance['duration'], glance['loop']) == (3, 2.5, False)
    assert glance['path'] == os.path.join(BUILTIN_DIR, 'glance.json')


def test_search_order_and_shadowing(dirs):
    write_look(dirs['shared2'], look('glance', title='shared2'))
    write_look(dirs['shared1'], look('glance', title='shared1'))
    write_look(dirs['shared2'], look('wave', title='only shared2'))
    lib = Library()
    assert lib.get('glance').title == 'shared1'
    write_look(dirs['user'], look('glance', title='mine'))
    entries = {e['name']: e for e in lib.list()}
    assert (entries['glance']['source'], entries['glance']['title'], entries['glance']['shadowed']) == ('user', 'mine', True)
    assert (entries['wave']['source'], entries['wave']['shadowed']) == ('shared', False)
    assert entries['happy']['source'] == 'builtin'
    assert [e['source'] for e in lib.list()] == ['user', 'shared'] + ['builtin'] * (len(BUILTINS) - 1)
    lib.delete('glance')
    assert lib.get('glance').title == 'shared1'
    assert {e['name']: e for e in lib.list()}['glance']['shadowed'] is True


def test_paths_replace_the_shared_env(dirs, tmp_path):
    write_look(dirs['shared1'], look('wave', title='env'))
    write_look(str(tmp_path / 'other'), look('wave', title='other'))
    assert Library().get('wave').title == 'env'
    lib = Library(paths=[tmp_path / 'other'])
    assert lib.get('wave').title == 'other'
    assert [s for s, d in lib.dirs()] == ['user', 'shared', 'builtin']


def test_invalid_and_misnamed_files_are_skipped_and_reported(dirs):
    write_look(dirs['user'], look('glance', steps=[{'hold': 0}]))
    with open(os.path.join(dirs['user'], 'other.json'), 'w') as f:
        json.dump(look('wave'), f)
    lib = Library()
    assert lib.get('glance').title == 'Glance left and right'  # falls through to the built-in
    assert 'wave' not in [e['name'] for e in lib.list()]
    errors = {os.path.basename(e['path']): e['error'] for e in lib.errors()}
    assert 'steps[0].hold' in errors['glance.json']
    assert 'does not match the file name' in errors['other.json']


def test_scan_all_matches_list_and_errors(dirs):
    write_look(dirs['user'], look('glance', title='mine'))
    write_look(dirs['user'], look('broken', steps=[{'hold': 0}]))
    lib = Library()
    entries, errors, seqs = lib.scan_all()
    assert entries == lib.list() and errors == lib.errors()
    assert sorted(seqs) == sorted(e['name'] for e in entries)
    assert seqs['glance'].title == 'mine'  # the first match, as get() returns
    assert seqs['thinking'] == lib.get('thinking')


def test_get_unknown_lists_the_names(dirs):
    with pytest.raises(KeyError, match='glance, happy'):
        Library().get('nope')


def test_save_writes_the_user_dir_only(dirs):
    lib = Library()
    path = lib.save(Sequence.from_dict(look()))
    assert path == os.path.join(dirs['user'], 'quick.json')
    with pytest.raises(FileExistsError, match='overwrite'):
        lib.save(look(title='second'))
    assert lib.save(look(title='second'), overwrite=True) == path
    assert lib.get('quick').title == 'second'
    assert lib.save(look('glance')) == os.path.join(dirs['user'], 'glance.json')  # shadows the built-in
    assert not os.path.exists(dirs['shared1']) and not os.path.exists(dirs['shared2'])
    assert sorted(os.listdir(dirs['user'])) == ['glance.json', 'quick.json']
    with pytest.raises(ValueError, match='name'):
        lib.save(look('Bad Name'))


def test_delete_is_user_only(dirs):
    write_look(dirs['shared1'], look('wave'))
    lib = Library()
    lib.save(look())
    lib.delete('quick')
    assert not os.path.exists(os.path.join(dirs['user'], 'quick.json'))
    with pytest.raises(PermissionError, match='shared'):
        lib.delete('wave')
    with pytest.raises(PermissionError, match='builtin'):
        lib.delete('glance')
    with pytest.raises(KeyError):
        lib.delete('quick')
    assert os.path.exists(os.path.join(BUILTIN_DIR, 'glance.json'))


def test_import_and_export(dirs, tmp_path):
    lib = Library()
    src = write_look(str(tmp_path / 'downloads'), look(steps=[{'left': 'blink', 'color': 'red', 'hold': 0.5}]))
    seq = lib.import_file(src)
    assert seq.name == 'quick'
    assert lib.export('quick') == seq.to_dict()
    assert lib.export('quick')['steps'] == [{'left': 'blink', 'color': '#ff0000', 'hold': 0.5}]
    with pytest.raises(FileExistsError):
        lib.import_file(src)
    lib.import_file(src, overwrite=True)
    bad = write_look(str(tmp_path / 'downloads'), look('broken', steps=[{'left': 'wink', 'hold': 1}]))
    with pytest.raises(ValueError, match=r'steps\[0\]\.left'):
        lib.import_file(bad)
    assert os.listdir(dirs['user']) == ['quick.json']
    assert lib.export('glance') == Sequence.load(os.path.join(BUILTIN_DIR, 'glance.json')).to_dict()


@pytest.mark.parametrize('name', BUILTINS)
def test_builtin_validates_and_is_canonical(name):
    path = os.path.join(BUILTIN_DIR, name + '.json')
    seq = Sequence.load(path)
    assert seq.name == name
    with open(path) as f:
        assert f.read() == seq.to_json()


def test_builtin_set_is_the_shipped_set():
    assert sorted(n[:-5] for n in os.listdir(BUILTIN_DIR) if n.endswith('.json')) == BUILTINS


# ---------------- Playback on the fake backend ----------------

class TimedBackend(FakeBackend):
    """FakeBackend that records when each payload was sent and every sentry change."""
    def __init__(self):
        super().__init__()
        self.times = []
        self.sentry_calls = []

    def send(self, payload):
        self.times.append(time.monotonic())
        return super().send(payload)

    def set_sentry(self, active, timeout=1.0):
        self.sentry_calls.append(active)
        return super().set_sentry(active, timeout)


def timed_eyes():
    return Eyes(backend=TimedBackend())


def long_look(name='long'):
    return look(name, steps=[{'left': 'blink', 'color': 'blue', 'hold': 5.0}, {'left': 'happy', 'hold': 5.0}])


def test_play_sends_steps_in_order_with_their_holds():
    eyes = timed_eyes()
    seq = Sequence.from_dict(look(steps=[{'left': 'look_left', 'right': 'look_left', 'color': '#00a0ff', 'intensity': 0.5, 'hold': 0.1},
                                         {'left': 'look_right', 'right': 'look_right', 'hold': 0.2},
                                         {'color': 'red', 'hold': 0.1}]))
    t0 = time.monotonic()
    assert eyes.play(seq) is None
    assert time.monotonic() - t0 < TOLERANCE_S  # returns at once
    assert eyes.wait(2.0)
    elapsed = time.monotonic() - t0
    assert eyes.backend.sent == [(4, 4, 128, 0, 160, 255), (5, 5, 128, 0, 160, 255), (5, 5, 128, 255, 0, 0)]
    offsets = [t - eyes.backend.times[0] for t in eyes.backend.times]
    for got, want in zip(offsets, [0.0, 0.1, 0.3]):
        assert abs(got - want) < TOLERANCE_S
    assert abs(elapsed - seq.duration) < TOLERANCE_S
    assert eyes.playing is None and eyes.state().playing is None


def test_playing_reports_progress():
    eyes = timed_eyes()
    eyes.play(look(steps=[{'left': 'blink', 'hold': 0.15}, {'left': 'happy', 'hold': 0.15}]))
    first = eyes.playing
    assert {k: first[k] for k in ('name', 'step', 'steps', 'loop')} == {'name': 'quick', 'step': 0, 'steps': 2, 'loop': False}
    assert abs(first['started'] - time.time()) < 1.0
    time.sleep(0.22)
    assert eyes.state().playing['step'] == 1
    assert eyes.state().to_dict()['playing']['name'] == 'quick'
    eyes.wait()
    assert eyes.playing is None


def test_loop_repeats_until_stopped():
    eyes = timed_eyes()
    seq = Sequence.from_dict(look(steps=[{'left': 'blink', 'hold': 0.05}, {'left': 'happy', 'hold': 0.05}], loop=True))
    eyes.play(seq)
    assert eyes.playing['loop'] is True
    time.sleep(0.32)
    assert eyes.stop() is True
    ids = [p[0] for p in eyes.backend.sent]
    assert len(ids) >= 6 and ids[:6] == [3, 8, 3, 8, 3, 8]
    offsets = [t - eyes.backend.times[0] for t in eyes.backend.times]
    assert abs(offsets[5] - 0.25) < TOLERANCE_S  # held from a schedule, so passes do not drift


def test_loop_argument_overrides_the_file():
    eyes = timed_eyes()
    eyes.play(look(steps=[{'left': 'blink', 'hold': 0.05}], loop=True), loop=False)
    assert eyes.wait(1.0)
    assert len(eyes.backend.sent) == 1
    eyes.play(look(steps=[{'left': 'blink', 'hold': 0.05}]), loop=True)
    assert not eyes.wait(0.2)
    eyes.stop()
    assert len(eyes.backend.sent) >= 4


def test_stop_mid_hold_returns_fast_and_sends_nothing_more():
    eyes = timed_eyes()
    eyes.play(long_look())
    time.sleep(0.05)
    t0 = time.monotonic()
    assert eyes.stop(restore=False) is True
    assert time.monotonic() - t0 < TOLERANCE_S
    assert eyes.playing is None
    time.sleep(0.1)
    assert len(eyes.backend.sent) == 1
    assert eyes.stop() is True  # nothing playing


IDLE_PAYLOAD = (2, 2, 255, 40, 48, 60)


@pytest.mark.parametrize('before,restored', [
    (None, IDLE_PAYLOAD),                                                      # nothing commanded: idle
    (dict(left='happy', right='alert', color='red', intensity=100), (8, 7, 100, 255, 0, 0)),
    (dict(color='red'), (2, 2, 255, 255, 0, 0)),                               # eyes never commanded go to idle
])
def test_stop_puts_back_the_look_from_before_play(before, restored):
    eyes = timed_eyes()
    if before:
        eyes.set(**before)
    eyes.play(long_look())
    time.sleep(0.05)
    assert eyes.stop() is True
    assert eyes.backend.sent[-1] == restored
    s = eyes.state()
    assert (s.playing, s.intensity, s.color) == (None, restored[2], restored[3:])
    assert eyes.backend.sentry_active is True
    time.sleep(0.1)
    assert eyes.backend.sent[-1] == restored  # nothing after the restore


def test_stop_after_a_replaced_playback_restores_the_first_look():
    eyes = timed_eyes()
    eyes.set(left='happy', right='happy', color='green')
    eyes.play(long_look('first'))
    time.sleep(0.05)
    eyes.play(long_look('second'))
    time.sleep(0.05)
    eyes.stop()
    assert eyes.backend.sent[-1] == (8, 8, 255, 0, 255, 0)


def test_a_sequence_that_ends_stays_on_its_last_step():
    eyes = timed_eyes()
    eyes.set(left='happy', right='happy')
    eyes.play(look(steps=[{'left': 'blink', 'right': 'blink', 'hold': 0.05}]))
    assert eyes.wait(1.0)
    assert eyes.stop() is True and eyes.close() is True
    assert eyes.backend.sent[-1][:2] == (3, 3)


def test_a_stall_skips_the_missed_steps_instead_of_sending_them_back_to_back():
    eyes = timed_eyes()
    send, calls = eyes.backend.send, []

    def stalls_once(payload):
        calls.append(payload)
        if len(calls) == 3:
            time.sleep(0.5)
        return send(payload)
    eyes.backend.send = stalls_once
    eyes.play(look(steps=[{'left': 'blink', 'hold': 0.1}, {'left': 'happy', 'hold': 0.1}], loop=True))
    time.sleep(1.3)
    eyes.stop(restore=False)
    times = eyes.backend.times
    assert len(times) >= 6
    gaps = [b - a for a, b in zip(times[2:], times[3:])]  # from the stalled step on
    assert min(gaps) > 0.1 - TOLERANCE_S, gaps


def test_play_and_stop_from_two_threads():
    eyes = timed_eyes()
    errors = []

    def guard(fn):
        try:
            fn()
        except Exception as e:
            errors.append(e)
    for _ in range(200):
        threads = [threading.Thread(target=guard, args=(lambda: eyes.play(long_look()),)),
                   threading.Thread(target=guard, args=(eyes.stop,))]
        [t.start() for t in threads]
        [t.join() for t in threads]
    assert errors == []
    assert eyes.stop() is True
    assert (eyes.playing, eyes.backend.sentry_active, eyes.state().in_control) == (None, True, False)


def test_exit_mid_sequence_restores_and_resumes_the_sentry(dirs):
    """A script that plays and exits: the atexit hook stops the look and resumes the sentry."""
    code = '\n'.join([
        'import atexit',
        'from stretch4_body.eyes import Eyes',
        'atexit.register(lambda: print(eyes.backend.sentry_active, eyes.backend.sent[-1], eyes.state().in_control))',
        "eyes = Eyes(backend='fake')",
        "eyes.play('glance')",
    ])
    run = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, cwd=REPO, timeout=30)
    assert run.returncode == 0, run.stderr
    assert run.stdout.strip() == 'True {} False'.format(IDLE_PAYLOAD)


@pytest.mark.parametrize('call,payload', [
    (lambda e: e.set(color='red'), (3, 0, 255, 255, 0, 0)),
    (lambda e: e.off(), (1, 1, 255, 0, 0, 255)),
    (lambda e: e.idle(), (2, 2, 255, 40, 48, 60)),
])
def test_a_direct_command_stops_playback_first(call, payload):
    eyes = timed_eyes()
    eyes.play(long_look())
    time.sleep(0.05)
    t0 = time.monotonic()
    call(eyes)
    assert time.monotonic() - t0 < TOLERANCE_S
    assert eyes.playing is None
    time.sleep(0.1)
    assert eyes.backend.sent[-1] == payload and len(eyes.backend.sent) == 2


def test_an_invalid_set_leaves_playback_running():
    eyes = timed_eyes()
    eyes.play(long_look())
    with pytest.raises(ValueError):
        eyes.set(color='mauve')
    assert eyes.playing['name'] == 'long'
    eyes.stop()


def test_play_takes_and_releases_control():
    eyes = timed_eyes()
    eyes.play(look())
    assert eyes.backend.sentry_active is False
    assert eyes.state().in_control
    eyes.wait()
    assert eyes.backend.sentry_active is True
    assert not eyes.state().in_control
    assert eyes.backend.sentry_calls == [False, True]


def test_play_nests_with_control_already_held():
    eyes = timed_eyes()
    with eyes.take_control():
        eyes.play(look())
        eyes.wait()
        assert eyes.backend.sentry_active is False  # still ours
        assert eyes.state().in_control
    assert eyes.backend.sentry_active is True
    assert eyes.backend.sentry_calls == [False, True]


def test_replacing_a_playback_keeps_the_sentry_paused():
    eyes = timed_eyes()
    eyes.play(long_look('first'))
    eyes.play(long_look('second'))
    assert eyes.playing['name'] == 'second'
    assert eyes.backend.sentry_calls == [False]
    eyes.stop()
    assert eyes.backend.sentry_calls == [False, True]
    assert not eyes.state().in_control


def test_close_stops_playback():
    eyes = timed_eyes()
    eyes.play(long_look())
    t0 = time.monotonic()
    assert eyes.close() is True
    assert time.monotonic() - t0 < TOLERANCE_S
    assert (eyes.playing, eyes.backend.sentry_active, eyes.backend.connected) == (None, True, False)
    assert eyes.backend.sent[-1] == IDLE_PAYLOAD


def test_play_by_library_name(dirs):
    eyes = timed_eyes()
    Library().save(look('mine', steps=[{'left': 'blink', 'color': 'green', 'hold': 0.05}]))
    eyes.play('mine')
    eyes.wait()
    assert eyes.backend.sent == [(3, 0, 255, 0, 255, 0)]
    eyes.play('off')
    eyes.wait()
    assert eyes.backend.sent[-1][:2] == (1, 1)


@pytest.mark.parametrize('seq,error', [('nope', KeyError), (look(steps=[{'hold': 0}]), ValueError), (42, ValueError)])
def test_a_bad_sequence_plays_nothing(dirs, seq, error):
    eyes = timed_eyes()
    with pytest.raises(error):
        eyes.play(seq)
    assert eyes.backend.sentry_calls == [] and eyes.playing is None


def test_stop_is_bounded_when_a_send_blocks(monkeypatch):
    """A step stuck in a server push holds the lock; stop() from another thread still returns."""
    monkeypatch.setattr(eyes_api, 'STOP_PLAYBACK_S', 0.2)
    eyes = timed_eyes()
    gate = threading.Event()
    send = eyes.backend.send
    eyes.backend.send = lambda payload: gate.wait(5) and send(payload)
    eyes.play(long_look())
    time.sleep(0.05)
    t0 = time.monotonic()
    assert eyes.stop() is False
    assert time.monotonic() - t0 < 0.2 + TOLERANCE_S
    gate.set()
    assert eyes.wait(1.0)
    time.sleep(0.05)
    assert eyes.backend.sent == [(3, 0, 255, 0, 0, 255), IDLE_PAYLOAD]  # the step in flight, then the restore
    assert (eyes.playing, eyes.backend.sentry_active) == (None, True)


def test_state_from_another_thread_during_playback():
    eyes = timed_eyes()
    eyes.play(look(steps=[{'left': 'blink', 'hold': 0.05}], loop=True))
    seen = []
    reader = threading.Thread(target=lambda: [seen.append(eyes.state().playing) for _ in range(50)])
    reader.start()
    reader.join(2.0)
    assert not reader.is_alive() and all(p['name'] == 'quick' for p in seen)
    eyes.close()


# ---------------- Command line ----------------

CLI = [sys.executable, '-m', 'stretch4_body.tools.stretch_eye_animations']


def run_cli(dirs, *args):
    env = dict(os.environ, HELLO_FLEET_PATH=str(dirs['fleet']), HELLO_FLEET_ID='stretch-se4-0000',
               STRETCH_EYES_LIBRARY=dirs['shared1'])
    return subprocess.run(CLI + list(args), capture_output=True, text=True, env=env, cwd=REPO, timeout=30)


def test_cli_looks_and_export(dirs):
    run = run_cli(dirs, '--looks')
    assert run.returncode == 0, run.stdout + run.stderr
    assert dirs['user'] in run.stdout and dirs['shared1'] in run.stdout
    lines = {line.split()[0]: line for line in run.stdout.splitlines() if line.startswith('  ') and len(line.split()) > 2}
    assert 'builtin' in lines['glance'] and 'loop' in lines['thinking']
    run = run_cli(dirs, '--export', 'glance', '--fake')
    assert run.returncode == 0
    assert json.loads(run.stdout) == Library().export('glance')  # nothing but JSON on stdout
    run = run_cli(dirs, '--export', 'nope')
    assert run.returncode == 1 and 'No look named' in run.stderr and run.stdout == ''  # a redirect gets no error text


def test_cli_import(dirs, tmp_path):
    src = write_look(str(tmp_path / 'downloads'), look('glance', title='mine'))
    run = run_cli(dirs, '--import', src)
    assert run.returncode == 0, run.stdout + run.stderr
    assert os.path.exists(os.path.join(dirs['user'], 'glance.json'))
    assert 'user' in [line for line in run_cli(dirs, '--looks').stdout.splitlines() if line.startswith('  glance')][0]
    run = run_cli(dirs, '--import', src)
    assert run.returncode == 1 and 'already exists. Use --overwrite' in run.stderr
    assert run_cli(dirs, '--import', src, '--overwrite').returncode == 0
    bad = write_look(str(tmp_path / 'downloads'), look('broken', steps=[{'hold': 900}]))
    run = run_cli(dirs, '--import', bad, '--fake')
    assert run.returncode == 1 and 'steps[0].hold' in run.stderr


def test_cli_play_by_name_and_file(dirs, tmp_path):
    Library().save(look('mine', steps=[{'left': 'blink', 'color': 'green', 'hold': 0.05},
                                       {'left': 'happy', 'hold': 0.05}]))
    run = run_cli(dirs, '--fake', '--play', 'mine')
    assert run.returncode == 0, run.stdout + run.stderr
    assert 'Playing mine (2 steps' in run.stdout
    assert '[(3, 0, 255, 0, 255, 0), (8, 0, 255, 0, 255, 0)]' in run.stdout
    assert 'Fake robot: 2 eye commands sent, {} active'.format(SENTRY_NAME) in run.stdout
    src = write_look(str(tmp_path), look('elsewhere', steps=[{'left': 'off', 'right': 'off', 'hold': 0.05}]))
    run = run_cli(dirs, '--fake', '--play', src)
    assert run.returncode == 0 and '[(1, 1, 255, 40, 48, 60)]' in run.stdout


@pytest.mark.parametrize('args,rc,text', [
    (['--play', 'nope'], 1, 'No look named'),
    (['--play', 'glance', '--both', 'blink'], 2, '--play'),
    (['--loop'], 2, '--loop goes with --play'),
    (['--overwrite'], 2, '--overwrite with --import'),
])
def test_cli_play_rejects(dirs, args, rc, text):
    run = run_cli(dirs, '--fake', *args)
    assert run.returncode == rc and text in run.stdout


def test_cli_ctrl_c_stops_and_releases(dirs):
    env = dict(os.environ, HELLO_FLEET_PATH=str(dirs['fleet']), HELLO_FLEET_ID='stretch-se4-0000')
    proc = subprocess.Popen(CLI + ['--fake', '--play', 'thinking', '--loop'], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True, env=env, cwd=REPO)
    try:
        for line in proc.stdout:
            if line.startswith('Playing'):
                break
        assert 'looping' in line
        proc.send_signal(signal.SIGINT)
        out, err = proc.communicate(timeout=5)
    finally:
        proc.kill()
    assert proc.returncode == 0, out + err
    assert 'Stopped.' in out
    assert 'Fake robot: 2 eye commands sent, {} active'.format(SENTRY_NAME) in out  # the step, then the restore


def test_cli_play_when_the_first_step_fails_at_once(dirs):
    """The play thread can end before the Playing line prints; the line must not read eyes.playing."""
    code = '\n'.join([
        'import runpy, sys',
        'import stretch4_body.eyes.api as api',
        'play = api.Eyes.play',
        'def play_and_fail(self, *a, **k):',
        '    self._send = lambda *x, **y: 1 / 0',
        '    play(self, *a, **k)',
        '    self.wait()',
        'api.Eyes.play = play_and_fail',
        "sys.argv = ['stretch_eye_animations', '--fake', '--play', 'thinking']",
        "runpy.run_module('stretch4_body.tools.stretch_eye_animations', run_name='__main__')",
    ])
    env = dict(os.environ, HELLO_FLEET_PATH=str(dirs['fleet']), HELLO_FLEET_ID='stretch-se4-0000')
    run = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, env=env, cwd=REPO, timeout=30)
    assert run.returncode == 0 and 'TypeError' not in run.stderr, run.stderr
    assert 'Playing thinking (' in run.stdout and 'looping' in run.stdout


# ---------------- Eyes Studio server ----------------

def studio_on_fake():
    from stretch4_body.eyes.studio.server import Studio
    return Studio(Eyes(backend='fake'), 'fake', library=Library())


def test_studio_import_text_parses_the_file_as_the_cli_does(dirs):
    from stretch4_body.eyes.studio.server import BadRequest
    studio = studio_on_fake()
    text = '{"format": "stretch-eyes/1", "name": "full", "steps": [{"left": "blink", "intensity": 1.0, "hold": 1}]}'
    assert studio.library_save({'text': text})['name'] == 'full'
    assert Library().get('full').steps[0].intensity == 255
    with pytest.raises(FileExistsError):
        studio.library_save({'text': text})
    studio.library_save({'text': text.replace('1.0', '0.5'), 'overwrite': True})
    assert Library().get('full').steps[0].intensity == 128
    for body in ({'text': '{"format": '}, {'text': 5}, {'text': text, 'sequence': {}}):
        with pytest.raises(BadRequest):
            studio.library_save(body)


def test_studio_library_list_is_one_scan_with_the_sequences(dirs, monkeypatch):
    write_look(dirs['shared1'], look('wave'))
    studio = studio_on_fake()
    scans = []
    scan = Library._scan
    monkeypatch.setattr(Library, '_scan', lambda self: scans.append(1) or scan(self))
    r = studio.library_list()
    assert len(scans) == 1
    assert sorted(r['sequences']) == sorted(e['name'] for e in r['looks'])
    assert r['sequences']['wave'] == Library().export('wave')
    assert all('path' not in e for e in r['looks'])


def test_studio_state_leaves_out_the_sequence_and_playing_carries_it(dirs):
    studio = studio_on_fake()
    eyes = studio.writer.eyes
    studio.wait(studio.writer.submit('set', {'left': 8, 'right': 8, 'color': (0, 0, 255), 'intensity': 90}))
    studio.wait(studio.writer.submit('play', studio.parse_play({'name': 'thinking'})))
    state = studio.state()
    assert state['playing']['name'] == 'thinking' and 'sequence' not in state['studio']
    r = studio.playing()
    assert r['playing']['name'] == 'thinking' and r['sequence'] == Library().export('thinking')
    studio.wait(studio.writer.submit('stop'))
    assert studio.playing() == {'playing': None, 'sequence': None}
    assert eyes.backend.sent[-1] == (8, 8, 90, 0, 0, 255)  # Stop puts back the look from before Play
    eyes.close()
