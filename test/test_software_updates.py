"""Unit tests for the software update checks. No network, hardware or server needed.

    python3 -m pytest test/test_software_updates.py
"""
import subprocess

import pytest

from stretch4_body.behavior.health import health_checks as hc
from stretch4_body.behavior.health import software_updates as su
from stretch4_body.behavior.health import health_manager as hm

PARAMS = hc.merged_params({})
HOUR = 3600.0

BOARDS = {'hello-motor-lift': {'version': 'Stepper.v0.8.2p8', 'supported_protocols': ['p7', 'p8']},
          'hello-power-periph': {'version': 'PowerPeriph.v0.4.0p9', 'supported_protocols': ['p8', 'p9']}}
AVAILABLE = {'hello-motor-lift': ['hello-stepper2.v0.8.1p8', 'hello-stepper2.v0.8.3p8', 'hello-stepper2.v0.9.0p9'],
             'hello-power-periph': ['hello-pimu2.v0.4.0p9']}


# ---------------------------------------------------------------- schedule

def test_never_checks_while_the_server_starts():
    t0 = 1_000_000.0
    # A check long ago is due, but not before the initial delay after start
    assert su.next_check_time(t0, {'ts_check': t0 - 100 * HOUR}, PARAMS) == t0 + 2 * HOUR
    assert su.next_check_time(t0, {}, PARAMS) == t0 + 2 * HOUR


def test_period_after_a_check_and_retry_after_no_internet():
    t0 = 1_000_000.0
    assert su.next_check_time(t0, {'ts_check': t0 + 3 * HOUR, 'ts_attempt': t0 + 3 * HOUR}, PARAMS) == t0 + 9 * HOUR
    offline = {'ts_check': t0 + 3 * HOUR, 'ts_attempt': t0 + 4 * HOUR}
    assert su.next_check_time(t0, offline, PARAMS) == t0 + 4.5 * HOUR


# ---------------------------------------------------------------- firmware

def test_firmware_recommends_newest_supported_protocol():
    recs = su.firmware_recommendations(BOARDS, AVAILABLE)
    lift = recs['hello-motor-lift']
    assert lift['action'] == su.FW_UPGRADE
    assert lift['recommended'].endswith('v0.8.3p8')  # p9 isn't supported by this stretch4_body
    assert recs['hello-power-periph']['action'] == su.FW_CURRENT
    assert su.firmware_command(recs) == 'REx_firmware_updater --install --lift'


def test_firmware_unknown_without_a_release_list():
    recs = su.firmware_recommendations(BOARDS, {})
    assert {r['action'] for r in recs.values()} == {su.FW_UNKNOWN}
    assert su.firmware_command(recs) is None


def test_firmware_boards_from_a_running_robot():
    class Motor:
        def __init__(self, name, version):
            self.name = name
            self.board_info = {'firmware_version': version}
            self._supported_protocols = {'p8': None, 'p7': None}

    class Joint:
        def __init__(self, motor):
            self.motor = motor

    class Robot:
        lift = Joint(Motor('hello-motor-lift', 'Stepper.v0.8.2p8'))
        arm = Joint(Motor('hello-motor-arm', None))  # Couldn't read it: left out

    boards = su.firmware_boards(Robot())
    assert boards == {'hello-motor-lift': {'version': 'Stepper.v0.8.2p8', 'supported_protocols': ['p7', 'p8']}}


# ---------------------------------------------------------------- results

def evaluation(**kw):
    ev = {'pip': {}, 'pypi_reachable': True, 'repos': [], 'firmware': {},
          'firmware_reachable': False, 'firmware_command': None}
    ev.update(kw)
    return ev


def test_updates_are_info_and_combine_in_the_summary():
    ev = evaluation(pip={'hello-robot-stretch4-body': ('2026.9.11', '2026.11.1')},
                    firmware=su.firmware_recommendations(BOARDS, AVAILABLE), firmware_reachable=True,
                    firmware_command='REx_firmware_updater --install --lift')
    results = {r['id']: r for r in su.update_results(ev)}
    pip = results['updates.pip_hello-robot-stretch4-body']
    assert pip['level'] == hc.INFO and '2026.9.11 → 2026.11.1' in pip['message']
    assert results['updates.firmware_hello-motor-lift']['level'] == hc.INFO
    assert hc.worst_level(results.values()) == hc.OK  # Updates never make the robot unhealthy
    assert {r['summary_group'] for r in results.values() if r['level'] == hc.INFO} == {'Updates available'}


def test_nothing_reported_for_what_wasnt_reached():
    assert su.update_results(evaluation(pypi_reachable=False)) == []
    ok = su.update_results(evaluation())
    assert [r['id'] for r in ok] == ['updates.pip']


def test_pip_updates_judge_against_installed_now():
    latest = {'hello-robot-stretch4-body': '2026.11.1'}
    assert su.pip_updates({'hello-robot-stretch4-body': '2026.9.11'}, latest) == latest
    # Installed since the cached check: no longer reported
    assert su.pip_updates({'hello-robot-stretch4-body': '2026.11.1'}, latest) == {}


# ---------------------------------------------------------------- git

def git(cwd, *args):
    subprocess.run(['git', '-C', str(cwd), *args], check=True, capture_output=True,
                   env={'GIT_AUTHOR_NAME': 't', 'GIT_AUTHOR_EMAIL': 't@t', 'GIT_COMMITTER_NAME': 't',
                        'GIT_COMMITTER_EMAIL': 't@t', 'HOME': str(cwd), 'PATH': '/usr/bin:/bin'})


@pytest.fixture
def repo(tmp_path):
    """A clone tracking origin/main, with its bare remote."""
    remote = tmp_path / 'remote.git'
    git(tmp_path, 'init', '--bare', '-b', 'main', str(remote))
    clone = tmp_path / 'clone'
    git(tmp_path, 'clone', str(remote), str(clone))
    git(clone, 'checkout', '-b', 'main')
    (clone / 'f').write_text('1')
    git(clone, 'add', 'f')
    git(clone, 'commit', '-m', 'one')
    git(clone, 'push', '-u', 'origin', 'main')
    return clone


def test_repo_judged_against_a_cached_remote(repo):
    fetched = su.fetch_repo_remote(str(repo))
    assert fetched['upstream'] == 'origin/main' and len(fetched['sha']) == 40
    assert su.check_repo_git_status(str(repo), remote=fetched)['status'] == su.GIT_OK
    newer = dict(fetched, sha='f' * 40)  # The remote moved on since we last fetched
    assert su.check_repo_git_status(str(repo), remote=newer)['status'] == su.GIT_UPDATE
    assert su.check_repo_git_status(str(repo), remote=None)['status'] == su.GIT_NOT_CHECKED
    other_branch = dict(fetched, upstream='origin/dev')
    assert su.check_repo_git_status(str(repo), remote=other_branch)['status'] == su.GIT_NOT_CHECKED
    assert su.check_repo_git_status(str(repo), remote=dict(fetched, sha=None))['status'] == su.GIT_UNREACHABLE


# ---------------------------------------------------------------- cache and worker

def test_cache_round_trip(tmp_path):
    path = str(tmp_path / 'sub' / 'software_updates.json')
    assert su.load_cache(path) == {}
    assert su.save_cache({'ts_check': 5.0}, path)
    assert su.load_cache(path) == {'ts_check': 5.0}


@pytest.fixture
def worker(tmp_path, monkeypatch):
    monkeypatch.setenv('XDG_CACHE_HOME', str(tmp_path))
    calls = {'fetch': 0}

    def fetch(boards):
        calls['fetch'] += 1
        return {'ts_check': hm.time.time(), 'online': True, 'pypi_latest': {}, 'repo_remotes': {},
                'firmware_available': AVAILABLE}
    monkeypatch.setattr(su, 'fetch_remote', fetch)
    monkeypatch.setattr(su, 'evaluate', lambda remote, boards: evaluation())
    w = hm._UpdateCheckWorker(dict(PARAMS), BOARDS)
    w.calls = calls
    return w


def test_worker_waits_then_checks_and_caches(worker, monkeypatch):
    monkeypatch.setattr(su, 'internet_available', lambda: True)
    out = {}
    hm._cb_update_step(worker, None, out)
    assert worker.calls['fetch'] == 0  # Just started: nothing checked
    assert out['updates']['ts_check'] == 0.0

    worker.ts_start -= 3 * HOUR  # Pretend the server has been up for 3 hours
    hm._cb_update_step(worker, None, out)
    assert worker.calls['fetch'] == 1
    assert out['updates']['firmware']['hello-motor-lift']['action'] == su.FW_UPGRADE
    assert su.load_cache()['ts_check'] > 0  # A restart will show this without checking again
    hm._cb_update_step(worker, None, out)
    assert worker.calls['fetch'] == 1  # Not again until update_check_period_s


def test_worker_skips_without_internet(worker, monkeypatch):
    monkeypatch.setattr(su, 'internet_available', lambda: False)
    worker.ts_start -= 3 * HOUR
    out = {}
    hm._cb_update_step(worker, None, out)
    assert worker.calls['fetch'] == 0
    assert out['updates']['online'] is False
    assert out['updates']['ts_next'] >= hm.time.time() + PARAMS['update_check_retry_s'] - 1
