"""The stretch_body_server command: session logs, the daemon helpers and launching."""
import argparse
import logging
import os
import subprocess
import tarfile
import zipfile
from unittest.mock import MagicMock, patch

import pytest

from stretch4_body.tools import stretch_body_server as sbs

INFO = '[2026-01-01 10:00:00] [robot] [INFO]: started'
ERROR = '[2026-01-01 10:00:01] [robot] [ERROR]: lift not found'


@pytest.fixture
def log_dir(tmp_path, monkeypatch):
    monkeypatch.setattr(sbs, 'LOG_DIR', str(tmp_path))
    (tmp_path / 'archive').mkdir()
    return tmp_path


def write_archive(log_dir, timestamp, lines, mtime):
    log = log_dir / 'packed.log'
    log.write_text('\n'.join(lines) + '\n')
    path = log_dir / 'archive' / f'stretch_body_server_logs_{timestamp}.tar.gz'
    with tarfile.open(path, 'w:gz') as tar:
        tar.add(log, arcname='stretch_body_server.log')
    log.unlink()
    os.utime(path, (mtime, mtime))
    return str(path)


def test_rotated_segments_sort_oldest_first():
    names = ['stretch_body_server.log', 'stretch_body_server.log.2', 'stretch_body_server.log.1']

    assert sorted(names, key=sbs._segment_order) == [
        'stretch_body_server.log.2', 'stretch_body_server.log.1', 'stretch_body_server.log']


def test_archive_packs_the_session_and_clears_the_active_logs(log_dir):
    (log_dir / 'stretch_body_server.log').write_text(INFO + '\n')
    (log_dir / 'stretch_body_server.log.1').write_text(ERROR + '\n')

    sbs.archive_session_logs()

    archives = list((log_dir / 'archive').glob('stretch_body_server_logs_*.tar.gz'))
    assert len(archives) == 1
    with tarfile.open(archives[0]) as tar:
        assert sorted(tar.getnames()) == ['stretch_body_server.log', 'stretch_body_server.log.1']
    assert list(log_dir.glob('stretch_body_server.log*')) == []


def test_archive_of_an_empty_session_only_removes_the_empty_log(log_dir):
    (log_dir / 'stretch_body_server.log').write_text('')

    sbs.archive_session_logs()

    assert list((log_dir / 'archive').glob('*.tar.gz')) == []
    assert list(log_dir.glob('stretch_body_server.log*')) == []


def test_recent_logs_count_the_running_session(log_dir):
    active = log_dir / 'stretch_body_server.log'
    active.write_text(INFO + '\n')
    old = write_archive(log_dir, '20260101000000', [INFO], mtime=1_700_000_100)
    mid = write_archive(log_dir, '20260102000000', [INFO], mtime=1_700_000_200)
    new = write_archive(log_dir, '20260103000000', [INFO], mtime=1_700_000_300)

    assert sbs.get_recent_session_logs(3) == [str(active), new, mid]
    active.unlink()
    assert sbs.get_recent_session_logs(3) == [new, mid, old]


def test_last_session_log_without_logs(log_dir):
    assert sbs.last_session_log() == (None, None)


def test_last_session_log_reads_an_unarchived_session_oldest_segment_first(log_dir):
    write_archive(log_dir, '20260101000000', ['archived'], mtime=1_700_000_100)
    (log_dir / 'stretch_body_server.log.1').write_text('first\n')
    (log_dir / 'stretch_body_server.log').write_text('second\n')

    lines, source = sbs.last_session_log()

    assert lines == ['first', 'second']
    assert source == str(log_dir / 'stretch_body_server.log')


def test_last_session_log_reads_the_newest_archive(log_dir):
    write_archive(log_dir, '20260101000000', ['older'], mtime=1_700_000_100)
    newest = write_archive(log_dir, '20260102000000', ['newer', 'session'], mtime=1_700_000_200)

    assert sbs.last_session_log() == (['newer', 'session'], newest)


def test_last_session_log_reports_an_unreadable_archive(log_dir):
    broken = log_dir / 'archive' / 'stretch_body_server_logs_20260101000000.tar.gz'
    broken.write_text('not a tarball')

    lines, source = sbs.last_session_log()

    assert source == str(broken)
    assert lines[0].startswith(f'Could not read {broken}')


def test_find_errors_keeps_error_records_and_tracebacks():
    lines = [
        INFO,
        '\x1b[31m' + ERROR + '\x1b[0m',
        '    while starting the lift',
        'printed by the server',
        '    indented print',
        'Traceback (most recent call last):',
        '  File "robot_server.py", line 1, in run',
        'RuntimeError: lost transport',
        'printed after the traceback',
        '[2026-01-01 10:00:02] [robot] [CRITICAL]: giving up',
        '[2026-01-01 10:00:03] [robot] [WARNING]: slow loop',
    ]

    assert sbs.find_errors(lines) == [
        ERROR,
        '    while starting the lift',
        'Traceback (most recent call last):',
        '  File "robot_server.py", line 1, in run',
        'RuntimeError: lost transport',
        '[2026-01-01 10:00:02] [robot] [CRITICAL]: giving up',
    ]


def test_print_errors_stops_at_the_limit(capsys):
    sbs.print_errors([ERROR] * 5, limit=2)

    out = capsys.readouterr().out
    assert out.count(ERROR) == 2
    assert '... and 3 more.' in out


def test_colorize_leaves_lines_alone_when_not_on_a_terminal():
    assert sbs.colorize(ERROR + '\n') == ERROR


def test_export_zips_the_logs_with_a_readme(log_dir, tmp_path):
    (log_dir / 'stretch_body_server.log').write_text(INFO + '\n')
    write_archive(log_dir, '20260101000000', [INFO], mtime=1_700_000_100)
    export_dir = tmp_path / 'exports'
    export_dir.mkdir()

    zip_path = sbs.export_session_logs(str(export_dir))

    with zipfile.ZipFile(zip_path) as zf:
        assert sorted(zf.namelist()) == [
            'README.md', 'stretch_body_server.log', 'stretch_body_server_logs_20260101000000.tar.gz']
        assert 'The 2 most recent' in zf.read('README.md').decode()


def test_export_fails_without_logs_or_directory(log_dir, tmp_path):
    assert sbs.export_session_logs(str(tmp_path)) is None
    (log_dir / 'stretch_body_server.log').write_text(INFO + '\n')
    assert sbs.export_session_logs(str(tmp_path / 'missing')) is None


def test_prune_keeps_the_newest_archives_and_removes_legacy_logs(log_dir):
    old = write_archive(log_dir, '20260101000000', [INFO], mtime=1_700_000_100)
    new = write_archive(log_dir, '20260102000000', [INFO], mtime=1_700_000_200)
    legacy = log_dir / 'stretchbody_20250101.log'
    legacy.write_text('old format\n')

    assert sbs.prune_logs(keep=1) is True

    assert not os.path.exists(old)
    assert os.path.exists(new)
    assert not legacy.exists()


def test_prune_rejects_a_negative_keep_and_accepts_nothing_to_do(log_dir):
    assert sbs.prune_logs(keep=-1) is False
    assert sbs.prune_logs(keep=1) is True


def test_logger_writer_logs_whole_lines(caplog):
    original = MagicMock()
    writer = sbs.LoggerWriter(logging.ERROR, original)

    with caplog.at_level(logging.ERROR, logger=sbs.logger.name):
        writer.write('TypeError')
        writer.write(': ')
        writer.write('bad argument\n  detail\n\n')

    assert [r.getMessage() for r in caplog.records] == ['TypeError: bad argument', '  detail']
    original.write.assert_not_called()
    writer.flush()
    original.flush.assert_called_once_with()


def test_server_processes_are_the_other_launched_servers():
    def proc(pid, cmdline):
        p = MagicMock()
        p.info = {'pid': pid, 'cmdline': cmdline}
        return p

    launched = proc(10, ['python3', '/usr/bin/stretch_body_server', '--launch'])
    procs = [
        launched,
        proc(os.getpid(), ['python3', '/usr/bin/stretch_body_server', '--launch']),
        proc(11, ['python3', '/usr/bin/stretch_body_server', '--status']),
        proc(12, ['roslaunch', '--launch']),
        proc(13, None),
    ]

    with patch.object(sbs.psutil, 'process_iter', return_value=procs):
        assert sbs._server_processes() == [launched]


@pytest.mark.parametrize('started, active, expected', [
    (True, True, True), (True, False, False), (False, True, False)])
def test_is_server_active_needs_a_connection_and_an_answer(started, active, expected):
    client = MagicMock()
    client.startup.return_value = started
    client.is_server_active.return_value = active

    assert sbs.is_server_active(client) is expected
    client.stop.assert_called_once_with()


def completed(returncode=0, stdout=''):
    return subprocess.CompletedProcess([], returncode, stdout=stdout)


@pytest.mark.parametrize('result, expected', [
    (completed(0, 'active\n'), True),
    (completed(3, 'inactive\n'), False),
    (completed(3, 'failed\n'), False)])
def test_daemon_is_running(result, expected):
    with patch.object(sbs.subprocess, 'run', return_value=result):
        assert sbs.daemon_is_running() is expected


@pytest.mark.parametrize('result, expected', [
    (completed(0, 'enabled\n'), True),
    (completed(1, 'disabled\n'), False),
    (completed(0, 'not-found\n'), False)])
def test_daemon_is_installed(result, expected):
    with patch.object(sbs.subprocess, 'run', return_value=result):
        assert sbs.daemon_is_installed() is expected


def test_daemon_checks_are_false_without_systemctl():
    with patch.object(sbs.subprocess, 'run', side_effect=FileNotFoundError):
        assert sbs.daemon_is_running() is False
        assert sbs.daemon_is_installed() is False
        assert sbs.daemon_is_retrying() is False
        assert sbs._daemon_journal(0) == []


def test_daemon_props_are_parsed_from_systemctl_show():
    shown = completed(0, 'ActiveState=activating\nSubState=auto-restart\n')

    with patch.object(sbs.subprocess, 'run', return_value=shown) as run:
        assert sbs._daemon_props('ActiveState', 'SubState') == {
            'ActiveState': 'activating', 'SubState': 'auto-restart'}
        assert sbs.daemon_is_retrying() is True

    assert '--property=SubState' in run.call_args[0][0]


def test_daemon_start_failure_is_reported(caplog):
    failed = subprocess.CalledProcessError(1, 'systemctl')

    with patch.object(sbs.subprocess, 'run', side_effect=[completed(), failed]):
        assert sbs._manage_daemon('start') is False

    assert 'Error while starting Stretch Body Server system service' in caplog.text


def test_restart_daemon_refuses_while_a_command_line_server_runs(caplog):
    with patch.object(sbs, 'daemon_is_running', return_value=False), \
         patch.object(sbs, 'is_server_active', return_value=True), \
         patch.object(sbs, 'RobotClient'), \
         patch.object(sbs, '_manage_daemon') as manage:
        assert sbs.restart_daemon() is False

    manage.assert_not_called()
    assert 'A non-daemon server is already active' in caplog.text


def test_restart_daemon_archives_only_when_no_daemon_is_running():
    with patch.object(sbs, 'is_server_active', return_value=False), \
         patch.object(sbs, 'RobotClient'), \
         patch.object(sbs, 'archive_session_logs') as archive, \
         patch.object(sbs, '_manage_daemon', return_value=True) as manage:
        with patch.object(sbs, 'daemon_is_running', return_value=True):
            assert sbs.restart_daemon() is True
        archive.assert_not_called()
        with patch.object(sbs, 'daemon_is_running', return_value=False):
            assert sbs.restart_daemon() is True
        archive.assert_called_once_with()

    manage.assert_called_with('restart')


def test_wait_for_daemon_returns_when_the_server_answers():
    with patch.object(sbs, 'is_server_active', side_effect=[False, True]), \
         patch.object(sbs, '_daemon_props', return_value={'ActiveState': 'active'}), \
         patch.object(sbs, 'report_daemon_failure') as report:
        assert sbs.wait_for_daemon(MagicMock(), started_at=0.0) is True

    report.assert_not_called()


def test_wait_for_daemon_gives_up_when_the_daemon_exits():
    with patch.object(sbs, 'is_server_active', return_value=False), \
         patch.object(sbs, '_daemon_props', return_value={'ActiveState': 'failed'}), \
         patch.object(sbs, 'report_daemon_failure') as report:
        assert sbs.wait_for_daemon(MagicMock(), started_at=5.0, timeout_s=15.0) is False

    report.assert_called_once_with(5.0, 15.0)


def test_daemon_failure_report_lists_the_logged_errors(capsys):
    props = {'ActiveState': 'activating', 'SubState': 'auto-restart', 'ExecMainStatus': '1'}

    with patch.object(sbs, '_daemon_props', return_value=props), \
         patch.object(sbs, '_daemon_journal', return_value=[INFO, ERROR]):
        sbs.report_daemon_failure(started_at=0.0, timeout_s=15.0)

    out = capsys.readouterr().out
    assert 'failed to start (exit status 1)' in out
    assert ERROR in out
    assert INFO not in out
    assert 'The daemon will retry in the background' in out


def launch_args(profile=False):
    return argparse.Namespace(profile=profile)


def test_launch_refuses_when_a_server_is_already_running(capsys):
    with patch.object(sbs, 'is_server_active', return_value=True), \
         patch.object(sbs.StretchBodyServer, 'is_server_owned_by_current_user', return_value=True), \
         patch.object(sbs.robot_server, 'run_server') as run_server:
        assert sbs._launch(MagicMock(), launch_args()) is False

    run_server.assert_not_called()
    assert 'A server is already running.' in capsys.readouterr().out


@pytest.mark.parametrize('outcome, expected', [
    (None, True), (SystemExit(0), True), (SystemExit(1), False), (RuntimeError('crash'), False)])
def test_launch_reports_how_the_server_ended_and_archives_its_log(outcome, expected):
    with patch.object(sbs, 'is_server_active', return_value=False), \
         patch.object(sbs.robot_server, 'run_server', side_effect=outcome), \
         patch.object(sbs, 'archive_session_logs') as archive:
        assert sbs._launch(MagicMock(), launch_args()) is expected

    archive.assert_called_once_with()
