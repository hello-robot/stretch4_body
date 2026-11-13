"""Stopping and restarting stretch_body_server around a power change."""
from unittest.mock import MagicMock, patch

import pytest

from stretch4_body.utils import server_power


def robot_client(startup):
    client = MagicMock()
    client.startup.side_effect = startup
    return patch('stretch4_body.robot.robot_client.RobotClient', return_value=client), client


@pytest.mark.parametrize('startup, expected', [
    ([True], True), ([False], False), (RuntimeError('no socket'), False)])
def test_is_server_running_asks_the_server_and_disconnects(startup, expected):
    patched, client = robot_client(startup)

    with patched:
        assert server_power.is_server_running() is expected

    client.startup.assert_called_once_with(verbose=False, allow_different_user_connection=True)
    client.stop.assert_called_once_with()


def test_is_server_running_retries():
    patched, client = robot_client([False, False, True])

    with patched:
        assert server_power.is_server_running(timeout_tries=3) is True

    assert client.startup.call_count == 3


def test_kill_server_waits_for_the_server_to_stop():
    with patch.object(server_power.subprocess, 'run') as run, \
         patch.object(server_power.time, 'sleep'), \
         patch.object(server_power, 'is_server_running', side_effect=[True, True, False]):
        assert server_power.kill_server() is True

    assert run.call_args[0][0] == ['stretch_body_server', '--kill']


def test_kill_server_gives_up_after_15_checks():
    with patch.object(server_power.subprocess, 'run'), \
         patch.object(server_power.time, 'sleep'), \
         patch.object(server_power, 'is_server_running', return_value=True) as running:
        assert server_power.kill_server() is False

    assert running.call_count == 15


def test_start_server_waits_for_the_server_to_answer():
    with patch.object(server_power.subprocess, 'Popen') as popen, \
         patch.object(server_power.time, 'sleep'), \
         patch.object(server_power, 'is_server_running', side_effect=[False, True]):
        assert server_power.start_server() is True

    assert popen.call_args[0][0] == ['stretch_body_server', '--restart']
    assert popen.call_args[1]['start_new_session'] is True


def test_start_server_times_out():
    with patch.object(server_power.subprocess, 'Popen'), \
         patch.object(server_power.time, 'sleep'), \
         patch.object(server_power.time, 'time', side_effect=[0.0, 1.0, 46.0]), \
         patch.object(server_power, 'is_server_running', return_value=False):
        assert server_power.start_server(timeout=45.0) is False


def test_power_change_needs_no_stop_when_no_server_runs():
    with patch.object(server_power, 'is_server_running', return_value=False), \
         patch.object(server_power, 'kill_server') as kill:
        assert server_power.stop_server_for_power_change() == (True, False)

    kill.assert_not_called()


def test_power_change_is_refused_when_the_user_declines():
    with patch.object(server_power, 'is_server_running', return_value=True), \
         patch.object(server_power.click, 'confirm', return_value=False), \
         patch.object(server_power, 'kill_server') as kill:
        assert server_power.stop_server_for_power_change() == (False, True)

    kill.assert_not_called()


@pytest.mark.parametrize('killed, expected', [(True, (True, True)), (False, (False, True))])
def test_power_change_stops_the_server_without_asking_on_assume_yes(killed, expected):
    with patch.object(server_power, 'is_server_running', return_value=True), \
         patch.object(server_power.click, 'confirm') as confirm, \
         patch.object(server_power, 'kill_server', return_value=killed):
        assert server_power.stop_server_for_power_change(assume_yes=True) == expected

    confirm.assert_not_called()


@pytest.mark.parametrize('was_running, started', [(True, True), (False, False)])
def test_resume_restores_the_prior_state_on_assume_yes(was_running, started):
    with patch.object(server_power, 'start_server', return_value=True) as start:
        assert server_power.resume_server(was_running, assume_yes=True) is True

    assert start.called is started


@pytest.mark.parametrize('was_running, always_prompt, answer, started', [
    (True, False, True, True),
    (True, False, False, False),
    (False, False, True, False),
    (False, True, True, True),
    (False, True, False, False),
])
def test_resume_asks_before_starting_the_server(was_running, always_prompt, answer, started):
    with patch.object(server_power.click, 'confirm', return_value=answer), \
         patch.object(server_power, 'start_server', return_value=True) as start:
        assert server_power.resume_server(was_running, always_prompt=always_prompt) is True

    assert start.called is started
