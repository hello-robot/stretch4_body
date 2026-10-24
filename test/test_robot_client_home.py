from unittest.mock import MagicMock

import pytest

from stretch4_body.robot.robot_client import RobotClient


def client(success, last_id, runstopped):
    r = RobotClient.__new__(RobotClient)
    r.logger = MagicMock()
    r.routines = MagicMock()
    r.routines.routine_robot_home.return_value = (success, 5)
    r._status = {'routines': {'last_routine_id': last_id}}
    r.is_runstopped = lambda: runstopped
    return r


@pytest.mark.parametrize('success,last_id,runstopped,msg', [
    (False, 4, False, 'Homing routine timed out before completion.'),
    (False, 5, True, 'Cannot home while run-stopped'),
    (False, 5, False, 'Homing routine failed.'),
])
def test_home_reports_why_it_failed(success, last_id, runstopped, msg):
    r = client(success, last_id, runstopped)
    r.home()
    r.logger.error.assert_called_once_with(msg)


def test_home_success_logs_no_error():
    r = client(True, 5, False)
    r.home()
    r.logger.error.assert_not_called()
