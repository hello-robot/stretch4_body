from types import SimpleNamespace
from unittest.mock import MagicMock

from stretch4_body.behavior.routines.routine_manager import RoutineManager


def manager(**routines):
    m = RoutineManager.__new__(RoutineManager)
    m.logger = MagicMock()
    m.status = {'last_routine_id': 0, 'active_routine': 'routine_nop',
                'active_routine_id': 0, 'last_routine_successful': True}
    m.next_routine = 'routine_nop'
    m.next_routine_id = 0
    m.routines = routines
    return m


def test_unknown_routine_finishes_as_failed():
    m = manager()
    m._routine_set_next('routine_robot_home', 7)
    assert m.status['last_routine_id'] == 7
    assert m.status['last_routine_successful'] is False


def test_invalid_routine_finishes_as_failed():
    m = manager(routine_robot_home=SimpleNamespace(is_valid=False))
    m._routine_set_next('routine_robot_home', 8)
    assert m.status['last_routine_id'] == 8
    assert m.status['last_routine_successful'] is False


def test_request_while_busy_finishes_as_failed():
    m = manager(routine_robot_home=SimpleNamespace(is_valid=True))
    m.status['active_routine'] = 'routine_robot_stow'
    m._routine_set_next('routine_robot_home', 9)
    assert m.status['last_routine_id'] == 9
    assert m.status['last_routine_successful'] is False


def test_valid_routine_is_queued_not_finished():
    m = manager(routine_robot_home=SimpleNamespace(is_valid=True))
    m._routine_set_next('routine_robot_home', 10)
    assert m.next_routine == 'routine_robot_home'
    assert m.status['last_routine_id'] == 0
