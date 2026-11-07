import importlib
import sys

from stretch4_body.core import robot_params as rp
from stretch4_body.subsystem.line_sensor import connect


def test_a_robot_without_line_sensors_skips_the_check(monkeypatch, capsys):
    monkeypatch.setattr(sys, 'argv', ['stretch_system_check', '--verbose'])
    check = importlib.import_module('stretch4_body.tools.stretch_system_check')
    monkeypatch.setitem(rp.RobotParams._robot_params['line_sensor_loop'], 'sensor_names', [])

    def no_port(*args, **kwargs):
        raise AssertionError('the port must not be opened')
    monkeypatch.setattr(connect, 'open_line_sensors', no_port)

    assert check.check_line_sensors() is True
    assert 'No line sensors fitted' in capsys.readouterr().out
