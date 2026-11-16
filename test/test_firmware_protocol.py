import logging
from unittest.mock import MagicMock, patch

import pytest

from stretch4_body.core.stepper import Stepper, StepperBase, Stepper_Protocol_P7, Stepper_Protocol_P8
from stretch4_body.subsystem.power_periph import IMU, IMU_Protocol_P8, PowerPeriph, PowerPeriphBase, PowerPeriph_Protocol_P12


def make_stepper(protocol):
    s = Stepper.__new__(Stepper)
    s.name = 'hello-motor-lift'
    s.logger = logging.getLogger('stepper')
    s.hw_valid = True
    s.board_info = {'protocol_version': protocol}
    s._supported_protocols = {'p8': (Stepper_Protocol_P8,), 'p7': (Stepper_Protocol_P7,)}
    s.transport = MagicMock()
    s.enable_safety = MagicMock()
    s.pull_status = MagicMock()
    s.push_command = MagicMock()
    return s


@pytest.fixture
def restore_bases():
    bases = PowerPeriph.__bases__, IMU.__bases__
    yield
    PowerPeriph.__bases__, IMU.__bases__ = bases


def test_stepper_runs_a_newer_protocol_as_the_newest_known(caplog):
    s = make_stepper('p9')

    with patch.object(StepperBase, 'startup'):
        assert s.startup() is True

    assert s.set_command.__func__ is Stepper_Protocol_P8.set_command
    assert 'protocol p9 is newer' in caplog.text
    s.transport.stop.assert_not_called()


def test_stepper_still_disables_an_older_unknown_protocol():
    s = make_stepper('p6')

    with patch.object(StepperBase, 'startup'):
        assert s.startup() is False

    s.transport.stop.assert_called_once_with()


def test_power_periph_runs_a_newer_protocol_as_the_newest_known(caplog, restore_bases):
    p = PowerPeriph.__new__(PowerPeriph)
    p.name = 'hello-power-periph'
    p.logger = logging.getLogger('power_periph')
    p.hw_valid = True
    p.board_info = {'protocol_version': 'p14'}
    p._supported_protocols = {'p12': (PowerPeriph_Protocol_P12,), 'p13': (PowerPeriph_Protocol_P12,)}
    p.imu = MagicMock()
    p.imu._supported_protocols = {'p13': (IMU_Protocol_P8,)}
    p.transport = MagicMock()
    p.push_command = MagicMock()
    p.pull_status = MagicMock()

    with patch.object(PowerPeriphBase, 'startup'):
        assert p.startup() is True

    assert PowerPeriph.__bases__ == (PowerPeriph_Protocol_P12,)
    assert 'protocol p14 is newer' in caplog.text
    p.transport.stop.assert_not_called()
