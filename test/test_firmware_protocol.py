import logging
from unittest.mock import MagicMock, patch

from stretch4_body.core.stepper import Stepper, StepperBase, Stepper_Protocol_P7, Stepper_Protocol_P8
from stretch4_body.subsystem.power_periph import PowerPeriph, PowerPeriphBase, PowerPeriph_Protocol_P12


def test_stepper_says_why_it_is_disabled():
    s = Stepper.__new__(Stepper)
    s.name = 'hello-motor-lift'
    s.logger = logging.getLogger('stepper')
    s.hw_valid = True
    s.status = {}
    s.board_info = {'protocol_version': 'p9'}
    s._supported_protocols = {'p8': (Stepper_Protocol_P8,), 'p7': (Stepper_Protocol_P7,)}
    s.transport = MagicMock()

    with patch.object(StepperBase, 'startup'):
        assert s.startup() is False

    assert s.status['disabled_reason'] == 'Firmware protocol p9 is not supported by this Stretch Body, valid protocols are p8, p7'


def test_power_periph_says_why_it_is_disabled():
    p = PowerPeriph.__new__(PowerPeriph)
    p.name = 'hello-power-periph'
    p.logger = logging.getLogger('power_periph')
    p.hw_valid = True
    p.status = {}
    p.board_info = {'protocol_version': 'p14'}
    p._supported_protocols = {'p12': (PowerPeriph_Protocol_P12,), 'p13': (PowerPeriph_Protocol_P12,)}
    p.transport = MagicMock()

    with patch.object(PowerPeriphBase, 'startup'):
        assert p.startup() is False

    assert p.status['disabled_reason'] == 'Firmware protocol p14 is not supported by this Stretch Body, valid protocols are p12, p13'
