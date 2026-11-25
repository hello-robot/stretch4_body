#!/usr/bin/env python3
"""Effort <-> current conversions in FeetechSMHello.

eeprom_cfg['overcurrent'] is a raw servo register value (6.5 mA/tick), so 100% effort
must correspond to overcurrent * 6.5 mA, not to overcurrent mA.
"""
import math

import pytest

from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello

OVERCURRENT_TICKS = 150  # Value used by the SE4 Feetech joints
OVERCURRENT_MA = OVERCURRENT_TICKS * 6.5  # 975 mA


@pytest.fixture
def servo():
    # Skip __init__ so no hardware or robot params are needed
    s = FeetechSMHello.__new__(FeetechSMHello)
    s.params = {'eeprom_cfg': {'overcurrent': OVERCURRENT_TICKS}}
    return s


def test_overcurrent_ticks_convert_to_mA(servo):
    assert math.isclose(servo.ticks_to_current(OVERCURRENT_TICKS), 975.0)


@pytest.mark.parametrize('i_mA, effort_pct', [
    (0.0, 0.0),
    (195.0, 20.0),   # Default stall_max_effort threshold
    (487.5, 50.0),
    (975.0, 100.0),
])
def test_current_to_effort_pct(servo, i_mA, effort_pct):
    assert math.isclose(servo.current_to_effort_pct(i_mA), effort_pct)


def test_current_equal_to_raw_tick_value_is_not_full_effort(servo):
    # Regression: the old code treated the tick value as mA, so 150 mA read as 100%
    assert math.isclose(servo.current_to_effort_pct(150.0), 100.0 * 150.0 / OVERCURRENT_MA)


def test_current_to_effort_pct_clamps(servo):
    assert servo.current_to_effort_pct(-200.0) == 0.0
    assert servo.current_to_effort_pct(5000.0) == 100.0


@pytest.mark.parametrize('effort_pct, i_mA', [
    (0.0, 0.0),
    (20.0, 195.0),
    (100.0, 975.0),
    (250.0, 975.0),  # Clamped to the overcurrent limit
])
def test_effort_pct_to_current(servo, effort_pct, i_mA):
    assert math.isclose(servo.effort_pct_to_current(effort_pct), i_mA)


@pytest.mark.parametrize('effort_pct', [0.0, 12.5, 20.0, 73.0, 100.0])
def test_effort_current_round_trip(servo, effort_pct):
    assert math.isclose(servo.current_to_effort_pct(servo.effort_pct_to_current(effort_pct)), effort_pct)
