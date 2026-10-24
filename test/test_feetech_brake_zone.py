#!/usr/bin/env python3
"""
FeetechSMHello._step_vel_braking near the joint limits, both ends. No hardware:
the servo object is built without __init__ and its motor is a stub that
records the speed sent.
"""
import math

import pytest

from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello

LOWER, UPPER = -1.5, 1.5        # rad
A_MAX = 8.0                     # rad/s^2
THRESH = 0.3                    # rad, the smallest brake zone


class Motor:
    def __init__(self):
        self.sent = None

    def set_vel(self, ticks_per_s):
        self.sent = ticks_per_s


def servo_at(x, v=0.0):
    s = FeetechSMHello.__new__(FeetechSMHello)
    s.motor = Motor()
    s.status = {'pos': x, 'vel': v, 'timestamp_pc': 1.0}
    s._prev_set_vel_ts = 0.0
    s.range_t = (LOWER, UPPER)
    s.ticks_to_world_rad = lambda t: t
    s.world_rad_to_ticks_per_sec = lambda r: r
    s.params = {'motion': {'max': {'accel': A_MAX}}}
    s.set_motion_params = lambda a_des=None: None
    s.vel_brake_zone_thresh = THRESH
    return s


def sent_speed(x, v_des, v=0.0):
    s = servo_at(x, v)
    s._step_vel_braking(v_des, 1.0)
    return s.motor.sent


@pytest.mark.parametrize('gap', [0.05, 0.09, 0.095, 0.099])
def test_inside_0p1_rad_of_either_limit_moving_toward_it_stops(gap):
    # 5 degrees (0.087 rad) of padding already stopped it closer than that;
    # between 0.087 and 0.1 rad only the upper limit stopped, the lower one
    # still ran at about a third of the asked speed.
    assert sent_speed(UPPER - gap, +0.5) == 0
    assert sent_speed(LOWER + gap, -0.5) == 0


def test_moving_away_from_a_limit_is_allowed_at_both_ends():
    assert sent_speed(UPPER - 0.05, -0.5) == -0.5
    assert sent_speed(LOWER + 0.05, +0.5) == +0.5


def test_outside_0p1_rad_the_taper_is_unchanged_and_symmetric():
    up = sent_speed(UPPER - 0.2, +0.5)
    down = sent_speed(LOWER + 0.2, -0.5)
    assert up == pytest.approx(0.5 * 0.2 / THRESH)
    assert down == pytest.approx(-up)
