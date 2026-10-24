#!/usr/bin/env python3
"""
OmniBase.set_velocity limits: a twist over the limits is scaled as a whole,
so the base keeps its direction of travel and its turn radius. No hardware.
"""
import math

import numpy as np
import pytest

from stretch4_body.subsystem.omnibase import OmniBase


@pytest.fixture
def base():
    b = OmniBase()
    b.curr_max_vel_xy_m = 0.3
    b.curr_max_vel_w_r = 2.0
    return b


def heading(vx, vy):
    return math.atan2(vy, vx)


def test_inside_the_limits_is_untouched(base):
    for tw in [(0.2, 0.1, 0.5), (0.0, 0.0, 0.0), (-0.3, 0.0, 2.0), (0.0, -0.1, -1.9)]:
        assert base.scale_twist_to_limits(*tw) == tw


def test_direction_of_travel_is_kept(base):
    vx, vy, wz = base.scale_twist_to_limits(0.6, 0.2, 0.0)
    assert math.isclose(math.hypot(vx, vy), 0.3)
    assert math.isclose(heading(vx, vy), heading(0.6, 0.2))
    # What the old per-axis clip gave: (0.3, 0.2), 15 degrees off course.
    old = (np.clip(0.6, -0.3, 0.3), np.clip(0.2, -0.3, 0.3))
    off = math.degrees(heading(*old) - heading(0.6, 0.2))
    print(f'per-axis clip turned the heading by {off:.1f} degrees')
    assert off > 10


def test_diagonal_is_limited_by_its_length(base):
    vx, vy, _ = base.scale_twist_to_limits(0.3, 0.3, 0.0)
    assert math.isclose(math.hypot(vx, vy), 0.3)


def test_turn_radius_is_kept_when_the_turn_rate_saturates(base):
    vx, vy, wz = base.scale_twist_to_limits(0.3, 0.0, 4.0)
    assert math.isclose(wz, 2.0)
    assert math.isclose(vx / wz, 0.3 / 4.0)          # radius v / w unchanged
    # The old clip kept 0.3 m/s and cut the rate to 2: a 0.075 m radius
    # (0.3 / 4) became 0.15 m (0.3 / 2), a turn twice as wide.


def test_both_limits_take_the_smaller_factor(base):
    vx, vy, wz = base.scale_twist_to_limits(0.6, 0.0, 3.0)   # 0.5 by speed, 0.667 by rate
    assert math.isclose(vx, 0.3) and math.isclose(wz, 1.5)


def test_signs_are_kept(base):
    vx, vy, wz = base.scale_twist_to_limits(-0.6, 0.3, -4.0)
    assert vx < 0 and vy > 0 and wz < 0
