#!/usr/bin/env python3
import pytest
from stretch4_body.subsystem.end_of_arm.contact_guard import ContactGuard

PARAMS = {'enabled': 1, 'contact_mA': 200.0, 'contact_s': 0.06, 'squeeze_rad': 0.1}
OPEN_PARAMS = dict(PARAMS, open_contact_mA=100.0, open_contact_s=0.5, open_backoff_rad=0.02)


def make_contact(cg, pos=1.0, t0=0.0):
    """Close toward 0 and push on an object at pos. Returns the virtual goal."""
    assert cg.filter_goal(0.0) == 0.0
    assert cg.step(pos, 300.0, t0) is None
    assert cg.step(pos - 0.01, 400.0, t0 + 0.03) is None
    return cg.step(pos - 0.02, 500.0, t0 + 0.07)


def test_disabled_by_default():
    assert not ContactGuard().enabled
    assert not ContactGuard({'enabled': 0}).enabled


def test_free_close_no_contact():
    cg = ContactGuard(PARAMS)
    cg.filter_goal(0.0)
    for i in range(50):
        assert cg.step(1.5 - i * 0.02, 50.0, i * 0.02) is None
    assert not cg.in_contact


def test_contact_latches_first_high_current_pos():
    cg = ContactGuard(PARAMS)
    vg = make_contact(cg, pos=1.0)
    assert cg.in_contact
    assert cg.contact_pos == pytest.approx(1.0)
    assert vg == pytest.approx(0.9)


def test_brief_current_spike_is_not_contact():
    cg = ContactGuard(PARAMS)
    cg.filter_goal(0.0)
    assert cg.step(1.0, 400.0, 0.0) is None
    assert cg.step(0.98, 50.0, 0.03) is None
    assert cg.step(0.96, 400.0, 0.08) is None
    assert not cg.in_contact


def test_high_current_while_opening_is_not_contact():
    cg = ContactGuard(PARAMS)
    cg.filter_goal(2.0)  # Opening toward the open hardstop
    for i in range(10):
        assert cg.step(1.99, 300.0, i * 0.05) is None
    assert not cg.in_contact


def test_closing_commands_clamped_in_contact():
    cg = ContactGuard(PARAMS)
    vg = make_contact(cg)
    assert cg.filter_goal(0.0) == vg  # 'close' again
    assert cg.filter_goal(0.5) == vg  # move_by closing
    assert cg.in_contact


def test_less_squeeze_passes_and_stays_in_contact():
    cg = ContactGuard(PARAMS)
    make_contact(cg)
    assert cg.filter_goal(0.95) == 0.95
    assert cg.in_contact


def test_open_past_contact_releases():
    cg = ContactGuard(PARAMS)
    make_contact(cg)
    assert cg.filter_goal(1.5) == 1.5
    assert not cg.in_contact
    assert cg.filter_goal(0.0) == 0.0  # Free to close again (re-arms detection)


def test_virtual_goal_not_past_user_goal():
    cg = ContactGuard(PARAMS)
    cg.filter_goal(0.95)  # Asked to close only slightly past the object
    cg.step(1.0, 300.0, 0.0)
    vg = cg.step(0.99, 300.0, 0.07)
    assert vg == pytest.approx(0.95)


def test_velocity_close_then_hold():
    cg = ContactGuard(PARAMS)
    assert cg.filter_velocity(-1.0) is None  # Closing in velocity mode passes through
    cg.step(1.0, 300.0, 0.0)
    vg = cg.step(0.99, 300.0, 0.07)
    assert vg == pytest.approx(0.9)
    assert cg.filter_velocity(-1.0) == vg  # Further closing holds
    assert cg.filter_velocity(0.0) == vg
    assert cg.filter_velocity(1.0) is None  # Opening releases
    assert not cg.in_contact



def test_open_hardstop_current_is_not_contact():
    """Pushing open past the hardstop, then closing: the leftover opening current must not latch contact."""
    cg = ContactGuard(PARAMS)
    cg.filter_goal(2.03)  # Goal past the open hardstop at 1.98
    for i in range(10):
        assert cg.step(1.98, 400.0, i * 0.02) is None
    cg.filter_goal(0.0)  # Close
    for i in range(10):  # Current still high from pushing open, gripper hasn't moved yet
        assert cg.step(1.98, 400.0, 0.2 + i * 0.02) is None
    assert not cg.in_contact
    for i in range(20):  # Now closing freely
        assert cg.step(1.9 - i * 0.05, 50.0, 0.4 + i * 0.02) is None
    assert not cg.in_contact


def test_contact_after_open_hardstop_still_detected():
    cg = ContactGuard(PARAMS)
    cg.filter_goal(2.03)
    cg.step(1.98, 400.0, 0.0)
    cg.filter_goal(0.0)
    cg.step(1.98, 400.0, 0.02)  # Leftover opening current
    cg.step(1.5, 50.0, 0.2)  # Closing freely
    assert cg.step(1.0, 300.0, 0.4) is None  # Hits an object
    vg = cg.step(0.99, 300.0, 0.47)
    assert cg.in_contact
    assert vg == pytest.approx(0.9)


def make_open_contact(cg, pos=1.98, t0=0.0):
    """Open toward a goal past an open hardstop at pos and stall there. Returns the virtual goal."""
    assert cg.filter_goal(2.03) == 2.03
    for i in range(5):
        assert cg.step(pos, 150.0, t0 + i * 0.1) is None
    return cg.step(pos, 150.0, t0 + 0.55)


def test_open_stall_holds_short_of_contact():
    cg = ContactGuard(OPEN_PARAMS)
    vg = make_open_contact(cg)
    assert cg.in_contact
    assert cg.contact_dir == 1
    assert cg.contact_pos == pytest.approx(1.98)
    assert vg == pytest.approx(1.96)


def test_open_guard_disabled_without_open_contact_mA():
    cg = ContactGuard(PARAMS)
    assert make_open_contact(cg) is None
    assert not cg.in_contact


def test_free_open_no_contact():
    cg = ContactGuard(OPEN_PARAMS)
    cg.filter_goal(2.0)
    for i in range(50):  # Brief acceleration spike, then low current while moving
        assert cg.step(0.5 + i * 0.03, 300.0 if i < 5 else 50.0, i * 0.02) is None
    assert not cg.in_contact


def test_opening_commands_clamped_in_open_contact():
    cg = ContactGuard(OPEN_PARAMS)
    vg = make_open_contact(cg)
    assert cg.filter_goal(2.03) == vg  # 'open' again
    assert cg.filter_goal(1.97) == vg  # move_by opening
    assert cg.filter_goal(vg) == vg
    assert cg.in_contact


def test_close_past_open_contact_releases():
    cg = ContactGuard(OPEN_PARAMS)
    make_open_contact(cg)
    assert cg.filter_goal(1.5) == 1.5
    assert not cg.in_contact
    assert cg.filter_goal(2.03) == 2.03  # Free to open again (re-arms detection)


def test_velocity_open_then_hold():
    cg = ContactGuard(OPEN_PARAMS)
    assert cg.filter_velocity(1.0) is None  # Opening in velocity mode passes through
    cg.step(1.98, 150.0, 0.0)
    vg = cg.step(1.98, 150.0, 0.55)
    assert vg == pytest.approx(1.96)
    assert cg.filter_velocity(1.0) == vg  # Further opening holds
    assert cg.filter_velocity(0.0) == vg
    assert cg.filter_velocity(-1.0) is None  # Closing releases
    assert not cg.in_contact


def test_squeeze_current_is_not_open_contact():
    """Opening off a grasped object: the leftover squeeze current must not latch an open contact."""
    cg = ContactGuard(OPEN_PARAMS)
    make_contact(cg, pos=1.0)
    cg.step(0.98, 300.0, 0.1)  # Holding the object
    cg.filter_goal(2.03)  # Open: releases the closing contact
    assert not cg.in_contact
    for i in range(10):  # Current still high from squeezing, gripper hasn't moved yet
        assert cg.step(0.98, 300.0, 0.2 + i * 0.1) is None
    assert not cg.in_contact
    for i in range(20):  # Now opening freely
        assert cg.step(1.0 + i * 0.04, 50.0, 1.2 + i * 0.02) is None
    assert not cg.in_contact


def test_open_stall_after_grasp_still_detected():
    cg = ContactGuard(OPEN_PARAMS)
    make_contact(cg, pos=1.0)
    cg.step(0.98, 300.0, 0.1)
    cg.filter_goal(2.03)
    cg.step(0.98, 300.0, 0.2)  # Leftover squeeze current
    cg.step(1.5, 50.0, 0.4)  # Opening freely
    assert cg.step(1.98, 150.0, 0.6) is None  # Hits the open hardstop
    vg = cg.step(1.98, 150.0, 1.15)
    assert cg.in_contact
    assert cg.contact_dir == 1
    assert vg == pytest.approx(1.96)


def test_close_contact_after_open_contact_still_detected():
    cg = ContactGuard(OPEN_PARAMS)
    make_open_contact(cg)
    cg.filter_goal(0.0)  # Close: releases the open contact
    cg.step(1.96, 150.0, 0.6)  # Leftover opening current
    cg.step(1.5, 50.0, 0.8)  # Closing freely
    assert make_contact(cg, pos=1.0, t0=1.0) == pytest.approx(0.9)
    assert cg.contact_dir == -1
