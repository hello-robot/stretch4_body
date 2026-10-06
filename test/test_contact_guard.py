#!/usr/bin/env python3
import pytest
from stretch4_body.subsystem.end_of_arm.contact_guard import ContactGuard

PARAMS = {'enabled': 1, 'contact_mA': 200.0, 'contact_s': 0.06, 'squeeze_rad': 0.1}


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

