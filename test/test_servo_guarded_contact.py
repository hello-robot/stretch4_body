#!/usr/bin/env python3
import pytest
from stretch4_body.subsystem.end_of_arm.servo_guarded_contact import ServoGuardedContact

PARAMS = {
    'enable_guarded_mode': 1,
    'drive_deadband_rad': 0.01,
    'medium_strength': {'contact_mA_pos': 100.0, 'contact_mA_neg': 100.0, 'contact_s': 0.15},
    'low_strength': {'contact_mA_pos': 65.0, 'contact_mA_neg': 65.0, 'contact_s': 0.1},
    'high_strength': {'contact_mA_pos': 150.0, 'contact_mA_neg': 150.0, 'contact_s': 0.25},
}


def drive_into_obstacle(g, pos=0.3, goal=1.0, current_mA=160.0, t0=0.0, duration=0.5, dt=0.02):
    """Command a goal past an obstacle at pos and push on it. Returns True if it tripped."""
    g.set_goal(goal)
    t = t0
    while t <= t0 + duration:
        if g.step(pos, current_mA, t):
            return True
        t += dt
    return False


def test_disabled_by_default():
    g = ServoGuardedContact()
    assert not g.enabled
    g.set_goal(1.0)
    assert not g.step(0.3, 500.0, 0.0)
    assert not g.step(0.3, 500.0, 5.0)


def test_needs_a_default_sensitivity():
    with pytest.raises(ValueError):
        ServoGuardedContact({'enable_guarded_mode': 1})


def test_sustained_current_trips_after_contact_s():
    g = ServoGuardedContact(PARAMS)
    g.set_goal(1.0)
    assert not g.step(0.3, 160.0, 0.0)
    assert not g.step(0.3, 160.0, 0.15)  # Not longer than contact_s yet
    assert g.step(0.3, 160.0, 0.16)
    assert g.in_event
    assert g.trip_direction == 1
    assert g.n_events == 1
    assert not g.step(0.3, 160.0, 0.5)  # Only the first sample of an event reports it


def test_negative_direction_trip():
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g, pos=0.3, goal=-1.0)
    assert g.trip_direction == -1


def test_pushing_while_still_moving_trips():
    """A hand that gives way: the joint keeps moving, but its current stays high. Measured on the pitch."""
    g = ServoGuardedContact(PARAMS)
    g.set_goal(2.0)
    tripped = [g.step(1.4 + 0.016 * i, 300.0, 0.02 * i) for i in range(20)]
    assert any(tripped)


def test_acceleration_spike_is_not_contact():
    """Free motion at teleop speed: ~160 mA peaks, but never held over 65 mA for more than 0.04 s."""
    g = ServoGuardedContact(PARAMS)
    g.set_sensitivity('low_strength')
    g.set_goal(1.0)
    currents = [0, 13, 45, 70, 130, 160, 120, 40, 70, 25, 0, 0]
    for i, c in enumerate(currents):
        assert not g.step(0.05 * i, c, 0.02 * i)
    assert not g.in_event


def test_holding_a_load_is_not_contact():
    """At its goal (within the deadband) the joint is not driven, however hard it works to stay there."""
    g = ServoGuardedContact(PARAMS)
    assert not drive_into_obstacle(g, pos=0.995, goal=1.0, current_mA=500.0, duration=3.0)


def test_one_degree_short_counts_as_driving():
    """Yaw stalled against a hand 1 deg short of its goal drew ~160 mA; that has to count."""
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g, pos=1.0 - 0.0175, goal=1.0)


def test_current_under_threshold_is_not_contact():
    g = ServoGuardedContact(PARAMS)
    assert not drive_into_obstacle(g, current_mA=95.0, duration=3.0)


def test_dip_restarts_timer():
    g = ServoGuardedContact(PARAMS)
    g.set_goal(1.0)
    assert not g.step(0.3, 160.0, 0.0)
    assert not g.step(0.3, 160.0, 0.12)
    assert not g.step(0.3, 50.0, 0.14)  # Dropped under
    assert not g.step(0.3, 160.0, 0.16)
    assert not g.step(0.3, 160.0, 0.30)
    assert g.step(0.3, 160.0, 0.32)


def test_negative_current_counts():
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g, goal=-1.0, current_mA=-160.0)


def test_pushing_on_keeps_the_event():
    """Holding a teleop button into the obstacle: each command is refused and keeps the joint held."""
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g)
    assert g.driving_direction(0.3) == 0  # Held where it tripped, not driven
    t = 1.0
    for _ in range(20):  # 15 Hz teleop, ~1.3 s
        assert not g.allow_goal(0.3 + 0.26, 0.3, t)
        assert not g.check_release(t)
        t += 1 / 15
    assert g.in_event
    assert not g.allow_velocity(0.5, t)


def test_released_after_commands_stop():
    """Letting go of the button: no command for release_after_s clears the event."""
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g)
    assert not g.allow_goal(0.56, 0.3, 1.0)
    assert not g.check_release(1.0 + g.release_after_s)
    assert g.check_release(1.0 + g.release_after_s + 0.01)
    assert not g.in_event
    assert g.allow_goal(0.56, 0.3, 2.0)  # Pressing again goes through, and can trip again
    assert drive_into_obstacle(g, goal=0.56, t0=10.0)
    assert g.n_events == 2


def test_release_timer_starts_at_the_trip():
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g)
    assert not g.check_release(5.0)  # First check after the trip starts the timer
    assert g.allow_goal(0.56, 0.3, 5.0 + g.release_after_s + 0.01)  # A late command is a new press


def test_backing_off_releases():
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g)
    assert g.allow_goal(0.0, 0.3, 1.0)
    assert not g.in_event


def test_goal_at_current_position_releases():
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g)
    assert g.allow_goal(0.305, 0.3, 1.0)  # Within the deadband: a stop
    assert not g.in_event


def test_streamed_goals_do_not_restart_timer():
    """Button teleop sends a new move_by goal every cycle while held; contact must still trip."""
    g = ServoGuardedContact(PARAMS)
    tripped = False
    for i in range(30):
        g.set_goal(0.3 + 0.26)  # 15 deg ahead of the stalled joint, every cycle
        tripped |= g.step(0.3, 160.0, 0.02 * i)
    assert tripped


def test_reverse_velocity_releases():
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g)
    assert g.allow_velocity(-0.5, 1.0)
    assert not g.in_event


def test_hold_releases_and_stops_driving():
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g)
    g.note_hold()
    assert not g.in_event
    assert g.driving_direction(0.3) == 0
    assert g.n_events == 1


def test_velocity_mode_trips():
    g = ServoGuardedContact(PARAMS)
    g.set_velocity(-0.5)
    assert not g.step(0.3, 160.0, 0.0)
    assert g.step(0.3, 160.0, 0.2)
    assert g.trip_direction == -1


def test_velocity_mode_at_soft_limit_is_not_contact():
    g = ServoGuardedContact(PARAMS)
    g.set_velocity(0.5)
    at_limit = {'pos': True, 'neg': False}
    assert not g.step(4.0, 160.0, 0.0, at_limit)
    assert not g.step(4.0, 160.0, 2.0, at_limit)
    g.set_velocity(-0.5)  # Driving away from that limit still counts
    assert not g.step(4.0, 160.0, 3.0, at_limit)
    assert g.step(4.0, 160.0, 3.2, at_limit)


def test_sensitivity_profiles():
    g = ServoGuardedContact(PARAMS)
    assert g.sensitivity == 'medium_strength'
    assert not drive_into_obstacle(g, current_mA=80.0, duration=2.0)

    assert g.set_sensitivity('low_strength')
    assert g.contact_mA_pos == 65.0
    assert drive_into_obstacle(g, current_mA=80.0, t0=10.0)

    g.note_hold()
    assert g.set_sensitivity('high_strength')
    assert not drive_into_obstacle(g, current_mA=130.0, t0=20.0, duration=2.0)


def test_pos_and_neg_thresholds_follow_driving_direction():
    params = dict(PARAMS, medium_strength={'contact_mA_pos': 100.0, 'contact_mA_neg': 200.0, 'contact_s': 0.15})
    g = ServoGuardedContact(params)
    assert drive_into_obstacle(g, goal=1.0, current_mA=150.0)  # Positive: over 100 mA
    assert g.trip_direction == 1

    g.note_hold()
    assert not drive_into_obstacle(g, goal=-1.0, current_mA=150.0, t0=10.0, duration=2.0)  # Negative: under 200
    assert drive_into_obstacle(g, goal=-1.0, current_mA=250.0, t0=20.0)
    assert g.trip_direction == -1
    assert g.contact_mA(1) == 100.0 and g.contact_mA(-1) == 200.0


def test_unknown_sensitivity_keeps_current():
    g = ServoGuardedContact(PARAMS)
    g.set_sensitivity('low_strength')
    assert not g.set_sensitivity('sensitivity_bogus')
    assert g.sensitivity == 'low_strength'
    assert g.contact_mA_pos == 65.0


def test_reset_keeps_event_count():
    g = ServoGuardedContact(PARAMS)
    assert drive_into_obstacle(g)
    g.note_hold()
    assert drive_into_obstacle(g, t0=10.0)
    assert g.n_events == 2

