#!/usr/bin/env python3
"""GuardedWristJoint against a mocked servo: no hardware needed."""
from unittest.mock import MagicMock

import pytest

from stretch4_body.subsystem.end_of_arm.wrist_pitch import WristPitch

OBSTACLE_RAD = 0.3


@pytest.fixture
def wrist():
    w = WristPitch()
    w.motor = MagicMock()
    w.motor.last_comm_success = True
    w.hw_valid = True
    w.status['pos_calibrated'] = True
    w.status['torque_enabled'] = True
    w.update_joint_limits()
    w.ts = 0.0
    w._clock = lambda: w.ts  # Commands and samples on one simulated clock
    sample(w, 0.0)
    return w


def sample(w, pos, vel=0.0, current_mA=0.0, dt=0.02):
    """One status sample, as the end-of-arm loop's sync read delivers it."""
    w.ts += dt
    w.motor.get_pos.return_value = w.world_rad_to_ticks(pos)
    w.pull_status({'x': w.world_rad_to_ticks(pos), 'v': w.world_rad_to_ticks_per_sec(vel),
                   'current_mA': current_mA, 'temp': 30.0, 'ts': w.ts, 'err': 0})


def stall(w, seconds, pos=OBSTACLE_RAD, current_mA=160.0):  # What a push into a hand drew
    for _ in range(int(seconds / 0.02)):
        sample(w, pos, current_mA=current_mA)


def goals_sent(w):
    return [c.args[0] for c in w.motor.go_to_pos.call_args_list]


def test_params_loaded(wrist):
    assert wrist.guarded_contact.enabled
    assert wrist.status['guarded_contact']['sensitivity'] == 'medium_strength'
    assert wrist.status['in_guarded_event'] is False


def test_driving_into_obstacle_trips_and_holds(wrist):
    wrist.move_to(1.0)
    assert goals_sent(wrist) == [wrist.world_rad_to_ticks(1.0)]
    contact_s = wrist.guarded_contact.contact_s
    stall(wrist, contact_s - 0.04)
    assert not wrist.status['in_guarded_event']
    stall(wrist, 0.1)
    assert wrist.status['in_guarded_event']
    assert wrist.status['guarded_event'] == 1
    assert wrist.status['guarded_contact']['trip_direction'] == 1
    # Held where it stalled
    assert goals_sent(wrist)[-1] == wrist.world_rad_to_ticks(OBSTACLE_RAD)


def test_teleop_button_held_then_released(wrist):
    """As the gamepad does: move_by 15 deg every cycle (15 Hz) while the button is held, nothing on release.
    Holding the button keeps the joint held in the event; letting go clears it after release_after_s."""
    goals_at_trip = None
    for _ in range(40):  # ~2.7 s of holding the button
        wrist.move_by(0.26)
        for _ in range(3):  # ~67 ms of samples between teleop cycles
            sample(wrist, OBSTACLE_RAD, current_mA=160.0)
        if wrist.status['in_guarded_event'] and goals_at_trip is None:
            goals_at_trip = len(goals_sent(wrist))
    assert wrist.status['guarded_event'] == 1  # Tripped once, and stayed held
    assert wrist.status['in_guarded_event']
    assert len(goals_sent(wrist)) == goals_at_trip  # Nothing more sent while held
    assert goals_sent(wrist)[-1] == wrist.world_rad_to_ticks(OBSTACLE_RAD)

    stall(wrist, wrist.guarded_contact.release_after_s + 0.04, current_mA=0.0)  # Button let go
    assert not wrist.status['in_guarded_event']
    wrist.move_by(0.26)  # Pressed again: goes through
    assert abs(goals_sent(wrist)[-1] - wrist.world_rad_to_ticks(OBSTACLE_RAD + 0.26)) <= 1


def test_reverse_clears_event_at_once(wrist):
    wrist.move_to(1.0)
    stall(wrist, wrist.guarded_contact.contact_s + 0.06)  # Just tripped
    wrist.move_by(0.2)
    assert wrist.status['in_guarded_event']
    wrist.set_velocity(-0.5)
    wrist.motor.set_vel.assert_called()
    assert not wrist.status['in_guarded_event']


def test_stop_command_clears_event(wrist):
    wrist.move_to(1.0)
    stall(wrist, 0.5)
    wrist.set_velocity(0.0)  # What teleop sends when the button is released
    assert not wrist.status['in_guarded_event']
    stall(wrist, 2.0)  # No longer driven, so does not trip again
    assert wrist.status['guarded_event'] == 1


def test_holding_load_at_goal_does_not_trip(wrist):
    wrist.move_to(OBSTACLE_RAD)
    stall(wrist, 3.0, current_mA=500.0)
    assert not wrist.status['in_guarded_event']


def test_refused_command_does_not_count_as_driving(wrist):
    """A goal the servo never got (collision stop here) must not make a loaded joint look stalled."""
    wrist.status['in_collision_stop']['pos'] = True
    wrist.move_to(1.0)
    wrist.motor.go_to_pos.assert_not_called()
    stall(wrist, 2.0)
    assert not wrist.status['in_guarded_event']


def test_runstop_clears_event(wrist):
    wrist.move_to(1.0)
    stall(wrist, 0.5)
    wrist.was_runstopped = True
    sample(wrist, OBSTACLE_RAD)
    assert not wrist.status['in_guarded_event']


def test_set_sensitivity(wrist):
    wrist.set_guarded_contact_sensitivity('low_strength')
    assert wrist.status['guarded_contact']['contact_mA_pos'] == 65.0
    wrist.move_to(1.0)
    stall(wrist, 0.3, current_mA=80.0)  # Over high's 65 mA, under default's 150
    assert wrist.status['in_guarded_event']

    wrist.set_guarded_contact_sensitivity('sensitivity_bogus')
    assert wrist.status['guarded_contact']['sensitivity'] == 'low_strength'
    wrist.set_guarded_contact_sensitivity(None)
    assert wrist.status['guarded_contact']['sensitivity'] == 'medium_strength'


def test_robot_profiles_name_wrist_sensitivities():
    from stretch4_body.core.robot_params import RobotParams
    params = RobotParams.get_params()[1]
    sensitivities = params['wrist_pitch']['guarded_contact']
    for mode, profile in params['robot']['guarded_contact'].items():
        if isinstance(profile, dict):
            assert profile['end_of_arm'] in sensitivities, mode


def test_reads_current_every_cycle_in_direct_mode(wrist):
    """The base class reads current every 3rd cycle; the guard needs it every cycle."""
    m = wrist.motor
    m.get_pos.return_value = wrist.world_rad_to_ticks(OBSTACLE_RAD)
    m.get_vel.return_value = 0
    m.get_temp.return_value = 30
    m.get_hardware_error.return_value = 0
    m.get_current_mA.return_value = 160.0
    for _ in range(3):
        wrist.pull_status()
    assert m.get_current_mA.call_count == 3
    assert wrist.status['current_mA'] == 160.0


def test_strong_ignores_a_hand(wrist):
    """Strong is nearly no guard: pushing into a hand (up to ~490 mA, briefly) does not trip it."""
    wrist.set_guarded_contact_sensitivity('high_strength')
    wrist.move_to(1.0)
    stall(wrist, 2.0, current_mA=300.0)
    assert not wrist.status['in_guarded_event']
    stall(wrist, 0.3, current_mA=490.0)
    assert not wrist.status['in_guarded_event']
    stall(wrist, 0.6, current_mA=550.0)  # A hard jam at nearly full servo effort
    assert wrist.status['in_guarded_event']


def test_end_of_arm_sets_one_joint_or_all(wrist):
    from stretch4_body.subsystem.end_of_arm.end_of_arm import EndOfArm
    other = MagicMock()
    eoa = EndOfArm.__new__(EndOfArm)
    eoa.motors = {'wrist_pitch': wrist, 'wrist_yaw': other}
    eoa.pt_lock = MagicMock()
    eoa.logger = MagicMock()
    eoa.set_guarded_contact_sensitivity('low_strength', 'wrist_pitch')
    assert wrist.status['guarded_contact']['sensitivity'] == 'low_strength'
    other.set_guarded_contact_sensitivity.assert_not_called()
    eoa.set_guarded_contact_sensitivity(None)
    assert wrist.status['guarded_contact']['sensitivity'] == 'medium_strength'
    other.set_guarded_contact_sensitivity.assert_called_once_with(None)
    eoa.set_guarded_contact_sensitivity('low_strength', 'no_such_joint')
    eoa.logger.warning.assert_called_once()

