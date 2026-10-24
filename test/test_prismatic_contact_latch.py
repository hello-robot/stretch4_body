#!/usr/bin/env python3
"""
PrismaticJoint: a streamed set_velocity does not re-push into a contact.

No hardware. The arm's Stepper is replaced by a board model with the
firmware's guarded contact rules as read from HelloController.cpp
(feature/firmware-p14): pushing into a wall raises the effort past the
threshold, the board drops to safety and reports in_guarded_event, and the
next command of any kind re-arms it and takes the new mode. The client
streams set_velocity at 30 Hz into the wall for 3 s (proof plan test 6).
The counts are from this model, not measured.
"""
import pytest

from stretch4_body.core.stepper import Stepper
from stretch4_body.subsystem.arm import Arm

HOST_HZ = 30
WALL_M = 0.30
CONTACT_RELEASE_S = 0.12   # 600 cycles at 5 kHz before the contact flag can clear


class BoardModel:
    def __init__(self, arm):
        self.arm = arm
        self.x = 0.29
        self.v_cmd = 0.0
        self.mode = Stepper.MODE_VEL_TRAJ
        self.trips = 0
        self.contact_until = -1.0
        self.status = {'pos_calibrated': True, 'in_guarded_event': False, 'pos': 0.0, 'vel': 0.0,
                       'mode': self.mode, 'runstop_on': False}
        self.params = arm.motor.params
        self.t = 0.0

    def set_command(self, mode=None, v_des=None, **_):
        self.mode = mode           # any command re-arms (guarded_override cleared)
        self.v_cmd = arm_m(self.arm, v_des) if v_des is not None else 0.0

    def step(self, dt):
        self.t += dt
        if self.mode == Stepper.MODE_SAFETY:
            moving = 0.0
        else:
            moving = self.v_cmd
        self.x = min(WALL_M, self.x + moving * dt)
        pushing = self.mode != Stepper.MODE_SAFETY and self.x >= WALL_M and self.v_cmd > 0
        if pushing and self.t >= self.contact_until:
            self.trips += 1
            self.mode = Stepper.MODE_SAFETY
            self.contact_until = self.t + CONTACT_RELEASE_S
        self.status['in_guarded_event'] = self.mode == Stepper.MODE_SAFETY
        self.status['mode'] = self.mode
        self.status['pos'] = self.arm.translate_m_to_motor_rad(self.x)

    def enable_safety(self):
        self.mode = Stepper.MODE_SAFETY


def arm_m(arm, v_r):
    return arm.motor_rad_to_translate_m(v_r)


def make_arm(latch=True):
    arm = Arm()
    board = BoardModel(arm)
    board.params = arm.motor.params
    arm.motor = board
    arm.status['motor'] = board.status
    arm.params['latch_contact_on_set_velocity'] = 1 if latch else 0
    arm.in_vel_brake_zone = False
    return arm, board


def stream(arm, board, v, seconds):
    n = int(seconds * HOST_HZ)
    for _ in range(n):
        arm.set_velocity(v)
        for _ in range(10):
            board.step(1.0 / HOST_HZ / 10)
        arm._PrismaticJoint__update_status()


def test_streaming_into_a_wall_trips_once():
    trips = {}
    for latch in (False, True):
        arm, board = make_arm(latch)
        stream(arm, board, 0.1, 3.0)
        trips[latch] = board.trips
    print(f'3 s of set_velocity(0.1 m/s) at {HOST_HZ} Hz into a wall: {trips[False]} trips before, {trips[True]} with the latch')
    assert trips[True] == 1
    assert trips[False] > 10


def test_zero_or_reverse_clears_the_latch():
    arm, board = make_arm()
    stream(arm, board, 0.1, 0.5)
    assert arm.status['contact_latch']['pos']
    stream(arm, board, 0.0, 0.1)
    assert not arm.status['contact_latch']['pos']
    stream(arm, board, 0.1, 0.5)      # a new, deliberate push trips again
    assert board.trips == 2


def test_reverse_moves_away_while_latched():
    arm, board = make_arm()
    stream(arm, board, 0.1, 0.5)
    x = board.x
    stream(arm, board, -0.05, 0.5)
    assert board.x < x - 0.02
    assert not arm.status['contact_latch']['pos']


def test_a_trip_during_a_position_move_is_not_latched():
    arm, board = make_arm()
    stream(arm, board, 0.1, 0.05)       # a velocity command, short of the wall
    assert board.trips == 0
    arm._last_vel_cmd = 0.0             # what move_to and move_by set
    board.status['in_guarded_event'] = True
    arm._PrismaticJoint__update_status()
    assert arm.status['contact_latch'] == {'pos': False, 'neg': False}


def test_clear_contact_latch():
    arm, board = make_arm()
    stream(arm, board, 0.1, 0.5)
    arm.clear_contact_latch()
    assert arm.status['contact_latch'] == {'pos': False, 'neg': False}


def test_param_off_keeps_the_old_behaviour():
    arm, board = make_arm(latch=False)
    stream(arm, board, 0.1, 0.5)
    assert arm.status['contact_latch'] == {'pos': False, 'neg': False}
