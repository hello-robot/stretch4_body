#!/usr/bin/env python3
"""
Wrist and gripper set_velocity by a moving goal (moving_goal.py, FeetechSMHello).

No hardware. A simulated Feetech servo in position mode follows its goal with
its own trapezoid profile (max speed and acceleration of the wrist yaw
params), its position is read back once per cycle, and set_velocity is called
from a loop with timing jitter like the end of arm loop. The servo model is an
assumption: how closely a real STS servo follows a goal moved every 20 ms is
the bench question this branch leaves open.
"""
import math
import logging

import numpy as np
import pytest

from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
from stretch4_body.core.feetech.moving_goal import MovingGoal

V_MAX = 12.0    # rad/s, wrist yaw motion max
A_MAX = 12.0    # rad/s^2
LO, HI = -1.13, 4.27   # rad, wrist yaw range_deg [-65, 245]
CYCLE = 0.02    # end of arm loop, 50 Hz


class SimServo:
    """Position mode: moves toward the goal at up to V_MAX, A_MAX, stopping on it."""

    def __init__(self, x=0.0):
        self.x = x
        self.v = 0.0
        self.goal = x
        self.blocked = False
        self.writes = []
        self.torque_toggles = 0
        self.last_comm_success = True

    def go_to_pos(self, t):
        self.goal = float(t)
        self.writes.append(self.goal)

    def disable_torque(self):
        self.torque_toggles += 1

    def enable_torque(self):
        pass

    def enable_pos(self):
        pass

    def step(self, dt):
        if self.blocked:
            self.v = 0.0
            return
        d = self.goal - self.x
        v_stop = math.copysign(math.sqrt(2 * A_MAX * abs(d)), d)
        v_target = max(-V_MAX, min(V_MAX, v_stop))
        dv = A_MAX * dt
        self.v += max(-dv, min(dv, v_target - self.v))
        self.x += self.v * dt


def make_joint(servo):
    j = FeetechSMHello.__new__(FeetechSMHello)
    j.name = 'wrist_yaw'
    j.logger = logging.getLogger('test')
    j.motor = servo
    j.hw_valid = True
    j.was_runstopped = False
    j.in_vel_mode = False
    j.is_direct = False
    j._is_moving = False
    j._prev_set_vel_ts = None
    j.params = {'req_calibration': 1,
                'motion': {'default': {'accel': 7.0, 'vel': 7.0}, 'max': {'accel': A_MAX, 'vel': V_MAX},
                           'vel_is_moving_thresh': 0.01}}
    j.status = {'pos': servo.x, 'pos_ticks': servo.x, 'pos_calibrated': True, 'torque_enabled': True,
                'in_collision_stop': {'pos': False, 'neg': False}, 'watchdog_errors': 0}
    j.comm_errors = type('C', (), {'add_error': lambda *a, **k: None})()
    j.get_soft_motion_limits = lambda: [LO, HI]
    j.world_rad_to_ticks = lambda r: r
    j.ticks_to_world_rad = lambda t: t
    j.set_motion_params = lambda v_des=None, a_des=None, force=False: None
    j._vel_goal = MovingGoal()
    j._vel_goal_timeout_s = 1.0
    j._vel_goal_last_cmd = None
    j.t = 0.0
    j._clock = lambda: j.t
    return j


class Rig:
    def __init__(self, x0=0.0, jitter_s=0.0, seed=0):
        self.servo = SimServo(x0)
        self.j = make_joint(self.servo)
        self.rng = np.random.default_rng(seed)
        self.jitter_s = jitter_s
        self.trace = []   # (t, servo x, servo v)

    def advance(self, dt):
        n = max(1, int(round(dt / 1e-3)))
        for _ in range(n):
            self.servo.step(dt / n)
            self.j.t += dt / n
            self.trace.append((self.j.t, self.servo.x, self.servo.v))
        self.j.status['pos'] = self.servo.x
        self.j.status['pos_ticks'] = self.servo.x

    def stream(self, v, a, duration, how='set_velocity'):
        t_end = self.j.t + duration
        while self.j.t < t_end - 1e-9:
            if how == 'set_velocity':
                self.j.set_velocity(v, a)
            else:   # what the gamepad and drivers do today: move_by(v x nominal dt)
                self.j.move_by(v * CYCLE, V_MAX, a)
            self.advance(CYCLE + (self.rng.uniform(-1, 1) * self.jitter_s))

    def speeds(self, t0, t1):
        return np.array([v for t, x, v in self.trace if t0 <= t <= t1])


def patch_move_to(j):
    """move_to without the servo plumbing it does not need here."""
    def move_to(x_des, v_des=None, a_des=None):
        j._leave_vel_goal()
        j.motor.go_to_pos(x_des)
    j.move_to = move_to


def test_steady_speed_is_the_commanded_speed():
    r = Rig()
    r.stream(1.0, 7.0, 2.0)
    s = r.speeds(1.0, 2.0)
    print(f'set_velocity 1.0 rad/s: servo speed mean {s.mean():.3f}, std {s.std():.3f} rad/s')
    assert s.mean() == pytest.approx(1.0, abs=0.02)


def test_distance_is_speed_times_time_and_move_by_droops():
    """2 s at 1 rad/s with +-8 ms of loop jitter. The goal moves by speed x
    the measured time, so the joint covers about 1 rad/s x 2 s less the ramp.
    Streaming move_by(v x 20 ms), what the gamepad and drivers send today,
    adds each step to the measured position, which trails the servo's goal,
    so the joint covers about half the distance in this model."""
    out = {}
    for how in ('set_velocity', 'move_by'):
        r = Rig(jitter_s=0.008, seed=2)
        patch_move_to(r.j)
        x0, t0 = r.servo.x, r.j.t
        r.stream(1.0, 7.0, 2.0, how)
        out[how] = (r.servo.x - x0, r.j.t - t0)
    (d, T), (d2, _) = out['set_velocity'], out['move_by']
    print(f'2 s at 1 rad/s with +-8 ms jitter: moving goal {d:.3f} rad, move_by stream {d2:.3f} rad, '
          f'asked {T:.3f} rad')
    ramp = 1.0 / 7.0 / 2   # distance lost to the 7 rad/s^2 ramp
    assert d == pytest.approx(T - ramp, abs=0.05)
    assert d2 < 0.7 * T


def test_never_re_based_on_the_measured_position():
    r = Rig()
    r.stream(1.0, 7.0, 0.5)
    x_goal = r.j._vel_goal.x
    r.j.status['pos'] -= 0.1      # a stale or noisy status read, 0.1 rad behind
    r.j.set_velocity(1.0, 7.0)
    assert r.j._vel_goal.x >= x_goal  # the goal kept going; it did not jump back 0.1


def test_stops_on_the_limit_without_passing_it():
    r = Rig(x0=HI - 0.6)
    r.stream(3.0, 7.0, 2.0)
    assert max(r.servo.writes) <= HI + 1e-12
    xs = [x for t, x, v in r.trace]
    print(f'toward the upper limit at 3 rad/s: highest servo position {max(xs) - HI:+.4f} rad from the limit')
    assert max(xs) <= HI + 1e-3
    assert r.servo.x == pytest.approx(HI, abs=1e-3)


def test_zero_ramps_down_and_holds_without_torque_toggle():
    r = Rig()
    r.stream(2.0, 7.0, 1.0)
    r.stream(0.0, 7.0, 1.0)
    held = r.servo.x
    r.stream(0.0, 7.0, 0.5)
    assert r.servo.torque_toggles == 0
    assert abs(r.servo.x - held) < 1e-4   # settling of the servo model only
    # ramp 2 rad/s to 0 at 7 rad/s^2: 0.29 s, 0.29 rad, plus what the lead had queued
    assert r.servo.writes[-1] == pytest.approx(r.j._vel_goal.x)


def test_watchdog_stops_a_goal_that_stopped_getting_commands():
    r = Rig()
    r.stream(1.0, 7.0, 0.5)
    r.advance(0.9)
    r.j._step_vel_goal_watchdog()
    assert r.j._vel_goal.v != 0.0           # not yet: 0.9 s
    r.advance(0.2)
    r.j._step_vel_goal_watchdog()
    assert r.j._vel_goal.v == 0.0
    assert r.servo.writes[-1] == r.j._vel_goal.x
    assert r.j.status['watchdog_errors'] == 1


def test_blocked_joint_does_not_lunge_when_freed():
    r = Rig()
    r.stream(1.0, 7.0, 0.5)
    r.servo.blocked = True
    r.stream(1.0, 7.0, 1.0)
    lag = r.j._vel_goal.x - r.servo.x
    assert lag <= 0.5 + 1e-9                 # vel_goal_max_lag_rad
    print(f'blocked for 1 s at 1 rad/s: goal ahead of the joint by {lag:.2f} rad (1.0 rad without the bound)')


def test_collision_direction_ramps_to_a_stop_other_direction_moves():
    r = Rig()
    r.stream(1.0, 7.0, 0.5)
    r.j.status['in_collision_stop']['pos'] = True
    r.stream(1.0, 7.0, 0.5)
    assert r.j._vel_goal.v == 0.0
    r.stream(-1.0, 7.0, 0.5)
    assert r.j._vel_goal.v < 0


def test_position_commands_leave_the_moving_goal():
    r = Rig()
    patch_move_to(r.j)
    r.stream(1.0, 7.0, 0.3)
    r.j.move_to(0.0)
    assert not r.j._vel_goal.active
    r.advance(1.0)
    r.j.set_velocity(0.5, 7.0)
    assert r.j._vel_goal.active
    assert r.j._vel_goal.x == pytest.approx(r.servo.x, abs=1e-6)   # restarted from where the joint is


def test_zero_when_not_moving_sends_nothing():
    r = Rig()
    r.j.set_velocity(0.0, 7.0)
    assert r.servo.writes == []
    assert not r.j._vel_goal.active
