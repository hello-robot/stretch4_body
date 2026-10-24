#!/usr/bin/env python3
"""
OmniBase odometry: pose from the change in wheel encoder position.

Before, _update_odom integrated the 10 Hz filtered wheel speed times the host
time between calls. The filter lags the wheels, so a start and stop lost
distance only if the calls were regular, and any late or missed call (a busy
host, a slow USB read) was integrated at the wrong speed. The encoder
position does not lag and does not depend on when it is read. No hardware.
"""
import math

import numpy as np
import pytest

from stretch4_body.subsystem.omnibase import OmniBase

STATUS_LPF_HZ = 10.0


class SimWheel:
    def __init__(self):
        self.status = {'pos': 0.0, 'vel': 0.0}
        self.pos = 0.0
        self.vel = 0.0
        self.filtered = 0.0

    def step(self, v, dt):
        self.vel = v
        self.pos += v * dt
        alpha = math.exp(-2 * math.pi * STATUS_LPF_HZ * dt)
        self.filtered = alpha * self.filtered + (1 - alpha) * v

    def publish(self):
        self.status['pos'] = self.pos
        self.status['vel'] = self.filtered


class Sim:
    def __init__(self, from_encoders=True):
        self.base = OmniBase()
        self.base.wheels = [SimWheel() for _ in range(3)]
        self.t = 0.0
        self.base._odom_clock = lambda: self.t
        self.from_encoders = from_encoders
        self.true = np.zeros(3)   # x, y, theta, integrated finely
        self.last_read = 0.0
        for w in self.base.wheels:   # first read at rest, as at startup
            w.publish()
        self.base._update_odom(None if from_encoders else 0.0)

    def run(self, twist_of_t, duration, read_period, late=None):
        """late: function of read index giving extra delay of that read (s)."""
        dt = 1e-4
        next_read = self.t + read_period
        k = 0
        n = int(round(duration / dt))
        for _ in range(n):
            tw = np.array(twist_of_t(self.t))
            u = self.base.base_vel_to_motor_vel(tw)
            for w, v in zip(self.base.wheels, u):
                w.step(v, dt)
            th = self.true[2]
            c, s = math.cos(th), math.sin(th)
            self.true += np.array([c * tw[0] - s * tw[1], s * tw[0] + c * tw[1], tw[2]]) * dt
            self.t += dt
            if self.t >= next_read - 1e-12:
                for w in self.base.wheels:
                    w.publish()
                if self.from_encoders:
                    self.base._update_odom(None)
                else:
                    self.base._update_odom(self.t - self.last_read)   # speed x host dt, as before
                self.last_read = self.t
                k += 1
                next_read = self.t + read_period + (late(k) if late else 0.0)

    def pose(self):
        s = self.base.status
        return np.array([s['x'], s['y'], s['theta']])


def trapezoid(v, accel, t_cruise):
    t_r = v / accel
    def f(t):
        if t < t_r:
            return accel * t
        if t < t_r + t_cruise:
            return v
        return max(0.0, v - accel * (t - t_r - t_cruise))
    return f, 2 * t_r + t_cruise + 0.3


def test_straight_distance_while_moving_and_after():
    """The filtered speed lags the wheel by about 1/(2 pi 10 Hz) = 16 ms, so
    speed x time is short by speed x 16 ms while moving and catches up once
    the filter settles after a stop. The encoder answer has no lag."""
    prof, T = trapezoid(0.3, 0.25, 1.0)
    for enc in (False, True):
        sim = Sim(enc)
        sim.run(lambda t: (prof(t), 0.0, 0.0), 1.6, 0.01)    # in the cruise
        moving = sim.pose()[0] - sim.true[0]
        sim.run(lambda t: (prof(t), 0.0, 0.0), T - 1.6, 0.01)
        settled = sim.pose()[0] - sim.true[0]
        print(f'{"encoders" if enc else "speed x time"}: x error at 0.3 m/s {moving*1000:.2f} mm, '
              f'0.3 s after the stop {settled*1000:.3f} mm')
        if enc:
            assert abs(moving) < 1e-6 and abs(settled) < 1e-6
        else:
            assert moving < -0.003


def test_late_reads_do_not_change_the_answer():
    prof, T = trapezoid(0.3, 0.25, 1.0)
    rng = np.random.default_rng(3)
    late = lambda k: float(rng.choice([0.0, 0.0, 0.0, 0.03, 0.08]))
    errs = {}
    for enc in (False, True):
        sim = Sim(enc)
        sim.run(lambda t: (prof(t), 0.0, 0.0), T, 0.01, late=late)
        errs[enc] = np.linalg.norm(sim.pose()[:2] - sim.true[:2])
    print(f'with late reads up to 80 ms: position error from speed x time {errs[False]*1000:.2f} mm, '
          f'from encoders {errs[True]*1000:.4f} mm')
    assert errs[True] < 1e-6
    assert errs[False] > errs[True]


def test_arc_heading_and_position():
    # 0.2 m/s forward, 0.5 rad/s turn for 3 s, read at 10 Hz (a slow client).
    sim = Sim(True)
    sim.run(lambda t: (0.2, 0.0, 0.5), 3.0, 0.1)
    err = sim.pose() - sim.true
    print(f'arc read at 10 Hz: x {err[0]*1000:.3f} mm, y {err[1]*1000:.3f} mm, theta {math.degrees(err[2]):.4f} deg')
    assert abs(err[2]) < 1e-9
    assert np.linalg.norm(err[:2]) < 1e-3     # midpoint heading, chord of a 0.02 m step


def test_a_position_jump_is_not_taken_as_motion():
    sim = Sim(True)
    sim.run(lambda t: (0.2, 0.0, 0.0), 0.5, 0.01)
    before = sim.pose()
    for w in sim.base.wheels:
        w.pos += 500.0        # board reset or position re-marked
    sim.run(lambda t: (0.0, 0.0, 0.0), 0.02, 0.01)
    assert np.linalg.norm(sim.pose()[:2] - before[:2]) < 0.01


def test_a_given_dt_integrates_speed_as_before():
    base = OmniBase()
    base.wheels = [SimWheel() for _ in range(3)]
    u = base.base_vel_to_motor_vel((0.2, 0.0, 0.0))
    for w, v in zip(base.wheels, u):
        w.status['vel'] = v
    base._update_odom(0.5)
    assert base.status['x'] == pytest.approx(0.1)
    assert base.status['x_vel'] == pytest.approx(0.2)
