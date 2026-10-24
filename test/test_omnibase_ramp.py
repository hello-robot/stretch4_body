#!/usr/bin/env python3
"""
OmniBase.set_velocity wheel ramp rates.

The wheel boards each slew their own ramp speed toward v_des at a_des
(VelocityGenerator in hello_stepper2). set_velocity picks the three a_des so the
wheels arrive together, which keeps the base on the straight line from the old
twist to the new one. Before this change it measured the remaining change from
the status speed, which is 10 Hz filtered, one server cycle old and noisy, so
the ratios moved with the noise. Now it keeps a copy of each board's ramp from
the commands it sent. These tests drive the real set_velocity against three
simulated boards, no hardware. The noise levels are assumptions: the wheel
status speed noise has not been measured yet.
"""
import math

import numpy as np

from stretch4_body.core.stepper import Stepper
from stretch4_body.subsystem.omnibase import OmniBase

CYCLE = 0.01          # body server cycle
SUB = 10              # board steps per server cycle in the simulation
STATUS_LPF_HZ = 10.0  # vel_status_LPF on the wheel boards
MOVING_RAD_S = math.radians(3.5)   # vel_near_setpoint_d, the board's is_moving threshold


class SimWheel:
    """One wheel board in MODE_VEL_TRAJ: ramp with the 1 s velocity watchdog,
    shaft follows the ramp, status speed through the 10 Hz filter plus noise,
    read once per server cycle."""

    def __init__(self, name, rng=None, sigma=0.0):
        self.name = name
        self.board_info = {'protocol_version': 'p8'}
        self.params = {'gains': {'enable_vel_watchdog': 1}}
        self.status = {'vel': 0.0, 'mode': Stepper.MODE_VEL_TRAJ, 'is_moving': False}
        self.ramp = 0.0
        self.v_des = 0.0
        self.a_des = 0.0
        self.since_cmd = 0.0
        self.filtered = 0.0
        self.rng = rng
        self.sigma = sigma

    def set_command(self, mode=None, v_des=None, a_des=None, **_):
        self.v_des = v_des
        self.a_des = a_des
        self.since_cmd = 0.0

    def step(self, dt):
        target = self.v_des if self.since_cmd < 1.0 else 0.0
        self.since_cmd += dt
        lim = self.a_des * dt
        self.ramp += max(-lim, min(lim, target - self.ramp))
        alpha = math.exp(-2 * math.pi * STATUS_LPF_HZ * dt)
        self.filtered = alpha * self.filtered + (1 - alpha) * self.ramp

    def publish_status(self):
        noise = self.rng.normal(0, self.sigma) if self.rng is not None else 0.0
        self.status['vel'] = self.filtered + noise
        self.status['is_moving'] = abs(self.status['vel']) > MOVING_RAD_S


class Sim:
    def __init__(self, use_ramp_model=True, sigma=0.0, seed=0):
        rng = np.random.default_rng(seed)
        self.base = OmniBase()
        self.base.wheels = [SimWheel('w%d' % i, rng, sigma) for i in range(3)]
        self.t = 0.0
        self.base._clock = lambda: self.t
        self.use_ramp_model = use_ramp_model
        # The base is parked in MODE_VEL_TRAJ at zero, as after any earlier
        # drive: the boards' ramps sit at exactly zero and the copy knows it.
        # (The very first velocity command after a mode change starts from the
        # status speed instead; test_first_command_... covers that.)
        self.base._ramp = {'t': -10.0, 'v': np.zeros(3), 'a': np.ones(3), 'g': np.zeros(3), 'watchdog': True}
        self.log = []   # base twist from the wheels' ramp speeds, every board step

    def cycle(self, twist, a_m=0.25, a_r=2.0):
        for w in self.base.wheels:
            w.publish_status()
        if self.use_ramp_model:
            self.base.set_velocity(*twist, a_m=a_m, a_r=a_r)
        else:
            self._set_velocity_from_measured(twist, a_m, a_r)
        self.idle(1)

    def _set_velocity_from_measured(self, twist, a_m, a_r):
        """set_velocity as it was before this change: ramp ratios from the
        status speed (compute_motor_acceleration reads it)."""
        b = self.base
        a_m_wheel = (2 / b.params['wheel_diameter_m']) * (a_m + b.params['base_radius_m'] * a_r)
        u = b.base_vel_to_motor_vel(twist)
        aa = b.compute_motor_acceleration(u, a_m_wheel)
        for w, v, a in zip(b.wheels, u, aa):
            w.set_command(mode=Stepper.MODE_VEL_TRAJ, v_des=v, a_des=abs(a))

    def idle(self, cycles):
        for _ in range(cycles * SUB):
            for w in self.base.wheels:
                w.step(CYCLE / SUB)
            self.t += CYCLE / SUB
            self.log.append(self.base.motor_vel_to_base_vel(self.ramp_speeds()))

    def ramp_speeds(self):
        return np.array([w.ramp for w in self.base.wheels])

    def yaw_deg(self):
        return math.degrees(sum(tw[2] for tw in self.log) * CYCLE / SUB)


def start_stop_yaw(use_ramp_model, sigma, seed):
    """Straight ahead from rest to 0.3 m/s and back to rest, streamed at 100 Hz.
    A base that ramps its wheels in proportion turns by exactly zero."""
    sim = Sim(use_ramp_model, sigma, seed)
    for _ in range(150):
        sim.cycle((0.3, 0.0, 0.0))
    for _ in range(150):
        sim.cycle((0.0, 0.0, 0.0))
    return abs(sim.yaw_deg())


def test_noise_free_both_ways_drive_straight():
    assert start_stop_yaw(False, 0.0, 0) < 1e-9
    assert start_stop_yaw(True, 0.0, 0) < 1e-9


def test_status_noise_no_longer_turns_the_base():
    # sigma in motor rad/s on each wheel's status speed (assumed, not measured)
    for sigma in (0.3, 1.0, 3.0):
        before = np.mean([start_stop_yaw(False, sigma, s) for s in range(5)])
        after = np.mean([start_stop_yaw(True, sigma, s) for s in range(5)])
        print(f'status noise {sigma} rad/s: yaw per start and stop, measured {before:.3f} deg, ramp copy {after:.6f} deg')
        assert after < 1e-6
        assert before > 0.01


def change_mid_ramp_deviation(use_ramp_model, sigma=0.0, seed=0):
    """Forward 0.3 m/s from rest; 0.3 s in, still ramping, the stick moves to a
    forward-left diagonal with some rotation. Largest distance of the base
    twist from the straight line between the twist at the change and the new
    target."""
    sim = Sim(use_ramp_model, sigma, seed)
    for _ in range(30):
        sim.cycle((0.3, 0.0, 0.0))
    v0 = sim.base.motor_vel_to_base_vel(sim.ramp_speeds())
    second = (0.15, 0.2, 0.5)
    v1 = np.array(second)
    start = len(sim.log)
    for _ in range(200):
        sim.cycle(second)
    seg = v1 - v0
    dev = 0.0
    for tw in sim.log[start:]:
        s = np.clip(np.dot(tw - v0, seg) / np.dot(seg, seg), 0, 1)
        dev = max(dev, float(np.linalg.norm(tw - (v0 + s * seg))))
    return dev


def test_change_mid_ramp_stays_on_the_line():
    lag_only_before = change_mid_ramp_deviation(False)
    lag_only_after = change_mid_ramp_deviation(True)
    noisy_before = change_mid_ramp_deviation(False, 1.0, 1)
    noisy_after = change_mid_ramp_deviation(True, 1.0, 1)
    print(f'largest twist deviation from the line: lag only {lag_only_before:.5f} -> {lag_only_after:.5f}, '
          f'with 1 rad/s status noise {noisy_before:.5f} -> {noisy_after:.5f}')
    assert lag_only_after < 1e-9
    assert noisy_after < 1e-9
    assert noisy_before > lag_only_before


def test_steady_twist_reaches_the_same_wheel_speeds():
    for model in (False, True):
        sim = Sim(model)
        for _ in range(300):
            sim.cycle((0.3, 0.0, 0.0))
        target = sim.base.base_vel_to_motor_vel((0.3, 0.0, 0.0))
        assert np.allclose(sim.ramp_speeds(), target, atol=1e-9)


def test_streaming_the_same_twist_never_sends_zero_acceleration():
    """a_des = 0 would stop a board's ramp where it is."""
    sim = Sim(True)
    for _ in range(100):
        sim.cycle((0.3, 0.0, 0.0))
        for w in sim.base.wheels:
            assert w.a_des > 0


def test_copy_follows_the_board_watchdog():
    sim = Sim(True)
    for _ in range(200):
        sim.cycle((0.3, 0.0, 0.0))
    sim.idle(300)   # commands stop for 3 s: the boards ramp to zero after 1 s
    assert np.allclose(sim.ramp_speeds(), 0.0)
    assert np.allclose(sim.base._ramp_wheel_speeds(sim.t), 0.0)


def test_first_command_starts_from_status_or_zero_at_rest():
    base = OmniBase()
    base.wheels = [SimWheel('w%d' % i) for i in range(3)]
    base._clock = lambda: 0.0
    for w, v, m in zip(base.wheels, (1.0, -2.0, 0.01), (True, True, False)):
        w.status['vel'] = v
        w.status['is_moving'] = m
    assert np.allclose(base._start_wheel_speeds(), [1.0, -2.0, 0.0])
    u = base.base_vel_to_motor_vel((0.2, 0.0, 0.0))
    base._vel_traj_motor_acceleration(u, 3.0)
    assert np.allclose(base._ramp['g'], [1.0, -2.0, 0.0])


def test_falls_back_when_a_board_left_vel_traj():
    sim = Sim(True)
    for _ in range(10):
        sim.cycle((0.3, 0.0, 0.0))
    assert sim.base._ramp_wheel_speeds(sim.t) is not None
    sim.base.wheels[1].status['mode'] = Stepper.MODE_SAFETY
    assert sim.base._ramp_wheel_speeds(sim.t) is None


def test_other_modes_forget_the_ramp():
    sim = Sim(True)
    sim.cycle((0.3, 0.0, 0.0))
    for w in sim.base.wheels:
        w.enable_freewheel = lambda: None
        w.enable_hold = lambda: None
    sim.base.enable_freewheel_mode()
    assert sim.base._ramp is None
    sim.cycle((0.3, 0.0, 0.0))
    sim.base.enable_hold_mode()
    assert sim.base._ramp is None
