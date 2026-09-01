#!/usr/bin/env python3
"""
Trajectory tracking test for the lift and arm, driven through RobotClient.

Three reference profiles are built over the same waypoint schedule and streamed
to the joint as position setpoints:

  linear  -- position waypoints only. Velocity is piecewise constant, so it
             steps at every knot (C0).
  cubic   -- position + velocity waypoints, Hermite cubic per segment. Velocity
             is continuous, acceleration steps at the knots (C1).
  quintic -- position + velocity + acceleration waypoints. Acceleration is
             continuous as well (C2).

The point of the test is the comparison: the same motion, the same joint, the
same streaming rate, with only the smoothness of the reference changing. The
plots put commanded and measured next to each other so the velocity ripple that
the linear profile injects at each knot is visible against the quintic one.

Usage
-----
  python3 test_trajectory_control.py                    # run on the robot
  python3 test_trajectory_control.py --dry-run          # no hardware: math + reference plots
  python3 test_trajectory_control.py --joints lift      # one joint
  python3 test_trajectory_control.py --profiles quintic
  python3 test_trajectory_control.py --amplitude 0.05 --rate 50

Motion is bounded: each joint moves within +/- amplitude of a start position
that is itself clamped into a safe band well inside the joint's soft limits.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

from scipy.signal import savgol_filter

import matplotlib
matplotlib.use('Agg')  # write PNGs; never needs a display
import matplotlib.pyplot as plt


# ############################ Trajectory math ############################
# A segment is a polynomial in local time tau = t - t0, evaluated by Horner.
# Degree is set by how much of the waypoint state is pinned down: position only
# gives degree 1, adding velocity gives 3, adding acceleration gives 5.

def linear_segment(duration, x0, x1):
    """Coefficients [c0, c1] for the degree-1 segment joining two positions."""
    return [x0, (x1 - x0) / duration]


def cubic_segment(duration, x0, v0, x1, v1):
    """Coefficients [c0..c3] for the Hermite cubic matching position+velocity."""
    d = duration
    return [x0,
            v0,
            (3.0 * (x1 - x0) / d ** 2) - (2.0 * v0 / d) - (v1 / d),
            (-2.0 * (x1 - x0) / d ** 3) + ((v1 + v0) / d ** 2)]


def quintic_segment(duration, x0, v0, a0, x1, v1, a1):
    """Coefficients [c0..c5] matching position+velocity+acceleration at both ends."""
    d = duration
    return [x0,
            v0,
            a0 / 2.0,
            (20.0 * (x1 - x0) - (8.0 * v1 + 12.0 * v0) * d - (3.0 * a0 - a1) * d ** 2) / (2.0 * d ** 3),
            (-30.0 * (x1 - x0) + (14.0 * v1 + 16.0 * v0) * d + (3.0 * a0 - 2.0 * a1) * d ** 2) / (2.0 * d ** 4),
            (12.0 * (x1 - x0) - 6.0 * (v1 + v0) * d + (a1 - a0) * d ** 2) / (2.0 * d ** 5)]


def evaluate_polynomial_at(coeffs, tau):
    """Return (pos, vel, accel) of a polynomial and its first two derivatives."""
    pos = vel = acc = 0.0
    for i in range(len(coeffs) - 1, -1, -1):
        pos = pos * tau + coeffs[i]
        if i >= 1:
            vel = vel * tau + i * coeffs[i]
        if i >= 2:
            acc = acc * tau + i * (i - 1) * coeffs[i]
    return pos, vel, acc


def differentiate(t, y, window_s=0.06):
    """d/dt of a measured signal, smoothed enough to be readable.

    A raw difference of the reported velocity is dominated by its own noise, so
    this fits a local quadratic (Savitzky-Golay) over `window_s` and takes that
    fit's derivative. The window is a deliberate low-pass: it passes the
    drivetrain oscillation the joints actually exhibit (order 10 Hz) and
    suppresses what sits above it. Anything read off this row is a filtered
    estimate, not a measurement.
    """
    t, y = np.asarray(t, dtype=float), np.asarray(y, dtype=float)
    dt = float(np.median(np.diff(t)))
    window = int(round(window_s / dt))
    window = max(5, window + 1 - window % 2)   # odd, and >= polyorder + 2
    if window >= len(y):
        return np.gradient(y, t)
    return savgol_filter(y, window_length=window, polyorder=2, deriv=1, delta=dt)


def finite_difference_derivative(t, y):
    """Centered differences inside, zero at the ends.

    Used to fill in the velocity (and then acceleration) waypoints that the
    cubic and quintic profiles need but the caller only gives positions for.
    Endpoints are pinned to zero so every profile starts and finishes at rest.
    """
    t, y = np.asarray(t, dtype=float), np.asarray(y, dtype=float)
    d = np.zeros_like(y)
    d[1:-1] = (y[2:] - y[:-2]) / (t[2:] - t[:-2])
    return d


class Trajectory:
    """A piecewise polynomial over a waypoint schedule.

    Parameters
    ----------
    t : list(float)
        Waypoint times (seconds, monotonically increasing, starting at 0).
    x : list(float)
        Waypoint positions (meters).
    degree : str
        'linear', 'cubic' or 'quintic'.
    """

    DEGREES = ('linear', 'cubic', 'quintic')

    def __init__(self, t, x, degree):
        if degree not in self.DEGREES:
            raise ValueError(f'degree must be one of {self.DEGREES}, got {degree!r}')
        t, x = np.asarray(t, dtype=float), np.asarray(x, dtype=float)
        if len(t) != len(x) or len(t) < 2:
            raise ValueError('need at least two waypoints, and one position per time')
        if np.any(np.diff(t) <= 0):
            raise ValueError('waypoint times must be strictly increasing')

        self.t, self.x, self.degree = t, x, degree
        self.v = finite_difference_derivative(t, x) if degree in ('cubic', 'quintic') else np.zeros_like(x)
        self.a = finite_difference_derivative(t, self.v) if degree == 'quintic' else np.zeros_like(x)

        self.segments = []
        for i in range(len(t) - 1):
            d = t[i + 1] - t[i]
            if degree == 'linear':
                c = linear_segment(d, x[i], x[i + 1])
            elif degree == 'cubic':
                c = cubic_segment(d, x[i], self.v[i], x[i + 1], self.v[i + 1])
            else:
                c = quintic_segment(d, x[i], self.v[i], self.a[i], x[i + 1], self.v[i + 1], self.a[i + 1])
            self.segments.append(c)

    @property
    def duration(self):
        return float(self.t[-1])

    def evaluate(self, t_s):
        """(pos, vel, accel) at time t_s. Clamped to the endpoints outside the span."""
        if t_s <= self.t[0]:
            return float(self.x[0]), 0.0, 0.0
        if t_s >= self.t[-1]:
            return float(self.x[-1]), 0.0, 0.0
        i = int(np.searchsorted(self.t, t_s, side='right') - 1)
        return evaluate_polynomial_at(self.segments[i], t_s - self.t[i])

    def sample(self, dt):
        """Dense (t, pos, vel, accel) arrays over the whole trajectory."""
        ts = np.arange(0.0, self.duration + dt, dt)
        pva = np.array([self.evaluate(t) for t in ts])
        return ts, pva[:, 0], pva[:, 1], pva[:, 2]

    def peak_velocity(self, dt=0.001):
        return float(np.max(np.abs(self.sample(dt)[2])))

    def peak_acceleration(self, dt=0.001):
        return float(np.max(np.abs(self.sample(dt)[3])))


# ######################## Continuity self-check ########################
# Runs without hardware. A profile that fails these is not the profile it
# claims to be, and any tracking numbers gathered with it are meaningless.

def check_continuity(traj, rel_tol=1e-9):
    """Verify the profile is as smooth as its degree promises at every knot.

    Each knot is evaluated exactly from both sides -- the end of the outgoing
    segment and the start of the incoming one -- so there is no sampling error
    to tolerance away. Returns a list of failure strings; empty means the
    profile is well formed.
    """
    expected_continuous = {'linear': 0, 'cubic': 1, 'quintic': 2}[traj.degree]
    scale = {0: max(np.max(np.abs(traj.x)), 1e-3),
             1: max(traj.peak_velocity(), 1e-3),
             2: max(traj.peak_acceleration(), 1e-3)}
    failures = []
    for i in range(1, len(traj.t) - 1):
        knot = float(traj.t[i])
        before = evaluate_polynomial_at(traj.segments[i - 1], traj.t[i] - traj.t[i - 1])
        after = evaluate_polynomial_at(traj.segments[i], 0.0)
        for order, label in ((0, 'position'), (1, 'velocity'), (2, 'acceleration')):
            if order > expected_continuous:
                continue
            jump = abs(after[order] - before[order])
            if jump > rel_tol * scale[order] + 1e-9:
                failures.append(f'{traj.degree}: {label} discontinuous at '
                                f't={knot:.2f}s (jump {jump:.3e})')
    return failures


# ############################ Robot execution ############################

class TrackingRun:
    """One trajectory executed on one joint, with everything recorded."""

    def __init__(self, joint_name, degree, traj, control='position'):
        self.joint_name, self.degree, self.traj = joint_name, degree, traj
        self.control = control
        self.t = []           # seconds since motion start
        self.x_cmd = []       # setpoint actually sent (a position, or a velocity
                              # in velocity control -- read it with self.control)
        self.x_ref = []       # reference position at the sample instant
        self.v_ref = []
        self.x_meas = []
        self.v_meas = []

    def append(self, t, x_cmd, x_ref, v_ref, x_meas, v_meas):
        self.t.append(t)
        self.x_cmd.append(x_cmd)
        self.x_ref.append(x_ref)
        self.v_ref.append(v_ref)
        self.x_meas.append(x_meas)
        self.v_meas.append(v_meas)

    def finalize(self):
        for k in ('t', 'x_cmd', 'x_ref', 'v_ref', 'x_meas', 'v_meas'):
            setattr(self, k, np.asarray(getattr(self, k), dtype=float))

    # -- metrics -----------------------------------------------------------

    @property
    def a_ref(self):
        """Reference acceleration, evaluated analytically from the spline."""
        return np.array([self.traj.evaluate(t)[2] for t in self.t])

    @property
    def a_meas(self):
        """Acceleration differentiated from the measured velocity.

        Lightly smoothed, so the drivetrain resonance survives -- this is what
        the joint is really doing, oscillation included.
        """
        return differentiate(self.t, self.v_meas, window_s=0.06)

    @property
    def a_meas_trend(self):
        """The same derivative, smoothed past the resonance.

        The resonance dominates the raw derivative by roughly 2x the reference's
        own amplitude, which makes the two impossible to compare by eye. This
        wider window is for reading the trend against the reference; `a_meas` is
        for seeing the oscillation.
        """
        return differentiate(self.t, self.v_meas, window_s=0.25)

    @property
    def pos_error(self):
        return self.x_meas - self.x_ref

    @property
    def vel_error(self):
        return self.v_meas - self.v_ref

    def save(self, path):
        """Persist the raw run so a plot can be redrawn without moving the robot."""
        np.savez(path, t=self.t, x_cmd=self.x_cmd, x_ref=self.x_ref, v_ref=self.v_ref,
                 x_meas=self.x_meas, v_meas=self.v_meas,
                 waypoint_t=self.traj.t, waypoint_x=self.traj.x,
                 joint_name=self.joint_name, degree=self.degree, control=self.control)
        return path

    @classmethod
    def load(cls, path):
        """Rebuild a run saved by save(), so plots can be redrawn offline."""
        d = np.load(path, allow_pickle=True)
        traj = Trajectory(d['waypoint_t'], d['waypoint_x'], str(d['degree']))
        control = str(d['control']) if 'control' in d.files else 'position'
        run = cls(str(d['joint_name']), str(d['degree']), traj, control=control)
        for k in ('t', 'x_cmd', 'x_ref', 'v_ref', 'x_meas', 'v_meas'):
            setattr(run, k, np.asarray(d[k], dtype=float))
        return run

    def metrics(self):
        e = self.pos_error
        return {
            'rms_pos_error_mm': float(np.sqrt(np.mean(e ** 2)) * 1000.0),
            'max_pos_error_mm': float(np.max(np.abs(e)) * 1000.0),
            'final_pos_error_mm': float(abs(e[-1]) * 1000.0),
            'rms_vel_error_mm_s': float(np.sqrt(np.mean(self.vel_error ** 2)) * 1000.0),
            'sample_rate_hz': float(len(self.t) / (self.t[-1] - self.t[0])) if len(self.t) > 1 else 0.0,
        }


def move_to_start(robot, joint, x_start, timeout=20.0):
    """Get the joint to the trajectory's first waypoint before streaming."""
    joint.move_to(x_start, v_m=0.05, a_m=0.1)
    robot.push_command()
    robot.wait_on_motion_finish([joint.name], timeout=timeout)
    robot.pull_status()


def acceleration_limit(joint, traj):
    """The a_m to hand the joint, floored at its own default acceleration.

    Scaling the reference's peak acceleration alone is not safe: a linear
    profile has *zero* acceleration inside every segment, so the scaled value
    collapses to the additive term and the joint is left barely able to change
    velocity. That is harmless in position mode -- it just rounds the corners --
    but in velocity mode a_m is also the deceleration used to stop, so a limit
    that small turns a halt into a slow coast. Floor it at the joint's own
    default so every profile gets an authority the hardware is built for.
    """
    return float(np.clip(max(traj.peak_acceleration() * 3.0,
                             joint.params['motion']['default']['accel_m']),
                         0.0, joint.params['motion']['max']['accel_m']))


def execute_trajectory(robot, joint, traj, rate_hz, control='position', **kwargs):
    """Dispatch to the position- or velocity-control follower."""
    if control == 'position':
        return execute_trajectory_position(robot, joint, traj, rate_hz, **kwargs)
    if control == 'velocity':
        return execute_trajectory_velocity(robot, joint, traj, rate_hz, **kwargs)
    raise ValueError(f'unknown control mode {control!r}')


def execute_trajectory_position(robot, joint, traj, rate_hz, lookahead_s=None,
                                settle_s=0.5, stiffness=None, vel_kp=None):
    """Stream `traj` as position setpoints via move_to().

    The setpoint sent at time t is the reference at t + lookahead, which covers
    the one-cycle transport delay between pushing a command and the joint acting
    on it. Without it every run carries a fixed phase lag that has nothing to do
    with the profile being compared.

    The joint's own trapezoidal generator closes the loop; v_m/a_m are limits on
    it, not feedforward, so this mode is pure position feedback against a moving
    target.
    """
    dt = 1.0 / rate_hz
    if lookahead_s is None:
        lookahead_s = dt
    run = TrackingRun(joint.name, traj.degree, traj, control='position')

    # Enough headroom that the joint's own trapezoidal generator is never the
    # limiter -- what we want to measure is how well it chases the spline.
    v_limit = min(traj.peak_velocity() * 3.0 + 0.02, joint.params['motion']['max']['vel_m'])
    a_limit = acceleration_limit(joint, traj)

    robot.pull_status()
    t_start = time.time()
    t_now = 0.0
    while t_now < traj.duration + settle_s:
        t_now = time.time() - t_start
        robot.pull_status()

        x_ref, v_ref, _ = traj.evaluate(t_now)
        x_cmd, _, _ = traj.evaluate(t_now + lookahead_s)
        joint.move_to(x_cmd, v_m=v_limit, a_m=a_limit, stiffness=stiffness)
        robot.push_command()   # push_command() paces itself to the server's max rate

        run.append(t_now, x_cmd, x_ref, v_ref,
                   joint.status['pos'], joint.status['vel'])

        sleep_s = dt - ((time.time() - t_start) - t_now)
        if sleep_s > 0:
            time.sleep(sleep_s)

    run.finalize()
    return run


# The joint has no velocity watchdog (`PrismaticJoint.watchdog_enabled` is False),
# so a commanded velocity persists until something replaces it. Everything below
# is written on that assumption: the loop is wrapped so that no exit path --
# normal, exception, or Ctrl-C -- leaves a nonzero velocity standing.
VELOCITY_KP = 6.0          # 1/s. Closes a position error over roughly 1/Kp seconds.
VELOCITY_GUARD_M = 0.02    # abort if the joint escapes the planned span by this much


def execute_trajectory_velocity(robot, joint, traj, rate_hz, lookahead_s=None,
                                settle_s=0.5, stiffness=None, vel_kp=None):
    """Stream `traj` as velocity commands via set_velocity().

    Velocity mode has no position setpoint, so the reference velocity alone is
    open-loop: any error integrates and never comes back. The commanded velocity
    is therefore the reference velocity as feedforward plus proportional feedback
    on the position error,

        v_cmd = v_ref(t) + Kp * (x_ref(t) - x_meas)

    which is the one structural advantage this mode has over move_to() here --
    v_ref enters as a genuine feedforward term, where move_to()'s v_m is only a
    limit on the onboard generator.
    """
    dt = 1.0 / rate_hz
    if lookahead_s is None:
        lookahead_s = dt
    kp = VELOCITY_KP if vel_kp is None else vel_kp
    run = TrackingRun(joint.name, traj.degree, traj, control='velocity')

    a_limit = acceleration_limit(joint, traj)
    # Stopping is not a tracking concern -- give the halt the joint's full
    # authority so the guard below is a hard stop rather than a slow coast.
    a_halt = joint.params['motion']['max']['accel_m']
    v_max = min(traj.peak_velocity() * 2.0 + 0.05, joint.params['motion']['max']['vel_m'])
    span_lo, span_hi = float(np.min(traj.x)) - VELOCITY_GUARD_M, float(np.max(traj.x)) + VELOCITY_GUARD_M

    def halt():
        joint.set_velocity(0.0, a_m=a_halt)
        robot.push_command()

    robot.pull_status()
    t_start = time.time()
    t_now = 0.0
    try:
        while t_now < traj.duration + settle_s:
            t_now = time.time() - t_start
            robot.pull_status()
            x_meas, v_meas = joint.status['pos'], joint.status['vel']

            # The server tapers velocity inside its own brake zone near the soft
            # limits; this is a tighter guard around the planned span, so a bad
            # gain shows up as an abort rather than a run to the limit.
            if not (span_lo <= x_meas <= span_hi):
                raise RuntimeError(
                    f'{joint.name} left the planned span at {x_meas:.4f} m '
                    f'(allowed {span_lo:.4f}..{span_hi:.4f}); velocity command halted')

            x_ref, v_ref, _ = traj.evaluate(t_now)
            _, v_ff, _ = traj.evaluate(t_now + lookahead_s)
            v_cmd = float(np.clip(v_ff + kp * (x_ref - x_meas), -v_max, v_max))
            joint.set_velocity(v_cmd, a_m=a_limit, stiffness=stiffness)
            robot.push_command()

            run.append(t_now, v_cmd, x_ref, v_ref, x_meas, v_meas)

            sleep_s = dt - ((time.time() - t_start) - t_now)
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        halt()
        robot.wait_on_motion_finish([joint.name], timeout=5.0, wait_on_motion_start=False)

    run.finalize()
    return run


# ############################### Plotting ###############################
# Two series only -- commanded and measured -- so identity never rests on color
# alone: each is direct-labeled in the legend and the pair is CVD-validated.

REFERENCE = '#2a78d6'   # categorical slot 1
MEASURED = '#eb6834'    # categorical slot 2
ERROR = '#4a3aa7'       # categorical slot 7
INK = '#0b0b0b'
INK_MUTED = '#52514e'
GRID = '#dcdcd8'


def _style_axis(ax, xlabel=None, ylabel=None, title=None):
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_MUTED, labelsize=8, length=3)
    if xlabel:
        ax.set_xlabel(xlabel, color=INK_MUTED, fontsize=9)
    if ylabel:
        ax.set_ylabel(ylabel, color=INK_MUTED, fontsize=9)
    if title:
        ax.set_title(title, color=INK, fontsize=10, pad=8)


def plot_joint_runs(joint_name, runs, out_path):
    """One figure per joint: profiles across columns, quantities down rows."""
    degrees = [r.degree for r in runs]
    fig, axes = plt.subplots(4, len(runs), figsize=(4.6 * len(runs), 11.5),
                             sharex=True, squeeze=False)
    fig.patch.set_facecolor('#fcfcfb')

    for col, run in enumerate(runs):
        m = run.metrics()

        ax = axes[0][col]
        ax.plot(run.t, run.x_ref, color=REFERENCE, linewidth=2.0, label='Reference')
        ax.plot(run.t, run.x_meas, color=MEASURED, linewidth=2.0, label='Measured')
        ax.plot(run.traj.t, run.traj.x, linestyle='none', marker='o', markersize=5,
                markerfacecolor='none', markeredgecolor=INK_MUTED, label='Waypoint')
        _style_axis(ax, ylabel='Position (m)' if col == 0 else None,
                    title=f'{run.degree.capitalize()}')
        if col == 0:
            ax.legend(frameon=False, fontsize=8, labelcolor=INK_MUTED, loc='best')

        ax = axes[1][col]
        ax.plot(run.t, run.v_ref, color=REFERENCE, linewidth=2.0)
        ax.plot(run.t, run.v_meas, color=MEASURED, linewidth=1.4, alpha=0.9)
        ax.axhline(0.0, color=GRID, linewidth=0.8)
        _style_axis(ax, ylabel='Velocity (m/s)' if col == 0 else None)

        ax = axes[2][col]
        ax.plot(run.t, run.a_meas, color=MEASURED, linewidth=0.9, alpha=0.35)
        ax.plot(run.t, run.a_ref, color=REFERENCE, linewidth=2.0)
        ax.plot(run.t, run.a_meas_trend, color=MEASURED, linewidth=1.8)
        ax.axhline(0.0, color=GRID, linewidth=0.8)
        _style_axis(ax, ylabel='Acceleration (m/s$^2$)' if col == 0 else None)
        if col == 0:
            ax.annotate('measured = d/dt of reported velocity;\n'
                        'faint = 60 ms window (resonance kept), bold = 250 ms window',
                        xy=(0.02, 0.04), xycoords='axes fraction',
                        color=INK_MUTED, fontsize=7)

        ax = axes[3][col]
        ax.plot(run.t, run.pos_error * 1000.0, color=ERROR, linewidth=1.6)
        ax.axhline(0.0, color=GRID, linewidth=0.8)
        _style_axis(ax, xlabel='Time (s)',
                    ylabel='Tracking error (mm)' if col == 0 else None)
        ax.annotate(f"RMS {m['rms_pos_error_mm']:.1f} mm   max {m['max_pos_error_mm']:.1f} mm",
                    xy=(0.02, 0.06), xycoords='axes fraction',
                    color=INK_MUTED, fontsize=8)

    # Same y-scale per row so the columns are actually comparable.
    for row in axes:
        lo = min(a.get_ylim()[0] for a in row)
        hi = max(a.get_ylim()[1] for a in row)
        for a in row:
            a.set_ylim(lo, hi)

    mode = runs[0].control
    fig.suptitle(f'{joint_name.capitalize()} trajectory tracking, {mode} control: '
                 f'{", ".join(degrees)}', color=INK, fontsize=13, y=0.98)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=140, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out_path


def plot_error_summary(runs_by_joint, out_path):
    """Grouped bars: RMS tracking error per profile, per joint."""
    joints = list(runs_by_joint.keys())
    degrees = [r.degree for r in runs_by_joint[joints[0]]]
    fig, ax = plt.subplots(figsize=(1.6 * len(degrees) + 2.5, 4.2))
    fig.patch.set_facecolor('#fcfcfb')

    width = 0.34
    xs = np.arange(len(degrees))
    colors = (REFERENCE, MEASURED, ERROR)
    for i, joint in enumerate(joints):
        vals = [r.metrics()['rms_pos_error_mm'] for r in runs_by_joint[joint]]
        offset = (i - (len(joints) - 1) / 2.0) * width
        bars = ax.bar(xs + offset, vals, width * 0.92, label=joint.capitalize(),
                      color=colors[i % len(colors)])
        for b, v in zip(bars, vals):
            ax.annotate(f'{v:.1f}', xy=(b.get_x() + b.get_width() / 2, v),
                        xytext=(0, 3), textcoords='offset points',
                        ha='center', color=INK_MUTED, fontsize=8)

    ax.set_xticks(xs)
    ax.set_xticklabels([d.capitalize() for d in degrees], color=INK_MUTED, fontsize=9)
    _style_axis(ax, ylabel='RMS tracking error (mm)',
                title='Tracking error by trajectory profile')
    ax.legend(frameon=False, fontsize=8, labelcolor=INK_MUTED)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out_path


def plot_reference_profiles(traj_by_degree, out_path, title):
    """Hardware-free view of the three references: position, velocity, accel."""
    fig, axes = plt.subplots(3, 1, figsize=(8.0, 8.0), sharex=True)
    fig.patch.set_facecolor('#fcfcfb')
    styles = {'linear': (ERROR, (0, (5, 2))), 'cubic': (MEASURED, (0, (1, 1.4))),
              'quintic': (REFERENCE, 'solid')}

    for degree, traj in traj_by_degree.items():
        ts, pos, vel, acc = traj.sample(0.002)
        color, dash = styles[degree]
        for ax, y in zip(axes, (pos, vel, acc)):
            ax.plot(ts, y, color=color, linewidth=1.8, linestyle=dash,
                    label=degree.capitalize() if ax is axes[0] else None)

    any_traj = next(iter(traj_by_degree.values()))
    axes[0].plot(any_traj.t, any_traj.x, linestyle='none', marker='o', markersize=5,
                 markerfacecolor='none', markeredgecolor=INK_MUTED, label='Waypoint')
    _style_axis(axes[0], ylabel='Position (m)')
    _style_axis(axes[1], ylabel='Velocity (m/s)')
    _style_axis(axes[2], xlabel='Time (s)', ylabel='Acceleration (m/s$^2$)')
    for ax in axes[1:]:
        ax.axhline(0.0, color=GRID, linewidth=0.8)
    axes[0].legend(frameon=False, fontsize=8, labelcolor=INK_MUTED)

    fig.suptitle(title, color=INK, fontsize=13, y=0.98)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=140, facecolor=fig.get_facecolor())
    plt.close(fig)
    return out_path


# ################################ Test ################################

# Offsets are fractions of the amplitude, so one schedule serves both joints.
# Deliberately asymmetric: with a symmetric ramp the finite-difference velocity
# waypoints come out zero and the cubic and quintic profiles collapse onto
# each other, which would make the comparison vacuous.
WAYPOINT_TIMES = [0.0, 1.5, 3.0, 4.5, 6.0]
WAYPOINT_OFFSET_FRACTIONS = [0.0, 0.6, 1.0, -0.4, 0.0]

# A joint is held this far inside its soft limits, and the trajectory itself is
# clipped into what remains.
SAFE_MARGIN_M = {'lift': 0.15, 'arm': 0.05}

# Tolerances are per profile, because the profiles are not equally trackable.
# A linear reference steps its velocity at every knot -- at t=3.0s it reverses
# by ~0.10 m/s instantaneously -- and no joint can follow a velocity step. Its
# tracking error is a property of the reference, not a defect, so holding it to
# the cubic/quintic numbers would bake in a permanent failure. The bound is set
# loose enough to pass the physics and tight enough to catch a regression.
TOLERANCE_MM = {
    'linear': {'max_pos_error_mm': 30.0, 'final_pos_error_mm': 5.0},
    'cubic': {'max_pos_error_mm': 25.0, 'final_pos_error_mm': 5.0},
    'quintic': {'max_pos_error_mm': 25.0, 'final_pos_error_mm': 5.0},
}

# How much better the smooth profiles must track than the linear one, by RMS.
#
# This is mode-dependent because the two modes use the reference differently.
# Velocity control feeds v_ref forward directly, so a profile whose velocity
# steps at every knot is literally unfollowable and the smooth ones win by an
# order of magnitude. Position control never sees v_ref at all -- v_m is only a
# limit on the onboard generator -- so its error is dominated by loop lag, which
# is much the same whichever profile produced the setpoints. Demanding a large
# advantage there would be demanding something the mode cannot deliver.
#
# Both numbers are set below measurement (position 1.0-1.5x, velocity 7.4-11.9x)
# with room for run-to-run scatter: they are regression guards, not targets.
MIN_SMOOTHNESS_ADVANTAGE = {'position': 0.8, 'velocity': 3.0}


def build_trajectory(joint_name, x_center, amplitude_m, degree, soft_limits):
    """Waypoints around x_center, clipped into the joint's safe band."""
    lo, hi = soft_limits[0] + SAFE_MARGIN_M.get(joint_name, 0.05), \
             soft_limits[1] - SAFE_MARGIN_M.get(joint_name, 0.05)
    if lo >= hi:
        raise ValueError(f'{joint_name}: safe band is empty inside {soft_limits}')
    x = [float(np.clip(x_center + f * amplitude_m, lo, hi)) for f in WAYPOINT_OFFSET_FRACTIONS]
    return Trajectory(WAYPOINT_TIMES, x, degree)


def check_smoothness_advantage(joint_name, runs, control='position'):
    """Assert the smooth profiles track better than the linear one.

    Only meaningful when linear ran alongside at least one smooth profile, so
    a partial `--profiles` selection quietly skips it rather than failing.
    """
    by_degree = {r.degree: r.metrics()['rms_pos_error_mm'] for r in runs}
    if 'linear' not in by_degree:
        return []
    need = MIN_SMOOTHNESS_ADVANTAGE[control]
    failures = []
    for degree in ('cubic', 'quintic'):
        if degree not in by_degree:
            continue
        ratio = by_degree['linear'] / max(by_degree[degree], 1e-6)
        ok = ratio >= need
        print(f'  [{"PASS" if ok else "FAIL"}] {joint_name}: {degree} RMS beats linear by '
              f'{ratio:.1f}x (need {need:.1f}x in {control} control)')
        if not ok:
            failures.append(f'{joint_name} ({control}): {degree} RMS advantage over '
                            f'linear {ratio:.2f}x < {need:.2f}x')
    return failures


def run_dry(args, out_dir):
    """Validate the trajectory math and plot the references. No robot needed."""
    print('Dry run: trajectory math and reference profiles only.\n')
    failures = []
    results = []
    for joint_name in args.joints:
        center = {'lift': 0.6, 'arm': 0.25}.get(joint_name, 0.25)
        limits = {'lift': (0.0, 1.2), 'arm': (0.0, 0.55)}.get(joint_name, (0.0, 0.5))
        trajs = {}
        for degree in args.profiles:
            traj = build_trajectory(joint_name, center, args.amplitude, degree, limits)
            trajs[degree] = traj
            problems = check_continuity(traj)
            status = 'PASS' if not problems else 'FAIL'
            print(f'[{status}] {joint_name} {degree:<8} continuity  '
                  f'peak |v| {traj.peak_velocity():.3f} m/s  '
                  f'peak |a| {traj.peak_acceleration():.3f} m/s^2')
            failures += problems
        path = os.path.join(out_dir, f'trajectory_reference_{joint_name}.png')
        plot_reference_profiles(trajs, path, f'{joint_name.capitalize()} reference profiles')
        results.append(path)

    for f in failures:
        print(f'  FAIL: {f}')
    print('\nPlots:')
    for p in results:
        print(f'  {p}')
    return 0 if not failures else 1


def run_replot(args, out_dir):
    """Redraw every plot from the saved runs. No robot, no motion."""
    runs_by_joint = {}
    for joint_name in args.joints:
        runs = []
        for degree in args.profiles:
            path = os.path.join(out_dir,
                                f'run_{joint_name}_{degree}_{args.control}.npz')
            if not os.path.exists(path):
                print(f'SKIP: no saved run at {path}')
                continue
            runs.append(TrackingRun.load(path))
        if runs:
            runs_by_joint[joint_name] = runs
            out = os.path.join(out_dir,
                               f'trajectory_tracking_{joint_name}_{args.control}.png')
            print(f'  plot: {plot_joint_runs(joint_name, runs, out)}')
    if not runs_by_joint:
        print('FAIL: no saved runs found. Run the test on the robot first.')
        return 1
    out = os.path.join(out_dir, f'trajectory_error_summary_{args.control}.png')
    print(f'  plot: {plot_error_summary(runs_by_joint, out)}')
    return 0


def run_on_robot(args, out_dir):
    from stretch4_body.robot.robot_client import RobotClient

    robot = RobotClient()
    if not robot.startup():
        print('FAIL: could not start RobotClient. Is stretch_body_server running?')
        return 1
    try:
        robot.pull_status()
        if not robot.is_homed():
            print('Robot is not homed. Homing now...')
            robot.home()
            robot.pull_status()
            if not robot.is_homed():
                print('FAIL: robot could not be homed.')
                return 1

        runs_by_joint = {}
        failures = []
        for joint_name in args.joints:
            joint = robot.get_subsystem(joint_name)
            if joint is None:
                print(f'SKIP: no subsystem named {joint_name!r}')
                continue
            limits = tuple(joint.params['range_m'])
            runs = []
            for degree in args.profiles:
                robot.pull_status()
                traj = build_trajectory(joint_name, joint.status['pos'], args.amplitude,
                                        degree, limits)
                print(f'\n--- {joint_name} / {degree} ---')
                print(f'  waypoints (m): {["%.3f" % v for v in traj.x]}')
                print(f'  peak |v| {traj.peak_velocity():.3f} m/s, '
                      f'peak |a| {traj.peak_acceleration():.3f} m/s^2')

                problems = check_continuity(traj)
                if problems:
                    failures += problems
                    print('  FAIL: reference profile is malformed, skipping execution')
                    continue

                move_to_start(robot, joint, float(traj.x[0]))
                try:
                    run = execute_trajectory(robot, joint, traj, args.rate,
                                             control=args.control,
                                             stiffness=args.stiffness,
                                             vel_kp=args.vel_kp)
                except RuntimeError as exc:
                    # The follower already halted the joint on its way out. Record
                    # it and keep going, so one bad profile does not cost the
                    # whole matrix.
                    failures.append(f'{joint_name} {degree}: {exc}')
                    print(f'  FAIL: {exc}')
                    continue
                runs.append(run)
                run.save(os.path.join(out_dir,
                                      f'run_{joint_name}_{degree}_{args.control}.npz'))

                m = run.metrics()
                tols = TOLERANCE_MM[degree]
                print(f'  streamed at {m["sample_rate_hz"]:.1f} Hz')
                print(f'  RMS error   {m["rms_pos_error_mm"]:6.2f} mm')
                print(f'  max error   {m["max_pos_error_mm"]:6.2f} mm '
                      f'(tol {tols["max_pos_error_mm"]:.1f})')
                print(f'  final error {m["final_pos_error_mm"]:6.2f} mm '
                      f'(tol {tols["final_pos_error_mm"]:.1f})')
                for key, tol in tols.items():
                    if m[key] > tol:
                        failures.append(f'{joint_name} {degree}: {key} {m[key]:.2f} > {tol:.2f}')
                        print(f'  FAIL: {key} out of tolerance')

            failures += check_smoothness_advantage(joint_name, runs, args.control)

            if runs:
                runs_by_joint[joint_name] = runs
                path = os.path.join(out_dir,
                                    f'trajectory_tracking_{joint_name}_{args.control}.png')
                plot_joint_runs(joint_name, runs, path)
                print(f'\n  plot: {path}')

        if len(runs_by_joint) > 0:
            path = os.path.join(out_dir,
                                f'trajectory_error_summary_{args.control}.png')
            plot_error_summary(runs_by_joint, path)
            print(f'  plot: {path}')

        print('\n' + '=' * 60)
        if failures:
            print(f'FAILED ({len(failures)})')
            for f in failures:
                print(f'  {f}')
            return 1
        print('PASSED')
        return 0
    finally:
        robot.stop()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--joints', nargs='+', default=['lift', 'arm'],
                   help='joints to test (default: lift arm)')
    p.add_argument('--profiles', nargs='+', default=list(Trajectory.DEGREES),
                   choices=Trajectory.DEGREES,
                   help='trajectory profiles to run (default: all three)')
    p.add_argument('--amplitude', type=float, default=0.08,
                   help='trajectory amplitude in meters (default: 0.08)')
    p.add_argument('--rate', type=float, default=100.0,
                   help='setpoint streaming rate in Hz (default: 100, the '
                        'server\'s max_push_command_rate_Hz)')
    p.add_argument('--out-dir', default=None,
                   help='where to write plots (default: alongside this script)')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--position_control', dest='control', action='store_const',
                      const='position',
                      help='follow the trajectory with move_to() position setpoints '
                           '(default)')
    mode.add_argument('--velocity_control', dest='control', action='store_const',
                      const='velocity',
                      help='follow the trajectory with set_velocity(), using v_ref '
                           'as feedforward plus proportional position feedback')
    p.set_defaults(control='position')
    p.add_argument('--vel_kp', type=float, default=None,
                   help=f'position-error gain for --velocity_control in 1/s '
                        f'(default: {VELOCITY_KP})')
    p.add_argument('--stiffness', type=float, default=None,
                   help='position-loop stiffness, 0.0 to 1.0 '
                        '(default: the joint default, which is already 1.0)')
    p.add_argument('--replot', action='store_true',
                   help='redraw the plots from previously saved .npz runs, no robot')
    p.add_argument('--dry-run', action='store_true',
                   help='validate the trajectory math and plot references, no robot')
    args = p.parse_args()

    out_dir = args.out_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                           'trajectory_plots')
    os.makedirs(out_dir, exist_ok=True)

    if args.dry_run:
        return run_dry(args, out_dir)
    if args.replot:
        return run_replot(args, out_dir)
    return run_on_robot(args, out_dir)


if __name__ == '__main__':
    sys.exit(main())
