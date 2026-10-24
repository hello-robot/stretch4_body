"""
Velocity control for a position servo by a moving goal.

The Feetech servos take a goal position. To move one at a commanded speed the
host keeps a goal of its own and moves it each cycle: the goal's speed ramps
toward the commanded speed at the commanded acceleration, the goal moves by
speed x the measured time since the last cycle, and the servo is sent the goal
plus a short lead so it does not reach the goal and wait between cycles.

Rules, and why:
- The goal is never re-based on the measured position. Re-basing (move_by from
  the present position each cycle) makes every late status read a short step
  and every early one a long step, so the speed follows the timing noise, and a
  joint that lags under load falls further behind every cycle (droop).
- The goal is kept within max_lag of the measured position. A blocked or
  overloaded joint would otherwise see the goal run away from it and lunge to
  catch up once freed. This is the same rule as the stepper VEL_TRAJ error
  bound.
- Near a limit the speed is held to what can still stop at the limit at the
  commanded acceleration, so the goal stops on the limit, not past it.
- Stopping is a ramp of the goal speed to zero, then the goal stays where it
  is. No torque toggle and no re-read of the present position, so a loaded
  joint does not droop or snap when it stops.

Units: radians, seconds. No I/O here; FeetechSMHello does the servo writes.
"""
import math


class MovingGoal:
    def __init__(self, lead_s=0.04, max_lag_rad=0.5, max_dt_s=0.1):
        self.lead_s = lead_s            # goal sent = x + v * lead_s
        self.max_lag_rad = max_lag_rad  # |x - measured| bound
        self.max_dt_s = max_dt_s        # a longer gap between cycles counts as this long
        self.active = False
        self.x = 0.0                    # integrated goal (rad)
        self.v = 0.0                    # goal speed (rad/s)
        self.t = None                   # time of the last step (s)
        self.at_lag_bound = False       # the last step held the goal at max_lag

    def start(self, x_measured, t):
        self.active = True
        self.x = float(x_measured)
        self.v = 0.0
        self.t = t

    def leave(self):
        self.active = False
        self.v = 0.0
        self.t = None

    def freeze(self):
        """Stop the goal where it is. Returns the goal to hold."""
        self.v = 0.0
        return self.x

    def step(self, v_des, a, t, lo=None, hi=None, x_measured=None):
        """
        One cycle. v_des and a are the commanded speed and acceleration
        (a > 0). lo and hi are the soft limits (None for no limit).
        Returns the goal position to send.
        """
        dt = 0.0 if self.t is None else t - self.t
        dt = min(max(dt, 0.0), self.max_dt_s)
        self.t = t
        a = abs(a)

        # Speed that can still stop at the limit ahead.
        v_target = v_des
        if hi is not None:
            v_target = min(v_target, math.sqrt(2 * a * max(hi - self.x, 0.0)))
        if lo is not None:
            v_target = max(v_target, -math.sqrt(2 * a * max(self.x - lo, 0.0)))

        dv = a * dt
        v_new = self.v + max(-dv, min(dv, v_target - self.v))
        self.x += (self.v + v_new) / 2 * dt
        self.v = v_new

        if hi is not None and self.x >= hi:
            self.x, self.v = hi, min(self.v, 0.0)
        if lo is not None and self.x <= lo:
            self.x, self.v = lo, max(self.v, 0.0)

        self.at_lag_bound = False
        if x_measured is not None and self.max_lag_rad > 0:
            bounded = min(max(self.x, x_measured - self.max_lag_rad), x_measured + self.max_lag_rad)
            self.at_lag_bound = bounded != self.x
            self.x = bounded

        goal = self.x + self.v * self.lead_s
        if hi is not None:
            goal = min(goal, hi)
        if lo is not None:
            goal = max(goal, lo)
        return goal
