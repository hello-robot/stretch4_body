class ServoGuardedContact:
    """
    Guarded contact for a Feetech servo joint (the wrist), in software.

    The steppers (lift, arm, base) have guarded contact in firmware: past a contact current they raise
    in_guarded_event and stop. The Feetech servos have nothing like it, so a wrist driven into an obstacle
    keeps pushing until the servo's overcurrent protection trips. This class is the software equivalent:

    Trip: while the joint is being driven (its goal is ahead of it, or it has a nonzero velocity command),
          its motor current stays over the sensitivity's contact_mA_pos (driven positive) or
          contact_mA_neg (driven negative) for longer than contact_s.
          The caller then holds the joint where it stopped.
    Hold: while in a guarded event, commands that keep driving the way the joint was going are refused.
          Each one also restarts the release timer, so holding a teleop button down keeps the joint held.
    Release: a command that stops the joint or drives it back the way it came clears the event at once.
             Otherwise it clears once no command has arrived for release_after_s (the button was let go),
             and the next command, in any direction, is taken.

    Why current, and why a duration (measured on an SE4 DW4 wrist, Nov 2026):
      - Moving freely the current only spikes while accelerating, and briefly: at teleop speed (7 rad/s,
        7 rad/s^2) it peaked at ~160 mA, but never stayed over 65 mA for more than 0.04 s.
      - Driven into a hand, it stayed over 130 mA for 0.7 s or more, whether the joint stalled or the hand
        gave way and the joint kept moving at full speed. So there is no stall (low velocity) condition.
      - The load register (PWM) is no use: free acceleration reached 47%, more than a stall.
      - A joint that is only holding position barely registers a push (the gearing takes it), so a
        stationary wrist cannot detect being bumped. Only a driven one can.

    Sensitivities are named profiles in the joint's 'guarded_contact' params (sensitivity_default,
    sensitivity_high, sensitivity_low), selected with set_sensitivity().

    Positions are joint radians. This class does no I/O; GuardedWristJoint feeds it and acts on it.
    """
    DEFAULT_SENSITIVITY = 'sensitivity_default' # Matches the robot_params_SE4.py key.

    def __init__(self, params=None):
        p = params or {}
        self.enabled = bool(p.get('enable_guarded_mode', 0))
        self.drive_deadband = p.get('drive_deadband_rad', 0.01)
        self.release_after_s = p.get('release_after_s', 0.2)
        self.sensitivities = {k: v for k, v in p.items() if k.startswith('sensitivity_') and isinstance(v, dict)}
        self.sensitivity = None
        self.contact_mA_pos = None
        self.contact_mA_neg = None
        self.contact_s = None
        if self.enabled and not self.set_sensitivity(self.DEFAULT_SENSITIVITY):
            raise ValueError(f"guarded_contact params need a '{self.DEFAULT_SENSITIVITY}' entry")
        self.n_events = 0
        self.goal = None  # Last position goal (rad), or None in velocity mode
        self.vel_cmd = 0.0  # Last velocity command (rad/s); 0 in position mode
        self.reset()

    def reset(self):
        """Clear any guarded event and the contact timer. Keeps the command, sensitivity and event count."""
        self._clear_event()
        self._over_since = None

    def _clear_event(self):
        self.in_event = False
        self.trip_direction = 0
        self._last_cmd_t = None  # When a command last arrived during the event

    def set_sensitivity(self, name):
        """Select a sensitivity profile by name. Returns False, leaving the current one, if it is unknown."""
        s = self.sensitivities.get(name)
        if s is None:
            return False
        self.sensitivity = name
        self.contact_mA_pos = s['contact_mA_pos']
        self.contact_mA_neg = s['contact_mA_neg']
        self.contact_s = s['contact_s']
        return True

    # ---- commands ----------------------------------------------------------------------------------
    # Each command is checked with allow_*() before it is sent, and recorded with set_*() only once the
    # servo has actually taken it. A command the servo never got (runstop, collision stop) must not
    # count as driving, or a joint holding a load where it stands would trip. Recording a command does
    # not restart the contact timer: teleop streams a new goal every cycle while a button is held.
    # 'now' for commands and check_release() is one clock (the caller's); step() uses the sample's.

    def allow_goal(self, goal, pos, now):
        """Called before every position goal is sent. False means refuse it: it pushes into the obstacle."""
        return self._allow(self._sign(goal - pos), now)

    def allow_velocity(self, v, now):
        """Called before every nonzero velocity command is sent. False means refuse it."""
        return self._allow(1 if v > 0 else -1 if v < 0 else 0, now)

    def set_goal(self, goal):
        """The servo took a position goal."""
        self.goal = goal
        self.vel_cmd = 0.0

    def set_velocity(self, v):
        """The servo took a velocity command."""
        self.goal = None
        self.vel_cmd = v

    def check_release(self, now):
        """Called regularly. Clears an event once no command has arrived for release_after_s.
        Returns True when it does."""
        if not self.in_event:
            return False
        if self._last_cmd_t is None:
            self._last_cmd_t = now  # The release timer starts at the trip
            return False
        if now - self._last_cmd_t <= self.release_after_s:
            return False
        self._clear_event()
        return True

    def _allow(self, direction, now):
        if not self.in_event or self.check_release(now):
            return True
        if direction == 0 or direction == -self.trip_direction:
            self._clear_event()  # Stopped, or backing out of the obstacle
            return True
        self._last_cmd_t = now  # Still pushing into it: keep holding
        return False

    def note_hold(self):
        """The joint was told to stop where it is."""
        self.reset()
        self.goal = None
        self.vel_cmd = 0.0

    def _sign(self, error):
        if abs(error) <= self.drive_deadband:
            return 0
        return 1 if error > 0 else -1

    # ---- detection ---------------------------------------------------------------------------------

    def driving_direction(self, pos, at_limit=None):
        """Which way the joint is being driven: -1, 0 or +1. A joint resting against its soft limit is not."""
        if self.goal is not None:
            return self._sign(self.goal - pos)
        if self.vel_cmd == 0.0:
            return 0
        direction = 1 if self.vel_cmd > 0 else -1
        if at_limit and at_limit.get('pos' if direction > 0 else 'neg'):
            return 0  # Velocity mode brakes at the soft limits; sitting there is not a contact
        return direction

    def contact_mA(self, direction):
        """The contact current threshold for driving in direction (+1 or -1)."""
        return self.contact_mA_pos if direction > 0 else self.contact_mA_neg

    def step(self, pos, current_mA, now, at_limit=None):
        """Called with every new status sample. Returns True on the sample a guarded event starts."""
        if not self.enabled or self.in_event:
            return False
        direction = self.driving_direction(pos, at_limit)
        if direction == 0 or abs(current_mA) <= self.contact_mA(direction):
            self._over_since = None
            return False
        if self._over_since is None:
            self._over_since = now
        if now - self._over_since <= self.contact_s:
            return False
        self.in_event = True
        self.trip_direction = direction
        self.n_events += 1
        self._over_since = None
        self._last_cmd_t = None
        self.goal = None  # The caller holds the joint where it is, so nothing drives it now
        self.vel_cmd = 0.0
        return True

    def get_status(self):
        return {'enabled': self.enabled, 'sensitivity': self.sensitivity,
                'contact_mA_pos': self.contact_mA_pos, 'contact_mA_neg': self.contact_mA_neg,
                'contact_s': self.contact_s,
                'release_after_s': self.release_after_s, 'trip_direction': self.trip_direction}
