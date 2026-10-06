class ContactGuard:
    """
    Virtual goal for the ParallelGripper (closing = decreasing joint angle).

    A gripper commanded to close past an object (eg 'close' = 0 mm) stalls with a large position error,
    drives the servo to full current, trips the servo overcurrent protection and springs open.
    Instead: when the gripper is commanded to close and the current stays above contact_mA for contact_s,
    the contact position is latched and the servo goal is replaced by a virtual goal squeeze_rad past
    contact. While in contact, further closing commands (move_to / move_by / pose / set_velocity) are
    clamped to the virtual goal. Commanding the gripper open past the contact position clears the contact.

    Detection runs in ParallelGripper.pull_status(). Under stretch_body_server that is called by the
    end-of-arm loop (~50 Hz). In direct mode (is_direct=True, eg stretch_gripper_jog -d) the guard only
    works if the caller keeps calling pull_status() while the gripper moves.

    Positions are joint radians.
    """
    CLOSING_MARGIN_RAD = 0.02  # Goal must be this far past pos to count as closing
    RELEASE_MARGIN_RAD = 0.02  # Goal this far open past contact clears the contact

    def __init__(self, params=None):
        p = params or {}
        self.enabled = bool(p.get('enabled', 0))
        self.contact_mA = p.get('contact_mA', 200.0)
        self.contact_s = p.get('contact_s', 0.06)
        self.squeeze_rad = p.get('squeeze_rad', 0.1)
        self.n_contacts = 0
        self.reset()

    def reset(self):
        self.user_goal = None  # Last goal requested via move_to (rad)
        self.closing_vel = False  # Last set_velocity was closing
        self.in_contact = False
        self.contact_pos = None
        self.virtual_goal = None
        self._t_hi = None
        self._pos_hi = None

    def filter_goal(self, x):
        """Called with every requested position goal. Returns the goal to send to the servo."""
        self.user_goal = x
        self.closing_vel = False
        if not self.in_contact:
            return x
        if x < self.virtual_goal:
            return self.virtual_goal
        if x > self.contact_pos + self.RELEASE_MARGIN_RAD:
            self.reset()  # Opened past the object
        return x

    def filter_velocity(self, v):
        """Called with every requested velocity. Returns a goal to hold in position mode, or None to pass v through."""
        if not self.in_contact:
            self.user_goal = None
            self.closing_vel = v < 0
            return None
        if v <= 0:
            return self.virtual_goal
        self.reset()  # Opening
        return None

    def step(self, pos, current_mA, now):
        """Called with every new status sample. Returns a new virtual goal to command when contact is detected, else None."""
        if self.in_contact:
            return None
        closing = self.closing_vel or (self.user_goal is not None and self.user_goal < pos - self.CLOSING_MARGIN_RAD)
        if not closing or abs(current_mA) < self.contact_mA:
            self._t_hi = None
            return None
        if self._t_hi is None:  # Mark where the current first rose: that is the contact position
            self._t_hi = now
            self._pos_hi = pos
        if now - self._t_hi < self.contact_s:
            return None
        self.in_contact = True
        self.n_contacts += 1
        self.contact_pos = self._pos_hi
        self.virtual_goal = self.contact_pos - self.squeeze_rad
        if self.user_goal is not None:
            self.virtual_goal = max(self.virtual_goal, self.user_goal)  # Don't squeeze further than was asked for
        return self.virtual_goal

    def get_status(self):
        return {'enabled': self.enabled, 'in_contact': self.in_contact, 'contact_pos': self.contact_pos,
                'virtual_goal': self.virtual_goal, 'user_goal': self.user_goal, 'n_contacts': self.n_contacts}
