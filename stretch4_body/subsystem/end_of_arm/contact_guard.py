class ContactGuard:
    """
    Virtual goal for the ParallelGripper (closing = decreasing joint angle).

    A gripper commanded to close past an object (eg 'close' = 0 mm) stalls with a large position error,
    drives the servo to full current, trips the servo overcurrent protection and springs open.
    Instead: when the gripper is commanded to close and the current stays above contact_mA for contact_s,
    the contact position is latched and the servo goal is replaced by a virtual goal squeeze_rad past
    contact. While in contact, further closing commands (move_to / move_by / pose / set_velocity) are
    clamped to the virtual goal. Commanding the gripper open past the contact position clears the contact.

    Opening is guarded the same way when open_contact_mA is set. A gripper commanded open against an
    obstruction (or an open hardstop short of the goal) stalls below the overcurrent trip and slowly
    overheats. When the current stays above open_contact_mA for open_contact_s, the goal is replaced by a
    virtual goal open_backoff_rad short of the contact so the servo stops pushing. Further opening commands
    are clamped to it, and commanding the gripper closed past the contact position clears the contact.

    Detection runs in ParallelGripper.pull_status(). Under stretch_body_server that is called by the
    end-of-arm loop (~50 Hz). In direct mode (is_direct=True, eg stretch_gripper_jog -d) the guard only
    works if the caller keeps calling pull_status() while the gripper moves.

    Positions are joint radians. Directions are -1 (closing) and 1 (opening).
    """
    MOTION_MARGIN_RAD = 0.02  # Goal must be this far past pos to count as closing / opening
    RELEASE_MARGIN_RAD = 0.02  # Goal this far back past contact clears the contact

    def __init__(self, params=None):
        p = params or {}
        self.enabled = bool(p.get('enabled', 0))
        self.contact_mA = p.get('contact_mA', 200.0)
        self.contact_s = p.get('contact_s', 0.06)
        self.squeeze_rad = p.get('squeeze_rad', 0.1)
        self.open_contact_mA = p.get('open_contact_mA')  # None disables the guard while opening
        self.open_contact_s = p.get('open_contact_s', 0.5)
        self.open_backoff_rad = p.get('open_backoff_rad', 0.02)
        self.n_contacts = 0
        self._prev_mA = 0.0  # Current on the previous step. Kept across reset() as it is a measurement
        self.reset()

    def reset(self):
        self.user_goal = None  # Last goal requested via move_to (rad)
        self.vel_dir = 0  # Direction of the last set_velocity, 0 if stopped or in position mode
        self.in_contact = False
        self.contact_dir = 0  # Direction of motion that made the contact
        self.contact_pos = None
        self.virtual_goal = None
        self._t_hi = None
        self._pos_hi = None
        self._dir = 0  # Direction of motion on the previous step
        self._armed = False  # Detection armed for the current move
        self._pos_move_start = None

    def filter_goal(self, x):
        """Called with every requested position goal. Returns the goal to send to the servo."""
        if self.in_contact:
            d = self.contact_dir
            if d * (x - self.virtual_goal) > 0:  # Further toward the contact
                self.user_goal = x
                return self.virtual_goal
            release_from = self.contact_pos if d < 0 else self.virtual_goal  # Opening holds short of the contact
            if d * (release_from - x) > self.RELEASE_MARGIN_RAD:
                self.reset()  # Moved back away from the contact
        self.user_goal = x
        self.vel_dir = 0
        return x

    def filter_velocity(self, v):
        """Called with every requested velocity. Returns a goal to hold in position mode, or None to pass v through."""
        if self.in_contact:
            if self.contact_dir * v >= 0:  # Toward the contact (or zero)
                return self.virtual_goal
            self.reset()  # Moving away from the contact
        self.user_goal = None
        self.vel_dir = (v > 0) - (v < 0)
        return None

    def step(self, pos, current_mA, now):
        """Called with every new status sample. Returns a new virtual goal to command when contact is detected, else None."""
        prev_mA, self._prev_mA = self._prev_mA, abs(current_mA)
        if self.in_contact:
            return None
        d = self._motion_dir(pos)
        if d == 0:
            self._dir = 0
            self._t_hi = None
            return None
        thresh_mA, contact_s = (self.contact_mA, self.contact_s) if d < 0 else (self.open_contact_mA, self.open_contact_s)
        hi = abs(current_mA) >= thresh_mA
        if d != self._dir:  # A move just started
            # Current is unsigned, so if it was already high before the move it is from pushing the other way
            # (eg against the open hardstop, or squeezing an object). Then arm only once the current drops or
            # the gripper starts moving.
            self._armed = prev_mA < thresh_mA
            self._pos_move_start = pos
        self._dir = d
        if not self._armed and (not hi or d * (pos - self._pos_move_start) > self.MOTION_MARGIN_RAD):
            self._armed = True
        if not self._armed or not hi:
            self._t_hi = None
            return None
        if self._t_hi is None or (d > 0 and pos - self._pos_hi > self.MOTION_MARGIN_RAD):
            # Mark where the current first rose: that is the contact position. While opening, high current
            # only counts once the gripper stops advancing, so free motion can't trip the lower threshold.
            self._t_hi = now
            self._pos_hi = pos
        if now - self._t_hi < contact_s:
            return None
        self.in_contact = True
        self.contact_dir = d
        self.n_contacts += 1
        self.contact_pos = self._pos_hi
        if d < 0:
            self.virtual_goal = self.contact_pos - self.squeeze_rad
            if self.user_goal is not None:
                self.virtual_goal = max(self.virtual_goal, self.user_goal)  # Don't squeeze further than was asked for
        else:
            self.virtual_goal = self.contact_pos - self.open_backoff_rad
        return self.virtual_goal

    def _motion_dir(self, pos):
        if self.vel_dir:
            d = self.vel_dir
        elif self.user_goal is None:
            d = 0
        elif self.user_goal < pos - self.MOTION_MARGIN_RAD:
            d = -1
        elif self.user_goal > pos + self.MOTION_MARGIN_RAD:
            d = 1
        else:
            d = 0
        if d > 0 and self.open_contact_mA is None:
            return 0
        return d

    def get_status(self):
        return {'enabled': self.enabled, 'in_contact': self.in_contact, 'contact_dir': self.contact_dir,
                'contact_pos': self.contact_pos, 'virtual_goal': self.virtual_goal, 'user_goal': self.user_goal,
                'n_contacts': self.n_contacts}
