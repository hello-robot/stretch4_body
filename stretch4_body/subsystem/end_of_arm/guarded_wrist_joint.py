import termios
import time

from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
from stretch4_body.core.feetech.feetech_SM_servo import FeetechCommError
import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.end_of_arm.servo_guarded_contact import ServoGuardedContact


class GuardedWristJoint(FeetechSMHello):
    """
    A wrist joint (yaw, pitch, roll) with software guarded contact. See ServoGuardedContact.

    Configured by the joint's 'guarded_contact' params; without them, or with enable_guarded_mode 0, this
    behaves exactly like FeetechSMHello. The sensitivity is chosen with set_guarded_contact_sensitivity(),
    normally from the robot-level guarded contact profile via EndOfArm.

    Detection runs in pull_status(). Under stretch_body_server that is called by the end-of-arm loop, so it
    is always running. In direct mode (is_direct=True) the guard only works if the caller keeps calling
    pull_status() while the joint moves.

    Status:
        in_guarded_event: True from a guarded contact until it is released (see ServoGuardedContact)
        guarded_event: count of guarded contacts since startup (as the steppers report it)
        guarded_contact: the active sensitivity and its thresholds, and which way the joint was driven
                         when it tripped (trip_direction, 0 when not in an event)
    """

    def __init__(self, name, chain=None, usb=None, is_direct=False):
        FeetechSMHello.__init__(self, name, chain, usb, is_direct=is_direct)
        self.guarded_contact = ServoGuardedContact(self.params.get('guarded_contact'))
        self._clock = time.time  # For the release timer: when commands arrive
        self._update_guarded_status()

    # ---- commands ----------------------------------------------------------------------------------

    def _guard_active(self):
        return self.guarded_contact.enabled and not self.status['is_homing']

    def move_to(self, x_des, v_des=None, a_des=None):
        if (not self._guard_active() or self.check_nan_value(x_des) or (v_des is not None and v_des == 0)):
            # Zero velocity is a hold, which hold_position() handles
            return FeetechSMHello.move_to(self, x_des, v_des, a_des)
        goal = self._clip_to_soft_limits(x_des)
        if not self.guarded_contact.allow_goal(goal, self.status['pos'], self._clock()):
            self._warn_refused('move_to')
            return
        self._update_guarded_status()  # Backing off, or a release, may have just cleared an event
        if self._was_sent(FeetechSMHello.move_to, x_des, v_des, a_des):
            self.guarded_contact.set_goal(goal)

    def set_velocity(self, v_des, a_des=None):
        if (not self._guard_active() or self.check_nan_value(v_des)
                or abs(v_des) <= self.params['motion']['vel_is_moving_thresh']):
            # A near-zero velocity is a hold, which hold_position() handles
            return FeetechSMHello.set_velocity(self, v_des, a_des)
        if not self.guarded_contact.allow_velocity(v_des, self._clock()):
            self._warn_refused('set_velocity')
            return
        self._update_guarded_status()  # Backing off, or a release, may have just cleared an event
        if self._was_sent(FeetechSMHello.set_velocity, v_des, a_des):
            self.guarded_contact.set_velocity(v_des)

    def hold_position(self):
        self.guarded_contact.note_hold()
        self._update_guarded_status()
        return FeetechSMHello.hold_position(self)

    def quick_stop(self):
        # A quick stop leaves the servo where it is, so the last goal no longer means it is being driven
        self.guarded_contact.note_hold()
        self._update_guarded_status()
        return FeetechSMHello.quick_stop(self)

    def set_guarded_contact_sensitivity(self, sensitivity_name=None):
        """
        sensitivity_name: a '*_strength' entry in this joint's guarded_contact params.
                          None means medium_strength.
        """
        g = self.guarded_contact
        if not g.enabled:
            return
        if not g.set_sensitivity(sensitivity_name or ServoGuardedContact.DEFAULT_SENSITIVITY):
            self.logger.error(f'{self.name}: unknown guarded contact sensitivity {sensitivity_name}, '
                              f'keeping {g.sensitivity}')
            return
        self._update_guarded_status()

    def _was_sent(self, command, *args):
        """Run a FeetechSMHello command and report whether it reached the servo.
        They set _is_moving only once the servo takes the command, and return early otherwise."""
        was_moving = self._is_moving
        self._is_moving = False
        command(self, *args)
        sent = self._is_moving
        if not sent:
            self._is_moving = was_moving
        return sent

    def _warn_refused(self, command):
        direction = 'positive' if self.guarded_contact.trip_direction > 0 else 'negative'
        self.logger.warning(f'{self.name} is in a guarded contact event. Not executing {command} in the '
                            f'{direction} direction. Stop, or move the other way, to clear it.',
                            extra={'throttle_s': 1.0})

    def _clip_to_soft_limits(self, x):
        lo, hi = self.get_soft_motion_limits()
        if lo is not None:
            x = max(lo, x)
        if hi is not None:
            x = min(hi, x)
        return x

    # ---- detection ---------------------------------------------------------------------------------

    def pull_status(self, data=None):
        current_was_read = self.status_mux_id == 0  # The base class reads current only every 3rd cycle
        FeetechSMHello.pull_status(self, data)
        if self.guarded_contact.enabled and self.hw_valid and data is None and not current_was_read:
            # Detection needs current every cycle, as ParallelGripper's contact guard does
            try:
                i_mA = self.motor.get_current_mA()
                if self.motor.last_comm_success:
                    self.status['current_mA'] = i_mA
                    self.status['effort'] = self.current_to_effort_pct(float(i_mA))
            except (termios.error, FeetechCommError, IndexError):
                self.comm_errors.add_error(rx=True, gsr=False)
        self._step_guarded_contact()

    def _step_guarded_contact(self):
        g = self.guarded_contact
        if not g.enabled or not self.hw_valid:
            return
        if (self.status['is_homing'] or not self.status['torque_enabled'] or self.was_runstopped
                or self.is_calibration_required()):
            g.note_hold()  # Nothing is driving the joint
        # Time the stall on the sample's own timestamp, so stale status after a comm error cannot trip it
        elif g.step(self.status['pos'], self.status['current_mA'], self.status['timestamp_pc'],
                    self.status['at_limit']):
            self.logger.warning(
                f'{self.name}: guarded contact. {self.status["current_mA"]:.0f} mA '
                f'(limit {g.contact_mA(g.trip_direction):.0f} mA, {g.sensitivity}) for over {g.contact_s:.2f} s, '
                f'holding at {hu.rad_to_deg(self.status["pos"]):.1f} deg')
            self._guarded_hold()
        g.check_release(self._clock())
        self._update_guarded_status()

    def _guarded_hold(self):
        """Stop the joint where it stalled and hold it there."""
        # As ParallelGripper._contact_guard_hold: a new goal does not cut short the servo's in-progress
        # motion profile, which is still running when the joint stalls, so stop it first
        if self.in_vel_mode:
            self.enable_pos()
        else:
            FeetechSMHello.quick_stop(self)  # Not self.quick_stop(), which would clear the event
        FeetechSMHello.move_to(self, x_des=self.status['pos'])  # Bypasses the filter in self.move_to

    def _update_guarded_status(self):
        g = self.guarded_contact
        self.status['in_guarded_event'] = g.in_event
        self.status['guarded_event'] = g.n_events
        self.status['guarded_contact'] = g.get_status()
