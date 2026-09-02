import termios
import time

import stretch4_body.core.hello_utils as hu
from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
from stretch4_body.core.feetech.feetech_SM_servo import FeetechCommError
from stretch4_body.utils.tool_metadata import ParallelGripperMetadata


class ParallelGripper(FeetechSMHello):
    """
    API to the Parallel Gripper
    The ParallelGripper motion is non-linear w.r.t to motor motion due to its design
    A position of zero is the fingertips  touching
    Units are in meters
    Contact guard (params 'contact_guard'): closing on an object holds a virtual goal just past the contact
    instead of stalling into the servo overcurrent protection. It runs in pull_status(), so in direct mode
    it only works if pull_status() is called in a loop while the gripper moves.
    """
    def __init__(self, chain=None, usb=None, name='parallel_gripper',is_direct=False):
        FeetechSMHello.__init__(self, name, chain, usb,is_direct=is_direct)
        self.status['pos_mm'] = 0.0
        self.tool_metadata = ParallelGripperMetadata()
        open_m = self.tool_metadata.actuator_to_aperture(hu.deg_to_rad(self.params['range_deg'][1]))
        self.poses = {
            'open': open_m,
            'mid': open_m / 2.0,
            'close': 0.0,
            'zero': 0.0}

    def startup(self):
        return FeetechSMHello.startup(self)

    def home(self, end_pos=hu.deg_to_rad(45.0),delay_at_stop=1.0):
        FeetechSMHello.home(self, end_pos=end_pos,delay_at_stop=delay_at_stop)

    def pretty_print(self):
        print('--- ParallelGripper ----')
        print("Position (mm): %f"%self.status['pos_mm'])
        FeetechSMHello.pretty_print(self)

    def pose(self,p,v_r=None, a_r=None):
        """
        p: Dictionary key to named pose (eg 'close')
        """
        self.move_to(self.poses[p],v_r,a_r)

    def move_to(self, x_m, v_r=None, a_r=None):
        """
        x_m: commanded absolute position (meters)
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        x_m = min(max(x_m, 0.0), self.params.get('range_mm', 80.0) / 1000.0)
        x_r = self.tool_metadata.aperture_to_actuator(x_m)
        FeetechSMHello.move_to(self, x_des=x_r, v_des=v_r, a_des=a_r)

    def move_by(self, x_m, v_r=None, a_r=None):
        """
        x_m: commanded incremental position (meters)
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        if self.is_direct:
            self.pull_status()
        x_final = (self.status.get('pos_mm', 0.0) / 1000.0) + x_m
        self.move_to(x_final, v_r, a_r)

    def set_velocity(self, v_r, a_r=None):
        """
        v_r: commanded velocity (rad/s)
        a_r: acceleration motion profile (rad/s^2)
        """
        if self.contact_guard.enabled and not self.status['is_homing']:
            hold = self.contact_guard.filter_velocity(v_r)
            if hold is not None:  # In contact: closing (or zero) velocity holds the virtual goal in position mode
                if self.in_vel_mode:
                    self._contact_guard_hold(hold)
                return
        FeetechSMHello.set_velocity(self, v_r, a_r)

    def move_to_mm(self, x_mm, v_r=None, a_r=None):
        self.move_to(x_mm / 1000.0, v_r, a_r)

    def move_by_mm(self, x_mm, v_r=None, a_r=None):
        self.move_by(x_mm / 1000.0, v_r, a_r)

    ############### Utilities ###############

    def pull_status(self,data=None):
        current_was_read = self.status_mux_id == 0  # The base class reads current only every 3rd cycle
        FeetechSMHello.pull_status(self,data)
        self.status['pos_mm']=self.tool_metadata.actuator_to_aperture(self.status['pos']) * 1000.0

    def step_sentry(self, robot):
        pass

    ############### Contact Guard ###############

    def _step_contact_guard(self):
        cg = self.contact_guard
        if self.status['is_homing'] or not self.status['torque_enabled'] or self.was_runstopped:
            cg.reset()
        else:
            goal = cg.step(self.status['pos'], self.status['current_mA'], time.time())
            if goal is not None:
                self.logger.info(f'{self.name}: contact at {parallel_gripper_servo_rad_to_mm(cg.contact_pos, self.params):.1f} mm '
                                 f'({self.status["current_mA"]:.0f} mA), holding virtual goal '
                                 f'{parallel_gripper_servo_rad_to_mm(goal, self.params):.1f} mm')
                self._contact_guard_hold(goal)
        s = cg.get_status()
        for k in ['contact_pos', 'virtual_goal', 'user_goal']:
            s[k + '_mm'] = None if s[k] is None else parallel_gripper_servo_rad_to_mm(s[k], self.params)
        self.status['contact_guard'] = s

    def _contact_guard_hold(self, goal):
        # A new goal does not cut short the servo's in-progress motion profile, so stop it first
        if self.in_vel_mode:
            self.enable_pos()
        else:
            self.quick_stop()
        FeetechSMHello.move_to(self, x_des=goal)  # Bypasses the filter in self.move_to
