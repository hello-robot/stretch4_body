import threading
from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
import stretch4_body.core.hello_utils as hu
import math
from stretch4_body.subsystem.end_of_arm.gripper_conversion import parallel_gripper_servo_rad_to_mm, parallel_gripper_mm_to_servo_rad

class ParallelGripper(FeetechSMHello):
    """
    API to the Parallel Gripper
    The ParallelGripper motion is non-linear w.r.t to motor motion due to its design
    A position of zero is the fingertips  touching
    Units are in mm
    """
    def __init__(self, chain=None, usb=None, name='parallel_gripper',is_direct=False):
        FeetechSMHello.__init__(self, name, chain, usb,is_direct=is_direct)
        self.status['pos_mm'] = 0.0
        open_mm = parallel_gripper_servo_rad_to_mm(hu.deg_to_rad(self.params['range_deg'][1]), self.params)
        self.poses = {
            'open': open_mm,
            'mid': open_mm / 2.0,
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

    def move_to(self, x_mm, v_r=None, a_r=None):
        """
        x_mm: commanded absolute position (mm)
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        x_mm = min(max(x_mm, 0.0), self.params.get('range_mm', 80.0))
        x_r = parallel_gripper_mm_to_servo_rad(x_mm, self.params)
        FeetechSMHello.move_to(self, x_des=x_r, v_des=v_r, a_des=a_r)

    def move_by(self, x_mm, v_r=None, a_r=None):
        """
        x_mm: commanded incremental position (mm)
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        if self.is_direct:
            self.pull_status()
        self.move_to(self.status.get('pos_mm', 0.0) + x_mm, v_r, a_r)

    def move_to_mm(self, x_mm, v_r=None, a_r=None):
        self.move_to(x_mm, v_r, a_r)

    def move_by_mm(self, x_mm, v_r=None, a_r=None):
        self.move_by(x_mm, v_r, a_r)

    ############### Utilities ###############

    def pull_status(self,data=None):
        FeetechSMHello.pull_status(self,data)
        self.status['pos_mm']=parallel_gripper_servo_rad_to_mm(self.status['pos'], self.params)

    def step_sentry(self, robot):
        pass

