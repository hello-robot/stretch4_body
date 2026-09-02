from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
import stretch4_body.core.hello_utils as hu
from stretch4_body.utils.tool_metadata import ParallelGripperMetadata

class ParallelGripper(FeetechSMHello):
    """
    API to the Parallel Gripper
    The ParallelGripper motion is non-linear w.r.t to motor motion due to its design
    A position of zero is the fingertips  touching
    Units are in meters
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

    def move_to_mm(self, x_mm, v_r=None, a_r=None):
        self.move_to(x_mm / 1000.0, v_r, a_r)

    def move_by_mm(self, x_mm, v_r=None, a_r=None):
        self.move_by(x_mm / 1000.0, v_r, a_r)

    ############### Utilities ###############

    def pull_status(self,data=None):
        FeetechSMHello.pull_status(self,data)
        self.status['pos_mm']=self.tool_metadata.actuator_to_aperture(self.status['pos']) * 1000.0

    def step_sentry(self, robot):
        pass

