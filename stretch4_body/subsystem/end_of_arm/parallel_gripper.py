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
        self.poses = self.tool_metadata.poses

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
        x_m: target absolute fingertip aperture (meters)
        v_r: motion-profile velocity limit, in actuator units (rad/s).
        a_r: motion-profile acceleration limit, in actuator units (rad/s^2).
        """
        low, high = self.tool_metadata.command_range
        x_m = min(max(x_m, low), high)
        x_r = self.tool_metadata.aperture_to_actuator(x_m)
        FeetechSMHello.move_to(self, x_des=x_r, v_des=v_r, a_des=a_r)

    def move_by(self, x_m, v_r=None, a_r=None):
        """
        x_m: target fingertip aperture position increment (meters)
        v_r: motion-profile velocity limit, in actuator units (rad/s).
        a_r: motion-profile acceleration limit, in actuator units (rad/s^2).
        """
        if self.is_direct:
            self.pull_status()
        x_final = (self.status.get('pos_mm', 0.0) / 1000.0) + x_m
        self.move_to(x_final, v_r, a_r)


    def set_velocity(self, v_r, a_r=None):
        """
        v_r: target velocity, in actuator units (rad/s)
        a_r: target acceleration, in actuator units (rad/s^2).
        """
        return super().set_velocity(v_r, a_r)

    ############### Utilities ###############

    def pull_status(self,data=None):
        FeetechSMHello.pull_status(self,data)
        self.status['pos_mm']=self.tool_metadata.actuator_to_aperture(self.status['pos']) * 1000.0

    def step_sentry(self, robot):
        pass

