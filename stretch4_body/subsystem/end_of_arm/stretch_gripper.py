import threading
from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.end_of_arm.gripper_conversion import GripperConversion


class StretchGripper(FeetechSMHello):
    """
    API to the Stretch Gripper
    The StretchGripper motion is non-linear w.r.t to motor motion due to its design
    As such, the position of the gripper is represented at as unit-less value, 'pct'
    The Pct ranges from approximately -100 (fully closed) to approximately +50 to +200 (fully open)
    The fully open value (self.pct_max_open) is dependent on mechanical design of the gripper
    which changes depending on the robot generation (RE1, RE2, SE3, etc)
    A Pct of zero is the fingertips just touching
    """
    def __init__(self, chain=None, usb=None, name='stretch_gripper',is_direct=False):
        FeetechSMHello.__init__(self, name, chain, usb,is_direct=is_direct)
        self.status['pos_pct']= 0.0
        self.pct_max_open=100*abs(self.params['range_deg'][1]/self.params['range_deg'][0])

        self.poses = {'zero': 0,
                      'open': self.pct_max_open,
                      'close': -100}

        self.gripper_conversion = GripperConversion(self.params)
        self.status['gripper_conversion'] = self.gripper_conversion.get_status(self.status)

    def startup(self):
        return FeetechSMHello.startup(self)


    def home(self, cancel_homing_event: threading.Event, end_pos=0,delay_at_stop=2.0):
        return FeetechSMHello.home(self,cancel_homing_event=cancel_homing_event,end_pos=end_pos,delay_at_stop=delay_at_stop)

    def pretty_print(self):
        print('--- StretchGripper ----')
        print("Position (%)",self.status['pos_pct'])
        FeetechSMHello.pretty_print(self)

    def pose(self,p,v_r=None, a_r=None):
        """
        p: Dictionary key to named pose (eg 'close')
        """
        self.move_to(self.poses[p],v_r,a_r)

    def move_to(self,pct, v_r=None, a_r=None):
        """
        pct: commanded absolute position (Pct).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        x_r=self.pct_to_world_rad(pct)
        FeetechSMHello.move_to(self,x_des=x_r, v_des=v_r, a_des=a_r)


    def move_by(self,delta_pct,v_r=None,a_r=None):
        """
        delta_pct: commanded incremental motion (pct).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        if self.is_direct:
            self.pull_status() #Ensure up to date as server not doing pull_status
        self.move_to(self.status['pos_pct']+delta_pct,v_r,a_r)

    ############### Utilities ###############

    def pull_status(self,data=None):
        FeetechSMHello.pull_status(self,data)
        self.status['pos_pct']=self.world_rad_to_pct(self.status['pos'])
        self.status['gripper_conversion']=self.gripper_conversion.get_status(self.status)

    def pct_to_world_rad(self,pct):
        return hu.deg_to_rad(self.params['range_deg'][0])*pct/-100

    def world_rad_to_pct(self,r):
        return -100*r/hu.deg_to_rad(self.params['range_deg'][0])

    # def step_sentry(self, robot):
    #     """This sentry attempts to prevent the gripper servo from overheating during a prolonged grasp
    #     When the servo is stalled and exerting an effort above a threshold it will command a 'back off'
    #     position (slightly opening the grasp). This reduces the PID steady state error and lowers the
    #     commanded current. The gripper's spring design allows it to retain its grasp despite the backoff.
    #     """
    #     FeetechSMHello.step_sentry(self, robot)
    #     # if self.hw_valid and self.robot_params['robot_sentry']['stretch_gripper_overload'] and not self.is_homing:
    #     #     if self.status['stall_overload']:
    #     #         if self.in_vel_mode:
    #     #             self.enable_pos()
    #     #         if self.status['effort'] < 0: #Only backoff in open direction
    #     #             self.logger.debug('Backoff at stall overload')
    #     #             self.move_by(self.params['stall_backoff'])

class StretchGripper4(StretchGripper):
    """
        Wrapper for version 4 (for DW4)
    """
    def __init__(self, chain=None, usb=None):
        StretchGripper.__init__(self, chain, usb,'stretch_gripper')
