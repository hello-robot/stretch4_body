import threading
from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
import stretch4_body.core.hello_utils as hu
import math

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
        self.poses = {
            'open': hu.deg_to_rad(self.params['range_deg'][1]),
            'mid': hu.deg_to_rad(self.params['range_deg'][1]) / 2,
            'close': 0,
            'zero': 0}

    def startup(self):
        return FeetechSMHello.startup(self)

    def home(self, cancel_homing_event: threading.Event, end_pos=hu.deg_to_rad(45.0),delay_at_stop=1.0):
        FeetechSMHello.home(self,cancel_homing_event=cancel_homing_event,end_pos=end_pos,delay_at_stop=delay_at_stop)

    def pretty_print(self):
        print('--- ParallelGripper ----')
        print("Position (mm): %f"%self.status['pos_mm'])
        FeetechSMHello.pretty_print(self)

    def pose(self,p,v_r=None, a_r=None):
        """
        p: Dictionary key to named pose (eg 'close')
        """
        self.move_to(self.poses[p],v_r,a_r)

    def move_to(self,x_r, v_r=None, a_r=None):
        """
        x_r: commanded absolute position of (servo frame)(rad).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """

        # x_r=self.mm_to_world_rad(x_m)
        # v_r=self.mm_to_world_rad(v_m)
        # a_des = a_r
        FeetechSMHello.move_to(self,x_des=x_r, v_des=v_r, a_des=a_r)


    def move_by(self,x_r, v_r=None, a_r=None):
        """
        x_r: commanded incremental position of (servo frame)(rad).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        self.move_to(self.status['pos']+x_r,v_r,a_r)

    def move_to_mm(self, x_mm, v_r=None, a_r=None):
        """
        x_mm: commanded absolute position (mm)
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        x_mm = min(max(x_mm, 0.0), self.params.get('range_mm', 80.0))
        x_r = self.mm_to_servo_rad(x_mm)
        FeetechSMHello.move_to(self, x_des=x_r, v_des=v_r, a_des=a_r)

    def move_by_mm(self, x_mm, v_r=None, a_r=None):
        """
        x_mm: commanded incremental position (mm)
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        if self.is_direct:
            self.pull_status()
        self.move_to_mm(self.status.get('pos_mm', 0.0) + x_mm, v_r, a_r)

    ############### Utilities ###############

    def pull_status(self,data=None):
        FeetechSMHello.pull_status(self,data)
        self.status['pos_mm']=self.servo_rad_to_mm(self.status['pos'])
        #self.status['pos']=self.world_rad_to_pct(self.status['pos'])
        #self.status['gripper_conversion']=self.gripper_conversion.get_status(self.status)

    def step_sentry(self, robot):
        pass

    def mm_to_servo_rad(self,pct):
        return hu.deg_to_rad(self.params['range_deg'][0])*pct/-100


    def servo_rad_to_mm(self,qr):
        # Constants from image
        L = self.params['kL']
        r = self.params['kR']
        finger_offset = self.params['kX0']# Derived thickness offset
        # Transformation: In this image, 0 deg is what used to be 44 deg
        q_eff = -1*qr +math.radians(self.params['kT0'])
        
        # Calculate horizontal pivot position relative to motor center
        # x_pivot = r*sin(q) - sqrt(L^2 - (r*cos(q))^2)
        term = L**2 - (r * math.cos(q_eff))**2
        x_pivot = r * math.sin(q_eff) - math.sqrt(term)
        
        # Convert pivot position to gap width
        # Gap x = 2 * (-x_pivot - offset)
        x_mm = 2 * (-x_pivot - finger_offset)
        
        #print('Convert', x_mm, self.mm_to_servo_rad(x_mm),qr)
        return round(x_mm, 3)

    def mm_to_servo_rad(self, x_mm):
        # Constants from your parameters
        L = self.params['kL']
        r = self.params['kR']
        finger_offset = self.params['kX0']
        kT0_rad = math.radians(self.params['kT0'])
        
        # 1. Solve for the horizontal pivot position (A)
        # x = 2 * (-x_pivot - finger_offset) 
        # => x/2 + finger_offset = -x_pivot
        A = -(x_mm / 2.0 + finger_offset)
        
        # 2. Solve the linkage geometry for sin(q_eff)
        # Squaring the forward equation r*sin(q) - sqrt(L^2 - (r*cos(q))^2) = A
        # yields: sin(q_eff) = (A^2 + r^2 - L^2) / (2 * A * r)
        numerator = A**2 + r**2 - L**2
        denominator = 2 * A * r
        
        sin_q_eff = numerator / denominator
        
        # 3. Handle floating point edge cases for the asin domain [-1, 1]
        sin_q_eff = max(-1.0, min(1.0, sin_q_eff))
        q_eff = math.asin(sin_q_eff)
        
        # 4. Reverse your specific motor transformation:
        # q_eff = -1*qr + kT0_rad  =>  qr = kT0_rad - q_eff
        qr = kT0_rad - q_eff
        
        return qr

