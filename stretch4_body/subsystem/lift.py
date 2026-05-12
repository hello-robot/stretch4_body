from stretch4_body.core.prismatic_joint import PrismaticJoint

import math
import logging

class Lift(PrismaticJoint):
    """
    API to the Stretch Lift
    """
    def __init__(self,usb=None,name='lift',motor_name=None):
        PrismaticJoint.__init__(self, name=name,usb=usb,motor_name=motor_name)
    # ######### Utilties ##############################

    def motor_rad_to_translate_m(self,ang): #input in rad
        d=self.params['pinion_t']*self.params['belt_pitch_m']/math.pi
        lift_m = (math.degrees(ang)/180.0)*math.pi*(d/2)
        return lift_m

    def translate_m_to_motor_rad(self, x):
        d = self.params['pinion_t'] * self.params['belt_pitch_m'] / math.pi
        ang = 180*x/((d/2)*math.pi)
        return math.radians(ang)

    def set_i_feedforward_payload(self,i):
        """
        Add additional payload feedforward current term for the tool.
        This adds to the standard term (i_feedforward) to gravity counterbalance the arm +wrist + tool

        For this to be used after a power cycle it is also necessary to call lift.motor.write_gains_to_flash after calling this.
        """
        if i < 0 or i > 1.0: #Amps, limit for safety
            self.logger.error(f'Invalid value for Lift set_i_feedforward_payload: {i}')
        else:
            self.logger.info(f'Setting payload feedforward current to: {i}')
            self.i_feedforward_payload=i
            self.motor.gains['i_safety_feedforward'] = self.motor.gains['i_safety_feedforward']+i
            self.motor.set_gains()