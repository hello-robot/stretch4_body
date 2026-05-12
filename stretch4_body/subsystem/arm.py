from stretch4_body.core.prismatic_joint import PrismaticJoint
import math


class Arm(PrismaticJoint):
    """
    API to the Stretch Arm
    """
    def __init__(self,usb=None,name='arm',motor_name=None):
        PrismaticJoint.__init__(self, name=name,usb=usb,motor_name=motor_name)

    # ######### Utilties ##############################

    def motor_rad_to_translate_m(self,ang): #input in rad, output m
        return (self.params['chain_pitch']*self.params['chain_sprocket_teeth']/self.params['gr_spur']/(math.pi*2))*ang

    def translate_m_to_motor_rad(self, x):
        return x/(self.params['chain_pitch']*self.params['chain_sprocket_teeth']/self.params['gr_spur']/(math.pi*2))


