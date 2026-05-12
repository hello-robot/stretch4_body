from stretch4_body.core.device import Device
import stretch4_body.core.hello_utils as hu
import time
import importlib
from colorama import Fore, Style

# #########################################################3
class SafeMotion(Device):
    """ 
    Base class for all safe motions 
    A SafeMotion executes at full rate (100hz) just before push_command()
    A SafeMotion computes adjustments to command motions to ensure safety
    It then overrides the subsystem commands prior to being pushed to the actuators
    """
    def __init__(self,name,robot,req_params=False):
        Device.__init__(self,name,req_params)
        self.robot=robot

    def startup(self):
        req = self.params.get('required_subsystems', [])
        for s in req:
            if s not in self.robot.subsystems:
                self.is_valid = False
                self.logger.warning(f"SafeMotion {self.name} disabled. Missing subsystem {s}")
                return False
        return Device.startup(self)
