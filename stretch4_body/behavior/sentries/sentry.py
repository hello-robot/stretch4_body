from stretch4_body.core.device import Device
import stretch4_body.core.hello_utils as hu
import time
import importlib
from colorama import Fore, Style

# #########################################################3
class Sentry(Device):
    """ 
    Base class for all sentries 
    A Sentry executes at full rate (100hz) just after pull_status()
    A Sentry monitors subsystems and computes limits for safety based on these recent sensor readings
    It then updates the limits on the subsystems such that subsequent commands to the subsystems
    will be safe
    """
    def __init__(self,name,robot,req_params=False):
        Device.__init__(self,name,req_params)
        self.robot=robot


    def startup(self):
        req = self.params.get('required_subsystems', [])
        for s in req:
            if s not in self.robot.subsystems:
                self.is_valid = False
                self.logger.warning(f"Sentry {self.name} disabled. Missing subsystem {s}")
                return False
        return Device.startup(self)

