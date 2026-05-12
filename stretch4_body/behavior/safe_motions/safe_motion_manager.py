from stretch4_body.core.device import Device
import stretch4_body.core.hello_utils as hu
import time
import importlib


# #########################################################3

class SafeMotionManager(Device):
    """
    Manages a set of plug-ins that restrict motions of the motors to help avoid hazards
    The step() is called at full rate (100hz) just prior to the push_command()
    The plug-ins can overwrite control modes, setpoints, etc.

    The following actions can be taken
    1. Limit joint range
    2. Limit velocity and acceleration
    3. Trigger joint safe stop
    4. Trigger full system safe stop
    """
    def __init__(self,robot):
        Device.__init__(self,"safe_motion_manager")
        self.robot=robot
        self.controllers = {}
        self.status={'safe_motions_triggered':[],'active':{}}
        for k in self.params['controllers']:
            s=getattr(importlib.import_module(self.robot_params[k]['py_module_name']),self.robot_params[k]['py_class_name'])(robot)
            if s.params.get('enabled',1):
                self.controllers[k]=s
                self.status[k]=self.controllers[k].status
                self.status['active'][k]=False

    def startup(self):
        success = True
        for k in self.controllers:
            if hasattr(self.controllers[k], 'startup'):
                self.logger.info('Starting SafeMotion %s...' % k)
                success = success & self.controllers[k].startup()
                self.status['active'][k]=success
                if not success:
                    self.logger.error('Failed to start SafeMotion %s' % k)
        return success

    def stop(self):
        success = True
        for k in self.controllers:
            if hasattr(self.controllers[k], 'stop'):
                success = success & self.controllers[k].stop()
        return success

    def pause(self,to_pause=None):
        if to_pause == None:
            for k in self.controllers:
                self.logger.info(f'Paused {k}')
                self.status['active'][k]=False
        else:
            for k in to_pause:
                if k in self.controllers:
                    self.logger.info(f'Paused {k}')
                    self.status['active'][k]=False


    def unpause(self,to_unpause=None):
        if to_unpause == None:
            for k in self.controllers:
                self.logger.info(f'Unpaused {k}')
                self.status['active'][k]=self.controllers[k].is_valid
        else:
            for k in to_unpause:
                if k in self.controllers:
                    self.logger.info(f'Unpaused {k}')
                    self.status['active'][k]=self.controllers[k].is_valid


    def step(self):
        self.status['safe_motions_triggered'] = []

        for sm in self.controllers:
            if self.status['active'][sm]:
                if self.controllers[sm].step():
                    self.status['safe_motions_triggered'].append(sm)

    def enter_safe_stop(self):
        if not self.status['in_safe_stop']:
            #Init safe stop here
            self.status['in_safe_stop']=True

        if 'omnibase' in self.robot.subsystems:
            self.robot.subsystems['omnibase'].enable_freewheel_mode()

        if 'arm' in self.robot.subsystems:
            self.subsystems['arm'].enable_safety()

        if 'lift' in self.robot.subsystems:
            self.robot.subsystems['lift'].enable_safety()

        # if 'end_of_arm' in self.robot.subsystems:
        #     self.robot.subsystems['end_of_arm'].enable_safety()
