from stretch4_body.core.device import Device
import stretch4_body.core.hello_utils as hu
import time
import importlib


# #########################################################3

class SentryManager(Device):
    """
    Manages a set of plug-ins that monitors the state of the robot

    The step() is called at full rate (100hz) just after push_status()
    The plug-ins can  and updates limits, settings, etc.

    """
    def __init__(self,robot):
        Device.__init__(self,"sentry_manager")
        self.robot=robot
        self.sentries = {}
        self.status={'active':{}}
        for k in self.params['controllers']:
            s=getattr(importlib.import_module(self.robot_params[k]['py_module_name']),self.robot_params[k]['py_class_name'])(robot)
            if s.params.get('enabled',1):
                self.sentries[k]=s
                self.status[k]=self.sentries[k].status
                self.status['active'][k]=False


    def startup(self):
        success=True
        for k in self.sentries:
            if hasattr(self.sentries[k],'startup'):
                self.logger.info('Starting Sentry %s...' % k)
                success=success & self.sentries[k].startup()
                self.status['active'][k]=success
                if not success:
                    self.logger.error('Failed to start Sentry %s' % k)
        return success

    def stop(self):
        success = True
        for k in self.sentries:
            if hasattr(self.sentries[k],'stop'):
                success=success & self.sentries[k].stop()
        return success


    def pause(self,to_pause=None):
        keys = self.sentries.keys() if to_pause is None else [k for k in to_pause if k in self.sentries]
        for k in keys:
            self.logger.info(f'Paused {k}')
            self.status['active'][k]=False
            if hasattr(self.sentries[k], 'pause'):
                self.sentries[k].pause()


    def unpause(self,to_unpause=None):
        keys = self.sentries.keys() if to_unpause is None else [k for k in to_unpause if k in self.sentries]
        for k in keys:
            self.logger.info(f'Unpaused {k}')
            self.status['active'][k]=self.sentries[k].is_valid
            if hasattr(self.sentries[k], 'unpause'):
                self.sentries[k].unpause()

    def step(self):
        #Reset limits on the max vel and accel each control step
        #Sentries will then limit them to most conservative value for each control cycle
        if 'omnibase' in self.robot.subsystems:
            self.robot.omnibase.set_curr_max_vel_xy_m()
            self.robot.omnibase.set_curr_max_accel_xy_m()
            self.robot.omnibase.set_curr_max_vel_w_r()
            self.robot.omnibase.set_curr_max_accel_w_r()
        
        #Each subsystem may have a local sentry
        for sn in self.robot.subsystems:
            if hasattr(self.robot.subsystems[sn],"step_sentry"):
                self.robot.subsystems[sn].step_sentry(self.robot.status)
        
        #Then step the system sentries
        for sm in self.sentries:
            if self.status['active'][sm]:
                self.sentries[sm].step()
            self.status[sm]=self.sentries[sm].status


