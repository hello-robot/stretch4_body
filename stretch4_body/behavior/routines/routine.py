from stretch4_body.core.device import Device
import time

# #########################################################3
class Routine(Device):
    """ Base class for all routines """
    def __init__(self,name,robot,timeout=None, req_params=False):
        Device.__init__(self,name,req_params)
        self.robot=robot
        self.timeout=timeout

        self.is_canceled = False

    def startup(self):
        req = self.params.get('required_subsystems', [])
        for s in req:
            if s not in self.robot.subsystems:
                self.is_valid = False
                self.logger.warning(f"Routine {self.name} disabled. Missing subsystem {s}")
                return False
        return Device.startup(self)

    def check_runstop(self):
        if 'power_periph' in self.robot.status and self.robot.status['power_periph']['runstop_event']:
            self.logger.warning(f"Routine {self.name} canceled during `check_runstop`.")
            self.cancel()
            
    def update_controller(self):
        """
        Step the control loop (pull status, push command, etc)
        """
        if not self.is_valid:
            return False
        self.check_runstop()
        if self.is_canceled:
            self.logger.warning(f"Routine {self.name} canceled during `update_controller`.")
            return False
        do_continue= self.robot.cb_routine_update_controller()
        return do_continue and not self.is_timed_out()

    def get_run_time(self):
        return time.time()-self.ts_start
    
    def is_timed_out(self):
        self.timed_out= self.timeout is not None and self.get_run_time()>self.timeout
        return self.timed_out
    
    def run(self,cmd_id,*args, **kwargs) -> bool:
        if type(self).run == Routine.run:
            raise NotImplementedError("Please override this method.")
        
        self.ts_start = time.time()
        self.is_canceled = False

    def cancel(self):
        self.logger.warning(f"Routine {self.name} has been cancelled.")
        self.is_canceled = True

    def wait_until_contact(self, motor,timeout,t_ignore=.1):
        """
        Wait for contact on a motor, starting after t_ignore seconds
        Return True if got contact, False if timed out
        """
        ts=time.time()
        while time.time()-ts<timeout:
            self.check_runstop()
            if self.is_canceled:
                self.logger.warning(f"Routine {self.name} canceled during `wait_until_contact`.")
                return False
            if not self.robot.cb_routine_update_controller():
                self.logger.warning(f"Routine {self.name} stopped by controller during `wait_until_contact`.")
                return False
            if motor.status['in_guarded_event'] and time.time()-ts>t_ignore:
                return True
        self.logger.warning(f"Routine {self.name} timed out during `wait_until_contact`.")
        return False

    def wait_duration(self, t):
        """
        Pause routine for time t (s) while updating controller
        """
        ts=time.time()
        while time.time()-ts<t:
            self.check_runstop()
            if self.is_canceled:
                self.logger.warning(f"Routine {self.name} canceled during `wait_duration`.")
                return False
            if not self.robot.cb_routine_update_controller():
                self.logger.warning(f"Routine {self.name} stopped by controller during `wait_duration`.")
                return False
            pass
        return True

    def wait_until_at_setpoint(self, motor,timeout,t_ignore=.1):
        ts = time.time()
        while time.time() - ts < timeout:
            self.check_runstop()
            if self.is_canceled:
                self.logger.warning(f"Routine {self.name} canceled during `wait_until_at_setpoint`.")
                return False
            if not self.robot.cb_routine_update_controller():
                self.logger.warning(f"Routine {self.name} stopped by controller during `wait_until_at_setpoint`.")
                return False
            if motor.status['near_pos_setpoint'] and time.time()-ts > t_ignore:
                return True
        self.logger.warning(f"Routine {self.name} timed out during `wait_until_at_setpoint`.")
        return False

    def wait_on_cb(self, cb_waiting_on,timeout):
        #Poll cb_waiting_on (status most likely) , returns true or timesout
        ts = time.time()
        while time.time() - ts < timeout:
            self.check_runstop()
            if self.is_canceled:
                self.logger.warning(f"Routine {self.name} canceled during `wait_on_cb`.")
                return False
            if not self.robot.cb_routine_update_controller():
                self.logger.warning(f"Routine {self.name} stopped by controller during `wait_on_cb`.")
                return False
            if cb_waiting_on():
                return True
        self.logger.warning(f"Routine {self.name} timed out during `wait_on_cb`.")
        return False

class RoutineNOP(Routine):
    """
    Default routine - do nothing, just process Commands and post Status
    """
    def __init__(self,robot):
        Routine.__init__(self,name="routine_nop",robot=robot,req_params=False)

    def check_runstop(self):
        ...

    def run(self,cmd_id,*args, **kwargs):
        super().run(cmd_id, *args, **kwargs)
        #No-op, place would normally do something
        do_continue = self.update_controller()
        return do_continue
