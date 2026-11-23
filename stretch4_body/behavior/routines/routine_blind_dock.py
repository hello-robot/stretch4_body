
from stretch4_body.behavior.routines.routine import *
from stretch4_body.core.hello_utils import rad_to_deg
import time

# ###############################################################3
from stretch4_body.core.stepper import Stepper
class RoutineBlindDock(Routine):
    def __init__(self,robot):
        Routine.__init__(self,name="routine_blind_dock",robot=robot)
        self.vel_gains_prior = [{}, {}, {}]

    def set_vel_gains(self):
        new_gains={'vKi_limit':200,'vKi_d':.005,'vKd_d':0.1,'vKp_d':1.7}
        for ii in range(3):
            for g in new_gains:
                self.vel_gains_prior[ii][g]=self.robot.omnibase.wheels[ii].gains[g]
                self.robot.omnibase.wheels[ii].gains[g] = new_gains[g]
            self.robot.omnibase.wheels[ii].set_gains(self.robot.omnibase.wheels[ii].gains)
        self.update_controller()

    def restore_vel_gains(self):
        for ii in range(3):
            for g in self.vel_gains_prior[0]:
                self.robot.omnibase.wheels[ii].gains[g] = self.vel_gains_prior[ii][g]
            self.robot.omnibase.wheels[ii].set_gains(self.robot.omnibase.wheels[ii].gains)
        self.update_controller()

    def is_charging(self):
        ts = time.time()
        t_start_charge=None
        while self.update_controller() and time.time() - ts < self.params['t_settle']:
            self.logger.info('Charge!',self.robot.power_periph.status['adapter_voltage_present'])#charger_is_charging'])
            if self.robot.power_periph.status['adapter_voltage_present']:
                if t_start_charge is None:
                    t_start_charge=time.time()
                if time.time()-t_start_charge>self.params['t_settle']*0.5:#Must be on constantly for 50% of t_settle
                    return True
            else:
                t_start_charge=None
        return False

    def is_tilted(self):
        tilt_thresh = 4.0
        tilted = abs(rad_to_deg(self.robot.power_periph.imu.status['gravity_tilt'])) > tilt_thresh
        if tilted:
            self.logger.warning('Tilted ', rad_to_deg(self.robot.power_periph.imu.status['gravity_tilt']))
        return tilted

    def wait_on_charging(self,timeout,t_connected):
        ts = time.time()
        t_start_charge=None
        in_charge=False
        while self.update_controller() and time.time() - ts < timeout and not self.is_tilted():
            if self.robot.power_periph.status['adapter_voltage_present']:
                self.logger.info('Connected!')
                in_charge=True
                if t_start_charge is None:
                    t_start_charge=time.time()
                if time.time()-t_start_charge>t_connected:#Must be on constantly for 50% of t_settle
                    self.logger.info('Dock')
                    return True
            else:
                t_start_charge=None #Reset as lost contact perhaps
                if in_charge:
                    self.logger.warning('Lost charge contact')
                in_charge=False
        return False


    def do_undock(self):
        # Drive forward
        self.logger.info('Undocking')
        x_incr=2.0 #-2.0
        self.robot
        self.robot.omnibase.wheels[0].set_command(mode=Stepper.MODE_POS_TRAJ_INCR, x_des=x_incr, v_des=10, a_des=5)
        self.robot.omnibase.wheels[1].set_command(mode=Stepper.MODE_POS_TRAJ_INCR, x_des=-x_incr,v_des=10, a_des=5)
        self.robot.omnibase.wheels[2].enable_freewheel()
        self.wait_duration(2.0)

    def do_dock(self):

        if self.robot.power_periph.status['adapter_voltage_present']:
            return True
        # Drive forward
        vv = -2.0
        duration=7.0
        self.robot.omnibase.wheels[0].set_command(mode=Stepper.MODE_VEL_TRAJ, v_des=vv,stiffness=0.3,)
        self.robot.omnibase.wheels[1].set_command(mode=Stepper.MODE_VEL_TRAJ, v_des=-vv, stiffness=0.3, )
        self.robot.omnibase.wheels[2].enable_freewheel()
        #wait_duration(self.robot, 4.0)
        #self.wait_on_charging(timeout=4.0,t_connected=0.5)
        ts=time.time()
        docked=False
        while time.time()-ts<duration and not docked and not self.is_tilted():
            if self.is_canceled:
                self.logger.warning(f"Routine {self.name} canceled during `do_dock`.")
                break
            self.robot.omnibase.wheels[0].set_command(mode=Stepper.MODE_VEL_TRAJ, v_des=vv,stiffness=0.3,)
            self.robot.omnibase.wheels[1].set_command(mode=Stepper.MODE_VEL_TRAJ, v_des=-vv, stiffness=0.3, )
            self.robot.omnibase.wheels[2].enable_freewheel()
            docked=self.wait_on_charging(0.9, 0.1)
            
        if docked:
            self.logger.info('Docked. Settling position now.')
            #wait_duration(self.robot, 1.5)
            self.robot.omnibase.wheels[0].set_command(mode=Stepper.MODE_VEL_TRAJ, v_des=0,stiffness=1.0)
            self.robot.omnibase.wheels[1].set_command(mode=Stepper.MODE_VEL_TRAJ, v_des=0, stiffness=1.0 )
            self.robot.omnibase.wheels[2].enable_freewheel()
            self.wait_duration(0.1)
            self.logger.info('Docked. Locking position now.')
            self.robot.omnibase.wheels[0].set_command(mode=Stepper.MODE_POS_TRAJ_INCR, x_des=0, v_des=10, a_des=5,stiffness=1.0)
            self.robot.omnibase.wheels[1].set_command(mode=Stepper.MODE_POS_TRAJ_INCR, x_des=0, v_des=10, a_des=5,stiffness=1.0)
            self.robot.omnibase.wheels[2].set_command(mode=Stepper.MODE_POS_TRAJ_INCR, x_des=0, v_des=10, a_des=5,stiffness=1.0)
            #self.robot.omnibase.wheels[2].enable_freewheel()
            self.wait_duration(0.25)
        #print('Done docking')
        return docked

    def run(self,cmd_id,*args, **kwargs):
        """
        """
        if  self.robot.get_subsystem('omnibase') is  None:
            self.logger.warning('Not able to run routine %s. Hardware not present' % self.name.capitalize())
            return False
        
        super().run(cmd_id, *args, **kwargs)
        
        
        #Configure settings
        self.set_vel_gains()
        self.robot.safe_motion_manager.pause()
        self.robot.sentry_manager.pause()

        was_guarded_mode=self.robot.omnibase.guarded_mode_active
        self.robot.omnibase.disable_guarded_mode()

        success=False
        for i in range(self.params['num_retries']):
            self.logger.info(f'Starting dock attempt {i} of {self.params["num_retries"]}')
            if not self.do_dock():
                #if not self.wait_on_charging(timeout=1.0,t_connected=0.5): #self.is_charging():
                self.logger.error('Unsuccessful dock. Trying again')
                self.do_undock()
            else:
                success=True
                break

        #Restore settings
        self.restore_vel_gains()
        self.robot.safe_motion_manager.unpause()
        self.robot.sentry_manager.unpause()
        if was_guarded_mode:
            self.robot.omnibase.enable_guarded_mode()

        self.update_controller()

        if success:
            self.logger.info('Successful dock. Robot is charging')
            #self.robot.power_periph.trigger_beep()
            return True
        else:
            self.logger.error('Unable to dock! Robot is not charging')
            return False