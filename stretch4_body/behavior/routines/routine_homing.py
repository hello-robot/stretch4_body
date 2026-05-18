
from stretch4_body.core.prismatic_joint import PrismaticJoint
import stretch4_body.behavior.routines.routine as routine
import time


class RoutineHome(routine.Routine):
    """Base class for all homing routines.

    Pauses Sentries and SafeMotion behaviors before running, then restores
    them afterward. Exclusion lists are read from params:
        - 'exclude_sentry_pause': list of sentry names to leave running
        - 'exclude_safe_motion_pause': list of safe_motion names to leave running
    Subclasses implement _run_homing() with their actual homing logic.
    """
    def __init__(self, name, robot):
        routine.Routine.__init__(self, name=name, robot=robot)

    def _pause_behaviors(self):
        """Pause sentries and safe_motion controllers (subject to exclusion lists).
        Returns (sm_to_pause, sentry_to_pause) so they can be unpaused later."""
        exclude_sentry_pause = self.params.get('exclude_sentry_pause', [])
        exclude_safe_m_pause = self.params.get('exclude_safe_motion_pause', [])

        sm_to_pause = [k for k in self.robot.safe_motion_manager.controllers.keys() if k not in exclude_safe_m_pause]
        sentry_to_pause = [k for k in self.robot.sentry_manager.sentries.keys() if k not in exclude_sentry_pause]

        self.robot.safe_motion_manager.pause(sm_to_pause)
        self.robot.sentry_manager.pause(sentry_to_pause)
        return sm_to_pause, sentry_to_pause

    def _unpause_behaviors(self, sm_to_pause, sentry_to_pause):
        """Unpause previously paused sentries and safe_motion controllers."""
        self.robot.safe_motion_manager.unpause(sm_to_pause)
        self.robot.sentry_manager.unpause(sentry_to_pause)

    def run(self, cmd_id, *args, **kwargs):
        if hasattr(self.robot, 'power_periph') and self.robot.power_periph.status['runstop_event']:
            self.logger.warning('Not able to home %s. Robot is runstopped' % self.name.capitalize())
            return False

        super().run(cmd_id, *args, **kwargs)

        sm_paused, sentry_paused = self._pause_behaviors()
        try:
            success = self._run_homing(cmd_id, *args, **kwargs)
        finally:
            self._unpause_behaviors(sm_paused, sentry_paused)

        return success

    def _run_homing(self, cmd_id, *args, **kwargs):
        """Override in subclasses with the actual homing logic.
        Return True on success, False on failure."""
        raise NotImplementedError("Override _run_homing in subclass.")


class RoutineEndOfArmHome(RoutineHome):
    def __init__(self,robot):
        RoutineHome.__init__(self,name="end_of_arm",robot=robot)

    def cancel(self, *args, **kwargs):
        self.robot.eoa_loop.q_cmd.put(['end_of_arm', 'cancel_homing', f'cancel_homing_{self.name}', args, kwargs])
        super().cancel()

    def _run_homing(self,cmd_id,*args, **kwargs):
        if not hasattr(self.robot,'eoa_loop') or not self.robot.eoa_loop.is_valid:
            self.logger.warning('Not able to home %s. Hardware not present' % self.name.capitalize())
            return False
        success = True
        def is_not_homing():
            return not self.robot.eoa_loop.status['is_homing']

        cmd = ['end_of_arm', 'home', cmd_id,args,kwargs]
        
        self.robot.eoa_loop.q_cmd.put(cmd)
        self.wait_duration(1.5) #Let homing begin
        
        if not self.wait_on_cb(is_not_homing,timeout=45.0): #Will return on timeout or when status starts updating again
            self.logger.warning('%s homing timed out' % self.name.capitalize())
            self.cancel()
            success = False

        if success:
            self.logger.info('%s homing successful' % self.name.capitalize())
            return True
        self.logger.error('%s homing failed' % self.name.capitalize())
        return False


class RoutineWristJointHome(RoutineHome):
    def __init__(self,robot):
        RoutineHome.__init__(self,name='routine_wrist_joint_home',robot=robot)

    def _run_homing(self,cmd_id,*args, **kwargs):
        if not hasattr(self.robot,'eoa_loop') or not self.robot.eoa_loop.is_valid:
            self.logger.warning('Not able to home %s. Hardware not present' % self.name.capitalize())
            return False
        success = True
        def is_not_homing():
            return not self.robot.eoa_loop.status['is_homing']

        cmd = ['end_of_arm', 'home_joint',cmd_id, args,kwargs]
        
        self.robot.eoa_loop.q_cmd.put(cmd)
        self.wait_duration(1.5) #Let homing begin
        
        if not self.wait_on_cb(is_not_homing,timeout=15.0): #Will return on timeout or when status starts updating again
            self.logger.warning('%s homing timed out' % self.name.capitalize())
            success = False

        if success:
            self.logger.info('%s homing successful' % kwargs['joint_name'].capitalize())
            return True
        self.logger.error('%s homing failed' % kwargs['joint_name'].capitalize())
        return False

# ###############################################################3
class RoutinePrismaticJointHome(RoutineHome):
    def __init__(self,robot,name):
        RoutineHome.__init__(self,name=name,robot=robot)
        self.joint:PrismaticJoint =None


    def _run_homing(self,cmd_id,*args, **kwargs):
        """
        end_pos: position to end on
        to_positive_stop:
        -- True: Move to the positive direction stop and mark to range_m[1]
        -- False: Move to the negative direction stop and mark to range_m[0]
        v_m: max velocity to move by during homing
        a_m: accelration to move by during homing
        return True if success
        """

        if self.joint is None or (not self.joint.motor.hw_valid):
            self.logger.warning(f'Not able to home {self.name.capitalize()}. Hardware not present')
            return False

        success = True
        self.logger.info(f'Homing {self.joint.name.capitalize()}...')

        end_pos = self.joint.params['homing']['end_pos']
        to_positive_stop = self.joint.params['homing']['to_positive_stop']
        v_m = self.joint.params['homing']['v_m']
        a_m = self.joint.params['homing']['a_m']

        prev_enable_guarded_mode = self.joint.motor.gains['enable_guarded_mode']
        prev_enable_sync_mode = self.joint.motor.gains['enable_sync_mode']
        prev_safety_hold = self.joint.motor.gains['safety_hold']
        prev_safety_stiffness = self.joint.motor.gains['safety_stiffness']

        # Set contact behavior
        self.joint.motor.enable_guarded_mode()
        self.joint.motor.disable_sync_mode()
        self.joint.motor.gains['safety_hold']=self.joint.params['homing']['safety_hold']
        self.joint.motor.gains['safety_stiffness'] = self.joint.params['homing']['safety_stiffness']
        self.joint.motor.set_gains()

        self.joint.motor.reset_pos_calibrated()

        if not self.update_controller():
            return False

        if to_positive_stop:
            x_goal_1 = 5.0  # Well past the stop
        else:
            x_goal_1 = -5.0

        # Move to stop
        self.joint.move_by(x_m=x_goal_1, v_m=v_m, a_m=a_m,
                     contact_sensitivity_pos=self.joint.params['homing']['contact_sensitivity'],
                     contact_sensitivity_neg=self.joint.params['homing']['contact_sensitivity'], req_calibration=False)
        #self.wait_duration(t=1.0)

        if to_positive_stop:
            x = self.joint.translate_m_to_motor_rad(self.joint.params['range_m'][1])
        else:
            x = self.joint.translate_m_to_motor_rad(self.joint.params['range_m'][0])

        self.wait_duration(0.5)
        self.joint.motor.mark_position_on_contact(x)
        self.update_controller()

        if self.wait_until_contact(self.joint.motor, timeout=15.0, t_ignore=0.5):
            self.wait_duration(t=1.0)
            self.logger.info(f'Hardstop detected at motor position (rad) {self.joint.motor.status["pos"]}')
            x_dir_1 = self.joint.status['pos']
            self.logger.info(f'Marking {self.joint.name.capitalize()} position to {x} (rad)')
            self.joint.motor.set_pos_calibrated()
            self.update_controller()
        else:
            if self.is_canceled:
                self.joint.motor.reset_mark_position_on_contact()
                self.joint.enable_safety()
                self.update_controller()
            if not self.is_canceled:
                self.logger.warning('%s homing failed. Failed to detect contact' % self.name.capitalize())
            success = False
        self.wait_duration(0.5)  # Allow time to settle
        if success:
            self.joint.move_to(x_m=end_pos, req_calibration=False)
            if not self.wait_until_at_setpoint(self.joint.motor, timeout=10.0, t_ignore=0.5):
                self.logger.warning('%s failed to reach final position' % self.joint.name.capitalize())
                success = False

        # Restore previous modes
        self.joint.motor.gains['enable_guarded_mode'] = prev_enable_guarded_mode
        self.joint.motor.gains['enable_sync_mode'] = prev_enable_sync_mode
        self.joint.motor.gains['safety_hold']=prev_safety_hold
        self.joint.motor.gains['safety_stiffness'] = prev_safety_stiffness
        self.joint.motor.set_gains()
        self.update_controller()

        if success:
            self.logger.info('%s homing successful' % self.joint.name.capitalize())
            return True
        self.logger.error('%s homing failed' % self.joint.name.capitalize())
        return False


class RoutineLiftHome(RoutinePrismaticJointHome):
    def __init__(self,robot):
        RoutinePrismaticJointHome.__init__(self,name="routine_lift_home",robot=robot)
        if hasattr(robot,'lift'):
            self.joint=robot.lift
class RoutineArmHome(RoutinePrismaticJointHome):
    def __init__(self,robot):
        RoutinePrismaticJointHome.__init__(self,name="routine_arm_home",robot=robot)
        if hasattr(robot,'arm'):
            self.joint=robot.arm


class RoutineRobotHome(RoutineHome):
    def __init__(self,robot):
        RoutineHome.__init__(self,name="routine_robot_home",robot=robot)

        self.active_subroutine = None

    def cancel(self):
        if self.active_subroutine is not None:
            self.active_subroutine.cancel()
        super().cancel()


    def _run_homing(self,cmd_id,*args, **kwargs):
        success = True
        if success and self.robot.get_subsystem('lift') is not None:
            self.active_subroutine = RoutineLiftHome(self.robot)
            success = success and self.active_subroutine._run_homing(cmd_id,*args,**kwargs)
        if success and self.robot.get_subsystem('arm') is not None:
            self.active_subroutine = RoutineArmHome(self.robot)
            success = success and self.active_subroutine._run_homing(cmd_id,*args, **kwargs)
        if success and self.robot.get_subsystem('end_of_arm') is not None:
            self.active_subroutine = RoutineEndOfArmHome(self.robot)
            success = success and self.active_subroutine._run_homing(cmd_id,*args, **kwargs)
        if hasattr(self.robot,'power_periph'):
            if success:
                self.robot.power_periph.trigger_beep()
            else:
                self.robot.power_periph.trigger_beep()
                self.robot.power_periph.push_command()
                time.sleep(0.5)
                self.robot.power_periph.trigger_beep()

        if success:
            self.logger.info(f'{self.name} homing successful')
            return True

        self.logger.error(f'{self.name} homing failed')
        return False