
from stretch4_body.core.prismatic_joint import PrismaticJoint
from stretch4_body.core.stepper import Stepper
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


    def run(self, cmd_id, *args, **kwargs):

        super().run(cmd_id, *args, **kwargs)

        sm_paused, sentry_paused = self.pause_behaviors()
        try:
            success = self._run_homing(cmd_id, *args, **kwargs)
        except:
            self.cancel()
            success = False
        finally:
            self.unpause_behaviors(sm_paused, sentry_paused)

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

        kwargs['wait_on_completion'] = False

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

        kwargs['wait_on_completion'] = False
        
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
        Home the joint using the procedure selected by joint.params['homing']['controller']:
          'imax': current-limited push into the hardstop with stall detection (see _run_homing_imax)
          'gc': guarded contact detection at the hardstop (see _run_homing_gc)
        Joints without a 'controller' param use 'gc'.
        Return True if success
        """
        if self.joint is None or (not self.joint.motor.hw_valid):
            self.logger.warning(f'Not able to home {self.name.capitalize()}. Hardware not present')
            return False

        controller = self.joint.params['homing'].get('controller', 'gc')
        src = 'homing.controller param' if 'controller' in self.joint.params['homing'] else 'default, no homing.controller param'
        self.logger.info(f'Homing {self.joint.name.capitalize()} using controller: {controller} ({src})')
        if controller == 'imax':
            return self._run_homing_imax(cmd_id, *args, **kwargs)
        if controller == 'gc':
            return self._run_homing_gc(cmd_id, *args, **kwargs)
        self.logger.error(f'Unknown homing controller {controller} for {self.joint.name.capitalize()}. '
                          f'Expected imax or gc')
        return False

    def _run_homing_imax(self,cmd_id,*args, **kwargs):
        """
        Home by pushing into the hardstop with limited motor current, detecting the stall via joint velocity.
        Server/routine version of PrismaticJoint.home_imax().
          1) Lower the motor current limits to homing.iMax_pos / homing.iMax_neg and disable guarded contacts
          2) Command motion well past the hardstop
          3) Wait t_ignore, then monitor velocity until it is near zero (joint stalled at the hardstop)
          4) Keep pushing for t_settle, then mark the home position (without leaving the motion mode)
          5) Restore guarded contacts / current limits and move to homing.end_pos
        Return True if success
        """
        # Keep in sync with PrismaticJoint.home_imax()
        t_ignore = 1.0        # Time after commanding motion before velocity monitoring starts (s)
        vel_thresh = 0.005    # |velocity| below which the joint is considered stalled (m/s)
        stall_samples = 3     # Consecutive control cycles below vel_thresh required to declare a stall
        t_settle = 0.5        # Time to keep pushing against the hardstop before marking (s)
        timeout_stall = 15.0  # Max time to wait for a stall (s)
        timeout_end = 10.0    # Max time to reach end_pos (s)

        success = True
        name = self.joint.name.capitalize()

        end_pos = self.joint.params['homing']['end_pos']
        to_positive_stop = self.joint.params['homing']['to_positive_stop']
        v_m = self.joint.params['homing']['v_m']
        a_m = self.joint.params['homing']['a_m']
        x_mark_m = self.joint.params['range_m'][1] if to_positive_stop else self.joint.params['range_m'][0]
        self.logger.info(f'[{name} imax] iMax_pos={self.joint.params["homing"]["iMax_pos"]} A, '
                         f'iMax_neg={self.joint.params["homing"]["iMax_neg"]} A, v_m={v_m}, a_m={a_m}, '
                         f'to_positive_stop={to_positive_stop}, mark={x_mark_m} m, end_pos={end_pos} m')

        # Track motor current while pushing toward / against the hardstop
        i_peak_approach = [0.0]  # Peak |current| from motion command until stall (signed value kept)
        i_peak_stop = [0.0]      # Peak |current| from stall until marking, i.e. while pushing on the hardstop
        def track_current(peak):
            i = self.joint.motor.status['current']
            if abs(i) > abs(peak[0]):
                peak[0] = i

        def wait_tracking(t, peak):
            """wait_duration(t) that also tracks peak current. Return False if canceled."""
            ts = time.time()
            def done():
                track_current(peak)
                return time.time() - ts >= t
            return self.wait_on_cb(done, timeout=t + 1.0)

        # Stash gains to restore afterwards
        prev_enable_guarded_mode = self.joint.motor.gains['enable_guarded_mode']
        prev_enable_sync_mode = self.joint.motor.gains['enable_sync_mode']
        prev_safety_hold = self.joint.motor.gains['safety_hold']
        prev_safety_stiffness = self.joint.motor.gains['safety_stiffness']
        prev_iMax_pos = self.joint.motor.gains['iMax_pos']
        prev_iMax_neg = self.joint.motor.gains['iMax_neg']

        try:
            # Limit current and disable guarded contacts
            self.joint.motor.gains['iMax_pos'] = self.joint.params['homing']['iMax_pos']
            self.joint.motor.gains['iMax_neg'] = self.joint.params['homing']['iMax_neg']
            self.joint.motor.disable_guarded_mode()
            self.joint.motor.disable_sync_mode()
            self.joint.motor.set_gains()
            self.joint.motor.reset_pos_calibrated()

            if not self.update_controller():
                return False
            self.logger.info(f'[{name} imax] Current limits set to [{self.joint.motor.gains["iMax_neg"]}, '
                             f'{self.joint.motor.gains["iMax_pos"]}] A, guarded contacts off, sync mode off')

            # Command motion well past the hardstop
            x_goal = 5.0 if to_positive_stop else -5.0
            self.joint.move_by(x_m=x_goal, v_m=v_m, a_m=a_m, req_calibration=False)
            self.logger.info(f'[{name} imax] Moving to {"positive" if to_positive_stop else "negative"} hardstop. '
                             f'Ignoring velocity for {t_ignore} s')

            # Wait, then monitor velocity until stalled at the hardstop
            n_below = [0]
            def is_stalled():
                track_current(i_peak_approach)
                # Use motor status directly: joint.status['vel'] is only refreshed on pull_status, one cycle stale
                vel = self.joint.motor_rad_to_translate_m(self.joint.motor.status['vel'])
                n_below[0] = n_below[0] + 1 if abs(vel) < vel_thresh else 0
                return n_below[0] >= stall_samples

            if not wait_tracking(t_ignore, i_peak_approach):
                success = False
            else:
                self.logger.info(f'[{name} imax] Monitoring velocity for stall (|v| < {vel_thresh} m/s for '
                                 f'{stall_samples} cycles, timeout {timeout_stall} s)')
                if self.wait_on_cb(is_stalled, timeout=timeout_stall):
                    self.logger.info(f'[{name} imax] Hardstop detected at motor position (rad) '
                                     f'{self.joint.motor.status["pos"]:.4f}, current {self.joint.motor.status["current"]:+.3f} A, '
                                     f'peak current during approach {i_peak_approach[0]:+.3f} A')
                    # Keep pushing against the hardstop to settle
                    self.logger.info(f'[{name} imax] Settling against hardstop for {t_settle} s')
                    if not wait_tracking(t_settle, i_peak_stop):
                        success = False
                else:
                    if not self.is_canceled:
                        self.logger.warning(f'[{name} imax] Homing failed. No stall detected. '
                                            f'Peak current during approach {i_peak_approach[0]:+.3f} A')
                    success = False

            if success:
                # Mark the home position while still pushing against the hardstop.
                # Stepper.mark_position() refuses unless in MODE_SAFETY/HOLD, but that check is Python-side only:
                # the firmware applies TRIGGER_MARK_POS in any mode and resets the motion generator to the marked
                # position (stretch_firmware_ii hello_stepper2/HelloController.cpp), so set the trigger directly.
                #
                # Mark, gain restore and the move to end_pos are staged together and sent in ONE controller update.
                # Stepper.push_command() always sends trigger -> gains -> command in that order, so the firmware marks
                # first, then restores gains, then starts the move in the newly marked frame. This avoids any window
                # where the old incremental target (in the uncalibrated frame) is still active with full current.
                #
                # Sync mode is restored only after the final move, matching PrismaticJoint.home_imax().
                self.logger.info(f'[{name} imax] Marking position to {x_mark_m} m. '
                                 f'Current at mark {self.joint.motor.status["current"]:+.3f} A, '
                                 f'peak current on hardstop {i_peak_stop[0]:+.3f} A '
                                 f'(limits [{self.joint.motor.gains["iMax_neg"]}, {self.joint.motor.gains["iMax_pos"]}] A)')
                self.joint.motor._trigger_data = self.joint.translate_m_to_motor_rad(x_mark_m)
                self.joint.motor._trigger |= Stepper.TRIGGER_MARK_POS
                self.joint.motor._dirty_trigger = True
                self.joint.motor.set_pos_calibrated()

                self.joint.motor.gains['enable_guarded_mode'] = prev_enable_guarded_mode
                self.joint.motor.gains['safety_hold'] = prev_safety_hold
                self.joint.motor.gains['safety_stiffness'] = prev_safety_stiffness
                self.joint.motor.gains['iMax_pos'] = prev_iMax_pos
                self.joint.motor.gains['iMax_neg'] = prev_iMax_neg
                self.joint.motor.set_gains()

                self.joint.move_to(x_m=end_pos, req_calibration=False)
                self.logger.info(f'[{name} imax] Restored guarded contacts and current limits '
                                 f'[{prev_iMax_neg}, {prev_iMax_pos}] A. Moving to end_pos {end_pos} m')

                if self.wait_until_at_setpoint(self.joint.motor, timeout=timeout_end, t_ignore=0.5):
                    self.logger.info(f'[{name} imax] Reached end_pos. Position {self.joint.status["pos"]:.4f} m')
                else:
                    self.logger.warning(f'[{name} imax] Failed to reach final position')
                    success = False
            else:
                self.logger.info(f'[{name} imax] Entering safety mode')
                self.joint.enable_safety()
        finally:
            # Restore all gains, including sync mode (also covers cancel / exception paths)
            self.joint.motor.gains['enable_guarded_mode'] = prev_enable_guarded_mode
            self.joint.motor.gains['enable_sync_mode'] = prev_enable_sync_mode
            self.joint.motor.gains['safety_hold'] = prev_safety_hold
            self.joint.motor.gains['safety_stiffness'] = prev_safety_stiffness
            self.joint.motor.gains['iMax_pos'] = prev_iMax_pos
            self.joint.motor.gains['iMax_neg'] = prev_iMax_neg
            self.joint.motor.set_gains()
            self.update_controller()
            self.logger.info(f'[{name} imax] Restored all homing gains (including sync mode)')

        if success:
            self.logger.info('%s homing successful' % self.joint.name.capitalize())
            return True
        self.logger.error('%s homing failed' % self.joint.name.capitalize())
        return False

    def _run_homing_gc(self,cmd_id,*args, **kwargs):
        """
        Home using guarded contact detection at the hardstop.
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
        name = self.joint.name.capitalize()

        end_pos = self.joint.params['homing']['end_pos']
        to_positive_stop = self.joint.params['homing']['to_positive_stop']
        v_m = self.joint.params['homing']['v_m']
        a_m = self.joint.params['homing']['a_m']
        self.logger.info(f'[{name} gc] contact_sensitivity={self.joint.params["homing"]["contact_sensitivity"]}, '
                         f'v_m={v_m}, a_m={a_m}, to_positive_stop={to_positive_stop}, end_pos={end_pos} m')

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
            self.logger.info(f'[{name} gc] Hardstop detected at motor position (rad) {self.joint.motor.status["pos"]}, '
                             f'current {self.joint.motor.status["current"]:+.3f} A')
            x_dir_1 = self.joint.status['pos']
            self.logger.info(f'[{name} gc] Marked position on contact to {x} (rad)')
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
        # Restore previous modes
        self.joint.motor.gains['enable_guarded_mode'] = prev_enable_guarded_mode
        self.joint.motor.gains['enable_sync_mode'] = prev_enable_sync_mode
        self.joint.motor.gains['safety_hold']=prev_safety_hold
        self.joint.motor.gains['safety_stiffness'] = prev_safety_stiffness
        self.joint.motor.set_gains()
        self.update_controller()
        if success:
            self.joint.move_to(x_m=end_pos, req_calibration=False)
            if not self.wait_until_at_setpoint(self.joint.motor, timeout=10.0, t_ignore=0.5):
                self.logger.warning('%s failed to reach final position' % self.joint.name.capitalize())
                success = False



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