
from typing import TypedDict

from stretch4_body.core.stepper import Stepper, StepperStatus
from stretch4_body.core.device import Device

import stretch4_body.core.hello_utils as hu
import time
import sys


class PrismaticJoint(Device):
    """
    API to the Stretch Prismatic Joints
    """
    def __init__(self,name,usb=None,motor_name=None):
        Device.__init__(self,name )
        if usb is None:
            usb=self.params['usb_name']
        if motor_name is None:
            if name == 'lift':
                motor_name = 'hello-motor-lift'
            elif name == 'arm':
                motor_name = 'hello-motor-arm'
        self.motor = Stepper(usb=usb, name=motor_name)

        self.status_aux = {}

        self.thread_rate_hz = 5.0

        # Default controller params
        self.stiffness = 1.0
        self.i_feedforward=self.params['i_feedforward']
        self.i_feedforward_payload=0
        self.vel_r = self.translate_m_to_motor_rad(self.params['motion']['default']['vel_m'])
        self.accel_r = self.translate_m_to_motor_rad(self.params['motion']['default']['accel_m'])
        self.accel_r_last = self.accel_r
        self.soft_motion_limits = {'hard': [self.params['range_m'][0], self.params['range_m'][1]],
                                   'current': [self.params['range_m'][0], self.params['range_m'][1]],
                                   'user': [None, None]}

        self.status: "PrismaticJointStatus" = {'timestamp_pc': 0, 
                                               'pos': 0.0, 
                                               'vel': 0.0, 
                                               'force': 0.0, 
                                               'motor': self.motor.status, 
                                               'in_collision_stop':{'pos': False, 'neg': False},
                                               'braking_distance':0,
                                               'soft_motion_limits': self.get_soft_motion_limits(),
                                               'at_limit':{'pos': False, 'neg': False}}

        self.in_vel_brake_zone = False
        self.in_vel_mode = False 
        self.dist_to_min_max = None # track dist to min,max limits
        self.vel_brake_zone_thresh = 0.02 # initial/minimum brake zone thresh value
        self._prev_set_vel_ts = None
        self._prev_collision_update_ts = None
        self.watchdog_enabled = False
        self.total_range = abs(self.params['range_m'][1] - self.params['range_m'][0])
        self.ts_collision_stop = {'pos': 0, 'neg': 0}
        self.contact_sensitivity_pos = self.motor.params['guarded_contact']['sensitivity_default']['coeff_sensitivity_pos']
        self.contact_sensitivity_neg = self.motor.params['guarded_contact']['sensitivity_default']['coeff_sensitivity_neg']

    # ###########  Device Methods #############
    def startup(self):
        # Startup stepper first so that status is populated before this Device thread begins
        self.logger.info('Starting %s...'%self.name.capitalize())
        success = self.motor.startup()
        if success:
            Device.startup(self)
            self.__update_status()
            self.motor.set_motion_limits(self.translate_m_to_motor_rad(self.soft_motion_limits['current'][0]),
                                         self.translate_m_to_motor_rad(self.soft_motion_limits['current'][1]))
            if int(str(self.motor.board_info['protocol_version'])[1:]) >= 8:
                self.set_guarded_contact_sensitivity('sensitivity_default')
        else:
            self.logger.error('Failed to start %s'%self.name.capitalize())
        return success

    def stop(self):
        Device.stop(self)
        self.motor.stop()

    def pull_status(self,blocking=True):
        success=self.motor.pull_status(blocking)
        self.__update_status()
        return success

    def load_rpc_results(self,wait_on_result=True):
        success = True
        success = success and self.motor.load_rpc_results(wait_on_result)
        return success

    def __update_status(self):
        self.status['timestamp_pc'] = time.time()
        self.status['pos'] = self.motor_rad_to_translate_m(self.status['motor']['pos'])
        self.status['vel'] = self.motor_rad_to_translate_m(self.status['motor']['vel'])
        self.status['braking_distance']=self.get_braking_distance()
        self.status['soft_motion_limits'] = self.get_soft_motion_limits()
        self.status['at_limit'] = self.get_at_limit(self.status['pos'])

    def push_command(self,blocking=True):
        return self.motor.push_command(blocking)


    def pretty_print(self):
        print('----- %s ------ '%self.name.capitalize())
        print('Pos (m): ', self.status['pos'])
        print('Vel (m/s): ', self.status['vel'])
        print('Soft motion limits (m)', self.soft_motion_limits['current'])
        print('Timestamp PC (s):', self.status['timestamp_pc'])
        self.motor.pretty_print()

    # ###########  Rate Logging #############
    def enable_rate_logging(self,max_samples=1000):
        self.motor.enable_rate_logging(max_samples)

    def get_rate_log(self):
        return self.motor.get_rate_log()

    # ###################################################
    def enable_safety(self):
        self.motor.enable_safety()

    def disable_guarded_mode(self):
        self.motor.disable_guarded_mode()

    def enable_guarded_mode(self):
        self.motor.enable_guarded_mode()

    def disable_sync_mode(self):
        self.motor.disable_sync_mode()

    def enable_sync_mode(self):
        self.motor.enable_sync_mode()

    def disable_runstop(self):
        self.motor.disable_runstop()

    def enable_runstop(self):
        self.motor.enable_runstop()

    def get_at_limit(self, pos):
        joint_min, joint_max = self.get_soft_motion_limits()

        return {'pos': pos >= joint_max-0.01, 'neg': pos <= joint_min+0.01}

    def get_soft_motion_limits(self):
        """
            Return the currently applied soft motion limits: [min, max]

            The soft motion limit restricts joint motion to be <= its physical limits.

            There are two types of limits:
            Hard: The physical limits
            User: Limits set by the user software

            The joint is limited to the most restrictive range of the Hard / User values.
            Specifying a value of None for a limit indicates that no constraint exists for that limit type.
            This allows a User limits to be disabled.

        """
        return self.soft_motion_limits['current']

    def set_soft_motion_limit_min(self,x):
        """
        x: value to set a joints limit to
        """
        self.soft_motion_limits['user'][0]=x
        xn=max(filter(lambda x: x is not None, [self.soft_motion_limits['hard'][0],self.soft_motion_limits['user'][0]]))
        prev=self.soft_motion_limits['current'][:]
        self.soft_motion_limits['current'][0]=xn
        if xn != prev[0]:
            #print('New soft limit on min',xn)
            self.motor.set_motion_limits(self.translate_m_to_motor_rad(self.soft_motion_limits['current'][0]), self.translate_m_to_motor_rad(self.soft_motion_limits['current'][1]))

    def set_soft_motion_limit_max(self,x):
        """
        x: value to set a joints limit to
        """
        self.soft_motion_limits['user'][1]=x
        xn=min(filter(lambda x: x is not None, [self.soft_motion_limits['hard'][1],self.soft_motion_limits['user'][1]]))
        prev=self.soft_motion_limits['current'][:]
        self.soft_motion_limits['current'][1]=xn
        if xn != prev[1]:
            #print('New soft limit on max', xn)
            self.motor.set_motion_limits(self.translate_m_to_motor_rad(self.soft_motion_limits['current'][0]), self.translate_m_to_motor_rad(self.soft_motion_limits['current'][1]))

    # ###################################################



    def set_guarded_contact_sensitivity(self, mode_name=None):
        if self.motor.hw_valid and int(str(self.motor.board_info['protocol_version'])[1:]) < 8:
            raise NotImplementedError('This method not supported for firmware on protocol {0}.'.format(self.motor.board_info['protocol_version']))
        mode_name = mode_name or 'sensitivity_default'
        if mode_name == 'off':
            self.disable_guarded_mode()
            return
        self.enable_guarded_mode()
        c_sens_p = self.contact_sensitivity_pos if self.motor.params['guarded_contact'][mode_name]['coeff_sensitivity_pos'] is None else self.motor.params['guarded_contact'][mode_name]['coeff_sensitivity_pos']
        c_sens_n = self.contact_sensitivity_neg if self.motor.params['guarded_contact'][mode_name]['coeff_sensitivity_neg'] is None else self.motor.params['guarded_contact'][mode_name]['coeff_sensitivity_neg']
        self.contact_sensitivity_pos = c_sens_p
        self.contact_sensitivity_neg = c_sens_n
        self.motor.set_guarded_contact_sensitivity(c_sens_p, c_sens_n)

    def set_velocity(self, v_m, a_m=None,stiffness=None, req_calibration=True,
                     contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        v_m: commanded joint velocity (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        req_calibration: Disallow motion prior to homing
        contact_sensitivity_pos: postive contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        contact_sensitivity_neg: negative contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        """

   
        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('%s not calibrated'%self.name.capitalize())
                return

        if self.status['in_collision_stop']['pos'] and v_m > 0:
            self.logger.warning(
                'In collision. Motion disabled in direction %s for %s. Not executing set_velocity' % ('pos', self.name),
                extra={'throttle_s': 1.0}
            )
            return
        elif self.status['in_collision_stop']['neg'] and v_m < 0:
            self.logger.warning(
                'In collision. Motion disabled in direction %s for %s. Not executing set_velocity' % ('neg', self.name),
                extra={'throttle_s': 1.0}
            )
            return

        v_m=min(self.params['motion']['max']['vel_m'],v_m) if v_m>=0 else max(-1*self.params['motion']['max']['vel_m'],v_m)
        v_r = self.translate_m_to_motor_rad(v_m)

        if stiffness is not None:
            stiffness = max(0.0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if a_m is not None:
            a_r = self.translate_m_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        self.accel_r_last = a_r

        if contact_sensitivity_pos is None:
            contact_sensitivity_pos = self.contact_sensitivity_pos

        if contact_sensitivity_neg is None:
            contact_sensitivity_neg = self.contact_sensitivity_neg

        if self.params['use_vel_traj']:
            ctrl_mode = Stepper.MODE_VEL_TRAJ
        else:
            ctrl_mode = Stepper.MODE_VEL_PID

        self.motor.set_command(mode=ctrl_mode,
                            v_des=v_r,
                            a_des=a_r,
                            stiffness=stiffness,
                            i_feedforward=self.i_feedforward+self.i_feedforward_payload,
                            coeff_sensitivity_pos=contact_sensitivity_pos,
                            coeff_sensitivity_neg=contact_sensitivity_neg)
        self.in_vel_mode = True

    def is_sync_required(self,ts_last_motor_sync):
        return self.motor.is_sync_required(ts_last_motor_sync)

    def move_to(self,x_m,v_m=None, a_m=None, stiffness=None,
                req_calibration=True,contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        x_m: commanded absolute position (meters). x_m=0 is down. x_m=~1.1 is up
        v_m: velocity for trapezoidal motion profile (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        req_calibration: Disallow motion prior to homing
        contact_sensitivity_pos: postive contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        contact_sensitivity_neg: negative contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        """

        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('%s not calibrated'%self.name.capitalize())
                return
            old_x_m = x_m
            x_m = min(max(self.soft_motion_limits['current'][0], x_m), self.soft_motion_limits['current'][1]) #Only clip motion when calibrated
            if x_m != old_x_m:
                self.logger.debug(f'Clipping move_to({old_x_m}) with soft limits {self.soft_motion_limits['current']}')

        if self.status['in_collision_stop']['pos'] and self.status['pos'] < x_m:
            self.logger.warning(
                'In collision. Motion disabled in direction %s for %s. Not executing move_by' % ('pos', self.name),
                extra={'throttle_s': 1.0}
            )
            return

        if self.status['in_collision_stop']['neg'] and self.status['pos'] > x_m:
            self.logger.warning(
                'In collision. Motion disabled in direction %s for %s. Not executing move_by' % ('neg', self.name),
                extra={'throttle_s': 1.0}
            )
            return

        if stiffness is not None:
            stiffness = max(0.0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if v_m is not None:
            v_r=self.translate_m_to_motor_rad(min(abs(v_m), self.params['motion']['max']['vel_m']))
        else:
            v_r = self.vel_r

        if a_m is not None:
            a_r = self.translate_m_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        self.accel_r_last = a_r

        if contact_sensitivity_pos is None:
            contact_sensitivity_pos = self.contact_sensitivity_pos
        if contact_sensitivity_neg is None:
            contact_sensitivity_neg = self.contact_sensitivity_neg

        self.motor.set_command(mode = Stepper.MODE_POS_TRAJ,
                                x_des=self.translate_m_to_motor_rad(x_m),
                                v_des=v_r,
                                a_des=a_r,
                                stiffness=stiffness,
                                i_feedforward=self.i_feedforward+self.i_feedforward_payload,
                                coeff_sensitivity_pos=contact_sensitivity_pos,
                                coeff_sensitivity_neg=contact_sensitivity_neg)
        self.in_vel_mode = False

    def move_by(self,x_m,v_m=None, a_m=None, stiffness=None,  req_calibration=True,
                contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        x_m: commanded incremental motion (meters).
        v_m: velocity for trapezoidal motion profile (m/s)
        a_m: acceleration for trapezoidal motion profile (m/s^2)
        stiffness: stiffness of motion. Range 0.0 (min) to 1.0 (max)
        req_calibration: Disallow motion prior to homing
        contact_sensitivity_pos: postive contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        contact_sensitivity_neg: negative contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        """

        if req_calibration:
            if not self.motor.status['pos_calibrated']:
                self.logger.warning('%s not calibrated'%self.name.capitalize())
                return
            else:
                old_x_m = x_m
                if self.status['pos'] + x_m < self.soft_motion_limits['current'][0]:  #Only clip motion when calibrated
                    x_m = self.soft_motion_limits['current'][0] - self.status['pos']
                if self.status['pos'] + x_m > self.soft_motion_limits['current'][1]:
                    x_m = self.soft_motion_limits['current'][1] - self.status['pos']
                if x_m != old_x_m:
                    self.logger.debug(
                        'Clipping {0} + move_by({1}) with soft limits {2}'.format(self.status['pos'], old_x_m,
                                                                                  self.soft_motion_limits['current']))

        # Handle collision logic
        if self.status['in_collision_stop']['pos'] and x_m > 0:
            self.logger.warning(
                'In collision. Motion disabled in direction %s for %s. Not executing move_by' % ('pos', self.name),
                extra={'throttle_s': 1.0}
            )
            return

        if self.status['in_collision_stop']['neg'] and x_m < 0:
            self.logger.warning(
                'In collision. Motion disabled in direction %s for %s. Not executing move_by' % ('neg', self.name),
                extra={'throttle_s': 1.0}
            )
            return

        if stiffness is not None:
            stiffness = max(0.0, min(1.0, stiffness))
        else:
            stiffness = self.stiffness

        if v_m is not None:
            v_r=self.translate_m_to_motor_rad(min(abs(v_m), self.params['motion']['max']['vel_m']))
        else:
            v_r = self.vel_r

        if a_m is not None:
            a_r = self.translate_m_to_motor_rad(min(abs(a_m), self.params['motion']['max']['accel_m']))
        else:
            a_r = self.accel_r

        self.accel_r_last = a_r

        if contact_sensitivity_pos is None:
            contact_sensitivity_pos = self.contact_sensitivity_pos
        if contact_sensitivity_neg is None:
            contact_sensitivity_neg = self.contact_sensitivity_neg
        self.motor.set_command(mode = Stepper.MODE_POS_TRAJ_INCR,
                                x_des=self.translate_m_to_motor_rad(x_m),
                                v_des=v_r,
                                a_des=a_r,
                                stiffness=stiffness,
                                i_feedforward=self.i_feedforward+self.i_feedforward_payload,
                                coeff_sensitivity_pos=contact_sensitivity_pos,
                                coeff_sensitivity_neg=contact_sensitivity_neg)
        self.in_vel_mode = False

    # ######### Utility ##############################

    def motor_rad_to_translate_m(self,ang): #Override
        self.logger.warning('motor_rad_to_translate_m not implemented in %s'%self.name)
        pass

    def translate_m_to_motor_rad(self, x):#Override
        self.logger.warning('motor_rad_to_translate_m not implemented in %s' % self.name)
        pass

    def wait_until_at_setpoint(self, timeout=15.0):
        return self.motor.wait_until_at_setpoint(timeout=timeout)

    def wait_while_is_moving(self,timeout=15.0, use_motion_generator=True):
        return self.motor.wait_while_is_moving(timeout=timeout, use_motion_generator=use_motion_generator)

    def wait_for_contact(self, timeout=5.0):
        ts=time.time()
        while (time.time()-ts<timeout):
            self.pull_status()
            if self.motor.status['in_guarded_event']:
                return True
            if self.motor.status['runstop_on']:
                return False
            time.sleep(0.01)
        return False

    def step_collision_avoidance(self,in_collision):
        """
        Disable the ability to command motion in the positive or negative direction
        If the joint is in motion in that direction, force it to stop
        Parameters
        ----------
        in_collision: {'pos': False, 'neg': False},etc
        """

        # if in_collision['pos'] and in_collision['neg']:
        #     print('Invalid IN_COLLISION for joint %s'%self.name)
        #     return
        if not self.is_homed():
            return

        for dir in ['pos', 'neg']:
            if in_collision[dir] and not self.status['in_collision_stop'][dir]:
                # Stop current motion
                self.motor.enable_safety()
                self.push_command()
                self.status['in_collision_stop'][dir] = True
                self.ts_collision_stop[dir] = time.time()
                self.collision_till_zero_vel_counter = 0

            # Reset if out of collision (at least 1s after collision)
            if self.status['in_collision_stop'][dir] and not in_collision[dir] and time.time() - self.ts_collision_stop[dir] > 1.0:
                self.status['in_collision_stop'][dir] = False

    def get_braking_distance(self, v=None, acc=None):
        """Compute distance to brake the joint from velocity v (default current status['vel'])"""
        if v is None:
            v = self.status['vel']
        if acc is None:
            acc = self.params['motion']['max']['accel_m']
        if acc <= 0:
            return 0.0
        t_brake = abs(v / acc)  # How long to brake from speed (s)
        d_brake = t_brake * abs(v) / 2.0  # How far it will go before stopping
        return d_brake

    # ############# Safe velocity control ###################

    def get_safe_velocity(self, v_des, v_deadband=.005, pad_m=0.003):

        if not self.params['set_safe_velocity'] or abs(v_des) < v_deadband: # m/s, filter noise if v_des is encoder signal, not controller setp
            return v_des

        x_curr = self.status['pos']
        v_curr = self.status['vel']
        lim_lower = self.get_soft_motion_limits()[0]
        lim_upper = self.get_soft_motion_limits()[1]

        accel_m = self.motor_rad_to_translate_m(self.accel_r_last)
        v_eval = max(abs(v_curr), abs(v_des))
        d_brake = self.get_braking_distance(v=v_eval, acc=accel_m) + pad_m

        if v_des > 0:
            if x_curr >= lim_upper - d_brake:
                v_des = 0.0
        elif v_des < 0:
            if x_curr <= lim_lower + d_brake:
                v_des = 0.0
        return v_des

    def step_sentry(self,robot_status):
        self.motor.step_sentry(robot_status)


    def is_homed(self):
        return self.motor.status['pos_calibrated']

    def home(self):
        """
          end_pos: position to end on
          to_positive_stop:
          -- True: Move to the positive direction stop and mark to range_m[1]
          -- False: Move to the negative direction stop and mark to range_m[0]
          v_m: max velocity to move by during homing
          a_m: accelration to move by during homing
          return True if successful
          """

        if not self.motor.hw_valid:
            self.logger.warning('Not able to home %s. Hardware not present' % self.name.capitalize())
            return False

        end_pos = self.params['homing']['end_pos']
        to_positive_stop = self.params['homing']['to_positive_stop']
        v_m = self.params['homing']['v_m']
        a_m = self.params['homing']['a_m']

        success = True
        print('Homing %s...' % self.name.capitalize())
        self.pull_status()

        # Set contact behavior
        prev_enable_guarded_mode = self.motor.gains['enable_guarded_mode']
        prev_enable_sync_mode = self.motor.gains['enable_sync_mode']
        prev_safety_hold = self.motor.gains['safety_hold']
        prev_safety_stiffness = self.motor.gains['safety_stiffness']

        self.motor.enable_guarded_mode()
        self.motor.disable_sync_mode()
        self.motor.gains['safety_hold'] = self.params['homing']['safety_hold']
        self.motor.gains['safety_stiffness'] = self.params['homing']['safety_stiffness']

        self.motor.set_gains()

        self.motor.reset_pos_calibrated()
        self.push_command()
        self.pull_status()

        if to_positive_stop:
            x_goal_1 = 5.0  # Well past the stop
        else:
            x_goal_1 = -5.0
        self.move_by(x_m=x_goal_1, v_m=v_m, a_m=a_m,
                     contact_sensitivity_pos=self.params['homing']['contact_sensitivity'],
                     contact_sensitivity_neg=self.params['homing']['contact_sensitivity'], req_calibration=False)
        self.push_command()
        if to_positive_stop:
            x = self.translate_m_to_motor_rad(self.params['range_m'][1])
        else:
            x = self.translate_m_to_motor_rad(self.params['range_m'][0])

        time.sleep(0.5)
        self.motor.mark_position_on_contact(x)
        self.push_command()

        # Move to stop
        # self.motor.pretty_print()
        if self.wait_for_contact(timeout=15.0):  # timeout=15.0):
            # input('Enter to continue')  # Needs time to settle
            #time.sleep(1.0)
            self.pull_status()
            # self.motor.pretty_print()
            self.logger.info(f'Hardstop detected at motor position (rad) {self.motor.status["pos"]}')

            if to_positive_stop:
                self.logger.info(f'Marking {self.name.capitalize()} position to {self.params["range_m"][1]} (m)')
            else:
                self.logger.info(f'Marking {self.name.capitalize()} position to {self.params["range_m"][0]} (m)')
            self.motor.set_pos_calibrated()
            self.push_command()
        else:
            self.logger.warning('%s homing failed. Failed to detect contact' % self.name.capitalize())
            self.motor.reset_mark_position_on_contact()
            self.motor.enable_safety()
            self.push_command()
            success = False
        # input('Enter to continue2')
        time.sleep(1.0)
        if success:
            self.move_to(x_m=end_pos, req_calibration=False)
            self.push_command()
            # time.sleep(1.0)
            if not self.motor.wait_until_at_setpoint():
                self.logger.warning('%s failed to reach final position' % self.name.capitalize())
                success = False

        # Restore previous modes
        self.motor.gains['enable_guarded_mode'] = prev_enable_guarded_mode
        self.motor.gains['enable_sync_mode'] = prev_enable_sync_mode
        self.motor.gains['safety_hold'] = prev_safety_hold
        self.motor.gains['safety_stiffness'] = prev_safety_stiffness
        self.motor.set_gains()

        self.push_command()
        if success:
            self.logger.info(f'{self.name.capitalize()} homing successful')
        return success

    def home2(self):
        """
        end_pos: position to end on
        to_positive_stop:
        -- True: Move to the positive direction stop and mark to range_m[1]
        -- False: Move to the negative direction stop and mark to range_m[0]
        v_m: max velocity to move by during homing
        a_m: accelration to move by during homing
        return True if successful
        """

        if not self.motor.hw_valid:
            self.logger.warning('Not able to home %s. Hardware not present' % self.name.capitalize())
            return False

        end_pos = self.params['homing']['end_pos']
        to_positive_stop = self.params['homing']['to_positive_stop']
        v_m = self.params['homing']['v_m']
        a_m = self.params['homing']['a_m']

        success = True
        self.logger.info(f'Homing {self.name.capitalize()}...')
        self.pull_status()

        prev_enable_guarded_mode = self.motor.gains['enable_guarded_mode']
        prev_enable_sync_mode = self.motor.gains['enable_sync_mode']
        prev_safety_hold = self.motor.gains['safety_hold']
        prev_safety_stiffness = self.motor.gains['safety_stiffness']
        self.motor.enable_guarded_mode()
        self.motor.disable_sync_mode()

        self.motor.reset_pos_calibrated()
        self.push_command()
        self.pull_status()

        if to_positive_stop:
            x_goal_1 = 5.0  # Well past the stop
        else:
            x_goal_1 = -5.0

        # Move to stop

        self.move_by(x_m=x_goal_1, v_m=v_m, a_m=a_m,
                     contact_sensitivity_pos=self.params['homing']['contact_sensitivity'],
                     contact_sensitivity_neg=self.params['homing']['contact_sensitivity'], req_calibration=False)
        self.push_command()

        # self.motor.pretty_print()
        if self.wait_for_contact(timeout=15.0):  # timeout=15.0):
            # input('Enter to continue')  # Needs time to settle
            time.sleep(1.0)
            self.pull_status()
            # self.motor.pretty_print()
            self.logger.info(f'Hardstop detected at motor position (rad) {self.motor.status["pos"]}')
            x_dir_1 = self.status['pos']

            if to_positive_stop:
                x = self.translate_m_to_motor_rad(self.params['range_m'][1])
                self.logger.info(f'Marking {self.name.capitalize()} position to {self.params["range_m"][1]} (m)')
            else:
                x = self.translate_m_to_motor_rad(self.params['range_m'][0])
                self.logger.info(f'Marking {self.name.capitalize()} position to {self.params["range_m"][0]} (m)')
            self.motor.mark_position(x)
            self.motor.set_pos_calibrated()
            self.push_command()

        else:
            self.logger.warning('%s homing failed. Failed to detect contact' % self.name.capitalize())
            success = False
        # input('Enter to continue2')
        time.sleep(1.0)
        if success:
            self.move_to(x_m=end_pos, req_calibration=False)
            self.push_command()
            # time.sleep(1.0)
            if not self.motor.wait_until_at_setpoint():
                self.logger.warning('%s failed to reach final position' % self.name.capitalize())
                success = False

        # Restore previous settings
        self.motor.gains['enable_guarded_mode'] = prev_enable_guarded_mode
        self.motor.gains['enable_sync_mode'] = prev_enable_sync_mode
        self.motor.gains['safety_hold']=prev_safety_hold
        self.motor.gains['safety_stiffness'] = prev_safety_stiffness
        self.motor.set_gains()

        self.push_command()
        if success:
            self.logger.info(f'{self.name.capitalize()} homing successful')
        return success


class PrismaticJointCollisionStatus(TypedDict):
    pos:bool
    neg: bool
class PrismaticJointStatus(TypedDict):
    timestamp_pc: float
    pos:float
    vel: float 
    force: float 
    motor: StepperStatus
    in_collision_stop: PrismaticJointCollisionStatus
    braking_distance: float 