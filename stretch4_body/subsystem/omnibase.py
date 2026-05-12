#!/usr/bin/env python3

from stretch4_body.core.stepper import *
from stretch4_body.core.device import Device
from stretch4_body.core.hello_utils import *

import time
import math
import numpy as np

class OmniBase(Device):
    """
    API to the Stretch Mobile Base
    """
    def __init__(self):
        Device.__init__(self, 'omnibase')
        self.wheels=[Stepper(usb='/dev/hello-motor-omni-0', name='hello-motor-omni-0'),
                     Stepper(usb='/dev/hello-motor-omni-1', name='hello-motor-omni-1'),
                     Stepper(usb='/dev/hello-motor-omni-2', name='hello-motor-omni-2')]
        self.status: "OmnibaseStatus" = {'timestamp_pc': 0, 'x': 0, 'y': 0, 'theta': 0, 'x_vel': 0, 'y_vel': 0, 'theta_vel': 0,
                       'pose_time_s': 0, 'effort': (0, 0,0), 'wheel_0': self.wheels[0].status,
                       'wheel_1': self.wheels[1].status,'wheel_2': self.wheels[2].status}
        self.status_aux = {}

        self.thread_rate_hz = 5.0
        self.first_step=True

        # Default controller params
        self.stiffness=1.0
        self.fast_motion_allowed = True

        self.H0 = H0_from_driving_dir(self.params['wheel_diameter_m'], self.params['base_radius_m'], self.params['forward_dir'])
        self.H0_inv = inverse_3x3_matrix(self.H0)
        self._init_odom = time.perf_counter()
        self.status['pose_time_s'] = None
        self.lsvl=None #Line sensor velocity limit. Set by Robot when possible.
        self.contact_sensitivity_pos = 0.5
        self.contact_sensitivity_neg = 0.5
        self.guarded_mode_active=False

     
        self.curr_max_vel_xy_m = self.params['motion']['max']['vel_xy_m']
        self.curr_max_vel_w_r = self.params['motion']['max']['vel_w_r']
        self.curr_max_accel_xy_m = self.params['motion']['max']['accel_xy_m']
        self.curr_max_accel_w_r = self.params['motion']['max']['accel_w_r']
        
    # ###########  Device Methods #############

    def startup(self):
        #Startup steppers first so that status is populated before this Device thread begins
        self.logger.info('Starting Omnibase...')
        self.motor_valid = True
        success = self.wheels[0].startup() and self.wheels[1].startup() and self.wheels[2].startup()
        if success:
            Device.startup(self)
            self._update_odom(None)
            if (self.params['enable_guarded_mode'] 
                and int(str(self.wheels[0].board_info['protocol_version'])[1:]) >= 8 
                and int(str(self.wheels[1].board_info['protocol_version'])[1:]) >= 8 
                and int(str(self.wheels[2].board_info['protocol_version'])[1:]) >= 8):
                    self.set_guarded_contact_sensitivity('sensitivity_default')
        else:
            self.logger.error('Failed to start Omnibase')
        return success


    def stop(self):
        Device.stop(self)
        for i in range(3):
            self.wheels[i].stop()

    # ############## Control Modes #############################3


    def enable_freewheel_mode(self):
        """
        Force motors into freewheel
        """
        for w in self.wheels:
            w.enable_freewheel()

    def enable_pos_incr_mode(self):
        """
                Force motors into incremental position mode
        """
        for w in self.wheels:
            w.enable_pos_traj_incr()

    def enable_hold_mode(self):
        """
        Force motors into hold mode
         """
        for w in self.wheels:
            w.enable_hold()

    # ###################### Motion API #############################
    
    
    def set_curr_max_vel_xy_m(self, v=None):
        """
        Set the current maximum velocity in the x-y plane
        v: new maximum velocity (m/s)
        """
        if v==None:
            self.curr_max_vel_xy_m = self.params['motion']['max']['vel_xy_m'] #Reset
        else:
            self.curr_max_vel_xy_m = min(v,self.curr_max_vel_xy_m)
    
    def set_curr_max_vel_w_r(self, v=None):
        """
        Set the current maximum velocity in the angular direction
        v: new maximum velocity (rad/s)
        """
        if v==None:
            self.curr_max_vel_w_r = self.params['motion']['max']['vel_w_r'] #Reset
        else:
            self.curr_max_vel_w_r = min(v,self.curr_max_vel_w_r)     

    def set_curr_max_accel_xy_m(self, a=None):
        """
        Set the current maximum acceleration in the x-y plane
        a: new maximum acceleration (m/s^2)
        """
        if a==None:
            self.curr_max_accel_xy_m = self.params['motion']['max']['accel_xy_m'] #Reset
        else:
            self.curr_max_accel_xy_m = min(a,self.curr_max_accel_xy_m)     

    def set_curr_max_accel_w_r(self, a=None):
        """
        Set the current maximum acceleration in the angular direction
        a: new maximum acceleration (rad/s^2)
        """
        if a==None:
            self.curr_max_accel_w_r = self.params['motion']['max']['accel_w_r'] #Reset
        else:
            self.curr_max_accel_w_r = min(a,self.curr_max_accel_w_r)     



    # ###########
    def disable_guarded_mode(self):
        for w in self.wheels:
            w.disable_guarded_mode()
        self.guarded_mode_active=False

    def enable_guarded_mode(self):
        if self.params['enable_guarded_mode']:
            for w in self.wheels:
                w.enable_guarded_mode()
            self.guarded_mode_active=True
    
    def set_guarded_contact_sensitivity(self, mode_name=None):
        for w in self.wheels:
            if w.hw_valid and int(str(w.board_info['protocol_version'])[1:]) < 8:
                raise NotImplementedError('This method not supported for firmware on protocol {0}.'.format(w.board_info['protocol_version']))
        mode_name = mode_name or 'sensitivity_default'

        if mode_name == 'off':
            self.disable_guarded_mode()
            return

        for w in self.wheels:
            w.enable_guarded_mode()

        for w in self.wheels:
            c_sens_p = self.contact_sensitivity_pos if w.params['guarded_contact'][mode_name]['coeff_sensitivity_pos'] is None else w.params['guarded_contact'][mode_name]['coeff_sensitivity_pos']
            c_sens_n = self.contact_sensitivity_neg if w.params['guarded_contact'][mode_name]['coeff_sensitivity_neg'] is None else w.params['guarded_contact'][mode_name]['coeff_sensitivity_neg']
            self.contact_sensitivity_pos = c_sens_p
            self.contact_sensitivity_neg = c_sens_n
            w.set_guarded_contact_sensitivity(c_sens_p, c_sens_n)

    def set_velocity(self, translational_velocity_x, translational_velocity_y, angular_velocity_z, a_m=None,
                     a_r=None, contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        Velocity control of the base.
        translational_velocity_x: desired vel (m/s) along x-axis (forward axis)
        translational_velocity_y: desired vel (m/s) along y-axis (left of the robot)
        angular_velocity_z: desired vel (rad/s) along z-axis (right-handed frame formed by x-y axes)
        a_m: acceleration for trapezoidal motion profile (m/s^2) in direction of translation
        a_r: acceleration for trapezoidal motion profile (rad/s^2) in direction of rotation
        contact_sensitivity_pos: postive contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        contact_sensitivity_neg: negative contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        """

        # motion limits
        translational_velocity_x = np.clip(translational_velocity_x, -self.curr_max_vel_xy_m,
                                           self.curr_max_vel_xy_m)
        translational_velocity_y = np.clip(translational_velocity_y, -self.curr_max_vel_xy_m,
                                           self.curr_max_vel_xy_m)
        angular_velocity_z = np.clip(angular_velocity_z, -self.curr_max_vel_w_r,
                                     self.curr_max_vel_w_r)

        if a_m is not None:
            a_m = min(abs(a_m), self.curr_max_accel_xy_m)
        else:
            a_m = self.params['motion']['default']['accel_xy_m']

        if a_r is not None:
            a_r = min(abs(a_r), self.curr_max_accel_w_r)
        else:
            a_r = self.params['motion']['default']['accel_w_r']


        if self.lsvl is not None:
            #Limit velocity based on line sensor state
            heading=rad_to_deg(math.atan2(translational_velocity_y,translational_velocity_x))#Angle headed
            vl=self.lsvl.get_velocity_limit(heading)
            self.logger.debug(f'Limit of {vl} on heading {heading} given input {translational_velocity_y} {translational_velocity_x}')
            translational_velocity_x=translational_velocity_x*vl
            translational_velocity_y=translational_velocity_y*vl
            #angular_velocity_z=angular_velocity_z*vl #Allow full rotation speed at obstacle?
            if vl<1:
                a_m=self.curr_max_accel_xy_m*(1-vl) #max deccel if towards an obstacle
  

        a_m_wheel = (2 / self.params['wheel_diameter_m']) * (
                    a_m + self.params['base_radius_m'] * a_r)  # max wheel velocity

        # calculate the motor vels
        u = self.base_vel_to_motor_vel([translational_velocity_x, translational_velocity_y, angular_velocity_z])
        aa = self.compute_motor_acceleration(u, a_m_wheel)
        
        if contact_sensitivity_pos is None:
            contact_sensitivity_pos = self.contact_sensitivity_pos
        if contact_sensitivity_neg is None:
            contact_sensitivity_neg = self.contact_sensitivity_neg

        # push commands to the steppers
        ctrl_mode = Stepper.MODE_VEL_TRAJ if self.params['use_vel_traj'] else Stepper.MODE_VEL_PID
        if int(str(self.wheels[0].board_info['protocol_version'])[1:]) < 8:
            self.wheels[0].set_command(mode=ctrl_mode, v_des=u[0], a_des=abs(aa[0]))
        else:
            self.wheels[0].set_command(mode=ctrl_mode, v_des=u[0], a_des=abs(aa[0]), coeff_sensitivity_pos=contact_sensitivity_pos, coeff_sensitivity_neg=contact_sensitivity_neg)
        if int(str(self.wheels[1].board_info['protocol_version'])[1:]) < 8:
            self.wheels[1].set_command(mode=ctrl_mode, v_des=u[1], a_des=abs(aa[1]))
        else:
            self.wheels[1].set_command(mode=ctrl_mode, v_des=u[1], a_des=abs(aa[1]), coeff_sensitivity_pos=contact_sensitivity_pos, coeff_sensitivity_neg=contact_sensitivity_neg)            
        if int(str(self.wheels[2].board_info['protocol_version'])[1:]) < 8:
            self.wheels[2].set_command(mode=ctrl_mode, v_des=u[2], a_des=abs(aa[2]))
        else:
            self.wheels[2].set_command(mode=ctrl_mode, v_des=u[2], a_des=abs(aa[2]), coeff_sensitivity_pos=contact_sensitivity_pos, coeff_sensitivity_neg=contact_sensitivity_neg)

    def translate_by(self,x_m,y_m,v_m=None,a_m=None, contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        Incremental translation of the base
        x_m, y_m: desired motion (m)
        v_m: velocity for trapezoidal motion profile (m/s) in direction of translation
        a_m: acceleration for trapezoidal motion profile (m/s^2) in direction of translation
        X is forward (direction of arm), Y is right
        contact_sensitivity_pos: postive contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        contact_sensitivity_neg: negative contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        """
        self.move_by(x_m,y_m,0,v_m,a_m,0,0, contact_sensitivity_pos, contact_sensitivity_neg)
    
    def rotate_by(self,w_r,v_r=None,a_r=None, contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        Incremental rotation of the base
        w_r: rotation (rad)
        v_r: rotational velocity max (v_r)
        a_r: rotational acceleration (rad/s^2)
        contact_sensitivity_pos: postive contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        contact_sensitivity_neg: negative contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        """ 
        self.move_by(0,0,w_r,0,0,v_r,a_r, contact_sensitivity_pos, contact_sensitivity_neg)

    def wheel_move_to(self, wheel_name, x_rad, v_r=None, a_r=None, contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        Move a specific wheel to an absolute position.
        """
        wheel_idx = int(wheel_name.split('_')[-1])
        w = self.wheels[wheel_idx]
        v_r = abs(v_r) if v_r is not None else w.params['motion'].get('default', w.params['motion']).get('vel', 5.0)
        a_r = abs(a_r) if a_r is not None else w.params['motion'].get('default', w.params['motion']).get('accel', 5.0)

        if contact_sensitivity_pos is None:
            contact_sensitivity_pos = self.contact_sensitivity_pos
        if contact_sensitivity_neg is None:
            contact_sensitivity_neg = self.contact_sensitivity_neg

        ctrl_mode = Stepper.MODE_POS_TRAJ
        if int(str(w.board_info['protocol_version'])[1:]) < 8:
            w.set_command(mode=ctrl_mode, x_des=x_rad, v_des=abs(v_r), a_des=abs(a_r))
        else:
            w.set_command(mode=ctrl_mode, x_des=x_rad, v_des=abs(v_r), a_des=abs(a_r), coeff_sensitivity_pos=contact_sensitivity_pos, coeff_sensitivity_neg=contact_sensitivity_neg)

    def wheel_move_by(self, wheel_name, x_rad, v_r=None, a_r=None, contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        Move a specific wheel by a relative amount.
        """
        wheel_idx = int(wheel_name.split('_')[-1])
        w = self.wheels[wheel_idx]
        v_r = abs(v_r) if v_r is not None else w.params['motion'].get('default', w.params['motion']).get('vel', 5.0)
        a_r = abs(a_r) if a_r is not None else w.params['motion'].get('default', w.params['motion']).get('accel', 5.0)

        if contact_sensitivity_pos is None:
            contact_sensitivity_pos = self.contact_sensitivity_pos
        if contact_sensitivity_neg is None:
            contact_sensitivity_neg = self.contact_sensitivity_neg

        ctrl_mode = Stepper.MODE_POS_TRAJ_INCR
        if int(str(w.board_info['protocol_version'])[1:]) < 8:
            w.set_command(mode=ctrl_mode, x_des=x_rad, v_des=abs(v_r), a_des=abs(a_r))
        else:
            w.set_command(mode=ctrl_mode, x_des=x_rad, v_des=abs(v_r), a_des=abs(a_r), coeff_sensitivity_pos=contact_sensitivity_pos, coeff_sensitivity_neg=contact_sensitivity_neg)


    def move_by(self,x_m,y_m,w_r,v_m=None,a_m=None,v_r=None,a_r=None, contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        Incremental motion of the base
        X is forward (direction of arm), Y is right

        x_m, y_m: desired motion (m)
        w_r: desired rotation (rad)
        v_m: velocity for trapezoidal motion profile (m/s) in direction of translation
        a_m: acceleration for trapezoidal motion profile (m/s^2) in direction of translation
        v_r: rotation velocity for trapezoidal motion profile (rad/s)
        a_r: acceleration for trapezoidal motion profile (rad/s^2) in direction of rotation
        contact_sensitivity_pos: postive contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        contact_sensitivity_neg: negative contact sensitivity factor (Range: 0.0 (max) to 1.0 (min))
        """
        if (x_m != 0 or y_m !=0) and (w_r != 0):
            return # can't blend translational & rotational cmds, use set_velocity instead

        if (x_m != 0 or y_m != 0):
            if v_m is None:
                v_m=self.params['motion']['default']['vel_xy_m']
            if a_m is None:
                a_m=self.params['motion']['default']['accel_xy_m']
            v_m = min(abs(v_m), self.curr_max_vel_xy_m)
            a_m = min(abs(a_m), self.curr_max_accel_xy_m)
        else: 
            v_m = 0
            a_m = 0
        
        if w_r != 0:
            if v_r is None:
                v_r=self.params['motion']['default']['vel_w_r']
            if a_r is None:
                a_r=self.params['motion']['default']['accel_w_r']
            v_r = min(abs(v_r), self.curr_max_vel_w_r)
            a_r = min(abs(a_r), self.curr_max_accel_w_r)
        else:
            v_r = 0
            a_r = 0

        # if self.lsvl is not None:
        #     #Limit velocity based on line sensor state
        #     heading=rad_to_deg(math.atan2(x_m,y_m))#Angle headed
        #     vl=self.lsvl.get_velocity_limit(heading)
        #     print('Limit of %f on heading %f given input %f %f'%(vl,heading,x_m,y_m))
        #     v_m=v_m*vl
        #     #angular_velocity_z=angular_velocity_z*vl #Allow full rotation speed at obstacle?
        #     if vl<1:
        #         a_m=self.curr_max_accel_xy_m*(1-vl) #max deccel if towards an obstacle

        a_m_wheel = (2 / self.params['wheel_diameter_m']) * (
                    a_m + self.params['base_radius_m'] * a_r)


        theta=math.atan2(y_m,x_m)#Angle headed
        v_x = v_m*math.cos(theta)
        v_y = v_m*math.sin(theta)


        xx = self.base_vel_to_motor_vel([x_m, y_m, w_r])
        uu = self.base_vel_to_motor_vel([v_x, v_y, v_r])
        aa = self.compute_motor_acceleration(uu, a_m_wheel)
        #aa = self.base_vel_to_motor_vel([a_x, a_y, 0])

        if contact_sensitivity_pos is None:
            contact_sensitivity_pos = self.contact_sensitivity_pos
        if contact_sensitivity_neg is None:
            contact_sensitivity_neg = self.contact_sensitivity_neg


        if int(str(self.wheels[0].board_info['protocol_version'])[1:]) < 8:
            self.wheels[0].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=xx[0], v_des=abs(uu[0]), a_des=abs(aa[0]))
        else:
            self.wheels[0].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=xx[0], v_des=abs(uu[0]), a_des=abs(aa[0]), coeff_sensitivity_pos=contact_sensitivity_pos, coeff_sensitivity_neg=contact_sensitivity_neg)
        if int(str(self.wheels[1].board_info['protocol_version'])[1:]) < 8:
            self.wheels[1].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=xx[1], v_des=abs(uu[1]), a_des=abs(aa[1]))
        else:
            self.wheels[1].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=xx[1], v_des=abs(uu[1]), a_des=abs(aa[1]), coeff_sensitivity_pos=contact_sensitivity_pos, coeff_sensitivity_neg=contact_sensitivity_neg)            
        if int(str(self.wheels[2].board_info['protocol_version'])[1:]) < 8:
            self.wheels[2].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=xx[2], v_des=abs(uu[2]), a_des=abs(aa[2]))
        else:
            self.wheels[2].set_command(mode=Stepper.MODE_POS_TRAJ_INCR,x_des=xx[2], v_des=abs(uu[2]), a_des=abs(aa[2]), coeff_sensitivity_pos=contact_sensitivity_pos, coeff_sensitivity_neg=contact_sensitivity_neg)        

    def hard_stop(self):
        """Come to a stop as quickly as possible.
        """
        u = np.array([0.0, 0.0, 0.0])
        stop_accel = 20

        aa = self.compute_motor_acceleration(u, stop_accel)

        ctrl_mode = Stepper.MODE_VEL_TRAJ if self.params['use_vel_traj'] else Stepper.MODE_VEL_PID
        self.wheels[0].set_command(mode=ctrl_mode, v_des=u[0], a_des=abs(aa[0]))
        self.wheels[1].set_command(mode=ctrl_mode, v_des=u[1], a_des=abs(aa[1]))
        self.wheels[2].set_command(mode=ctrl_mode, v_des=u[2], a_des=abs(aa[2]))




    # ############## Utility #############################3
    def pause_transport(self):
        for i in range(3):
            self.wheels[i].pause_transport()

    def unpause_transport(self):
        for i in range(3):
            self.wheels[i].unpause_transport()
    def pretty_print(self):
        print('----------Base------')
        print('X (m)',self.status['x'])
        print('Y (m)',self.status['y'])
        print('Theta (rad)',self.status['theta'])
        print('X_vel (m/s)', self.status['x_vel'])
        print('Y_vel (m/s)', self.status['y_vel'])
        print('Theta_vel (rad/s)', self.status['theta_vel'])
        print('Pose time (s)', self.status['pose_time_s'])
        print('Timestamp PC (s):', self.status['timestamp_pc'])
        print('-----Omni0-----')
        self.wheels[0].pretty_print()
        print('-----Omni1-----')
        self.wheels[1].pretty_print()
        print('-----Omni2-----')
        self.wheels[2].pretty_print()


    def wait_while_is_moving(self,timeout=15.0, use_motion_generator=True):
        done = []
        def check_wait(wait_method):
            done.append(wait_method(timeout,use_motion_generator))
        threads = []
        for w in self.wheels:
            threads.append(threading.Thread(target=check_wait, args=(w.wait_while_is_moving,)))
        [thread.start() for thread in threads]
        [thread.join() for thread in threads]
        return all(done)

    def wait_until_at_setpoint(self, timeout=15.0):
        #Assume all are in motion. This will exit once all are at setpoints
        at_setpoint = []
        def check_wait(wait_method):
            at_setpoint.append(wait_method(timeout))
        threads = []
        for w in self.wheels:
            threads.append(threading.Thread(target=check_wait, args=(w.wait_while_is_moving,)))
        [done_thread.start() for done_thread in threads]
        [done_thread.join() for done_thread in threads]
        return all(at_setpoint)


    def push_command(self,blocking=True):
        success=True
        for w in self.wheels:
            success=success and w.push_command(blocking)
        return success


    def pull_status(self, blocking=True):
        success=True
        for w in self.wheels:
            success=success and w.pull_status(blocking)
            # if w.status['in_guarded_event']:
            #     self.enable_freewheel_mode()
            #     self.push_command()
        if success:
            self._update_odom(None)
        return success

    def load_rpc_results(self, wait_on_result=True):
        success = True
        for w in self.wheels:
            success = success and w.load_rpc_results(wait_on_result)
        return success

    def enable_rate_logging(self,max_samples=1000):
        for w in self.wheels:
            w.enable_rate_logging(max_samples)

    def get_rate_log(self):
        log={}
        for w in self.wheels:
            log[w.name]=w.get_rate_log()
        return log

    def _update_odom(self, dt):
        """
        Calculate SE2 position of the base in odom frame from wheel odometry
        Important:
         - Assumes pull_status() was just called
         - Assumes the user calls this method at a regular frequency (>30hz)

        dt: 1/frequency at which this method is called
        """
        wheel_speeds = np.array([self.wheels[0].status['vel'],
                                 self.wheels[1].status['vel'],
                                 self.wheels[2].status['vel']])
        Vb = self.H0_inv @ (wheel_speeds/self.params['gr'])
        self.status['x_vel'] = float(Vb[0])
        self.status['y_vel'] = float(Vb[1])
        self.status['theta_vel'] = float(Vb[2])

        if dt is None:
            if self.status['pose_time_s'] is None:
                #print("pose_time_s is NONE")
                self.status['pose_time_s'] = time.time()

            dt = time.time() - self.status['pose_time_s']

        Sb = Vb*dt
        Sb = rotation_3x3_matrix(self.status['theta']) @ Sb
        self.status['x'] += float(Sb[0])
        self.status['y'] += float(Sb[1])
        self.status['theta'] += float(Sb[2])
        self.status['pose_time_s'] = time.time()
    
    def get_odom(self):
        """
        Returns the current odometry of the base
        """
        state = { 'x': self.status['x'],
                  'y':self.status['y'],
                  'theta': self.status['theta'],
                  'time': self.status['pose_time_s']}
        return state

    # ##################### Sentries    ##############################

    def step_sentry(self,robot_status):

        self._sentry_fast_motion_allowed_on_stow(robot_status)
        for w in self.wheels:
            w.step_sentry(robot_status)


    def _sentry_fast_motion_allowed_on_stow(self,robot_status):
        """
        Only allow fast mobile base motion if the lift is low,
        the arm is retracted, and the wrist is stowed. This is
        intended to keep the center of mass low for increased
        stability and avoid catching the arm or tool on
        something.
        """
        if self.robot_params['robot_sentry']['omnibase']['fast_motion_allowed_on_stow']:
            print('TODO _sentry_fast_motion_allowed_on_stow')
            # if 'lift' in robot_status and 'arm'  and 'end_of_arm' in robot_status:
            #     x_lift=robot_status['lift']['pos']
            #     x_arm =robot_status['arm']['pos']
            #     x_wrist =robot.end_of_arm.motors['wrist_yaw'].status['pos']
            #
            #     if ((x_lift < self.params['sentry_fast_motion_allowed_on_stow']['max_lift_height_m']) and
            #             (x_arm < self.params['sentry_fast_motion_allowed_on_stow']['max_arm_extension_m']) and
            #             (x_wrist > self.params['sentry_fast_motion_allowed_on_stow']['min_wrist_yaw_rad'])):
            #         if not self.fast_motion_allowed:
            #             self.logger.debug('Fast motion turned on')
            #         self.fast_motion_allowed = True
            #     else:
            #         if self.fast_motion_allowed:
            #             self.logger.debug('Fast motion turned off')
            #         self.fast_motion_allowed = False


    # ############## Transforms #####################################
    
    def base_vel_to_motor_vel(self,v):
        #Convert base velocities to motor velocities
        Vb = np.array(v)
        u_w = self.H0 @ Vb #  wheel target velocites
        u = u_w * self.params['gr'] # motor target velocities
        return u

    def motor_vel_to_base_vel(self,u):
        u_w = u / self.params['gr']
        Vb = self.H0_inv @ u_w
        return Vb

    def compute_motor_acceleration(self, u_target, a_m_wheel):
        """
        Take target velocities (motor frame) (rad/s) and a desired acceleration
        Return the accelerations (motor frame) that achieve the target
        at the same time in the future.
        """
        u_target_w=u_target/self.params['gr']
        return self.compute_wheel_acceleration(u_target_w,a_m_wheel)

    def compute_wheel_acceleration(self, u_target_w, a_m_wheel):
        """
        Take target velocities (wheel frame) (rad/s) and a desired acceleration
        Return the accelerations (motor frame) that achieve the target
        at the same time in the future.
        """
        wheel_speeds = np.array([self.wheels[0].status['vel'],
                                 self.wheels[1].status['vel'],
                                 self.wheels[2].status['vel']])  # current motor velocities

        u_current_w = wheel_speeds / self.params['gr']  # current wheel velocities

        delta_u = u_target_w - u_current_w
        max_delta = np.max(np.abs(delta_u))

        if max_delta == 0:
            accel = np.zeros_like(delta_u)
        else:
            # Avoid divide by zero by multiplying instead of dividing
            accel = (delta_u / max_delta) * a_m_wheel

        return accel * self.params["gr"]  # Convert to motor velocities

    def is_sync_required(self,ts_last_motor_sync):
        return (self.wheels[0].is_sync_required(ts_last_motor_sync) or
                self.wheels[1].is_sync_required(ts_last_motor_sync) or
                self.wheels[2].is_sync_required(ts_last_motor_sync))

    def is_homed(self):
        return True


class OmnibaseStatus(TypedDict):
    timestamp_pc: float
    x:float
    y:float
    theta: float
    x_vel: float
    y_vel:float
    theta_vel:float
    pose_time_s :float
    effort: tuple[float,float,float] 
    wheel_0: StepperStatus
    wheel_1: StepperStatus
    wheel_2: StepperStatus


if __name__ == "__main__":
    o=OmniBase()
    ina=[22,-12,41]
    print('IN',ina)
    ww=o.base_vel_to_motor_vel(ina)
    vv=o.motor_vel_to_base_vel(ww)
    print('W',ww)
    print('V',vv)
