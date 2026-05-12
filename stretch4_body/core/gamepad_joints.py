from stretch4_body.core.hello_utils import *
from stretch4_body.core.robot_params import RobotParams

"""
The gamepad_joints library provides the abstract motion command classes 
for each robot joint that can be used in a control loop to make a motion through an gamepad 
type inputs elements (Button presses, Analog Stick motions).

The Gamepad joints command classes primarily uses velocity controls. All the 
acceleration profiles are dynamically optimized based on the user input type to 
provide smooth and responsive robot motions.

A gamepad joint command class will provide the below four main attributs 
to convert a gamepad input to an appropriate motion:

command_stick_to_motion()
    Supply a float value between -1.0 to 1.0 from a control loop. 
    The value supplied and it's sign determines the speed of joint motion and direction
    Use this method to map values from an analog UI elements to a joint motion.
    Note the base motion class needs an aditional y axis value / x,y axis values for  linear,rotion motion.

command_button_to_motion()
    Supply a direction integere either +1 or -1 in a control loop for the joint to move in that direction
    Use this method to map a boolean button state UI elements to a joint motion.
    
stop_motion()
    Use this method when ever a joints needs to be still with no motion in a control loop.

precision_mode
    Set this flag to true or false to enable and disable precision mode for each joint.

fast_base_mode
    This flag is specific to base command class. Set this flag True to move the base
    faster but only allowed when the arm and lift are in safe positions to avoid tipping.
"""


class CommandBase:
    def __init__(self, motion_profile:str = 'default', motion_profile_angular:str = 'slow'):
        self.motion_profile = motion_profile
        self.motion_profile_angular = motion_profile_angular
        self.params = RobotParams().get_params()[1]['omnibase']
        self.dead_zone = 0.0001
        self._prev_set_vel_ts = time.time()
        self.accel_xy = self.params['motion'][self.motion_profile]['accel_xy_m']*0.25
        self.vel_xy = self.params['motion'][self.motion_profile]['vel_xy_m']

        self.accel_w_for_translation = self.params['motion'][self.motion_profile]['accel_w_r']
        self.vel_w_for_translation = self.params['motion'][self.motion_profile]['vel_w_r']
        self.accel_w_for_rotation_only = self.params['motion'][self.motion_profile_angular]['accel_w_r']
        self.vel_w_for_rotation_only = self.params['motion'][self.motion_profile_angular]['vel_w_r']

        # self.max_linear_vel = self.params['motion']['max']['vel_m']
        # self.max_rotation_vel = 1.90241 # rad/s
        # self.normal_linear_vel = self.params['motion'][self.motion_profile]['vel_m']
        # self.normal_rotation_vel = self.max_rotation_vel*0.4
        #self.arm_extended_rotation_vel = self.max_rotation_vel*0.2
        self.precision_mode = False
        # self.fast_base_mode = False
        # self.acc = self.params['motion']['max']['accel_m']
        # self.start_pos = None
        #
        # # Precision mode params
        # self.precision_max_linear_vel = 0.05 #0.02 m/s Very precise: 0.01
        # self.precision_max_rot_vel = 0.3 #0.08 rad/s Very precise: 0.04
    
    def command_stick_to_motion(self, x, y, w,robot):
        """Convert a stick axis value to robot base's tank driving motion.

        Args:
            x (float): Range [-1.0,+1.0], control rotation speed
            y (float): Range [-1.0,+1.0], control linear speed
            robot (robot.Robot): Valid robot instance
        """
        #print('BASE',x,y,w)
        # if controller_state['right_trigger_pulled']>0.5: #slow mode## BM - scale = 0.25
        #     scale=0.5
        #     accel_xy=accel_xy*scale
        #     vel_xy=vel_xy*scale
        #     accel_w = accel_w * scale
        #     vel_w = vel_w * scale

        v_x=self.vel_xy*(0 if abs(x)<self.dead_zone else x)
        v_y=self.vel_xy*(0 if abs(y)<self.dead_zone else y)
        v_w=self.vel_w_for_rotation_only*(0 if abs(w)<self.dead_zone else w)

        accel_w = self.accel_w_for_translation if v_w == 0 else self.accel_w_for_rotation_only

        if not self.precision_mode:
            robot.base.set_velocity(v_x,v_y,v_w,self.accel_xy, accel_w)
        else:
            kk=0.25
            robot.base.set_velocity(kk*v_x, kk*v_y, kk*v_w, self.accel_xy, accel_w)
        #robot.base.pretty_print()
        # if abs(x) < self.dead_zone:
        #     x = 0
        # if abs(y) < self.dead_zone:
        #     y = 0
        #
        # x = to_parabola_transform(x)
        # # y = to_parabola_transform(y)
        #
        # # Standard Mode
        # if not self.precision_mode:
        #     self.start_pos = None
        #     self.start_theta = None
        #     v_m, w_r = self._process_stick_to_vel(x,y, robot)
        #     robot.base.set_velocity(v_m, w_r, a=self.acc)
        #     self._prev_set_vel_ts = time.time()
        # else:
        # # Precision Mode
        #     if self.start_pos is None:
        #         self.start_pos = [robot.base.status['x'],robot.base.status['y']]
        #         self.start_theta = robot.base.status['theta']
        #         self.target_position = self.start_pos
        #         self.target_theta = self.start_theta
        #     yv = map_to_range(abs(y),0,self.precision_max_linear_vel)
        #     xv = map_to_range(abs(x),0,self.precision_max_rot_vel)
        #     if x<0:
        #         xv = -1*xv
        #     if y<0:
        #         yv = -1*yv
        #     if abs(x)>abs(y):
        #         self._step_precision_rotate(xv, robot)
        #     else:
        #         self._step_precision_translate(yv, robot)
        #     # Update the previous_time for the next iteration
        #     self._prev_set_vel_ts = time.time()
    


    
    def stop_motion(self, robot):
        """Stop the joint motion. To be used when ever the controller is idle/no-inputs
        to stop unnecessary robot motion.

        Args:
            robot (robot.Robot): Valid robot instance
        """
        robot.base.set_velocity(0, 0, 0, self.accel_xy, self.accel_w_for_rotation_only)


    def is_fastbase_safe(self, robot):
        """Check if the base is fast navigation mode safe

        Args:
            robot (robot.Robot): Valid robot instance

        Returns:
            bool: True if fast navigation safe
        """
        arm = robot.arm.status['pos'] < (robot.get_stow_pos('arm') + 0.1) # check if arm pos under stow pose + 0.1 m
        lift = robot.lift.status['pos'] < (robot.get_stow_pos('lift') + 0.05) # check if lift pos is under stow pose + 0.05m
        return arm and lift

    def _process_stick_to_vel(self, x, y, robot):
        max_linear_vel = self.normal_linear_vel
        max_rotation_vel = self.normal_rotation_vel
        if self.fast_base_mode and self.is_fastbase_safe(robot):
            max_linear_vel =  self.max_linear_vel
            max_rotation_vel = self.max_rotation_vel
        if not robot.arm.status['pos'] < (robot.get_stow_pos('arm') + 0.1):
            max_rotation_vel = self.arm_extended_rotation_vel
        v_m = map_to_range(abs(y), 0, max_linear_vel)
        if y<0:
            v_m = -1*v_m
        x = -1*x
        w_r = map_to_range(abs(x), 0, max_rotation_vel)
        if x<0:
            w_r = -1*w_r
        return v_m, w_r
            
    def _step_precision_rotate(self, xv, robot):
        # Calculate the time elapsed since the last iteration
        current_time = time.time()
        elapsed_time = current_time - self._prev_set_vel_ts 

        # Calculate the desired change in position to achieve the desired velocity
        desired_theta_change = xv * elapsed_time

        # Set the control_effort as the position setpoint for the joint
        if abs(xv)>=0:
            robot.base.rotate_by(-1*desired_theta_change)

    def _step_precision_translate(self, yv, robot):
        
        # Calculate the time elapsed since the last iteration
        current_time = time.time()
        elapsed_time = current_time - self._prev_set_vel_ts 

        # Calculate the desired change in position to achieve the desired velocity
        desired_position_change = yv * elapsed_time

        # Set the control_effort as the position setpoint for the joint
        if abs(yv)>=0:
            robot.base.translate_by(desired_position_change)
            
class CommandLift:
    def __init__(self, motion_profile:str = 'default'):
        self.motion_profile = motion_profile
        self.params = RobotParams().get_params()[1]['lift']
        self.dead_zone = 0.0001
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.params['motion'][self.motion_profile]['vel_m']
        self.precision_mode = False
        self.acc = self.params['motion']['max']['accel_m']
        
        # Precision mode params
        self.start_pos = None
        self.target_position = self.start_pos
        self.precision_kp = 0.5 # Very Precise: 0.5
        self.precision_max_vel = 0.04 # m/s Very Precise: 0.02 m/s
        self.stopped_for_precision = False 
    
    def command_stick_to_motion(self, x, robot):
        """Convert a stick axis value to robot lift motion.

        Args:
            x (float): Range [-1.0,+1.0], control lift speed
            robot (robot.Robot): Valid robot instance
        """
        if abs(x) < self.dead_zone:
            x = 0
        # x = to_parabola_transform(x)
        #print('Vel %f Current %f Effort %f'%(robot.lift.status['vel'],robot.lift.motor.status['current'],robot.lift.motor.status['effort_pct']))
        # Standard Mode

        # self.start_pos = None
        # self.stopped_for_precision = False
        v_m = self._process_stick_to_vel(x)
        # self._prev_set_vel_ts = time.time()
        if not self.precision_mode:
            robot.lift.set_velocity(v_m, a_m=self.acc)
            # print(f"[CommandLift]  X: {x} || v_m: {self.safety_v_m}")
        else:
            kk=0.25
            robot.lift.set_velocity(v_m*kk, a_m=self.acc)
        # Precision Mode
        #     if abs(robot.lift.status['vel'])>0.001 and not self.stopped_for_precision:
        #         # wait till joint stops before recording the start pos
        #         self.stop_motion(robot)
        #     else:
        #         self.stopped_for_precision = True
        #         if self.start_pos is None:
        #             self.start_pos = robot.lift.status['pos']
        #             self.target_position = self.start_pos
        #
        #     r = self.precision_max_vel
        #     xv = map_to_range(abs(x),0,r)
        #     if x<0:
        #         xv = -1*xv
        #     if self.stopped_for_precision:
        #         self._step_precision_move(xv, robot)
    
    def command_button_to_motion(self, direction, robot):
        """Make lift move based on a button state.

        Args:
            direction (int): Direction integer -1 or +1
            robot (robot.Robot): Valid robot instance
        """
        v_m = direction*self.max_linear_vel
        if self.precision_mode:
            v_m = direction*self.precision_max_vel
        robot.lift.set_velocity(v_m, a_m=self.params['motion'][self.motion_profile]['accel_m'])
    
    def stop_motion(self, robot):
        """Stop the joint motion. To be used when ever the controller is idle/no-inputs
        to stop unnecessary robot motion.

        Args:
            robot (robot.Robot): Valid robot instance
        """
        robot.lift.set_velocity(0, a_m=self.params['motion']['max']['accel_m'])

    def _process_stick_to_vel(self, x):
        v_m = map_to_range(abs(x), 0, self.max_linear_vel)
        if x<0:
            v_m = -1*v_m
        return v_m
    
    def _step_precision_move(self,xv, robot):
        # Read the current joint position
        current_position = robot.lift.status['pos']

        # Calculate the time elapsed since the last iteration
        current_time = time.time()
        elapsed_time = current_time - self._prev_set_vel_ts 

        # Calculate the desired change in position to achieve the desired velocity
        desired_position_change = xv * elapsed_time
        
        # Update the target position based on the desired position change
        self.target_position = self.target_position + desired_position_change

        # Calculate the position error
        position_error = self.target_position - current_position

        # Calculate the control effort (position control)
        x_des = self.precision_kp * position_error

        # Set the control_effort as the position setpoint for the joint
        robot.lift.move_to(self.start_pos + x_des)

        # Update the previous_time for the next iteration
        self._prev_set_vel_ts = time.time()

class CommandArm:
    def __init__(self, motion_profile:str = 'default'):
        self.motion_profile = motion_profile
        self.params = RobotParams().get_params()[1]['arm']
        self.dead_zone = 0.0001
        self._prev_set_vel_ts = None
        self.max_linear_vel = self.params['motion'][self.motion_profile]['vel_m']*0.75
        self.precision_mode = False
        self.acc = self.params['motion']['max']['accel_m']
        
        # Precision mode params
        self.start_pos = None
        self.target_position = self.start_pos
        self.precision_kp = 0.6 # Very Precise: 0.6
        self.precision_max_vel = 0.04 # m/s  Very Precise: 0.02 m/s
        self.stopped_for_precision = False

    def command_stick_to_motion(self, x, robot):
        """Convert a stick axis value to robot arm motion.

        Args:
            x (float): Range [-1.0,+1.0], control lift speed
            robot (robot.Robot): Valid robot instance
        """

        if abs(x) < self.dead_zone:
            x = 0
        # x = to_parabola_transform(x)
        v_m = self._process_stick_to_vel(x)
        if not self.precision_mode:
        # Standard Mode
        #     self.start_pos = None
        #     self.stopped_for_precision = False
            robot.arm.set_velocity(v_m,a_m=self.acc)
            #self._prev_set_vel_ts = time.time()
        else:
            kk=0.25
            robot.arm.set_velocity(v_m*kk, a_m=self.acc)
        # # Precision Mode
        #     if abs(robot.arm.status['vel'])>0.001 and not self.stopped_for_precision:
        #         # wait till joint stops before recording the start pos
        #         self.stop_motion(robot)
        #     else:
        #         self.stopped_for_precision = True
        #         if self.start_pos is None:
        #             self.start_pos = robot.arm.status['pos']
        #             self.target_position = self.start_pos
        #
        #     r = self.precision_max_vel
        #     xv = map_to_range(abs(x),0,r)
        #     if x<0:
        #         xv = -1*xv
        #     if self.stopped_for_precision:
        #         self._step_precision_move(xv, robot)

    def command_button_to_motion(self, direction, robot):
        """Make arm move based on a button state.

        Args:
            direction (int): Direction integer -1 or +1
            robot (robot.Robot): Valid robot instance
        """
        v_m = direction*self.max_linear_vel
        if self.precision_mode:
            v_m = direction*self.precision_max_vel
        robot.arm.set_velocity(v_m, a_m=self.params['motion'][self.motion_profile]['accel_m'])

    def stop_motion(self, robot):
        """Stop the joint motion. To be used when ever the controller is idle/no-inputs
        to stop unnecessary robot motion.

        Args:
            robot (robot.Robot): Valid robot instance
        """
        robot.arm.set_velocity(0, a_m=self.params['motion']['max']['accel_m'])

    def _process_stick_to_vel(self, x):
        v_m = map_to_range(abs(x), 0, self.max_linear_vel)
        if x<0:
            v_m = -1*v_m
        return v_m
    
    def _step_precision_move(self,xv, robot):
        # Read the current joint position
        current_position = robot.arm.status['pos']

        # Calculate the time elapsed since the last iteration
        current_time = time.time()
        elapsed_time = current_time - self._prev_set_vel_ts 

        # Calculate the desired change in position to achieve the desired velocity
        desired_position_change = xv * elapsed_time
        
        # Update the target position based on the desired position change
        self.target_position = self.target_position + desired_position_change

        # Calculate the position error
        position_error = self.target_position - current_position

        # Calculate the control effort (position control)
        x_des = self.precision_kp * position_error

        # Set the control_effort as the position setpoint for the joint
        if abs(xv)>=0:
            robot.arm.move_to(self.start_pos + x_des)

        # Update the previous_time for the next iteration
        self._prev_set_vel_ts = time.time()

# class CommandEOAJoint:
#     """Abstract motion command class for EndOfArm joints
#     """
#     def __init__(self, name, max_vel=None, acc_type=None):
#         """Initiate a  joint
#
#         Args:
#             name (str): Name of the device name
#             max_vel (float, optional): Set a custom max velocity (rad/s)
#             acc_type (str, optional): Set custom acceleration profile (fast,slow,default)
#         """
#         self.params = RobotParams().get_params()[1][name]
#         self.name = name
#         self.dead_zone = 0.001
#         self._prev_set_vel_ts = None
#         self.max_vel = max_vel if max_vel else self.params['motion']['default']['vel']
#         self.precision_mode = False
#         self.acc = None
#         if acc_type:
#             self.acc = self.params['motion'][acc_type]['accel']
#
#         self.precision_scale_down = 0.05
#
#     def command_stick_to_motion(self, x, robot):
#         """Convert a stick axis value to dynamixel servo motion.
#
#         Args:
#             x (float): Range [-1.0,+1.0], control servo speed
#             robot (robot.Robot): Valid robot instance
#         """
#         if abs(x)<self.dead_zone:
#             x = 0
#         acc = self.acc
#         if x==0:
#             acc = self.params['motion']['max']['accel'] #Stop with Strong Acceleration
#         # x = to_parabola_transform(x)
#         v = self._process_stick_to_vel(x)
#         if self.precision_mode:
#             v = v*self.precision_scale_down
#         robot.end_of_arm.set_velocity(self.name,v, acc)
#         self._prev_set_vel_ts = time.time()
#
#     def command_button_to_motion(self,direction, robot):
#         """Make servo move based on a button state.
#
#         Args:
#             direction (int): Direction integer -1 or +1
#             robot (robot.Robot): Valid robot instance
#         """
#         vel = self.max_vel
#         if self.precision_mode:
#             vel = vel*self.precision_scale_down
#         if direction==1:
#             robot.end_of_arm.set_velocity(self.name, vel, self.acc)
#         elif direction==-1:
#             robot.end_of_arm.set_velocity(self.name, -1*vel, self.acc)
#         self._prev_set_vel_ts = time.time()
#
#     def stop_motion(self, robot):
#         """Stop the joint motion. To be used when ever the controller is idle/no-inputs
#         to stop unnecessary robot motion.
#
#         Args:
#             robot (robot.Robot): Valid robot instance
#         """
#         robot.end_of_arm.set_velocity(self.name,0,self.params['motion']['max']['accel'])
#
#     def _process_stick_to_vel(self, x):
#         x = -1*x
#         v = map_to_range(abs(x), 0, self.max_vel)
#         if x<0:
#             v = -1*v
#         return v

class CommandFeetechJoint:
    """Abstract motion command class for Feetech joints
    """

    def __init__(self, name, dx_deg,vel_type, acc_type):
        """Initiate a  joint

        Args:
            name (str): Name of the device name
            max_vel (float, optional): Set a custom max velocity (rad/s)
            acc_type (str, optional): Set custom acceleration profile (fast,slow,default)
        """
        self.params = RobotParams().get_params()[1][name]
        self.name = name
        self.dead_zone = 0.001
        self._prev_set_vel_ts = None
        self.dx_deg=dx_deg
        self.max_vel = self.params['motion'][vel_type]['vel']
        self.acc = self.params['motion'][acc_type]['accel']
        self.precision_mode = False
        self.precision_scale_down = 0.05

    # def command_stick_to_motion(self, x, robot):
    #     """Convert a stick axis value to dynamixel servo motion.
    #
    #     Args:
    #         x (float): Range [-1.0,+1.0], control servo speed
    #         robot (robot.Robot): Valid robot instance
    #     """
    #     if abs(x) < self.dead_zone:
    #         x = 0
    #     acc = self.acc
    #     if x == 0:
    #         acc = self.params['motion']['max']['accel']  # Stop with Strong Acceleration
    #     # x = to_parabola_transform(x)
    #     v = self._process_stick_to_vel(x)
    #     if self.precision_mode:
    #         v = v * self.precision_scale_down
    #     robot.end_of_arm.set_velocity(self.name, v, acc)
    #     self._prev_set_vel_ts = time.time()

    # def command_button_to_motion(self, direction, robot):
    #     """Make servo move based on a button state.
    #
    #     Args:
    #         direction (int): Direction integer -1 or +1
    #         robot (robot.Robot): Valid robot instance
    #     """
    #     vel = self.max_vel
    #     if self.precision_mode:
    #         vel = vel * self.precision_scale_down
    #     if direction == 1:
    #         robot.end_of_arm.set_velocity(self.name, vel, self.acc)
    #     elif direction == -1:
    #         robot.end_of_arm.set_velocity(self.name, -1 * vel, self.acc)
    #     self._prev_set_vel_ts = time.time()
    def command_button_to_motion(self, direction, robot):
        """Make servo move based on a button state.

        Args:
            direction (int): Direction integer -1 or +1
            robot (robot.Robot): Valid robot instance
        """
        dx_deg=self.dx_deg
        if self.precision_mode:
            dx_deg = self.dx_deg * self.precision_scale_down
        if direction == 1:
            robot.end_of_arm.move_by(self.name, deg_to_rad(dx_deg),self.max_vel, self.acc)
        elif direction == -1:
            robot.end_of_arm.move_by(self.name, -1 * deg_to_rad(dx_deg),self.max_vel,self.acc)
        self._prev_set_vel_ts = time.time()

    def command_stick_to_motion(self, x, robot):
        """Convert a stick axis value to robot arm motion.

        Args:
            x (float): Range [-1.0,+1.0], control lift speed
            robot (robot.Robot): Valid robot instance
        """
        if abs(x) < self.dead_zone:
            x = 0
        
        dx_deg=self.dx_deg * x  # scale the input to the joint motion

        if self.precision_mode:
            dx_deg *= self.precision_scale_down

        robot.end_of_arm.move_by(self.name, deg_to_rad(dx_deg),self.max_vel, self.acc)

        self._prev_set_vel_ts = time.time()
        
    def stop_motion(self, robot):
        """Stop the joint motion. To be used when ever the controller is idle/no-inputs
        to stop unnecessary robot motion.

        Args:
            robot (robot.Robot): Valid robot instance
        """
        robot.end_of_arm.move_by(self.name, 0)
        #robot.end_of_arm.set_velocity(self.name, 0, self.params['motion']['max']['accel'])

    def _process_stick_to_vel(self, x):
        x = -1 * x
        v = map_to_range(abs(x), 0, self.max_vel)
        if x < 0:
            v = -1 * v
        return v


class CommandWristYaw(CommandFeetechJoint):
    """Wrist Yaw motion command class for Dynamixel joints
    """
    # def __init__(self, name='wrist_yaw', dx_deg=15.0,vel_type='max', acc_type='slow'):
    def __init__(self, name='wrist_yaw',  dx_deg=15.0, motion_profile:str = 'default'):
        # super().__init__(name,dx_deg, vel_type, acc_type)
        super().__init__(name, dx_deg, motion_profile, motion_profile)

class CommandWristPitch(CommandFeetechJoint):
    """Wrist Pitch motion command class for Dynamixel joints
    """
    # def __init__(self, name='wrist_pitch', dx_deg=15.0,vel_type='max', acc_type='slow'):
    def __init__(self, name='wrist_pitch',  dx_deg=15.0, motion_profile:str = 'default'):
        # super().__init__(name,dx_deg, vel_type, acc_type)
        super().__init__(name, dx_deg, motion_profile, motion_profile)

class CommandWristRoll(CommandFeetechJoint):
    """Wrist Roll motion command class for Dynamixel joints
    """
    # def __init__(self, name='wrist_roll', dx_deg=15.0,vel_type='max', acc_type='slow'):
    def __init__(self, name='wrist_roll',  dx_deg=15.0, motion_profile:str = 'default'):
        # super().__init__(name, dx_deg,vel_type, acc_type)
        super().__init__(name, dx_deg, motion_profile, motion_profile)

            
class CommandStretchGripperPosition:
    """Stretch Gripper motion command class for Dynamixel joints
    For this class only simple open and close methods are provided
    and expected only to be controlled on a button state.
    """
    def __init__(self, motion_profile:str = 'max'):
        self.name = 'stretch_gripper'
        self.params = RobotParams().get_params()[1][self.name]
        self.gripper_rotate_pct = 60.0
        self.gripper_accel = self.params['motion'][motion_profile]['accel']
        self.gripper_vel = self.params['motion'][motion_profile]['vel']
        self.precision_mode = False
        self.stop_reqd = False
    
    def open_gripper(self, robot):
        pct = self.gripper_rotate_pct
        if self.precision_mode:
            pct = self.gripper_rotate_pct * 0.05
        robot.end_of_arm.move_by(self.name, pct, self.gripper_vel, self.gripper_accel)
        self.stop_reqd = True
        
    def close_gripper(self, robot):
        pct = self.gripper_rotate_pct
        if self.precision_mode:
            pct = self.gripper_rotate_pct * 0.05
        robot.end_of_arm.move_by(self.name, -pct, self.gripper_vel, self.gripper_accel)
        self.stop_reqd = True

    def stop_gripper(self, robot):
        if self.stop_reqd:
            robot.end_of_arm.quick_stop(self.name)
            self.stop_reqd = False

class CommandParallelGripperPosition:
    """Parallel Gripper motion command class for Feetech joints
    For this class only simple open and close methods are provided
    and expected only to be controlled on a button state.
    """
    def __init__(self, motion_profile:str = 'max'):
        self.name = 'parallel_gripper'
        self.params = RobotParams().get_params()[1][self.name]
        self.gripper_rotate_deg = 15.0
        self.gripper_accel = self.params['motion'][motion_profile]['accel']
        self.gripper_vel = self.params['motion'][motion_profile]['vel']
        self.precision_mode = False
        self.stop_reqd = False
    
    def open_gripper(self, robot):
        dx_deg = self.gripper_rotate_deg
        if self.precision_mode:
            dx_deg = self.gripper_rotate_deg * 0.05
        robot.end_of_arm.move_by(self.name, deg_to_rad(dx_deg), self.gripper_vel, self.gripper_accel)
        self.stop_reqd = True
        
    def close_gripper(self, robot):
        dx_deg = self.gripper_rotate_deg
        if self.precision_mode:
            dx_deg = self.gripper_rotate_deg * 0.05
        robot.end_of_arm.move_by(self.name, -deg_to_rad(dx_deg), self.gripper_vel, self.gripper_accel)
        self.stop_reqd = True

    def stop_gripper(self, robot):
        if self.stop_reqd:
            robot.end_of_arm.move_by(self.name, 0)
            self.stop_reqd = False

