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

"""


class CommandBase:
    def __init__(self, motion_profile:str = 'default', motion_profile_angular:str = 'slow'):
        self.motion_profile = motion_profile
        self.motion_profile_angular = motion_profile_angular
        self.params = RobotParams().get_params()[1]['omnibase']
        self.dead_zone = 0.0001
        self.accel_xy = self.params['motion'][self.motion_profile]['accel_xy_m']*0.25
        self.vel_xy = self.params['motion'][self.motion_profile]['vel_xy_m']

        self.accel_w_for_translation = self.params['motion'][self.motion_profile]['accel_w_r']
        self.vel_w_for_translation = self.params['motion'][self.motion_profile]['vel_w_r']
        self.accel_w_for_rotation_only = self.params['motion'][self.motion_profile_angular]['accel_w_r']
        self.vel_w_for_rotation_only = self.params['motion'][self.motion_profile_angular]['vel_w_r']

        self.accel_xy_max = self.params['motion']['max']['accel_xy_m']
        self.accel_w_max = self.params['motion']['max']['accel_w_r']

        self.precision_mode = 0.0
    
    def command_stick_to_motion(self, x, y, w,robot):
        """Convert a stick axis value to robot base's tank driving motion.

        Args:
            x (float): Range [-1.0,+1.0], control rotation speed
            y (float): Range [-1.0,+1.0], control linear speed
            robot (robot.Robot): Valid robot instance
        """
        v_x=self.vel_xy*(0 if abs(x)<self.dead_zone else x)
        v_y=self.vel_xy*(0 if abs(y)<self.dead_zone else y)
        v_w=self.vel_w_for_rotation_only*(0 if abs(w)<self.dead_zone else w)

        accel_w = self.accel_w_for_translation if v_w == 0 else self.accel_w_for_rotation_only

        kk = 1.0 - 0.75 * self.precision_mode
        robot.base.set_velocity(kk*v_x, kk*v_y, kk*v_w, self.accel_xy, accel_w)
    
    def stop_motion(self, robot):
        """Stop the joint motion. To be used when ever the controller is idle/no-inputs
        to stop unnecessary robot motion.

        Args:
            robot (robot.Robot): Valid robot instance
        """
        robot.base.set_velocity(0, 0, 0, self.accel_xy_max, self.accel_w_max)
            
class CommandLift:
    def __init__(self, motion_profile:str = 'default'):
        self.motion_profile = motion_profile
        self.params = RobotParams().get_params()[1]['lift']
        self.dead_zone = 0.0001
        self.max_linear_vel = self.params['motion'][self.motion_profile]['vel_m']
        self.precision_mode = 0.0
        self.acc = self.params['motion'][self.motion_profile]['accel_m']
        
    def _move(self, v_m, robot):
        scale = 1.0 - 0.75 * self.precision_mode
        v_m = v_m * scale
        robot.lift.set_velocity(v_m, a_m=self.acc)

    def command_stick_to_motion(self, x, robot):
        """Convert a stick axis value to robot lift motion.

        Args:
            x (float): Range [-1.0,+1.0], control lift speed
            robot (robot.Robot): Valid robot instance
        """
        if abs(x) < self.dead_zone:
            x = 0
        v_m = map_to_range(abs(x), 0, self.max_linear_vel)
        v_m *= -1 if x < 0 else 1

        self._move(v_m, robot)
    
    def command_button_to_motion(self, direction, robot):
        """Make lift move based on a button state.

        Args:
            direction (int): Direction integer -1 or +1
            robot (robot.Robot): Valid robot instance
        """
        v_m = self.max_linear_vel * direction
        self._move(v_m, robot)
    
    def stop_motion(self, robot):
        """Stop the joint motion. To be used when ever the controller is idle/no-inputs
        to stop unnecessary robot motion.

        Args:
            robot (robot.Robot): Valid robot instance
        """
        robot.lift.set_velocity(0, a_m=self.params['motion']['max']['accel_m'])

class CommandArm:
    def __init__(self, motion_profile:str = 'default'):
        self.motion_profile = motion_profile
        self.params = RobotParams().get_params()[1]['arm']
        self.dead_zone = 0.0001
        self.max_linear_vel = self.params['motion'][self.motion_profile]['vel_m']*0.75
        self.precision_mode = 0.0
        self.acc = self.params['motion'][self.motion_profile]['accel_m']

    def _move(self, v_m, robot):
        scale = 1.0 - 0.75 * self.precision_mode
        v_m = v_m * scale
        robot.arm.set_velocity(v_m, a_m=self.acc)

    def command_stick_to_motion(self, x, robot):
        """Convert a stick axis value to robot arm motion.

        Args:
            x (float): Range [-1.0,+1.0], control lift speed
            robot (robot.Robot): Valid robot instance
        """

        if abs(x) < self.dead_zone:
            x = 0

        v_m = map_to_range(abs(x), 0, self.max_linear_vel)
        v_m *= -1 if x < 0 else 1
        
        self._move(v_m, robot)

    def command_button_to_motion(self, direction, robot):
        """Make arm move based on a button state.

        Args:
            direction (int): Direction integer -1 or +1
            robot (robot.Robot): Valid robot instance
        """
        v_m = self.max_linear_vel * direction
        self._move(v_m, robot)

    def stop_motion(self, robot):
        """Stop the joint motion. To be used when ever the controller is idle/no-inputs
        to stop unnecessary robot motion.

        Args:
            robot (robot.Robot): Valid robot instance
        """
        robot.arm.set_velocity(0, a_m=self.params['motion']['max']['accel_m'])

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
        self.dx_deg=dx_deg
        self.max_vel = self.params['motion'][vel_type]['vel']
        self.acc = self.params['motion'][acc_type]['accel']
        self.precision_mode = 0.0

    def _move(self, dx_deg, robot):
        scale = 1.0 - (0.95 * self.precision_mode)
        dx_deg = dx_deg * scale
        robot.end_of_arm.move_by(self.name, deg_to_rad(dx_deg),self.max_vel, self.acc)

    def command_button_to_motion(self, direction, robot):
        """Make servo move based on a button state.

        Args:
            direction (int): Direction integer -1 or +1
            robot (robot.Robot): Valid robot instance
        """
        self._move(self.dx_deg * direction, robot)

    def command_stick_to_motion(self, x, robot):
        """Convert a stick axis value to robot arm motion.

        Args:
            x (float): Range [-1.0,+1.0], control lift speed
            robot (robot.Robot): Valid robot instance
        """
        if abs(x) < self.dead_zone:
            x = 0

        self._move(self.dx_deg * x, robot)
        
    def stop_motion(self, robot):
        """Stop the joint motion. To be used when ever the controller is idle/no-inputs
        to stop unnecessary robot motion.

        Args:
            robot (robot.Robot): Valid robot instance
        """
        robot.end_of_arm.move_by(self.name, 0)


class CommandWristYaw(CommandFeetechJoint):
    """Wrist Yaw motion command class for Dynamixel joints
    """
    def __init__(self, name='wrist_yaw', dx_deg=15.0, motion_profile:str = 'default'):
        super().__init__(name, dx_deg, motion_profile, motion_profile)

class CommandWristPitch(CommandFeetechJoint):
    """Wrist Pitch motion command class for Dynamixel joints
    """
    def __init__(self, name='wrist_pitch', dx_deg=15.0, motion_profile:str = 'default'):
        super().__init__(name, dx_deg, motion_profile, motion_profile)

class CommandWristRoll(CommandFeetechJoint):
    """Wrist Roll motion command class for Dynamixel joints
    """
    def __init__(self, name='wrist_roll', dx_deg=15.0, motion_profile:str = 'default'):
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
        self.precision_mode = 0.0
        self.stop_reqd=False

    def _move(self, pct, robot):
        scale = 1.0 - 0.75 * self.precision_mode
        pct = pct * scale
        robot.end_of_arm.move_by(self.name, pct, self.gripper_vel, self.gripper_accel)
        self.stop_reqd = True
    
    def open_gripper(self, robot):
        self._move(self.gripper_rotate_pct, robot)
        
    def close_gripper(self, robot):
        self._move(-self.gripper_rotate_pct, robot)

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
        self.precision_mode = 0.0
        self.stop_reqd = False

    def _move(self, dx_deg, robot):
        scale = 1.0 - 0.75 * self.precision_mode
        dx_deg = dx_deg * scale
        robot.end_of_arm.move_by(self.name, -deg_to_rad(dx_deg), self.gripper_vel, self.gripper_accel)
        self.stop_reqd = True
    
    def open_gripper(self, robot):
        self._move(self.gripper_rotate_deg, robot)
        
    def close_gripper(self, robot):
        self._move(-self.gripper_rotate_deg, robot)

    def stop_gripper(self, robot):
        if self.stop_reqd:
            robot.end_of_arm.move_by(self.name, 0)
            self.stop_reqd = False

