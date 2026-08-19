from stretch4_body.core.hello_utils import *
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.utils.stretch_pose_models import RobotJoints
from stretch4_body.utils.tool_metadata import get_tool_metadata

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

        self.accel_xy_max = self.params['motion']['max']['accel_xy_m']
        self.accel_w_max = self.params['motion']['max']['accel_w_r']

        self.precision_mode = 0.0
    
    def _get_motion_params(self, is_rotating:bool):
        motion_profile = self.motion_profile
        if is_rotating:
            motion_profile = self.motion_profile_angular

        vel_xy = self.params['motion'][motion_profile]['vel_xy_m']
        accel_xy = self.params['motion'][motion_profile]['accel_xy_m']
        vel_w = self.params['motion'][motion_profile]['vel_w_r']
        accel_w = self.params['motion'][motion_profile]['accel_w_r']

        return vel_xy, accel_xy, vel_w, accel_w

    
    def _move(self, x, y, w, accel_xy, accel_w, robot):
        scale = 1.0 - 0.75 * self.precision_mode
        robot.base.set_velocity(scale*x, scale*y, scale*w, accel_xy, accel_w)
    
    def command_stick_to_motion(self, x, y, w,robot):
        """Convert a stick axis value to robot base's tank driving motion.

        Args:
            x (float): Range [-1.0,+1.0], control linear x speed
            y (float): Range [-1.0,+1.0], control linear y speed
            w (float): Range [-1.0,+1.0], control angular speed
            robot (robot.Robot): Valid robot instance
        """
        vel_xy, accel_xy, vel_w, accel_w = self._get_motion_params(is_rotating=abs(w)>=0.1)

        v_x=vel_xy*(0 if abs(x)<self.dead_zone else x)
        v_y=vel_xy*(0 if abs(y)<self.dead_zone else y)
        v_w=vel_w*(0 if abs(w)<self.dead_zone else w)

        self._move(v_x, v_y, v_w, accel_xy, accel_w, robot)
    
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

    def _move(self, dx_deg, robot, velocity:float|None = None):
        scale = 1.0 - (0.95 * self.precision_mode)
        dx_deg = dx_deg * scale

        capped_velocity = min(self.max_vel, velocity) if velocity is not None else self.max_vel
        robot.end_of_arm.move_by(self.name, deg_to_rad(dx_deg),capped_velocity, self.acc)

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

            
class CommandToolPosition:
    """Generic end-of-arm tool motion command class using ToolMetadata for range calculation."""

    def __init__(self, name: str | None = None, motion_profile: str = 'max'):
        self.name = name or RobotJoints.gripper.value or 'stretch_gripper'
        try:
            self.metadata = get_tool_metadata(self.name)
            low, high = self.metadata.actuator_command_range
            self.step_inc = (high - low) / 10.0 if high != low else 1.0
        except Exception:
            self.metadata = None
            self.step_inc = 10.0 if 'parallel' not in self.name else 0.01

        _, robot_params = RobotParams().get_params()
        tool_params = robot_params.get(self.name, {})
        motion_params = tool_params.get('motion', {}).get(motion_profile, {})
        self.gripper_accel = motion_params.get('accel', None)
        self.gripper_vel = motion_params.get('vel', None)
        self.precision_mode = 0.0
        self.stop_reqd = False

    def _move(self, inc: float, robot):
        scale = 1.0 - 0.75 * self.precision_mode
        inc = inc * scale
        robot.end_of_arm.move_by(self.name, inc, self.gripper_vel, self.gripper_accel)
        self.stop_reqd = True

    def open_gripper(self, robot):
        self._move(self.step_inc, robot)

    def close_gripper(self, robot):
        self._move(-self.step_inc, robot)

    def stop_gripper(self, robot):
        if self.stop_reqd:
            try:
                robot.end_of_arm.quick_stop(self.name)
            except (AttributeError, KeyError):
                robot.end_of_arm.move_by(self.name, 0.0)
            self.stop_reqd = False


# Backwards compatibility aliases
CommandGripperPosition = CommandToolPosition
CommandStretchGripperPosition = CommandToolPosition
CommandParallelGripperPosition = CommandToolPosition

