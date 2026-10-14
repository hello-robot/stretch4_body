import threading
from enum import Enum

import numpy as np

from stretch4_body.core.feetech.feetech_SM_chain import FeetechSMChain
from stretch4_body.core.hello_utils import get_sounds_dir, play_sound
from stretch4_body.core.robot_params import RobotParams


# Half-width of the band around the robot's x-z plane where the tool is reported as facing 'center'
TOOL_FACING_CENTER_BAND_DEG = 20.0


def is_wrist_flipped(pitch_rad):
    """
    The yaw-pitch-roll wrist reaches every tool orientation with two joint solutions:
    (yaw, pitch, roll) and (yaw + pi, pi - pitch, roll + pi). The two solutions are separated
    by the wrist singularity at pitch = +/-pi/2, so the branch is set by the pitch joint alone:
        not flipped: cos(pitch) >= 0  (pitch in [-pi/2, pi/2])
        flipped:     cos(pitch) <  0  (pitch beyond pi/2, i.e. the wrist is folded over)
    """
    return bool(np.cos(pitch_rad) < 0)


def get_tool_facing(yaw_rad, pitch_rad):
    """
    Returns which side of the robot the tool points toward: 'left' | 'right' | 'center'.

    The lift and arm are prismatic and all fixed mounts down to the wrist are unrotated, so the tool
    approach axis (wrist_roll_link +x) in the robot base frame depends only on wrist yaw and pitch:
        approach = Rz(yaw) * Ry(pitch) * x = (cos(yaw)cos(pitch), sin(yaw)cos(pitch), -sin(pitch))
    Base frame +y is the robot's left. Roll spins about the approach axis, so it does not change facing.
    'center' covers pointing forward, backward, up or down, within TOOL_FACING_CENTER_BAND_DEG.
    """
    lateral = np.sin(yaw_rad) * np.cos(pitch_rad)
    band = np.sin(np.deg2rad(TOOL_FACING_CENTER_BAND_DEG))
    if lateral > band:
        return 'left'
    if lateral < -band:
        return 'right'
    return 'center'


class GripperHandedness(Enum):
    """
    Teleop handedness of the gripper, which is the wrist's joint-space branch (see is_wrist_flipped):
        RIGHT: wrist not flipped
        LEFT:  wrist flipped
    """
    LEFT = 0
    RIGHT = 1

    @classmethod
    def from_wrist_flip(cls, wrist_flip):
        return cls.LEFT if wrist_flip else cls.RIGHT

    @classmethod
    def from_status(cls, status):
        """Returns the handedness implied by an end_of_arm status dict's 'wrist_flip', or None if unknown"""
        wrist_flip = status.get('wrist_flip')
        return None if wrist_flip is None else cls.from_wrist_flip(wrist_flip)

    def get_name(self):
        return self.name.lower()

    def opposite(self):
        return GripperHandedness.LEFT if self is GripperHandedness.RIGHT else GripperHandedness.RIGHT

    def play_sound_file(self):
        file_name:str
        if self is GripperHandedness.LEFT:
            file_name = "left_handed_mode.wav"
        elif self is GripperHandedness.RIGHT:
            file_name = "right_handed_mode.wav"
        else:
            raise NotImplementedError(f"No sound file for {self}")

        play_sound(get_sounds_dir()+f'/{file_name}')

    def move_to(self, end_of_arm):
        """
        Queues the wrist motion to this handedness's home configuration.
        end_of_arm: EndOfArm or EndOfArmClient. Caller is responsible for pushing the command.
        """
        print(f"Moving wrist to {self}")
        if self is GripperHandedness.RIGHT:
            targets = {'wrist_yaw': 0.0, 'wrist_pitch': 0.0, 'wrist_roll': 0.0}
        elif self is GripperHandedness.LEFT:
            targets = {'wrist_yaw': np.pi, 'wrist_pitch': np.pi, 'wrist_roll': np.pi}
        else:
            raise NotImplementedError(f"No move_to defined for {self}")

        params = RobotParams().get_params()[1]
        for joint, x_r in targets.items():
            slow = params[joint]['motion']['slow']
            end_of_arm.move_to(joint, x_r, slow['vel'], slow['accel'])


class EndOfArm(FeetechSMChain):
    """
    The EndOfArm class allows for an extensible serial chain of Feetech SM series devices
    It allows the specific type of device to be declared at runtime via the Yaml parameters
    In this way, a user can add their own custom Feetechv based tools to the robot end-of-arm by
    simply deriving it from FeetechSMHello and declaring the class name / Python module name
    in the User YAML file
    """
    def __init__(self, name='end_of_arm', usb=None):
        if usb is None:
            usb = RobotParams.get_params()[1]['end_of_arm']['usb_name']
        FeetechSMChain.__init__(self, usb=usb, name=name)
        self.status['is_homing']=False
        self.status['is_homed']=False
        self.status['wrist_flip']=None #True | False | None if no calibrated wrist_pitch. See is_wrist_flipped()
        self.status['tool_facing']=None #'left' | 'right' | 'center' | None if no calibrated wrist_yaw/pitch. See get_tool_facing()
        #NOTE: This may be a general bug. Fix. We make motors before port handler is set.
        #self.joints = self.params.get('devices', {}).keys()
        # for j in self.joints:
        #     module_name = self.params['devices'][j]['py_module_name']
        #     class_name = self.params['devices'][j]['py_class_name']
        #     servo_device = getattr(importlib.import_module(module_name), class_name)(chain=self)
        #     self.add_motor(servo_device)
        self.urdf_map={} #Override
        self.status_aux = {}

    def startup(self):
        if FeetechSMChain.startup(self):
            for i in range(10):
                self.pull_status()

            if self.params['devices']['wrist_yaw']['py_class_name'] == 'WristYaw' and len(list(self.motors.keys()))>0:
                pass # The instruction provided an incomplete 'if' block, adding 'pass' to maintain syntax.
            return True
        return False


    def pull_status(self,blocking=True):
        FeetechSMChain.pull_status(self)
        self.status['is_homed']=self.is_homed()
        self._update_wrist_orientation_status()

    def _update_wrist_orientation_status(self):
        def calibrated_pos(joint):
            m = self.motors.get(joint)
            if m is None or not m.status.get('pos_calibrated', False):
                return None
            return m.status['pos']
        yaw = calibrated_pos('wrist_yaw')
        pitch = calibrated_pos('wrist_pitch')
        self.status['wrist_flip'] = None if pitch is None else is_wrist_flipped(pitch)
        self.status['tool_facing'] = None if yaw is None or pitch is None else get_tool_facing(yaw, pitch)

    def change_handedness(self):
        """
        Moves the wrist to the opposite handedness of status['wrist_flip'].
        Returns the target GripperHandedness, or None if the current handedness is unknown.
        """
        current = GripperHandedness.from_status(self.status)
        if current is None:
            print("EndOfArm: Ignoring change_handedness, handedness unknown (wrist_pitch absent or not homed)")
            return None
        target = current.opposite()
        target.move_to(self)
        return target

    def get_joint(self, joint_name):
        """Retrieves joint by name.

        Parameters
        ----------
        joint_name : str
            valid joints defined as defined in params['devices']

        Returns
        -------
        FeetechSMHello or None
            Motor object on valid joint name, else None
        """
        return self.get_motor(joint_name)

    def quick_stop(self,joint):
        """
                joint: name of joint (string)
        """
        if joint not in self.motors:
            print("EndOfArm: Ignoring quick_stop command for inactive or non-existent joint '%s'" % joint)
            return
        with  self.pt_lock:
            self.motors[joint].quick_stop()

    def hold_position(self, joint: str) -> bool:
        """
        Stop and hold the current position of the specified joint with torque maintained.

        Args:
            joint: Name of the joint.

        Returns:
            True if hold command was executed, False otherwise.
        """
        if joint not in self.motors:
            self.logger.warning(
                f"EndOfArm: Ignoring hold_position command for inactive or non-existent joint '{joint}'"
            )
            return False
        with self.pt_lock:
            return self.motors[joint].hold_position()

    def disable_torque(self, joint):
        """
        joint: name of joint (string)
        """
        if joint not in self.motors:
            print("EndOfArm: Ignoring disable_torque command for inactive or non-existent joint '%s'" % joint)
            return
        with self.pt_lock:
            self.motors[joint].disable_torque()

    def enable_torque(self, joint):
        """
        joint: name of joint (string)
        """
        if joint in self.motors:
            self.motors[joint].enable_torque()

    def pause_sentry(self, joint):
        if joint in self.motors and hasattr(self.motors[joint], 'pause_sentry'):
            self.motors[joint].pause_sentry()

    def unpause_sentry(self, joint):
        if joint in self.motors and hasattr(self.motors[joint], 'unpause_sentry'):
            self.motors[joint].unpause_sentry()

    def move_to(self, joint, x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        if joint not in self.motors:
            print("EndOfArm: Ignoring move_to command for inactive or non-existent joint '%s'" % joint)
            return
        with self.pt_lock:
            self.motors[joint].move_to(x_r, v_r, a_r)
    def move_by(self, joint, x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded incremental motion (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        if joint not in self.motors:
            print("EndOfArm: Ignoring move_by command for inactive or non-existent joint '%s'" % joint)
            return
        with self.pt_lock:
            self.motors[joint].move_by(x_r, v_r, a_r)
    
    def set_velocity(self, joint, v_r, a_r=None):
        """
        joint: name of joint (string)
        v_r: commanded velocity (rad/s).
        a_r: acceleration motion profile (rad/s^2)
        """
        if joint not in self.motors:
            print("EndOfArm: Ignoring set_velocity command for inactive or non-existent joint '%s'" % joint)
            return
        with self.pt_lock:
            self.motors[joint].set_velocity(v_r, a_r)

    def pose(self,joint, p,v_r=None, a_r=None):
        """
                joint: name of joint (string)
                p: named pose of joint
                v_r: velocity for trapezoidal motion profile (rad/s).
                a_r: acceleration for trapezoidal motion profile (rad/s^2)
                """
        if joint not in self.motors:
            print("EndOfArm: Ignoring pose command for inactive or non-existent joint '%s'" % joint)
            return
        with self.pt_lock:
            self.motors[joint].pose(p, v_r, a_r)

    def stow(self):
        pass #Override by specific tool

    def pre_stow(self,robot=None):
        pass #Override by specific tool

    def is_homed(self):
        #Return true if calibration is  required
        for m in self.motors:
            if not self.motors[m].is_homed():
                return False
        return True


    def cancel_homing(self):
        self.cancel_homing_event.set()
        self.logger.warning(f"Feetech homing cancelled for: {self.name}")

    def home(self):
        """
        Home to hardstops
        """
        #Naive version. Should override with tool specific safe motions.
        raise NotImplementedError('EndOfArm Homing not implemented at this level.')

    def home_joint(self, joint_name=None,end_pos=0,wait_on_completion:bool=True):
        """
        Home to hardstops
        """
        self.logger.info(f'--------- Homing {joint_name} ----')
        self.status['is_homing'] = True
        if wait_on_completion:
            self.motors[joint_name].home(end_pos=end_pos)
        else:
            thread = threading.Thread(target=self.motors[joint_name].home, args=(end_pos,))
            thread.start()
        self.status['is_homing'] = False


    def is_tool_present(self,class_name):
        """
        Return true if the given tool type is present (eg. StretchGripper)
        Allows for conditional logic when switching end-of-arm tools
        """
        for j in self.joints:
            if class_name == self.params['devices'][j]['py_class_name']:
                return True
        return False

    def step_collision_avoidance(self,joint_name,in_collision_stop):
        self.get_joint(joint_name).step_collision_avoidance(in_collision_stop)

    # def get_joint_configuration(self,brake_joints={}):
    #     """
    #     Construct a dictionary of tools current pose (for robot_collision_mgmt)
    #     Keys match joint names in URDF
    #     Specific tools should define urdf_map
    #     """
    #     ret = {}
    #     for j in self.urdf_map:
    #         jn = self.urdf_map[j]
    #         motor = self.get_joint(jn)
    #         dx = 0.0
    #         try:
    #             if brake_joints[j]:
    #                 dx = self.params['collision_mgmt']['k_brake_distance'][jn] * motor.get_braking_distance()
    #         except KeyError:
    #             dx=0
    #         ret[j] = motor.status['pos'] + dx
    #
    #     gripper_joint = None
    #     for j in self.joints:
    #         if 'gripper' in j:
    #             gripper_joint = j
    #
    #     if gripper_joint:
    #         dx = 0
    #         if brake_joints:
    #             for j in brake_joints:
    #                 if 'gripper' in j:
    #                     dx = self.params['collision_mgmt']['k_brake_distance'][j]
    #         finger_angle = self.get_joint(gripper_joint).status['gripper_conversion']['finger_rad'] + dx
    #         ret['gripper_finger_left_joint'] = finger_angle/2
    #         ret['gripper_finger_right_joint'] = finger_angle/2
    #
    #     return ret
    
    def pre_stow(self,robot=None):
        pass



