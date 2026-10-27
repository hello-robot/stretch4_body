import threading
from enum import Enum
import time

import numpy as np

from stretch4_body.core.feetech.feetech_SM_chain import FeetechSMChain
from stretch4_body.core.hello_utils import get_sounds_dir, play_sound
from stretch4_body.core.robot_params import RobotParams


# Half-width of the band around the robot's x-z plane where the tool is reported as facing 'center'
TOOL_FACING_CENTER_BAND_DEG = 10.0
# Half-width of the band around the arm's centre line where the tool mount is reported as being on the 'center'
TOOL_SIDE_CENTER_BAND_M = 0.01


def should_invert_pitch(roll_rad):
    """
    Returns True when teleop must invert wrist pitch so that pitching up still moves the gripper camera image up.

    A +pitch motion moves the tool along -(0, sin(roll), cos(roll)) in wrist_roll_link, whose +z is the
    camera's image-up axis. So seen through the camera, +pitch moves the image down when cos(roll) > 0 and
    up when cos(roll) < 0, regardless of yaw and pitch. Pitch must be inverted once roll crosses +/-pi/2:
        roll in [-pi/2, pi/2]: not inverted
        roll beyond pi/2:      inverted
    """
    return bool(np.cos(roll_rad) < 0)


def is_camera_upright(pitch_rad, roll_rad):
    """
    Returns True when the gripper camera is upright in the world (image-up points above the horizon).

    The camera's image-up axis (wrist_roll_link +z) has a world-up component of cos(pitch)*cos(roll);
    yaw spins about world-up, so it does not contribute:
        pitch    roll    camera
          0        0     upright
          0       pi     upside down
         pi        0     upside down
         pi       pi     upright
    """
    return bool(np.cos(pitch_rad) * np.cos(roll_rad) > 0)


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


def is_tool_right_handed(pitch_rad):
    """
    Returns True when the tool is right handed: the wrist is not folded over, so the tool can reach
    under the arm on the robot's left side. False (left handed) is the folded wrist, which mirrors this.

    The yaw-pitch-roll wrist reaches every tool orientation with two joint solutions:
    (yaw, pitch, roll) and (yaw + pi, pi - pitch, roll + pi), separated by the wrist singularity at
    pitch = +/-pi/2, so handedness is set by the pitch joint alone (the middle of its [-65, 245] deg range):
        right handed: cos(pitch) >= 0  (pitch in [-pi/2, pi/2]), e.g. home yaw, pitch, roll == 0,0,0
        left handed:  cos(pitch) <  0  (pitch beyond pi/2),      e.g. home yaw, pitch, roll == pi,pi,pi
    Pitch inversion for teleop depends on roll instead, see should_invert_pitch().
    """
    return bool(np.cos(pitch_rad) >= 0)


def get_tool_side(yaw_rad, pitch_rad, tool_reach_m, wrist_offset_m):
    """
    Returns which side of the arm the tool is mounted on: 'left' | 'right' | 'center'.
    (Which way the tool points is get_tool_facing(); e.g. a tool mounted on the left can point right.)

    The wrist yaw axis lies on the arm's centre line, and the pitch and roll housings hold the tool
    wrist_offset_m to the right (-y) of it. The tool mount lies on the roll axis, tool_reach_m from the
    pitch axis, so its lateral position from the arm's centre line depends only on yaw and pitch:
        lateral = tool_reach_m * cos(pitch) * sin(yaw) - wrist_offset_m * cos(yaw)
    Base frame +y is the robot's left. 'center' is within TOOL_SIDE_CENTER_BAND_M of the centre line.
    See get_wrist_tool_geometry() for the two lengths.
    """
    lateral = tool_reach_m * np.cos(pitch_rad) * np.sin(yaw_rad) - wrist_offset_m * np.cos(yaw_rad)
    if lateral > TOOL_SIDE_CENTER_BAND_M:
        return 'left'
    if lateral < -TOOL_SIDE_CENTER_BAND_M:
        return 'right'
    return 'center'


def get_wrist_tool_geometry():
    """
    Returns (tool_reach_m, wrist_offset_m) for get_tool_side(), read from the robot's URDF:
        tool_reach_m:   pitch axis to the tool mount (tool_attachment_site_link) along the roll axis
        wrist_offset_m: lateral offset of the roll axis from the yaw axis
    """
    from stretch4_urdf import get_transform, get_urdf
    robot = RobotParams.get_params()[1]['robot']
    urdf = get_urdf(robot['model_name'], robot['batch_name'], robot['tool'], do_add_file_prefix_to_absolute_paths=False)
    pitch_origin = get_transform(urdf, 'wrist_pitch_link', 'wrist_yaw_link')[:3, 3]
    roll_origin = get_transform(urdf, 'wrist_roll_link', 'wrist_pitch_link')[:3, 3]
    mount_x = get_transform(urdf, 'tool_attachment_site_link', 'wrist_roll_link')[0, 3]
    return roll_origin[0] + mount_x, -(pitch_origin[1] + roll_origin[1])


def get_handedness_lift_min():
    """
    Lift height (m) at or below which a handedness change is refused, so the wrist does not swing
    into the ground or base. Shared by routine_wrist_handedness, EndOfArmClient and gamepad teleop.
    """
    return RobotParams.get_params()[1].get('routine_wrist_handedness', {}).get('lift_min', 0.35)


class GripperHandedness(Enum):
    """
    Teleop handedness of the gripper, set by whether the wrist is folded over (see is_tool_right_handed):
        RIGHT: yaw, pitch, roll == 0,0,0. Wrist not folded, tool can reach under the arm on the robot's left.
        LEFT:  yaw, pitch, roll == pi,pi,pi. Wrist folded, mirrors RIGHT.
    Both home configurations keep the camera upright.
    """
    LEFT = 0
    RIGHT = 1

    @classmethod
    def from_right_handed(cls, right_handed):
        return cls.RIGHT if right_handed else cls.LEFT

    @classmethod
    def from_status(cls, status):
        """Returns the handedness implied by an end_of_arm status dict's 'tool_right_handed', or None if unknown"""
        right_handed = status.get('tool_right_handed')
        return None if right_handed is None else cls.from_right_handed(right_handed)

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

    def get_joint_targets(self):
        """Wrist joint positions (rad) of this handedness's home configuration"""
        if self is GripperHandedness.RIGHT:
            return {'wrist_yaw': 0.0, 'wrist_pitch': 0.0, 'wrist_roll': 0.0}
        if self is GripperHandedness.LEFT:
            return {'wrist_yaw': np.pi, 'wrist_pitch': np.pi, 'wrist_roll': np.pi}
        raise NotImplementedError(f"No joint targets defined for {self}")

    def move_to(self, end_of_arm):
        """
        Queues the wrist motion to this handedness's home configuration.
        end_of_arm: EndOfArm or EndOfArmClient. Caller is responsible for pushing the command.
        """
        print(f"Moving wrist to {self}")
        params = RobotParams().get_params()[1]
        for joint, x_r in self.get_joint_targets().items():
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
        self.status['tool_right_handed']=None #True | False | None if no calibrated wrist_pitch. See is_tool_right_handed()
        self.status['camera_upright']=None #True | False | None if no calibrated wrist_pitch/roll. See is_camera_upright()
        self.status['tool_facing']=None #'left' | 'right' | 'center' | None if no calibrated wrist_yaw/pitch. See get_tool_facing()
        self.status['tool_side']=None #'left' | 'right' | 'center' | None if no calibrated wrist_yaw/pitch or no URDF. See get_tool_side()
        #NOTE: This may be a general bug. Fix. We make motors before port handler is set.
        #self.joints = self.params.get('devices', {}).keys()
        # for j in self.joints:
        #     module_name = self.params['devices'][j]['py_module_name']
        #     class_name = self.params['devices'][j]['py_class_name']
        #     servo_device = getattr(importlib.import_module(module_name), class_name)(chain=self)
        #     self.add_motor(servo_device)
        self.urdf_map={} #Override
        self.status_aux = {}
        self._wrist_tool_geometry = None # (tool_reach_m, wrist_offset_m), loaded from the URDF on first use
        self._wrist_tool_geometry_failed = False

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
        roll = calibrated_pos('wrist_roll')
        geometry = None if yaw is None or pitch is None else self._get_wrist_tool_geometry()
        self.status['tool_right_handed'] = None if pitch is None else is_tool_right_handed(pitch)
        self.status['camera_upright'] = None if pitch is None or roll is None else is_camera_upright(pitch, roll)
        self.status['tool_facing'] = None if yaw is None or pitch is None else get_tool_facing(yaw, pitch)
        self.status['tool_side'] = None if geometry is None else get_tool_side(yaw, pitch, *geometry)

    def _get_wrist_tool_geometry(self):
        if self._wrist_tool_geometry is None and not self._wrist_tool_geometry_failed:
            try:
                self._wrist_tool_geometry = get_wrist_tool_geometry()
            except Exception as e:
                self._wrist_tool_geometry_failed = True
                self.logger.warning(f"EndOfArm: tool_side unavailable, could not read wrist geometry from URDF: {e}")
        return self._wrist_tool_geometry

    def change_handedness(self):
        """
        Moves the wrist to the opposite handedness of status['tool_right_handed'].
        Returns the target GripperHandedness, or None if the current handedness is unknown.

        The end of arm cannot see the lift, so callers must first check that the lift is above
        get_handedness_lift_min() so the wrist does not swing into the ground or base
        (as routine_wrist_handedness does).
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

    def is_homed(self):
        #Return true if calibration is  required
        for m in self.motors:
            if not self.motors[m].is_homed():
                return False
        return True


    def cancel_homing(self):
        self.cancel_homing_event.set()
        self.logger.warning(f"Feetech homing cancelled for: {self.name}")

    def _home_all_joints(self):
        self.logger.info(f'Homing {self.name}')
        self.status['is_homing'] = True

        if 'wrist_pitch' in self.motors:
            success = self.motors['wrist_pitch'].pre_home(pwm_val=175,negative_vel=-0.66,positive_vel=0.7)

            if not success or self.cancel_homing_event.is_set():
                self.logger.error("Wrist pitch pre-homing failed")
                self.status['is_homing'] = False
                return False

            time.sleep(0.5)

            self.motors['wrist_pitch'].motor.set_overcurrent(self.motors['wrist_pitch'].params['eeprom_cfg']['overcurrent'])

        joints = ['wrist_yaw', 'wrist_roll', 'wrist_pitch']

        joints = [name for name in joints if name in self.motors] # Remove yaw/roll/pitch if they aren't in self.motors.
        joints += [name for name in self.motors if name not in joints]

        self.logger.info(f"Homing joints: {joints}")
        for joint_name in joints:
            success = self.home_joint(joint_name, set_homing_status=False)
            if not success:
                self.logger.error(f"{joint_name} homing failed")
                break
            if self.cancel_homing_event.is_set():
                self.logger.error(f"Homing canceled during {joint_name} homing.")
            
        self.status['is_homing'] = False

        return success

    def home(self, wait_on_completion:bool=True) -> bool|None:
        """
        Homes the wrist joints. Warning: self-collision detection is usually disabled during homing! If you are implementing a new tool, be sure to update the homing parameters for the joints, so that your tool does not collide with the robot.
        """
        if wait_on_completion:
            return self._home_all_joints()
        
        thread = threading.Thread(target=self._home_all_joints)

        thread.start()

        return None

    def home_joint(self, joint_name:str,end_pos:float|None=None, set_homing_status:bool=True, wait_on_completion:bool=True) -> bool|None:
        """
        Home to hardstops
        end_pos: final position after homing. Defaults to the tool's `homing.<joint_name>` param, else 0.
        wait_on_completion: if False, home in a background thread and return None.
        """
        if not wait_on_completion:
            threading.Thread(target=self.home_joint, args=(joint_name, end_pos, set_homing_status)).start()
            return None

        self.logger.info(f'--------- Homing {joint_name} ----')

        if end_pos is None:
            end_pos = self.params.get('homing', {}).get(joint_name, 0)

        if set_homing_status:
            self.status['is_homing'] = True

        success = self.motors[joint_name].home(end_pos=end_pos)

        if set_homing_status:
            self.status['is_homing'] = False

        return success


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
    

