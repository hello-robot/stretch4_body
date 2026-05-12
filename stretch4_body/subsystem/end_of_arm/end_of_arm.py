import threading
from stretch4_body.core.feetech.feetech_SM_chain import FeetechSMChain
from stretch4_body.core.robot_params import RobotParams
import time


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
        #NOTE: This may be a general bug. Fix. We make motors before port handler is set.
        #self.joints = self.params.get('devices', {}).keys()
        # for j in self.joints:
        #     module_name = self.params['devices'][j]['py_module_name']
        #     class_name = self.params['devices'][j]['py_class_name']
        #     servo_device = getattr(importlib.import_module(module_name), class_name)(chain=self)
        #     self.add_motor(servo_device)
        self.urdf_map={} #Override
        self.status_aux = {}

        self.cancel_homing_event = threading.Event()

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

    def move_to(self, joint,x_r, v_r=None, a_r=None):
        """
        joint: name of joint (string)
        x_r: commanded absolute position (radians).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        if joint not in self.motors:
            print("EndOfArm: Ignoring move_to command for inactive or non-existent joint '%s'" % joint)
            return
        with  self.pt_lock:
            self.motors[joint].move_to(x_r, v_r, a_r)

    def move_to_mm(self, joint, x_mm, v_r=None, a_r=None):
        if joint not in self.motors:
            print("EndOfArm: Ignoring move_to_mm command for inactive or non-existent joint '%s'" % joint)
            return
        if not hasattr(self.motors[joint], 'move_to_mm'):
            print("EndOfArm: Ignoring move_to_mm command for joint '%s' which doesn't support mm" % joint)
            return
        with self.pt_lock:
            self.motors[joint].move_to_mm(x_mm, v_r, a_r)

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

    def move_by_mm(self, joint, x_mm, v_r=None, a_r=None):
        if joint not in self.motors:
            print("EndOfArm: Ignoring move_by_mm command for inactive or non-existent joint '%s'" % joint)
            return
        if not hasattr(self.motors[joint], 'move_by_mm'):
            print("EndOfArm: Ignoring move_by_mm command for joint '%s' which doesn't support mm" % joint)
            return
        with self.pt_lock:
            self.motors[joint].move_by_mm(x_mm, v_r, a_r)
    
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

    def home_joint(self, joint_name=None,end_pos=0):
        """
        Home to hardstops
        """
        self.logger.info(f'--------- Homing {joint_name} ----')
        self.status['is_homing'] = True
        self.motors[joint_name].home(end_pos=end_pos, cancel_homing_event=self.cancel_homing_event)
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
    #         ret['joint_gripper_finger_left'] = finger_angle/2
    #         ret['joint_gripper_finger_right'] = finger_angle/2
    #
    #     return ret
    
    def pre_stow(self,robot=None):
        pass



