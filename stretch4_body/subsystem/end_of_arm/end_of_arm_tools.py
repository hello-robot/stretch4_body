import threading
from stretch4_body.core.hello_utils import *
from stretch4_body.subsystem.end_of_arm.end_of_arm import EndOfArm
import time



# ##########################################################3#

def _home_joint(eoa: "EndOfArm", joint_name: str):
    success = eoa.motors[f"wrist_{joint_name}"].home(end_pos=0)
    if not success or eoa.cancel_homing_event.is_set():
        eoa.logger.error(f"{joint_name} homing failed")
        return False
    return True

def home_dw4_joints(eoa: "EndOfArm"):
    success = eoa.motors['wrist_pitch'].pre_home(pwm_val=175,negative_vel=-0.66,positive_vel=0.7)

    if not success or eoa.cancel_homing_event.is_set():
        eoa.logger.error("Wrist pitch pre-homing failed")
        return False

    time.sleep(0.5)
    if not _home_joint(eoa, 'yaw'):
        return False
    if not _home_joint(eoa, 'roll'):
        return False
    eoa.motors['wrist_pitch'].motor.set_overcurrent(eoa.motors['wrist_pitch'].params['eeprom_cfg']['overcurrent'])
    if not _home_joint(eoa, 'pitch'):
        return False

    return True


class EOA_Wrist_DW4_Tool_NIL(EndOfArm):
    """
    Wrist Yaw / Pitch / Roll only for version 3 of DexWrist
    """
    def __init__(self, name='eoa_wrist_dw4_tool_nil'):
        EndOfArm.__init__(self, name)

        #This maps from the name of a joint in the URDF to the name of the joint in Stretch Body
        #It is used by CollisionMgmt.
        self.urdf_map={
            'wrist_yaw_joint':'wrist_yaw',
            'wrist_pitch_joint': 'wrist_pitch',
            'wrist_roll_joint':'wrist_roll'}
    def stow(self):
        # Fold in wrist and gripper
        self.logger.info(f'--------- Stowing {self.name} ----')
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])

    def home(self, wait_on_completion=True):
        def _do_home():
            self.logger.info(f'Homing {self.name}')
            self.status['is_homing'] = True
            success = home_dw4_joints(self)
            self.status['is_homing'] = False
            return success
        if wait_on_completion:
            return _do_home()
        
        thread = threading.Thread(target=_do_home)
        thread.start()
        return None
        

    def pre_stow(self,robot=None):
        if robot:
            robot.end_of_arm.move_to('wrist_pitch', robot.end_of_arm.params['stow']['wrist_pitch'])
        else:
            self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])

class EOA_Wrist_DW4_Tool_SG4(EndOfArm):
    """
    Wrist Yaw / Pitch / Roll /Gripper for version 4 of DexWrist
    """
    def __init__(self, name='eoa_wrist_dw4_tool_sg4'):
        EndOfArm.__init__(self, name)

        #This maps from the name of a joint in the URDF to the name of the joint in Stretch Body
        #It is used by CollisionMgmt.
        self.urdf_map={
            'wrist_yaw_joint':'wrist_yaw',
            'wrist_pitch_joint': 'wrist_pitch',
            'wrist_roll_joint':'wrist_roll'}
    def stow(self):
        # Fold in wrist and gripper
        self.logger.info(f'--------- Stowing {self.name} ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        time.sleep(3.0)
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])


        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])

    def home(self, wait_on_completion=True):
        def _do_home():
            self.logger.debug(f'Homing {self.name} started.')
            start_time = time.time()
            self.status['is_homing'] = True
            success = home_dw4_joints(self)
            success = success and self.motors['stretch_gripper'].home(end_pos=0)
            self.status['is_homing'] = False
            self.logger.debug(f'Homing {self.name} completed in {time.time() - start_time} seconds.')
            return success
        if wait_on_completion:
            return _do_home()
        
        thread = threading.Thread(target=_do_home)
        thread.start()
        return None


    def pre_stow(self,robot=None):
        if robot:
            robot.end_of_arm.move_to('wrist_pitch', robot.end_of_arm.params['stow']['wrist_pitch'])
        else:
            self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])


class EOA_Wrist_DW4_Tool_PG4(EndOfArm):
    """
    Wrist Yaw / Pitch / Roll /Gripper for version 4 of DexWrist
    """
    def __init__(self, name='eoa_wrist_dw4_tool_pg4'):
        EndOfArm.__init__(self, name)

        #This maps from the name of a joint in the URDF to the name of the joint in Stretch Body
        #It is used by CollisionMgmt.
        self.urdf_map={
            'wrist_yaw_joint':'wrist_yaw',
            'wrist_pitch_joint': 'wrist_pitch',
            'wrist_roll_joint':'wrist_roll'}
    def stow(self):
        # Fold in wrist and gripper
        self.logger.info(f'--------- Stowing {self.name} ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        time.sleep(3.0)
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])


        self.move_to('parallel_gripper', self.params['stow']['parallel_gripper'])

    def home(self, wait_on_completion=True):
        def _do_home():
            self.logger.info(f'Homing {self.name}')
            self.status['is_homing'] = True
            success = home_dw4_joints(self)
            success = success and self.motors['parallel_gripper'].home(end_pos=0)
            self.status['is_homing'] = False
            return success

        if wait_on_completion:
            return _do_home()
        
        thread = threading.Thread(target=_do_home)
        thread.start()
        return None


    def pre_stow(self,robot=None):
        if robot:
            robot.end_of_arm.move_to('wrist_pitch', robot.end_of_arm.params['stow']['wrist_pitch'])
        else:
            self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])


class EOA_Wrist_DW4_Tool_Calibration(EOA_Wrist_DW4_Tool_NIL):
    """
    Wrist Yaw / Pitch / Roll only for version 3 of DexWrist
    """
    def __init__(self, name='eoa_wrist_dw4_tool_calibration'):
        EOA_Wrist_DW4_Tool_NIL.__init__(self, name)


class EOA_Wrist_DW4_Tool_Tablet(EOA_Wrist_DW4_Tool_NIL):
    """
    Wrist Yaw / Pitch / Roll only for version 3 of DexWrist
    """
    def __init__(self, name='eoa_wrist_dw4_tool_tablet'):
        EOA_Wrist_DW4_Tool_NIL.__init__(self, name)
        self.logger.info(f"Wrist yaw stow position: {self.params['stow']['wrist_yaw']}")

    def home(self, wait_on_completion=True):
        def _do_home():
            self.logger.info(f'Homing {self.name}')
            self.status['is_homing'] = True
            success = self._home_tablet_joints()
            self.status['is_homing'] = False
            return success
        if wait_on_completion:
            return _do_home()

        thread = threading.Thread(target=_do_home)
        thread.start()
        return None

    def _home_tablet_joints(self):
        success = self.motors['wrist_pitch'].pre_home(pwm_val=175,negative_vel=-0.66,positive_vel=0.7)

        if not success or self.cancel_homing_event.is_set():
            self.logger.error("Wrist pitch pre-homing failed")
            return False

        time.sleep(0.5)
        if not _home_joint(self, 'yaw'):
            return False

        # The tablet hits the arm before pitch reaches its hardstop unless roll
        # is first rotated to a clearance position. 
        roll_clearance = self.params['homing']['wrist_roll']
        success = self.motors['wrist_roll'].home(end_pos=roll_clearance)
        if not success or self.cancel_homing_event.is_set():
            self.logger.error("roll homing failed")
            return False

        self.motors['wrist_pitch'].motor.set_overcurrent(self.motors['wrist_pitch'].params['eeprom_cfg']['overcurrent'])
        if not _home_joint(self, 'pitch'):
            return False

        self.move_to('wrist_roll', 0.0)
        return True
