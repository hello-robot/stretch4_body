import threading
from stretch4_body.core.hello_utils import *
from stretch4_body.subsystem.end_of_arm.end_of_arm import EndOfArm
import time



# ##########################################################3#

def home_dw4_joints(eoa: "EndOfArm"):
    eoa.cancel_homing_event.clear()
    eoa.motors['wrist_pitch'].motor.set_overcurrent(10)
    eoa.motors['wrist_pitch'].enable_pwm()
    eoa.motors['wrist_pitch'].set_pwm(-175)
    time.sleep(0.2)
    t = time.time()
    while eoa.motors['wrist_pitch'].status['vel'] < -0.66 and time.time() - t < 5:
        pass
    eoa.motors['wrist_pitch'].set_pwm(175)
    time.sleep(0.2)
    t = time.time()
    while eoa.motors['wrist_pitch'].status['vel'] > 0.7 and time.time() - t < 5: 
        pass
    eoa.motors['wrist_pitch'].enable_pos()
    time.sleep(0.5)
    if eoa.cancel_homing_event.is_set() or not eoa.motors['wrist_yaw'].home(end_pos=0, cancel_homing_event=eoa.cancel_homing_event):
        eoa.logger.error("Wrist yaw homing failed")
        return False
    if eoa.cancel_homing_event.is_set() or not eoa.motors['wrist_roll'].home(end_pos=0, cancel_homing_event=eoa.cancel_homing_event):
        eoa.logger.error("Wrist roll homing failed")
        return False
    eoa.motors['wrist_pitch'].motor.set_overcurrent(eoa.motors['wrist_pitch'].params['eeprom_cfg']['overcurrent'])
    if eoa.cancel_homing_event.is_set() or not eoa.motors['wrist_pitch'].home(end_pos=0, cancel_homing_event=eoa.cancel_homing_event):  # deg_to_rad(-90.0))
        eoa.logger.error("Wrist pitch homing failed")
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
            'joint_wrist_yaw':'wrist_yaw',
            'joint_wrist_pitch': 'wrist_pitch',
            'joint_wrist_roll':'wrist_roll'}
    def stow(self):
        # Fold in wrist and gripper
        self.logger.info(f'--------- Stowing {self.name} ----')
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])

    def home(self):
        def _do_home():
            self.logger.info(f'Homing {self.name}')
            self.status['is_homing'] = True
            home_dw4_joints(self)
            self.status['is_homing'] = False
        thread = threading.Thread(target=_do_home)
        thread.start()
        

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
            'joint_wrist_yaw':'wrist_yaw',
            'joint_wrist_pitch': 'wrist_pitch',
            'joint_wrist_roll':'wrist_roll'}
    def stow(self):
        # Fold in wrist and gripper
        self.logger.info(f'--------- Stowing {self.name} ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        time.sleep(3.0)
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])


        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])

    def home(self):
        def _do_home():
            self.logger.debug(f'Homing {self.name} started.')
            start_time = time.time()
            self.status['is_homing'] = True
            home_dw4_joints(self)
            self.motors['stretch_gripper'].home(cancel_homing_event=self.cancel_homing_event,end_pos=0)
            self.status['is_homing'] = False
            self.logger.debug(f'Homing {self.name} completed in {time.time() - start_time} seconds.')
        thread = threading.Thread(target=_do_home)
        thread.start()


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
            'joint_wrist_yaw':'wrist_yaw',
            'joint_wrist_pitch': 'wrist_pitch',
            'joint_wrist_roll':'wrist_roll'}
    def stow(self):
        # Fold in wrist and gripper
        self.logger.info(f'--------- Stowing {self.name} ----')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        time.sleep(3.0)
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])


        self.move_to('parallel_gripper', self.params['stow']['parallel_gripper'])

    def home(self):
        def _do_home():
            self.logger.info(f'Homing {self.name}')
            self.status['is_homing'] = True
            home_dw4_joints(self)
            self.motors['parallel_gripper'].home(cancel_homing_event=self.cancel_homing_event,end_pos=0)
            self.status['is_homing'] = False
        thread = threading.Thread(target=_do_home)
        thread.start()


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