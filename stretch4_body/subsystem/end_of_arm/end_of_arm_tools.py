import threading
from stretch4_body.core.hello_utils import *
from stretch4_body.subsystem.end_of_arm.end_of_arm import EndOfArm
import time



# ##########################################################3#

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
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('stretch_gripper', self.params['stow']['stretch_gripper'])


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
        
        self.home()

        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        self.move_to('parallel_gripper', self.params['stow']['parallel_gripper'])


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

    def _home_all_joints(self):
        success = super()._home_all_joints()
        if success:
            self.move_to('wrist_roll', 0.0)
        return success
