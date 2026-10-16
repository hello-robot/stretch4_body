from stretch4_body.core.hello_utils import *
from stretch4_body.core.robot_params import RobotParams
import numpy as np
from enum import Enum

class GuardedContactSensitivity(Enum):
    """
    Name of the sensitivity mode as defined in robot_params_SE4. (e.g. 'off', 'default','high_sensitivity_nav', 'high_sensitivity_manipulation')
    """

    HIGH_SENSITIVITY_NAV = 1
    HIGH_SENSITIVITY_MANIPULATION = 2  # Low Strength
    MEDIUM = 3 # Medium Strength (default in Stretch Body)
    STRONG_MANIPULATION = 4 # High Strength
    def _get_cycleable_options(self):
        # return list(type(self))
        return [GuardedContactSensitivity.HIGH_SENSITIVITY_MANIPULATION, GuardedContactSensitivity.MEDIUM, GuardedContactSensitivity.STRONG_MANIPULATION]


    def cycle(self, is_forward:bool):
        index_offset = 1 if is_forward else -1
        members = self._get_cycleable_options()
        index = members.index(self)
        return members[(index + index_offset) % len(members)]

    def play_sound_file(self):
        file_name:str
        if self is GuardedContactSensitivity.MEDIUM:
            file_name = "contact_sensitivity_medium.wav"
        elif self is GuardedContactSensitivity.HIGH_SENSITIVITY_NAV:
            file_name = "contact_sensitivity_high_nav.wav"
        elif self is GuardedContactSensitivity.HIGH_SENSITIVITY_MANIPULATION:
            file_name = "contact_sensitivity_high_manipulation.wav"
        elif self is GuardedContactSensitivity.STRONG_MANIPULATION:
            file_name = "contact_sensitivity_strong_manipulation.wav"
        else:
            raise NotImplementedError(f"No sound file for {self}")
        
        play_sound(get_sounds_dir()+f'/{file_name}')

    def get_name(self):
        """Get the name mapping that works with Stretch Body"""
        if self == GuardedContactSensitivity.MEDIUM:
            return "default"
        return self.name.lower()
    
    def apply(self, robot):
        robot.set_guarded_contact_sensitivity(self.get_name())



class MotionProfile(Enum):
    """
    Name of the motion profile as defined in robot_params_SE4. (e.g. default, slow, fast, max)
    """

    SLOW = 1 # The ordering here is important for get_one_lower_speed()
    MEDIUM = 2 # This is called 'default' in Stretch Body
    FAST = 3
    MAX = 4


    def get_name(self):
        """Get the name mapping that works with Stretch Body"""
        if self == MotionProfile.MEDIUM:
            return "default"
        return self.name.lower()
    
    def get_one_lower_speed(self):
        if self is MotionProfile.SLOW:
            return self
        lower_speed = self.cycle(is_forward=False, use_cyclable_options=False)
        return lower_speed
    
    def _get_cycleable_options(self):
        # return list(type(self))
        # return [MotionProfile.DEFAULT, MotionProfile.SLOW, MotionProfile.FAST, MotionProfile.MAX]
        return [MotionProfile.SLOW, MotionProfile.MEDIUM, MotionProfile.FAST]

    def cycle(self, is_forward:bool, use_cyclable_options:bool=True):
        index_offset = 1 if is_forward else -1

        members = self._get_cycleable_options() if use_cyclable_options else list(type(self))
        index = members.index(self)
        return members[(index + index_offset) % len(members)]

    def play_sound_file(self):
        file_name:str
        if self is MotionProfile.MEDIUM:
            file_name = "motion_profile_medium.wav"
        elif self is MotionProfile.SLOW:
            file_name = "motion_profile_slow.wav"
        elif self is MotionProfile.FAST:
            file_name = "motion_profile_fast.wav"
        elif self is MotionProfile.MAX:
            file_name = "motion_profile_max.wav"
        else:
            raise NotImplementedError(f"No sound file for {self}")
        
        play_sound(get_sounds_dir()+f'/{file_name}')
