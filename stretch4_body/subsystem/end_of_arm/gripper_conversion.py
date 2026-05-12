#!/usr/bin/env python3
import math
from stretch4_body.core.robot_params import RobotParams
import importlib
def map_range(value:float, in_min:float, in_max:float, out_min:float, out_max:float):
    """
    Linearly maps a value from one range to another.
    
    Parameters:
        value   : number to map
        in_min  : lower bound of the input range
        in_max  : upper bound of the input range
        out_min : lower bound of the output range
        out_max : upper bound of the output range

    Returns:
        Mapped value in the new range.
    """
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def get_angle_from_chord_length_and_radius(radius_m, chord_m):
    return 2 * math.asin(chord_m / (2 * radius_m))   # radians

def get_chord_from_radius_and_angle(radius_m, angle_rad):
    return 2 * radius_m * math.sin(angle_rad / 2)


"""
This is the gripper_conversion library moved from stretch4_ros2
"""

def get_gripper_params():
    rp=RobotParams()._robot_params
    eoa_name = rp['robot']['tool']
    module_name = rp[eoa_name]['py_module_name']
    class_name = rp[eoa_name]['py_class_name']
    eoa = getattr(importlib.import_module(module_name), class_name)()
    if 'stretch_gripper' in list(eoa.params.get('devices', {}).keys()):
        module_name = eoa.params['devices']['stretch_gripper']['py_module_name']
        class_name = eoa.params['devices']['stretch_gripper']['py_class_name']
        gripper= getattr(importlib.import_module(module_name), class_name)()
        return gripper.params
    return None

class GripperConversion():
    """
    This class models the SG4 by using simple linear mappings from servo angles to geometric aperture length and angles.
    Note: This is a simplified model, it is not accurate to the real motion of the gripper.
    Note: `aperture_open_m` and `finger_length_m` are defined in robot_params_SE4.py.
    """
    def __init__(self,gripper_params=None):
        if gripper_params is None: #Allow to load params based on unknown tool type
            gripper_params=get_gripper_params()
        self.params=gripper_params['gripper_conversion']
        self.params['servo_open_angle']=gripper_params['range_deg'][1]
        self.params['servo_closed_angle']=gripper_params['range_deg'][0]
        aperture_open_rad = get_angle_from_chord_length_and_radius(self.params['finger_length_m'], self.params['aperture_open_m'])
        self.aperture_open_deg = math.degrees(aperture_open_rad)
        self.aperture_close_deg = 0.0

        self.servo_to_aperture_slope = ((self.params['aperture_open_m'] - self.params['aperture_closed_m']) / (self.aperture_open_deg - self.aperture_close_deg))

    def servo_angle_degrees_to_aperture_angle_degrees(self, servo_angle_degrees):
        return map_range(servo_angle_degrees, self.params['servo_closed_angle'], self.params['servo_open_angle'], self.aperture_close_deg, self.aperture_open_deg)
    
    def servo_angle_degrees_to_aperture_m(self, servo_angle_degrees):
        aperature_angle = self.servo_angle_degrees_to_aperture_angle_degrees(servo_angle_degrees)
        aperature_angle = math.radians(aperature_angle)
        return get_chord_from_radius_and_angle(radius_m=self.params['finger_length_m'], angle_rad=aperature_angle)
    
    def aperture_angle_degrees_to_servo_angle_degrees(self, aperture_angle_degrees): # returns degrees
        return map_range(aperture_angle_degrees, self.aperture_close_deg, self.aperture_open_deg, self.params['servo_closed_angle'], self.params['servo_open_angle'])

    def aperture_m_to_aperture_angle_degrees(self, aperture_m):
        return math.degrees(get_angle_from_chord_length_and_radius(self.params['finger_length_m'], aperture_m))

    def aperture_m_to_servo_angle_degrees(self, aperture_m):
        aperture_angle = self.aperture_m_to_aperture_angle_degrees(aperture_m)
        return self.aperture_angle_degrees_to_servo_angle_degrees(aperture_angle_degrees=aperture_angle)
    

    # Keeping these for backward compatibility:
    # aperture -> aperture_m
    # finger -> actual finger/aperture angle
    def servo_to_aperture(self, servo_in):
        return self.servo_angle_degrees_to_aperture_m(servo_in)

    def aperture_to_servo(self, aperture_m):
        return self.aperture_m_to_servo_angle_degrees(aperture_m)

    def finger_to_servo(self, finger_ang_rad):
        return self.aperture_angle_degrees_to_servo_angle_degrees(math.degrees(finger_ang_rad))

    def servo_to_finger(self, servo_pct):
        finger_rad = math.radians(self.servo_angle_degrees_to_aperture_angle_degrees(servo_pct))
        return finger_rad

    def status_to_all(self, gripper_status):
        aperture_m = self.servo_to_aperture(gripper_status['pos_pct'])
        finger_rad = math.radians(self.aperture_m_to_aperture_angle_degrees(aperture_m)) / 2.0
        finger_effort = gripper_status['effort']
        finger_vel = (self.servo_to_aperture_slope * gripper_status['vel'])/2.0
        return aperture_m, finger_rad, finger_effort, finger_vel

    def get_status(self, gripper_status):
        aperture_m = self.servo_to_aperture(gripper_status['pos_pct']) 
        finger_rad = math.radians(self.aperture_m_to_aperture_angle_degrees(aperture_m))  / 2.0
        finger_effort = gripper_status['effort']
        finger_vel = (self.servo_to_aperture_slope * gripper_status['vel'])/2.0
        sts = {'aperture_m':aperture_m,
               'finger_rad':finger_rad,
               'finger_effort':finger_effort,
               'finger_vel':finger_vel}
        
        return sts

if __name__ == "__main__":
    conversion = GripperConversion()

    value = conversion.servo_angle_degrees_to_aperture_angle_degrees(conversion.params['servo_closed_angle'])
    expected = 0.0 #radians
    assert math.isclose(value, expected, abs_tol=0.001), f"Expected aperture close angle to be {expected}, got {value}"
    value = conversion.servo_angle_degrees_to_urdf_angle_radians(conversion.params['servo_closed_angle'])
    expected = -0.5 #radians
    assert math.isclose(value, expected, abs_tol=0.001), f"Expected aperture close angle to be {expected}, got {value}"

    value = math.radians(conversion.servo_angle_degrees_to_aperture_angle_degrees(conversion.params['servo_open_angle']))
    expected = 1.015 #radians
    assert math.isclose(value, expected, abs_tol=0.001), f"Expected aperture open angle to be {expected}, got {value}"
    value = conversion.servo_angle_degrees_to_urdf_angle_radians(conversion.params['servo_open_angle'])
    expected = 0.0 #radians
    assert math.isclose(value, expected, abs_tol=0.001), f"Expected aperture open angle to be {expected}, got {value}"