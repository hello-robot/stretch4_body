#!/usr/bin/env python3
import math
from stretch4_body.core.robot_params import RobotParams
import importlib
from stretch4_urdf import get_urdf, get_joint_limits
from functools import lru_cache

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


def parallel_gripper_servo_rad_to_mm(servo_rad, params):
    """
    Convert parallel gripper servo angle (in radians) to gap width (in mm).
    """
    # L: Length of the connecting linkage rod (in mm)
    L = params.get('kL', 30.25)
    # r: Radius of rotation of the servo horn pivot (in mm)
    r = params.get('kR', 22.0)
    # finger_offset: Horizontal distance from slider pivot to the fingertip contact face (in mm)
    finger_offset = params.get('kX0', 10.5)
    # kT0: Angular offset aligning the servo zero coordinate with the kinematic reference frame (in degrees)
    kT0 = params.get('kT0', 44.0)
    
    # q_eff: Effective angle of the servo arm relative to the vertical axis
    q_eff = -1 * servo_rad + math.radians(kT0)
    # term: The squared horizontal distance spanned by the connecting rod (derived via Pythagorean theorem)
    term = L**2 - (r * math.cos(q_eff))**2
    # x_pivot: Horizontal position of the slider pivot relative to the motor axis center (in mm)
    x_pivot = r * math.sin(q_eff) - math.sqrt(term)
    # x_mm: Combined gap width between both fingers (twice the distance from slider to contact face)
    x_mm = 2 * (-x_pivot - finger_offset)
    return round(x_mm, 3)


def parallel_gripper_mm_to_servo_rad(x_mm, params):
    """
    Convert parallel gripper gap width (in mm) to servo angle (in radians).
    """
    # L: Length of the connecting linkage rod (in mm)
    L = params.get('kL', 30.25)
    # r: Radius of rotation of the servo horn pivot (in mm)
    r = params.get('kR', 22.0)
    # finger_offset: Horizontal distance from slider pivot to the fingertip contact face (in mm)
    finger_offset = params.get('kX0', 10.5)
    # kT0_rad: Angular offset in radians aligning the servo zero coordinate with the kinematic reference frame
    kT0_rad = math.radians(params.get('kT0', 44.0))
    
    # A: The horizontal position of the slider pivot relative to the motor axis center (in mm)
    A = -(x_mm / 2.0 + finger_offset)
    # numerator: Derived from squaring the linkage geometry equation to isolate sin(q_eff)
    numerator = A**2 + r**2 - L**2
    # denominator: Twice the product of slider distance and crank radius
    denominator = 2 * A * r
    
    # sin_q_eff: The sine of the effective servo arm angle
    sin_q_eff = numerator / denominator
    # Clamp to [-1.0, 1.0] to prevent floating point out-of-bounds domain errors in arcsin
    sin_q_eff = max(-1.0, min(1.0, sin_q_eff))
    # q_eff: Effective servo arm angle in radians
    q_eff = math.asin(sin_q_eff)
    # qr: Rescaled physical servo angle in radians
    qr = kT0_rad - q_eff
    return qr



@lru_cache(maxsize=1)
def get_finger_joint_limits():
    """
    Get the lower and upper limits of finger_left_joint from the URDF contents.
    """    

    rp = RobotParams().get_params()[1]
    model_name = rp['robot']['model_name']
    batch_name = rp['robot']['batch_name']
    eoa_name = rp['robot']['tool']
    
    urdf_contents = get_urdf(model_name, batch_name, eoa_name, do_add_file_prefix_to_absolute_paths=False)
    limits = get_joint_limits(urdf_contents)
    return limits.get('finger_left_joint', (-0.04, 0.0))



def parallel_gripper_pos_mm_to_urdf_m(pos_mm, params):
    """
    Convert parallel gripper finger aperture (in mm) to URDF finger slide joint value (in meters).
    Slide joint limits are loaded dynamically from the URDF.
    """
    lower, upper = get_finger_joint_limits()
    range_mm = params.get('range_mm', 80.0)
    pct = pos_mm / range_mm
    return upper + pct * (lower - upper)


def parallel_gripper_rad_to_urdf_m(qr, params):
    """
    Convert parallel gripper servo angle (in radians) to URDF finger slide joint value (in meters).
    """
    x_mm = parallel_gripper_servo_rad_to_mm(qr, params)
    return parallel_gripper_pos_mm_to_urdf_m(x_mm, params)


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