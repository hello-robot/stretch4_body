#!/usr/bin/env python3

from typing import Any, TypedDict

from stretch4_body.core.transport.transport import *
from stretch4_body.core.device import Device
from stretch4_body.core.hello_utils import *
import textwrap
import psutil
import time
import array as arr
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
# ######################## POWER_PERIPH #################################

"""
The POWER_PERIPH is the power and IMU Arduino board in the base
"""


class IMUBase(Device):
    """
    API to the Stretch IMU found in the base
    """

    def __init__(self):
        Device.__init__(self, 'imu', req_params=False)
        # pitch //-180 to 180, rolls over
        # roll //-90 to  90, rolls over at 180
        # heading //0-360.0, rolls over
        self.status: "IMUBaseStatus" = {'ax': 0, 'ay': 0, 'az': 0, 'gx': 0, 'gy': 0, 'gz': 0, 'mx': 0, 'my': 0, 'mz': 0, 'roll': 0,
                       'pitch': 0, 'heading': 0, 'timestamp': 0, 'qw': 0, 'qx': 0, 'qy': 0, 'qz': 0, 'bump': 0,
                       'gravity_tilt':0}
        self.config = self.params['config']
        self._dirty_config = True

    def get_status(self):
        s = self.status.copy()
        return s

    def get_quaternion(self):
        return [self.status['qw'], self.status['qx'], self.status['qy'], self.status['qz']]

    # ####################################################

    def pretty_print(self):
        print('----------IMU -------------')
        print('AX (m/s^2)', self.status['ax'])
        print('AY (m/s^2)', self.status['ay'])
        print('AZ (m/s^2)', self.status['az'])
        print('GX (rad/s)', self.status['gx'])
        print('GY (rad/s)', self.status['gy'])
        print('GZ (rad/s)', self.status['gz'])
        print('MX (uTesla)', self.status['mx'])
        print('MY (uTesla)', self.status['my'])
        print('MZ (uTesla)', self.status['mz'])
        print('QW', self.status['qw'])
        print('QX', self.status['qx'])
        print('QY', self.status['qy'])
        print('QZ', self.status['qz'])


        #[yaw, pitch, roll]=self.quat_to_ypr(q)

        print('Roll (deg)', rad_to_deg(self.status['roll']))
        print('Pitch (deg)', rad_to_deg(self.status['pitch']))
        print('Heading (deg)', rad_to_deg(self.status['heading']))

        print('Gravity tilt (deg)',rad_to_deg(self.status['gravity_tilt']))
        print('Bump', self.status['bump'])
        print('Timestamp (s)', self.status['timestamp'])
        print('-----------------------')


    def _pack_config(self, s, sidx):
        for i in range(3):
            pack_float_t(s, sidx, self.config['mag_offsets'][i])
            sidx += 4
        for i in range(9):
            pack_float_t(s, sidx, self.config['mag_softiron_matrix'][i])
            sidx += 4
        for i in range(3):
            pack_float_t(s, sidx, self.config['gyro_zero_offsets'][i])
            sidx += 4
        pack_float_t(s, sidx, self.config['rate_gyro_vector_scale']);
        sidx += 4
        pack_float_t(s, sidx, self.config['gravity_vector_scale']);
        sidx += 4
        pack_float_t(s, sidx, self.config['accel_LPF'])
        sidx += 4
        return sidx

    def calculate_tilt_angle(self,q_wxyz):
        """
        Calculates the tilt angle of a system represented by a quaternion
        relative to the Z-axis (direction of gravity).

        The tilt angle is the angle between the system's local Z-axis
        and the World Z-axis (gravity vector).

        Args:
            q_wxyz (array-like): The IMU quaternion in (w, x, y, z) format.

        Returns:
            float: The tilt angle in degrees (0 to 180).
        """
        # 1. Convert the quaternion to a Rotation object
        # Scipy expects (x, y, z, w) format, so reorder the input if necessary.
        # Note: If your input is already in (x, y, z, w), use 'xyzw' instead of 'wxyz'.
        q_xyzw = np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]])
        try:
            rotation = R.from_quat(q_xyzw)
        except ValueError:
            return 0 #Hack, after power up takes a few cycles for quat data to be valid. Need to debug.

        # 2. Define the Local Z-axis
        # This is the axis that points in the direction of gravity *when the robot is not tilted*
        local_z_axis = np.array([0, 0, 1])

        # 3. Rotate the Local Z-axis to World Space
        # This vector (the "World Z Vector") now points in the direction of the robot's "down"
        # in the world frame.
        world_z_vector = rotation.apply(local_z_axis)

        # 4. Define the World Gravity Vector
        # This is the actual direction of gravity in the world frame (World Z-axis)
        gravity_vector = np.array([0, 0, 1])

        # 5. Calculate the Angle between the two vectors
        # We use the dot product formula: cos(theta) = (A dot B) / (||A|| * ||B||)
        # Since both vectors are unit vectors, this simplifies to cos(theta) = A dot B.

        dot_product = np.dot(world_z_vector, gravity_vector)

        # Clip the dot product to the valid range [-1, 1] for arccos due to
        # potential floating-point errors
        dot_product = np.clip(dot_product, -1.0, 1.0)

        # Calculate the angle in radians, then convert to degrees
        angle_rad = np.arccos(dot_product)
        angle_deg = np.rad2deg(angle_rad)

        return angle_rad

    def quat_to_ypr(self,q):
        yaw = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])
        pitch = -math.asin(2.0 * (q[1] * q[3] - q[0] * q[2]))
        roll = math.atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3])
        pitch *= 180.0 / math.pi
        yaw *= 180.0 / math.pi
        roll *= 180.0 / math.pi
        return [yaw, pitch, roll]

    def _unpack_status(self, s, unpack_to=None):
        if unpack_to is None:
            unpack_to = self.status
        sidx = 0
        unpack_to['ax'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['ay'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['az'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['gx'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['gy'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['gz'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['mx'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['my'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['mz'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['roll'] = deg_to_rad(unpack_float_t(s[sidx:]))
        sidx += 4
        unpack_to['pitch'] = deg_to_rad(unpack_float_t(s[sidx:]))
        sidx += 4
        unpack_to['heading'] = deg_to_rad(unpack_float_t(s[sidx:]))
        sidx += 4
        unpack_to['qw'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['qx'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['qy'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['qz'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['bump'] = unpack_float_t(s[sidx:])
        sidx += 4
        # self.status['timestamp'] = self.timestamp.set(unpack_uint32_t(s[sidx:]))
        # sidx += 4

        # Compute IMU fields
        unpack_to['gravity_tilt']=(
            self.calculate_tilt_angle([self.status['qw'],self.status['qx'],self.status['qy'],self.status['qz']]))

        return sidx


class IMU_Protocol_P8(IMUBase):
    def foo(self):
        pass
# ######################## IMU #################################
class IMU(IMUBase):
    def __init__(self):
        IMUBase.__init__(self)
        # Order in descending order so more recent protocols/methods override less recent
        self._supported_protocols = {'p9': (IMU_Protocol_P8,),'p8': (IMU_Protocol_P8,),
                                     'p10': (IMU_Protocol_P8,),'p11':(IMU_Protocol_P8,),'p12':(IMU_Protocol_P8,),'p13':(IMU_Protocol_P8,)}

# ##################################################################################
class PowerPeriphDefn():
    """
    API to the Stretch Power and IMU board (PowerPeriph)
    """
    def __init__(self):
        pass
    RPC_SET_POWER_PERIPH_CONFIG = 1
    RPC_REPLY_POWER_PERIPH_CONFIG = 2
    RPC_GET_POWER_PERIPH_STATUS = 3
    RPC_REPLY_POWER_PERIPH_STATUS = 4
    RPC_SET_POWER_PERIPH_TRIGGER = 5
    RPC_REPLY_POWER_PERIPH_TRIGGER = 6
    RPC_GET_POWER_PERIPH_BOARD_INFO = 7
    RPC_REPLY_POWER_PERIPH_BOARD_INFO = 8
    RPC_SET_MOTOR_SYNC = 9
    RPC_REPLY_MOTOR_SYNC = 10
    RPC_READ_TRACE = 11
    RPC_REPLY_READ_TRACE = 12
    RPC_GET_POWER_PERIPH_STATUS_AUX = 13
    RPC_REPLY_POWER_PERIPH_STATUS_AUX = 14
    RPC_LOAD_TEST_PULL = 15
    RPC_REPLY_LOAD_TEST_PULL = 16
    RPC_LOAD_TEST_PUSH = 17
    RPC_REPLY_LOAD_TEST_PUSH = 18
    RPC_ACTUATOR_ENABLE = 19
    RPC_REPLY_ACTUATOR_ENABLE = 20
    RPC_ACTUATOR_DISABLE = 21
    RPC_REPLY_ACTUATOR_DISABLE = 22
    RPC_NETWORK_INFO = 23
    RPC_REPLY_NETWORK_INFO = 24
    RPC_FIREBASE_DB_INFO = 25
    RPC_FIREBASE_DB_INFO_REPLY = 26
    RPC_SET_IMU_CONFIG =27
    RPC_SET_IMU_CONFIG_REPLY=28

    RPC_SET_EYE_ANIMATION = 29
    RPC_REPLY_SET_EYE_ANIMATION = 30

    EYE_ANIM_NOP = 0
    EYE_ANIM_OFF = 1
    EYE_ANIM_IDLE_GLOW = 2
    EYE_ANIM_BLINK = 3
    EYE_ANIM_LOOK_LEFT = 4
    EYE_ANIM_LOOK_RIGHT = 5
    EYE_ANIM_RAINBOW_SPIN = 6
    EYE_ANIM_ALERT = 7
    EYE_ANIM_HAPPY = 8
    EYE_ANIM_LEFT_HALF = 9
    EYE_ANIM_RIGHT_HALF = 10
    EYE_ANIM_TOP_HALF = 11
    EYE_ANIM_BOTTOM_HALF = 12
    EYE_ANIM_CIRCLE_CW = 13
    EYE_ANIM_CIRCLE_CCW = 14
    EYE_ANIM_COUNT = 15

    EYE_ANIM_NAME_TO_IDX = {
        'NOP': EYE_ANIM_NOP,
        'OFF': EYE_ANIM_OFF,
        'IDLE_GLOW': EYE_ANIM_IDLE_GLOW,
        'BLINK': EYE_ANIM_BLINK,
        'LOOK_LEFT': EYE_ANIM_LOOK_LEFT,
        'LOOK_RIGHT': EYE_ANIM_LOOK_RIGHT,
        'RAINBOW_SPIN': EYE_ANIM_RAINBOW_SPIN,
        'ALERT': EYE_ANIM_ALERT,
        'HAPPY': EYE_ANIM_HAPPY,
        'LEFT_HALF': EYE_ANIM_LEFT_HALF,
        'RIGHT_HALF': EYE_ANIM_RIGHT_HALF,
        'TOP_HALF': EYE_ANIM_TOP_HALF,
        'BOTTOM_HALF': EYE_ANIM_BOTTOM_HALF,
        'CIRCLE_CW': EYE_ANIM_CIRCLE_CW,
        'CIRCLE_CCW': EYE_ANIM_CIRCLE_CCW,
    }

    STATE_RUNSTOP_EVENT = 16
    STATE_FAN_ON = 64
    STATE_BUZZER_ON = 128
    STATE_LOW_SOC_ALERT = 256
    STATE_OVER_TILT_ALERT = 512
    STATE_HIGH_CURRENT_ALERT = 1024
    STATE_ADAPTER_VOLTAGE_PRESENT = 2048
    STATE_BOOT_DETECTED = 4096
    STATE_IS_TRACE_ON = 8192
    STATE_IS_CHARGER_CHARGING = 16384
    STATE_CONNECTED_TO_NETWORK = (1 << 15)
    STATE_CONNECTED_TO_FIREBASE = (1 << 16)
    STATE_HIGH_CURRENT_EOA_ALERT = (1<<17)
    STATE_ADAPTER_CONNECTED = (1<<18)
    STATE_ADAPTER_FAULT = (1<<19)

    TRIGGER_BOARD_RESET = 1
    TRIGGER_RUNSTOP_RESET = 2
    TRIGGER_BUZZER_ON = 8
    TRIGGER_BUZZER_OFF = 16
    TRIGGER_FAN_ON = 32
    TRIGGER_FAN_OFF = 64
    TRIGGER_IMU_RESET = 128
    TRIGGER_RUNSTOP_ON = 256
    TRIGGER_BEEP = 512
    TRIGGER_LIGHTBAR_TEST = 1024
    TRIGGER_ENABLE_TRACE = 2048
    TRIGGER_DISABLE_TRACE = 4096
    TRIGGER_CHARGER_ON = 8192
    TRIGGER_CHARGER_OFF = 16384
    TRIGGER_ESP_FW_UPDATE = 32768
    TRIGGER_ESP_RESET = 65536
    TRIGGER_LIDAR_OFF = (1 << 17)
    TRIGGER_LIDAR_ON = (1 << 18)
    TRIGGER_20V0_AUX_OFF = (1 << 19)
    TRIGGER_20V0_AUX_ON = (1 << 20)
    TRIGGER_CPU_PWR_CYCLE = (1 << 21)
    TRIGGER_ESP32_STATUS_PRINT = (1 << 22)
    TRIGGER_SLEEP = (1 << 23)

    TRACE_TYPE_STATUS = 0
    TRACE_TYPE_DEBUG = 1
    TRACE_TYPE_PRINT = 2

# ##################################################################################

class PowerPeriphPowerMgmt(PowerPeriphDefn):
    def __init__(self):
        pass
    def get_voltage(self, raw):
        raw_to_V = 3.3 * 11 / 4095  # 10bit adc, 0-20V per 0-3.3V reading
        if int(self.board_info['protocol_version'][1:]) >= 7:
            return raw
        return raw * raw_to_V

    def set_charger_on(self):
        self._trigger = self._trigger | self.TRIGGER_CHARGER_ON
        self._dirty_trigger = True

    def set_charger_off(self):
        self._trigger = self._trigger | self.TRIGGER_CHARGER_OFF
        self._dirty_trigger = True

    def get_current(self, raw):
        if int(self.board_info['protocol_version'][1:]) >= 7:
            return raw
        if self.board_info['hardware_id'] >= 3:
            return self.get_current_efuse(raw)
        else:
            return self.get_current_shunt(raw)

    def get_current_shunt(self, raw):
        """
        RE1 / RE2 PowerPeriphs using shunt resistor for current measurement
        """
        raw_to_mV = 3300 / 1024.0
        mV = raw * raw_to_mV
        mA = mV / .408  # conversion per circuit
        return mA / 1000.0

    def get_current_efuse(self, raw):
        """
        S3 PowerPeriph's using Efuse current measurement
        """
        A = 0.004118832 * raw
        return A

    def get_current_charge(self, raw):
        """
        S3 PowerPeriph's shunt measurement of charger current
        """

        raw_to_mV = 3300 / 4095.0
        mV = raw * raw_to_mV
        mA = mV / .215  # conversion per circuit
        if int(self.board_info['protocol_version'][1:]) >= 7:
            return raw
        return mA / 1000.0

class PowerPeriphPeriphControl(PowerPeriphDefn):
    def __init__(self,event_reset):
        self._fan_on_last = False
        # Reset POWER_PERIPH state so that Ctrl-C and re-instantiate PowerPeriph class is efficient way to get out of an event
        if event_reset:
            self.clear_runstop()
        self._ts_last_fan_on = None

    def get_temp(self, raw):
        raw_to_mV = 3300 / 4095
        T = (raw_to_mV * raw - 500) / 10
        if int(self.board_info['protocol_version'][1:]) >= 7:
            return raw
        return T

    def imu_reset(self):
        self._trigger = self._trigger | self.TRIGGER_IMU_RESET
        self._dirty_trigger = True

    def set_esp_fw_update(self):
        self._trigger = self._trigger | self.TRIGGER_ESP_FW_UPDATE
        self._dirty_trigger = True

    def set_esp_reset(self):
        self._trigger = self._trigger | self.TRIGGER_ESP_RESET
        self._dirty_trigger = True

    def set_esp_status_print(self):
        self._trigger = self._trigger | self.TRIGGER_ESP32_STATUS_PRINT
        self._dirty_trigger = True

    def trigger_sleep(self):
        print('Putting robot to sleep!!')
        self._trigger = self._trigger | self.TRIGGER_SLEEP
        self._dirty_trigger = True

    def clear_runstop(self):
        """
        Reset the robot runstop, allowing motion to continue
        """
        self._trigger = self._trigger | self.TRIGGER_RUNSTOP_RESET
        self._dirty_trigger = True

    def trigger_runstop(self):
        """
        Trigger the robot runstop, stopping motion
        """
        self._trigger = self._trigger | self.TRIGGER_RUNSTOP_ON
        self._dirty_trigger = True

    def set_fan_on(self):
        self._trigger = self._trigger | self.TRIGGER_FAN_ON
        self._dirty_trigger = True

    def set_fan_off(self):
        self._trigger = self._trigger | self.TRIGGER_FAN_OFF
        self._dirty_trigger = True

    def set_buzzer_on(self):
        self._trigger = self._trigger | self.TRIGGER_BUZZER_ON
        self._dirty_trigger = True

    def set_buzzer_off(self):
        self._trigger = self._trigger | self.TRIGGER_BUZZER_OFF
        self._dirty_trigger = True

    def trigger_beep(self):
        """
        Generate a single short beep
        """
        self._trigger = self._trigger | self.TRIGGER_BEEP
        self._dirty_trigger = True

    def trigger_lightbar_test(self):
        self._trigger = self._trigger | self.TRIGGER_LIGHTBAR_TEST
        self._dirty_trigger = True

    def set_lidar_off(self):
        self._trigger = self._trigger | self.TRIGGER_LIDAR_OFF
        self._dirty_trigger = True

    def set_lidar_on(self):
        self._trigger = self._trigger | self.TRIGGER_LIDAR_ON
        self._dirty_trigger = True

    def set_aux_cpu_off(self):
        self._trigger = self._trigger | self.TRIGGER_20V0_AUX_OFF
        self._dirty_trigger = True

    def set_aux_cpu_on(self):
        self._trigger = self._trigger | self.TRIGGER_20V0_AUX_ON
        self._dirty_trigger = True

    def power_cycle_cpu(self):
        self._trigger = self._trigger | self.TRIGGER_CPU_PWR_CYCLE
        self._dirty_trigger = True

    def set_eye_animation(self, left_idx=None, right_idx=None, intensity=255, r=255, g=255, b=255):
        if left_idx is not None:
            self._eye_animation_left = left_idx
        else:
            self._eye_animation_left = 0
        if right_idx is not None:
            self._eye_animation_right = right_idx
        else:
            self._eye_animation_right = 0   
        self._eye_animation_intensity = intensity
        self._eye_animation_r = r
        self._eye_animation_g = g
        self._eye_animation_b = b
        self._dirty_eye_animation = True

    def actuator_control(self, motor_type, enable, blocking=True):
        mt = [None, 'lift', 'omni-0', 'omni-1', 'omni-2', 'arm', 'eoa']
        for i in range(0, len(mt)):
            if motor_type == mt[i]:
                motor_type = i
                break
        if not isinstance(motor_type, int) or (motor_type < 0 or motor_type > 6):
            print("Error Unrecoginzed Motor Type")
            return
        if enable:
            payload = arr.array('B', [self.RPC_ACTUATOR_ENABLE, motor_type])
            return self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload,
                                         rpc_callback=self._rpc_actuator_enable_reply) is not None
        else:
            payload = arr.array('B', [self.RPC_ACTUATOR_DISABLE, motor_type])
            return self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload,
                                         rpc_callback=self._rpc_actuator_disable_reply) is not None

    def _rpc_actuator_enable_reply(self, reply):
        if reply[0] != self.RPC_REPLY_ACTUATOR_ENABLE:
            print('Error RPC_REPLY_ACTUATOR_CONTROL', reply[0])

    def _rpc_actuator_disable_reply(self, reply):
        if reply[0] != self.RPC_REPLY_ACTUATOR_DISABLE:
            print('Error RPC_REPLY_ACTUATOR_CONTROL', reply[0])




    # ###################### Firebase Methods ###########################3
    def connect_to_firebase(self,timeout=10.0):
        #This likely will need some work to be robust to initial conditions, drop-outs, etc.
        #Nominal example for now
        #Will block server...
        print('Connecting to Firebase')
        ts=time.time()

        if not self.status['connected_to_network']:
            self.send_network_info()
            self.push_command()
        while not self.status['connected_to_network'] and time.time()-ts<timeout:
            self.pull_status() #Todo: make server loop safe.
        if not self.status['connected_to_network']:
            print('Failed to connect to Wifi network')
            return False

        if not self.status['connected_to_firebase']:
            self.send_firebase_info()
            self.push_command()
        while not self.status['connected_to_firebase'] and time.time() - ts < timeout:
            self.pull_status()  # Todo: make server loop safe.
        if not self.status['connected_to_firebase']:
            print('Failed to connect to Firbase')
            return False

        return True

    def send_firebase_info(self, fb_host_url=None, fb_api_key=None, fb_user_email=None, fb_user_password=None):
        if fb_host_url is None:
            fb_host_url=self.params['firebase']['url']
        if fb_api_key is None:
            fb_api_key=self.params['firebase']['api_key']
        if fb_user_email is None:
            fb_user_email=self.params['firebase']['user_email']
        if fb_user_password is None:
            fb_user_password=self.params['firebase']['user_password']
        if fb_host_url=='NA' or fb_api_key=='NA' or fb_user_email=='NA' or fb_user_password=='NA':
            print('Invalid credentials for Firebase connection',fb_host_url,fb_api_key,fb_user_email,fb_user_password)
            return

        fb_host_url = fb_host_url.encode('utf-8').ljust(250, b'\x00')
        fb_api_key = fb_api_key.encode('utf-8').ljust(128, b'\x00')
        fb_user_email = fb_user_email.encode('utf-8').ljust(250, b'\x00')
        fb_user_password=fb_user_password.encode('utf-8').ljust(64, b'\x00')

        payload = self.transport.get_empty_payload()
        payload[0] = self.RPC_FIREBASE_DB_INFO
        sidx = 1
        string_data = [fb_host_url, fb_api_key, fb_user_email, fb_user_password]

        for i in string_data:
            pack_string_t(s=payload, sidx=sidx, x=i);
            sidx += len(i)
        self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_firebase_info_reply)

    def send_network_info(self, ssid=None, password=None):
        if ssid is None:
            ssid=self.params['firebase']['network_ssid']
        if password is None:
            password=self.params['firebase']['network_password']

        if ssid=='NA' or password=='NA':
            print('Invalid SSID/Password for ESP32 Wifi',ssid,password)
            return

        ssid = ssid.encode('utf-8').ljust(32, b'\x00')
        password = password.encode('utf-8').ljust(64, b'\x00')
        payload = self.transport.get_empty_payload()
        payload[0] = self.RPC_NETWORK_INFO
        sidx = 1
        pack_string_t(s=payload, sidx=sidx, x=ssid);
        sidx += len(ssid)
        pack_string_t(s=payload, sidx=sidx, x=password);
        sidx += len(password)
        return self.transport.do_rpc(blocking=True,is_push=True,payload=payload[:sidx],rpc_callback=self.rpc_network_info_reply) is not None

    def rpc_network_info_reply(self, reply):
        if reply[0] != self.RPC_REPLY_NETWORK_INFO:
            print('Error RPC_REPLY_NETWORK_INFO', reply[0])

    def rpc_firebase_info_reply(self, reply):
        if reply[0] != self.RPC_FIREBASE_DB_INFO_REPLY:
            print('Error RPC_FIREBASE_DB_INFO_REPLY', reply[0])

class PowerPeriphTrace(PowerPeriphDefn):
    def __init__(self):
        pass

    def enable_firmware_trace(self):
        self._trigger = self._trigger | self.TRIGGER_ENABLE_TRACE
        self._dirty_trigger = True

    def disable_firmware_trace(self):
        self._trigger = self._trigger | self.TRIGGER_DISABLE_TRACE
        self._dirty_trigger = True
        
    def read_firmware_trace(self):
        self.trace_buf = []
        self.timestamp.reset()  # Timestamp holds state, reset within lock to avoid threading issues
        self.n_trace_read = 1
        ts = time.time()
        payload = arr.array('B', [self.RPC_READ_TRACE])
        while (self.n_trace_read) and time.time() - ts < 60.0:
            #Only support blocking
            self.transport.do_rpc(blocking=True, is_push=False, payload=payload,rpc_callback=self._rpc_read_firmware_trace_reply)
            time.sleep(.001)
        return self.trace_buf

    def _unpack_debug_trace(self, s, unpack_to):
        sidx = 0
        unpack_to['u8_1'] = unpack_uint8_t(s[sidx:])
        sidx += 1
        unpack_to['u8_2'] = unpack_uint8_t(s[sidx:])
        sidx += 1
        unpack_to['f_1'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['f_2'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['f_3'] = unpack_float_t(s[sidx:])
        sidx += 4
        return sidx

    def _unpack_print_trace(self, s, unpack_to):
        sidx = 0
        line_len = 32
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:]))
        sidx += 8
        unpack_to['line'] = unpack_string_t(s[sidx:], line_len)
        sidx += line_len
        unpack_to['x'] = unpack_float_t(s[sidx:])
        sidx += 4
        return sidx

    def _rpc_read_firmware_trace_reply(self, reply):
        if len(reply) > 0 and reply[0] == self.RPC_REPLY_READ_TRACE:
            self.n_trace_read = reply[1]
            self.trace_buf.append({'id': len(self.trace_buf), 'status': {}, 'debug': {}, 'print': {}})
            if reply[2] == self.TRACE_TYPE_STATUS:
                self.trace_buf[-1]['status'] = self.status_zero.copy()
                self._unpack_status(reply[3:], unpack_to=self.trace_buf[-1]['status'])
            elif reply[2] == self.TRACE_TYPE_DEBUG:
                self._unpack_debug_trace(reply[3:], unpack_to=self.trace_buf[-1]['debug'])
            elif reply[2] == self.TRACE_TYPE_PRINT:
                self._unpack_print_trace(reply[3:], unpack_to=self.trace_buf[-1]['print'])
            else:
                print('Unrecognized trace type %d' % reply[2])
        else:
            print('Error RPC_REPLY_READ_TRACE')
            self.n_trace_read = 0
            self.trace_buf = []


class PowerPeriphAux(PowerPeriphDefn):
    def __init__(self):
        self.status_aux = {'foo': 0}
        self._load_test_payload = arr.array('B', range(256)) * 4

    def pull_status_aux(self, blocking=True):
        if not self.hw_valid:
            return False
        payload = arr.array('B', [self.RPC_GET_POWER_PERIPH_STATUS_AUX])
        return self.transport.do_rpc(blocking=blocking, is_push=False, payload=payload,
                                     rpc_callback=self._rpc_status_aux_reply) is not None

    def push_load_test(self, blocking=True):
        if not self.hw_valid:
            return False
        payload = self.transport.get_empty_payload()
        payload[0] = self.RPC_LOAD_TEST_PUSH
        payload[1:] = self._load_test_payload
        return self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload,
                                     rpc_callback=self._rpc_load_test_push_reply)  is not None

    def pull_load_test(self, blocking=True, quiet=False):
        if not self.hw_valid:
            return False
        self.pull_load_test_quiet=quiet
        payload = arr.array('B', [self.RPC_LOAD_TEST_PULL])
        return self.transport.do_rpc(blocking=blocking, is_push=False, payload=payload,
                                     rpc_callback=self._rpc_load_test_pull_reply)  is not None

    def _rpc_status_aux_reply(self, reply):
        if reply[0] == self.RPC_REPLY_POWER_PERIPH_STATUS_AUX:
            self._unpack_status_aux(reply[1:])
        else:
            self.logger.warning('Error RPC_REPLY_POWER_PERIPH_AUX_STATUS', reply[0])

    def _unpack_status_aux(self, s):
        # take in an array of bytes
        # this needs to exactly match the C struct format
        sidx = 0
        self.status_aux['foo'] = unpack_int16_t(s[sidx:])
        sidx += 2
        return sidx
    
    def _rpc_load_test_push_reply(self, reply):
        if reply[0] != self.RPC_REPLY_LOAD_TEST_PUSH:
            print('Error RPC_REPLY_LOAD_TEST_PUSH', reply[0])

    def _rpc_load_test_pull_reply(self, reply):
        if reply[0] == self.RPC_REPLY_LOAD_TEST_PULL:
            d = reply[1:]
            for i in range(1024):
                if d[i] != self._load_test_payload[(i + 1) % 1024]:
                    print('Load test pull bad data', d[i], self._load_test_payload[(i + 1) % 1024])
            self._load_test_payload = d
            if not self.pull_load_test_quiet:
                print('Successful load test pull')
        else:
            print('Error RPC_REPLY_LOAD_TEST_PULL', reply[0])


class PowerPeriphBase(PowerPeriphPowerMgmt,PowerPeriphPeriphControl,PowerPeriphTrace,PowerPeriphAux,Device):
    """
    API to the Stretch Power and IMU board (PowerPeriph)
    """

    def __init__(self, event_reset=False, usb=None,backend=None):
        Device.__init__(self, 'power_periph')
        PowerPeriphDefn.__init__(self)
        PowerPeriphPowerMgmt.__init__(self)
        PowerPeriphPeriphControl.__init__(self,event_reset)
        PowerPeriphTrace.__init__(self)
        PowerPeriphAux.__init__(self)

        self.config = self.params['config']
        self.imu = IMU()
        self._dirty_config = True
        self._dirty_trigger = False
        self._dirty_eye_animation = False
        self._eye_animation_left = 0
        self._eye_animation_right = 0
        self._eye_animation_intensity = 255
        self._eye_animation_r = 255
        self._eye_animation_g = 255
        self._eye_animation_b = 255

        if usb is None:
            usb = self.params['usb_name']
        if backend is None:
            backend=self.params['transport']['default_backend']
        self.transport = Transport(port_name=usb, logger=self.logger,
                                   default_backend=backend,
                                   qid=self.params['transport']['qid'])
        self.status: "PowerPeriphStatus" = {'voltage': 0, 'current': 0, 'temp': 0, 'cpu_temp': 0, 'frame_id': 0,
                       'timestamp': 0,  'runstop_event': False,
                       'bump_event_cnt': 0,
                       'fan_on': False, 'buzzer_on': False, 'low_soc_alert': False,
                       'high_current_alert': False, 'high_current_eoa_alert': False,
                       'adapter_voltage_present': False, 'boot_detected': False, 'imu': self.imu.status, 'debug': 0,
                       'state': 0, 'trace_on': 0,
                       'motor_sync_rate': 0, 'motor_sync_cnt': 0, 'motor_sync_queues': 0, 'motor_sync_drop': 0,
                       'transport': self.transport.status, 'current_charge': 0, 'current_eoa':0,'charger_is_charging': False,
                       'periph_power_state':{},
                       'over_tilt_type': 0, 'battery_current': 0, 'battery_soc':0, 'battery_soh':0,
                       'battery_chrg_dischrg_cycles': 0, 'voltage_cpu':0, 'voltage_5v0':0, 'voltage_36v0':0,
                       'voltage_12v0':0, 'voltage_aux_cpu':0, 'current_cpu':0, 'cpu_on_sts': False,
                       'runstop_cause':0, 'connected_to_network':False, 'connected_to_firebase': False,'adapter_fault':False,'adapter_connected':False,
                       'us_loop_time':0}

        self.status_zero = self.status.copy()
        self._trigger = 0
        self.board_info = {'board_variant': None, 'firmware_version': None, 'protocol_version': None, 'hardware_id': 0}
        self.hw_valid = False
        self.ts_last_motor_sync = None
        self.ts_last_motor_sync_warn = None


    # ###########  Device Methods #############

    def startup(self):
        try:
            self.logger.info('Starting PowerPeriph...')
            Device.startup(self)
            self.hw_valid = self.transport.startup()
            if self.hw_valid:
                payload = arr.array('B', [self.RPC_GET_POWER_PERIPH_BOARD_INFO])
                self.transport.do_rpc(blocking=True,is_push=False,payload=payload, rpc_callback=self._rpc_board_info_reply,backend=self.transport.BACKEND_PY_SERIAL) #Use py as C may not be supported yet
                self.transport.configure_version(self.board_info['firmware_version'])
                return True
            self.logger.error('Failed to start PowerPeriph')
            return False
        except Exception as e:
            self.hw_valid = False
            self.logger.error('Failed to start PowerPeriph: %s', e)
            return False

    def stop(self):
        Device.stop(self)
        if not self.hw_valid:
            return
        self.set_fan_off()
        self.push_command(exiting=True)
        self.transport.stop()
        self.hw_valid = False


    def pause_transport(self):
        self.transport.pause()

    def unpause_transport(self):
        self.transport.unpause()

    def enable_rate_logging(self,max_samples=1000):
        self.transport.n_rate_log=max_samples

    def get_rate_log(self):
        return self.transport.rate_log

    def _set_config(self, c):
        self.config = c.copy()
        self._dirty_config = True

    def pull_status(self, exiting=False,blocking=True):
        """
        Returns True if successful
        """
        if not self.hw_valid:
            return False
        payload = arr.array('B', [self.RPC_GET_POWER_PERIPH_STATUS])
        return self.transport.do_rpc(blocking=blocking, is_push=False, payload=payload, rpc_callback=self._rpc_status_reply) is not None


    def push_command(self, exiting=False, blocking=True):
        """
           Returns True if successful
       """
        if not self.hw_valid:
            return False
        payload = self.transport.get_empty_payload()
        success=True



        if self.imu._dirty_config:
            payload[0] = self.RPC_SET_IMU_CONFIG
            sidx = self.imu._pack_config(payload, 1)
            success = success and self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx],rpc_callback=self._rpc_imu_config_reply) is not None
            self.imu._dirty_config = False

        if self._dirty_config:
            payload[0] = self.RPC_SET_POWER_PERIPH_CONFIG
            sidx = self._pack_config(payload, 1)
            success=success and self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx], rpc_callback=self._rpc_config_reply) is not None
            self._dirty_config = False

        if self._dirty_trigger:
            payload[0] = self.RPC_SET_POWER_PERIPH_TRIGGER
            sidx = self._pack_trigger(payload, 1)
            success=success and self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx],rpc_callback=self._rpc_trigger_reply) is not None
            self._trigger = 0
            self._dirty_trigger = False

        if self._dirty_eye_animation:
            payload[0] = self.RPC_SET_EYE_ANIMATION
            sidx = 1
            pack_uint8_t(payload, sidx, self._eye_animation_left)
            sidx += 1
            pack_uint8_t(payload, sidx, self._eye_animation_right)
            sidx += 1
            if self.board_info['protocol_version'] is not None and int(self.board_info['protocol_version'][1:]) >= 13:
                pack_uint8_t(payload, sidx, self._eye_animation_intensity)
                sidx += 1
                pack_uint8_t(payload, sidx, self._eye_animation_r)
                sidx += 1
                pack_uint8_t(payload, sidx, self._eye_animation_g)
                sidx += 1
                pack_uint8_t(payload, sidx, self._eye_animation_b)
                sidx += 1
            success = success and self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx], rpc_callback=self._rpc_eye_animation_reply) is not None
            self._eye_animation_left = 0
            self._eye_animation_right = 0
            self._eye_animation_intensity = 255
            self._eye_animation_r = 255
            self._eye_animation_g = 255
            self._eye_animation_b = 255
            self._dirty_eye_animation = False

        return success

    def load_rpc_results(self,wait_on_result=True):
        self.transport.load_rpc_results(wait_on_result)

    def board_reset(self):
        self._trigger = self._trigger | self.TRIGGER_BOARD_RESET
        self._dirty_trigger = True


    def pretty_print(self):
        print('----------- PowerPeriph ----------')
        print('Timestamp (s)', self.status['timestamp'])
        print('---Runstop--')
        print('Runstop Event', self.status['runstop_event'])
        print('Runstop Cause', self.status['runstop_cause'])
        print('---Motor--')
        print('Motor sync queued', self.status['motor_sync_queues'])
        print('Motor sync dropped', self.status['motor_sync_drop'])
        print('Motor sync rate', self.status['motor_sync_rate'])
        print('Motor sync cnt', self.status['motor_sync_cnt'])
        print('--Temp---')
        print('CPU Temp', self.status['cpu_temp'])
        print('Board Temp', self.status['temp'])
        print('--Voltage---')
        print('Voltage Bus', self.status['voltage'])
        print('Voltage CPU', self.status['voltage_cpu'])
        print('Voltage 5v0', self.status['voltage_5v0'])
        print('Voltage 36v0', self.status['voltage_36v0'])
        print('Voltage 12v0', self.status['voltage_12v0'])
        print('Voltage aux cpu', self.status['voltage_aux_cpu'])
        print('--Current---')
        print('Current System', self.status['current'])
        print('Current EOA',self.status['current_eoa'])
        print('Current Charge', self.status['current_charge'])
        print('Current Battery', self.status['battery_current'])
        print('Current CPU', self.status['current_cpu'])
        print('---Alerts--')
        print('Low SOC Alert', self.status['low_soc_alert'])
        print('High Current Alert', self.status['high_current_alert'])
        print('High Current EOA Alert', self.status['high_current_eoa_alert'])
        print('---Battery Charge--')
        print('Charger is charging', self.status['charger_is_charging'])
        print('Adapter voltage present', self.status['adapter_voltage_present'])
        print('Adapter fault',self.status['adapter_fault'])
        print('Adapter connected', self.status['adapter_connected'])
        print('Battery SOC', self.status['battery_soc'])
        print('Battery SOH', self.status['battery_soh'])
        print('Battery discharge cycles', self.status['battery_chrg_dischrg_cycles'])
        print('---Periph Power--')
        for k in self.status['periph_power_state'].keys():
            print('Periph: ',k,self.status['periph_power_state'][k])
        print('---IMU--')
        self.imu.pretty_print()
        print('---Firebase---')
        print('Connected to network',self.status['connected_to_network'])
        print('Connected to Firebase', self.status['connected_to_firebase'])
        print('---Util---')
        print('Board variant:', self.board_info['board_variant'])
        print('Firmware version:', self.board_info['firmware_version'])
        print('Transport version:', self.transport.version)
        print('State', self.status['state'])
        print('Bump Event Cnt', self.status['bump_event_cnt'])
        print('CPU on sts', self.status['cpu_on_sts'])
        print('Fan On', self.status['fan_on'])
        print('Buzzer On', self.status['buzzer_on'])
        print('Trace on:', self.status['trace_on'])
        print('Boot Detected', self.status['boot_detected'])
        print('Debug', self.status['debug'])
        print('Loop time (us)',self.status['us_loop_time'])
        
    def step_sentry(self, robot_status=None):
        pass

    def trigger_motor_sync(self, blocking=True):
        # Push out immediately
        if not self.hw_valid:
            return False
        payload = arr.array('B', [self.RPC_SET_MOTOR_SYNC])
        old_sync_cnt = self.status['motor_sync_cnt']
        rpc_id = self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload,
                                       rpc_callback=self._rpc_motor_sync_reply)

        t = time.time()
        # Should motor_sync_cnt should increment with each call to trigger_motor_sync, if not it is an overrun
        if self.status['motor_sync_cnt'] == old_sync_cnt:
            self.status['motor_sync_queues'] = self.status['motor_sync_queues'] + 1
            # print('Warning: Queued motor_sync as trigger_motor_sync calls above maximum rate. Overruns: %d' % (
            #     self.status['motor_sync_queues']))
            self.ts_last_motor_sync_warn = t

        if self.ts_last_motor_sync is not None:
            self.status['motor_sync_rate'] = 1 / (t - self.ts_last_motor_sync)
        self.ts_last_motor_sync = t
        return rpc_id is not None

    def _get_tilt_type(self, raw):
        tilt_type = {
            1: 'Left Tilt',
            2: 'Right Tilt',
            3: 'Front Tilt'
        }
        return tilt_type.get(raw, None)
    # ################Data Packing #####################

    def _unpack_board_info(self, s):
        sidx = 0
        self.board_info['board_variant'] = unpack_string_t(s[sidx:], 20)
        self.board_info['hardware_id'] = 0
        if len(self.board_info['board_variant']) == 6:  # New format of PowerPeriph.x Older format of PowerPeriph.BoardName.Vx' If older format,default to 0
            self.board_info['hardware_id'] = int(self.board_info['board_variant'][-1])
        sidx += 20
        self.board_info['firmware_version'] = unpack_string_t(s[sidx:], 20).strip('\x00')
        sidx += 20
        if self.board_info['firmware_version'].find('hello-pimu2')==0: #Newer format, length is 30 not 20
            str10=unpack_string_t(s[sidx:], 10).strip('\x00')
            sidx += 10
            self.board_info['firmware_version']=self.board_info['firmware_version']+str10
        self.board_info['protocol_version'] = self.board_info['firmware_version'][
                                                  self.board_info['firmware_version'].rfind('p'):]
        return sidx


    def _pack_config(self, s, sidx):
        pack_float_t(s, sidx, self.config['voltage_LPF'])
        sidx += 4
        pack_float_t(s, sidx, self.config['current_LPF'])
        sidx += 4
        pack_float_t(s, sidx, self.config['temp_LPF'])
        sidx += 4
        pack_uint8_t(s, sidx, self.config['stop_at_runstop'])
        sidx += 1
        pack_uint8_t(s, sidx,0) # Deprecate self.config['runstop_at_tilt'])
        sidx += 1
        pack_uint8_t(s, sidx, self.config['runstop_at_low_soc'])
        sidx += 1
        pack_uint8_t(s, sidx, self.config['runstop_at_high_current'])
        sidx += 1
        pack_float_t(s, sidx, self.config['bump_thresh'])
        sidx += 4
        pack_float_t(s, sidx, self.config['low_soc_alert'])
        sidx += 4
        pack_float_t(s, sidx, self.config['high_current_alert'])
        sidx += 4
        pack_float_t(s, sidx, self.config['over_tilt_alert'])
        sidx += 4
        return sidx

    def _pack_trigger(self, s, sidx):
        pack_uint32_t(s, sidx, self._trigger)
        sidx += 4
        return sidx

    def get_runstop_cause(self, raw):
        r_cause = [None, 'RUNSTOP_BUTTON', 'RUNSTOP_PYTHON_CMD', 'RUNSTOP_LOW_SOC', 'RUNSTOP_HIGH_CURRENT', 'RUNSTOP_HIGH_CURRENT_EOA']
        for i in range(0, len(r_cause)):
            if i == raw:
                break
        return r_cause[i]

    def get_periph_power_state(self, raw):
        return {'power_to_lift': (raw & 1<<1)>0,
               'power_to_omni_0_motor': (raw & 1<<2)>0,
               'power_to_omni_1_motor': (raw & 1<<3)>0,
               'power_to_omni_2_motor': (raw & 1<<4)>0,
               'power_to_arm': (raw & 1<<5)>0,
               'power_to_eoa': (raw & 1<<6)>0}

    def _unpack_status(self, s, unpack_to=None):  # P7
        if unpack_to is None:
            unpack_to = self.status
        sidx = 0
        sidx += self.imu._unpack_status((s[sidx:]))


        unpack_to['voltage'] = self.get_voltage(unpack_float_t(s[sidx:]))
        sidx += 4
        unpack_to['current'] = self.get_current(unpack_float_t(s[sidx:]))
        sidx += 4
        unpack_to['temp'] = self.get_temp(unpack_float_t(s[sidx:]))
        sidx += 4
        unpack_to['state'] = unpack_uint32_t(s[sidx:])
        sidx += 4

        #Map state to flags
        unpack_to['runstop_event'] = (unpack_to['state'] & self.STATE_RUNSTOP_EVENT) != 0
        unpack_to['fan_on'] = (unpack_to['state'] & self.STATE_FAN_ON) != 0
        unpack_to['buzzer_on'] = (unpack_to['state'] & self.STATE_BUZZER_ON) != 0
        unpack_to['low_soc_alert'] = (unpack_to['state'] & self.STATE_LOW_SOC_ALERT) != 0
        unpack_to['high_current_alert'] = (unpack_to['state'] & self.STATE_HIGH_CURRENT_ALERT) != 0
        unpack_to['high_current_eoa_alert'] = (unpack_to['state'] & self.STATE_HIGH_CURRENT_EOA_ALERT) != 0
        if self.board_info['hardware_id'] > 0:
            unpack_to['adapter_voltage_present'] = (unpack_to['state'] & self.STATE_ADAPTER_VOLTAGE_PRESENT) != 0
            unpack_to['boot_detected'] = (unpack_to['state'] & self.STATE_BOOT_DETECTED) != 0
        unpack_to['trace_on'] = (unpack_to['state'] & self.STATE_IS_TRACE_ON) != 0
        unpack_to['connected_to_network'] = (unpack_to['state'] & self.STATE_CONNECTED_TO_NETWORK) != 0
        unpack_to['adapter_fault'] = (unpack_to['state'] & self.STATE_ADAPTER_FAULT) != 0
        unpack_to['adapter_connected'] = (unpack_to['state'] & self.STATE_ADAPTER_CONNECTED) != 0

        unpack_to['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:]))
        sidx += 8
        self.imu.status['timestamp'] = unpack_to['timestamp']
        unpack_to['bump_event_cnt'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        unpack_to['debug'] = unpack_float_t(s[sidx:])
        sidx += 4
        self.status['current_charge'] = self.get_current_charge(unpack_float_t(s[sidx:]))
        sidx += 4
        unpack_to['charger_is_charging'] = (unpack_to['state'] & self.STATE_IS_CHARGER_CHARGING) != 0
        unpack_to['over_tilt_type'] = self._get_tilt_type(unpack_uint8_t(s[sidx:]))
        sidx += 1
        unpack_to['battery_current'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['battery_soc'] = unpack_uint8_t(s[sidx:])
        sidx += 1
        unpack_to['battery_soh'] = unpack_uint8_t(s[sidx:])
        sidx += 1
        unpack_to['battery_chrg_dischrg_cycles'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        unpack_to['voltage_cpu'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['voltage_5v0'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['voltage_36v0'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['voltage_12v0'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['voltage_aux_cpu'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['current_cpu'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['cpu_on_sts'] = unpack_uint8_t(s[sidx:])
        sidx += 1
        unpack_to['runstop_cause'] = self.get_runstop_cause(unpack_uint8_t(s[sidx:]));
        sidx += 1

        return sidx

        # ################Transport Callbacks #####################
    def _rpc_eye_animation_reply(self, reply):
        if len(reply) == 0 or reply[0] != self.RPC_REPLY_SET_EYE_ANIMATION:
            self.logger.warning(f'Error RPC_REPLY_SET_EYE_ANIMATION. Reply contents: {reply}')

    def _rpc_imu_config_reply(self, reply):
        if reply[0] != self.RPC_SET_IMU_CONFIG_REPLY:
            self.logger.warning('Error RPC_SET_IMU_CONFIG_REPLY', reply[0])

    def _rpc_config_reply(self, reply):
        if reply[0] != self.RPC_REPLY_POWER_PERIPH_CONFIG:
            self.logger.warning('Error RPC_REPLY_POWER_PERIPH_CONFIG', reply[0])

    def _rpc_imu_config_reply(self, reply):
        if reply[0] != self.RPC_SET_IMU_CONFIG_REPLY:
            self.logger.warning('Error RPC_SET_IMU_CONFIG_REPLY', reply[0])

    def _rpc_board_info_reply(self, reply):
        if reply[0] == self.RPC_REPLY_POWER_PERIPH_BOARD_INFO:
            self._unpack_board_info(reply[1:])
        else:
            self.logger.warning('Error RPC_REPLY_POWER_PERIPH_BOARD_INFO', reply[0])

    def _rpc_trigger_reply(self, reply):
        if reply[0] != self.RPC_REPLY_POWER_PERIPH_TRIGGER:
            self.logger.warning('Error RPC_REPLY_POWER_PERIPH_TRIGGER', reply[0])
        else:
            tt = unpack_uint32_t(reply[1:])

    def _rpc_status_reply(self, reply):
        if reply[0] == self.RPC_REPLY_POWER_PERIPH_STATUS:
            self._unpack_status(reply[1:])
        else:
            self.logger.warning('Error RPC_REPLY_POWER_PERIPH_STATUS', reply[0])

    def _rpc_motor_sync_reply(self, reply):
        if reply[0] != self.RPC_REPLY_MOTOR_SYNC:
            self.logger.warning('Error RPC_REPLY_MOTOR_SYNC', reply[0])
        else:
            self._unpack_motor_sync_reply(reply[1:])

    def _unpack_motor_sync_reply(self, s):
        # take in an array of bytes
        # this needs to exactly match the C struct format
        sidx = 0
        self.status['motor_sync_cnt'] = unpack_int16_t(s[sidx:])
        sidx += 2
        return sidx


# ######################## POWER_PERIPH PROTOCOL P8 #################################

class PowerPeriph_Protocol_P8(PowerPeriphBase):
    def foo(self):
        pass

# ######################## POWER_PERIPH PROTOCOL P8 #################################

class PowerPeriph_Protocol_P10(PowerPeriphBase):
    def _unpack_status(self, s, unpack_to=None):  # P7
        if unpack_to is None:
            unpack_to = self.status
        sidx = 0
        sidx = sidx + PowerPeriphBase._unpack_status(self, s, unpack_to)
        unpack_to['connected_to_firebase'] = (unpack_to['state'] & self.STATE_CONNECTED_TO_FIREBASE) != 0
        return sidx
    
class PowerPeriph_Protocol_P11(PowerPeriph_Protocol_P10):
    def _unpack_status(self, s, unpack_to=None):  # P7
        if unpack_to is None:
            unpack_to = self.status
        sidx = 0
        sidx = sidx + PowerPeriph_Protocol_P10._unpack_status(self, s, unpack_to)
        unpack_to['us_loop_time']  = unpack_uint16_t(s[sidx:])
        sidx += 2
        return sidx

    def _pack_config(self, s, sidx):
        pack_float_t(s, sidx, self.config['voltage_LPF'])
        sidx += 4
        pack_float_t(s, sidx, self.config['current_LPF'])
        sidx += 4
        pack_float_t(s, sidx, self.config['temp_LPF'])
        sidx += 4
        pack_uint8_t(s, sidx, self.config['stop_at_runstop'])
        sidx += 1
        pack_uint8_t(s, sidx, 0) # Deprecate self.config['runstop_at_tilt'])
        sidx += 1
        pack_uint8_t(s, sidx, self.config['runstop_at_low_soc'])
        sidx += 1
        pack_uint8_t(s, sidx, self.config['runstop_at_high_current'])
        sidx += 1
        pack_float_t(s, sidx, self.config['bump_thresh'])
        sidx += 4
        pack_float_t(s, sidx, self.config['low_soc_alert'])
        sidx += 4
        pack_float_t(s, sidx, self.config['high_current_alert'])
        sidx += 4
        pack_float_t(s, sidx, 0) #self.config['over_tilt_alert']) Deprecated
        sidx += 4
        return sidx

class PowerPeriph_Protocol_P12(PowerPeriph_Protocol_P11):
    def _unpack_status(self, s, unpack_to=None):  # P7
        if unpack_to is None:
            unpack_to = self.status
        sidx = 0
        sidx = sidx + PowerPeriph_Protocol_P11._unpack_status(self, s, unpack_to)
        unpack_to['current_eoa']  = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['periph_power_state']  = self.get_periph_power_state(unpack_uint8_t(s[sidx:]))
        sidx += 2
        return sidx

    def _pack_config(self, s, sidx):
        pack_float_t(s, sidx, self.config['voltage_LPF'])
        sidx += 4
        pack_float_t(s, sidx, self.config['current_LPF'])
        sidx += 4
        pack_float_t(s, sidx, self.config['temp_LPF'])
        sidx += 4
        pack_uint8_t(s, sidx, self.config['stop_at_runstop'])
        sidx += 1
        pack_uint8_t(s, sidx, self.config['runstop_at_low_soc'])
        sidx += 1
        pack_uint8_t(s, sidx, self.config['runstop_at_high_current'])
        sidx += 1
        pack_uint8_t(s, sidx, self.config['runstop_at_high_current_eoa'])
        sidx += 1
        pack_uint8_t(s, sidx, self.config['disable_eoa_at_high_current_eoa'])
        sidx += 1
        pack_float_t(s, sidx, self.config['bump_thresh'])
        sidx += 4
        pack_float_t(s, sidx, self.config['low_soc_alert'])
        sidx += 4
        pack_float_t(s, sidx, self.config['high_current_alert'])
        sidx += 4
        pack_float_t(s, sidx, self.config['high_current_eoa_alert'])
        sidx += 4
        pack_uint8_t(s, sidx, self.config['nuc_safe_shutdown'])
        sidx += 1
        return sidx
# ######################## POWER_PERIPH #################################

class PowerPeriph(PowerPeriphBase):
    """
    API to the Stretch Power and IMU board (PowerPeriph)
    """

    def __init__(self, event_reset=False, usb=None,backend=None):
        PowerPeriphBase.__init__(self, event_reset, usb,backend)
        # Order in descending order so more recent protocols/methods override less recent
        self._supported_protocols = {'p9': (PowerPeriph_Protocol_P8,),'p8': (PowerPeriph_Protocol_P8,), 
                                     'p10': (PowerPeriph_Protocol_P10,),
                                     'p11': (PowerPeriph_Protocol_P11,),
                                     'p12': (PowerPeriph_Protocol_P12,),'p13': (PowerPeriph_Protocol_P12,),}


    def startup(self):
        """
        First determine which protocol version the uC firmware is running.
        Based on that version, replaces PowerPeriphBase class inheritance with a inheritance to a child class of PowerPeriphBase that supports that protocol
        """

        PowerPeriphBase.startup(self)
        if self.hw_valid:
            if self.board_info['protocol_version'] in self._supported_protocols:
                PowerPeriph.__bases__ = self._supported_protocols[self.board_info['protocol_version']]
                IMU.__bases__ = self.imu._supported_protocols[self.board_info['protocol_version']]
            else:
                if self.board_info['protocol_version'] is None:
                    protocol_msg = """
                                    ----------------
                                    Failure in communications for {0} on startup.
                                    Please power cycle the robot and try again.
                                    ----------------
                                    """.format(self.name)
                else:
                    protocol_msg = """
                    ----------------
                    Firmware protocol mismatch on {0}.
                    Protocol on board is {1}.
                    Valid protocols are: {2}.
                    Disabling device.
                    Please upgrade the firmware and/or version of Stretch Body.
                    ----------------
                    """.format(self.name, self.board_info['protocol_version'], self._supported_protocols.keys())
                self.logger.warning(textwrap.dedent(protocol_msg))
                self.hw_valid = False
                self.transport.stop()
        if self.hw_valid:
            self.push_command()
            self.pull_status()
        return self.hw_valid

if __name__ == '__main__':
    p=PowerPeriph()
    p.startup()
    for i in range(100):
        p.pull_status(blocking=False)
        time.sleep(.01)
        p.load_rpc_results()
        p.pretty_print()
    p.stop()

class IMUBaseStatus(TypedDict):
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float
    my: float
    mz: float
    roll: float
    pitch: float
    heading: float
    timestamp: float
    qw: float
    qx: float
    qy: float
    qz: float
    bump: int
    gravity_tilt: float

class PowerPeriphStatus(TypedDict):
    voltage: float
    current: float
    temp: float
    cpu_temp: float
    frame_id: int
    timestamp: float
    runstop_event: bool
    bump_event_cnt: int
    fan_on: bool
    buzzer_on: bool
    low_soc_alert: bool
    high_current_alert: bool
    high_current_eoa_alert: bool
    adapter_voltage_present: bool
    boot_detected: bool
    imu: IMUBaseStatus
    debug: int
    state: int
    trace_on: int
    motor_sync_rate: float
    motor_sync_cnt: int
    motor_sync_queues: int
    motor_sync_drop: int
    transport: Any
    current_charge: float
    current_eoa: float
    charger_is_charging: bool
    periph_power_state: dict
    over_tilt_type: int
    battery_current: float
    battery_soc: float
    battery_soh: float
    battery_chrg_dischrg_cycles: int
    voltage_cpu: float
    voltage_5v0: float
    voltage_36v0: float
    voltage_12v0: float
    voltage_aux_cpu: float
    current_cpu: float
    cpu_on_sts: bool
    runstop_cause: int
    connected_to_network: bool
    connected_to_firebase: bool
    adapter_fault: bool
    adapter_connected: bool
    us_loop_time: float