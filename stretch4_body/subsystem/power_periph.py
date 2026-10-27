#!/usr/bin/env python3

from typing import Any, Optional, TypedDict

from stretch4_body.core.transport.transport import *
from stretch4_body.core.device import Device
from stretch4_body.core.hello_utils import *
import struct
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
            self.calculate_tilt_angle([unpack_to['qw'],unpack_to['qx'],unpack_to['qy'],unpack_to['qz']]))

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
                                     'p10': (IMU_Protocol_P8,),'p11':(IMU_Protocol_P8,),
                                     'p12':(IMU_Protocol_P8,),'p13':(IMU_Protocol_P8,),
                                     'p14':(IMU_Protocol_P8,)}

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
    # RPC 33 frame mode (firmware v0.1.52p14): byte 0 = EYE_CMD_FRAME, then u16 sequence,
    # u16 hold_ms, brightness left and right, 20 RGB pixels (left ring 0..9, right 0..9,
    # index 0 just clockwise of 12 o'clock, clockwise). 68 bytes with the id, two push
    # frames. Reply 34: status u8, last applied sequence u16.
    RPC_SET_EYE_PER_EYE = 33
    RPC_REPLY_SET_EYE_PER_EYE = 34
    EYE_CMD_FRAME = 0x80
    EYE_FRAME_PIXELS_PER_RING = 10
    EYE_FRAME_HOLD_MS_MIN = 50
    EYE_FRAME_HOLD_MS_MAX = 2000
    EYE_FRAME_REQUEST_BYTES = 68
    EYE_FRAME_REPLY_PAYLOAD_BYTES = 3
    EYE_FRAME_STATUS_NAMES = {0: 'ok', 1: 'bad length', 3: 'hold out of range'}
    EYE_FRAME_MIN_FIRMWARE = 'hello-pimu2.v0.1.52p14'
    RPC_GET_POWER_DIAGNOSTICS = 31
    RPC_REPLY_POWER_DIAGNOSTICS = 32
    RPC_REPLY_REJECTED = 38
    RPC_REJECTED_REPLY_BYTES = 5
    RPC_REJECT_REASON_NAMES = {1: 'unknown id', 2: 'short request', 3: 'empty request'}
    # RPC ids 39 and 40 (Easter egg set and clear, v0.1.38p14 to v0.1.50p14) are retired,
    # never reuse them. The Easter egg now goes as TRIGGER_EASTER_EGG_ON/OFF (v0.1.51p14).
    # Diagnostics schema asked for first. Schema 3 (firmware v0.1.53p14) is the 289-byte
    # snapshot: schema 2 plus the ESP firmware version. Schema 2 (v0.1.47p14 to
    # v0.1.52p14) is its first 255 bytes; schema 1 its first 190, the only schema of
    # v0.1.18p14 to v0.1.23p14. Newer firmware answers an older schema with that prefix.
    # The reply header, the snapshot's schema_version and struct_size must all agree
    # with these sizes.
    POWER_DIAGNOSTICS_SCHEMA_VERSION = 3
    POWER_DIAGNOSTICS_SCHEMA1_WIRE_SIZE = 190
    POWER_DIAGNOSTICS_SCHEMA2_WIRE_SIZE = 255
    POWER_DIAGNOSTICS_WIRE_SIZE = 289
    POWER_DIAGNOSTICS_WIRE_SIZES = {1: POWER_DIAGNOSTICS_SCHEMA1_WIRE_SIZE,
                                    2: POWER_DIAGNOSTICS_SCHEMA2_WIRE_SIZE,
                                    3: POWER_DIAGNOSTICS_WIRE_SIZE}
    POWER_EVENT_LOG_HEADER_SIZE = 24
    POWER_EVENT_LOG_ENTRY_SIZE = 16
    POWER_EVENT_LOG_MAX_RPC_ENTRIES = 16
    POWER_DIAGNOSTICS_REPLY_OK = 0
    POWER_DIAGNOSTICS_REPLY_BAD_REQUEST_SIZE = 1
    POWER_DIAGNOSTICS_REPLY_UNSUPPORTED_SCHEMA = 2
    POWER_DIAGNOSTICS_REPLY_STATUS_NAMES = {
        POWER_DIAGNOSTICS_REPLY_OK: 'ok',
        POWER_DIAGNOSTICS_REPLY_BAD_REQUEST_SIZE: 'bad_request_size',
        POWER_DIAGNOSTICS_REPLY_UNSUPPORTED_SCHEMA: 'unsupported_schema',
    }

    POWER_EVENT_NAMES = {
        1: 'boot',
        2: 'power_state_transition',
        3: 'button_raw_changed',
        4: 'button_debounced',
        5: 'button_request',
        6: 'adapter_changed',
        7: 'charger_enable_command_changed',
        8: 'charger_expected_active_changed',
        9: 'bms_sample_valid',
        10: 'bms_timeout',
        11: 'bms_crc_error',
        12: 'bms_rx_overflow',
        13: 'bms_recovered',
        14: 'bms_alarm_changed',
        15: 'bms_mos_changed',
        16: 'cpu_status_changed',
        17: 'esp_status_recovered',
        18: 'esp_status_stale',
        19: 'runstop_changed',
        20: 'peripheral_state_changed',
        21: 'cpu_power_cycle_request',
        22: 'esp_crc_error',
        23: 'esp_rx_overflow',
        24: 'esp_tx_queue_drop',
        25: 'bms_protocol_error',
        26: 'bms_boot_wait',
        27: 'bms_decision_blocked',
        28: 'boot_watchdog_arm',
        29: 'shutdown_requested',
        30: 'shutdown_nuc_notified',
        31: 'shutdown_nuc_wait_ended',
        32: 'shutdown_rails_down',
        33: 'shutdown_retargeted',
        34: 'shutdown_ended',
        35: 'charge_stop',
        36: 'charge_restart',
        40: 'adc_bank_reinit',
        41: 'adc_stale_changed',
        42: 'adapter_source_changed',
        43: 'low_battery_qualified',
        44: 'esp_request_rejected',
        45: 'esp_rx_length_error',
        46: 'adapter_disagreement',
        47: 'esp_shutdown_cmd_delivered',
        48: 'esp_trigger_dropped',
        49: 'eoa_trip',
        50: 'easter_egg_entered',
        51: 'easter_egg_left',
        52: 'low_battery_auto_wake',
        53: 'button_pulse_rejected',
        54: 'nuc_boot_watch',
    }
    POWER_EVENT_EXPLANATIONS = {
        1: 'PIMU restarted; data0 is the reset-cause bitmask and data1 is the board variant.',
        2: 'PIMU power state changed; data0 is the previous state and data1 is the transition reason.',
        3: 'Raw PWR_EN input changed before debounce.',
        4: 'PWR_EN crossed the 500 ms debounce threshold.',
        5: 'A physical-button request was accepted by the power-state machine.',
        6: 'The PIMU 36 V adapter-presence measurement changed.',
        7: ('The firmware charger-enable command changed. This is the MCU GPIO/flag, '
            'not proof that the charger IC physically enabled.'),
        8: ('The firmware-derived charging expectation changed (adapter present and '
            'enable commanded); confirm physical charging with measured current and BMS MOS state.'),
        9: 'The first CRC-valid BMS response was received after boot.',
        10: 'The BMS did not complete a Modbus response before the framing timeout.',
        11: 'A complete BMS Modbus frame failed CRC validation.',
        12: 'The BMS receive frame exceeded the fixed receive buffer.',
        13: 'BMS communication recovered; data0 is the consecutive failure count.',
        14: 'One or more BMS alarm bits changed; data0/data1 are the old/new masks.',
        15: 'One or more BMS MOS state bits changed; data0/data1 are the old/new masks.',
        16: ('The ESP-observed NUC USB port-A VBUS-presence input changed. This '
             'does not prove that the NUC OS is running or that the NUC rail is healthy.'),
        17: 'ESP status was received for the first time or after becoming stale.',
        18: 'No ESP status packet was received for more than two seconds.',
        19: 'Runstop state or cause changed.',
        20: 'The PIMU actuator/peripheral power mask changed.',
        21: 'The host requested a NUC power cycle.',
        22: 'An ESP UART frame failed CRC validation.',
        23: 'The ESP UART receive frame exceeded its fixed buffer.',
        24: 'The PIMU-to-ESP transmit queue was full and dropped a packet.',
        25: 'A CRC-valid BMS reply had the wrong address, function, byte count, or length; data1 is the exception code or received length.',
        26: 'The BMS boot wait finished; data0 is 1 present or 2 absent, data1 the wait in ms.',
        27: ('A low-battery shutdown (data0 1) or charger decision (data0 2) was held because the '
             'BMS sample was absent or stale; data1 is the sample age in ms, 32767 with no sample.'),
        28: ('Wake sent to the ESP with its boot-watchdog arm byte; data0 is the arm value, data1 the '
             'reason (0 RDK build, 1 no host config yet, 2 host nuc_safe_shutdown).'),
        29: 'Shutdown transaction requested; data0 is the transition reason, data1 is 1 when the rails wait for the NUC.',
        30: ('Shutdown transaction told the ESP the target state; data0 is the UART message id, '
             'data1 the ticks waited so far.'),
        31: ('Shutdown transaction NUC wait ended; data0 is the cause (0 no wait, 1 NUC off, 2 cap, '
             '4 cap with only stale "off" status), data1 the ticks waited.'),
        32: 'Shutdown transaction dropped the rails; data0 is the rail mask (1 20V0, 2 5V0), data1 the starting reason.',
        33: 'Shutdown transaction retargeted while running; data0 is the transition reason, data1 the rail mask down.',
        34: 'Shutdown transaction ended by a wake; data0 is the wake reason, data1 the furthest step reached.',
        35: ('Firmware stopped charging on a sustained BMS high-voltage alarm; data0 is reason << 8 | SoC, '
             'data1 the cell max mV.'),
        36: 'Charging restarted after a charge stop; data0 is reason << 8 | SoC, data1 the pack mV.',
        40: 'An ADC bank was rebuilt by the stall watchdog; data0 is the bank, data1 its rebuild count.',
        41: 'The ADC stale mask changed; data0/data1 are the old/new masks (bit 0 ADC0, bit 1 ADC1).',
        42: 'The adapter presence source changed; data0/data1 are old/new (0 ADC, 1 ESP, 2 held).',
        43: ('Low-battery shutdown qualified in ACTIVE; data0 is the cause bits (1 SoC 0, 2 undervoltage), '
             'data1 the consecutive low BMS frames.'),
        44: ('An ESP sleep or wake request was rejected in a state that does not consume it; data0 is the '
             'request bits (1 sleep, 2 wake), data1 the rejections since boot.'),
        45: ('A CRC-valid ESP frame failed the per-id length check; data0 is the error count (negative: '
             'unknown id count), data1 is id << 8 | payload length.'),
        46: ('The ESP adapter_connected bit disagreed with the SAMD 36 V input for 1 s; data0/data1 are '
             'the old/new masks (1 ESP only, 2 voltage only).'),
        47: 'The ESP proved it consumed UART_PWR_SHUTDOWN; data0 is the sends, data1 the ms since the first.',
        48: ('An ESP rail-on request was dropped outside ACTIVE; data0 is what (1 host lidar-on, 2 host AUX-on, '
             '4 wake AUX-on), data1 is where (1 at the ACTIVE exit, 2 requested outside ACTIVE).'),
        49: ('The end-of-arm output was cut on an overcurrent, came back, or a runstop clear was refused '
             '(firmware v0.1.50p14). A cut: data0 is the source (1 ADC filter, 2 U44 comparator), data1 the '
             'EOA current in 10 mA. Back on: data0 is 16 runstop cleared, 32 host enable, 64 ACTIVE entry; '
             '128 a runstop clear refused after 3 trips in 60 s; data1 is the trips in the last 60 s.'),
        50: 'Easter egg mode entered; data0 is 1 for a 5 s runstop hold, 2 for a host request.',
        51: 'Easter egg mode left; data0 is 2 for a host request, 3 for a runstop entry.',
        52: ('Low-battery auto-wake; data0 is 1 armed by a low-battery shutdown, 2 armed found in flash at '
             'boot, 3 woke, 4 disarmed by another wake; data1 is the SoC, or that wake\'s transition reason for 4.'),
        53: ('A power button pulse too short to be a press was ignored; data0 is its width in ms, data1 the '
             'pulses rejected since boot.'),
        54: ('NUC boot watch; data0 is 1 NUC not booted 15 s into ACTIVE (data1 1 VBUS low, 0 no fresh ESP '
             'status), 2 seen up by VBUS, 3 seen up by host traffic (data1 ms since ACTIVE), 4 not-booted '
             'ended by leaving ACTIVE (data1 seconds).'),
    }
    POWER_TRANSITION_REASON_NAMES = {
        0: 'unknown',
        1: 'boot_button',
        2: 'boot_no_button',
        3: 'user_button',
        4: 'esp_sleep_request',
        5: 'host_sleep_trigger',
        6: 'low_battery',
        7: 'charging_expectation_set',
        8: 'charging_expectation_cleared',
        9: 'power_button_high',
        10: 'user_wake',
        11: 'esp_wake_request',
    }
    # Schema 2 enum fields (Common.h, PROTOCOL_P14_POWER_DIAGNOSTICS.md). The decoder
    # publishes '<field>_name' beside each raw value; None on a schema 1 snapshot.
    ADAPTER_PRESENCE_SOURCE_NAMES = {0: 'adc', 1: 'esp_fallback', 2: 'held'}
    SHUTDOWN_WAIT_END_NAMES = {
        0: 'no_wait_configured',
        1: 'nuc_off',
        2: 'cap',
        3: 'cancelled_by_wake',
        4: 'cap_stale_off',
        255: 'none_since_boot',
    }
    CHARGE_STOP_STATE_NAMES = {0: 'idle', 1: 'arming', 2: 'hold'}
    ESP_ADAPTER_DISAGREE_NAMES = {0: 'agree', 1: 'esp_only', 2: 'voltage_only', 3: 'both'}
    # Schema 3 esp_version_state (firmware EspVersionQuery.h). no_reply: five requests
    # went unanswered, an ESP older than hello-esp.v0.1.11p0.
    ESP_VERSION_STATE_NAMES = {0: 'waiting', 1: 'reported', 2: 'no_reply'}
    POWER_EVENT_SOURCE_NAMES = {
        1: 'pimu', 2: 'power_state', 3: 'bms', 4: 'esp', 5: 'peripheral', 6: 'runstop',
    }

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
    STATE_EASTER_EGG_MODE = (1 << 20)
    STATE_EOA_OUTPUT_TRIPPED = (1 << 21)  # firmware v0.1.50p14; always 0 on older builds

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
    TRIGGER_EASTER_EGG_ON = (1 << 24)
    TRIGGER_EASTER_EGG_OFF = (1 << 25)

    TRACE_TYPE_STATUS = 0
    TRACE_TYPE_DEBUG = 1
    TRACE_TYPE_PRINT = 2

    def _do_rpc(self, blocking, is_push, payload, rpc_callback, **kwargs):
        """
        Every PIMU RPC leaves through here. The transport is shared by every board and
        has no device-aware reply hook, so this is the one place a rejected reply
        (id 38, five bytes, firmware v0.1.26p14 and later) is caught before it reaches
        the decoder the request expected.
        """
        def on_reply(reply):
            if len(reply) == self.RPC_REJECTED_REPLY_BYTES and reply[0] == self.RPC_REPLY_REJECTED:
                self._record_rpc_rejected(reply)
            else:
                rpc_callback(reply)
        return self.transport.do_rpc(blocking=blocking, is_push=is_push, payload=payload,
                                     rpc_callback=on_reply, **kwargs)

    def _record_rpc_rejected(self, reply):
        request_id = reply[1]
        reason = reply[2]
        length = unpack_uint16_t(reply[3:])
        rejected = self.status['rpc_rejected']
        rejected['count'] += 1
        rejected['last_request_id'] = request_id
        rejected['last_reason'] = reason
        rejected['last_length'] = length
        key = (request_id, reason)
        if key not in self._warned_rpc_rejections:
            self._warned_rpc_rejections.add(key)
            self.logger.warning('PIMU rejected RPC %d: %s (reason %d), request length %d bytes' % (
                request_id, self.RPC_REJECT_REASON_NAMES.get(reason, 'unknown reason'), reason, length))

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

    def restore_eoa_power(self):
        """
        Power the end-of-arm port again after the PIMU cut it on an overcurrent
        (status['eoa_output_tripped'], firmware v0.1.50p14). Goes out on the next
        push_command as actuator_control('eoa', True), the existing RPC_ACTUATOR_ENABLE.
        Clearing the runstop does the same in the firmware, unless the port tripped 3
        times within 60 s; this works in that case too. Remove the cause first: a real
        short trips it again.
        """
        self._dirty_eoa_restore = True

    def set_easter_egg_mode(self, on=True):
        """
        Set (on=False: clear) the PIMU Easter egg mode, firmware v0.1.51p14 and later.
        Goes out as trigger bit TRIGGER_EASTER_EGG_ON or _OFF on the next push_command;
        the last call before the push wins. Firmware before v0.1.51p14 ignores both bits.
        Read the mode back in status['easter_egg_mode']. Any runstop entry clears it on
        the board.
        """
        if on:
            self._trigger = (self._trigger & ~self.TRIGGER_EASTER_EGG_OFF) | self.TRIGGER_EASTER_EGG_ON
        else:
            self._trigger = (self._trigger & ~self.TRIGGER_EASTER_EGG_ON) | self.TRIGGER_EASTER_EGG_OFF
        self._dirty_trigger = True

    def clear_easter_egg_mode(self):
        self.set_easter_egg_mode(False)

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

    def set_lidar_off(self, source='client'):
        # Never send both bits for one rail; the last request wins.
        self._trigger = (self._trigger & ~self.TRIGGER_LIDAR_ON) | self.TRIGGER_LIDAR_OFF
        self._dirty_trigger = True
        self._note_lidar_cmd('off', source)

    def set_lidar_on(self, source='client'):
        self._trigger = (self._trigger & ~self.TRIGGER_LIDAR_OFF) | self.TRIGGER_LIDAR_ON
        self._dirty_trigger = True
        self._note_lidar_cmd('on', source)

    def release_lidar_hold(self, source='client'):
        """
        End a manual lidar hold (see sentry_lidar_idle) without switching the rail.
        Host side only, sends nothing to the PIMU.
        """
        self._note_lidar_cmd('release', source)

    def _note_lidar_cmd(self, action, source):
        # The board reports no lidar state; this is the last host request.
        self.lidar_cmd['action'] = action
        self.lidar_cmd['source'] = source
        self.lidar_cmd['ts'] = time.time()
        self.lidar_cmd['seq'] += 1

    def set_aux_cpu_off(self):
        self._trigger = (self._trigger & ~self.TRIGGER_20V0_AUX_ON) | self.TRIGGER_20V0_AUX_OFF
        self._dirty_trigger = True

    def set_aux_cpu_on(self):
        self._trigger = (self._trigger & ~self.TRIGGER_20V0_AUX_OFF) | self.TRIGGER_20V0_AUX_ON
        self._dirty_trigger = True

    def power_cycle_cpu(self):
        self._trigger = self._trigger | self.TRIGGER_CPU_PWR_CYCLE
        self._dirty_trigger = True

    def set_eye_animation(self, left_idx=None, right_idx=None, intensity=255, r=255, g=255, b=255):
        """
        Queue RPC_SET_EYE_ANIMATION for the next push_command. None for an eye sends
        EYE_ANIM_NOP (keep that eye). intensity and r, g, b are sent on every call.
        Runs inside the server control loop when called through the client, so a bad
        value is logged and the command dropped instead of raised: pack_uint8_t would
        otherwise end the loop. Returns whether the command was queued.
        """
        fields = [('left_idx', 0 if left_idx is None else left_idx), ('right_idx', 0 if right_idx is None else right_idx),
                  ('intensity', intensity), ('r', r), ('g', g), ('b', b)]
        clean = []
        for name, value in fields:
            try:
                byte = int(value)
                ok = byte == value and not isinstance(value, bool) and 0 <= byte <= 255
            except (TypeError, ValueError, OverflowError):
                ok = False
            if not ok:
                self.logger.warning('set_eye_animation: {} = {!r} is not an integer 0-255, command dropped'.format(name, value))
                return False
            clean.append(byte)
        (self._eye_animation_left, self._eye_animation_right, self._eye_animation_intensity,
         self._eye_animation_r, self._eye_animation_g, self._eye_animation_b) = clean
        self._dirty_eye_animation = True
        self.eye_animation_count += 1
        return True

    def eye_frame_supported(self):
        """True when the board's firmware has RPC 33 frame mode (v0.1.52p14 and later)."""
        from stretch4_body.core.factory.firmware_version import FirmwareVersion
        fw = FirmwareVersion(self.board_info.get('firmware_version') or '')
        return fw.valid and not fw < FirmwareVersion(self.EYE_FRAME_MIN_FIRMWARE)

    def set_eye_frame(self, left, right, brightness=(255, 255), hold_ms=500, sequence=None):
        """
        Show one per-pixel frame on both eye rings (RPC 33 frame mode, firmware
        v0.1.52p14 and later). left and right are 10 (r, g, b) tuples, 0..255, in ring
        order: index 0 just clockwise of 12 o'clock seen from the front, then clockwise.
        brightness scales each ring (255 passes the values through). hold_ms (50..2000)
        is how long the board keeps the frame without a newer one before the rings go
        back to their animation; stream faster than that to keep frame mode up.
        set_eye_animation (RPC 29) ends frame mode on the board.

        Goes out on the next push_command; a newer frame queued before then replaces
        this one. Returns the frame's sequence number (counts from 1 unless given).
        status['eye_frame'] has the board's answer: last_status, applied_sequence.
        status['eye_frame']['supported'] is set from the firmware version at startup; on
        older firmware nothing is sent.
        """
        rings = (left, right)
        rgb = []
        for ring in rings:
            if len(ring) != self.EYE_FRAME_PIXELS_PER_RING:
                raise ValueError('eye frame: %d pixels per ring, got %d' % (self.EYE_FRAME_PIXELS_PER_RING, len(ring)))
            for px in ring:
                if len(px) != 3 or any((not isinstance(v, (int, np.integer))) or v < 0 or v > 255 for v in px):
                    raise ValueError('eye frame: pixel must be three ints 0..255, got %r' % (px,))
                rgb.extend(int(v) for v in px)
        if len(brightness) != 2 or any(not 0 <= int(b) <= 255 for b in brightness):
            raise ValueError('eye frame: brightness is (left, right), 0..255, got %r' % (brightness,))
        if not self.EYE_FRAME_HOLD_MS_MIN <= int(hold_ms) <= self.EYE_FRAME_HOLD_MS_MAX:
            raise ValueError('eye frame: hold_ms %d outside %d..%d' % (
                hold_ms, self.EYE_FRAME_HOLD_MS_MIN, self.EYE_FRAME_HOLD_MS_MAX))
        if sequence is None:
            self._eye_frame_sequence = self._eye_frame_sequence % 0xFFFF + 1
            sequence = self._eye_frame_sequence
        elif not 0 <= int(sequence) <= 0xFFFF:
            raise ValueError('eye frame: sequence is u16, got %r' % (sequence,))
        self._eye_frame = struct.pack('<BBHHBB60B', self.RPC_SET_EYE_PER_EYE, self.EYE_CMD_FRAME,
                                      int(sequence), int(hold_ms), int(brightness[0]), int(brightness[1]), *rgb)
        return int(sequence)

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
            return self._do_rpc(blocking=blocking, is_push=True, payload=payload,
                                rpc_callback=self._rpc_actuator_enable_reply) is not None
        else:
            payload = arr.array('B', [self.RPC_ACTUATOR_DISABLE, motor_type])
            return self._do_rpc(blocking=blocking, is_push=True, payload=payload,
                                rpc_callback=self._rpc_actuator_disable_reply) is not None

    def _rpc_actuator_enable_reply(self, reply):
        if len(reply) == 0 or reply[0] != self.RPC_REPLY_ACTUATOR_ENABLE:
            print('Error RPC_REPLY_ACTUATOR_CONTROL', reply[:1])

    def _rpc_actuator_disable_reply(self, reply):
        if len(reply) == 0 or reply[0] != self.RPC_REPLY_ACTUATOR_DISABLE:
            print('Error RPC_REPLY_ACTUATOR_CONTROL', reply[:1])




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
        return self._do_rpc(blocking=True,is_push=True,payload=payload[:sidx],rpc_callback=self.rpc_network_info_reply) is not None

    def rpc_network_info_reply(self, reply):
        if len(reply) == 0 or reply[0] != self.RPC_REPLY_NETWORK_INFO:
            print('Error RPC_REPLY_NETWORK_INFO', reply[:1])

    def rpc_firebase_info_reply(self, reply):
        if len(reply) == 0 or reply[0] != self.RPC_FIREBASE_DB_INFO_REPLY:
            print('Error RPC_FIREBASE_DB_INFO_REPLY', reply[:1])

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
            self._do_rpc(blocking=True, is_push=False, payload=payload,rpc_callback=self._rpc_read_firmware_trace_reply)
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
        self.status_aux = {
            'current_pwr_state': 0,
            'current_pwr_state_name': 'STATE_ACTIVE',
            'runstop_cause_raw': 0,
            'motor_sync_cnt': 0,
            'battery_cell_temp_1': 0,
            'battery_cell_temp_2': 0,
            'battery_cell_temp_3': 0,
            'battery_ambient_temp': 0,
            'battery_mosfet_temp': 0,
            '_valid': False,
            '_packet_length': None,
            '_decode_error': None,
            '_diagnostics_present': False,
        }
        self.status_aux_reply_sequence = 0
        self.status_aux_reply_monotonic_s = None
        self.power_diagnostics_reply_sequence = 0
        self.power_diagnostics_reply_monotonic_s = None
        self.power_event_log_reply_sequence = 0
        self.power_event_log_reply_monotonic_s = None
        self.power_diagnostics = {
            '_valid': False,
            '_packet_length': None,
            '_decode_error': None,
            'schema_version': None,
        }
        # Lowered for good when the firmware answers unsupported-schema (see
        # _unpack_power_event_log).
        self.power_diagnostics_request_schema = self.POWER_DIAGNOSTICS_SCHEMA_VERSION
        self._power_diagnostics_retry = False
        self.power_event_log_page = {
            '_valid': False,
            '_packet_length': None,
            '_decode_error': None,
            'events': [],
        }
        self._load_test_payload = arr.array('B', range(256)) * 4

    # Background telemetry pulls are collected without waiting in the server's
    # control loop; the server gives up on a reply after this long.
    POWER_DIAGNOSTICS_RPC_TIMEOUT_S = 2.0

    def pull_status_aux(self, blocking=True):
        if not self.hw_valid:
            return False
        payload = arr.array('B', [self.RPC_GET_POWER_PERIPH_STATUS_AUX])
        return self._do_rpc(blocking=blocking, is_push=False, payload=payload,
                            rpc_callback=self._rpc_status_aux_reply,
                            wait_on_collect=False,
                            timeout_s=self.POWER_DIAGNOSTICS_RPC_TIMEOUT_S) is not None

    def pull_power_diagnostics(self, blocking=True, after_sequence=0, max_entries=16):
        """Pull the protocol-14 snapshot and a cursor-paged event-log page.

        Asks for schema 3. Firmware that answers unsupported-schema is asked once more
        with the schema its error header names when that is lower (2 from v0.1.47p14 to
        v0.1.52p14), else with schema 1 (v0.1.18p14 to v0.1.23p14), and that schema is
        kept from then on: within this call when blocking, on the next pull otherwise.
        """
        if not self.hw_valid or not self.supports_power_diagnostics():
            return False
        queued = self._request_power_diagnostics(blocking, after_sequence, max_entries)
        if blocking and self._power_diagnostics_retry:
            queued = self._request_power_diagnostics(blocking, after_sequence, max_entries)
        return queued

    def _request_power_diagnostics(self, blocking, after_sequence, max_entries):
        self._power_diagnostics_retry = False
        schema = self.power_diagnostics_request_schema
        max_entries = max(0, min(self.POWER_EVENT_LOG_MAX_RPC_ENTRIES,
                                 int(max_entries)))
        payload = arr.array('B', [0] * 9)
        payload[0] = self.RPC_GET_POWER_DIAGNOSTICS
        pack_uint8_t(payload, 1, schema)
        pack_uint8_t(payload, 2, max_entries)
        pack_uint16_t(payload, 3, 0)
        pack_uint32_t(payload, 5, int(after_sequence) & 0xFFFFFFFF)
        return self._do_rpc(
            blocking=blocking,
            is_push=False,
            payload=payload,
            rpc_callback=lambda reply: self._rpc_power_diagnostics_reply(reply, requested_schema=schema),
            wait_on_collect=False,
            timeout_s=self.POWER_DIAGNOSTICS_RPC_TIMEOUT_S,
        ) is not None

    def push_load_test(self, blocking=True):
        if not self.hw_valid:
            return False
        payload = self.transport.get_empty_payload()
        payload[0] = self.RPC_LOAD_TEST_PUSH
        payload[1:] = self._load_test_payload
        return self._do_rpc(blocking=blocking, is_push=True, payload=payload,
                            rpc_callback=self._rpc_load_test_push_reply)  is not None

    def pull_load_test(self, blocking=True, quiet=False):
        if not self.hw_valid:
            return False
        self.pull_load_test_quiet=quiet
        payload = arr.array('B', [self.RPC_LOAD_TEST_PULL])
        return self._do_rpc(blocking=blocking, is_push=False, payload=payload,
                            rpc_callback=self._rpc_load_test_pull_reply)  is not None

    def _rpc_status_aux_reply(self, reply):
        self.status_aux_reply_sequence += 1
        self.status_aux_reply_monotonic_s = time.monotonic()
        if len(reply) > 0 and reply[0] == self.RPC_REPLY_POWER_PERIPH_STATUS_AUX:
            self._unpack_status_aux(reply[1:])
        else:
            self.status_aux['_valid'] = False
            self.status_aux['_decode_error'] = 'unexpected or empty aux reply'
            self.logger.warning('Error RPC_REPLY_POWER_PERIPH_AUX_STATUS')

    def _rpc_power_diagnostics_reply(self, reply, requested_schema=None):
        now = time.monotonic()
        self.power_diagnostics_reply_sequence += 1
        self.power_diagnostics_reply_monotonic_s = now
        self.power_event_log_reply_sequence += 1
        self.power_event_log_reply_monotonic_s = now
        if len(reply) > 0 and reply[0] == self.RPC_REPLY_POWER_DIAGNOSTICS:
            consumed = self._unpack_power_event_log(reply[1:], requested_schema)
            if consumed > 0:
                self._mirror_power_diagnostics_to_status_aux()
                return
        else:
            self.power_diagnostics.clear()
            self.power_diagnostics.update({
                '_valid': False,
                '_packet_length': max(0, len(reply) - 1),
                '_decode_error': 'unexpected or empty power diagnostics reply',
            })
            self.power_event_log_page.clear()
            self.power_event_log_page.update({
                '_valid': False,
                '_packet_length': max(0, len(reply) - 1),
                '_decode_error': 'unexpected or empty power diagnostics reply',
                'events': [],
            })
            self.logger.warning('Error RPC_REPLY_POWER_DIAGNOSTICS')
        self.status_aux.update({
            '_valid': False,
            '_diagnostics_present': False,
            '_decode_error': self.power_diagnostics.get('_decode_error'),
        })

    def _mirror_power_diagnostics_to_status_aux(self):
        diagnostics = self.power_diagnostics
        if not diagnostics.get('_valid'):
            self.status_aux.update({
                '_valid': False,
                '_diagnostics_present': False,
                '_decode_error': diagnostics.get('_decode_error'),
            })
            return
        self.status_aux.update({
            'current_pwr_state': diagnostics['power_state'],
            'current_pwr_state_name': diagnostics['power_state_name'],
            'runstop_cause_raw': diagnostics['runstop_cause_raw'],
            'motor_sync_cnt': diagnostics['motor_sync_cnt'],
            'battery_cell_temp_1': diagnostics['battery_cell_temp_1_c'],
            'battery_cell_temp_2': diagnostics['battery_cell_temp_2_c'],
            'battery_cell_temp_3': diagnostics['battery_cell_temp_3_c'],
            'battery_ambient_temp': diagnostics['battery_ambient_temp_c'],
            'battery_mosfet_temp': diagnostics['battery_mosfet_temp_c'],
            '_valid': True,
            # No 9-byte auxiliary packet was received. Preserve its source
            # honestly while exposing which diagnostics packet populated it.
            '_packet_length': None,
            '_diagnostics_packet_length': diagnostics['_packet_length'],
            '_decode_error': None,
            '_diagnostics_present': True,
        })

    def _unpack_status_aux(self, s):
        # take in an array of bytes
        # This exactly matches packed firmware Pimu_Status_Aux.
        self.status_aux['_packet_length'] = len(s)
        if len(s) < 9:
            self.status_aux['_valid'] = False
            self.status_aux['_decode_error'] = (
                'short aux packet: %d bytes (expected at least 9)' % len(s)
            )
            return 0
        sidx = 0
        self.status_aux['current_pwr_state'] = unpack_uint8_t(s[sidx:])
        self.status_aux['current_pwr_state_name'] = self.get_power_state_name(
            self.status_aux['current_pwr_state']
        )
        sidx += 1
        self.status_aux['runstop_cause_raw'] = unpack_uint8_t(s[sidx:])
        sidx += 1
        self.status_aux['motor_sync_cnt'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        for key in (
            'battery_cell_temp_1',
            'battery_cell_temp_2',
            'battery_cell_temp_3',
            'battery_ambient_temp',
            'battery_mosfet_temp',
        ):
            value = unpack_uint8_t(s[sidx:])
            self.status_aux[key] = value - 256 if value > 127 else value
            sidx += 1
        self.status_aux['_valid'] = True
        self.status_aux['_decode_error'] = None
        self.status_aux['_diagnostics_present'] = False
        return sidx

    def supports_power_diagnostics(self):
        protocol = getattr(self, 'board_info', {}).get('protocol_version')
        try:
            return protocol.startswith('p') and int(protocol[1:]) >= 14
        except (AttributeError, TypeError, ValueError):
            return False

    @staticmethod
    def _signed_u8(value):
        return value - 256 if value > 127 else value

    @staticmethod
    def _decode_reset_causes(mask):
        # SAMD51 RSTC.RCAUSE bit assignments. Unknown bits remain visible in
        # reset_cause_raw so decoding never destroys evidence.
        names = (
            (1 << 0, 'power_on'),
            (1 << 1, 'brownout_core'),
            (1 << 2, 'brownout_vdd'),
            (1 << 3, 'nvm'),
            (1 << 4, 'external'),
            (1 << 5, 'watchdog'),
            (1 << 6, 'system_request'),
            (1 << 7, 'backup'),
        )
        return [name for bit, name in names if mask & bit]

    # Fields that exist from schema 2, None in a decoded schema 1 snapshot.
    POWER_DIAGNOSTICS_SCHEMA2_KEYS = (
        'charge_stop_state', 'eye_left', 'eye_right', 'eye_overlay', 'eye_reserved',
        'easter_egg_mode', 'shutdown_wait_active', 'shutdown_wait_last_end',
        'shutdown_wait_count', 'shutdown_wait_elapsed_ms', 'shutdown_wait_last_duration_ms',
        'shutdown_wait_cap_ms', 'adc_reinit_cnt', 'adc_stale', 'adapter_presence_source',
        'esp_rx_length_errors', 'esp_rx_unknown_ids', 'esp_tx_mark_resets',
        'esp_shutdown_cmd_sends', 'esp_adapter_disagree', 'wait_sd_ack', 'reserved2',
        'imu_tap_last', 'ina_vbat_mv', 'ina_current_ma', 'ina_flags', 'ina_reserved',
    )
    # Fields that exist only in schema 3, None in a decoded schema 1 or 2 snapshot.
    POWER_DIAGNOSTICS_SCHEMA3_KEYS = ('esp_version_state', 'esp_version_requests', 'esp_version')

    def _unpack_power_diagnostics(self, s):
        packet_length = len(s)
        schema = {size: v for v, size in self.POWER_DIAGNOSTICS_WIRE_SIZES.items()}.get(packet_length)
        if schema is None:
            self.power_diagnostics.clear()
            self.power_diagnostics.update({
                '_valid': False,
                '_packet_length': packet_length,
                '_decode_error': 'invalid diagnostics packet: %d bytes (expected %s)' % (
                    packet_length,
                    ' or '.join(str(n) for n in sorted(self.POWER_DIAGNOSTICS_WIRE_SIZES.values()))),
            })
            return 0

        sidx = 0

        def u8():
            nonlocal sidx
            value = unpack_uint8_t(s[sidx:])
            sidx += 1
            return value

        def i8():
            return self._signed_u8(u8())

        def u16():
            nonlocal sidx
            value = unpack_uint16_t(s[sidx:])
            sidx += 2
            return value

        def i16():
            nonlocal sidx
            value = unpack_int16_t(s[sidx:])
            sidx += 2
            return value

        def u32():
            nonlocal sidx
            value = unpack_uint32_t(s[sidx:])
            sidx += 4
            return value

        def u64():
            nonlocal sidx
            value = unpack_uint64_t(s[sidx:])
            sidx += 8
            return value

        def i32():
            nonlocal sidx
            value = unpack_int32_t(s[sidx:])
            sidx += 4
            return value

        decoded = {}
        decoded['schema_version'] = u8()
        decoded['reset_cause_raw'] = u8()
        decoded['struct_size'] = u16()
        if decoded['schema_version'] != schema or decoded['struct_size'] != packet_length:
            self.power_diagnostics.clear()
            self.power_diagnostics.update({
                '_valid': False,
                '_packet_length': packet_length,
                '_decode_error': 'unsupported diagnostics schema %d/size %d' % (
                    decoded['schema_version'], decoded['struct_size']),
            })
            return 0

        decoded['uptime_ms'] = u64()
        decoded['sample_sequence'] = u32()
        decoded['capability_flags_raw'] = u32()
        decoded['validity_flags_raw'] = u32()
        decoded['power_transition_sequence'] = u32()
        decoded['power_transition_timestamp_ms'] = u32()
        decoded['power_state'] = u8()
        decoded['runstop_cause_raw'] = u8()
        decoded['periph_power_state_raw'] = u8()
        decoded['cpu_on_status'] = bool(u8())
        decoded['power_transition_reason_raw'] = u16()
        decoded['power_button_high_count'] = u16()
        decoded['power_button_raw'] = bool(u8())
        decoded['power_button_debounced'] = bool(u8())
        decoded['adapter_voltage_present'] = bool(u8())
        decoded['charger_enable_command'] = bool(u8())
        decoded['charger_expected_active'] = bool(u8())
        decoded['user_charger_control'] = bool(u8())
        decoded['wait_wake_ack'] = bool(u8())
        decoded['wait_sleep_ack'] = bool(u8())
        decoded['esp_adapter_connected'] = bool(u8())
        decoded['esp_adapter_fault'] = bool(u8())
        decoded['esp_network_connected'] = bool(u8())
        decoded['esp_firebase_configured'] = bool(u8())

        decoded['bms_sample_sequence'] = u32()
        decoded['bms_last_valid_ms'] = u32()
        decoded['bms_sample_age_ms'] = u32()
        decoded['bms_valid_replies'] = u32()
        decoded['bms_crc_errors'] = u32()
        decoded['bms_timeouts'] = u32()
        decoded['bms_rx_overflows'] = u32()
        decoded['bms_protocol_errors'] = u32()
        decoded['bms_consecutive_failures'] = u32()
        decoded['battery_voltage_mv'] = i32()
        decoded['battery_current_ma'] = i32()
        decoded['charger_current_ma'] = i32()
        decoded['battery_soc'] = u8()
        decoded['battery_soh'] = u8()
        decoded['battery_cycles'] = u16()
        decoded['battery_cell_temp_1_c'] = i8()
        decoded['battery_cell_temp_2_c'] = i8()
        decoded['battery_cell_temp_3_c'] = i8()
        decoded['battery_ambient_temp_c'] = i8()
        decoded['battery_mosfet_temp_c'] = i8()
        decoded['bms_alarm_mask_raw'] = u8()
        decoded['bms_mos_mask_raw'] = u8()
        # Schema 1 reserved0; schema 2 charge_stop_state (low nibble state, bit 4 enabled).
        if schema >= 2:
            decoded['charge_stop_state'] = u8()
        else:
            decoded['reserved0'] = u8()
        decoded['bms_last_failure_rx_len'] = u16()
        decoded['bms_last_failure_event'] = u16()
        decoded['bms_last_failure_function'] = u8()
        decoded['bms_last_failure_exception_code'] = u8()
        decoded['bms_last_failure_address'] = u8()
        decoded['imu_tap_count'] = u8()

        decoded['esp_status_age_ms'] = u32()
        decoded['esp_rx_packets'] = u32()
        decoded['esp_crc_errors'] = u32()
        decoded['esp_rx_overflows'] = u32()
        decoded['esp_requests_sent'] = u32()
        decoded['esp_tx_queue_drops'] = u32()

        decoded['voltage_cpu_mv'] = i32()
        decoded['voltage_5v0_mv'] = i32()
        decoded['voltage_36v0_mv'] = i32()
        decoded['voltage_12v0_mv'] = i32()
        decoded['voltage_20v0_aux_mv'] = i32()
        decoded['current_cpu_ma'] = i32()
        decoded['pimu_temp_centi_c'] = i16()
        decoded['motor_sync_cnt'] = u16()
        decoded['loop_time_us'] = u16()
        decoded['loop_time_max_us'] = u16()
        decoded['loop_overrun_count'] = u32()
        decoded['diagnostics_rpc_count'] = u32()
        decoded['diagnostics_reply_bytes'] = u16()
        decoded['diagnostics_lock_time_us'] = u16()
        decoded['diagnostics_lock_time_max_us'] = u16()

        if schema >= 2:
            def eye_ring():
                return {key: u8() for key in ('animation', 'intensity', 'r', 'g', 'b', 'source')}

            decoded['eye_left'] = eye_ring()
            decoded['eye_right'] = eye_ring()
            decoded['eye_overlay'] = u8()
            decoded['eye_reserved'] = u8()
            decoded['easter_egg_mode'] = u8()
            decoded['shutdown_wait_active'] = bool(u8())
            decoded['shutdown_wait_last_end'] = u8()
            decoded['shutdown_wait_count'] = u16()
            decoded['shutdown_wait_elapsed_ms'] = u32()
            decoded['shutdown_wait_last_duration_ms'] = u32()
            decoded['shutdown_wait_cap_ms'] = u32()
            decoded['adc_reinit_cnt'] = [u8(), u8()]
            decoded['adc_stale'] = u8()
            decoded['adapter_presence_source'] = u8()
            decoded['esp_rx_length_errors'] = u32()
            decoded['esp_rx_unknown_ids'] = u32()
            decoded['esp_tx_mark_resets'] = u32()
            decoded['esp_shutdown_cmd_sends'] = u32()
            decoded['esp_adapter_disagree'] = u8()
            decoded['wait_sd_ack'] = bool(u8())
            decoded['reserved2'] = u8()
            decoded['imu_tap_last'] = u8()
            decoded['ina_vbat_mv'] = i32()
            decoded['ina_current_ma'] = i32()
            decoded['ina_flags'] = u8()
            decoded['ina_reserved'] = u8()
        else:
            for key in self.POWER_DIAGNOSTICS_SCHEMA2_KEYS:
                decoded[key] = None

        if schema >= 3:
            decoded['esp_version_state'] = u8()
            decoded['esp_version_requests'] = u8()
            raw = bytes(s[sidx:sidx + 32])
            sidx += 32
            # NUL padded; the firmware keeps it printable ASCII. Empty unless reported.
            decoded['esp_version'] = raw.split(b'\0', 1)[0].decode('ascii', 'replace')
        else:
            for key in self.POWER_DIAGNOSTICS_SCHEMA3_KEYS:
                decoded[key] = None

        if sidx != packet_length:
            self.power_diagnostics.update({
                '_valid': False,
                '_packet_length': packet_length,
                '_decode_error': 'diagnostics decoder consumed %d bytes (expected %d)' % (
                    sidx, packet_length),
            })
            return 0

        caps = decoded['capability_flags_raw']
        validity = decoded['validity_flags_raw']
        decoded['capabilities'] = {
            'bms_pack': bool(caps & (1 << 0)),
            'bms_temperatures': bool(caps & (1 << 1)),
            'bms_alarms': bool(caps & (1 << 2)),
            'bms_mos_states': bool(caps & (1 << 3)),
            'event_log': bool(caps & (1 << 4)),
            'esp_status': bool(caps & (1 << 5)),
            'bms_cell_voltages': bool(caps & (1 << 6)),
            'bms_capacity': bool(caps & (1 << 7)),
            'bms_balancing': bool(caps & (1 << 8)),
            'bench_rig_build': bool(caps & (1 << 9)),
            'eye_readback': bool(caps & (1 << 10)),
            'ina_vbat': bool(caps & (1 << 11)),
            'simulated_battery': bool(caps & (1 << 12)),
        }
        decoded['validity'] = {
            'bms_sample': bool(validity & (1 << 0)),
            'esp_status': bool(validity & (1 << 1)),
            'analog_rails': bool(validity & (1 << 2)),
            'bms_extended_sample': bool(validity & (1 << 3)),
            'ina_sample': bool(validity & (1 << 4)),
        }
        decoded['reset_causes'] = self._decode_reset_causes(decoded['reset_cause_raw'])
        decoded['power_state_name'] = self.get_power_state_name(decoded['power_state'])
        decoded['power_transition_reason'] = self.POWER_TRANSITION_REASON_NAMES.get(
            decoded['power_transition_reason_raw'],
            'unknown_%d' % decoded['power_transition_reason_raw'],
        )
        transition_uptime_ms = (
            (decoded['uptime_ms'] & ~0xFFFFFFFF) |
            decoded['power_transition_timestamp_ms']
        )
        if (transition_uptime_ms > decoded['uptime_ms'] and
                transition_uptime_ms >= (1 << 32)):
            transition_uptime_ms -= (1 << 32)
        decoded['power_transition_uptime_ms'] = transition_uptime_ms
        periph_raw = decoded['periph_power_state_raw']
        periph_decoder = getattr(self, 'get_periph_power_state', None)
        decoded['periph_power_state'] = (
            periph_decoder(periph_raw) if periph_decoder is not None else {
                'power_to_lift': bool(periph_raw & (1 << 1)),
                'power_to_omni_0_motor': bool(periph_raw & (1 << 2)),
                'power_to_omni_1_motor': bool(periph_raw & (1 << 3)),
                'power_to_omni_2_motor': bool(periph_raw & (1 << 4)),
                'power_to_arm': bool(periph_raw & (1 << 5)),
                'power_to_eoa': bool(periph_raw & (1 << 6)),
            }
        )
        decoded['bms_alarms'] = {
            'total_voltage_high': bool(decoded['bms_alarm_mask_raw'] & (1 << 0)),
            'cell_voltage_high': bool(decoded['bms_alarm_mask_raw'] & (1 << 1)),
            'cell_voltage_difference_high': bool(decoded['bms_alarm_mask_raw'] & (1 << 2)),
        }
        decoded['bms_mos_states'] = {
            'charge_current_limit': bool(decoded['bms_mos_mask_raw'] & (1 << 0)),
            'pre_discharge': bool(decoded['bms_mos_mask_raw'] & (1 << 1)),
            'discharging': bool(decoded['bms_mos_mask_raw'] & (1 << 2)),
            'charging': bool(decoded['bms_mos_mask_raw'] & (1 << 3)),
        }
        decoded.update(self._decode_schema2_enums(decoded))
        decoded['battery_voltage_v'] = decoded['battery_voltage_mv'] / 1000.0
        decoded['battery_current_a'] = decoded['battery_current_ma'] / 1000.0
        decoded['charger_current_a'] = decoded['charger_current_ma'] / 1000.0
        decoded['voltage_cpu_v'] = decoded['voltage_cpu_mv'] / 1000.0
        decoded['voltage_5v0_v'] = decoded['voltage_5v0_mv'] / 1000.0
        decoded['voltage_36v0_v'] = decoded['voltage_36v0_mv'] / 1000.0
        decoded['voltage_12v0_v'] = decoded['voltage_12v0_mv'] / 1000.0
        decoded['voltage_20v0_aux_v'] = decoded['voltage_20v0_aux_mv'] / 1000.0
        decoded['current_cpu_a'] = decoded['current_cpu_ma'] / 1000.0
        decoded['pimu_temp_c'] = decoded['pimu_temp_centi_c'] / 100.0
        decoded['bms_sample_age_s'] = (
            None if decoded['bms_sample_age_ms'] == 0xFFFFFFFF
            else decoded['bms_sample_age_ms'] / 1000.0
        )
        decoded['esp_status_age_s'] = (
            None if decoded['esp_status_age_ms'] == 0xFFFFFFFF
            else decoded['esp_status_age_ms'] / 1000.0
        )
        decoded.update({
            '_valid': True,
            '_packet_length': packet_length,
            '_decode_error': None,
        })
        self.power_diagnostics.clear()
        self.power_diagnostics.update(decoded)
        return sidx

    @classmethod
    def _decode_schema2_enums(cls, decoded):
        """Names beside the raw schema 2 enum values, None where the field is None."""
        def name(table, raw):
            return None if raw is None else table.get(raw, 'unknown_%d' % raw)

        charge_stop = decoded.get('charge_stop_state')
        return {
            'adapter_presence_source_name': name(
                cls.ADAPTER_PRESENCE_SOURCE_NAMES, decoded.get('adapter_presence_source')),
            'shutdown_wait_last_end_name': name(
                cls.SHUTDOWN_WAIT_END_NAMES, decoded.get('shutdown_wait_last_end')),
            'esp_adapter_disagree_name': name(
                cls.ESP_ADAPTER_DISAGREE_NAMES, decoded.get('esp_adapter_disagree')),
            # Low nibble is the policy state, bit 4 says the policy is compiled in.
            'charge_stop_state_name': name(
                cls.CHARGE_STOP_STATE_NAMES, None if charge_stop is None else charge_stop & 0x0F),
            'charge_stop_enabled': None if charge_stop is None else bool(charge_stop & 0x10),
            'esp_version_state_name': name(
                cls.ESP_VERSION_STATE_NAMES, decoded.get('esp_version_state')),
        }

    def _unpack_power_event_log(self, s, requested_schema=None):
        packet_length = len(s)
        if packet_length < self.POWER_EVENT_LOG_HEADER_SIZE:
            error = 'short event-log header: %d bytes' % packet_length
            self.power_diagnostics.clear()
            self.power_diagnostics.update({
                '_valid': False,
                '_packet_length': packet_length,
                '_decode_error': error,
            })
            self.power_event_log_page.clear()
            self.power_event_log_page.update({
                '_valid': False,
                '_packet_length': packet_length,
                '_decode_error': error,
                'events': [],
            })
            return 0

        sidx = 0
        schema_version = unpack_uint8_t(s[sidx:]); sidx += 1
        header_size = unpack_uint8_t(s[sidx:]); sidx += 1
        diagnostics_size = unpack_uint16_t(s[sidx:]); sidx += 2
        entry_size = unpack_uint8_t(s[sidx:]); sidx += 1
        entry_count = unpack_uint8_t(s[sidx:]); sidx += 1
        has_more = bool(unpack_uint8_t(s[sidx:])); sidx += 1
        reply_status = unpack_uint8_t(s[sidx:]); sidx += 1
        capacity = unpack_uint16_t(s[sidx:]); sidx += 2
        retained_count = unpack_uint16_t(s[sidx:]); sidx += 2
        oldest_sequence = unpack_uint32_t(s[sidx:]); sidx += 4
        newest_sequence = unpack_uint32_t(s[sidx:]); sidx += 4
        dropped_count = unpack_uint32_t(s[sidx:]); sidx += 4
        expected_length = header_size + diagnostics_size + entry_count * entry_size
        if reply_status != self.POWER_DIAGNOSTICS_REPLY_OK:
            status_name = self.POWER_DIAGNOSTICS_REPLY_STATUS_NAMES.get(
                reply_status, 'unknown_%d' % reply_status)
            # The header of an error reply carries the newest schema the firmware
            # speaks (1 on v0.1.18p14, 2 from v0.1.24p14, 3 from v0.1.53p14), not the
            # one asked for.
            error_shape_valid = (
                schema_version >= 1 and
                header_size == self.POWER_EVENT_LOG_HEADER_SIZE and
                diagnostics_size == 0 and
                entry_size == self.POWER_EVENT_LOG_ENTRY_SIZE and
                entry_count == 0 and
                packet_length == self.POWER_EVENT_LOG_HEADER_SIZE
            )
            error = (
                'firmware rejected power diagnostics request: %s (firmware schema %d)' % (
                    status_name, schema_version)
                if error_shape_valid else
                'malformed power diagnostics error reply'
            )
            if (error_shape_valid and
                    reply_status == self.POWER_DIAGNOSTICS_REPLY_UNSUPPORTED_SCHEMA and
                    requested_schema is not None and requested_schema > 1 and
                    self.power_diagnostics_request_schema == requested_schema):
                # Step down to the firmware's newest schema when it is one we decode,
                # else to schema 1. Always lower than asked, so this cannot loop.
                fallback = (schema_version
                            if schema_version < requested_schema and
                            schema_version in self.POWER_DIAGNOSTICS_WIRE_SIZES else 1)
                self.power_diagnostics_request_schema = fallback
                self._power_diagnostics_retry = True
                error += '; schema %d not supported, asking for schema %d' % (
                    requested_schema, fallback)
                logger = getattr(self, 'logger', None)
                if logger is not None:
                    logger.warning('PIMU does not support power diagnostics schema %d '
                                   '(firmware schema %d); using schema %d' % (
                                       requested_schema, schema_version, fallback))
            self.power_diagnostics.clear()
            self.power_diagnostics.update({
                '_valid': False,
                '_packet_length': packet_length,
                '_decode_error': error,
            })
            self.power_event_log_page.clear()
            self.power_event_log_page.update({
                '_valid': False,
                '_packet_length': packet_length,
                '_decode_error': error,
                'reply_status': reply_status,
                'reply_status_name': status_name,
                'events': [],
            })
            return 0
        if (schema_version not in self.POWER_DIAGNOSTICS_WIRE_SIZES or
                header_size != self.POWER_EVENT_LOG_HEADER_SIZE or
                diagnostics_size != self.POWER_DIAGNOSTICS_WIRE_SIZES[schema_version] or
                entry_size != self.POWER_EVENT_LOG_ENTRY_SIZE or
                entry_count > self.POWER_EVENT_LOG_MAX_RPC_ENTRIES or
                packet_length != expected_length):
            error = 'unsupported/malformed event log schema=%d entry_size=%d count=%d' % (
                schema_version, entry_size, entry_count)
            self.power_diagnostics.clear()
            self.power_diagnostics.update({
                '_valid': False,
                '_packet_length': packet_length,
                '_decode_error': error,
            })
            self.power_event_log_page.clear()
            self.power_event_log_page.update({
                '_valid': False,
                '_packet_length': packet_length,
                '_decode_error': error,
                'events': [],
            })
            return 0

        # diagnostics_size matched the header's schema, so the snapshot decoder's own
        # schema_version and struct_size check holds the two to the same schema.
        if self._unpack_power_diagnostics(s[header_size:header_size + diagnostics_size]) == 0:
            self.power_event_log_page.clear()
            self.power_event_log_page.update({
                '_valid': False,
                '_packet_length': packet_length,
                '_decode_error': self.power_diagnostics.get('_decode_error'),
                'events': [],
            })
            return 0
        sidx = header_size + diagnostics_size

        source_names = self.POWER_EVENT_SOURCE_NAMES
        events = []
        for _ in range(entry_count):
            sequence = unpack_uint32_t(s[sidx:]); sidx += 4
            timestamp_ms = unpack_uint32_t(s[sidx:]); sidx += 4
            event_id = unpack_uint16_t(s[sidx:]); sidx += 2
            source = unpack_uint8_t(s[sidx:]); sidx += 1
            power_state = unpack_uint8_t(s[sidx:]); sidx += 1
            data0 = unpack_int16_t(s[sidx:]); sidx += 2
            data1 = unpack_int16_t(s[sidx:]); sidx += 2
            event = {
                'sequence': sequence,
                'timestamp_ms': timestamp_ms,
                'timestamp_s': timestamp_ms / 1000.0,
                'event_id': event_id,
                'event_name': self.POWER_EVENT_NAMES.get(event_id, 'unknown_%d' % event_id),
                'explanation': self.POWER_EVENT_EXPLANATIONS.get(event_id, 'Unknown firmware event.'),
                'source_raw': source,
                'source': source_names.get(source, 'unknown_%d' % source),
                'power_state': power_state,
                'power_state_name': self.get_power_state_name(power_state),
                'data0': data0,
                'data1': data1,
            }
            if event_id == 2:
                event['from_state_name'] = self.get_power_state_name(data0)
                event['transition_reason'] = self.POWER_TRANSITION_REASON_NAMES.get(
                    data1, 'unknown_%d' % data1)
            events.append(event)

        self.power_event_log_page.clear()
        self.power_event_log_page.update({
            '_valid': True,
            '_packet_length': packet_length,
            '_decode_error': None,
            'schema_version': schema_version,
            'header_size': header_size,
            'diagnostics_size': diagnostics_size,
            'entry_size': entry_size,
            'entry_count': entry_count,
            'has_more': has_more,
            'reply_status': reply_status,
            'reply_status_name': self.POWER_DIAGNOSTICS_REPLY_STATUS_NAMES[reply_status],
            'capacity': capacity,
            'retained_count': retained_count,
            'oldest_sequence': oldest_sequence,
            'newest_sequence': newest_sequence,
            'dropped_count': dropped_count,
            'events': events,
        })
        return sidx

    @staticmethod
    def get_power_state_name(raw):
        states = (
            'STATE_ACTIVE',
            'STATE_SLEEP',
            'STATE_SHUTDOWN_CHRG',
            'STATE_SLEEP_CHRG',
            'STATE_SHUTDOWN',
        )
        if 0 <= raw < len(states):
            return states[raw]
        return 'UNKNOWN_%d' % raw
    
    def _rpc_load_test_push_reply(self, reply):
        if len(reply) == 0 or reply[0] != self.RPC_REPLY_LOAD_TEST_PUSH:
            print('Error RPC_REPLY_LOAD_TEST_PUSH', reply[:1])

    def _rpc_load_test_pull_reply(self, reply):
        if len(reply) > 0 and reply[0] == self.RPC_REPLY_LOAD_TEST_PULL:
            d = reply[1:]
            for i in range(1024):
                if d[i] != self._load_test_payload[(i + 1) % 1024]:
                    print('Load test pull bad data', d[i], self._load_test_payload[(i + 1) % 1024])
            self._load_test_payload = d
            if not self.pull_load_test_quiet:
                print('Successful load test pull')
        else:
            print('Error RPC_REPLY_LOAD_TEST_PULL', reply[:1])


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
        self._dirty_eoa_restore = False
        self._eye_frame = None
        self._eye_frame_sequence = 0
        self._eye_animation_left = 0
        self._eye_animation_right = 0
        self._eye_animation_intensity = 255
        self._eye_animation_r = 255
        self._eye_animation_g = 255
        self._eye_animation_b = 255
        self.eye_animation_count = 0

        if usb is None:
            usb = self.params['usb_name']
        if backend is None:
            backend=self.params['transport']['default_backend']
        self.transport = Transport(port_name=usb, logger=self.logger,
                                   default_backend=backend,
                                   qid=self.params['transport']['qid'])
        self.lidar_cmd = {'action': None, 'source': None, 'ts': None, 'seq': 0}
        self.status: "PowerPeriphStatus" = {'voltage': 0, 'current': 0, 'temp': 0, 'cpu_temp': 0, 'frame_id': 0,
                       'timestamp': 0,  'runstop_event': False,
                       'bump_event_cnt': 0,
                       'fan_on': False, 'buzzer_on': False, 'low_soc_alert': False,
                       'high_current_alert': False, 'high_current_eoa_alert': False,
                       'eoa_output_tripped': False,
                       'adapter_voltage_present': False, 'boot_detected': False, 'imu': self.imu.status, 'debug': 0,
                       'state': 0, 'trace_on': 0,
                       'motor_sync_rate': 0, 'motor_sync_cnt': 0, 'motor_sync_queues': 0, 'motor_sync_drop': 0,
                       'transport': self.transport.status, 'current_charge': 0, 'current_eoa':0,'charger_is_charging': False,
                       'periph_power_state':{},
                       'over_tilt_type': 0, 'battery_current': 0, 'battery_soc':0, 'battery_soh':0,
                       'battery_chrg_dischrg_cycles': 0, 'voltage_cpu':0, 'voltage_5v0':0, 'voltage_36v0':0,
                       'voltage_12v0':0, 'voltage_aux_cpu':0, 'current_cpu':0, 'cpu_on_sts': False,
                       'runstop_cause':0, 'connected_to_network':False, 'connected_to_firebase': False,'adapter_fault':False,'adapter_connected':False,
                       'us_loop_time':0, 'easter_egg_mode': 0,
                       'eye_frame': {'supported': None, 'sent': 0, 'last_status': None,
                                     'applied_sequence': None},
                       'rpc_reply_errors': 0, 'rpc_reply_last_error': None,
                       'status_aux': self.status_aux,
                       'power_diagnostics': self.power_diagnostics,
                       'lidar_cmd': self.lidar_cmd,
                       'rpc_rejected': {'count': 0, 'last_request_id': None, 'last_reason': None, 'last_length': None}}

        self._warned_rpc_rejections = set()
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
                self._do_rpc(blocking=True,is_push=False,payload=payload, rpc_callback=self._rpc_board_info_reply,backend=self.transport.BACKEND_PY_SERIAL) #Use py as C may not be supported yet
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
        return self._do_rpc(blocking=blocking, is_push=False, payload=payload, rpc_callback=self._rpc_status_reply) is not None


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
            success = success and self._do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx],rpc_callback=self._rpc_imu_config_reply) is not None
            self.imu._dirty_config = False

        if self._dirty_config:
            payload[0] = self.RPC_SET_POWER_PERIPH_CONFIG
            sidx = self._pack_config(payload, 1)
            success=success and self._do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx], rpc_callback=self._rpc_config_reply) is not None
            self._dirty_config = False

        if self._dirty_trigger:
            payload[0] = self.RPC_SET_POWER_PERIPH_TRIGGER
            sidx = self._pack_trigger(payload, 1)
            success=success and self._do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx],rpc_callback=self._rpc_trigger_reply) is not None
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
            success = success and self._do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx], rpc_callback=self._rpc_eye_animation_reply) is not None
            self._eye_animation_left = 0
            self._eye_animation_right = 0
            self._eye_animation_intensity = 255
            self._eye_animation_r = 255
            self._eye_animation_g = 255
            self._eye_animation_b = 255
            self._dirty_eye_animation = False

        if self._eye_frame is not None:
            frame, self._eye_frame = self._eye_frame, None
            supported = self.eye_frame_supported()
            self.status['eye_frame']['supported'] = supported
            if supported:
                payload[:len(frame)] = arr.array('B', frame)
                self.status['eye_frame']['sent'] += 1
                success = success and self._do_rpc(blocking=blocking, is_push=True, payload=payload[:len(frame)],
                                                   rpc_callback=self._rpc_eye_frame_reply) is not None

        if self._dirty_eoa_restore:
            self._dirty_eoa_restore = False
            success = success and bool(self.actuator_control('eoa', True, blocking=blocking))

        return success

    def load_rpc_results(self,wait_on_result=True):
        if not self.hw_valid:
            return []
        res = self.transport.load_rpc_results(wait_on_result)
        if 0 in res:
            if check_usb_disconnection(self.transport.port_name, self.name, self.logger):
                self.hw_valid = False
        return res

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
        rpc_id = self._do_rpc(blocking=blocking, is_push=True, payload=payload,
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
        # Known from the version at startup, so a client can read it before sending a frame.
        self.status['eye_frame']['supported'] = self.eye_frame_supported()
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
        if unpack_to is self.status:
            imu_to = self.imu.status
        else:
            imu_to = unpack_to.setdefault('imu', dict(self.imu.status))
        sidx += self.imu._unpack_status(s[sidx:], unpack_to=imu_to)

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
        unpack_to['easter_egg_mode'] = 1 if (unpack_to['state'] & self.STATE_EASTER_EGG_MODE) else 0
        unpack_to['fan_on'] = (unpack_to['state'] & self.STATE_FAN_ON) != 0
        unpack_to['buzzer_on'] = (unpack_to['state'] & self.STATE_BUZZER_ON) != 0
        unpack_to['low_soc_alert'] = (unpack_to['state'] & self.STATE_LOW_SOC_ALERT) != 0
        unpack_to['high_current_alert'] = (unpack_to['state'] & self.STATE_HIGH_CURRENT_ALERT) != 0
        unpack_to['high_current_eoa_alert'] = (unpack_to['state'] & self.STATE_HIGH_CURRENT_EOA_ALERT) != 0
        unpack_to['eoa_output_tripped'] = (unpack_to['state'] & self.STATE_EOA_OUTPUT_TRIPPED) != 0
        if self.board_info['hardware_id'] > 0:
            unpack_to['adapter_voltage_present'] = (unpack_to['state'] & self.STATE_ADAPTER_VOLTAGE_PRESENT) != 0
            unpack_to['boot_detected'] = (unpack_to['state'] & self.STATE_BOOT_DETECTED) != 0
        unpack_to['trace_on'] = (unpack_to['state'] & self.STATE_IS_TRACE_ON) != 0
        unpack_to['connected_to_network'] = (unpack_to['state'] & self.STATE_CONNECTED_TO_NETWORK) != 0
        unpack_to['adapter_fault'] = (unpack_to['state'] & self.STATE_ADAPTER_FAULT) != 0
        unpack_to['adapter_connected'] = (unpack_to['state'] & self.STATE_ADAPTER_CONNECTED) != 0

        unpack_to['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:]))
        sidx += 8
        imu_to['timestamp'] = unpack_to['timestamp']
        unpack_to['bump_event_cnt'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        unpack_to['debug'] = unpack_float_t(s[sidx:])
        sidx += 4
        unpack_to['current_charge'] = self.get_current_charge(unpack_float_t(s[sidx:]))
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
    # Payload bytes after the reply id. None means the length is not pinned for
    # this protocol and only decode errors are rejected.
    STATUS_PAYLOAD_BYTES = None
    MOTOR_SYNC_PAYLOAD_BYTES = 2   # Pimu_Motor_Sync_Reply: uint16 motor_sync_cnt
    TRIGGER_PAYLOAD_BYTES = 4      # Pimu_Trigger echo: uint32 data

    def _reply_is_valid(self, name, reply, expected_id, payload_bytes=None):
        """
        Check a reply's id and, when payload_bytes is given, its exact length
        before anything is decoded. A rejected reply bumps rpc_reply_errors and
        leaves every status field as it was.
        """
        if len(reply) == 0:
            return self._reject_reply(name, 'empty reply')
        if reply[0] != expected_id:
            return self._reject_reply(name, 'reply id %d, expected %d' % (reply[0], expected_id))
        if payload_bytes is not None and len(reply) - 1 != payload_bytes:
            return self._reject_reply(
                name, 'payload %d bytes, expected %d' % (len(reply) - 1, payload_bytes))
        return True

    def _reject_reply(self, name, why):
        self.status['rpc_reply_errors'] = self.status.get('rpc_reply_errors', 0) + 1
        self.status['rpc_reply_last_error'] = '%s: %s' % (name, why)
        self.logger.warning('Error %s: %s', name, why)
        return False

    def _rpc_eye_animation_reply(self, reply):
        self._reply_is_valid('RPC_REPLY_SET_EYE_ANIMATION', reply, self.RPC_REPLY_SET_EYE_ANIMATION)

    def _rpc_eye_frame_reply(self, reply):
        if not self._reply_is_valid('RPC_REPLY_SET_EYE_PER_EYE', reply, self.RPC_REPLY_SET_EYE_PER_EYE):
            return
        if len(reply) == 2:
            # A per-eye reply: firmware without frame mode read the request as per-eye.
            self.status['eye_frame']['supported'] = False
            self.status['eye_frame']['last_status'] = reply[1]
            return
        if len(reply) - 1 != self.EYE_FRAME_REPLY_PAYLOAD_BYTES:
            self._reject_reply('RPC_REPLY_SET_EYE_PER_EYE', 'payload %d bytes, expected %d' % (
                len(reply) - 1, self.EYE_FRAME_REPLY_PAYLOAD_BYTES))
            return
        status, applied = struct.unpack('<BH', bytes(reply[1:4]))
        self.status['eye_frame']['last_status'] = status
        self.status['eye_frame']['applied_sequence'] = applied
        if status != 0:
            self.logger.warning('PIMU refused an eye frame: %s (status %d)' % (
                self.EYE_FRAME_STATUS_NAMES.get(status, 'unknown'), status))

    def _rpc_imu_config_reply(self, reply):
        self._reply_is_valid('RPC_SET_IMU_CONFIG_REPLY', reply, self.RPC_SET_IMU_CONFIG_REPLY)

    def _rpc_config_reply(self, reply):
        self._reply_is_valid('RPC_REPLY_POWER_PERIPH_CONFIG', reply, self.RPC_REPLY_POWER_PERIPH_CONFIG)

    def _rpc_board_info_reply(self, reply):
        if not self._reply_is_valid('RPC_REPLY_POWER_PERIPH_BOARD_INFO', reply,
                                    self.RPC_REPLY_POWER_PERIPH_BOARD_INFO):
            return
        try:
            self._unpack_board_info(reply[1:])
        except (struct.error, IndexError, UnicodeDecodeError) as exc:
            self._reject_reply('RPC_REPLY_POWER_PERIPH_BOARD_INFO', 'decode failed: %s' % exc)

    def _rpc_trigger_reply(self, reply):
        if self._reply_is_valid('RPC_REPLY_POWER_PERIPH_TRIGGER', reply,
                                self.RPC_REPLY_POWER_PERIPH_TRIGGER, self.TRIGGER_PAYLOAD_BYTES):
            unpack_uint32_t(reply[1:])  # firmware echoes the trigger mask; value unused

    def _rpc_status_reply(self, reply):
        if not self._reply_is_valid('RPC_REPLY_POWER_PERIPH_STATUS', reply,
                                    self.RPC_REPLY_POWER_PERIPH_STATUS, self.STATUS_PAYLOAD_BYTES):
            return
        # Decode into temporaries and commit only a complete sample, so a short
        # or garbled payload cannot leave a mix of old and new fields behind.
        decoded = dict(self.status)
        decoded['imu'] = dict(self.imu.status)
        try:
            self._unpack_status(reply[1:], unpack_to=decoded)
        except (struct.error, IndexError, ValueError) as exc:
            self._reject_reply('RPC_REPLY_POWER_PERIPH_STATUS', 'decode failed: %s' % exc)
            return
        self.imu.status.update(decoded.pop('imu'))
        self.status.update(decoded)

    def _rpc_motor_sync_reply(self, reply):
        if self._reply_is_valid('RPC_REPLY_MOTOR_SYNC', reply,
                                self.RPC_REPLY_MOTOR_SYNC, self.MOTOR_SYNC_PAYLOAD_BYTES):
            self._unpack_motor_sync_reply(reply[1:])

    def _unpack_motor_sync_reply(self, s):
        # take in an array of bytes
        # this needs to exactly match the C struct format
        sidx = 0
        self.status['motor_sync_cnt'] = unpack_uint16_t(s[sidx:])
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


class PowerPeriph_Protocol_P14(PowerPeriph_Protocol_P12):
    """Advertise p14; its additive decoder lives in the shared aux mixin."""

    # sizeof(Pimu_Status) in hello_pimu2 Common.h for protocol p14: 68 byte IMU
    # block plus 76 bytes of power fields, packed. Replies of any other length
    # are rejected before decoding.
    STATUS_PAYLOAD_BYTES = 144

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
                                     'p12': (PowerPeriph_Protocol_P12,),
                                     'p13': (PowerPeriph_Protocol_P12,),
                                     'p14': (PowerPeriph_Protocol_P14,),}


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

    
    def is_runstopped(self):
        return self.status['runstop_event']

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

class PowerPeriphRpcRejectedStatus(TypedDict):
    count: int
    last_request_id: Optional[int]
    last_reason: Optional[int]
    last_length: Optional[int]

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
    eoa_output_tripped: bool
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
    rpc_reply_errors: int
    rpc_reply_last_error: Any
    rpc_rejected: PowerPeriphRpcRejectedStatus
    easter_egg_mode: int
    eye_frame: dict
