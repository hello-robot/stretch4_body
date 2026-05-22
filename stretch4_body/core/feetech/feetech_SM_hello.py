import threading
from typing import TypedDict

from stretch4_body.core.feetech.feetech_SM_servo import *
from stretch4_body.core.device import Device
from stretch4_body.core.feetech.protocol_packet_handler import *
from stretch4_body.core.hello_utils import *
import termios
import numpy
import math
import time
class FeetechCommErrorStats(Device):
    def __init__(self, name, logger):
        Device.__init__(self, name='fee_comm_errors')
        self.name = name
        self.status: "FeetechCommErrorStatsStatus" = {'n_rx': 0, 'n_tx': 0, 'n_gsr': 0, 'error_rate_avg_hz': 0}
        self.rate_log = None
        self.n_log = 10
        self.log_idx = 0
        self.ts_error_last = time.time()
        self.ts_warn_last = time.time()
        self.logger = logger

    def add_error(self, rx=True, gsr=False):
        t = time.time()
        if type(self.rate_log) == type(None):  # First error
            self.rate_log = numpy.array([0.0] * self.n_log)
        self.rate_log[self.log_idx] = 1 / (t - self.ts_error_last)
        self.log_idx = (self.log_idx + 1) % self.n_log
        self.status['error_rate_avg_hz'] = numpy.average(self.rate_log)
        if rx:
            self.status['n_rx'] += 1
        else:
            self.status['n_tx'] += 1
        if gsr:
            self.status['n_gsr'] += 1
        if t - self.ts_warn_last > self.params['warn_every_s']:
            self.ts_warn_last = t
            if self.status['error_rate_avg_hz'] > self.params['warn_above_rate']:
                self.logger.warning(
                    'Device %s generating %f errors per minute' % (self.name, (self.status['error_rate_avg_hz'] * 60)))
        if self.params['verbose']:
            self.pretty_print()

    def pretty_print(self):
        print('---- Feetech Comm Errors %s ----' % self.name)
        print('Rate (Hz): %f' % self.status['error_rate_avg_hz'])
        print('Rate (errors per minute): %f' % (self.status['error_rate_avg_hz'] * 60))
        print('Num TX: %f' % self.status['n_tx'])
        print('Num RX: %f' % self.status['n_rx'])
        print('Num Group Sync RX: %f' % self.status['n_gsr'])


class FeetechSMHello(Device):
    """
    Abstract the Feetech SM-Series to handle calibration, radians, etc
    """

    def __init__(self, name, chain=None, usb=None, params=None,is_direct=False):
        Device.__init__(self, name)
        if params is not None:
            self.params.update(params)
        try:
            self.is_direct=is_direct
            self.chain = chain
            self.hw_valid = False
            self.status: "FeetechSMHelloStatus" = {'timestamp_pc': 0, 'comm_errors': 0,'pos': 0, 'vel': 0, 'effort': 0, 'temp': 0,
                           'shutdown': 0, 'hardware_error': 0,
                           'input_voltage_error': 0, 'overtemp_error': 0, 'overcurrent_error': 0,
                           'motor_encoder_error': 0,
                           'electrical_shock_error': 0, 'overload_error': 0,
                           'stalled': 0, 'stall_overload': 0, 'pos_ticks': 0, 'vel_ticks': 0, 'current_mA': 0,
                           'watchdog_errors': 0,'pos_calibrated': False,'is_homing':False,
                           'is_moving':False,
                           'braking_distance':0,'torque_enabled':False,
                           'in_collision_stop':{'pos': False, 'neg': False},
                            'at_limit':{'pos': False, 'neg': False},'soft_motion_limits': (None, None),
                           }

            self.thread_rate_hz = 15.0
            self.usb = usb

            if self.usb is None:
                self.usb = self.params['usb_name']
            # Share bus resource amongst many XL430s

            #print("Creating servo", self.name, self.params['id'], self.usb, self.params['baud'])
            self.motor = FeetechSMServo(id=self.params['id'],
                                        usb=self.usb,
                                        port_handler=None if chain is None else chain.port_handler,
                                        pt_lock=None if chain is None else chain.pt_lock,
                                        baud=self.params['baud'],
                                        logger=self.logger)


            self.ts_over_eff_start = None

            #Setup Calibration and joint limits
            self.polarity = -1.0 if self.params['flip_encoder_polarity'] else 1.0
            self.home_pos_offset = 0
            lim_neg = self.params['range_deg'][0]
            lim_pos = self.params['range_deg'][1]
            # Calculate the range, in ticks
            self.range_nom_t = self.rad_to_ticks(deg_to_rad(lim_pos - lim_neg) * self.params['gr'])

            # Calculate the delta from the homing hardstop to zero, in ticks
            # The code is untested if homing_to_neg_limit is false, so commenting out for now.
            # if self.params['homing_to_neg_limit']:
            self.zero_nom_t = self.rad_to_ticks(deg_to_rad(lim_neg)) * self.params['gr'] * self.polarity * -1  # Gets the sign right, at least in neg direction
            # else:
            #     self.zero_nom_t = self.rad_to_ticks(deg_to_rad(lim_pos)) * self.params['gr']*self.polarity*-1

            self.status_mux_id = 0
            self.was_runstopped = False
            self.comm_errors = FeetechCommErrorStats(name, logger=self.logger)
            self.status['comm_errors']=self.comm_errors.status
            self.v_des = None  # Track the motion profile settings on servo
            self.a_des = None  # Track the motion profile settings on servo
            self.warn_error = False
            self.bubble_up_comm_exception = False

            self.in_vel_brake_zone = False
            self.in_vel_mode = False
            self.dist_to_min_max = None  # track dist to min,max limits
            self.vel_brake_zone_thresh = 0.2  # initial/minimum brake zone thresh value
            self._prev_set_vel_ts = None

            self.ts_collision_stop = {'pos': 0.0, 'neg': 0.0}
        except KeyError:
            self.motor = None

    def stop(self, close_port=True):
        Device.stop(self)
        self._waypoint_ts, self._waypoint_vel, self._waypoint_accel = None, None, None
        if self.hw_valid:
            if self.in_vel_mode:
                self.quick_stop()
            if self.params['disable_torque_on_runstop']:
                self.disable_torque()
            self.motor.stop(close_port)
            self.hw_valid = False
    # ###########  Device Methods #############

    def startup(self):
        """
        :return: True if startup successful
        """
        if self.motor is None:
            self.logger.error('Failed to start %s'%self.name.capitalize())
            return False
        self.logger.info('Starting %s...'%self.name.capitalize())
        Device.startup(self)
        try:
            self.motor.startup()
            if self.motor.do_ping(verbose=True):
                self.hw_valid = True
                self.status['torque_enabled'] = True
                self.motor.unlock_eeprom()

                self.motor.set_temp_limit(self.temp_to_ticks(self.params['eeprom_cfg']['temperature_limit']))
                self.motor.set_min_input_voltage(self.voltage_to_ticks(self.params['eeprom_cfg']['min_voltage_limit']))
                self.motor.set_max_input_voltage(self.voltage_to_ticks(self.params['eeprom_cfg']['max_voltage_limit']))
                self.motor.set_max_load_limit_pct(self.params['eeprom_cfg']['max_load_limit_pct'])  # 0-100 max pwm
                self.motor.set_pos_p_gain(self.params['eeprom_cfg']['pid'][0])
                self.motor.set_pos_i_gain(self.params['eeprom_cfg']['pid'][1])
                self.motor.set_pos_d_gain(self.params['eeprom_cfg']['pid'][2])
                self.motor.set_return_delay(self.params['eeprom_cfg']['return_delay_time'])
                self.motor.set_angular_res(self.params['eeprom_cfg']['angular_resolution'])
                self.motor.set_pos_offset(0)  # Dont use as doesn't work in multi-turn
                self.motor.set_phase(self.params['eeprom_cfg']['phase'])
                self.motor.set_max_pos_limit(self.params['eeprom_cfg']['max_pos_limit'])
                self.motor.set_min_pos_limit(self.params['eeprom_cfg']['min_pos_limit'])
                self.motor.set_overload_safe(self.params['eeprom_cfg']['overload_safe'])
                self.motor.set_overload_time_ms(self.params['eeprom_cfg']['overload_time_ms'])
                self.motor.set_overload_thresh(self.params['eeprom_cfg']['overload_thresh'])
                self.motor.set_overcurrent(self.params['eeprom_cfg'][
                                               'overcurrent'])  # _mA(10)#150*1.5) #500 max (3250mA), 975mA rated cont for SM80=150
                self.motor.set_overcurrent_time_ms(self.params['eeprom_cfg']['overcurrent_time_ms'])  # *10ms, 10=100ms

                flag = 0
                if self.params['eeprom_cfg']['enable_protection_overload']:
                    flag = flag | SMS_PROTECTION_OVERLOAD_FLAG
                if self.params['eeprom_cfg']['enable_protection_current']:
                    flag = flag | SMS_PROTECTION_CURRENT_FLAG
                if self.params['eeprom_cfg']['enable_protection_temp']:
                    flag = flag | SMS_PROTECTION_TEMP_FLAG
                if self.params['eeprom_cfg']['enable_protection_sensor']:
                    flag = flag | SMS_PROTECTION_SENSOR_FLAG
                if self.params['eeprom_cfg']['enable_protection_voltage']:
                    flag = flag | SMS_PROTECTION_VOLTAGE_FLAG
                self.motor.set_protection_switch(flag)

                self.motor.enable_pos()
                self.in_vel_mode=False
                self.set_motion_params()  # Initialize servo motion profile with default values


                self.status['pos_calibrated'] = self.motor.get_is_calibrated()
                if self.status['pos_calibrated']:
                    # Read calibration data stored in servo SRAM at the time of last calibration
                    self.home_pos_offset = self.motor.get_hello_robot_pos_offset()
                    #print('Read home_pos_offset of: %f'%self.home_pos_offset)

                self.update_joint_limits()

                self.motor.lock_eeprom()
                # Pull 3 times given mux / init the status dict
                self.pull_status()
                self.pull_status()
                self.pull_status()

                if not self.check_servo_errors():
                    self.hw_valid = False
                    return False

                # if self.params['use_pos_current_ctrl']:
                #     self.enable_pos_current_ctrl()
                return True
            else:
                self.logger.warning('FeetechSMHello Ping failed... %s' % self.name)
                return False
        except FeetechCommError:
            self.logger.warning('FeetechSMHello Ping failed... %s' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)
            return False


    # ################## Joint limits ##########################
    def get_at_limit(self, pos):
        joint_min, joint_max = self.get_soft_motion_limits()
        if joint_min is None or joint_max is None: 
            return {'pos': False, 'neg': False}
        return {'pos': pos >= joint_max - 0.02, 'neg': pos <= joint_min + 0.02}

    def get_soft_motion_limits(self):
        """
            Return the currently applied soft motion limits: [min, max]

            The soft motion limit restricts joint motion to be <= its physical limits.

            There are two types of limits:
            Hard: The physical limits
            User: Limits set by the user software

            The joint is limited to the most restrictive range of the Hard / User values.
            Specifying a value of None for a limit indicates that no constraint exists for that limit type.
            This allows a User limits to be disabled.
        """
        if self.is_homed():
            return self.soft_motion_limits['current']
        else:
            return [None, None]

    def set_soft_motion_limit_min(self, x):
        """
        x: value to set a joints limit to
        """
        self.soft_motion_limits['user'][0] = x
        self.soft_motion_limits['current'][0] = max(filter(lambda x: x is not None,
                                                            [self.soft_motion_limits['hard'][0],
                                                            self.soft_motion_limits['user'][0]]))

    def set_soft_motion_limit_max(self, x):
        """
        x: value to set a joints limit to
        """
        self.soft_motion_limits['user'][1] = x
        self.soft_motion_limits['current'][1] = min(filter(lambda x: x is not None,
                                                           [self.soft_motion_limits['hard'][1],
                                                            self.soft_motion_limits['user'][1]]))

    def update_joint_limits(self):
        if self.params['req_calibration'] and not self.status['pos_calibrated']:
            self.logger.warning('Feetech not calibrated: %s' % self.name)
            print('Feetech not calibrated:', self.name)
            return
        lim_neg = self.params['range_deg'][0]
        lim_pos = self.params['range_deg'][1]
        self.total_range = deg_to_rad(lim_pos-lim_neg)
        wr_max = deg_to_rad(lim_pos)
        wr_min = deg_to_rad(lim_neg)
        wrp_max=wr_max-abs(deg_to_rad(self.params['range_pad_deg'][1]))
        wrp_min = wr_min+ abs(deg_to_rad(self.params['range_pad_deg'][0]))
        self.range_t=[self.world_rad_to_ticks(wrp_min),self.world_rad_to_ticks(wrp_max)]

        self.soft_motion_limits = { 'user': [None, None], 'hard': [wr_min, wr_max],
                                   'current': [wrp_min, wrp_max]}
        self.status['soft_motion_limits'] = self.get_soft_motion_limits()
        self.status['at_limit'] = self.get_at_limit(self.status['pos'])

    # ################# Configure Motion ##################################

    def enable_pos(self):
        if not self.hw_valid:
            return
        try:
            if not self.status['torque_enabled']:
                self.enable_torque()
            self.motor.enable_pos()
            self.set_motion_params(force=True)
            self.in_vel_mode = False
        except (termios.error, FeetechCommError):
            self.logger.warning('FeetechSMHello communication error during enable_pos on %s: ' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)
            if self.bubble_up_comm_exception:
                raise FeetechCommError

    def enable_pwm(self):
        if not self.hw_valid:
            return
        try:
            if not self.status['torque_enabled']:
                self.enable_torque()
            self.motor.enable_pwm()
            self.in_vel_mode = False
        except (termios.error, FeetechCommError):
            self.logger.warning('FeetechSMHello communication error during enable_pwm on %s: ' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)
            if self.bubble_up_comm_exception:
                raise FeetechCommError

    def enable_velocity_ctrl(self):
        if not self.hw_valid:
            return
        try:
            self.in_vel_mode = True
            if not self.status['torque_enabled']:
                self.enable_torque()
            self.motor.enable_vel()
        except (termios.error, FeetechCommError):
            self.in_vel_mode = False
            self.logger.warning('FeetechSMHello communication error during enable_vel on %s: ' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)
            if self.bubble_up_comm_exception:
                raise FeetechCommError

    def enable_torque(self):
        if not self.hw_valid:
            return
        self.motor.enable_torque()
        self.status['torque_enabled'] = True

    def disable_torque(self):
        if not self.hw_valid:
            return
        self.motor.disable_torque()
        self.status['torque_enabled'] = False

    def set_motion_params(self, v_des=None, a_des=None, force=False):
        try:
            if not self.hw_valid:
                return
            v_des = abs(v_des) if v_des is not None else self.params['motion']['default']['vel']
            v_des = min(self.params['motion']['max']['vel'], v_des)
            if v_des != self.v_des or force:
                self.motor.set_profile_velocity(abs(self.world_rad_to_ticks_per_sec(v_des)))
                self.v_des = v_des

            a_des = abs(a_des) if a_des is not None else self.params['motion']['default']['accel']
            a_des = min(self.params['motion']['max']['accel'], a_des)
            if a_des != self.a_des or force:
                self.motor.set_profile_acceleration(abs(self.world_rad_to_ticks_per_sec_sec(a_des)))
                self.a_des = a_des
        except (termios.error, FeetechCommError):
            self.logger.warning('FeetechSMHello communication error during set_motion_params on: %s' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)
            if self.bubble_up_comm_exception:
                raise FeetechCommError

    # ############## Utility #####################################

    def do_ping(self, verbose=False):
        return self.motor.do_ping(verbose)

    def is_homed(self):
        return self.status['pos_calibrated']

    def check_servo_errors(self):
        if self.status['overload_error']:
            msg = 'WARNING: Servo %s in error state: overload_error. Reboot servo with stretch_robot_feetech_reboot' % self.name
            self.logger.warning(msg)
            self.warn_error = True
            return False

        if self.status['overtemp_error']:
            msg = 'WARNING: Servo %s in error state: overtemp_error. Reboot servo with stretch_robot_feetech_reboot' % self.name
            self.logger.warning(msg)
            self.warn_error = True
            return False

        if self.status['overcurrent_error']:
            msg = 'WARNING: Servo %s in error state: overcurrent_error. Reboot servo with stretch_robot_feetech_reboot' % self.name
            self.logger.warning(msg)
            self.warn_error = True
            return False
        return True

    def pretty_print(self):
        if not self.hw_valid:
            print('----- FeetechSMHello ------ ')
            print('Servo %s not on bus'%self.name)
            return
        print('----- FeetechSMHello ------ ')
        print('Name', self.name)
        print('Position (rad)', self.status['pos'])
        print('Position (deg)', rad_to_deg(self.status['pos']))
        print('Position (ticks)', self.status['pos_ticks'])
        print('Velocity (rad/s)', self.status['vel'])
        print('Velocity (ticks/s)', self.status['vel_ticks'])
        print('Effort (%)', self.status['effort'])
        print('Current (mA)', self.status['current_mA'])
        print('Temp', self.status['temp'])
        print('--------')
        print('Comm Errors', self.comm_errors.pretty_print())
        print('Hardware Error', self.status['hardware_error'])
        print('Hardware Error: Input Voltage Error: ', self.status['input_voltage_error'])
        print('Hardware Error: Overheating Error: ', self.status['overtemp_error'])
        print('Hardware Error: Motor Encoder Error: ', self.status['motor_encoder_error'])
        print('Hardware Error: Over Current Error: ', self.status['over_current_error'])
        print('Hardware Error: Overload Error: ', self.status['overload_error'])
        print('Watchdog Errors: ', self.status['watchdog_errors'])
        print('--------')
        print('Timestamp PC', self.status['timestamp_pc'])
        # print('Range (ticks)', self.range_t)
        # print('Range (rad) [', self.ticks_to_world_rad(self.range_t[0]), ' , ',
        #       self.ticks_to_world_rad(self.range_t[1]), ']')
        print('Stalled', self.status['stalled'])
        print('Stall Overload', self.status['stall_overload'])
        print('Is Calibrated', self.status['pos_calibrated'])
        print('Is homing: %d' % self.status['is_homing'])
        # self.motor.pretty_print()

    def bound_value(self, value, lower_bound, upper_bound):
        if value < lower_bound:
            return lower_bound
        elif value > upper_bound:
            return upper_bound
        else:
            return value

    def pull_status(self, data=None):
        if not self.hw_valid:
            return

        if not hasattr(self, '_last_pos_valid'):
            self._last_pos_valid = True


        pos_valid = True
        vel_valid = True
        i_mA_valid = True
        temp_valid = True
        err_valid = True

        # First pull new data from servo
        # Or bring in data from a synchronized read
        if data is None:
            try:
                x = self.motor.get_pos()
                if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                    x = self.motor.get_pos()
                pos_valid = self.motor.last_comm_success

                v = self.motor.get_vel()
                if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                    v = self.motor.get_vel()
                vel_valid = self.motor.last_comm_success

                if self.status_mux_id == 0:
                    i_mA = self.motor.get_current_mA()
                    if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                        i_mA = self.motor.get_current_mA()
                    i_mA_valid = self.motor.last_comm_success
                else:
                    i_mA = self.status['current_mA']

                if self.status_mux_id == 1:
                    temp = self.motor.get_temp()
                    if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                        temp = self.motor.get_temp()
                    temp_valid = self.motor.last_comm_success
                else:
                    temp = self.status['temp']

                if self.status_mux_id == 2:
                    err = self.motor.get_hardware_error()
                    if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                        err = self.motor.get_hardware_error()
                    err_valid = self.motor.last_comm_success
                else:
                    err = self.status['hardware_error']

                self.status_mux_id = (self.status_mux_id + 1) % 3
                self.check_servo_errors()
                if not pos_valid or not vel_valid or not i_mA_valid or not temp_valid or not err_valid:
                    self.logger.warning('FeetechSMHello communication error during pull_status on %s: ' % self.name)
                    self.comm_errors.add_error(rx=True, gsr=False)
                    self._last_pos_valid = False
                    return
                ts = time.time()
            except(termios.error, FeetechCommError, IndexError):
                self.logger.warning('FeetechSMHello communication error during pull_status  on %s: ' % self.name)
                # self.motor.port_handler.ser.reset_output_buffer()
                # self.motor.port_handler.ser.reset_input_buffer()
                self.comm_errors.add_error(rx=True, gsr=False)
                self._last_pos_valid = False
                if self.bubble_up_comm_exception:
                    raise FeetechCommError
                return
        else:
            x = data['x']
            pos_valid = x != None
            v = data['v']
            vel_valid = v != None
            i_mA = data['current_mA']
            i_mA_valid = i_mA != None
            temp = data['temp']
            temp_valid = temp != None
            ts = data['ts']
            err = data['err']
            err_valid = err != None

        # Now update status dictionary

        if pos_valid:
            if not self._last_pos_valid:
                self.logger.info(f"FeetechSMHello {self.name}: Communication recovered. Auto-reenabling torque.")
                if self.status.get('torque_enabled', False):
                    try:
                        self.motor.enable_torque()
                    except:
                        pass
            self._last_pos_valid = True
            
            self.status['pos_ticks'] = x
            self.status['pos'] = self.ticks_to_world_rad(float(x))
        if vel_valid:
            self.status['vel_ticks'] = v
            self.status['vel'] = self.ticks_to_world_rad_per_sec(float(v))
            self.status['is_moving']= abs(self.status['vel'])>self.params['motion']['vel_is_moving_thresh']
        if i_mA_valid:
            self.status['current_mA'] = i_mA
            self.status['effort'] = self.current_to_effort_pct(float(i_mA))
        if temp_valid:
            self.status['temp'] = float(temp)
        if err_valid:
            self.status['hardware_error'] = err

        self.status['timestamp_pc'] = ts

        self.status['hardware_error'] = err
        self.status['input_voltage_error'] = self.status['hardware_error'] & ERRBIT_VOLTAGE != 0
        self.status['motor_encoder_error'] = self.status['hardware_error'] & ERRBIT_ANGLE != 0
        self.status['overtemp_error'] = self.status['hardware_error'] & ERRBIT_OVERHEAT != 0
        self.status['over_current_error'] = self.status['hardware_error'] & ERRBIT_OVERELE != 0
        self.status['overload_error'] = self.status['hardware_error'] & ERRBIT_OVERLOAD != 0

        # Finally flag if stalled at high effort for too long
        self.status['stalled'] = abs(self.status['vel']) < self.params['stall_min_vel']
        over_eff = abs(self.status['effort']) > self.params['stall_max_effort']

        if self.status['stalled']:
            if not over_eff:
                self.ts_over_eff_start = None
            if over_eff and self.ts_over_eff_start is None:  # Mark the start of being stalled and over-effort
                self.ts_over_eff_start = time.time()
            if self.ts_over_eff_start is not None and time.time() - self.ts_over_eff_start > self.params[
                'stall_max_time']:
                self.status['stall_overload'] = True
            else:
                self.status['stall_overload'] = False
        else:
            self.ts_over_eff_start = None
            self.status['stall_overload'] = False

        self.status['braking_distance']=self.get_braking_distance()
        self.status['at_limit'] = self.get_at_limit(self.status['pos'])

    def wait_until_at_setpoint(self, timeout=15.0):
        """Polls for moving status to wait until at commanded position goal

        Returns
        -------
        bool
            True if success, False if timeout
        """
        ts = time.time()
        while time.time() - ts < timeout:
            if self.motor.is_moving() == False:
                return True
            time.sleep(0.1)
        return False

    def check_nan_value(self, x):
        try:
            return math.isnan(x)
        except TypeError:
            return False

    def quick_stop(self):
        if not self.hw_valid:
            return
        try:
            self.motor.disable_torque()
            self.motor.enable_torque()
        except (termios.error, FeetechCommError):
            self.logger.warning('FeetechSMHello communication error during quick_stop on %s: ' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)
            if self.bubble_up_comm_exception:
                raise FeetechCommError

    def set_pwm(self, x):
        if self.was_runstopped:
            return
        if not self.hw_valid:
            return
        try:
            self.motor.set_goal_pwm(x)
        except (termios.error, FeetechCommError):
            self.logger.warning('FeetechSMHello communication error during set_pwm on %s: ' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)
            if self.bubble_up_comm_exception:
                raise FeetechCommError

    # ############## Safety and Sentry #####################################

    def step_collision_avoidance(self, in_collision):
        """
        Disable the ability to command motion in the positive or negative direction
        If the joint is in motion in that direction, force it to stop
        Parameters
        ----------
        in_collision: {'pos': False, 'neg': False},etc
        """
        if self.is_calibration_required():
            return
        for dir in ['pos', 'neg']:
            if in_collision[dir] and not self.status['in_collision_stop'][dir] and not self.was_runstopped:
                # Stop current motion
                self.quick_stop()
                self.status['in_collision_stop'][dir] = True
                self.ts_collision_stop[dir] = time.time()

            # Reset if out of collision (at least 1s after collision)
            if self.status['in_collision_stop'][dir] and not in_collision[dir] and time.time() - self.ts_collision_stop[dir] > 1.0:
                self.status['in_collision_stop'][dir] = False

    def step_sentry(self, robot_status):
        if getattr(self, 'sentry_paused', False):
            return
        is_runstopped = robot_status['power_periph']['runstop_event']
        if (self.hw_valid and  self.params['enable_runstop']):
            if is_runstopped and not self.was_runstopped:  #Runstop enabled since last call
                    if self.params['disable_torque_on_runstop']:
                        self.disable_torque() #Stops and makes backdriveable
                    else:
                        self.quick_stop()  # Stops but holds position
            if not is_runstopped and  self.was_runstopped:  # Runstop enabled since last call
                if self.params['enable_torque_after_runstop']:
                    self.enable_torque()
        self.was_runstopped=is_runstopped

    def pause_sentry(self):
        self.sentry_paused = True

    def unpause_sentry(self):
        self.sentry_paused = False

        if not self.is_calibration_required():
            delta1, delta2 = self.get_dist_to_limits()  # calculate dist to min,max limits
            self.dist_to_min_max = [delta1, delta2]

            if self.dist_to_min_max[0] < self.vel_brake_zone_thresh or self.dist_to_min_max[1] < self.vel_brake_zone_thresh:
                self.logger.debug(f"In Vel-Braking Zone.")
                self.in_vel_brake_zone = True
            else:
                self.in_vel_brake_zone = False
            self._update_safety_vel_brake_zone()

        if self.in_vel_mode:
            #Watchdog on velocity control
            # disable if a set_velocity() command is not passed above 1s
            if self._prev_set_vel_ts:
                if time.time() - self._prev_set_vel_ts >= 1:
                    self.disable_torque()
                    self._prev_set_vel_ts = None
                    self.logger.warning(f'Watchdog error during Velocity control for {self.name}. Disabling torque')
                    self.status['watchdog_errors'] = self.status['watchdog_errors'] + 1

    # ############## Position Control  #####################################

    def move_to(self, x_des, v_des=None, a_des=None):
        if True in [self.check_nan_value(d) for d in (x_des, v_des, a_des)]:
            self.logger.warning('Received NaN value. dropping the command.')
            return
        if self.was_runstopped:
            return
        nretry = 2
        if not self.hw_valid:
            return
        if self.params['req_calibration'] and not self.status['pos_calibrated']:
            self.logger.warning('Feetech not calibrated: %s' % self.name)
            print('Feetech not calibrated:', self.name)
            return

        if self.status['in_collision_stop']['pos'] and self.status['pos'] < x_des:
            self.logger.warning(
                'move_to in collision. Motion disabled in direction %s for %s. Not executing move_to' % ('pos', self.name),
                extra={'throttle_s': 1.0}
            )
            return

        if self.status['in_collision_stop']['neg'] and self.status['pos'] > x_des:
            self.logger.warning(
                'move_to in collision. Motion disabled in direction %s for %s. Not executing move_to' % ('neg', self.name),
                extra={'throttle_s': 1.0}
            )
            return

        if self.in_vel_mode or not self.status['torque_enabled']:
            self.enable_pos()
        # print('Motion Params',v_des,a_des)
        self.set_motion_params(v_des, a_des)
        old_x_des = x_des
        x_des = min(max(self.get_soft_motion_limits()[0], x_des), self.get_soft_motion_limits()[1])
        if x_des != old_x_des:
            self.logger.debug(
                'Clipping move_to({0}) with soft limits {1}'.format(old_x_des, self.soft_motion_limits['current']))

        t_des = self.world_rad_to_ticks(x_des)
        # t_des = max(self.range_t[0], min(self.range_t[1], t_des))
        success = False

        for i in range(nretry):
            try:
                self.motor.go_to_pos(t_des)
                success = True
                break
            except (termios.error, FeetechCommError, IndexError):
                self.logger.warning('FeetechSMHello communication error during move_to on %s: ' % self.name)
                self.comm_errors.add_error(rx=False, gsr=False)
                if self.bubble_up_comm_exception:
                    raise FeetechCommError

    def move_by(self, x_des, v_des=None, a_des=None):
        if True in [self.check_nan_value(d) for d in (x_des, v_des, a_des)]:
            self.logger.warning('Received NaN value. dropping the command.')
            return
        if self.was_runstopped:
            return
        if not self.hw_valid:
            return
        if self.in_vel_mode or not self.status['torque_enabled']:
            self.enable_pos()
        try:
            if abs(x_des) > 0.00002:  # Avoid drift
                if not self.is_direct:
                    x = self.status['pos_ticks']
                else:
                    x = self.motor.get_pos()
                if not self.motor.last_comm_success and self.params['retry_on_comm_failure']:
                    if not self.is_direct:
                        x = self.status['pos_ticks']
                    else:
                        x = self.motor.get_pos()

                if self.motor.last_comm_success:
                    cx = self.ticks_to_world_rad(x)

                    if self.status['in_collision_stop']['pos'] and self.status['pos'] < cx + x_des:
                        self.logger.warning(
                            'move_by in collision. Motion disabled in direction %s for %s. Not executing move_by' % (
                                'pos', self.name),
                            extra={'throttle_s': 1.0}
                        )
                        return

                    if self.status['in_collision_stop']['neg'] and self.status['pos'] > cx + x_des:
                        self.logger.warning(
                            'move_by in collision. Motion disabled in direction %s for %s. Not executing move_by' % (
                                'neg', self.name),
                            extra={'throttle_s': 1.0}
                        )
                        return
                    self.move_to(cx + x_des, v_des, a_des)
                else:
                    self.logger.error('Move_By comm failure on %s' % self.name)
        except (termios.error, FeetechCommError):
            self.logger.warning('FeetechSMHello communication error during move_by on %s: ' % self.name)
            self.comm_errors.add_error(rx=False, gsr=False)
            if self.bubble_up_comm_exception:
                raise FeetechCommError

    # #############Safe Velocity Control ########################

    def set_velocity(self, v_des, a_des=None):
        if True in [self.check_nan_value(d) for d in (v_des, a_des)]:
            self.logger.warning('Received NaN value. dropping the command.')
            return
        if self.was_runstopped:
            return

        v = min(self.params['motion']['max']['vel'], abs(v_des))

        if self.status['in_collision_stop']['pos'] and v_des > 0:
            self._prev_set_vel_ts = time.time()
            self.logger.warning(
                'set_velocity in collision . Motion disabled in direction %s for %s. Not executing set_velocity' % (
                    'pos', self.name),
                extra={'throttle_s': 1.0}
            )
            return

        if self.status['in_collision_stop']['neg'] and v_des < 0:
            self._prev_set_vel_ts = time.time()
            self.logger.warning(
                'set_velocity in collision. Motion disabled in direction %s for %s. Not executing set_velocity' % (
                    'neg', self.name),
                extra={'throttle_s': 1.0}
            )
            return

        v_des = -1 * v if v_des < 0 else v
        nretry = 2
        if not self.hw_valid:
            return
        if self.params['req_calibration'] and not self.status['pos_calibrated']:
            self.logger.warning('Feetech not calibrated: %s' % self.name)
            print('Feetech not calibrated:', self.name)
            return
        success = False

        if not self.in_vel_mode or not self.status['torque_enabled']:
            self.enable_velocity_ctrl()

        for i in range(nretry):
            try:
                if self.params['set_safe_velocity'] and self.in_vel_brake_zone:  # in_vel_brake_zone only when sentry is active
                    self._step_vel_braking(v_des, a_des)
                else:
                    self.set_motion_params(a_des=a_des)
                    t_des = self.world_rad_to_ticks_per_sec(v_des)
                    self.motor.set_vel(t_des)
                    self._prev_set_vel_ts = time.time()
                success = True
                break
            except(termios.error, FeetechCommError, IndexError):
                self.logger.warning('FeetechSMHello communication error during set_velocity on %s: ' % self.name)
                self.comm_errors.add_error(rx=True, gsr=False)
                if self.bubble_up_comm_exception:
                    raise FeetechCommError

    def _step_vel_braking(self, v_des, a_des):
        """
        In velocity mode while using set_velocity() command, when the joint is in a braking zone,
        the input velocities are tapered till the joint limits  to zero and smoothly braked at the limits to
        avoid hitting the hardstops.
        """
        if self._prev_set_vel_ts is None:
            self._prev_set_vel_ts = time.time()

        if self.status[
            'timestamp_pc'] > self._prev_set_vel_ts:  # Braking control syncs with the pull status's freaquency for accurate motion control
            # Honor joint limits in velocity mode
            lim_lower = min(self.ticks_to_world_rad(self.range_t[0]),
                            self.ticks_to_world_rad(self.range_t[1]))
            lim_upper = max(self.ticks_to_world_rad(self.range_t[0]),
                            self.ticks_to_world_rad(self.range_t[1]))

            v_curr = self.status['vel']
            x_curr = self.status['pos']

            to_min = abs(x_curr - lim_lower)
            to_max = abs(x_curr - lim_upper)

            c1 = to_min < to_max and v_des > 0  # if v_des -ve
            c2 = to_min > to_max and v_des < 0  # if v_des +ve
            opp_vel = c1 or c2

            t_brake = abs(v_curr / self.params['motion']['max']['accel'])  # How long to brake from current speed (s)
            d_brake = t_brake * abs(v_curr) / 2  # How far it will go before breaking (pos/neg)
            d_brake = d_brake + deg_to_rad(5.0)  # Pad out by 5 degrees to give a bit of safety margin
            v = 0
            if opp_vel:
                self.set_motion_params(a_des=a_des)
                v = v_des  # allow input velocity if direction is opposite to nearest limit
            elif (v_des > 0 and x_curr + d_brake >= lim_upper) or (v_des <= 0 and x_curr - d_brake <= lim_lower) or min(
                    to_max, to_max) < 0.1:
                self.set_motion_params(a_des=self.params['motion']['max']['accel'])
                v = 0  # apply brakes if the braking distance is >= limits
            else:
                self.set_motion_params(a_des=self.params['motion']['max']['accel'])
                taper = min(to_max, to_min) / self.vel_brake_zone_thresh  # normalized (0~1) distance to limits
                v = v_des * taper  # apply tapered velocity inside braking zone
            # self.logger.warning(f"Applied safety brakes near limits. reduced set_vel={v} rad/s")
            self.motor.set_vel(self.world_rad_to_ticks_per_sec(v))
            self._prev_set_vel_ts = time.time()

    def _update_safety_vel_brake_zone(self):
        """
        dynamically update the braking zone thresh based on it is propotional nature to the
        current velocity and the inverse of distance left to reach the nearest hardstop.
        """
        delta1, delta2 = self.dist_to_min_max
        distance_to_limit = min(delta1, delta2)
        brake_zone_factor = self.params['motion'][
            'vel_brakezone_factor']  # Propotional value, for now value 1 seems to work fine with all fee joints
        if distance_to_limit != 0:
            brake_zone_thresh = brake_zone_factor * abs(self.status['vel']) / distance_to_limit
            brake_zone_thresh = self.bound_value(brake_zone_thresh, 0, self.total_range / 2)
            brake_zone_thresh = brake_zone_thresh + 0.3  # 0.3 rad is minimum brake zone thresh
            self._set_vel_brake_thresh(brake_zone_thresh)

    def _set_vel_brake_thresh(self, thresh):
        self.vel_brake_zone_thresh = thresh

    def get_dist_to_limits(self, threshold=0.2):
        current_position = self.status['pos']
        min_position = self.get_soft_motion_limits()[0]
        max_position = self.get_soft_motion_limits()[1]
        delta1 = abs(current_position - min_position)
        delta2 = abs(current_position - max_position)

        if delta2 < threshold or delta1 < threshold:
            return delta1, delta2
        else:
            return delta1, delta2

    def get_braking_distance(self, acc=None):
        """Compute signed distance to brake the joint from the current velocity"""
        v_curr = self.status['vel']
        if acc is None:
            acc = self.params['motion']['max']['accel']
        t_brake = abs(v_curr / acc)  # How long to brake from current speed (s)
        d_brake = t_brake * v_curr / 2  # How far it will go before breaking (pos/neg)
        return d_brake


    # ############## Homing  ############################

    def is_calibration_required(self):
        return self.params['req_calibration'] and not self.status['pos_calibrated']

    """ 
    Servo calibration works by:

    Homing
    ==============
    * Move with a fixed PWM to a hardstop
    * Store the current position (ticks) in the SRAM homing offset such that self.motor.status['pos']==0 at that hardstop

    Calibration to SI / joint range:
    ===============
    Once homed, the servo can move to positions within self.range_t (ticks).

    For SE4 Feetech wrist we simplify homing to multi-turn mode only, where:
    *  self.params['range_nom_t'] is the (signed) full range of motion in ticks
    *  self.params['zero_nom_t'] is the (signed) offset from the first hard stop to jont zero (in ticks)
    These values are known from CAD. Then,
    1) Move to first hardstop via PWM and mark position self.home_pos_offset
    2) The second hardstop, Position B = self.home_pos_offset + self.params['range_nom_t'], 
    3) The raw encoder position of the joint zero Z = self.params['zero_nom_t']+self.home_pos_offset

    So then, given a servo reported position X (ticks), the joint position in ticks is 
    Q = X-Z, where Q is limited between A and B

    Ex 1, PWM of -300 moves --> A=1420
    self.params['range_nom_t']=-6250, so B=-4830
    self.params['zero_nom_t']=-3042, so Z=-1622
    ,then when X=-=1622, Q=0

    """


    def home(self, cancel_homing_event:threading.Event, end_pos:float|None=None, delay_at_stop:float=0.0):
        self.bubble_up_comm_exception = True
        self.status['is_homing']=False
        try:
            if not self.hw_valid:
                self.logger.warning('Not able to home %s. Hardware not present' % self.name)
                return False
            if not self.params['req_calibration']:
                self.logger.info('Homing not required for: ' + self.name)
                return False


            self.pull_status()

            if not self.check_servo_errors():
                self.logger.warning('Hardware error, unable to home. Exiting')
                return False

            self.status['is_homing'] = True
            ########## First Hardstop ############3
            # This switches the encoder from multi-turn to single-turn
            self.enable_pwm()

            self.logger.info(f'Moving to first hardstop ({self.name})...')
            self.set_pwm(self.params['homing_pwm'])
            ts = time.time()
            time.sleep(1.0)
            timeout = False
            # Note, is_moving doesn't work in PWM mode. Hard coded vel for now.
            while abs(self.motor.get_vel()) > 100 and not timeout and not cancel_homing_event.is_set():
                timeout = time.time() - ts > 15.0
                time.sleep(0.1) 
                # print('Pos (ticks)', self.motor.get_pos())
            time.sleep(delay_at_stop)

            self.set_pwm(0.0)

            if cancel_homing_event.is_set():
                self.logger.error('Homing cancelled for: ' + self.name)
                self.status['is_homing'] = False
                return False
            if timeout:
                self.logger.error('Timed out moving to first hardstop. Exiting.')
                self.status['is_homing'] = False
                return False
            if not self.check_servo_errors():
                self.logger.error('Hardware error, unable to home. Exiting')
                self.status['is_homing'] = False
                return False

            self.home_pos_offset = self.motor.get_pos()
            
            bias_t = self.params.get('homing_offset_bias_t', 0)
            if bias_t != 0:
                self.logger.info(f"Applying homing offset bias of {bias_t} ticks")
                self.home_pos_offset += bias_t
                
            self.logger.info('First hardstop contact at position (ticks): %d' % self.home_pos_offset)
            self.motor.set_hello_robot_pos_offset(self.home_pos_offset)
            self.motor.set_is_calibrated(1)
            self.status['pos_calibrated'] = True
            self.update_joint_limits()

            # This switches the encoder from single back to multi-turn
            # It locks in the encoder offset at this point
            # A subsequent call to enable_pwm will make  this offset invalid
            self.enable_pos()
            # print('MODE',self.motor.get_operating_mode())
            # print('RANGE', self.range_t)
            # print('Current position (ticks):', self.motor.get_pos())
            if end_pos is not None:
                self.logger.info(f'Moving to calibrated pos: (ticks) {self.world_rad_to_ticks(end_pos)}')
                self.move_to(end_pos)
                time.sleep(2.0)
                self.wait_until_at_setpoint(timeout=6.0)
            self.status['is_homing'] = False
            self.bubble_up_comm_exception = False
            self.logger.info(f"Done homing {self.name}")
            return True
        except Exception as e:
            self.logger.error(f'Communication error, unable to home. Exiting. {e=}')
            return False

    # ############### Conversions ###########################
    def ticks_to_rad(self, t):  # target position * angular resolution * 360/4096
        deg_per_tick = (
                    360.0 * int(self.params['eeprom_cfg']['angular_resolution']) / 4096.0)  # deg per step from encoder
        return deg_to_rad(deg_per_tick * t)

    def rad_to_ticks(self, r):
        deg_per_tick = (
                    360.0 * int(self.params['eeprom_cfg']['angular_resolution']) / 4096.0)  # deg per step from encoder
        return int(rad_to_deg(r) / deg_per_tick)

    def ticks_to_rad_per_sec(self, t):
        deg_per_tick = (360.0 / 4096.0)
        return deg_to_rad(deg_per_tick * t)  # 50

    def rad_per_sec_to_ticks(self, r):
        deg_per_tick = (360.0 / 4096.0)
        return rad_to_deg(r) / (deg_per_tick)  # *50)

    def ticks_to_rad_per_sec_sec(self, t):
        deg_per_tick = (360.0 / 4096.0)
        return deg_to_rad(deg_per_tick * 100 * t)

    def rad_per_sec_sec_to_ticks(self, r):
        deg_per_tick = (360.0 / 4096.0)
        return rad_to_deg(r) / (deg_per_tick * 100)

    def ticks_to_temp(self, t):
        return t

    def temp_to_ticks(self, t):
        return int(max(0, min(100, t)))

    def ticks_to_voltage(self, v):
        return v / 10.0

    def voltage_to_ticks(self, v):
        return max(0, int(v * 10))

    def ticks_to_current(self, t):
        return t * 6.5

    def current_to_ticks(self, i):
        return int(i / 6.5)

    def ticks_to_pct_load(self, t):
        # 1000 ticks = 100 pct
        return max(-100, min(100, t / 10))

    def pct_load_to_ticks(self, p):
        return int(p * 10)

    def current_to_effort_pct(self, i_mA):
        return 100 * max(0.0, min(1.0, i_mA / self.params['eeprom_cfg']['overcurrent']))

    def effort_pct_to_current(self, e_pct):
        return min(1.0, e_pct / 100.0) * self.params['eeprom_cfg']['overcurrent']

    def world_rad_to_ticks(self, r):
        #Convert from joint frame to servo frame
        rad_servo = r * self.params['gr'] * self.polarity
        t = self.rad_to_ticks(rad_servo)
        #Get the zero offset
        Z = self.zero_nom_t + self.home_pos_offset
        #print('world_rad_to_ticks',r,rad_servo,t,self.zero_nom_t,self.home_pos_offset,Z,t+Z)
        return t + Z

    def ticks_to_world_rad(self, t):
        Z = self.zero_nom_t + self.home_pos_offset
        t = t - Z
        rad_servo = self.ticks_to_rad(t)
        return (self.polarity * rad_servo / self.params['gr'])

    def ticks_to_world_rad_per_sec_sec(self, t):
        rps_servo = self.ticks_to_rad_per_sec_sec(t)
        return self.polarity * rps_servo / self.params['gr']

    def ticks_to_world_rad_per_sec(self, t):
        rps_servo = self.ticks_to_rad_per_sec(t)
        return self.polarity * rps_servo / self.params['gr']

    def world_rad_to_ticks_per_sec(self, r):
        rad_per_sec_servo = r * self.params['gr'] * self.polarity
        t = self.rad_per_sec_to_ticks(rad_per_sec_servo)
        return t

    def world_rad_to_ticks_per_sec_sec(self, r):
        rad_per_sec_sec_servo = r * self.params['gr'] * self.polarity
        t = self.rad_per_sec_sec_to_ticks(rad_per_sec_sec_servo)
        return t


if __name__ == "__main__":

    s = FeetechSMHello(name='ma_gripper', usb='/dev/hello-feetech-marionette', params=MA_wrist_gripper)
    s.startup()
    s.do_ping()
    # print("Test rad_to_ticks",s.rad_to_ticks(s.ticks_to_rad(5.0)),5.0)
    # print("Test ticks_to_rad_per_sec", s.rad_per_sec_to_ticks(s.ticks_to_rad_per_sec(5.0)), 5.0)
    # print("Test ticks_to_rad_per_sec_sec", s.rad_per_sec_sec_to_ticks(s.ticks_to_rad_per_sec_sec(5.0)), 5.0)

    if 1:
        s.disable_torque()
        for i in range(1000):
            s.pull_status()
            xd = max(0, min(180, 180 * rad_to_deg(s.status['pos']) / 90.0))
            print('Ticks', s.status['pos_ticks'], 'deg', rad_to_deg(s.status['pos']), 'Des', xd)
            time.sleep(0.1)

    if 0:
        ts = time.time()
        for i in range(100):
            s.pull_status()
            print('Pos', s.status['pos_ticks'])
        dt = time.time() - ts
        print('Pull status rate', 100 / dt)

    if 0:
        s.move_to(0)
        time.sleep(1.0)
        s.pull_status()
        x_last = rad_to_deg(s.status['pos'])
        print('XLast', x_last)
        ts = time.time()
        nits = 0

        tn = 30
        s.set_velocity(v_des=deg_to_rad(360.0 / 2), a_des=254)
        while nits < tn:
            s.pull_status()
            # print(rad_to_deg(s.status['pos']))
            if x_last > 0 and rad_to_deg(s.status['pos']) < 0:
                nits = nits + 1
                print('Rollover', nits)
                print(rad_to_deg(s.status['vel']))
            x_last = rad_to_deg(s.status['pos'])
            # time.sleep(.01)
        dt = time.time() - ts
        print('RPS', tn / dt)
        s.set_velocity(0)
        s.stop()

    if 0:
        v_des = 5000
        a_des = 200  # deg_to_rad(360)
        s.pretty_print()
        for j in range(20):
            s.move_to(deg_to_rad(0.0), v_des, a_des)
            # for i in range(10):
            #     s.pull_status()
            #     s.motor.pretty_print()
            #     time.sleep(.2)
            input('A: enter to continue')
            s.move_to(deg_to_rad(180.0), v_des, a_des)
            # for i in range(10):
            #     s.pull_status()
            #     s.motor.pretty_print()
            #     time.sleep(.2)
            input('B: enter to continue')

    s.stop()

class FeetechCommErrorStatsStatus(TypedDict):
    n_rx: int
    n_tx: int
    n_gsr: int
    error_rate_avg_hz: float

class FeetechSMHelloInCollisionStopStatus(TypedDict):
    pos: bool
    neg: bool


class FeetechSMHelloStatus(TypedDict):
    timestamp_pc: float
    comm_errors: int
    pos: float
    vel: float
    effort: float
    temp: float
    shutdown: int
    hardware_error: int
    input_voltage_error: int
    overtemp_error: int
    overcurrent_error: int
    motor_encoder_error: int
    electrical_shock_error: int
    overload_error: int
    stalled: int
    stall_overload: int
    pos_ticks: int
    vel_ticks: int
    current_mA: int
    watchdog_errors: int
    pos_calibrated: bool
    is_homing: bool
    is_moving: bool
    torque_enabled: bool
    braking_distance: float
    in_collision_stop: FeetechSMHelloInCollisionStopStatus