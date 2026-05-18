from typing import Any, TypedDict

from stretch4_body.core.transport.transport import *
from stretch4_body.core.device import Device
from stretch4_body.core.hello_utils import *
import textwrap
import threading
import sys
import time
import array as arr
import math

# ######################## STEPPER #################################
class StepperDefn():
    RPC_SET_COMMAND = 1
    RPC_REPLY_COMMAND = 2
    RPC_GET_STATUS = 3
    RPC_REPLY_STATUS = 4
    RPC_SET_GAINS = 5
    RPC_REPLY_GAINS = 6
    RPC_LOAD_TEST_PUSH = 7
    RPC_REPLY_LOAD_TEST_PUSH = 8
    RPC_SET_TRIGGER = 9
    RPC_REPLY_SET_TRIGGER = 10
    RPC_SET_ENC_CALIB = 11
    RPC_REPLY_ENC_CALIB = 12
    RPC_READ_GAINS_FROM_FLASH = 13
    RPC_REPLY_READ_GAINS_FROM_FLASH = 14
    RPC_SET_MENU_ON = 15
    RPC_REPLY_MENU_ON = 16
    RPC_GET_STEPPER_BOARD_INFO = 17
    RPC_REPLY_STEPPER_BOARD_INFO = 18
    RPC_SET_MOTION_LIMITS = 19
    RPC_REPLY_MOTION_LIMITS = 20
    RPC_SET_NEXT_TRAJECTORY_SEG = 21 # DEPRECATED
    RPC_REPLY_SET_NEXT_TRAJECTORY_SEG = 22 # DEPRECATED
    RPC_START_NEW_TRAJECTORY = 23 # DEPRECATED
    RPC_REPLY_START_NEW_TRAJECTORY = 24 # DEPRECATED
    RPC_RESET_TRAJECTORY = 25 # DEPRECATED
    RPC_REPLY_RESET_TRAJECTORY = 26 # DEPRECATED
    RPC_READ_TRACE = 27
    RPC_REPLY_READ_TRACE = 28
    RPC_GET_STATUS_AUX = 29
    RPC_REPLY_STATUS_AUX = 30
    RPC_LOAD_TEST_PULL = 31
    RPC_REPLY_LOAD_TEST_PULL = 32

    RPC_SET_STEPPER_TYPE = 33
    RPC_REPLY_SET_STEPPER_TYPE = 34
    RPC_READ_STEPPER_TYPE_FROM_FLASH = 35
    RPC_REPLY_READ_STEPPER_TYPE_FROM_FLASH = 36

    MODE_SAFETY = 0
    MODE_FREEWHEEL = 1
    MODE_HOLD = 2
    MODE_POS_PID = 3
    MODE_VEL_PID = 4
    MODE_POS_TRAJ = 5
    MODE_VEL_TRAJ = 6
    MODE_CURRENT = 7
    MODE_POS_TRAJ_INCR = 8
    MODE_POS_TRAJ_WAYPOINT = 9 # DEPRECATED

    MODE_NAMES = {
        MODE_SAFETY: 'MODE_SAFETY',
        MODE_FREEWHEEL: 'MODE_FREEWHEEL',
        MODE_HOLD: 'MODE_HOLD',
        MODE_POS_PID: 'MODE_POS_PID',
        MODE_VEL_PID: 'MODE_VEL_PID',
        MODE_POS_TRAJ: 'MODE_POS_TRAJ',
        MODE_VEL_TRAJ: 'MODE_VEL_TRAJ',
        MODE_CURRENT: 'MODE_CURRENT',
        MODE_POS_TRAJ_INCR: 'MODE_POS_TRAJ_INCR',
        MODE_POS_TRAJ_WAYPOINT: 'MODE_POS_TRAJ_WAYPOINT',
    }

    DIAG_POS_CALIBRATED = 1  # Has a pos zero RPC been received since powerup
    DIAG_RUNSTOP_ON = 2  # Is controller in runstop mode
    DIAG_NEAR_POS_SETPOINT = 4  # Is pos controller within gains.pAs_d of setpoint
    DIAG_NEAR_VEL_SETPOINT = 8  # Is vel controller within gains.vAs_d of setpoint
    DIAG_IS_MOVING = 16  # Is measured velocity greater than gains.vAs_d
    DIAG_IS_DRV_FAULT = 32  # Is controller current saturated
    DIAG_IS_MG_ACCELERATING = 64  # Is controller motion generator acceleration non-zero
    DIAG_IS_MG_MOVING = 128  # Is controller motion generator velocity non-zero
    DIAG_CALIBRATION_RCVD = 256  # Is calibration table in flash
    DIAG_IN_GUARDED_EVENT = 512  # Guarded event occurred during motion
    DIAG_IN_SAFETY_EVENT = 1024  # Is it forced into safety mode
    DIAG_WAITING_ON_SYNC = 2048  # Command received but no sync yet
    DIAG_TRAJ_ACTIVE = 4096  # DEPRECATED: Whether a waypoint trajectory is actively executing
    DIAG_TRAJ_WAITING_ON_SYNC = 8192  # DEPRECATED: Currently waiting on a sync signal before starting trajectory
    DIAG_IN_SYNC_MODE = 16384  # Currently running in sync mode
    DIAG_IS_TRACE_ON = 32768  # Is trace recording

    CONFIG_SAFETY_HOLD = 1  # Hold position in safety mode? Otherwise freewheel
    CONFIG_ENABLE_RUNSTOP = 2  # Recognize runstop signal?
    CONFIG_ENABLE_SYNC_MODE = 4  # Commands are synchronized from digital trigger
    CONFIG_ENABLE_GUARDED_MODE = 8  # Stops on current threshold
    CONFIG_FLIP_ENCODER_POLARITY = 16
    CONFIG_FLIP_EFFORT_POLARITY = 32
    CONFIG_ENABLE_VEL_WATCHDOG = 64  # Timeout velocity commands

    TRIGGER_MARK_POS = 1
    TRIGGER_RESET_MOTION_GEN = 2
    TRIGGER_BOARD_RESET = 4
    TRIGGER_WRITE_GAINS_TO_FLASH = 8
    TRIGGER_RESET_POS_CALIBRATED = 16
    TRIGGER_POS_CALIBRATED = 32
    TRIGGER_MARK_POS_ON_CONTACT = 64
    TRIGGER_ENABLE_TRACE = 128
    TRIGGER_DISABLE_TRACE = 256
    TRIGGER_RESET_DRV_FAULT = 512
    TRIGGER_RESET_MARK_POS_ON_CONTACT=1024

    TRACE_TYPE_STATUS = 0
    TRACE_TYPE_DEBUG = 1
    TRACE_TYPE_PRINT = 2
    def __init__(self):
        pass
class StepperMotion(StepperDefn):
    def __init__(self):
        self.motion_limits = [0, 0]
        self.is_moving_history = [False] * 10
        self.ts_last_syncd_motion=0
    # ###########################################################################
    def enable_safety(self):
        self.set_command(mode=self.MODE_SAFETY)

    def enable_freewheel(self):
        self.set_command(mode=self.MODE_FREEWHEEL)

    def enable_hold(self):
        self.set_command(mode=self.MODE_HOLD)

    def enable_vel_pid(self):
        self.set_command(mode=self.MODE_VEL_PID, v_des=0)

    def enable_pos_pid(self):
        self.set_command(mode=self.MODE_POS_PID, x_des=self.status['pos'])

    def enable_vel_traj(self):
        self.set_command(mode=self.MODE_VEL_TRAJ, v_des=0)

    def enable_pos_traj(self):
        self.set_command(mode=self.MODE_POS_TRAJ, x_des=self.status['pos'])



    def enable_pos_traj_incr(self):
        self.set_command(mode=self.MODE_POS_TRAJ_INCR, x_des=0)

    def enable_current(self):
        self.set_command(mode=self.MODE_CURRENT, i_des=0)

    def enable_sync_mode(self):
        self.gains['enable_sync_mode'] = 1
        self._dirty_gains = 1

    def disable_sync_mode(self):
        self.gains['enable_sync_mode'] = 0
        self._dirty_gains = 1

    def is_sync_required(self, ts_last_sync):
        if ts_last_sync is None:
            return True
        return self.status['in_sync_mode'] and self.ts_last_syncd_motion > ts_last_sync

    def enable_runstop(self):
        self.gains['enable_runstop'] = 1
        self._dirty_gains = 1

    def disable_runstop(self):
        self.gains['enable_runstop'] = 0
        self._dirty_gains = 1

    def enable_guarded_mode(self):
        self.gains['enable_guarded_mode'] = 1
        self._dirty_gains = 1

    def disable_guarded_mode(self):
        self.gains['enable_guarded_mode'] = 0
        self._dirty_gains = 1

    def set_motion_limits(self, limit_neg, limit_pos, blocking=True):
        """
        Returns True if success
        """
        if limit_neg != self.motion_limits[0] or limit_pos != self.motion_limits[1]:
            # Push out immediately
            self.motion_limits = [limit_neg, limit_pos]
            payload = self.transport.get_empty_payload()
            payload[0] = self.RPC_SET_MOTION_LIMITS
            sidx = self._pack_motion_limits(payload, 1)
            return self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx],
                                         rpc_callback=self._rpc_motion_limits_reply) is not None


    def step_sentry(self, robot_status):
        if self.hw_valid:
            self.is_moving_history.pop(0)
            self.is_moving_history.append(self.status['is_moving'])
            self.status['is_moving_filtered'] = max(set(self.is_moving_history), key=self.is_moving_history.count)

    # ######################################################################
    # Primary interface to controlling the stepper
    # YAML defaults are used if values not provided
    # This allows user to override defaults every control cycle and then easily revert to defaults
    def set_guarded_contact_sensitivity(self, coeff_sensitivity_pos=None, coeff_sensitivity_neg=None):
        self._command['coeff_sensitivity_pos'] = coeff_sensitivity_pos
        self._command['coeff_sensitivity_neg'] = coeff_sensitivity_neg
        self._dirty_command = True
    
    def set_command(self, mode=None, x_des=None, v_des=None, a_des=None, i_des=None, stiffness=None, i_feedforward=None,
                    i_contact_pos=None, i_contact_neg=None):

        if True in [self.check_nan_value(d) for d in
                    (x_des, v_des, a_des, i_des, stiffness, i_feedforward, i_contact_pos, i_contact_neg)]:
            self.logger.warning('Received NaN value. dropping the command.')
            return

        if mode is not None:
            self._command['mode'] = mode

        if x_des is not None:
            self._command['x_des'] = x_des
            if self._command['mode'] == self.MODE_POS_TRAJ_INCR:
                self._command['incr_trigger'] = (self._command['incr_trigger'] + 1) % 255

        if v_des is not None:
            self._command['v_des'] = v_des
        else:
            if mode == self.MODE_VEL_PID or mode == self.MODE_VEL_TRAJ:
                self._command['v_des'] = 0
            else:
                self._command['v_des'] = self.params['motion']['vel']

        if a_des is not None:
            # Hack to avoid drift bug in firmware motion gen. Need to fix in stretch_firmware
            self._command['a_des'] = a_des
        else:
            self._command['a_des'] = self.params['motion']['accel']

        if stiffness is not None:
            self._command['stiffness'] = max(0.0, min(1.0, stiffness))
        else:
            self._command['stiffness'] = 1

        if i_feedforward is not None:
            self._command['i_feedforward'] = i_feedforward
        else:
            self._command['i_feedforward'] = 0

        if i_des is not None and mode == self.MODE_CURRENT:
            self._command['i_feedforward'] = i_des

        if i_contact_pos is not None:
            self._command['i_contact_pos'] = i_contact_pos
        else:
            self._command['i_contact_pos'] = self.params['gains']['i_contact_pos']

        if i_contact_neg is not None:
            self._command['i_contact_neg'] = i_contact_neg
        else:
            self._command['i_contact_neg'] = self.params['gains']['i_contact_neg']
        # print(time.time(), i_des, self._command['i_feedforward'],mode == self.MODE_CURRENT)
        # print(time.time(),self._command['x_des'],self._command['incr_trigger'],self._command['v_des'],self._command['a_des'])
        self._dirty_command = True

    def wait_while_is_moving(self, timeout=15.0, use_motion_generator=True):
        """
        Poll until is moving flag is false
        Return True if success
        Return False if timeout
        """
        ts = time.time()
        self.pull_status()
        s = 'is_mg_moving' if use_motion_generator else 'is_moving_filtered'
        while self.status[s] and time.time() - ts < timeout:
            time.sleep(0.1)
            self.pull_status()
        return not self.status[s]

    def wait_until_at_setpoint(self, timeout=15.0):
        """
        Poll until near setpoint
        Return True if success
        Return False if timeout
        """
        ts = time.time()
        self.pull_status()
        while not self.status['near_pos_setpoint'] and time.time() - ts < timeout:
            time.sleep(0.1)
            #self.pretty_print()
            self.pull_status()
        return self.status['near_pos_setpoint']

    def _rpc_motion_limits_reply(self, reply):
        if reply[0] != self.RPC_REPLY_MOTION_LIMITS:
            print('Error RPC_REPLY_MOTION_LIMITS', reply[0])

class StepperCalibration(StepperDefn):
    def __init__(self):
        pass
    # ####################### Encoder Calibration ######################

    def get_chip_id(self):
        self.turn_menu_interface_on()
        time.sleep(0.5)
        cid = self.menu_transaction(b'b', do_print=False)[0][:-2]
        self.turn_rpc_interface_on()
        time.sleep(0.5)
        return cid.decode('utf-8')

    def read_encoder_calibration_from_YAML(self):
        device_name = self.usb[5:]
        sn = self.robot_params[device_name]['serial_no']
        fn = 'calibration_steppers/' + device_name + '_' + sn + '.yaml'
        enc_data = read_fleet_yaml(fn)
        return enc_data

    def write_encoder_calibration_to_YAML(self, data, filename=None, fleet_dir=None):
        device_name = self.usb[5:]
        if filename is None:
            sn = self.robot_params[device_name]['serial_no']
            filename = 'calibration_steppers/' + device_name + '_' + sn + '.yaml'
        print('Writing encoder calibration: %s' % filename)
        write_fleet_yaml(filename, data, fleet_dir=fleet_dir)

    def read_encoder_calibration_from_flash(self):
        self.turn_menu_interface_on()
        time.sleep(0.5)
        time.sleep(0.5)
        self.logger.info('Reading encoder calibration...')
        e = self.menu_transaction(b'q', do_print=False)[19]
        print(e)
        self.turn_rpc_interface_on()
        self.push_command()
        self.logger.info('Reseting board')
        self.board_reset()
        self.push_command()
        e = e[:-4].decode('utf-8')  # We now have string of floats, convert to list of floats

        enc_calib = []

        while len(e):
            ff = e.find(',')
            if ff != -1:
                enc_calib.append(float(e[:ff]))
                e = e[ff + 2:]
            else:
                enc_calib.append(float(e))
                e = []
        if len(enc_calib) == 16384:
            self.logger.info('Successful read of encoder calibration')
        else:
            self.logger.error('Failed to read encoder calibration')
        return enc_calib

    def write_encoder_calibration_to_flash(self, data):
        if not self.hw_valid:
            return
        # This will take a few seconds. Blocks until complete.
        if len(data) != 16384:
            self.logger.warning('Bad encoder data. Got data of len %d' % len(data))
        else:
            self.logger.info('Writing encoder calibration...')
            payload = self.transport.get_empty_payload()
            if self.board_info['hardware_id'] < 5:
                total_pages = 256
                floats_per_page = 64
            else:
                # SAMD51 uses block erases, each block is 8kb large
                total_pages = 8
                floats_per_page = 2048
            for p in range(total_pages):
                if p % 10 == 0:
                    sys.stdout.write('.')
                    sys.stdout.flush()
                payload[0] = self.RPC_SET_ENC_CALIB
                payload[1] = p
                sidx = 2
                for i in range(floats_per_page):
                    pack_float_t(payload, sidx, data[p * floats_per_page + i])
                    sidx += 4
                # self.logger.debug('Sending encoder calibration rpc of size',sidx)
                self.transport.do_rpc(blocking=True, is_push=True, payload=payload[:sidx],
                                      rpc_callback=self._rpc_enc_calib_reply)

    # ######################Menu Interface ################################3

    def turn_rpc_interface_on(self):
        self.menu_transaction(b'zyx')

    def turn_menu_interface_on(self):
        if not self.hw_valid:
            return
        payload = arr.array('B', [self.RPC_SET_MENU_ON])
        self.transport.do_rpc(blocking=True, is_push=True, payload=payload,
                              rpc_callback=self._rpc_menu_on_reply)

    def print_menu(self):
        self.menu_transaction(b'm')

    def menu_transaction(self, x, do_print=True):
        #NOTE: Not yet thread safe with C++ backend, but OK if not using that concurrentl
        #Which typically is not the case when using this
        if not self.hw_valid:
            return
        self.transport.ser.write(x)
        time.sleep(0.1)
        reply = []
        while self.transport.ser.inWaiting():
            r = self.transport.ser.readline()
            if do_print:
                if type(r) == bytes:
                    print(r.decode('UTF-8'), end=' ')
                else:
                    print(r, end=' ')
            reply.append(r)
        return reply

    def _rpc_enc_calib_reply(self, reply):
        if reply[0] != self.RPC_REPLY_ENC_CALIB:
            self.logger.error(f'Error RPC_REPLY_ENC_CALIB {reply[0]}')

    def _rpc_menu_on_reply(self, reply):
        if reply[0] != self.RPC_REPLY_MENU_ON:
            self.logger.error(f'Error RPC_REPLY_MENU_ON {reply[0]}')



class StepperTrace(StepperDefn):
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
        self.timestamp.reset() #Timestamp holds state, reset within lock to avoid threading issues
        self.n_trace_read=1
        ts=time.time()
        while ( self.n_trace_read) and time.time()-ts<60.0:
            payload = arr.array('B', [self.RPC_READ_TRACE])
            self.transport.do_rpc(blocking=True, is_push=False, payload=payload,
                                  rpc_callback=self._rpc_read_firmware_trace_reply)
            time.sleep(.001)
        return self.trace_buf
    
    def _unpack_debug_trace(self,s,unpack_to):
        sidx=0
        unpack_to['u8_1']=unpack_uint8_t(s[sidx:]);sidx+=1
        unpack_to['u8_2'] = unpack_uint8_t(s[sidx:]);sidx += 1
        unpack_to['f_1'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['f_2'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['f_3'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx

    def _unpack_print_trace(self,s,unpack_to):
        sidx=0
        line_len=32
        unpack_to['timestamp']=self.timestamp.set(unpack_uint64_t(s[sidx:]));sidx += 8
        unpack_to['line'] = unpack_string_t(s[sidx:], line_len); sidx += line_len
        unpack_to['x'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx
    def _rpc_read_firmware_trace_reply(self, reply):
        if len(reply)>0 and reply[0] == self.RPC_REPLY_READ_TRACE:
            self.n_trace_read=reply[1]
            self.trace_buf.append({'id': len(self.trace_buf), 'status': {},'debug':{},'print':{}})
            if reply[2]==self.TRACE_TYPE_STATUS:
                self.trace_buf[-1]['status']= self.status_zero.copy()
                self._unpack_status(reply[3:],unpack_to=self.trace_buf[-1]['status'])
            elif reply[2]==self.TRACE_TYPE_DEBUG:
                self._unpack_debug_trace(reply[3:],unpack_to=self.trace_buf[-1]['debug'])
            elif reply[2]==self.TRACE_TYPE_PRINT:
                self._unpack_print_trace(reply[3:],unpack_to=self.trace_buf[-1]['print'])
            else:
                print('Unrecognized trace type %d'%reply[2])
        else:
            print('Error RPC_REPLY_READ_TRACE')
            self.n_trace_read=0
            self.trace_buf = []


class StepperAux(StepperDefn):
    def __init__(self):
        self.status_aux={'cmd_cnt_rpc':0,'cmd_cnt_exec':0,'cmd_rpc_overflow':0,'sync_irq_cnt':0,'sync_irq_overflow':0}
        self._load_test_payload = arr.array('B', range(256)) * 4
    def push_load_test(self):
        """
        Returns True if success
        """
        if not self.hw_valid:
            return False
        payload = self.transport.get_empty_payload()
        payload[0] = self.RPC_LOAD_TEST_PUSH
        payload[1:] = self._load_test_payload
        return self.transport.do_rpc(blocking=True, is_push=True, payload=payload,
                              rpc_callback=self._rpc_load_test_push_reply) is not None

    def pull_load_test(self):
        if not self.hw_valid:
            return
        payload = arr.array('B',[self.RPC_LOAD_TEST_PULL])
        self.transport.do_rpc(blocking=True, is_push=False, payload=payload,
                              rpc_callback=self._rpc_load_test_pull_reply)

    def pull_status_aux(self,blocking=True):
        """
        Returns True if success
        """
        if not self.hw_valid:
            return False
        payload = arr.array('B',[self.RPC_GET_STATUS_AUX])
        return self.transport.do_rpc(blocking=blocking, is_push=False, payload=payload,
                              rpc_callback=self._rpc_status_aux_reply) is not None

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
            print('Successful load test pull')
        else:
            print('Error RPC_REPLY_LOAD_TEST_PULL', reply[0])


    def __unpack_status_aux(self,s):
        sidx = 0
        self.status_aux['cmd_cnt_rpc'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        self.status_aux['cmd_cnt_exec'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        self.status_aux['cmd_rpc_overflow'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        self.status_aux['sync_irq_cnt'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        self.status_aux['sync_irq_overflow'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        return sidx

    def _rpc_status_aux_reply(self, reply):
        if reply[0] == self.RPC_REPLY_STATUS_AUX:
            nr = self.__unpack_status_aux(reply[1:])
        else:
            print('Error RPC_REPLY_STATUS', reply[0])


class StepperHelpers(StepperDefn):
    def __init__(self):
        pass
    ########### Handle current and effort conversions  ###########

    # Effort_ticks are in the units of the uC current controller (0-255 8 bit)
    # Conversion to A is based on the sense resistor and motor driver (see firmware)
    def current_to_effort_ticks(self, i_A):
        if self.board_info['hardware_id'] == 0:  # I = Vref / (10 * R), Rs = 0.10 Ohm, Vref = 3.3V -->3.3A
            mA_per_tick = (3300 / 255) / (10 * 0.1)
        if self.board_info['hardware_id'] >= 1 and self.board_info[
            'hardware_id'] <= 5:  # I = Vref / (5 * R), Rs = 0.150 Ohm, Vref = 3.3V -->4.4A
            mA_per_tick = (3300 / 255) / (5 * 0.15)
        if self.board_info['hardware_id'] >= 6:  # I = 3.3V => 7.78A
            mA_per_tick = (3300 / 4096) / (0.424)
            effort_ticks = (i_A * 1000.0) / mA_per_tick
            return min(4096, max(-4096, int(effort_ticks)))

        effort_ticks = (i_A * 1000.0) / mA_per_tick
        return min(255, max(-255, int(effort_ticks)))

    def effort_ticks_to_current(self, e):
        if self.board_info['hardware_id'] == 0:  # I = Vref / (10 * R), Rs = 0.10 Ohm, Vref = 3.3V -->3.3A
            mA_per_tick = (3300 / 255) / (10 * 0.1)
        if self.board_info['hardware_id'] >= 1 and self.board_info[
            'hardware_id'] <= 5:  # I = Vref / (5 * R), Rs = 0.150 Ohm, Vref = 3.3V -->4.4A
            mA_per_tick = (3300 / 255) / (5 * 0.15)
        if self.board_info['hardware_id'] >= 6:  # I = Vref / (5 * R), Rs = 0.150 Ohm, Vref = 3.3V -->4.4A

            mA_per_tick = (3300 / 4096) / (0.424)
        return e * mA_per_tick / 1000.0

    # Effort_pct is defined as a percentage of the maximum allowable motor winding current
    # Range is -100.0 to 100.0
    def current_to_effort_pct(self, i_A):
        if i_A > 0:
            return 100 * max(0.0, min(1.0, i_A / self.gains['iMax_pos']))
        else:
            return 100 * min(0.0, max(-1.0, i_A / abs(self.gains['iMax_neg'])))

    def effort_pct_to_current(self, e_pct):
        if e_pct > 0:
            return min(1.0, e_pct / 100.0) * self.gains['iMax_pos']
        else:
            return max(-1.0, e_pct / 100.0) * abs(self.gains['iMax_neg'])

    def get_temperature(self, raw):
        v = 3.3*(raw/4095)
        temp = (v - 0.5)/(0.01)
        return temp

    def get_voltage(self,raw):
        if self.board_info['hardware_id'] < 5:
            raw_to_V = 20.0/1024 #10bit adc, 0-20V per 0-3.3V reading
            v = (raw*raw_to_V)-0.3 #0.3 is needed to account for leakage current of TVS
        else:
            #For stretch 4 Steppers
            raw_to_V = (3.3/4095)*(11) #12bit adc,
            v = (raw*raw_to_V)
        return v
    def get_stepper_type(self, raw):
        try:
            mt = [None, 'hello-motor-omni-0', 'hello-motor-omni-1','hello-motor-omni-2', 'hello-motor-arm', 'hello-motor-lift']
            return mt[raw]
        except IndexError:
            return None

    def get_decay_mode(self):
        decay_setting = ['Mixed decay', 'Smart Tune Decay', 'Slow decay']
        for i in range(0, len(decay_setting)):
            if self.gains['decay_setting'] == i:
                return decay_setting[i]
        return 'Unknown decay mode'

    def get_toff_time(self):
        toff_setting = ['32us', '16us', '7us']
        for i in range(0, len(toff_setting)):
            if self.gains['toff_setting'] == i:
                return toff_setting[i]
        return 'Unknown toff time'

    def check_nan_value(self,x):
        try:
            return math.isnan(x)
        except TypeError:
            return False

    def write_stepper_type_to_flash(self, motor_type):  # P5
        if not self.hw_valid:
            return
        mt = [None, 'hello-motor-omni-0', 'hello-motor-omni-1','hello-motor-omni-2', 'hello-motor-arm', 'hello-motor-lift']
        for i in range(0, len(mt)):
            if motor_type == mt[i]:
                motor_type = i
                break
        if not isinstance(motor_type, int) or (motor_type < 0 or motor_type > 5):
            print("Error Unrecoginzed Stepper Motor Type")
            return
        payload = arr.array('B', [self.RPC_SET_STEPPER_TYPE,motor_type])
        self.transport.do_rpc(blocking=True, is_push=True, payload=payload,
                              rpc_callback=self._rpc_write_stepper_type_to_flash_reply)

    def read_stepper_type_from_flash(self):
        if not self.hw_valid:
            return
        payload = arr.array('B', [self.RPC_READ_STEPPER_TYPE_FROM_FLASH])
        self.transport.do_rpc(blocking=True, is_push=False, payload=payload,
                              rpc_callback=self._rpc_read_stepper_type_from_flash_reply)

    def _rpc_write_stepper_type_to_flash_reply(self, reply):
        if reply[0] != self.RPC_REPLY_SET_STEPPER_TYPE:
            print('Error RPC_REPLY_SET_STEPPER_TYPE', reply[0])

    def _rpc_read_stepper_type_from_flash_reply(self, reply):
        if reply[0] == self.RPC_REPLY_READ_STEPPER_TYPE_FROM_FLASH:
            sidx = 0
            self.board_info['stepper_type'] = self.get_stepper_type(unpack_uint8_t(reply[1:][sidx:]))
        else:
            print('Error RPC_REPLY_READ_STEPPER_TYPE_FROM_FLASH', reply[0])

class StepperBase(StepperMotion, StepperCalibration, StepperTrace, StepperHelpers,Device):
    """
    API to the Stretch Stepper Board
    """
    def __init__(self, usb,name=None,backend=None):
        if name is None:
            name=usb[5:] #Pull from usb device name
        Device.__init__(self, name=name)
        StepperDefn.__init__(self)
        StepperMotion.__init__(self)
        StepperCalibration.__init__(self)

        StepperTrace.__init__(self)
        StepperHelpers.__init__(self)

        self.usb=usb
        if backend is None:
            backend=self.params['transport']['default_backend']
        self.transport = Transport(port_name=usb, logger=self.logger,
                                   default_backend=backend,
                                   qid=self.params['transport']['qid'])
        self._command = {'mode':0, 'x_des':0,'v_des':0,'a_des':0,'stiffness':1.0,'i_feedforward':0.0,'i_contact_pos':0,'i_contact_neg':0,'incr_trigger':0,'coeff_sensitivity_pos':0,'coeff_sensitivity_neg':0}
        self.status: "StepperStatus" = {'mode': 0, 'effort_ticks': 0, 'effort_pct':0,'current':0,'pos': 0, 'vel': 0, 'err':0,'diag': 0,'timestamp': 0, 'debug':0,'guarded_event':0,
                       'transport': self.transport.status,'pos_calibrated':0,'runstop_on':0,'near_pos_setpoint':0,'near_vel_setpoint':0,
                       'is_moving':0,'is_moving_filtered':0,'is_drv_fault':0,'is_mg_accelerating':0,'is_mg_moving':0,'calibration_rcvd': 0,'in_guarded_event':0,
                       'in_safety_event':0,'waiting_on_sync':0,'in_sync_mode':0,'trace_on':0,'ctrl_cycle_cnt':0,
                       'voltage':0, 'temperature':0}

        self.status_zero=self.status.copy()
        self.board_info={'board_variant':None, 'firmware_version':None,'protocol_version':None,'hardware_id':0, 'stepper_type':0}
        self._dirty_command = False
        self._dirty_gains = False
        self._dirty_trigger = False
        self._dirty_read_gains_from_flash=False
        self._trigger=0
        self._trigger_data=0
        self.hw_valid=False
        self.gains = self.params['gains'].copy()
        self.gains_flash = {}

    # ###########  Device Methods #############
    def startup(self):
        try:
            Device.startup(self)
            self.hw_valid = self.transport.startup()
            if self.hw_valid:
                # Pull board info
                payload = arr.array('B', [self.RPC_GET_STEPPER_BOARD_INFO])
                self.transport.do_rpc(blocking=True, is_push=False, payload=payload,
                                     rpc_callback=self._rpc_board_info_reply,
                                     backend=self.transport.BACKEND_PY_SERIAL)  # Use py as C may not be supported yet
                self.transport.configure_version(self.board_info['firmware_version'])
                return True
            return False
        except KeyError:
            self.hw_valid =False
            return False

    #Configure control mode prior to calling this on process shutdown (or default to freewheel)
    def stop(self):
        Device.stop(self)
        if not self.hw_valid:
            return
        self.logger.info(f'Shutting down Stepper on: {self.usb}')
        self.enable_safety()
        self.push_command()
        self.transport.stop()
        self.hw_valid = False

    def push_command(self, blocking=True):
        """
        Return True if success
        """
        if not self.hw_valid:
            return False
        payload = self.transport.get_empty_payload()
        success=True
        if self._dirty_trigger:
            payload[0] = self.RPC_SET_TRIGGER
            sidx = self._pack_trigger(payload, 1)
            success=success and self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx],rpc_callback=self._rpc_trigger_reply) is not None
            self._trigger=0
            self._dirty_trigger = False

        if self._dirty_gains:
            payload[0] = self.RPC_SET_GAINS
            sidx = self._pack_gains(payload, 1)
            success = success and self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx],
                                                      rpc_callback=self._rpc_gains_reply) is not None
            self._dirty_gains = False

        if self._dirty_command:
            if self.status['in_sync_mode']:  # Mark the time of latest new motion command sent
                self.ts_last_syncd_motion = time.time()
            else:
                self.ts_last_syncd_motion = 0

            payload[0] = self.RPC_SET_COMMAND
            sidx = self._pack_command(payload, 1)
            success=success and self.transport.do_rpc(blocking=blocking, is_push=True, payload=payload[:sidx],
                                                      rpc_callback=self._rpc_command_reply) is not None
            self._dirty_command = False
        return success


    def pull_status(self, blocking=True):
        """
        Return True if success
        """
        if not self.hw_valid:
            return False
        success=True
        if self._dirty_read_gains_from_flash:
            payload = arr.array('B', [self.RPC_READ_GAINS_FROM_FLASH])
            success = success and self.transport.do_rpc(blocking=blocking, is_push=False, payload=payload,
                                         rpc_callback=self._rpc_read_gains_from_flash_reply) is not None

            self._dirty_read_gains_from_flash = False
        payload = arr.array('B', [self.RPC_GET_STATUS])
        success = success and self.transport.do_rpc(blocking=blocking, is_push=False, payload=payload,
                                                  rpc_callback=self._rpc_status_reply) is not None
        return success

    def set_gains(self,g=None):
        if g is not None:
            self.gains=g.copy()
        self._dirty_gains = True

    def write_gains_to_flash(self):
        self._trigger = self._trigger | self.TRIGGER_WRITE_GAINS_TO_FLASH
        self._dirty_trigger = True

    def read_gains_from_flash(self):
        self._dirty_read_gains_from_flash=True

    def board_reset(self):
        self._trigger = self._trigger | self.TRIGGER_BOARD_RESET
        self._dirty_trigger=True

    def reset_mark_position_on_contact(self):
        self._trigger = self._trigger | self.TRIGGER_RESET_MARK_POS_ON_CONTACT
        self._dirty_trigger=True

    def mark_position_on_contact(self,x):
        self._trigger_data = x
        self._trigger = self._trigger | self.TRIGGER_MARK_POS_ON_CONTACT
        self._dirty_trigger=True

    def mark_position(self,x):
        if self.status['mode']!=self.MODE_SAFETY and self.status['mode'] != self.MODE_HOLD:
            self.logger.warning('Can not mark position. Must be in MODE_SAFETY for %s'%self.usb)
            return
        self._trigger_data=x
        self._trigger = self._trigger | self.TRIGGER_MARK_POS
        self._dirty_trigger=True

    def reset_motion_gen(self):
        self._trigger = self._trigger | self.TRIGGER_RESET_MOTION_GEN
        self._dirty_trigger = True

    def reset_pos_calibrated(self):
        self._trigger = self._trigger | self.TRIGGER_RESET_POS_CALIBRATED
        self._dirty_trigger = True

    def set_pos_calibrated(self):
        self._trigger = self._trigger | self.TRIGGER_POS_CALIBRATED
        self._dirty_trigger = True

    def load_rpc_results(self, wait_on_result=True):
        return self.transport.load_rpc_results(wait_on_result)

    def pretty_print(self): #P1
        print('-----------')
        print('Mode', self.MODE_NAMES[self.status['mode']])
        print('x_des (rad)', self._command['x_des'], '(deg)',rad_to_deg(self._command['x_des']))
        print('v_des (rad)', self._command['v_des'], '(deg)',rad_to_deg(self._command['v_des']))
        print('a_des (rad)', self._command['a_des'], '(deg)',rad_to_deg(self._command['a_des']))
        print('Stiffness',self._command['stiffness'])
        print('Feedforward', self._command['i_feedforward'])
        print('Pos (rad)', self.status['pos'], '(deg)',rad_to_deg(self.status['pos']))
        print('Vel (rad/s)', self.status['vel'], '(deg)',rad_to_deg(self.status['vel']))
        print('Effort (Ticks)', self.status['effort_ticks'])
        print('Effort (Pct)', self.status['effort_pct'])
        print('Current (A)', self.status['current'])
        if self.board_info['hardware_id'] >= 3:
            print('Voltage (V)', self.status['voltage'])
        if self.board_info['hardware_id']>=6:
            print('Temperature (C)',self.status['temperature'])
        print('Error (deg)', rad_to_deg(self.status['err']))
        print('Debug', self.status['debug'])
        print('Guarded Events:', self.status['guarded_event'])
        print('Diag', format(self.status['diag'], '032b'))
        print('       Position Calibrated:', self.status['pos_calibrated'])
        print('       Runstop on:', self.status['runstop_on'])
        print('       Near Pos Setpoint:', self.status['near_pos_setpoint'])
        print('       Near Vel Setpoint:', self.status['near_vel_setpoint'])
        print('       Is Moving:', self.status['is_moving'])
        print('       Is Moving Filtered:', self.status['is_moving_filtered'])
        print('       Is Drv Fault:', self.status['is_drv_fault'])
        print('       Is MG Accelerating:', self.status['is_mg_accelerating'])
        print('       Is MG Moving:', self.status['is_mg_moving'])
        print('       Encoder Calibration in Flash:', self.status['calibration_rcvd'])
        print('       In Guarded Event:', self.status['in_guarded_event'])
        print('       In Safety Event:', self.status['in_safety_event'])
        print('       Waiting on Sync:', self.status['waiting_on_sync'])
        print('       Trace recording:', self.status['trace_on'])

    def pause_transport(self):
        self.transport.pause()

    def unpause_transport(self):
        self.transport.unpause()

    # ################ PACK / UNPACK #####################
    def _unpack_board_info(self,s):
        sidx=0
        self.board_info['board_variant'] = unpack_string_t(s[sidx:], 20).strip('\x00')
        self.board_info['hardware_id']=int(self.board_info['board_variant'][-1])
        sidx += 20
        self.board_info['firmware_version'] = unpack_string_t(s[sidx:], 20).strip('\x00')
        sidx += 20
        if self.board_info['firmware_version'].find('hello-stepper2')==0: #Newer format, length is 30 not 20
            str10=unpack_string_t(s[sidx:], 10).strip('\x00')
            sidx += 10
            self.board_info['firmware_version']=self.board_info['firmware_version']+str10
        self.board_info['protocol_version'] = self.board_info['firmware_version'][self.board_info['firmware_version'].rfind('p'):]
        
        self.board_info['stepper_type'] = self.get_stepper_type(unpack_uint8_t(s[sidx:]));sidx += 1
        return sidx

    def _un_pack_command_reply(self,s):
        sidx = 0
        self.status['ctrl_cycle_cnt'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        return sidx

    def _rpc_command_reply(self, reply):
        if reply[0] == self.RPC_REPLY_COMMAND:
            nr = self._un_pack_command_reply(reply[1:])
        else:
            print('Error RPC_REPLY_COMMAND', reply[0])

    def _un_pack_gains(self,s): #Base
        sidx=0
        self.gains_flash['pKp_d'] = unpack_float_t(s[sidx:]);sidx+=4
        self.gains_flash['pKi_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['pKd_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['pLPF'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['pKi_limit'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vKp_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vKi_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vKd_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vLPF'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vKi_limit'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vTe_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['iMax_pos'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['iMax_neg'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['phase_advance_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['pos_near_setpoint_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vel_near_setpoint_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vel_status_LPF'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['effort_LPF'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['safety_stiffness'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['i_safety_feedforward'] = unpack_float_t(s[sidx:]);sidx += 4

        config = unpack_uint8_t(s[sidx:]);sidx += 1
        self.gains_flash['safety_hold']= int(config & self.CONFIG_SAFETY_HOLD>0)
        self.gains_flash['enable_runstop'] = int(config & self.CONFIG_ENABLE_RUNSTOP>0)
        self.gains_flash['enable_sync_mode'] = int(config & self.CONFIG_ENABLE_SYNC_MODE>0)
        self.gains_flash['enable_guarded_mode'] = int(config & self.CONFIG_ENABLE_GUARDED_MODE > 0)
        self.gains_flash['flip_encoder_polarity'] = int(config & self.CONFIG_FLIP_ENCODER_POLARITY > 0)
        self.gains_flash['flip_effort_polarity'] = int(config & self.CONFIG_FLIP_EFFORT_POLARITY > 0)
        self.gains_flash['enable_vel_watchdog'] = int(config & self.CONFIG_ENABLE_VEL_WATCHDOG > 0)

        #self.gains_flash['voltage_LPF'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['toff_setting'] = unpack_uint8_t(s[sidx:]);
        sidx += 1
        self.gains_flash['decay_setting'] = unpack_uint8_t(s[sidx:]);
        sidx += 1
        self.gains_flash['drv8262_min_vref'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        self.gains_flash['k_calibration_step'] = unpack_float_t(s[sidx:])
        sidx += 4
        return sidx
    
    def _unpack_status(self,s,unpack_to=None):

        if unpack_to is None:
            unpack_to=self.status
        sidx=0
        unpack_to['mode']=unpack_uint8_t(s[sidx:]);sidx+=1
        unpack_to['effort_ticks'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['current']=self.effort_ticks_to_current(unpack_to['effort_ticks'])
        unpack_to['effort_pct'] = self.current_to_effort_pct(unpack_to['current'])
        unpack_to['pos'] = unpack_double_t(s[sidx:]);sidx+=8
        unpack_to['vel'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['err'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['diag'] = unpack_uint32_t(s[sidx:]);sidx += 4
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:]));sidx += 8
        unpack_to['debug'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['guarded_event'] = unpack_uint32_t(s[sidx:]);sidx += 4
        foo = unpack_float_t(s[sidx:]);sidx += 4 #deprecated
        foo = unpack_uint16_t(s[sidx:]);sidx += 2 #deprecated

        unpack_to['pos_calibrated'] =unpack_to['diag'] & self.DIAG_POS_CALIBRATED > 0
        unpack_to['runstop_on'] =unpack_to['diag'] & self.DIAG_RUNSTOP_ON > 0
        unpack_to['near_pos_setpoint'] =unpack_to['diag'] & self.DIAG_NEAR_POS_SETPOINT > 0
        unpack_to['near_vel_setpoint'] = unpack_to['diag'] & self.DIAG_NEAR_VEL_SETPOINT > 0
        unpack_to['is_moving'] =unpack_to['diag'] & self.DIAG_IS_MOVING > 0
        unpack_to['is_drv_fault'] =unpack_to['diag'] & self.DIAG_IS_DRV_FAULT > 0
        unpack_to['is_mg_accelerating'] = unpack_to['diag'] & self.DIAG_IS_MG_ACCELERATING > 0
        unpack_to['is_mg_moving'] =unpack_to['diag'] & self.DIAG_IS_MG_MOVING > 0
        unpack_to['calibration_rcvd'] = unpack_to['diag'] & self.DIAG_CALIBRATION_RCVD > 0
        unpack_to['in_guarded_event'] = unpack_to['diag'] & self.DIAG_IN_GUARDED_EVENT > 0
        unpack_to['in_safety_event'] = unpack_to['diag'] & self.DIAG_IN_SAFETY_EVENT > 0
        unpack_to['waiting_on_sync'] = unpack_to['diag'] & self.DIAG_WAITING_ON_SYNC > 0
        unpack_to['in_sync_mode'] = unpack_to['diag'] & self.DIAG_IN_SYNC_MODE > 0
        unpack_to['trace_on'] = unpack_to['diag'] & self.DIAG_IS_TRACE_ON > 0

        # if unpack_to['diag'] & self.DIAG_TRAJ_WAITING_ON_SYNC > 0:
        #     unpack_to['waypoint_traj']['state']='waiting_on_sync'
        # elif unpack_to['diag'] & self.DIAG_TRAJ_ACTIVE > 0:
        #     unpack_to['waypoint_traj']['state']='active'
        # else:
        #     unpack_to['waypoint_traj']['state']='idle'

        unpack_to['voltage']=self.get_voltage(unpack_float_t(s[sidx:]));sidx+=4
        unpack_to['temperature']=self.get_temperature(unpack_float_t(s[sidx:]));sidx+=4
        return sidx

    def _pack_motion_limits(self, s, sidx):
        pack_float_t(s, sidx, self.motion_limits[0])
        sidx += 4
        pack_float_t(s, sidx, self.motion_limits[1])
        sidx += 4
        return sidx

    def _pack_command(self, s, sidx):
        pack_uint8_t(s, sidx, self._command['mode'])
        sidx += 1
        pack_float_t(s, sidx, self._command['x_des'])
        sidx += 4
        pack_float_t(s, sidx, self._command['v_des'])
        sidx += 4
        pack_float_t(s, sidx, self._command['a_des'])
        sidx += 4
        pack_float_t(s, sidx, self._command['stiffness'])
        sidx += 4
        pack_float_t(s, sidx, self._command['i_feedforward'])
        sidx += 4
        pack_float_t(s, sidx, self._command['i_contact_pos'])
        sidx += 4
        pack_float_t(s, sidx, self._command['i_contact_neg'])
        sidx += 4
        pack_uint8_t(s, sidx, self._command['incr_trigger'])
        sidx += 1
        return sidx

    def _pack_gains(self,s,sidx): #Base
        pack_float_t(s, sidx, self.gains['pKp_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['pKi_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['pKd_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['pLPF']);sidx += 4
        pack_float_t(s, sidx, self.gains['pKi_limit']);sidx += 4
        pack_float_t(s, sidx, self.gains['vKp_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['vKi_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['vKd_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['vLPF']);sidx += 4
        pack_float_t(s, sidx, self.gains['vKi_limit']); sidx += 4
        pack_float_t(s, sidx, self.gains['vTe_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['iMax_pos']);sidx += 4
        pack_float_t(s, sidx, self.gains['iMax_neg']);sidx += 4
        pack_float_t(s, sidx, self.gains['phase_advance_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['pos_near_setpoint_d']); sidx += 4
        pack_float_t(s, sidx, self.gains['vel_near_setpoint_d']); sidx += 4
        pack_float_t(s, sidx, self.gains['vel_status_LPF']);sidx += 4
        pack_float_t(s, sidx, self.gains['effort_LPF']);sidx += 4
        pack_float_t(s, sidx, self.gains['safety_stiffness']);sidx += 4
        pack_float_t(s, sidx, self.gains['i_safety_feedforward']);sidx += 4
        config=0
        if self.gains['safety_hold']:
            config=config | self.CONFIG_SAFETY_HOLD
        if self.gains['enable_runstop']:
            config=config | self.CONFIG_ENABLE_RUNSTOP
        if self.gains['enable_sync_mode']:
            config=config | self.CONFIG_ENABLE_SYNC_MODE
        if self.gains['enable_guarded_mode']:
            config=config | self.CONFIG_ENABLE_GUARDED_MODE
        if self.gains['flip_encoder_polarity']:
            config = config | self.CONFIG_FLIP_ENCODER_POLARITY
        if self.gains['flip_effort_polarity']:
            config = config | self.CONFIG_FLIP_EFFORT_POLARITY
        if self.gains['enable_vel_watchdog']:
            config=config | self.CONFIG_ENABLE_VEL_WATCHDOG
        pack_uint8_t(s, sidx, config); sidx += 1

        #pack_float_t(s, sidx, self.gains['voltage_LPF']);
        #sidx += 4
        pack_uint8_t(s, sidx, int(self.gains['toff_setting']));
        sidx += 1
        pack_uint8_t(s, sidx, int(self.gains['decay_setting']));
        sidx += 1
        pack_uint16_t(s, sidx, int(self.gains['drv8262_min_vref']))
        sidx += 2
        pack_float_t(s, sidx, self.gains['k_calibration_step'])
        sidx += 4

        return sidx

    def _pack_trigger(self, s, sidx):
        pack_uint32_t(s, sidx, self._trigger)
        sidx += 4
        pack_float_t(s, sidx, self._trigger_data)
        sidx += 4
        return sidx

    # ################### RPC CALLBACKS ####################

    def _rpc_board_info_reply(self, reply):
        if reply[0] == self.RPC_REPLY_STEPPER_BOARD_INFO:
            self._unpack_board_info(reply[1:])
        else:
            print('Error RPC_REPLY_STEPPER_BOARD_INFO', reply[0])

    def _rpc_gains_reply(self, reply):
        if reply[0] != self.RPC_REPLY_GAINS:
            print('Error RPC_REPLY_GAINS', reply[0])

    def _rpc_trigger_reply(self, reply):
        if reply[0] != self.RPC_REPLY_SET_TRIGGER:
            print('Error RPC_REPLY_SET_TRIGGER', reply[0])

    def _rpc_command_reply(self, reply):
        if reply[0] != self.RPC_REPLY_COMMAND:
            print('Error RPC_REPLY_COMMAND', reply[0])


    def _rpc_status_reply(self, reply):
        if reply[0] == self.RPC_REPLY_STATUS:
            nr = self._unpack_status(reply[1:])
        else:
            print('Error RPC_REPLY_STATUS', reply[0])

    def _rpc_read_gains_from_flash_reply(self, reply):
        if reply[0] == self.RPC_REPLY_READ_GAINS_FROM_FLASH:
            nr = self._un_pack_gains(reply[1:])
        else:
            print('Error RPC_REPLY_READ_GAINS_FROM_FLASH', reply[0])
    
    def enable_rate_logging(self,max_samples=1000):
        self.transport.n_rate_log=max_samples

    def get_rate_log(self):
        return self.transport.rate_log

# ######################## STEPPER PROTOCOL P7 #################################

class Stepper_Protocol_P7(StepperBase):
    def foo(self):
        pass

# ######################## STEPPER PROTOCOL P8 #################################

class Stepper_Protocol_P8(StepperBase):
    def _pack_gains(self, s, sidx):
        sidx = StepperBase._pack_gains(self, s, sidx)
        pack_float_t(s, sidx, self.gains['coeff_acc_pos'])
        sidx += 4
        pack_float_t(s, sidx, self.gains['coeff_intercept_pos'])
        sidx += 4
        pack_float_t(s, sidx, self.gains['coeff_acc_neg'])
        sidx += 4
        pack_float_t(s, sidx, self.gains['coeff_intercept_neg'])
        sidx += 4
        return sidx

    def _un_pack_gains(self,s):
        sidx = StepperBase._un_pack_gains(self,s)
        self.gains_flash['coeff_acc_pos'] = unpack_float_t(s[sidx:])
        sidx += 4
        self.gains_flash['coeff_intercept_pos'] = unpack_float_t(s[sidx:])
        sidx += 4
        self.gains_flash['coeff_acc_neg'] = unpack_float_t(s[sidx:])
        sidx += 4
        self.gains_flash['coeff_intercept_neg'] = unpack_float_t(s[sidx:])
        sidx += 4
        return sidx
    
    def _pack_command(self, s, sidx):
        sidx = StepperBase._pack_command(self, s, sidx)
        pack_float_t(s, sidx, self._command['coeff_sensitivity_pos'])
        sidx += 4
        pack_float_t(s, sidx, self._command['coeff_sensitivity_neg'])
        sidx += 4
        return sidx
    
    def set_command(self,mode=None, x_des=None, v_des=None, a_des=None,i_des=None, stiffness=None,i_feedforward=None, coeff_sensitivity_pos=None, coeff_sensitivity_neg=None):
        if  True in [self.check_nan_value(d) for d in (x_des, v_des, a_des, i_des, stiffness, i_feedforward)]:
            self.logger.warning('Received NaN value. dropping the command.')
            return

        #if self.name=='hello-motor-omni-0':
        if mode is not None:
            self._command['mode'] = mode

        if x_des is not None:
            self._command['x_des'] = x_des
            if self._command['mode'] == self.MODE_POS_TRAJ_INCR:
                self._command['incr_trigger'] = (self._command['incr_trigger']+1)%255

        if v_des is not None:
            self._command['v_des'] = v_des
        else:
            if mode == self.MODE_VEL_PID or mode == self.MODE_VEL_TRAJ:
                self._command['v_des'] = 0
            else:
                self._command['v_des'] = self.params['motion']['vel']

        if a_des is not None:
            #Hack to avoid drift bug in firmware motion gen. Need to fix in stretch_firmware
            self._command['a_des'] = a_des
        else:
            self._command['a_des'] = self.params['motion']['accel']

        if stiffness is not None:
            self._command['stiffness'] = max(0.0, min(1.0, stiffness))
        else:
            self._command['stiffness'] = 1

        if i_feedforward is not None:
            self._command['i_feedforward'] = i_feedforward
        else:
            self._command['i_feedforward'] = 0

        if i_des is not None and mode == self.MODE_CURRENT:
            self._command['i_feedforward'] = i_des

        if coeff_sensitivity_pos is not None:
            self._command['coeff_sensitivity_pos'] = max(0.0, min(1.0, coeff_sensitivity_pos))
        else:
            self._command['coeff_sensitivity_pos'] = self.params['guarded_contact']['sensitivity_default']['coeff_sensitivity_pos']


        if coeff_sensitivity_neg is not None:
            self._command['coeff_sensitivity_neg'] = max(0.0, min(1.0, coeff_sensitivity_neg))
        else:
            self._command['coeff_sensitivity_neg'] = self.params['guarded_contact']['sensitivity_default']['coeff_sensitivity_neg']

        #print('SETTTTT', self._command['coeff_sensitivity_pos'])
        self._dirty_command=True

# ######################## STEPPER #################################
class Stepper(StepperBase):
    """
    API to the Stretch Stepper Board
    """
    def __init__(self,usb, name=None,backend=None):
        StepperBase.__init__(self,usb,name,backend)
        # Order in descending order so more recent protocols/methods override less recent
        self._supported_protocols = {'p8': (Stepper_Protocol_P8,), 'p7': (Stepper_Protocol_P7,),}
    
    def _expand_protocol_methods(self, protocol_class):
        for attr_name, attr_value in protocol_class.__dict__.items():
            if callable(attr_value) and not attr_name.startswith("__"):
                setattr(self, attr_name, attr_value.__get__(self, Stepper))
                
    def startup(self):
        """Starts machinery required to interface with this device

        Returns
        -------
        bool
            whether the startup procedure succeeded
        """
        StepperBase.startup(self)
        if self.hw_valid:
            if self.board_info['protocol_version'] in self._supported_protocols:
                protocol_classes = self._supported_protocols[self.board_info['protocol_version']]
                for p in protocol_classes[::-1]:
                    self._expand_protocol_methods(p)
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
            self.enable_safety()
            self._dirty_gains = True
            self.pull_status()
            self.push_command()
        return self.hw_valid

class StepperBaseWaypointTrajStatus(TypedDict):
    state: str
    setpoint: Any
    segment_id: int

class StepperStatus(TypedDict):
    mode: int
    effort_ticks: int
    effort_pct: float
    current: float
    pos: float
    vel: float
    err: float
    diag: int
    timestamp: float
    debug: int
    guarded_event: int
    transport: Any
    pos_calibrated: int
    runstop_on: int
    near_pos_setpoint: int
    near_vel_setpoint: int
    is_moving: int
    is_moving_filtered: int
    is_drv_fault: int
    is_mg_accelerating: int
    is_mg_moving: int
    calibration_rcvd: int
    in_guarded_event: int
    in_safety_event: int
    waiting_on_sync: int
    in_sync_mode: int
    trace_on: int
    ctrl_cycle_cnt: int
    waypoint_traj: StepperBaseWaypointTrajStatus
    voltage: float
    temperature: float


