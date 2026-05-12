#!/usr/bin/env python3

import logging
import threading
# https://github.com/iotdesignshop/Feetech-tuna

from stretch4_body.core.feetech.sms_sts import *
from stretch4_body.core.feetech.port_handler import *


#-------EPROM--------
SMS_FIRMWARE_VERSION = 0
SMS_MODEL = 3
SMS_ID = 5
SMS_BAUD_RATE = 6
SMS_RETURN_DELAY_TIME = 7
SMS_STATUS_RETURN_LEVEL =8
SMS_MIN_POS_LIMIT = 9
SMS_MAX_POS_LIMIT = 11
SMS_MAX_TEMP_LIMIT = 13
SMS_MAX_INPUT_VOLTAGE=14
SMS_MIN_INPUT_VOLTAGE=15
SMS_MAX_LOAD_LIMIT=16
SMS_PHASE=18
SMS_PROTECTION_SWITCH=19
SMS_LED_ALARM = 20
SMS_POS_P_GAIN = 21
SMS_POS_I_GAIN = 22
SMS_POS_D_GAIN = 23
SMS_STARTUP_FORCE = 24
SMS_MAX_I = 25
SMS_CW_DEAD = 26
SMS_CCW_DEAD = 27
SMS_OVERCURRENT =28
SMS_ANGULAR_RES = 30
SMS_POS_OFFSET= 31
SMS_MODE = 33


"""
PROTECTIONS

https://www.feetechrc.com/Data/feetechrc/upload/file/20201127/start%20%20tutorial201015.pdf


Protections are set with the SMS_PROTECTION_SWITCH:
32: Overload enable
8: Overcurent enable
4: Overtemp enable
2: Sensor flag
1: Overvoltage enable

OVERCURRENT protection limits:  (SMS_OVERCURRENT) the maximum current that can be sent to the drivers for duration (SMS_OVERCURRENT_PROTECT) (s). The drivers will recognize the next command after an overcurrent.

OVERVOLTAGE protection limits: Disables driver if input voltage out of range (SMS_MAX_INPUT_VOLTAGE,  SMS_MIN_INPUT_VOLTAGE)

OVERTEMP protection limits: Disables driver if temp over SMS_MAX_TEMP_LIMIT

OVERLOAD protection limits: 
 Note: Documentation mixes 'torque' and 'load'. They mean PWM applied to the driver. We've renamed registers here to be easier to understand. We use 'load' throughout.

 On power-on, the load limit % (SMS_LOAD_LIMIT) is set to the max (SMS_MAX_LOAD_LIMIT). This limit (SMS_LOAD_LIMIT) can then be modifed at run-time as needed. 
 This allows the PWM applied to the motor to be capped every control cycle to a fixed value.

The overload function works by: When (SMS_PRESENT_LOAD) exceeds  a % (SMS_OVERLOAD_THRESH) of the Maximum Load (SMS_MAX_LOAD_LIMIT) for more than a time (SMS_OVERLOAD_TIME), 
 then protection mode is entered (and the load is limited to % (SMS_OVERLOAD_SAFE))



FROM the manual:
When the servo scs45 is blocked in motion，Unable to reach the target position，At thistime, the overload load (address: 39) 
monitors that the current load (address: 60) reaches80% of the load, and the protection time (address: 38) starts to count down 
(timeiscalculated according to the set value * unit 40ms). After the time is over, the protection load(address: 37) starts to turn on, 
and the rotation is blocked according to the set load(maximum load * set percentage, 0 is the free state). At this time, 
the load is reduced dueto the load When it is small, the current will not rise again until the next command 
(thecommand packet opposite to the locked rotor direction) is sent, and the servo returns tonormal.
"""

#Flags
SMS_PROTECTION_OVERLOAD_FLAG = 32
SMS_PROTECTION_CURRENT_FLAG = 8
SMS_PROTECTION_TEMP_FLAG = 4
SMS_PROTECTION_SENSOR_FLAG = 2
SMS_PROTECTION_VOLTAGE_FLAG = 1
SMS_PROTECTION_OFF_FLAG = 0

SMS_OVERLOAD_SAFE =34
SMS_OVERLOAD_TIME=35
SMS_OVERLOAD_THRESH=36
SMS_VEL_P_GAIN=37
SMS_OVERCURRENT_PROTECT=38
SMS_VEL_I_GAIN=39

#-------SRAM R/W--------
SMS_TORQUE_ENABLE = 40
SMS_GOAL_ACCEL = 41
SMS_GOAL_POS = 42
SMS_GOAL_PWM = 44
SMS_GOAL_VEL = 46
SMS_LOAD_LIMIT = 48
SMS_HELLO_ROBOT_FLAGS=50
SMS_HELLO_ROBOT_POS_OFFSET=52
FLAG_IS_CALIBRATED=0b00000001
SMS_LOCK = 55

#-------SRAM READ ONLY--------
SMS_PRESENT_POS = 56
SMS_PRESENT_VEL = 58
SMS_PRESENT_LOAD = 60
SMS_PRESENT_VOLTAGE = 62
SMS_PRESENT_TEMP = 63
SMS_SYNC_WRITE_FLAG=64
SMS_HARDWARE_ERROR_STATUS=65
SMS_MOVING_STATUS = 66
SMS_PRESENT_CURRENT = 69

#See "Feetech Servo FD Connection Information (230616).pdf"
servoRegs = {
    SMS_FIRMWARE_VERSION: { "name": "SMS_FIRMWARE_VERSION", "size": 2, "type": "uint16","bitlen":15},#EPROM
    SMS_MODEL: { "name": "SMS_MODEL", "size": 2, "type": "uint16", "bitlen":15},#EPROM
    SMS_ID: { "name": "SMS_ID", "size": 1, "type": "uint8"},#EPROM
    SMS_BAUD_RATE: { "name": "SMS_BAUD_RATE", "size": 1, "type": "uint8"},#EPROM
    SMS_RETURN_DELAY_TIME: { "name": "SMS_RETURN_DELAY_TIME", "size": 1, "type": "uint8"},#EPROM. Default 250
    SMS_STATUS_RETURN_LEVEL: {"name": "SMS_STATUS_RETURN_LEVEL", "size": 1, "type": "uint8"},
    SMS_MIN_POS_LIMIT:  { "name": "SMS_MIN_POS_LIMIT",  "size": 2, "type": "uint16","bitlen":15 },#EPROM
    SMS_MAX_POS_LIMIT: { "name": "SMS_MAX_POS_LIMIT", "size": 2, "type": "uint16" , "bitlen":15},#EPROM
    SMS_MAX_TEMP_LIMIT: { "name": "SMS_MAX_TEMP_LIMIT", "size": 1, "type": "uint8"},#EPROM
    SMS_MAX_INPUT_VOLTAGE: { "name": "SMS_MAX_INPUT_VOLTAGE", "size": 1, "type": "uint8"},#EPROM
    SMS_MIN_INPUT_VOLTAGE: { "name": "SMS_MIN_INPUT_VOLTAGE", "size": 1, "type": "uint8"},#EPROM
    SMS_MAX_LOAD_LIMIT: { "name": "SMS_MAX_LOAD_LIMIT", "size": 2, "type": "uint16", "bitlen":15},#EPROM, 1000 default
    SMS_PHASE: { "name": "SMS_PHASE", "size": 1, "type": "uint8"},#EPROM
    SMS_PROTECTION_SWITCH: { "name": "SMS_PROTECTION_SWITCH", "size": 1, "type": "uint8"},#EPROM
    SMS_LED_ALARM: { "name": "SMS_LED_ALARM", "size": 1, "type": "uint8"},#EPROM
    SMS_POS_P_GAIN: { "name": "SMS_POS_P_GAIN", "size": 1, "type": "uint8"},#EPROM
    SMS_POS_D_GAIN: { "name": "SMS_POS_D_GAIN", "size": 1, "type": "uint8"},#EPROM
    SMS_POS_I_GAIN: { "name": "SMS_POS_I_GAIN", "size": 1, "type": "uint8"},#EPROM
    SMS_STARTUP_FORCE: { "name": "SMS_STARTUP_FORCE", "size": 2, "type": "uint16", "bitlen":15},#EPROM
    SMS_MAX_I: { "name": "SMS_MAX_I",  "size": 1, "type": "uint8"},#EPROM
    SMS_CW_DEAD: { "name": "SMS_CW_DEAD",  "size": 1, "type": "uint8"},#EPROM
    SMS_CCW_DEAD: { "name": "SMS_CCW_DEAD",  "size": 1, "type": "uint8"},#EPROM
    SMS_OVERCURRENT: { "name": "SMS_OVERCURRENT", "size": 2, "type": "uint16", "bitlen":15},#EPROM
    SMS_ANGULAR_RES: { "name": "SMS_ANGULAR_RES", "size": 1, "type": "uint8"},#EPROM
    SMS_POS_OFFSET: { "name": "SMS_POS_OFFSET",  "size": 2, "type": "int16", "bitlen":15},#EPROM
    SMS_MODE: { "name": "SMS_MODE",  "size": 1, "type": "uint8"},#EPROM
    SMS_OVERLOAD_SAFE: { "name": "SMS_OVERLOAD_SAFE", "size": 1, "type": "uint8"},#EPROM
    SMS_OVERLOAD_TIME: { "name": "SMS_OVERLOAD_TIME", "size": 1, "type": "uint8"},#EPROM
    SMS_OVERLOAD_THRESH: { "name": "SMS_OVERLOAD_THRESH", "size": 1, "type": "uint8"},#EPROM
    SMS_VEL_P_GAIN: { "name": "SMS_VEL_P_GAIN",  "size": 1, "type": "uint8"},#EPROM
    SMS_OVERCURRENT_PROTECT: { "name": "SMS_OVERCURRENT_PROTECT", "size": 1, "type": "uint8"},#EPROM
    SMS_VEL_I_GAIN: { "name": "SMS_VEL_I_GAIN",  "size": 1, "type": "uint8"},#EPROM
    SMS_TORQUE_ENABLE: { "name": "SMS_TORQUE_ENABLE", "size": 1, "type": "uint8"},#SRAM
    SMS_GOAL_ACCEL: { "name": "SMS_GOAL_ACCEL",  "size": 1, "type": "uint8"},#SRAM
    SMS_GOAL_POS: { "name": "SMS_GOAL_POS",  "size": 2, "type": "uint16", "bitlen":15},#SRAM
    SMS_GOAL_PWM: { "name": "SMS_GOAL_PWM",  "size": 2, "type": "int16", "bitlen":15},#SRAM
    SMS_GOAL_VEL: { "name": "SMS_GOAL_VEL",  "size": 2, "type": "uint16", "bitlen":15},#SRAM
    SMS_LOAD_LIMIT: { "name": "SMS_LOAD_LIMIT",  "size": 2, "type": "uint16", "bitlen":15},#SRAM
    SMS_HELLO_ROBOT_FLAGS: { "name": "SMS_HELLO_ROBOT_FLAGS", "size": 1, "type": "uint8"},#SRAM
    SMS_HELLO_ROBOT_POS_OFFSET: { "name": "SMS_HELLO_ROBOT_POS_OFFSET", "size": 2, "type": "int16", "bitlen":15},#SRAM
    SMS_LOCK: { "name": "SMS_LOCK",  "size": 1, "type": "uint8" },#SRAM
    SMS_PRESENT_POS: { "name": "SMS_PRESENT_POS",  "size": 2, "type": "uint16" , "bitlen":15},#SRAM
    SMS_PRESENT_VEL: { "name": "SMS_PRESENT_VEL",  "size": 2, "type": "int16", "bitlen":15},#SRAM
    SMS_PRESENT_LOAD: { "name": "SMS_PRESENT_LOAD",  "size": 2, "type": "int16" , "bitlen":10},#SRAM
    SMS_PRESENT_VOLTAGE: { "name": "SMS_PRESENT_VOLTAGE",  "size": 1, "type": "uint8"},#SRAM
    SMS_PRESENT_TEMP: { "name": "SMS_PRESENT_TEMP",  "size": 1, "type": "uint8"},#SRAM
    SMS_SYNC_WRITE_FLAG: { "name": "SMS_SYNC_WRITE_FLAG",  "size": 1, "type": "uint8"},#SRAM
    SMS_HARDWARE_ERROR_STATUS: { "name": "SMS_HARDWARE_ERROR_STATUS",  "size": 1, "type": "uint8"},#SRAM
    SMS_MOVING_STATUS: { "name": "SMS_MOVING_STATUS",  "size": 1, "type": "uint8"},#SRAM
    SMS_PRESENT_CURRENT: { "name": "SMS_PRESENT_CURRENT",  "size": 2, "type": "uint16" , "bitlen":15}#SRAM
}

COMM_CODES = {
    COMM_SUCCESS: "COMM_SUCCESS",
    COMM_PORT_BUSY: "COMM_PORT_BUSY",
    COMM_TX_FAIL: "COMM_TX_FAIL",
    COMM_RX_FAIL: "COMM_RX_FAIL",
    COMM_TX_ERROR: "COMM_TX_ERROR",
    COMM_RX_WAITING: "COMM_RX_WAITING",
    COMM_RX_TIMEOUT: "COMM_RX_TIMEOUT",
    COMM_RX_CORRUPT: "COMM_RX_CORRUPT",
    COMM_NOT_AVAILABLE: "COMM_NOT_AVAILABLE"
}
BAUDRATES ={
 0: 1000000,
 1:500000,
 2:250000,
 3:128000,
 4:115200,
 5:76800,
 6:57600,
 7:38400}

class FeetechCommError(Exception):
    pass

class FeetechSMServo():
    """
    Wrapping of Feetech SM Series interface
    """

    def __init__(self, id, usb, port_handler=None, pt_lock=None, baud=1000000, logger=logging.getLogger()):
        self.id = id
        self.usb = usb
        self.comm_errors = 0
        self.last_comm_success = True
        self.logger = logger
        self.baud = baud
        self.dxl_model_name = ''
        # Make access to portHandler threadsafe
        self.pt_lock = threading.RLock() if pt_lock is None else pt_lock
        self.hw_valid = False
        # Allow sharing of port handler across multiple servos
        self.port_handler = port_handler
        self.packet_handler = None

    @staticmethod
    def identify_baud_rate(id, usb):
        """Identify the baud rate a servo is communicating at.
        Parameters
        ----------
        id : int
            ID on chain. Must be [0, 25]
        usb : str
            the USB port, typically "/dev/something"

        Returns
        -------
        int
            the baud rate the servo is communicating at
        """
        port_handler = PortHandler(usb)
        if port_handler.openPort():
            packet_handler = sms_sts(port_handler)
            print("Scanning baud rates on ID %d and bus %s. Please wait..."%(id,usb))
            for k in BAUDRATES:
                port_handler.setBaudRate(BAUDRATES[k])
                model_number, comm_result, error = packet_handler.ping(id)
                if comm_result == COMM_SUCCESS:
                    print('Found baudrate %d for servo %d'%(BAUDRATES[k],id))
                    return BAUDRATES[k]
        else:
            print('Unable to open port %s'%usb)
        print('Unable to identify baudrate on servo %d'%id)
        return 0

    @staticmethod
    def list_servos(usb, baudrate=115200):
        """Identify servos on the bus.
        Parameters
        ----------
        usb : str
            the USB port, typically "/dev/something"
        baudrate : int
            baudrate to scan at
        Returns
        -------
        dict
            id and model of found servos (if any)
        """
        result = []
        port_handler = PortHandler(usb)
        try:
            port_handler.openPort()
            port_handler.setBaudRate(baudrate)
            packet_handler = sms_sts(port_handler)
            print("Scanning servo bus. Please wait...")
            for id in range(0, 45):
                print('Testing %d'%id)
                model_number, comm_result, error = packet_handler.ping(id)
                if comm_result == COMM_SUCCESS:
                    print('Found id: %d model: %d'%(id,model_number))
                    result.append({"id": id, "model": model_number})
        except:
            print('Unable to open port %s'%usb)
        return result

    def create_port_handler(self):
        if self.port_handler is None:# or not self.port_handler.is_open:
            self.port_handler = PortHandler(self.usb)
        self.packet_handler = sms_sts(self.port_handler)
        if not self.port_handler.openPort():
            self.packet_handler=None
            self.port_handler = None
            self.hw_valid=False
            print("Failed to open the port %s" % self.usb)
            return False
        else:
            if (self.port_handler.setBaudRate(self.baud)):
                self.hw_valid=True
                return True
            else:
                self.packet_handler = None
                self.port_handler = None
                self.hw_valid = False
                print("Failed to set baudrate %d on the port %s" % (self.baud,self.usb))
                return False


    def startup(self):
        self.create_port_handler()
        if self.hw_valid:
            try:
                self.enable_torque()
            except FeetechCommError:
                baud=self.identify_baud_rate(self.id,self.usb)
                if baud!=self.baud:
                    self.logger.error('FeetechCommError. Mismatched baud rate. Expected %d but servo is set to %d.'%(self.baud,baud))
                else:
                    self.logger.error('FeetechCommError. Failed to startup servo %s at id %d . Check that id and usb bus are valid'%(self.usb,self.id))
                self.hw_valid=False
                return False
            return True
        return False


    def stop(self, close_port=True, disable_torque=False):
        if self.hw_valid:
            self.hw_valid = False
            # if disable_torque:
            #     self.disable_torque()
            if close_port:
                self.port_handler.closePort()

    # ###############################################

    def read_reg(self, regAddr):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            # with DelayedKeyboardInterrupt():
            value, comm_result, error = self.packet_handler.readTxRx(self.id, regAddr, servoRegs[regAddr]["size"])
            if comm_result == COMM_SUCCESS:
                if servoRegs[regAddr]["size"] == 2:
                    value = self.packet_handler.scs_tohost(self.packet_handler.scs_makeword(value[0], value[1]), servoRegs[regAddr]["bitlen"])
                else:
                    value = value[0]
        self.handle_comm_result(servoRegs[regAddr]["name"], comm_result, error)
        return value

    def write_reg(self, regAddr, value):
        if not self.hw_valid:
            return False
        if servoRegs[regAddr]["size"] == 2:
            value = self.packet_handler.scs_toscs(value,servoRegs[regAddr]["bitlen"])
            value = [self.packet_handler.scs_lobyte(int(value)), self.packet_handler.scs_hibyte(int(value))]
        else:
            value = [int(value)]
        retries = 3
        while retries > 0:
            with self.pt_lock:
                comm_result, error = self.packet_handler.writeTxRx(self.id, regAddr, servoRegs[regAddr]["size"], value)
                self.handle_comm_result(servoRegs[regAddr]["name"], comm_result, error)
            if self.last_comm_success:
                return True
            else:
                retries -= 1
        print("Feetech failed to write register: %s - giving up", servoRegs[regAddr]["name"])
        return False

    # ###############################################
    def do_ping(self,verbose=True):
        if not self.hw_valid:
            return False
        with self.pt_lock:
            scs_model_number, scs_comm_result, scs_error = self.packet_handler.ping(self.id)
        if scs_comm_result != COMM_SUCCESS:
            if verbose:
                print("%s" % self.packet_handler.getTxRxResult(scs_comm_result))
            return False
        else:
            if verbose:
                print("[ID:%03d] ping Succeeded. SCServo model number : %d" % (self.id, scs_model_number))
        if scs_error != 0:
            if verbose:
                print("%s" % self.packet_handler.getRxPacketError(scs_error))
            #return False
        return True
    # ###############################################
    def pretty_print(self):
        h = self.get_hardware_error()

        status = {
            'Firmware':self.get_firmware_version(),
            'ID:': self.get_id(),
            'Model': self.get_model(),
            'Baudrate': str(BAUDRATES[self.get_baudrate()]),
            'Min pos limit': self.get_min_pos_limit(),
            'Max pos limit': self.get_max_pos_limit(),
            'Temperature limit': self.get_temp_limit(),
            'Max voltage limit': self.get_max_input_voltage()/10.0,
            'Min voltage limit': self.get_min_input_voltage()/10.0,
            'Load limit (%)':f'{self.get_load_limit_pct() :.3f} %',
            'Operating Mode:': self.get_mode(),
            'Temperature:': f'{self.get_temp()} °C',
            'Position:': f'{self.get_pos()} ticks',
            'Velocity:': f'{self.get_vel():.3f} ticks/s',
            'Load :': f'{self.get_load_pct() :.3f} %',
            'Voltage': f'{self.get_voltage()/10.0:.3f} V',
            'Current':f'{self.get_current_mA():.3f} mA',
            'Goal accel': self.get_goal_accel(),
            'Goal vel': self.get_goal_vel(),
            'Goal pwm': self.get_goal_pwm(),
            'Is Moving:': str(self.is_moving() != 0),
            'Is Calibrated:': str(self.get_is_calibrated() != 0),
            'Hello homing position':f'{self.get_hello_robot_pos_offset()} ticks',
            #'Profile Velocity:': f'{self.get_profile_velocity() * 0.299:.3f} rev/min',
            #'Profile Acceleration:': f'{self.get_profile_acceleration() * 214.577:.3f} rev/min^2',
            'Hardware Error Status:': format(h, '#010b'),
            '  Input Voltage Error:': str(h & ERRBIT_VOLTAGE != 0),
            '  Over Temp Error: ': str(h & ERRBIT_OVERHEAT != 0),
            '  Motor Encoder Error:': str(h & ERRBIT_ANGLE!= 0),
            '  Over Current Error:': str(h & ERRBIT_OVERELE != 0),
            '  Over Load Error:': str(h & ERRBIT_OVERLOAD != 0),
            '  Communication Errors:': self.comm_errors}
        status2={
            'Phase': self.get_phase(),
            'Protection switch': self.get_protection_switch(),
            'Return delay': self.get_return_delay(),
            'Status return level': self.get_status_return_level(),
            'Led alarm': self.get_led_alarm(),
            'Pos P Gain': self.get_pos_p_gain(),
            'Pos I Gain': self.get_pos_i_gain(),
            'Pos D Gain': self.get_pos_d_gain(),
            'Startup force': self.get_startup_force(),
            'Max I': self.get_max_i(),
            'CW deadzone': self.get_cw_dead(),
            'CCW deadzone': self.get_ccw_dead(),
            'Angular resolution': self.get_angular_res(),
            'Position offset': self.get_pos_offset(),
            'Overload safe (%)': self.get_overload_safe(),
            'Overload time (ms)': self.get_overload_time_ms(),
            'Overload thresh (%)': self.get_overload_thresh(),
            'Overcurrent (mA)': self.get_overcurrent_mA(),
            'Overcurrent time (ms)': self.get_overcurrent_time_ms(),
            'Vel P Gain': self.get_vel_p_gain(),
            'Vel I Gain': self.get_vel_i_gain(),
            'Load enable': self.get_torque_enable(),
            'Lock': self.get_lock(),
            'Sync write flag': self.get_sync_write_flag()}

        print('------------------- FeetechSMServo -------------------')
        for elem, value in status.items():
            print(f"{elem: <25}{value: >20}")
        print('         ------       ')
        for elem, value in status2.items():
            print(f"{elem: <25}{value: >20}")
    # ###############################################
    def get_firmware_version(self):
        return self.read_reg(SMS_FIRMWARE_VERSION)
    # ###############################################
    def get_model(self):
        return self.read_reg(SMS_MODEL)
    # ###############################################
    #The unique ID number on the bus. No duplicate ID number can appear on the same bus.
    # No. 254 (OxFE) is the broadcast ID, and the broadcast does not return the reply package.
    def get_id(self):
        return self.read_reg(SMS_ID)
    def set_id(self,v):
        self.write_reg(SMS_ID,max(0,min(254,int(v))))
    # ###############################################
    # "0-7 represents the baud rate as follows:
    # 1000000，500000，250000，128000，115200，76800，57600，38400"
    def get_baudrate(self):
        return self.read_reg(SMS_BAUD_RATE)
    def set_baudrate(self,b):
        res=[key for key, value in BAUDRATES.items() if value == b]
        if len(res)==1:
            return self.write_reg(SMS_BAUD_RATE,  res[0])
        else:
            print('Invalid baudrate in set_baudrate:',b)
        return 0

    # ###############################################
    # USE REG SMS_HELLO_ROBOT_FLAGS to hold temporary data in SRAM
    def get_is_calibrated(self):
        return self.get_hello_robot_flags()&FLAG_IS_CALIBRATED

    def set_is_calibrated(self,v):
        if v:
            self.set_hello_robot_flags(self.get_hello_robot_flags()|FLAG_IS_CALIBRATED)
        else:
            self.set_hello_robot_flags(self.get_hello_robot_flags()&~FLAG_IS_CALIBRATED)

    def set_hello_robot_flags(self,v):
        self.write_reg(SMS_HELLO_ROBOT_FLAGS, max(0, min(255, int(v))))

    def get_hello_robot_flags(self):
        return self.read_reg(SMS_HELLO_ROBOT_FLAGS)

    def set_hello_robot_pos_offset(self,v):
        self.write_reg(SMS_HELLO_ROBOT_POS_OFFSET, int(v))
    def get_hello_robot_pos_offset(self):
        return self.read_reg(SMS_HELLO_ROBOT_POS_OFFSET)
    # ###############################################
    # minimum unit is 2us, maximum settable return delay 254*2=508us
    def get_return_delay(self):
        return self.read_reg(SMS_RETURN_DELAY_TIME)
    def set_return_delay(self,v):
        self.write_reg(SMS_RETURN_DELAY_TIME,  max(0,min(254,int(v))))
    # ###############################################
    #"0: Instructions other than read instructions
    # and PING instructions do not return reply packages
    # Return reply packages for all instructions"
    def get_status_return_level(self):
        return self.read_reg(SMS_STATUS_RETURN_LEVEL)
    def set_status_return_level(self,v):
        self.write_reg(SMS_STATUS_RETURN_LEVEL,max(0,min(1,int(v))))
    # ###############################################
    # Set the minimum limit of motion travel,
    # which is less than the maximum angle limit.
    # This value is 0 in multi-circle absolute position control.
    def get_min_pos_limit(self):
        return self.read_reg(SMS_MIN_POS_LIMIT)
    def set_min_pos_limit(self,v):
        self.write_reg(SMS_MIN_POS_LIMIT,max(0,min(4094,int(v))))
    # ###############################################
    # Set the maximum limit of the travel distance, the value is greater than the
    # minimum angle limit, and the value is 0 in multi-loop
    # absolute position control.
    def get_max_pos_limit(self):
        return self.read_reg(SMS_MAX_POS_LIMIT)
    def set_max_pos_limit(self,v):
        self.write_reg(SMS_MAX_POS_LIMIT,max(0,min(4095,int(v))))
    # ###############################################
    # Maximum operating temperature limit,
    # if set to 70, maximum temperature to 70 degrees Celsius,
    # set accuracy to 1 degrees Celsius
    def get_temp_limit(self):
        return self.read_reg(SMS_MAX_TEMP_LIMIT)
    def set_temp_limit(self,v):
        self.write_reg(SMS_MAX_TEMP_LIMIT, max(0,min(100,int(v))))
    # ###############################################
    # If the maximum input voltage is set to 140,
    # the maximum operating voltage is limited to 14.0V
    # and the setting accuracy is 0.1V.
    def get_max_input_voltage(self):
        return self.read_reg(SMS_MAX_INPUT_VOLTAGE)
    def set_max_input_voltage(self,v):
        self.write_reg(SMS_MAX_INPUT_VOLTAGE, min(max(0,int(v)),254))
    # ###############################################
    # If the minimum input voltage is set to 90,
    # the minimum operating voltage is limited to 9.0V
    # and the setting accuracy is 0.1V.
    def get_min_input_voltage(self):
        return self.read_reg(SMS_MIN_INPUT_VOLTAGE)
    def set_min_input_voltage(self,v):
        self.write_reg(SMS_MIN_INPUT_VOLTAGE, min(max(0,int(v)),254))


    # ###############################################
    #Not clear how this param works.
    #Set to 124 enables multi-turn encoder (per support email)
    #Nominal from factory is 45
    def get_phase(self):
        return self.read_reg(SMS_PHASE)
    def set_phase(self,v):
        self.write_reg(SMS_PHASE,v)
    # ###############################################
    # "Bit0  Bit1  Bit2 Bit3 Bit4 Bit5 Corresponding bit 1 is
    # set to enable corresponding protection
    # Voltage Sensor Temperature and
    # Current Angle Overload Corresponding Position Set 0
    # to Close Corresponding Protection"
    def get_protection_switch(self):
        return self.read_reg(SMS_PROTECTION_SWITCH)
    def set_protection_switch(self,v):
        self.write_reg(SMS_PROTECTION_SWITCH, min(max(0,int(v)),254))
    # ###############################################
    # "Bit0  Bit1  Bit2 Bit3 Bit4 Bit5
    # Set the corresponding position 1 to turn on the flash alarm
    # Voltage sensor temperature and current angle overload
    # corresponding position set 0 to turn off flashlight alarm"
    def get_led_alarm(self):
        return self.read_reg(SMS_LED_ALARM)
    def set_led_alarm(self, v):
        self.write_reg(SMS_LED_ALARM, min(max(0,int(v)),254))
    # ###############################################
    def get_pos_p_gain(self):
        return self.read_reg(SMS_POS_P_GAIN)
    def set_pos_p_gain(self, v):
        self.write_reg(SMS_POS_P_GAIN, min(max(0,int(v)),254))
    # ###############################################
    def get_pos_i_gain(self):
        return self.read_reg(SMS_POS_I_GAIN)
    def set_pos_i_gain(self, v):
        self.write_reg(SMS_POS_I_GAIN, min(max(0,int(v)),254))
    # ###############################################
    def get_pos_d_gain(self):
        return self.read_reg(SMS_POS_D_GAIN)
    def set_pos_d_gain(self, v):
        self.write_reg(SMS_POS_D_GAIN, min(max(0,int(v)),254))
    # ###############################################
    # Setting the servo Minimum Output Starting Load，set1000 = 100% * stall load
    def get_startup_force(self):
        return self.read_reg(SMS_STARTUP_FORCE)
    def set_startup_force(self, v):
        self.write_reg(SMS_STARTUP_FORCE, min(max(0,int(v)),1000))
    # ###############################################
    #Not clear what this is
    def get_max_i(self):
        return self.read_reg(SMS_MAX_I)
    def set_max_i(self, v):
        self.write_reg(SMS_MAX_I, v)
    # ###############################################
    # The minimum unit is a minimum resolution angle.
    def get_cw_dead(self):
        return self.read_reg(SMS_CW_DEAD)
    def set_cw_dead(self, v):
        self.write_reg(SMS_CW_DEAD, min(max(0,int(v)),32))
    # ###############################################
    # The minimum unit is a minimum resolution angle.
    def get_ccw_dead(self):
        return self.read_reg(SMS_CCW_DEAD)
    def set_ccw_dead(self, v):
        self.write_reg(SMS_CCW_DEAD, min(max(0,int(v)),32))

    # ###############################################
    # For the amplification factor of the minimum resolution angle
    # (degree/step) of the sensor,
    # the number of control cycles can be
    # extended by modifying this value.
    def get_angular_res(self):
        return self.read_reg(SMS_ANGULAR_RES)
    def set_angular_res(self, v):
        self.write_reg(SMS_ANGULAR_RES, min(max(1,int(v)),100))
    # ###############################################
    # BIT11 is the directional bit, indicating the positive and negative direction,
    # and other bits can be expressed in the range of 0-2047 steps.
    def get_pos_offset(self):
        v=self.read_reg(SMS_POS_OFFSET)
        if v&0b100000000000:
            return -1*(v&0b011111111111)
        return v
    def set_pos_offset(self, v):
        v=max(-2047,min(2047,v))
        if v<0:
            v=abs(v)+0b100000000000
        self.write_reg(SMS_POS_OFFSET, v)

    # ###############################################
    # Operating mode 1 byte, address 0x21
    # Used to switch the servo working mode: 0 is the position servo mode; 1 is the motor
    # constant speed mode; In motor mode, the operating speed at address 46 is used tocontrol electricity
    # Machine speed, BIT15 is the direction position (some servos have 2 mode, switch
    # motor mode, controlled with address 44; 3-mode stepper motor mode).
    #Note that when switching to PWM mode the position looses its multi-turn count history
    #And the system returns to single turn encoder pos
    def get_mode(self):
        return self.read_reg(SMS_MODE)
    def set_mode(self, v):
        self.write_reg(SMS_MODE, min(max(0,int(v)),2))
    def get_operating_mode(self):
        return self.get_mode()
    def enable_pos(self):
        self.set_mode(0)
    def enable_vel(self):
        self.set_mode(1)
    def enable_pwm(self):
        self.set_mode(2)

    # ###############################################
    # "Set the servo maximum output load limit
    # and set 1000 = 100% *blocking load.
    # Power-on assignment to address 48 Load Limitation"
    def get_max_load_limit_pct(self):
        return self.get_max_load_limit()/10.0
    def set_max_load_limit_pct(self,v):
        self.set_max_load_limit(v*10)
    def get_max_load_limit(self):
        return self.read_reg(SMS_MAX_LOAD_LIMIT)
    def set_max_load_limit(self,v):
        self.write_reg(SMS_MAX_LOAD_LIMIT,max(0,min(1000,int(v))))
    # ###############################################
    #The initial power-on value is assigned by the maximum load (0x10).
    # Users can modify this value to control the output of the maximum load.
    # This effectively limits the PWM to the driver during control
    def get_load_limit_pct(self):
        return self.get_load_limit()/10.0
    def set_load_limit_pct(self, v):
        return self.set_load_limit(v*10)
    def get_load_limit(self):
        return self.read_reg(SMS_LOAD_LIMIT)
    def set_load_limit(self, v):
        self.write_reg(SMS_LOAD_LIMIT, min(1000,max(0,int(v))))

    # ###############################################
    # Output load after entering overload protection,
    # e.g. set 20 to represent 20% maximum load
    def get_overload_safe(self):
        return self.read_reg(SMS_OVERLOAD_SAFE)
    def set_overload_safe(self, v):
        self.write_reg(SMS_OVERLOAD_SAFE, min(max(0,int(v)),100)) #data sheet says 254, not 100

    # ###############################################
    # Load that triggers an overload event
    def get_overload_thresh(self):
        return self.read_reg(SMS_OVERLOAD_THRESH)
    def set_overload_thresh(self, v):
        self.write_reg(SMS_OVERLOAD_THRESH, min(max(0,int(v)),254))

    # ###############################################
    # The current load output exceeds the overload load
    # and maintains the timing time,
    # such as 200 for 2 seconds, the maximum can be set to 2.5 seconds.

    def get_overload_time_ms(self):
        return self.get_overload_time()*10.0
    def set_overload_time_ms(self, v):
        self.set_overload_time(v/10.0)

    def get_overload_time(self):
        return self.read_reg(SMS_OVERLOAD_TIME)
    def set_overload_time(self, v):
        self.write_reg(SMS_OVERLOAD_TIME, min(max(0,int(v)),254))

    # ###############################################
    # Proportional Coefficient of Speed Loop in Constant Speed Mode (Mode 1)
    def get_vel_p_gain(self):
        return self.read_reg(SMS_VEL_P_GAIN)
    def set_vel_p_gain(self, v):
        self.write_reg(SMS_VEL_P_GAIN,min(max(0,int(v)),254))
    # ###############################################
    #Turn off driver (until next command) if over current for more than designated time
    # Maximum settable current is 500 * 6.5mA= 3250mA
    def get_overcurrent_mA(self):
        return self.get_overcurrent()*6.5
    def set_overcurrent_mA(self,v):
        return self.set_overcurrent(v/6.5)
    def get_overcurrent(self):
        return self.read_reg(SMS_OVERCURRENT)
    def set_overcurrent(self, v):
        self.write_reg(SMS_OVERCURRENT, min(max(0,int(v)),511))

    # Maximum settable 254 * 10ms = 2540ms
    def get_overcurrent_time_ms(self):
        return self.get_overcurrent_time()*10
    def set_overcurrent_time_ms(self,t):
        return self.set_overcurrent_time(t/10.0)
    def get_overcurrent_time(self):
        return self.read_reg(SMS_OVERCURRENT_PROTECT)
    def set_overcurrent_time(self, v):
        self.write_reg(SMS_OVERCURRENT_PROTECT, min(max(0,int(v)),254))

    # ###############################################
    # Integral Coefficient of Speed Loop in Constant Speed Mode of Motor (Mode 1)
    def get_vel_i_gain(self):
        return self.read_reg(SMS_VEL_I_GAIN)
    def set_vel_i_gain(self, v):
        self.write_reg(SMS_VEL_I_GAIN, min(max(0,int(v)),254))
    # ###############################################
    # Write 0: Turn off the torque output;
    # Write 1: Turn on torque output;
    # Write 128: The current position (56) is positive to 2048, and the load switch is automatically set to 0
    def get_torque_enable(self):
        return self.read_reg(SMS_TORQUE_ENABLE)
    def set_torque_enable(self, v):
        self.write_reg(SMS_TORQUE_ENABLE,min(max(0,int(v)),2))
    def enable_torque(self):
        self.set_torque_enable(1)
    def disable_torque(self):
        self.set_torque_enable(0)
    def reset_torque_pos(self):
        self.set_torque_enable(128)
    # ###############################################
    # Acceleration 1 byte/address 0x29. The unit is 100 steps/second^2, and each
    # step is the minimum position resolution
    # accuracy 360/4096=0.088°. 100 steps/sec^2 is equivalent to 100 *
    # 360/4096=8.789 degrees/s^2 If the value is set to 10, the speed starts
    # from 0, and the speed will become 1000 steps per second after 1 second, and the
    # acceleration will not increase after reaching the running speed (46, and set this value to move smoothly)
    def get_goal_accel(self):
        return self.read_reg(SMS_GOAL_ACCEL)
    def set_goal_accel(self, v):
        self.write_reg(SMS_GOAL_ACCEL, min(254, max(0, int(v))))
    def set_profile_acceleration(self, v):
        self.set_goal_accel(v)
    # ###############################################
    # Each step is a minimum resolution angle,
    # absolute position control mode,
    # maximum corresponding maximum effective angle.
    def get_goal_pos(self):
        v= self.read_reg(SMS_GOAL_POS)
        if v&0b1000000000000000:
            return -1*(v&0b0111111111111111)
    def set_goal_pos(self, v):
        v=min(32767, max(-32767, int(v)))
        #NOTE: Scratch this, handled by read/write reg
        if v<0:
            v=abs(v)+32768
        self.write_reg(SMS_GOAL_POS, v)
    def go_to_pos(self,x):
        self.set_goal_pos(int(x))
    # ###############################################
    #Range not documented, 2 bytes
    def get_goal_pwm(self):
        #BIT10 is the direction bit
        v= self.read_reg(SMS_GOAL_PWM)
        #NOTE: Scratch this, handled by read/write reg
        if v&0b10000000000:
            return -1*(v&0b01111111111)
        return v

        return self.read_reg(SMS_GOAL_PWM)
    def set_goal_pwm(self, v):
        # It is effective in PWM open-loop speed regulation mode, and BIT10 is the direction bit
        v=int(max(-1023,min(1023,v)))
        #NOTE: Scratch this, handled by read/write reg
        if v<0:
            v=abs(v)^0b10000000000
        self.write_reg(SMS_GOAL_PWM, v)
    # ###############################################
    # Operating speed 2 bytes/address 0x2E lowbytes/address 0x2F high bytes
    # The number of steps moved per unit time (per second), with BIT15 as the direction bit
    # representing positive and negative directions
    # Speed units can be selected from the following:
    # Unit 1: 50 steps/sec = 0.732
    # RPM (default)
    # Unit 2: steps/sec (additional configuration
    # required) Each step is the minimum position
    # resolution accuracy of 360/4096=0.088 ° The response speed depends on the
    # maximum speed of the servo body, such as SM40BL at 12V operating voltage, no-
    # load 65RPM / 0.732RPM, operating speed can respond up to 88, setting more than
    # this value will respond to lag
    #NOTE documentation range of 255 incorrect. Need to check with vendor. May be 1 steps/sec.
    def get_goal_vel(self):
        return self.read_reg(SMS_GOAL_VEL)
    def set_goal_vel(self, v):
        v=min(10000, max(-10000, int(v)))
        #NOTE: Scratch this, handled by read/write reg
        if v<0:
             v=abs(v)^0b1000000000000000
        self.write_reg(SMS_GOAL_VEL, v)
    def set_vel(self,v):
        v=min(32767,max(-32767,v))
    def set_profile_velocity(self, v):
        self.set_goal_vel(v)
    # ###############################################
    # "Write 0 closes the write lock,
    # and the value written to the EPROM address is power-down and saved
    # Write 1 opens the write lock, and the value written to the
    # EPROM address is power-down and not saved"
    def get_lock(self):
        return self.read_reg(SMS_LOCK)
    def set_lock(self, v):
        self.write_reg(SMS_LOCK, min(1,max(0,int(v))))

    def lock_eeprom(self):
        return self.set_lock(1)

    def unlock_eeprom(self):
        return self.set_lock(0)
    # ###############################################
    # Feedback the number of steps of the current position,
    # each step is a minimum resolution angle; absolute position control mode,
    # the maximum corresponding to the maximum effective angle
    def get_pos(self):
        return self.read_reg(SMS_PRESENT_POS)
    # ###############################################
    # Feedback of the current motor speed,
    # the number of steps in a unit time (per second)
    # Doc says unit 50 step/s but seems to be ticks/sec (4096 = 360 deg/s)
    def get_vel(self):
        return self.read_reg(SMS_PRESENT_VEL)
    # ###############################################
    # The Voltage duty cycle of the current control output drive motor
    def get_load_pct(self):
        return self.get_load()/10.0
    def get_load(self):
        v = self.read_reg(SMS_PRESENT_LOAD)
        #NOTE: Scratch this, handled by read/write reg
        if v & 0b10000000000:
            return -1 * (v & 0b01111111111)
        return v
    # ###############################################
    # The servo Current operating voltage
    def get_voltage(self):
        return self.read_reg(SMS_PRESENT_VOLTAGE)
    # ###############################################
    # the servo Current internal operating temperature
    def get_temp(self):
        return self.read_reg(SMS_PRESENT_TEMP)
    # ###############################################
    # When writing instructions asynchronously, mark bit
    def get_sync_write_flag(self):
        return self.read_reg(SMS_SYNC_WRITE_FLAG)
    # ###############################################
    def get_hardware_error(self):
        # "Bit0  Bit1  Bit2 Bit3 Bit4 Bit5 Corresponding
        # position 1 indicates the occurrence of corresponding errors
        # There is no corresponding error when the corresponding bit 0
        # of temperature and current angle overload of voltage sensor is zero."
        return self.read_reg(SMS_HARDWARE_ERROR_STATUS)
    # ###############################################
    # The servo is marked 1 when moving and 0 when stops.
    def get_moving_status(self):
        return self.read_reg(SMS_MOVING_STATUS)
    def is_moving(self):
        #Doesn't work in PWM mode
        return self.get_moving_status()
    # ###############################################
    # The maximum measurable current is 500 * 6.5mA= 3250mA
    def get_current_mA(self):
        return self.get_current()*6.5
    def get_current(self):
        return self.read_reg(SMS_PRESENT_CURRENT)
    # ###############################################

    def list_regs(self, servoId):
        result = []
        for addr in servoRegs:
            try:
                with self.pt_lock:
                    value, comm_result, error = self.packet_handler.readTxRx(servoId, addr, servoRegs[addr]["size"])
                    if comm_result == COMM_SUCCESS:
                        if (servoRegs[addr]["size"] == 2):
                            value = self.packet_handler.scs_tohost(self.packet_handler.scs_makeword(value[0], value[1]), 15)
                        else:
                            value = value[0]
                        result.append({ "name": servoRegs[addr]["name"], "addr" : addr, "value": value })
                    else:
                        print("Failed to read register " + servoRegs[addr]["name"])
                        print("Comm result: " + self.packet_handler.getTxRxResult(comm_result))
            except:
                print("Warning: Error occurred when reading register " + servoRegs[addr]["name"] + " (addr: " + str(servoRegs[addr]["name"]) + ")")
        return result

    def handle_comm_result(self, fx, comm_result, error):
        """Handles comm result and tracks comm errors.

        Parameters
        ----------
        fx : str
            control table address label
        comm_result : int
            communication result from options `COMM_CODES`
        error : int
            hardware errors sent by the servo

        Returns
        -------
        bool
            True if successful result, False otherwise
        """
        if comm_result==COMM_SUCCESS:
            self.last_comm_success=True
            return True

        self.last_comm_success = False
        self.comm_errors += 1
        comm_error_msg = f'Feetech Comm Error on {self.usb} ID {self.id}. Attempted {fx}. Result {COMM_CODES[comm_result]}. Error {error}. Total Errors {self.comm_errors}.'
        self.logger.debug(comm_error_msg)
        raise FeetechCommError(comm_error_msg)

    def unlockEEPROM(self, servoId):
        with self.pt_lock:
            self.packet_handler.unLockEprom(servoId)
        print("EEPROM unlocked")

    def lockEEPROM(self, servoId):
        with self.pt_lock:
            self.packet_handler.LockEprom(servoId)
        print("EEPROM locked")





if __name__ == "__main__":
    s=FeetechSMServo(id=20,usb='/dev/hello-feetech-wrist',baud=115200)
    s.startup()
    s.do_ping()
    s.unlock_eeprom() #1
    #print(s.list_regs(2))
    s.pretty_print()
    # s.disable_torque()
    # s.enable_pwm()
    # s.enable_torque()
    # s.set_profile_velocity(100)
    # time.sleep(3.0)a


    # s.enable_torque()
    # print("Load enabled",s.get_torque_enable())
    # s.enable_pos()
    # print('Mode',s.get_mode())
    # s.set_profile_acceleration(100)
    # s.set_profile_velocity(25)
    # for i in range(2):
    #     s.go_to_pos(1000)
    #     time.sleep(2.0)
    #     s.go_to_pos(0)
    #     time.sleep(2.0)
    #     print('Pos', s.get_pos())
    # s.enable_torque()
    # print("Load enabled",s.get_torque_enable())
    # s.reset_load_pos()
    # print("Load enabled",s.get_torque_enable())
    # print('Pos', s.get_pos())
    # s.disable_torque()
    # s.set_vel(1.0)
    # time.sleep(2.0)

    s.stop()