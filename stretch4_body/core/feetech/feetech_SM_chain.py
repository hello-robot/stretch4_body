
from typing import TypedDict

from stretch4_body.core.device import Device
from stretch4_body.core.feetech.feetech_SM_servo import *
from stretch4_body.core.feetech.feetech_SM_hello import FeetechCommErrorStats, FeetechCommErrorStatsStatus
import time
import importlib
import struct
import array as arr

SMS_END=0
def SMS_MAKEWORD(a, b):
    global SMS_END
    if SMS_END==0:
        return (a & 0xFF) | ((b & 0xFF) << 8)
    else:
        return (b & 0xFF) | ((a & 0xFF) << 8)


def SMS_MAKEDWORD(a, b):
    return (a & 0xFFFF) | (b & 0xFFFF) << 16


def SMS_LOWORD(l):
    return l & 0xFFFF


def SMS_HIWORD(l):
    return (l >> 16) & 0xFFFF


def SMS_LOBYTE(w):
    global SMS_END
    if SMS_END==0:
        return w & 0xFF
    else:
        return (w >> 8) & 0xFF


def SMS_HIBYTE(w):
    global SMS_END
    if SMS_END==0:
        return (w >> 8) & 0xFF
    else:
        return w & 0xFF
    
class FeetechSMChain(Device):
    """
    This class manages a daisy chain of Feetech SM Series servos
    It allows adding more than one servo at run time
    It allows manage group reading of status data from servos so as to not overload the control bus
    """
    def __init__(self, usb, name,params=None):
        Device.__init__(self, name)
        if params is not None:
            self.params.update(params)
        self.usb = usb
        self.pt_lock = threading.RLock()
        self.thread_rate_hz = 15.0

        self.packet_handler = None
        self.port_handler = None
        self.hw_valid = False

        self.status: "FeetechSMChainStatus" = {}
        self.motors = {}
        self.readers={}
        self.writers={}
        self.comm_errors = FeetechCommErrorStats(name, logger=self.logger)
        self.status['comm_errors']=self.comm_errors.status
        self.status_mux_id = 0


    def add_motor(self,m):
        self.motors[m.name]=m

    def get_motor(self,motor_name):
        try:
            return self.motors[motor_name]
        except (AttributeError, KeyError):
            return None


    def create_port_handler(self):
        try:
            self.port_handler = PortHandler(self.usb)
            if self.port_handler.openPort():
                self.packet_handler = sms_sts(self.port_handler)
                self.hw_valid = True
            else:
                self.logger.error("FeetechSMChain: Failed to open port %s"%self.usb)
                self.packet_handler = None
                self.port_handler = None
                self.hw_valid = False
        except serial.SerialException as e:
            self.logger.error("Feetech SerialException({1}): {2}".format(self.usb, e.errno, e.strerror))
            self.packet_handler = None
            self.port_handler = None
            self.hw_valid = False
        self.hw_valid = self.packet_handler is not None
        if self.hw_valid:
            self.port_handler.setBaudRate(int(self.params['baud']))
        return self.hw_valid


    def startup(self):
        self.create_port_handler()

        self.joints = list(self.params.get('devices', {}).keys())
        self.status['joint_names']=self.joints
        for j in self.joints:
            module_name = self.params['devices'][j]['py_module_name']
            class_name = self.params['devices'][j]['py_class_name']
            servo_device = getattr(importlib.import_module(module_name), class_name)(chain=self)
            self.add_motor(servo_device)
        for mk in self.motors.keys():  # Provide nop data in case comm failures
            self.status[mk] = self.motors[mk].status
        
        if not self.hw_valid:
            print("HW Not Valid")
            return False
        if len(self.motors.keys()):
            try:
                if self.params['use_group_sync_read']:
                    # Initialize GroupSyncRead instace for Present Position
                    self.readers['pos'] = GroupSyncRead(self.packet_handler, SMS_PRESENT_POS, 2)
                    self.readers['current'] = GroupSyncRead(self.packet_handler, SMS_PRESENT_CURRENT, 2)
                    self.readers['vel'] = GroupSyncRead(self.packet_handler,  SMS_PRESENT_VEL, 2)
                    self.readers['temp'] = GroupSyncRead(self.packet_handler,  SMS_PRESENT_TEMP, 1)
                    self.readers['hardware_error'] = GroupSyncRead(self.packet_handler,  SMS_HARDWARE_ERROR_STATUS, 1)
                    for mk in self.motors.keys():
                        for k in self.readers.keys():
                            if not self.readers[k].addParam(self.motors[mk].motor.id):
                                self.logger.error('FeetechSMChain sync read addParam failed ID %d.'%self.motors[mk].motor.id)
                                raise FeetechCommError
                
                if self.params['use_group_sync_write']:
                    self.writers['vel_stream'] = GroupSyncWrite(self.packet_handler, SMS_GOAL_VEL, 2)
                    self.writers['pos_stream'] = GroupSyncWrite(self.packet_handler, SMS_GOAL_POS, 2)
                    for mk in self.motors.keys():
                        for k in self.writers.keys():
                            if not self.writers[k].addParam(self.motors[mk].motor.id,[0,0]):
                                self.logger.error('FeetechSMChain sync write addParam failed ID %d.'%self.motors[mk].motor.id)
                                raise FeetechCommError

                for mk in self.motors.keys():
                    if not self.motors[mk].startup():
                        self.hw_valid = False
                        return False
                    self.status[mk] = self.motors[mk].status
                self.pull_status()
            except FeetechCommError as e:
                print(f"Feetech Com error: {e}")
                self.comm_errors.add_error(rx=True,gsr=True)
                self.hw_valid = False
                return False
        Device.startup(self)
        return True

    def _thread_loop(self):
        self.pull_status()


    def enable_rate_logging(self,max_samples=1000):
        #Not implemented
        pass

    def get_rate_log(self):
        return []

    def stop(self):
        Device.stop(self)
        if not self.hw_valid:
            return
        for motor in self.motors:
            self.motors[motor].stop(close_port=False)
        self.port_handler.closePort()
        self.hw_valid = False

    def wait_while_is_moving(self, timeout=15.0, use_motion_generator=True):
        at_setpoint = []
        def check_wait(wait_method):
            at_setpoint.append(wait_method(timeout,use_motion_generator))
        threads = []
        for motor in self.motors:
            threads.append(threading.Thread(target=check_wait, args=(self.motors[motor].wait_until_at_setpoint,)))
        [done_thread.start() for done_thread in threads]
        [done_thread.join() for done_thread in threads]
        return all(at_setpoint)


    def pull_status(self):
        if not self.hw_valid:
            return

        try:
            ts = time.time()
            error=False
            if self.params['use_group_sync_read']:
                pos = self.sync_read(self.readers['pos'])
                if pos==None and self.params['retry_on_comm_failure']:
                    pos = self.sync_read(self.readers['pos'])
                error= error or (pos==None)

                vel = self.sync_read(self.readers['vel'])
                if vel == None and self.params['retry_on_comm_failure']:
                    vel = self.sync_read(self.readers['vel'])
                error = error or (vel == None)

                if self.status_mux_id == 0:
                    current = self.sync_read(self.readers['current'])
                    if current == None and self.params['retry_on_comm_failure']:
                        current = self.sync_read(self.readers['current'])
                    error = error or (current == None)
                else:
                    current = None

                if self.status_mux_id == 1:
                    temp = self.sync_read(self.readers['temp'])
                    if temp == None and self.params['retry_on_comm_failure']:
                        temp = self.sync_read(self.readers['temp'])
                    error = error or (temp == None)
                else:
                    temp = None

                if self.status_mux_id == 2:
                    hardware_error = self.sync_read(self.readers['hardware_error'])
                    if hardware_error == None and self.params['retry_on_comm_failure']:
                        hardware_error = self.sync_read(self.readers['hardware_error'])
                    error = error or (hardware_error == None)
                else:
                    hardware_error = None

                self.status_mux_id = (self.status_mux_id + 1) % 3

                if error:
                    self.comm_errors.add_error(rx=True, gsr=True)
                    self.logger.warning('Feetech communication error (1) during pull_status on %s: ' % self.name)
                    for mk in self.motors.keys():
                        self.motors[mk]._last_pos_valid = False
                    #Todo: test if any of this helps
                    self.port_handler.is_using = False
                    try:
                        self.port_handler.closePort()
                        self.port_handler.openPort()
                    except Exception:
                        pass

                idx = 0
                # Build dictionary of status data and push to each motor status
                # None may indicate comm error or the field wasn't read on this mux cycle
                for mk in self.motors.keys():
                    data = {'ts': time.time()}
                    if pos is not None:
                        data['x'] = pos[idx]
                    else:
                        data['x'] = self.motors[mk].status['pos_ticks']
                    if vel is not None:
                        data['v'] = vel[idx]
                    else:
                        data['v'] = self.motors[mk].status['vel_ticks']
                    if current is not current:
                        data['current_mA'] = current[idx]
                    else:
                        data['current_mA'] = self.motors[mk].status['current_mA']
                    if temp is not None:
                        data['temp'] = temp[idx]
                    else:
                        data['temp'] = self.motors[mk].status['temp']
                    if hardware_error is not None:
                        data['err'] = hardware_error[idx]
                    else:
                        data['err'] = self.motors[mk].status['hardware_error']
                    self.motors[mk].pull_status(data)
                    idx = idx + 1
            else:
                for m in self.motors:
                    with self.pt_lock:
                        self.motors[m].pull_status()
        except(FeetechCommError, IOError):
            self.comm_errors.add_error(rx=True, gsr=True)
            self.logger.warning('Feetech communication error (2) during pull_status on %s: ' % self.name)
            for mk in self.motors.keys():
                self.motors[mk]._last_pos_valid = False
            if self.port_handler:
                self.port_handler.is_using = False
                try:
                    self.port_handler.closePort()
                    self.port_handler.openPort()
                except Exception:
                    pass

    def pretty_print(self):
        print('--- FeetechSMChain Chain ---')
        self.comm_errors.pretty_print()
        print('USB', self.usb)
        for mk in self.motors.keys():
            self.motors[mk].pretty_print()

    def sync_write(self,writer,data):
        print('FeetechSMChain sync_write not supported yet')
        return
        if not self.hw_valid:
            return None
        for m in self.motors:
            value = data[self.motors[m].name]
            vv=value
            if servoRegs[writer.start_address]['size'] == 2:
                value = self.packet_handler.scs_toscs(value,servoRegs[writer.start_address]["bitlen"])
                value = [self.packet_handler.scs_lobyte(int(value)), self.packet_handler.scs_hibyte(int(value))]
            else:
                value = [int(value)]
            writer.changeParam(self.motors[m].motor.id,value)
        with self.pt_lock:
            writer.txPacket()

    def sync_read(self, reader):
        print('FeetechSMChain sync_read not supported yet')
        return
        if not self.hw_valid:
            return None

        with self.pt_lock:
            result = reader.txRxPacket()
        if result != COMM_SUCCESS:
            self.logger.error(f'FeetechSMChain sync read txRxPacket failed with error code = {result}')
            return None
            #raise FeetechCommError

        def get_val(id_num):
            try:
                with self.pt_lock:
                    b = reader.getData(id_num, reader.start_address, reader.data_length)
            except:
                #Bad data struct size possible to raise Index Error
                return None #raise FeetechCommError
            if reader.data_length == 4:
                val = struct.unpack('i', arr.array('B', [SMS_LOBYTE(SMS_LOWORD(b)), SMS_HIBYTE(SMS_LOWORD(b)),
                                                         SMS_LOBYTE(SMS_HIWORD(b)), SMS_HIBYTE(SMS_HIWORD(b))]))[0]
            if reader.data_length == 2:
                val = struct.unpack('h', arr.array('B', [SMS_LOBYTE(b), SMS_HIBYTE(b)]))[0]
            if reader.data_length == 1:
                val = struct.unpack('b', arr.array('B', [b]))[0]
            return val
        try:
            values = [get_val(self.motors[mk].motor.id) for mk in self.motors.keys()]
        except:
            return None
        return values

    def step_sentry(self,robot_status):
        for k in self.motors.keys():
            self.motors[k].step_sentry(robot_status)

class FeetechSMChainStatus(TypedDict):
    comm_errors: FeetechCommErrorStatsStatus
    joint_names: list
