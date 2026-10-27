#!/usr/bin/env python3
"""
Hardware-free tests for reconnecting end of arm servos that went offline (eg. a tool swapped over the
quick connect), and for homing only the joints that need it afterwards.
"""
import time
from unittest.mock import MagicMock, patch

import pytest

from stretch4_body.core.feetech import feetech_SM_chain
from stretch4_body.core.feetech.feetech_SM_chain import FeetechSMChain
from stretch4_body.core.feetech.feetech_SM_servo import FeetechCommError
from stretch4_body.subsystem.end_of_arm import end_of_arm_tools
from stretch4_body.subsystem.end_of_arm.end_of_arm import EndOfArm
from stretch4_body.subsystem.end_of_arm.wrist_yaw import WristYaw


class FakeServo:
    """Stands in for a FeetechSMHello on the chain"""
    def __init__(self, name):
        self.name = name
        self.hw_valid = True
        self.online = True # Whether the servo is plugged in
        self.ts_last_comm_success = time.time()
        self.status = {'pos_calibrated': True}
        self.params = {'req_calibration': 1, 'eeprom_cfg': {'overcurrent': 100}}
        self.motor = MagicMock()
        self.n_homed = 0

    def pull_status(self, data=None):
        if self.hw_valid and self.online:
            self.ts_last_comm_success = time.time()

    def mark_offline(self):
        self.hw_valid = False
        self.status['pos_calibrated'] = False

    def reconnect(self):
        if not self.online:
            return False
        self.hw_valid = True
        self.ts_last_comm_success = time.time()
        return True

    def is_homed(self):
        return self.status['pos_calibrated']

    def is_calibration_required(self):
        return self.params['req_calibration'] and not self.status['pos_calibrated']

    def home(self, end_pos=None):
        self.n_homed += 1
        self.status['pos_calibrated'] = True
        return True

    def pre_home(self, **kwargs):
        return True


def make_chain(joints=('wrist_pitch', 'wrist_roll', 'wrist_yaw', 'stretch_gripper')):
    chain = FeetechSMChain(usb='/dev/null', name='eoa_wrist_dw4_tool_sg4')
    chain.hw_valid = True
    chain.port_handler = MagicMock()
    chain.reconnect_enabled = True
    for j in joints:
        chain.add_motor(FakeServo(j))
    return chain


def unplug(chain, joints):
    """Unplug joints and let them be detected as offline"""
    for j in joints:
        chain.motors[j].online = False
        chain.motors[j].ts_last_comm_success -= feetech_SM_chain.SERVO_OFFLINE_AFTER_S + 1
    chain.pull_status()


def replug(chain, joints):
    for j in joints:
        chain.motors[j].online = True
    chain.ts_last_reconnect_attempt = 0 # Skip the wait between reconnect attempts
    chain.pull_status()


def is_homed(chain, joints):
    return {j: chain.motors[j].is_homed() for j in joints}


def test_gripper_swap_only_unhomes_gripper():
    chain = make_chain()
    unplug(chain, ['stretch_gripper'])
    assert chain.offline_motors == {'stretch_gripper'}
    assert not chain.motors['stretch_gripper'].hw_valid
    assert is_homed(chain, chain.motors) == {'wrist_pitch': True, 'wrist_roll': True, 'wrist_yaw': True, 'stretch_gripper': False}

    replug(chain, ['stretch_gripper'])
    assert chain.offline_motors == set()
    assert chain.motors['stretch_gripper'].hw_valid
    assert not chain.motors['stretch_gripper'].is_homed() # Reconnected, but needs homing


def test_wrist_swap_unhomes_all():
    chain = make_chain()
    unplug(chain, list(chain.motors))
    assert chain.offline_motors == set(chain.motors)
    replug(chain, ['wrist_yaw']) # Servos may come back one at a time
    assert chain.offline_motors == {'wrist_pitch', 'wrist_roll', 'stretch_gripper'}
    replug(chain, list(chain.motors))
    assert chain.offline_motors == set()
    assert not any(is_homed(chain, chain.motors).values())


def test_reconnect_waits_between_attempts():
    chain = make_chain()
    unplug(chain, ['stretch_gripper'])
    chain.motors['stretch_gripper'].reconnect = MagicMock(return_value=False)
    chain.ts_last_reconnect_attempt = 0
    chain.pull_status()
    chain.pull_status()
    assert chain.motors['stretch_gripper'].reconnect.call_count == 1


def test_transient_comm_error_does_not_unhome():
    chain = make_chain()
    chain.motors['stretch_gripper'].online = False
    chain.pull_status() # Missed a read, but has not been silent for SERVO_OFFLINE_AFTER_S
    assert chain.offline_motors == set()
    assert chain.motors['stretch_gripper'].is_homed()


def test_usb_lost_and_returned():
    chain = make_chain()
    with patch.object(feetech_SM_chain.os.path, 'exists', return_value=False):
        unplug(chain, list(chain.motors))
        chain.ts_last_reconnect_attempt = 0
        chain.pull_status()
    assert not chain.hw_valid

    def reopen():
        chain.hw_valid = True
        return True
    with patch.object(feetech_SM_chain.os.path, 'exists', return_value=True), \
         patch.object(chain, 'create_port_handler', side_effect=reopen):
        replug(chain, list(chain.motors))
    assert chain.hw_valid
    assert chain.offline_motors == set()
    for m in chain.motors.values():
        assert m.motor.port_handler is chain.port_handler


def test_stale_port_is_reopened():
    chain = make_chain()
    unplug(chain, ['stretch_gripper'])
    chain.motors['stretch_gripper'].reconnect = MagicMock(side_effect=OSError('device reports readiness to read but returned no data'))
    with patch.object(feetech_SM_chain.os.path, 'exists', return_value=True):
        replug(chain, ['stretch_gripper'])
    assert not chain.hw_valid # Port is recreated on the next attempt


def test_servo_mark_offline_and_reconnect():
    servo = WristYaw()
    servo.hw_valid = True
    servo.status['pos_calibrated'] = True
    servo.mark_offline()
    assert not servo.hw_valid and not servo.is_homed()

    servo.motor = MagicMock()
    servo.motor.do_ping.return_value = False
    assert not servo.reconnect()

    servo.motor.do_ping.return_value = True
    def startup():
        servo.hw_valid = True
        servo.status['pos_calibrated'] = True # Servo kept its calibration flag, eg. never lost power
        return True
    with patch.object(servo, 'startup', side_effect=startup):
        assert servo.reconnect()
    assert servo.hw_valid
    assert not servo.is_homed()
    servo.motor.set_is_calibrated.assert_called_once_with(0)

    servo.mark_offline()
    with patch.object(servo, 'startup', side_effect=FeetechCommError('lost mid-startup')):
        assert not servo.reconnect()
    assert not servo.hw_valid


@pytest.fixture(autouse=True)
def no_homing_sleep():
    with patch.object(end_of_arm_tools.time, 'sleep'):
        yield


def make_eoa(uncalibrated):
    eoa = MagicMock()
    eoa.cancel_homing_event.is_set.return_value = False
    eoa.params = {}
    eoa.motors = {j: FakeServo(j) for j in ('wrist_pitch', 'wrist_roll', 'wrist_yaw', 'stretch_gripper')}
    for j in uncalibrated:
        eoa.motors[j].status['pos_calibrated'] = False
    return eoa


def n_homed(eoa):
    return {j: m.n_homed for j, m in eoa.motors.items()}


def test_home_only_unhomed_gripper():
    eoa = make_eoa(['stretch_gripper'])
    assert end_of_arm_tools.home_dw4_joints(eoa, only_unhomed=True)
    assert n_homed(eoa) == {'wrist_pitch': 0, 'wrist_roll': 0, 'wrist_yaw': 0, 'stretch_gripper': 1}


def test_home_only_unhomed_wrist_homes_wrist_as_unit():
    eoa = make_eoa(['wrist_roll'])
    assert end_of_arm_tools.home_dw4_joints(eoa, only_unhomed=True)
    assert n_homed(eoa) == {'wrist_pitch': 1, 'wrist_roll': 1, 'wrist_yaw': 1, 'stretch_gripper': 0}


def test_home_all():
    eoa = make_eoa([])
    assert end_of_arm_tools.home_dw4_joints(eoa)
    assert n_homed(eoa) == {'wrist_pitch': 1, 'wrist_roll': 1, 'wrist_yaw': 1, 'stretch_gripper': 1}


def test_home_unhomed_falls_back_for_user_tools():
    class UserTool(EndOfArm):
        def home(self, wait_on_completion=True):
            self.homed_with = {'wait_on_completion': wait_on_completion}
    class UserToolPartial(EndOfArm):
        def home(self, wait_on_completion=True, only_unhomed=False):
            self.homed_with = {'wait_on_completion': wait_on_completion, 'only_unhomed': only_unhomed}

    tool = UserTool.__new__(UserTool)
    tool.home_unhomed(wait_on_completion=False)
    assert tool.homed_with == {'wait_on_completion': False}

    tool = UserToolPartial.__new__(UserToolPartial)
    tool.home_unhomed(wait_on_completion=False)
    assert tool.homed_with == {'wait_on_completion': False, 'only_unhomed': True}
