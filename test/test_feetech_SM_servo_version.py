"""Feetech firmware version: one read of addresses 0 to 4 at startup.

Address 0 firmware main version, 1 sub version, 3 and 4 the servo model word
(SMS_FIRMWARE_VERSION and SMS_MODEL in feetech_SM_servo). Nothing is written.
"""
import threading
import types

import pytest

from stretch4_body.core.feetech import feetech_SM_servo as servo_mod
from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
from stretch4_body.core.feetech.feetech_SM_servo import (
    FeetechSMServo, SMS_FIRMWARE_VERSION, SMS_VERSION_BLOCK_SIZE, decode_version_regs)
from stretch4_body.core.feetech.protocol_packet_handler import protocol_packet_handler
from stretch4_body.core.feetech.scservo_def import COMM_SUCCESS, COMM_RX_TIMEOUT


def test_decode_little_endian_bus():
    ph = protocol_packet_handler(None, 0)
    info = decode_version_regs([3, 10, 0, 0x09, 0x03], ph.scs_makeword)
    assert info == {'firmware_main': 3, 'firmware_sub': 10, 'version': '3.10', 'model': 0x0309}


def test_decode_big_endian_bus_uses_the_handlers_word_order():
    ph = protocol_packet_handler(None, 1)
    assert decode_version_regs([3, 10, 0, 0x09, 0x03], ph.scs_makeword)['model'] == 0x0903


def test_decode_refuses_a_short_block():
    ph = protocol_packet_handler(None, 0)
    for data in (None, [], [3, 10, 0, 9]):
        with pytest.raises(ValueError):
            decode_version_regs(data, ph.scs_makeword)


class FakePacketHandler(protocol_packet_handler):
    def __init__(self, reply):
        super().__init__(None, 0)
        self.reply = reply
        self.reads = []
        self.writes = []

    def readTxRx(self, scs_id, address, length):
        self.reads.append((scs_id, address, length))
        return self.reply

    def writeTxRx(self, *args):
        self.writes.append(args)
        return COMM_SUCCESS, 0


def fake_servo(reply):
    s = FeetechSMServo.__new__(FeetechSMServo)
    s.id = 7
    s.usb = '/dev/fake'
    s.hw_valid = True
    s.pt_lock = threading.RLock()
    s.comm_errors = 0
    s.last_comm_success = True
    s.logger = servo_mod.logging.getLogger('test')
    s.packet_handler = FakePacketHandler(reply)
    return s


def test_read_version_info_is_one_five_byte_read_and_no_write():
    s = fake_servo(([3, 10, 0, 0x09, 0x03], COMM_SUCCESS, 0))
    assert s.read_version_info()['version'] == '3.10'
    assert s.packet_handler.reads == [(7, SMS_FIRMWARE_VERSION, SMS_VERSION_BLOCK_SIZE)]
    assert s.packet_handler.writes == []


def test_read_version_info_raises_on_comm_failure_like_read_reg():
    s = fake_servo(([], COMM_RX_TIMEOUT, 0))
    with pytest.raises(servo_mod.FeetechCommError):
        s.read_version_info()
    assert s.comm_errors == 1


def test_read_version_info_when_not_connected():
    s = fake_servo(([3, 10, 0, 9, 3], COMM_SUCCESS, 0))
    s.hw_valid = False
    assert s.read_version_info() is None
    assert s.packet_handler.reads == []


def hello_with(motor):
    h = types.SimpleNamespace(motor=motor, status={})
    h._firmware_unread = FeetechSMHello._firmware_unread
    return h


def test_hello_records_the_version_with_a_read_time():
    h = hello_with(fake_servo(([3, 10, 0, 0x09, 0x03], COMM_SUCCESS, 0)))
    FeetechSMHello._read_firmware_version(h)
    fw = h.status['firmware']
    assert fw['version'] == '3.10' and fw['model'] == 0x0309
    assert fw['error'] is None and fw['read_time'] > 0


def test_hello_records_a_failed_read_and_does_not_raise():
    h = hello_with(fake_servo(([], COMM_RX_TIMEOUT, 0)))
    FeetechSMHello._read_firmware_version(h)
    fw = h.status['firmware']
    assert fw['version'] is None and fw['error'].startswith('read failed')


def test_status_default_says_not_started():
    assert FeetechSMHello._firmware_unread('not started') == {
        'version': None, 'firmware_main': None, 'firmware_sub': None, 'model': None,
        'read_time': None, 'error': 'not started'}
