"""PIMU Easter egg mode (firmware v0.1.38p14): RPC 39 out, reply 40 in, readback.

Drives PowerPeriph through a fake transport, no hardware. Request 39 carries one
byte (0 clear, 1 set); reply 40 carries one status byte (0 ok, 1 bad length,
2 bad value). The mode reads back as bit 20 of the status state word and, once
the snapshot schema is bumped, as the u8 at snapshot offset 204.
"""
import array as arr
import logging
import os
import struct
import tempfile

import pytest
import yaml

if 'HELLO_FLEET_PATH' not in os.environ:
    # Off a robot the parameter loader still needs a fleet directory at import time.
    _fleet_id = 'stretch-se4-0000'
    _fleet_dir = os.path.join(tempfile.mkdtemp(prefix='fleet-'), _fleet_id)
    os.makedirs(_fleet_dir)
    for _fn in ('stretch_user_params.yaml', 'stretch_configuration_params.yaml'):
        with open(os.path.join(_fleet_dir, _fn), 'w') as _f:
            yaml.dump({'robot': {'model_name': 'SE4', 'tool': 'eoa_wrist_dw4_tool_sg4'}}, _f)
    os.environ['HELLO_FLEET_PATH'] = os.path.dirname(_fleet_dir)
    os.environ['HELLO_FLEET_ID'] = _fleet_id

from stretch4_body.subsystem.power_periph import PowerPeriph


class FakeTransport:
    BACKEND_PY_SERIAL = 0
    BACKEND_C_SERIAL = 1

    def __init__(self):
        self.status = {}
        self.replies = []
        self.payloads = []

    def get_empty_payload(self):
        return arr.array('B', [0] * 1025)

    def do_rpc(self, blocking, is_push, payload, rpc_callback, backend=None, exiting=False):
        self.payloads.append(list(payload))
        rpc_callback(arr.array('B', self.replies.pop(0)))
        return 1


@pytest.fixture
def pimu():
    p = PowerPeriph()
    p.transport = FakeTransport()
    p.hw_valid = True
    p.board_info['protocol_version'] = 'p14'
    # Not started against a board: no config or IMU config to push.
    p._dirty_config = False
    p.imu._dirty_config = False
    return p


def status_reply(state):
    # Protocol 7 status layout: 17 IMU floats (qw at 48 must be a unit quaternion),
    # then voltage, current, temp and the u32 state word at 80; zeros after that.
    s = bytearray(256)
    struct.pack_into('<f', s, 48, 1.0)
    struct.pack_into('<I', s, 80, state)
    return [PowerPeriph.RPC_REPLY_POWER_PERIPH_STATUS] + list(s)


def test_set_sends_rpc_39_with_one_and_records_the_reply_status(pimu):
    assert pimu.status['easter_egg_mode_reply_status'] is None
    pimu.set_easter_egg_mode(True)
    pimu.transport.replies.append([PowerPeriph.RPC_REPLY_SET_EASTER_EGG_MODE, 0])
    assert pimu.push_command() is True
    assert pimu.transport.payloads == [[PowerPeriph.RPC_SET_EASTER_EGG_MODE, 1]]
    assert pimu.status['easter_egg_mode_reply_status'] == 0

    # Sent once, not on every push.
    assert pimu.push_command() is True
    assert len(pimu.transport.payloads) == 1


def test_clear_sends_rpc_39_with_zero_and_a_refusal_is_recorded(pimu, caplog):
    pimu.clear_easter_egg_mode()
    pimu.transport.replies.append([PowerPeriph.RPC_REPLY_SET_EASTER_EGG_MODE, 2])
    with caplog.at_level(logging.WARNING, logger='power_periph'):
        pimu.push_command()
    assert pimu.transport.payloads == [[PowerPeriph.RPC_SET_EASTER_EGG_MODE, 0]]
    assert pimu.status['easter_egg_mode_reply_status'] == 2
    assert any('bad value (status 2)' in r.getMessage() for r in caplog.records)


def test_wrong_reply_id_leaves_the_status_alone(pimu):
    pimu.set_easter_egg_mode(False)
    pimu.transport.replies.append([PowerPeriph.RPC_REPLY_POWER_PERIPH_TRIGGER])
    pimu.push_command()
    assert pimu.status['easter_egg_mode_reply_status'] is None


def test_status_bit_20_reads_back_as_0_or_1(pimu):
    assert pimu.status['easter_egg_mode'] == 0
    pimu.transport.replies.append(status_reply(PowerPeriph.STATE_EASTER_EGG_MODE | PowerPeriph.STATE_FAN_ON))
    pimu.pull_status()
    assert pimu.status['easter_egg_mode'] == 1
    assert pimu.status['fan_on'] is True
    assert pimu.status['runstop_event'] is False

    pimu.transport.replies.append(status_reply(PowerPeriph.STATE_RUNSTOP_EVENT))
    pimu.pull_status()
    assert pimu.status['easter_egg_mode'] == 0
    assert pimu.status['runstop_event'] is True


def test_snapshot_field_comes_from_the_schema_2_decoder(pimu):
    schema1 = bytearray(PowerPeriph.POWER_DIAGNOSTICS_SCHEMA1_WIRE_SIZE)
    struct.pack_into('<BBH', schema1, 0, 1, 0, len(schema1))
    assert pimu._unpack_power_diagnostics(arr.array('B', schema1)) == len(schema1)
    assert pimu.power_diagnostics['_valid'] is True
    assert pimu.power_diagnostics['easter_egg_mode'] is None

    for value in (0, 1):
        schema2 = bytearray(PowerPeriph.POWER_DIAGNOSTICS_WIRE_SIZE)
        struct.pack_into('<BBH', schema2, 0, 2, 0, len(schema2))
        schema2[204] = value
        assert pimu._unpack_power_diagnostics(arr.array('B', schema2)) == len(schema2)
        assert pimu.power_diagnostics['easter_egg_mode'] == value

    # A snapshot cut off after byte 204 is no snapshot at all.
    cut = bytearray(205)
    struct.pack_into('<BBH', cut, 0, 2, 0, len(cut))
    assert pimu._unpack_power_diagnostics(arr.array('B', cut)) == 0
    assert 'easter_egg_mode' not in pimu.power_diagnostics
