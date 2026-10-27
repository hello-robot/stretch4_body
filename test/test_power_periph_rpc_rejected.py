"""PIMU rejected-request reply (id 38) handled before the decoders.

Drives PowerPeriph through a fake transport, no hardware. The fixed reply is five
bytes: 38, the request id received, a reason (1 unknown id, 2 short request,
3 empty request) and the request length as u16 little-endian.
"""
import array as arr
import logging
import os
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

    def do_rpc(self, blocking, is_push, payload, rpc_callback, backend=None, exiting=False, **kwargs):
        self.payloads.append(list(payload))
        rpc_callback(arr.array('B', self.replies.pop(0)))
        return 1


@pytest.fixture
def pimu():
    p = PowerPeriph()
    p.transport = FakeTransport()
    p.hw_valid = True
    # Not started against a board: no config or IMU config to push.
    p._dirty_config = False
    p.imu._dirty_config = False
    return p


def rejected(request_id, reason, length):
    return [PowerPeriph.RPC_REPLY_REJECTED, request_id, reason, length & 0xFF, length >> 8]


def test_rejected_reply_never_reaches_the_decoder(pimu, monkeypatch):
    pimu.status['motor_sync_cnt'] = 123
    pimu.transport.replies.append(rejected(PowerPeriph.RPC_SET_MOTOR_SYNC, 2, 1))
    assert pimu.trigger_motor_sync() is True
    assert pimu.status['motor_sync_cnt'] == 123

    monkeypatch.setattr(pimu, '_rpc_status_reply', lambda reply: pytest.fail('status decoder ran on a rejected reply'))
    pimu.transport.replies.append(rejected(PowerPeriph.RPC_GET_POWER_PERIPH_STATUS, 1, 1))
    assert pimu.pull_status() is True
    assert pimu.status['rpc_rejected']['count'] == 2


def test_rejected_counters_track_the_last_reply(pimu):
    assert pimu.status['rpc_rejected'] == {'count': 0, 'last_request_id': None, 'last_reason': None, 'last_length': None}

    pimu.transport.replies.append(rejected(PowerPeriph.RPC_SET_POWER_PERIPH_TRIGGER, 2, 3))
    pimu.clear_runstop()
    assert pimu.push_command() is True
    assert pimu.transport.payloads[-1][0] == PowerPeriph.RPC_SET_POWER_PERIPH_TRIGGER
    assert pimu.status['rpc_rejected'] == {'count': 1, 'last_request_id': 5, 'last_reason': 2, 'last_length': 3}

    pimu.transport.replies.append(rejected(99, 1, 1025))
    pimu.pull_status()
    assert pimu.status['rpc_rejected'] == {'count': 2, 'last_request_id': 99, 'last_reason': 1, 'last_length': 1025}

    pimu.transport.replies.append(rejected(0, 3, 0))
    pimu.pull_status()
    assert pimu.status['rpc_rejected'] == {'count': 3, 'last_request_id': 0, 'last_reason': 3, 'last_length': 0}


def test_normal_reply_still_decodes(pimu):
    pimu.transport.replies.append([PowerPeriph.RPC_REPLY_MOTOR_SYNC, 77, 0])
    pimu.trigger_motor_sync()
    assert pimu.status['motor_sync_cnt'] == 77
    assert pimu.status['rpc_rejected']['count'] == 0


def test_five_byte_reply_with_another_id_is_untouched(pimu):
    pimu.transport.replies.append([PowerPeriph.RPC_REPLY_POWER_PERIPH_TRIGGER, 1, 0, 0, 0])
    # The trigger acknowledgement is the other five-byte PIMU reply (id 6 plus the
    # echoed 32-bit mask). It must reach its own decoder, not the id 38 path.
    pimu._do_rpc(blocking=True, is_push=True, payload=pimu.transport.get_empty_payload(),
                 rpc_callback=pimu._rpc_trigger_reply)
    assert pimu.status['rpc_reply_errors'] == 0
    assert pimu.status['rpc_rejected'] == {'count': 0, 'last_request_id': None, 'last_reason': None, 'last_length': None}


def test_one_warning_per_request_id_and_reason(pimu, caplog):
    with caplog.at_level(logging.WARNING, logger='power_periph'):
        for reply in (rejected(3, 1, 1), rejected(3, 1, 1), rejected(3, 1, 1), rejected(3, 2, 1), rejected(5, 2, 5)):
            pimu.transport.replies.append(reply)
            pimu.pull_status()
    warnings = [r.getMessage() for r in caplog.records if r.levelno == logging.WARNING and 'rejected RPC' in r.getMessage()]
    assert warnings == [
        'PIMU rejected RPC 3: unknown id (reason 1), request length 1 bytes',
        'PIMU rejected RPC 3: short request (reason 2), request length 1 bytes',
        'PIMU rejected RPC 5: short request (reason 2), request length 5 bytes',
    ]
    assert pimu.status['rpc_rejected']['count'] == 5
