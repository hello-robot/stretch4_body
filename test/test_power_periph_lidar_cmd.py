"""Lidar and AUX rail setters: one bit per rail in a trigger word, and the lidar_cmd record.

Firmware v0.1.49p14 resolves an ON and OFF bit for one rail in the same word as off-wins,
so the host clears the opposite bit and the last request wins. No hardware.
"""
import array as arr
import struct

import pytest

from stretch4_body.subsystem.power_periph import PowerPeriph


class FakeTransport:
    BACKEND_PY_SERIAL = 0
    BACKEND_C_SERIAL = 1

    def __init__(self):
        self.status = {}
        self.payloads = []

    def get_empty_payload(self):
        return arr.array('B', [0] * 1025)

    def do_rpc(self, blocking, is_push, payload, rpc_callback, backend=None, exiting=False, **kwargs):
        self.payloads.append(list(payload))
        rpc_callback(arr.array('B', [PowerPeriph.RPC_REPLY_POWER_PERIPH_TRIGGER] + list(payload[1:5])))
        return 1


@pytest.fixture
def pimu():
    p = PowerPeriph()
    p.transport = FakeTransport()
    p.hw_valid = True
    p._dirty_config = False
    p.imu._dirty_config = False
    return p


def pushed_word(p):
    p.push_command()
    payload = p.transport.payloads[-1]
    assert payload[0] == PowerPeriph.RPC_SET_POWER_PERIPH_TRIGGER
    return struct.unpack('<I', bytes(payload[1:5]))[0]


def test_off_then_on_sends_only_on(pimu):
    pimu.set_lidar_off()
    pimu.set_lidar_on()
    word = pushed_word(pimu)
    assert word & PowerPeriph.TRIGGER_LIDAR_ON
    assert not word & PowerPeriph.TRIGGER_LIDAR_OFF


def test_on_then_off_sends_only_off(pimu):
    pimu.set_lidar_on()
    pimu.set_lidar_off()
    word = pushed_word(pimu)
    assert word & PowerPeriph.TRIGGER_LIDAR_OFF
    assert not word & PowerPeriph.TRIGGER_LIDAR_ON


def test_lidar_pairing_keeps_other_bits(pimu):
    pimu.set_fan_on()
    pimu.set_lidar_off()
    pimu.set_lidar_on()
    assert pimu._trigger == PowerPeriph.TRIGGER_FAN_ON | PowerPeriph.TRIGGER_LIDAR_ON


def test_aux_pairing_last_request_wins(pimu):
    pimu.set_aux_cpu_off()
    pimu.set_aux_cpu_on()
    assert pimu._trigger == PowerPeriph.TRIGGER_20V0_AUX_ON
    pimu.set_aux_cpu_off()
    assert pimu._trigger == PowerPeriph.TRIGGER_20V0_AUX_OFF


def test_lidar_cmd_records_action_source_and_sequence(pimu):
    assert pimu.status['lidar_cmd'] is pimu.lidar_cmd
    assert pimu.lidar_cmd['seq'] == 0 and pimu.lidar_cmd['action'] is None
    pimu.set_lidar_on()
    assert pimu.lidar_cmd['action'] == 'on' and pimu.lidar_cmd['source'] == 'client'
    assert pimu.lidar_cmd['seq'] == 1 and pimu.lidar_cmd['ts'] is not None
    pimu.set_lidar_off(source='sentry')
    assert (pimu.lidar_cmd['action'], pimu.lidar_cmd['source'], pimu.lidar_cmd['seq']) == ('off', 'sentry', 2)


def test_release_hold_sends_nothing(pimu):
    pimu.release_lidar_hold()
    assert pimu.lidar_cmd['action'] == 'release' and pimu.lidar_cmd['seq'] == 1
    assert pimu._trigger == 0 and not pimu._dirty_trigger
