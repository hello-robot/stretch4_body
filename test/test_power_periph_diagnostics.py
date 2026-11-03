import struct
from array import array

from stretch4_body.subsystem.power_periph import PowerPeriphAux

SCHEMA1_FORMAT = ('<BBHQIIIIIBBBBHHBBBBBBBBBBBB' 'IIIIIIIII' 'iiiBBH' 'bbbbbBBB' 'HHBBBB'
                  'IIIIII' 'iiiiiihHHHIIHHH')


def test_schema1_reply_decodes_bms_link_alarms_and_mos():
    assert struct.calcsize(SCHEMA1_FORMAT) == 190
    values = [0] * (len(SCHEMA1_FORMAT) - 1)
    values[0], values[2] = 1, 190
    values[5] = 0b101111
    values[6] = 0b1011
    values[27:34] = [42, 1000, 0xFFFFFFFF, 42, 3, 7, 1]
    values[36:39] = [25300, -1500, 0]
    values[39:47] = [87, 100, 12, 21, 22, 23, -4, 30]
    values[47] = 0b010
    values[48] = 0b0100
    snapshot = struct.pack(SCHEMA1_FORMAT, *values)
    header = struct.pack('<BBHBBBBHHIII', 1, 24, 190, 16, 0, 0, 0, 0, 0, 0, 0, 0)

    power = PowerPeriphAux()
    power._rpc_power_diagnostics_reply(array('B', bytes([32]) + header + snapshot))
    d = power.power_diagnostics
    assert d['_valid']
    assert d['capabilities']['bms_mos_states'] and not d['capabilities']['event_log']
    assert d['validity']['bms_sample'] and not d['validity']['analog_rails']
    assert (d['bms_valid_replies'], d['bms_crc_errors'], d['bms_timeouts'], d['bms_rx_overflows']) == (42, 3, 7, 1)
    assert d['bms_sample_age_s'] is None
    assert d['battery_voltage_mv'] == 25300 and d['battery_current_ma'] == -1500
    assert d['battery_ambient_temp_c'] == -4 and d['battery_mosfet_temp_c'] == 30
    assert d['bms_alarms']['cell_voltage_high'] and not d['bms_alarms']['total_voltage_high']
    assert d['bms_mos_states']['discharging'] and not d['bms_mos_states']['charging']
