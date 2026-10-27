"""Power diagnostics schema 2 (firmware v0.1.47p14), the schema 1 fallback, and
schema 3 (v0.1.53p14, the ESP firmware version appended).

The two replies in test/fixtures are the bytes the firmware's native test packs
(stretch_firmware_ii tests/native/test_power_diag_reply.cpp): a snapshot with a
different value at every byte position, two events. Every field is checked
against its offset in the firmware struct, read straight out of the fixture.
"""
from array import array
import os
import struct

from stretch4_body.subsystem.power_periph import PowerPeriphAux

FIXTURES = os.path.join(os.path.dirname(__file__), 'fixtures')

# (name in power_diagnostics, snapshot offset, struct format) for every field
# the decoder exposes under the firmware's name. Offsets from Common.h.
SCHEMA1_FIELDS = [
    ('schema_version', 0, 'B'), ('reset_cause_raw', 1, 'B'), ('struct_size', 2, 'H'),
    ('uptime_ms', 4, 'Q'), ('sample_sequence', 12, 'I'), ('capability_flags_raw', 16, 'I'),
    ('validity_flags_raw', 20, 'I'), ('power_transition_sequence', 24, 'I'),
    ('power_transition_timestamp_ms', 28, 'I'), ('power_state', 32, 'B'),
    ('runstop_cause_raw', 33, 'B'), ('periph_power_state_raw', 34, 'B'),
    ('cpu_on_status', 35, '?'), ('power_transition_reason_raw', 36, 'H'),
    ('power_button_high_count', 38, 'H'), ('power_button_raw', 40, '?'),
    ('power_button_debounced', 41, '?'), ('adapter_voltage_present', 42, '?'),
    ('charger_enable_command', 43, '?'), ('charger_expected_active', 44, '?'),
    ('user_charger_control', 45, '?'), ('wait_wake_ack', 46, '?'), ('wait_sleep_ack', 47, '?'),
    ('esp_adapter_connected', 48, '?'), ('esp_adapter_fault', 49, '?'),
    ('esp_network_connected', 50, '?'), ('esp_firebase_configured', 51, '?'),
    ('bms_sample_sequence', 52, 'I'), ('bms_last_valid_ms', 56, 'I'),
    ('bms_sample_age_ms', 60, 'I'), ('bms_valid_replies', 64, 'I'), ('bms_crc_errors', 68, 'I'),
    ('bms_timeouts', 72, 'I'), ('bms_rx_overflows', 76, 'I'), ('bms_protocol_errors', 80, 'I'),
    ('bms_consecutive_failures', 84, 'I'), ('battery_voltage_mv', 88, 'i'),
    ('battery_current_ma', 92, 'i'), ('charger_current_ma', 96, 'i'), ('battery_soc', 100, 'B'),
    ('battery_soh', 101, 'B'), ('battery_cycles', 102, 'H'), ('battery_cell_temp_1_c', 104, 'b'),
    ('battery_cell_temp_2_c', 105, 'b'), ('battery_cell_temp_3_c', 106, 'b'),
    ('battery_ambient_temp_c', 107, 'b'), ('battery_mosfet_temp_c', 108, 'b'),
    ('bms_alarm_mask_raw', 109, 'B'), ('bms_mos_mask_raw', 110, 'B'),
    ('bms_last_failure_rx_len', 112, 'H'), ('bms_last_failure_event', 114, 'H'),
    ('bms_last_failure_function', 116, 'B'), ('bms_last_failure_exception_code', 117, 'B'),
    ('bms_last_failure_address', 118, 'B'), ('imu_tap_count', 119, 'B'),
    ('esp_status_age_ms', 120, 'I'), ('esp_rx_packets', 124, 'I'), ('esp_crc_errors', 128, 'I'),
    ('esp_rx_overflows', 132, 'I'), ('esp_requests_sent', 136, 'I'),
    ('esp_tx_queue_drops', 140, 'I'), ('voltage_cpu_mv', 144, 'i'), ('voltage_5v0_mv', 148, 'i'),
    ('voltage_36v0_mv', 152, 'i'), ('voltage_12v0_mv', 156, 'i'),
    ('voltage_20v0_aux_mv', 160, 'i'), ('current_cpu_ma', 164, 'i'),
    ('pimu_temp_centi_c', 168, 'h'), ('motor_sync_cnt', 170, 'H'), ('loop_time_us', 172, 'H'),
    ('loop_time_max_us', 174, 'H'), ('loop_overrun_count', 176, 'I'),
    ('diagnostics_rpc_count', 180, 'I'), ('diagnostics_reply_bytes', 184, 'H'),
    ('diagnostics_lock_time_us', 186, 'H'), ('diagnostics_lock_time_max_us', 188, 'H'),
]
SCHEMA2_FIELDS = [
    ('charge_stop_state', 111, 'B'),
    ('eye_overlay', 202, 'B'), ('eye_reserved', 203, 'B'), ('easter_egg_mode', 204, 'B'),
    ('shutdown_wait_active', 205, '?'), ('shutdown_wait_last_end', 206, 'B'),
    ('shutdown_wait_count', 207, 'H'), ('shutdown_wait_elapsed_ms', 209, 'I'),
    ('shutdown_wait_last_duration_ms', 213, 'I'), ('shutdown_wait_cap_ms', 217, 'I'),
    ('adc_stale', 223, 'B'), ('adapter_presence_source', 224, 'B'),
    ('esp_rx_length_errors', 225, 'I'), ('esp_rx_unknown_ids', 229, 'I'),
    ('esp_tx_mark_resets', 233, 'I'), ('esp_shutdown_cmd_sends', 237, 'I'),
    ('esp_adapter_disagree', 241, 'B'), ('wait_sd_ack', 242, '?'), ('reserved2', 243, 'B'),
    ('imu_tap_last', 244, 'B'), ('ina_vbat_mv', 245, 'i'), ('ina_current_ma', 249, 'i'),
    ('ina_flags', 253, 'B'), ('ina_reserved', 254, 'B'),
]
SCHEMA2_ONLY_KEYS = [name for name, _, _ in SCHEMA2_FIELDS] + ['eye_left', 'eye_right', 'adc_reinit_cnt']


def load_fixture(schema):
    data = bytearray()
    with open(os.path.join(FIXTURES, 'power_diag_schema%d_reply.hex' % schema)) as f:
        for line in f:
            if not line.startswith('#'):
                data.extend(int(tok, 16) for tok in line.split())
    return bytes(data)


def raw_field(snapshot, offset, fmt):
    value = struct.unpack_from('<' + ('B' if fmt == '?' else fmt), snapshot, offset)[0]
    return bool(value) if fmt == '?' else value


def error_reply(status, header_schema=2):
    return bytes([PowerPeriphAux.RPC_REPLY_POWER_DIAGNOSTICS]) + struct.pack(
        '<BBHBBBBHHIII', header_schema, 24, 0, 16, 0, 0, status, 0, 0, 0, 0, 0)


class ScriptedTransport:
    """Answers each do_rpc with the next scripted reply through the callback."""

    def __init__(self, replies):
        self.replies = list(replies)
        self.payloads = []

    def do_rpc(self, blocking, is_push, payload, rpc_callback, **kwargs):
        self.payloads.append(bytes(payload))
        if blocking and self.replies:
            rpc_callback(array('B', self.replies.pop(0)))
        return True


def p14_power(replies=()):
    power = PowerPeriphAux()
    power.hw_valid = True
    power.board_info = {'protocol_version': 'p14'}
    power.transport = ScriptedTransport(replies)
    return power


def test_fixture_sizes_match_the_firmware_framing():
    assert len(load_fixture(3)) == 1 + 24 + 289 + 2 * 16
    assert len(load_fixture(2)) == 1 + 24 + 255 + 2 * 16
    assert len(load_fixture(1)) == 1 + 24 + 190 + 2 * 16
    assert PowerPeriphAux.POWER_DIAGNOSTICS_SCHEMA_VERSION == 3
    assert PowerPeriphAux.POWER_DIAGNOSTICS_WIRE_SIZES == {1: 190, 2: 255, 3: 289}


def test_schema2_fixture_decodes_every_field_byte_for_byte():
    reply = load_fixture(2)
    snapshot = reply[25:25 + 255]
    power = PowerPeriphAux()
    power._rpc_power_diagnostics_reply(array('B', reply), requested_schema=2)
    d = power.power_diagnostics
    assert d['_valid'], d['_decode_error']
    assert d['_packet_length'] == 255
    assert d['schema_version'] == 2 and d['struct_size'] == 255
    for name, offset, fmt in SCHEMA1_FIELDS + SCHEMA2_FIELDS:
        assert d[name] == raw_field(snapshot, offset, fmt), name
    for name, base in (('eye_left', 190), ('eye_right', 196)):
        assert d[name] == dict(zip(('animation', 'intensity', 'r', 'g', 'b', 'source'),
                                   snapshot[base:base + 6])), name
    assert d['adc_reinit_cnt'] == [snapshot[221], snapshot[222]]
    assert 'reserved0' not in d
    page = power.power_event_log_page
    assert page['_valid'] and page['schema_version'] == 2 and page['diagnostics_size'] == 255
    assert page['entry_count'] == 2
    for i, event in enumerate(page['events']):
        raw = struct.unpack_from('<IIHBBhh', reply, 25 + 255 + 16 * i)
        assert (event['sequence'], event['timestamp_ms'], event['event_id'], event['source_raw'],
                event['power_state'], event['data0'], event['data1']) == raw
    assert power.status_aux['_diagnostics_present']
    assert power.status_aux['_diagnostics_packet_length'] == 255
    for name in PowerPeriphAux.POWER_DIAGNOSTICS_SCHEMA3_KEYS:
        assert d[name] is None, name
    assert d['esp_version_state_name'] is None


def test_schema3_fixture_decodes_the_esp_version_after_the_schema2_bytes():
    reply = load_fixture(3)
    snapshot = reply[25:25 + 289]
    power = PowerPeriphAux()
    power._rpc_power_diagnostics_reply(array('B', reply), requested_schema=3)
    d = power.power_diagnostics
    assert d['_valid'], d['_decode_error']
    assert d['_packet_length'] == 289
    assert d['schema_version'] == 3 and d['struct_size'] == 289
    for name, offset, fmt in SCHEMA1_FIELDS + SCHEMA2_FIELDS:
        assert d[name] == raw_field(snapshot, offset, fmt), name
    assert d['esp_version_state'] == snapshot[255]
    assert d['esp_version_requests'] == snapshot[256]
    # The fixture pattern has no NUL in the 32 bytes; bytes over 0x7F decode as U+FFFD.
    assert d['esp_version'] == bytes(snapshot[257:289]).decode('ascii', 'replace')
    page = power.power_event_log_page
    assert page['_valid'] and page['schema_version'] == 3 and page['diagnostics_size'] == 289
    for i, event in enumerate(page['events']):
        raw = struct.unpack_from('<IIHBBhh', reply, 25 + 289 + 16 * i)
        assert (event['sequence'], event['timestamp_ms'], event['event_id'], event['source_raw'],
                event['power_state'], event['data0'], event['data1']) == raw


def test_schema3_esp_version_as_the_firmware_fills_it():
    reply = bytearray(load_fixture(3))
    base = 25 + 255
    for state, requests, text, name in ((1, 1, b'hello-esp.v0.1.11p0', 'reported'),
                                        (2, 5, b'', 'no_reply'), (0, 0, b'', 'waiting'),
                                        (7, 0, b'', 'unknown_7')):
        reply[base] = state
        reply[base + 1] = requests
        reply[base + 2:base + 34] = text.ljust(32, b'\0')
        power = PowerPeriphAux()
        power._rpc_power_diagnostics_reply(array('B', bytes(reply)), requested_schema=3)
        d = power.power_diagnostics
        assert d['_valid'], d['_decode_error']
        assert d['esp_version_state'] == state and d['esp_version_state_name'] == name
        assert d['esp_version_requests'] == requests
        assert d['esp_version'] == text.decode()


def test_schema2_and_schema3_agree_on_the_shared_bytes():
    two, three = PowerPeriphAux(), PowerPeriphAux()
    two._rpc_power_diagnostics_reply(array('B', load_fixture(2)))
    three._rpc_power_diagnostics_reply(array('B', load_fixture(3)))
    for name, _, _ in SCHEMA1_FIELDS[3:] + SCHEMA2_FIELDS:
        assert two.power_diagnostics[name] == three.power_diagnostics[name], name
    assert two.power_event_log_page['events'] == three.power_event_log_page['events']


def test_schema1_fixture_decodes_with_the_schema2_fields_none():
    reply = load_fixture(1)
    snapshot = reply[25:25 + 190]
    power = PowerPeriphAux()
    power._rpc_power_diagnostics_reply(array('B', reply), requested_schema=1)
    d = power.power_diagnostics
    assert d['_valid'], d['_decode_error']
    assert d['_packet_length'] == 190
    assert d['schema_version'] == 1 and d['struct_size'] == 190
    for name, offset, fmt in SCHEMA1_FIELDS:
        assert d[name] == raw_field(snapshot, offset, fmt), name
    assert d['reserved0'] == snapshot[111]
    for name in SCHEMA2_ONLY_KEYS + list(PowerPeriphAux.POWER_DIAGNOSTICS_SCHEMA3_KEYS):
        assert d[name] is None, name
    assert power.power_event_log_page['entry_count'] == 2
    assert power.status_aux['_diagnostics_packet_length'] == 190


def test_schema1_and_schema2_agree_on_the_shared_bytes():
    one, two = PowerPeriphAux(), PowerPeriphAux()
    one._rpc_power_diagnostics_reply(array('B', load_fixture(1)))
    two._rpc_power_diagnostics_reply(array('B', load_fixture(2)))
    for name, _, _ in SCHEMA1_FIELDS[3:]:
        assert one.power_diagnostics[name] == two.power_diagnostics[name], name
    assert one.power_event_log_page['events'] == two.power_event_log_page['events']


def test_request_asks_for_schema_3_first():
    power = p14_power([load_fixture(3)])
    assert power.pull_power_diagnostics(blocking=True, after_sequence=5, max_entries=4)
    assert power.transport.payloads == [bytes([31, 3, 4, 0, 0, 5, 0, 0, 0])]
    assert power.power_diagnostics['schema_version'] == 3
    assert power.power_diagnostics_request_schema == 3


def test_firmware_v0_1_47_to_v0_1_52_steps_down_to_schema_2_and_remembers():
    # Those builds answer a schema 3 request with status 2 and header schema 2.
    power = p14_power([error_reply(2, header_schema=2), load_fixture(2)])
    assert power.pull_power_diagnostics(blocking=True)
    assert [p[1] for p in power.transport.payloads] == [3, 2]
    assert power.power_diagnostics['_valid']
    assert power.power_diagnostics['schema_version'] == 2
    assert power.power_diagnostics['esp_version'] is None
    assert power.power_diagnostics_request_schema == 2
    power.transport.replies.append(load_fixture(2))
    assert power.pull_power_diagnostics(blocking=True)
    assert [p[1] for p in power.transport.payloads] == [3, 2, 2]


def test_error_header_naming_an_unknown_newer_schema_falls_back_to_1():
    power = p14_power([error_reply(2, header_schema=9), load_fixture(1)])
    assert power.pull_power_diagnostics(blocking=True)
    assert [p[1] for p in power.transport.payloads] == [3, 1]


def test_unsupported_schema_falls_back_to_schema_1_once_and_remembers():
    # v0.1.18p14 answers a newer schema request with status 2 and header schema 1.
    power = p14_power([error_reply(2, header_schema=1), load_fixture(1)])
    assert power.pull_power_diagnostics(blocking=True)
    assert [p[1] for p in power.transport.payloads] == [3, 1]
    assert power.power_diagnostics['_valid']
    assert power.power_diagnostics['schema_version'] == 1
    assert power.power_diagnostics_request_schema == 1

    power.transport.replies.append(load_fixture(1))
    assert power.pull_power_diagnostics(blocking=True)
    assert [p[1] for p in power.transport.payloads] == [3, 1, 1]


def test_unsupported_reply_is_reported_as_unsupported_not_malformed():
    power = p14_power([error_reply(2, header_schema=1)])
    power.pull_power_diagnostics(blocking=False)
    power.transport.payloads.clear()
    power._rpc_power_diagnostics_reply(array('B', error_reply(2, header_schema=1)), requested_schema=3)
    error = power.power_diagnostics['_decode_error']
    assert 'unsupported_schema' in error and 'malformed' not in error
    assert power.power_event_log_page['reply_status_name'] == 'unsupported_schema'
    # Not blocking: no second request inside the callback, the next pull asks for 1.
    assert power.transport.payloads == []
    assert power.power_diagnostics_request_schema == 1
    power.pull_power_diagnostics(blocking=False)
    assert power.transport.payloads[-1][1] == 1


def test_firmware_v0_1_46_unsupported_reply_with_header_schema_2_is_not_malformed():
    power = PowerPeriphAux()
    power._rpc_power_diagnostics_reply(array('B', error_reply(2, header_schema=2)), requested_schema=1)
    assert 'unsupported_schema' in power.power_diagnostics['_decode_error']
    assert 'malformed' not in power.power_diagnostics['_decode_error']
    assert 'firmware schema 2' in power.power_diagnostics['_decode_error']


def test_schema_1_unsupported_does_not_loop():
    power = p14_power([error_reply(2, 1), error_reply(2, 1), error_reply(2, 1)])
    assert power.pull_power_diagnostics(blocking=True)
    assert [p[1] for p in power.transport.payloads] == [3, 1]
    assert not power.power_diagnostics['_valid']
    power.pull_power_diagnostics(blocking=True)
    assert [p[1] for p in power.transport.payloads] == [3, 1, 1]


def test_truncated_and_padded_replies_are_rejected():
    for schema in (1, 2, 3):
        reply = load_fixture(schema)
        for bad in (reply[:-1], reply[:-16], reply[:25 + 189], reply[:24], reply + b'\x00'):
            power = PowerPeriphAux()
            power._rpc_power_diagnostics_reply(array('B', bad))
            assert not power.power_diagnostics['_valid'], (schema, len(bad))
            assert not power.power_event_log_page['_valid'], (schema, len(bad))


def test_snapshot_sizes_other_than_190_255_and_289_are_rejected():
    power = PowerPeriphAux()
    for size in (189, 191, 203, 205, 245, 254, 256, 287, 288, 290):
        snapshot = bytearray(size)
        struct.pack_into('<BBH', snapshot, 0, 2, 0, size)
        assert power._unpack_power_diagnostics(array('B', snapshot)) == 0, size
        assert not power.power_diagnostics['_valid']


def test_header_and_snapshot_must_agree_on_the_schema():
    reply = bytearray(load_fixture(2))
    reply[1] = 1                                    # header says schema 1, 255 bytes follow
    power = PowerPeriphAux()
    power._rpc_power_diagnostics_reply(array('B', bytes(reply)))
    assert not power.power_diagnostics['_valid']

    reply = bytearray(load_fixture(2))
    reply[25] = 1                                   # snapshot says schema 1 at 255 bytes
    power._rpc_power_diagnostics_reply(array('B', bytes(reply)))
    assert not power.power_diagnostics['_valid']
    assert 'unsupported diagnostics schema' in power.power_diagnostics['_decode_error']


def test_capability_and_validity_bits_of_schema_2_are_named():
    reply = bytearray(load_fixture(2))
    struct.pack_into('<II', reply, 25 + 16, (1 << 9) | (1 << 10) | (1 << 11), 1 << 4)
    power = PowerPeriphAux()
    power._rpc_power_diagnostics_reply(array('B', bytes(reply)))
    caps = power.power_diagnostics['capabilities']
    assert caps['bench_rig_build'] and caps['eye_readback'] and caps['ina_vbat']
    assert not caps['simulated_battery']
    assert power.power_diagnostics['validity']['ina_sample']


def test_every_firmware_event_id_and_transition_reason_has_a_name():
    ids = list(range(1, 37)) + list(range(40, 48)) + [50, 51, 52, 53]
    for event_id in ids:
        assert event_id in PowerPeriphAux.POWER_EVENT_NAMES, event_id
        assert event_id in PowerPeriphAux.POWER_EVENT_EXPLANATIONS, event_id
    assert PowerPeriphAux.POWER_EVENT_NAMES[40] == 'adc_bank_reinit'
    assert PowerPeriphAux.POWER_EVENT_NAMES[47] == 'esp_shutdown_cmd_delivered'
    assert PowerPeriphAux.POWER_TRANSITION_REASON_NAMES[11] == 'esp_wake_request'
    assert PowerPeriphAux.POWER_TRANSITION_REASON_NAMES[12] == 'low_battery_recovered'
