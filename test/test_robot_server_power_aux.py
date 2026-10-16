from array import array
from collections import deque
import struct

from stretch4_body.robot.robot_server import RobotServer
from stretch4_body.subsystem.power_periph import PowerPeriphAux
from stretch4_body.core.transport.transport import Transport


class FakePowerPeriph:
    def __init__(self):
        self.status_aux = {
            "current_pwr_state": 3,
            "current_pwr_state_name": "STATE_SLEEP_CHRG",
            "runstop_cause_raw": 4,
            "motor_sync_cnt": 123,
        }
        self.requests = 0
        self.diagnostics_requests = []
        self.diagnostics_supported = False
        self.status_aux_reply_sequence = 0
        self.power_diagnostics_reply_sequence = 0
        self.complete_on_queue = True
        self.power_diagnostics = {
            "_valid": True,
            "uptime_ms": 1000,
            "bms_sample_age_s": 0.1,
            "esp_status_age_s": 0.1,
        }
        self.power_event_log_page = {
            "_valid": True,
            "newest_sequence": 0,
            "oldest_sequence": 0,
            "dropped_count": 0,
            "has_more": False,
            "events": [],
        }

    def pull_status_aux(self, blocking):
        assert blocking is False
        self.requests += 1
        if self.complete_on_queue:
            self.status_aux_reply_sequence += 1
        return True

    def supports_power_diagnostics(self):
        return self.diagnostics_supported

    def pull_power_diagnostics(self, blocking, after_sequence, max_entries):
        assert blocking is False
        self.diagnostics_requests.append((after_sequence, max_entries))
        if self.complete_on_queue:
            self.power_diagnostics_reply_sequence += 1
        return True


def make_server():
    server = RobotServer.__new__(RobotServer)
    server.power_periph = FakePowerPeriph()
    server.get_subsystem = lambda name: (
        server.power_periph if name == "power_periph" else None
    )
    server._power_aux_interval_s = 0.5
    server._power_aux_pending = False
    server._power_aux_expected_reply_sequence = None
    server._power_aux_request_kind = None
    server._power_aux_request_started_mono = None
    server._power_aux_last_request_mono = float("-inf")
    server._power_aux_last_sample_mono = None
    server._power_aux_last_state = None
    server._power_aux_transition_seq = 0
    server._power_aux_transitions = deque(maxlen=64)
    server._power_aux_meta = {
        "available": False,
        "sample_seq": 0,
        "request_errors": 0,
        "request_timeouts": 0,
        "transport_error": None,
        "transitions": [],
    }
    server._power_diag_last_sample_mono = None
    server._power_diag_last_uptime_ms = None
    server._power_event_discard_page = False
    server._power_diag_meta = {
        "available": False,
        "sample_seq": 0,
        "request_errors": 0,
        "request_timeouts": 0,
        "request_latency_max_s": 0.0,
        "pimu_boot_count_observed": 0,
        "transport_error": None,
    }
    server._power_event_after_sequence = 0
    server._power_event_last_dropped_count = 0
    server._power_events = deque(maxlen=64)
    server._power_event_meta = {
        "available": False,
        "coverage_gaps": [],
        "coverage_gap_count": 0,
        "request_errors": 0,
        "request_timeouts": 0,
        "reboot_pages_discarded": 0,
        "transport_error": None,
    }
    return server


def test_power_aux_is_queued_nonblocking_and_records_transition():
    server = make_server()
    server._queue_power_periph_aux_status()
    assert server._power_aux_pending
    assert server.power_periph.requests == 1

    server._complete_power_periph_aux_status()
    assert not server._power_aux_pending
    assert server._power_aux_meta["available"]
    assert server._power_aux_meta["sample_seq"] == 1
    assert server._power_aux_meta["transitions"] == [{
        "seq": 1,
        "observed_wall_time_s": server._power_aux_meta["sample_wall_time_s"],
        "observed_monotonic_s": server._power_aux_meta["sample_monotonic_s"],
        "from_state": None,
        "to_state": 3,
        "to_state_name": "STATE_SLEEP_CHRG",
        "runstop_cause_raw": 4,
    }]


def test_power_aux_sample_is_not_fresh_until_rpc_callback_runs():
    server = make_server()
    server.power_periph.complete_on_queue = False
    server._queue_power_periph_aux_status()
    server._complete_power_periph_aux_status()
    assert server._power_aux_pending
    assert server._power_aux_meta["sample_seq"] == 0

    server.power_periph.status_aux_reply_sequence += 1
    server._complete_power_periph_aux_status()
    assert not server._power_aux_pending
    assert server._power_aux_meta["sample_seq"] == 1


def test_power_diagnostics_timeout_is_transport_not_schema_error():
    server = make_server()
    server.power_periph.diagnostics_supported = True
    server.power_periph.complete_on_queue = False
    server._queue_power_periph_aux_status()
    server._power_aux_request_started_mono -= 2.1

    server._complete_power_periph_aux_status()

    assert not server._power_aux_pending
    assert server._power_aux_meta["request_timeouts"] == 1
    assert "callback timeout" in server._power_aux_meta["transport_error"]
    assert server._power_diag_meta["request_timeouts"] == 1
    assert server._power_event_meta["request_timeouts"] == 1


def test_server_uses_p14_diagnostics_rpc_without_a_second_aux_request():
    server = make_server()
    server.power_periph.diagnostics_supported = True
    server._power_event_after_sequence = 27

    server._queue_power_periph_aux_status()
    assert server._power_aux_request_kind == "diagnostics"
    assert server.power_periph.requests == 0
    assert server.power_periph.diagnostics_requests == [(27, 16)]

    server._complete_power_periph_aux_status()
    assert server._power_diag_meta["available"]
    assert server._power_event_meta["available"]


def test_firmware_event_sequence_comparison_handles_u32_wrap():
    assert RobotServer._u32_sequence_is_newer(1, 0xFFFFFFFF)
    assert RobotServer._u32_sequence_is_newer(101, 100)
    assert not RobotServer._u32_sequence_is_newer(1, 100)
    assert not RobotServer._u32_sequence_is_newer(100, 100)


def test_server_expands_event_uptime_across_millis_wrap():
    server = make_server()
    server.power_periph.power_diagnostics["uptime_ms"] = (1 << 32) + 1000
    server.power_periph.power_event_log_page.update({
        "newest_sequence": 1,
        "events": [{"sequence": 1, "timestamp_ms": 0xFFFFFF00}],
    })

    server._ingest_power_periph_event_log()
    assert server._power_event_meta["events"][0]["uptime_ms"] == 0xFFFFFF00


def test_server_event_history_is_bounded_to_firmware_ring_capacity():
    server = make_server()
    server._power_events.extend({"sequence": sequence} for sequence in range(1, 66))

    assert len(server._power_events) == 64
    assert server._power_events[0]["sequence"] == 2
    assert server._power_events[-1]["sequence"] == 65


def test_server_discards_stale_cursor_page_after_pimu_reboot():
    server = make_server()
    server._power_diag_last_uptime_ms = 50000
    server._power_event_after_sequence = 27
    server._power_events.append({"sequence": 27})
    server.power_periph.power_diagnostics["uptime_ms"] = 1000
    server.power_periph.power_event_log_page.update({
        "newest_sequence": 30,
        "events": [{"sequence": 28, "timestamp_ms": 900}],
    })

    server._ingest_power_periph_diagnostics()
    server._ingest_power_periph_event_log()

    assert server._power_event_after_sequence == 0
    assert not server._power_event_discard_page
    assert list(server._power_events) == []
    assert server._power_event_meta["events"] == []
    assert server._power_event_meta["reboot_pages_discarded"] == 1
    assert server._power_aux_last_request_mono == float("-inf")

    server.power_periph.power_event_log_page.update({
        "newest_sequence": 2,
        "oldest_sequence": 1,
        "events": [
            {"sequence": 1, "timestamp_ms": 0},
            {"sequence": 2, "timestamp_ms": 500},
        ],
    })
    server._ingest_power_periph_event_log()
    assert server._power_event_after_sequence == 2
    assert [event["sequence"] for event in server._power_event_meta["events"]] == [1, 2]


def test_power_aux_decoder_exposes_state_machine_and_signed_temperatures():
    power = PowerPeriphAux()
    consumed = power._unpack_status_aux(array(
        "B",
        [3, 4, 0x34, 0x12, 29, 30, 0xFF, 27, 0x80],
    ))
    assert consumed == 9
    assert power.status_aux["current_pwr_state_name"] == "STATE_SLEEP_CHRG"
    assert power.status_aux["runstop_cause_raw"] == 4
    assert power.status_aux["motor_sync_cnt"] == 0x1234
    assert power.status_aux["battery_cell_temp_3"] == -1
    assert power.status_aux["battery_mosfet_temp"] == -128
    assert power.status_aux["_valid"]


def test_power_aux_decoder_rejects_short_packets():
    power = PowerPeriphAux()
    assert power._unpack_status_aux(array("B", [0, 0])) == 0
    assert not power.status_aux["_valid"]
    assert "short aux packet" in power.status_aux["_decode_error"]


def test_power_aux_motor_sync_counter_is_unsigned():
    power = PowerPeriphAux()
    power._unpack_status_aux(array(
        "B", [0, 0, 0xFE, 0xFF, 0, 0, 0, 0, 0],
    ))
    assert power.status_aux["motor_sync_cnt"] == 65534


def make_power_diagnostics_packet(uptime_ms=123456, transition_timestamp_ms=120000):
    packet = bytearray()
    packet.extend(struct.pack("<BBH", 1, 0x21, 190))
    packet.extend(struct.pack(
        "<Q5I", uptime_ms, 17, 0x3F, 0x0F, 4, transition_timestamp_ms,
    ))
    packet.extend(struct.pack(
        "<4B2H12B",
        3, 4, 0x2A, 1,
        7, 125,
        1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0,
    ))
    packet.extend(struct.pack(
        "<9I3i2BH5b3B2H4B",
        99, 123400, 56, 95, 2, 3, 0, 4, 0,
        26750, -1800, 2400,
        54, 97, 321,
        28, 29, 30, 27, 34,
        0b010, 0b1100, 0,
        39, 25, 3, 0, 1, 0,
    ))
    packet.extend(struct.pack("<6I", 44, 1000, 2, 1, 1100, 3))
    packet.extend(struct.pack(
        "<6ihHHHIIHHH",
        19750, 5100, 36100, 12050, 19800, 1200,
        4250, 0x1234, 850, 900, 2, 99, 471, 42, 55,
    ))
    assert len(packet) == 190
    return bytes(packet)


def test_p14_rpc_decodes_versioned_bms_and_event_page():
    power = PowerPeriphAux()
    diagnostics = make_power_diagnostics_packet()
    event = struct.pack("<IIHBBhh", 41, 120100, 2, 2, 3, 1, 7)
    header = struct.pack(
        "<BBHBBBBHHIII",
        1, 24, len(diagnostics), 16, 1, 0, 0,
        64, 12, 30, 41, 5,
    )
    reply = array("B", bytes([power.RPC_REPLY_POWER_DIAGNOSTICS]) + header + diagnostics + event)

    power._rpc_power_diagnostics_reply(reply)
    assert power.status_aux["_diagnostics_present"]
    assert power.status_aux["motor_sync_cnt"] == 0x1234
    assert power.power_diagnostics["_valid"]
    assert power.power_diagnostics["power_state_name"] == "STATE_SLEEP_CHRG"
    assert power.power_diagnostics["battery_voltage_v"] == 26.75
    assert power.power_diagnostics["battery_current_a"] == -1.8
    assert power.power_diagnostics["charger_enable_command"]
    assert power.power_diagnostics["charger_expected_active"]
    assert power.power_diagnostics["bms_alarms"]["cell_voltage_high"]
    assert power.power_diagnostics["bms_mos_states"]["charging"]
    assert power.power_diagnostics["bms_sample_age_s"] == 0.056
    assert power.power_diagnostics["bms_protocol_errors"] == 4
    assert power.power_diagnostics["bms_last_failure_rx_len"] == 39
    assert power.power_diagnostics["bms_last_failure_event"] == 25
    assert power.power_diagnostics["bms_last_failure_address"] == 1
    assert power.power_diagnostics["validity"]["bms_extended_sample"]
    assert power.power_diagnostics["loop_time_max_us"] == 900
    assert power.power_diagnostics["loop_overrun_count"] == 2
    assert power.power_diagnostics["diagnostics_rpc_count"] == 99
    assert power.power_diagnostics["diagnostics_reply_bytes"] == 471
    assert power.power_diagnostics["diagnostics_lock_time_us"] == 42
    assert power.power_diagnostics["diagnostics_lock_time_max_us"] == 55
    assert power.power_event_log_page["_valid"]
    assert power.power_event_log_page["dropped_count"] == 5
    assert power.power_event_log_page["events"][0]["sequence"] == 41
    assert power.power_event_log_page["events"][0]["transition_reason"] == "charging_expectation_set"
    assert "charger_enabled" not in power.power_diagnostics
    assert "charger_is_charging" not in power.power_diagnostics


def test_diagnostics_expands_transition_time_across_millis_wrap():
    power = PowerPeriphAux()
    packet = make_power_diagnostics_packet(
        uptime_ms=(1 << 32) + 1000,
        transition_timestamp_ms=0xFFFFFF00,
    )
    assert power._unpack_power_diagnostics(packet) == len(packet)
    assert power.power_diagnostics["power_transition_uptime_ms"] == 0xFFFFFF00


def test_power_diagnostics_is_a_p14_protocol_feature():
    power = PowerPeriphAux()
    power.board_info = {
        "protocol_version": "p13",
        "firmware_version": "hello-pimu2.v0.1.14p13",
    }
    assert not power.supports_power_diagnostics()
    power.board_info = {
        "protocol_version": "p14",
        "firmware_version": "hello-pimu2.v0.1.14p14",
    }
    assert power.supports_power_diagnostics()
    power.board_info["protocol_version"] = "p15"
    assert power.supports_power_diagnostics()


def test_p14_request_uses_dedicated_rpc_and_cursor_layout():
    class CaptureTransport:
        def do_rpc(self, **kwargs):
            self.kwargs = kwargs
            return True

    power = PowerPeriphAux()
    power.hw_valid = True
    power.board_info = {"protocol_version": "p14"}
    power.transport = CaptureTransport()

    assert power.pull_power_diagnostics(
        blocking=False,
        after_sequence=0x78563412,
        max_entries=99,
    )
    assert bytes(power.transport.kwargs["payload"]) == bytes([
        power.RPC_GET_POWER_DIAGNOSTICS,
        power.POWER_DIAGNOSTICS_SCHEMA_VERSION,
        power.POWER_EVENT_LOG_MAX_RPC_ENTRIES,
        0, 0,
        0x12, 0x34, 0x56, 0x78,
    ])
    assert power.transport.kwargs["rpc_callback"] == power._rpc_power_diagnostics_reply


def test_p14_firmware_error_reply_is_not_treated_as_a_sample():
    power = PowerPeriphAux()
    header = struct.pack(
        "<BBHBBBBHHIII",
        1, 24, 0, 16, 0, 0,
        power.POWER_DIAGNOSTICS_REPLY_UNSUPPORTED_SCHEMA,
        0, 0, 0, 0, 0,
    )
    power._rpc_power_diagnostics_reply(array(
        "B", bytes([power.RPC_REPLY_POWER_DIAGNOSTICS]) + header,
    ))
    assert not power.power_diagnostics["_valid"]
    assert not power.power_event_log_page["_valid"]
    assert not power.status_aux["_valid"]
    assert "unsupported_schema" in power.power_diagnostics["_decode_error"]


def test_p14_decoder_rejects_trailing_bytes_not_declared_by_header():
    power = PowerPeriphAux()
    diagnostics = make_power_diagnostics_packet()
    header = struct.pack(
        "<BBHBBBBHHIII",
        1, 24, len(diagnostics), 16, 0, 0, 0,
        64, 0, 0, 0, 0,
    )
    reply = array(
        "B",
        bytes([power.RPC_REPLY_POWER_DIAGNOSTICS]) + header + diagnostics + b"\x00",
    )

    power._rpc_power_diagnostics_reply(reply)

    assert not power.power_diagnostics["_valid"]
    assert "malformed" in power.power_diagnostics["_decode_error"]


def test_decode_error_clears_stale_diagnostics_fields():
    power = PowerPeriphAux()
    assert power._unpack_power_diagnostics(make_power_diagnostics_packet()) == 190
    assert "battery_voltage_v" in power.power_diagnostics

    assert power._unpack_power_diagnostics(array("B", [0, 0])) == 0
    assert not power.power_diagnostics["_valid"]
    assert "battery_voltage_v" not in power.power_diagnostics


def test_p14_decoder_rejects_trailing_bytes_on_error_reply():
    power = PowerPeriphAux()
    header = struct.pack(
        "<BBHBBBBHHIII",
        1, 24, 0, 16, 0, 0,
        power.POWER_DIAGNOSTICS_REPLY_UNSUPPORTED_SCHEMA,
        0, 0, 0, 0, 0,
    )
    reply = array(
        "B",
        bytes([power.RPC_REPLY_POWER_DIAGNOSTICS]) + header + b"\x00",
    )

    power._rpc_power_diagnostics_reply(reply)

    assert not power.power_diagnostics["_valid"]
    assert "malformed" in power.power_diagnostics["_decode_error"]


def test_transport_releases_callback_after_definitive_rpc_failure():
    class FakeCTransport:
        def __init__(self):
            self.discarded = []

        def load_rpc_result(self, _rpc_id):
            return 0

        def discard_rpc_callback(self, rpc_id):
            self.discarded.append(rpc_id)

    transport = Transport.__new__(Transport)
    transport.use_c_backend = True
    transport.transport_c = FakeCTransport()
    transport.status = {"rpc_duration_us": 0}
    transport.n_rate_log = 0
    transport.rate_log = []

    assert transport._load_rpc_result(41, wait_on_result=False) == 0
    assert transport.transport_c.discarded == [41]
