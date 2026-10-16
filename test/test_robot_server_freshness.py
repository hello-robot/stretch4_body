"""BMS and ESP freshness track host time and go stale with the diagnostics.

Audit finding 5: bms_freshness and esp_freshness were classified once from the
firmware's snapshot-time ages and never touched again, so after the
diagnostics stopped arriving the server kept publishing 'fresh' next to a
transport timeout and available=True. Uses the fake PowerPeriph server
fixture from test_robot_server_power_aux, no hardware.
"""
from test_robot_server_power_aux import make_server


def sampled_server():
    server = make_server()
    server.power_periph.diagnostics_supported = True
    server._power_diag_meta.update({
        "bms_freshness": "unknown", "esp_freshness": "unknown",
        "bms_age_s": None, "esp_age_s": None,
        "bms_freshness_at_sample": "unknown", "esp_freshness_at_sample": "unknown",
    })
    server._power_diag_fw_ages = (None, None)
    server._queue_power_periph_aux_status()
    server._complete_power_periph_aux_status()
    assert server._power_diag_meta["available"]
    return server


def test_fresh_sample_classifies_from_firmware_age():
    server = sampled_server()
    meta = server._power_diag_meta
    assert meta["bms_freshness"] == "fresh"
    assert meta["esp_freshness"] == "fresh"
    assert meta["bms_freshness_at_sample"] == "fresh"
    assert 0.1 <= meta["bms_age_s"] < 0.2


def test_freshness_ages_with_host_monotonic_time():
    server = sampled_server()
    meta = server._power_diag_meta

    server._power_diag_last_sample_mono -= 4.0
    server._refresh_power_freshness()
    assert meta["bms_freshness"] == "warning"   # 0.1 + 4.0 >= 3.0
    assert meta["esp_freshness"] == "warning"   # 0.1 + 4.0 >= 2.0
    assert 4.0 < meta["bms_age_s"] < 4.3

    server._power_diag_last_sample_mono -= 6.0
    server._refresh_power_freshness()
    assert meta["bms_freshness"] == "fault"     # >= 10.0
    assert meta["esp_freshness"] == "fault"     # >= 5.0
    assert meta["bms_freshness_at_sample"] == "fresh"


def test_timeout_marks_diagnostics_unavailable_and_freshness_stale():
    server = sampled_server()
    meta = server._power_diag_meta
    server._power_aux_last_request_mono = float("-inf")
    server.power_periph.complete_on_queue = False
    server._queue_power_periph_aux_status()
    server._power_aux_request_started_mono -= 2.1

    server._complete_power_periph_aux_status()

    assert meta["available"] is False
    assert meta["transport_error"] == "diagnostics RPC callback timeout"
    assert meta["bms_freshness"] == "stale"
    assert meta["esp_freshness"] == "stale"
    assert meta["bms_age_s"] is not None
    # The last sample and its own classification survive for the consumer.
    assert meta["bms_freshness_at_sample"] == "fresh"
    assert server.power_periph.power_diagnostics["_valid"]
    assert meta["sample_seq"] == 1


def test_decode_failure_after_a_sample_reads_stale():
    server = sampled_server()
    server._power_aux_last_request_mono = float("-inf")
    server.power_periph.power_diagnostics["_valid"] = False
    server.power_periph.power_diagnostics["_decode_error"] = "short packet"
    server._queue_power_periph_aux_status()
    server._complete_power_periph_aux_status()
    meta = server._power_diag_meta
    assert meta["available"] is False
    assert meta["schema_error"] == "short packet"
    assert meta["bms_freshness"] == "stale"


def test_freshness_stays_unknown_before_any_sample():
    server = make_server()
    server.power_periph.diagnostics_supported = True
    server._power_diag_meta.update({"bms_freshness": "unknown", "esp_freshness": "unknown"})
    server._power_diag_fw_ages = (None, None)
    server.power_periph.complete_on_queue = False
    server._queue_power_periph_aux_status()
    server._power_aux_request_started_mono -= 2.1
    server._complete_power_periph_aux_status()
    assert server._power_diag_meta["bms_freshness"] == "unknown"
    assert server._power_diag_meta["available"] is False


def test_missing_firmware_age_is_unavailable_while_diagnostics_flow():
    server = make_server()
    server.power_periph.diagnostics_supported = True
    server.power_periph.power_diagnostics["bms_sample_age_s"] = None
    server._power_diag_meta.update({"bms_freshness": "unknown", "esp_freshness": "unknown"})
    server._power_diag_fw_ages = (None, None)
    server._queue_power_periph_aux_status()
    server._complete_power_periph_aux_status()
    meta = server._power_diag_meta
    assert meta["bms_freshness"] == "unavailable"
    assert meta["bms_age_s"] is None
    assert meta["esp_freshness"] == "fresh"
