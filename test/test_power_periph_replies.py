"""Ordinary PIMU replies are validated before they touch live status.

The status, motor sync and trigger callbacks indexed reply[0]
without a length check and decoded straight into self.status, so a short
payload raised struct.error half way through and left a mix of old and new
fields with no marker. These tests construct the real protocol classes with a
dummy port name and call the callbacks directly, no hardware.
"""
import array
import struct

from stretch4_body.subsystem.power_periph import (
    PowerPeriph_Protocol_P12,
    PowerPeriph_Protocol_P14,
)

STATUS_FMT = "<17f3fIQHffBfBBH6f?BHfB"
assert struct.calcsize(STATUS_FMT) == 144


def make_status_payload(voltage=14.5, ax=0.25, motor_state=0x10):
    imu = [ax] + [0.0] * 12 + [1.0, 0.0, 0.0, 0.0]  # ax.., qw=1
    return struct.pack(
        STATUS_FMT, *imu,
        voltage, 1.2, 30.0, motor_state, 123456, 3, 0.0, 0.0, 0,
        0.5, 80, 99, 12,
        19.0, 5.0, 36.0, 12.0, 20.0, 1.0,
        True, 0, 500, 0.1, 0,
    )


def make_device(cls=PowerPeriph_Protocol_P14, protocol="p14"):
    device = cls(usb="/dev/null")
    device.board_info["protocol_version"] = protocol
    device.board_info["hardware_id"] = 3
    return device


def snapshot(device):
    nested = {"imu", "transport", "status_aux", "power_diagnostics"}
    return (
        {k: v for k, v in device.status.items() if k not in nested},
        dict(device.imu.status),
    )


def reply(rpc_id, payload=b""):
    return array.array("B", bytes([rpc_id]) + bytes(payload))


def test_exact_length_status_reply_is_committed():
    device = make_device()
    device._rpc_status_reply(reply(device.RPC_REPLY_POWER_PERIPH_STATUS, make_status_payload()))
    assert device.status["voltage"] == 14.5
    assert device.status["bump_event_cnt"] == 3
    assert device.status["us_loop_time"] == 500
    assert device.status["cpu_on_sts"] == 1
    assert device.imu.status["ax"] == 0.25
    assert device.imu.status["timestamp"] == device.status["timestamp"]
    assert device.status["rpc_reply_errors"] == 0
    assert device.status["rpc_reply_last_error"] is None


def test_short_status_reply_is_rejected_without_partial_update():
    device = make_device()
    device._rpc_status_reply(reply(device.RPC_REPLY_POWER_PERIPH_STATUS, make_status_payload()))
    before = snapshot(device)

    truncated = make_status_payload(voltage=99.0, ax=9.0)[:143]
    device._rpc_status_reply(reply(device.RPC_REPLY_POWER_PERIPH_STATUS, truncated))

    after = snapshot(device)
    after[0].pop("rpc_reply_errors"); after[0].pop("rpc_reply_last_error")
    before[0].pop("rpc_reply_errors"); before[0].pop("rpc_reply_last_error")
    assert after == before
    assert device.status["voltage"] == 14.5
    assert device.imu.status["ax"] == 0.25
    assert device.status["rpc_reply_errors"] == 1
    assert "payload 143 bytes, expected 144" in device.status["rpc_reply_last_error"]


def test_long_status_reply_is_rejected():
    device = make_device()
    device._rpc_status_reply(reply(device.RPC_REPLY_POWER_PERIPH_STATUS,
                                   make_status_payload(voltage=99.0) + b"\x00"))
    assert device.status["voltage"] == 0
    assert device.status["rpc_reply_errors"] == 1


def test_empty_and_wrong_id_replies_do_not_raise():
    device = make_device()
    device._rpc_status_reply(array.array("B", []))
    device._rpc_status_reply(reply(device.RPC_REPLY_MOTOR_SYNC, make_status_payload()))
    device._rpc_trigger_reply(array.array("B", []))
    device._rpc_motor_sync_reply(array.array("B", []))
    device._rpc_config_reply(array.array("B", []))
    device._rpc_imu_config_reply(array.array("B", []))
    device._rpc_board_info_reply(array.array("B", []))
    device._rpc_eye_animation_reply(array.array("B", []))
    assert device.status["rpc_reply_errors"] == 8
    assert device.status["voltage"] == 0


def test_unpinned_protocol_rejects_short_payload_on_decode_error():
    device = make_device(PowerPeriph_Protocol_P12, "p12")
    assert device.STATUS_PAYLOAD_BYTES is None
    device._rpc_status_reply(reply(device.RPC_REPLY_POWER_PERIPH_STATUS, make_status_payload()))
    assert device.status["voltage"] == 14.5
    before = snapshot(device)

    device._rpc_status_reply(reply(device.RPC_REPLY_POWER_PERIPH_STATUS,
                                   make_status_payload(voltage=77.0, ax=7.0)[:100]))

    after = snapshot(device)
    assert after[1] == before[1]
    assert device.status["voltage"] == 14.5
    assert device.status["rpc_reply_errors"] == 1
    assert "decode failed" in device.status["rpc_reply_last_error"]


def test_motor_sync_reply_requires_exactly_two_payload_bytes():
    device = make_device()
    device._rpc_motor_sync_reply(reply(device.RPC_REPLY_MOTOR_SYNC, b"\x34\x12"))
    assert device.status["motor_sync_cnt"] == 0x1234

    device._rpc_motor_sync_reply(reply(device.RPC_REPLY_MOTOR_SYNC, b"\x78"))
    device._rpc_motor_sync_reply(reply(device.RPC_REPLY_MOTOR_SYNC, b"\x78\x56\x00"))
    assert device.status["motor_sync_cnt"] == 0x1234
    assert device.status["rpc_reply_errors"] == 2


def test_trigger_reply_requires_exactly_four_payload_bytes():
    device = make_device()
    device._rpc_trigger_reply(reply(device.RPC_REPLY_POWER_PERIPH_TRIGGER, b"\x01\x00\x00\x00"))
    assert device.status["rpc_reply_errors"] == 0
    device._rpc_trigger_reply(reply(device.RPC_REPLY_POWER_PERIPH_TRIGGER, b"\x01\x00"))
    assert device.status["rpc_reply_errors"] == 1
    assert "payload 2 bytes, expected 4" in device.status["rpc_reply_last_error"]
