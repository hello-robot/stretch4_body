"""PIMU eye frames: RPC 33 frame mode (firmware v0.1.52p14), fake transport, no hardware."""
import array as arr
import struct

import pytest

from test_power_periph_easter_egg import PowerPeriph, pimu  # noqa: F401

LEFT = [(i, 2 * i, 255 - i) for i in range(10)]
RIGHT = [(100 + i, 0, i) for i in range(10)]


def frame_reply(status, applied):
    return [PowerPeriph.RPC_REPLY_SET_EYE_PER_EYE] + list(struct.pack('<BH', status, applied))


@pytest.fixture
def pimu52(pimu):
    pimu.board_info['firmware_version'] = 'hello-pimu2.v0.1.52p14'
    return pimu


def test_request_is_68_bytes_in_firmware_layout(pimu52):
    seq = pimu52.set_eye_frame(LEFT, RIGHT, brightness=(200, 7), hold_ms=300)
    assert seq == 1
    pimu52.transport.replies.append(frame_reply(0, 1))
    assert pimu52.push_command() is True
    sent = bytes(pimu52.transport.payloads[-1])
    assert len(sent) == 68
    rid, cmd, s, hold, bl, br = struct.unpack('<BBHHBB', sent[:8])
    assert (rid, cmd, s, hold, bl, br) == (33, 0x80, 1, 300, 200, 7)
    pixels = list(sent[8:])
    assert pixels[:3] == [0, 0, 255]
    assert pixels[27:30] == [9, 18, 246]
    assert pixels[30:33] == [100, 0, 0]
    assert pixels[57:60] == [109, 0, 9]
    assert pimu52.status['eye_frame'] == {'supported': True, 'sent': 1, 'last_status': 0, 'applied_sequence': 1}


def test_sequence_counts_from_1_and_wraps_past_0(pimu52):
    assert [pimu52.set_eye_frame(LEFT, RIGHT) for _ in range(3)] == [1, 2, 3]
    pimu52._eye_frame_sequence = 0xFFFE
    assert pimu52.set_eye_frame(LEFT, RIGHT) == 0xFFFF
    assert pimu52.set_eye_frame(LEFT, RIGHT) == 1
    assert pimu52.set_eye_frame(LEFT, RIGHT, sequence=0) == 0


def test_only_the_newest_queued_frame_goes_out_once(pimu52):
    pimu52.set_eye_frame(LEFT, RIGHT)
    pimu52.set_eye_frame(RIGHT, LEFT)
    pimu52.transport.replies.append(frame_reply(0, 2))
    pimu52.push_command()
    pimu52.push_command()
    assert len(pimu52.transport.payloads) == 1
    assert pimu52.transport.payloads[0][2] == 2


def test_refusal_keeps_previous_applied_sequence_and_warns(pimu52, caplog):
    pimu52.set_eye_frame(LEFT, RIGHT)
    pimu52.transport.replies.append(frame_reply(3, 41))
    pimu52.push_command()
    assert pimu52.status['eye_frame']['last_status'] == 3
    assert pimu52.status['eye_frame']['applied_sequence'] == 41
    assert any('hold out of range (status 3)' in r.getMessage() for r in caplog.records)


@pytest.mark.parametrize('fw', ['hello-pimu2.v0.1.51p14', 'hello-pimu2.v0.1.36p14', 'hello-pimu2.v0.2.0p13', ''])
def test_older_firmware_sends_nothing(pimu, fw):
    pimu.board_info['firmware_version'] = fw
    pimu.set_eye_frame(LEFT, RIGHT)
    assert pimu.push_command() is True
    assert pimu.transport.payloads == []
    assert pimu.status['eye_frame']['supported'] is False
    assert pimu.status['eye_frame']['sent'] == 0


def test_per_eye_reply_means_no_frame_mode(pimu52):
    # A board that reads the request as per-eye answers with the 1 byte status 1.
    pimu52.set_eye_frame(LEFT, RIGHT)
    pimu52.transport.replies.append([PowerPeriph.RPC_REPLY_SET_EYE_PER_EYE, 1])
    pimu52.push_command()
    assert pimu52.status['eye_frame']['supported'] is False
    assert pimu52.status['eye_frame']['applied_sequence'] is None


def test_bad_reply_length_is_counted_not_decoded(pimu52):
    errors = pimu52.status['rpc_reply_errors']
    pimu52.set_eye_frame(LEFT, RIGHT)
    pimu52.transport.replies.append([PowerPeriph.RPC_REPLY_SET_EYE_PER_EYE, 0, 1])
    pimu52.push_command()
    assert pimu52.status['rpc_reply_errors'] == errors + 1
    assert pimu52.status['eye_frame']['applied_sequence'] is None


@pytest.mark.parametrize('kwargs', [
    dict(left=LEFT[:9], right=RIGHT),
    dict(left=LEFT, right=RIGHT + [(0, 0, 0)]),
    dict(left=[(0, 0)] * 10, right=RIGHT),
    dict(left=[(0, 0, 256)] * 10, right=RIGHT),
    dict(left=[(0, -1, 0)] * 10, right=RIGHT),
    dict(left=[(0, 0.5, 0)] * 10, right=RIGHT),
    dict(left=LEFT, right=RIGHT, brightness=(255,)),
    dict(left=LEFT, right=RIGHT, brightness=(0, 256)),
    dict(left=LEFT, right=RIGHT, hold_ms=49),
    dict(left=LEFT, right=RIGHT, hold_ms=2001),
    dict(left=LEFT, right=RIGHT, sequence=0x10000),
])
def test_bad_arguments_raise_and_queue_nothing(pimu52, kwargs):
    with pytest.raises(ValueError):
        pimu52.set_eye_frame(**kwargs)
    pimu52.push_command()
    assert pimu52.transport.payloads == []


def test_hold_bounds_accepted(pimu52):
    pimu52.set_eye_frame(LEFT, RIGHT, hold_ms=50)
    pimu52.set_eye_frame(LEFT, RIGHT, hold_ms=2000)


@pytest.mark.parametrize('fw, supported', [('hello-pimu2.v0.1.52p14', True), ('hello-pimu2.v0.1.51p14', False),
                                           ('hello-pimu2.v0.1.53p14', True), ('hello-pimu2.v0.2.0p13', False)])
def test_board_info_sets_supported_at_startup(pimu, fw, supported):
    raw = b'PowerPeriph.4'.ljust(20, b'\0') + fw.encode().ljust(30, b'\0')
    pimu._unpack_board_info(arr.array('B', raw))
    assert pimu.board_info['firmware_version'] == fw
    assert pimu.status['eye_frame']['supported'] is supported
