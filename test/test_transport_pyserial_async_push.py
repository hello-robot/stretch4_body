"""The async v1 push handler must decode the ACK and forward its payload.

Review of 2026-10-16, finding A: AsyncTransactionHandler.do_push_transaction_v1
unpacked two values from receiveFramedData() (which returns three) and then read
an unpopulated frame_buf_in, so every valid ACK raised after the frame was sent.
These tests run the real handler against a fake serial port that serves
COBS-encoded frames, no device.
"""
import array as arr
import asyncio
import logging
import threading

from stretch4_body.core.transport import cobbs_framing
from stretch4_body.core.transport.transport_pyserial import (
    RPC_V1_PUSH_ACK,
    AsyncTransactionHandler,
)
from stretch4_body.core.transport.transport_util import RPC_TRANSPORT_VERSION_1

RPC_ID = 40


def encode(frame):
    return bytes(cobbs_framing.CobbsFraming().encode_data(arr.array('B', frame)))


class FakeSerial:
    def __init__(self, replies):
        self.replies = list(replies)
        self.pending = b""
        self.written = []
        self.resets = 0

    async def write_async(self, data):
        self.written.append(bytes(data))
        if self.replies:
            self.pending += self.replies.pop(0)

    def inWaiting(self):
        return len(self.pending)

    async def read_async(self, n):
        out, self.pending = self.pending[:n], self.pending[n:]
        return out

    def reset_output_buffer(self):
        self.resets += 1

    def reset_input_buffer(self):
        self.resets += 1


def make_handler(replies):
    ser = FakeSerial(replies)
    handler = AsyncTransactionHandler("/dev/null", ser, logging.getLogger("test"),
                                      threading.Lock())
    handler.version = RPC_TRANSPORT_VERSION_1
    handler.timeout = 0.2
    return handler, ser


def test_valid_ack_runs_callback_with_decoded_payload():
    handler, ser = make_handler([encode([RPC_V1_PUSH_ACK, RPC_ID, 0])])
    replies = []
    ok = asyncio.run(handler.do_rpc(push=True, payload=arr.array('B', [RPC_ID, 1]),
                                    reply_callback=replies.append))
    assert ok is True
    assert len(ser.written) == 1
    assert [list(r) for r in replies] == [[RPC_ID, 0]]
    assert handler.status['read_error'] == 0


def test_wrong_ack_code_fails_without_callback():
    handler, ser = make_handler([encode([RPC_V1_PUSH_ACK + 1, RPC_ID, 0])])
    replies = []
    ok = asyncio.run(handler.do_rpc(push=True, payload=arr.array('B', [RPC_ID, 1]),
                                    reply_callback=replies.append))
    assert ok is False
    assert replies == []
    assert handler.status['read_error'] == 1
    assert ser.resets == 2


def test_bad_crc_fails_without_callback():
    frame = bytearray(encode([RPC_V1_PUSH_ACK, RPC_ID, 0]))
    frame[-2] ^= 0x01  # corrupt a CRC byte, keep the delimiter
    handler, _ = make_handler([bytes(frame)])
    replies = []
    ok = asyncio.run(handler.do_rpc(push=True, payload=arr.array('B', [RPC_ID, 1]),
                                    reply_callback=replies.append))
    assert ok is False
    assert replies == []
    assert handler.status['read_error'] == 1
