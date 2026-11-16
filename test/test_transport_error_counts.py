"""Transport error counts, against a fake board on a pseudo terminal."""
import array as arr
import os
import pty
import threading

from stretch4_body.core.transport.cobbs_framing import CobbsFraming
from stretch4_body.core.transport.transport import Transport

PULL_ACK_LAST = 209
PUSH_ACK = 205


def fake_board(fd, replies):
    cobbs = CobbsFraming()
    for reply in replies:
        while os.read(fd, 1) != b'\x00':
            pass
        if reply == 'bad crc':
            frame = cobbs._encode(arr.array('B', [PULL_ACK_LAST, 4, 0x12, 0x34]))
            frame.append(0)
        elif reply == 'wrong ack':
            frame = cobbs.encode_data(arr.array('B', [PUSH_ACK, 4]))
        elif reply == 'good':
            frame = cobbs.encode_data(arr.array('B', [PULL_ACK_LAST, 4]))
        else:
            continue
        os.write(fd, bytes(frame))


def test_errors_and_round_trip_are_counted_in_status():
    master, slave = pty.openpty()
    t = Transport(os.ttyname(slave), default_backend=Transport.BACKEND_C_SERIAL, qid=7)
    assert t.transport_c.startup() and t.transport_c.start_nonblocking_thread()
    replies = ['bad crc', 'wrong ack', 'silence', 'good']
    board = threading.Thread(target=fake_board, args=(master, replies), daemon=True)
    board.start()

    for _ in replies:
        t.do_rpc(blocking=True, is_push=False, payload=arr.array('B', [3]), rpc_callback=lambda reply: None)
    t.transport_c.stop()

    assert t.status['crc_errors'] == 1
    assert t.status['framing_errors'] == 1
    assert t.status['timeouts'] == 1
    assert t.status['rpc_duration_max_us'] > 0
