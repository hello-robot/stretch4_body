"""Line sensor firmware version: the '?' query and its status line.

hello-pixart-j3 v0.1.1p0 and later answer '?' with
{"status":{"fw":...,"up":...,"rcause":...,...}} between reports. The reader
asks after every open, keeps the version, uptime and reset cause, and must
not let the line disturb the frame parser. Older firmware never answers: the
reader stops after FIRMWARE_QUERY_TRIES and says no_reply, without errors.
"""
import json

import serial as _serial

from stretch4_body.subsystem.line_sensor.pixart_j3_reader import PixartJ3Reader
from test_line_sensor_control import FakeSerial, frame, make_reader

# What hello-pixart-j3 v0.1.2p1 prints (UsbWatch::PrintStatus).
STATUS = (b'{"status":{"fw":"hello-pixart-j3.v0.1.2p1","up":123456,"rcause":64,'
          b'"fid":812,"fp":4872,"srec":0,"rx":1,"open":1}}\n')


class Clock:
    def __init__(self, monkeypatch, t=1000.0):
        self.t = t
        monkeypatch.setattr('stretch4_body.subsystem.line_sensor.pixart_j3_reader.time.time',
                            lambda: self.t)


def opened_reader(monkeypatch):
    """A reader as startup() leaves it, on a FakeSerial."""
    port = FakeSerial()
    monkeypatch.setattr(_serial, 'Serial', lambda **kw: port)
    r = PixartJ3Reader(bus_sensor_map=[[1, 0], [3, 2], [5, 4]], flip_range_ordering=False,
                       report_num=8)
    assert r.startup()
    return r, port


def test_before_open_the_version_is_unknown_and_nothing_is_sent():
    r = make_reader()
    assert r.firmware_status()['state'] == 'not_open'
    assert r.firmware_status()['version'] is None


def test_query_goes_out_after_the_first_read_flush(monkeypatch):
    clock = Clock(monkeypatch)
    r, port = opened_reader(monkeypatch)
    assert port.written == b''
    r.step()
    # Flushed first, then asked, so the answer cannot be flushed away.
    assert port.flush_count == 2
    assert port.written == b'?'
    assert r.firmware_status()['state'] == 'waiting'
    clock.t += 0.5
    r.step()
    assert port.written == b'?'


def test_status_line_between_frames_is_kept_and_frames_still_decode(monkeypatch):
    clock = Clock(monkeypatch)
    r, port = opened_reader(monkeypatch)
    r.step()
    port.feed(frame(1) + STATUS + frame(2))
    r.step()
    fw = r.firmware_status()
    assert fw['state'] == 'reported'
    assert fw['version'] == 'hello-pixart-j3.v0.1.2p1'
    assert fw['uptime_ms'] == 123456
    assert fw['reset_cause_raw'] == 64
    assert fw['received_time'] == clock.t
    assert fw['queries'] == 1
    assert r.status['decode_errors'] == 0
    assert r.status['sensor_0']['frame_id'] == 2
    # Answered: no more queries.
    clock.t += 60
    r.step()
    assert port.written == b'?'


def test_status_line_split_across_reads(monkeypatch):
    Clock(monkeypatch)
    r, port = opened_reader(monkeypatch)
    r.step()
    data = frame(1) + STATUS + frame(2)
    cut = len(frame(1)) + 30
    for chunk in (data[:cut], data[cut:cut + 7], data[cut + 7:]):
        port.feed(chunk)
        r.step()
    assert r.firmware_status()['version'] == 'hello-pixart-j3.v0.1.2p1'
    assert r.status['decode_errors'] == 0
    assert r.status['sensor_0']['frame_id'] == 2


def test_older_firmware_gets_three_tries_then_no_reply_without_errors(monkeypatch, caplog):
    clock = Clock(monkeypatch)
    r, port = opened_reader(monkeypatch)
    for i in range(40):
        port.feed(frame(i + 1))
        r.step()
        clock.t += 0.25
    assert port.written == b'???'
    fw = r.firmware_status()
    assert fw['state'] == 'no_reply' and fw['version'] is None and fw['queries'] == 3
    assert r.status['decode_errors'] == 0
    assert not [rec for rec in caplog.records if rec.levelname in ('WARNING', 'ERROR')]
    clock.t += 600
    r.step()
    assert port.written == b'???'


def test_a_reopen_asks_again(monkeypatch):
    Clock(monkeypatch)
    r, port = opened_reader(monkeypatch)
    r.step()
    port.feed(STATUS)
    r.step()
    assert r.firmware_status()['state'] == 'reported'
    port.raise_on_read = _serial.SerialException('unplugged')
    port.feed(frame(1))
    r.step()
    assert r.is_valid is False
    fresh = FakeSerial()
    monkeypatch.setattr(_serial, 'Serial', lambda **kw: fresh)
    r._reopen_at = 0.0
    r.step()
    assert r.is_valid
    assert r.firmware_status()['state'] == 'waiting'
    assert r.firmware_status()['version'] is None
    r.step()
    assert fresh.written == b'?'


def test_a_write_timeout_is_a_lost_try_not_a_reader_failure(monkeypatch):
    Clock(monkeypatch)
    r, port = opened_reader(monkeypatch)

    def timeout(data):
        raise _serial.SerialTimeoutException('write timeout')
    port.write = timeout
    r.step()
    assert r.is_valid
    assert r.firmware_status()['queries'] == 1
    port.feed(frame(1))
    r.step()
    assert r.status['sensor_0']['frame_id'] == 1


def test_no_query_while_streaming_is_paused(monkeypatch):
    clock = Clock(monkeypatch)
    r, port = opened_reader(monkeypatch)
    r.set_streaming(False)
    for _ in range(10):
        r.step()
        clock.t += 1.0
    assert port.written == b''
    r.set_streaming(True)
    r.step()
    assert port.written == b'?'


def test_status_without_a_version_string_is_ignored():
    r = make_reader()
    for line in ('{"status":{"up":5}}', '{"status":{"fw":7}}'):
        assert r.process_json_line(line) is False
    assert r.firmware_status()['version'] is None
    assert r.status['decode_errors'] == 0


def test_reset_line_is_still_not_a_version():
    r = make_reader()
    assert r.process_json_line(json.dumps({'reset': 'host-command'})) is False
    assert r.firmware_status()['version'] is None
