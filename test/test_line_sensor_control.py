"""runtime on/off and self-recovery.

Both features fail in ways that look like something else, which is why they
are tested rather than eyeballed:

  * a disabled sensor that still counted as missing would be reported DEAD,
    sending someone to check a cable for a fault they created themselves;
  * a reader that marks itself invalid without closing the port can never
    reopen it (the port is opened exclusive=True), so one stray exception
    used to kill the subsystem until the next reboot -- silently, while the
    process stayed alive and system check kept passing.
"""

import json

import numpy as np
import pytest

from stretch4_body.subsystem.line_sensor import protocol
from stretch4_body.subsystem.line_sensor.pixart_j3_reader import PixartJ3Reader

BUS_MAP = [[1, 0], [3, 2], [5, 4]]
NBINS = 8


class FakeSerial:
    """Enough of serial.Serial to drive the reader, plus fault injection."""

    def __init__(self, chunks=()):
        self.is_open = True
        self._buf = b''.join(chunks)
        self.raise_on_read = None
        self.closed_count = 0
        self.total_read = 0
        self.written = b''
        self.flush_count = 0

    @property
    def in_waiting(self):
        return len(self._buf)

    def read(self, n):
        if self.raise_on_read is not None:
            exc, self.raise_on_read = self.raise_on_read, None
            raise exc
        out, self._buf = self._buf[:n], self._buf[n:]
        self.total_read += len(out)
        return out

    def feed(self, data):
        self._buf += data

    def write(self, data):
        # startup() sends the '?' firmware query here.
        self.written += data
        return len(data)

    def reset_input_buffer(self):
        self._buf = b''
        self.flush_count += 1

    def close(self):
        self.is_open = False
        self.closed_count += 1


def report(frame_id, bus, dev, value_mm=200.0):
    key = 'distances%d%d' % (bus, dev)
    return (json.dumps({'frameId': frame_id, key: [value_mm] * NBINS})
            + '\n').encode()


def frame(frame_id, value_mm=200.0, skip=()):
    out = []
    for bus in (1, 2, 3):
        for dev in (0, 1):
            if BUS_MAP[bus - 1][dev] in skip:
                continue
            out.append(report(frame_id, bus, dev, value_mm))
    return b''.join(out)


def make_reader(chunks=()):
    r = PixartJ3Reader(bus_sensor_map=BUS_MAP, flip_range_ordering=False,
                       report_num=NBINS)
    r.ser = FakeSerial(chunks)
    r.is_valid = True
    return r


# ---------------------------------------------------------------------------
# streaming on/off
# ---------------------------------------------------------------------------

def test_reader_starts_streaming_with_every_sensor_enabled():
    r = make_reader()
    assert r.streaming is True
    assert r.disabled == set()
    assert r.health()['disabled_sensors'] == []


def test_paused_reader_still_drains_the_port():
    """The whole point: a paused reader that stopped reading would let the
    kernel buffer fill and hand back a corrupt part-report on resume."""
    r = make_reader([frame(1)])
    r.set_streaming(False)
    r.step()
    assert r.ser.in_waiting == 0          # consumed
    assert r.ser.total_read > 0
    assert r.status['sensor_0']['frame_id'] == 0   # but not decoded


def test_pause_then_resume_produces_clean_frames():
    r = make_reader([frame(1)])
    r.set_streaming(False)
    r.step()                               # frame 1 discarded
    r.set_streaming(True)
    r.ser.feed(frame(2, value_mm=250.0))
    r.step()
    for i in range(6):
        s = r.status['sensor_%d' % i]
        assert s['frame_id'] == 2
        assert np.allclose(s['ranges'], 0.250)


def test_resume_discards_a_half_read_line():
    """Bytes are dropped mid-report while paused; resuming must resync rather
    than glue the tail of a discarded report onto the next one."""
    r = make_reader()
    r.ser.feed(b'{"frameId": 1, "distances10": [200.0, 200.0')
    r.step()
    assert r.json_line != ''               # mid-line
    r.set_streaming(False)
    r.set_streaming(True)
    assert r.json_line == ''               # dropped, will resync on a newline
    r.ser.feed(frame(7))
    r.step()
    assert r.status['sensor_0']['frame_id'] == 7
    assert r.status['decode_errors'] == 0


def test_health_reports_streaming_state():
    r = make_reader()
    assert r.health()['streaming'] is True
    r.set_streaming(False)
    assert r.health()['streaming'] is False
    assert r.health()['port_open'] is True   # paused is not disconnected


def test_set_streaming_is_idempotent():
    r = make_reader([frame(1)])
    r.step()
    r.set_streaming(True)                  # already on: must not resync
    assert r.last_frame_id == 1


# ---------------------------------------------------------------------------
# per-sensor enable/disable
# ---------------------------------------------------------------------------

def test_disabled_sensor_is_skipped_before_json_parsing(monkeypatch):
    """Skipping after the parse would save nothing -- json.loads is the cost."""
    r = make_reader()
    r.set_sensor_enabled('sensor_3', False)
    parsed = []
    real = json.loads
    monkeypatch.setattr(json, 'loads', lambda s, **k: (parsed.append(s), real(s))[1])
    r.ser.feed(frame(1))
    r.step()
    assert len(parsed) == 5                        # not 6
    # Derive the key from the map rather than hardcoding it: with
    # [[1,0],[3,2],[5,4]] sensor_3 is distances20, which is exactly the kind
    # of thing a hand-written constant gets wrong.
    key = next(k for k, i in r.key_table.items() if i == 3)
    assert not any(key in p for p in parsed)


def test_disabled_sensor_is_blanked_not_left_stale():
    r = make_reader([frame(1)])
    r.step()
    assert len(r.status['sensor_3']['ranges']) == NBINS
    r.set_sensor_enabled('sensor_3', False)
    s = r.status['sensor_3']
    assert len(s['ranges']) == 0 and len(s['codes']) == 0
    assert s['enabled'] is False


def test_disabled_sensor_is_not_reported_dead():
    """Off on purpose is not a fault."""
    r = make_reader()
    r.set_sensor_enabled('sensor_2', False)
    for f in range(1, 12):                 # well past DEAD_AFTER_MISSED_FRAMES
        r.ser.feed(frame(f, skip={2}))
        r.step()
    assert r.health()['sensors_dead'] == []
    assert r.health()['disabled_sensors'] == ['sensor_2']


def test_a_genuinely_missing_sensor_is_still_reported_dead():
    """The disabled path must not blind the liveness check."""
    r = make_reader()
    for f in range(1, 12):
        r.ser.feed(frame(f, skip={2}))     # absent, but NOT disabled
        r.step()
    assert 'sensor_2' in r.health()['sensors_dead']


def test_frame_accounting_tolerates_a_disabled_sensor():
    r = make_reader()
    r.set_sensor_enabled('sensor_5', False)
    for f in range(1, 6):
        r.ser.feed(frame(f, skip={5}))
        r.step()
    assert r.status['not_six_sensors_err'] == 0
    assert r.status['sensor_0']['frame_id'] == 5


def test_re_enabling_restores_decoding():
    r = make_reader()
    r.set_sensor_enabled('sensor_1', False)
    r.ser.feed(frame(1))
    r.step()
    assert len(r.status['sensor_1']['ranges']) == 0
    r.set_sensor_enabled('sensor_1', True)
    r.ser.feed(frame(2))
    r.step()
    assert r.status['sensor_1']['frame_id'] == 2
    assert r.status['sensor_1']['enabled'] is True


def test_bad_sensor_name_raises():
    r = make_reader()
    for bad in ('sensor_9', 'nonsense', 'sensor_'):
        with pytest.raises(ValueError):
            r.set_sensor_enabled(bad, False)


# ---------------------------------------------------------------------------
# auto-recovery
# ---------------------------------------------------------------------------

import serial as _serial


def test_serial_error_closes_the_port():
    """The fd leak that made failures permanent: opened exclusive=True, a port
    left open blocks every reopen with 'device busy'."""
    r = make_reader()
    r.ser.feed(frame(1))
    r.ser.raise_on_read = _serial.SerialException('device disconnected')
    r.step()
    assert r.is_valid is False
    assert r.ser.closed_count == 1


def test_unexpected_error_also_closes_the_port():
    """This branch used to mark the reader invalid WITHOUT closing."""
    r = make_reader()
    r.ser.feed(frame(1))
    r.ser.raise_on_read = RuntimeError('something else entirely')
    r.step()
    assert r.is_valid is False
    assert r.ser.closed_count == 1


def test_reader_reopens_itself_and_counts_the_restart(monkeypatch):
    r = make_reader()
    r.ser.feed(frame(1))
    r.ser.raise_on_read = _serial.SerialException('boom')
    r.step()
    assert r.is_valid is False

    opened = []
    fresh = FakeSerial()
    def fake_open(**kw):
        opened.append(kw)
        return fresh
    monkeypatch.setattr(_serial, 'Serial', fake_open)

    r._reopen_at = 0.0                     # pretend the backoff elapsed
    r.step()                               # this step performs the reopen
    assert r.is_valid is True
    assert r.status['reader_restarts'] == 1
    assert opened and opened[0]['exclusive'] is True

    # Feed only now: opening flushes the port, and so does the first read
    # after it, so anything queued before that is deliberately discarded
    # rather than parsed mid-report.
    r.step()
    assert fresh.flush_count == 2
    fresh.feed(frame(9))
    r.step()                               # and data flows again
    assert r.status['sensor_0']['frame_id'] == 9


def test_reopen_is_not_attempted_before_the_backoff_elapses(monkeypatch):
    r = make_reader()
    r.ser.raise_on_read = _serial.SerialException('boom')
    r.ser.feed(frame(1))
    r.step()
    calls = []
    monkeypatch.setattr(_serial, 'Serial',
                        lambda **kw: calls.append(kw) or FakeSerial())
    r._reopen_at = 1e18                    # far future
    for _ in range(50):
        r.step()
    assert calls == []                     # not spinning on the port


def test_backoff_grows_and_is_capped(monkeypatch):
    r = make_reader()
    monkeypatch.setattr(_serial, 'Serial',
                        lambda **kw: (_ for _ in ()).throw(
                            _serial.SerialException('still gone')))
    r._fail('injected')
    seen = []
    for _ in range(20):
        r._reopen_at = 0.0
        r._try_reopen()
        seen.append(r._reopen_backoff_s)
    assert seen[0] < seen[1]                               # grows
    assert max(seen) <= PixartJ3Reader.REOPEN_BACKOFF_MAX_S
    assert r.status['reader_restarts'] == 0                # never succeeded


def test_backoff_resets_after_a_successful_reopen(monkeypatch):
    r = make_reader()
    monkeypatch.setattr(_serial, 'Serial',
                        lambda **kw: (_ for _ in ()).throw(
                            _serial.SerialException('gone')))
    r._fail('injected')
    for _ in range(5):
        r._reopen_at = 0.0
        r._try_reopen()
    assert r._reopen_backoff_s > PixartJ3Reader.REOPEN_BACKOFF_MIN_S
    monkeypatch.setattr(_serial, 'Serial', lambda **kw: FakeSerial())
    r._reopen_at = 0.0
    assert r._try_reopen() is True
    assert r._reopen_backoff_s == PixartJ3Reader.REOPEN_BACKOFF_MIN_S


def test_recovery_survives_a_missing_device_node(monkeypatch):
    """On a USB replug the /dev node itself disappears, which raises
    FileNotFoundError -- not a SerialException."""
    r = make_reader()
    r._fail('injected')
    monkeypatch.setattr(_serial, 'Serial',
                        lambda **kw: (_ for _ in ()).throw(
                            FileNotFoundError('/dev/hello-pixart-j3')))
    r._reopen_at = 0.0
    assert r._try_reopen() is False        # handled, not raised
    assert r.is_valid is False


def test_losing_the_port_reports_every_sensor_dead():
    """Caught on the robot: during a real USB dropout dead_sensors() stayed
    empty, because liveness is decided in process_frame() and no frames were
    arriving. All six kept publishing their last scan, looking healthy."""
    r = make_reader([frame(1)])
    r.step()
    assert r.health()['sensors_dead'] == []
    r.ser.raise_on_read = _serial.SerialException('cable pulled')
    r.ser.feed(frame(2))
    r.step()
    assert r.health()['sensors_dead'] == ['sensor_%d' % i for i in range(6)]


def test_a_disabled_sensor_is_not_called_dead_when_the_port_drops():
    """Still not a fault -- it was switched off on purpose."""
    r = make_reader([frame(1)])
    r.step()
    r.set_sensor_enabled('sensor_4', False)
    r.ser.raise_on_read = _serial.SerialException('cable pulled')
    r.ser.feed(frame(2))
    r.step()
    assert 'sensor_4' not in r.health()['sensors_dead']
    assert len(r.health()['sensors_dead']) == 5


def test_recovery_clears_the_dead_list():
    r = make_reader([frame(1)])
    r.step()
    r.ser.raise_on_read = _serial.SerialException('cable pulled')
    r.ser.feed(frame(2))              # else read() is never reached
    r.step()
    assert r.health()['sensors_dead']
    r.ser = FakeSerial([frame(3), frame(4), frame(5), frame(6), frame(7)])
    r.is_valid = True
    r._reset_framing()
    for _ in range(5):
        r.step()
    assert r.health()['sensors_dead'] == []


def test_recovery_handles_the_board_restarting_its_frame_counter():
    """A real unplug cuts VBUS, so the sensor board reboots and frameId
    restarts near zero 
    """
    r = make_reader([frame(50_000)])
    r.step()
    assert r.last_frame_id == 50_000
    errs = r.status['frame_advance_err']

    r.ser.raise_on_read = _serial.SerialException('cable pulled')
    r.ser.feed(frame(50_001))
    r.step()
    assert r.is_valid is False

    # Board came back and is counting from 1 again.
    r.ser = FakeSerial([frame(1), frame(2), frame(3)])
    r.is_valid = True
    r._reset_framing()
    for _ in range(3):
        r.step()
    assert r.status['sensor_0']['frame_id'] == 3
    assert r.status['frame_advance_err'] == errs   # the jump is not a fault
    assert r.health()['sensors_dead'] == []


def test_streaming_state_survives_a_reconnect(monkeypatch):
    """Recovery must not quietly turn the sensors back on."""
    r = make_reader()
    r.set_streaming(False)
    r._fail('injected')
    monkeypatch.setattr(_serial, 'Serial', lambda **kw: FakeSerial())
    r._reopen_at = 0.0
    r._try_reopen()
    assert r.is_valid is True
    assert r.streaming is False
    assert r.health()['streaming'] is False


def test_sensors_dead_has_exactly_one_home():
    r = make_reader()
    assert 'sensors_dead' not in r.status
    assert 'sensors_dead' not in PixartJ3Reader.HEALTH_KEYS
    assert r.health()['sensors_dead'] == ['sensor_%d' % i for i in range(6)]


def test_health_reports_dead_sensors_with_no_frame_to_carry_them():
    """The point of the single home: a port loss produces no frame, and health
    still tells the truth."""
    r = make_reader()
    r._fail('injected')
    assert r.step() is False                      # nothing published from step
    assert r.health()['sensors_dead'] == ['sensor_%d' % i for i in range(6)]
    assert r.health()['port_open'] is False


def test_the_reader_logs_through_the_subsystem_logger(caplog):
    """print() went to a block-buffered stdout under systemd and could be lost
    outright -- reader_restarts would read 2 with nothing in the journal."""
    import logging
    r = make_reader()
    with caplog.at_level(logging.ERROR, logger=PixartJ3Reader.LOGGER_NAME):
        r._fail('injected fault')
    assert any('injected fault' in rec.message for rec in caplog.records)
    assert r.logger.name == 'line_sensor_loop'    # what worker_loop/LoopStats use
