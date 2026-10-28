"""what the loop puts on the wire, and what a client gets back.
"""

import queue

import numpy as np
import pytest

from stretch4_body.subsystem.line_sensor import calibration, protocol


# ---------------------------------------------------------------------------
# tare wire form
# ---------------------------------------------------------------------------

def _tare(n=320, seed=0):
    rng = np.random.default_rng(seed)
    return (rng.uniform(-0.05, 0.05, n),
            rng.random(n) > 0.15,
            rng.random(n))


def test_round_trip_preserves_everything_that_matters():
    offsets, mask, null_rate = _tare()
    o, m, nr = calibration.unpack_tare(calibration.pack_tare(offsets, mask, null_rate))
    # The chip quantises to 1 mm; the wire must be far finer than that.
    assert np.abs(offsets - o).max() < 1e-5
    assert (mask == m).all()          # a mask bit is never approximate
    assert np.abs(null_rate - nr).max() < 1.0 / 255


def test_mask_survives_lengths_that_are_not_multiples_of_eight():
    # packbits pads to a byte boundary; unpack must trim back or every
    # consumer silently gains up to 7 phantom bins at the end of the fan.
    for n in (1, 7, 8, 9, 100, 319, 320, 321):
        offsets, mask, null_rate = _tare(n, seed=n)
        o, m, nr = calibration.unpack_tare(
            calibration.pack_tare(offsets, mask, null_rate))
        assert m.size == n and o.size == n and nr.size == n
        assert (mask == m).all()


def test_all_false_mask_round_trips():
    n = 320
    o, m, nr = calibration.unpack_tare(
        calibration.pack_tare(np.zeros(n), np.zeros(n, bool), np.zeros(n)))
    assert not m.any()


def test_pack_refuses_what_it_cannot_represent():
    n = 320
    mask, null_rate = np.ones(n, bool), np.zeros(n)
    with pytest.raises(ValueError):                      # NaN offset
        calibration.pack_tare(np.full(n, np.nan), mask, null_rate)
    with pytest.raises(ValueError):                      # beyond int16 range
        calibration.pack_tare(np.full(n, 1.0), mask, null_rate)
    with pytest.raises(ValueError):                      # mask of another length
        calibration.pack_tare(np.zeros(n), np.ones(n - 1, bool), null_rate)


def test_unpack_refuses_a_block_whose_arrays_contradict_its_bin_count():
    block = calibration.pack_tare(*_tare())
    block['n_bins'] = 319
    with pytest.raises(ValueError):
        calibration.unpack_tare(block)


def test_packed_tare_is_much_smaller_than_the_raw_arrays():
    import pickle
    offsets, mask, null_rate = _tare()
    packed = len(pickle.dumps(calibration.pack_tare(offsets, mask, null_rate)))
    raw = len(pickle.dumps({'offsets': offsets, 'valid_mask': mask,
                            'null_rate_per_bin': null_rate}))
    # It rides every 100 Hz status message, so the saving is the whole point.
    assert packed < raw / 3


def test_quantised_offsets_still_tare_the_floor_flat():
    """The lossy step must not show up in the corrected ranges."""
    n = 320
    rng = np.random.default_rng(7)
    ideal = 0.2296
    offsets = rng.uniform(-0.03, 0.03, n)
    mask = np.ones(n, bool)
    o, m, _ = calibration.unpack_tare(
        calibration.pack_tare(offsets, mask, np.zeros(n)))
    measured = ideal + offsets
    codes = np.zeros(n, np.uint8)
    corrected = calibration.apply_tare_array(measured, o, m, codes)
    assert np.abs(corrected - ideal).max() < 1e-5


# ---------------------------------------------------------------------------
# status messages must carry complete state
# ---------------------------------------------------------------------------

class _FakeReader:
    """Stands in for PixartJ3Reader: records what step() produced."""

    DEAD_AFTER_MISSED_FRAMES = 5
    HEALTH_KEYS = ('rate_hz', 'last_frame_time', 'sensors_dead', 'decode_errors',
                   'frame_advance_err', 'frame_not_full_err', 'not_six_sensors_err')

    def __init__(self):
        self.status = {k: 0 for k in self.HEALTH_KEYS}
        self.status['sensors_dead'] = []
        for i in range(6):
            self.status['sensor_%d' % i] = {
                'frame_id': 0, 'rate_hz': 30.0, 'ranges': np.zeros(320),
                'codes': np.zeros(320, np.uint8), 'missed_frames': 0}
        self.is_valid = True
        self._advance = []

    def will_update(self, *indices):
        self._advance = list(indices)

    def step(self):
        for i in self._advance:
            s = self.status['sensor_%d' % i]
            s['frame_id'] += 1
            s['ranges'] = np.full(320, 0.1 * (i + 1) + s['frame_id'])
        moved = bool(self._advance)
        self._advance = []
        return moved

    def health(self):
        h = {k: self.status[k] for k in self.HEALTH_KEYS}
        h['streaming'] = self.is_valid
        return h


class _NoCmds:
    """Empty command queue: the step callback drains q_cmd before stepping."""

    def qsize(self):
        return 0

    def get_nowait(self):
        raise queue.Empty


def _step():
    # Importing the loop pulls in Device -> RobotParams, which needs a fleet
    # directory. Off-robot that raises KeyError, not ImportError, so
    # importorskip does not catch it.
    try:
        from stretch4_body.subsystem.line_sensor.line_sensor_loop import (
            _cb_line_sensor_loop_step)
    except (ImportError, KeyError) as exc:
        pytest.skip(f'needs a robot fleet directory: {exc}')
    return _cb_line_sensor_loop_step


def test_every_message_carries_all_six_sensors():
    """The rule the queue forces: one sensor moving publishes complete state.

    If this ever fails because someone re-introduced delta encoding, read the
    module docstring first -- the saving is ~1 point of CPU and the cost is
    silent, permanent frame loss on queue overflow.
    """
    cb = _step()
    r = _FakeReader()
    r.will_update(2)
    out = {}
    cb(r, _NoCmds(), out)
    assert set(k for k in out if k.startswith('sensor_')) == {
        'sensor_%d' % i for i in range(6)}
    assert 'health' in out


def test_no_sensor_arrays_are_published_when_nothing_moved():
    """Health still goes out. A paused subsystem produces no frames by
    definition, so gating health on a frame would leave it unable to say it
    was paused -- it would read as a hardware failure instead."""
    cb = _step()
    r = _FakeReader()
    out = {}
    cb(r, _NoCmds(), out)
    assert [k for k in out if k.startswith('sensor_')] == []
    assert 'health' in out


def test_a_dropped_message_costs_nothing():
    """Why complete state matters: simulate the queue discarding a message.

    sensor_3 updates in a message that is thrown away. Because the NEXT
    message also carries sensor_3's current value, the consumer still ends up
    correct. Under delta encoding sensor_3 would have been stuck at frame 0.
    """
    cb = _step()
    r = _FakeReader()
    merged = {}

    r.will_update(3)
    dropped = {}
    cb(r, _NoCmds(), dropped)              # this message never reaches the parent

    r.will_update(1)
    delivered = {}
    cb(r, _NoCmds(), delivered)
    merged.update(delivered)

    assert merged['sensor_3']['frame_id'] == 1          # survived the drop
    assert np.allclose(merged['sensor_3']['ranges'], 0.1 * 4 + 1)


def test_merging_keeps_every_sensor_current():
    cb = _step()
    r = _FakeReader()
    merged = {}
    for i in range(6):                      # each reports once, one at a time
        r.will_update(i)
        msg = {}
        cb(r, _NoCmds(), msg)
        merged.update(msg)
    for i in range(6):
        assert merged['sensor_%d' % i]['frame_id'] == 1
        assert np.allclose(merged['sensor_%d' % i]['ranges'], 0.1 * (i + 1) + 1)


def test_a_status_message_never_clobbers_the_calibration_block():
    """calibration lives parent-side and must survive every merge."""
    cb = _step()
    r = _FakeReader()
    merged = {'calibration': {'id': 'abc', 'loaded': ['sensor_0']}}
    r.will_update(1)
    msg = {}
    cb(r, _NoCmds(), msg)
    merged.update(msg)
    assert merged['calibration']['id'] == 'abc'
