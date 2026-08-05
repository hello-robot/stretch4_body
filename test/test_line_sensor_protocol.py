import warnings

import numpy as np
import pytest

from stretch4_body.subsystem.line_sensor import protocol
from stretch4_body.subsystem.line_sensor.pixart_j3_reader import PixartJ3Reader

SE4_BUS_MAP = [[1, 0], [3, 2], [5, 4]]


class TestClassify:
    def test_exact_codes(self):
        codes = protocol.classify_ranges_mm([5090, 5110, 5090.0, 5110.0])
        assert codes.tolist() == [protocol.CODE_BEYOND_LIMIT,
                                  protocol.CODE_NO_RETURN,
                                  protocol.CODE_BEYOND_LIMIT,
                                  protocol.CODE_NO_RETURN]

    def test_raw_chip_codes_match_mm_values(self):
        # 16-bit fixed-point cm with 7 fractional bits -> mm
        assert protocol.RAW_CODE_NO_DETECTION / 128.0 * 10.0 == protocol.MM_NO_DETECTION
        assert protocol.RAW_CODE_BEYOND_LIMIT / 128.0 * 10.0 == protocol.MM_BEYOND_LIMIT

    def test_valid_distances(self):
        # Real distances classify VALID everywhere below the code band --
        # there is deliberately no 4 m style cutoff (the chip reports every
        # beyond-range measurement as 5.09/5.11, never as a large distance).
        codes = protocol.classify_ranges_mm([1.0, 150.0, 202.0, 3999.0, 5089.9])
        assert (codes == protocol.CODE_VALID).all()

    def test_invalid_values(self):
        codes = protocol.classify_ranges_mm([0.0, -5.0, np.nan, np.inf])
        assert (codes == protocol.CODE_OTHER_INVALID).all()

    def test_unknown_codes_in_band_are_invalid_not_distance(self):
        # Anything in the code band that is not an exact known code must not
        # masquerade as a distance.
        codes = protocol.classify_ranges_mm([5100.0, 5119.0, 6000.0, 5090.5])
        assert (codes == protocol.CODE_OTHER_INVALID).all()


class TestDecode:
    def test_units_and_nan_masking(self):
        ranges, codes = protocol.decode_distances_mm([202, 5090, 5110, 0])
        # a status code is not a distance: it must never survive in `ranges`,
        # where it could be silently averaged into a floor estimate
        assert ranges[0] == 0.202
        assert np.isnan(ranges[1:]).all()
        assert codes.tolist() == [protocol.CODE_VALID, protocol.CODE_BEYOND_LIMIT,
                                  protocol.CODE_NO_RETURN, protocol.CODE_OTHER_INVALID]

    def test_code_identity_survives_nan_masking(self):
        # 5.09 vs 5.11 must stay distinguishable, via one integer compare
        ranges, codes = protocol.decode_distances_mm([5090, 5110])
        assert np.isnan(ranges).all()
        assert (codes == protocol.CODE_BEYOND_LIMIT).tolist() == [True, False]
        assert (codes == protocol.CODE_NO_RETURN).tolist() == [False, True]
        assert protocol.CODE_VALUE_M[protocol.CODE_BEYOND_LIMIT] == 5.09
        assert protocol.CODE_VALUE_M[protocol.CODE_NO_RETURN] == 5.11

    def test_nan_ranges_do_not_poison_aggregates(self):
        ranges, _ = protocol.decode_distances_mm([200, 200, 5110, 5110])
        # the whole point: a blind bin must not pull the floor estimate up
        assert np.nanmedian(ranges) == 0.2
        assert np.isnan(np.median(ranges))  # plain median stays loudly NaN

    def test_clean_frame_leaves_all_valid(self):
        ranges, codes = protocol.decode_distances_mm([201, 202, 203])
        assert (codes == protocol.CODE_VALID).all()
        assert not np.isnan(ranges).any()

    def test_flip_keeps_ranges_and_codes_aligned(self):
        raw = [100, 200, 5110, 5090]
        fwd_r, fwd_c = protocol.decode_distances_mm(raw, flip=False)
        rev_r, rev_c = protocol.decode_distances_mm(raw, flip=True)
        assert rev_c.tolist() == fwd_c.tolist()[::-1]
        np.testing.assert_array_equal(rev_r, fwd_r[::-1])


class TestKeyTable:
    def test_se4_map(self):
        table = protocol.build_key_table(SE4_BUS_MAP)
        assert table == {'distances10': 1, 'distances11': 0,
                         'distances20': 3, 'distances21': 2,
                         'distances30': 5, 'distances31': 4}

    def test_rejects_bad_shape(self):
        with pytest.raises(ValueError):
            protocol.build_key_table([[0, 1], [2, 3]])

    def test_rejects_duplicate_indices(self):
        with pytest.raises(ValueError):
            protocol.build_key_table([[0, 1], [2, 3], [4, 4]])


def make_reader():
    # All config passed explicitly: no robot params needed, no serial opened.
    return PixartJ3Reader(bus_sensor_map=SE4_BUS_MAP,
                          flip_range_ordering=True, report_num=4)


def payload(frame_id, key, values):
    import json
    return json.dumps({'frameId': frame_id, key: values})


class TestNoAliasing:
    def test_each_sensor_gets_an_independent_buffer(self):
        # Two sensors sharing one numpy buffer would make one sensor's reading
        # silently overwrite another's -- a mis-mapping that is invisible
        # downstream. Decode must always produce fresh arrays.
        pjr = make_reader()
        pjr.process_json_line(payload(1, 'distances10', [201, 202, 203, 204]))
        pjr.process_json_line(payload(1, 'distances11', [101, 102, 103, 104]))
        a, b = pjr.status['sensor_1'], pjr.status['sensor_0']
        assert not np.shares_memory(a['ranges'], b['ranges'])
        assert not np.shares_memory(a['codes'], b['codes'])
        a['ranges'][0] = -999.0
        assert b['ranges'][0] != -999.0

    def test_decode_does_not_alias_its_input(self):
        raw = np.array([201.0, 202.0, 5110.0, 204.0])
        ranges, codes = protocol.decode_distances_mm(raw)
        assert not np.shares_memory(ranges, raw)
        ranges[0] = -1.0
        assert raw[0] == 201.0


class TestReaderConfig:
    def test_flip_param_actually_controls_ordering(self):
        raw = [201, 202, 203, 204]
        flipped = PixartJ3Reader(bus_sensor_map=SE4_BUS_MAP,
                                 flip_range_ordering=True, report_num=4)
        straight = PixartJ3Reader(bus_sensor_map=SE4_BUS_MAP,
                                  flip_range_ordering=False, report_num=4)
        flipped.process_json_line(payload(1, 'distances10', raw))
        straight.process_json_line(payload(1, 'distances10', raw))
        assert straight.status['sensor_1']['ranges'].tolist() == [0.201, 0.202, 0.203, 0.204]
        assert flipped.status['sensor_1']['ranges'].tolist() == [0.204, 0.203, 0.202, 0.201]


class TestLiveness:
    def _frame(self, pjr, frame_id, keys):
        for key in keys:
            pjr.process_json_line(payload(frame_id, key, [200, 200, 200, 200]))

    ALL = ['distances10', 'distances11', 'distances20',
           'distances21', 'distances30', 'distances31']

    def test_all_healthy_reports_nothing_dead(self):
        pjr = make_reader()
        self._frame(pjr, 1, self.ALL)
        assert pjr.dead_sensors() == []
        assert all(pjr.status[f'sensor_{i}']['missed_frames'] == 0 for i in range(6))

    def test_silent_sensor_is_reported_dead(self):
        pjr = make_reader()
        # sensor_4 lives on bus 3 dev 1; drop that key from every frame
        quiet = [k for k in self.ALL if k != 'distances31']
        for frame_id in range(1, 9):
            self._frame(pjr, frame_id, quiet)
        assert 'sensor_4' in pjr.dead_sensors()
        assert pjr.dead_sensors() == ['sensor_4']  # and only that one

    def test_sensor_recovers(self):
        pjr = make_reader()
        quiet = [k for k in self.ALL if k != 'distances31']
        for frame_id in range(1, 9):
            self._frame(pjr, frame_id, quiet)
        assert pjr.dead_sensors() == ['sensor_4']
        for frame_id in range(9, 12):
            self._frame(pjr, frame_id, self.ALL)
        assert pjr.dead_sensors() == []

    def test_never_seen_sensor_starts_dead(self):
        # A sensor that never comes up at all must not read as merely quiet.
        pjr = make_reader()
        assert set(pjr.dead_sensors()) == {f'sensor_{i}' for i in range(6)}

    def test_blind_sensor_is_alive_but_sees_nothing(self):
        pjr = make_reader()
        self._frame(pjr, 1, [k for k in self.ALL if k != 'distances10'])
        # sensor_1 reports, but every bin is a no-return
        pjr.process_json_line(payload(1, 'distances10', [5110, 5110, 5110, 5110]))
        assert 'sensor_1' not in pjr.dead_sensors()   # the link is fine
        assert 'sensor_1' in pjr.blind_sensors()      # it just cannot see

    def test_missing_flip_param_raises_rather_than_defaulting(self):
        # A silently-wrong bin order mirrors every reading and nothing
        # downstream can detect it, so it must never be guessed.
        with pytest.raises(ValueError, match='flip_range_ordering'):
            PixartJ3Reader(bus_sensor_map=SE4_BUS_MAP, flip_range_ordering=None,
                           report_num=4, _params={})

    def test_missing_bus_map_raises(self):
        with pytest.raises(ValueError, match='bus_sensor_map'):
            PixartJ3Reader(bus_sensor_map=None, flip_range_ordering=True,
                           report_num=4, _params={})


class TestReaderDecode:
    def test_one_sensor_report(self):
        pjr = make_reader()
        assert pjr.process_json_line(payload(7, 'distances10', [202, 203, 5090, 5110]))
        s = pjr.status['sensor_1']  # bus 1 dev 0 -> sensor_1 on the SE4 map
        # flip_range_ordering reverses bin order at decode
        assert np.isnan(s['ranges'][:2]).all()
        assert s['ranges'][2:].tolist() == [0.203, 0.202]
        assert s['codes'].tolist() == [protocol.CODE_NO_RETURN,
                                       protocol.CODE_BEYOND_LIMIT,
                                       protocol.CODE_VALID, protocol.CODE_VALID]
        assert s['n_no_return'] == 1
        assert s['n_beyond_limit'] == 1
        assert s['frame_id'] == 7

    def test_full_frame_updates_frame_stats(self):
        pjr = make_reader()
        for key in ['distances10', 'distances11', 'distances20',
                    'distances21', 'distances30', 'distances31']:
            assert pjr.process_json_line(payload(3, key, [200, 200, 200, 200]))
        assert pjr.status['last_frame_time'] > 0
        assert pjr.status['not_six_sensors_err'] == 0
        assert pjr.sensors_seen == {}  # frame flushed

    def test_bad_json_counts_and_survives(self):
        pjr = make_reader()
        assert not pjr.process_json_line('{"frameId": 3, "distances10": [1, 2,}')
        assert pjr.status['decode_errors'] == 1
        # reader still works afterwards
        assert pjr.process_json_line(payload(4, 'distances10', [200, 200, 200, 200]))

    def test_missing_frame_id_counts(self):
        pjr = make_reader()
        assert not pjr.process_json_line('{"distances10": [1, 2, 3, 4]}')
        assert pjr.status['decode_errors'] == 1

    def test_wrong_report_length_dropped(self):
        pjr = make_reader()
        assert not pjr.process_json_line(payload(2, 'distances10', [200, 200]))
        assert pjr.status['decode_errors'] == 1
        assert pjr.status['sensor_1']['frame_id'] == 0

    def test_out_of_order_frame_resyncs_instead_of_dying(self):
        pjr = make_reader()
        assert pjr.process_json_line(payload(10, 'distances10', [200, 200, 200, 200]))
        # a frame id going backwards (uC restart) is counted, then adopted
        assert pjr.process_json_line(payload(2, 'distances11', [200, 200, 200, 200]))
        assert pjr.status['frame_advance_err'] == 1
        assert pjr.status['sensor_0']['frame_id'] == 2
        assert pjr.last_frame_id == 2


class _FakeSerial:
    """Feeds a canned byte stream through the real step() framing code."""

    def __init__(self, blob):
        self.buf = blob
        self.is_open = True

    @property
    def in_waiting(self):
        return len(self.buf)

    def read(self, n):
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def close(self):
        self.is_open = False


class TestOneSensorFailureStaysContained:
    """A failing sensor must never let another sensor's data land in its slot.

    Assignment is by JSON key, not arrival order, so a missing or corrupt
    report cannot shift the ones after it. These tests pin that down: each
    report carries a signature encoding the index it must land in, so any
    shift shows up as a slot holding someone else's signature.
    """

    KEYS = ['distances10', 'distances11', 'distances20',
            'distances21', 'distances30', 'distances31']
    EXPECT = {'distances10': 1, 'distances11': 0, 'distances20': 3,
              'distances21': 2, 'distances30': 5, 'distances31': 4}

    def _line(self, frame_id, key, sig):
        import json
        return json.dumps({'frameId': frame_id, key: [100 + sig] * 4}) + '\n'

    def _misattributed(self, pjr):
        out = []
        for i in range(6):
            r = pjr.status['sensor_%d' % i]['ranges']
            if len(r) and round(float(r[0]) * 1000) != 100 + i:
                out.append(i)
        return out

    def _run_stream(self, blob):
        pjr = make_reader()
        pjr.is_valid = True
        pjr.ser = _FakeSerial(blob.encode())
        while pjr.ser.in_waiting:
            pjr.step()
        return pjr

    def test_silent_sensor_does_not_get_filled_by_another(self):
        pjr = make_reader()
        for key in self.KEYS:
            pjr.process_json_line(self._line(1, key, self.EXPECT[key]))
        # sensor_4's report (distances31) stops arriving
        for frame_id in range(2, 9):
            for key in (k for k in self.KEYS if k != 'distances31'):
                pjr.process_json_line(self._line(frame_id, key, self.EXPECT[key]))
        assert self._misattributed(pjr) == []
        s4 = pjr.status['sensor_4']
        assert round(float(s4['ranges'][0]) * 1000) == 104  # its OWN old data
        assert s4['frame_id'] == 1                          # visibly frozen
        assert pjr.dead_sensors() == ['sensor_4']
        for i in (0, 1, 2, 3, 5):
            assert pjr.status['sensor_%d' % i]['frame_id'] == 8

    def test_clean_stream_through_framing(self):
        pjr = self._run_stream(
            ''.join(self._line(1, k, self.EXPECT[k]) for k in self.KEYS))
        assert self._misattributed(pjr) == []
        assert pjr.status['decode_errors'] == 0
        assert all(len(pjr.status['sensor_%d' % i]['ranges']) for i in range(6))

    def test_lost_newline_loses_only_the_corrupt_report(self):
        # Dropped bytes swallow the newline, so a truncated report and the
        # next one arrive as a single unparsable string. The intact tail must
        # still be recovered, or one bad report takes its neighbour with it.
        blob = (self._line(1, 'distances10', 1)[:25]
                + ''.join(self._line(1, k, self.EXPECT[k])
                          for k in self.KEYS if k != 'distances10'))
        pjr = self._run_stream(blob)
        assert self._misattributed(pjr) == []
        lost = [i for i in range(6) if not len(pjr.status['sensor_%d' % i]['ranges'])]
        assert lost == [1], f'corrupt report took neighbours down too: {lost}'
        assert pjr.status['decode_errors'] == 1
        assert pjr.is_valid  # and the reader survives

    def test_garbage_burst_between_reports_is_survived(self):
        blob = (self._line(1, 'distances11', 0) + '\x00 garbage not json \n'
                + ''.join(self._line(1, k, self.EXPECT[k])
                          for k in self.KEYS if k != 'distances11'))
        pjr = self._run_stream(blob)
        assert self._misattributed(pjr) == []
        assert all(len(pjr.status['sensor_%d' % i]['ranges']) for i in range(6))
        assert pjr.is_valid


if __name__ == '__main__':
    pytest.main([__file__])
