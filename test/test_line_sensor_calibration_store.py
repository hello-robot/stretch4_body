"""Calibration on-disk layout. Runs with no robot and no fleet directory."""

import os

import numpy as np
import pytest
import yaml

from stretch4_body.subsystem.line_sensor import protocol
from stretch4_body.subsystem.line_sensor import calibration_store as store
from stretch4_body.subsystem.line_sensor.calibration import (
    CalibrationThresholds,
    RecordingSession,
    SensorRecording,
    compute_sensor_tare,
    config_fingerprint,
    fingerprint_hash,
)

NBINS = 16
NFRAMES = 120
IDEAL = 0.229646
TH = CalibrationThresholds(min_frames=10)

PARAMS = {
    'bus_sensor_map': [[1, 0], [3, 2], [5, 4]],
    'flip_range_ordering': True,
    'line_sensor_geometry': {
        'pixart_report_num': NBINS,
        'emitter_height_above_floor_mm': 100.67,
        'sensor_angle_down_deg': 26.0,
        'sensor_horizontal_fov_degrees': 103.0,
        'sensor_angles_deg': [10.18, 39.64, 80.36, 39.64, 80.36, 39.64],
        'sensor_normals_deg': [0.0, 60.0, 120.0, 180.0, 240.0, 300.0],
    },
}


def make_recording(name='sensor_0', index=0, n_frames=NFRAMES, floor_mm=IDEAL * 1000):
    raw = np.full((n_frames, NBINS), floor_mm)
    ranges, codes = [], []
    for row in raw:
        r, c = protocol.decode_distances_mm(row)
        ranges.append(r)
        codes.append(c)
    return SensorRecording(
        sensor_name=name, sensor_index=index,
        ranges=np.stack(ranges), codes=np.stack(codes),
        frame_id=np.arange(n_frames, dtype=np.int64),
        ts=np.linspace(0, n_frames / 30.0, n_frames),
        missed_frames=np.zeros(n_frames, dtype=np.int32),
        stats={'distinct_frames_captured': n_frames})


def make_session(names=('sensor_0', 'sensor_1')):
    s = RecordingSession(session_id='20260804171205', started_at='2026-08-04T17:12:05',
                         ended_at='2026-08-04T17:12:20', requested_frames=NFRAMES,
                         poll_iterations=400, stretch_body_version='test',
                         loop_params_snapshot=PARAMS)
    for i, n in enumerate(names):
        s.recordings[n] = make_recording(n, i)
        fp = config_fingerprint(n, i, PARAMS)
        s.fingerprints[n] = {'fingerprint': fp, 'sha256': fingerprint_hash(fp)}
    return s


def make_tare(tmp_path, name='sensor_0', index=0, params=None, n_frames=NFRAMES):
    rec = make_recording(name, index, n_frames)
    result = compute_sensor_tare(rec.ranges, rec.codes, IDEAL, TH)
    fp = config_fingerprint(name, index, params or PARAMS)
    tare = store.build_tare_yaml(name, index, result, IDEAL, TH, fp,
                                 session_id='20260804171205', n_frames=n_frames)
    out = store.session_dir(str(tmp_path), '20260804171205')
    os.makedirs(out, exist_ok=True)
    return store.write_tare(tare, out, str(tmp_path)), fp


class TestSessionRoundTrip:
    def test_one_npz_holds_every_sensor(self, tmp_path):
        out = store.write_session(make_session(), str(tmp_path))
        assert os.path.exists(os.path.join(out, 'session.npz'))
        assert os.path.exists(os.path.join(out, 'session_meta.yaml'))
        # the whole point: one file, not thousands of per-sample .npy
        assert len([f for f in os.listdir(out) if f.endswith('.npy')]) == 0

    def test_round_trip_preserves_arrays_and_metadata(self, tmp_path):
        original = make_session()
        out = store.write_session(original, str(tmp_path))
        back = store.read_session(out)
        assert back.session_id == original.session_id
        assert back.sensor_names == original.sensor_names
        assert back.requested_frames == NFRAMES
        for n in original.sensor_names:
            a, b = original.recordings[n], back.recordings[n]
            np.testing.assert_array_equal(a.codes, b.codes)
            np.testing.assert_allclose(np.nan_to_num(a.ranges, nan=-1),
                                       np.nan_to_num(b.ranges, nan=-1))
            np.testing.assert_array_equal(a.frame_id, b.frame_id)
            assert b.sensor_index == a.sensor_index

    def test_readable_from_the_npz_path_directly(self, tmp_path):
        out = store.write_session(make_session(), str(tmp_path))
        back = store.read_session(os.path.join(out, 'session.npz'))
        assert back.sensor_names == ['sensor_0', 'sensor_1']

    def test_recompute_from_a_saved_session_reproduces_the_tare(self, tmp_path):
        original = make_session(('sensor_0',))
        out = store.write_session(original, str(tmp_path))
        rec0 = original.recordings['sensor_0']
        first = compute_sensor_tare(rec0.ranges, rec0.codes, IDEAL, TH)
        rec1 = store.read_session(out).recordings['sensor_0']
        again = compute_sensor_tare(rec1.ranges, rec1.codes, IDEAL, TH)
        np.testing.assert_allclose(first.offsets, again.offsets)
        np.testing.assert_array_equal(first.valid_mask, again.valid_mask)

    def test_failed_sensor_keeps_its_evidence_on_disk(self, tmp_path):
        s = make_session(('sensor_0',))
        s.recordings['sensor_0'].status = 'DEAD'
        s.recordings['sensor_0'].notes = ['stopped reporting at frame 12']
        back = store.read_session(store.write_session(s, str(tmp_path)))
        assert back.recordings['sensor_0'].status == 'DEAD'
        assert back.recordings['sensor_0'].notes == ['stopped reporting at frame 12']

    def test_zero_length_recording_round_trips(self, tmp_path):
        s = make_session(('sensor_0',))
        s.recordings['sensor_0'] = SensorRecording(
            'sensor_0', 0, np.zeros((0, NBINS)), np.zeros((0, NBINS), np.uint8),
            np.zeros(0, np.int64), np.zeros(0), np.zeros(0, np.int32),
            status='NO_DATA')
        back = store.read_session(store.write_session(s, str(tmp_path)))
        assert back.recordings['sensor_0'].n_frames == 0
        assert back.recordings['sensor_0'].status == 'NO_DATA'


class TestTarePromotion:
    def test_canonical_path_is_a_lookup_not_a_scan(self, tmp_path):
        path, _ = make_tare(tmp_path)
        assert path == store.tare_path(str(tmp_path), 'sensor_0')
        assert os.path.exists(path)

    def test_session_keeps_its_own_copy_for_history(self, tmp_path):
        make_tare(tmp_path)
        hist = os.path.join(store.session_dir(str(tmp_path), '20260804171205'),
                            'sensor_0_tare.yaml')
        assert os.path.exists(hist)

    def test_promotion_leaves_no_temp_file(self, tmp_path):
        make_tare(tmp_path)
        assert [f for f in os.listdir(store.tare_dir(str(tmp_path)))
                if f.endswith('.tmp')] == []

    def test_recomputing_an_older_session_still_wins(self, tmp_path):
        # the old scheme picked the lexicographically-newest session dir, so a
        # recompute into an older session was shadowed forever
        make_tare(tmp_path)
        rec = make_recording('sensor_0', 0, floor_mm=IDEAL * 1000 - 20)
        result = compute_sensor_tare(rec.ranges, rec.codes, IDEAL, TH)
        fp = config_fingerprint('sensor_0', 0, PARAMS)
        old = store.session_dir(str(tmp_path), '19990101000000')
        os.makedirs(old, exist_ok=True)
        store.write_tare(store.build_tare_yaml('sensor_0', 0, result, IDEAL, TH, fp,
                                               n_frames=NFRAMES), old, str(tmp_path))
        loaded = store.load_validated_tare(
            store.tare_path(str(tmp_path), 'sensor_0'), fp, NBINS)
        assert loaded.offsets[0] == pytest.approx(-0.020)


class TestValidationRefuses:
    def test_valid_tare_loads(self, tmp_path):
        path, fp = make_tare(tmp_path)
        t = store.load_validated_tare(path, fp, NBINS)
        assert t.sensor_name == 'sensor_0'
        assert t.valid_mask.all()
        assert t.offsets.size == NBINS

    def test_missing(self, tmp_path):
        with pytest.raises(store.TareRejected) as e:
            store.load_validated_tare(str(tmp_path / 'nope.yaml'), {}, NBINS)
        assert e.value.reason == store.REJECT_MISSING

    def test_legacy_v1_is_refused_not_migrated(self, tmp_path):
        p = tmp_path / 'legacy.yaml'
        p.write_text(yaml.safe_dump({'sensor_name': 'sensor_0',
                                     'tare_offsets': [0.0] * NBINS}))
        with pytest.raises(store.TareRejected) as e:
            store.load_validated_tare(str(p), {}, NBINS)
        assert e.value.reason == store.REJECT_LEGACY_V1

    def test_fingerprint_mismatch_names_the_parameter(self, tmp_path):
        path, _ = make_tare(tmp_path)
        other = {**PARAMS, 'bus_sensor_map': [[0, 1], [2, 3], [4, 5]]}
        current = config_fingerprint('sensor_0', 0, other)
        with pytest.raises(store.TareRejected) as e:
            store.load_validated_tare(path, current, NBINS)
        assert e.value.reason == store.REJECT_FINGERPRINT_MISMATCH
        assert 'bus_sensor_map' in e.value.detail

    def test_flip_change_invalidates_the_tare(self, tmp_path):
        path, _ = make_tare(tmp_path)
        current = config_fingerprint('sensor_0', 0,
                                     {**PARAMS, 'flip_range_ordering': False})
        with pytest.raises(store.TareRejected) as e:
            store.load_validated_tare(path, current, NBINS)
        assert e.value.reason == store.REJECT_FINGERPRINT_MISMATCH

    def test_hand_edited_file_is_caught(self, tmp_path):
        path, fp = make_tare(tmp_path)
        d = store.read_tare(path)
        d['config_fingerprint']['geometry']['sensor_angle_down_deg'] = 30.0
        with open(path, 'w') as f:
            yaml.safe_dump(d, f, sort_keys=False)
        with pytest.raises(store.TareRejected) as e:
            store.load_validated_tare(path, fp, NBINS)
        assert e.value.reason == store.REJECT_BAD_SCHEMA

    def test_bin_count_mismatch(self, tmp_path):
        path, fp = make_tare(tmp_path)
        with pytest.raises(store.TareRejected) as e:
            store.load_validated_tare(path, fp, NBINS + 1)
        assert e.value.reason == store.REJECT_BIN_COUNT

    def test_nonfinite_offsets_refused(self, tmp_path):
        path, fp = make_tare(tmp_path)
        d = store.read_tare(path)
        d['tare_offsets'][3] = float('nan')
        with open(path, 'w') as f:
            yaml.safe_dump(d, f, sort_keys=False)
        with pytest.raises(store.TareRejected) as e:
            store.load_validated_tare(path, fp, NBINS)
        assert e.value.reason == store.REJECT_NONFINITE

    def test_no_valid_bins_refused(self, tmp_path):
        path, fp = make_tare(tmp_path)
        d = store.read_tare(path)
        d['tare_valid_mask'] = [False] * NBINS
        with open(path, 'w') as f:
            yaml.safe_dump(d, f, sort_keys=False)
        with pytest.raises(store.TareRejected) as e:
            store.load_validated_tare(path, fp, NBINS)
        assert e.value.reason == store.REJECT_NO_VALID_BINS

    def test_refusal_never_falls_back_to_an_older_file(self, tmp_path):
        # a good tare exists in an old session; the canonical one is bad.
        # There must be nowhere to silently fall back to.
        path, fp = make_tare(tmp_path)
        good = store.read_tare(path)
        os.makedirs(store.session_dir(str(tmp_path), '19990101000000'), exist_ok=True)
        with open(os.path.join(store.session_dir(str(tmp_path), '19990101000000'),
                               'sensor_0_tare.yaml'), 'w') as f:
            yaml.safe_dump(good, f, sort_keys=False)
        bad = dict(good, format_version=1)
        with open(path, 'w') as f:
            yaml.safe_dump(bad, f, sort_keys=False)
        with pytest.raises(store.TareRejected):
            store.load_validated_tare(path, fp, NBINS)


if __name__ == '__main__':
    pytest.main([__file__])
