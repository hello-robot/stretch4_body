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
)

NBINS = 16
NFRAMES = 120
IDEAL = 0.229646
FLIP = True
TH = CalibrationThresholds(min_frames=10)

PARAMS = {
    'bus_sensor_map': [[1, 0], [3, 2], [5, 4]],
    'flip_range_ordering': FLIP,
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
    return s


def make_tare(tmp_path, name='sensor_0', index=0, flip=FLIP, ideal=IDEAL,
              n_frames=NFRAMES):
    rec = make_recording(name, index, n_frames)
    result = compute_sensor_tare(rec.ranges, rec.codes, ideal, TH)
    tare = store.build_tare_yaml(name, index, result, ideal, TH, flip,
                                 session_id='20260804171205', n_frames=n_frames)
    out = store.session_dir(str(tmp_path), '20260804171205')
    os.makedirs(out, exist_ok=True)
    return store.write_tare(tare, out, str(tmp_path))


def load(path, n_bins=NBINS, flip=FLIP, ideal=IDEAL):
    return store.load_validated_tare(path, n_bins, flip, ideal)


def meta_of(session_out):
    with open(os.path.join(session_out, 'session_meta.yaml')) as f:
        return yaml.safe_load(f)


class TestSessionOnDisk:
    """Sessions are written as evidence. Nothing reads them back, so these
    check what lands on disk, not a round trip."""

    def test_one_npz_holds_every_sensor(self, tmp_path):
        out = store.write_session(make_session(), str(tmp_path))
        assert os.path.exists(os.path.join(out, 'session.npz'))
        assert os.path.exists(os.path.join(out, 'session_meta.yaml'))
        # the whole point: one file, not thousands of per-sample .npy
        assert [f for f in os.listdir(out) if f.endswith('.npy')] == []

    def test_npz_carries_each_sensors_arrays(self, tmp_path):
        original = make_session()
        out = store.write_session(original, str(tmp_path))
        with np.load(os.path.join(out, 'session.npz')) as z:
            assert sorted(str(n) for n in z['sensor_names']) == ['sensor_0', 'sensor_1']
            for n in original.sensor_names:
                np.testing.assert_array_equal(z[f'{n}__codes'],
                                              original.recordings[n].codes)
                np.testing.assert_allclose(
                    np.nan_to_num(z[f'{n}__ranges'], nan=-1),
                    np.nan_to_num(original.recordings[n].ranges, nan=-1))
                np.testing.assert_array_equal(z[f'{n}__frame_id'],
                                              original.recordings[n].frame_id)

    def test_meta_yaml_mirrors_the_run(self, tmp_path):
        m = meta_of(store.write_session(make_session(), str(tmp_path)))
        assert m['session_id'] == '20260804171205'
        assert m['requested_frames'] == NFRAMES
        assert sorted(m['sensors']) == ['sensor_0', 'sensor_1']
        assert m['sensors']['sensor_1']['sensor_index'] == 1

    def test_failed_sensor_keeps_its_evidence_on_disk(self, tmp_path):
        s = make_session(('sensor_0',))
        s.recordings['sensor_0'].status = 'DEAD'
        s.recordings['sensor_0'].notes = ['stopped reporting at frame 12']
        m = meta_of(store.write_session(s, str(tmp_path)))
        assert m['sensors']['sensor_0']['status'] == 'DEAD'
        assert m['sensors']['sensor_0']['notes'] == ['stopped reporting at frame 12']

    def test_zero_length_recording_is_still_written(self, tmp_path):
        s = make_session(('sensor_0',))
        s.recordings['sensor_0'] = SensorRecording(
            'sensor_0', 0, np.zeros((0, NBINS)), np.zeros((0, NBINS), np.uint8),
            np.zeros(0, np.int64), np.zeros(0), np.zeros(0, np.int32),
            status='NO_DATA')
        out = store.write_session(s, str(tmp_path))
        with np.load(os.path.join(out, 'session.npz')) as z:
            assert z['sensor_0__ranges'].shape == (0, NBINS)
        assert meta_of(out)['sensors']['sensor_0']['status'] == 'NO_DATA'


class TestTarePromotion:
    def test_canonical_path_is_a_lookup_not_a_scan(self, tmp_path):
        path = make_tare(tmp_path)
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

    def test_writing_from_an_older_session_still_wins(self, tmp_path):
        # the old scheme picked the lexicographically-newest session dir, so a
        # tare written into an older session was shadowed forever
        make_tare(tmp_path)
        rec = make_recording('sensor_0', 0, floor_mm=IDEAL * 1000 - 20)
        result = compute_sensor_tare(rec.ranges, rec.codes, IDEAL, TH)
        old = store.session_dir(str(tmp_path), '19990101000000')
        os.makedirs(old, exist_ok=True)
        store.write_tare(store.build_tare_yaml('sensor_0', 0, result, IDEAL, TH,
                                               FLIP, n_frames=NFRAMES),
                         old, str(tmp_path))
        loaded = load(store.tare_path(str(tmp_path), 'sensor_0'))
        assert loaded.offsets[0] == pytest.approx(-0.020)


class TestValidationRefuses:
    def test_valid_tare_loads(self, tmp_path):
        t = load(make_tare(tmp_path))
        assert t.sensor_name == 'sensor_0'
        assert t.valid_mask.all()
        assert t.offsets.size == NBINS

    def test_missing(self, tmp_path):
        with pytest.raises(store.TareRejected) as e:
            load(str(tmp_path / 'nope.yaml'))
        assert e.value.reason == store.REJECT_MISSING

    def test_unversioned_legacy_file_is_refused_not_migrated(self, tmp_path):
        p = tmp_path / 'legacy.yaml'
        p.write_text(yaml.safe_dump({'sensor_name': 'sensor_0',
                                     'tare_offsets': [0.0] * NBINS}))
        with pytest.raises(store.TareRejected) as e:
            load(str(p))
        assert e.value.reason == store.REJECT_LEGACY

    def test_previous_format_is_refused_and_points_at_the_migration(self, tmp_path):
        path = make_tare(tmp_path)
        d = store.read_tare(path)
        d['format_version'] = store.TARE_FORMAT_VERSION - 1
        with open(path, 'w') as f:
            yaml.safe_dump(d, f, sort_keys=False)
        with pytest.raises(store.TareRejected) as e:
            load(path)
        assert e.value.reason == store.REJECT_LEGACY
        assert 'migrate' in e.value.detail

    def test_newer_format_is_refused(self, tmp_path):
        path = make_tare(tmp_path)
        d = store.read_tare(path)
        d['format_version'] = store.TARE_FORMAT_VERSION + 1
        with open(path, 'w') as f:
            yaml.safe_dump(d, f, sort_keys=False)
        with pytest.raises(store.TareRejected) as e:
            load(path)
        assert e.value.reason == store.REJECT_BAD_SCHEMA

    def test_flip_change_invalidates_the_tare(self, tmp_path):
        # every offset would be mirrored, and nothing downstream could tell
        path = make_tare(tmp_path, flip=True)
        with pytest.raises(store.TareRejected) as e:
            load(path, flip=False)
        assert e.value.reason == store.REJECT_BIN_ORDER

    def test_moved_geometry_invalidates_the_tare(self, tmp_path):
        # emitter height or mounting angle changed, so every offset is shifted
        path = make_tare(tmp_path)
        with pytest.raises(store.TareRejected) as e:
            load(path, ideal=IDEAL + 0.01)
        assert e.value.reason == store.REJECT_GEOMETRY

    def test_geometry_tolerance_absorbs_float_repr_drift(self, tmp_path):
        path = make_tare(tmp_path)
        assert load(path, ideal=IDEAL + 1e-9).offsets.size == NBINS

    def test_bin_count_mismatch(self, tmp_path):
        path = make_tare(tmp_path)
        with pytest.raises(store.TareRejected) as e:
            load(path, n_bins=NBINS + 1)
        assert e.value.reason == store.REJECT_BIN_COUNT

    def test_nonfinite_offsets_refused(self, tmp_path):
        path = make_tare(tmp_path)
        d = store.read_tare(path)
        d['tare_offsets'][3] = float('nan')
        with open(path, 'w') as f:
            yaml.safe_dump(d, f, sort_keys=False)
        with pytest.raises(store.TareRejected) as e:
            load(path)
        assert e.value.reason == store.REJECT_NONFINITE

    def test_no_valid_bins_refused(self, tmp_path):
        path = make_tare(tmp_path)
        d = store.read_tare(path)
        d['tare_valid_mask'] = [False] * NBINS
        with open(path, 'w') as f:
            yaml.safe_dump(d, f, sort_keys=False)
        with pytest.raises(store.TareRejected) as e:
            load(path)
        assert e.value.reason == store.REJECT_NO_VALID_BINS

    def test_refusal_never_falls_back_to_an_older_file(self, tmp_path):
        # a good tare exists in an old session; the canonical one is bad.
        # There must be nowhere to silently fall back to.
        path = make_tare(tmp_path)
        good = store.read_tare(path)
        os.makedirs(store.session_dir(str(tmp_path), '19990101000000'), exist_ok=True)
        with open(os.path.join(store.session_dir(str(tmp_path), '19990101000000'),
                               'sensor_0_tare.yaml'), 'w') as f:
            yaml.safe_dump(good, f, sort_keys=False)
        bad = dict(good, format_version=1)
        with open(path, 'w') as f:
            yaml.safe_dump(bad, f, sort_keys=False)
        with pytest.raises(store.TareRejected):
            load(path)

    def test_auto_migration_missing_tare(self, tmp_path):
        # 1. Create the base directory structure for the old format tare
        base_dir = str(tmp_path)
        sensor_name = 'sensor_0'
        session_id = '20250101120000'
        old_tare_dir = os.path.join(base_dir, sensor_name, session_id)
        os.makedirs(old_tare_dir, exist_ok=True)
        
        old_tare_path = os.path.join(old_tare_dir, 'calibration_tare.yaml')
        old_tare_data = {
            'ideal_range_m': IDEAL,
            'tare_offsets': [0.0] * NBINS,
            'tare_valid_mask': [True] * NBINS,
            'timestamp': '2025-01-01T12:00:00',
        }
        with open(old_tare_path, 'w') as f:
            yaml.safe_dump(old_tare_data, f)
            
        # 2. Mock RobotParams.get_params to return PARAMS
        from unittest.mock import patch
        
        with patch('stretch4_body.core.robot_params.RobotParams.get_params') as mock_params:
            mock_params.return_value = ({}, PARAMS)
            
            # The new canonical path where load_validated_tare will look
            canonical_path = store.tare_path(base_dir, sensor_name)
            assert not os.path.exists(canonical_path)
            
            # Load validated tare (which should trigger auto-migration)
            t = store.load_validated_tare(canonical_path, NBINS, FLIP, IDEAL)
            
            # Verify it migrated successfully
            assert t.sensor_name == sensor_name
            assert t.valid_mask.all()
            assert t.offsets.size == NBINS
            assert os.path.exists(canonical_path)

    def test_auto_migration_legacy_tare(self, tmp_path):
        # 1. Create the base directory structure for the old format tare
        base_dir = str(tmp_path)
        sensor_name = 'sensor_0'
        session_id = '20250101120000'
        old_tare_dir = os.path.join(base_dir, sensor_name, session_id)
        os.makedirs(old_tare_dir, exist_ok=True)
        
        old_tare_path = os.path.join(old_tare_dir, 'calibration_tare.yaml')
        old_tare_data = {
            'ideal_range_m': IDEAL,
            'tare_offsets': [0.05] * NBINS,
            'tare_valid_mask': [True] * NBINS,
            'timestamp': '2025-01-01T12:00:00',
        }
        with open(old_tare_path, 'w') as f:
            yaml.safe_dump(old_tare_data, f)
            
        # 2. Create a legacy tare file at the canonical path (format_version = 1)
        canonical_path = store.tare_path(base_dir, sensor_name)
        os.makedirs(os.path.dirname(canonical_path), exist_ok=True)
        legacy_tare_data = {
            'format_version': 1,
            'sensor_name': sensor_name,
            'ideal_range_m': IDEAL,
            'tare_offsets': [0.01] * NBINS,
            'tare_valid_mask': [True] * NBINS,
        }
        with open(canonical_path, 'w') as f:
            yaml.safe_dump(legacy_tare_data, f)
            
        # 3. Mock RobotParams.get_params
        from unittest.mock import patch
        with patch('stretch4_body.core.robot_params.RobotParams.get_params') as mock_params:
            mock_params.return_value = ({}, PARAMS)
            
            # Load validated tare (which should trigger auto-migration from the old_tare_path)
            t = store.load_validated_tare(canonical_path, NBINS, FLIP, IDEAL)
            
            # Verify it migrated successfully and has updated offsets from the old source
            assert t.sensor_name == sensor_name
            assert t.valid_mask.all()
            assert t.offsets[0] == 0.05


if __name__ == '__main__':
    pytest.main([__file__])
