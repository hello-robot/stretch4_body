"""Tare calibration maths. Runs without a robot and without a fleet directory."""

import numpy as np
import pytest

from stretch4_body.subsystem.line_sensor import protocol
from stretch4_body.subsystem.line_sensor.calibration import (
    CalibrationThresholds,
    apply_tare_array,
    compare_fingerprints,
    compute_sensor_tare,
    config_fingerprint,
    fingerprint_hash,
    tare_diagnostics,
)

IDEAL = 0.229646
NBINS = 16
NFRAMES = 120
TH = CalibrationThresholds(min_frames=10)


def make_run(n_frames=NFRAMES, n_bins=NBINS, floor_mm=229.646):
    """A clean flat-floor recording: every bin a valid measurement."""
    raw = np.full((n_frames, n_bins), floor_mm)
    return decode_stack(raw)


def decode_stack(raw_mm):
    """Push raw wire millimetres through the real decoder, frame by frame, so
    tests exercise the same NaN/codes representation the robot produces."""
    ranges, codes = [], []
    for row in np.asarray(raw_mm, dtype=np.float64):
        r, c = protocol.decode_distances_mm(row)
        ranges.append(r)
        codes.append(c)
    return np.stack(ranges), np.stack(codes)


class TestThresholds:
    def test_defaults_validate(self):
        CalibrationThresholds().validate()

    @pytest.mark.parametrize('kw', [
        {'min_range_m': 0.5, 'max_range_m': 0.4},
        {'min_valid_fraction': 0.0},
        {'min_valid_fraction': 1.5},
        {'max_abs_tare_m': 0.0},
        {'max_bin_mad_m': -1.0},
        {'min_accepted_bin_fraction': 0.0},
        {'min_frames': 1},
        {'warn_median_offset_m': -0.1},
    ])
    def test_bad_thresholds_raise(self, kw):
        with pytest.raises(ValueError):
            CalibrationThresholds(**kw).validate()


class TestInputValidation:
    def test_codes_are_required_positionally(self):
        # Not a keyword with a default: without codes the sentinel
        # classification would silently degrade to "no sentinels at all".
        ranges, codes = make_run()
        with pytest.raises(TypeError):
            compute_sensor_tare(ranges, ideal_range_m=IDEAL, thresholds=TH)

    def test_shape_mismatch_raises(self):
        ranges, codes = make_run()
        with pytest.raises(ValueError, match='codes shape'):
            compute_sensor_tare(ranges, codes[:-1], IDEAL, TH)

    def test_too_few_frames_raises(self):
        ranges, codes = make_run(n_frames=5)
        with pytest.raises(ValueError, match='at least'):
            compute_sensor_tare(ranges, codes, IDEAL, TH)

    def test_ideal_may_be_scalar_or_per_bin(self):
        ranges, codes = make_run()
        a = compute_sensor_tare(ranges, codes, IDEAL, TH)
        b = compute_sensor_tare(ranges, codes, np.full(NBINS, IDEAL), TH)
        np.testing.assert_allclose(a.offsets, b.offsets)

    def test_non_finite_ideal_raises(self):
        ranges, codes = make_run()
        with pytest.raises(ValueError, match='finite'):
            compute_sensor_tare(ranges, codes, np.nan, TH)


class TestCleanRun:
    def test_flat_floor_at_ideal_gives_zero_offsets(self):
        ranges, codes = make_run(floor_mm=IDEAL * 1000)
        r = compute_sensor_tare(ranges, codes, IDEAL, TH)
        assert r.sufficient
        assert r.valid_mask.all()
        assert np.abs(r.offsets).max() < 1e-6
        assert (r.null_rate_per_bin == 0).all()

    def test_uniform_offset_is_recovered(self):
        # floor measured 20 mm short of ideal -> every offset -0.020
        ranges, codes = make_run(floor_mm=IDEAL * 1000 - 20)
        r = compute_sensor_tare(ranges, codes, IDEAL, TH)
        assert r.sufficient
        np.testing.assert_allclose(r.offsets, -0.020, atol=1e-9)
        # and it warns, because a uniform shift is a geometry parameter error
        assert any('geometry PARAMETER' in w for w in r.warnings)

    def test_offsets_are_never_nan(self):
        raw = np.full((NFRAMES, NBINS), IDEAL * 1000)
        raw[:, 3] = protocol.MM_NO_DETECTION      # bin 3 never returns
        r = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        assert np.isfinite(r.offsets).all()
        assert r.offsets[3] == 0.0                # rejected -> zero, not NaN
        assert not r.valid_mask[3]


class TestSentinelsNeverEnterTheTare:
    def test_occasional_no_return_does_not_shift_the_offset(self):
        # 5% blind, inside the 10% the min_valid_fraction gate allows
        raw = np.full((NFRAMES, NBINS), IDEAL * 1000 - 10)
        raw[::20, 5] = protocol.MM_NO_DETECTION
        r = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        # the surviving samples still say -0.010, exactly like its neighbours
        assert r.valid_mask[5]
        assert r.offsets[5] == pytest.approx(r.offsets[4])
        assert r.null_rate_per_bin[5] == pytest.approx(0.05)

    def test_bin_blind_more_often_than_the_gate_allows_is_rejected(self):
        raw = np.full((NFRAMES, NBINS), IDEAL * 1000 - 10)
        raw[::7, 5] = protocol.MM_NO_DETECTION    # ~14% blind, gate allows 10%
        r = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        assert not r.valid_mask[5]
        assert 5 in r.indices['insufficient_data']

    def test_all_sentinel_bin_is_rejected_not_averaged(self):
        raw = np.full((NFRAMES, NBINS), IDEAL * 1000)
        raw[:, 2] = protocol.MM_BEYOND_LIMIT
        r = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        assert not r.valid_mask[2]
        assert r.offsets[2] == 0.0
        assert r.null_rate_per_bin[2] == 1.0
        assert 2 in r.indices['never_returned']

    def test_5_09_and_5_11_are_counted_separately(self):
        raw = np.full((NFRAMES, NBINS), IDEAL * 1000)
        raw[:, 1] = protocol.MM_NO_DETECTION      # blind
        raw[:, 2] = protocol.MM_BEYOND_LIMIT      # beam passed the floor
        r = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        assert r.per_bin['no_return_counts'][1] == NFRAMES
        assert r.per_bin['beyond_limit_counts'][1] == 0
        assert r.per_bin['beyond_limit_counts'][2] == NFRAMES
        assert r.per_bin['no_return_counts'][2] == 0
        # 5.09 on a supposedly flat floor is called out specifically
        assert any('beyond-limit (5.09)' in x for x in r.insufficiency_reasons)

    def test_all_sentinel_sensor_produces_no_tare_and_no_warning_spam(self):
        raw = np.full((NFRAMES, NBINS), protocol.MM_NO_DETECTION)
        r = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        assert not r.sufficient
        assert not r.valid_mask.any()
        assert np.isfinite(r.offsets).all()
        assert (r.null_rate_per_bin == 1.0).all()


class TestRejectionGates:
    def test_obstacle_samples_are_out_of_window_not_invalid(self):
        raw = np.full((NFRAMES, NBINS), IDEAL * 1000)
        raw[:, 7] = 600.0                          # a real return, but not floor
        r = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        assert r.summary['out_of_window_samples'] == NFRAMES
        assert r.summary['other_invalid_samples'] == 0
        assert not r.valid_mask[7]

    def test_bimodal_bin_is_caught_by_dispersion(self):
        # both modes inside the window, so count and magnitude gates pass
        raw = np.full((NFRAMES, NBINS), IDEAL * 1000)
        raw[::2, 9] = IDEAL * 1000 + 40
        r = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        assert not r.valid_mask[9], 'a bimodal bin shipped as a trustworthy tare'
        assert 9 in r.indices['high_dispersion']

    def test_implausible_offset_rejected(self):
        raw = np.full((NFRAMES, NBINS), IDEAL * 1000)
        raw[:, 4] = 380.0                          # in window, but way off ideal
        r = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        assert not r.valid_mask[4]
        assert 4 in r.indices['implausible_offset']

    def test_verdict_uses_integer_counts(self):
        # 16 bins, 0.95 -> ceil(15.2) = 16 accepted bins required
        raw = np.full((NFRAMES, NBINS), IDEAL * 1000)
        r = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        assert r.summary['required_accepted_bins'] == 16
        assert r.sufficient
        raw[:, 0] = protocol.MM_NO_DETECTION
        r2 = compute_sensor_tare(*decode_stack(raw), IDEAL, TH)
        assert not r2.sufficient
        assert '15/16' not in r2.insufficiency_reasons[0]
        assert '1/16 bins rejected' in r2.insufficiency_reasons[0]


class TestApplyTare:
    def _fixture(self):
        ranges, codes = protocol.decode_distances_mm(
            [200.0, protocol.MM_NO_DETECTION, protocol.MM_BEYOND_LIMIT, 210.0, 220.0])
        offsets = np.array([0.01, 0.01, 0.01, 0.01, 0.01])
        mask = np.array([True, True, True, False, True])
        return ranges, codes, offsets, mask

    def test_sentinel_bins_stay_nan_even_with_a_valid_tare(self):
        ranges, codes, offsets, mask = self._fixture()
        out = apply_tare_array(ranges, offsets, mask, codes)
        assert np.isnan(out[1]) and np.isnan(out[2])

    def test_untrusted_bin_is_returned_bit_identical(self):
        ranges, codes, offsets, mask = self._fixture()
        out = apply_tare_array(ranges, offsets, mask, codes)
        assert out[3] == ranges[3]

    def test_trusted_bins_are_corrected(self):
        ranges, codes, offsets, mask = self._fixture()
        out = apply_tare_array(ranges, offsets, mask, codes)
        assert out[0] == pytest.approx(0.190)
        assert out[4] == pytest.approx(0.210)

    def test_input_is_not_mutated(self):
        ranges, codes, offsets, mask = self._fixture()
        before = ranges.copy()
        apply_tare_array(ranges, offsets, mask, codes)
        np.testing.assert_array_equal(
            np.nan_to_num(ranges, nan=-1), np.nan_to_num(before, nan=-1))

    def test_never_manufactures_a_nonpositive_range(self):
        ranges, codes = protocol.decode_distances_mm([50.0])
        out = apply_tare_array(ranges, np.array([0.20]), np.array([True]), codes)
        assert out[0] == pytest.approx(0.050), 'correction drove the bin <= 0'

    def test_shape_mismatch_raises_rather_than_silently_skipping(self):
        ranges, codes = protocol.decode_distances_mm([200.0, 210.0])
        with pytest.raises(ValueError, match='does not match'):
            apply_tare_array(ranges, np.zeros(3), np.ones(3, bool), codes)

    def test_diagnostics_account_for_every_bin(self):
        ranges, codes, offsets, mask = self._fixture()
        d = tare_diagnostics(ranges, offsets, mask, codes)
        # bins 0 and 4 are measurable and trusted; 1 and 2 are sentinels;
        # 3 is measurable but has no trustworthy tare
        assert d['n_applied'] == 2
        assert d['n_no_tare'] == 1
        assert d['n_sentinel'] == 2
        assert d['n_applied'] + d['n_no_tare'] + d['n_sentinel'] == len(ranges)


PARAMS = {
    'bus_sensor_map': [[1, 0], [3, 2], [5, 4]],
    'flip_range_ordering': True,
    'line_sensor_geometry': {
        'pixart_report_num': 320,
        'emitter_height_above_floor_mm': 100.67,
        'sensor_angle_down_deg': 26.0,
        'sensor_horizontal_fov_degrees': 103.0,
        'sensor_angles_deg': [10.18, 39.64, 80.36, 39.64, 80.36, 39.64],
        'sensor_normals_deg': [0.0, 60.0, 120.0, 180.0, 240.0, 300.0],
    },
}


class TestFingerprint:
    def test_stable_across_calls(self):
        a = config_fingerprint('sensor_0', 0, PARAMS)
        b = config_fingerprint('sensor_0', 0, PARAMS)
        assert fingerprint_hash(a) == fingerprint_hash(b)
        assert compare_fingerprints(a, b) == []

    def test_bus_map_change_is_detected_and_explained(self):
        other = {**PARAMS, 'bus_sensor_map': [[0, 1], [2, 3], [4, 5]]}
        a = config_fingerprint('sensor_0', 0, PARAMS)
        b = config_fingerprint('sensor_0', 0, other)
        assert fingerprint_hash(a) != fingerprint_hash(b)
        diffs = compare_fingerprints(a, b)
        assert any('bus_sensor_map' in d for d in diffs)

    def test_flip_change_is_detected(self):
        other = {**PARAMS, 'flip_range_ordering': False}
        a = config_fingerprint('sensor_0', 0, PARAMS)
        b = config_fingerprint('sensor_0', 0, other)
        assert fingerprint_hash(a) != fingerprint_hash(b)

    def test_geometry_change_names_the_parameter(self):
        geom = {**PARAMS['line_sensor_geometry'],
                'emitter_height_above_floor_mm': 92.0}
        b = config_fingerprint('sensor_0', 0,
                               {**PARAMS, 'line_sensor_geometry': geom})
        diffs = compare_fingerprints(config_fingerprint('sensor_0', 0, PARAMS), b)
        assert any('emitter_height_above_floor_mm' in d and '100.67' in d
                   for d in diffs), diffs

    def test_sensor_identity_is_part_of_the_fingerprint(self):
        a = config_fingerprint('sensor_0', 0, PARAMS)
        b = config_fingerprint('sensor_1', 1, PARAMS)
        assert fingerprint_hash(a) != fingerprint_hash(b)

    def test_bool_is_not_canonicalised_as_int(self):
        # bool subclasses int; ordering the isinstance checks wrongly would
        # make flip_range_ordering=True hash the same as =1
        a = config_fingerprint('sensor_0', 0, PARAMS)
        b = config_fingerprint('sensor_0', 0, {**PARAMS, 'flip_range_ordering': 1})
        assert a['flip_range_ordering'] is True
        assert b['flip_range_ordering'] is True

    def test_hash_is_insensitive_to_float_repr_drift(self):
        geom = {**PARAMS['line_sensor_geometry'],
                'sensor_angle_down_deg': 26.000000000000004}
        b = config_fingerprint('sensor_0', 0,
                               {**PARAMS, 'line_sensor_geometry': geom})
        assert fingerprint_hash(config_fingerprint('sensor_0', 0, PARAMS)) == \
            fingerprint_hash(b)


if __name__ == '__main__':
    pytest.main([__file__])
