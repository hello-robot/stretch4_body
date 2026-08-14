#!/usr/bin/env python3
"""Flat-floor tare calibration: the maths and the schema.

"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from stretch4_body.subsystem.line_sensor import protocol

# Identifies the geometry assumption a stored tare was computed under, so a
# future change to the range convention refuses old files instead of silently
# reinterpreting them.
IDEAL_RANGE_MODEL = 'axial_depth_constant_v1'


@dataclass(frozen=True)
class CalibrationThresholds:
    """Acceptance gates for the flat-floor tare.

    max_abs_tare_m bounds GEOMETRIC MODEL MISMATCH, not sensor noise. Real
    per-bin offsets reach 0.045 m on this hardware because the emitter-height
    parameter is off; the frame-to-frame noise floor is under 1 mm. Tightening
    this toward the noise floor would reject legitimate bins.
    """

    min_range_m: float = 0.03          # trusted flat-floor window, inclusive
    max_range_m: float = 0.40          #   (ideal 0.2296: -0.20 / +0.17)
    min_valid_fraction: float = 0.90   # of recorded frames, per bin
    max_abs_tare_m: float = 0.10
    max_bin_mad_m: float = 0.005       # temporal spread of one bin's samples
    min_accepted_bin_fraction: float = 0.90
    min_frames: int = 100
    warn_median_offset_m: float = 0.010

    def validate(self):
        """Raise ValueError on any inconsistent threshold. One place, so the
        CLI does not restate these rules in its argument handling."""
        if not 0.0 < self.min_range_m < self.max_range_m:
            raise ValueError('require 0 < min_range_m < max_range_m, got '
                             f'{self.min_range_m} / {self.max_range_m}')
        if not 0.0 < self.min_valid_fraction <= 1.0:
            raise ValueError('min_valid_fraction must be in (0, 1], got '
                             f'{self.min_valid_fraction}')
        if self.max_abs_tare_m <= 0.0:
            raise ValueError(f'max_abs_tare_m must be > 0, got {self.max_abs_tare_m}')
        if self.max_bin_mad_m <= 0.0:
            raise ValueError(f'max_bin_mad_m must be > 0, got {self.max_bin_mad_m}')
        if not 0.0 < self.min_accepted_bin_fraction <= 1.0:
            raise ValueError('min_accepted_bin_fraction must be in (0, 1], got '
                             f'{self.min_accepted_bin_fraction}')
        if self.min_frames < 2:
            raise ValueError(f'min_frames must be >= 2, got {self.min_frames}')
        if self.warn_median_offset_m < 0.0:
            raise ValueError('warn_median_offset_m must be >= 0, got '
                             f'{self.warn_median_offset_m}')


@dataclass
class SensorCalibrationResult:
    """Per-bin tare plus everything needed to explain or refuse it."""

    offsets: np.ndarray            # (n_bins,) float64; exactly 0.0 where rejected, never NaN
    valid_mask: np.ndarray         # (n_bins,) bool -- "bin_reliable" downstream
    null_rate_per_bin: np.ndarray  # (n_bins,) float64 in [0, 1] -- "bin_null_rate"
    summary: dict                  # scalars only, YAML-safe
    per_bin: dict                  # name -> list, every value length n_bins
    indices: dict                  # name -> list[int]
    sufficient: bool
    insufficiency_reasons: list = field(default_factory=list)
    warnings: list = field(default_factory=list)


def compute_sensor_tare(samples, codes, ideal_range_m, thresholds):
    """Per-bin flat-floor tare from a stack of recorded frames.

    samples : (n_frames, n_bins) float64, NaN at every non-measurement bin
    codes   : (n_frames, n_bins) uint8, protocol.CODE_* -- REQUIRED
    ideal_range_m : float, or (n_bins,) array-like

    """
    samples = np.asarray(samples, dtype=np.float64)
    codes = np.asarray(codes)
    if samples.ndim != 2 or samples.shape[0] == 0 or samples.shape[1] == 0:
        raise ValueError('samples must be a non-empty (n_frames, n_bins) array, '
                         f'got shape {samples.shape}')
    if codes.shape != samples.shape:
        raise ValueError(f'codes shape {codes.shape} != samples shape {samples.shape}')
    n_frames, n_bins = samples.shape
    if n_frames < thresholds.min_frames:
        raise ValueError(f'need at least {thresholds.min_frames} frames to compute a '
                         f'tare, got {n_frames}')
    ideal = np.asarray(ideal_range_m, dtype=np.float64).reshape(-1)
    if ideal.size not in (1, n_bins):
        raise ValueError(f'ideal_range_m must be scalar or length {n_bins}, '
                         f'got {ideal.size}')
    ideal = np.broadcast_to(ideal, (n_bins,)) if ideal.size == 1 else ideal
    if not np.isfinite(ideal).all():
        raise ValueError('ideal_range_m must be finite')
    codes = codes.astype(np.uint8, copy=False)

    # --- classify every sample, by code; no float compare against a sentinel
    code_valid = codes == protocol.CODE_VALID
    finite = np.isfinite(samples)
    in_window = ((samples >= thresholds.min_range_m)
                 & (samples <= thresholds.max_range_m))

    valid = code_valid & finite & in_window
    no_return = codes == protocol.CODE_NO_RETURN         # 5.11 dark floor / lighting
    beyond_limit = codes == protocol.CODE_BEYOND_LIMIT   # 5.09 beam passed the floor
    # A real distance that is not floor -- an obstacle in shot.
    out_of_window = code_valid & finite & ~in_window
    other_invalid = ~(valid | no_return | beyond_limit | out_of_window)

    valid_counts = np.count_nonzero(valid, axis=0)
    no_return_counts = np.count_nonzero(no_return, axis=0)
    beyond_counts = np.count_nonzero(beyond_limit, axis=0)
    out_counts = np.count_nonzero(out_of_window, axis=0)
    other_counts = np.count_nonzero(other_invalid, axis=0)

    # --- per-bin median and dispersion over the filtered samples only.
    # Not nanmedian down the raw column: that would fold obstacle samples in.
    median_ranges = np.full(n_bins, np.nan)
    mad = np.full(n_bins, np.nan)
    for b in range(n_bins):
        v = samples[valid[:, b], b]
        if v.size:
            m = float(np.median(v))
            median_ranges[b] = m
            if v.size >= 2:
                mad[b] = float(np.median(np.abs(v - m)))

    # --- acceptance. offset = measured - ideal, and apply subtracts it.
    required_valid = max(1, int(math.ceil(n_frames * thresholds.min_valid_fraction)))
    required_bins = int(math.ceil(n_bins * thresholds.min_accepted_bin_fraction))

    candidate = median_ranges - ideal
    enough = valid_counts >= required_valid
    plausible = np.isfinite(candidate) & (np.abs(candidate) <= thresholds.max_abs_tare_m)
    # Catches what neither count nor magnitude can see: a bin whose samples are
    # bimodal (half on floor, half on a cable edge, both inside the window)
    # yields a median that passes every other gate and ships as trustworthy.
    dispersion_ok = np.isfinite(mad) & (mad <= thresholds.max_bin_mad_m)
    accepted = enough & plausible & dispersion_ok

    # Rejected bins get a ZERO offset and a False mask -- never a NaN offset.
    # The mask is what stops runtime application; the zero is what keeps every
    # consumer's arithmetic finite.
    offsets = np.zeros(n_bins, dtype=np.float64)
    offsets[accepted] = candidate[accepted]

    accepted_count = int(np.count_nonzero(accepted))
    sufficient = accepted_count >= required_bins   # integer compare, not a rounded fraction

    insufficient_idx = np.flatnonzero(~enough)
    implausible_idx = np.flatnonzero(enough & ~plausible)
    dispersion_idx = np.flatnonzero(enough & plausible & ~dispersion_ok)
    never_returned_idx = np.flatnonzero(valid_counts == 0)
    beyond_heavy_idx = np.flatnonzero(beyond_counts > n_frames / 2)

    # float() everywhere a numpy scalar could survive: `summary` is written
    # straight to YAML by the store, and yaml.safe_dump refuses np.float64.
    total = float(samples.size)
    pct_no_return = float(100.0 * int(no_return.sum()) / total)
    pct_beyond = float(100.0 * int(beyond_limit.sum()) / total)

    reasons = []
    if not sufficient:
        reasons.append(
            f'{n_bins - accepted_count}/{n_bins} bins rejected; at most '
            f'{n_bins - required_bins} rejected bins are allowed '
            f'(min_accepted_bin_fraction={thresholds.min_accepted_bin_fraction})')
        if insufficient_idx.size:
            reasons.append(
                f'{insufficient_idx.size} bins had fewer than {required_valid}/'
                f'{n_frames} valid frames (no-return 5.11 was {pct_no_return:.1f}% of '
                f'all samples, beyond-limit 5.09 {pct_beyond:.1f}%)')
        if implausible_idx.size:
            reasons.append(f'{implausible_idx.size} bins had |offset| > '
                           f'{thresholds.max_abs_tare_m} m')
        if dispersion_idx.size:
            reasons.append(
                f'{dispersion_idx.size} bins had MAD > '
                f'{thresholds.max_bin_mad_m * 1000:.1f} mm (robot moving, vibration, '
                f'or a non-flat patch)')
    if beyond_heavy_idx.size:
        reasons.append(
            f'{beyond_heavy_idx.size} bins saw beyond-limit (5.09) returns on a '
            f'supposedly flat floor -- the beam went past where the floor should be. '
            f'Check sensor mounting/aim and that no void or cliff is under the base.')

    warnings = []
    median_offset = float(np.median(offsets[accepted])) if accepted_count else 0.0
    if abs(median_offset) > thresholds.warn_median_offset_m:
        warnings.append(
            f'median tare across accepted bins is {median_offset * 1000:+.1f} mm; a '
            f'uniform shift that large is a geometry PARAMETER error rather than '
            f'sensor bias -- emitter_height_above_floor_mm is probably wrong')
    mad_p95 = float(np.nanpercentile(mad, 95)) if np.isfinite(mad).any() else float('nan')
    if np.isfinite(mad_p95) and mad_p95 > 0.5 * thresholds.max_bin_mad_m:
        warnings.append(f'noisy run: 95th-percentile per-bin MAD is '
                        f'{mad_p95 * 1000:.2f} mm; consider re-recording')

    def _stat(a, fn):
        return float(fn(a[np.isfinite(a)])) if np.isfinite(a).any() else float('nan')

    def _r(x, n):
        """round() that always yields a native float, so the summary stays
        YAML-serialisable."""
        return float(round(float(x), n))

    summary = {
        'frames_used': int(n_frames),
        'bins_per_frame': int(n_bins),
        'total_bin_samples': int(samples.size),
        'valid_samples': int(valid.sum()),
        'no_return_samples': int(no_return.sum()),
        'beyond_limit_samples': int(beyond_limit.sum()),
        'out_of_window_samples': int(out_of_window.sum()),
        'other_invalid_samples': int(other_invalid.sum()),
        'no_return_pct': _r(pct_no_return, 4),
        'beyond_limit_pct': _r(pct_beyond, 4),
        'bins_returned_at_least_once': int(np.count_nonzero(valid_counts > 0)),
        'bins_accepted_for_tare': accepted_count,
        'bins_rejected_for_tare': int(n_bins - accepted_count),
        'required_accepted_bins': required_bins,
        'required_valid_frames_per_bin': required_valid,
        'run_sufficient': bool(sufficient),
        'median_accepted_offset_m': _r(median_offset, 6),
        'bin_mad_m_median': _r(_stat(mad, np.median), 6),
        'bin_mad_m_p95': _r(mad_p95, 6) if np.isfinite(mad_p95) else None,
        'bin_mad_m_max': _r(_stat(mad, np.max), 6),
        'ideal_range_model': IDEAL_RANGE_MODEL,
    }

    per_bin = {
        'valid_counts': valid_counts.astype(int).tolist(),
        'no_return_counts': no_return_counts.astype(int).tolist(),
        'beyond_limit_counts': beyond_counts.astype(int).tolist(),
        'out_of_window_counts': out_counts.astype(int).tolist(),
        'other_invalid_counts': other_counts.astype(int).tolist(),
        'median_range_m': [None if not np.isfinite(v) else _r(v, 6)
                           for v in median_ranges],
        'mad_m': [None if not np.isfinite(v) else _r(v, 6) for v in mad],
    }
    indices = {
        'rejected': np.flatnonzero(~accepted).tolist(),
        'insufficient_data': insufficient_idx.tolist(),
        'implausible_offset': implausible_idx.tolist(),
        'high_dispersion': dispersion_idx.tolist(),
        'never_returned': never_returned_idx.tolist(),
        'beyond_limit_majority': beyond_heavy_idx.tolist(),
    }

    null_rate = (no_return_counts + beyond_counts).astype(np.float64) / float(n_frames)

    return SensorCalibrationResult(
        offsets=offsets, valid_mask=accepted, null_rate_per_bin=null_rate,
        summary=summary, per_bin=per_bin, indices=indices,
        sufficient=bool(sufficient), insufficiency_reasons=reasons, warnings=warnings)


def apply_tare_array(ranges, offsets, valid_mask, codes=None):
    """Correct only bins that carry a distance AND have a trustworthy tare.

    """
    ranges = np.asarray(ranges, dtype=np.float64)
    offsets = np.asarray(offsets, dtype=np.float64)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    if offsets.shape != ranges.shape or valid_mask.shape != ranges.shape:
        raise ValueError(f'tare shape {offsets.shape}/{valid_mask.shape} does not '
                         f'match ranges shape {ranges.shape}')
    if codes is None:
        measurable = np.isfinite(ranges)
    else:
        measurable = (np.asarray(codes) == protocol.CODE_VALID) & np.isfinite(ranges)
    apply = measurable & valid_mask & np.isfinite(offsets)
    # A correction that drives a bin to zero or below is nonsense; keep the raw
    # value rather than manufacture a negative range that downstream filters
    # would silently delete.
    apply &= (ranges - offsets) > 0.0
    out = ranges.copy()
    out[apply] -= offsets[apply]
    return out


def tare_diagnostics(ranges, offsets, valid_mask, codes=None):
    """Counts behind one apply_tare_array call, for tools. The hot path does
    not compute these."""
    ranges = np.asarray(ranges, dtype=np.float64)
    offsets = np.asarray(offsets, dtype=np.float64)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    if codes is None:
        measurable = np.isfinite(ranges)
        n_sentinel = int(np.count_nonzero(~measurable))
    else:
        codes = np.asarray(codes)
        measurable = (codes == protocol.CODE_VALID) & np.isfinite(ranges)
        n_sentinel = int(np.count_nonzero((codes == protocol.CODE_NO_RETURN)
                                          | (codes == protocol.CODE_BEYOND_LIMIT)))
    eligible = measurable & valid_mask & np.isfinite(offsets)
    nonpositive = eligible & ~((ranges - offsets) > 0.0)
    return {
        'n_applied': int(np.count_nonzero(eligible & ~nonpositive)),
        'n_no_tare': int(np.count_nonzero(measurable & ~valid_mask)),
        'n_sentinel': n_sentinel,
        'n_suppressed_nonpositive': int(np.count_nonzero(nonpositive)),
    }


#
# Only two things can silently invalidate a stored tare: the bin order it was
# recorded in, and the ideal range its offsets are measured against. 
def ideal_range_m(loop_params):
    """Flat-floor axial depth: what an untared bin should read."""
    geom = loop_params['line_sensor_geometry']
    h_m = float(geom['emitter_height_above_floor_mm']) / 1000.0
    return h_m / math.sin(math.radians(float(geom['sensor_angle_down_deg'])))


@dataclass
class SensorRecording:
    """One sensor's raw capture from a calibration session.

    `status` uses the same vocabulary the report and stretch_system_check
    speak, so a failure reason survives from capture to disk unchanged.
    """

    sensor_name: str
    sensor_index: int
    ranges: np.ndarray         # (n_frames, n_bins) float64
    codes: np.ndarray          # (n_frames, n_bins) uint8
    frame_id: np.ndarray       # (n_frames,) int64
    ts: np.ndarray             # (n_frames,) float64
    missed_frames: np.ndarray  # (n_frames,) int32
    status: str = 'OK'
    notes: list = field(default_factory=list)
    stats: dict = field(default_factory=dict)

    @property
    def n_frames(self):
        return int(self.ranges.shape[0]) if self.ranges.ndim == 2 else 0


@dataclass
class RecordingSession:
    """Everything captured in one calibration run, across all sensors."""

    session_id: str
    started_at: str
    ended_at: str = ''
    requested_frames: int = 0
    poll_iterations: int = 0
    stretch_body_version: str = ''
    loop_params_snapshot: dict = field(default_factory=dict)
    recordings: dict = field(default_factory=dict)     # name -> SensorRecording

    @property
    def sensor_names(self):
        return list(self.recordings)

_OFFSET_QUANTUM_M = 1e-5      # 0.01 mm; the chip itself quantises to 1 mm
_OFFSET_LIMIT_M = 32767 * _OFFSET_QUANTUM_M   # 0.327 m, > max_abs_tare_m


def pack_tare(offsets, valid_mask, null_rate_per_bin):
    """Compact one sensor's tare for the status dict.

    Lossy by design and bounded: offsets to 0.01 mm, null rates to 1/255.

    """
    offsets = np.asarray(offsets, dtype=np.float64)
    valid_mask = np.asarray(valid_mask, dtype=bool)
    null_rate = np.asarray(null_rate_per_bin, dtype=np.float64)
    if offsets.shape != valid_mask.shape:
        raise ValueError(f'offsets {offsets.shape} and mask {valid_mask.shape} differ')
    if null_rate.shape != offsets.shape:
        null_rate = np.zeros(offsets.shape, dtype=np.float64)
    if not np.isfinite(offsets).all():
        raise ValueError('cannot pack non-finite offsets')
    if np.abs(offsets).max(initial=0.0) > _OFFSET_LIMIT_M:
        raise ValueError(f'offset exceeds the {_OFFSET_LIMIT_M:.3f} m wire range')
    return {
        'n_bins': int(offsets.size),
        'offsets_q': np.round(offsets / _OFFSET_QUANTUM_M).astype(np.int16),
        'valid_packed': np.packbits(valid_mask),
        'null_rate_q': np.round(np.clip(null_rate, 0.0, 1.0) * 255.0).astype(np.uint8),
    }


def unpack_tare(block):
    """Inverse of pack_tare. Returns (offsets, valid_mask, null_rate)."""
    n_bins = int(block['n_bins'])
    offsets = np.asarray(block['offsets_q'], dtype=np.float64) * _OFFSET_QUANTUM_M
    # unpackbits always returns a multiple of 8; trim back to the bin count.
    valid = np.unpackbits(np.asarray(block['valid_packed'], dtype=np.uint8))[:n_bins].astype(bool)
    null_rate = np.asarray(block['null_rate_q'], dtype=np.float64) / 255.0
    if offsets.size != n_bins or valid.size != n_bins or null_rate.size != n_bins:
        raise ValueError(f'tare block claims {n_bins} bins but carries '
                         f'{offsets.size}/{valid.size}/{null_rate.size}')
    return offsets, valid, null_rate
