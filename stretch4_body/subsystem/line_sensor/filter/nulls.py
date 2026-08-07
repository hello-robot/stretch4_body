"""Reading the silences: no-return bins as evidence of cliffs and lost coverage.

Every other stage works on bins that *return* a range. This one works on the
bins that don't, and it is where the two status codes earn their keep:

  CODE_BEYOND_LIMIT (5.09) -- something returned from PAST the range limit.
      The sensors point down, so the beam travelled beyond where the floor
      should be. This is the strongest cliff evidence the hardware can give.
  CODE_NO_RETURN (5.11) -- nothing came back at all. Genuinely ambiguous:
      dark floor, sunlight, a glossy surface angled away, or a void.

A trusted null run is typed by its context: proven a void by the far code,
shadowed by an obstacle, suppressed by a near bright return, next to a drop, or
simply dark floor. Trusted nulls that stay unexplained measure lost floor
coverage and, once a smoothed hysteretic fraction crosses the threshold,
publish as degraded.

WHY THIS MODULE WAS REWRITTEN. The previous version asked which bins were
no-returns by comparing floats: `isfinite(r) & (r > 4.0)` for the far code and
`isclose(r, 5.11)` for the null. Under the current reader `ranges` is NaN at
every non-measurement bin, so `isfinite` is False for all of them and BOTH
masks matched nothing -- every cliff was silently discarded while all six
sensors reported 30 Hz and system check passed. Worse, the comparison ran
AFTER the tare, so a tared 5.09 could land within tolerance of 5.11 and lose
the cliff/dark-floor distinction outright. Codes are assigned once at decode,
before any arithmetic touches the value, and compared as integers.
"""

from __future__ import annotations

import numpy as np

from stretch4_body.subsystem.line_sensor import protocol

from .arrays import measuring, run_ids, runs
from .config import LineSensorConfig
from .geometry import Projector
from .hits import BinClass, DROP_FAMILY, OBSTACLE_FAMILY


class NullEvidenceDetector:
    def __init__(
        self,
        config: LineSensorConfig,
        projector: Projector,
        bin_reliable: dict,
        bin_null_rate: dict,
    ):
        self.config = config
        self.projector = projector
        self.bin_reliable = bin_reliable
        self.bin_null_rate = bin_null_rate
        # Per-sensor last-frame null mask (persistence check).
        self._prev_null: dict = {}
        # Degraded hysteresis state: smoothed blind fraction and on/off latch.
        self._deg_frac_ema: dict = {}
        self._deg_active: dict = {}
        self.last_void_runs: list = []

    def detect(self, sensor_frames, gated):
        """Type the silences.

        `sensor_frames` maps sensor_idx -> (name, ranges, codes).
        Returns (cliff_xy, cliff_id, benign_xy, benign_id, degraded_xy,
        degraded_id) -- each xy paired with the (sensor, bin) that produced it.
        """
        cfg = self.config
        empty = np.zeros((0, 2))
        self.last_void_runs = []
        no_ids = np.zeros((0, 2), np.int32)
        if not cfg.use_null_evidence:
            return empty, no_ids, empty, no_ids, empty, no_ids

        drop_bins: dict = {}
        obstacle_bins: dict = {}
        spray_bins: dict = {}
        for sensor_idx, bin_idx, cls, _pt in gated:
            if cls in DROP_FAMILY:
                drop_bins.setdefault(sensor_idx, []).append(bin_idx)
            elif cls in OBSTACLE_FAMILY:
                obstacle_bins.setdefault(sensor_idx, []).append(bin_idx)
            elif cls == BinClass.SPRAY:
                spray_bins.setdefault(sensor_idx, []).append(bin_idx)

        drop_bearings: dict = {
            sensor_idx: self.projector.bin_bearings(sensor_idx)[np.array(bins, dtype=int)]
            for sensor_idx, bins in drop_bins.items()
        }

        cliff_pts: list = []
        benign_pts: list = []
        degraded_pts: list = []
        # Identity travels with every published point: which sensor, which bin.
        cliff_ids: list = []
        benign_ids: list = []
        degraded_ids: list = []
        new_prev: dict = {}

        # --- Pass 1: gate the runs, and mark the ones the far code proves are
        # voids.
        prepared: list = []
        void_bearings: dict = {}
        for sensor_idx, (sensor_name, ranges, codes) in sensor_frames.items():
            null = self._no_return_mask(codes)
            prev = self._prev_null.get(sensor_idx)
            new_prev[sensor_idx] = null

            # A null is only evidence if the bin is known to return on clear
            # floor. Without this a chronically blind bin -- dirty lens, dead
            # pixel, a bracket clipping the view -- manufactures a cliff on
            # every frame instead of reading as degraded coverage.
            evidence = null.copy()
            trusted = self._null_trusted(sensor_name, len(evidence))
            evidence &= trusted
            reliable_count = max(int(trusted.sum()), 1)

            # Exposure suppression: one bright near return can blank the rest
            # of the array. Spray points should not count as suppressors.
            valid = measuring(codes)
            suppressors = valid & (ranges < cfg.suppression_near_range_m)
            own_spray = spray_bins.get(sensor_idx)
            if own_spray:
                suppressors[np.array(own_spray, dtype=int)] = False
            near_suppressor = bool(np.any(suppressors))

            far = self._far_code_mask(codes) & evidence

            bearings = self.projector.bin_bearings(sensor_idx)
            gated_runs: list = []
            for start, end in runs(np.flatnonzero(evidence)):
                if end - start + 1 < cfg.null_min_run_bins:
                    continue
                # Persistence: the run must have been mostly null last frame
                # too (2-frame latency, matching marginal obstacle confirm).
                if prev is None or len(prev) <= end:
                    continue
                if float(np.mean(prev[start:end + 1])) < cfg.null_persist_min_fraction:
                    continue
                void = self._is_void_run(far, start, end)
                if void:
                    void_bearings.setdefault(sensor_idx, []).append(
                        bearings[start:end + 1])
                    self.last_void_runs.append((sensor_idx, start, end))
                gated_runs.append((start, end, void))

            prepared.append((
                sensor_idx, evidence, reliable_count, near_suppressor, gated_runs,
            ))

        # --- Pass 2: type each gated run and account for the leftovers.
        for sensor_idx, evidence, reliable_count, near_suppressor, gated_runs in prepared:
            explained = np.zeros(len(evidence), dtype=bool)

            # Every null output is published at the EXPECTED floor
            # intersection, which is the nearest place the hazard can be.
            floor_xy = self.projector.floor_intersections(sensor_idx)
            bearings = self.projector.bin_bearings(sensor_idx)
            own_drops = np.array(sorted(drop_bins.get(sensor_idx, [])), dtype=int)
            own_obstacles = np.array(sorted(obstacle_bins.get(sensor_idx, [])), dtype=int)
            other_drop_bearings = self._other_bearings(drop_bearings, sensor_idx)
            other_void_bearings = self._other_bearings(void_bearings, sensor_idx)

            for start, end in ((run[0], run[1]) for run in gated_runs if run[2]):
                cliff_pts.append(floor_xy[start:end + 1])
                cliff_ids.append(run_ids(sensor_idx, start, end))
                explained[start:end + 1] = True

            for start, end, void in gated_runs:
                if void:
                    continue
                run_pts = floor_xy[start:end + 1]
                if self._bearing_overlap(bearings, start, end, other_void_bearings,
                                         cfg.void_bearing_adjacency_deg):
                    cliff_pts.append(run_pts)  # same edge, seen by a neighbour
                    cliff_ids.append(run_ids(sensor_idx, start, end))
                    explained[start:end + 1] = True
                    continue
                if len(own_obstacles) and bool(np.any(
                    (own_obstacles >= start - cfg.shadow_adjacency_bins)
                    & (own_obstacles <= end + cfg.shadow_adjacency_bins)
                )):
                    benign_pts.append(run_pts)  # occlusion shadow
                    benign_ids.append(run_ids(sensor_idx, start, end))
                    explained[start:end + 1] = True
                    continue
                if near_suppressor:
                    benign_pts.append(run_pts)  # exposure suppression
                    benign_ids.append(run_ids(sensor_idx, start, end))
                    explained[start:end + 1] = True
                    continue

                cliff = bool(len(own_drops)) and bool(np.any(
                    (own_drops >= start - cfg.cliff_adjacent_drop_bins)
                    & (own_drops <= end + cfg.cliff_adjacent_drop_bins)
                ))
                if not cliff:
                    cliff = self._bearing_overlap(
                        bearings, start, end, other_drop_bearings,
                        cfg.cliff_bearing_adjacency_deg)

                if cliff:
                    cliff_pts.append(run_pts)
                    cliff_ids.append(run_ids(sensor_idx, start, end))
                    explained[start:end + 1] = True
                else:
                    benign_pts.append(run_pts)  # dark floor: benign, unexplained
                    benign_ids.append(run_ids(sensor_idx, start, end))

            unexplained = evidence & ~explained
            frac = float(unexplained.sum()) / reliable_count
            alpha = min(max(cfg.degraded_frac_alpha, 0.0), 1.0)
            ema = self._deg_frac_ema.get(sensor_idx, frac)
            ema = ema + alpha * (frac - ema)
            self._deg_frac_ema[sensor_idx] = ema
            active = self._deg_active.get(sensor_idx, False)
            if active:
                active = ema >= min(cfg.degraded_exit_fraction, cfg.degraded_min_fraction)
            else:
                active = ema >= cfg.degraded_min_fraction
            self._deg_active[sensor_idx] = active
            if active and unexplained.any():
                bins = np.flatnonzero(unexplained).astype(np.int32)
                degraded_pts.append(floor_xy[unexplained])
                degraded_ids.append(np.column_stack(
                    [np.full(bins.size, sensor_idx, np.int32), bins]))

        self._prev_null = new_prev
        no_ids = np.zeros((0, 2), np.int32)

        def stack(pts, ids):
            return (np.vstack(pts) if pts else empty,
                    np.vstack(ids) if ids else no_ids)

        return (stack(cliff_pts, cliff_ids)
                + stack(benign_pts, benign_ids)
                + stack(degraded_pts, degraded_ids))

    def forget_sensor(self, sensor_idx: int) -> None:
        """Drop a sensor's history when it stops contributing.

        A sensor that goes dead mid-run must not come back and be compared
        against a null mask from before the dropout -- the persistence gate
        would pass instantly on stale evidence.
        """
        self._prev_null.pop(sensor_idx, None)
        self._deg_frac_ema.pop(sensor_idx, None)
        self._deg_active.pop(sensor_idx, None)

    @staticmethod
    def _other_bearings(by_sensor: dict, sensor_idx: int) -> np.ndarray:
        """Every bearing recorded for a sensor other than this one, flattened."""
        parts: list = []
        for other_idx, value in by_sensor.items():
            if other_idx == sensor_idx:
                continue
            if isinstance(value, list):
                parts.extend(value)
            else:
                parts.append(value)
        return np.concatenate(parts) if parts else np.zeros(0)

    @staticmethod
    def _bearing_overlap(bearings, start, end, other, margin_deg) -> bool:
        """Whether another sensor is looking within `margin_deg` of this run's
        angular span."""
        if not np.size(other):
            return False
        mid = 0.5 * (bearings[start] + bearings[end])
        half_span = 0.5 * abs(bearings[end] - bearings[start])
        delta = np.abs((other - mid + 180.0) % 360.0 - 180.0)
        return bool(np.any(delta <= half_span + margin_deg))

    def _is_void_run(self, far: np.ndarray, start: int, end: int) -> bool:
        """Whether the run carries enough far codes to call it a void."""
        cfg = self.config
        if not cfg.use_far_sentinel_void:
            return False
        count = int(far[start:end + 1].sum())
        if count < cfg.void_far_sentinel_min_bins:
            return False
        return count / float(end - start + 1) >= cfg.void_far_sentinel_min_fraction

    @staticmethod
    def _far_code_mask(codes: np.ndarray) -> np.ndarray:
        """5.09: a return from past the range limit. The sensors point down,
        so the beam went beyond where the floor should be -- the floor is
        gone. One integer compare; no tolerance, no threshold."""
        return np.asarray(codes) == protocol.CODE_BEYOND_LIMIT

    @staticmethod
    def _no_return_mask(codes: np.ndarray) -> np.ndarray:
        """Every bin that is not a distance measurement -- 5.11, 5.09, and
        anything the decoder could not classify."""
        return ~measuring(codes)

    def _null_trusted(self, sensor_name: str, bin_count: int) -> np.ndarray:
        """Bins whose nulls carry evidence: calibration null rate below the
        chronic threshold, falling back to the tare mask when no rates exist,
        and to 'trust everything' when the sensor is uncalibrated."""
        rates = self.bin_null_rate.get(sensor_name)
        if rates is not None and len(rates) == bin_count:
            return (
                np.asarray(rates, dtype=np.float64)
                <= self.config.chronic_null_rate_max
            )
        reliable = self.bin_reliable.get(sensor_name)
        if reliable is not None and len(reliable) == bin_count:
            return np.asarray(reliable, dtype=bool)
        return np.ones(bin_count, dtype=bool)
