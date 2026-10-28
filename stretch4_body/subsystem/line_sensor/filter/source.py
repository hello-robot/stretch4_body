"""The orchestrator: one class, one `process()` per frame.

`LineSensorSource` owns the per-frame state (confirmation histories, frame
counter) and wires the stages together. `process()` reads top-to-bottom as the
pipeline: classify each returning bin, quarantine glossy phantoms, shape-gate
the runs, confirm across frames, and -- in parallel -- read the silences for
cliffs and lost coverage.

TWO CONTRACT RULES THIS FILE ENFORCES, both easy to get silently wrong:

1. A DEAD SENSOR'S `ranges` IS STALE, NOT EMPTY. The body deliberately keeps
   the last good scan in `status[name]['ranges']` when a sensor stops
   reporting -- liveness lives in `status['health']`, and the array stays put
   so consumers can choose. If you iterate sensor names and read `ranges`
   without consulting health, a sensor whose cable fell out an hour ago keeps
   projecting a full, plausible, completely fictional floor forever. There is
   nothing in the array itself to warn you. Hence `_live_sensors()`.

2. PER-BIN CONFIRMATION IS DE-FLICKER ONLY. It requires a hazard to appear on
   N consecutive frames at the same bin index. On a moving base the same bin
   index looks at different ground each frame, so this is NOT world
   persistence and must not be relied on as such. Accumulating evidence over
   actual ground is a separate downstream job (the odom-frame rolling grid in
   the hazard layer). The two are complementary; neither replaces the other.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from .arrays import (
    as_code_array, as_range_array, items_to_ids, items_to_xy, items_to_z,
    measuring)
from .classify import classify_bin
from .config import LineSensorConfig
from .confirm import bin_confirmed, confirm_frames_for_bin
from .geometry import Projector
from .gloss import FlipTracker, quarantine_spray_candidates
from .hits import (
    BinClass,
    DROP_FAMILY,
    LineSensorHits,
    OBSTACLE_FAMILY,
    family,
)
from .nulls import NullEvidenceDetector
from .shape import ShapeGate


class LineSensorSource:
    """Turn a line_sensor_loop status dict into hazard evidence."""

    def __init__(
        self,
        geometry,
        sensor_names: list,
        config: LineSensorConfig,
        apply_tare=None,
        bin_reliable=None,
        bin_null_rate=None,
    ):
        self.geometry = geometry
        self.sensor_names = list(sensor_names)
        self.config = config
        # Callable(ranges, sensor_name, codes) -> tared ranges. Pass the
        # client's bound method: it corrects only bins whose code is VALID and
        # whose tare the body accepted, so status-code bins pass through
        # untouched and a bin with no trustworthy tare stays raw.
        self.apply_tare = apply_tare
        # Per-sensor bool arrays: bins with a valid tare floor reference. Fine
        # deviation classification is only allowed on reliable bins; untared
        # bins fall back to the coarse absolute-height bands.
        self.bin_reliable = bin_reliable or {}
        # Per-sensor float arrays: fraction of calibration frames each bin
        # returned null on clear floor. Gates null evidence and the degraded
        # denominator -- a different question from "has a floor reference".
        self.bin_null_rate = bin_null_rate or {}

        # Stage helpers, built once (never per frame).
        self.projector = Projector(geometry, config)
        self.flip_tracker = FlipTracker(config)
        self.shape_gate = ShapeGate(config, self.projector)
        self.null_detector = NullEvidenceDetector(
            config, self.projector, self.bin_reliable, self.bin_null_rate)

        self._frames_processed = 0
        self._history: deque = deque(
            maxlen=max(
                config.line_window_frames,
                config.line_confirm_frames,
                config.line_fast_confirm_frames,
                1,
            ),
        )
        self._raw_history: deque = deque(
            maxlen=max(
                config.line_window_frames,
                config.spray_temporal_window_frames,
                1,
            ),
        )

    # -- liveness ----------------------------------------------------------

    def _live_sensors(self, status: dict):
        """Names safe to read this frame, and why each of the others is not.

        Reads `status['health']`, which the loop republishes on EVERY cycle
        even when no frame arrived -- that is what makes it correct during a
        dropout, when the per-sensor blocks have stopped updating.
        """
        health = status.get('health') or {}
        dead = set(health.get('sensors_dead', ()) or ())
        disabled = set(health.get('disabled_sensors', ()) or ())
        # A paused or closed port means nothing in the status is live, however
        # recent the arrays look.
        port_down = not bool(health.get('port_open', True))
        paused = not bool(health.get('streaming', True))

        live, skipped = [], {}
        for name in self.sensor_names:
            if port_down:
                skipped[name] = 'port_closed'
            elif paused:
                skipped[name] = 'streaming_off'
            elif name in dead:
                skipped[name] = 'dead'
            elif name in disabled:
                skipped[name] = 'disabled'
            else:
                live.append(name)
        return live, skipped

    def _frames(self, status: dict):
        """sensor_idx -> (name, tared ranges, codes) for every live sensor."""
        live, skipped = self._live_sensors(status)
        frames = {}
        for sensor_idx, sensor_name in enumerate(self.sensor_names):
            if sensor_name not in live:
                # Its history is now meaningless; drop it so a reconnect does
                # not confirm instantly against pre-dropout evidence.
                self.null_detector.forget_sensor(sensor_idx)
                continue
            sensor_status = status.get(sensor_name)
            if not isinstance(sensor_status, dict):
                skipped[sensor_name] = 'no_status'
                continue
            ranges = as_range_array(sensor_status.get('ranges'))
            if ranges.size == 0:
                skipped[sensor_name] = 'no_data'
                continue
            codes = as_code_array(sensor_status.get('codes'), ranges.size)
            if self.apply_tare is not None:
                ranges = as_range_array(
                    self.apply_tare(ranges, sensor_name, codes))
            frames[sensor_idx] = (sensor_name, ranges, codes)
        return frames, tuple(live), skipped

    # -- the frame ---------------------------------------------------------

    def project_all(self, status: dict) -> np.ndarray:
        """Every measuring bin as an (N, 3) cloud. Debug / visualisation."""
        points = []
        frames, _live, _skipped = self._frames(status)
        for sensor_idx, (_name, ranges, codes) in frames.items():
            valid = measuring(codes)
            if not np.any(valid):
                continue
            points.append(self.projector.project(sensor_idx, ranges)[valid])
        return np.vstack(points) if points else np.zeros((0, 3))

    def process(self, status: dict) -> LineSensorHits:
        cfg = self.config
        candidates: list = []
        suspect_bins: dict = {}
        near_sensor_count = 0

        sensor_frames, live, skipped = self._frames(status)

        # --- Stage 1: classify every returning bin --------------------------
        for sensor_idx, (sensor_name, ranges, codes) in sensor_frames.items():
            valid = measuring(codes)
            projected = self.projector.project(sensor_idx, ranges)
            contrast = self.projector.local_contrast(projected[:, 2], codes)
            reliable = self.bin_reliable.get(sensor_name)

            if cfg.use_spray_bin_quarantine:
                hazard_band = valid & (projected[:, 2] > cfg.dev_floor_band_m)
                suspect_bins[sensor_idx] = self.flip_tracker.update(
                    sensor_idx, hazard_band)
                if int(np.count_nonzero(
                    hazard_band & (ranges < cfg.spray_cross_sensor_near_range_m)
                )) >= cfg.spray_cross_sensor_min_bins:
                    near_sensor_count += 1

            for bin_idx in np.flatnonzero(valid):
                bin_idx = int(bin_idx)
                pt = projected[bin_idx]
                bin_ok = (bool(reliable[bin_idx])
                          if reliable is not None and bin_idx < len(reliable)
                          else True)
                cls = classify_bin(cfg, pt[2], contrast[bin_idx], bin_ok)
                if cls in OBSTACLE_FAMILY or cls in DROP_FAMILY:
                    candidates.append((sensor_idx, bin_idx, cls, pt))

        # --- Stage 2: glossy-floor quarantine -------------------------------
        candidates, quarantined = quarantine_spray_candidates(
            candidates, sensor_frames, suspect_bins, near_sensor_count, cfg)
        self.last_quarantined = quarantined
        self.last_suspect_bins = suspect_bins
        self.last_near_sensor_count = near_sensor_count

        # History stores the family class so a bin flapping between strong and
        # marginal keeps its confirmation streak.
        self._raw_history.append({
            (sensor_idx, bin_idx): family(cls)
            for sensor_idx, bin_idx, cls, _pt in candidates
        })

        # --- Stage 3: spatial (shape) gate ----------------------------------
        gated = self.shape_gate.gate(candidates, list(self._raw_history))
        gated.extend(quarantined)
        self.last_raw_candidates = candidates
        self.last_gated = gated
        self._history.append({
            (sensor_idx, bin_idx): family(cls)
            for sensor_idx, bin_idx, cls, _pt in gated
        })

        # --- Stage 4: temporal confirmation (de-flicker) --------------------
        self._frames_processed += 1
        warmup = self._frames_processed <= cfg.spray_warmup_frames
        promoted: list = []
        for sensor_idx, bin_idx, cls, pt in gated:
            if cls == BinClass.SPRAY:
                continue
            confirm_frames = confirm_frames_for_bin(cfg, pt, cls)
            if cfg.use_spray_bin_quarantine and (warmup or (
                sensor_idx in suspect_bins
                and int(np.count_nonzero(suspect_bins[sensor_idx]))
                >= cfg.spray_suspect_confirm_min_bins
            )):
                confirm_frames = max(confirm_frames, 2)
            if bin_confirmed(
                self._history, sensor_idx, bin_idx, family(cls),
                confirm_frames, cfg.line_require_consecutive,
            ):
                promoted.append((sensor_idx, bin_idx, cls, pt))

        self.last_promoted = promoted

        # --- Stage 5: null evidence (runs in parallel on full arrays) -------
        (cliff_xy, cliff_id, benign_xy, _benign_id,
         degraded_xy, degraded_id) = self.null_detector.detect(sensor_frames, gated)

        # --- Stage 6: package outputs ---------------------------------------
        # Obstacles publish where the beam hit -- that IS the object. Drops
        # publish at the expected floor intersection: a long reading means the
        # beam flew over a ledge, so the measured point lies beyond the edge
        # and steering to it puts the wheel over.
        near = dict(projector=self.projector, near_edge=True)
        return LineSensorHits(
            obstacle_xy=items_to_xy(promoted, OBSTACLE_FAMILY),
            small_drop_xy=items_to_xy(promoted, BinClass.SMALL_DROP, **near),
            deep_drop_xy=items_to_xy(promoted, BinClass.DEEP_DROP, **near),
            probable_cliff_xy=cliff_xy,
            degraded_xy=degraded_xy,
            obstacle_z=items_to_z(promoted, OBSTACLE_FAMILY),
            small_drop_z=items_to_z(promoted, BinClass.SMALL_DROP),
            deep_drop_z=items_to_z(promoted, BinClass.DEEP_DROP),
            probable_cliff_z=np.zeros(len(cliff_xy)),
            degraded_z=np.zeros(len(degraded_xy)),
            obstacle_id=items_to_ids(promoted, OBSTACLE_FAMILY),
            small_drop_id=items_to_ids(promoted, BinClass.SMALL_DROP),
            deep_drop_id=items_to_ids(promoted, BinClass.DEEP_DROP),
            probable_cliff_id=cliff_id,
            degraded_id=degraded_id,
            observed_sensors=live,
            skipped_sensors=skipped,
            benign_null_xy=benign_xy,
            raw_obstacle_xy=items_to_xy(candidates, OBSTACLE_FAMILY),
            raw_small_drop_xy=items_to_xy(candidates, DROP_FAMILY, **near),
            spatial_obstacle_xy=items_to_xy(gated, OBSTACLE_FAMILY),
            spatial_small_drop_xy=items_to_xy(gated, DROP_FAMILY, **near),
            raw_spray_xy=items_to_xy(gated, BinClass.SPRAY),
            raw_marginal_obstacle_xy=items_to_xy(candidates, BinClass.OBSTACLE_MARGINAL),
        )
