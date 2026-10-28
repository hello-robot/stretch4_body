"""Every tunable in one place.

`LineSensorConfig` is the single knob-board for the whole pipeline. Each field
is read by exactly one stage; the grouping follows the stages so the knobs for
a behaviour sit together. A ROS param override where the node is built
supersedes these defaults.

Three knobs the old filter needed are GONE, and their absence is the point:
`null_range_m` (5.11), `null_tolerance_m`, and `null_sentinel_min_m` (4.0).
They existed to recover, by float comparison, a classification the chip had
already made -- after the tare had shifted the value, so a tared 5.09 could
land on 5.11 and destroy the cliff/dark-floor distinction. The reader now
publishes `codes`, so that recovery is one integer compare and has no knobs.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LineSensorConfig:
    # --- Stage: classify (classify.py) --------------------------------------
    line_obstacle_min_height_m: float = 0.025
    floor_band_m: float = 0.015

    use_range_deviation: bool = True
    dev_floor_band_m: float = 0.010
    dev_obstacle_strong_m: float = 0.020
    marginal_min_run_bins: int = 5
    strong_confirm_frames: int = 1
    marginal_confirm_frames: int = 2
    # Marginal evidence is judged on LOCAL contrast (z minus the rolling
    # median of surrounding bins).
    marginal_contrast_min_m: float = 0.010
    contrast_window_bins: int = 81

    cliff_min_drop_m: float = 0.02
    cliff_max_drop_m: float = 0.10
    # Deep drops: returning bins deeper than cliff_max_drop classify as
    # DEEP_DROP and publish on the deep-drop output alongside cliff-typed
    # null runs.
    use_deep_drop: bool = True
    # Depth under-read correction: the sensor reads drops shallow by a roughly
    # depth-proportional factor. 1.0 disables.
    depth_underread_scale: float = 0.91

    # --- Stage: null evidence (nulls.py) ------------------------------------
    use_null_evidence: bool = True
    # Chronic-null prior: a bin only carries null evidence if it demonstrably
    # returned on clear floor during calibration. The body measures this per
    # bin (null_rate_per_bin) and serves it; the tare mask is the fallback.
    # Without it a dirty lens manufactures cliffs instead of reading degraded.
    chronic_null_rate_max: float = 0.10
    null_min_run_bins: int = 8
    null_persist_min_fraction: float = 0.6
    suppression_near_range_m: float = 0.15
    shadow_adjacency_bins: int = 3
    cliff_adjacent_drop_bins: int = 6
    cliff_bearing_adjacency_deg: float = 15.0

    # A run is void-typed when enough of its bins carry CODE_BEYOND_LIMIT --
    # the beam travelled past where the floor should be, which is the
    # strongest cliff evidence the hardware can give.
    use_far_sentinel_void: bool = True
    void_far_sentinel_min_fraction: float = 0.02
    void_far_sentinel_min_bins: int = 3
    # Null runs on OTHER sensors whose bearings fall inside a void's bearing
    # span (plus this margin) are cliff-typed too: a ledge is continuous
    # across the floor and does not stop at a sensor boundary.
    void_bearing_adjacency_deg: float = 20.0

    # Degraded sectors: a sensor whose reliable bins are mostly nulls with no
    # benign explanation has lost floor coverage. Published so the hazard
    # layer can slow through that sector rather than stop.
    degraded_min_fraction: float = 0.35
    # Anti-strobe: reflective floors sit in the 0.25-0.45 band around the
    # threshold, so the fraction is EMA-smoothed and the state has hysteresis.
    degraded_exit_fraction: float = 0.25
    degraded_frac_alpha: float = 0.1

    # --- Stage: shape (shape.py) --------------------------------------------
    line_min_run_bins: int = 3
    line_max_run_radial_span_m: float = 0.25
    line_point_noise_max_run_bins: int = 12
    line_point_noise_xy_span_max_m: float = 0.010
    line_point_noise_radial_span_max_m: float = 0.010
    line_spray_merge_gap_bins: int = 6
    line_radial_streak_head_radius_max_m: float = 0.35
    line_radial_streak_span_min_m: float = 0.04
    line_radial_streak_angular_spread_max_deg: float = 20.0
    line_radial_streak_aspect_ratio_min: float = 3.0
    spray_min_run_bins: int = 3
    spray_roughness_thresh_m: float = 0.03
    spray_max_run_bins: int = 0
    spray_head_radius_max_m: float = 0.30
    spray_radial_span_min_m: float = 0.05
    spray_angular_spread_max_deg: float = 15.0
    spray_aspect_ratio_min: float = 5.0
    spray_direction_cluster_gap_deg: float = 5.0
    spray_monotonic_score_min: float = 0.70
    spray_monotonic_tolerance_m: float = 0.005
    spray_short_run_bonus_max_bins: int = 15
    spray_temporal_window_frames: int = 5
    spray_temporal_stable_min_frames: int = 2
    spray_temporal_stable_fraction: float = 0.50

    # --- Stage: confirm (confirm.py) ----------------------------------------
    line_confirm_frames: int = 3
    line_fast_confirm_frames: int = 2
    line_fast_confirm_range_m: float = 0.55
    line_window_frames: int = 4
    line_require_consecutive: bool = True

    # --- Stage: gloss (gloss.py) --------------------------------------------
    # Glossy-floor defence. Suppressed candidates are rerouted to SPRAY so they
    # stay visible on the spray debug topic rather than vanishing.
    use_spray_bin_quarantine: bool = True
    # EMA decay per frame for the flip counter (~2 s memory at 30 Hz).
    spray_flip_decay: float = 0.967
    spray_flip_suspect_threshold: float = 4.0
    # Cross-sensor gate: this many sensors each showing at least min_bins
    # hazard candidates nearer than near_range makes near-field arcs on ALL
    # sensors spray for that frame -- no single object could cause that.
    spray_cross_sensor_min_sensors: int = 3
    spray_cross_sensor_min_bins: int = 8
    spray_cross_sensor_near_range_m: float = 0.20
    # A near-field run is exempt when it looks like a solid object pressed
    # against the base.
    spray_cross_sensor_exempt_run_bins: int = 60
    spray_cross_sensor_exempt_max_ragged_m: float = 0.006
    spray_suspect_confirm_min_bins: int = 10
    # The flip counter has no history right after startup, so the stricter
    # confirmation also applies to every sensor for this many frames.
    spray_warmup_frames: int = 30
