"""The hazard filter, against the current reader contract.

The tests that matter here are about the two ways this filter can be wrong
while looking completely healthy:

  * It can read a sensor that stopped reporting. The body keeps a dead
    sensor's last good `ranges` array on purpose -- liveness lives in
    `health` -- so a consumer that ignores health projects an hour-old scan
    as live floor and nothing in the data says otherwise.

  * It can discard every cliff. The previous filter found no-return bins by
    magnitude (`isfinite(r) & (r > 4.0)`), but `ranges` is NaN at every
    non-measurement bin now, so that matched NOTHING -- silently, at 30 Hz,
    with system check passing. `test_the_regression_that_motivated_this` is
    that failure, pinned.
"""

import numpy as np
import pytest

from stretch4_body.subsystem.line_sensor import protocol
from stretch4_body.subsystem.line_sensor.filter import (
    BinClass,
    LineSensorConfig,
    LineSensorSource,
    measuring,
)
from stretch4_body.subsystem.line_sensor.line_sensor_utils import LineSensorGeometry

NAMES = ['sensor_%d' % i for i in range(6)]
NBINS = 320


def geometry():
    return LineSensorGeometry({})


def ideal_range():
    g = geometry()
    return (g.param_height_cm / 100.0) / np.sin(np.deg2rad(g.angle_down_deg))


def sensor_block(ranges, codes, frame_id=1):
    return {'ranges': np.asarray(ranges, float), 'codes': np.asarray(codes, np.uint8),
            'frame_id': frame_id, 'rate_hz': 30.0, 'missed_frames': 0,
            'ts_last_read': 1.0, 'enabled': True,
            'n_no_return': int(np.count_nonzero(np.asarray(codes) == protocol.CODE_NO_RETURN)),
            'n_beyond_limit': int(np.count_nonzero(np.asarray(codes) == protocol.CODE_BEYOND_LIMIT))}


def clear_floor():
    """Every bin a valid measurement at the ideal range."""
    return (np.full(NBINS, ideal_range()), np.zeros(NBINS, np.uint8))


def status(per_sensor=None, dead=(), disabled=(), port_open=True, streaming=True):
    """A whole line_sensor_loop status dict, health included."""
    st = {'health': {'sensors_dead': list(dead), 'disabled_sensors': list(disabled),
                     'port_open': port_open, 'streaming': streaming,
                     'rate_hz': 30.0, 'reader_restarts': 0, 'decode_errors': 0}}
    for name in NAMES:
        r, c = clear_floor()
        st[name] = sensor_block(r, c)
    for name, block in (per_sensor or {}).items():
        st[name] = block
    return st


def source(**kw):
    return LineSensorSource(geometry(), NAMES, LineSensorConfig(), **kw)


def run_frames(src, st, n=4):
    """Several identical frames: null evidence needs persistence, hazards need
    confirmation, so a single frame proves nothing either way."""
    hits = None
    for _ in range(n):
        hits = src.process(st)
    return hits


# ---------------------------------------------------------------------------
# the regression this rewrite exists for
# ---------------------------------------------------------------------------

def test_the_regression_that_motivated_this():
    """A blind sensor must produce null evidence, not silence.

    Old code did `np.isfinite(ranges) & (ranges > 4.0)`. Every one of these
    bins is NaN, so that expression is all-False and the whole fan reads as
    'nothing to report' -- indistinguishable from clear floor.
    """
    blind_r = np.full(NBINS, np.nan)
    blind_c = np.full(NBINS, protocol.CODE_NO_RETURN, np.uint8)

    old_style = np.isfinite(blind_r) & (blind_r > 4.0)
    assert not old_style.any(), 'premise: the old magnitude test finds nothing'

    assert (~measuring(blind_c)).all(), 'codes see every bin as a non-measurement'


def test_a_blind_sensor_is_not_read_as_clear_floor():
    blind_r = np.full(NBINS, np.nan)
    blind_c = np.full(NBINS, protocol.CODE_NO_RETURN, np.uint8)
    st = status({'sensor_2': sensor_block(blind_r, blind_c)})
    hits = run_frames(source(), st)
    # Unexplained blindness is either typed as a hazard or accounted as lost
    # coverage. What it must never be is nothing at all.
    assert len(hits.probable_cliff_xy) or len(hits.benign_null_xy) or len(hits.degraded_xy)


# ---------------------------------------------------------------------------
# codes carry the cliff
# ---------------------------------------------------------------------------

def test_a_beyond_limit_run_is_typed_a_probable_cliff():
    """5.09 means the beam travelled past where the floor should be."""
    r, c = clear_floor()
    r = r.copy(); c = c.copy()
    r[100:160] = np.nan
    c[100:160] = protocol.CODE_BEYOND_LIMIT
    hits = run_frames(source(), status({'sensor_0': sensor_block(r, c)}))
    assert len(hits.probable_cliff_xy) > 0


def test_no_return_alone_is_not_promoted_to_a_cliff():
    """5.11 is ambiguous -- dark floor, sunlight, a void. Without corroboration
    it is benign, otherwise every dark doormat becomes a cliff."""
    r, c = clear_floor()
    r = r.copy(); c = c.copy()
    r[100:160] = np.nan
    c[100:160] = protocol.CODE_NO_RETURN
    hits = run_frames(source(), status({'sensor_0': sensor_block(r, c)}))
    assert len(hits.probable_cliff_xy) == 0
    assert len(hits.benign_null_xy) > 0


def test_the_two_codes_are_not_interchangeable():
    """The whole point of carrying codes: same NaN ranges, different verdict."""
    def cliff_count(code):
        r, c = clear_floor()
        r = r.copy(); c = c.copy()
        r[100:160] = np.nan
        c[100:160] = code
        return len(run_frames(source(), status({'sensor_0': sensor_block(r, c)}))
                   .probable_cliff_xy)

    assert cliff_count(protocol.CODE_BEYOND_LIMIT) > cliff_count(protocol.CODE_NO_RETURN)


# ---------------------------------------------------------------------------
# cliffs and drops publish at the near edge
# ---------------------------------------------------------------------------

def test_a_cliff_publishes_nearer_than_where_the_beam_landed():
    """A drop reads LONG: the beam flew over the ledge and hit floor further
    out. Publishing that far point puts the hazard behind the real edge, and
    the wheel is over it before the robot ever sees a reason to stop."""
    src = source()
    r, c = clear_floor()
    r = r.copy(); c = c.copy()
    drop_bins = slice(120, 150)
    r[drop_bins] = ideal_range() + 0.25          # floor is 25 cm further away
    hits = run_frames(src, status({'sensor_0': sensor_block(r, c)}))

    published = np.vstack([a for a in (hits.small_drop_xy, hits.deep_drop_xy)
                           if len(a)]) if (len(hits.small_drop_xy)
                                           or len(hits.deep_drop_xy)) else None
    assert published is not None, 'the drop should have been detected at all'

    measured_xy = src.projector.project(0, r)[drop_bins, :2]
    expected_xy = src.projector.floor_intersections(0)[drop_bins]

    pub_radius = np.linalg.norm(published, axis=1).max()
    measured_radius = np.linalg.norm(measured_xy, axis=1).min()
    expected_radius = np.linalg.norm(expected_xy, axis=1).max()

    assert pub_radius < measured_radius, 'published behind the edge — unsafe'
    assert pub_radius == pytest.approx(expected_radius, abs=1e-6)


def test_an_obstacle_still_publishes_where_the_beam_hit():
    """Only drops move to the near edge. An obstacle's measured point IS the
    object, and pulling it inward would invent clearance that is not there."""
    src = source()
    r, c = clear_floor()
    r = r.copy()
    r[120:150] = ideal_range() - 0.15            # something 15 cm up
    hits = run_frames(src, status({'sensor_0': sensor_block(r, c)}))
    assert len(hits.obstacle_xy) > 0
    measured_xy = src.projector.project(0, r)[120:150, :2]
    floor_xy = src.projector.floor_intersections(0)[120:150]
    d_measured = np.abs(np.linalg.norm(hits.obstacle_xy, axis=1).max()
                        - np.linalg.norm(measured_xy, axis=1).max())
    d_floor = np.abs(np.linalg.norm(hits.obstacle_xy, axis=1).max()
                     - np.linalg.norm(floor_xy, axis=1).max())
    assert d_measured < d_floor


# ---------------------------------------------------------------------------
# liveness: the stale-array trap
# ---------------------------------------------------------------------------

def test_a_dead_sensor_contributes_nothing_though_its_array_looks_perfect():
    """The trap. sensor_3's ranges are full, finite and hazardous, and the only
    thing saying otherwise is health."""
    r, c = clear_floor()
    r = r.copy()
    r[100:200] = ideal_range() - 0.20            # a wall, in stale data
    st = status({'sensor_3': sensor_block(r, c)}, dead=['sensor_3'])
    hits = run_frames(source(), st)
    assert len(hits.obstacle_xy) == 0
    assert 'sensor_3' not in hits.observed_sensors
    assert hits.skipped_sensors['sensor_3'] == 'dead'


def test_a_disabled_sensor_is_skipped_and_named_as_a_choice_not_a_fault():
    r, c = clear_floor()
    r = r.copy()
    r[100:200] = ideal_range() - 0.20
    st = status({'sensor_4': sensor_block(r, c)}, disabled=['sensor_4'])
    hits = run_frames(source(), st)
    assert len(hits.obstacle_xy) == 0
    assert hits.skipped_sensors['sensor_4'] == 'disabled'
    assert 'sensor_4' not in hits.observed_sensors


def test_a_closed_port_stops_every_sensor_being_read():
    """Six full, finite, recent-looking arrays and not one of them is live."""
    st = status(port_open=False)
    hits = run_frames(source(), st)
    assert hits.observed_sensors == ()
    assert set(hits.skipped_sensors.values()) == {'port_closed'}
    assert len(hits.obstacle_xy) == len(hits.probable_cliff_xy) == 0


def test_paused_streaming_is_not_mistaken_for_clear_floor():
    hits = run_frames(source(), status(streaming=False))
    assert hits.observed_sensors == ()
    assert set(hits.skipped_sensors.values()) == {'streaming_off'}


def test_observed_sensors_is_the_only_honest_coverage_signal():
    """Empty hazard arrays mean 'clear' AND 'not looking'. Only this field
    separates them."""
    hits = run_frames(source(), status(dead=['sensor_1'], disabled=['sensor_5']))
    assert set(hits.observed_sensors) == {'sensor_0', 'sensor_2', 'sensor_3', 'sensor_4'}
    assert len(hits.obstacle_xy) == 0        # nothing found...
    assert len(hits.skipped_sensors) == 2    # ...but two wedges were never checked


# ---------------------------------------------------------------------------
# calibration inputs
# ---------------------------------------------------------------------------

def test_a_chronically_blind_bin_does_not_manufacture_evidence():
    """bin_null_rate is what stops a dirty lens reporting a permanent cliff."""
    r, c = clear_floor()
    r = r.copy(); c = c.copy()
    r[100:160] = np.nan
    c[100:160] = protocol.CODE_NO_RETURN
    st = status({'sensor_0': sensor_block(r, c)})

    chronic = np.zeros(NBINS)
    chronic[100:160] = 0.95                  # these bins never return anyway
    trusting = run_frames(source(), st)
    knowing = run_frames(source(bin_null_rate={'sensor_0': chronic}), st)
    assert len(knowing.benign_null_xy) < len(trusting.benign_null_xy)


def test_missing_codes_are_treated_as_not_measurements():
    """CODE_VALID is 0, so a np.zeros fallback would declare every NaN bin a
    good reading. Absent codes must mean 'no measurements here'."""
    from stretch4_body.subsystem.line_sensor.filter import as_code_array
    codes = as_code_array(None, NBINS)
    assert not measuring(codes).any()
    assert not measuring(as_code_array(np.zeros(7, np.uint8), NBINS)).any()


def test_clear_floor_produces_no_hazards():
    """The baseline that keeps every other test honest."""
    hits = run_frames(source(), status(), n=6)
    assert len(hits.obstacle_xy) == 0
    assert len(hits.small_drop_xy) == 0
    assert len(hits.deep_drop_xy) == 0
    assert len(hits.probable_cliff_xy) == 0
    assert set(hits.observed_sensors) == set(NAMES)


# ---------------------------------------------------------------------------
# identity travels with the points
# ---------------------------------------------------------------------------

def test_every_published_point_says_which_sensor_and_bin_made_it():
    """Six identical 60-degree wedges tile the circle, so a rotated, mirrored
    or permuted bus map all produce a complete, plausible ring. Without
    identity on the point a mounting error is not falsifiable from the output
    -- and self-consistency is the only thing software can check."""
    src = source()
    r, c = clear_floor()
    r = r.copy()
    r[120:150] = ideal_range() - 0.15
    hits = run_frames(src, status({'sensor_2': sensor_block(r, c)}))

    assert len(hits.obstacle_xy) == len(hits.obstacle_id)
    assert (hits.obstacle_id[:, 0] == 2).all()               # sensor_2, as staged
    assert hits.obstacle_id[:, 1].min() >= 120
    assert hits.obstacle_id[:, 1].max() < 150


def test_cliff_points_carry_identity_too():
    r, c = clear_floor()
    r = r.copy(); c = c.copy()
    r[100:160] = np.nan
    c[100:160] = protocol.CODE_BEYOND_LIMIT
    hits = run_frames(source(), status({'sensor_1': sensor_block(r, c)}))
    assert len(hits.probable_cliff_xy) == len(hits.probable_cliff_id)
    assert (hits.probable_cliff_id[:, 0] == 1).all()
