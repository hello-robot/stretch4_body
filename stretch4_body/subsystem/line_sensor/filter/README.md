# Line-sensor hazard filtering

Turns raw line-sensor ranges into hazards the base can act on: **obstacles**,
**small drops**, **deep drops**, **probable cliffs**, and **degraded sectors**.

Pure numpy and geometry, no ROS. It lives in `stretch4_body` because it is
versioned with the contract it consumes — the per-bin `codes` from
`protocol.py` and the calibration the loop serves — and because every consumer
already depends on this package.

```python
from stretch4_body.subsystem.line_sensor.filter import (
    LineSensorSource, LineSensorConfig)

source = LineSensorSource(
    geometry, sensor_names, LineSensorConfig(),
    apply_tare=loop.apply_tare,
    bin_reliable=loop.bin_reliable(),
    bin_null_rate=loop.bin_null_rate())

hits = source.process(loop.status)      # the WHOLE status dict, health included
hits.obstacle_xy, hits.probable_cliff_xy, hits.observed_sensors
```

---

## How the data gets here

```
  6× PixArt J3                  one USB CDC serial port, newline-JSON, ~30 Hz each
        │
        ▼
  ┌──────────────────────────────────────────────────────────────────┐
  │ pixart_j3_reader.py            (line_sensor_loop child process)  │
  │                                                                  │
  │   raw mm ──► protocol.decode_distances_mm()                      │
  │                    │                                             │
  │                    ├──► ranges[320]  float, NaN at every         │
  │                    │                 non-measurement bin         │
  │                    └──► codes[320]   uint8, WHY it is NaN        │
  │                                                                  │
  │   Classified ONCE, here, before any arithmetic touches the       │
  │   value. This is the only place sentinels are interpreted.       │
  └──────────────────────────────────────────────────────────────────┘
        │  q_status  (CircularMultiprocessingQueue, drops OLDEST)
        ▼
  ┌──────────────────────────────────────────────────────────────────┐
  │ line_sensor_loop.py            (robot server, parent process)    │
  │   status = { sensor_0..5 : {ranges, codes, frame_id, ...},       │
  │              health      : {sensors_dead, disabled_sensors,      │
  │                             port_open, streaming, ...},          │
  │              calibration : packed tares, validated once }        │
  └──────────────────────────────────────────────────────────────────┘
        │  ZMQ  (CONFLATE=1, drops all but the newest)
        ▼
  ┌──────────────────────────────────────────────────────────────────┐
  │ RobotClient.line_sensor_loop   (any process, any machine)        │
  │   .status  .health()  .apply_tare()  .bin_reliable()             │
  │   .bin_null_rate()  .dead_sensors()  .disabled_sensors()         │
  └──────────────────────────────────────────────────────────────────┘
        │
        ▼
  ┌──────────────────────────────────────────────────────────────────┐
  │ filter/  ── THIS PACKAGE ──                          ~9 ms/frame │
  │   LineSensorSource.process(status)  ──►  LineSensorHits          │
  └──────────────────────────────────────────────────────────────────┘
        │
        ├──► line_sensor_publisher (ROS)  ──► PointCloud2 topics
        │                                     └─► stretch_base_hazard
        │                                         odom rolling grid,
        │                                         velocity gate
        └──► stretch_line_sensor_viz_3d, other tools
```

Both hops between the reader and you are **lossy latest-value channels**: the
queue discards the oldest message when full, and the ZMQ socket keeps only the
newest. That is why every message carries *complete* state rather than deltas,
and why `health` is republished on **every** loop cycle even when no frame
arrived — during a dropout that is the only part still telling the truth.

---

## Two rules you cannot see in the data

**1. A dead sensor's `ranges` is stale, not empty.**

When a sensor stops reporting, the body keeps its last good scan in
`status[name]['ranges']` on purpose — liveness lives in `status['health']`.
A full, finite, plausible 320-point array will keep arriving forever. Nothing
in it says the cable fell out an hour ago.

So `process()` takes the **whole status dict**, not the per-sensor blocks, and
consults `health` first. `hits.observed_sensors` reports who was actually read
and `hits.skipped_sensors` says why each of the others was not.

**2. Empty hazard arrays mean both "clear" and "not looking".**

`len(hits.obstacle_xy) == 0` is not safety information on its own. Coverage
comes from `observed_sensors`; compute swept area from that, never from the
absence of points.

---

## The pipeline

```
   status ──► liveness gate ──► tare ──► ┌─ Stage 1  classify   ─┐
                                         │  Stage 2  gloss       │  returning bins
                                         │  Stage 3  shape       │
                                         │  Stage 4  confirm    ─┘
                                         │
                                         └─ Stage 5  nulls  ◄── the silences
                                                    │
                                                    ▼
                                            Stage 6  package
```

| File | What lives there | Front door |
|---|---|---|
| `source.py` | orchestrator; liveness gate; per-frame state | `LineSensorSource.process()` |
| `config.py` | every tunable, grouped by stage | `LineSensorConfig` |
| `hits.py` | the vocabulary and the output object | `BinClass`, `LineSensorHits` |
| `geometry.py` | range → point math, shared by every stage | `Projector` |
| `arrays.py` | numpy helpers; the one validity test | `measuring()`, `runs()` |
| `classify.py` | label one returning bin | `classify_bin()` |
| `gloss.py` | reject glossy-floor phantoms | `FlipTracker` |
| `shape.py` | reject spray / streaks / noise by shape | `ShapeGate` |
| `confirm.py` | require a hazard to persist across frames | `bin_confirmed()` |
| `nulls.py` | read the *silences* → cliffs & degraded | `NullEvidenceDetector` |

### The one idea underneath everything: range → z

The sensor measures **range**, not height. Each bin is compared to its own
clear-floor reference (from the tare); the difference, as a height, is `z`.

```
z ≈ (floor_reference_range − measured_range) × sin(down_pitch)

   beam reaches the floor where expected  →  z ≈ 0    free
   something blocks it early              →  z > 0    obstacle height
   floor is lower than expected           →  z < 0    a drop
```

---

## The filters, and what each one rejects

**Stage 1 — `classify.py`.** Pure function: `z` and local contrast in, a
`BinClass` out. Bins with a trustworthy tare (`bin_reliable`) get the sensitive
deviation bands; untared bins fall back to coarse absolute-height bands.
Obstacle-family and drop-family bins become candidates; everything else is
dropped.

**Stage 2 — `gloss.py`. Rejects: shiny-floor phantoms.** A glossy floor throws
back compact near-field arcs that are shape-identical to a real object pressed
against the base, so shape cannot help. Two context signatures do:

- *time* — `FlipTracker` keeps a decayed per-bin count of hazard↔quiet flips.
  Gloss flickers constantly; a real object flips once per approach.
- *space* — if several sensors see near-field hazards at once, no single object
  could cause it, so all near-field candidates become SPRAY. Long, smooth runs
  are exempt: that is what a real object pressed against the base looks like.

**Stage 3 — `shape.py`. Rejects: spray, streaks, point noise.** Groups
survivors into contiguous per-sensor runs and tests each. A thin monotonic
streak radiating from the sensor is spray; a tiny isolated blob is point noise;
an over-long or over-spread run is rejected. Runs separated by a small gap are
merged and re-tested so a broken-up streak cannot slip through in pieces.

**Stage 4 — `confirm.py`. Rejects: flicker.** A hazard must appear on N
consecutive frames at the same bin index. Strong obstacles clear in one frame;
marginal ones and anything seen during active gloss wait longer.

> This is **de-flicker only**. On a moving base the same bin index looks at
> different ground each frame, so it is *not* world persistence and must not be
> relied on as such. Accumulating evidence over actual ground is a separate
> downstream job — the odom-frame rolling grid in the hazard layer. The two are
> complementary; neither replaces the other.

**Stage 5 — `nulls.py`. Reads what the others throw away.** Everything above
works on bins that *return*. This works on the bins that do not, and it is
where the two status codes earn their keep:

| code | meaning | reading |
|---|---|---|
| `CODE_BEYOND_LIMIT` (5.09) | a return from **past** the range limit | the sensors point down, so the beam went beyond where the floor should be — **the strongest cliff evidence the hardware can give** |
| `CODE_NO_RETURN` (5.11) | nothing came back at all | genuinely ambiguous: dark floor, sunlight, a glossy surface angled away, or a void |

A null run must clear three gates before it is evidence at all: the bins must
be ones that *do* return on clear floor (`bin_null_rate`, so a dirty lens reads
as degraded rather than manufacturing a cliff), the run must be long enough,
and it must have been mostly null last frame too.

Surviving runs are then typed by context:

- enough 5.09 bins → **probable cliff** (a proven void)
- bearings overlapping another sensor's void → **probable cliff** (a ledge is
  continuous across the floor; it does not stop at a sensor boundary)
- adjacent to an obstacle → benign (occlusion shadow)
- a bright near return on the same sensor → benign (exposure suppression)
- adjacent to, or bearing-aligned with, a returning drop → **probable cliff**
- anything left → dark floor: benign, but *unexplained*

Unexplained nulls measure lost floor coverage. Once their smoothed, hysteretic
fraction crosses a threshold, that sensor publishes as **degraded** — a reason
to slow down, not to stop.

---

## What a frame produces

| Field | Meaning | Severity |
|---|---|---|
| `obstacle_xy` | confirmed obstacles | stop |
| `small_drop_xy` | confirmed 2–10 cm drops | soft hazard |
| `deep_drop_xy` | confirmed returning drops deeper than `cliff_max_drop_m` | stop (lethal) |
| `probable_cliff_xy` | cliff-typed null runs | stop (lethal) |
| `degraded_xy` | sectors that lost floor coverage | slow down |
| `observed_sensors` | who was actually read this frame | **coverage** |
| `skipped_sensors` | `{name: why}` — dead / disabled / port_closed / streaming_off | — |

`raw_*`, `spatial_*` and `benign_null_xy` are debug views of the intermediate
stages.

### Drops and cliffs are published at the near edge

**Never steer to a reported drop distance.** A bin that reads long means the
beam flew over a ledge and hit floor further away — the measured point lies
*beyond* the edge, and a robot that treats it as the hazard location has
already put a wheel over by the time it reacts.

Every drop-family and cliff output is therefore reported at the bin's
**expected floor intersection**: the nearest place the hazard can be, because
if the floor were intact there the beam would have returned at the expected
range, and it did not. Obstacles are unaffected — the measured point *is* the
object, and moving it inward would invent clearance that does not exist.

---

## Why this was rewritten rather than ported

The previous filter found no-return bins by magnitude:

```python
np.isfinite(ranges) & (ranges > 4.0)          # far sentinel
np.isclose(ranges, 5.11, atol=0.005)          # no-return
```

`ranges` is NaN at every non-measurement bin now, so `isfinite` is False for
all of them and **both masks matched nothing** — every cliff silently
discarded, at 30 Hz, with all six sensors reporting and system check passing.

The magnitude test was also applied *after* the tare, so a tared 5.09 could
land within tolerance of 5.11 and destroy the cliff/dark-floor distinction
outright.

Codes are assigned once at decode, before any arithmetic touches the value, and
compared as integers. Three fragile tunables (`null_range_m`,
`null_tolerance_m`, `null_sentinel_min_m`) and the 4.0 m cutoff are gone with
them. There is exactly one validity test in the package, `arrays.measuring()`,
and it is `codes == CODE_VALID`.
