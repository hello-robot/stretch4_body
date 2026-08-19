#!/usr/bin/env python3
"""On-disk layout for line-sensor calibration: sessions, tares, and reports.

Path-based only. 

    <base_dir>/
        tare/
            sensor_0_tare.yaml        <- THE current tare. A lookup, not a scan.
            ...
        sessions/
            20260804171205/
                session.npz           <- one file, every recorded sensor
                session_meta.yaml     <- greppable mirror of the npz metadata
                calibration_report.yaml
                sensor_0_tare.yaml    <- exact bytes of what was promoted

Two properties of this layout are load-bearing.

The current tare is a fixed path, not the lexicographically-newest session
directory that happens to contain a tare file. Under the old scheme,
recomputing into an older session left the result shadowed forever, and a
sensor whose newest run was refused silently kept using a much older tare
while the directory listing suggested otherwise.

Loading REFUSES rather than warns, and never falls back to an older file. The
previous implementation caught exceptions inside its scan loop, so a shape
mismatch printed one line and then quietly loaded the previous session's
calibration.
"""

from __future__ import annotations

from dataclasses import asdict
import datetime
import os

import numpy as np
import yaml

from stretch4_body.subsystem.line_sensor.calibration import IDEAL_RANGE_MODEL

TARE_FORMAT_VERSION = 2

# An offsets-vs-ideal disagreement past this is a changed parameter, not the
# rounding of a full-precision float.
IDEAL_RANGE_TOL_M = 1e-3


REJECT_MISSING = 'missing'
REJECT_UNREADABLE = 'unreadable'
REJECT_LEGACY = 'legacy_format'
REJECT_BAD_SCHEMA = 'bad_schema'
REJECT_BIN_ORDER = 'bin_order_changed'
REJECT_GEOMETRY = 'geometry_changed'
REJECT_BIN_COUNT = 'bin_count_mismatch'
REJECT_NONFINITE = 'nonfinite_offsets'
REJECT_NO_VALID_BINS = 'no_valid_bins'

_REMEDIATE = ('Re-run  REx_line_sensor_calibrate --all  on a clean, flat, '
              'light-coloured floor.')


class TareRejected(Exception):
    """A stored tare exists but must not be used. Never downgrade this to a
    warning: running uncalibrated is recoverable, running on a tare that
    belongs to a different robot configuration is not."""

    def __init__(self, reason, detail=''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason
        self.detail = detail


class LoadedTare:
    def __init__(self, **kw):
        self.__dict__.update(kw)

    @property
    def age_days(self):
        try:
            t = datetime.datetime.fromisoformat(self.timestamp)
        except (TypeError, ValueError):
            return float('nan')
        return (datetime.datetime.now() - t).total_seconds() / 86400.0

    def __repr__(self):
        return (f'LoadedTare({self.sensor_name}, {int(self.valid_mask.sum())}/'
                f'{self.valid_mask.size} bins, {self.age_days:.1f} days old)')


# ---------------------------------------------------------------------------
# paths
# ---------------------------------------------------------------------------

def tare_dir(base_dir):
    return os.path.join(base_dir, 'tare')


def tare_path(base_dir, sensor_name):
    return os.path.join(tare_dir(base_dir), f'{sensor_name}_tare.yaml')


def sessions_dir(base_dir):
    return os.path.join(base_dir, 'sessions')


def session_dir(base_dir, session_id):
    return os.path.join(sessions_dir(base_dir), session_id)


def new_session_id(now=None):
    return (now or datetime.datetime.now()).strftime('%Y%m%d%H%M%S')


def list_sessions(base_dir):
    d = sessions_dir(base_dir)
    return sorted(os.listdir(d)) if os.path.isdir(d) else []


# ---------------------------------------------------------------------------
# sessions
# ---------------------------------------------------------------------------

def write_session(session, base_dir):
    """Write one .npz plus a greppable yaml mirror. Returns the session dir."""
    out = session_dir(base_dir, session.session_id)
    os.makedirs(out, exist_ok=True)

    meta = {
        'tare_format_version': TARE_FORMAT_VERSION,
        'session_id': session.session_id,
        'started_at': session.started_at,
        'ended_at': session.ended_at,
        'requested_frames': int(session.requested_frames),
        'poll_iterations': int(session.poll_iterations),
        'stretch_body_version': session.stretch_body_version,
        'loop_params_snapshot': session.loop_params_snapshot,
        'sensors': {
            name: {'sensor_index': int(r.sensor_index), 'status': r.status,
                   'notes': list(r.notes), 'stats': dict(r.stats)}
            for name, r in session.recordings.items()
        },
    }

    arrays = {'sensor_names': np.array(session.sensor_names, dtype='<U16')}
    for name, rec in session.recordings.items():
        # A failed sensor still gets its arrays written, possibly zero-length:
        # the evidence for why a run failed is exactly what you want on disk.
        arrays[f'{name}__ranges'] = np.asarray(rec.ranges, dtype=np.float64)
        arrays[f'{name}__codes'] = np.asarray(rec.codes, dtype=np.uint8)
        arrays[f'{name}__frame_id'] = np.asarray(rec.frame_id, dtype=np.int64)
        arrays[f'{name}__ts'] = np.asarray(rec.ts, dtype=np.float64)
        arrays[f'{name}__missed_frames'] = np.asarray(rec.missed_frames, dtype=np.int32)

    np.savez_compressed(os.path.join(out, 'session.npz'), **arrays)
    with open(os.path.join(out, 'session_meta.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return out


def write_report(report, path):
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(report, f, sort_keys=False)


# ---------------------------------------------------------------------------
# tares
# ---------------------------------------------------------------------------

def build_tare_yaml(sensor_name, sensor_index, result, ideal_range_m,
                    thresholds, flip_range_ordering, session_id='',
                    stretch_body_version='', timestamp=None, n_frames=0):
    """The single place a tare document is constructed. Every writer goes
    through here so the schema cannot drift between them."""
    ideal = np.asarray(ideal_range_m, dtype=np.float64).reshape(-1)
    per_bin_ideal = (None if ideal.size == 1 or np.allclose(ideal, ideal[0])
                     else [float(v) for v in ideal])
    return {
        'format_version': TARE_FORMAT_VERSION,
        'sensor_name': str(sensor_name),
        'sensor_index': int(sensor_index),
        'timestamp': timestamp or datetime.datetime.now().isoformat(),
        'session_id': str(session_id),
        'stretch_body_version': str(stretch_body_version),
        'flip_range_ordering': bool(flip_range_ordering),
        'n_bins': int(result.offsets.size),
        'n_frames': int(n_frames),
        'ideal_range_m': float(ideal[0]),
        'ideal_range_per_bin': per_bin_ideal,
        'ideal_range_model': IDEAL_RANGE_MODEL,
        'tare_offsets': [float(v) for v in result.offsets],
        'tare_valid_mask': [bool(v) for v in result.valid_mask],
        'null_rate_per_bin': [round(float(v), 4) for v in result.null_rate_per_bin],
        'calibration_thresholds': asdict(thresholds),
        'calibration_summary': result.summary,
    }


def write_tare(tare, session_out, base_dir):
    """Write the tare into the session (history), then atomically promote it
    to the canonical path. Returns the canonical path."""
    name = tare['sensor_name']
    blob = yaml.safe_dump(tare, sort_keys=False)
    with open(os.path.join(session_out, f'{name}_tare.yaml'), 'w',
              encoding='utf-8') as f:
        f.write(blob)

    os.makedirs(tare_dir(base_dir), exist_ok=True)
    final = tare_path(base_dir, name)
    tmp = final + '.tmp'                       # same directory, so replace is atomic
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(blob)
    os.replace(tmp, final)
    return final


def read_tare(path):
    """Raw parse, no validation. Use load_validated_tare at runtime."""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _attempt_auto_migration(path):
    """Helper to auto-migrate a line sensor tare from legacy formats if present."""
    base_dir = os.path.dirname(os.path.dirname(path))
    sensor_name = os.path.basename(path).replace('_tare.yaml', '')

    try:
        # Deferred imports to avoid circular dependencies
        from stretch4_body.core.robot_params import RobotParams
        from stretch4_body.tools.factory import REx_line_sensor_migrate_tares

        user_params, robot_params = RobotParams.get_params()
        ls_params = robot_params.get('line_sensor_loop') or {}
        if not ls_params:
            if 'line_sensor_geometry' in robot_params:
                ls_params = robot_params
            else:
                return

        # Check if a source exists for this sensor before trying to migrate
        src_path, kind = REx_line_sensor_migrate_tares.find_source(base_dir, sensor_name)
        if src_path is None:
            return

        print(f"Auto-migrating legacy line sensor calibration for {sensor_name}...")
        success, detail, advice_or_reason = REx_line_sensor_migrate_tares.migrate_single_sensor(
            base_dir, sensor_name, ls_params, user_params
        )
        if success:
            print(f"Successfully auto-migrated {sensor_name}: {detail}")
        else:
            print(f"Failed to auto-migrate {sensor_name}: {detail} - {advice_or_reason}")
    except Exception as exc:
        print(f"Warning: error during auto-migration for {sensor_name}: {exc}")


def load_validated_tare(path, expected_n_bins, expected_flip, expected_ideal_m, allow_migrate=True):
    """Load a tare, or raise TareRejected. Never returns partially-valid data
    and never falls back to another file.
    """
    if allow_migrate:
        should_migrate = False
        if not os.path.exists(path):
            should_migrate = True
        else:
            try:
                data = read_tare(path)
                if isinstance(data, dict):
                    version = data.get('format_version')
                    if version is None or int(version) < TARE_FORMAT_VERSION:
                        should_migrate = True
            except Exception:
                should_migrate = True

        if should_migrate:
            _attempt_auto_migration(path)

    if not os.path.exists(path):
        raise TareRejected(REJECT_MISSING,
                           f'no tare at {path}. {_REMEDIATE}')
    try:
        data = read_tare(path)
    except Exception as exc:
        raise TareRejected(REJECT_UNREADABLE, f'{path}: {exc}')
    if not isinstance(data, dict):
        raise TareRejected(REJECT_UNREADABLE, f'{path}: not a mapping')

    version = data.get('format_version')
    if version is None or int(version) < TARE_FORMAT_VERSION:
        raise TareRejected(
            REJECT_LEGACY,
            f'{path} is format v{version}, older than v{TARE_FORMAT_VERSION}. '
            f'Adopt it with  REx_line_sensor_migrate_tares  or {_REMEDIATE.lower()}')
    if int(version) > TARE_FORMAT_VERSION:
        raise TareRejected(REJECT_BAD_SCHEMA,
                           f'{path}: format_version {version} is newer than this '
                           f'code understands ({TARE_FORMAT_VERSION})')

    if bool(data.get('flip_range_ordering')) != bool(expected_flip):
        raise TareRejected(
            REJECT_BIN_ORDER,
            f'{path} was recorded with flip_range_ordering='
            f'{bool(data.get("flip_range_ordering"))}, but the robot now runs '
            f'{bool(expected_flip)} -- every offset would be mirrored. {_REMEDIATE}')

    stored_ideal = data.get('ideal_range_m')
    if stored_ideal is None:
        raise TareRejected(REJECT_GEOMETRY,
                           f'{path} records no ideal_range_m. {_REMEDIATE}')
    if abs(float(stored_ideal) - float(expected_ideal_m)) > IDEAL_RANGE_TOL_M:
        raise TareRejected(
            REJECT_GEOMETRY,
            f'{path} was recorded against an ideal range of '
            f'{float(stored_ideal):.4f} m but the robot now computes '
            f'{float(expected_ideal_m):.4f} m -- the emitter height or mounting '
            f'angle has changed. {_REMEDIATE}')

    offsets = np.asarray(data.get('tare_offsets', []), dtype=np.float64)
    mask = np.asarray(data.get('tare_valid_mask', []), dtype=bool)
    null_rate = np.asarray(data.get('null_rate_per_bin', []), dtype=np.float64)
    if offsets.size != expected_n_bins or mask.size != expected_n_bins:
        raise TareRejected(
            REJECT_BIN_COUNT,
            f'{path}: {offsets.size} offsets / {mask.size} mask entries, '
            f'expected {expected_n_bins}')
    if null_rate.size != expected_n_bins:
        null_rate = np.zeros(expected_n_bins, dtype=np.float64)
    if not np.isfinite(offsets).all():
        raise TareRejected(REJECT_NONFINITE,
                           f'{path}: {int(np.count_nonzero(~np.isfinite(offsets)))} '
                           f'non-finite offsets')
    if not mask.any():
        raise TareRejected(REJECT_NO_VALID_BINS,
                           f'{path}: no bin has a trustworthy tare. {_REMEDIATE}')

    per_bin = data.get('ideal_range_per_bin')
    ideal = (np.asarray(per_bin, dtype=np.float64) if per_bin
             else np.full(expected_n_bins, float(data.get('ideal_range_m', 0.0))))

    return LoadedTare(
        sensor_name=data.get('sensor_name', ''),
        sensor_index=int(data.get('sensor_index', -1)),
        offsets=offsets, valid_mask=mask, null_rate_per_bin=null_rate,
        ideal_range=ideal,
        timestamp=data.get('timestamp', ''),
        session_id=data.get('session_id', ''),
        path=path,
        summary=data.get('calibration_summary', {}) or {})
