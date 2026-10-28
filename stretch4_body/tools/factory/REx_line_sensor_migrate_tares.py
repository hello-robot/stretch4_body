#!/usr/bin/env python3
"""Bring older line sensor tares up to the current format. 

Poor quality calibrations are not refused but recalibration is advised.
The weak bins carry no offset and run uncalibrated either way, so the tare is
better than none.

Refused only when there is nothing usable to adopt: unreadable or wrong-sized
files, too few frames, no valid bin, an ideal_range_m that disagrees with
h/sin(angle) or flip_range_ordering overridden.
Those sensors need REx_line_sensor_calibrate --all.
"""


import argparse
import datetime
import glob
import os
import re
import sys

import numpy as np
import yaml

import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.line_sensor import calibration, calibration_store, protocol
from stretch4_body.subsystem.line_sensor.calibration import (
    CalibrationThresholds,
    SensorCalibrationResult,
)

LEGACY_TARE_NAME = 'calibration_tare.yaml'
LEGACY_RANGES_GLOB = '*_j3_ranges.npy'
# only <timestamp> directories are sessions
SESSION_DIR = re.compile(r'^\d{14}$')

_M_NO_RETURN = protocol.MM_NO_DETECTION / 1000.0
_M_BEYOND_LIMIT = protocol.MM_BEYOND_LIMIT / 1000.0


class Refused(Exception):
    """Reason why one sensor cannot be migrated."""

    def __init__(self, reason, detail):
        super().__init__(f'{reason}: {detail}')
        self.reason = reason
        self.detail = detail


def flip_is_overridden(user_params):
    """Check if the robot's own params file override the bin order"""
    return 'flip_range_ordering' in ((user_params or {}).get('line_sensor_loop') or {})


def find_source(base_dir, sensor_name):
    """Best old source for this sensor, or (None, None). """
    sensor_dir = os.path.join(base_dir, sensor_name)
    if not os.path.isdir(sensor_dir):
        return None, None
    tare = None
    for session in sorted((d for d in os.listdir(sensor_dir) if SESSION_DIR.match(d)),
                          reverse=True):
        out = os.path.join(sensor_dir, session)
        if glob.glob(os.path.join(out, LEGACY_RANGES_GLOB)):
            return out, 'recording'
        if tare is None and os.path.isfile(os.path.join(out, LEGACY_TARE_NAME)):
            tare = os.path.join(out, LEGACY_TARE_NAME)
    return (tare, 'legacy') if tare else (None, None)


def recompute_from_recording(session_dir, ls_params, n_bins, thresholds):
    """Run the current tare maths over an old recording."""
    files = sorted(glob.glob(os.path.join(session_dir, LEGACY_RANGES_GLOB)))
    frames = []
    for f in files:
        a = np.asarray(np.load(f), dtype=np.float64).reshape(-1)
        if a.size == n_bins:
            frames.append(a)
    if not frames:
        raise Refused('unreadable', f'{len(files)} frame file(s), none with '
                                    f'{n_bins} bins')
    ranges = np.stack(frames)

    codes = np.full(ranges.shape, protocol.CODE_VALID, dtype=np.uint8)
    codes[np.isclose(ranges, _M_NO_RETURN)] = protocol.CODE_NO_RETURN
    codes[np.isclose(ranges, _M_BEYOND_LIMIT)] = protocol.CODE_BEYOND_LIMIT
    codes[~np.isfinite(ranges) | (ranges <= 0.0)] = protocol.CODE_OTHER_INVALID
    # compute_sensor_tare wants NaN at every non-measurement bin
    samples = np.where(codes == protocol.CODE_VALID, ranges, np.nan)

    ideal = calibration.ideal_range_m(ls_params)
    try:
        result = calibration.compute_sensor_tare(samples, codes, ideal, thresholds)
    except ValueError as exc:
        raise Refused('unusable_recording', str(exc))
    return result, ideal, len(frames)


def assess(path, ls_params, n_bins, thresholds, user_params):
    """Read one older tare and decide whether it can be adopted.

    Returns (doc, offsets, mask, null_rate, flip). Raises Refused.
    """
    try:
        doc = calibration_store.read_tare(path)
    except Exception as exc:
        raise Refused('unreadable', f'{path}: {exc}')
    if not isinstance(doc, dict):
        raise Refused('unreadable', f'{path}: not a mapping')

    offsets = np.asarray(doc.get('tare_offsets', []), dtype=np.float64)
    mask = np.asarray(doc.get('tare_valid_mask', []), dtype=bool)
    if offsets.size != n_bins or mask.size != n_bins:
        raise Refused(
            'bin_count',
            f'{offsets.size} offsets / {mask.size} mask entries, expected {n_bins}')
    if not np.isfinite(offsets).all():
        raise Refused(
            'nonfinite',
            f'{int(np.count_nonzero(~np.isfinite(offsets)))} non-finite offsets')
    if not mask.any():
        raise Refused('no_valid_bins', 'no bin has a trustworthy tare')

    worst = float(np.max(np.abs(offsets)))
    if worst >= thresholds.max_abs_tare_m:
        n_bad = int(np.count_nonzero(np.abs(offsets) >= thresholds.max_abs_tare_m))
        raise Refused(
            'contaminated',
            f'{n_bad} bin(s) with |offset| up to {worst:.2f} m (limit '
            f'{thresholds.max_abs_tare_m:.2f} m) -- the recording included '
            f'no-return sentinels, which the legacy tare maths averaged in')

    stored_ideal = doc.get('ideal_range_m')
    if stored_ideal is None:
        raise Refused('geometry', 'no ideal_range_m recorded, so the geometry '
                                  'it assumed cannot be checked')
    current_ideal = calibration.ideal_range_m(ls_params)
    if abs(float(stored_ideal) - current_ideal) > calibration_store.IDEAL_RANGE_TOL_M:
        raise Refused(
            'geometry',
            f'recorded against an ideal range of {float(stored_ideal):.4f} m, '
            f'but the running params give {current_ideal:.4f} m -- the emitter '
            f'height or mounting angle has changed')

    if flip_is_overridden(user_params):
        raise Refused(
            'unproven_order',
            'flip_range_ordering is overridden in stretch_user_params.yaml, and '
            'a legacy tare records no bin order, so the running value cannot '
            'stand in for what the recording used')
    flip = bool(ls_params['flip_range_ordering'])

    null_rate = np.asarray(doc.get('null_rate_per_bin', []), dtype=np.float64)
    if null_rate.size != n_bins:
        # not measured by that format; zeros match what an untared sensor gets
        null_rate = np.zeros(n_bins, dtype=np.float64)

    return doc, offsets, mask, null_rate, flip


def build_migrated_tare(sensor_name, sensor_index, doc, offsets, mask, null_rate,
                        flip, path, n_bins, thresholds):
    """Assemble the tare document """
    old_summary = dict(doc.get('calibration_summary') or {})
    n_frames = int(old_summary.get('frames_used',
                                   old_summary.get('frames_recorded', 0)) or 0)

    result = SensorCalibrationResult(
        offsets=offsets, valid_mask=mask, null_rate_per_bin=null_rate,
        summary={
            'source': 'migrated from a pre-v2 stored tare',
            'frames_used': n_frames,
            'bins_per_frame': int(n_bins),
            'n_valid_bins': int(mask.sum()),
            'max_abs_offset_m': round(float(np.max(np.abs(offsets))), 6),
            'median_offset_m': round(float(np.median(offsets)), 6),
            'null_rate_per_bin': ('carried over' if null_rate.any() else
                                  'not measured by that format; zeros'),
        },
        per_bin={}, indices={}, sufficient=True)

    tare = calibration_store.build_tare_yaml(
        sensor_name=sensor_name,
        sensor_index=sensor_index,
        result=result,
        ideal_range_m=float(doc['ideal_range_m']),
        thresholds=thresholds,
        flip_range_ordering=flip,
        session_id=str(doc.get('session_id')
                       or os.path.basename(os.path.dirname(path))),
        stretch_body_version=str(doc.get('stretch_body_version', '')),
        # original recording time, so age_days keeps meaning what it says
        timestamp=doc.get('timestamp') or '',
        n_frames=n_frames)
    tare['migrated_from'] = path
    tare['migrated_at'] = datetime.datetime.now().isoformat()
    return tare


def _timestamp_of(session_id):
    """Recording time, from the session directory name."""
    try:
        return datetime.datetime.strptime(session_id, '%Y%m%d%H%M%S').isoformat()
    except ValueError:
        return ''


def migrate_recording(name, index, session_dir, ls_params, n_bins, thresholds,
                      user_params):
    """Tare from an old raw recording. Returns (tare, detail, advice).

    """
    if flip_is_overridden(user_params):
        raise Refused(
            'unproven_order',
            'flip_range_ordering is overridden in stretch_user_params.yaml, and '
            'an old recording carries no bin order, so the running value cannot '
            'stand in for the one it was captured under')

    result, ideal, n_frames = recompute_from_recording(
        session_dir, ls_params, n_bins, thresholds)
    session_id = os.path.basename(session_dir.rstrip(os.sep))
    result.summary = dict(result.summary,
                          source=f'recomputed from {n_frames} raw frames of the '
                                 f'pre-fingerprint recorder')

    tare = calibration_store.build_tare_yaml(
        sensor_name=name, sensor_index=index, result=result,
        ideal_range_m=ideal, thresholds=thresholds,
        flip_range_ordering=bool(ls_params['flip_range_ordering']),
        session_id=session_id,
        # original recording time, so age_days keeps meaning what it says
        timestamp=_timestamp_of(session_id), n_frames=n_frames)
    tare['migrated_from'] = session_dir
    tare['migrated_at'] = datetime.datetime.now().isoformat()

    offsets = np.asarray(result.offsets, dtype=np.float64)
    detail = (f'recording, {n_frames} frames from {_timestamp_of(session_id)[:19]}, '
              f'{int(result.valid_mask.sum())}/{n_bins} bins, '
              f'max |offset| {np.max(np.abs(offsets)):.3f} m')
    advice = [] if result.sufficient else list(result.insufficiency_reasons) or [
        'the recording does not meet the acceptance thresholds']
    return tare, detail, advice


def _print_advice(advice):
    for line in advice:
        print(f'      recalibration advised: {line}')


def promote(tare, base_dir, sensor_name):
    """Write the canonical tare atomically."""
    os.makedirs(calibration_store.tare_dir(base_dir), exist_ok=True)
    final = calibration_store.tare_path(base_dir, sensor_name)
    tmp = final + '.tmp'                    # same directory, so replace is atomic
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write(yaml.safe_dump(tare, sort_keys=False))
    os.replace(tmp, final)
    return final


def loads_now(base_dir, sensor_name, n_bins, flip, ideal):
    """Does the runtime already accept the tare at the canonical path?"""
    try:
        calibration_store.load_validated_tare(
            calibration_store.tare_path(base_dir, sensor_name), n_bins, flip, ideal, allow_migrate=False)
        return True
    except Exception:
        return False


def migrate_single_sensor(base_dir, name, ls_params, user_params, force=False):
    """Migrate a single line sensor's calibration tare if an old source is found."""
    n_bins = int(ls_params['line_sensor_geometry']['pixart_report_num'])
    flip = bool(ls_params['flip_range_ordering'])
    ideal = calibration.ideal_range_m(ls_params)
    thresholds = CalibrationThresholds()

    if not force and loads_now(base_dir, name, n_bins, flip, ideal):
        return True, "already current", []

    path, kind = find_source(base_dir, name)
    if path is None:
        return False, "nothing to migrate", "no older tare or recording found"

    index = int(name.rsplit('_', 1)[1])
    try:
        advice = []
        if kind == 'recording':
            tare, detail, advice = migrate_recording(
                name, index, path, ls_params, n_bins, thresholds, user_params)
        else:
            doc, offsets, mask, null_rate, src_flip = assess(
                path, ls_params, n_bins, thresholds, user_params)
            tare = build_migrated_tare(name, index, doc, offsets, mask,
                                       null_rate, src_flip, path,
                                       n_bins, thresholds)
            recorded = str(doc.get('timestamp', ''))[:19] or 'unknown date'
            detail = (f'stored tare, recorded {recorded}, '
                      f'{int(mask.sum())}/{n_bins} bins, '
                      f'max |offset| {np.max(np.abs(offsets)):.3f} m')
        
        promote(tare, base_dir, name)
        if not loads_now(base_dir, name, n_bins, flip, ideal):
            return False, "validation failed", "runtime still refuses it after migration"
        
        return True, detail, advice
    except Refused as exc:
        return False, f"refused ({exc.reason})", exc.detail


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('-s', '--sensor-name', '--sensor_name', action='append',
                    dest='sensors', metavar='NAME',
                    help='limit to this sensor (repeatable); default all')
    ap.add_argument('--dry-run', action='store_true',
                    help='report what would happen and write nothing')
    ap.add_argument('--force', action='store_true',
                    help='replace a tare the runtime already accepts')
    ap.add_argument('--base-dir', default=None,
                    help='calibration directory (default: the fleet one)')
    args = ap.parse_args()

    from stretch4_body.core.robot_params import RobotParams
    user_params, robot_params = RobotParams.get_params()
    ls_params = robot_params.get('line_sensor_loop') or {}
    if not ls_params:
        sys.exit('FAIL: no line_sensor_loop params on this robot')

    sensor_names = list(ls_params['sensor_names'])
    n_bins = int(ls_params['line_sensor_geometry']['pixart_report_num'])
    flip = bool(ls_params['flip_range_ordering'])
    ideal = calibration.ideal_range_m(ls_params)
    thresholds = CalibrationThresholds()
    base_dir = args.base_dir or os.path.join(hu.get_fleet_directory(),
                                             'calibration_line_sensors')

    targets = args.sensors or sensor_names
    unknown = [s for s in targets if s not in sensor_names]
    if unknown:
        sys.exit(f'FAIL: unknown sensor(s): {", ".join(unknown)}')

    print(f'calibration directory: {base_dir}')
    print(f'target format: v{calibration_store.TARE_FORMAT_VERSION}, '
          f'{n_bins} bins, flip_range_ordering={flip}, ideal {ideal:.4f} m')

    migrated, skipped, refused, advised = [], [], [], []
    for name in targets:
        index = int(name.rsplit('_', 1)[1])

        if not args.force and loads_now(base_dir, name, n_bins, flip, ideal):
            print(f'  {name}: already current, left alone')
            skipped.append(name)
            continue

        path, kind = find_source(base_dir, name)
        if path is None:
            print(f'  {name}: nothing to migrate (no older tare found)')
            refused.append(name)
            continue

        try:
            advice = []
            if kind == 'recording':
                tare, detail, advice = migrate_recording(
                    name, index, path, ls_params, n_bins, thresholds, user_params)
            else:
                doc, offsets, mask, null_rate, src_flip = assess(
                    path, ls_params, n_bins, thresholds, user_params)
                tare = build_migrated_tare(name, index, doc, offsets, mask,
                                           null_rate, src_flip, path,
                                           n_bins, thresholds)
                recorded = str(doc.get('timestamp', ''))[:19] or 'unknown date'
                detail = (f'stored tare, recorded {recorded}, '
                          f'{int(mask.sum())}/{n_bins} bins, '
                          f'max |offset| {np.max(np.abs(offsets)):.3f} m')
        except Refused as exc:
            print(f'  {name}: REFUSED ({exc.reason}) -- {exc.detail}')
            refused.append(name)
            continue
        if args.dry_run:
            print(f'  {name}: would migrate  ({detail})')
            _print_advice(advice)
        else:
            final = promote(tare, base_dir, name)
            if not loads_now(base_dir, name, n_bins, flip, ideal):
                print(f'  {name}: FAILED -- wrote {final} but the runtime still '
                      f'refuses it')
                refused.append(name)
                continue
            print(f'  {name}: migrated  ({detail})')
            _print_advice(advice)
        if advice:
            advised.append(name)
        migrated.append(name)

    print()
    verb = 'would be migrated' if args.dry_run else 'migrated'
    print(f'{len(migrated)}/{len(targets)} {verb}'
          + (f', {len(skipped)} already current' if skipped else '')
          + (f', {len(refused)} need recalibration' if refused else ''))
    if advised:
        print(f'Usable now, but recalibrating is advised for: {", ".join(advised)}')
    if refused:
        print('Recalibrate these on a clean, flat, light-coloured floor:\n'
              '  REx_line_sensor_calibrate '
              + ' '.join(f'-s {n}' for n in refused))
    if migrated and not args.dry_run:
        print('Restart the body server to load them if line sensor loop is enabled.')
    sys.exit(0 if not refused else 1)


if __name__ == '__main__':
    main()
