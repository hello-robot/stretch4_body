#!/usr/bin/env python3
"""Calibrate the line sensors against a flat floor.

Records a session, computes a per-bin tare from it, and saves that tare only
if the run clears its acceptance gates. A run that does not clear them writes
its report and refuses to save -- a partial tare is worse than none, because
nothing downstream can tell a bad correction from a good one.

The chip's status codes (5.11 no-detection, 5.09 beyond-range) never take part
in the arithmetic. They are identified from the per-bin `codes` array, and a
bin that mostly returns them is rejected rather than averaged in.

    REx_line_sensor_calibrate --all
    REx_line_sensor_calibrate -s sensor_1 -s sensor_5 --print-per-bin
    REx_line_sensor_calibrate --all --dry-run
    REx_line_sensor_calibrate --all --inspect
    REx_line_sensor_calibrate --recompute <session_id|path>   # no robot needed

A full run also starts with the same quick inspection and refuses to record
the long session on a floor that cannot pass (--no-preflight skips this).

Calibrate on a clean, flat, light-coloured floor with nothing within ~0.5 m of
the base. Dark or glossy surfaces make the sensors no-return, and a run built
on those is refused rather than shipped.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import datetime
import os
import sys

import numpy as np

from stretch4_body.subsystem.line_sensor import calibration, calibration_store, protocol
from stretch4_body.subsystem.line_sensor.calibration import CalibrationThresholds

_D = CalibrationThresholds()

# The quick floor inspection records this many frames (~2 s at 30 Hz) --
INSPECT_FRAMES = 60


def _parse_args():
    p = argparse.ArgumentParser(
        prog='REx_line_sensor_calibrate',
        description='Flat-floor tare calibration for the PixArt line sensors.')
    sel = p.add_mutually_exclusive_group()
    sel.add_argument('-s', '--sensor-name', '--sensor_name', action='append',
                     help='sensor to calibrate, e.g. sensor_1. May be repeated.')
    sel.add_argument('--all', action='store_true', help='calibrate every sensor')
    p.add_argument('--frames', type=int, default=300,
                   help='DISTINCT frames to record per sensor (default 300)')
    p.add_argument('--timeout', type=float, default=None,
                   help='seconds to wait for those frames (default scales with --frames)')
    p.add_argument('--recompute', metavar='SESSION',
                   help='re-run the maths on a saved session (id or path) and '
                        'promote the result; needs no robot')
    p.add_argument('--dry-run', action='store_true',
                   help='record and report, but never save a tare')
    p.add_argument('--inspect', action='store_true',
                   help='quick (~2s) check of whether this floor can support a '
                        'calibration at all.')
    p.add_argument('--no-preflight', action='store_true',
                   help='skip the automatic floor inspection that normally runs '
                        'before the long recording')
    p.add_argument('--print-per-bin', action='store_true',
                   help='print per-bin counts for every bin')
    p.add_argument('--min-range-m', type=float, default=_D.min_range_m)
    p.add_argument('--max-range-m', type=float, default=_D.max_range_m)
    p.add_argument('--min-valid-fraction', type=float, default=_D.min_valid_fraction)
    p.add_argument('--max-abs-tare-m', type=float, default=_D.max_abs_tare_m)
    p.add_argument('--max-bin-mad-m', type=float, default=_D.max_bin_mad_m)
    p.add_argument('--min-accepted-bin-fraction', type=float,
                   default=_D.min_accepted_bin_fraction)
    p.add_argument('--min-frames', type=int, default=_D.min_frames)
    return p.parse_args()


def _thresholds(args):
    th = CalibrationThresholds(
        min_range_m=args.min_range_m, max_range_m=args.max_range_m,
        min_valid_fraction=args.min_valid_fraction,
        max_abs_tare_m=args.max_abs_tare_m, max_bin_mad_m=args.max_bin_mad_m,
        min_accepted_bin_fraction=args.min_accepted_bin_fraction,
        min_frames=args.min_frames)
    th.validate()      # one place; argparse does not restate these rules
    return th


def _print_report(name, rec, result, print_per_bin):
    s = result.summary
    st = rec.stats
    total = max(s['total_bin_samples'], 1)
    print(f"\n=== {name} ===")
    print(f"  frames: {st['distinct_frames_captured']} distinct of "
          f"{st['requested_frames']} requested in {st.get('wall_clock_s', 0)}s "
          f"({st['achieved_frames_per_s']}/s); "
          f"{st['duplicate_frames_skipped']} duplicate polls skipped")
    if st.get('frame_id_regressions'):
        print(f"  WARNING {st['frame_id_regressions']} frame-id regressions")
    if st.get('ever_in_sensors_dead'):
        print('  WARNING sensor appeared in sensors_dead during the run')
    print(f"  samples: valid {s['valid_samples']} ({100*s['valid_samples']/total:.2f}%)  "
          f"no-return 5.11 {s['no_return_samples']} ({s['no_return_pct']:.2f}%)  "
          f"beyond-limit 5.09 {s['beyond_limit_samples']} ({s['beyond_limit_pct']:.2f}%)")
    print(f"  out-of-window (a real return, but not floor): {s['out_of_window_samples']}"
          f"   other invalid: {s['other_invalid_samples']}")
    print(f"  bins accepted: {s['bins_accepted_for_tare']}/{s['bins_per_frame']} "
          f"(need {s['required_accepted_bins']}; "
          f"{s['required_valid_frames_per_bin']} valid frames per bin)")
    print(f"  median offset {s['median_accepted_offset_m']*1000:+.2f} mm   "
          f"per-bin MAD median/p95/max "
          f"{(s['bin_mad_m_median'] or 0)*1000:.2f}/"
          f"{(s['bin_mad_m_p95'] or 0)*1000:.2f}/"
          f"{(s['bin_mad_m_max'] or 0)*1000:.2f} mm")
    for key, label in (('never_returned', 'never returned'),
                       ('insufficient_data', 'too few valid frames'),
                       ('implausible_offset', 'implausible offset'),
                       ('high_dispersion', 'high dispersion'),
                       ('beyond_limit_majority', 'mostly 5.09 beyond-limit')):
        idx = result.indices.get(key) or []
        if idx:
            shown = idx if len(idx) <= 12 else idx[:12] + ['...']
            print(f"  bins {label}: {len(idx)} {shown}")
    for w in result.warnings:
        print(f"  ~ {w}")

    if print_per_bin:
        print('  per-bin:')
        pb = result.per_bin
        for b in range(s['bins_per_frame']):
            print(f"    bin {b:03d}: valid={pb['valid_counts'][b]:4d} "
                  f"5.11={pb['no_return_counts'][b]:4d} "
                  f"5.09={pb['beyond_limit_counts'][b]:4d} "
                  f"out={pb['out_of_window_counts'][b]:4d} "
                  f"{'ACCEPTED' if result.valid_mask[b] else 'REJECTED'}")


def _inspect_floor(calib, targets, thresholds, n_frames=INSPECT_FRAMES):
    """Fast check to see if the floor is suitable to run calibration on.
    Returns (all_ok, {name: (ok, verdict_line)}).
    """
    session = calib.record_session(n_frames=n_frames, sensors=targets,
                                   timeout_s=max(15.0, n_frames / 10.0))
    verdicts = {}
    for name in targets:
        rec = session.recordings.get(name)
        if rec is None or rec.n_frames == 0:
            verdicts[name] = (False, 'no frames received -- sensor dead?')
            continue
        codes = np.asarray(rec.codes)
        ranges = np.asarray(rec.ranges, dtype=float)
        total = float(codes.size)
        no_ret = np.count_nonzero(codes == protocol.CODE_NO_RETURN) / total
        beyond = np.count_nonzero(codes == protocol.CODE_BEYOND_LIMIT) / total
        valid = codes == protocol.CODE_VALID
        floor = valid & (ranges >= thresholds.min_range_m) \
                      & (ranges <= thresholds.max_range_m)
        out_win = np.count_nonzero(valid) / total - np.count_nonzero(floor) / total
        
        bin_ok = np.mean(floor, axis=0) >= thresholds.min_valid_fraction
        bins_frac = float(np.mean(bin_ok))
        stats = (f'floor {100 * np.count_nonzero(floor) / total:5.1f}%  '
                 f'5.11 {100 * no_ret:5.1f}%  5.09 {100 * beyond:5.1f}%  '
                 f'off-window {100 * out_win:5.1f}%  '
                 f'bins-would-pass {100 * bins_frac:5.1f}%')
        if bins_frac >= thresholds.min_accepted_bin_fraction:
            note = ('  (note: some no-return; a darker patch?)'
                    if no_ret > 0.02 else '')
            verdicts[name] = (True, f'OK    {stats}{note}')
        elif no_ret >= 0.5:
            verdicts[name] = (False, f'DARK FLOOR  {stats} -- the surface '
                                     f'absorbs the beam (5.11 no-return); '
                                     f'calibrate on a lighter floor')
        elif beyond >= 0.10:
            verdicts[name] = (False, f'NO FLOOR    {stats} -- 5.09 beyond-limit: '
                                     f'the beam passes where the floor should '
                                     f'be (edge/cliff/very glossy)')
        elif out_win >= 0.10:
            verdicts[name] = (False, f'OBSTRUCTED  {stats} -- returns exist but '
                                     f'not at floor range; clear ~0.5 m around '
                                     f'the base')
        else:
            verdicts[name] = (False, f'PATCHY      {stats} -- too many bins '
                                     f'below the {thresholds.min_valid_fraction:.0%} '
                                     f'valid-frame gate')
    return all(ok for ok, _ in verdicts.values()), verdicts


def _print_inspection(verdicts):
    print('\nfloor inspection:')
    for name, (ok, line) in verdicts.items():
        print(f"  {name}: {line}")
    if all(ok for ok, _ in verdicts.values()):
        print('  => floor looks usable for calibration')
    else:
        print('  => this floor CANNOT produce a full tare; fix the setup '
              '(clean, flat, light-coloured floor,\n     nothing within ~0.5 m '
              'of the base) and re-run')


def _score_session(session, ideal, thresholds, base_dir, out_dir, args, targets):
    """Compute, report and (unless refused) promote a tare for each sensor."""
    report = {
        'format_version': calibration_store.TARE_FORMAT_VERSION,
        'session_id': session.session_id,
        'timestamp': datetime.datetime.now().isoformat(),
        'thresholds': asdict(thresholds),
        'ideal_range_m': float(ideal),
        'ideal_range_model': calibration.IDEAL_RANGE_MODEL,
        'recording': {
            'requested_frames': session.requested_frames,
            'poll_iterations': session.poll_iterations,
        },
        'sensors': {},
    }
    saved = refused = 0

    for name in targets:
        rec = session.recordings.get(name)
        entry = {'status': 'NO_DATA', 'provenance': dict(rec.stats) if rec else {}}
        if rec is None or rec.n_frames == 0:
            print(f"\n=== {name} ===\n  no frames captured; nothing to compute")
            entry['status'] = rec.status if rec else 'NO_DATA'
            report['sensors'][name] = entry
            refused += 1
            continue

        fp = session.fingerprints.get(name, {})
        entry['config_fingerprint_sha256'] = fp.get('sha256', '')
        try:
            result = calibration.compute_sensor_tare(
                rec.ranges, rec.codes, ideal, thresholds)
        except ValueError as exc:
            print(f"\n=== {name} ===\n  cannot compute: {exc}")
            entry['status'] = 'NO_DATA'
            entry['error'] = str(exc)
            report['sensors'][name] = entry
            refused += 1
            continue

        _print_report(name, rec, result, args.print_per_bin)
        entry.update({
            'summary': result.summary,
            'indices': result.indices,
            'per_bin': result.per_bin,
            'proposed_tare_offsets': [float(v) for v in result.offsets],
            'tare_valid_mask': [bool(v) for v in result.valid_mask],
            'insufficiency_reasons': list(result.insufficiency_reasons),
            'warnings': list(result.warnings),
        })

        if not result.sufficient:
            print(f"\n  *** {name}: RUN INSUFFICIENT -- NO TARE SAVED ***")
            for r in result.insufficiency_reasons:
                print(f"    - {r}")
            print('    Fix the setup (clean, flat, light-coloured floor; nothing')
            print('    within ~0.5 m of the base) and re-run.')
            entry['status'] = 'INSUFFICIENT'
            refused += 1
        elif args.dry_run:
            print(f"  {name}: dry run, tare not saved")
            entry['status'] = 'DRY_RUN'
        else:
            tare = calibration_store.build_tare_yaml(
                name, rec.sensor_index, result, ideal, thresholds,
                fp.get('fingerprint', {}), session_id=session.session_id,
                stretch_body_version=session.stretch_body_version,
                n_frames=rec.n_frames)
            path = calibration_store.write_tare(tare, out_dir, base_dir)
            print(f"  {name}: SAVED {path}")
            entry['status'] = 'SAVED'
            saved += 1
        report['sensors'][name] = entry

    # Always written, for every sensor, saved or not: the report is the
    # debugging artifact for a refused run, the tare is the gated output.
    report_path = os.path.join(out_dir, 'calibration_report.yaml')
    calibration_store.write_report(report, report_path)
    print(f"\nreport: {report_path}")
    return saved, refused


def _recompute(args, thresholds):
    """Re-score a saved session. Uses the session's own params snapshot, so an
    archived run reproduces its original numbers on any machine."""
    from stretch4_body.core import hello_utils as hu
    base_dir = os.path.join(hu.get_fleet_directory(), 'calibration_line_sensors')
    path = (args.recompute if os.path.exists(args.recompute)
            else calibration_store.session_dir(base_dir, args.recompute))
    session = calibration_store.read_session(path)
    geom = session.loop_params_snapshot['line_sensor_geometry']
    ideal = (geom['emitter_height_above_floor_mm'] / 1000.0
             / np.sin(np.deg2rad(geom['sensor_angle_down_deg'])))
    targets = (args.sensor_name if args.sensor_name else list(session.recordings))
    print(f'recomputing session {session.session_id}, recorded {session.started_at}')
    print(f'ideal flat-floor range {ideal:.6f} m')
    out_dir = calibration_store.session_dir(base_dir, session.session_id)
    saved, refused = _score_session(session, ideal, thresholds, base_dir,
                                    out_dir, args, targets)
    return 1 if refused or (not args.dry_run and not saved) else 0


def main():
    args = _parse_args()
    try:
        thresholds = _thresholds(args)
    except ValueError as exc:
        print(f'bad thresholds: {exc}', file=sys.stderr)
        return 2
    if args.frames <= 0:
        print('--frames must be positive', file=sys.stderr)
        return 2

    if args.recompute:
        return _recompute(args, thresholds)

    if not args.all and not args.sensor_name:
        print('choose --all or one or more -s <sensor>', file=sys.stderr)
        return 2

    from stretch4_body.subsystem.line_sensor import connect
    from stretch4_body.subsystem.line_sensor.line_sensor_utils import LineSensorCalibration

    try:
        conn = connect.open_line_sensors('line_sensor_calibrate')
    except connect.LineSensorUnavailable as exc:
        print(exc.detail, file=sys.stderr)
        return 1
    print(conn.describe())
    try:
        calib = LineSensorCalibration(conn.loop)
        targets = list(calib.sensor_names) if args.all else args.sensor_name
        unknown = [s for s in targets if s not in calib.sensor_names]
        if unknown:
            print(f'unknown sensors {unknown}; configured: {calib.sensor_names}',
                  file=sys.stderr)
            return 2

        ideal = calib.compute_ideal_range()
        print('\nPlace the robot on a clean, flat, light-coloured floor with')
        print('nothing within ~0.5 m of the base.')
        print(f'ideal flat-floor range {ideal:.6f} m ({calibration.IDEAL_RANGE_MODEL})')

        if args.inspect or not args.no_preflight:
            print(f'inspecting the floor ({INSPECT_FRAMES} frames)...')
            all_ok, verdicts = _inspect_floor(calib, targets, thresholds)
            _print_inspection(verdicts)
            if args.inspect:
                return 0 if all_ok else 1
            if not all_ok:
                print('\naborting before the long recording (--no-preflight '
                      'records anyway)')
                return 1

        print(f'recording {args.frames} distinct frames for: {", ".join(targets)}')

        session = calib.record_session(n_frames=args.frames, sensors=targets,
                                       timeout_s=args.timeout)
        base_dir = calib.get_calibration_base_dir()
        out_dir = calibration_store.write_session(session, base_dir)
        print(f'session: {out_dir}')

        saved, refused = _score_session(session, ideal, thresholds, base_dir,
                                        out_dir, args, targets)
        if refused:
            print(f'\n{refused} sensor(s) produced no tare.')
            return 1
        return 0 if (args.dry_run or saved) else 1
    except KeyboardInterrupt:
        print('\ninterrupted; no tare saved')
        return 130
    finally:
        conn.close()


if __name__ == '__main__':
    raise SystemExit(main())
