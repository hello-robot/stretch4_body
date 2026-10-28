#!/usr/bin/env python3
"""Live per-bin view of each line-sensor bin's HEIGHT z, in metres, with the
classification bands the hazard filter actually uses.

The sibling tool stretch_line_sensor_ranges shows the slant range the chip
reported; this one projects every bin into the robot floor frame with the same
`Projector` the filter pipeline uses and plots z, so a flat floor sits on the
zero line. The shaded bands are the filter's real thresholds (LineSensorConfig):
green = floor band (FREE), red = at/above the obstacle height, blue = the
small-drop band. 

Status-code bins (5.11 no-return, 5.09 beyond-limit) have no height; they are
drawn in their own rows above the axis, same colours as the ranges tool.

Hover any bin for sensor / bin / z / class.
Keys: [space] freeze, [s] dump the current frame to CSV, [q] quit.
"""

from __future__ import annotations

import argparse
import os
import time

import numpy as np

from stretch4_body.subsystem.line_sensor import connect, protocol
from stretch4_body.subsystem.line_sensor.line_sensor_utils import LineSensorGeometry
from stretch4_body.subsystem.line_sensor.filter.classify import classify_bin
from stretch4_body.subsystem.line_sensor.filter.config import LineSensorConfig
from stretch4_body.subsystem.line_sensor.filter.geometry import Projector
from stretch4_body.subsystem.line_sensor.filter.hits import BinClass


def _unbreak_qt():
    """cv2 (imported via RobotClient) points QT_QPA_PLATFORM_PLUGIN_PATH at
    its own Qt plugins, which the system Qt refuses to load. Scrub it."""
    path = os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH', '')
    if 'cv2' in path:
        del os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']


CLASS_COLOR = {
    BinClass.FREE: '#95a5a6',
    BinClass.OBSTACLE: '#e74c3c',
    BinClass.OBSTACLE_MARGINAL: '#e67e22',
    BinClass.SMALL_DROP: '#16a085',
    BinClass.DEEP_DROP: '#34495e',
    BinClass.UNKNOWN: '#d5d8dc',
}

CODE_COLOR = {protocol.CODE_NO_RETURN: '#4a90d9',
              protocol.CODE_BEYOND_LIMIT: '#9b59b6'}
CODE_LABEL = {
    protocol.CODE_NO_RETURN: 'no return 5.11 (blind: dark floor / cliff)',
    protocol.CODE_BEYOND_LIMIT: 'beyond limit 5.09 (beam passed the floor: cliff)',
}


def main() -> int:
    ap = argparse.ArgumentParser(
        description='Live per-bin line-sensor height (z band) view. '
                    'Run on the robot; needs a display.')
    ap.add_argument('--hz', type=float, default=10.0, help='refresh rate')
    ap.add_argument('--zmax', type=float, default=0.15,
                    help='half-height of the z axis, m (default 0.15)')
    ap.add_argument('--calib', action='store_true',
                    help='apply the tare the body serves and classify with the '
                         'fine deviation bands on bins whose tare is trusted; '
                         'without it every bin uses the coarse absolute bands.')
    args = ap.parse_args()

    # Connect first: this reaches RobotClient, which imports cv2, which
    # hijacks the Qt plugin path.
    try:
        conn = connect.open_line_sensors('stretch_line_sensor_z_band')
    except connect.LineSensorUnavailable as exc:
        print(exc.detail)
        return 1
    _unbreak_qt()
    print(conn.describe())

    import matplotlib
    if not os.environ.get('MPLBACKEND'):
        matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    loop = conn.loop
    names = list(loop.params['sensor_names'])
    geom = LineSensorGeometry(loop.params.get('line_sensor_geometry', {}) or {})
    cfg = LineSensorConfig()
    projector = Projector(geom, cfg)
    nbins = geom.pixart_report_num

    reliable = {}
    if args.calib:
        got = loop.calibrated_sensors()
        print(f'--calib: {len(got)}/{len(names)} sensors tared'
              f'{" -> " + ", ".join(got) if got else ""}')
        for name, why in sorted(loop.uncalibrated_sensors().items()):
            print(f'  {name}: UNCALIBRATED, shown raw ({why.split(":")[0]})')
        if not got:
            print('  nothing to apply; run REx_line_sensor_calibrate --all')
        reliable = loop.bin_reliable()

    zmax = float(args.zmax)
    # The filter's real bands. The drop band is expressed in z: classify
    # computes drop = -z / depth_underread_scale, so invert that here.
    floor_band = cfg.dev_floor_band_m if args.calib else cfg.floor_band_m
    scale = cfg.depth_underread_scale
    drop_lo = -cfg.cliff_max_drop_m * scale
    drop_hi = -cfg.cliff_min_drop_m * scale

    step = zmax * 0.15
    code_rows = {protocol.CODE_NO_RETURN: zmax + step,
                 protocol.CODE_BEYOND_LIMIT: zmax + 2 * step}
    ytop = zmax + 3 * step

    plt.ion()
    fig, axes = plt.subplots(3, 2, figsize=(15, 9.5), sharex=True, sharey=True)
    fig.canvas.manager.set_window_title('line sensor z band (m)')
    axes = axes.ravel()
    bins = np.arange(nbins)

    state = {'paused': False, 'frame': None}

    scatters, titles = [], []
    for ax, name in zip(axes, names):
        ax.axhspan(zmax, ytop, color='#000000', alpha=0.04)      # code rows
        ax.axhline(zmax, color='#666', lw=0.9, ls='--')
        ax.axhline(0.0, color='#888888', lw=0.8)                 # the floor
        ax.axhspan(-floor_band, floor_band, color='#2ecc71', alpha=0.10)
        ax.axhspan(cfg.line_obstacle_min_height_m, zmax,
                   color='#e74c3c', alpha=0.08)
        ax.axhspan(max(drop_lo, -zmax), drop_hi, color='#16a085', alpha=0.10)
        scatters.append(ax.scatter(bins, np.full(nbins, np.nan), s=8, zorder=3))
        titles.append(ax.set_title(name, fontsize=9, family='monospace'))
        ax.set_ylim(-zmax, ytop)
        ax.set_xlim(0, nbins - 1)
        ax.set_ylabel('z m')
        ax.grid(axis='y', color='#eeeeee', lw=0.5)
    axes[-1].set_xlabel('bin')
    axes[-2].set_xlabel('bin')

    readout = fig.text(0.01, 0.985, '', family='monospace', fontsize=9,
                       va='top', color='#2c3e50')
    hover = fig.text(0.99, 0.985, '', family='monospace', fontsize=10,
                     va='top', ha='right', color='#c0392b')
    readout.set_text(
        f'floor band ±{floor_band:.3f} m   '
        f'obstacle ≥ {cfg.line_obstacle_min_height_m:.3f} m   '
        f'drop band {drop_hi:.3f}..{drop_lo:.3f} m   '
        f'axis ±{zmax:.2f} m, codes above the dashed line   '
        f'[space] freeze  [s] save CSV  [q] quit')

    handles = [Line2D([], [], marker='o', ls='', color=CLASS_COLOR[c],
                      label=c.name.lower())
               for c in (BinClass.FREE, BinClass.OBSTACLE,
                         BinClass.OBSTACLE_MARGINAL, BinClass.SMALL_DROP,
                         BinClass.DEEP_DROP, BinClass.UNKNOWN)]
    handles += [Line2D([], [], marker='o', ls='', color=CODE_COLOR[c],
                       label=CODE_LABEL[c]) for c in CODE_COLOR]
    fig.legend(handles=handles, loc='lower center', ncol=4, fontsize=8)

    def on_move(event):
        if event.inaxes is None or state['frame'] is None or event.xdata is None:
            hover.set_text('')
            return
        try:
            k = list(axes).index(event.inaxes)
        except ValueError:
            hover.set_text('')
            return
        entry = state['frame'].get(names[k])
        b = int(round(event.xdata))
        if not entry or not 0 <= b < len(entry['z']):
            hover.set_text('')
            return
        c = int(entry['codes'][b])
        if c in protocol.CODE_VALUE_M:
            hover.set_text(f'{names[k]}  bin {b:3d}   '
                           f'{protocol.CODE_VALUE_M[c]:.2f} code')
            return
        z = float(entry['z'][b])
        cls = entry['cls'][b]
        shown = f'{z:+.4f} m' if np.isfinite(z) else 'no value'
        hover.set_text(f'{names[k]}  bin {b:3d}   z {shown}   {cls.name}')

    def on_key(event):
        if event.key == ' ':
            state['paused'] = not state['paused']
            hover.set_text('FROZEN' if state['paused'] else '')
        elif event.key == 's' and state['frame'] is not None:
            path = time.strftime('line_sensor_z_band_%Y%m%d_%H%M%S.csv')
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write('sensor,bin,z_m,class,code,code_name\n')
                for name, entry in state['frame'].items():
                    for b, (z, cls, c) in enumerate(zip(
                            entry['z'], entry['cls'], entry['codes'])):
                        fh.write(f'{name},{b},{float(z):.4f},{cls.name},'
                                 f'{int(c)},{protocol.CODE_NAMES[int(c)]}\n')
            hover.set_text(f'wrote {path}')
        elif event.key == 'q':
            plt.close(fig)

    fig.canvas.mpl_connect('motion_notify_event', on_move)
    fig.canvas.mpl_connect('key_press_event', on_key)
    fig.tight_layout(rect=(0, 0.08, 1, 0.97))

    period = 1.0 / max(args.hz, 0.1)
    try:
        # flush_events(), never plt.pause(): pause raises the Tk window every
        # call, which makes it behave always-on-top.
        while plt.fignum_exists(fig.number):
            start = time.monotonic()
            if state['paused']:
                fig.canvas.flush_events()
                time.sleep(0.02)
                continue

            conn.pull_status()
            status = loop.status
            dead = set((status.get('health') or {}).get('sensors_dead', ()))
            frame = {}
            for k, name in enumerate(names):
                entry = status.get(name, {})
                ranges = entry.get('ranges') if isinstance(entry, dict) else None
                codes = entry.get('codes') if isinstance(entry, dict) else None
                if ranges is None or codes is None or not len(ranges):
                    continue
                ranges = np.asarray(ranges, dtype=float)
                codes = np.asarray(codes)
                if args.calib:
                    ranges = loop.apply_tare(ranges, name, codes)
                # Same projection the filter uses; code bins are NaN in ranges
                # so their z comes out NaN and classify returns UNKNOWN.
                z = projector.project(k, ranges)[:, 2]
                contrast = projector.local_contrast(z, codes)
                rel = reliable.get(name)
                cls = [classify_bin(cfg, float(z[b]), float(contrast[b]),
                                    bool(rel[b]) if rel is not None else False)
                       for b in range(len(z))]
                frame[name] = {'z': z, 'codes': codes, 'cls': cls}
            state['frame'] = frame

            for k, name in enumerate(names):
                entry = frame.get(name)
                if entry is None:
                    titles[k].set_text(f'{name}   (no data)')
                    titles[k].set_color('#c0392b')
                    continue
                z, codes, cls = entry['z'], entry['codes'], entry['cls']
                n = len(z)
                y = np.clip(z, -zmax, zmax).astype(float)
                colors = [CLASS_COLOR[c] for c in cls]
                for code, row in code_rows.items():
                    at = codes == code
                    y[at] = row
                    for b in np.flatnonzero(at):
                        colors[b] = CODE_COLOR[code]
                # Codes not in CODE_VALUE_M and true NaN z stay NaN: invisible.
                off_rows = ~np.isin(codes, list(code_rows))
                y[off_rows & ~np.isfinite(z)] = np.nan

                sc = scatters[k]
                sc.set_offsets(np.column_stack([bins[:n], y]))
                sc.set_color(colors)

                counts = {}
                for c in cls:
                    counts[c] = counts.get(c, 0) + 1
                n_haz = (counts.get(BinClass.OBSTACLE, 0)
                         + counts.get(BinClass.OBSTACLE_MARGINAL, 0)
                         + counts.get(BinClass.SMALL_DROP, 0)
                         + counts.get(BinClass.DEEP_DROP, 0))
                valid = np.isfinite(z)
                zspan = (f'z {np.nanmin(z):+.3f}..{np.nanmax(z):+.3f}'
                         if valid.any() else 'z  -  ')
                titles[k].set_text(
                    f'{name}{" DEAD" if name in dead else ""}  '
                    f'free {counts.get(BinClass.FREE, 0):3d}/{n}  '
                    f'hazard {n_haz:3d}  {zspan}')
                titles[k].set_color('#c0392b' if (n_haz or name in dead)
                                    else 'black')

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(max(0.0, period - (time.monotonic() - start)))
    except KeyboardInterrupt:
        pass
    finally:
        conn.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
