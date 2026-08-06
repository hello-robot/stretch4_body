#!/usr/bin/env python3
"""Live per-bin view of the EXACT range each line-sensor bin reports, in metres.

Shows what the chip actually said.

Bins are classified by the per-bin `codes` array the reader publishes,
classified once at decode from the raw wire values. `ranges` is NaN wherever
a bin is not a distance measurement, so nothing here can average a status
code into a floor reading.

Shaded bands: the floor line sits at the ideal range (emitter height /
sin(down pitch)); shorter ranges are the obstacle band, longer the drop band.

Hover any bin for a readout of sensor / bin / exact value / class.
Keys: [space] freeze, [s] dump the current frame to CSV, [q] quit.

"""

from __future__ import annotations

import argparse
import os
import time

import numpy as np

from stretch4_body.subsystem.line_sensor import protocol


def _unbreak_qt():
    """opencv-python ships its own Qt plugins and points
    QT_QPA_PLATFORM_PLUGIN_PATH at them on import. They do not match the
    system Qt matplotlib links against, so the xcb platform plugin is "found"
    but refuses to load and the process aborts. Anything importing cv2 --
    RobotClient does, via the camera subsystem -- poisons the variable for
    whatever creates a window later. Drop it if it points into cv2 and let Qt
    find the system plugins the normal way.
    """
    path = os.environ.get('QT_QPA_PLATFORM_PLUGIN_PATH', '')
    if 'cv2' in path:
        del os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']

# Colours handed to code values in first-seen order. 5.11 and 5.09 are pinned
# so they keep the same colour from run to run.
PINNED = {protocol.RANGE_NO_DETECTION_M: '#4a90d9',
          protocol.RANGE_BEYOND_LIMIT_M: '#9b59b6'}
PALETTE = ['#e74c3c', '#16a085', '#e67e22', '#8e44ad', '#2c3e50', '#c0392b']

CODE_LABEL = {
    protocol.CODE_NO_RETURN: 'no return 5.11 m (blind: dark floor / cliff)',
    protocol.CODE_BEYOND_LIMIT: 'beyond limit 5.09 m (beam passed the floor: cliff)',
}
HOVER_KIND = {
    protocol.CODE_VALID: 'return',
    protocol.CODE_BEYOND_LIMIT: 'BEYOND-LIMIT CODE (beam passed the floor)',
    protocol.CODE_NO_RETURN: 'NO-RETURN CODE (nothing came back)',
    protocol.CODE_OTHER_INVALID: 'invalid',
}


class SentinelRows:
    """Gives each distinct status-code value its own colour and its own y row,
    in first-seen order, so an unknown code still lands somewhere visible."""

    def __init__(self, base: float, step: float):
        self.base = base
        self.step = step
        self.order: list = []
        self.colors: dict = {}

    def slot(self, value: float):
        key = round(float(value), 2)
        if key not in self.colors:
            self.order.append(key)
            self.colors[key] = PINNED.get(
                key, PALETTE[(len(self.order) - 1) % len(PALETTE)])
        return self.base + self.step * self.order.index(key), self.colors[key]

    def rows(self):
        return [(v, self.base + self.step * i, self.colors[v])
                for i, v in enumerate(self.order)]


def code_values(codes):
    """Per-bin float array holding the value each status code stands for, NaN
    at measurement bins. `ranges` is NaN at code bins, so the literal 5.09 /
    5.11 is recovered from the code itself."""
    values = np.full(len(codes), np.nan)
    for code, value in protocol.CODE_VALUE_M.items():
        values[codes == code] = value
    return values


def main() -> int:
    ap = argparse.ArgumentParser(
        description='Live per-bin line-sensor range values, in metres. '
                    'Run on the robot; needs a display.')
    ap.add_argument('--hz', type=float, default=10.0, help='refresh rate')
    ap.add_argument('--rmax', type=float, default=0.8,
                    help='top of the real-range axis, m (default 0.8)')
    ap.add_argument('--calib', action='store_true',
                    help='apply the tare the body serves, so a flat floor '
                         'should sit on the ideal-range line. Bins with no '
                         'trustworthy tare are left raw and reported.')
    args = ap.parse_args()

    # Import RobotClient FIRST: it pulls in cv2, which hijacks the Qt plugin
    # path. Scrub that before matplotlib gets a chance to initialise Qt.
    from stretch4_body.robot.robot_client import RobotClient
    _unbreak_qt()

    import matplotlib
    if not os.environ.get('MPLBACKEND'):
        # Tk is what the rest of the repo plots on (core/scope.py,
        # hello_utils.py, the REx_* tools) and it does not go near Qt, so the
        # cv2 plugin clash cannot arise at all. _unbreak_qt above still covers
        # anyone who forces a Qt backend through MPLBACKEND.
        matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    client = RobotClient(client_id='stretch_line_sensor_ranges')
    if not client.startup():
        print('RobotClient startup failed: is the body server running?')
        return 1
    if not hasattr(client, 'line_sensor_loop'):
        client.stop()
        print('line_sensor_loop is not enabled on the robot server '
              '(add it to the subsystems list in stretch_user_params.yaml)')
        return 1

    loop = client.line_sensor_loop
    names = list(loop.params['sensor_names'])
    geom = loop.params.get('line_sensor_geometry', {}) or {}
    nbins = geom.get('pixart_report_num', 320)
    r_ideal = (geom.get('emitter_height_above_floor_mm', 100.67) / 1000.0
               / np.sin(np.deg2rad(geom.get('sensor_angle_down_deg', 26.0))))

    if args.calib:
        # The body loaded and validated these; nothing is read from disk here.
        got = loop.calibrated_sensors()
        print(f'--calib: {len(got)}/{len(names)} sensors tared'
              f'{" -> " + ", ".join(got) if got else ""}')
        for name, why in sorted(loop.uncalibrated_sensors().items()):
            print(f'  {name}: UNCALIBRATED, shown raw ({why.split(":")[0]})')
        if not got:
            print('  nothing to apply; run REx_line_sensor_calibrate --all')

    rmax = float(args.rmax)
    cutoff = rmax                      # dashed line: real ranges below, codes above
    step = rmax * 0.075
    over_row = cutoff + step           # valid returns beyond rmax
    rows = SentinelRows(base=cutoff + 2 * step, step=step)
    ytop = cutoff + 7 * step

    plt.ion()
    fig, axes = plt.subplots(3, 2, figsize=(15, 9.5), sharex=True, sharey=True)
    fig.canvas.manager.set_window_title('line sensor raw ranges (m)')
    axes = axes.ravel()
    bins = np.arange(nbins)

    state = {'paused': False, 'frame': None, 'nrows': -1}

    real_sc, sent_sc, over_sc, titles, row_labels = [], [], [], [], []
    for ax, name in zip(axes, names):
        ax.axhspan(cutoff, ytop, color='#000000', alpha=0.04)   # code band
        ax.axhline(cutoff, color='#666', lw=0.9, ls='--')
        # floor line at the ideal range; shorter = obstacle, longer = drop
        ax.axhline(r_ideal, color='#888888', lw=0.8)
        ax.axhspan(max(r_ideal - 0.075, 0.0), max(r_ideal - 0.010, 0.0),
                   color='#f0ad4e', alpha=0.10)
        ax.axhspan(min(r_ideal + 0.010, cutoff), min(r_ideal + 0.30, cutoff),
                   color='#d9534f', alpha=0.08)
        real_sc.append(ax.scatter(bins, np.full(nbins, np.nan), s=6,
                                  c='#b0b0b0', zorder=3))
        sent_sc.append(ax.scatter([], [], s=10, zorder=4))
        over_sc.append(ax.scatter([], [], s=24, marker='^', c='#7f8c8d', zorder=4))
        titles.append(ax.set_title(name, fontsize=9, family='monospace'))
        row_labels.append([])
        ax.set_ylim(0.0, ytop)
        ax.set_xlim(0, nbins - 1)
        ax.set_ylabel('range m')
        ax.grid(axis='y', color='#eeeeee', lw=0.5)
    axes[-1].set_xlabel('bin')
    axes[-2].set_xlabel('bin')

    readout = fig.text(0.01, 0.985, '', family='monospace', fontsize=9,
                       va='top', color='#2c3e50')
    hover = fig.text(0.99, 0.985, '', family='monospace', fontsize=10,
                     va='top', ha='right', color='#c0392b')
    readout.set_text(
        f'ideal floor range {r_ideal:.4f} m    '
        f'axis 0..{rmax:.2f} m, status codes above the dashed line    '
        f'[space] freeze   [s] save CSV   [q] quit')

    def relabel_rows():
        """Print each active code's value at the right edge of every strip, so
        a row is readable without crossing to the legend."""
        for k, ax in enumerate(axes):
            for txt in row_labels[k]:
                txt.remove()
            row_labels[k] = [
                ax.text(nbins - 3, y + rows.step * 0.32, f'{value:.2f}',
                        color=colour, fontsize=7.5, family='monospace',
                        ha='right', va='center', zorder=6,
                        # the row is a solid line of markers all the way to the
                        # last bin, so the label needs to punch through it
                        bbox=dict(facecolor='white', edgecolor='none',
                                  alpha=0.85, pad=1.0))
                for value, y, colour in rows.rows()
            ]
        label_for = {v: CODE_LABEL[c] for c, v in protocol.CODE_VALUE_M.items()}
        handles = [Line2D([], [], marker='o', ls='', color=colour,
                          label=label_for.get(value, f'unknown code {value:.2f} m'))
                   for value, _y, colour in rows.rows()]
        handles.append(Line2D([], [], marker='^', ls='', color='#7f8c8d',
                              label=f'return > {rmax:.2f} m (off scale)'))
        handles.append(Line2D([], [], marker='o', ls='', color='#b0b0b0',
                              label='return (exact value on axis)'))
        if fig.legends:
            fig.legends.clear()
        fig.legend(handles=handles, loc='lower center',
                   ncol=max(len(handles), 1), fontsize=8)

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
        if not entry or not 0 <= b < len(entry['ranges']):
            hover.set_text('')
            return
        c = int(entry['codes'][b])
        v = float(entry['ranges'][b])
        shown = (f'{protocol.CODE_VALUE_M[c]:.2f} m' if c in protocol.CODE_VALUE_M
                 else ('no value' if not np.isfinite(v) else f'{v:.4f} m'))
        hover.set_text(f'{names[k]}  bin {b:3d}   {shown}   {HOVER_KIND.get(c, "?")}')

    def on_key(event):
        if event.key == ' ':
            state['paused'] = not state['paused']
            hover.set_text('FROZEN' if state['paused'] else '')
        elif event.key == 's' and state['frame'] is not None:
            path = time.strftime('line_sensor_ranges_%Y%m%d_%H%M%S.csv')
            with open(path, 'w', encoding='utf-8') as fh:
                fh.write('sensor,bin,range_m,code,code_name\n')
                for name, entry in state['frame'].items():
                    for b, (v, c) in enumerate(zip(entry['ranges'], entry['codes'])):
                        fh.write(f'{name},{b},{float(v):.4f},{int(c)},'
                                 f'{protocol.CODE_NAMES[int(c)]}\n')
            hover.set_text(f'wrote {path}')
        elif event.key == 'q':
            plt.close(fig)

    fig.canvas.mpl_connect('motion_notify_event', on_move)
    fig.canvas.mpl_connect('key_press_event', on_key)
    relabel_rows()
    fig.tight_layout(rect=(0, 0.06, 1, 0.97))

    period = 1.0 / max(args.hz, 0.1)
    try:
        # NOTE: never use plt.pause() here -- on the Tk backend it raises the
        # window every call, which makes it behave always-on-top.
        # flush_events() pumps GUI events without stealing focus.
        while plt.fignum_exists(fig.number):
            start = time.monotonic()
            if state['paused']:
                fig.canvas.flush_events()
                time.sleep(0.02)
                continue

            client.pull_status()
            status = loop.status
            # Liveness moved into the health block when status messages became
            dead = set((status.get('health') or {}).get('sensors_dead', ()))
            frame = {}
            for name in names:
                entry = status.get(name, {})
                ranges = entry.get('ranges') if isinstance(entry, dict) else None
                codes = entry.get('codes') if isinstance(entry, dict) else None
                if ranges is None or codes is None or not len(ranges):
                    continue
                ranges = np.asarray(ranges, dtype=float)
                codes = np.asarray(codes)
                if args.calib:
                    # Sentinel bins and bins without a trustworthy tare come
                    # back unchanged, so what you see is corrected where the
                    # correction is real and raw everywhere else.
                    ranges = loop.apply_tare(ranges, name, codes)
                frame[name] = {'ranges': ranges, 'codes': codes}
            state['frame'] = frame

            for k, name in enumerate(names):
                entry = frame.get(name)
                if entry is None:
                    titles[k].set_text(f'{name}   (no data)')
                    titles[k].set_color('#c0392b')
                    continue
                r, codes = entry['ranges'], entry['codes']
                n = len(r)
                idx = np.arange(n)
                real = codes == protocol.CODE_VALID
                values = code_values(codes)
                is_code = np.isfinite(values)
                on_scale = real & (r <= rmax)
                over = real & (r > rmax)

                real_sc[k].set_offsets(np.column_stack(
                    [idx, np.where(on_scale, r, np.nan)]))
                over_sc[k].set_offsets(
                    np.column_stack([idx[over], np.full(int(over.sum()), over_row)])
                    if over.any() else np.empty((0, 2)))

                pts, cols, counts = [], [], {}
                for b in np.flatnonzero(is_code):
                    value = round(float(values[b]), 2)
                    y, colour = rows.slot(value)
                    pts.append((b, y))
                    cols.append(colour)
                    counts[value] = counts.get(value, 0) + 1
                sent_sc[k].set_offsets(np.array(pts) if pts else np.empty((0, 2)))
                if cols:
                    sent_sc[k].set_color(cols)

                codes_txt = '  '.join(f'{v:.2f}x{c}' for v, c in
                                      sorted(counts.items(), key=lambda kv: -kv[1]))
                rmin = f'{np.nanmin(r[real]):.3f}' if real.any() else '  -  '

                # The verdict: of this sensor's no-returns, how many are the
                # 5.09 beyond-limit code rather than the blind 5.11. That share
                # is what separates "there is no floor here" from "the floor is
                # here but too dark to see", so put it in the title.
                n_code = int(is_code.sum())
                n_far = counts.get(protocol.RANGE_BEYOND_LIMIT_M, 0)
                verdict = (f'   5.09 {n_far}/{n_code} = {100 * n_far / n_code:4.1f}%'
                           if n_code else '')
                titles[k].set_text(
                    f'{name}{" DEAD" if name in dead else ""}  '
                    f'valid {int(real.sum()):3d}/{n}  rmin {rmin}m'
                    + (f'   {codes_txt}' if codes_txt else '') + verdict)
                titles[k].set_color(
                    '#c0392b' if (n_far or name in dead)
                    else ('#e67e22' if n_code > n // 2 else 'black'))

            if len(rows.order) != state['nrows']:
                state['nrows'] = len(rows.order)
                relabel_rows()

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(max(0.0, period - (time.monotonic() - start)))
    except KeyboardInterrupt:
        pass
    finally:
        client.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
