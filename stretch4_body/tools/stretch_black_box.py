#!/usr/bin/env python3
"""
stretch_black_box: the robot's black box (stretch4_body/core/black_box.py).

  run      record until stopped; this is what the stretch_black_box user service runs
  why      what happened around a time, or around the last trigger
  mark     write a hand mark, which also takes a capture ("that looked wrong just now")
  install  install and start the user service

It reads only what stretch_body_server publishes and the journal; it never opens a
robot serial port.
"""
import argparse
import datetime
import json
import os
import pathlib
import queue
import subprocess
import sys
import threading
import time

from stretch4_body.core import black_box

SERVICE_NAME = 'stretch_black_box.service'


def default_root():
    return os.path.join(os.getenv('HELLO_FLEET_PATH', os.path.expanduser('~')), 'log', 'black_box')


def follow_journal(args, source, out):
    p = subprocess.Popen(['journalctl', '-f', '-n', '0', '-o', 'json'] + args,
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    for line in p.stdout:
        try:
            entry = json.loads(line)
        except ValueError:
            continue
        t = int(entry.get('__REALTIME_TIMESTAMP', 0)) / 1e6 or time.time()
        out.put((t, source, entry))


def run(root, rate_hz):
    from stretch4_body.robot.robot_client import RobotClient
    os.nice(10)
    rec = black_box.Recorder(root, black_box.ParquetWriter())
    lines = queue.Queue()
    for args, source in ((['-k'], 'kernel'), (['--user', '-u', 'stretch_body_server'], 'server')):
        threading.Thread(target=follow_journal, args=(args, source, lines), daemon=True).start()
    r = RobotClient()
    while not r.startup():
        rec.event(time.time(), 'black_box', 'server_down', 'RobotClient could not start', 'server_down')
        rec.tick(time.time())
        time.sleep(5)
    period = 1.0 / rate_hz
    last_tick = 0.0
    try:
        while True:
            t = time.time()
            r.pull_status()
            rec.sample(t, r.status)
            while not lines.empty():
                rec.journal(*lines.get())
            if t - last_tick >= 1.0:
                last_tick = t
                rec.poll_marks(t)
                rec.tick(t)
            time.sleep(max(0.0, period - (time.time() - t)))
    except KeyboardInterrupt:
        pass
    finally:
        rec.flush(time.time())
        r.stop()


def parse_time(text):
    if text in (None, 'last'):
        return None
    try:
        return float(text)
    except ValueError:
        d = datetime.datetime.fromisoformat(text) if '-' in text else datetime.datetime.combine(
            datetime.date.today(), datetime.time.fromisoformat(text))
        return d.timestamp()


def why(root, at, span_s):
    import pyarrow.compute as pc
    import pyarrow.parquet as pq
    files = sorted(str(p) for d in ('hours', 'chunks') for p in pathlib.Path(root, d).glob('*.events.parquet'))
    tables = [pq.read_table(f) for f in files]
    if not tables:
        print('No black box events yet.')
        return 1
    import pyarrow
    ev = pyarrow.concat_tables(tables).sort_by('t').to_pylist()
    if at is None:
        trig = [e for e in ev if e['kind'] == 'trigger']
        if not trig:
            print('No trigger recorded.')
            return 1
        at = trig[-1]['t']
    fmt = lambda t: time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t)) + f'.{int(t * 1000) % 1000:03d}'
    print(f'Around {fmt(at)}, {span_s:g} s either side:')
    for e in ev:
        if abs(e['t'] - at) <= span_s:
            detail = e['detail']
            if e['source'] == 'pimu':
                d = json.loads(detail)
                detail = d.get('detail') or d.get('explanation', '')
            print(f"  {fmt(e['t'])}  {e['source']:<9} {e['kind']:<28} {detail[:140]}")
    caps = sorted(pathlib.Path(root, 'captures').glob('*.parquet'))
    near = [c for c in caps if abs(time.mktime(time.strptime(c.name[:15], '%Y%m%d_%H%M%S')) - at) <= span_s]
    for c in near:
        trig = json.loads(pq.read_schema(c).metadata[b'triggers'])
        print(f'  capture {c}: ' + ', '.join(f"{x['kind']} (+{x['repeats_before']} before)" for x in trig))
    samples = [str(p) for d in ('hours', 'chunks') for p in pathlib.Path(root, d).glob('*.samples.parquet')]
    if samples:
        table = pyarrow.concat_tables([pq.read_table(f, filters=[('t', '>=', at - span_s), ('t', '<=', at + span_s)])
                                       for f in samples])
        table = table.set_column(1, 'channel', pc.cast(table['channel'], pyarrow.string()))
        agg = table.group_by('channel').aggregate([('value', 'min'), ('value', 'max'), ('value', 'count')])
        moved = [(r['channel'], r['value_min'], r['value_max'], r['value_count']) for r in agg.to_pylist()
                 if r['value_count'] > 1 and r['value_max'] != r['value_min']]
        print(f'  {len(moved)} channels moved in the window:')
        for ch, lo, hi, n in sorted(moved)[:60]:
            print(f'    {ch:<64} {lo:>12.4g} to {hi:<12.4g} ({n} rows)')
    return 0


def mark(root, text):
    d = os.path.join(root, 'marks')
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, f'{time.time():.3f}'), 'w') as f:
        f.write(text)


def install():
    unit = pathlib.Path.home() / '.config' / 'systemd' / 'user' / SERVICE_NAME
    exe = subprocess.run(['which', 'stretch_black_box'], capture_output=True, text=True).stdout.strip() or \
        f'{sys.executable} -m stretch4_body.tools.stretch_black_box'
    unit.write_text(f"""\
[Unit]
Description=Stretch black box recorder
After=stretch_body_server.service

[Service]
Type=simple
Environment="HELLO_FLEET_PATH={os.getenv('HELLO_FLEET_PATH', '')}"
Environment="HELLO_FLEET_ID={os.getenv('HELLO_FLEET_ID', '')}"
ExecStart={exe} run
Nice=10
IOSchedulingClass=idle
Restart=always
RestartSec=10

[Install]
WantedBy=default.target
""")
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    subprocess.run(['systemctl', '--user', 'enable', '--now', SERVICE_NAME], check=True)
    print(f'Installed and started {unit}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=('run', 'why', 'mark', 'install'))
    parser.add_argument('arg', nargs='?', help="why: a time (HH:MM:SS, ISO or epoch) or 'last'; mark: the text")
    parser.add_argument('--root', default=default_root())
    parser.add_argument('--rate', type=float, default=100.0, help='status pulls per second')
    parser.add_argument('--span', type=float, default=60.0, help='why: seconds either side')
    a = parser.parse_args()
    if a.command == 'run':
        run(a.root, a.rate)
    elif a.command == 'why':
        sys.exit(why(a.root, parse_time(a.arg), a.span))
    elif a.command == 'mark':
        mark(a.root, a.arg or 'hand mark')
    else:
        install()


if __name__ == '__main__':
    main()
