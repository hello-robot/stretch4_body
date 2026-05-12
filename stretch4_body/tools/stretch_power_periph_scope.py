#!/usr/bin/env python3

from stretch4_body.subsystem.power_periph import PowerPeriph
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()
from stretch4_body.core.rerun_plot import RRplot


import argparse
import os
import json

parser=argparse.ArgumentParser(description='Scope values of the Power Periph board')
parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
parser.add_argument("-n", "--num_scalars", help="Number of scalars to plot at a time (default=1, max=5)", type=int, default=1)
parser.add_argument("--last_config", help="Load the last used configuration", action="store_true")
args=parser.parse_args()

if args.num_scalars < 1:
    args.num_scalars = 1
elif args.num_scalars > 5:
    args.num_scalars = 5

CONFIG_FILE = "/tmp/stretch_power_periph_scope_config.json"

if not args.direct:
    from stretch4_body.robot.robot_client import PowerPeriphClient as PowerPeriph
else:
    from stretch4_body.subsystem.power_periph import PowerPeriph

p=PowerPeriph()

if not p.startup():
    exit()

p.pull_status()

# Find all scalar keys
scalar_keys = [k for k, v in p.status.items() if type(v) in [int, float]]
scalar_keys.sort()

selected_keys = []

if args.last_config and os.path.exists(CONFIG_FILE):
    try:
        with open(CONFIG_FILE, 'r') as f:
            selected_keys = json.load(f)
        print(f"Loaded config: {selected_keys}")
    except Exception as e:
        print(f"Failed to load config: {e}")

if not selected_keys:
    print("Select scalars to scope:")
    for i, k in enumerate(scalar_keys):
        print(f"[{i}] {k}")

    for i in range(args.num_scalars):
        try:
            selection = input(f"Enter index for scalar {i+1}/{args.num_scalars}: ")
            idx = int(selection)
            selected_keys.append(scalar_keys[idx])
        except (ValueError, IndexError):
            print("Invalid index. Exiting...")
            p.stop()
            exit()
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(selected_keys, f)
    except Exception:
        pass

print(f"Scoping {', '.join(selected_keys)}...")

try:
    rrplot = RRplot(name="PowerPeriph", open_browser=True)
    for i, key in enumerate(selected_keys):
        rrplot.register(key=key, color_idx=i)
    rrplot.setup_blueprint(collapse_panels=False)
    while True:
        try:
            p.pull_status()
            for key in selected_keys:
                rrplot.log_scalar(key=key, value=p.status[key])
        except (ValueError):
            print('Bad input...')
except (KeyboardInterrupt, SystemExit):
    p.stop()
