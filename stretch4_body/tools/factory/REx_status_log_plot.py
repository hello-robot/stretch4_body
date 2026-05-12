#!/usr/bin/env python3
import json
import argparse
import glob
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import stretch4_body.core.hello_utils as hu

def flatten_dict(d, parent_key='', sep='.'):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)

def get_val_from_path(d, path):
    keys = path.split('.')
    val = d
    for k in keys:
        if isinstance(val, dict) and k in val:
            val = val[k]
        else:
            return None
    return val

def get_status_log_dir():
    return hu.get_stretch_directory() + 'log/stretch_status'

def main():
    hu.print_stretch_re_use()
    parser = argparse.ArgumentParser(description='Plot time series from stretch status logs.')
    parser.add_argument('--duration_hrs', type=float, default=1.0, help='Time to plot in hours from most recent log')
    parser.add_argument('--n_plot', type=int, default=1, choices=[1, 2, 3, 4], help='Number of fields to plot (max 4)')
    parser.add_argument('--filter', nargs='+', help='Filter the available fields (must contain all provided words)')
    args = parser.parse_args()

    log_dir = get_status_log_dir()
    log_files = glob.glob(os.path.join(log_dir, 'status_*.json'))
    log_files.sort(reverse=True) # newest first

    if not log_files:
        print(f"No log files found in {log_dir}")
        return

    # Find the most recent entry to get the schema and T_max
    print(f"Reading schema from the most recent log file: {log_files[0]}")
    with open(log_files[0], 'r') as f:
        data = json.load(f)
        
    if not data:
        print("Most recent log file is empty.")
        return
        
    most_recent_entry = data[-1]
    T_max = most_recent_entry['timestamp']
    flat_entry = flatten_dict(most_recent_entry)
    
    # Filter valid fields (bool, int, float)
    valid_fields = []
    for k, v in flat_entry.items():
        if type(v) in [int, float, bool]:
            if args.filter:
                if all(word in k for word in args.filter):
                    valid_fields.append(k)
            else:
                valid_fields.append(k)
            
    valid_fields.sort()
    
    # Interactive menu
    print("Available fields:")
    for i, field in enumerate(valid_fields):
        print(f"{i}: {field}")
        
    selected_indices = []
    for i in range(args.n_plot):
        while True:
            try:
                idx_str = input(f"Select field {i+1} of {args.n_plot} (enter index): ")
                idx = int(idx_str)
                if 0 <= idx < len(valid_fields):
                    if idx not in selected_indices:
                        selected_indices.append(idx)
                        break
                    else:
                        print("Field already selected.")
                else:
                    print("Invalid index.")
            except ValueError:
                print("Please enter a valid integer.")
                
    selected_fields = [valid_fields[idx] for idx in selected_indices]
    print(f"Selected fields: {selected_fields}")
    
    # Load data for the duration
    T_min = T_max - args.duration_hrs * 3600.0
    
    # We will gather data backwards, then reverse it for plotting
    timestamps = []
    plot_data = {field: [] for field in selected_fields}
    
    print(f"Loading data from {T_min} to {T_max}...")
    for lf in log_files:
        print(f"Loading {lf}...")
        with open(lf, 'r') as f:
            file_data = json.load(f)
            
        # file_data is sorted by time chronologically
        in_range = False
        for entry in reversed(file_data):
            t = entry['timestamp']
            if t < T_min:
                break
            if t <= T_max:
                in_range = True
                timestamps.append(t)
                for field in selected_fields:
                    val = get_val_from_path(entry, field)
                    if val is None:
                        val = 0.0
                    elif isinstance(val, bool):
                        val = float(val)
                    plot_data[field].append(val)
        
        # If the first element we checked (the oldest in this file) is < T_min, we don't need older files
        if len(file_data) > 0 and file_data[0]['timestamp'] < T_min:
            break

    if not timestamps:
        print("No data found in the specified time range.")
        return

    # Reverse to make it chronological
    timestamps.reverse()
    for field in selected_fields:
        plot_data[field].reverse()

    # Make timestamps relative to start
    t0 = timestamps[0]
    rel_timestamps = [(t - t0) for t in timestamps]

    # Plotting
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    plt.ion()
    print("Plotting data...")
    plt.figure(figsize=(10, 6))
    for field in selected_fields:
        plt.plot(rel_timestamps, plot_data[field], label=field)
        
    plt.xlabel(f'Time (s) since {t0}')
    plt.ylabel('Value')
    plt.title(f'Stretch Status Fields (Last {args.duration_hrs} hours)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    # png_path = os.path.join(log_dir, 'status_plot.png')
    # plt.savefig(png_path)
    # print(f"Plot saved to {png_path}")
    plt.show()
    input('Hit enter to exit')
    
if __name__ == '__main__':
    main()
