#!/usr/bin/env python3
import json
import os
import time
import argparse
import sys
import rerun as rr

from stretch4_body.robot.robot_client import RobotClient

def flatten_status(d, parent_key='robot', sep='.'):
    """
    Recursively flatten a dictionary into dot-separated paths.
    Only includes scalar leaf values (int, float, bool).
    """
    items = []
    for k, v in d.items():
        if k == "timestamp":
            continue
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_status(v, new_key, sep=sep).items())
        elif isinstance(v, (int, float, bool, str)):
            items.append((new_key, v))
    return dict(items)

def filter_dict_by_fields(d, selected_fields, current_path="robot"):
    if not selected_fields or 'all' in selected_fields:
        return d
        
    filtered = {}
    for k, v in d.items():
        if k == "timestamp" and current_path == "robot":
            continue
            
        new_path = f"{current_path}.{k}" if current_path else k
        
        match = False
        full_match = False
        for sf in selected_fields:
            if sf == 'all':
                match = True
                full_match = True
                break
            if new_path == sf or new_path.startswith(sf + '.'):
                match = True
                full_match = True
                break
            if sf.startswith(new_path + '.'):
                match = True
                break
                
        if match:
            if full_match:
                filtered[k] = v
            elif isinstance(v, dict):
                filtered_sub = filter_dict_by_fields(v, selected_fields, new_path)
                if filtered_sub:
                    filtered[k] = filtered_sub
                    
    return filtered

def print_status_pretty(d, depth=0):
    for k, v in d.items():
        if k == "timestamp" and depth == 0:
            continue
        if depth == 0:
            prefix = "- "
        else:
            prefix = "-" * (4 * depth) + " "
            
        if isinstance(v, dict):
            print(f"{prefix}{k}:")
            print_status_pretty(v, depth + 1)
        elif isinstance(v, float):
            print(f"{prefix}{k}: {v:.4f}")
        else:
            print(f"{prefix}{k}: {v}")

def log_selected_fields(flat_status, selected_fields):
    for path, value in flat_status.items():
        match = False
        if selected_fields:
            for sf in selected_fields:
                if sf == 'all':
                    match = True
                    break
                if path == sf or path.startswith(sf + '.'):
                    match = True
                    break
        else:
            match = True
            
        if match:
            rr_path = path.replace('.', '/')
            # If boolean, cast to int
            if isinstance(value, bool):
                val = 1 if value else 0
                rr.log(rr_path, rr.Scalars(val))
            else:
                rr.log(rr_path, rr.Scalars(value))

def extract_all_groups(d, current_path="robot"):
    """
    Recursively extract all valid group prefixes and their corresponding leaf paths.
    """
    groups = {}
    
    def traverse(node, path):
        leaves = []
        for k, v in node.items():
            if k == "timestamp" and path == "robot":
                continue
            
            new_path = f"{path}.{k}" if path else k
            
            if isinstance(v, dict):
                sub_leaves = traverse(v, new_path)
                leaves.extend(sub_leaves)
            elif isinstance(v, (int, float, bool, str)):
                leaves.append(new_path)
                
        if leaves:
            groups[path] = leaves
        return leaves

    traverse(d, current_path)
    return groups

def setup_rerun_blueprint(rs, selected_fields):
    """
    Sets up a Rerun blueprint so that graphs are organized into fewer 
    TimeSeries views instead of spanning hundreds of default views.
    """
    try:
        import rerun.blueprint as rrb
        
        if not selected_fields or 'all' in selected_fields:
            views = []
            for k in rs.keys():
                if k == "timestamp":
                    continue
                views.append(rrb.TimeSeriesView(origin=f"robot/{k}", name=k, visible=False))
            
            views.append(rrb.TimeSeriesView(
                origin="robot/server/control_loop/avg_rate_hz",
                name="server.control_loop.avg_rate_hz",
                visible=True
            ))
            
            if views:
                blueprint = rrb.Blueprint(rrb.Tabs(*views))
                rr.send_blueprint(blueprint)
            return

        views = []
        for sf in selected_fields:
            if sf == 'all':
                views.append(rrb.TimeSeriesView(origin="/", name="Data"))
            else:
                views.append(rrb.TimeSeriesView(origin=sf.replace('.', '/'), name=sf))
                
        if len(views) == 1:
            layout = views[0]
        elif len(views) <= 4:
            layout = rrb.Grid(*views)
        else:
            layout = rrb.Tabs(*views)
            
        blueprint = rrb.Blueprint(layout)
        rr.send_blueprint(blueprint)
    except Exception as e:
        # Fallback to default auto-layout if rerun.blueprint is unavailable
        pass

def validate_selected_fields_or_exit(rs, selected_fields):
    if not selected_fields or 'all' in selected_fields:
        return
    
    flat = flatten_status(rs, parent_key='robot')
    paths = flat.keys()
    
    invalid_parts = []
    for part in selected_fields:
        match = False
        for p in paths:
            if part == p or p.startswith(part + '.'):
                match = True
                break
        if not match:
            invalid_parts.append(part)
            
    if invalid_parts:
        print(f"\n[!] Error: The following fields provided via --fields do not exist: {', '.join(invalid_parts)}")
        sys.exit(1)

def build_interactive_menu(sample_status):
    """
    Displays an interactive menu grouped by subsystem.
    Returns a list of selected field prefixes.
    """
    flat = flatten_status(sample_status, parent_key='robot')
    paths = sorted(list(flat.keys()))
    
    groups = extract_all_groups(sample_status, current_path='robot')
        
    print("\nAvailable subsystems and joints to visualize:")
    
    index_to_prefix = {}
    current_idx = 1
    
    group_names = sorted(list(groups.keys()))
    for g in group_names:
        if g == 'robot':
            continue
        print(f" {current_idx:2d}: {g} ({len(groups[g])} fields)")
        index_to_prefix[str(current_idx)] = g
        current_idx += 1
            
    while True:
        print("\nEnter comma-separated indices to view (e.g., 1, 2, 5),")
        print("or a prefix (e.g., robot.lift), or 'all' to select everything:")
        try:
            selection = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting...")
            sys.exit(0)
        
        if selection == 'all' or selection == '':
            return ['all']
            
        selected_fields = []
        parts_entered = [x.strip() for x in selection.split(',')]
        
        invalid_parts = []
        for part in parts_entered:
            if not part:
                continue
            if part in index_to_prefix:
                selected_fields.append(index_to_prefix[part])
            else:
                match = False
                for p in paths:
                    if part == p or p.startswith(part + '.'):
                        match = True
                        break
                if match:
                    selected_fields.append(part)
                else:
                    invalid_parts.append(part)
                    
        if invalid_parts:
            print(f"\n[!] Error: The following fields or indices do not exist: {', '.join(invalid_parts)}")
            print("Please try again.")
            continue
            
        if not selected_fields:
            return ['all']
            
        return selected_fields

def main():
    parser = argparse.ArgumentParser(description="Visualize Stretch status in Rerun.")
    parser.add_argument(
        "--history",
        type=float,
        default=None,
        help="Read from offline logs. Specify how many minutes ago to show.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=50,
        help="Rate at which to pull status in live mode. Defaults to 50 Hz.",
    )
    parser.add_argument(
        "--fields",
        nargs="+",
        default=None,
        help="List of field prefixes to plot (e.g. robot.power_periph.voltage robot.lift)",
    )
    parser.add_argument(
        "--print",
        action="store_true",
        help="Print the status in a pretty format to the console.",
    )
    parser.add_argument(
        "--export",
        type=str,
        default=None,
        help="Export history to a zip file in the specified directory. Requires --history.",
    )
    parser.add_argument(
        "--import",
        dest="import_file",
        type=str,
        default=None,
        help="Import and replay a zip file of exported history.",
    )
    args = parser.parse_args()

    def _start_rerun():
        rr.init("stretch_status_viz", spawn=False)
        rr.spawn(memory_limit="5GB")

    selected_fields = args.fields
    menu_shown = False

    if args.import_file is not None:
        import zipfile
        zip_path = os.path.expanduser(args.import_file)
        if not os.path.exists(zip_path):
            print(f"Error: Import file {zip_path} does not exist.")
            return

        print(f"Importing history from {zip_path}...")
        with zipfile.ZipFile(zip_path, 'r') as zf:
            file_names = sorted([n for n in zf.namelist() if n.endswith('.json')])
            for f_name in file_names:
                with zf.open(f_name) as file:
                    try:
                        batch = json.loads(file.read().decode('utf-8'))
                        for rs in batch:
                            if not menu_shown:
                                if selected_fields:
                                    validate_selected_fields_or_exit(rs, selected_fields)
                                else:
                                    selected_fields = build_interactive_menu(rs)

                                if not args.print:
                                    _start_rerun()
                                    setup_rerun_blueprint(rs, selected_fields)

                                menu_shown = True
                                
                            t = rs.get("timestamp", time.time())
                            rr.set_time("log_time", timestamp=t)
                            if args.print:
                                print("\n=== Status ===")
                                filtered_rs = filter_dict_by_fields(rs, selected_fields)
                                print_status_pretty(filtered_rs)

                            flat_status = flatten_status(rs)
                            log_selected_fields(flat_status, selected_fields)
                    except Exception as e:
                        print(f"Error reading {f_name} from zip: {e}")
        print("Finished reading imported history.")
    elif args.history is not None:
        fleet_path = os.getenv("HELLO_FLEET_PATH", os.path.expanduser("~"))
        log_dir = os.path.join(fleet_path, "log", "stretch_status")

        if not os.path.exists(log_dir):
            print(f"Log directory {log_dir} does not exist.")
            return

        start_time = time.time() - (args.history * 60)

        files = [
            os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".json")
        ]
        files.sort(key=os.path.getmtime)

        if files:
            oldest_file_mtime = os.path.getmtime(files[0])
            available_minutes = max(0.0, (time.time() - oldest_file_mtime) / 60.0)
            print(f"Maximum available history: {available_minutes:.1f} minutes.")
            if args.history > available_minutes:
                print(f"[!] Warning: You requested {args.history} minutes of history, but only {available_minutes:.1f} minutes are available.")

        if args.export:
            import zipfile
            from datetime import datetime
            export_dir = os.path.expanduser(args.export)
            if not os.path.isdir(export_dir):
                print(f"Error: Export directory {export_dir} does not exist.")
                return
            
            iso_time = datetime.now().isoformat().replace(':', '-')
            zip_filename = f"stretch_status_{iso_time}.zip"
            zip_path = os.path.join(export_dir, zip_filename)
            
            print(f"Exporting data to {zip_path}...")
            with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
                for f in files:
                    mtime = os.path.getmtime(f)
                    if mtime < start_time - 60:
                        continue
                    zf.write(f, os.path.basename(f))
            
            size_mb = os.path.getsize(zip_path) / (1024 * 1024)
            print(f"Export complete: {zip_path} ({size_mb:.2f} MB)")
            return

        for f in files:
            mtime = os.path.getmtime(f)
            if mtime < start_time - 60:
                continue

            with open(f, "r") as file:
                try:
                    batch = json.load(file)
                    for rs in batch:
                        if rs.get("timestamp", 0) >= start_time:
                            if not menu_shown:
                                if selected_fields:
                                    validate_selected_fields_or_exit(rs, selected_fields)
                                else:
                                    selected_fields = build_interactive_menu(rs)

                                if not args.print:
                                    _start_rerun()
                                    setup_rerun_blueprint(rs, selected_fields)

                                menu_shown = True
                                
                            t = rs.get("timestamp", time.time())
                            rr.set_time("log_time", timestamp=t)
                            if args.print:
                                print("\n=== Status ===")
                                filtered_rs = filter_dict_by_fields(rs, selected_fields)
                                print_status_pretty(filtered_rs)

                            flat_status = flatten_status(rs)
                            log_selected_fields(flat_status, selected_fields)
                except Exception as e:
                    print(f"Error reading {f}: {e}")

        print("Finished reading history.")
    else:
        # Live mode
        print("Starting live mode...")
        r = RobotClient()
        if not r.startup():
            print("Failed to start RobotClient")
            return

        # rate_hz = r.robot_params.get("sentry_status_logger", {}).get("check_rate", 10)
        rate_hz = args.rate
        sleep_time = 1.0 / rate_hz
        print(f"Pulling status at {rate_hz} Hz...")

        try:
            while True:
                r.pull_status()
                rs = r.status.copy()
                if "timestamp" not in rs:
                    rs['timestamp'] = time.time()
                
                if not menu_shown:
                    if selected_fields:
                        validate_selected_fields_or_exit(rs, selected_fields)
                    else:
                        selected_fields = build_interactive_menu(rs)
                    
                    if not args.print:
                        _start_rerun()
                        setup_rerun_blueprint(rs, selected_fields)
                        
                    menu_shown = True


                if args.print:
                    print("\n=== Status ===")
                    filtered_rs = filter_dict_by_fields(rs, selected_fields)
                    print_status_pretty(filtered_rs)
                else:
                    t = rs.get("timestamp", time.time())
                    rr.set_time("log_time", timestamp=t)

                flat_status = flatten_status(rs)
                log_selected_fields(flat_status, selected_fields)
                
                time.sleep(sleep_time)
        except KeyboardInterrupt:
            r.stop()
            print("Stopped live mode.")

if __name__ == "__main__":
    main()
