#!/usr/bin/env python3
import json
import os
import time
import argparse
import sys
import rerun as rr
from colorama import Fore, Back, Style, init
from collections import defaultdict
from datetime import datetime

init(autoreset=True)

def is_field_selected(path, selected_fields):
    if not selected_fields or 'all' in selected_fields:
        return True
    for sf in selected_fields:
        if sf == 'all':
            return True
        if path == sf or path.startswith(sf + '.'):
            return True
    return False

def get_file_timestamp(filepath):
    """
    Get the timestamp of a log file.
    Attempts to parse the timestamp from the filename (e.g., status_YYYYMMDD_HHMMSS.json)
    to handle files copied/transferred across systems where mtime is overwritten.
    Falls back to os.path.getmtime if parsing fails.
    """
    basename = os.path.basename(filepath)
    try:
        dt = datetime.strptime(basename, "status_%Y%m%d_%H%M%S.json")
        return dt.timestamp()
    except Exception:
        return os.path.getmtime(filepath)

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

def get_default_fields(sample_status):
    """
    Returns the default set of fields to visualize.
    """
    flat = flatten_status(sample_status, parent_key='robot')
    paths = sorted(list(flat.keys()))
    
    # Defaults: server, base x/y/theta, and non-motor joint positions
    defaults = ['robot.server', 'robot.base.x', 'robot.base.y', 'robot.base.theta']
    joint_positions = [
        k for k in paths 
        if (k.endswith('.pos') or k.endswith('.pos_pct')) 
        and not isinstance(flat[k], bool) 
        and 'motor' not in k 
        and 'in_collision_stop' not in k
    ]
    return defaults + joint_positions

def build_recursive_menu(sample_status):
    """
    Displays an interactive recursive menu grouped by subsystem.
    Returns a list of selected field prefixes.
    """
    flat = flatten_status(sample_status, parent_key='robot')
    all_paths = sorted(list(flat.keys()))
    
    # Build a tree structure for navigation
    tree = {}
    for path in all_paths:
        parts = path.split('.')
        curr = tree
        for part in parts:
            if part not in curr:
                curr[part] = {}
            curr = curr[part]

    selected_fields = set()
    current_path_parts = ['robot']
    show_groups = True
    
    def get_node(path_parts):
        curr = tree
        for p in path_parts:
            if p in curr:
                curr = curr[p]
            else:
                return None
        return curr

    def is_selected(path):
        if path in selected_fields:
            return True
        # If any parent is selected, it's implicitly selected (recursive)
        parts = path.split('.')
        for i in range(1, len(parts)):
            parent = '.'.join(parts[:i])
            if parent in selected_fields:
                return True
        return False

    def get_selection_info(path, node):
        """Returns (total_fields_under, selected_fields_under)"""
        total = 0
        selected = 0
        
        def count_recursive(p, n):
            nonlocal total, selected
            if not n: # Leaf
                total += 1
                if is_selected(p):
                    selected += 1
            else:
                for child_name, child_node in n.items():
                    count_recursive(f"{p}.{child_name}", child_node)
        
        count_recursive(path, node)
        return total, selected

    def toggle_recursive(path, node, state):
        if state:
            selected_fields.add(path)
        else:
            if path in selected_fields:
                selected_fields.remove(path)
            
        # Recursive toggle for children
        if node:
            for child_name, child_node in node.items():
                child_path = f"{path}.{child_name}"
                toggle_recursive(child_path, child_node, state)

    while True:
        os.system('clear' if os.name == 'posix' else 'cls')
        curr_path_str = '.'.join(current_path_parts)
        curr_node = get_node(current_path_parts)
        
        print(f"{Fore.CYAN}{Style.BRIGHT}=== Stretch Status Selection ==={Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Path: {Fore.WHITE}{Style.BRIGHT}{curr_path_str}{Style.RESET_ALL}")
        print(f"{Fore.GREEN}Selected: {len(selected_fields)} fields/groups{Style.RESET_ALL}\n")
        
        items = sorted(curr_node.keys())
        index_to_item = {}
        
        if len(current_path_parts) > 1:
            print(f"  {Fore.WHITE}0: .. (Go up){Style.RESET_ALL}")
            index_to_item[0] = '..'

        visible_idx = 1
        for item in items:
            item_path = f"{curr_path_str}.{item}"
            item_node = curr_node[item]
            is_group = len(item_node) > 0
            
            if not show_groups and is_group:
                continue
                
            idx = visible_idx
            visible_idx += 1
            
            if is_group:
                total, sel = get_selection_info(item_path, item_node)
                if sel == total:
                    check = f"{Fore.GREEN}[x]{Style.RESET_ALL}"
                elif sel > 0:
                    check = f"{Fore.YELLOW}[/]{Style.RESET_ALL}"
                else:
                    check = "[ ]"
                
                count_str = f" {Fore.BLACK}{Style.BRIGHT}({sel}/{total}){Style.RESET_ALL}" if sel > 0 else ""
                type_indicator = f"{Fore.BLUE}> {Style.RESET_ALL}"
                color = Fore.CYAN
                print(f" {idx:2d}: {check} {type_indicator}{color}{item}{count_str}{Style.RESET_ALL}")
            else:
                check = f"{Fore.GREEN}[x]{Style.RESET_ALL}" if is_selected(item_path) else "[ ]"
                print(f" {idx:2d}: {check}   {Fore.WHITE}{item}{Style.RESET_ALL}")
            
            index_to_item[idx] = item

        print(f"\n{Fore.YELLOW}Commands:{Style.RESET_ALL}")
        print(f"  {Fore.WHITE}0{Style.RESET_ALL}         Go back")
        print(f"  {Fore.WHITE}[index]{Style.RESET_ALL}  Toggle field/Enter group")
        print(f"  {Fore.WHITE}t [index]{Style.RESET_ALL} Toggle group selection without entering")
        print(f"  {Fore.WHITE}h{Style.RESET_ALL}         Toggle show/hide folders")
        print(f"  {Fore.WHITE}f{Style.RESET_ALL}         Finish and start visualization")
        print(f"  {Fore.WHITE}q{Style.RESET_ALL}         Quit")
        
        try:
            choice = input(f"\n{Fore.CYAN}> {Style.RESET_ALL}").strip().lower()
        except (EOFError, KeyboardInterrupt):
            sys.exit(0)
            
        if choice == 'f':
            if not selected_fields:
                return get_default_fields(sample_status)
            return list(selected_fields)
        if choice == 'q':
            sys.exit(0)
        if choice == 'h':
            show_groups = not show_groups
            continue
            
        if not choice:
            continue
            
        # Handle "t index"
        toggle_only = False
        if choice.startswith('t '):
            toggle_only = True
            choice = choice[2:].strip()
            
        try:
            val = int(choice)
            if val == 0 and 0 in index_to_item:
                current_path_parts.pop()
                continue
            
            if val in index_to_item:
                item = index_to_item[val]
                item_path = f"{curr_path_str}.{item}"
                item_node = curr_node[item]
                is_group = len(item_node) > 0
                
                if is_group and not toggle_only:
                    current_path_parts.append(item)
                else:
                    # Toggle selection
                    new_state = not is_selected(item_path)
                    toggle_recursive(item_path, item_node, new_state)
            else:
                print(f"{Fore.RED}Invalid index{Style.RESET_ALL}")
                time.sleep(1)
        except ValueError:
            print(f"{Fore.RED}Invalid input{Style.RESET_ALL}")
            time.sleep(1)

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


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Visualize Stretch status.\n\n"
            "This tool allows you to visualize, replay, and export robot telemetry data.\n"
            "To replay saved telemetry runs, use --import <zip_file>. You can also filter the replay window\n"
            "from the start and end of the import using --start_seconds_offset and --end_seconds_offset.\n"
            "If --rerun is specified with --import, the whole file is imported and dumped into Rerun."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter
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
        help="List of field prefixes (e.g. robot.power_periph.voltage robot.lift)",
    )
    parser.add_argument(
        "--rerun",
        action="store_true",
        help="Visualize the status in Rerun alongside console output.",
    )
    parser.add_argument(
        "--interactive", "-i",
        action="store_true",
        help="Choose fields to visualize via an interactive menu.",
    )
    parser.add_argument(
        "--export",
        type=str,
        default=None,
        help="Export all available telemetry history to a zip file in the specified directory.",
    )
    parser.add_argument(
        "--import",
        dest="import_file",
        type=str,
        default=None,
        help="Import and replay a zip file of exported history.",
    )
    parser.add_argument(
        "--start_seconds_offset",
        type=float,
        default=None,
        help="Start offset in seconds relative to the beginning of the imported file (only works with --import).",
    )
    parser.add_argument(
        "--end_seconds_offset",
        type=float,
        default=None,
        help="End offset in seconds relative to the end of the imported file (only works with --import).",
    )
    args = parser.parse_args()

    if (args.start_seconds_offset is not None or args.end_seconds_offset is not None) and args.import_file is None:
        parser.error("The --start_seconds_offset and --end_seconds_offset flags can only be used when --import is specified.")

    if args.export is not None and args.import_file is None:
        fleet_path = os.getenv("HELLO_FLEET_PATH", os.path.expanduser("~"))
        log_dir = os.path.join(fleet_path, "log", "stretch_status")
        if not os.path.exists(log_dir):
            print(f"Log directory {log_dir} does not exist.")
            return

        files = [
            os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.endswith(".json")
        ]
        files.sort(key=get_file_timestamp)

        if not files:
            print(f"No status log files found in {log_dir} to export.")
            return

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
                zf.write(f, os.path.basename(f))
        
        size_mb = os.path.getsize(zip_path) / (1024 * 1024)
        print(f"Export complete: {zip_path} ({size_mb:.2f} MB)")
        return

    def _start_rerun():
        rr.init("stretch_status", spawn=False)
        rr.spawn(memory_limit="5GB")

    selected_fields = args.fields
    menu_shown = False

    if args.import_file is not None:
        import zipfile
        zip_path = os.path.expanduser(args.import_file)
        if not os.path.exists(zip_path):
            print(f"Error: Import file {zip_path} does not exist.")
            return

        try:
            from tqdm import tqdm
        except ImportError:
            class tqdm:
                def __init__(self, iterable=None, total=None, desc="", unit="", **kwargs):
                    self.iterable = iterable
                    self.total = total or (len(iterable) if iterable else None)
                    self.desc = desc
                    self.n = 0

                def __iter__(self):
                    if self.iterable is not None:
                        for item in self.iterable:
                            yield item
                            self.n += 1
                    print()

        with zipfile.ZipFile(zip_path, 'r') as zf:
            file_names = sorted([n for n in zf.namelist() if n.endswith('.json')])
            if not file_names:
                print("Error: No status log files found in the import archive.")
                return

            min_ts = None
            max_ts = None

            # Find min_ts from the first file with data
            for f_name in file_names:
                try:
                    with zf.open(f_name) as file:
                        batch = json.loads(file.read().decode('utf-8'))
                        if batch and isinstance(batch, list):
                            for rs in batch:
                                min_ts = rs.get("timestamp")
                                if min_ts is not None:
                                    break
                            if min_ts is not None:
                                break
                except Exception:
                    pass

            # Find max_ts from the last file with data
            for f_name in reversed(file_names):
                try:
                    with zf.open(f_name) as file:
                        batch = json.loads(file.read().decode('utf-8'))
                        if batch and isinstance(batch, list):
                            for rs in reversed(batch):
                                max_ts = rs.get("timestamp")
                                if max_ts is not None:
                                    break
                            if max_ts is not None:
                                break
                except Exception:
                    pass

            start_time_ts = None
            end_time_ts = None
            if min_ts is not None and args.start_seconds_offset is not None:
                start_time_ts = min_ts + args.start_seconds_offset
            if max_ts is not None and args.end_seconds_offset is not None:
                end_time_ts = max_ts - args.end_seconds_offset

            if args.rerun:
                print("Warning: Rerun has a memory cap of 5GB. For very large imported logs, older frames may be evicted by Rerun's memory manager.")

            print(f"Importing history from {zip_path}...")

            times_by_field = defaultdict(list)
            values_by_field = defaultdict(list)

            imported_count = 0
            for f_name in tqdm(file_names, desc="Importing status files", unit="file"):
                with zf.open(f_name) as file:
                    try:
                        batch = json.loads(file.read().decode('utf-8'))
                        for rs in batch:
                            t = rs.get("timestamp", 0.0)
                            if start_time_ts is not None and t < start_time_ts:
                                continue
                            if end_time_ts is not None and t > end_time_ts:
                                continue

                            if not menu_shown:
                                if selected_fields:
                                    validate_selected_fields_or_exit(rs, selected_fields)
                                elif args.interactive:
                                    selected_fields = build_recursive_menu(rs)
                                elif args.rerun:
                                    selected_fields = ['all']
                                else:
                                    selected_fields = get_default_fields(rs)

                                if args.rerun:
                                    _start_rerun()
                                    setup_rerun_blueprint(rs, selected_fields)

                                menu_shown = True
                                
                            if args.rerun:
                                t = rs.get("timestamp", time.time())
                                flat_status = flatten_status(rs)
                                for path, value in flat_status.items():
                                    if is_field_selected(path, selected_fields):
                                        if isinstance(value, bool):
                                            val = 1.0 if value else 0.0
                                        elif isinstance(value, (int, float)):
                                            val = float(value)
                                        else:
                                            continue
                                        times_by_field[path].append(t)
                                        values_by_field[path].append(val)
                            else:
                                print("\n=== Status ===")
                                filtered_rs = filter_dict_by_fields(rs, selected_fields)
                                print_status_pretty(filtered_rs)

                            imported_count += 1
                    except Exception as e:
                        print(f"Error reading {f_name} from zip: {e}")

        if args.rerun and times_by_field:
            print(f"\nBulk-sending {len(times_by_field)} signals across {imported_count} frames to Rerun...")
            import numpy as np
            for path, vals in values_by_field.items():
                if not vals:
                    continue
                rr_path = path.replace('.', '/')
                ts = np.array(times_by_field[path], dtype=np.float64)
                vs = np.array(vals, dtype=np.float64)
                time_col = rr.TimeColumn('log_time', timestamp=ts)
                scalars_col = rr.Scalars.columns(scalars=vs)
                rr.send_columns(rr_path, indexes=[time_col], columns=scalars_col)
            print(f"Successfully bulk-logged {imported_count} status frames to Rerun.")

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
        files.sort(key=get_file_timestamp)

        if files:
            oldest_file_timestamp = get_file_timestamp(files[0])
            available_minutes = max(0.0, (time.time() - oldest_file_timestamp) / 60.0)
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
                    file_timestamp = get_file_timestamp(f)
                    if file_timestamp < start_time - 60:
                        continue
                    zf.write(f, os.path.basename(f))
            
            size_mb = os.path.getsize(zip_path) / (1024 * 1024)
            print(f"Export complete: {zip_path} ({size_mb:.2f} MB)")
            return

        for f in files:
            file_timestamp = get_file_timestamp(f)
            if file_timestamp < start_time - 60:
                continue

            with open(f, "r") as file:
                try:
                    batch = json.load(file)
                    for rs in batch:
                        if rs.get("timestamp", 0) >= start_time:
                            if not menu_shown:
                                if selected_fields:
                                    validate_selected_fields_or_exit(rs, selected_fields)
                                elif args.interactive:
                                    selected_fields = build_recursive_menu(rs)
                                else:
                                    selected_fields = get_default_fields(rs)

                                if args.rerun:
                                    _start_rerun()
                                    setup_rerun_blueprint(rs, selected_fields)

                                menu_shown = True
                                
                            print("\n=== Status ===")
                            filtered_rs = filter_dict_by_fields(rs, selected_fields)
                            print_status_pretty(filtered_rs)

                            if args.rerun:
                                t = rs.get("timestamp", time.time())
                                rr.set_time("log_time", timestamp=t)
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
                    elif args.interactive:
                        selected_fields = build_recursive_menu(rs)
                    else:
                        selected_fields = get_default_fields(rs)
                    
                    if args.rerun:
                        _start_rerun()
                        setup_rerun_blueprint(rs, selected_fields)
                        
                    menu_shown = True

                print("\n=== Status ===")
                filtered_rs = filter_dict_by_fields(rs, selected_fields)
                print_status_pretty(filtered_rs)
                
                if args.rerun:
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
