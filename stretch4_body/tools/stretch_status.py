#!/usr/bin/env python3
import json
import os
import time
import argparse
import sys
import rerun as rr
from colorama import Fore, Back, Style, init

init(autoreset=True)

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

    def toggle_recursive(path, node, state):
        if state:
            selected_fields.add(path)
        else:
            if path in selected_fields:
                selected_fields.remove(path)
            # Also need to handle cases where a parent was selected
            # but we want to deselect this child only? 
            # The prompt says "Recursively select all fields underneath".
            # Usually this means if you select a group, everything under it is on.
            # If you deselect a group, everything under it is off.
            
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

        for i, item in enumerate(items):
            idx = i + 1
            item_path = f"{curr_path_str}.{item}"
            is_group = len(curr_node[item]) > 0
            
            check = f"{Fore.GREEN}[x]{Style.RESET_ALL}" if is_selected(item_path) else "[ ]"
            type_indicator = f"{Fore.BLUE}> {Style.RESET_ALL}" if is_group else "  "
            
            color = Fore.WHITE if not is_group else Fore.CYAN
            print(f" {idx:2d}: {check} {type_indicator}{color}{item}{Style.RESET_ALL}")
            index_to_item[idx] = item

        print(f"\n{Fore.YELLOW}Commands:{Style.RESET_ALL}")
        print(f"  {Fore.WHITE}0{Style.RESET_ALL}         Go back")
        print(f"  {Fore.WHITE}[index]{Style.RESET_ALL}  Toggle field/Enter group")
        print(f"  {Fore.WHITE}t [index]{Style.RESET_ALL} Toggle group selection without entering")
        print(f"  {Fore.WHITE}f{Style.RESET_ALL}         Finish and start playback")
        print(f"  {Fore.WHITE}q{Style.RESET_ALL}         Quit")
        print(f"  {Fore.YELLOW}Press Enter after each command{Style.RESET_ALL}")
        
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
    parser = argparse.ArgumentParser(description="Visualize Stretch status.")
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
