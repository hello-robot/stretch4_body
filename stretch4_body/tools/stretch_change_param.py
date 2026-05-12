#!/usr/bin/env python3

from stretch4_body.core.hello_utils import print_stretch_re_use
import os
import sys
import yaml
import click
import ast
import importlib
from stretch4_body.core.robot_params import RobotParams
import stretch4_body.core.hello_utils as hello_utils

def get_fleet_directory():
    try:
        return os.environ['HELLO_FLEET_PATH'] + '/' + os.environ['HELLO_FLEET_ID'] + '/'
    except KeyError as e:
        print(f"Environment variable {e} not set.")
        sys.exit(1)

def write_fleet_yaml(fn, rp, fleet_dir=None, header=None):
    if fleet_dir is None:
        fleet_dir = get_fleet_directory()
    if fleet_dir[-1] != '/':
        fleet_dir += '/'
    with open(fleet_dir + fn, 'w') as yaml_file:
        if header is not None:
            yaml_file.write(header)
        yaml.dump(rp, yaml_file, default_flow_style=False)

def set_nested_dict(d, path, value):
    """Sets a value in a nested dictionary given a key path."""
    for key in path[:-1]:
        if key not in d or not isinstance(d[key], dict):
            d[key] = {}
        d = d[key]
    d[path[-1]] = value

def get_dynamic_options(path, current_val, full_dict):
    """Dynamically discover available options for a given parameter."""
    import os
    import glob
    path_str = '.'.join(path)
    options = []
    
    if path_str == 'robot.tool':
        options = full_dict.get('supported_eoa', [])
        # Also include any keys starting with 'eoa_' if missing
        for key in full_dict.keys():
            if key.startswith('eoa_') and key not in options:
                options.append(key)
                
    if path_str == 'robot.model_name':
        # Scan for robot_params_*.py files to discover models
        try:
            import stretch4_body.core.hello_utils as hello_utils
            src_dir = os.path.dirname(os.path.dirname(hello_utils.__file__))
            robot_dir = os.path.join(src_dir, 'robot')
            files = glob.glob(os.path.join(robot_dir, 'robot_params_*.py'))
            for f in files:
                basename = os.path.basename(f)
                if basename.startswith('robot_params_') and basename.endswith('.py'):
                    model = basename[13:-3]
                    if model not in options:
                        options.append(model)
        except Exception:
            pass
            
    if path_str == 'omnibase.forward_dir':
        options = ['calder', 'basquiat', 'basquiat+']
    
    # Automatically provide boolean options
    if isinstance(current_val, bool) and not options:
        options = [True, False]
        
    # Generic implicit lookups via predefined dictionary naming conventions
    implicit_keys = ['supported_' + path[-1], 'supported_' + path[-1] + 's']
    for test_key in implicit_keys:
        if test_key in full_dict and isinstance(full_dict[test_key], list):
            for o in full_dict[test_key]:
                if o not in options:
                    options.append(o)
                
    return options

def traverse_params(current_path, current_dict, full_dict):
    """Interactive loop to traverse the dictionary structure."""
    while True:
        keys = sorted(list(current_dict.keys()))
        
        if not current_path:
            allowed_keys = ['robot', 'cameras', 'stretch_gamepad']
            keys = [k for k in keys if k in allowed_keys or k.startswith('sentry_') or k.startswith('routine_')]

        print("\n" + "="*50)
        curr_p_str = '/'.join(current_path) if current_path else 'Root'
        print(f"Current Parameter Path: {curr_p_str}")
        print("="*50)
        
        for i, option in enumerate(keys):
            val = current_dict[option]
            if isinstance(val, dict):
                print(f"  {i}) {option}/")
            else:
                v_str = str(val)
                if len(v_str) > 50:
                    v_str = v_str[:47] + '...'
                print(f"  {i}) {option} = {v_str}")
        
        offset = len(keys)
        print(f"  {offset}) other (type your own)")
        print(f"  {offset + 1}) back to parent (..)")
        print(f"  {offset + 2}) quit without saving")
        
        choice = click.prompt(f"\nSelect an option [0-{offset+2}]", type=str)
        try:
            ch_idx = int(choice)
        except ValueError:
            if choice == '..':
                ch_idx = offset + 1
            elif choice == 'q':
                ch_idx = offset + 2
            else:
                print("Invalid choice.")
                continue

        if ch_idx == offset + 2:
            print("Quitting without saving.")
            sys.exit(0)
        elif ch_idx == offset + 1:
            return None # back
        elif ch_idx == offset:
            # Add an 'other' field dynamically
            new_key = click.prompt("Enter new parameter key", type=str)
            if new_key == "":
                continue
            is_dict = click.prompt("Is this a nested dictionary?", type=bool, default=False)
            if is_dict:
                if new_key not in current_dict or not isinstance(current_dict[new_key], dict):
                    current_dict[new_key] = {}
                result = traverse_params(current_path + [new_key], current_dict[new_key], full_dict)
                if result is not None:
                    return result
            else:
                new_val_str = click.prompt("Enter value (will be evaluated as python type, or string if eval fails)", type=str)
                try:
                    new_val = ast.literal_eval(new_val_str)
                except Exception:
                    new_val = new_val_str
                return current_path + [new_key], new_val
        elif 0 <= ch_idx < len(keys):
            selected_key = keys[ch_idx]
            val = current_dict[selected_key]
            
            if isinstance(val, dict):
                result = traverse_params(current_path + [selected_key], val, full_dict)
                if result is not None:
                    return result
            else:
                print(f"\nCurrent value for {selected_key}: {val}  (type: {type(val).__name__})")
                options = get_dynamic_options(current_path + [selected_key], val, full_dict)
                
                if options:
                    print("\nAvailable Options:")
                    for idx, opt in enumerate(options):
                        curr_marker = " (Current)" if opt == val else ""
                        print(f"  {idx}) {opt}{curr_marker}")
                    
                    print(f"  {len(options)}) other (type your own)")
                    
                    opt_choice = click.prompt(f"\nSelect an option [0-{len(options)}]", type=int)
                    if 0 <= opt_choice < len(options):
                        new_val = options[opt_choice]
                    else:
                        new_val_str = click.prompt("Enter new value (evaluated as python type, or string if eval fails)", type=str)
                        try:
                            new_val = ast.literal_eval(new_val_str)
                        except Exception:
                            new_val = new_val_str
                else:
                    new_val_str = click.prompt("Enter new value (evaluated as python type, or string if eval fails)", type=str)
                    try:
                        new_val = ast.literal_eval(new_val_str)
                    except Exception:
                        new_val = new_val_str
                return current_path + [selected_key], new_val
        else:
            print("Invalid choice.")


@click.command()
@click.option('--diff', is_flag=True, help='Show the user overrides from stretch_user_params.yaml')
@click.option('--factory-reset', is_flag=True, help='Revert all user parameters to factory defaults')
def main(diff, factory_reset):
    print_stretch_re_use()
    print("""
==================================================

Choose a number to configure Stretch System Parameters

WARNING: Please be careful when changing parameters as they may affect the behavior and safety of the robot. 

If you have any questions, please contact support@hello-robot.com.

==================================================
""")
    user_params_fn = 'stretch_user_params.yaml'
    fleet_dir = get_fleet_directory()

    if diff:
        user_params_on_disk = hello_utils.read_fleet_yaml(user_params_fn)
        if not user_params_on_disk:
            print("No user parameters overrides found or file is empty.")
        else:
            print(f"--- User Overrides in {user_params_fn} ---")
            print(yaml.dump(user_params_on_disk, default_flow_style=False))
        sys.exit(0)

    # We read using hello_utils.read_fleet_yaml to avoid getting our config overridden
    # Wait, getting the combined dictionary gives the user the ability to browse all params, not just the ones they have overwritten.
    # We will use RobotParams to get the full resolved parameters dictionary for exploration.
    if not RobotParams.are_params_valid():
        print("Parameters are invalid, exiting.")
        sys.exit(1)
        
    _user_params, _robot_params = RobotParams.get_params()

    # Determine Robot Model for header
    model_name = _robot_params.get('robot', {}).get('model_name', 'SE4UNH')
    param_module_name = 'stretch4_body.robot.robot_params_' + model_name

    if factory_reset:
        confirm = click.confirm("Are you sure you want to revert all parameters to factory defaults?", default=False)
        if confirm:
            try:
                user_params_header = getattr(importlib.import_module(param_module_name), 'user_params_header', '')
            except Exception:
                user_params_header = None
            write_fleet_yaml(user_params_fn, {}, fleet_dir, user_params_header)
            print(f"Factory reset complete. Cleared {fleet_dir}{user_params_fn}")
        else:
            print("Factory reset aborted.")
        sys.exit(0)

    result = traverse_params([], dict(_robot_params), _robot_params)
    
    if result is None:
        print("Quitting without saving.")
        sys.exit(0)
    
    path, new_value = result
    path_str = '.'.join(path)
    print(f"\nSetting parameter {path_str} to: {new_value} (type: {type(new_value).__name__})")

    # Ensure the user param dictionary is properly loaded so we preserve existing values
    user_params_on_disk = hello_utils.read_fleet_yaml(user_params_fn)
    
    set_nested_dict(user_params_on_disk, path, new_value)

    user_params_header = getattr(importlib.import_module(param_module_name), 'user_params_header', '')

    write_fleet_yaml(user_params_fn, user_params_on_disk, fleet_dir, user_params_header)
    print(f"Saved to {fleet_dir}{user_params_fn}")
    print("Done! You may need to restart any running code for changes to take effect.")

if __name__ == '__main__':
    main()
