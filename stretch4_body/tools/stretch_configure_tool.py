#!/usr/bin/env python3

import os
import sys
import yaml
import click
import importlib
from colorama import Fore, Style

def get_fleet_directory():
    return os.environ['HELLO_FLEET_PATH'] + '/' + os.environ['HELLO_FLEET_ID'] + '/'

def check_file_exists(fn):
    return os.path.isfile(fn)

def read_fleet_yaml(f, fleet_dir=None):
    try:
        if fleet_dir is None:
            fleet_dir = get_fleet_directory()
        else:
            if fleet_dir[-1] != '/':
                fleet_dir = fleet_dir + '/'
        with open(fleet_dir + f, 'r') as s:
            p = yaml.load(s, Loader=yaml.FullLoader)
            return {} if p is None else p
    except IOError:
        return {}

def write_fleet_yaml(fn, rp, fleet_dir=None, header=None):
    if fleet_dir is None:
        fleet_dir = get_fleet_directory()
    if fleet_dir[-1] != '/':
        fleet_dir += '/'
    with open(fleet_dir + fn, 'w') as yaml_file:
        if header is not None:
            yaml_file.write(header)
        yaml.dump(rp, yaml_file, default_flow_style=False)

@click.command()
def main():
    print('--- Configuring End-Of-Arm Tool ---')
    user_params_fn = 'stretch_user_params.yaml'
    config_params_fn = 'stretch_configuration_params.yaml'
    try:
        fleet_dir = get_fleet_directory()
    except KeyError as e:
        print(f"Environment variable {e} not set.")
        sys.exit(1)
    
    if not check_file_exists(fleet_dir + user_params_fn) or not check_file_exists(fleet_dir + config_params_fn):
        print('Please verify if Stretch configuration YAML files are present before continuing.')
        sys.exit(1)

    _user_params = read_fleet_yaml(user_params_fn, fleet_dir)
    _config_params = read_fleet_yaml(config_params_fn, fleet_dir)

    #Get the name of the robot model
    if 'robot' in _user_params and 'model_name' in _user_params['robot']:
        model_name = _user_params['robot']['model_name']
    elif 'robot' in _config_params and 'model_name' in _config_params['robot']:
        model_name = _config_params['robot']['model_name']
    else:
        print("ERROR: Could not find 'robot.model_name' in stretch_configuration_params.yaml or stretch_user_params.yaml")
        sys.exit(1)

    print(f"Detected Robot Model: {model_name}")
    param_module_name = 'stretch4_body.robot.robot_params_' + model_name

    try:
        _nominal_params = getattr(importlib.import_module(param_module_name), 'nominal_params')
    except Exception as e:
        print(f"ERROR: Could not load parameters for model {model_name} from {param_module_name}")
        print(e)
        sys.exit(1)

    supported_eoa = _nominal_params.get('supported_eoa', [])
    supported_eoa_metadata = _nominal_params.get('supported_eoa_metadata', {})
    if not supported_eoa:
        print("WARNING: No 'supported_eoa' found in nominal parameters.")

    current_tool = None
    if 'robot' in _user_params and 'tool' in _user_params['robot']:
        current_tool = _user_params['robot']['tool']
    elif 'robot' in _config_params and 'tool' in _config_params['robot']:
        current_tool = _config_params['robot']['tool']
    elif 'tool' in _nominal_params.get('robot', {}):
        current_tool = _nominal_params['robot']['tool']

    print(f"Current End-Of-Arm Tool: {current_tool}")
    print("\nAvailable Tools:")
    for i, tool in enumerate(supported_eoa):
        print(f"""  {Fore.GREEN if tool == current_tool else ""}{i}) {supported_eoa_metadata[tool]['name']}: {tool} {"(current)" if tool == current_tool else ""}
      {supported_eoa_metadata[tool]['description']}{Style.RESET_ALL}""")

    print(f"  {len(supported_eoa)}) Enter a custom tool name")
    print(f"  {len(supported_eoa) + 1}) Quit without saving")

    choice = click.prompt(f"\nSelect a tool [0-{len(supported_eoa)+1}]", type=int)

    if choice == len(supported_eoa) + 1:
        print("Quitting without saving.")
        sys.exit(0)
    
    if choice == len(supported_eoa):
        new_tool = click.prompt("Enter custom tool name", type=str)
    elif 0 <= choice < len(supported_eoa):
        new_tool = supported_eoa[choice]
    else:
        print("Invalid choice. Quitting without saving.")
        sys.exit(1)

    print(f"\nSetting End-Of-Arm Tool to: {new_tool}")

    if 'robot' not in _user_params:
        _user_params['robot'] = {}
    
    _user_params['robot']['tool'] = new_tool

    user_params_header = getattr(importlib.import_module(param_module_name), 'user_params_header', '')

    write_fleet_yaml(user_params_fn, _user_params, fleet_dir, user_params_header)
    print(f"Saved to {fleet_dir}{user_params_fn}")
    print("""Done! You may need to home the robot or restart services for the tool to be recognized.

It is strongly recommended to run:

stretch_body_server --restart
stretch_robot_home
""")

if __name__ == '__main__':
    main()
