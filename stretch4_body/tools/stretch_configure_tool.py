#!/usr/bin/env python3

import os
import sys
import yaml
import click
import importlib
import time
import subprocess
import contextlib
from colorama import Fore, Style
from stretch4_body.core.feetech.feetech_SM_servo import FeetechSMServo

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
@click.option('--quick', '-q', is_flag=True, help='Skip interactive steps.')
@click.option('--auto-detect', is_flag=True, help='Automatically detect tool and set it.')
def main(quick, auto_detect):
    import stretch4_body.core.hello_utils as hu
    hu.print_stretch_re_use()
    
    print('This script will guide you through swapping your end-of-arm tool.')
    print('Steps:')
    print('1. Power off the current tool.')
    print('2. Swap the tool after power is off.')
    print('3. Select the new tool (auto-detection will highlight it).')
    print('4. Restart the server and home the new tool.')
    print('---------------------------------------------------------')

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

    # Get the name of the robot model
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
        module = importlib.import_module(param_module_name)
        _nominal_params = getattr(module, 'nominal_params')
    except Exception as e:
        print(f"ERROR: Could not load parameters for model {model_name} from {param_module_name}")
        print(e)
        sys.exit(1)

    supported_eoa = _nominal_params.get('supported_eoa', [])
    supported_eoa_metadata = _nominal_params.get('supported_eoa_metadata', {})

    direct = False
    detected_tool = None
    if not quick:
        try:
            from stretch4_body.robot.robot_client import PowerPeriphClient as PowerPeriph
            
            p = PowerPeriph()
            
            if not p.startup():
                # If the client can't connect, try without the client:
                direct = True

                from stretch4_body.subsystem.power_periph import PowerPeriph
            
                p = PowerPeriph()

                if not p.startup():
                    return print("Failed to connect to the robot's power management. Please run `stretch_system_check`.")

            if not auto_detect:
                if click.confirm('Turn off power to the peripheral?', default=True):
                    print('Powering off eoa...')
                    p.actuator_control('eoa', enable=False)
                    p.push_command()
                
                try:
                    click.pause('Connect the tool then press any key to continue...')
                except (KeyboardInterrupt, click.Abort):
                    print("\nAborting.")
                    p.stop()
                    sys.exit(0)

                print('Powering on eoa...')
                p.actuator_control('eoa', enable=True)
                p.push_command()
                time.sleep(2.0) # Wait for motors to boot

            # Auto-detect tool ID
            print('Scanning for tool on Feetech bus...')
            eoa_usb = _nominal_params.get('end_of_arm', {}).get('usb_name', '/dev/hello-feetech-wrist')
            
            # Suppress the spammy output from list_servos
            with contextlib.redirect_stdout(None):
                servos = FeetechSMServo.list_servos(eoa_usb, baudrate=1000000)
            
            servo_ids = set([s['id'] for s in servos])
            
            # Dynamically determine tool IDs from nominal params
            module = importlib.import_module(param_module_name)
            tool_to_ids = {}
            for tool_name in supported_eoa:
                tool_config = _nominal_params.get(tool_name)
                if tool_config and 'devices' in tool_config:
                    tool_ids = []
                    for dev_name, dev_info in tool_config['devices'].items():
                        dev_params_name = dev_info.get('device_params')
                        if dev_params_name:
                            dev_params = getattr(module, dev_params_name, {})
                            if 'id' in dev_params:
                                tool_ids.append(dev_params['id'])
                    tool_to_ids[tool_name] = set(tool_ids)
            
            # Find the best matching tool (the one with all IDs present and most IDs)
            best_match = None
            max_ids = -1
            for tool_name, tool_ids in tool_to_ids.items():
                if tool_ids and tool_ids.issubset(servo_ids):
                    if len(tool_ids) > max_ids:
                        max_ids = len(tool_ids)
                        best_match = tool_name
            
            detected_tool = best_match
            
            if detected_tool:
                print(f'Detected tool: {detected_tool} ({supported_eoa_metadata.get(detected_tool, {}).get("name", "Unknown")})')
            else:
                print('No tool detected or unknown tool.')
            
            p.stop()
        except KeyboardInterrupt:
            print("\nAborted.")
            sys.exit(0)

    if auto_detect:
        if detected_tool:
            new_tool = detected_tool
        else:
            print("Auto-detect failed to find a known tool. Quitting.")
            sys.exit(1)
    else:
        current_tool = None
        if 'robot' in _user_params and 'tool' in _user_params['robot']:
            current_tool = _user_params['robot']['tool']
        elif 'robot' in _config_params and 'tool' in _config_params['robot']:
            current_tool = _config_params['robot']['tool']
        elif 'tool' in _nominal_params.get('robot', {}):
            current_tool = _nominal_params['robot']['tool']

        print(f"Current End-Of-Arm Tool: {current_tool}")
        print("\nAvailable Tools:")
        
        default_choice = 0
        for i, tool in enumerate(supported_eoa):
            is_current = (tool == current_tool)
            is_detected = (tool == detected_tool)
            if is_detected:
                default_choice = i
            elif is_current and detected_tool is None:
                default_choice = i
                
            print(f"""  {Fore.GREEN if is_current else Fore.YELLOW if is_detected else ""}{i}) {supported_eoa_metadata[tool]['name']}: {tool} {"(current)" if is_current else ""} {"(detected)" if is_detected else ""}{Style.RESET_ALL}
      {supported_eoa_metadata[tool]['description']}""")

        print(f"  {len(supported_eoa)}) Enter a custom tool name")
        print(f"  {len(supported_eoa) + 1}) Quit without saving")

        choice = click.prompt(f"\nSelect a tool [0-{len(supported_eoa)+1}]", type=int, default=default_choice)

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

    is_do_home = False
    if direct:
        is_do_home = click.confirm("\nWould you like to home the end_of_arm?", default=True)
    else:
        is_do_home= click.confirm('\nWould you like to restart the stretch_body_server and home the end_of_arm?', default=True)

    if not quick and is_do_home:
        p_restart = None
        if not direct:
            print('Restarting stretch_body_server...')
            p_restart = subprocess.Popen(['stretch_body_server', '--restart'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            
            print('Waiting for stretch_body_server to come back online...')
        if direct:
            from stretch4_body.robot.robot import Robot as RobotClient
        else:
            from stretch4_body.robot.robot_client import RobotClient
        r = RobotClient()
        connected = False
        for i in range(20): # try for 20 seconds
            if r.startup():
                connected = True
                break
            time.sleep(1.0)
        
        if connected:
            time.sleep(2.0) # Extra wait for server to be fully ready
            print('Homing end_of_arm...')
            try:
                r.end_of_arm.home()
            except Exception as e:
                print(f"Error during homing: {e}")
            finally:
                r.stop()
            if p_restart is not None:
                p_restart.terminate()
            print("Done! You are ready to use the tool.")
        else:
            print("Failed to connect to robot server after restart. Please try homing manually.")
            if p_restart is not None:
                p_restart.terminate()
    else:
        print("""Done! You may need to home the robot or restart services for the tool to be recognized.

It is strongly recommended to run:

stretch_body_server --restart
stretch_robot_home
""")

if __name__ == '__main__':
    main()
