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

SERVER_START_FAILED_MSG = '''Could not start the stretch_body_server. Start it manually with:

    stretch_body_server --restart
'''

HOME_HINT = '''Home the tool before using it with:

    stretch_robot_home or stretch_dex_wrist_home
'''

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

def get_tool_name(tool, supported_eoa_metadata):
    return supported_eoa_metadata.get(tool, {}).get('name', tool)

def exit_without_saving(server_was_running, code=0):
    """Leave the tool setting as it was, restarting the server if this script stopped it."""
    print('\nThe tool setting was not changed.')
    if server_was_running:
        from stretch4_body.utils.server_power import start_server
        if not start_server():
            print(SERVER_START_FAILED_MSG)
    sys.exit(code)

def detect_tool(robot_params, module, supported_eoa):
    """Return the supported tool whose servo IDs best match the servos found on the end-of-arm bus, or None."""
    print('Detecting the attached tool...')
    eoa_usb = robot_params.get('end_of_arm', {}).get('usb_name', '/dev/hello-feetech-wrist')

    # Suppress the spammy output from list_servos
    with contextlib.redirect_stdout(None):
        servos = FeetechSMServo.list_servos(eoa_usb, baudrate=1000000)

    servo_ids = set([s['id'] for s in servos])

    # Dynamically determine tool IDs from robot params
    import copy
    tool_to_ids = {}
    for tool_name in supported_eoa:
        tool_config = robot_params.get(tool_name)
        if tool_config and 'devices' in tool_config:
            tool_ids = []
            for dev_name, dev_info in tool_config['devices'].items():
                dev_params = {}
                dev_params_name = dev_info.get('device_params')
                if dev_params_name:
                    dev_params = getattr(module, dev_params_name, {})

                resolved_dev_params = copy.deepcopy(dev_params)
                resolved_dev_params.update(dev_info)

                if 'id' in resolved_dev_params:
                    tool_ids.append(resolved_dev_params['id'])
            tool_to_ids[tool_name] = set(tool_ids)

    # Find the best matching tool (the one with all IDs present and most IDs)
    best_match = None
    max_ids = -1
    for tool_name, tool_ids in tool_to_ids.items():
        if tool_ids and tool_ids.issubset(servo_ids):
            if len(tool_ids) > max_ids:
                max_ids = len(tool_ids)
                best_match = tool_name
    return best_match

def choose_tool(supported_eoa, supported_eoa_metadata, current_tool, detected_tool):
    """Show the supported tools and ask which one is attached. Returns the tool, or None to quit."""
    if current_tool:
        print(f"\nCurrent tool: {get_tool_name(current_tool, supported_eoa_metadata)} ({current_tool})")
    print("\nWhich tool is attached now?")

    default_choice = 0
    for i, tool in enumerate(supported_eoa):
        is_current = (tool == current_tool)
        is_detected = (tool == detected_tool)
        if is_detected:
            default_choice = i
        elif is_current and detected_tool is None:
            default_choice = i

        tags = [t for t, on in (('current', is_current), ('detected', is_detected)) if on]
        tag_str = f" ({', '.join(tags)})" if tags else ''
        color = Fore.GREEN if is_current else Fore.YELLOW if is_detected else ''
        print(f"  {color}{i}) {get_tool_name(tool, supported_eoa_metadata)} [{tool}]{tag_str}{Style.RESET_ALL}")
        description = supported_eoa_metadata.get(tool, {}).get('description')
        if description:
            print(f"     {description}")

    custom_choice = len(supported_eoa)
    quit_choice = len(supported_eoa) + 1
    print(f"  {custom_choice}) Other (enter a tool name)")
    print(f"  {quit_choice}) Quit without changing the tool")

    choice = click.prompt("\nEnter a number", type=click.IntRange(0, quit_choice), default=default_choice)
    if choice == quit_choice:
        return None
    if choice == custom_choice:
        return click.prompt("Tool name", type=str)
    return supported_eoa[choice]

@click.command()
@click.option('--quick', '-q', is_flag=True, help='Skip interactive steps.')
@click.option('--auto-detect', is_flag=True, help='Automatically detect tool and set it.')
def main(quick, auto_detect):
    import stretch4_body.core.hello_utils as hu
    hu.print_stretch_re_use()

    if quick:
        print('This script sets which end-of-arm tool is attached to the robot.')
    elif auto_detect:
        print('This script detects the attached end-of-arm tool and saves it to the robot configuration.')
    else:
        print('This script walks you through swapping the end-of-arm tool:')
        print('  1. Stop the stretch_body_server, if it is running')
        print('  2. Power off the tool so you can swap it')
        print('  3. Detect and confirm the new tool')
        print('  4. Restart the server and home the new tool')
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

    from stretch4_body.core.robot_params import RobotParams

    if not RobotParams.are_params_valid():
        print('Please verify if Stretch configuration YAML files are present before continuing.')
        sys.exit(1)

    _user_params, _robot_params = RobotParams.get_params()
    _config_params = read_fleet_yaml(config_params_fn, fleet_dir)

    # Get the name of the robot model
    if 'robot' in _user_params and 'model_name' in _user_params['robot']:
        model_name = _user_params['robot']['model_name']
    elif 'robot' in _config_params and 'model_name' in _config_params['robot']:
        model_name = _config_params['robot']['model_name']
    else:
        model_name = _robot_params.get('robot', {}).get('model_name', 'unknown')

    param_module_name = 'stretch4_body.robot.robot_params_' + model_name
    try:
        module = importlib.import_module(param_module_name)
    except Exception as e:
        print(f"ERROR: Could not load parameters for model {model_name} from {param_module_name}")
        print(e)
        sys.exit(1)

    supported_eoa = _robot_params.get('supported_eoa', [])
    supported_eoa_metadata = _robot_params.get('supported_eoa_metadata', {})

    current_tool = None
    if 'robot' in _user_params and 'tool' in _user_params['robot']:
        current_tool = _user_params['robot']['tool']
    elif 'robot' in _config_params and 'tool' in _config_params['robot']:
        current_tool = _config_params['robot']['tool']
    elif 'tool' in _robot_params.get('robot', {}):
        current_tool = _robot_params['robot']['tool']

    detected_tool = None
    server_was_running = False
    if not quick:
        # Tool power is changed over the direct API with the server stopped. The
        # server holds the eoa transport open, so cutting power underneath it
        # leaves the servos unable to re-enumerate.
        from stretch4_body.utils.server_power import stop_server_for_power_change

        ok, server_was_running = stop_server_for_power_change(reason='swap the tool')
        if not ok:
            return print('\nThe tool setting was not changed.')

        # Hold the lift in place so it doesn't droop under the new tool's weight
        # before the server restarts and applies the new tool's feedforward.
        from stretch4_body.utils.lift_clearance import hold_lift_stiff
        if not hold_lift_stiff():
            print('WARNING: Could not hold the lift in place. Support the arm while swapping the tool, as the lift may droop.')

        try:
            from stretch4_body.subsystem.power_periph import PowerPeriph

            p = PowerPeriph()
            if not p.startup():
                print("Could not connect to the robot's power board. Run stretch_system_check to diagnose.")
                exit_without_saving(server_was_running, code=1)

            try:
                if not auto_detect and click.confirm('\nPower off the current tool so you can swap it? (Answer no if the new tool is already attached)', default=True):
                    print('Powering off the tool...')
                    p.actuator_control('eoa', enable=False)
                    try:
                        click.pause('Swap in the new tool, then press any key to power it on...')
                    finally:
                        # Power the tool back on even when aborting, so the server can find it
                        print('Powering on the tool...')
                        p.actuator_control('eoa', enable=True)
                        time.sleep(2.0) # Wait for motors to boot

                detected_tool = detect_tool(_robot_params, module, supported_eoa)
                if detected_tool:
                    print(f'Detected: {get_tool_name(detected_tool, supported_eoa_metadata)} ({detected_tool})')
                else:
                    print('Could not identify the attached tool.')
            finally:
                p.stop()
        except (KeyboardInterrupt, click.Abort):
            exit_without_saving(server_was_running)

    if auto_detect:
        if detected_tool is None:
            print('Could not detect a supported tool.')
            exit_without_saving(server_was_running, code=1)
        new_tool = detected_tool
    else:
        try:
            new_tool = choose_tool(supported_eoa, supported_eoa_metadata, current_tool, detected_tool)
        except (KeyboardInterrupt, click.Abort):
            new_tool = None
        if new_tool is None:
            exit_without_saving(server_was_running)

    if 'robot' not in _user_params:
        _user_params['robot'] = {}

    _user_params['robot']['tool'] = new_tool

    user_params_header = getattr(importlib.import_module(param_module_name), 'user_params_header', '')

    write_fleet_yaml(user_params_fn, _user_params, fleet_dir, user_params_header)
    print(f"\nTool set to {get_tool_name(new_tool, supported_eoa_metadata)} ({new_tool}). Saved to {fleet_dir}{user_params_fn}")

    # The server was stopped to change tool power, so bring it back whether or not the tool gets homed
    from stretch4_body.utils.server_power import start_server
    if server_was_running and not start_server():
        return print(SERVER_START_FAILED_MSG + '\n' + HOME_HINT)

    if quick or not click.confirm('\nHome the new tool now? The wrist will move.', default=True):
        return print('\n' + HOME_HINT)

    # Reload RobotParams in-place to ensure the new tool settings are loaded from disk
    RobotParams.reload()
    from stretch4_body.utils.lift_clearance import ensure_lift_clearance, hold_lift_stiff

    if server_was_running:
        from stretch4_body.robot.robot_client import RobotClient
    else:
        from stretch4_body.robot.robot import Robot as RobotClient
    r = RobotClient()
    connected = False
    for i in range(20): # try for 20 seconds
        if r.startup():
            connected = True
            break
        time.sleep(1.0)

    if not connected:
        if server_was_running:
            print("Could not connect to the stretch_body_server. Make sure it is running (stretch_body_server --restart).")
        else:
            print("Could not connect to the robot.")
        return print(HOME_HINT)

    time.sleep(2.0) # Extra wait for server to be fully ready
    success = False
    try:
        r.pull_status()
        while r.is_runstopped():
            if not click.confirm('The runstop is active. Release it, then continue homing?', default=True):
                break
            r.pull_status()
        has_lift = r.get_subsystem('lift') is not None
        if r.is_runstopped():
            print("Skipped homing because the runstop is active. Release the runstop first.")
        else:
            if has_lift and r.lift.is_homed():
                # Actively hold the lift, as its safety mode may not counterbalance the new tool yet
                r.lift.move_to(r.lift.status['pos'])
                r.push_command()
            if has_lift and not ensure_lift_clearance(r.lift, root=r, tool=new_tool):
                print("Skipped homing because the lift is too low for the tool to move safely. Raise the lift first.")
            else:
                print('Homing the tool...')
                success = r.end_of_arm.home()
                if not success:
                    print("Homing failed.")
    except (KeyboardInterrupt, click.Abort):
        print("\nHoming canceled.")
    except Exception as e:
        print(f"Error during homing: {e}")
    finally:
        r.stop()
        if not server_was_running:
            # Stopping the direct Robot drops the lift back to its default
            # safety mode, without the new tool's feedforward.
            hold_lift_stiff()

    if success:
        print("\nDone! The new tool is ready to use.")
    else:
        print('\n' + HOME_HINT)

if __name__ == '__main__':
    main()
