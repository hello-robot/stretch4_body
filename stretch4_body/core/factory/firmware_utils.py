#!/usr/bin/env python

import click
import os
from subprocess import Popen, PIPE
import stretch4_body.core.stepper
import stretch4_body.subsystem.power_periph
import stretch4_body.core.device
import time
import sys
import stretch4_body.core.device
import stretch4_body.core.hello_utils
import shlex


log_device = stretch4_body.core.device.Device(req_params=False)

def user_msg_log(msg, user_display=True, fg=None, bg=None, bold=False):
    if user_display:
        click.secho(str(msg), fg=fg, bg=bg, bold=bold)
    log_device.logger.debug(str(msg))

def check_ubuntu_version():
    res = Popen(shlex.split('cat /etc/lsb-release | grep DISTRIB_RELEASE'), shell=False, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read().strip(b'\n')
    return res == b'DISTRIB_RELEASE=24.04'

def check_arduino_cli_install(no_prompts=False):
    import shutil
    import subprocess
    
    # Ensure ~/.local/bin is in PATH since that's where arduino-cli is installed
    local_bin = os.path.expanduser("~/.local/bin")
    if local_bin not in os.environ.get('PATH', '').split(os.pathsep):
        os.environ['PATH'] = local_bin + os.pathsep + os.environ.get('PATH', '')

    if shutil.which('arduino-cli') is None:
        click.secho("arduino-cli not found. Installing to ~/.local/bin...", fg="cyan")
        install_cmd = "curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin/ sh"
        try:
            subprocess.run(install_cmd, shell=True, check=True)
            subprocess.run(["arduino-cli", "config", "init"], check=False)
        except subprocess.CalledProcessError as e:
            click.secho(f"Failed to install arduino-cli: {e}", fg="red")
            return False

    try:
        # Check if the platform is installed natively
        res = exec_process(['arduino-cli', 'core', 'list'], True).decode('utf-8')
        if 'hello-robot:samd' not in res:
            click.secho("Installing hello-robot Arduino board packages...", fg="cyan")
            #index_url = "https://github.com/hello-robot/stretch_firmware/releases/download/All_Release_Binaries/package_hello-robot_index.json"
            index_url = "https://github.com/hello-robot/stretch_firmware/releases/download/All_Release_Binaries/package_hello-robot_index.json,https://adafruit.github.io/arduino-board-index/package_adafruit_index.json,https://espressif.github.io/arduino-esp32/package_esp32_index.json"
            exec_process(['arduino-cli', 'core', 'update-index', '--additional-urls', index_url], False)
            exec_process(['arduino-cli', 'core', 'install', 'hello-robot:samd', '--additional-urls', index_url], False)
            exec_process(['arduino-cli', 'core', 'install', 'hello-robot:esp32', '--additional-urls', index_url], False)
        return True
    except Exception as e:
        click.secho(f"Failed to fetch or install arduino-cli board package index: {e}", fg="red")
        return False



def get_sketch_name(device_name):
    if device_name=='hello-motor-omni-0' or device_name=='hello-motor-omni-1' or device_name=='hello-motor-omni-2' or device_name=='hello-motor-arm' or device_name=='hello-motor-lift':
        return 'hello_stepper2'
    if device_name == 'hello-power-periph':
        return 'hello_pimu2'
    if device_name == 'hello-pixart-j3':
        return 'hello_pixart_j3'
    if device_name in ['hello-esp', 'hello-esp32']:
        return 'hello_esp'

def exec_process(cmdline, silent, input=None, **kwargs):
    """Execute a subprocess and returns the returncode, stdout buffer and stderr buffer.
       Optionally prints stdout and stderr while running."""
    try:
        sub = Popen(cmdline, stdin=PIPE, stdout=PIPE, stderr=PIPE,**kwargs)
        stdout, stderr = sub.communicate(input=input)
        returncode = sub.returncode
        if not silent:
            sys.stdout.write(stdout.decode('utf-8'))
            sys.stderr.write(stderr.decode('utf-8'))
    except OSError as e:
        if e.errno == 2:
            raise RuntimeError('"%s" is not present on this system' % cmdline[0])
        else:
            raise
    if returncode != 0:
        raise RuntimeError('Got return value %d while executing "%s", stderr output was:\n%s' % (
        returncode, " ".join(cmdline), stderr.rstrip(b"\n")))
    return stdout


def is_device_present(device_name):
    try:
        exec_process(['ls', '/dev/'+device_name], True)
        return True
    except RuntimeError as e:
        return False

def wait_on_device(device_name,timeout=10.0):
    #Wait for device to appear on bus for timeout seconds
    print('Waiting for device %s to return to bus.'%device_name)
    ts=time.time()
    itr=0
    while(time.time()-ts<timeout):
        if is_device_present(device_name):
            return True
        itr=itr+1
        if itr % 5 == 0:
            sys.stdout.write('.')
            sys.stdout.flush()
        time.sleep(0.1)
    return False

def get_port_name(device_name):
    try:
        port_name = Popen(shlex.split("ls -l /dev/" + device_name), shell=False, bufsize=64, stdin=PIPE, stdout=PIPE,close_fds=True).stdout.read().strip().split()[-1]
        if not type(port_name)==str:
            port_name=port_name.decode('utf-8')
        return port_name
    except IndexError:
        return None

def does_stepper_have_encoder_calibration_YAML(device_name):
    d=stretch4_body.core.device.Device(req_params=False)
    sn = d.robot_params[device_name]['serial_no']
    fn = 'calibration_steppers/' + device_name + '_' + sn + '.yaml'
    enc_data = stretch4_body.core.hello_utils.read_fleet_yaml(fn)
    return len(enc_data)!=0

def get_device_protocols(device_name):
    #return list like ['p0','p1']
    s=get_sketch_name(device_name)
    if s == 'hello_pimu2':
        import stretch4_body.subsystem.power_periph
        dd = stretch4_body.subsystem.power_periph.PowerPeriph(backend=0)
        return list(dd.supported_protocols.keys())
    elif s == 'hello_stepper2':
        import stretch4_body.core.stepper
        dd = stretch4_body.core.stepper.Stepper('/dev/'+device_name, backend=0)
        return list(dd.supported_protocols.keys())
    elif s == 'hello_pixart_j3':
        import stretch4_body.subsystem.pixart_j3
        dd = stretch4_body.subsystem.pixart_j3.PixartJ3()
        return list(dd.supported_protocols.keys())
    elif s == 'hello_esp':
        import stretch4_body.subsystem.esp
        dd = stretch4_body.subsystem.esp.ESP()
        return list(dd.supported_protocols.keys())
    return []

def print_tty_mapping():
    import stretch4_body.core.factory.hello_device_utils as hdu
    import click
    mapping = hdu.find_tty_devices()
    click.secho(('-' * 95), fg="yellow", bold=True)
    click.secho('%-20s | %-15s | %-25s | %-25s' % ('PORT', 'SERIAL', 'MODEL', 'VENDOR'), fg="cyan", bold=True)
    click.secho(('-' * 95), fg="yellow", bold=True)
    for port in sorted(mapping.keys()):
        info = mapping[port]
        print('%-20s | %-15s | %-25s | %-25s' % (
            port, 
            str(info.get('serial', 'None')), 
            str(info.get('model', 'None')), 
            str(info.get('vendor', 'None'))))
    click.secho(('-' * 95), fg="yellow", bold=True)
    print('')


def flash_firmware_update(device_name, version_str, port, verbose, dummy=False):
    import time
    import subprocess
    import sys
    import requests
    import click
    sketch_name = get_sketch_name(device_name)
    
    file_name = f"{version_str}.bin"
    file_name = file_name.replace('.v', '_v')
        
    url = f"https://github.com/hello-robot/stretch_firmware/releases/download/All_Release_Binaries/{file_name}"
    dest_path = f"/tmp/{file_name}"

    click.secho(f"Downloading {file_name}...", fg="cyan")
    
    if dummy:
        click.secho(f"Dummy mode enabled. Pretending to download {file_name} from {url} and flash it.", fg="yellow")
        return True

    try:
        response = requests.get(url)
        if response.status_code == 200:
            with open(dest_path, 'wb') as f:
                f.write(response.content)
            click.secho(f"Downloaded nicely to {dest_path}", fg="green")
        else:
            click.secho(f"Failed to download firmware from {url} (Status {response.status_code})", fg="red")
            return False
    except Exception as e:
        click.secho(f"Failed to download firmware: {e}", fg="red")
        return False

    # verify arduino cli setup
    if sketch_name != 'hello_esp':
        if not check_arduino_cli_install():
            click.secho("Arduino CLI not available.", fg="red")
            return False

        fqbn = f"hello-robot:samd:hello_robot_{sketch_name}"
        upload_command = ['arduino-cli', 'upload', '-p', port, '--fqbn', fqbn, '-i', dest_path]
        
        if verbose:
            upload_command.append('-v')
            click.secho(f"Flashing arduino firmware via: {' '.join(upload_command)}", fg="cyan")
        else:
            click.secho(f"Flashing {device_name} (this may take a moment)...", fg="cyan")
        
        time.sleep(1.0)

        try:
            if verbose:
                subprocess.run(upload_command, check=True)
            else:
                subprocess.run(upload_command, check=True, capture_output=True, text=True)
            click.secho(f"Burned Arduino Sketch:{sketch_name} Successfully to port:{port}.", fg="green")
            return True
        except subprocess.CalledProcessError as e:
            if not verbose:
                if e.stdout:
                    click.secho(e.stdout, fg="red")
                if e.stderr:
                    click.secho(e.stderr, fg="red")
            click.secho(f"Failed to burn Arduino Sketch:{sketch_name} to port:{port}.", fg="red")
            return False
    else:
        import glob
        import os

        #Force ESP32 into bootloader as is more reliable to flash this way
        import stretch4_body.subsystem.power_periph
        dd = stretch4_body.subsystem.power_periph.PowerPeriph(backend=0)
        dd.startup()
        dd.set_esp_fw_update()
        dd.push_command()
        dd.stop()

        
        esptools = glob.glob(os.path.expanduser("~/.arduino15/packages/esp32/tools/esptool_py/*/esptool"))
        if not esptools:
            click.secho("esptool not found! Please run 'arduino-cli core install esp32:esp32' first.", fg="red")
            return False
        esptool_bin = esptools[-1]
        upload_command = [esptool_bin, '--chip', 'esp32s3', '--port', port, '--baud', '921600',
                          '--before', 'default_reset', '--after', 'hard_reset', 'write_flash', '-z', 
                          '--flash_mode', 'dio', '--flash_freq', '80m', '--flash_size', 'detect',
                          '0x10000', dest_path]
        
        if verbose:
            click.secho(f"Flashing ESP32 firmware via: {' '.join(upload_command)}", fg="cyan")
        else:
            click.secho(f"Flashing {device_name} (this may take a moment)...", fg="cyan")
        
        time.sleep(1.0)

        try:
            if verbose:
                subprocess.run(upload_command, check=True)
            else:
                subprocess.run(upload_command, check=True, capture_output=True, text=True)
            click.secho(f"Burned Arduino Sketch:{sketch_name} Successfully to port:{port}.", fg="green")
            return True
        except subprocess.CalledProcessError as e:
            if not verbose:
                if e.stdout:
                    click.secho(e.stdout, fg="red")
                if e.stderr:
                    click.secho(e.stderr, fg="red")
            click.secho(f"Failed to burn Arduino Sketch:{sketch_name} to port:{port}.", fg="red")
            return False
