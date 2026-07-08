#!/usr/bin/env python3
"""
stretch_discover_device.py

Standalone utility to discover Hello Robot device serial numbers and push
the corresponding udev symlink rules, one component at a time.

Supported flags:
  --lift          Discover hello-motor-lift by manual movement
  --arm           Discover hello-motor-arm by manual movement
  --omni-0        Discover hello-motor-omni-0 by manual movement
  --omni-1        Discover hello-motor-omni-1 by manual movement
  --omni-2        Discover hello-motor-omni-2 by manual movement
  --power-periph  Discover hello-power-periph (auto, by model ID)
  --pixart        Discover hello-pixart-j3 (auto, by model ID)
  --feetech       Discover hello-feetech-wrist (auto, by servo ID scan)
  --esp32         Discover hello-esp32 (auto, by vendor)
  --all           Discover all devices

Usage examples:
  ./stretch_discover_device.py --lift
  ./stretch_discover_device.py --arm --omni-0 --omni-1 --omni-2
  ./stretch_discover_device.py --power-periph --pixart
  ./stretch_discover_device.py --feetech
"""

import os
import re
import subprocess
import sys
import click

import stretch4_body.core.hello_utils as hu
import stretch4_body.core.stepper
from stretch4_body.core.device import Device
from stretch4_body.core.factory import hello_device_utils as hdu
from stretch4_body.core.feetech.feetech_SM_servo import FeetechCommError, FeetechSMServo

from stretch4_body.core.factory.hello_device_utils import find_tty_devices

# ─────────────────────────────────────────────────────────────────────────────
# Internal helpers
# ─────────────────────────────────────────────────────────────────────────────

def _get_stepper_devices(all_tty):
    """Return only ttyACM ports that are Hello_Stepper2 boards."""
    return {k: v for k, v in all_tty.items()
            if v.get('vendor') == 'Hello-Robot' and v.get('model') == 'Hello_Stepper2'}


def _get_all_stepper_poses(stepper_devices):
    """
    Start each stepper, read position, stop.
    Returns dict: {port: pos}
    """
    poses = {}
    generic_names = ['hello-motor-lift', 'hello-motor-arm',
                     'hello-motor-omni-0', 'hello-motor-omni-1', 'hello-motor-omni-2']
    for i, k in enumerate(sorted(stepper_devices.keys())):
        name = generic_names[i % len(generic_names)]
        motor = stretch4_body.core.stepper.Stepper(usb=k, name=name, backend=0)
        if not motor.startup():
            click.secho(f"  [WARN] Could not start stepper at {k}", fg='yellow')
            continue
        motor.pull_status()
        poses[k] = motor.status['pos']
        motor.stop()
    return poses


def _detect_moved_motor(stepper_devices, prompt, pos_diff_thresh=0.8):
    """
    Read positions before/after a manual prompt and return the port that moved.
    Returns (port, serial) or (None, None) if detection fails.
    """
    click.secho("\nReading initial stepper positions...", fg='cyan')
    start = _get_all_stepper_poses(stepper_devices)

    input(click.style(f"\n{prompt}\nThen hit ENTER", fg='green', bold=True))

    click.secho("\nReading final stepper positions...", fg='cyan')
    end = _get_all_stepper_poses(stepper_devices)

    diffs = {}
    for k in start:
        if k in end:
            diffs[k] = abs(start[k] - end[k])
            click.echo(f"  {k}: Δpos = {diffs[k]:.4f}")

    moved = {k: v for k, v in diffs.items() if v > pos_diff_thresh}

    if len(moved) == 0:
        click.secho("  ERROR: No motor detected as moved (diff below threshold).", fg='red')
        return None, None
    if len(moved) > 1:
        click.secho(f"  ERROR: Multiple motors moved ({list(moved.keys())}). Only move one joint at a time.", fg='red')
        return None, None

    port = list(moved.keys())[0]
    serial = stepper_devices[port]['serial']
    return port, serial


def _prepare_fleet_dir():
    """Ensure fleet directory is writable and robot serial_no is set.
    Mirrors the setup done in test_push_arduino_sns_to_udev_rules.
    """
    os.system("chmod -R 777 ~/stretch_user")
    fleet_id = hu.get_fleet_id()
    click.secho(f"  Assigning Robot SN ({fleet_id}) to robot...", fg='cyan')
    robot_dev = Device('robot')
    robot_dev.write_configuration_param_to_YAML(
        'robot.serial_no', fleet_id, hu.get_fleet_directory())


def _update_serial_in_rules_file(rules_file: str, device_name: str, new_serial: str) -> bool:
    """
    Targeted in-place update: find the udev rule line for `device_name` by its
    SYMLINK and replace ONLY its ATTRS{serial} value — leaving every other rule
    in the file untouched.

    Returns True if the line was found and updated, False if the device entry
    doesn't exist yet in the file (caller should then add it).
    """
    if not os.path.isfile(rules_file):
        return False

    with open(rules_file, 'r') as f:
        lines = f.readlines()

    symlink_pat = re.compile(r'SYMLINK\+="' + re.escape(device_name) + r'"')
    serial_pat  = re.compile(r'(ATTRS\{serial\}==")[^"]*(")')

    updated = False
    new_lines = []
    for line in lines:
        if symlink_pat.search(line):
            new_line = serial_pat.sub(r'\g<1>' + new_serial + r'\g<2>', line)
            new_lines.append(new_line)
            updated = True
        else:
            new_lines.append(line)

    if updated:
        with open(rules_file, 'w') as f:
            f.writelines(new_lines)
    return updated


def _update_etc_rules_in_place(etc_rules_path, device_name, serial_no):
    """Update ONLY the target device's serial in /etc udev rules, preserving
    all other entries.  Falls back to a full copy from fleet_dir when the
    /etc file does not exist yet.

    Writes via a temp file + sudo cp so no elevated Python process is needed.
    """
    import tempfile

    if not os.path.isfile(etc_rules_path):
        # /etc file doesn't exist yet — nothing to update in-place
        return False

    with open(etc_rules_path, 'r') as f:
        lines = f.readlines()

    symlink_pat = re.compile(r'SYMLINK\+="' + re.escape(device_name) + '"')
    serial_pat  = re.compile(r'(ATTRS\{serial\}==")[^"]*(")')

    updated = False
    new_lines = []
    for line in lines:
        if symlink_pat.search(line):
            new_line = serial_pat.sub(r'\g<1>' + serial_no + r'\g<2>', line)
            new_lines.append(new_line)
            updated = True
        else:
            new_lines.append(line)

    if not updated:
        # Device not yet in /etc — append a new rule line
        new_lines.append(
            f'KERNEL=="ttyACM*", ATTRS{{idVendor}}=="239a", '
            f'ATTRS{{idProduct}}=="8022",MODE:="0666", '
            f'ATTRS{{serial}}=="{serial_no}", '
            f'SYMLINK+="{device_name}", '
            f'ENV{{ID_MM_DEVICE_IGNORE}}="1"\n'
        )

    with tempfile.NamedTemporaryFile(mode='w', suffix='.rules', delete=False) as tmp:
        tmp.writelines(new_lines)
        tmp_path = tmp.name

    subprocess.run(['sudo', 'cp', tmp_path, etc_rules_path], check=False)
    os.remove(tmp_path)
    return True


def _update_etc_ftdi_rules_in_place(etc_rules_path, device_name, serial_no):
    """Update ONLY the target device's serial in /etc feetech rules,
    preserving all other entries."""
    import tempfile

    if not os.path.isfile(etc_rules_path):
        return False

    with open(etc_rules_path, 'r') as f:
        lines = f.readlines()

    symlink_pat = re.compile(r'SYMLINK\+="' + re.escape(device_name) + '"')
    serial_pat  = re.compile(r'(ATTRS\{serial\}==")[^"]*(")')

    updated = False
    new_lines = []
    for line in lines:
        if symlink_pat.search(line):
            new_line = serial_pat.sub(r'\g<1>' + serial_no + r'\g<2>', line)
            new_lines.append(new_line)
            updated = True
        else:
            new_lines.append(line)

    if not updated:
        new_lines.append(
            f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="0403", '
            f'ATTRS{{idProduct}}=="6001", ATTR{{device/latency_timer}}="1", '
            f'ATTRS{{serial}}=="{serial_no}", '
            f'SYMLINK+="{device_name}"\n'
        )

    with tempfile.NamedTemporaryFile(mode='w', suffix='.rules', delete=False) as tmp:
        tmp.writelines(new_lines)
        tmp_path = tmp.name

    subprocess.run(['sudo', 'cp', tmp_path, etc_rules_path], check=False)
    os.remove(tmp_path)
    return True


def _reload_arduino_udev(device_name, serial_no):
    """Update only this device's entry in /etc arduino udev rules, then reload.

    Reads from /etc (which has the live, populated serials) and updates only
    the target device line — all other entries are preserved.  Falls back to
    a full copy from fleet_dir only when the /etc file doesn't exist yet.
    """
    etc_rules = '/etc/udev/rules.d/95-hello-arduino.rules'
    if not _update_etc_rules_in_place(etc_rules, device_name, serial_no):
        # /etc file missing — bootstrap from fleet_dir
        fleet_dir = hu.get_fleet_directory()
        src = os.path.join(fleet_dir, 'udev', '95-hello-arduino.rules')
        subprocess.run(['sudo', 'cp', src, '/etc/udev/rules.d/'], check=False)
    subprocess.run(['sudo', 'udevadm', 'control', '--reload'], check=False)
    subprocess.run(['sudo', 'udevadm', 'trigger'], check=False)
    click.secho("  ✓ Arduino udev rules updated.", fg='green', bold=True)


def _reload_ftdi_udev(device_name, serial_no):
    """Update only this device's entry in /etc feetech udev rules, then reload.

    Same in-place strategy as _reload_arduino_udev.
    """
    etc_rules = '/etc/udev/rules.d/99-hello-feetech.rules'
    if not _update_etc_ftdi_rules_in_place(etc_rules, device_name, serial_no):
        fleet_dir = hu.get_fleet_directory()
        src = os.path.join(fleet_dir, 'udev', '99-hello-feetech.rules')
        subprocess.run(['sudo', 'cp', src, '/etc/udev/rules.d/'], check=False)
    subprocess.run(['sudo', 'udevadm', 'control', '--reload'], check=False)
    subprocess.run(['sudo', 'udevadm', 'trigger'], check=False)
    click.secho("  ✓ Feetech udev rules updated.", fg='green', bold=True)


def _ensure_ftdi_latency_timer_rule():
    """Ensure the generic FTDI latency timer udev rule is active in /etc BEFORE
    scanning the bus for feetech devices.

    FTDI USB-UART converters default to a 16 ms latency timer under Linux.
    At that setting, short servo response packets are held in the FTDI chip's
    buffer until the timer expires — far longer than the Feetech driver's read
    timeout — causing every ping to fail with "no status packet" errors.

    Setting latency_timer to 0 (immediate flush) via a generic udev rule fixes
    this for every FTDI converter on the rig, not just the robot's own
    registered serial.  The rule must be loaded and triggered BEFORE the scan
    so it applies to already-connected devices.
    """
    import tempfile

    ETC_RULES = '/etc/udev/rules.d/99-hello-feetech.rules'
    LATENCY_COMMENT = (
        '# Set latency timer to 0 for all FTDI USB-UART converters on the '
        'system to enable high-speed real-time servo control\n'
    )
    LATENCY_RULE = (
        'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", '
        'ATTR{device/latency_timer}="0"\n'
    )

    lines = []
    if os.path.isfile(ETC_RULES):
        with open(ETC_RULES, 'r') as f:
            lines = f.readlines()

    if any('ATTR{device/latency_timer}="0"' in l for l in lines):
        click.secho('  Generic FTDI latency timer rule already present — skipping.', fg='cyan')
        return

    click.secho('  Adding generic FTDI latency timer rule to /etc/udev/rules.d/...', fg='cyan')
    # Prepend comment + rule before the first non-comment, non-empty line
    insert_idx = 0
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped and not stripped.startswith('#'):
            insert_idx = i
            break
    lines.insert(insert_idx, LATENCY_RULE)
    lines.insert(insert_idx, LATENCY_COMMENT)

    with tempfile.NamedTemporaryFile(mode='w', suffix='.rules', delete=False) as tmp:
        tmp.writelines(lines)
        tmp_path = tmp.name

    ret = subprocess.run(['sudo', 'cp', tmp_path, ETC_RULES], check=False).returncode
    os.remove(tmp_path)
    if ret != 0:
        click.secho(f'  [WARN] Failed to write latency timer rule to {ETC_RULES}', fg='yellow')
        return

    subprocess.run(['sudo', 'udevadm', 'control', '--reload'], check=False)
    subprocess.run(['sudo', 'udevadm', 'trigger'], check=False)
    click.secho('  ✓ Generic FTDI latency timer rule written and udev reloaded.', fg='green', bold=True)


def _scan_feetech_ids(port, retries=3):
    """Scan a ttyUSB port for feetech servo IDs 20–24.

    Each servo ID is probed up to `retries` times to guard against
    intermittent serial comm failures (partial responses, stale buffer
    data, etc.).  The serial port is always closed after each attempt to
    prevent file-descriptor leaks that corrupt subsequent scans.
    """
    found = []
    for servo_id in [20, 21, 22, 23, 24]:
        for attempt in range(1, retries + 1):
            m = FeetechSMServo(servo_id, port)
            m.logger.disabled = True
            try:
                if m.startup():
                    found.append(servo_id)
                    break  # success — no more retries for this ID
            except FeetechCommError:
                pass
            except (IndexError, Exception) as e:
                # Upstream bug: identify_baud_rate() → ping() can raise
                # IndexError on partial/empty servo responses during
                # baud-rate scanning.  Retry before giving up.
                if attempt == retries:
                    click.secho(f"    [WARN] Servo ID {servo_id} scan failed after {retries} attempts: {e}", fg='yellow')
            finally:
                # Always close the port to avoid leaking file descriptors
                # which corrupt serial buffers for subsequent scans.
                try:
                    m.stop()
                except Exception:
                    pass
    return found


def _push_stepper_udev(device_name, serial_no):
    """Update ONLY this device's serial in the arduino rules file; append if new."""
    click.secho(f"  Assigning {device_name} → serial {serial_no}", fg='cyan')
    fleet_dir  = hu.get_fleet_directory()
    rules_file = os.path.join(fleet_dir, 'udev', '95-hello-arduino.rules')
    if not _update_serial_in_rules_file(rules_file, device_name, serial_no):
        click.secho(f"  (Device not yet in rules — adding new entry)", fg='yellow')
        hdu.add_arduino_udev_line(device_name=device_name,
                                   serial_no=serial_no,
                                   fleet_dir=fleet_dir)
    dev = Device(device_name)
    dev.write_configuration_param_to_YAML(
        f"{device_name}.serial_no", serial_no, fleet_dir)
    _reload_arduino_udev(device_name, serial_no)


def _push_arduino_udev(device_name, serial_no):
    """Update ONLY this device's serial in the arduino rules file; append if new."""
    click.secho(f"  Assigning {device_name} → serial {serial_no}", fg='cyan')
    fleet_dir  = hu.get_fleet_directory()
    rules_file = os.path.join(fleet_dir, 'udev', '95-hello-arduino.rules')
    if not _update_serial_in_rules_file(rules_file, device_name, serial_no):
        click.secho(f"  (Device not yet in rules — adding new entry)", fg='yellow')
        hdu.add_arduino_udev_line(device_name=device_name,
                                   serial_no=serial_no,
                                   fleet_dir=fleet_dir)
    _reload_arduino_udev(device_name, serial_no)


def _push_ftdi_udev(device_name, serial_no):
    """Update ONLY this device's serial in the FTDI rules file; append if new."""
    click.secho(f"  Assigning {device_name} → serial {serial_no}", fg='cyan')
    fleet_dir  = hu.get_fleet_directory()
    rules_file = os.path.join(fleet_dir, 'udev', '99-hello-feetech.rules')
    if not _update_serial_in_rules_file(rules_file, device_name, serial_no):
        click.secho(f"  (Device not yet in rules — adding new entry)", fg='yellow')
        hdu.add_ftdi_udev_line(device_name=device_name,
                                serial_no=serial_no,
                                fleet_dir=fleet_dir)
    _reload_ftdi_udev(device_name, serial_no)


# ─────────────────────────────────────────────────────────────────────────────
# Per-device discovery functions
# ─────────────────────────────────────────────────────────────────────────────

def discover_stepper(device_name, prompt, all_tty):
    """Generic stepper discovery via manual movement detection."""
    click.secho(f"\n{'='*60}", fg='cyan')
    click.secho(f"  Discovering: {device_name}", fg='cyan', bold=True)
    click.secho(f"{'='*60}", fg='cyan')
    steppers = _get_stepper_devices(all_tty)
    if not steppers:
        click.secho("  ERROR: No Hello_Stepper2 devices found on bus.", fg='red')
        return False
    click.echo(f"  Found {len(steppers)} stepper(s): {list(steppers.keys())}")

    if len(steppers) == 1:
        port = list(steppers.keys())[0]
        serial = steppers[port]['serial']
        click.secho(f"  Only 1 stepper detected. Auto-assigning...", fg='cyan')
    else:
        port, serial = _detect_moved_motor(steppers, prompt)
        if serial is None:
            return False

    click.secho(f"  → Detected {device_name} at {port} (serial: {serial})", fg='green', bold=True)
    _prepare_fleet_dir()
    _push_stepper_udev(device_name, serial)
    return True


def discover_power_periph(all_tty):
    """Auto-detect hello-power-periph by model name (Hello_Pimu2)."""
    click.secho(f"\n{'='*60}", fg='cyan')
    click.secho("  Discovering: hello-power-periph", fg='cyan', bold=True)
    click.secho(f"{'='*60}", fg='cyan')
    for k, v in all_tty.items():
        if v.get('vendor') == 'Hello-Robot' and v.get('model') == 'Hello_Pimu2':
            serial = v['serial']
            click.secho(f"  → Found hello-power-periph at {k} (serial: {serial})", fg='green', bold=True)
            _prepare_fleet_dir()
            _push_arduino_udev('hello-power-periph', serial)
            return True
    click.secho("  ERROR: hello-power-periph (Hello_Pimu2) not found.", fg='red')
    return False


def discover_pixart(all_tty):
    """Auto-detect hello-pixart-j3 by model name (Hello_Pixart_J3)."""
    click.secho(f"\n{'='*60}", fg='cyan')
    click.secho("  Discovering: hello-pixart-j3", fg='cyan', bold=True)
    click.secho(f"{'='*60}", fg='cyan')
    for k, v in all_tty.items():
        if v.get('vendor') == 'Hello-Robot' and v.get('model') == 'Hello_Pixart_J3':
            serial = v['serial']
            click.secho(f"  → Found hello-pixart-j3 at {k} (serial: {serial})", fg='green', bold=True)
            _prepare_fleet_dir()
            _push_arduino_udev('hello-pixart-j3', serial)
            return True
    click.secho("  ERROR: hello-pixart-j3 (Hello_Pixart_J3) not found.", fg='red')
    return False

def discover_esp32(all_tty):
    """Auto-detect hello-esp32 by vendor name (Espressif)."""
    click.secho(f"\n{'='*60}", fg='cyan')
    click.secho("  Discovering: hello-esp32", fg='cyan', bold=True)
    click.secho(f"{'='*60}", fg='cyan')
    for k, v in all_tty.items():
        if v.get('vendor') == 'Espressif':
            serial = v['serial']
            click.secho(f"  → Found hello-esp32 at {k} (serial: {serial})", fg='green', bold=True)
            _prepare_fleet_dir()
            _push_arduino_udev('hello-esp32', serial)
            return True
    click.secho("  ERROR: hello-esp32 (Espressif) not found.", fg='red')
    return False


def discover_feetech(all_tty):
    """Auto-detect hello-feetech-wrist by scanning for servo IDs 20-24 on FTDI ports.

    Pushes the generic FTDI latency timer udev rule to /etc first so that
    any connected FTDI converter is immediately configured for low-latency
    operation before the servo scan begins.
    """
    click.secho(f"\n{'='*60}", fg='cyan')
    click.secho("  Discovering: hello-feetech-wrist", fg='cyan', bold=True)
    click.secho(f"{'='*60}", fg='cyan')

    # Ensure the generic latency timer rule is active before scanning so that
    # newly connected FTDI boards are not stuck at the 16 ms default.
    _ensure_ftdi_latency_timer_rule()

    ftdi_ports = {k: v for k, v in all_tty.items()
                  if v.get('vendor') == 'FTDI' or v.get('vendor_id') == '0403'}
    if not ftdi_ports:
        click.secho("  ERROR: No FTDI devices found on bus.", fg='red')
        return False

    for port, info in ftdi_ports.items():
        click.echo(f"  Scanning {port} for feetech servos...")
        ids = _scan_feetech_ids(port)
        click.echo(f"    Found servo IDs: {ids}")
        if any(x in ids for x in [20, 21, 22, 23, 24]):
            serial = info['serial']
            click.secho(f"  → Found hello-feetech-wrist at {port} (serial: {serial})", fg='green', bold=True)
            _prepare_fleet_dir()
            _push_ftdi_udev('hello-feetech-wrist', serial)
            return True

    click.secho("  ERROR: hello-feetech-wrist (servo ID 20-24) not found on any FTDI port.", fg='red')
    return False


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

@click.command(context_settings=dict(help_option_names=['-h', '--help']))
@click.option('--lift',        is_flag=True, help='Discover hello-motor-lift   (manual movement)')
@click.option('--arm',         is_flag=True, help='Discover hello-motor-arm    (manual movement)')
@click.option('--omni-0',      is_flag=True, help='Discover hello-motor-omni-0 (manual movement)')
@click.option('--omni-1',      is_flag=True, help='Discover hello-motor-omni-1 (manual movement)')
@click.option('--omni-2',      is_flag=True, help='Discover hello-motor-omni-2 (manual movement)')
@click.option('--power-periph',is_flag=True, help='Discover hello-power-periph (auto, by model)')
@click.option('--pixart',      is_flag=True, help='Discover hello-pixart-j3    (auto, by model)')
@click.option('--feetech',     is_flag=True, help='Discover hello-feetech-wrist (auto, servo scan)')
@click.option('--esp32',       is_flag=True, help='Discover hello-esp32        (auto, by vendor)')
@click.option('--all',  'all_devices', is_flag=True,
              help='Discover all devices (auto first, then each stepper in sequence)')
def main(lift, arm, omni_0, omni_1, omni_2, power_periph, pixart, feetech, esp32, all_devices):
    """
    Discover Hello Robot USB devices and push their udev symlink rules.

    Only the serial number for the discovered device is updated in the udev
    rules file — all other entries are preserved.

    Each flag discovers one component. Multiple flags can be combined.
    Stepper flags (lift, arm, omni-*) require manual joint movement for identification.
    Other flags (power-periph, pixart, feetech) are auto-detected.
    Use --all to run every discovery in sequence.

    \b
    Examples:
      ./stretch_discover_device.py --lift
      ./stretch_discover_device.py --arm --omni-0
      ./stretch_discover_device.py --power-periph --pixart --feetech --esp32
      ./stretch_discover_device.py --all
    """
    if all_devices:
        power_periph = pixart = feetech = esp32 = True
        lift = arm = omni_0 = omni_1 = omni_2 = True

    if not any([lift, arm, omni_0, omni_1, omni_2, power_periph, pixart, feetech, esp32]):
        # No flags — show a pretty-printed table of all USB TTY devices present on the bus
        click.secho("\n======================================", fg='cyan', bold=True)
        click.secho("   USB TTY DEVICES ON BUS            ", fg='cyan', bold=True)
        click.secho("======================================\n", fg='cyan', bold=True)
        click.echo("Scanning USB bus...")
        all_tty = find_tty_devices()
        if not all_tty:
            click.secho("  No USB TTY devices found.", fg='yellow')
            sys.exit(0)

        # Column widths
        COL_PORT   = 18
        COL_VENDOR = 18
        COL_MODEL  = 24
        COL_SERIAL = 28
        COL_SYMLINK = 28

        import glob as _glob
        # Build a reverse map: serial → symlink name for /dev/hello-* devices
        serial_to_symlink = {}
        for symlink in _glob.glob('/dev/hello-*'):
            try:
                real = os.path.realpath(symlink)
                # Find which port this symlink resolves to
                for port in all_tty:
                    if os.path.realpath(port) == real or port == real:
                        serial_to_symlink[all_tty[port].get('serial', '')] = os.path.basename(symlink)
            except Exception:
                pass

        header = (
            f"{'Port':<{COL_PORT}}"
            f"{'Vendor':<{COL_VENDOR}}"
            f"{'Model':<{COL_MODEL}}"
            f"{'Serial':<{COL_SERIAL}}"
            f"{'Symlink':<{COL_SYMLINK}}"
        )
        divider = '-' * len(header)
        click.secho(header, fg='white', bold=True)
        click.echo(divider)

        for port in sorted(all_tty.keys()):
            info   = all_tty[port]
            vendor = info.get('vendor', info.get('vendor_id', '?'))
            model  = info.get('model',  '?')
            serial = info.get('serial', '?')
            symlink = serial_to_symlink.get(serial, '')

            # Colour Hello Robot devices cyan, others white
            fg = 'cyan' if info.get('vendor') == 'Hello-Robot' else 'white'
            line = (
                f"{port:<{COL_PORT}}"
                f"{vendor:<{COL_VENDOR}}"
                f"{model:<{COL_MODEL}}"
                f"{serial:<{COL_SERIAL}}"
                f"{symlink:<{COL_SYMLINK}}"
            )
            click.secho(line, fg=fg)

        click.echo(divider)
        click.secho(f"\n  {len(all_tty)} device(s) found. Pass --help to see discovery flags.\n", fg='cyan')
        sys.exit(0)

    click.secho("\n======================================", fg='cyan', bold=True)
    click.secho("   STRETCH DEVICE DISCOVERY TOOL     ", fg='cyan', bold=True)
    click.secho("======================================\n", fg='cyan', bold=True)

    click.echo("Scanning USB bus for connected devices...")
    all_tty = find_tty_devices()

    results = {}

    if power_periph:
        results['hello-power-periph'] = discover_power_periph(all_tty)

    if pixart:
        results['hello-pixart-j3'] = discover_pixart(all_tty)

    if esp32:
        results['hello-esp32'] = discover_esp32(all_tty)

    if feetech:
        results['hello-feetech-wrist'] = discover_feetech(all_tty)

    if lift:
        results['hello-motor-lift'] = discover_stepper(
            'hello-motor-lift', 'Move the LIFT joint manually', all_tty)

    if arm:
        results['hello-motor-arm'] = discover_stepper(
            'hello-motor-arm', 'Move the ARM joint manually', all_tty)

    if omni_0:
        results['hello-motor-omni-0'] = discover_stepper(
            'hello-motor-omni-0', 'Move the WHEEL 0 (omni-0) manually', all_tty)

    if omni_1:
        results['hello-motor-omni-1'] = discover_stepper(
            'hello-motor-omni-1', 'Move the WHEEL 1 (omni-1) manually', all_tty)

    if omni_2:
        results['hello-motor-omni-2'] = discover_stepper(
            'hello-motor-omni-2', 'Move the WHEEL 2 (omni-2) manually', all_tty)

    # Summary
    click.secho(f"\n{'='*40}", fg='cyan')
    click.secho("  SUMMARY", fg='cyan', bold=True)
    click.secho(f"{'='*40}", fg='cyan')
    for device, ok in results.items():
        status = click.style('PASS', fg='green', bold=True) if ok else click.style('FAIL', fg='red', bold=True)
        click.echo(f"  {device:30s} {status}")
    click.secho(f"{'='*40}\n", fg='cyan')


if __name__ == '__main__':
    main()
