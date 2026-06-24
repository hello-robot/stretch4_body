#!/usr/bin/env python3
"""
stretch_system_check.py

Comprehensive hardware and software diagnostic tool for the Stretch 4 robot.

Usage:
    stretch_system_check              # Full system check (requires server)
    stretch_system_check --firmware   # Firmware version check (kills/restarts server)
    stretch_system_check --sensors    # Lidar + camera check (no server needed)
    stretch_system_check --verbose    # Show additional detail in all checks
    stretch_system_check --direct     # Use Robot API directly instead of server client
"""
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

import os
import sys
import io
import fnmatch
import argparse
import subprocess
import logging
from importlib.metadata import version as pkg_version

import click
from stretch4_body.core import robot_params as rp


# ==============================================================================
# CLI Arguments
# ==============================================================================

parser = argparse.ArgumentParser(
    description='Check that all Stretch 4 robot hardware is present and reporting sane values'
)
parser.add_argument('-v', '--verbose',  help='Print additional detail',                  action='store_true')
parser.add_argument('-d', '--direct',   help='Use direct Robot API (no server)',          action='store_true')
parser.add_argument('--firmware',       help='Kill server, check firmware, restart server', action='store_true')
parser.add_argument('--sensors',        help='Check lidars and cameras',                 action='store_true')
args = parser.parse_args()

logging.getLogger('stretch4_body').setLevel(logging.WARNING)
logging.getLogger('stretch_body_client').setLevel(logging.WARNING)


# ==============================================================================
# Robot identity (read once at startup)
# ==============================================================================

_robot_info   = rp.RobotParams._robot_params.get('robot', {})
stretch_serial_no = _robot_info.get('serial_no', 'N/A')
stretch_model     = _robot_info.get('model_name', 'N/A')
stretch_batch     = _robot_info.get('batch_name', 'N/A')
stretch_tool      = _robot_info.get('tool', 'N/A')

TOOL_DISPLAY = {
    'eoa_wrist_dw4_tool_nil':         'DexWrist 4 — No Tool',
    'eoa_wrist_dw4_tool_sg4':         'DexWrist 4 — Stretch Gripper (SG4)',
    'eoa_wrist_dw4_tool_pg4':         'DexWrist 4 — Parallel Gripper (PG4)',
    'eoa_wrist_dw4_tool_tablet':      'DexWrist 4 — Tablet Holder',
    'eoa_wrist_dw4_tool_calibration': 'DexWrist 4 — Calibration Tool',
}

_model_display = 'Stretch 4' if stretch_model == 'SE4' else stretch_model

click.secho('\n======== Stretch 4 System Check ========', fg='cyan', bold=True)
click.secho(f'  Model         : {_model_display}',                                  fg='bright_white')
click.secho(f'  Serial Number : {stretch_serial_no}',                               fg='bright_white')
click.secho(f'  Tool          : {TOOL_DISPLAY.get(stretch_tool, stretch_tool)}',    fg='bright_white')
if args.verbose:
    click.secho(f'  Batch         : {stretch_batch}',                               fg='bright_white')
click.secho('========================================\n', fg='cyan', bold=True)


# Robot client handle — assigned in main() after server startup
r = None


# ==============================================================================
# Output helpers
# ==============================================================================

def print_section(title):
    click.secho(f'\n---- {title} ----', fg='cyan', bold=True)

def print_result(passed, msg, indent=2):
    pad = ' ' * indent
    if passed:
        click.secho(f'{pad}[PASS] {msg}', fg='green')
    else:
        click.secho(f'{pad}[FAIL] {msg}', fg='red')

def print_warn(msg, indent=2):
    click.secho(f'{" " * indent}[WARN] {msg}', fg='yellow')

def print_info(msg, indent=4):
    click.secho(f'{" " * indent}{msg}', fg='white')

def val_in_range(label, val, vmin, vmax):
    ok = vmin <= val <= vmax
    return ok, f'{label} = {val:.3f} (range [{vmin:.2f}, {vmax:.2f}])'


# ==============================================================================
# Check functions
# ==============================================================================

def print_software_versions():
    print_section('Software Versions')

    # Always-shown core packages
    CORE_PIP = {'hello-robot-stretch4-body', 'hello-robot-stretch4-urdf'}
    try:
        s4b_ver = pkg_version('hello-robot-stretch4-body')
    except Exception:
        s4b_ver = 'unknown'
    try:
        urdf_ver = pkg_version('hello-robot-stretch4-urdf')
    except Exception:
        urdf_ver = 'unknown'
    py_ver = f'{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}'

    print_info(f'hello-robot-stretch4-body : {s4b_ver}')
    print_info(f'hello-robot-stretch4-urdf : {urdf_ver}')
    print_info(f'Python                    : {py_ver}')

    ros_distro = os.environ.get('ROS_DISTRO', '')
    if ros_distro:
        print_info(f'ROS2 Distro               : {ros_distro}')

    # Additional pip packages — auto-discovered by name keyword
    try:
        from importlib.metadata import distributions as _distributions
        pip_extras = {}
        for dist in _distributions():
            name = (dist.metadata.get('Name') or '').strip()
            name_lc = name.lower()
            if not name or name.lower() in {n.lower() for n in CORE_PIP}:
                continue
            if 'hello' in name_lc or 'stretch' in name_lc or 'hesai' in name_lc:
                if name not in pip_extras:  # keep first occurrence
                    pip_extras[name] = (dist.metadata.get('Version') or 'unknown').strip()
        if pip_extras:
            col = max(len(k) for k in pip_extras)
            click.secho('\n  Python / pip:', fg='white', bold=True)
            for name in sorted(pip_extras):
                print_info(f'  {name:<{col}} : {pip_extras[name]}')
    except Exception:
        pass

    # ROS2 packages — auto-discovered via AMENT_PREFIX_PATH
    try:
        import xml.etree.ElementTree as ET
        ros2_pkgs = {}
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
            share = os.path.join(prefix, 'share')
            if not os.path.isdir(share):
                continue
            for pkg in os.listdir(share):
                if pkg in ros2_pkgs:
                    continue
                if 'stretch' not in pkg.lower() and 'hello' not in pkg.lower():
                    continue
                xml_path = os.path.join(share, pkg, 'package.xml')
                if os.path.isfile(xml_path):
                    try:
                        tree = ET.parse(xml_path)
                        ver = tree.find('version')
                        ros2_pkgs[pkg] = ver.text.strip() if ver is not None else 'unknown'
                    except Exception:
                        pass
        if ros2_pkgs:
            col = max(len(k) for k in ros2_pkgs)
            click.secho('\n  ROS2 Packages:', fg='white', bold=True)
            for pkg in sorted(ros2_pkgs):
                print_info(f'  {pkg:<{col}} : {ros2_pkgs[pkg]}')
    except Exception:
        pass


def check_usb_devices():
    print_section('USB Devices')
    expected = [
        'hello-motor-arm',
        'hello-motor-lift',
        'hello-motor-omni-0',
        'hello-motor-omni-1',
        'hello-motor-omni-2',
        'hello-power-periph',
        'hello-feetech-wrist',
        'hello-esp32',
        'hello-nav-head-camera-stereo',
        'hello-pixart-j3',
    ]
    dev_list = set(os.listdir('/dev'))
    all_pass = True
    for dev in expected:
        present = dev in dev_list
        print_result(present, f'/dev/{dev}')
        if not present:
            all_pass = False
    if args.verbose:
        extras = sorted(e for e in dev_list if fnmatch.fnmatch(e, 'hello-*') and e not in expected)
        for extra in extras:
            print_info(f'(extra) /dev/{extra}')
    return all_pass


def check_firmware_versions():
    """Query installed firmware via FirmwareInstalled. Server must be stopped first."""
    print_section('Firmware Versions')

    from stretch4_body.core.device import Device
    from stretch4_body.core.factory.firmware_installed import FirmwareInstalled
    from stretch4_body.core.factory.firmware_recommended import FirmwareRecommended

    d = Device(req_params=False)
    is_unh = d.robot_params.get('robot', {}).get('model_name') == 'SE4UNH'

    use_device = {
        'hello-esp32':        True,
        'hello-motor-arm':    not is_unh,
        'hello-motor-lift':   True,
        'hello-motor-omni-0': True,
        'hello-motor-omni-1': True,
        'hello-motor-omni-2': True,
        'hello-power-periph': True,
        'hello-pixart-j3':    True,
    }

    DEVICE_LABELS = {
        'hello-motor-arm':    'Arm Stepper',
        'hello-motor-lift':   'Lift Stepper',
        'hello-motor-omni-0': 'Omni Wheel 0',
        'hello-motor-omni-1': 'Omni Wheel 1',
        'hello-motor-omni-2': 'Omni Wheel 2',
        'hello-power-periph': 'Power Periph (pimu2)',
        'hello-pixart-j3':    'PixArt J3 (line sensor)',
        'hello-esp32':        'ESP32',
    }

    # Suppress all log/print output while querying firmware
    logging.disable(logging.CRITICAL)
    _old_stdout, _old_stderr = sys.stdout, sys.stderr
    sys.stdout = sys.stderr = io.StringIO()
    try:
        fw_installed = FirmwareInstalled(use_device)
    except Exception as e:
        sys.stdout, sys.stderr = _old_stdout, _old_stderr
        logging.disable(logging.NOTSET)
        print_warn(f'Could not query firmware: {e}')
        return True
    try:
        fw_recommended = FirmwareRecommended(use_device, installed=fw_installed)
    except Exception:
        fw_recommended = None
    sys.stdout, sys.stderr = _old_stdout, _old_stderr
    logging.disable(logging.NOTSET)

    all_pass = True
    for dev_name, enabled in use_device.items():
        if not enabled:
            continue
        label = DEVICE_LABELS.get(dev_name, dev_name)

        if not fw_installed.is_device_valid(dev_name):
            print_result(False, f'{label}: not found / comms failure')
            all_pass = False
            continue

        # ESP32 and PixArt J3 don't expose queryable firmware versions
        if dev_name in ('hello-pixart-j3', 'hello-esp32'):
            dev_present = os.path.exists(f'/dev/{dev_name}')
            status = 'present' if dev_present else 'not present'
            print_result(dev_present, f'{label}: {status} (firmware version not queryable)')
            if not dev_present:
                all_pass = False
            continue

        fw_ver   = fw_installed.config_info[dev_name]['board_info']['firmware_version']
        proto    = fw_installed.config_info[dev_name]['board_info'].get('protocol_version', '?')
        hw_id    = fw_installed.config_info[dev_name]['board_info'].get('hardware_id', '?')
        proto_ok = fw_installed.config_info[dev_name].get('installed_protocol_valid', True)

        up_to_date = True
        rec_str = ''
        if fw_recommended is not None:
            rec = fw_recommended.recommended.get(dev_name)
            installed_ver = fw_installed.get_version(dev_name)
            if rec is not None:
                if rec > installed_ver:
                    up_to_date = False
                    rec_str = f' → recommended: {rec}'
                elif rec < installed_ver:
                    rec_str = ' (dev/ahead of recommended)'

        detail = f'protocol: {proto}  |  hw_id: {hw_id}  |  proto_valid: {proto_ok}'
        if not proto_ok:
            print_result(False, f'{label}: {fw_ver}{rec_str}')
            print_info(detail)
            all_pass = False
        elif not up_to_date:
            print_warn(f'{label}: {fw_ver}{rec_str}')
            print_info(detail)
        else:
            print_result(True, f'{label}: {fw_ver}{rec_str}')
            print_info(detail)

    return all_pass


def check_power_periph():
    print_section('Power & Battery')
    all_pass = True
    ps = r.power_periph.status

    soc = ps.get('battery_soc', 0)
    if soc >= 20:
        print_result(True, f'Battery SOC = {soc:.0f}%')
    elif soc >= 10:
        print_warn(f'Battery SOC = {soc:.0f}% (low — consider charging)')
    else:
        print_result(False, f'Battery SOC = {soc:.0f}% (critically low!)')
        all_pass = False

    soh = ps.get('battery_soh', 0)
    if soh >= 75:
        print_result(True, f'Battery SOH = {soh:.1f}%')
    else:
        print_warn(f'Battery SOH = {soh:.1f}% (degraded battery)')

    for label, key, vmin, vmax in [
        ('Bus Voltage (V)',  'voltage',     20.0, 30.0),
        ('CPU Voltage (V)',  'voltage_cpu', 15.0, 30.0),
        ('12V Rail (V)',     'voltage_12v0', 10.0, 14.0),
        ('5V Rail (V)',      'voltage_5v0',   4.5,  5.5),
    ]:
        val = ps.get(key, 0)
        if val > 0:
            p, msg = val_in_range(label, val, vmin, vmax)
            print_result(p, msg)
            if not p:
                all_pass = False

    runstop = ps.get('runstop_event', False)
    print_warn('Runstop is active') if runstop else print_result(True, 'Runstop not active')

    temp = ps.get('temp', 0)
    p, msg = val_in_range('Board Temp (°C)', temp, 0, 80)
    print_result(p, msg)
    if not p:
        all_pass = False

    if args.verbose:
        print_info(f'Battery Current  : {ps.get("battery_current", 0):.2f} A')
        print_info(f'Adapter present  : {ps.get("adapter_voltage_present", False)}')
        print_info(f'Adapter fault    : {ps.get("adapter_fault", False)}')
        print_info(f'Charging         : {ps.get("charger_is_charging", False)}')

    return all_pass


def check_esp32():
    print_section('ESP32 Connectivity')
    ps = r.power_periph.status

    esp32_present = os.path.exists('/dev/hello-esp32')
    print_result(esp32_present, '/dev/hello-esp32 present')
    if not esp32_present:
        return False

    if args.verbose:
        print_info(f'Aux CPU on  : {ps.get("cpu_on_sts", False)}')

    return True


def check_line_sensors():
    print_section('Line Sensors (PixArt J3)')

    if r is None:
        print_warn('Server offline — cannot check line sensor status')
        return True

    line_sensor = getattr(r, 'line_sensor_loop', None)
    if line_sensor is None:
        print_result(False, 'line_sensor_loop not in server subsystems (sensors not detected)')
        return False

    all_pass = True
    lss = line_sensor.status
    rate = lss.get('rate_hz', 0)
    p = rate > 0
    print_result(p, f'Line sensor loop running at {rate:.1f} Hz')
    if not p:
        all_pass = False

    sensor_names = [k for k in lss if k not in ('rate_hz', 'last_frame_time')]
    if not sensor_names:
        print_result(False, 'No individual sensors reporting')
        return False

    for sn in sensor_names:
        frame_id = lss.get(sn, {}).get('frame_id', 0)
        print_result(frame_id > 0, f'Sensor {sn}: frame_id = {frame_id}')
        if frame_id == 0:
            all_pass = False

    return all_pass


def check_omnibase():
    print_section('OmniBase (3-Wheel Drive)')
    all_pass = True
    obs = r.omnibase.status

    for i in range(3):
        ws = obs.get(f'wheel_{i}', {})
        pos = ws.get('pos') if ws else None
        if pos is None:
            print_result(False, f'omni-{i}: no status data')
            all_pass = False
            continue
        print_result(True, f'omni-{i}: pos = {pos:.3f} rad')
        if args.verbose:
            effort = ws.get('effort_pct')
            vel    = ws.get('vel')
            if effort is not None:
                print_info(f'omni-{i} effort = {effort:.1f}%')
            if vel is not None:
                print_info(f'omni-{i} vel    = {vel:.3f} rad/s')

    return all_pass


def check_arm():
    print_section('Arm')
    all_pass = True
    arm_s   = r.arm.status
    motor_s = arm_s.get('motor', {})

    if motor_s.get('pos_calibrated', False):
        print_result(True, 'Arm is homed')
    else:
        print_warn('Arm not homed (pos_calibrated = False)')

    pos = arm_s.get('pos')
    if pos is not None:
        p, msg = val_in_range('Arm pos (m)', pos, -0.01, 0.56)
        print_result(p, msg)
        if not p:
            all_pass = False
    else:
        print_result(False, 'Arm pos not available')
        all_pass = False

    return all_pass


def check_lift():
    print_section('Lift')
    all_pass = True
    lift_s  = r.lift.status
    motor_s = lift_s.get('motor', {})

    if motor_s.get('pos_calibrated', False):
        print_result(True, 'Lift is homed')
    else:
        print_warn('Lift not homed (pos_calibrated = False)')

    pos = lift_s.get('pos')
    if pos is not None:
        p, msg = val_in_range('Lift pos (m)', pos, -0.01, 1.12)
        print_result(p, msg)
        if not p:
            all_pass = False
    else:
        print_result(False, 'Lift pos not available')
        all_pass = False

    return all_pass


def check_end_of_arm():
    print_section(f'End-of-Arm ({TOOL_DISPLAY.get(stretch_tool, stretch_tool)})')
    all_pass = True
    eoa    = r.end_of_arm
    joints = getattr(eoa, 'joints', [])

    if not joints:
        print_warn('No joints defined in end-of-arm params')
        return True

    eoa_s = eoa.status
    for joint_name in joints:
        js = eoa_s.get(joint_name, {})
        if not js:
            print_result(False, f'{joint_name}: no status data')
            all_pass = False
            continue

        pos            = js.get('pos')
        pos_calibrated = js.get('pos_calibrated', False)
        temp           = js.get('temp')
        hw_err         = js.get('hardware_error', 0)

        if pos is None:
            print_result(False, f'{joint_name}: no position data')
            all_pass = False
            continue

        joint_ok = (hw_err == 0) and (temp is None or temp < 70)
        temp_str = f'  temp={temp:.0f}°C' if temp is not None else ''
        err_str  = f'  hw_error={hw_err}' if hw_err else ''
        print_result(joint_ok, f'{joint_name}: pos={pos:.3f} rad  homed={int(pos_calibrated)}{temp_str}{err_str}')
        if not joint_ok:
            all_pass = False

        if args.verbose:
            effort  = js.get('effort')
            curr_mA = js.get('current_mA')
            if effort is not None:
                print_info(f'{joint_name} effort = {effort:.1f}%')
            if curr_mA is not None:
                print_info(f'{joint_name} current = {curr_mA:.1f} mA')

    return all_pass


def check_imu():
    print_section('IMU (Base)')
    imu_s = r.power_periph.status.get('imu', {})
    if not imu_s:
        print_result(False, 'IMU status not available')
        return False

    all_pass = True
    az = imu_s.get('az', 0)
    p, msg = val_in_range('IMU az (m/s²)', az, 7.0, 11.0)
    print_result(p, msg)
    if not p:
        all_pass = False

    if args.verbose:
        ax, ay = imu_s.get('ax', 0), imu_s.get('ay', 0)
        print_info(f'ax = {ax:.3f}  ay = {ay:.3f}  az = {az:.3f} m/s²')
        print_info(f'roll = {imu_s.get("roll", 0):.4f} rad  pitch = {imu_s.get("pitch", 0):.4f} rad')
        print_info(f'gravity_tilt = {imu_s.get("gravity_tilt", 0):.4f} rad')

    return all_pass


def check_eye_animations():
    print_section('Eye LED Animations')

    if r is None:
        print_warn('Server offline — cannot check eye animation status')
        return True

    from stretch4_body.core.device import Device
    eye_cfg  = Device(req_params=False).robot_params.get('sentry_eye_animations', {})
    enabled  = bool(eye_cfg.get('enabled', 0))
    behavior = eye_cfg.get('behavior', 'unknown')

    if not enabled:
        print_result(False, 'sentry_eye_animations disabled in robot params')
        return False

    print_result(True, f'sentry_eye_animations enabled  (behavior: {behavior})')

    proto = None
    try:
        bi = getattr(r.power_periph, 'board_info', {}) or {}
        proto_str = bi.get('protocol_version')
        if proto_str:
            proto = int(proto_str.lstrip('p'))
    except Exception:
        pass

    if proto is not None:
        ok = proto >= 13
        print_result(ok, f'PowerPeriph protocol: p{proto} (≥p13 required for LED support)')
        return ok
    else:
        print_warn('Could not read PowerPeriph protocol version')
        return True


def check_calibrations():
    print_section('Calibrations Present')
    all_pass = True

    fleet_path = os.environ.get('HELLO_FLEET_PATH', os.path.expanduser('~/stretch_user'))
    fleet_id   = os.environ.get('HELLO_FLEET_ID', stretch_serial_no)
    cal_root   = os.path.join(fleet_path, fleet_id)

    click.secho('    Steppers:', fg='white', bold=True)
    stepper_dir = os.path.join(cal_root, 'calibration_steppers')
    for m in ['hello-motor-arm', 'hello-motor-lift',
              'hello-motor-omni-0', 'hello-motor-omni-1', 'hello-motor-omni-2']:
        files = ([f for f in os.listdir(stepper_dir)
                  if f.startswith(m + '_') and f.endswith('.yaml')]
                 if os.path.isdir(stepper_dir) else [])
        ok = len(files) > 0
        print_result(ok, m, indent=6)
        if not ok:
            all_pass = False

    click.secho('    Cameras:', fg='white', bold=True)
    cam_dir = os.path.join(cal_root, 'calibration_cameras')
    for label, fname in {
        'intrinsics: center': 'calibration_ros_camera_info_center.yaml',
        'intrinsics: left':   'calibration_ros_camera_info_left.yaml',
        'intrinsics: right':  'calibration_ros_camera_info_right.yaml',
        'extrinsics':         'camera_extrinsics.yaml',
    }.items():
        ok = os.path.isfile(os.path.join(cam_dir, fname))
        print_result(ok, label, indent=6)
        if not ok:
            all_pass = False

    click.secho('    Line Sensors:', fg='white', bold=True)
    ls_dir = os.path.join(cal_root, 'calibration_line_sensors')
    if os.path.isdir(ls_dir):
        for i in range(6):
            sensor_dir = os.path.join(ls_dir, f'sensor_{i}')
            has_cal = (os.path.isdir(sensor_dir) and
                       any(f.endswith('.yaml') for _, _, files in os.walk(sensor_dir) for f in files))
            print_result(has_cal, f'sensor_{i}', indent=6)
            if not has_cal:
                all_pass = False
    else:
        print_result(False, f'calibration directory missing: {ls_dir}', indent=6)
        all_pass = False

    click.secho('    Lidars:', fg='white', bold=True)
    hesai_dir = os.path.join(cal_root, 'calibration_hesais')
    for side in ('left', 'right'):
        ok = os.path.isfile(os.path.join(hesai_dir, f'{side}_lidar_calibration.dat'))
        print_result(ok, f'{side} lidar', indent=6)
        if not ok:
            all_pass = False

    click.secho('    OmniBase:', fg='white', bold=True)
    imu_dir = os.path.join(cal_root, 'calibration_omnibase_imu')
    has_imu_cal = os.path.isdir(imu_dir) and any(os.listdir(imu_dir))
    print_result(has_imu_cal, 'IMU calibration', indent=6)
    if not has_imu_cal:
        all_pass = False

    return all_pass


def _ptc_call(fn, *args, label='', fail_ok=False):
    """
    Call a pyhesai_wrapper.ptc_client helper, suppressing SDK stdout/stderr.
    Returns (value, error_string).  error_string is None on success.
    """
    _o, _e = sys.stdout, sys.stderr
    try:
        logging.disable(logging.CRITICAL)
        sys.stdout = sys.stderr = io.StringIO()
        result = fn(*args)
        sys.stdout, sys.stderr = _o, _e
        logging.disable(logging.NOTSET)
        return result, None
    except Exception as exc:
        sys.stdout, sys.stderr = _o, _e
        logging.disable(logging.NOTSET)
        return None, str(exc)


def _check_lidar_ptc(ip, side, all_pass):
    """Run the full PTC config check for a single lidar. Returns updated all_pass."""
    from pyhesai_wrapper.ptc_client import (
        FILTER_NAMES, FILTER_STRONG,
        PTP_LOCK_OFFSET_US, PTP_STATUS_LOCKED, PTP_STATUS_NAMES,
        RETURN_MODE_LAST_AND_STRONGEST, RETURN_MODE_NAMES,
        get_lidar_ptp_status, get_point_cloud_config,
        get_ptp_lock_offset_us, get_return_mode,
    )

    mode_val, err = _ptc_call(get_return_mode, ip)
    if err:
        print_warn(f'Return mode query failed: {err}')
    else:
        mode_name = RETURN_MODE_NAMES.get(mode_val, f'mode {mode_val}')
        ok = mode_val == RETURN_MODE_LAST_AND_STRONGEST
        suffix = '' if ok else f'  (expected: {RETURN_MODE_NAMES[RETURN_MODE_LAST_AND_STRONGEST]})'
        if ok:
            print_result(True, f'Return mode = {mode_val} ({mode_name})')
        else:
            print_warn(f'Return mode = {mode_val} ({mode_name}){suffix}')
            all_pass = False

    cfg_val, err = _ptc_call(get_point_cloud_config, ip)
    if err:
        print_warn(f'Point-cloud filter query failed: {err}')
    else:
        _, filt = cfg_val
        filter_name = FILTER_NAMES.get(filt, f'filter {filt}')
        ok = filt == FILTER_STRONG
        suffix = '' if ok else f'  (expected: {FILTER_NAMES[FILTER_STRONG]})'
        if ok:
            print_result(True, f'Filter = {filt} ({filter_name})')
        else:
            print_warn(f'Filter = {filt} ({filter_name}){suffix}')
            all_pass = False

    offset_val, err = _ptc_call(get_ptp_lock_offset_us, ip)
    if err:
        print_warn(f'PTP lock offset query failed: {err}')
    else:
        ok = offset_val == PTP_LOCK_OFFSET_US
        suffix = '' if ok else f'  (expected: {PTP_LOCK_OFFSET_US} µs)'
        if ok:
            print_result(True, f'PTP lock offset = {offset_val} µs')
        else:
            print_warn(f'PTP lock offset = {offset_val} µs{suffix}')
            all_pass = False

    ptp_val, err = _ptc_call(get_lidar_ptp_status, ip)
    if err:
        print_warn(f'PTP status query failed: {err}')
    else:
        status      = ptp_val.get('ptp_status')
        status_name = ptp_val.get('ptp_status_name', PTP_STATUS_NAMES.get(status, str(status)))
        ok = status == PTP_STATUS_LOCKED
        if ok:
            print_result(True, f'PTP status = {status_name}')
        else:
            print_warn(f'PTP status = {status_name}  (expected: locked)')
            all_pass = False

    return all_pass


def _check_lidar_streaming(lidars, timeout=5.0):
    """
    Listen on Hesai UDP ports for packets from each lidar.
    Returns dict {side: bool} indicating which lidars sent data.
    """
    import socket, select, time
    LIDAR_PORTS = [2368, 2378]
    ip_to_side  = {cfg['ip']: side for side, cfg in lidars.items()}
    received    = {}

    sockets = []
    try:
        for port in LIDAR_PORTS:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.setblocking(False)
                s.bind(('0.0.0.0', port))
                sockets.append(s)
            except OSError as e:
                print_warn(f'Could not bind port {port}: {e}  (ensure no lidar driver is running)')

        if not sockets:
            return {side: False for side in lidars}

        deadline = time.time() + timeout
        while time.time() < deadline and len(received) < len(lidars):
            ready, _, _ = select.select(sockets, [], [], 0.1)
            for sock in ready:
                try:
                    _, addr = sock.recvfrom(2048)
                    sender_ip = addr[0]
                    side = ip_to_side.get(sender_ip)
                    if side and side not in received:
                        received[side] = True
                except socket.error:
                    pass
    finally:
        for s in sockets:
            s.close()

    return {side: received.get(side, False) for side in lidars}


def check_sensors():
    """Check Hesai lidars (PTC config + streaming) and OAK cameras (via DepthAI)."""
    import socket, time
    print_section('Sensors')
    all_pass = True

    LIDARS = {
        'left':  {'ip': '192.168.1.202', 'ptc_port': 9347},
        'right': {'ip': '192.168.1.201', 'ptc_port': 9347},
    }
    try:
        import yaml, importlib.util
        spec = importlib.util.find_spec('pyhesai_wrapper')
        if spec:
            cfg_path = os.path.join(os.path.dirname(spec.origin), 'config.yaml')
            if os.path.isfile(cfg_path):
                with open(cfg_path) as f:
                    hw_cfg = yaml.safe_load(os.path.expandvars(f.read()))
                LIDARS['left']['ip']        = hw_cfg['left_lidar']['ip']
                LIDARS['right']['ip']       = hw_cfg['right_lidar']['ip']
                LIDARS['left']['ptc_port']  = hw_cfg['left_lidar']['ptc_port']
                LIDARS['right']['ptc_port'] = hw_cfg['right_lidar']['ptc_port']
    except Exception:
        pass

    print_section('Lidars (Hesai)')
    ptc_available = False
    try:
        from pyhesai_wrapper.ptc_client import get_return_mode as _test_import  # noqa
        ptc_available = True
    except ImportError:
        print_warn('pyhesai_wrapper.ptc_client not available — PTC config checks skipped')

    for side, cfg in LIDARS.items():
        ip, port = cfg['ip'], cfg['ptc_port']
        click.secho(f'\n  {side.capitalize()} lidar ({ip})', fg='cyan', bold=True)

        ping_ok = subprocess.call(
            ['ping', '-c', '1', '-W', '1', ip],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        ) == 0
        print_result(ping_ok, f'Ping reachable')
        if not ping_ok:
            all_pass = False
            continue

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            ptc_ok = sock.connect_ex((ip, port)) == 0
            sock.close()
        except Exception:
            ptc_ok = False
        print_result(ptc_ok, f'PTC reachable ({ip}:{port})')
        if not ptc_ok:
            all_pass = False
            continue

        if ptc_available:
            all_pass = _check_lidar_ptc(ip, side, all_pass)

    click.secho(f'\n  Streaming check (listening 5 s for UDP packets)...', fg='white')
    stream_results = _check_lidar_streaming(LIDARS, timeout=5.0)
    for side, streaming in stream_results.items():
        ip = LIDARS[side]['ip']
        print_result(streaming, f'{side.capitalize()} lidar ({ip}): streaming UDP data')
        if not streaming:
            all_pass = False

    print_section('Cameras')

    HEAD_STREAMS = [
        {'name': 'Center', 'socket_name': 'CAM_A', 'width': 4032, 'height': 3040, 'fps': 5},
        {'name': 'Left',   'socket_name': 'CAM_C', 'width': 1280, 'height':  800, 'fps': 12},
        {'name': 'Right',  'socket_name': 'CAM_B', 'width': 1280, 'height':  800, 'fps': 12},
    ]
    GRIPPER_STREAMS = [
        {'name': 'Left',  'socket_name': 'CAM_C', 'width': 1280, 'height': 800, 'fps': 12},
        {'name': 'Right', 'socket_name': 'CAM_B', 'width': 1280, 'height': 800, 'fps': 12},
    ]
    CAMERA_ROLES = [
        {'label': 'Head camera (OAK-FFC-3P)',   'n_sensors': 3, 'streams': HEAD_STREAMS},
        {'label': 'Gripper camera (OAK-D-SR)',   'n_sensors': 2, 'streams': GRIPPER_STREAMS},
    ]
    FPS_TOLERANCE    = 0.50
    FPS_MEASURE_SECS = 3.0

    SPINNER_FRAMES = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏']

    def _spin(label, stop_event):
        """Animate a braille spinner on the label line until stop_event is set."""
        import itertools
        for frame in itertools.cycle(SPINNER_FRAMES):
            if stop_event.is_set():
                break
            sys.stdout.write(f'\r  {frame} {label}  ')
            sys.stdout.flush()
            time.sleep(0.08)
        # Clear the spinner line so the caller can print cleanly
        sys.stdout.write(f'\r{" " * (len(label) + 8)}\r')
        sys.stdout.flush()

    try:
        import depthai as dai
    except ImportError:
        print_warn('depthai not installed — cannot check OAK cameras')
        return all_pass

    for role in CAMERA_ROLES:
        label    = role['label']
        n_expect = role['n_sensors']
        streams  = role['streams']

        import threading
        stop_spin = threading.Event()
        spin_thread = threading.Thread(target=_spin, args=(label, stop_spin), daemon=True)
        spin_thread.start()

        # Re-enumerate fresh — stale DeviceInfo becomes invalid after the previous
        # camera's pipeline closes, causing the next probe to fail.
        try:
            device_infos = dai.Device.getAllAvailableDevices()
        except Exception as e:
            stop_spin.set()
            spin_thread.join()
            print_warn(f'DepthAI enumeration failed: {e}')
            all_pass = False
            continue

        # Find device with matching sensor count
        target_info = None
        for info in device_infos:
            try:
                tmp = dai.Device(info)
                n = len(tmp.getConnectedCameras())
                tmp.close()
                if n == n_expect:
                    target_info = info
                    break
            except Exception:
                pass

        if target_info is None:
            stop_spin.set()
            spin_thread.join()
            print_result(False, f'No OAK device with {n_expect} sensors found')
            all_pass = False
            continue

        try:
            device = dai.Device(target_info)
        except Exception as e:
            stop_spin.set()
            spin_thread.join()
            print_result(False, f'Could not open device: {e}')
            all_pass = False
            continue

        try:
            mxid  = device.getDeviceId()
            speed = device.getUsbSpeed()
            connected = device.getConnectedCameras()
            n_actual  = len(connected)

            pipeline = dai.Pipeline(defaultDevice=device)
            queues = {}
            socket_map = dai.CameraBoardSocket.__members__

            for cfg in streams:
                sock = socket_map.get(cfg['socket_name'])
                if sock is None or sock not in connected:
                    continue
                cam_node = pipeline.create(dai.node.Camera)
                cam_node.setSensorType(dai.CameraSensorType.COLOR)
                cam_node.build(boardSocket=sock, sensorFps=cfg['fps'])
                cam_out = cam_node.requestOutput(
                    size=(cfg['width'], cfg['height']),
                    fps=cfg['fps'],
                    type=dai.ImgFrame.Type.NV12,
                    resizeMode=dai.ImgResizeMode.CROP,
                    enableUndistortion=False,
                )
                queues[cfg['name']] = cam_out.createOutputQueue(maxSize=4, blocking=False)

            stop_spin.set()
            spin_thread.join()
            click.secho(f'  {label}', fg='cyan', bold=True)

            ok = n_actual == n_expect
            print_result(ok, f'Sensors detected: {n_actual}  (id={mxid})')
            if not ok:
                all_pass = False

            speed_ok = speed == dai.UsbSpeed.SUPER
            if speed_ok:
                print_result(True, f'USB speed: {speed.name}')
            else:
                print_warn(f'USB speed: {speed.name}  (expected SUPER / USB 3 — check cable)')

            # Suppress SDK calibration warnings (C++ threads emit during start + warmup)
            print_info('Starting pipeline...')
            _devnull_fd = os.open(os.devnull, os.O_WRONLY)
            _saved_stderr_fd = os.dup(2)
            os.dup2(_devnull_fd, 2)
            os.close(_devnull_fd)
            try:
                pipeline.start()
                print_info('Warming up streams (2 s)...')
                warmup_end = time.time() + 2.0
                while time.time() < warmup_end:
                    for q in queues.values():
                        if q.has():
                            q.get()
                    time.sleep(0.01)
            finally:
                os.dup2(_saved_stderr_fd, 2)
                os.close(_saved_stderr_fd)

            print_info('Checking stream activity...')
            frames_seen = {name: False for name in queues}
            deadline = time.time() + 2.0
            while time.time() < deadline:
                for name, q in queues.items():
                    if q.has():
                        q.get()
                        frames_seen[name] = True
                if all(frames_seen.values()):
                    break
                time.sleep(0.01)

            for name, seen in frames_seen.items():
                print_result(seen, f'Stream active: {name}')
                if not seen:
                    all_pass = False

            print_info(f'Measuring FPS ({FPS_MEASURE_SECS:.0f} s)...')
            counters = {name: 0 for name in queues}
            for q in queues.values():
                while q.has():
                    q.get()
            t0 = time.time()
            while time.time() - t0 < FPS_MEASURE_SECS:
                for name, q in queues.items():
                    while q.has():
                        q.get()
                        counters[name] += 1
                time.sleep(0.001)
            elapsed = time.time() - t0

            for cfg in streams:
                name = cfg['name']
                if name not in queues:
                    continue
                actual_fps = counters[name] / elapsed
                target_fps = cfg['fps']
                min_fps    = target_fps * (1 - FPS_TOLERANCE)
                fps_ok     = actual_fps >= min_fps
                print_result(fps_ok, f'FPS {name}: {actual_fps:.1f}  (target {target_fps})')
                if not fps_ok:
                    all_pass = False

            print_info('Capturing frames for resolution check...')
            captured = {}
            deadline = time.time() + 2.0
            while time.time() < deadline and len(captured) < len(queues):
                for name, q in queues.items():
                    if name not in captured and q.has():
                        captured[name] = q.get()
                time.sleep(0.01)

            for cfg in streams:
                name = cfg['name']
                if name not in queues:
                    continue
                if name not in captured:
                    print_result(False, f'Resolution {name}: no frame captured')
                    all_pass = False
                    continue
                frame = captured[name]
                w, h  = frame.getWidth(), frame.getHeight()
                res_ok = (w == cfg['width'] and h == cfg['height'])
                print_result(res_ok, f'Resolution {name}: {w}×{h}  (expected {cfg["width"]}×{cfg["height"]})')
                if not res_ok:
                    all_pass = False

            pipeline.stop()

        except Exception as e:
            stop_spin.set()
            spin_thread.join()
            print_result(False, f'Camera check error: {e}')
            all_pass = False
        finally:
            try:
                device.close()
            except Exception:
                pass

    return all_pass

# ==============================================================================
# Server lifecycle helpers
# ==============================================================================

def _kill_server():
    click.secho('  Stopping robot server...', fg='yellow')
    ret = subprocess.call(
        ['stretch_body_server', '--kill'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    if ret == 0:
        click.secho('  Server stopped.', fg='yellow')
    else:
        click.secho(f'  Server stop may have failed (exit code {ret}).', fg='yellow')
    return ret == 0


def _restart_server():
    click.secho('  Restarting robot server...', fg='yellow')
    subprocess.Popen(
        ['stretch_body_server', '--restart'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    click.secho('  Server restarting in background.', fg='yellow')


# ==============================================================================
# Main
# ==============================================================================

_ALL_CHECKS = [
    'USB Devices', 'Firmware',
    'Power/Battery', 'ESP32', 'Line Sensors',
    'Eye LEDs', 'Calibrations',
    'OmniBase', 'Arm', 'Lift', 'End-of-Arm', 'IMU',
]
_REQUIRE_SERVER = {
    'Power/Battery', 'ESP32', 'OmniBase', 'Arm', 'Lift', 'End-of-Arm', 'IMU'
}


def main():
    global r
    results = {}

    if args.firmware:
        click.secho('\n---- Firmware Check Mode ----', fg='cyan', bold=True)
        _kill_server()
        results['USB Devices'] = check_usb_devices()
        results['Firmware']    = check_firmware_versions()
        _restart_server()

        print_section('Summary')
        all_pass = True
        for name in ['USB Devices', 'Firmware']:
            passed = results.get(name)
            print_result(passed, name)
            if not passed:
                all_pass = False
        click.echo()
        click.secho('All firmware checks PASSED.' if all_pass else 'One or more firmware checks FAILED.',
                    fg='green' if all_pass else 'red', bold=True)
        sys.exit(0 if all_pass else 1)

    if args.sensors:
        results['Sensors'] = check_sensors()
        print_section('Summary')
        all_pass = results['Sensors']
        print_result(all_pass, 'Sensors')
        click.echo()
        click.secho('All sensor checks PASSED.' if all_pass else 'One or more sensor checks FAILED.',
                    fg='green' if all_pass else 'red', bold=True)
        sys.exit(0 if all_pass else 1)

    # Full system check
    print_software_versions()
    results['USB Devices'] = check_usb_devices()
    results['Firmware']    = None  # requires --firmware
    results['Calibrations'] = check_calibrations()

    if args.direct:
        from stretch4_body.robot.robot import Robot
        r = Robot()
    else:
        from stretch4_body.robot.robot_client import RobotClient
        r = RobotClient()

    _old_stdout, _old_stderr = sys.stdout, sys.stderr
    sys.stdout = sys.stderr = io.StringIO()
    server_online = r.startup()
    sys.stdout, sys.stderr = _old_stdout, _old_stderr

    if not server_online:
        click.secho(
            '\n[WARN] Could not connect to robot server — live hardware checks skipped.\n'
            '       Launch the server first:  stretch_body_server --launch\n',
            fg='yellow'
        )
        r = None
    else:
        r.pull_status()

    results['Line Sensors'] = check_line_sensors()
    results['Eye LEDs']     = check_eye_animations()

    if r is not None:
        results['Power/Battery'] = check_power_periph()
        results['ESP32']         = check_esp32()
        results['OmniBase']      = check_omnibase()
        results['Arm']           = check_arm()
        results['Lift']          = check_lift()
        results['End-of-Arm']    = check_end_of_arm()
        results['IMU']           = check_imu()
    else:
        for name in _REQUIRE_SERVER:
            results[name] = None

    print_section('Summary')
    all_pass = True
    for name in _ALL_CHECKS:
        passed = results.get(name)
        if passed is None:
            skip_reason = 'run with --firmware to check' if name == 'Firmware' else 'server offline'
            click.secho(f'  [SKIP] {name} ({skip_reason})', fg='yellow')
        else:
            print_result(passed, name)
            if not passed:
                all_pass = False

    click.echo()
    click.secho('All checks PASSED.' if all_pass else 'One or more checks FAILED.',
                fg='green' if all_pass else 'red', bold=True)

    if r is not None:
        r.stop()
    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
