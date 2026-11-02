#!/usr/bin/env python3
"""
Health checks for Stretch 4.

These are plain functions shared by the HealthManager (which runs them continuously
inside stretch_body_server) and stretch_system_check (which renders their results).
Keeping one implementation means the CLI and the server can't disagree about what
"healthy" means.

Every check returns a list of result dicts:

    {'id':      'power_periph.battery_soc',   # unique, stable
     'group':   'power_periph',               # which section it belongs to
     'level':   'OK' | 'INFO' | 'WARN' | 'FAIL' | 'STALE',
     'message': 'Battery SOC = 100%',          # human readable, same text stretch_system_check prints
     'value':   100,                           # the measured value, if any
     'detail':  ['...'],                       # optional extra lines, shown with --verbose
     'subgroup': 'Steppers'}                   # optional label to group results within a section

Status checks take the robot status dict (the same one the server publishes) and are
cheap enough to run in the control loop. Host checks touch the filesystem and run in
a background process on the server.

Health only reports. Nothing here gates commands.
"""
import os
import time

OK = 'OK'
INFO = 'INFO'   # A state worth knowing (not homed, runstop on) that isn't a problem with the robot
WARN = 'WARN'
FAIL = 'FAIL'
STALE = 'STALE'

# INFO ranks with OK: it never makes the robot less than healthy
LEVEL_RANK = {OK: 0, INFO: 0, WARN: 1, STALE: 2, FAIL: 3}

# Groups, in the order stretch_system_check shows them
GROUP_USB = 'usb'
GROUP_CALIBRATION = 'calibration'
GROUP_LINE_SENSOR = 'line_sensor'
GROUP_POWER = 'power_periph'
GROUP_ESP32 = 'esp32'
GROUP_OMNIBASE = 'omnibase'
GROUP_ARM = 'arm'
GROUP_LIFT = 'lift'
GROUP_END_OF_ARM = 'end_of_arm'
GROUP_IMU = 'imu'
GROUP_SERVER = 'server'

HOST_GROUPS = (GROUP_USB, GROUP_CALIBRATION)

# Defaults for the 'health_manager' params. Robot or user params override any of these.
DEFAULT_PARAMS = {
    'enabled': 1,
    'check_rate_Hz': 2.0,
    'host_check_period_s': 10.0,
    'battery_soc_warn': 20.0,
    'battery_soc_fail': 10.0,
    'battery_soh_warn': 75.0,
    'voltage_range': [20.0, 30.0],
    'voltage_cpu_range': [15.0, 30.0],
    'voltage_12v0_range': [10.0, 14.0],
    'voltage_5v0_range': [4.5, 5.5],
    'board_temp_range': [0.0, 80.0],
    'cpu_temp_warn': 90.0,
    'imu_az_range': [7.0, 11.0],
    'arm_pos_range': [-0.01, 0.56],
    'lift_pos_range': [-0.01, 1.12],
    'eoa_temp_max': 70.0,
    'eoa_stale_s': 1.0,
    'line_sensor_min_hz': 25.0,
    'line_sensor_rate_window_s': 2.0,
    'loop_rate_min_fraction': 0.9,
    'counter_window_s': 30.0,       # climbing counters are judged over this window
    'missed_loops_warn': 10,        # control loop overruns per window before warning
}

# USB devices, in display order, with the subsystem that needs each one.
# None means the device is expected on every robot.
USB_DEVICES = [
    ('hello-motor-arm', 'arm'),
    ('hello-motor-lift', 'lift'),
    ('hello-motor-omni-0', 'omnibase'),
    ('hello-motor-omni-1', 'omnibase'),
    ('hello-motor-omni-2', 'omnibase'),
    ('hello-power-periph', 'power_periph'),
    ('hello-feetech-wrist', 'end_of_arm'),
    ('hello-esp32', None),
    ('hello-nav-head-camera-stereo', None),
    ('hello-pixart-j3', None),
]

STEPPER_CALIBRATIONS = [
    ('hello-motor-arm', 'arm'),
    ('hello-motor-lift', 'lift'),
    ('hello-motor-omni-0', 'omnibase'),
    ('hello-motor-omni-1', 'omnibase'),
    ('hello-motor-omni-2', 'omnibase'),
]


def merged_params(robot_params):
    """DEFAULT_PARAMS overlaid with robot_params['health_manager']."""
    p = dict(DEFAULT_PARAMS)
    p.update((robot_params or {}).get('health_manager', {}) or {})
    return p


def result(group, id, level, message, value=None, detail=None, subgroup=None):
    r = {'id': f'{group}.{id}', 'group': group, 'level': level, 'message': message, 'value': value}
    if detail:
        r['detail'] = list(detail)
    if subgroup:
        r['subgroup'] = subgroup
    return r


def short_text(r):
    """One-line description of a result for summaries, readable without its section title."""
    if r['level'] != OK and r['group'] == GROUP_USB:
        return f"{r['message']} missing"
    if r['level'] != OK and r['group'] == GROUP_CALIBRATION:
        return f"{r.get('subgroup', 'Calibration')} calibration: {r['message']}"
    return r['message']


def worst_level(results):
    level = OK
    for r in results:
        if LEVEL_RANK.get(r['level'], 0) > LEVEL_RANK[level]:
            level = r['level']
    return level


def group_passed(results):
    """A group passes unless it has a FAIL or STALE result. WARNs don't fail a group."""
    return LEVEL_RANK[worst_level(results)] < LEVEL_RANK[STALE]


def _range(label, val, vmin, vmax):
    """stretch_system_check's HTML report parses this wording (_RE_RANGE), so keep it."""
    ok = vmin <= val <= vmax
    return ok, f'{label} = {val:.3f} (range [{vmin:.2f}, {vmax:.2f}])'


def _range_result(group, id, label, val, rng):
    ok, msg = _range(label, val, rng[0], rng[1])
    return result(group, id, OK if ok else FAIL, msg, val)


# ##################################################################################
# Status checks: computed from the robot status dict
# ##################################################################################

def check_power_periph(status, params):
    g = GROUP_POWER
    ps = status.get('power_periph') or {}
    if not ps:
        return [result(g, 'status', FAIL, 'Power periph status not available')]
    out = []

    soc = ps.get('battery_soc', 0)
    if soc >= params['battery_soc_warn']:
        out.append(result(g, 'battery_soc', OK, f'Battery SOC = {soc:.0f}%', soc))
    elif soc >= params['battery_soc_fail']:
        out.append(result(g, 'battery_soc', WARN, f'Battery SOC = {soc:.0f}% (low — consider charging)', soc))
    else:
        out.append(result(g, 'battery_soc', FAIL, f'Battery SOC = {soc:.0f}% (critically low!)', soc))

    soh = ps.get('battery_soh', 0)
    if soh >= params['battery_soh_warn']:
        out.append(result(g, 'battery_soh', OK, f'Battery SOH = {soh:.1f}%', soh))
    else:
        out.append(result(g, 'battery_soh', WARN, f'Battery SOH = {soh:.1f}% (degraded battery)', soh))

    for label, key in [('Bus Voltage (V)', 'voltage'),
                       ('CPU Voltage (V)', 'voltage_cpu'),
                       ('12V Rail (V)', 'voltage_12v0'),
                       ('5V Rail (V)', 'voltage_5v0')]:
        val = ps.get(key, 0)
        if val > 0:  # 0 means the board doesn't report this rail
            out.append(_range_result(g, key, label, val, params[f'{key}_range']))

    if ps.get('runstop_event', False):
        out.append(result(g, 'runstop', INFO, 'Runstop is active', True))
    else:
        out.append(result(g, 'runstop', OK, 'Runstop not active', False))

    out.append(_range_result(g, 'board_temp', 'Board Temp (°C)', ps.get('temp', 0), params['board_temp_range']))

    cpu_temp = ps.get('cpu_temp')
    if cpu_temp:
        level = OK if cpu_temp < params['cpu_temp_warn'] else WARN
        out.append(result(g, 'cpu_temp', level, f'CPU Temp = {cpu_temp:.0f}°C', cpu_temp))

    # The extras stretch_system_check shows at the end of the section with --verbose
    out[-1]['detail'] = out[-1].get('detail', []) + [
        f'Battery Current  : {ps.get("battery_current", 0):.2f} A',
        f'Adapter present  : {ps.get("adapter_voltage_present", False)}',
        f'Adapter fault    : {ps.get("adapter_fault", False)}',
        f'Charging         : {ps.get("charger_is_charging", False)}',
    ]
    return out


def check_esp32(status, params):
    g = GROUP_ESP32
    present = os.path.exists('/dev/hello-esp32')
    ps = status.get('power_periph') or {}
    return [result(g, 'present', OK if present else FAIL, '/dev/hello-esp32 present', present,
                   detail=[f'Aux CPU on  : {ps.get("cpu_on_sts", False)}'] if present else None)]


def check_omnibase(status, params):
    g = GROUP_OMNIBASE
    obs = status.get('omnibase') or {}
    out = []
    for i in range(3):
        ws = obs.get(f'wheel_{i}') or {}
        pos = ws.get('pos')
        if pos is None:
            out.append(result(g, f'wheel_{i}', FAIL, f'omni-{i}: no status data'))
            continue
        detail = []
        if ws.get('effort_pct') is not None:
            detail.append(f'omni-{i} effort = {ws["effort_pct"]:.1f}%')
        if ws.get('vel') is not None:
            detail.append(f'omni-{i} vel    = {ws["vel"]:.3f} rad/s')
        out.append(result(g, f'wheel_{i}', OK, f'omni-{i}: pos = {pos:.3f} rad', pos, detail=detail))
    return out


def homed_result(group, id, name, homed):
    """Not homed is a state, not a problem: INFO. 'joint' lets summaries combine them."""
    if homed:
        r = result(group, id, OK, f'{name} is homed', True)
    else:
        r = result(group, id, INFO, f'{name} not homed', False)
    r['joint'] = name
    return r


def is_homing(status):
    """True while a homing routine runs. Joints then travel to their hardstops, outside normal ranges."""
    return 'home' in str((status.get('routines') or {}).get('active_routine', ''))


def _check_prismatic(group, label, status, rng):
    js = status.get(group) or {}
    homed = bool((js.get('motor') or {}).get('pos_calibrated', False))
    out = [homed_result(group, 'homed', label, homed)]
    pos = js.get('pos')
    if pos is None:
        out.append(result(group, 'pos', FAIL, f'{label} pos not available'))
    elif not homed or is_homing(status):
        # The position means nothing before homing, and homing drives past the normal range
        why = 'homing' if homed else 'not homed'
        out.append(result(group, 'pos', OK, f'{label} pos (m) = {pos:.3f} (range not checked: {why})', pos))
    else:
        out.append(_range_result(group, 'pos', f'{label} pos (m)', pos, rng))
    return out


def check_arm(status, params):
    return _check_prismatic(GROUP_ARM, 'Arm', status, params['arm_pos_range'])


def check_lift(status, params):
    return _check_prismatic(GROUP_LIFT, 'Lift', status, params['lift_pos_range'])


def check_end_of_arm(status, params, joints, now=None, needs_homing=None):
    """`joints` is the list of joint names in the tool's 'devices' params.
    `needs_homing` is the joints that must be homed (req_calibration); None means all of them."""
    g = GROUP_END_OF_ARM
    if not joints:
        return [result(g, 'joints', WARN, 'No joints defined in end-of-arm params')]
    eoa_s = status.get('end_of_arm') or {}
    now = time.time() if now is None else now
    out = []
    for jn in joints:
        js = eoa_s.get(jn) or {}
        if not js:
            out.append(result(g, jn, FAIL, f'{jn}: no status data'))
            continue
        pos = js.get('pos')
        if pos is None:
            out.append(result(g, jn, FAIL, f'{jn}: no position data'))
            continue
        temp = js.get('temp')
        hw_err = js.get('hardware_error', 0)
        temp_str = f'  temp={temp:.0f}°C' if temp is not None else ''
        err_str = f'  hw_error={hw_err}' if hw_err else ''
        msg = f'{jn}: pos={pos:.3f} rad{temp_str}{err_str}'
        ok = hw_err == 0 and (temp is None or temp < params['eoa_temp_max'])
        level = OK if ok else FAIL
        ts = js.get('timestamp_pc')
        if ok and ts and now - ts > params['eoa_stale_s']:
            level = STALE
            msg += f'  (no update for {now - ts:.1f} s)'
        detail = []
        if js.get('effort') is not None:
            detail.append(f'{jn} effort = {js["effort"]:.1f}%')
        if js.get('current_mA') is not None:
            detail.append(f'{jn} current = {js["current_mA"]:.1f} mA')
        out.append(result(g, jn, level, msg, pos, detail=detail))
        if needs_homing is None or jn in needs_homing:
            out.append(homed_result(g, f'{jn}_homed', jn, bool(js.get('pos_calibrated', False))))
    return out


def check_imu(status, params):
    g = GROUP_IMU
    imu = (status.get('power_periph') or {}).get('imu') or {}
    if not imu:
        return [result(g, 'status', FAIL, 'IMU status not available')]
    az = imu.get('az', 0)
    r = _range_result(g, 'az', 'IMU az (m/s²)', az, params['imu_az_range'])
    r['detail'] = [
        f'ax = {imu.get("ax", 0):.3f}  ay = {imu.get("ay", 0):.3f}  az = {az:.3f} m/s²',
        f'roll = {imu.get("roll", 0):.4f} rad  pitch = {imu.get("pitch", 0):.4f} rad',
        f'gravity_tilt = {imu.get("gravity_tilt", 0):.4f} rad',
    ]
    return [r]


def check_line_sensors(lss, sensor_names, rates, params, prev_counters=None):
    """
    `lss` is the line_sensor_loop status. `rates` maps sensor name to a measured
    frame rate in Hz (the server measures it from frame_id between steps, the CLI's
    direct path measures it with stretch_line_sensor_hz_check). `prev_counters` is the
    {counter: value} from about counter_window_s ago, so the server can flag counters that are climbing; when
    None, any nonzero total is reported as history (the CLI's behavior).
    """
    g = GROUP_LINE_SENSOR
    health = lss.get('health') or {}
    min_hz = params['line_sensor_min_hz']
    out = []

    port_open = bool(health.get('port_open', False))
    out.append(result(g, 'port_open', OK if port_open else FAIL,
                      'Serial port open (/dev/hello-pixart-j3)', port_open))
    if not health.get('streaming', False):
        out.append(result(g, 'streaming', WARN, 'Streaming is OFF — cliff detection is disabled', False))

    dead = list(health.get('sensors_dead', []))
    disabled = list(health.get('disabled_sensors', []))
    alive = [sn for sn in sensor_names if sn not in dead and sn not in disabled]
    out.append(result(g, 'sensors_reporting', FAIL if dead else OK,
                      f'{len(alive)}/{len(sensor_names)} sensors reporting'
                      + (f' — DEAD: {", ".join(dead)}' if dead else ''), len(alive)))
    if disabled:
        out.append(result(g, 'sensors_disabled', WARN,
                          f'DISABLED at runtime (not a fault): {", ".join(disabled)} — '
                          f'{len(disabled)} of {len(sensor_names)} sensors are not looking', disabled))
    else:
        out.append(result(g, 'sensors_disabled', OK,
                          f'All {len(sensor_names)} sensors enabled (none disabled)', []))

    # A rate is either a float (Hz) or a stretch_line_sensor_hz_check.measure() entry
    rates = {sn: (m if isinstance(m, dict) else {'advance_hz': m}) for sn, m in (rates or {}).items()}

    active = [sn for sn in sensor_names if sn not in disabled]
    if not active:
        out.append(result(g, 'frame_rate', WARN, 'Every sensor is disabled — nothing to time'))
    else:
        measured = [rates[sn]['advance_hz'] for sn in active if sn in rates]
        if measured:
            slowest = min(measured)
            out.append(result(g, 'frame_rate', OK if slowest >= min_hz else FAIL,
                              f'Frame rate {slowest:.1f} Hz on the slowest sensor (need >= {min_hz:.0f} Hz)',
                              slowest))

    for sn in sensor_names:
        s = lss.get(sn)
        if not isinstance(s, dict):
            out.append(result(g, sn, FAIL, f'{sn}: no status block'))
            continue
        if sn in disabled:
            continue
        m = rates.get(sn)
        hz = m['advance_hz'] if m else None
        missed = s.get('missed_frames', 0)
        good = sn not in dead and (hz is None or hz >= min_hz) and not (m or {}).get('backwards')
        if sn in dead:
            msg = f'{sn}: dead'
        elif hz is not None:
            msg = f'{sn}: {hz:.1f} Hz'
        else:
            msg = f'{sn}: reporting'
        if m and m.get('skips') and m.get('fresh_hz', 0.0) >= 0.9 * hz:
            msg += f', dropped {m["skips"]}x (longest gap {m["max_gap"]} frames)'
        if missed:
            msg += f', missed {missed} frames'
        if m and m.get('backwards'):
            msg += f', frame_id went BACKWARDS {m["backwards"]}x'
        out.append(result(g, sn, OK if good else FAIL, msg, hz))

    for key, what in [('frame_not_full_err', 'incomplete frames — a sensor dropped out of frames'),
                      ('reader_restarts', 'serial port self-recoveries — suspect a flaky cable'),
                      ('decode_errors', 'decode errors')]:
        total = health.get(key, 0)
        if prev_counters is None:
            if total:
                out.append(result(g, key, WARN, f'{total} {what} since startup', total))
        else:
            rise = total - prev_counters.get(key, total)
            window = params['counter_window_s']
            if rise > 0:
                out.append(result(g, key, WARN, f'{rise} {what} in the last {window:.0f} s ({total} since startup)', total))
            else:
                out.append(result(g, key, OK, f'No {what.split(" —")[0]} in the last {window:.0f} s ({total} since startup)', total))

    cal = lss.get('calibration') or {}
    loaded, rejected = cal.get('loaded', []), cal.get('rejected', {})
    detail = [f'{name}: NO TARE ({str(why).split(":")[0]})' for name, why in sorted(rejected.items())]
    out.append(result(g, 'tares', OK if len(loaded) == len(sensor_names) else FAIL,
                      f'Calibration: {len(loaded)}/{len(sensor_names)} tares loaded', len(loaded),
                      detail=detail))
    return out


def check_server(status, params, prev=None, processes=None):
    """
    Control loop rate, loop overruns and worker process liveness.
    `prev` is the server status from about counter_window_s ago, to judge overruns over that window.
    `processes` maps a name to whether that worker process is alive.
    """
    g = GROUP_SERVER
    out = []
    srv = status.get('server') or {}
    cl = srv.get('control_loop') or {}
    target = cl.get('target_rate_hz')
    avg = cl.get('avg_rate_hz')
    warmed_up = cl.get('num_loops', 0) >= 2 * (target or 0)  # avg rate is meaningless at first
    if target and avg is not None and warmed_up:
        floor = params['loop_rate_min_fraction'] * target
        out.append(result(g, 'loop_rate', OK if avg >= floor else WARN,
                          f'Control loop rate = {avg:.1f} Hz (target {target} Hz)', avg))
    missed = cl.get('missed_loops')
    if missed is not None and prev is not None:
        rise = missed - ((prev.get('control_loop') or {}).get('missed_loops', missed))
        window = params['counter_window_s']
        out.append(result(g, 'missed_loops', WARN if rise > params['missed_loops_warn'] else OK,
                          f'{rise} loop overruns in the last {window:.0f} s ({missed} total)', missed))
    for name, alive in (processes or {}).items():
        out.append(result(g, f'process_{name}', OK if alive else FAIL,
                          f'{name} process {"running" if alive else "NOT running"}', alive))
    return out


def run_status_checks(status, robot_params, params=None, eoa_joints=None,
                      line_sensor_rates=None, line_sensor_prev=None,
                      server_prev=None, processes=None, now=None):
    """
    Run every status check that applies to the subsystems in robot_params.
    Each check is isolated: one that raises becomes a WARN, the others still run.
    """
    params = params or merged_params(robot_params)
    robot = robot_params.get('robot', {})
    subsystems = robot.get('subsystems', [])
    server_subsystems = (robot.get('server') or {}).get('subsystems', []) or []
    if eoa_joints is None:
        eoa_joints = list((robot_params.get(robot.get('tool'), {}) or {}).get('devices', {}).keys())
    eoa_needs_homing = [jn for jn in eoa_joints if (robot_params.get(jn) or {}).get('req_calibration', 1)]

    checks = []
    if 'line_sensor_loop' in server_subsystems and 'line_sensor_loop' in status:
        sensor_names = robot_params.get('line_sensor_loop', {}).get('sensor_names', [])
        checks.append((GROUP_LINE_SENSOR, lambda: check_line_sensors(
            status['line_sensor_loop'], sensor_names, line_sensor_rates, params, line_sensor_prev)))
    if 'power_periph' in subsystems:
        checks.append((GROUP_POWER, lambda: check_power_periph(status, params)))
        checks.append((GROUP_ESP32, lambda: check_esp32(status, params)))
    if 'omnibase' in subsystems:
        checks.append((GROUP_OMNIBASE, lambda: check_omnibase(status, params)))
    if 'arm' in subsystems:
        checks.append((GROUP_ARM, lambda: check_arm(status, params)))
    if 'lift' in subsystems:
        checks.append((GROUP_LIFT, lambda: check_lift(status, params)))
    if 'end_of_arm' in subsystems:
        checks.append((GROUP_END_OF_ARM, lambda: check_end_of_arm(status, params, eoa_joints, now, eoa_needs_homing)))
    if 'power_periph' in subsystems:
        checks.append((GROUP_IMU, lambda: check_imu(status, params)))
    if 'server' in status:
        checks.append((GROUP_SERVER, lambda: check_server(status, params, server_prev, processes)))
    return _run_isolated(checks)


def _run_isolated(checks):
    out = []
    for group, fn in checks:
        try:
            out.extend(fn())
        except Exception as e:
            out.append(result(group, 'check_error', WARN, f'{group} health check raised {type(e).__name__}: {e}'))
    return out


# ##################################################################################
# Host checks: filesystem only, run off the control loop
# ##################################################################################

def check_usb_devices(robot_params, dev_dir='/dev'):
    g = GROUP_USB
    subsystems = robot_params.get('robot', {}).get('subsystems', [])
    try:
        present = set(os.listdir(dev_dir))
    except OSError:
        present = set()
    out = []
    for dev, needed_by in USB_DEVICES:
        if needed_by is not None and needed_by not in subsystems:
            continue
        ok = dev in present
        out.append(result(g, dev, OK if ok else FAIL, f'/dev/{dev}', ok))
    return out


def calibration_root(robot_params):
    fleet_path = os.environ.get('HELLO_FLEET_PATH', os.path.expanduser('~/stretch_user'))
    fleet_id = os.environ.get('HELLO_FLEET_ID', robot_params.get('robot', {}).get('serial_no', ''))
    return os.path.join(fleet_path, fleet_id)


def check_calibrations(robot_params, cal_root=None, check_camera_intrinsics=True):
    g = GROUP_CALIBRATION
    cal_root = cal_root or calibration_root(robot_params)
    subsystems = robot_params.get('robot', {}).get('subsystems', [])
    out = []

    stepper_dir = os.path.join(cal_root, 'calibration_steppers')
    files = os.listdir(stepper_dir) if os.path.isdir(stepper_dir) else []
    for m, needed_by in STEPPER_CALIBRATIONS:
        if needed_by not in subsystems:
            continue
        ok = any(f.startswith(m + '_') and f.endswith('.yaml') for f in files)
        out.append(result(g, m, OK if ok else FAIL, m, ok, subgroup='Steppers'))

    if check_camera_intrinsics:
        out.extend(_check_camera_intrinsics())
    cam_dir = os.path.join(cal_root, 'calibration_cameras')
    ok = os.path.isfile(os.path.join(cam_dir, 'camera_extrinsics.yaml'))
    out.append(result(g, 'camera_extrinsics', OK if ok else FAIL, 'extrinsics', ok, subgroup='Cameras'))

    from stretch4_body.subsystem.line_sensor import calibration_store
    ls_dir = os.path.join(cal_root, 'calibration_line_sensors')
    ls_names = robot_params.get('line_sensor_loop', {}).get('sensor_names', [f'sensor_{i}' for i in range(6)])
    tare_dir = calibration_store.tare_dir(ls_dir)
    if os.path.isdir(tare_dir):
        for name in ls_names:
            ok = os.path.isfile(calibration_store.tare_path(ls_dir, name))
            out.append(result(g, f'tare_{name}', OK if ok else FAIL, name, ok, subgroup='Line Sensors'))
    else:
        out.append(result(g, 'tare_dir', FAIL, f'tare directory missing: {tare_dir}', False, subgroup='Line Sensors'))

    hesai_dir = os.path.join(cal_root, 'calibration_hesais')
    for side in ('left', 'right'):
        ok = os.path.isfile(os.path.join(hesai_dir, f'{side}_lidar_calibration.dat'))
        out.append(result(g, f'lidar_{side}', OK if ok else FAIL, f'{side} lidar', ok, subgroup='Lidars'))

    imu_dir = os.path.join(cal_root, 'calibration_omnibase_imu')
    ok = os.path.isdir(imu_dir) and any(os.listdir(imu_dir))
    out.append(result(g, 'omnibase_imu', OK if ok else FAIL, 'IMU calibration', ok, subgroup='OmniBase'))
    return out


def _check_camera_intrinsics():
    """Load each camera's calibration the way the rest of the codebase does: checks the
    entry exists, parses, and matches the camera's configured size."""
    from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
    out = []
    for id, label, camera_type in (
        ('intrinsics_left', 'intrinsics: left', RGBCameras.left()),
        ('intrinsics_right', 'intrinsics: right', RGBCameras.right()),
        ('intrinsics_center', 'intrinsics: center', RGBCameras.center()),
        ('intrinsics_gripper_left', 'intrinsics: gripper left', RGBCameras.gripper_left),
        ('intrinsics_gripper_right', 'intrinsics: gripper right', RGBCameras.gripper_right),
    ):
        try:
            c = camera_type.load_calibration()
            out.append(result(GROUP_CALIBRATION, id, OK, f'{label}  ({c.width}x{c.height})', True, subgroup='Cameras'))
        except Exception as e:
            out.append(result(GROUP_CALIBRATION, id, FAIL, label, False,
                              detail=[str(e).strip()], subgroup='Cameras'))
    return out


def run_host_checks(robot_params, check_camera_intrinsics=True):
    return _run_isolated([
        (GROUP_USB, lambda: check_usb_devices(robot_params)),
        (GROUP_CALIBRATION, lambda: check_calibrations(robot_params, check_camera_intrinsics=check_camera_intrinsics)),
    ])
