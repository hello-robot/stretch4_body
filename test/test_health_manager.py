"""Unit tests for the health checks and HealthManager. No hardware or server needed.

    python3 -m pytest test/test_health_manager.py
"""
import copy
import logging
import time

import pytest

from stretch4_body.behavior.health import health_checks as hc
from stretch4_body.behavior.health.health_manager import HealthManager

ROBOT_PARAMS = {
    'robot': {'subsystems': ['power_periph', 'arm', 'lift', 'omnibase', 'end_of_arm'],
              'server': {'subsystems': ['line_sensor_loop']},
              'tool': 'eoa_test', 'serial_no': 'stretch-test'},
    'eoa_test': {'devices': {'wrist_yaw': {}, 'wrist_pitch': {}}},
    'line_sensor_loop': {'sensor_names': ['sensor_0', 'sensor_1']},
}
PARAMS = hc.merged_params(ROBOT_PARAMS)


def healthy_status(now=None):
    now = time.time() if now is None else now
    joint = {'pos': 0.1, 'temp': 40.0, 'hardware_error': 0, 'pos_calibrated': 1,
             'timestamp_pc': now, 'effort': 0.0, 'current_mA': 0.0}
    sensor = {'frame_id': 100, 'missed_frames': 0}
    return {
        'power_periph': {'battery_soc': 90, 'battery_soh': 99, 'voltage': 27.0, 'voltage_cpu': 19.5,
                         'voltage_12v0': 11.7, 'voltage_5v0': 5.0, 'temp': 40.0, 'cpu_temp': 60.0,
                         'runstop_event': False, 'imu': {'az': 9.8}},
        'arm': {'pos': 0.1, 'motor': {'pos_calibrated': True}},
        'lift': {'pos': 0.5, 'motor': {'pos_calibrated': True}},
        'omnibase': {f'wheel_{i}': {'pos': 0.0, 'vel': 0.0, 'effort_pct': 0.0} for i in range(3)},
        'end_of_arm': {'wrist_yaw': dict(joint), 'wrist_pitch': dict(joint)},
        'line_sensor_loop': {
            'health': {'port_open': True, 'streaming': True, 'sensors_dead': [], 'disabled_sensors': [],
                       'reader_restarts': 0, 'decode_errors': 0, 'frame_not_full_err': 0},
            'sensor_0': dict(sensor), 'sensor_1': dict(sensor),
            'calibration': {'loaded': ['sensor_0', 'sensor_1'], 'rejected': {}}},
        'server': {'control_loop': {'target_rate_hz': 100, 'avg_rate_hz': 100.5, 'missed_loops': 3,
                                    'num_loops': 10000}},
    }


def by_id(results):
    return {r['id']: r for r in results}


def run(status, **kw):
    return by_id(hc.run_status_checks(status, ROBOT_PARAMS, params=PARAMS, **kw))


# ############################## health_checks ##############################

def test_healthy_robot_is_all_ok():
    results = run(healthy_status(), line_sensor_rates={'sensor_0': 30.0, 'sensor_1': 30.0})
    bad = {id: r['message'] for id, r in results.items() if r['level'] != hc.OK}
    assert bad == {}
    assert hc.worst_level(results.values()) == hc.OK


@pytest.mark.parametrize('soc, level', [(90, hc.OK), (20, hc.OK), (15, hc.WARN), (10, hc.WARN), (5, hc.FAIL)])
def test_battery_soc_thresholds(soc, level):
    s = healthy_status()
    s['power_periph']['battery_soc'] = soc
    assert run(s)['power_periph.battery_soc']['level'] == level


def test_rail_out_of_range_fails_and_keeps_report_wording():
    s = healthy_status()
    s['power_periph']['voltage_5v0'] = 4.2
    r = run(s)['power_periph.voltage_5v0']
    assert r['level'] == hc.FAIL
    assert r['message'] == '5V Rail (V) = 4.200 (range [4.50, 5.50])'


def test_unreported_rail_is_skipped():
    s = healthy_status()
    s['power_periph']['voltage_12v0'] = 0
    assert 'power_periph.voltage_12v0' not in run(s)


def test_not_homed_and_runstop_are_info_not_problems():
    s = healthy_status()
    s['arm']['motor']['pos_calibrated'] = False
    s['power_periph']['runstop_event'] = True
    results = run(s)
    assert results['arm.homed']['level'] == hc.INFO
    assert results['power_periph.runstop']['level'] == hc.INFO
    assert hc.worst_level(results.values()) == hc.OK  # The robot is still healthy
    assert hc.group_passed([r for r in results.values() if r['group'] == hc.GROUP_ARM])


def test_end_of_arm_missing_joint_hw_error_and_stale():
    now = time.time()
    s = healthy_status(now)
    del s['end_of_arm']['wrist_pitch']
    s['end_of_arm']['wrist_yaw']['timestamp_pc'] = now - 5.0
    results = run(s, now=now)
    assert results['end_of_arm.wrist_pitch']['level'] == hc.FAIL
    assert results['end_of_arm.wrist_yaw']['level'] == hc.STALE
    assert not hc.group_passed([results['end_of_arm.wrist_yaw']])

    s = healthy_status(now)
    s['end_of_arm']['wrist_yaw']['hardware_error'] = 32
    assert run(s, now=now)['end_of_arm.wrist_yaw']['level'] == hc.FAIL


def test_line_sensor_counters_only_flag_when_climbing():
    s = healthy_status()
    s['line_sensor_loop']['health']['reader_restarts'] = 4
    prev = dict(s['line_sensor_loop']['health'])
    # Old history alone isn't a problem for the server...
    assert run(s, line_sensor_prev=prev)['line_sensor.reader_restarts']['level'] == hc.OK
    # ...but a counter that climbs is
    s['line_sensor_loop']['health']['reader_restarts'] = 5
    assert run(s, line_sensor_prev=prev)['line_sensor.reader_restarts']['level'] == hc.WARN
    # The CLI (no previous sample) reports any history
    assert run(s)['line_sensor.reader_restarts']['level'] == hc.WARN


def test_line_sensor_slow_and_dead():
    s = healthy_status()
    s['line_sensor_loop']['health']['sensors_dead'] = ['sensor_1']
    results = run(s, line_sensor_rates={'sensor_0': 10.0, 'sensor_1': 0.0})
    assert results['line_sensor.frame_rate']['level'] == hc.FAIL
    assert results['line_sensor.sensor_0']['level'] == hc.FAIL
    assert results['line_sensor.sensor_1']['message'] == 'sensor_1: dead'
    assert results['line_sensor.sensors_reporting']['level'] == hc.FAIL


def test_line_sensor_accepts_hz_check_measurements():
    rates = {sn: {'advance_hz': 30.0, 'fresh_hz': 30.0, 'skips': 2, 'max_gap': 3, 'backwards': 0}
             for sn in ('sensor_0', 'sensor_1')}
    r = run(healthy_status(), line_sensor_rates=rates)['line_sensor.sensor_0']
    assert r['level'] == hc.OK
    assert 'dropped 2x' in r['message']


def test_server_loop_rate_overruns_and_processes():
    s = healthy_status()
    s['server']['control_loop']['avg_rate_hz'] = 70.0
    s['server']['control_loop']['missed_loops'] = 20
    results = run(s, server_prev={'control_loop': {'missed_loops': 15}},
                  processes={'end_of_arm': False, 'self_collision': True})
    assert results['server.loop_rate']['level'] == hc.WARN
    assert results['server.missed_loops']['level'] == hc.OK  # a few overruns are normal
    assert run(s, server_prev={'control_loop': {'missed_loops': 5}})['server.missed_loops']['level'] == hc.WARN
    assert results['server.process_end_of_arm']['level'] == hc.FAIL
    assert results['server.process_self_collision']['level'] == hc.OK


def test_missing_subsystem_status_fails():
    s = healthy_status()
    del s['power_periph']
    results = run(s)
    assert results['power_periph.status']['level'] == hc.FAIL
    assert results['imu.status']['level'] == hc.FAIL


def test_loop_rate_not_judged_before_warm_up():
    s = healthy_status()
    s['server']['control_loop'].update(avg_rate_hz=0.0, num_loops=10)
    assert 'server.loop_rate' not in run(s)


def test_manager_counter_window(manager):
    srv = manager.robot.status['server']['control_loop']
    manager.step(force=True)
    srv['missed_loops'] += 50
    manager.step(force=True)
    assert manager.status['checks']['server.missed_loops']['level'] == hc.WARN
    # Once the burst is older than the window, it's OK again
    manager._counter_samples[-1] = (time.time() - 100, *manager._counter_samples[-1][1:])
    manager._counter_samples.popleft()
    manager.step(force=True)
    manager.step(force=True)
    assert manager.status['checks']['server.missed_loops']['level'] == hc.OK


def test_a_raising_check_is_contained():
    s = healthy_status()
    s['arm']['pos'] = 'not a number'  # makes the range check raise
    results = run(s)
    assert results['arm.check_error']['level'] == hc.WARN
    assert results['lift.pos']['level'] == hc.OK  # the others still ran


def test_subsystems_not_on_the_robot_are_not_checked():
    params = copy.deepcopy(ROBOT_PARAMS)
    params['robot']['subsystems'] = ['power_periph']
    params['robot']['server']['subsystems'] = []
    groups = {r['group'] for r in hc.run_status_checks(healthy_status(), params, params=PARAMS)}
    assert groups == {hc.GROUP_POWER, hc.GROUP_IMU, hc.GROUP_SERVER}


def test_usb_devices(tmp_path):
    for dev in ('hello-motor-arm', 'hello-power-periph'):
        (tmp_path / dev).touch()
    params = copy.deepcopy(ROBOT_PARAMS)
    params['robot']['subsystems'] = ['power_periph', 'arm']
    results = by_id(hc.check_usb_devices(params, dev_dir=str(tmp_path)))
    assert results['usb.hello-motor-arm']['level'] == hc.OK
    assert results['usb.hello-power-periph']['level'] == hc.OK
    assert results['usb.hello-esp32']['level'] == hc.FAIL  # expected on every robot
    assert 'usb.hello-motor-lift' not in results  # no lift subsystem


def test_calibrations(tmp_path):
    (tmp_path / 'calibration_steppers').mkdir()
    (tmp_path / 'calibration_steppers' / 'hello-motor-arm_2026.yaml').touch()
    results = by_id(hc.check_calibrations(ROBOT_PARAMS, cal_root=str(tmp_path), check_camera_intrinsics=False))
    assert results['calibration.hello-motor-arm']['level'] == hc.OK
    assert results['calibration.hello-motor-arm']['subgroup'] == 'Steppers'
    assert results['calibration.hello-motor-lift']['level'] == hc.FAIL
    assert results['calibration.tare_dir']['level'] == hc.FAIL


# ############################## HealthManager ##############################

class FakeRobot:
    def __init__(self, status):
        self.status = status


@pytest.fixture
def manager():
    m = HealthManager(FakeRobot(healthy_status()))
    m.robot_params = ROBOT_PARAMS
    m.hparams = dict(PARAMS)
    m.enabled = True
    return m  # startup() is not called, so no host check process is started


def test_manager_publishes_and_tracks_transitions(manager, caplog):
    caplog.set_level(logging.INFO, logger='health_manager')
    manager.step(force=True)
    assert manager.status['level'] == hc.OK
    assert manager.status['issues'] == []
    assert manager.summary() == 'OK'
    assert caplog.records == []  # Nothing logged while everything is OK

    manager.robot.status['power_periph']['battery_soc'] = 5
    manager.step(force=True)
    assert manager.status['level'] == hc.FAIL
    assert manager.status['issues'] == ['power_periph.battery_soc']
    assert manager.summary() == 'FAIL: Battery SOC = 5% (critically low!)'
    since = manager.status['checks']['power_periph.battery_soc']['since']
    errors = [rec for rec in caplog.records if rec.levelno == logging.ERROR]
    assert len(errors) == 1 and 'battery_soc' in errors[0].getMessage()

    # Same level again: no new log line, 'since' is kept
    manager.step(force=True)
    assert len([rec for rec in caplog.records if rec.levelno == logging.ERROR]) == 1
    assert manager.status['checks']['power_periph.battery_soc']['since'] == since

    manager.robot.status['power_periph']['battery_soc'] = 90
    manager.step(force=True)
    assert manager.status['level'] == hc.OK
    assert manager.status['checks']['power_periph.battery_soc']['count'] == 1
    assert any(rec.levelno == logging.INFO and '-> OK' in rec.getMessage() for rec in caplog.records)


def test_manager_status_dict_identity_is_stable(manager):
    published = manager.status  # RobotServer publishes this exact dict
    manager.step(force=True)
    manager.step(force=True)
    assert manager.status is published


def test_manager_throttles(manager):
    manager.step(force=True)
    ts = manager.status['ts_update']
    manager.step()  # Inside the 1/check_rate_Hz window: skipped
    assert manager.status['ts_update'] == ts
    manager._ts_last_step -= 1.0
    manager.step()
    assert manager.status['ts_update'] > ts


def test_manager_measures_line_sensor_rate(manager):
    lss = manager.robot.status['line_sensor_loop']
    t0 = time.time()
    for i in range(5):  # 30 frames per 0.5 s step = 60 Hz
        lss['sensor_0']['frame_id'] = 100 + 30 * i
        lss['sensor_1']['frame_id'] = 100 + 30 * i
        rates = manager._line_sensor_rates(t0 + 0.5 * i)
    assert rates['sensor_0'] == pytest.approx(60.0)


def test_manager_never_raises(manager):
    manager.robot.status = None  # Anything unexpected in the status
    manager.step(force=True)  # Must not raise into the control loop


def test_disabled_manager_does_nothing(manager):
    manager.enabled = False
    manager.step(force=True)
    assert manager.status['ts_update'] == 0.0
    assert manager.summary() == 'DISABLED'


def test_manager_summary_is_readable(manager, caplog):
    caplog.set_level(logging.INFO, logger='health_manager')
    st = manager.robot.status
    st['arm']['motor']['pos_calibrated'] = False
    st['lift']['motor']['pos_calibrated'] = False
    manager.step(force=True)
    assert manager.status['level'] == hc.OK
    assert manager.status['issues'] == []
    assert manager.status['notices'] == ['arm.homed', 'lift.homed']
    assert manager.summary() == 'OK (Not homed: Arm, Lift)'
    assert not [rec for rec in caplog.records if rec.levelno >= logging.WARNING]  # Not homed isn't a warning

    st['power_periph']['battery_soc'] = 15
    st['power_periph']['temp'] = 200
    manager.step(force=True)
    assert manager.status['issues'][0] == 'power_periph.board_temp'  # Worst first
    assert manager.summary().startswith('FAIL: Board Temp')
    assert 'Battery SOC = 15%' in manager.summary()
    assert manager.summary().endswith('; also Not homed: Arm, Lift')
    assert manager.summary(max_items=1) == ('FAIL: Board Temp (°C) = 200.000 (range [0.00, 80.00]); +1 more; '
                                          'also Not homed: Arm +1 more')


def test_short_text_names_host_results():
    usb = hc.result(hc.GROUP_USB, 'hello-motor-arm', hc.FAIL, '/dev/hello-motor-arm', False)
    cal = hc.result(hc.GROUP_CALIBRATION, 'camera_extrinsics', hc.FAIL, 'extrinsics', False, subgroup='Cameras')
    assert hc.short_text(usb) == '/dev/hello-motor-arm missing'
    assert hc.short_text(cal) == 'Cameras calibration: extrinsics'


def test_end_of_arm_joints_report_homing():
    s = healthy_status()
    jn = next(iter(s['end_of_arm']))
    s['end_of_arm'][jn]['pos_calibrated'] = 0
    results = run(s)
    assert results[f'end_of_arm.{jn}_homed']['level'] == hc.INFO
    assert results[f'end_of_arm.{jn}']['level'] == hc.OK
    # A joint that doesn't need homing (req_calibration 0) gets no homed result
    results = {r['id']: r for r in hc.check_end_of_arm(s, PARAMS, [jn], now=time.time(), needs_homing=[])}
    assert f'end_of_arm.{jn}_homed' not in results


def test_pos_range_not_checked_unhomed_or_homing():
    s = healthy_status()
    s['lift']['pos'] = 1.2  # Past the range: at the hardstop
    assert run(s)['lift.pos']['level'] == hc.FAIL
    s['routines'] = {'active_routine': 'routine_robot_home'}
    assert run(s)['lift.pos']['level'] == hc.OK
    s['routines'] = {'active_routine': 'routine_nop'}
    s['lift']['motor']['pos_calibrated'] = False
    assert run(s)['lift.pos']['level'] == hc.OK


def test_summary_combines_unhomed_joints(manager):
    st = manager.robot.status
    st['arm']['motor']['pos_calibrated'] = False
    for js in st['end_of_arm'].values():
        js['pos_calibrated'] = 0
    manager.step(force=True)
    assert manager.status['level'] == hc.OK
    assert manager.summary() == 'OK (Not homed: Arm, ' + ', '.join(st['end_of_arm']) + ')'


def test_esp32_is_a_host_check(tmp_path):
    assert hc.check_esp32(dev_dir=str(tmp_path))[0]['level'] == hc.FAIL
    (tmp_path / 'hello-esp32').touch()
    assert hc.check_esp32(dev_dir=str(tmp_path))[0]['level'] == hc.OK
    assert hc.GROUP_ESP32 in hc.HOST_GROUPS


def test_summary_combines_updates(manager):
    from stretch4_body.behavior.health import software_updates as su
    ev = {'pip': {'hello-robot-stretch4-body': ('2026.9.11', '2026.11.1')}, 'pypi_reachable': True,
          'repos': [{'name': 'stretch_ros2', 'status': su.GIT_BEHIND, 'detail': '3 commit(s) behind origin/main',
                     'command': 'git pull'}],
          'firmware': {}, 'firmware_reachable': False, 'firmware_command': None}
    manager._update_results = su.update_results(ev)
    manager.step(force=True)
    assert manager.status['level'] == hc.OK
    assert manager.summary() == 'OK (Updates available: hello-robot-stretch4-body, stretch_ros2)'


def test_summary_caps_a_long_group(manager):
    st = manager.robot.status
    st['arm']['motor']['pos_calibrated'] = False
    st['lift']['motor']['pos_calibrated'] = False
    for js in st['end_of_arm'].values():
        js['pos_calibrated'] = 0
    manager.step(force=True)
    assert manager.summary() == 'OK (Not homed: Arm, Lift, wrist_yaw +1 more)'
