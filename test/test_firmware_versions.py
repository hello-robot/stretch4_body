"""status['firmware_versions']: one row per board, built from what the server already holds."""
import types

from stretch4_body.robot import firmware_versions as fv

T0 = 1_760_000_000.0


def stepper(usb, version='hello-stepper.v0.7.2p9', hw_valid=True):
    return types.SimpleNamespace(usb=usb, hw_valid=hw_valid, board_info={
        'board_variant': 'Stepper.Irving.V2', 'firmware_version': version,
        'protocol_version': 'p9', 'hardware_id': 2, 'stepper_type': 0})


def schema3_diag(state='reported', version='hello-esp.v0.1.11p0', requests=1):
    return {'_valid': True, 'schema_version': 3, 'esp_version_state_name': state,
            'esp_version': version if state == 'reported' else '',
            'esp_version_requests': requests}


def fake_robot(diag=None, eoa=None, line_sensor=None, server_subsystems=()):
    pp = types.SimpleNamespace(
        transport=types.SimpleNamespace(port_name='/dev/hello-power-periph'),
        board_info={'board_variant': 'PowerPeriph.V3', 'firmware_version': 'hello-pimu2.v0.1.53p14',
                    'protocol_version': 'p14', 'hardware_id': 3},
        hw_valid=True, power_diagnostics=diag if diag is not None else schema3_diag())
    subsystems = {
        'power_periph': pp,
        'lift': types.SimpleNamespace(motor=stepper('/dev/hello-motor-lift')),
        'arm': types.SimpleNamespace(motor=stepper('/dev/hello-motor-arm', hw_valid=False,
                                                   version=None)),
        'omnibase': types.SimpleNamespace(wheels=[stepper('/dev/hello-motor-omni-%d' % i)
                                                  for i in range(3)]),
    }
    status = {'power_periph': {'power_diagnostics_meta': {'sample_age_s': 0.5,
                                                          'firmware_version': 'hello-pimu2.v0.1.53p14'}}}
    if eoa is not None:
        subsystems['end_of_arm'] = object()
        status['end_of_arm'] = eoa
    if line_sensor is not None:
        status['line_sensor_loop'] = line_sensor
    return types.SimpleNamespace(
        get_subsystem=subsystems.get, status=status,
        params={'server': {'subsystems': list(server_subsystems)}},
        robot_params={'end_of_arm': {'usb_name': '/dev/hello-feetech-wrist'},
                      'wrist_yaw': {'id': 11}, 'gripper': {'id': 14}})


def by_board(rows):
    return {r['board']: r for r in rows}


def test_every_board_has_a_row_in_order():
    rows = fv.collect(fake_robot(), T0, now=T0 + 10)
    assert [r['board'] for r in rows] == [
        'pimu', 'esp', 'stepper lift', 'stepper arm', 'stepper wheel 0', 'stepper wheel 1',
        'stepper wheel 2', 'line sensor']
    for r in rows:
        assert set(r) == {'board', 'port', 'version', 'detail', 'source', 'time', 'reason'}
        assert (r['version'] is None) == (r['reason'] is not None), r


def test_pimu_and_steppers_come_from_board_info_at_startup():
    rows = by_board(fv.collect(fake_robot(), T0, now=T0 + 10))
    assert rows['pimu']['version'] == 'hello-pimu2.v0.1.53p14'
    assert rows['pimu']['port'] == '/dev/hello-power-periph'
    assert rows['pimu']['detail']['protocol_version'] == 'p14'
    assert rows['pimu']['time'] == T0
    assert rows['stepper lift']['version'] == 'hello-stepper.v0.7.2p9'
    assert rows['stepper wheel 2']['port'] == '/dev/hello-motor-omni-2'
    assert rows['stepper arm']['version'] is None
    assert rows['stepper arm']['reason'] == 'not connected'
    assert rows['stepper arm']['time'] is None


def test_esp_reported_has_the_sample_time():
    r = by_board(fv.collect(fake_robot(), T0, now=T0 + 10))['esp']
    assert r['version'] == 'hello-esp.v0.1.11p0'
    assert r['time'] == T0 + 9.5
    assert r['port'] == 'via /dev/hello-power-periph'


def test_esp_reasons():
    meta = {'firmware_version': 'hello-pimu2.v0.1.50p14'}
    cases = [
        ({'_valid': False, '_decode_error': None}, 'no PIMU diagnostics sample'),
        ({'_valid': True, 'schema_version': 2}, 'needs hello-pimu2.v0.1.53p14'),
        (schema3_diag('no_reply', requests=5), 'older than hello-esp.v0.1.11p0'),
        (schema3_diag('waiting', requests=2), 'the PIMU is asking the ESP'),
    ]
    for diag, words in cases:
        r = fv.esp_row('via p', diag, meta, T0)
        assert r['version'] is None and words in r['reason'], (diag, r['reason'])
    assert 'hello-pimu2.v0.1.50p14' in fv.esp_row('p', cases[1][0], meta, T0)['reason']


def test_end_of_arm_servos_with_bus_and_id():
    eoa = {'joint_names': ['wrist_yaw', 'gripper'],
           'wrist_yaw': {'firmware': {'version': '3.10', 'model': 777, 'read_time': T0 + 3,
                                      'error': None}},
           'gripper': {'firmware': {'version': None, 'model': None, 'read_time': None,
                                    'error': 'read failed: timeout'}}}
    rows = by_board(fv.collect(fake_robot(eoa=eoa), T0, now=T0 + 10))
    yaw = rows['servo wrist_yaw']
    assert yaw['version'] == '3.10' and yaw['detail'] == {'model': 777}
    assert yaw['port'] == '/dev/hello-feetech-wrist id 11'
    assert yaw['time'] == T0 + 3
    assert rows['servo gripper']['reason'] == 'read failed: timeout'


def test_end_of_arm_child_not_started_yet():
    rows = by_board(fv.collect(fake_robot(eoa={'joint_names': ['gripper'], 'gripper': {}}),
                               T0, now=T0))
    assert rows['servo gripper']['reason'] == 'end of arm not started'


def test_line_sensor_states():
    off = by_board(fv.collect(fake_robot(), T0))['line sensor']
    assert off['reason'] == 'line_sensor_loop is not in server.subsystems'
    for state, words in (('no_reply', 'older than hello-pixart-j3.v0.1.1p0'),
                         ('waiting', 'no answer yet'), ('not_open', 'not open')):
        ls = {'firmware': {'state': state, 'version': None}}
        r = by_board(fv.collect(fake_robot(line_sensor=ls,
                                           server_subsystems=['line_sensor_loop']), T0))
        assert words in r['line sensor']['reason']
    ls = {'firmware': {'state': 'reported', 'version': 'hello-pixart-j3.v0.1.2p1',
                       'uptime_ms': 5000, 'reset_cause_raw': 64, 'received_time': T0 + 2}}
    r = by_board(fv.collect(fake_robot(line_sensor=ls, server_subsystems=['line_sensor_loop']),
                            T0))['line sensor']
    assert r['version'] == 'hello-pixart-j3.v0.1.2p1'
    assert r['detail'] == {'uptime_ms': 5000, 'reset_cause_raw': 64}
    assert r['time'] == T0 + 2
