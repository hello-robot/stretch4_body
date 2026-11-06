"""stretch_doctor: the rules table, each source parser, the report and --export.

The FRA4 case is the status read on 2026-11-05 19:47 while its button was red: 134 bad
BMS replies in 6.4 h, nine of them inside one minute, and the button change already
pushed out of the 64 held events. Nothing here opens a device or needs a server.
"""
import json
import os

from stretch4_body.tools import stretch_doctor as doctor

T0 = 1793936000.0


def event(sequence, event_id, t, data0=0, data1=0):
    return {'sequence': sequence, 'event_id': event_id, 'event_name': 'e%d' % event_id,
            'power_state': 0, 'power_state_name': 'STATE_ACTIVE', 'data0': data0, 'data1': data1,
            'estimated_wall_time_s': t}


def checks(level='ok'):
    return [{'id': 'bms_link', 'level': level, 'message': 'Last valid BMS sample 0.7 s ago.'}]


def status(events, crc_errors=0, valid=23000, dropped=0, health=None):
    return {
        'server': {'control_loop': {'num_loops': 61136, 'missed_loops': 235, 'avg_rate_hz': 103.3}},
        'power_periph': {
            'battery_soc': 96, 'battery_soh': 96,
            'power_diagnostics': {'uptime_ms': 6.4 * 3600 * 1000, 'bms_valid_replies': valid,
                                  'bms_crc_errors': crc_errors, 'bms_timeouts': 0,
                                  'bms_rx_overflows': 0, 'bms_protocol_errors': 0},
            'power_health': {'checks': health or checks()},
            'power_event_log': {'events': events, 'firmware_dropped_count': dropped},
        },
        'firmware_versions': {'boards': []},
    }


FRA4_BURST = [event(260 + i, 11, T0 + 6.0 * i, 1, 75) for i in range(9)]
FRA4 = status(FRA4_BURST + [event(300, 13, T0 + 600.0, 1, 11)], crc_errors=134, valid=22906,
              dropped=263)
HEALTHY = status([event(1, 1, T0, 1, 5)])


def findings_for(readings):
    return {f['reading']: f for f in doctor.examine(readings)}


def test_healthy_robot_has_no_findings():
    readings = doctor.status_readings(HEALTHY)
    found = doctor.examine(readings)
    assert doctor.headline(found) == doctor.OK
    assert readings['lamp_held'] == 0 and readings['lamp_unknown'] == 0
    text = doctor.render(found, 'robot', now=T0)
    assert 'Healthy' in text and '[ATTENTION]' not in text and '[WATCH]' not in text


def test_fra4_bms_judged_over_its_window():
    readings = doctor.status_readings(FRA4)
    assert readings['bms_failures'] == 134 and readings['bms_polls'] == 23040
    assert abs(readings['bms_fail_pct'] - 0.58) < 0.01
    assert readings['bms_worst_minute'] == 9
    found = findings_for(readings)
    assert found['bms_fail_pct']['level'] == doctor.ATTENTION
    assert '134 bad of 23040' in found['bms_fail_pct']['evidence']
    assert found['bms_worst_minute']['level'] == doctor.ATTENTION
    assert found['lamp_unknown']['level'] == doctor.WATCH
    assert doctor.headline(list(found.values())) == doctor.ATTENTION


def test_lamp_from_the_newest_button_event():
    red = status([event(1, 1, T0, 1, 5), event(2, 55, T0 + 60, 0x002, 0x01)])
    readings = doctor.status_readings(red)
    assert readings['lamp_held'] == 2
    assert readings['lamp_names'] == 'BMS unreliable + adapter fault'
    assert findings_for(readings)['lamp_held']['symptom'] == \
        'Power button light red: BMS unreliable + adapter fault'
    cleared = status([event(1, 1, T0, 1, 5), event(2, 55, T0 + 60, 0x002), event(3, 55, T0 + 90)])
    assert doctor.status_readings(cleared)['lamp_held'] == 0


def test_no_server_needs_attention():
    found = doctor.examine(doctor.status_readings(None))
    assert found[0]['reading'] == 'server_up' and found[0]['level'] == doctor.ATTENTION
    assert {f['level'] for f in found if f['area'] == 'power'} == {doctor.UNKNOWN}


def test_power_check_fault_becomes_a_finding():
    readings = doctor.status_readings(status([event(1, 1, T0)], health=checks('fault')))
    found = findings_for(readings)['power_faults']
    assert found['level'] == doctor.ATTENTION
    assert found['symptom'] == 'Power board check failing: bms link'


def test_missing_device_named_once_per_link():
    s = status([event(1, 1, T0)])
    s['firmware_versions']['boards'] = [{'port': '/dev/hello-doctor-test id 20'},
                                        {'port': '/dev/hello-doctor-test id 21'},
                                        {'port': 'via /dev/hello-power-periph'}]
    readings = doctor.status_readings(s)
    assert readings['devices_missing'] == 1
    assert readings['devices_missing_names'] == '/dev/hello-doctor-test'


def test_server_journal_counts_failures_and_last_error():
    text = '\n'.join([
        'Started stretch_body_server.service - Stretch Body Server Service.',
        '\x1b[34m\x1b[0m \x1b[32m[stretch_body_server]\x1b[0m \x1b[31m[ERROR]:\x1b[0m '
        '\x1b[31mTraceback (most recent call last):\x1b[0m',
        'RuntimeError: lift did not home',
        'stretch_body_server.service: Failed with result \'exit-code\'.',
        'Started stretch_body_server.service - Stretch Body Server Service.',
    ])
    readings = doctor.server_journal_readings(text)
    assert readings == {'server_starts': 2, 'server_failed_starts': 1, 'server_tracebacks': 1,
                        'server_last_error': 'RuntimeError: lift did not home'}
    assert doctor.server_journal_readings(None) == {}


def test_kernel_usb_counts():
    text = '\n'.join(['usb 3-2: USB disconnect, device number 7',
                      'usb 3-2: reset high-speed USB device number 8 using xhci_hcd',
                      'usb 3-2: new full-speed USB device number 9 using xhci_hcd'])
    readings = doctor.kernel_readings(text)
    assert readings['usb_disconnects'] == 1 and readings['usb_resets'] == 1
    assert readings['usb_overcurrent'] == 0 and readings['usb_overcurrent_last'] == 'none'


def test_rules_table_rows_are_complete():
    for rule in doctor.RULES:
        assert rule.level in (doctor.WATCH, doctor.ATTENTION)
        assert rule.low is not None or rule.high is not None
        assert rule.symptom and rule.cause and rule.action and rule.window


def test_verbose_lists_unjudged_rules_and_export_writes_evidence(tmp_path):
    found = doctor.examine({'server_up': 1})
    assert '[UNKNOWN]' not in doctor.render(found, 'robot', now=T0)
    assert 'Not judged: no bms_fail_pct reading.' in doctor.render(found, 'robot', verbose=True, now=T0)
    path = doctor.export(str(tmp_path), 'report', found, {'server_up': 1}, {'status.json': '{}'})
    assert sorted(os.listdir(path)) == ['report.json', 'report.txt', 'status.json']
    with open(os.path.join(path, 'report.json')) as f:
        assert json.load(f)['readings'] == {'server_up': 1}
