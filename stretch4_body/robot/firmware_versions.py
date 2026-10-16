#!/usr/bin/env python3
"""
Firmware versions of the boards stretch_body_server talks to, for status['firmware_versions'].

Every version here comes from code that already owns the port: board_info read by each
stepper and the PIMU at startup, the ESP version in the PIMU diagnostics snapshot
(schema 3), the Feetech version registers read once at servo startup, and the line
sensor's answer to '?'. Nothing here opens a device or sends a request.

Each board is one row: board, port, version, detail, source, time (epoch seconds the
value was obtained, None if never) and reason (why version is None, else None).
stretch_firmware_versions prints these rows.
"""
import time

LINE_SENSOR_PORT = '/dev/hello-pixart-j3'

ESP_REASONS = {
    'waiting': 'the PIMU is asking the ESP (ESP link down or just up)',
    'no_reply': 'ESP firmware older than hello-esp.v0.1.11p0 does not report its version',
}
LINE_SENSOR_REASONS = {
    'not_open': 'line sensor port not open',
    'waiting': 'asked the board, no answer yet',
    'no_reply': 'line sensor firmware older than hello-pixart-j3.v0.1.1p0 does not report its version',
}


def row(board, port, version, source, t=None, detail=None, reason=None):
    return {'board': board, 'port': port, 'version': version or None,
            'detail': detail or {}, 'source': source,
            'time': t if version else None,
            'reason': None if version else (reason or 'not reported')}


def board_info_row(board, port, board_info, hw_valid, t):
    """A stepper or the PIMU, from the board_info its startup read."""
    bi = board_info or {}
    version = bi.get('firmware_version')
    detail = {k: bi.get(k) for k in ('protocol_version', 'board_variant', 'hardware_id')
              if bi.get(k) is not None}
    reason = None
    if not version:
        reason = 'not connected' if not hw_valid else 'board did not answer board_info'
    return row(board, port, version, 'board_info at server startup', t, detail, reason)


def esp_row(port, diag, meta, now):
    """The ESP, from the PIMU's diagnostics snapshot (schema 3, PIMU v0.1.53p14)."""
    diag = diag or {}
    meta = meta or {}
    source = 'PIMU diagnostics snapshot'
    if not diag.get('_valid'):
        return row('esp', port, None, source,
                   reason='no PIMU diagnostics sample (%s)' % (
                       diag.get('_decode_error') or 'none received yet'))
    if diag.get('schema_version', 0) < 3:
        return row('esp', port, None, source,
                   reason='PIMU firmware %s (diagnostics schema %s) does not report the ESP '
                          'version; needs hello-pimu2.v0.1.53p14' % (
                              meta.get('firmware_version') or 'unknown',
                              diag.get('schema_version')))
    detail = {'requests': diag.get('esp_version_requests')}
    state = diag.get('esp_version_state_name')
    if state == 'reported' and diag.get('esp_version'):
        age = meta.get('sample_age_s')
        t = now - age if isinstance(age, (int, float)) else now
        return row('esp', port, diag['esp_version'], source, t, detail)
    return row('esp', port, None, source, detail=detail,
               reason=ESP_REASONS.get(state, 'ESP version state %s' % state))


def servo_row(joint, port, servo_id, joint_status):
    """One Feetech servo, from the version read at its startup."""
    fw = (joint_status or {}).get('firmware')
    board = 'servo %s' % joint
    port = '%s id %s' % (port, servo_id) if servo_id is not None else port
    source = 'servo registers 0 to 4 at startup'
    if not isinstance(fw, dict):
        return row(board, port, None, source, reason='end of arm not started')
    detail = {'model': fw.get('model')} if fw.get('model') is not None else {}
    return row(board, port, fw.get('version'), source, fw.get('read_time'), detail,
               fw.get('error'))


def line_sensor_row(ls_status, enabled):
    source = "board's answer to '?'"
    if not enabled:
        return row('line sensor', LINE_SENSOR_PORT, None, source,
                   reason='line_sensor_loop is not in server.subsystems')
    fw = (ls_status or {}).get('firmware') or {}
    state = fw.get('state', 'not_open')
    detail = {k: fw.get(k) for k in ('uptime_ms', 'reset_cause_raw') if fw.get(k) is not None}
    return row('line sensor', LINE_SENSOR_PORT, fw.get('version'), source,
               fw.get('received_time'), detail,
               LINE_SENSOR_REASONS.get(state, 'line sensor state %s' % state))


def collect(robot, startup_time, now=None):
    """Rows for every board of a RobotServer (or RobotCore) as it stands now."""
    now = time.time() if now is None else now
    rows = []
    pp = robot.get_subsystem('power_periph')
    if pp is not None:
        port = pp.transport.port_name
        rows.append(board_info_row('pimu', port, pp.board_info, pp.hw_valid, startup_time))
        pstatus = robot.status.get('power_periph', {})
        rows.append(esp_row('via %s' % port, pp.power_diagnostics,
                            pstatus.get('power_diagnostics_meta'), now))
    for name in ('lift', 'arm'):
        joint = robot.get_subsystem(name)
        if joint is not None:
            m = joint.motor
            rows.append(board_info_row('stepper %s' % name, m.usb, m.board_info, m.hw_valid,
                                       startup_time))
    base = robot.get_subsystem('omnibase')
    if base is not None:
        for i, w in enumerate(base.wheels):
            rows.append(board_info_row('stepper wheel %d' % i, w.usb, w.board_info, w.hw_valid,
                                       startup_time))
    if robot.get_subsystem('end_of_arm') is not None:
        eoa = robot.status.get('end_of_arm') or {}
        params = robot.robot_params
        bus = params.get('end_of_arm', {}).get('usb_name')
        for joint in eoa.get('joint_names') or []:
            jp = params.get(joint, {})
            rows.append(servo_row(joint, jp.get('usb_name', bus), jp.get('id'), eoa.get(joint)))
    enabled = 'line_sensor_loop' in robot.params.get('server', {}).get('subsystems', [])
    rows.append(line_sensor_row(robot.status.get('line_sensor_loop'), enabled))
    return rows
