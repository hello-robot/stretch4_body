"""RobotServer publishes power_health at the 2 Hz sample and a 2 Hz tick, not per cycle.

Uses the fake PowerPeriph server from test_robot_server_power_aux and a real
decoded snapshot from test_power_health. No hardware.
"""
import time

from stretch4_body.subsystem.power_health import PowerHealthEvaluator
from test_power_health import snapshot
from test_robot_server_power_aux import make_server


def health_server():
    server = make_server()
    pp = server.power_periph
    pp.diagnostics_supported = True
    pp.power_diagnostics = snapshot()
    pp.board_info = {'firmware_version': 'hello-pimu2.v0.1.48p14', 'protocol_version': 'p14'}
    server._power_diag_meta.update({
        'bms_freshness': 'unknown', 'esp_freshness': 'unknown', 'bms_age_s': None,
        'esp_age_s': None, 'bms_freshness_at_sample': 'unknown',
        'esp_freshness_at_sample': 'unknown', 'sample_age_s': None,
        'schema_error': None, 'firmware_version': None, 'protocol_version': None,
    })
    server._power_diag_fw_ages = (None, None)
    server.status = {'power_periph': {
        'power_diagnostics': pp.power_diagnostics,
        'power_diagnostics_meta': server._power_diag_meta,
        'power_event_log': server._power_event_meta,
        'rpc_rejected': {'count': 0}, 'rpc_reply_errors': 0,
    }}
    server._power_health_evaluator = PowerHealthEvaluator()
    server._power_health_last_eval_mono = float('-inf')
    return server


def sample(server):
    server._queue_power_periph_aux_status()
    server._complete_power_periph_aux_status()


def test_health_is_published_with_each_diagnostics_sample():
    server = health_server()
    server._record_power_firmware_version()
    sample(server)
    health = server.status['power_periph']['power_health']
    assert health['sample_seq'] == server._power_diag_meta['sample_seq'] == 1
    assert health['level'] == 'ok', health['summary']
    link = {c['id']: c for c in health['checks']}['diagnostics_link']
    assert link['values']['firmware_version'] == 'hello-pimu2.v0.1.48p14'
    assert link['values']['protocol_version'] == 'p14'


def test_callback_timeout_turns_the_link_check_stale():
    server = health_server()
    sample(server)
    server.power_periph.complete_on_queue = False
    server._power_aux_last_request_mono = float('-inf')
    server._queue_power_periph_aux_status()
    server._power_aux_request_started_mono -= 3.0
    server._power_diag_meta['sample_age_s'] = 3.0
    server._complete_power_periph_aux_status()
    health = server.status['power_periph']['power_health']
    link = {c['id']: c for c in health['checks']}['diagnostics_link']
    assert link['level'] == 'warn'
    assert 'timeout' in link['message']
    assert health['level'] == 'warn'


def test_publish_tick_is_throttled_to_2hz():
    server = health_server()
    calls = []
    real = server._power_health_evaluator.evaluate

    def spy(st, now, wall_time_s=None):
        calls.append(now)
        return real(st, now, wall_time_s)

    server._power_health_evaluator.evaluate = spy
    server._update_power_health()
    for _ in range(50):          # half a second of 100 Hz cycles at most
        server._update_power_health()
    assert len(calls) == 1
    server._power_health_last_eval_mono -= server.POWER_HEALTH_TICK_S
    server._update_power_health()
    assert len(calls) == 2
    server._update_power_health(force=True)
    assert len(calls) == 3


def test_evaluator_error_never_reaches_the_control_loop():
    server = health_server()

    class Broken:
        def evaluate(self, *args, **kwargs):
            raise RuntimeError('boom')

    class Log:
        def __init__(self):
            self.lines = []

        def warning(self, *args):
            self.lines.append(args)

    server._power_health_evaluator = Broken()
    server.logger = Log()
    server._update_power_health(force=True)
    server._update_power_health(force=True)
    health = server.status['power_periph']['power_health']
    assert health['level'] == 'unknown'
    assert 'boom' in health['summary']
    assert len(server.logger.lines) == 1


def test_server_without_evaluator_or_status_is_a_no_op():
    server = make_server()
    server._update_power_health(force=True)   # no evaluator, no status: nothing to do
    assert not hasattr(server, 'status')


def test_evaluation_is_cheap():
    server = health_server()
    sample(server)
    st = server.status['power_periph']
    evaluator = PowerHealthEvaluator()
    t0 = time.perf_counter()
    for i in range(200):
        st['power_diagnostics_meta']['sample_seq'] = i + 2
        evaluator.evaluate(st, float(i) * 0.5)
    per_eval_ms = (time.perf_counter() - t0) / 200 * 1000.0
    # Runs at 2 Hz; even 2 ms would be 0.4 % of one core.
    assert per_eval_ms < 2.0, per_eval_ms
