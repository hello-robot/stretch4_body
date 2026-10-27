"""Nonblocking RPC collection must not stall on background telemetry.

Regression guard: load_rpc_results(wait_on_result=True) waited up to one second
for every unfinished RPC, including the diagnostics pull queued at 2 Hz, and
then dropped every id whether or not it had finished. These tests drive the
real Transport methods against a scripted C transport, no hardware.
"""
import array
import time

from stretch4_body.core.transport.transport import Transport
from stretch4_body.subsystem.power_periph import PowerPeriphAux


class ScriptedCTransport:
    """load_rpc_result returns the next scripted value per id, last value repeats."""

    def __init__(self, script):
        self.script = {k: list(v) for k, v in script.items()}
        self.discarded = []
        self.submitted = []
        self.loaded = []

    def load_rpc_result(self, rpc_id):
        # On the real C transport a load of a ready result runs its callback.
        self.loaded.append(rpc_id)
        values = self.script.get(rpc_id, [0])
        return values.pop(0) if len(values) > 1 else values[0]

    def discard_rpc_callback(self, rpc_id):
        self.discarded.append(rpc_id)

    def is_rpc_result_ready(self, rpc_id):
        return False

    def get_rpc_duration_us(self, rpc_id):
        return 123

    def do_rpc(self, blocking, is_push, payload, nb_payload, rpc_callback):
        self.submitted.append((blocking, is_push, bytes(payload[:nb_payload])))
        return 7


def make_transport(script):
    transport = Transport.__new__(Transport)
    transport.use_c_backend = True
    transport.default_backend = Transport.BACKEND_C_SERIAL
    transport.transport_c = ScriptedCTransport(script)
    transport.status = {"rpc_duration_us": 0, "rpc_expired": 0}
    transport.n_rate_log = 0
    transport.rate_log = []
    transport.rpc_ids = []
    transport._rpc_pending = {}
    transport.port_name = "/dev/null"
    return transport


def queue(transport, rpc_id, wait_on_collect, timeout_s=2.0):
    transport._rpc_pending[rpc_id] = (time.monotonic() + timeout_s, wait_on_collect)
    transport.rpc_ids.append(rpc_id)


def test_deferred_rpc_is_polled_once_and_kept_when_not_ready():
    transport = make_transport({31: [-1]})
    queue(transport, 31, wait_on_collect=False)

    t0 = time.monotonic()
    res = transport.load_rpc_results(wait_on_result=True)
    elapsed = time.monotonic() - t0

    assert res == []
    assert elapsed < 0.05, "deferred RPC entered the blocking wait (%.3f s)" % elapsed
    assert transport.rpc_ids == [31]
    assert transport.transport_c.discarded == []


def test_deferred_rpc_is_collected_on_a_later_call():
    transport = make_transport({31: [-1, -1, 1]})
    queue(transport, 31, wait_on_collect=False)

    assert transport.load_rpc_results(wait_on_result=True) == []
    assert transport.load_rpc_results(wait_on_result=True) == []
    assert transport.load_rpc_results(wait_on_result=True) == [1]
    assert transport.rpc_ids == []
    assert 31 not in transport._rpc_pending
    assert transport.status["rpc_duration_us"] == 123


def test_deferred_rpc_expires_on_monotonic_deadline():
    transport = make_transport({31: [-1]})
    queue(transport, 31, wait_on_collect=False, timeout_s=0.02)
    assert transport.load_rpc_results(wait_on_result=True) == []

    time.sleep(0.03)
    res = transport.load_rpc_results(wait_on_result=True)

    assert res == [-1]
    assert transport.rpc_ids == []
    assert transport.transport_c.discarded == [31]
    assert transport.status["rpc_expired"] == 1


def test_deferred_rpc_ready_after_deadline_is_expired_not_loaded():
    """A reply that lands after the deadline
    must not run its callback or count as a success."""
    transport = make_transport({31: [1]})
    queue(transport, 31, wait_on_collect=False, timeout_s=0.02)

    time.sleep(0.03)
    res = transport.load_rpc_results(wait_on_result=True)

    assert res == [-1]
    assert transport.transport_c.loaded == [], "expired result was loaded, its callback ran"
    assert transport.transport_c.discarded == [31]
    assert transport.status["rpc_expired"] == 1
    assert transport.status["rpc_duration_us"] == 0
    assert transport.rpc_ids == []
    assert 31 not in transport._rpc_pending


def test_foreground_rpc_still_waits_for_its_result():
    transport = make_transport({5: [-1, -1, -1, 1]})
    queue(transport, 5, wait_on_collect=True)

    assert transport.load_rpc_results(wait_on_result=True) == [1]
    assert transport.rpc_ids == []


def test_foreground_rpc_without_wait_is_retained_not_dropped():
    transport = make_transport({5: [-1, 1]})
    queue(transport, 5, wait_on_collect=True)

    assert transport.load_rpc_results(wait_on_result=False) == []
    assert transport.rpc_ids == [5]
    assert transport.load_rpc_results(wait_on_result=False) == [1]
    assert transport.rpc_ids == []


def test_mixed_queue_returns_only_finished_results_in_order():
    transport = make_transport({5: [1], 31: [-1], 6: [0]})
    queue(transport, 5, wait_on_collect=True)
    queue(transport, 31, wait_on_collect=False)
    queue(transport, 6, wait_on_collect=True)

    assert transport.load_rpc_results(wait_on_result=True) == [1, 0]
    assert transport.rpc_ids == [31]
    # The definitive failure released its callback, the pending one did not.
    assert transport.transport_c.discarded == [6]


def test_do_rpc_records_deadline_and_collect_mode():
    transport = make_transport({7: [-1]})
    before = time.monotonic()
    rpc_id = transport.do_rpc(
        blocking=False, is_push=False, payload=array.array("B", [31]),
        rpc_callback=lambda reply: None, wait_on_collect=False, timeout_s=2.0,
    )
    assert rpc_id == 7
    deadline, wait_on_collect = transport._rpc_pending[7]
    assert wait_on_collect is False
    assert before + 1.9 <= deadline <= time.monotonic() + 2.0
    assert transport.pending_rpc_count() == 1


def test_default_collect_mode_waits_with_one_second_deadline():
    transport = make_transport({7: [-1]})
    before = time.monotonic()
    transport.do_rpc(blocking=False, is_push=True, payload=array.array("B", [9]),
                     rpc_callback=lambda reply: None)
    deadline, wait_on_collect = transport._rpc_pending[7]
    assert wait_on_collect is True
    assert before + 0.9 <= deadline <= time.monotonic() + 1.0


class RecordingTransport:
    def __init__(self):
        self.calls = []

    def do_rpc(self, **kwargs):
        self.calls.append(kwargs)
        return 42


def test_power_periph_queues_diagnostics_as_deferred_telemetry():
    power = PowerPeriphAux()
    power.hw_valid = True
    power.board_info = {"protocol_version": "p14"}
    power.transport = RecordingTransport()

    assert power.pull_power_diagnostics(blocking=False, after_sequence=3, max_entries=16)
    assert power.pull_status_aux(blocking=False)
    for call in power.transport.calls:
        assert call["blocking"] is False
        assert call["is_push"] is False
        assert call["wait_on_collect"] is False
        assert call["timeout_s"] == PowerPeriphAux.POWER_DIAGNOSTICS_RPC_TIMEOUT_S
