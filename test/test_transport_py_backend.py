"""The Python serial backend routes pushes as pushes and reports success.

Transport.do_rpc on BACKEND_PY_SERIAL always called the pull
transaction, whatever is_push said, and always returned None, so a push longer
than one pull frame was truncated and every synchronous caller saw failure
even when its callback had decoded a good reply. These tests use the real
Transport and handler code with fake transactions, no serial port.
"""
import array
import logging
import struct
import threading

from stretch4_body.core.transport.transport import Transport
from stretch4_body.core.transport.transport_pyserial import (
    SyncTransactionHandler,
    TransportPySerial,
)
from stretch4_body.core.transport.transport_util import RPC_TRANSPORT_VERSION_1
from stretch4_body.subsystem.power_periph import PowerPeriphAux
from test_robot_server_power_aux import make_power_diagnostics_packet


class FakeSyncHandler:
    def __init__(self, result=True, reply=None):
        self.result = result
        self.reply = reply
        self.calls = []

    def do_rpc(self, push, payload, reply_callback, exiting=False):
        self.calls.append((push, bytes(payload)))
        if self.reply is not None:
            reply_callback(self.reply)
        return self.result


def make_py_transport(handler):
    transport = Transport.__new__(Transport)
    transport.port_name = "/dev/null"
    transport.logger = logging.getLogger("test")
    transport.default_backend = Transport.BACKEND_PY_SERIAL
    transport.use_c_backend = False
    transport.transport_c = None
    transport.lock = threading.Lock()
    transport.sync_handler = handler
    transport.status = {}
    transport.rpc_ids = []
    transport._rpc_pending = {}
    return transport


def test_python_backend_dispatches_on_is_push():
    handler = FakeSyncHandler()
    transport = make_py_transport(handler)
    assert transport.do_rpc(blocking=True, is_push=True, payload=array.array("B", [9, 1]),
                            rpc_callback=lambda r: None) is True
    assert transport.do_rpc(blocking=True, is_push=False, payload=array.array("B", [3]),
                            rpc_callback=lambda r: None) is True
    assert handler.calls == [(True, b"\x09\x01"), (False, b"\x03")]
    assert not transport.lock.locked()


def test_python_backend_reports_a_failed_transaction_as_none():
    transport = make_py_transport(FakeSyncHandler(result=False))
    assert transport.do_rpc(blocking=True, is_push=False, payload=array.array("B", [3]),
                            rpc_callback=lambda r: None) is None


def test_python_backend_still_refuses_nonblocking_requests():
    transport = make_py_transport(FakeSyncHandler())
    assert transport.do_rpc(blocking=False, is_push=False, payload=array.array("B", [3]),
                            rpc_callback=lambda r: None) is None
    assert transport.sync_handler.calls == []


def make_handler():
    handler = SyncTransactionHandler.__new__(SyncTransactionHandler)
    handler.ser = object()
    handler.version = RPC_TRANSPORT_VERSION_1
    handler.status = {"transactions": 0, "read_error": 0, "write_error": 0}
    handler.logger = logging.getLogger("test")
    handler.port_name = "/dev/null"
    return handler


def test_sync_handler_returns_transaction_result_per_direction():
    handler = make_handler()
    seen = []
    handler.do_push_transaction_v1 = lambda payload, cb: seen.append("push") or True
    handler.do_pull_transaction_v1 = lambda payload, cb: seen.append("pull") or False
    assert handler.do_rpc(push=True, payload=b"\x09", reply_callback=lambda r: None) is True
    assert handler.do_rpc(push=False, payload=b"\x03", reply_callback=lambda r: None) is False
    assert seen == ["push", "pull"]
    assert handler.status["transactions"] == 2


def test_sync_handler_returns_false_on_io_error_and_without_a_port():
    handler = make_handler()

    def explode(payload, cb):
        raise IOError(5, "Input/output error")

    handler.do_pull_transaction_v1 = explode
    assert handler.do_rpc(push=False, payload=b"\x03", reply_callback=lambda r: None) is False
    assert handler.status["read_error"] == 1
    handler.ser = None
    assert handler.do_rpc(push=False, payload=b"\x03", reply_callback=lambda r: None) is False


def test_sync_wrappers_return_the_handler_result():
    transport = TransportPySerial.__new__(TransportPySerial)
    transport.lock = threading.Lock()
    transport.sync_handler = FakeSyncHandler(result=True)
    assert transport.do_pull_rpc_sync(b"\x03", lambda r: None) is True
    assert transport.do_push_rpc_sync(b"\x09", lambda r: None) is True
    assert transport.sync_handler.calls == [(False, b"\x03"), (True, b"\x09")]


def test_blocking_power_diagnostics_pull_returns_true_on_a_decoded_reply():
    power = PowerPeriphAux()
    power.hw_valid = True
    power.board_info = {"protocol_version": "p14"}
    diagnostics = make_power_diagnostics_packet()
    header = struct.pack("<BBHBBBBHHIII", 1, 24, len(diagnostics), 16, 0, 0, 0, 0, 0, 0, 0, 0)
    reply = array.array("B", bytes([power.RPC_REPLY_POWER_DIAGNOSTICS]) + header + diagnostics)
    power.transport = make_py_transport(FakeSyncHandler(result=True, reply=reply))

    assert power.pull_power_diagnostics(blocking=True) is True
    assert power.power_diagnostics["_valid"]
    assert power.power_diagnostics_reply_sequence == 1
    (push, payload), = power.transport.sync_handler.calls
    assert push is False
    assert payload[0] == power.RPC_GET_POWER_DIAGNOSTICS


def test_blocking_power_diagnostics_pull_returns_false_when_transport_fails():
    power = PowerPeriphAux()
    power.hw_valid = True
    power.board_info = {"protocol_version": "p14"}
    power.transport = make_py_transport(FakeSyncHandler(result=False))
    assert power.pull_power_diagnostics(blocking=True) is False
