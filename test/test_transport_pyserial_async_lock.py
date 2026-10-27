"""The async RPC paths must never leave the serial lock held.

Regression guard: do_pull_rpc_async and do_push_rpc_async released self.lock
only on a normal return, so a decoder exception or a cancelled task kept the
lock forever and every later serial transaction waited on it. These tests run
the real methods with a fake transaction handler, no serial port.
"""
import asyncio
import struct
import threading
import time

import pytest

from stretch4_body.core.transport.transport_pyserial import TransportPySerial


class FakeHandler:
    def __init__(self, outcome):
        self.outcome = outcome
        self.calls = []

    async def do_rpc(self, push, payload, reply_callback, exiting=False):
        self.calls.append(push)
        if isinstance(self.outcome, BaseException):
            raise self.outcome
        return self.outcome


def make_transport(handler):
    transport = TransportPySerial.__new__(TransportPySerial)
    transport.port_name = "/dev/null"
    transport.lock = threading.Lock()
    transport.async_handler = handler
    return transport


def test_pull_releases_lock_when_decoder_raises():
    transport = make_transport(FakeHandler(struct.error("short packet")))
    with pytest.raises(struct.error):
        asyncio.run(transport.do_pull_rpc_async(b"\x03", lambda reply: None))
    assert not transport.lock.locked()


def test_push_releases_lock_when_decoder_raises_and_routes_as_push():
    handler = FakeHandler(RuntimeError("boom"))
    transport = make_transport(handler)
    with pytest.raises(RuntimeError):
        asyncio.run(transport.do_push_rpc_async(b"\x09", lambda reply: None))
    assert not transport.lock.locked()
    assert handler.calls == [True]


def test_pull_and_push_return_the_transaction_result():
    transport = make_transport(FakeHandler(True))
    assert asyncio.run(transport.do_pull_rpc_async(b"\x03", lambda r: None)) is True
    assert asyncio.run(transport.do_push_rpc_async(b"\x09", lambda r: None)) is True
    assert transport.async_handler.calls == [False, True]
    assert not transport.lock.locked()


def test_cancelled_waiter_does_not_orphan_the_lock():
    """Cancel a task while its worker thread is blocked in lock.acquire()."""
    transport = make_transport(FakeHandler(True))
    transport.lock.acquire()  # held by someone else for now

    async def scenario():
        task = asyncio.ensure_future(
            transport.do_pull_rpc_async(b"\x03", lambda reply: None))
        await asyncio.sleep(0.05)  # worker is now parked in acquire()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert transport.async_handler.calls == []

        # The other holder lets go. The abandoned worker must acquire and
        # release on its own, so a fresh caller gets the lock in bounded time.
        transport.lock.release()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if transport.lock.acquire(timeout=0.05):
                transport.lock.release()
                return
            await asyncio.sleep(0.01)
        pytest.fail("lock stayed held after the waiter was cancelled")

    asyncio.run(scenario())
    assert not transport.lock.locked()
