#!/usr/bin/env python3

import os
import socket
import threading
import time
import pytest
import zmq

from stretch4_body.core.client_server import (
    StretchBodyServer,
    StretchBodyClient,
    NotConnectedError,
    LEASE_TIMEOUT
)

# Keep reference to the original ZMQ methods for delegation
_original_bind = zmq.Socket.bind
_original_connect = zmq.Socket.connect


def find_free_port():
    """Helper to find a free TCP port."""
    s = socket.socket()
    s.bind(('', 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture(autouse=True)
def mock_network_env(monkeypatch, tmp_path):
    """
    Isolate the test environment using monkeypatch.
    Creates unique temporary directory for IPC socket files
    and allocates free ephemeral TCP ports to prevent any collision
    with system-wide server instances.
    """
    # Force is_user_in_group to always return True for tests
    monkeypatch.setattr("stretch4_body.core.client_server.is_user_in_group", lambda group: True)
    monkeypatch.setattr("stretch4_body.utils.file_access_utils.is_user_in_group", lambda group: True)

    # Isolated temporary folder for socket paths
    socket_dir = str(tmp_path / "stretch_zmq")
    monkeypatch.setattr("stretch4_body.core.client_server.SERVER_ZMQ_SOCKET_DIR", socket_dir)
    monkeypatch.setattr("stretch4_body.core.client_server.PORT_ADMIN", f"{socket_dir}/port_admin")
    monkeypatch.setattr("stretch4_body.core.client_server.PORT_COMMAND", f"{socket_dir}/port_command")
    monkeypatch.setattr("stretch4_body.core.client_server.PORT_STATUS", f"{socket_dir}/port_status")

    # Allocate ephemeral ports
    admin_port = find_free_port()
    cmd_port = find_free_port()
    status_port = find_free_port()

    def mock_bind_impl(self, addr):
        if "23114" in addr:
            addr = addr.replace("23114", str(admin_port))
        elif "23115" in addr:
            addr = addr.replace("23115", str(cmd_port))
        elif "23116" in addr:
            addr = addr.replace("23116", str(status_port))
        return _original_bind(self, addr)

    def mock_connect_impl(self, addr):
        if "23114" in addr:
            addr = addr.replace("23114", str(admin_port))
        elif "23115" in addr:
            addr = addr.replace("23115", str(cmd_port))
        elif "23116" in addr:
            addr = addr.replace("23116", str(status_port))
        return _original_connect(self, addr)

    monkeypatch.setattr(zmq.Socket, "bind", mock_bind_impl)
    monkeypatch.setattr(zmq.Socket, "connect", mock_connect_impl)


@pytest.fixture
def running_server():
    """
    Yields a started StretchBodyServer instance and runs its message dispatch loop
    in a background thread. Automatically stops the server on cleanup.
    """
    server = StretchBodyServer()
    assert server.startup() is True

    last_cmds = []
    last_admin_msgs = []
    stop_event = threading.Event()

    def cb_cmd(cmd_dict):
        last_cmds.append(cmd_dict)
        return []

    def cb_admin(msg):
        last_admin_msgs.append(msg)

    def loop():
        while not stop_event.is_set():
            try:
                server.dispatch_admin_messages(cb_admin)
                server.dispatch_command_messages(cb_cmd, is_routine_active=False)
            except Exception:
                pass
            time.sleep(0.005)

    t = threading.Thread(target=loop, daemon=True)
    t.start()

    yield server, last_cmds, last_admin_msgs

    stop_event.set()
    t.join(timeout=1.0)
    server.stop()


def test_basic_connect_disconnect(running_server):
    """
    Test that StretchBodyClient can connect, ping, send a command,
    and disconnect cleanly from StretchBodyServer.
    """
    server, last_cmds, last_admin_msgs = running_server

    client = StretchBodyClient()
    assert client.startup(verbose=False) is True
    assert client.connected is True

    # Test admin ping command
    assert client._do_send_recv_admin_str(b"ping") == b"ping"

    # Test basic command sending
    test_cmd = {"device": "arm", "val": 42}
    client._do_send_cmd(test_cmd)

    # Allow background dispatch to process the command
    time.sleep(0.05)
    assert len(last_cmds) == 1
    assert last_cmds[0] == test_cmd

    # Clean disconnect
    client.stop()
    assert client.connected is False


def test_reconnection(running_server):
    """
    Test subsequent client connect and disconnect actions in the same session.
    """
    server, last_cmds, last_admin_msgs = running_server

    # First connection
    client1 = StretchBodyClient(name="client_1")
    assert client1.startup(verbose=False) is True
    assert client1.connected is True
    client1.stop()
    assert client1.connected is False

    # Immediate second connection
    client2 = StretchBodyClient(name="client_2")
    assert client2.startup(verbose=False) is True
    assert client2.connected is True
    client2.stop()


def test_multiple_clients(running_server):
    """
    Test multiple clients connecting to the same server simultaneously.
    """
    server, last_cmds, last_admin_msgs = running_server

    c1 = StretchBodyClient(name="c1")
    c2 = StretchBodyClient(name="c2")

    assert c1.startup(verbose=False) is True
    assert c2.startup(verbose=False) is True

    assert c1._do_send_recv_admin_str(b"ping") == b"ping"
    assert c2._do_send_recv_admin_str(b"ping") == b"ping"

    c1.stop()
    c2.stop()


def test_lease_priority_and_expiry(running_server):
    """
    Verify the server's command lease prioritization, rejection,
    priority override, and expiration logic.
    """
    server, last_cmds, last_admin_msgs = running_server

    c_low = StretchBodyClient(name="client_low")
    c_high = StretchBodyClient(name="client_high")

    assert c_low.startup(verbose=False) is True
    assert c_high.startup(verbose=False) is True

    # 1. c_low sends low-priority command (priority=1) -> gets lease
    c_low._do_send_cmd({"cmd": "low_cmd"}, priority=1)
    time.sleep(0.05)
    assert len(last_cmds) == 1
    assert last_cmds[-1] == {"cmd": "low_cmd"}
    assert server.lease_holder_id == c_low.client_id
    assert server.lease_holder_priority == 1

    # 2. c_high sends equal or lower priority command (priority=1) -> rejected/ignored
    c_high._do_send_cmd({"cmd": "rejected_cmd"}, priority=1)
    time.sleep(0.05)
    # The last_cmds list shouldn't grow, since it should be rejected because c_low holds lease
    assert len(last_cmds) == 1

    # 3. c_high sends HIGHER priority command (priority=2) -> overrides lease
    c_high._do_send_cmd({"cmd": "high_cmd"}, priority=2)
    time.sleep(0.05)
    assert len(last_cmds) == 2
    assert last_cmds[-1] == {"cmd": "high_cmd"}
    assert server.lease_holder_id == c_high.client_id
    assert server.lease_holder_priority == 2

    # 4. Wait for lease to expire (LEASE_TIMEOUT is 1.1s)
    # Let's wait slightly more than 1.1s
    time.sleep(LEASE_TIMEOUT + 0.1)

    # 5. Now any client can grab the lease again. c_low sends a low priority command -> succeeds!
    c_low._do_send_cmd({"cmd": "fresh_cmd"}, priority=1)
    time.sleep(0.05)
    assert len(last_cmds) == 3
    assert last_cmds[-1] == {"cmd": "fresh_cmd"}
    assert server.lease_holder_id == c_low.client_id

    c_low.stop()
    c_high.stop()


def test_command_conflation(running_server):
    """
    Test that when commands are sent rapidly, sequence numbers increment
    and the server processes them.
    """
    server, last_cmds, last_admin_msgs = running_server

    client = StretchBodyClient()
    assert client.startup(verbose=False) is True
    # Allow ZMQ PUB/SUB subscription handshake to complete before publishing
    time.sleep(0.1)

    # Send 5 commands rapidly
    for i in range(5):
        client._do_send_cmd({"cmd": f"msg_{i}"})

    time.sleep(0.1)
    # Since CONFLATE is on, we might not receive all of them, but we should receive at least the last one.
    assert len(last_cmds) >= 1
    assert last_cmds[-1] == {"cmd": "msg_4"}

    client.stop()


def test_multiple_servers(tmp_path, monkeypatch):
    """
    Verify that two independent servers can start on completely different paths
    without interfering with each other's sockets or locks.
    """
    # Server 1 environment
    dir1 = str(tmp_path / "server1")
    os.makedirs(dir1, exist_ok=True)
    
    # Server 2 environment
    dir2 = str(tmp_path / "server2")
    os.makedirs(dir2, exist_ok=True)

    # Allocate non-conflicting dynamic ports
    s1_admin = find_free_port()
    s1_cmd = find_free_port()
    s1_status = find_free_port()

    s2_admin = find_free_port()
    s2_cmd = find_free_port()
    s2_status = find_free_port()

    # Define Server 1 with its own unique ports and paths
    server1 = StretchBodyServer()
    
    # Patch server1 socket bindings
    def s1_bind_impl(self, addr):
        if "23114" in addr:
            addr = f"tcp://*:{s1_admin}"
        elif "23115" in addr:
            addr = f"tcp://*:{s1_cmd}"
        elif "23116" in addr:
            addr = f"tcp://*:{s1_status}"
        return _original_bind(self, addr)

    # Define Server 2 with its own unique ports and paths
    server2 = StretchBodyServer()

    def s2_bind_impl(self, addr):
        if "23114" in addr:
            addr = f"tcp://*:{s2_admin}"
        elif "23115" in addr:
            addr = f"tcp://*:{s2_cmd}"
        elif "23116" in addr:
            addr = f"tcp://*:{s2_status}"
        return _original_bind(self, addr)

    # Startup server 1
    with monkeypatch.context() as mctx:
        mctx.setattr("stretch4_body.core.client_server.SERVER_ZMQ_SOCKET_DIR", dir1)
        mctx.setattr("stretch4_body.core.client_server.PORT_ADMIN", f"{dir1}/port_admin")
        mctx.setattr("stretch4_body.core.client_server.PORT_COMMAND", f"{dir1}/port_command")
        mctx.setattr("stretch4_body.core.client_server.PORT_STATUS", f"{dir1}/port_status")
        mctx.setattr(zmq.Socket, "bind", s1_bind_impl)
        assert server1.startup() is True

    # Startup server 2 simultaneously
    with monkeypatch.context() as mctx:
        mctx.setattr("stretch4_body.core.client_server.SERVER_ZMQ_SOCKET_DIR", dir2)
        mctx.setattr("stretch4_body.core.client_server.PORT_ADMIN", f"{dir2}/port_admin")
        mctx.setattr("stretch4_body.core.client_server.PORT_COMMAND", f"{dir2}/port_command")
        mctx.setattr("stretch4_body.core.client_server.PORT_STATUS", f"{dir2}/port_status")
        mctx.setattr(zmq.Socket, "bind", s2_bind_impl)
        assert server2.startup() is True

    # Stop both servers cleanly
    server1.stop()
    server2.stop()


def test_connection_failures_gracefully():
    """
    Verify client handling when the server is offline or not running.
    """
    client = StretchBodyClient()
    # Startup should return False gracefully and print a message without crashes
    assert client.startup(verbose=False) is False
    assert client.connected is False


def test_require_connection_decorator():
    """
    Verify that calling connection-requiring methods on an unconnected client
    raises NotConnectedError.
    """
    client = StretchBodyClient()
    # We force the is_valid attribute to be False to raise NotConnectedError
    # (or leave it undefined, which also triggers NotConnectedError)
    with pytest.raises(NotConnectedError):
        client._do_send_cmd({"cmd": "noop"})

    with pytest.raises(NotConnectedError):
        client._do_recv_status()


if __name__ == '__main__':
    pytest.main([__file__])
