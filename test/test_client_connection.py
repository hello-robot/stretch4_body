#!/usr/bin/env python3

import os
import socket
import threading
import time
import pytest
import zmq

from stretch4_body.core.client_server import StretchBodyServer
from stretch4_body.robot.robot_client import RobotClient

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
def running_robot_server():
    """
    Yields a started StretchBodyServer instance and runs a mockup status publisher
    periodically in a background thread to allow RobotClient to pull status successfully.
    """
    server = StretchBodyServer()
    assert server.startup() is True

    stop_event = threading.Event()
    last_cmds = []

    mock_status = {
        "server": {
            "state": "RUNNING",
            "control_loop": {"target_rate_hz": 25.0, "curr_rate_hz": 25.0, "num_loops": 100, "missed_loops": 0},
            "cpu": {}
        },
        "routines": {
            "last_routine_id": None,
            "last_routine_successful": True,
            "active_routine": None
        },
        "safety_layer": {
            "safe_motion_manager": {"active": False},
            "sentry_manager": {"active": False}
        },
        "robot": {},
        "power_periph": {},
        "arm": {"motor": {"pos_calibrated": True}},
        "lift": {"motor": {"pos_calibrated": True}},
        "omnibase": {},
        "end_of_arm": {},
        "line_sensor_loop": {}
    }

    def cb_cmd(cmd_dict):
        last_cmds.append(cmd_dict)
        return []

    def cb_admin(msg):
        pass

    def loop():
        while not stop_event.is_set():
            try:
                server.dispatch_admin_messages(cb_admin)
                server.dispatch_command_messages(cb_cmd, is_routine_active=False)
                server.publish_status(mock_status)
            except Exception:
                pass
            time.sleep(0.01)

    t = threading.Thread(target=loop, daemon=True)
    t.start()

    yield server, mock_status, last_cmds

    stop_event.set()
    t.join(timeout=1.0)
    server.stop()


def test_robot_client_basic_startup_shutdown(running_robot_server):
    """
    Verify basic startup and shutdown sequence for RobotClient when the server is active.
    """
    r = RobotClient()
    assert r.startup(verbose=False) is True
    assert r.connected is True

    r.stop()
    assert r.connected is False

def test_robot_client_double_startup(running_robot_server):
    """
    Verify basic startup and shutdown sequence for RobotClient when the server is active.
    """
    r = RobotClient()
    assert r.startup(verbose=False) is True
    assert r.startup(verbose=False) is True
    assert r.connected is True

    r.stop()
    assert r.connected is False

def test_multiple_stop_and_startup(running_robot_server):
    r = RobotClient()

    for i in range(10):
        assert r.startup(verbose=False) is True
        assert r.connected is True

        r.arm.move_by(0)
        r.push_command()

        time.sleep(1/10)

        r.stop()
        assert r.connected is False


def test_robot_client_context_manager(running_robot_server):
    """
    Verify that RobotClient works correctly as a Python context manager.
    """
    with RobotClient() as r:
        assert r is not None
        assert r.connected is True

    # After exiting the block, the client should automatically shut down
    assert r.connected is False


def test_robot_client_offline_graceful():
    """
    Verify that RobotClient fails to startup gracefully if no server is running.
    """
    r = RobotClient()
    # Startup should return False without hanging or raising exceptions
    assert r.startup(verbose=False) is False
    assert r.connected is False


def test_robot_client_reconnection(running_robot_server):
    """
    Verify subsequent connect, disconnect, and reconnect behaviors of RobotClient in the same session.
    """
    # First connection
    r1 = RobotClient()
    assert r1.startup(verbose=False) is True
    r1.stop()
    assert r1.connected is False

    # Second connection
    r2 = RobotClient()
    assert r2.startup(verbose=False) is True
    r2.stop()
    assert r2.connected is False


def test_robot_client_multiple_clients(running_robot_server):
    """
    Verify that multiple RobotClients can connect to the server at the same time.
    """
    r1 = RobotClient(client_id="robot_client_1")
    r2 = RobotClient(client_id="robot_client_2")

    assert r1.startup(verbose=False) is True
    assert r2.startup(verbose=False) is True

    assert r1.connected is True
    assert r2.connected is True

    r1.stop()
    r2.stop()


def test_robot_client_command_queuing(running_robot_server, monkeypatch):
    """
    Verify command queuing and dispatcher push from RobotClient to the server.
    """
    server, mock_status, last_cmds = running_robot_server

    # Bypass pusher lock file to allow clean pushing in tests without lock file issues
    monkeypatch.setattr("stretch4_body.utils.freeable_file_lock.FreeableFileLock.acquire", lambda self: True)

    with RobotClient() as r:
        assert r is not None
        # Check that we can command subsystems
        if hasattr(r, 'power_periph'):
            r.power_periph.trigger_beep()
        if hasattr(r, 'omnibase'):
            r.omnibase.translate_by(0.1, 0.0)

        # Allow handshakes to complete, then push command
        time.sleep(0.1)
        r.push_command()

        # Check server received commands
        time.sleep(0.1)
        assert len(last_cmds) >= 1
        
        # Verify the format of sent command dictionary
        cmd_sent = last_cmds[-1]
        assert "omnibase" in cmd_sent or "power_periph" in cmd_sent
def test_robot_client_hang_on_server_disconnect(running_robot_server):
    """
    Test that if a RobotClient is running and the server suddenly disappears/stops publishing,
    a blocking pull_status() loop would block. We verify this thread-safety scenario
    by running pull_status in a background thread and verifying it doesn't return
    once the server is stopped, and we can handle it safely.
    """
    server, mock_status, last_cmds = running_robot_server

    r = RobotClient()
    assert r.startup(verbose=False) is True

    # 1. First pull_status succeeds when server is active
    assert r.pull_status(blocking=True) is True

    # 2. Stop the server (simulating a crash or disconnect)
    server.stop()

    # Drain any queued status messages remaining in the client's ZMQ buffer
    for _ in range(100):
        if not r.pull_status(blocking=False):
            break

    # 3. Try to pull status in a thread
    status_received = []
    def pull_worker():
        try:
            res = r.pull_status(blocking=True)
            status_received.append(res)
        except Exception as e:
            status_received.append(e)

    t = threading.Thread(target=pull_worker, daemon=True)
    t.start()

    # Wait to see if it finishes or hangs
    t.join(timeout=0.5)

    # Since the server is offline, pull_status(blocking=True) should be stuck waiting/hanging,
    # so the thread should still be alive, and status_received should be empty!
    print(f"\n--- DEBUG: status_received = {status_received} ---")
    assert t.is_alive() is True
    assert len(status_received) == 0

    # Stop the client cleanly
    r.stop()


def test_robot_client_multithreaded_gamepad_teleop_simulation(running_robot_server, monkeypatch):
    """
    Simulates calibrate_intrinsics_robot_move.py scenario:
    - Main thread starts RobotClient() -> `r_main`
    - Background thread starts GamePadTeleop-like client -> `r_teleop`
    - Both pull status and send commands concurrently.
    - Test that they operate correctly concurrently, and stop gracefully without hangs.
    """
    server, mock_status, last_cmds = running_robot_server

    # Bypass pusher lock file to allow clean pushing in concurrent threads without lock file issues
    monkeypatch.setattr("stretch4_body.utils.freeable_file_lock.FreeableFileLock.acquire", lambda self: True)

    r_main = RobotClient(client_id="main_client")
    r_teleop = RobotClient(client_id="teleop_client")

    assert r_main.startup(verbose=False) is True
    assert r_teleop.startup(verbose=False) is True

    stop_event = threading.Event()
    teleop_errors = []

    def teleop_loop():
        while not stop_event.is_set():
            try:
                # Teleop thread pulls status and pushes a command periodically
                r_teleop.pull_status(blocking=True)
                if hasattr(r_teleop, 'omnibase'):
                    r_teleop.omnibase.set_velocity(0.1, 0.0, 0.1)
                r_teleop.push_command(ignore_control_lock=True)
                time.sleep(0.01)
            except Exception as e:
                teleop_errors.append(e)
                break

    t_teleop = threading.Thread(target=teleop_loop, daemon=True)
    t_teleop.start()

    # Main thread also does operations concurrently
    for _ in range(5):
        assert r_main.pull_status(blocking=True) is True
        time.sleep(0.02)

    # Clean stop
    stop_event.set()
    t_teleop.join(timeout=1.0)

    r_main.stop()
    r_teleop.stop()

    assert len(teleop_errors) == 0


def test_client_stop_no_hang_on_offline_server_with_pending_messages(running_robot_server):
    """
    Verifies that calling stop() on a client with pending messages (e.g., when the
    server is offline and cannot receive them) does not hang.
    Previously, without linger=0 and context.destroy(linger=0), this would hang indefinitely.
    """
    server, mock_status, last_cmds = running_robot_server

    r = RobotClient(client_id="hang_test_client")
    assert r.startup(verbose=False) is True

    # Stop the server so it is offline
    server.stop()

    # Wait a bit for the server ports to close
    time.sleep(0.1)

    # Queue some commands. This places them in the socket's outbound buffer.
    # Because the server is offline, they cannot be delivered.
    if hasattr(r, 'omnibase'):
        r.omnibase.set_velocity(0.1, 0.0, 0.1)
    r.push_command(ignore_control_lock=True)

    # Calling stop() must complete instantly (within 1.5 seconds).
    # If there was a linger, this would block indefinitely.
    start_time = time.monotonic()
    r.stop()
    duration = time.monotonic() - start_time

    assert duration < 1.5, f"stop() took {duration:.3f}s, which indicates a linger/hang!"


def test_client_gc_no_hang_on_offline_server_with_pending_messages(running_robot_server):
    """
    Verifies that garbage collection/destruction of a RobotClient with pending
    messages (when the server is offline) does not hang.
    Previously, GC of the context during __del__ would block indefinitely inside Context.term().
    """
    server, mock_status, last_cmds = running_robot_server

    r = RobotClient(client_id="gc_test_client")
    assert r.startup(verbose=False) is True

    # Stop the server so it is offline
    server.stop()
    time.sleep(0.1)

    # Send a command to queue message in outbound buffer
    if hasattr(r, 'omnibase'):
        r.omnibase.set_velocity(0.1, 0.0, 0.1)
    r.push_command(ignore_control_lock=True)

    # Trigger deletion/garbage collection of the client.
    # It must complete instantly.
    start_time = time.monotonic()
    del r
    import gc
    gc.collect()
    duration = time.monotonic() - start_time

    assert duration < 1.5, f"Garbage collection took {duration:.3f}s, indicating a destructor/Context.term() hang!"


if __name__ == '__main__':
    pytest.main([__file__])
