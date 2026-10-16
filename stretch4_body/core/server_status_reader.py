#!/usr/bin/env python3
"""
Read-only view of the status stretch_body_server publishes.

For tools that only read. It subscribes to the server's status socket and opens no
command or admin socket, so it cannot change robot state. A RobotClient is not a
substitute: its stop() (also run from __del__) pushes enable_freewheel_mode for the
base and enable_safety for the lift and arm.
"""
import os

import zmq

from stretch4_body.core.client_server import PORT_STATUS


class ServerStatusReader:
    """Subscriber to the server status. read() returns the latest status dict or None."""

    def __init__(self, port=PORT_STATUS):
        self.port = port
        self.socket = None

    def open(self):
        """False when no server has created its status socket."""
        if not os.path.exists(self.port):
            return False
        self.socket = zmq.Context.instance().socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.CONFLATE, 1)   # latest message only
        self.socket.setsockopt(zmq.SUBSCRIBE, b'')
        self.socket.connect('ipc://%s' % self.port)
        return True

    def read(self, timeout=2.0):
        """The newest status, waiting up to timeout seconds; None if none arrived."""
        if self.socket is None:
            return None
        if not self.socket.poll(int(timeout * 1000), zmq.POLLIN):
            return None
        return self.socket.recv_pyobj(flags=zmq.NOBLOCK)

    def close(self):
        if self.socket is not None:
            self.socket.close(linger=0)
            self.socket = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()


def read_server_status(timeout=2.0):
    """One status message from stretch_body_server, or None if none arrives in timeout."""
    with ServerStatusReader() as reader:
        return reader.read(timeout)
