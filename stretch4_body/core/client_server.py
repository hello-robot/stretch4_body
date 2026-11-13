#!/usr/bin/env python3

import logging
import os
from pathlib import Path
import psutil
import socket
import uuid
import zmq
import time
import functools
from stretch4_body.utils.file_access_utils import get_file_owner, is_file_owned_by_current_user, acquire_lock_if_available, setup_shared_directory, is_user_in_group

SERVER_ZMQ_SOCKET_DIR = "/tmp/stretch_zmq"

PORT_ADMIN = f"{SERVER_ZMQ_SOCKET_DIR}/port_admin"
PORT_COMMAND = f"{SERVER_ZMQ_SOCKET_DIR}/port_command"
PORT_STATUS = f"{SERVER_ZMQ_SOCKET_DIR}/port_status"
LEASE_TIMEOUT = 1.1 # Writers at >=1hz (0.909hz actually) keep control


class NotConnectedError(Exception):
    """Exception raised when an operation is attempted without a connection."""
    def __init__(self, class_name):
        self.message = f"{class_name} object is not connected to the Stretch Body Server. Call `.startup()` first. If you used `robot_client.stop()`, you need to call `.startup()` again. If you are trying to stop robot motion, use `hard_stop()` or `set_velocity(0)`, instead of `stop()`. `stop()` disconnects the client from the Stretch Body Server."
        super().__init__(self.message)

def require_connection(function):
    @functools.wraps(function)
    def wrapper_function(self, *args, **kwargs):
        if not self.connected:
            raise NotConnectedError(self.__class__.__name__)

        return function(self, *args, **kwargs)
    return wrapper_function


class StretchBodyServer:
    def __init__(self, enable_network_access=None):
        """
        Parameters
        ----------
        enable_network_access : str | None, optional
            The address to also bind the admin/command/status TCP sockets to, so remote
            clients can connect. None binds only the local IPC sockets. "*" listens on all
            network interfaces. An IP address (e.g. "192.168.1.20") listens only on the
            interface that has that address. By default None.
        """
        self.enable_network_access = enable_network_access
        self.lease_holder_priority = None
        self.lease_holder_id = None
        self.lease_expiry = 0.0
        self.last_cmd_seq = None
        self.logger = logging.getLogger(name='stretch_body_server')

        self.context=None
        self.socket_cmd=None
        self.socket_admin=None
        self.socket_status=None

    def startup(self):
        if not is_user_in_group('users'):
            self.logger.error("Cannot start Stretch Body Server: User is not a member of the 'users' group. The user should be a member of the 'users' group for locks to work properly.")
            return False

        bind_host = self.enable_network_access
        if bind_host is not None and (not isinstance(bind_host, str) or not bind_host.strip()):
            self.logger.error(f"Cannot start Stretch Body Server: robot.server.enable_network_access must be None, '*', or an IP address of this machine, got {bind_host!r}.")
            return False
        if bind_host is not None:
            bind_host = bind_host.strip()

        try:
            for file in [PORT_ADMIN, PORT_COMMAND, PORT_STATUS]:
                file = f"{file}_lock"
                setup_shared_directory(Path(file).parent)

                if not acquire_lock_if_available(file, remove_if_exists_and_unused=True):
                    raise RuntimeError("ZMQ socket file is in use.")

            # Setup status publishing
            self.context = zmq.Context()
            self.socket_status = self.context.socket(zmq.PUB)
            self.socket_status.setsockopt(zmq.SNDHWM, 1)
            self.socket_status.setsockopt(zmq.CONFLATE, 1)
            self.socket_status.bind(f"ipc://{PORT_STATUS}")
            if bind_host is not None:
                self._bind_tcp(self.socket_status, bind_host, 23116)
            self.logger.info(f'Starting Stretch Body Server Status publishing on port: {PORT_STATUS}')

            # Setup admin connection
            self.socket_admin = self.context.socket(zmq.REP)
            self.socket_admin.bind(f"ipc://{PORT_ADMIN}")
            if bind_host is not None:
                self._bind_tcp(self.socket_admin, bind_host, 23114)
            self.logger.info(f'Starting Stretch Body Server Admin service on port: {PORT_ADMIN}')

            # Setup command subscribing
            self.socket_cmd = self.context.socket(zmq.SUB)
            self.socket_cmd.setsockopt(zmq.CONFLATE, 1) # Only read latest message
            self.socket_cmd.setsockopt(zmq.SUBSCRIBE, b"") # Subscribe to all topics
            self.socket_cmd.bind(f"ipc://{PORT_COMMAND}")
            if bind_host is not None:
                self._bind_tcp(self.socket_cmd, bind_host, 23115)
            self.logger.info(f'Starting Stretch Body Server Command service on port: {PORT_COMMAND}')

            if bind_host is None:
                self.logger.info("Stretch Body Server network access is disabled (local IPC only). Set robot.server.enable_network_access to '*' or an IP address of this machine to allow remote clients.")
            elif bind_host == '*':
                self.logger.warning('Stretch Body Server network access is ENABLED on ALL interfaces (TCP ports 23114-23116). Any host that can reach this machine can command the robot.')
            else:
                self.logger.warning(f'Stretch Body Server network access is ENABLED on {bind_host} (TCP ports 23114-23116). Any host that can reach that address can command the robot.')

            return True
        except (zmq.ZMQError, RuntimeError) as z:
            self.logger.error('Failed to start Stretch Body Server. It may already be running.')
            self.logger.error(z)
            return False

    def _bind_tcp(self, socket, host, port):
        try:
            socket.bind(f"tcp://{host}:{port}")
        except zmq.ZMQError:
            self.logger.error(f"Stretch Body Server could not listen on tcp://{host}:{port} (robot.server.enable_network_access={host!r}). Check that it is '*' or an IP address assigned to this machine, and that the port is free.")
            raise

    def stop(self):
        time.sleep(0.1) # Required here for all freewheel, etc. commands to transmit
        if self.socket_cmd is not None:
            self.socket_cmd.close(linger=0)
        if self.socket_status is not None:
            self.socket_status.close(linger=0)
        if self.socket_admin is not None:
            self.socket_admin.close(linger=0)
        if self.context is not None:
            try:
                self.context.destroy(linger=0)
            except Exception:
                pass

    def dispatch_command_messages(self,cb_dispatch, is_routine_active):
        # Check if messages available
        try:
            message = self.socket_cmd.recv_pyobj(flags=zmq.NOBLOCK)
        except zmq.Again:
            # No new data
            return

        current_time = time.monotonic()

        client_id = message['client_id']
        cmd_priority = message['priority']
        cmd_seq = message.get('seq', None)

        # If a routine is active, extend the lease holder's time
        if is_routine_active:
            self.lease_expiry = current_time + LEASE_TIMEOUT

        # Check if incoming command has a higher priority
        if self.lease_holder_priority is not None and cmd_priority > self.lease_holder_priority:
            # The current lease holder has lower priority, so the new command takes the lease
            self.lease_holder_id = None
            self.lease_holder_priority = None

        # Check if lease has expired
        if self.lease_holder_id is not None and current_time > self.lease_expiry:
            # Client hasn't sent a command in last second, so their lease has expired
            self.lease_holder_id = None
            self.lease_holder_priority = None

        # Check if client is allowed to command
        if self.lease_holder_id is None:
            # New lease
            self.lease_holder_id = client_id
            self.lease_holder_priority = cmd_priority
            self.lease_expiry = current_time + LEASE_TIMEOUT
            self.last_cmd_seq = cmd_seq
        elif client_id == self.lease_holder_id:
            # Renew lease
            if cmd_seq is not None and self.last_cmd_seq is not None:
                if cmd_seq > self.last_cmd_seq + 1:
                    dropped = cmd_seq - self.last_cmd_seq - 1
                    if dropped > 10:
                        self.logger.warning(f"Server fell behind and dropped {dropped} commands from client {client_id} (likely due to ZMQ CONFLATE).")
                    else:
                        self.logger.debug(f"Server conflated {dropped} incoming commands from {client_id}.")
            self.last_cmd_seq = cmd_seq
            self.lease_expiry = current_time + LEASE_TIMEOUT
        else:
            # Reject command. Client doesn't hold the lease.
            return

        # Process command
        cmd_dict = message['cmd_dict']
        result = cb_dispatch(cmd_dict) # result(arr) is the ids of the commands that got dispatched to hardware

    def dispatch_admin_messages(self,cb_dispatch):
        while True:
            try:
                message = self.socket_admin.recv(flags=zmq.NOBLOCK)
                cb_dispatch(message)
                self.socket_admin.send_string(message.decode('utf-8')) #send str back as an ack
            except zmq.Again:
                break

    def publish_status(self, s):
        self.socket_status.send_pyobj(s, flags=zmq.NOBLOCK)

    @staticmethod
    def is_server_owned_by_current_user():
        if is_file_owned_by_current_user(PORT_ADMIN):
            return True
        return False
    
    @staticmethod
    def get_server_owning_user():
        return get_file_owner(PORT_ADMIN)


class StretchBodyClient:
    def __init__(self, name=None, ip_address=None):
        self._server_connected=False
        self.admin_poller=None
        self.context=None
        self.socket_cmd=None
        self.socket_admin=None
        self.socket_status=None
        self.client_id= name if name is not None else self.client_id()
        self.ip_address=ip_address
        self.cmd_seq = 0

        self.logger = logging.getLogger(name='stretch_body_client')

    @property
    def connected(self):
        """Warning: server_connected may become stale; it's not a computed property. Call `check_connection()` to refresh it."""
        return self._server_connected
    
    def check_connection(self):
        """
        Warning: mutation of `self._server_connected` happens here.

        Updates `self._server_connected` with whether the server responds to a ping.
        """
        if self.socket_admin is None:
            self._server_connected = False
            return self._server_connected
        
        ack = self._do_send_recv_admin_str(b"ping")
        self._server_connected = ack is not None

        return self._server_connected

    def free_up_control(self):
        ack = self._do_send_recv_admin_str(b"free_up_control", timeout=3.0)
        self._server_connected = (ack == b"free_up_control")
    
    def client_id(self): 
        pid = os.getpid()
        pname = psutil.Process(pid).name()
        return f"client_{pname}_{pid}_{str(uuid.uuid4())[:8]}"

    def _targets_this_machine(self):
        """True if `self.ip_address` resolves to one of this machine's own addresses."""
        try:
            targets = {info[4][0] for info in socket.getaddrinfo(self.ip_address, None)}
        except (socket.gaierror, UnicodeError):
            return False
        local = {'::1'}
        for if_addrs in psutil.net_if_addrs().values():
            for a in if_addrs:
                local.add(a.address.split('%')[0]) # drop IPv6 zone index, e.g. fe80::1%eth0
        return any(t in local or t.startswith('127.') for t in targets)

    def _remote_user_params_location(self):
        """
        Where the user should set `enable_network_access`, for the connection-failure message.

        If the server being connected to is this machine and its fleet variables are set, give the
        expanded path. Otherwise the server is another robot, whose fleet path and ID this machine
        doesn't know, so leave the variables for the user to fill in on that robot.
        """
        fleet_path = os.environ.get('HELLO_FLEET_PATH')
        fleet_id = os.environ.get('HELLO_FLEET_ID')
        if fleet_path and fleet_id and self._targets_this_machine():
            return f"On this robot, in {os.path.join(fleet_path, fleet_id, 'stretch_user_params.yaml')}"
        return "On the remote robot, in ${HELLO_FLEET_PATH}/${HELLO_FLEET_ID}/stretch_user_params.yaml"

    def _connect_admin_socket(self):
        """
        (Re)create the admin REQ socket and its poller, closing any previous one.

        A REQ socket is stuck once it has sent a request that never got a reply, so a
        failed `check_connection()` leaves it unusable and it has to be replaced.
        """
        if self.socket_admin is not None:
            if self.admin_poller is not None:
                try:
                    self.admin_poller.unregister(self.socket_admin)
                except Exception:
                    pass
            self.socket_admin.close(linger=0)

        self.socket_admin = self.context.socket(zmq.REQ)
        self.socket_admin.setsockopt(zmq.LINGER, 0) # the purpose of this is to exit without hang when the server socket died, otherwise `socket.close()` waits for all queued commands to go out
        if self.ip_address is not None:
            self.socket_admin.connect(f"tcp://{self.ip_address}:23114")
        else:
            self.socket_admin.connect(f"ipc://{PORT_ADMIN}")

        self.admin_poller = zmq.Poller()
        self.admin_poller.register(self.socket_admin, zmq.POLLIN)

    def startup(self, *, verbose:bool = True, allow_different_user_connection:bool = False, block_timeout_s:float|None = None):
        """
        Connect this client to the Stretch Body Server.

        Opens the admin (REQ), command (PUB), and status (SUB) sockets, over IPC by
        default or over TCP when this client was constructed with an `ip_address`.

        Parameters
        ----------
        verbose : bool, optional
            If True, log an explanation when the connection cannot be made, by default True.
        allow_different_user_connection : bool, optional
            If True, connect even when the server was started by a different user on this
            machine. If False, the connection is refused in that case, by default False.
        block_timeout_s : float | None, optional
            How long to wait for the server to become available, retrying the connection
            every 100ms. If None, don't wait: try once and return False right away if no
            server responds. If 0, wait forever, so this call never returns if no server
            ever comes up. If greater than 0, wait up to that many seconds before giving
            up and returning False. By default None.

        Returns
        -------
        bool
            True if the client is connected to the server. False if the connection failed,
            in which case the sockets have already been cleaned up with `stop()`.
        """
        if not is_user_in_group('users'):
            if verbose:
                self.logger.error("StretchBodyClient: Cannot connect to the server because the current user is not a member of the 'users' group. The user should be a member of the 'users' group for locks to work properly.")
            return False

        # Start admin REQ-REP connection
        self.context = zmq.Context()
        self._connect_admin_socket()

        self.check_connection()

        if not self.connected and block_timeout_s is not None:
            if verbose:
                self.logger.info("StretchBodyClient: Waiting for the Stretch Body Server to become available...")
            deadline = None if block_timeout_s == 0 else time.time() + block_timeout_s
            while not self.connected and (deadline is None or time.time() < deadline):
                time.sleep(0.1)
                # A REQ socket with an unanswered request can't send again, so start fresh each retry
                self._connect_admin_socket()
                self.check_connection()

        if not self.connected:
            if verbose and self.ip_address is not None:
                self.logger.error(f"""
===============================================

StretchBodyClient: Not able to connect to the Stretch Body Server at {self.ip_address} (TCP ports 23114-23116).
StretchBodyClient: Check that the server is running on that robot (stretch_body_server --launch).
StretchBodyClient: Network access is disabled on the server by default.
StretchBodyClient: {self._remote_user_params_location()}, set:

robot:
  server:
    enable_network_access: '*' # or ip address of the remote machine

StretchBodyClient: then restart its server.

===============================================

""")
            elif verbose:
                self.logger.error("""
===============================================
                  
StretchBodyClient: Not able to connect to Stretch Body Server. Check that server is running
StretchBodyClient: Try running the server with stretch_body_server --launch
                  
===============================================
                  
""")
            self.stop()
            return False
        
        # The ownership check reads the local IPC socket file, so it only applies to a local server.
        # For a remote server (ip_address set) that file belongs to this machine, not the robot, and may not exist.
        if self.ip_address is None and not allow_different_user_connection and not StretchBodyServer.is_server_owned_by_current_user():
            if verbose:
                self.logger.error(f"""
===============================================
                
StretchBodyClient: A server is already running, but it was started by a different user ({StretchBodyServer.get_server_owning_user()}).
StretchBodyClient: You can run `stretch_body_server --kill` to forcefully end the other user's session.
                
===============================================
                
""")
            self.stop()
            return False

        # Start command PUB connection
        self.socket_cmd = self.context.socket(zmq.PUB)
        self.socket_cmd.setsockopt(zmq.SNDHWM, 1)
        self.socket_cmd.setsockopt(zmq.CONFLATE, 1)
        self.socket_cmd.setsockopt(zmq.LINGER, 0) # the purpose of this is to exit without hang when the server socket died, otherwise `socket.close()` waits for all queued commands to go out
        if self.ip_address is not None:
            self.socket_cmd.connect(f"tcp://{self.ip_address}:23115")
        else:
            self.socket_cmd.connect(f"ipc://{PORT_COMMAND}")

        # Start status SUB connection
        self.socket_status = self.context.socket(zmq.SUB)
        self.socket_status.setsockopt(zmq.CONFLATE, 1) # Only read latest message
        self.socket_status.setsockopt(zmq.SUBSCRIBE, b"") # Subscribe to all topics
        self.socket_status.setsockopt(zmq.LINGER, 0) # the purpose of this is to exit without hang when the server socket died, otherwise `socket.close()` waits for all queued commands to go out
        if self.ip_address is not None:
            self.socket_status.connect(f"tcp://{self.ip_address}:23116")
        else:
            self.socket_status.connect(f"ipc://{PORT_STATUS}")

        return True

    def stop(self):
        time.sleep(0.1) # Required here for all freewheel, etc. commands to transmit
        if self.socket_cmd is not None:
            self.socket_cmd.close(linger=0)
        if self.socket_status is not None:
            self.socket_status.close(linger=0)
        if self.socket_admin is not None:
            if self.admin_poller is not None:
                try:
                    self.admin_poller.unregister(self.socket_admin)
                except Exception:
                    pass
            self.socket_admin.close(linger=0)
        if self.context is not None:
            try:
                self.context.destroy(linger=0)
            except Exception:
                pass
        self._server_connected = False

    def __del__(self):
        try:
            self.stop()
        except Exception:
            pass

    @require_connection
    def _do_recv_status(self):        
        try:
            message = self.socket_status.recv_pyobj(flags=zmq.NOBLOCK)
            return message
        except zmq.Again:
            # No new data
            return None

    @require_connection
    def _do_send_cmd(self, cmd_dict, priority=0):
        self.cmd_seq += 1
        # Attach client_id
        message = {'client_id': self.client_id, 'cmd_dict': cmd_dict, 'priority': priority, 'seq': self.cmd_seq}

        # Send the message
        self.socket_cmd.send_pyobj(message)

    @require_connection
    def do_send_recv_admin_str(self, send, timeout=1.0):
        return self._do_send_recv_admin_str(send, timeout)
        
    def _do_send_recv_admin_str(self, send, timeout=1.0):
        try:
            self.socket_admin.send(send)

            # Poll to check if a status message is available within the timeout period
            if self.admin_poller.poll(int(timeout * 1000)):
                message = self.socket_admin.recv(flags=zmq.NOBLOCK)
                return message
        except zmq.ZMQError as e:
            return None