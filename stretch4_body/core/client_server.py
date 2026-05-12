#!/usr/bin/env python3

import logging
import os
from pathlib import Path
import psutil
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
        self.message = f"{class_name} object is not connected. Call .startup() first."
        super().__init__(self.message)

def require_connection(function):
    @functools.wraps(function)
    def wrapper_function(self, *args, **kwargs):
        if not self.connected:
            raise NotConnectedError(self.__class__.__name__)

        return function(self, *args, **kwargs)
    return wrapper_function


class StretchBodyServer:
    def __init__(self):
        self.lease_holder_priority = None
        self.lease_holder_id = None
        self.lease_expiry = 0.0
        self.last_cmd_seq = None
        self.logger = logging.getLogger(name='stretch_body_server')

    def startup(self):
        if not is_user_in_group('users'):
            self.logger.error("Cannot start Stretch Body Server: User is not a member of the 'users' group. The user should be a member of the 'users' group for locks to work properly.")
            return False

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
            
            self.socket_status.bind("tcp://*:23116")
            self.logger.info(f'Starting Stretch Body Server Status publishing on port: {PORT_STATUS}')

            # Setup admin connection
            self.socket_admin = self.context.socket(zmq.REP)
            self.socket_admin.bind(f"ipc://{PORT_ADMIN}")

            self.socket_admin.bind("tcp://*:23114")
            self.logger.info(f'Starting Stretch Body Server Admin service on port: {PORT_ADMIN}')

            # Setup command subscribing
            self.socket_cmd = self.context.socket(zmq.SUB)
            self.socket_cmd.setsockopt(zmq.CONFLATE, 1) # Only read latest message
            self.socket_cmd.setsockopt(zmq.SUBSCRIBE, b"") # Subscribe to all topics
            self.socket_cmd.bind(f"ipc://{PORT_COMMAND}")
            
            self.socket_cmd.bind("tcp://*:23115")
            self.logger.info(f'Starting Stretch Body Server Command service on port: {PORT_COMMAND}')

            return True
        except (zmq.ZMQError, RuntimeError) as z:
            self.logger.error('Failed to start Stretch Body Server. It may already be running.')
            self.logger.error(z)
            return False

    def stop(self):
        if hasattr(self, 'socket_status'):
            self.socket_status.close(linger=0)
        if hasattr(self, 'socket_admin'):
            self.socket_admin.close(linger=0)
        if hasattr(self, 'socket_cmd'):
            self.socket_cmd.close(linger=0)
        if hasattr(self, 'context'):
            self.context.term()

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
        self.server_connected=False
        self.admin_poller=None
        self.socket_cmd=None
        self.socket_admin=None
        self.socket_status=None
        self.client_id= name if name is not None else self.client_id()
        self.ip_address=ip_address
        self.cmd_seq = 0

    @property
    def connected(self):
        return self.is_valid
    
    def client_id(self): 
        pid = os.getpid()
        pname = psutil.Process(pid).name()
        return f"client_{pname}_{pid}_{str(uuid.uuid4())[:8]}"

    def startup(self, *, verbose:bool = True, allow_different_user_connection:bool = False):
        if not is_user_in_group('users'):
            if verbose:
                print("StretchBodyClient: Cannot connect to the server because the current user is not a member of the 'users' group. The user should be a member of the 'users' group for locks to work properly.")
            return False

        # Start admin REQ-REP connection       
        self.context = zmq.Context()
        self.socket_admin = self.context.socket(zmq.REQ)
        self.socket_admin.setsockopt(zmq.LINGER, 0) # the purpose of this is to exit without hang when the server socket died, otherwise `socket.close()` waits for all queued commands to go out       
        if self.ip_address is not None:
            self.socket_admin.connect(f"tcp://{self.ip_address}:23114")
        else:
            self.socket_admin.connect(f"ipc://{PORT_ADMIN}")
        self.admin_poller = zmq.Poller()
        self.admin_poller.register(self.socket_admin, zmq.POLLIN)
        self.is_valid=True
        ack = self._do_send_recv_admin_str(b"ping")
        self.server_connected = (ack is not None)
        if ack is None:
            if verbose:
                print("""
===============================================
                  
StretchBodyClient: Not able to connect to Stretch Body Server. Check that server is running
StretchBodyClient: Try running the server with stretch_body_server --launch
                  
===============================================
                  
""")
            self.is_valid=False
            return False
        
        if not allow_different_user_connection and not StretchBodyServer.is_server_owned_by_current_user():
            if verbose:
                print(f"""
===============================================
                
StretchBodyClient: A server is already running, but it was started by a different user ({StretchBodyServer.get_server_owning_user()}).
StretchBodyClient: You can run `stretch_body_server --kill` to forcefully end the other user's session.
                
===============================================
                
""")
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

        # self.status_poller = zmq.Poller()
        # self.status_poller.register(self.socket_status, zmq.POLLIN)

        self.is_valid=True
        return True

    def stop(self):
        if self.is_valid:
            time.sleep(0.1) # Required here for all freewheel, etc. commands to transmit
            self.socket_cmd.close()
            self.socket_status.close()
            self.socket_admin.close()
            self.context.term()
            self.is_valid=False

    @require_connection
    def _do_recv_status(self, timeout_ms=None):
        # Check if messages available
        if timeout_ms is not None:
            if not self.status_poller.poll(int(timeout_ms)):
                return None
        
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
    def _do_send_recv_admin_str(self,send,timeout=1.0):
        self.socket_admin.send(send)

        # Poll to check if a status message is available within the timeout period
        if self.admin_poller.poll(int(timeout * 1000)):
            message = self.socket_admin.recv(flags=zmq.NOBLOCK)
            return message
