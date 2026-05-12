
import stretch4_body.core.hello_utils as hello_utils
from stretch4_body.core.client_server import  StretchBodyClient, require_connection
from stretch4_body.core.device import Device
from stretch4_body.utils.thread_safe_dict import ThreadSafeDict
from stretch4_body.utils.freeable_file_lock import FreeableFileLock
import sys
import time
import uuid

# #####################################################################
class SubsystemClient(Device):
    """
    This manages the client networking interface.
    It can work for an individual subsystem (eg power_periph), or a set of subsystems (eg robot)
    This requires a 'parent', so that it the same class can support a RobotClient or a PowerPeriphClient (eg)
    This enables existing REx style tools to work with the server.
    """

    def __init__(self, name, client_id=None, parent=None, ip_address=None):
        Device.__init__(self,name)
        self.status = {}
        self.subsystems = {}
        self.is_valid=False
        self.parent = parent
        if self.parent == None: #No parent, so this instance owns the client connection
            self.client=StretchBodyClient(name=client_id, ip_address=ip_address)
            self.cmd_dict = ThreadSafeDict()
            self.push_lock = FreeableFileLock('pusher_client')
            self._last_push_command_time = 0.0
            self._last_rate_warn_time = 0.0
            self._startup_time = time.time()
        else:
            self.client = self.parent.client
            self.cmd_dict = self.parent.cmd_dict
            self.push_lock = self.parent.push_lock

    @property
    def connected(self):
        return self.is_valid and self.client.server_connected

    def startup(self, *, verbose:bool = True, allow_different_user_connection:bool=False):
        """
        `verbose`: Whether to print messages about the server starting up
        `allow_different_user_connection`: Whether to allow connecting to a server running as a different user on the same machine.
        """
        Device.startup(self)
        if self.parent is None:
            if not self.client.startup(verbose=verbose, allow_different_user_connection=allow_different_user_connection):
                return False
        self.is_valid=True
        for k in self.subsystems:
            self.is_valid=self.is_valid and self.subsystems[k].startup()
        if self.is_valid and self.parent is None:
            self.pull_status(blocking=True) #must populate initial dict
        return self.is_valid

    def stop(self):
        if not self.is_valid:
            return
        if self.push_lock.is_locked:
            self.push_lock.release()
        #A hard exit could happen in the middle of a command by the main loop, clear out socket recv
        for k in self.subsystems:
            self.subsystems[k].stop()
        if self.parent is None:
            self.push_command(ignore_control_lock=True) #Push out clean shutdown commands to the subsystems
            self.client.stop() #Close the sockets
        Device.stop(self)

    # ########## Control loop #############3

    @require_connection
    def pull_status(self, blocking=True):
        """
        Get latest status dict from server
        Copy to the subsystems.
        Should be called by user  at the start of each control cycle
        Returns
        -------
        Whether a status message got applied (only possible to miss when blocking=False)
        """
        if self.parent is not None:
            return self.parent.pull_status(blocking=blocking)

        status_server = self.client._do_recv_status()
        # if blocking:
        #     t_sleep_ms = (1 / self.robot_params['robot']['server']['control_loop_rate_Hz']) * 1000
        #     while status_server is None:
        #         status_server = self.client._do_recv_status(timeout_ms=t_sleep_ms/10) #poll 10x faster than control loop
        if blocking:
            while status_server is None:
                time.sleep(0.005)
                status_server = self.client._do_recv_status()
        if status_server is not None:
            #Reformat server dict to be more user friendly
            self.status['server']=status_server['server']
            self.status['routines']=status_server['routines']
            self.status['safety_layer']=status_server['safety_layer']

            if self.name in status_server:
                self.status.update(status_server[self.name]) #eg, robot.status['power_periph']
            elif 'end_of_arm' in status_server and self.name in status_server['end_of_arm']:
                self.status.update(status_server['end_of_arm'][self.name]) #eg, robot.status['end_of_arm']['wrist_yaw']

            for sn in self.subsystems: #Robot class only will have subsystems
                #self.subsystems[sn].status_server = status_server
                self.subsystems[sn].status['server'] = status_server['server']  # make copy so each subsystem can monitor the server / routines
                self.subsystems[sn].status['routines'] = status_server['routines']
                self.subsystems[sn].status['safety_layer'] = status_server['safety_layer']
                if sn in status_server:
                    self.subsystems[sn].status.update(status_server[sn])
        return status_server is not None

    @require_connection
    def push_command(self, ignore_control_lock=False, priority=0):
        """
        Send list of queued commands to server
        Should be called by user at the end of each control cycle
        Return True if successful
        """
        if self.parent is None:
            max_rate = 100.0
            if hasattr(self, 'robot_params') and type(self.robot_params) is dict:
                cfg = self.robot_params.get('robot', {})
                if type(cfg) is dict:
                    srv = cfg.get('server', {})
                    if type(srv) is dict:
                        max_rate = srv.get('max_push_command_rate_Hz', max_rate)

            if max_rate > 0:
                min_dt = 1.0 / max_rate
                now = time.time()
                
                dt = now - self._last_push_command_time
                if dt < min_dt:
                    # Only warn if the call is outside a 40% jitter margin (allow OS sleep inaccuracies)
                    if dt < (min_dt * 0.6):
                        if now - self._last_rate_warn_time > 1.0 and (now - self._startup_time > 2.0):
                            self.logger.warning(f"Warning: push_command rate throttled to {max_rate} Hz. Decrease push_command occurrences in your client loop.")
                            self._last_rate_warn_time = now
                    
                    sleep_time = min_dt - dt
                    time.sleep(sleep_time)
                    now = time.time()
                
                self._last_push_command_time = now

        if not ignore_control_lock:
            if not self.push_lock.is_locked:
                did_acquire = self.push_lock.acquire()
                if not did_acquire:
                    self.logger.error('Another process is already controlling Stretch. Try running "stretch_body_server --free_up_control"')
                    sys.exit(1)
                    return False

        if len(self.cmd_dict) == 0:
            return True
        self.client._do_send_cmd(self.cmd_dict, priority=priority)
        self.cmd_dict.clear()
        return True

    # ############# Helpers #################

    def pretty_print(self):
        print('--------------------------------------------')
        hello_utils.pretty_print_dict(self.name.capitalize(),self.status)
        print('')

    def get_subsystem(self,s):
        return self.subsystems.get(s, None)

    #Default APIs to override
    def is_homed(self):
        return True

    def is_moving(self):
        return False

    # ########## Server Admin #############
    def is_server_active(self):
        return self.client.server_connected

    def ping_server(self):
        ack = self.client._do_send_recv_admin_str(b"ping")
        self.client.server_connected = (ack == b"ping")
        return self.client.server_connected

    @require_connection
    def kill_server(self):
        ack = self.client._do_send_recv_admin_str(b"kill")
        self.client.server_connected = False

    # @require_connection
    # def pause_control_loop(self):
    #     ack = self.client._do_send_recv_admin_str(b"pause")
    #     self.client.server_connected = (ack ==b"pause")
    #
    # @require_connection
    # def unpause_control_loop(self):
    #     ack = self.client._do_send_recv_admin_str(b"unpause")
    #     self.client.server_connected = (ack ==b"unpause")

    @require_connection
    def free_up_control(self):
        ack = self.client._do_send_recv_admin_str(b"free_up_control", timeout=3.0)
        self.client.server_connected = (ack == b"free_up_control")

    # ########## Utility #############3

    def _wait_on_routine(self, routine_id, timeout=None, do_pull=True):
        try:
            return self._wait_on_status(lambda : self.status['routines']['last_routine_id']==routine_id, timeout, do_pull)
        except KeyboardInterrupt:
            self.logger.warning("\nKeyboard Interrupt detected. Cancelling routine...")
            self._queue_command("routines", "cancel")
            self.push_command()
            raise

    def _wait_on_status(self,cb_waiting_on,timeout=None, do_pull=True):
        """Poll status until cb_waiting_on returns true or timesout
        """
        if timeout == None:
            timeout = float('inf')
        ts=time.time()
        t_sleep=1/self.robot_params['robot']['server']['control_loop_rate_Hz']
        while time.time()-ts < timeout:
            if do_pull:
                self.pull_status()
            if cb_waiting_on():
                return True
            time.sleep(t_sleep)
        return False

    @staticmethod
    def _construct_command(subsystem, command,cmd_id, *args, **kwargs):
        cmd = [subsystem, command, cmd_id, args, kwargs]
        return cmd

    def _queue_command(self,subsystem, command, *args, **kwargs):
        # emit warning if overwriting existing command
        if subsystem in self.cmd_dict.keys():
            self.logger.warning(f"Warn: overwriting previous command for {subsystem}")

        # queue command for transmitting
        cmd_id = uuid.uuid1()
        cmd = self._construct_command(subsystem, command, cmd_id, *args, **kwargs)
        self.cmd_dict[subsystem] = cmd
        return cmd_id
    