#!/usr/bin/env python3
from stretch4_body.core.device import Device
import time
import stretch4_body.core.hello_utils as hello_utils
from stretch4_body.robot.robot_core import RobotCore
from stretch4_body.core.client_server import StretchBodyServer
from stretch4_body.behavior.safe_motions.safe_motion_manager import SafeMotionManager
from stretch4_body.behavior.sentries.sentry_manager import SentryManager
from stretch4_body.behavior.routines.routine_manager import RoutineManager
from stretch4_body.subsystem.line_sensor.line_sensor_loop import LineSensorLoop
from stretch4_body.utils.freeable_file_lock import FreeableFileLock

import logging
import importlib
from colorama import Fore, Back, Style, init
import psutil
import os


# ###########################################################################################
LOOP_STATE_INVALID = "INVALID"
LOOP_STATE_RUNNING = "RUNNING"
LOOP_STATE_PAUSED = "PAUSED"


class RobotServer(RobotCore):
    """
    Extend a standard RobotCore instance with additional RobotController specific subsystems.
    This runs as an independent process, and is managed by the RobotServer.
    """

    def __init__(self):
        RobotCore.__init__(self)
        # Extend RobotCore with additional RobotServer specific subsystems and behaviors

        # Handle end_of_arm in Robot instead of RobotCore, as RobotServer will handle it differently
        if 'end_of_arm' in self.params['subsystems']:
            from stretch4_body.subsystem.end_of_arm.end_of_arm_loop import EndOfArmLoop
            self.eoa_name = self.params['tool']
            self.eoa_loop = self.end_of_arm = self.subsystems['end_of_arm'] = EndOfArmLoop()
            if self.robot_params['robot']['enable_rate_log']:
                self.eoa_loop.enable_rate_logging(self.robot_params['robot']['max_rate_log_samples'])
            self.status['end_of_arm'] = self.subsystems['end_of_arm'].status
            self.status_aux['end_of_arm'] = self.subsystems['end_of_arm'].status_aux
            if 'lift' in self.params['subsystems']:
                self.lift.set_i_feedforward_payload(self.robot_params[self.eoa_name]['i_feedforward_payload'])

        # Instantiate server-onlysubsystems
        for k in self.params['server']['subsystems']: #not available to RobotDirect
            if k == 'line_sensor_loop':
                self.line_sensor_loop = self.subsystems[k] = LineSensorLoop()
                if self.robot_params['robot']['enable_rate_log']:
                    self.line_sensor_loop.enable_rate_logging(self.robot_params['robot']['max_rate_log_samples'])
            self.status[k] = self.subsystems[k].status
            self.status_aux[k] = self.subsystems[k].status_aux

        self.server = StretchBodyServer()

        self.do_exit = False
        self.state = LOOP_STATE_INVALID
        self.loop_mgmt = hello_utils.LoopStats("control_loop", self.params['server']['control_loop_rate_Hz'])

        
        # Instantiate sentry
        self.sentry_manager=SentryManager(self)

        # Instantiate safe_motion checks
        self.safe_motion_manager=SafeMotionManager(self)

        # Instantiate routines
        self.routine_manager=RoutineManager(self)

        self.status['server']={'control_loop':self.loop_mgmt.status, 
                                'state': self.state, 
                                'lease_holder': self.server.lease_holder_id, 
                                'lease_holder_priority': self.server.lease_holder_priority,
                                'lease_expiry': self.server.lease_expiry}
        self.status['routines']=self.routine_manager.status
        self.status['server']['status_id'] =0
        self.status['safety_layer']={
            'safe_motion_manager':self.safe_motion_manager.status,'sentry_manager':self.sentry_manager.status}

        self.cmd_results={}
        
        # CPU Monitoring
        self.status['server']['cpu'] = {}
        self.psutil_processes = {}
        self.last_cpu_update = 0.0
        # We delay process map initialization until startup or first update to ensure processes are running?


    def startup(self):
        if (RobotCore.startup(self) and
            self.sentry_manager.startup() and
            self.routine_manager.startup() and
            self.safe_motion_manager.startup() and
            self.server.startup()):
            self.logger.info("RobotServer started successfully.")
            self.state = LOOP_STATE_RUNNING
            return True
        else:
            self.logger.error("Failed to start RobotServer.")
            return False

    def is_homed(self):
        eoa=True
        if self.get_subsystem('end_of_arm') is not None:
            eoa=self.subsystems['end_of_arm'].is_homed()
        return eoa and RobotCore.is_homed(self)

    def pause_sentry(self, sentry_name):
        self.sentry_manager.pause([sentry_name])

    def unpause_sentry(self, sentry_name):
        self.sentry_manager.unpause([sentry_name])

    def _cb_admin_dispatch(self,message):

        if message == b"ping":
            print("RobotServer PING!")
            self.logger.debug('RobotServer PING!')
            return message

        if message == b"pause" and self.state == LOOP_STATE_RUNNING:
            self.logger.warn('RobotServer PAUSE!')
            self.safe_motion.enter_safe_stop()
            self.pause_transport()
            hello_utils.free_transport_filelock(self.name)
            self.state = LOOP_STATE_PAUSED
            return message

        if message == b'unpause' and self.state == LOOP_STATE_PAUSED:
            if not hello_utils.acquire_transport_filelock(self.name):
                self.logger.error('Unable for RobotServer to aquire transport_filelock. Not able to UNPAUSE.')
            else:
                self.logger.warn('RobotServer UNPAUSE!')
                self.safe_motion.exit_safe_stop()
                self.state = LOOP_STATE_RUNNING
                self.unpause_transport()
            return message

        if message == b"free_up_control":
            lock = FreeableFileLock('pusher_client')
            lock.free()
            self.server.lease_holder_id = None # free up lease
            self.server.lease_holder_priority = None
            return message

        if message == b"kill":
            self.logger.warn('RobotServer KILL!')
            hello_utils.free_transport_filelock(self.name)
            self.stop()
            self.server.stop()
            self.state = LOOP_STATE_INVALID
            self.do_exit = True
            return message
        return None



    def _cb_command_dispatch(self,cmd_dict):
        """
        #Recieve a dict of commands from client with subsystem -> command key-value mapping
        # A command has the form (subsystem_name,command_name,args,kwargs,cmd_id)
        #Dispatch each command to the appropriate subsystem

        """
        cmd_ids_dispatched=[]
        if self.state == LOOP_STATE_PAUSED or self.state == LOOP_STATE_INVALID:
            return cmd_ids_dispatched #Ignore commands while paused, discard

        is_routine_active = self.routine_manager.status['active_routine'] != 'routine_nop'

        for cmd in cmd_dict:
            subsystem, method, cmd_id, args, kwargs = cmd_dict[cmd]

            if subsystem =='routines':
                if method == 'cancel':
                    self.routine_manager.cancel(*args, **kwargs)
                else:
                    self.routine_manager._routine_set_next(method,cmd_id,*args,**kwargs)
                
                cmd_ids_dispatched.append(cmd_id)
                continue

            if is_routine_active:
                # Reject non-routine commands while a routine is executing
                continue

            # Handle robot commands:
            subsystem_instance = self

            if subsystem !='robot':
                if 'end_of_arm' in subsystem:
                    subsystem = 'end_of_arm' # remove joint from e.g. "end_of_arm.wrist_yaw" to "end_of_arm"
                elif 'omnibase' in subsystem:
                    subsystem = 'omnibase'
                subsystem_instance = self.get_subsystem(subsystem)

            if subsystem_instance is not None:
                if subsystem=='end_of_arm':

                    self.eoa_loop.q_cmd.put(cmd_dict[cmd])
                    cmd_ids_dispatched.append(cmd_id)
                else:
                    try:
                        method_to_call=getattr(subsystem_instance,method)
                        self.cmd_results[cmd_id]={'ts':time.time(),'result':method_to_call(*args, **kwargs)}
                        cmd_ids_dispatched.append(cmd_id)
                    except AttributeError:
                        self.logger.error(f'RobotServer _cb_command_dispatch : invalid  cmd {cmd}')
            else:
                self.logger.error(f'RobotServer not able to run command {method} as subsystem {subsystem} is not present')
        # Cleanup cmd results to keep max history using O(1) operations
        MAX_CMD_HISTORY = 5000
        while len(self.cmd_results) > MAX_CMD_HISTORY:
            # In Python 3.7+, dicts maintain insertion order. iter() gets the oldest key.
            oldest_key = next(iter(self.cmd_results))
            del self.cmd_results[oldest_key]
        return cmd_ids_dispatched

    def publish_status_msg(self):
        self.update_cpu_status()
        self.status['server']['state']=self.state
        self.status['server']['status_id'] = self.status['server']['status_id'] + 1
        self.status['server']['lease_holder'] = str(self.server.lease_holder_id)
        self.status['server']['lease_holder_priority'] = str(self.server.lease_holder_priority)
        self.status['server']['lease_expiry'] = self.server.lease_expiry
        
        self.server.publish_status(self.status)

    def update_cpu_status(self):
        # Throttle to 1Hz
        if time.time() - self.last_cpu_update < 1.0:
            return
        self.last_cpu_update = time.time()

        # Initialize psutil processes on first run if empty, but only if running

        if not self.psutil_processes and self.state == LOOP_STATE_RUNNING:
             # Main process
            try:
                self.psutil_processes['server'] = psutil.Process(os.getpid())
            except psutil.NoSuchProcess:
                pass

            # End of Arm
            if hasattr(self, 'eoa_loop') and self.eoa_loop.eoa_process:
                try:
                    self.psutil_processes['end_of_arm'] = psutil.Process(self.eoa_loop.eoa_process.pid)
                except psutil.NoSuchProcess:
                    pass
            
            # Line Sensor
            if hasattr(self, 'line_sensor_loop') and self.line_sensor_loop.pjr_process:
                try:
                    self.psutil_processes['line_sensor'] = psutil.Process(self.line_sensor_loop.pjr_process.pid)
                except psutil.NoSuchProcess:
                    pass

            # Self Collision Loop
   
            # Located in sentry_manager -> sentries['self_collision_loop']
            if 'sentry_self_collision' in self.sentry_manager.sentries:
                 scl = self.sentry_manager.sentries['sentry_self_collision'].self_collision_loop
                 if scl.solver_process:
                     try:
                        self.psutil_processes['self_collision'] = psutil.Process(scl.solver_process.pid)
                     except psutil.NoSuchProcess:
                        pass

        # Update status
        for name, proc in self.psutil_processes.items():
            try:
                # cpu_percent(interval=None) is non-blocking after first call
                self.status['server']['cpu'][name] = proc.cpu_percent(interval=None)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                self.status['server']['cpu'][name] = -1.0


    def _print_server_state(self):
        if time.time() - self.t_print_last > 1.0:
            msg = (f'StretchBodyServer : State: {self.state} | Routine: {self.routine_manager.status['active_routine'].upper()} | Runtime {self.status['server']['control_loop']['execution_time_s']:.8f} (s) | Rate {self.status['server']['control_loop']['avg_rate_hz']:.2f} (Hz)')
            print(msg)
            self.logger.debug(msg)
            self.t_print_last = time.time()

    def run_controller(self):
        self.t_print_last = time.time()
        use_tqdm=0
        if use_tqdm:
            from tqdm import tqdm
            pbar = tqdm(unit="hz")
        self.loop_mgmt.reset()
        self.logger.info("""
==================================

The Stretch Body Server Control Loop is running.

Use `stretch_body_server --help` for available options.
Use `stretch_status_viz --fields robot.server robot.routines --print` to view the status of the robot.

==================================
""")
        try:
            while not self.do_exit:

                if self.state == LOOP_STATE_RUNNING:
                    self.routine_manager._routine_run_next()
                else:
                    if self.state == LOOP_STATE_PAUSED or self.state == LOOP_STATE_INVALID:
                        self.loop_mgmt.mark_loop_start()
                        self.loop_mgmt.busy_wait_until_next_cycle(busy_wait_ms=0.1, warn_delay=5.0, overrun_thresh_s=0.005)
                        self.server.dispatch_admin_messages(self._cb_admin_dispatch)
                        self.server.dispatch_command_messages(self._cb_command_dispatch, self.routine_manager.status['active_routine'] != 'routine_nop')
                        self.publish_status_msg()
                        self.loop_mgmt.mark_loop_end()
                if use_tqdm:
                    pbar.update(1)
                else:
                    self._print_server_state()

        except KeyboardInterrupt:
            self.logger.info("Received request to stop stretch body server via keyboard interrupt.")
        except Exception as e:
            self.logger.error(f"Error in stretch body server: {e}")


    def cb_routine_update_controller(self):
        """
        Called by a routine in order to pull status/ push command etc
        Return True if success / Routine should continue
        Return False if no success / Routine should stop gracefully and exit

        This is effectively called in the middle of the control cycle. The control cycle is
        1. (Mark start) Do admin of the control loop
        2. Wait until start of cycle
        3. Trigger a new pull_status (non blocking)
        4. Update sentries
        5. Ingest new control commands from Client
        6. Run routines, possibly overriding control commands
        7. Run safe motion limits, possibly overriding control commands
        8. Push commands to uC
        9. Collect the results of the pull status (for next control cycle) and post to Client
        10. Mark end of loop.

        As this is called from a Routine, we start the cycle at step #7 and wrap around up to 5.
        """

        # 7
        self.safe_motion_manager.step()

        # 8
        self.push_command(blocking=False)
        if self.get_subsystem('power_periph') is not None:
            self.power_periph.trigger_motor_sync(blocking=False)

        # 9
        self.load_rpc_results(wait_on_result=True)
        self.publish_status_msg()


        # 10
        self.loop_mgmt.mark_loop_end()

        # 1
        self.loop_mgmt.mark_loop_start()
        self.server.dispatch_admin_messages(self._cb_admin_dispatch)# May pause/exit
        if self.state is not LOOP_STATE_RUNNING:
            return False
        # 2
        self.loop_mgmt.busy_wait_until_next_cycle(busy_wait_ms=1.0, warn_delay=5.0, overrun_thresh_s=0.005)

        # 3
        self.pull_status(blocking=False)

        # 4 Step sentries
        self.sentry_manager.step()

        # 5 Get new commands from client
        self.server.dispatch_command_messages(self._cb_command_dispatch, self.routine_manager.status['active_routine'] != 'routine_nop')

        self._print_server_state()
        return True

    def stop(self):
        self.sentry_manager.stop()
        self.safe_motion_manager.stop()
        self.routine_manager.stop()
        RobotCore.stop(self)
        if hasattr(self, 'server') and hasattr(self.server, 'stop'):
            self.server.stop()



# ###########################################################################################
def run_server():
    """
    Launch the RobotController as a seperate process and manage the network interface to it in
    the loop below.
    """
    # num_cores = os.cpu_count()
    rs = RobotServer()
    if not rs.startup():
        rs.logger.error('Failure to start RobotServer')
        rs.stop()
        exit(1)

    rs.run_controller()
    rs.stop()


# ###########################################################################################

if __name__ == '__main__':
    run_server()



