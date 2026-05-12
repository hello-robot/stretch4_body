#!/usr/bin/env python3
import time
import stretch4_body.core.hello_utils as hello_utils
import os
from stretch4_body.subsystem.end_of_arm.end_of_arm import EndOfArm
from stretch4_body.core.device import Device
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.core.subsystem_client import SubsystemClient
import queue
from multiprocessing import Process, Event
import signal
from stretch4_body.core.worker_loop import *
import importlib
from stretch4_body.core.subsystem_client import SubsystemClient
import uuid

# ###########################################################################################

def _cb_end_of_arm_loop_exit(eoa):
    return True


def _cb_end_of_arm_loop_pause(eoa):
    return True


def _cb_end_of_arm_unpause(eoa):
    return True


def _cb_end_of_arm_loop_step(eoa, q_cmd_in, status_out):
    eoa.pull_status()
    status_out.update(eoa.status)
    while q_cmd_in.qsize():
        try:
            cmd=q_cmd_in.get(block=False)
            subsystem, method, cmd_id,args, kwargs = cmd
            try:
                method_to_call = getattr(eoa, method)
                method_to_call(*args, **kwargs)
                #self.cmd_results[cmd_id] = {'ts': time.time(), 'result': method_to_call(*args, **kwargs)}
            except AttributeError:
                print('EndOfArmLoop _cb_end_of_arm_loop_step : invalid  cmd', cmd)
        except queue.Empty:
            pass
    return True

# ###########################################################################################
#Todo: move eoa sentry to sentry_manager, where a eoa.trigger_runstop(joint_name) can be called and push to the loop

def end_of_arm_loop_worker(do_exit, rate_hz, q_admin, q_cmd, q_status):
    """
    """
    eoa_name = RobotParams.eoa_name
    rp=RobotParams._robot_params
    module_name = rp[eoa_name]['py_module_name']
    class_name = rp[eoa_name]['py_class_name']
    eoa = getattr(importlib.import_module(module_name), class_name)()
    if eoa.startup():
        worker_loop(
            loop_name='end_of_arm_loop',
            rate_hz=rate_hz,
            worker_instance=eoa,
            q_admin=q_admin,
            q_status=q_status,
            q_cmd=q_cmd,
            do_exit=do_exit,
            callback_step=_cb_end_of_arm_loop_step,
            callback_pause=_cb_end_of_arm_loop_pause,
            callback_unpause=_cb_end_of_arm_unpause,
            callback_exit=_cb_end_of_arm_loop_exit
        )

        eoa.stop()
        print('End_of_arm_loop_worker done!')
        return True
    return False

# ###########################################################################################

class EndOfArmLoop(Device):
    """
    EndOfArmLoop runs a background process that provides looping status / command to the end of arm servos

    """
    def __init__(self):
        Device.__init__(self, 'end_of_arm_loop')
        self.eoa_process = None
        self.q_cmd = hello_utils.CircularMultiprocessingQueue(100)
        self.q_status = hello_utils.CircularMultiprocessingQueue(100)
        self.q_admin = hello_utils.CircularMultiprocessingQueue(100)
        self.status = {}
        self.status_aux={}
        self.do_exit = Event()
        self.n_rate_log = 0
        self.rate_log = []

    def startup(self):
        """
        Launch the line sensor loop process.
        """
        self.logger.info('Starting EndOfArm...')
        if self.eoa_process is None:
            self.eoa_process = Process(
                target=end_of_arm_loop_worker,
                args=(self.do_exit, self.params['loop_rate_Hz'], self.q_admin, self.q_cmd, self.q_status)
            )
            self.eoa_process.start()
            self.logger.info('Started EndofArmLoop process %d'%self.eoa_process.pid)
            #os.system("taskset -p -c %d %d" % (self.params['cpu_affinity'], self.eoa_process.pid)) #Assign process to core
            # Wait for system to start posting status
            try:
                self.status.update(self.q_status.get(block=True, timeout=5.0))
                return True
            except queue.Empty:
                self.logger.error('Failed to start EndOfArmLoop, timed out')
                return False
        self.logger.error('Failed to start EndOfArmLoop')
        return False

    def _manage_ctrlC(self, *args):
        # If you have multiple event processing processes, set each Event.
        self.do_exit.set()

    def stop(self):
        print('Stopping EndOfArmLoop')
        original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._manage_ctrlC)
        self.q_admin.put('exit')
        if self.eoa_process is not None:
            self.eoa_process.join()
            self.eoa_process = None
            
        self.q_admin.queue.cancel_join_thread()
        self.q_cmd.queue.cancel_join_thread()
        self.q_status.queue.cancel_join_thread()
            
        signal.signal(signal.SIGINT, original_sigint)

    def push_command(self, blocking=False):
        pass

    def is_homed(self):
        return self.status['is_homed']

    def pull_status(self, blocking=False):
        """
        Get latest status, empty queue. Non blocking.
        Empties the queue of older data.
        """
        while self.q_status.qsize():
            try:
                self.status.update(self.q_status.get(block=False))
                if self.n_rate_log:
                    self.rate_log.append(self.status['loop']['stats']['curr_rate_hz'])
                    if len(self.rate_log)>self.n_rate_log:
                        self.rate_log.pop(0)
                # print(self.status)
            except queue.Empty:
                pass

    def enable_rate_logging(self,max_samples=1000):
        self.n_rate_log=max_samples

    def get_rate_log(self):
        return self.rate_log


    def step_sentry(self,robot_status):
        if 'power_periph' in robot_status:
            cmd_id=uuid.uuid1()
            cmd=SubsystemClient._construct_command('end_of_arm','step_sentry',cmd_id,{'power_periph':robot_status['power_periph']})
            self.q_cmd.put(cmd)

    def step_collision_avoidance(self,joint_name,in_collision_stop):
        cmd_id = uuid.uuid1()
        cmd = SubsystemClient._construct_command('end_of_arm', 'step_collision_avoidance', cmd_id,joint_name,in_collision_stop)
        self.q_cmd.put(cmd)

if __name__ == '__main__':
    eoa = EndOfArmLoop()
    if eoa.startup():
        try:

                while True:
                    eoa.pull_status()
                    print('-------------')
                    #print(eoa.status['loop'])
                    print('EOA update rate: ', eoa.status['loop']['stats']['curr_rate_hz'])
                    time.sleep(0.1)
        except (KeyboardInterrupt, SystemExit):
            pass
    eoa.stop()

