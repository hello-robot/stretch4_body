#!/usr/bin/env python3
import time
import psutil
from stretch4_body.behavior.sentries.sentry import Sentry
from multiprocessing import Process, Event
from stretch4_body.core.worker_loop import worker_loop
import stretch4_body.core.hello_utils as hello_utils
import signal

def _cb_temp_loop_exit(worker_inst):
    return True

def _cb_temp_loop_pause(worker_inst):
    return True

def _cb_temp_loop_unpause(worker_inst):
    return True

def _cb_temp_loop_step(worker_inst, q_cmd_in, status_out):
    cpu_temp = 25.0
    try:
        t = psutil.sensors_temperatures()['coretemp']
        for c in t:
            cpu_temp = max(cpu_temp, c.current)
    except (KeyError, IOError, AttributeError):
        pass
    status_out['cpu_temp'] = cpu_temp
    status_out['ts_temp'] = time.time()
    return True

def temp_worker_loop(do_exit, rate_hz, q_admin, q_cmd, q_status):
    # Dummy worker instance as we don't have a complex object
    class CPUTempWorker:
        pass
    worker_inst = CPUTempWorker()
    worker_loop(
        loop_name='cpu_temp_loop',
        rate_hz=rate_hz,
        worker_instance=worker_inst,
        q_admin=q_admin,
        q_status=q_status,
        q_cmd=q_cmd,
        do_exit=do_exit,
        callback_step=_cb_temp_loop_step,
        callback_pause=_cb_temp_loop_pause,
        callback_unpause=_cb_temp_loop_unpause,
        callback_exit=_cb_temp_loop_exit
    )
    return True

class SentryCPUTemp(Sentry):
    """
    Sentry to monitor CPU temperature via a background WorkerLoop and toggle system fans dynamically.
    """
    def __init__(self, robot):
        Sentry.__init__(self, name="sentry_cpu_temp", robot=robot)
        self.status = {'cpu_temp': 0.0, 'ts_temp': 0}
        self.temp_process = None
        self.q_cmd = hello_utils.CircularMultiprocessingQueue(10)
        self.q_status = hello_utils.CircularMultiprocessingQueue(10)
        self.q_admin = hello_utils.CircularMultiprocessingQueue(10)
        self.do_exit = Event()
        self._ts_last_fan_on = None

    def startup(self):
        super().startup()
        if not self.is_valid:
            self.logger.warning('SentryCPUTemp Not Valid. Disabling.')
            return False
            
        if self.temp_process is None:
            self.temp_process = Process(
                target=temp_worker_loop,
                args=(self.do_exit, self.params.get('loop_rate_Hz', 0.5), self.q_admin, self.q_cmd, self.q_status)
            )
            self.temp_process.start()
        return True

    def _manage_ctrlC(self, *args):
        self.do_exit.set()

    def stop(self):
        self.logger.info('Stopping SentryCPUTemp process')
        original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._manage_ctrlC)
        self.q_admin.put('exit')
        if self.temp_process is not None:
            self.temp_process.join()
            self.temp_process = None
            
        self.q_admin.queue.cancel_join_thread()
        self.q_cmd.queue.cancel_join_thread()
        self.q_status.queue.cancel_join_thread()
        
        signal.signal(signal.SIGINT, original_sigint)
        return True

    def step(self):
        if not self.is_valid:
            return

        # Fetch latest from background queue
        s = self.q_status.get_latest()
        if s is not None:
            self.status['cpu_temp'] = s.get('cpu_temp', 0.0)
            self.status['ts_temp'] = s.get('ts_temp', 0)

        # Update the overall system status
        if self.robot is not None and getattr(self.robot, 'power_periph', None) is not None:
            if 'cpu_temp' in self.robot.power_periph.status:
                self.robot.power_periph.status['cpu_temp'] = self.status['cpu_temp']

            # Fan control logic
            if self.params.get('fan_control', True) and self.status['cpu_temp'] > 0:
                is_fan_on = self.robot.power_periph.status.get('fan_on', False)
                base_fan_on_thresh = self.params.get('base_fan_on', 60.0)
                base_fan_off_thresh = self.params.get('base_fan_off', 50.0)
                
                if self.status['cpu_temp'] > base_fan_on_thresh:
                    if self._ts_last_fan_on is None or time.time() - self._ts_last_fan_on > 3.0:
                        self.robot.power_periph.set_fan_on()
                        self._ts_last_fan_on = time.time()
                elif self.status['cpu_temp'] < base_fan_off_thresh:
                    if is_fan_on:
                        self.robot.power_periph.set_fan_off()
