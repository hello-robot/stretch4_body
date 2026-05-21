#!/usr/bin/env python3
import time
from typing import TypedDict
from stretch4_body.core.device import Device
from multiprocessing import Process, Event
from stretch4_body.core.worker_loop import *
from stretch4_body.subsystem.line_sensor.pixart_j3_reader import PixartJ3Reader

# ###########################################################################################

def _cb_line_sensor_loop_exit(lsa):
    return True

def _cb_line_sensor_loop_pause(lsa):
    return True

def _cb_line_sensor_unpause(lsa):
    return True

def _cb_line_sensor_loop_step(pjr, q_cmd_in, status_out):
    if pjr.step():
        status_out.update(pjr.status)
    # status_aux_out.update(lsa.status_aux)
    return True

# ###########################################################################################

def line_sensor_loop(do_exit, rate_hz, q_admin, q_cmd, q_status,bus_sensor_map):
    """
    Do line sensor DAQ and model updates in its own process as can take 100% CPU
    Run at a high rate (100hz assuming that every 2 or 3 cycles all sensor models will be updated,
    as the sensor DAQ is asynchronous, at 30hz, to this loop..
    """
    pjr = PixartJ3Reader(verbose=False,bus_sensor_map=bus_sensor_map)
    if pjr.startup():
        worker_loop(
            loop_name='line_sensor_loop',
            rate_hz=rate_hz,
            worker_instance=pjr,
            q_admin=q_admin,
            q_status=q_status,
            q_cmd=q_cmd,
            do_exit=do_exit,
            callback_step=_cb_line_sensor_loop_step,
            callback_pause=_cb_line_sensor_loop_pause,
            callback_unpause=_cb_line_sensor_unpause,
            callback_exit=_cb_line_sensor_loop_exit
        )
        pjr.stop()
        return True
    return False

# ###########################################################################################

class LineSensorLoop(Device):
    """
    LineSensorLoop runs a background process that does the line sensor DAQ and model updates.

    """
    def __init__(self):
        Device.__init__(self, 'line_sensor_loop')
        self.pjr_process = None
        self.q_cmd = hello_utils.CircularMultiprocessingQueue(3)
        self.q_status = hello_utils.CircularMultiprocessingQueue(3)
        self.q_admin = hello_utils.CircularMultiprocessingQueue(3)
        self.status: "LineSensorLoopStatus" = {'last_frame_time':0, 'rate_hz': 0}
        self.status_aux = {}
        self.do_exit = Event()
        self.n_rate_log = 0
        self.rate_log={}
        self.frame_id_last = {}
        for sn in self.params['sensor_names']:
            self.rate_log[sn] = []
            self.frame_id_last[sn]=0
            self.status[sn]={'frame_id':0}

    def startup(self):
        """
        Launch the line sensor loop process.
        """
        self.logger.info('Starting LineSensorLoop...')
        timeout = False
        if self.pjr_process is None:
            self.pjr_process = Process(
                target=line_sensor_loop,
                args=(self.do_exit, self.params['loop_rate_Hz'], self.q_admin, self.q_cmd, self.q_status,self.params['bus_sensor_map'],)
            )
            self.pjr_process.start()
            #os.system("taskset -p -c %d %d" % (self.params['cpu_affinity'], self.pjr_process.pid)) #Assign process to core
            
            # Wait for system to start posting status
            ts=time.time()
            while self.status['last_frame_time']==0 and not timeout:
                self.status.update(self.q_status.get(block=True, timeout=0.1))
                if time.time()-ts>2.0:
                    timeout=True


        if timeout:
            self.logger.error('Timed out waiting for LineSensorLoop')
        return not timeout

    def _manage_ctrlC(self, *args):
        # If you have multiple event processing processes, set each Event.
        self.do_exit.set()

    def stop(self):
        original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._manage_ctrlC)
        self.q_admin.put('exit')
        if self.pjr_process is not None:
            self.pjr_process.join()
            self.pjr_process = None
            
        self.q_admin.queue.cancel_join_thread()
        self.q_cmd.queue.cancel_join_thread()
        self.q_status.queue.cancel_join_thread()
            
        signal.signal(signal.SIGINT, original_sigint)

    def push_command(self, blocking=False):
        pass


    def is_sensor_updated(self,sensor_name):
        return self.frame_id_last[sensor_name]!=self.status[sensor_name]['frame_id']

    def wait_on_sensor_updated(self,sensor_name,timeout=1.0):
        ts=time.time()
        while time.time()-ts<timeout:
            self.pull_status()
            if self.is_sensor_updated(sensor_name):
                return True
            time.sleep(.001)
        return False

    def pull_status(self, blocking=False):
        """
        Get latest status, empty queue. Non blocking.
        Empties the queue of older data.
        """

        for sn in self.params['sensor_names']:
            self.frame_id_last[sn]=self.status[sn]['frame_id']

        while True:
            try:
                um_status=self.q_status.get(block=False)
                #print(um_status.keys())
                if self.params['flip_range_ordering']:
                    for sn in self.params['sensor_names']:
                        if sn in um_status:
                            um_status[sn]['ranges']=um_status[sn]['ranges'][::-1]
                self.status.update(um_status)
                if self.n_rate_log:
                    for sn in self.params['sensor_names']:
                        self.rate_log[sn].append(self.status[sn]['rate_hz'])
                        if len(self.rate_log[sn])>self.n_rate_log:
                            self.rate_log[sn].pop(0)
            except queue.Empty:
                break

    def enable_rate_logging(self,max_samples=1000):
        self.n_rate_log=max_samples
    def get_rate_log(self):
        return self.rate_log
    def load_rpc_results(self, wait_on_result=True):
        """Not needed as no transport."""
        pass

class LineSensorLoopStatus(TypedDict):
    last_frame_time: float
    rate_hz: int


if __name__ == '__main__':
    pjl = LineSensorLoop()
    if pjl.startup():
        try:
            while True:
                pjl.pull_status()
                print('Rate: %f (Hz)'%pjl.status['rate_hz'])#['sensor_0'])
                #print('Model update rate: ', pjl.status['model_update_stats']['curr_rate_hz'])
                time.sleep(0.01)
        except:
            pjl.stop()
