#!/usr/bin/env python3
import os
import time
import uuid
from typing import TypedDict
import numpy as np
from stretch4_body.core.device import Device
from multiprocessing import Process, Event
from stretch4_body.core.worker_loop import *
import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.line_sensor import calibration, calibration_store
from stretch4_body.subsystem.line_sensor.pixart_j3_reader import PixartJ3Reader

# ###########################################################################################

def _cb_line_sensor_loop_exit(lsa):
    return True

def _cb_line_sensor_loop_pause(lsa):
    return True

def _cb_line_sensor_unpause(lsa):
    return True

def _cb_line_sensor_loop_step(pjr, q_cmd_in, status_out):
    """Publish the reader status, every time anything moved.

    """
    while q_cmd_in.qsize():
        try:
            cmd = q_cmd_in.get_nowait()
            subsystem, method, cmd_id, args, kwargs = cmd
            getattr(pjr, method)(*args, **kwargs)
        except queue.Empty:
            break
        except AttributeError:
            print(f'LineSensorLoop: no such reader command: {cmd}')
        except Exception as e:
            # A bad command must not take the reader down with it.
            print(f'LineSensorLoop: command {cmd} failed: {e}')
    if pjr.step():
        status_out.update(pjr.status)
    status_out['health'] = pjr.health()
    return True

# ###########################################################################################

def line_sensor_loop(do_exit, rate_hz, q_admin, q_cmd, q_status, bus_sensor_map,
                     flip_range_ordering, report_num):
    """
    Do line sensor DAQ and model updates in its own process as can take 100% CPU
    Run at a high rate (100hz assuming that every 2 or 3 cycles all sensor models will be updated,
    as the sensor DAQ is asynchronous, at 30hz, to this loop..
    """
    pjr = PixartJ3Reader(verbose=False, bus_sensor_map=bus_sensor_map,
                         flip_range_ordering=flip_range_ordering,
                         report_num=report_num)
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
        # Commands get room (matching end_of_arm), status does not.
        self.q_cmd = hello_utils.CircularMultiprocessingQueue(100)
        self.q_status = hello_utils.CircularMultiprocessingQueue(3)
        self.q_admin = hello_utils.CircularMultiprocessingQueue(3)
        self.n_bins = int(self.params['line_sensor_geometry']['pixart_report_num'])
        self.status: "LineSensorLoopStatus" = {
            'last_frame_time': 0, 'rate_hz': 0,
            'health': {'streaming': False, 'port_open': False, 'rate_hz': 0,
                       'last_frame_time': 0, 'disabled_sensors': [],
                       'sensors_dead': list(self.params['sensor_names']),
                       'decode_errors': 0, 'frame_advance_err': 0,
                       'frame_not_full_err': 0, 'not_six_sensors_err': 0,
                       'reader_restarts': 0},
            'calibration': {'loaded': [], 'rejected': {}, 'id': '',
                            'n_bins': self.n_bins}}
        self.status_aux = {}
        self.do_exit = Event()
        self.n_rate_log = 0
        self.rate_log={}
        self.frame_id_last = {}
        for sn in self.params['sensor_names']:
            self.rate_log[sn] = []
            self.frame_id_last[sn]=0
            self.status[sn] = {
                'ts_last_read': 0, 'frame_id': 0, 'rate_hz': 0,
                'ranges': np.zeros(0, dtype=np.float64),
                'codes': np.zeros(0, dtype=np.uint8),
                'n_no_return': 0, 'n_beyond_limit': 0, 'enabled': True,
                'missed_frames': PixartJ3Reader.DEAD_AFTER_MISSED_FRAMES}

    def startup(self):
        """
        Launch the line sensor loop process.
        """
        self.logger.info('Starting LineSensorLoop...')
        timeout = False
        if self.pjr_process is None:
            self.pjr_process = Process(
                target=line_sensor_loop,
                args=(self.do_exit, self.params['loop_rate_Hz'], self.q_admin,
                      self.q_cmd, self.q_status, self.params['bus_sensor_map'],
                      self.params['flip_range_ordering'],
                      self.params['line_sensor_geometry']['pixart_report_num'],)
            )
            self.pjr_process.start()
            #os.system("taskset -p -c %d %d" % (self.params['cpu_affinity'], self.pjr_process.pid)) #Assign process to core
            
            # Wait for system to start posting status.
            ts=time.time()
            while self.status['last_frame_time']==0 and not timeout:
                try:
                    self.status.update(self.q_status.get(block=True, timeout=0.1))
                except queue.Empty:
                    pass
                if time.time()-ts>2.0:
                    timeout=True


        if timeout:
            self.logger.error('Timed out waiting for LineSensorLoop')
        self.load_calibration()
        return not timeout

    # -- calibration -------------------------------------------------------

    def calibration_base_dir(self):
        return os.path.join(hu.get_fleet_directory(), 'calibration_line_sensors')

    def load_calibration(self, verbose=True):
        """Load, validate and publish the tares.

        A refusal leaves that sensor uncalibrated and says why. It is never
        downgraded to a warning: running uncalibrated is recoverable, running
        on a tare from a different configuration is not.
        """
        base = self.calibration_base_dir()
        block = {'loaded': [], 'rejected': {}, 'n_bins': self.n_bins,
                 'base_dir': base}
        for idx, name in enumerate(self.params['sensor_names']):
            path = calibration_store.tare_path(base, name)
            try:
                fp = calibration.config_fingerprint(name, idx, self.params)
                tare = calibration_store.load_validated_tare(path, fp, self.n_bins)
                entry = calibration.pack_tare(tare.offsets, tare.valid_mask,
                                              tare.null_rate_per_bin)
                entry.update({
                    'timestamp': tare.timestamp,
                    'session_id': tare.session_id,
                    'fingerprint_sha256': tare.fingerprint_sha256,
                    'ideal_range_m': float(np.mean(tare.ideal_range)),
                    'n_valid_bins': int(tare.valid_mask.sum()),
                })
                block[name] = entry
                block['loaded'].append(name)
            except Exception as exc:
                reason = getattr(exc, 'reason', exc.__class__.__name__)
                block['rejected'][name] = f'{reason}: {exc}'
                if verbose:
                    self.logger.warning('%s: NO TARE (%s)', name, reason)
        # An id over what was actually loaded, so a client can cache the
        # unpacked arrays and notice a recalibration without diffing 320-point
        # arrays on every status message.
        block['id'] = calibration.fingerprint_hash(
            {n: block[n]['fingerprint_sha256'] + block[n]['timestamp']
             for n in block['loaded']})
        self.status['calibration'] = block
        if verbose:
            self.logger.info('Line sensor calibration: %d/%d sensors loaded',
                             len(block['loaded']), len(self.params['sensor_names']))
        return block

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
            

            
        signal.signal(signal.SIGINT, original_sigint)

    # -- runtime control ---------------------------------------------------
    #
    # The server's generic dispatch calls these directly on this object (the
    # parent process). The reader lives in the child, so each one forwards
    # onto q_cmd, which the step callback drains. 

    def _queue_reader_command(self, method, *args, **kwargs):
        self.q_cmd.put(['line_sensor_loop', method, uuid.uuid1(), args, kwargs])

    def set_streaming(self, on):
        """Pause or resume decoding without restarting anything.

        Turning this off removes cliff detection. It is reported in
        status['health']['streaming'] so a consumer can refuse to drive, and
        it is deliberately not persisted -- a restart always comes up
        streaming.
        """
        self.logger.warning('Line sensors streaming -> %s',
                            'ON' if on else 'OFF (cliff detection is off)')
        self._queue_reader_command('set_streaming', bool(on))

    def set_sensor_enabled(self, sensor_name, on):
        """Turn one sensor's decoding on or off. Not persisted."""
        if sensor_name not in self.params['sensor_names']:
            raise ValueError(f'unknown sensor: {sensor_name!r}')
        self.logger.warning('%s -> %s', sensor_name,
                            'ENABLED' if on else 'DISABLED')
        self._queue_reader_command('set_sensor_enabled', sensor_name, bool(on))

    def push_command(self, blocking=False):
        """No-op: commands reach the reader through q_cmd as they are
        dispatched, so there is nothing to flush at the end of a cycle."""
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

        while self.q_status.qsize():
            try:
                # flip_range_ordering is applied at decode in PixartJ3Reader;
                # messages arrive here in final bin order.
                um_status=self.q_status.get(block=False)
                self.status.update(um_status)
                if self.n_rate_log:
                    for sn in self.params['sensor_names']:
                        self.rate_log[sn].append(self.status[sn]['rate_hz'])
                        if len(self.rate_log[sn])>self.n_rate_log:
                            self.rate_log[sn].pop(0)
            except queue.Empty:
                pass

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
