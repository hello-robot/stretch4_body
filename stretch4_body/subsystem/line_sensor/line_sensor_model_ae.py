#!/usr/bin/env python3
from stretch4_body.core.device import Device
from stretch4_body.core.pixart_j3.background_model import LineBackgroundModel
from stretch4_body.subsystem.line_sensor.line_sensor_loop import LineSensorLoop
import stretch4_body.core.hello_utils as hello_utils
import numpy
import time



class LineSensorModelAE(Device):
    def __init__(self, verbose=False):
        Device.__init__(self, 'line_sensor_model_ae')
        self.sensors = {}
        self.models = {}
        self.status = {}
        self.verbose = verbose
        self.model_update_stats = {}
        self.model_update_cntr=1
        self.ts_last_model_update_=time.time()
        self.loop_stats = hello_utils.LoopStats("LineSensorModelAE", 30)
    
    def pretty_print(self):
        print('----- LineSensorModelAE ------')
        for sn in self.params['sensor_names']:  
            print('%s: %s'%(sn,self.status[sn]['detection']))

    def startup(self):
        print('Starting LineSensorModelAE...')
        for sn in self.params['sensor_names']:
            if self.verbose:
                print('-------------------', sn, '---------------------')
            self.models[sn] = FloorBackgroundModel(sn, self.params['background_model_params'], self.verbose)
            try:
                self.models[sn].load_most_recent_calibration_data()
            except (FileNotFoundError, IndexError):
                print('Calibrated model not found for: %s' % sn)
                self.models[sn] = None
                return False
            self.status[sn] = {'detection': None}
            self.model_update_stats[sn] = {'last_update_cntr': 0}
        self.loop_stats.mark_loop_start()
        self.status['model_update_stats'] = self.loop_stats.status
        return True

    def stop(self):
        pass

    def step_range_adjust(self,loop_reader):
        for sn in self.params['sensor_names']:
            if self.models[sn] is not None:
                self.models[sn].step_range_adjust()

    # def step_model(self,loop_reader):
    #     for sn in self.params['sensor_names']:
    #         if loop_reader.is_sensor_updated(sn):
    #             #self.status_aux[sn]['ranges'] = self.loop_reader.status[sn]['ranges']
    #             try:
    #                 # Returns ['obstacle', 'floor', 'threshold']
    #                 self.model_update_stats[sn]['last_update_cntr'] = self.model_update_cntr
    #                 self.status[sn]['detection'] = self.models[sn].apply(loop_reader.status[sn]['ranges'], visualize=False, enlarge=False, output_image=False)['category']
    #                 self.status[sn]['scan_rate_hz'] = loop_reader.status[sn]['rate_hz']
    #             except numpy.linalg.LinAlgError:
    #                 self.status[sn]['detection'] = 'unknown'
    #                 self.status[sn]['scan_rate_hz'] = 0

        # all_updated=True
        # for sn in self.params['sensor_names']:
        #     if self.model_update_stats[sn]['last_update_cntr']!=self.model_update_cntr:
        #         all_updated=False
        # if all_updated:
        #     self.model_update_cntr+=1
        #     self.loop_stats.mark_loop_end()
        #     self.loop_stats.mark_loop_start()
        #     self.status['model_update_stats'] = self.loop_stats.status
        # return all_updated


if __name__ == '__main__':
    success=False
    lsl = LineSensorLoop()
    if lsl.startup():
        l = LineSensorModelCK(verbose=False)
        if l.startup():
            ts_update=time.time()
            try:
                for i in range(1000):
                    lsl.pull_status()
                    l.step_model(lsl)
                    time.sleep(0.01)
                    success=True
                    print('Itr',i,'Rate',l.status['model_update_stats']['curr_rate_hz'])

            except:
                pass
        lsl.stop()
        l.stop()
        if success:
            l.loop_stats.generate_rate_histogram()