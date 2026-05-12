#!/usr/bin/env python3
import time
from stretch4_body.core.device import Device
import stretch4_body.subsystem.omnibase as omnibase
import stretch4_body.subsystem.arm as arm
import stretch4_body.subsystem.lift as lift
import stretch4_body.subsystem.power_periph as power_periph
import stretch4_body.core.hello_utils as hello_utils
import json
import pathlib

class RobotCore(Device):
    """
    RobotCore provides the common, minimal management of subsystems
    and utility methods.
    It is used by both Robot and RobotServer.
    """

    def __init__(self):
        Device.__init__(self, 'robot')
        self.subsystems = {}
        self.status = {}
        self.status_aux = {}
        for k in self.params['subsystems']:
            if k == 'power_periph':
                self.power_periph = power_periph.PowerPeriph()
                self.subsystems[k] = self.power_periph
                self.pimu = self.power_periph  # legacy naming
                self.status['pimu'] = self.power_periph.status  # legacy naming
            if k == 'arm':
                self.arm = arm.Arm()
                self.subsystems[k] = self.arm
            if k == 'lift':
                self.lift = lift.Lift()
                self.subsystems[k] = self.lift
            if k == 'omnibase':
                self.omnibase = omnibase.OmniBase()
                self.subsystems[k] = self.omnibase
                self.base = self.omnibase  # legacy naming
                self.status['base'] = self.omnibase.status  # legacy naming
            #Don't handle end_of_arm here, handle in Robot or RobotServer differently
        # Note, self.status isn't a deepcopy, so it will automaticaly
        # update on pull_status of the subsystems
        for k in self.subsystems:
            self.status[k] = self.subsystems[k].status
            self.status_aux[k] = self.subsystems[k].status_aux
            if self.params['enable_rate_log']:
                self.subsystems[k].enable_rate_logging(self.params['max_rate_log_samples'])

    def startup(self):
        if not hello_utils.acquire_transport_filelock(self.name):
            self.logger.error('Unable for RobotCore to aquire transport_filelock, server may already be running.\
                \nTry running stretch_body_server --status or stretch_free_robot_process. \nExiting loop.')
            return False

        self.logger.info(
            'Starting up Robot {0} of batch {1}'.format(self.params['serial_no'], self.params['batch_name']))
        ready=True
        for s in self.subsystems:
            ready = ready and self.subsystems[s].startup()
        return ready

    def stop(self):
        """
        To be called once before exiting a program
        Cleanly stops down motion and communication
        """
        hello_utils.free_transport_filelock(self.name)
        self.logger.info('---- Shutting down robot ----')
        for k in self.subsystems.keys():
            self.subsystems[k].stop()
        if self.params['enable_rate_log']:
            pathlib.Path(hello_utils.get_stretch_directory() + 'log/robot_rate_log/').mkdir(parents=True, exist_ok=True)
            fn = hello_utils.get_stretch_directory() + 'log/robot_rate_log/robot_rate_log_' + hello_utils.create_time_string() + '.json'
            rate_log = self.get_rate_log()
            with open(fn, 'w') as f:
                json.dump(rate_log, f)

        self.logger.info('---- Shutdown complete ----')

    def get_rate_log(self):
        rate_log = {}
        for s in self.subsystems:
            try:
                rate_log[s] = self.subsystems[s].get_rate_log()
            except AttributeError:
                pass
        return rate_log

    def get_status(self):
        return self.status.copy()

    def get_status_aux(self):
        return self.status_aux.copy()

    def get_subsystem(self, s):
        if s in self.subsystems:
            return self.subsystems[s]
        else:
            #print('Subsystem not present on RobotCore', s)
            return None

    def is_homed(self):
        """
        Returns true if homing has been run all joints that require it
        """
        ready = True
        for s in self.subsystems.values():
            if hasattr(s,'is_homed'):
                ready = ready and s.is_homed()  
        return ready

    def wait_command(self, timeout=15.0, use_motion_generator=True):
        raise NotImplementedError('RobotCore::wait_command method not implemented')

    def stow(self):
        raise NotImplementedError('RobotCore::stow method not implemented')

    def home(self):
        raise NotImplementedError('RobotCore::home method not implemented')

    def pretty_print(self):
        print('##################### HELLO ROBOT ##################### ')
        print('Time', time.time())
        print('Serial No', self.params['serial_no'])
        print('Batch', self.params['batch_name'])
        hello_utils.pretty_print_dict('Status', self.status)

    def pull_status(self, blocking=True):
        rpc_status_ids = {}
        for k in self.subsystems:
            rpc_status_ids[k] = self.subsystems[k].pull_status(blocking=blocking)
        return rpc_status_ids

    def pull_status_aux(self, blocking=True):
        rpc_status_ids = {}
        for k in self.subsystems:
            rpc_status_ids[k] = self.subsystems[k].pull_status_aux(blocking=blocking)
        return rpc_status_ids

    def push_command(self, blocking=True):
        rpc_cmd_ids = {}
        for k in self.subsystems:
            rpc_cmd_ids[k] = self.subsystems[k].push_command(blocking=blocking)
        if blocking:
            self.trigger_motor_sync()
        return rpc_cmd_ids

    def trigger_motor_sync(self):
        # Check if need to do a motor sync by looking at if there's been a pimu sync signal sent
        # since the last stepper.set_command for each joint
        if self.get_subsystem('power_periph') is not None:
            sync_needed = False
            for s in self.subsystems.values():
                if hasattr(s,'is_sync_required'):
                    sync_needed = sync_needed or s.is_sync_required(self.power_periph.ts_last_motor_sync)
            if sync_needed or self.power_periph.ts_last_motor_sync is None:  # First
                self.power_periph.trigger_motor_sync()

    def pause_transport(self):
        for s in self.subsystems:
            try:
                self.subsystems[s].pause_transport()
            except AttributeError:
                pass

    def unpause_transport(self):
        for s in self.subsystems:
            try:
                self.subsystems[s].unpause_transport()
            except AttributeError:
                pass

    def load_rpc_results(self, wait_on_result=True):
        """
        rpc_ids: dictionary of outstanding rpc calls, eg
        {'power_periph':[100],'arm':[55,60]}
        for each id, unpack the results from the transport into the device
        """
        for s in self.subsystems:
            self.subsystems[s].load_rpc_results(wait_on_result=wait_on_result)


if __name__ == '__main__':
    r = RobotCore()
    if r.startup():
        r.power_periph.trigger_beep()
        r.push_command()
        for i in range(100):
            r.pull_status()
            print('Voltage CPU', r.status['power_periph']['voltage_cpu'])
            time.sleep(.01)
        if r.params['enable_rate_log']:
            print(r.get_rate_log())
        r.stop()

