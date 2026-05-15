from stretch4_body.core.robot_params import RobotParams
import stretch4_body.core.hello_utils as hello_utils
import time
import logging, logging.config
import sys
import os

class DeviceTimestamp:
    def __init__(self):
        self.reset()

    def reset(self):
        self.timestamp_last = None
        self.timestamp_base = 0
        self.timestamp_first = None
        self.ts_start = time.time()

    def set(self, ts): #take a timestamp from a uC in uS and put in terms of system clock
        if self.timestamp_last is None:  # First time
            self.timestamp_last = ts
            self.timestamp_first=ts
        if ts - self.timestamp_last < 0:  # rollover
            self.timestamp_base = self.timestamp_base + 0xFFFFFFFF
        self.timestamp_last = ts
        s=(self.timestamp_base + ts - self.timestamp_first) / 1000000.0
        return self.ts_start+s

class Device:
    logging_params = RobotParams.get_params()[1]['logging']
    os.system('mkdir -p '+hello_utils.get_stretch_directory("/log/stretch_body_logger")) #Some robots may not have this directory yet
    logging.config.dictConfig(logging_params)
    """
    Generic base class for all custom Stretch hardware
    """
    def __init__(self, name='',req_params=True):
        self.name = name
        self.user_params, self.robot_params = RobotParams.get_params()
        self.params = self.robot_params.get(self.name, {})
        self.logger = logging.getLogger(self.name)
        
        throttle_filters = [filter for filter in self.logger.filters if filter.name == self.name and isinstance(filter, hello_utils.LoggerThrottleFilter)]
        if len(throttle_filters) == 0:
            self.logger.addFilter(hello_utils.LoggerThrottleFilter(self.name))

        if self.params == {} and req_params:
            self.logger.error('Parameters for device %s not found. Check parameter YAML and device name. Exiting...' % self.name.upper())
            sys.exit(1)

        self.timestamp = DeviceTimestamp()
        self.is_valid = True

    # ########### Primary interface #############

    def startup(self):
        """Starts machinery required to interface with this device

        Returns
        -------
        bool
            whether the startup procedure succeeded
        """
        return True

    def stop(self):
        """Shuts down machinery started in `startup()`
        """
        return True

    def load_rpc_results(self,wait_on_result=True):
        pass

    def push_command(self,blocking=True):
        pass

    def pull_status(self,blocking=True):
        pass

    def step_sentry(self,robot_status):
        pass

    def pretty_print(self):
        print(f"----- {self.name} -----")
        hello_utils.pretty_print_dict("params", self.params)

    def write_configuration_param_to_YAML(self,param_name,value,fleet_dir=None,force_creation=False):
        """
        Update the robot configuration YAML with a new value
        """
        self._write_param_to_YAML(param_name,value,filename='stretch_configuration_params.yaml',fleet_dir=fleet_dir,force_creation=force_creation)

    def write_user_param_to_YAML(self, param_name, value, fleet_dir=None,force_creation=False):
        """
        Update the robot configuration YAML with a new value
        """
        self._write_param_to_YAML(param_name, value, filename='stretch_user_params.yaml', fleet_dir=fleet_dir,force_creation=force_creation)

    def _write_param_to_YAML(self,param_name,value,filename,fleet_dir=None,force_creation=False):
        """
        Update the YAML with a new value
        The param_name has the form device.key, or for a nested dictionary, device.key1.key2...
        For example, write_configuration_param_to_YAML('pimu.config.cliff_zero',100) will set this value to 100 in the YAML
        """
        cp = hello_utils.read_fleet_yaml(filename, fleet_dir=fleet_dir)
        param_keys=param_name.split('.')
        d=cp
        for param_key in param_keys:
            if param_key in d:
                if param_key==param_keys[-1]:
                    d[param_key] = value
                else:
                    d = d[param_key]
            else:
                if force_creation:
                    if param_key == param_keys[-1]:
                        d[param_key] = value
                    else:
                        d[param_key] = {}
                        d=d[param_key]
                else:
                    print('Improper param_name in _write_param_to_YAML. Not able to update %s' % param_name)
        hello_utils.write_fleet_yaml(filename, cp, fleet_dir=fleet_dir)


