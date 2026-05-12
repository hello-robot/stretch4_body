import stretch4_body.core.hello_utils as hello_utils
import importlib
import sys
import os
import click
from datetime import datetime

#System parameters that are common across models. May be updated by the factory via Pip.
nominal_system_params={
    "logging": {
            "version": 1,
            "disable_existing_loggers": False,
            "root": {
                "level": "INFO",
                "handlers": ["console_handler", "file_handler"],
                "propagate": False
            },
            "handlers": {
                "console_handler": {
                    "class": "logging.StreamHandler",
                    "level": "INFO",
                    "formatter": "default_console_formatter",
                },
                "file_handler": {
                    "class": "logging.handlers.RotatingFileHandler",
                    "level": "INFO",
                    "formatter": "default_file_formatter",
                    "filename": hello_utils.get_stretch_directory('log/stretch_body_logger/') + 'stretch_body_server.log',
                    "maxBytes": 10485760,
                    "backupCount": 10
                }
            },
            "formatters": {
                "default_console_formatter": {
                    "()": "stretch4_body.core.hello_utils.HelloLoggerScreen",
                },
                "brief_console_formatter": {
                    "format": "%(message)s"
                },
                "default_file_formatter": {
                    "()": "stretch4_body.core.hello_utils.HelloLoggerFile",
                }
            }
        },
    "system_check": {
            "show_sw_exc": False
        },
}

class RobotParams:
    """Build the parameter dictionary that is available as stretch4_body.Device().robot_params.
    Overwrite dictionaries in order of ascending priority
    1. stretch4_body.robot_params.nominal_system_params  | Generic systems settings (Common across all robot models. Factory may modify these via Pip updates)
    2. stretch4_body.robot_params_XXXX.py                | Nominal robot paramters for this robot model (e.g., RE1V0) as defined in stretch_user_params.yaml. Factory may modify these via Pip updates
    3. Outside parameters                               | Include other sourcesthrough 'params' field. (eg, from stretch_tool_share.stretch_dex_wrist.params). Factory may modify these via Pip updates.
    4. stretch_configuration_params.yaml                | Robot specific data (eg, serial numbers and calibrations). Calibration tools may update these.
    5. stretch_user_params.yaml                         | User specific data (eg, contact thresholds, controller tunings, etc)
    """
    user_params_fn = hello_utils.get_fleet_directory()+'stretch_user_params.yaml'
    config_params_fn = hello_utils.get_fleet_directory()+'stretch_configuration_params.yaml'
    if not hello_utils.check_file_exists(user_params_fn) or not hello_utils.check_file_exists(config_params_fn):
        _valid_params=False
        print('Please verify if Stretch configuration YAML files are present before continuing.')
        sys.exit(1)
    else:
        _user_params = hello_utils.read_fleet_yaml('stretch_user_params.yaml')
        _config_params = hello_utils.read_fleet_yaml('stretch_configuration_params.yaml')
        _robot_params=nominal_system_params

        #Check for user / config overrides that impact what data is loaded
        #Get the name of the robot model
        if 'robot' in _user_params and 'model_name' in _user_params['robot']:
            param_module_name = 'stretch4_body.robot.robot_params_' + _user_params['robot']['model_name']
        elif 'robot' in _config_params and 'model_name' in _config_params['robot']:
            param_module_name = 'stretch4_body.robot.robot_params_' + _config_params['robot']['model_name']
        else:
            print("ERROR: Could not find 'robot.model_name' in stretch_configuration_params.yaml or stretch_user_params.yaml")
            print(f"  HELLO_FLEET_PATH={hello_utils.get_fleet_directory()}")
            print(f"  config_params keys: {list(_config_params.keys())}")
            sys.exit(1)

        _nominal_params = getattr(importlib.import_module(param_module_name), 'nominal_params')

        #Get the name of the current end-of-arm
        eoa_name=None
        if 'robot' in _user_params and 'tool' in _user_params['robot']:
            eoa_name = _user_params['robot']['tool']
        elif 'robot' in _config_params and 'tool' in _config_params['robot']:
            eoa_name = _config_params['robot']['tool']
        elif 'tool' in _nominal_params['robot']:
            eoa_name = _nominal_params['robot']['tool']

        if not eoa_name in _nominal_params['supported_eoa'] or not eoa_name in _nominal_params:
            _valid_params = False
            print('%s not supported for robot %s'%(eoa_name.upper(), param_module_name))
            print('Check your YAML definition of robot.tool')
            sys.exit(1)

        #Now expand the params for each EOA
        for d in _nominal_params[eoa_name]['devices']:
            g=getattr(importlib.import_module(param_module_name),_nominal_params[eoa_name]['devices'][d]['device_params'])
            _nominal_params[d]=g
        #     _nominal_params[d]=_nominal_params[eoa_name]['devices'][d]['device_params']
        if 'ros' in _nominal_params[eoa_name]:
                _nominal_params['ros']['joints'].extend(_nominal_params[eoa_name]['ros']['joints'])


        hello_utils.overwrite_dict(_robot_params, _nominal_params)

        for external_params_module in _config_params.get('params', []):
            hello_utils.overwrite_dict(_robot_params,getattr(importlib.import_module(external_params_module), 'params'))

        for external_params_module in _user_params.get('params', []):
            hello_utils.overwrite_dict(_robot_params,getattr(importlib.import_module(external_params_module), 'params'))

        hello_utils.overwrite_dict(_robot_params, _config_params)

        hello_utils.overwrite_dict(_robot_params, _user_params)

        _valid_params=True

    @classmethod
    def get_user_params_header(cls):
        return getattr(importlib.import_module(cls.param_module_name), 'user_params_header')

    @classmethod
    def get_configuration_params_header(cls):
        return getattr(importlib.import_module(cls.param_module_name), 'configuration_params_header')

    @classmethod
    def are_params_valid(cls):
        return (cls._valid_params)

    @classmethod
    def get_params(cls):
        return (cls._user_params, cls._robot_params)

    @classmethod
    def add_params(cls, new_params):
        hello_utils.overwrite_dict(cls._robot_params, new_params)

    @classmethod
    def set_logging_level(cls, level, handler='console_handler'):
        level_names={0: 'NOTSET', 10: 'DEBUG', 'WARN': 30, 20: 'INFO', 'ERROR': 40, 'DEBUG': 10, 30:
            'WARNING', 'INFO': 20, 'WARNING': 30, 40: 'ERROR', 50: 'CRITICAL', 'CRITICAL': 50, 'NOTSET': 0}
        if level in level_names and handler in cls._robot_params['logging']['handlers']:
            cls._robot_params['logging']['handlers'][handler]['level'] = level

    @classmethod
    def set_logging_formatter(cls, formatter, handler='console_handler'):
        formatter_names = ["default_console_formatter", "brief_console_formatter", "default_file_formatter"]
        if formatter in formatter_names and handler in cls._robot_params['logging']['handlers']:
            cls._robot_params['logging']['handlers'][handler]['formatter'] = formatter

