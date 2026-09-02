import stretch4_body.core.hello_utils as hello_utils
import importlib
import sys
import os
import copy
import importlib
import importlib.util

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
        if 'devices' in _nominal_params[eoa_name]:
            devices_copy = copy.deepcopy(_nominal_params[eoa_name]['devices'])
            for d in devices_copy:
                if d == eoa_name:
                    temp_device_params = {}
                    device_params_name = devices_copy[d].get('device_params')
                    if device_params_name:
                        try:
                            g=getattr(importlib.import_module(param_module_name), device_params_name)
                            temp_device_params=copy.deepcopy(g)
                        except AttributeError:
                            pass
                    hello_utils.overwrite_dict(temp_device_params, devices_copy[d])
                    for key in temp_device_params:
                        if key not in ['py_class_name', 'py_module_name', 'client_class_name', 'client_module_name']:
                            _nominal_params[eoa_name][key] = copy.deepcopy(temp_device_params[key])
                else:
                    device_params_name = devices_copy[d].get('device_params')
                    if device_params_name:
                        try:
                            g=getattr(importlib.import_module(param_module_name), device_params_name)
                            _nominal_params[d]=copy.deepcopy(g)
                        except AttributeError:
                            _nominal_params[d] = {}
                    else:
                        _nominal_params[d] = {}
                    hello_utils.overwrite_dict(_nominal_params[d], devices_copy[d])
        
        # Expand user-defined tool devices as well
        if eoa_name in _user_params and 'devices' in _user_params[eoa_name]:
            for d in _user_params[eoa_name]['devices']:
                if d == eoa_name:
                    temp_device_params = {}
                    device_params_name = _user_params[eoa_name]['devices'][d].get('device_params')
                    if device_params_name:
                        try:
                            g = getattr(importlib.import_module(param_module_name), device_params_name)
                            temp_device_params = copy.deepcopy(g)
                        except AttributeError:
                            pass
                    hello_utils.overwrite_dict(temp_device_params, _user_params[eoa_name]['devices'][d])
                    for key in temp_device_params:
                        if key not in ['py_class_name', 'py_module_name', 'client_class_name', 'client_module_name']:
                            _nominal_params[eoa_name][key] = copy.deepcopy(temp_device_params[key])
                else:
                    device_params_name = _user_params[eoa_name]['devices'][d].get('device_params')
                    if device_params_name:
                        try:
                            g = getattr(importlib.import_module(param_module_name), device_params_name)
                            _nominal_params[d] = copy.deepcopy(g)
                            hello_utils.overwrite_dict(_nominal_params[d], _user_params[eoa_name]['devices'][d])
                        except AttributeError:
                            pass
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
    def reload(cls):
        """
        Forces a fresh re-scan of the fleet's YAML/user_tools configuration from disk (e.g. after
        `stretch_configure_tool` writes new tool settings, or a test writes a temporary user_tools
        directory), then updates this class's already-loaded _user_params/_robot_params dicts
        IN PLACE with the freshly-scanned content.

        Reloading in place (rather than just doing a fresh `from stretch4_body.core.robot_params
        import RobotParams`) matters because the class body that builds _user_params/_robot_params
        only runs once, at first import: a plain re-import returns the same cached class unless the
        module is force-reloaded, and force-reloading naively would swap in a *new* RobotParams
        class object that any code already holding a reference to the old one (via a module-level
        `import`, a `self.foo = RobotParams` attribute, etc.) would not see. Mutating this class's
        dicts in place, then restoring the module's `RobotParams` name to point back at this same
        class, means every existing reference keeps working and observes the refreshed data.

        This also drops every already-imported `*robot_params*` module (not just this one) from
        `sys.modules` before re-importing: the per-model nominal-params module (e.g.
        stretch4_body.robot.robot_params_SE4) does its own YAML/user_tools disk scan, but only the
        first time it's imported — this class's body fetches it via `importlib.import_module()`,
        which returns whatever's already cached rather than re-scanning. Without dropping it too,
        only the very first `reload()` call in a process would ever see freshly-written files.
        """
        for mod_name in list(sys.modules):
            if 'robot_params' in mod_name:
                del sys.modules[mod_name]

        import stretch4_body.core.robot_params as robot_params_module
        new_user, new_robot = robot_params_module.RobotParams.get_params()

        old_user, old_robot = cls.get_params()
        old_user.clear()
        old_user.update(new_user)
        old_robot.clear()
        old_robot.update(new_robot)

        # Restore the module's RobotParams name to this (the original) class object, now updated,
        # so it keeps being the single shared identity every caller resolves to.
        robot_params_module.RobotParams = cls

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
    @classmethod
    def import_user_tool_module(cls, eoa_name, module_name, is_server=True):
        """
        Dynamically imports a user tool module in a collision-safe manner.
        Handles generic module names (e.g. 'client', 'tool', 'end_of_arm') without colliding.
        """
        _dirs = []
        _fleet_path = os.environ.get('HELLO_FLEET_PATH')
        if _fleet_path:
            _shared_dir = os.path.join(_fleet_path, 'user_tools')
            if os.path.exists(_shared_dir):
                _dirs.append(_shared_dir)
        else:
            _default_dir = os.path.expanduser('~/stretch_user/user_tools')
            if os.path.exists(_default_dir):
                _dirs.append(_default_dir)

        module_name_clean = module_name[:-3] if module_name.endswith('.py') else module_name

        current_module = None
        for _user_tools_dir in _dirs:
            _candidate = os.path.join(_user_tools_dir, eoa_name)
            if os.path.exists(_candidate):
                if _candidate not in sys.path:
                    sys.path.insert(0, _candidate)
                _py_file = os.path.join(_candidate, f"{module_name_clean}.py")
                if os.path.exists(_py_file):
                    side = "server" if is_server else "client"
                    unique_mod_name = f"user_tool_{side}_{eoa_name}_{module_name_clean}"
                    spec = importlib.util.spec_from_file_location(unique_mod_name, _py_file)
                    if spec and spec.loader:
                        try:
                            current_module = importlib.util.module_from_spec(spec)
                            sys.modules[unique_mod_name] = current_module
                            spec.loader.exec_module(current_module)
                            break
                        except Exception as e:
                            print(f"Error loading custom tool module {module_name_clean} directly: {e}")
                            current_module = None
        if current_module is None:
            try:
                current_module = importlib.import_module(module_name_clean)
            except Exception:
                current_module = None
        return current_module

    @classmethod
    def is_user_defined_tool(cls, tool_name):
        """
        Dynamically check if a tool's folder exists under user_tools directories.
        """
        return cls.get_user_defined_tool_path(tool_name) is not None

    @classmethod
    def get_user_defined_tool_path(cls, tool_name):
        """
        Get the absolute path to a custom tool folder if it exists.
        """
        if not tool_name:
            return None
        _dirs = []
        _fleet_path = os.environ.get('HELLO_FLEET_PATH')
        if _fleet_path:
            _shared_dir = os.path.join(_fleet_path, 'user_tools')
            if os.path.exists(_shared_dir):
                _dirs.append(_shared_dir)
        else:
            _default_dir = os.path.expanduser('~/stretch_user/user_tools')
            if os.path.exists(_default_dir):
                _dirs.append(_default_dir)
        
        for _user_tools_dir in _dirs:
            p = os.path.join(_user_tools_dir, tool_name)
            if os.path.exists(p):
                return p
        return None

    @classmethod
    def add_user_tool_to_sys_path(cls, tool_name):
        """
        Finds and adds the user tool's directory to sys.path dynamically.
        """
        if not tool_name:
            return
        _dirs = []
        _fleet_path = os.environ.get("HELLO_FLEET_PATH")
        _fleet_id = os.environ.get("HELLO_FLEET_ID")
        if _fleet_path:
            if _fleet_id:
                _specific_dir = os.path.join(_fleet_path, _fleet_id, "user_tools")
                if os.path.exists(_specific_dir):
                    _dirs.append(_specific_dir)
            _shared_dir = os.path.join(_fleet_path, "user_tools")
            if os.path.exists(_shared_dir):
                _dirs.append(_shared_dir)
        else:
            _default_dir = os.path.expanduser("~/stretch_user/user_tools")
            if os.path.exists(_default_dir):
                _dirs.append(_default_dir)

        for _user_tools_dir in _dirs:
            _candidate = os.path.join(_user_tools_dir, tool_name)
            if os.path.exists(_candidate):
                if _candidate not in sys.path:
                    sys.path.append(_candidate)
                break


