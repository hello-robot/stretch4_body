#!/usr/bin/env python3
import sys
import os
import importlib

def add_user_tool_to_sys_path(tool_name):
    """
    Finds and adds the user tool's directory to sys.path dynamically.
    """
    if not tool_name:
        return
    _dirs = []
    _fleet_path = os.environ.get('HELLO_FLEET_PATH')
    _fleet_id = os.environ.get('HELLO_FLEET_ID')
    if _fleet_path:
        if _fleet_id:
            _specific_dir = os.path.join(_fleet_path, _fleet_id, 'user_tools')
            if os.path.exists(_specific_dir):
                _dirs.append(_specific_dir)
        _shared_dir = os.path.join(_fleet_path, 'user_tools')
        if os.path.exists(_shared_dir):
            _dirs.append(_shared_dir)
    else:
        _default_dir = os.path.expanduser('~/stretch_user/user_tools')
        if os.path.exists(_default_dir):
            _dirs.append(_default_dir)
    
    for _user_tools_dir in _dirs:
        _candidate = os.path.join(_user_tools_dir, tool_name)
        if os.path.exists(_candidate):
            if _candidate not in sys.path:
                sys.path.append(_candidate)
            break

def get_gripper_instance(direct=False, ip_address=None):
    """
    Constructs and returns a direct or remote gripper instance along with its 
    type name and a flag indicating if it behaves as a parallel jaw gripper.
    """
    from stretch4_body.utils.stretch_pose_models import RobotJoints
    from stretch4_body.core.robot_params import RobotParams
    
    gripper_type = RobotJoints.gripper.value
    if gripper_type is None:
        return None, None, False
        
    is_parallel = gripper_type == 'parallel_gripper' or 'parallel' in gripper_type.lower() or 'jaw' in gripper_type.lower()
    
    if not direct:
        from stretch4_body.utils.gripper_metadata import get_tool_metadata
        try:
            meta = get_tool_metadata(gripper_type)
            ClientClass = meta.client_class
            try:
                g = ClientClass(ip_address=ip_address)
            except TypeError:
                g = ClientClass()
        except Exception as e:
            raise ValueError(f"Failed to instantiate client for tool '{gripper_type}': {e}")
    else:
        if gripper_type == 'parallel_gripper':
            from stretch4_body.subsystem.end_of_arm.parallel_gripper import ParallelGripper as Gripper
            g = Gripper(is_direct=True)
        elif gripper_type == 'stretch_gripper':
            from stretch4_body.subsystem.end_of_arm.stretch_gripper import StretchGripper as Gripper
            g = Gripper(is_direct=True)
        else:
            _, robot_params = RobotParams.get_params()
            tool_name = robot_params.get('robot', {}).get('tool')
            if tool_name and tool_name in robot_params:
                tool_params = robot_params[tool_name]
                device_params = tool_params.get('devices', {}).get(gripper_type, {})
                py_module = device_params.get('py_module_name')
                py_class = device_params.get('py_class_name')
                
                add_user_tool_to_sys_path(tool_name)
                
                if py_module and py_class:
                    module = RobotParams.import_user_tool_module(tool_name, py_module, is_server=True)
                    Gripper = getattr(module, py_class)
                    g = Gripper(is_direct=True)
                else:
                    raise ValueError(f"Custom gripper device parameters for {gripper_type} must specify py_module_name and py_class_name")
            else:
                raise ValueError(f"Custom tool {tool_name} params not found in robot_params")
                
    return g, gripper_type, is_parallel
