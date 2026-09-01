#!/usr/bin/env python3
import importlib
import os
import sys

def add_user_tool_to_sys_path(tool_name: str | None) -> None:
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


def get_gripper_instance(direct: bool = False, ip_address: str | None = None):
    """
    Constructs and returns a direct or remote tool instance along with its type name.
    Fully generic across built-in grippers and custom user tools.
    """

    from stretch4_body.utils.tool_metadata import get_tool_metadata

    try:
        meta = get_tool_metadata()
    except Exception:
        return None, None

    gripper_type = meta.joint_name

    try:
        if not direct:
            ClientClass = meta.client_class
            try:
                g = ClientClass(ip_address=ip_address)
            except TypeError:
                g = ClientClass()
        else:
            DriverClass = meta.driver_class
            g = DriverClass(is_direct=True)
    except Exception as e:
        raise ValueError(
            f"Failed to instantiate {'direct driver' if direct else 'client'} for tool '{gripper_type}': {e}"
        )

    return g, gripper_type
