"""
Locating user tool directories on disk.

Kept free of other stretch4_body imports so that both `core.robot_params` and the
`robot.robot_params_SE4` data module can share one definition of where user tools live.
"""

import os


def user_tool_dirs():
    """
    Existing directories that may hold user tool folders, in search order: the fleet-id
    directory, then the shared fleet directory, else ~/stretch_user/user_tools.
    """
    dirs = []
    fleet_path = os.environ.get('HELLO_FLEET_PATH')
    fleet_id = os.environ.get('HELLO_FLEET_ID')
    if fleet_path:
        if fleet_id:
            specific_dir = os.path.join(fleet_path, fleet_id, 'user_tools')
            if os.path.exists(specific_dir):
                dirs.append(specific_dir)
        shared_dir = os.path.join(fleet_path, 'user_tools')
        if os.path.exists(shared_dir):
            dirs.append(shared_dir)
    else:
        default_dir = os.path.expanduser('~/stretch_user/user_tools')
        if os.path.exists(default_dir):
            dirs.append(default_dir)
    return dirs


def find_user_tool_dir(tool_name):
    """Absolute path to `tool_name`'s folder in the first directory that holds it, else None."""
    if not tool_name:
        return None
    for user_tools_dir in user_tool_dirs():
        candidate = os.path.join(user_tools_dir, tool_name)
        if os.path.exists(candidate):
            return candidate
    return None


def list_user_tools():
    """Names of every tool folder found across the search directories."""
    names = set()
    for user_tools_dir in user_tool_dirs():
        for entry in os.listdir(user_tools_dir):
            if os.path.isdir(os.path.join(user_tools_dir, entry)):
                names.add(entry)
    return sorted(names)
