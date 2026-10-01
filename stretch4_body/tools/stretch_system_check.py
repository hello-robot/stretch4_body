#!/usr/bin/env python3
"""
stretch_system_check.py

Comprehensive hardware and software diagnostic tool for the Stretch 4 robot.

Usage:
    stretch_system_check                  # Full system check (requires server)
    stretch_system_check --firmware       # Firmware version check (kills/restarts server)
    stretch_system_check --sensors        # Lidar + camera check (no server needed)
    stretch_system_check --check_updates  # pip + firmware + workspace git updates, with commands to run
    stretch_system_check --repos          # ROS2 workspace (~/ament_ws/src) git status only
    stretch_system_check --verbose        # Show additional detail in all checks
    stretch_system_check --direct         # Use Robot API directly instead of server client
    stretch_system_check --export [DIR]   # Save a diagnostics zip for support to DIR (default: cwd)
"""
import os

# Mute DepthAI warnings 
os.environ.setdefault('DEPTHAI_LEVEL', 'error')

import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

import sys
import io
import re
import json
import fnmatch
import argparse
import subprocess
import logging
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from importlib.metadata import version as pkg_version

import click
from stretch4_body.core import robot_params as rp
from stretch4_body.subsystem.line_sensor import calibration_store


# ==============================================================================
# CLI Arguments
# ==============================================================================

parser = argparse.ArgumentParser(
    description='Check that all Stretch 4 robot hardware is present and reporting sane values'
)
parser.add_argument('-v', '--verbose',  help='Print additional detail',                  action='store_true')
parser.add_argument('-d', '--direct',   help='Use direct Robot API (no server)',          action='store_true')
parser.add_argument('--firmware',       help='Kill server, check firmware, restart server', action='store_true')
parser.add_argument('--sensors',        help='Check lidars and cameras',                 action='store_true')
parser.add_argument('--check_updates',  help='Check pip + firmware + workspace git updates and print the commands to run',
                                                                                        action='store_true')
parser.add_argument('--repos',          help='Check the git status of the repos in ~/ament_ws/src',
                                                                                        action='store_true')
parser.add_argument('--export',         help='Save a diagnostics zip (status history, system checks, server logs) '
                                             'to the given directory (defaults to the current directory)',
                                        nargs='?', const='.', metavar='DIR', default=None)
args = parser.parse_args()

logging.getLogger('stretch4_body').setLevel(logging.WARNING)
logging.getLogger('stretch_body_client').setLevel(logging.WARNING)


# ==============================================================================
# Robot identity (read once at startup)
# ==============================================================================

_robot_info   = rp.RobotParams._robot_params.get('robot', {})
stretch_serial_no = _robot_info.get('serial_no', 'N/A')
stretch_model     = _robot_info.get('model_name', 'N/A')
stretch_batch     = _robot_info.get('batch_name', 'N/A')
stretch_tool      = _robot_info.get('tool', 'N/A')

TOOL_DISPLAY = {
    'eoa_wrist_dw4_tool_nil':         'DexWrist 4 — No Tool',
    'eoa_wrist_dw4_tool_sg4':         'DexWrist 4 — Stretch Gripper (SG4)',
    'eoa_wrist_dw4_tool_pg4':         'DexWrist 4 — Parallel Gripper (PG4)',
    'eoa_wrist_dw4_tool_tablet':      'DexWrist 4 — Tablet Holder',
    'eoa_wrist_dw4_tool_calibration': 'DexWrist 4 — Calibration Tool',
}

# Resolve custom user-defined tool display name if available in metadata
try:
    from stretch4_body.robot.robot_params import RobotParams
    # First, let's load or check if we can get supported_eoa_metadata
    meta = RobotParams._robot_params.get('supported_eoa_metadata', {}).get(stretch_tool, {})
    if 'name' in meta:
        TOOL_DISPLAY[stretch_tool] = f"User Custom — {meta['name']}"
except Exception:
    pass

_model_display = 'Stretch 4' if stretch_model == 'SE4' else stretch_model

click.secho('\n======== Stretch 4 System Check ========', fg='cyan', bold=True)
click.secho(f'  Model         : {_model_display}',                                  fg='bright_white')
click.secho(f'  Serial Number : {stretch_serial_no}',                               fg='bright_white')
click.secho(f'  Tool          : {TOOL_DISPLAY.get(stretch_tool, stretch_tool)}',    fg='bright_white')
if args.verbose:
    click.secho(f'  Batch         : {stretch_batch}',                               fg='bright_white')
click.secho('========================================\n', fg='cyan', bold=True)


# Robot client handle — assigned in main() after server startup
r = None


# ==============================================================================
# Output helpers
# ==============================================================================

_SKIP_REASONS = {}

def print_section(title):
    click.secho(f'\n---- {title} ----', fg='cyan', bold=True)

def print_result(passed, msg, indent=2):
    pad = ' ' * indent
    if passed:
        click.secho(f'{pad}[PASS] {msg}', fg='green')
    else:
        click.secho(f'{pad}[FAIL] {msg}', fg='red')

def print_warn(msg, indent=2):
    click.secho(f'{" " * indent}[WARN] {msg}', fg='yellow')

def print_info(msg, indent=4):
    click.secho(f'{" " * indent}{msg}', fg='white')

def print_version_info(msg, update_ver=None, indent=4):
    """Print a version line, appending '(Update Available: x.y.z)' when newer on PyPI."""
    click.secho(f'{" " * indent}{msg}', fg='white', nl=False)
    if update_ver:
        click.secho(f'  (Update Available: {update_ver})', fg='yellow', bold=True)
    else:
        click.echo()

def val_in_range(label, val, vmin, vmax):
    ok = vmin <= val <= vmax
    return ok, f'{label} = {val:.3f} (range [{vmin:.2f}, {vmax:.2f}])'


# ==============================================================================
# PyPI update checks
# ==============================================================================

PYPI_TIMEOUT_S = 3.0


def _pypi_latest_version(pkg_name):
    """Return the latest release version on PyPI, or None if it can't be determined."""
    url = f'https://pypi.org/pypi/{pkg_name}/json'
    try:
        with urllib.request.urlopen(url, timeout=PYPI_TIMEOUT_S) as resp:
            info = json.load(resp).get('info') or {}
        return info.get('version')
    except Exception:
        return None


def _is_newer(candidate, installed):
    """True if candidate is a strictly newer version than installed."""
    try:
        from packaging.version import Version
        return Version(candidate) > Version(installed)
    except Exception:
        pass
    # Fallback: compare numeric components (versions here are date-based, e.g. 2026.6.25)
    try:
        to_tuple = lambda v: tuple(int(n) for n in re.findall(r'\d+', v))
        return to_tuple(candidate) > to_tuple(installed)
    except Exception:
        return False


CORE_PIP = ('hello-robot-stretch4-body', 'hello-robot-stretch4-urdf')

# Command shown to the user for applying pip updates (matches README)
PIP_UPDATE_CMD = 'python3 -m pip install -U'


def discover_pip_packages():
    """
    Return (core, extras): installed versions of the always-shown Stretch packages,
    and of any other hello/stretch/hesai pip packages found in the environment.
    """
    core = {}
    for name in CORE_PIP:
        try:
            core[name] = pkg_version(name)
        except Exception:
            core[name] = 'unknown'

    extras = {}
    try:
        from importlib.metadata import distributions as _distributions
        core_lc = {n.lower() for n in CORE_PIP}
        for dist in _distributions():
            name = (dist.metadata.get('Name') or '').strip()
            name_lc = name.lower()
            if not name or name_lc in core_lc:
                continue
            if 'hello' in name_lc or 'stretch' in name_lc or 'hesai' in name_lc:
                if name not in extras:  # keep first occurrence
                    extras[name] = (dist.metadata.get('Version') or 'unknown').strip()
    except Exception:
        pass

    return core, extras


def check_pypi_updates(installed):
    """
    Query PyPI for newer releases of the hello-robot-* packages in `installed`
    ({name: version}). Queries run concurrently and fail silently (offline robot).

    Returns (updates, reachable) where updates is {name: latest_version} for
    packages with a newer release, and reachable is False if no query succeeded.
    """
    names = [n for n in installed if n.lower().startswith('hello-robot-')
             and installed[n] not in (None, '', 'unknown')]
    if not names:
        return {}, True

    try:
        with ThreadPoolExecutor(max_workers=min(8, len(names))) as pool:
            latest = dict(zip(names, pool.map(_pypi_latest_version, names)))
    except Exception:
        return {}, False

    reachable = any(v is not None for v in latest.values())
    updates = {n: latest[n] for n in names
               if latest[n] and _is_newer(latest[n], installed[n])}
    return updates, reachable


# ==============================================================================
# ROS2 workspace (~/ament_ws/src) git checks
# ==============================================================================

GIT_TIMEOUT_S = 10.0

# Status codes reported per repo, and how each is rendered
GIT_OK        = 'up to date'
GIT_BEHIND    = 'behind'
GIT_UPDATE    = 'update available'   # remote has commits that aren't in the local object store
GIT_AHEAD     = 'ahead'
GIT_DIVERGED  = 'diverged'
GIT_DETACHED  = 'detached HEAD'
GIT_NO_UPSTREAM = 'no upstream branch'
GIT_NO_BRANCH   = 'branch not on remote'
GIT_UNREACHABLE = 'remote unreachable'
GIT_NOT_A_REPO  = 'not a git repo'

# Statuses that mean the remote has work the local checkout doesn't
GIT_NEEDS_PULL = (GIT_BEHIND, GIT_UPDATE)


def ament_src_dir():
    """
    Path of the ROS2 workspace src directory.

    Honors STRETCH_AMENT_WS, then the sourced workspace (COLCON_PREFIX_PATH),
    then falls back to ~/ament_ws.
    """
    ws = os.environ.get('STRETCH_AMENT_WS', '')
    if not ws:
        prefix = os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0]
        ws = os.path.dirname(prefix) if prefix else ''
    if not ws:
        ws = os.path.expanduser('~/ament_ws')
    return os.path.join(os.path.expanduser(ws), 'src')


def _git(repo, *cmd, timeout=GIT_TIMEOUT_S):
    """
    Run a git command in `repo`. Returns (ok, stdout-stripped).

    Credential and host-key prompts are disabled so an unauthenticated remote
    fails immediately instead of blocking the check on stdin.
    """
    env = dict(os.environ,
               GIT_TERMINAL_PROMPT='0',
               GIT_ASKPASS='',
               SSH_ASKPASS='',
               GIT_SSH_COMMAND='ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new')
    try:
        res = subprocess.run(['git', '-C', repo, *cmd],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             text=True, timeout=timeout, env=env)
        return res.returncode == 0, res.stdout.strip()
    except Exception:
        return False, ''


def check_repo_git_status(repo):
    """
    Report the git state of a single workspace repo, and whether the remote has
    a newer commit than the local checkout.

    Read-only: queries the remote with `git ls-remote` rather than fetching, so
    nothing in the repo is modified.

    Returns a dict with name, branch, sha, dirty, status, detail and command.
    """
    out = {'name': os.path.basename(repo.rstrip('/')), 'branch': '', 'sha': '',
           'dirty': False, 'status': GIT_NOT_A_REPO, 'detail': '', 'command': None}

    ok, _ = _git(repo, 'rev-parse', '--git-dir')
    if not ok:
        return out

    ok, out['sha'] = _git(repo, 'rev-parse', '--short', 'HEAD')
    if not ok:
        out['detail'] = 'no commits'
        return out

    _, porcelain = _git(repo, 'status', '--porcelain')
    out['dirty'] = bool(porcelain)

    _, branch = _git(repo, 'rev-parse', '--abbrev-ref', 'HEAD')
    out['branch'] = branch

    if branch == 'HEAD':
        out['status'] = GIT_DETACHED
        return out

    ok, upstream = _git(repo, 'rev-parse', '--abbrev-ref', '--symbolic-full-name', '@{u}')
    if not ok or '/' not in upstream:
        out['status'] = GIT_NO_UPSTREAM
        return out

    remote, remote_branch = upstream.split('/', 1)
    ok, ls = _git(repo, 'ls-remote', '--heads', remote, remote_branch)
    if not ok:
        out['status'] = GIT_UNREACHABLE
        out['detail'] = f'could not query {remote}'
        return out
    if not ls:
        # The remote answered but has no such branch (deleted upstream, or never pushed)
        out['status'] = GIT_NO_BRANCH
        out['detail'] = f'{upstream} no longer exists'
        return out

    remote_sha = ls.split()[0]
    _, local_sha = _git(repo, 'rev-parse', 'HEAD')

    if remote_sha == local_sha:
        out['status'] = GIT_OK
        return out

    # The remote commit is only comparable if it's already in the local object
    # store (i.e. someone has fetched since it was pushed). If it isn't, the
    # remote is simply ahead of anything we know about.
    have_remote, _ = _git(repo, 'cat-file', '-e', f'{remote_sha}^{{commit}}')
    if not have_remote:
        out['status'] = GIT_UPDATE
        out['detail'] = f'{upstream} at {remote_sha[:7]}'
        out['command'] = f'git -C {repo} pull'
        return out

    behind, _ = _git(repo, 'merge-base', '--is-ancestor', local_sha, remote_sha)
    ahead, _  = _git(repo, 'merge-base', '--is-ancestor', remote_sha, local_sha)
    if behind:
        _, n = _git(repo, 'rev-list', '--count', f'{local_sha}..{remote_sha}')
        out['status'] = GIT_BEHIND
        out['detail'] = f'{n} commit(s) behind {upstream}'
        out['command'] = f'git -C {repo} pull'
    elif ahead:
        _, n = _git(repo, 'rev-list', '--count', f'{remote_sha}..{local_sha}')
        out['status'] = GIT_AHEAD
        out['detail'] = f'{n} unpushed commit(s) vs {upstream}'
    else:
        _, n_behind = _git(repo, 'rev-list', '--count', f'{local_sha}..{remote_sha}')
        _, n_ahead  = _git(repo, 'rev-list', '--count', f'{remote_sha}..{local_sha}')
        out['status'] = GIT_DIVERGED
        out['detail'] = f'{n_ahead} ahead / {n_behind} behind {upstream}'

    return out


def check_workspace_repos():
    """
    Check every repo in the ROS2 workspace src directory, concurrently.

    Returns (src_dir, [status dicts sorted by name]). The list is empty if the
    workspace directory doesn't exist.
    """
    src = ament_src_dir()
    if not os.path.isdir(src):
        return src, []

    repos = sorted(os.path.join(src, d) for d in os.listdir(src)
                   if os.path.isdir(os.path.join(src, d)))
    if not repos:
        return src, []

    try:
        with ThreadPoolExecutor(max_workers=min(8, len(repos))) as pool:
            results = list(pool.map(check_repo_git_status, repos))
    except Exception:
        results = [check_repo_git_status(p) for p in repos]

    return src, sorted(results, key=lambda x: x['name'].lower())


def print_workspace_repos(repos, indent=4):
    """Print one line per workspace repo: branch, sha and update state."""
    col = max(len(x['name']) for x in repos)
    pad = ' ' * indent
    for x in repos:
        if x['status'] == GIT_NOT_A_REPO:
            click.secho(f'{pad}{x["name"]:<{col}} : {GIT_NOT_A_REPO}', fg='white')
            continue

        where = f'{x["branch"]} @ {x["sha"]}' if x['branch'] else f'@ {x["sha"]}'
        if x['dirty']:
            where += ' *'

        if x['status'] in GIT_NEEDS_PULL:
            click.secho(f'{pad}{x["name"]:<{col}} : {where}', fg='white', nl=False)
            note = x['detail'] or x['status']
            click.secho(f'  (Update Available: {note})', fg='yellow', bold=True)
        elif x['status'] == GIT_OK:
            click.secho(f'{pad}{x["name"]:<{col}} : {where}  (up to date)', fg='white')
        else:
            detail = f' — {x["detail"]}' if x['detail'] else ''
            click.secho(f'{pad}{x["name"]:<{col}} : {where}  ({x["status"]}{detail})', fg='white')

    if any(x['dirty'] for x in repos):
        print_info('* = uncommitted local changes', indent=indent)


def check_repos():
    """Standalone --repos mode: report the git state of every workspace repo."""
    print_section('ROS2 Workspace Repos')

    src, repos = check_workspace_repos()
    print_info(f'Workspace: {src}', indent=2)
    if not repos:
        print_warn(f'No repos found in {src}')
        return False

    print_workspace_repos(repos)

    cmds = [x['command'] for x in repos if x['command']]
    unreachable = [x['name'] for x in repos if x['status'] == GIT_UNREACHABLE]
    if unreachable:
        print_warn('Could not reach the remote for: ' + ', '.join(unreachable))
    if cmds:
        print_section('Commands To Run')
        click.echo()
        for cmd in cmds:
            click.secho(f'    {cmd}', fg='green', bold=True)
        click.echo()
        print_info('Rebuild after pulling:  cd ~/ament_ws && colcon build --symlink-install')
    else:
        click.secho('\n  All workspace repos are up to date.', fg='green', bold=True)

    return not unreachable


# ==============================================================================
# Check functions
# ==============================================================================

def print_software_versions():
    print_section('Software Versions')

    core, pip_extras = discover_pip_packages()
    py_ver = f'{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}'

    # Ask PyPI (once, concurrently) which hello-robot-* packages have newer releases
    updates, pypi_reachable = check_pypi_updates({**core, **pip_extras})

    print_version_info(f'hello-robot-stretch4-body : {core["hello-robot-stretch4-body"]}',
                       updates.get('hello-robot-stretch4-body'))
    print_version_info(f'hello-robot-stretch4-urdf : {core["hello-robot-stretch4-urdf"]}',
                       updates.get('hello-robot-stretch4-urdf'))
    print_info(f'Python                    : {py_ver}')

    ros_distro = os.environ.get('ROS_DISTRO', '')
    if ros_distro:
        print_info(f'ROS2 Distro               : {ros_distro}')

    if pip_extras:
        col = max(len(k) for k in pip_extras)
        click.secho('\n  Python / pip:', fg='white', bold=True)
        for name in sorted(pip_extras):
            print_version_info(f'  {name:<{col}} : {pip_extras[name]}', updates.get(name))

    if not pypi_reachable:
        print_warn('Could not reach PyPI — update availability not checked')
    elif updates:
        print_info(f'Update with: {PIP_UPDATE_CMD} ' + ' '.join(sorted(updates)))
        print_info('Run with --check_updates for pip + firmware update commands')

    # ROS2 packages — auto-discovered via AMENT_PREFIX_PATH
    try:
        import xml.etree.ElementTree as ET
        ros2_pkgs = {}
        for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(':'):
            share = os.path.join(prefix, 'share')
            if not os.path.isdir(share):
                continue
            for pkg in os.listdir(share):
                if pkg in ros2_pkgs:
                    continue
                if 'stretch' not in pkg.lower() and 'hello' not in pkg.lower():
                    continue
                xml_path = os.path.join(share, pkg, 'package.xml')
                if os.path.isfile(xml_path):
                    try:
                        tree = ET.parse(xml_path)
                        ver = tree.find('version')
                        ros2_pkgs[pkg] = ver.text.strip() if ver is not None else 'unknown'
                    except Exception:
                        pass
        if ros2_pkgs:
            col = max(len(k) for k in ros2_pkgs)
            click.secho('\n  ROS2 Packages:', fg='white', bold=True)
            for pkg in sorted(ros2_pkgs):
                print_info(f'  {pkg:<{col}} : {ros2_pkgs[pkg]}')
    except Exception:
        pass

    # ROS2 workspace repos — git branch/sha and whether the remote is ahead
    src, repos = check_workspace_repos()
    if repos:
        click.secho(f'\n  ROS2 Workspace ({src}):', fg='white', bold=True)
        print_workspace_repos(repos, indent=4)
        if any(x['command'] for x in repos):
            print_info('Run with --repos for the git commands to update them')


def check_usb_devices():
    print_section('USB Devices')
    expected = [
        'hello-motor-arm',
        'hello-motor-lift',
        'hello-motor-omni-0',
        'hello-motor-omni-1',
        'hello-motor-omni-2',
        'hello-power-periph',
        'hello-feetech-wrist',
        'hello-esp32',
        'hello-nav-head-camera-stereo',
        'hello-pixart-j3',
    ]
    dev_list = set(os.listdir('/dev'))
    all_pass = True
    for dev in expected:
        present = dev in dev_list
        print_result(present, f'/dev/{dev}')
        if not present:
            all_pass = False
    if args.verbose:
        extras = sorted(e for e in dev_list if fnmatch.fnmatch(e, 'hello-*') and e not in expected)
        for extra in extras:
            print_info(f'(extra) /dev/{extra}')
    return all_pass


DEVICE_LABELS = {
    'hello-motor-arm':    'Arm Stepper',
    'hello-motor-lift':   'Lift Stepper',
    'hello-motor-omni-0': 'Omni Wheel 0',
    'hello-motor-omni-1': 'Omni Wheel 1',
    'hello-motor-omni-2': 'Omni Wheel 2',
    'hello-power-periph': 'Power Periph (pimu2)',
    'hello-pixart-j3':    'PixArt J3 (line sensor)',
    'hello-esp32':        'ESP32',
}

# Firmware version can't be read back from these boards
UNQUERYABLE_FIRMWARE = ('hello-pixart-j3', 'hello-esp32')


def firmware_use_device():
    """Devices to query for firmware, honoring the SE4UNH (no arm) configuration."""
    from stretch4_body.core.device import Device
    d = Device(req_params=False)
    is_unh = d.robot_params.get('robot', {}).get('model_name') == 'SE4UNH'
    return {
        'hello-esp32':        True,
        'hello-motor-arm':    not is_unh,
        'hello-motor-lift':   True,
        'hello-motor-omni-0': True,
        'hello-motor-omni-1': True,
        'hello-motor-omni-2': True,
        'hello-power-periph': True,
        'hello-pixart-j3':    True,
    }


def query_firmware(use_device):
    """
    Query installed and recommended firmware with all SDK log/print output suppressed.
    The robot server must be stopped first (exclusive USB access).

    Returns (fw_installed, fw_recommended, error_str). fw_installed is None if the
    query failed; fw_recommended is None if the available-firmware lookup failed
    (e.g. no internet).
    """
    from stretch4_body.core.factory.firmware_installed import FirmwareInstalled
    from stretch4_body.core.factory.firmware_recommended import FirmwareRecommended

    logging.disable(logging.CRITICAL)
    _old_stdout, _old_stderr = sys.stdout, sys.stderr
    sys.stdout = sys.stderr = io.StringIO()
    try:
        fw_installed = FirmwareInstalled(use_device)
    except Exception as e:
        sys.stdout, sys.stderr = _old_stdout, _old_stderr
        logging.disable(logging.NOTSET)
        return None, None, str(e)
    try:
        fw_recommended = FirmwareRecommended(use_device, installed=fw_installed)
    except Exception:
        fw_recommended = None
    sys.stdout, sys.stderr = _old_stdout, _old_stderr
    logging.disable(logging.NOTSET)
    return fw_installed, fw_recommended, None


def check_firmware_versions():
    """Query installed firmware via FirmwareInstalled. Server must be stopped first."""
    print_section('Firmware Versions')

    use_device = firmware_use_device()
    fw_installed, fw_recommended, err = query_firmware(use_device)
    if fw_installed is None:
        print_warn(f'Could not query firmware: {err}')
        return True

    all_pass = True
    for dev_name, enabled in use_device.items():
        if not enabled:
            continue
        label = DEVICE_LABELS.get(dev_name, dev_name)

        if not fw_installed.is_device_valid(dev_name):
            print_result(False, f'{label}: not found / comms failure')
            all_pass = False
            continue

        # ESP32 and PixArt J3 don't expose queryable firmware versions
        if dev_name in UNQUERYABLE_FIRMWARE:
            dev_present = os.path.exists(f'/dev/{dev_name}')
            status = 'present' if dev_present else 'not present'
            print_result(dev_present, f'{label}: {status} (firmware version not queryable)')
            if not dev_present:
                all_pass = False
            continue

        fw_ver   = fw_installed.config_info[dev_name]['board_info']['firmware_version']
        proto    = fw_installed.config_info[dev_name]['board_info'].get('protocol_version', '?')
        hw_id    = fw_installed.config_info[dev_name]['board_info'].get('hardware_id', '?')
        proto_ok = fw_installed.config_info[dev_name].get('installed_protocol_valid', True)

        up_to_date = True
        rec_str = ''
        if fw_recommended is not None:
            rec = fw_recommended.recommended.get(dev_name)
            installed_ver = fw_installed.get_version(dev_name)
            if rec is not None:
                if rec > installed_ver:
                    up_to_date = False
                    rec_str = f' → recommended: {rec}'
                elif rec < installed_ver:
                    rec_str = ' (dev/ahead of recommended)'

        detail = f'protocol: {proto}  |  hw_id: {hw_id}  |  proto_valid: {proto_ok}'
        if not proto_ok:
            print_result(False, f'{label}: {fw_ver}{rec_str}')
            print_info(detail)
            all_pass = False
        elif not up_to_date:
            print_warn(f'{label}: {fw_ver}{rec_str}')
            print_info(detail)
        else:
            print_result(True, f'{label}: {fw_ver}{rec_str}')
            print_info(detail)

    return all_pass


def collect_firmware_updates():
    """
    Delegate the firmware check to FirmwareRecommended — the same report and
    recommendation that `REx_firmware_updater --recommended` produces — and capture
    its output. The robot server must be stopped before calling this.

    Returns a dict with:
      table   : the recommended-firmware table as printed by the firmware tooling
      command : the 'REx_firmware_updater --install ...' line it recommends, or None
      error   : message if the check could not be run, else None
    """
    out = {'table': '', 'command': None, 'error': None}

    use_device = firmware_use_device()
    fw_installed, fw_recommended, err = query_firmware(use_device)
    if fw_installed is None:
        out['error'] = f'Could not query firmware: {err}'
        return out
    if fw_recommended is None:
        out['error'] = ('Could not fetch the available firmware list — '
                        'check the robot\'s internet connection')
        return out

    # Capture the tool's own report instead of re-deriving which boards need flashing
    logging.disable(logging.CRITICAL)
    _old_stdout, _old_stderr = sys.stdout, sys.stderr
    sys.stdout = sys.stderr = buf = io.StringIO()
    try:
        fw_recommended.pretty_print()
        fw_recommended.print_recommended_args()
    except Exception as e:
        sys.stdout, sys.stderr = _old_stdout, _old_stderr
        logging.disable(logging.NOTSET)
        out['error'] = f'Firmware recommendation failed: {e}'
        return out
    finally:
        sys.stdout, sys.stderr = _old_stdout, _old_stderr
        logging.disable(logging.NOTSET)

    # print_recommended_args() emits 'REx_firmware_updater --install  --pimu ...' when an
    # upgrade is recommended, or 'Firmware upgrade not necessary' when nothing is needed.
    SKIP = ('Run recommended command', 'Collecting information', 'Firmware upgrade not necessary')
    table = []
    for line in buf.getvalue().splitlines():
        stripped = line.strip()
        if stripped.startswith('REx_firmware_updater'):
            out['command'] = ' '.join(stripped.split())
            continue
        if stripped.startswith(SKIP):
            continue
        line = line.rstrip()
        if not line and (not table or not table[-1]):
            continue  # drop leading blanks and collapse blank runs
        table.append(line)
    out['table'] = '\n'.join(table).rstrip()

    return out


def _print_pip_row(name, current, latest, col, checked=True):
    """Print one 'package : current → latest' row."""
    if latest:
        click.secho(f'    {name:<{col}} : {current}  →  {latest}', fg='yellow', nl=False)
        click.secho('  (Update Available)', fg='yellow', bold=True)
    else:
        print_info(f'{name:<{col}} : {current}' + ('  (up to date)' if checked else '  (not checked)'))


def check_updates():
    """
    Report pip and firmware updates, then print the exact commands to apply them.
    Stops and restarts the robot server, since firmware queries need the USB devices.

    Returns True if both checks completed — not whether updates were found.
    """
    click.secho('\n======== Update Check ========', fg='cyan', bold=True)

    # ---- pip packages ------------------------------------------------------
    print_section('Python / pip Packages')
    core, extras = discover_pip_packages()
    installed   = {**core, **extras}
    hello_pkgs  = sorted(n for n in installed if n.lower().startswith('hello-robot-'))
    pip_updates, pypi_reachable = check_pypi_updates(installed)

    if not hello_pkgs:
        print_warn('No hello-robot-* packages found in this environment')
    else:
        col = max(len(n) for n in hello_pkgs)
        for name in hello_pkgs:
            current = installed[name]
            checked = pypi_reachable and current not in ('unknown', '')
            _print_pip_row(name, current, pip_updates.get(name), col, checked)
    if not pypi_reachable:
        print_warn('Could not reach PyPI — pip update check incomplete')

    # ---- ROS2 workspace repos ---------------------------------------------
    print_section('ROS2 Workspace Repos')
    src, repos = check_workspace_repos()
    git_unreachable = []
    if not repos:
        print_warn(f'No repos found in {src}')
    else:
        print_info(f'Workspace: {src}', indent=2)
        print_workspace_repos(repos)
        git_unreachable = [x['name'] for x in repos if x['status'] == GIT_UNREACHABLE]
        if git_unreachable:
            print_warn('Could not reach the remote for: ' + ', '.join(git_unreachable))

    # ---- firmware ----------------------------------------------------------
    print_section('Firmware')
    click.secho('  Firmware queries need exclusive access to the USB devices.', fg='yellow')
    _kill_server()
    fw = collect_firmware_updates()
    _restart_server()

    if fw['error']:
        print_warn(fw['error'])
    else:
        # Printed unindented — the table is already 110 columns wide
        for line in fw['table'].splitlines():
            click.secho(line, fg='white')

    # ---- copy-paste commands ----------------------------------------------
    print_section('Commands To Run')
    cmds = []
    if pip_updates:
        cmds.append(f'{PIP_UPDATE_CMD} ' + ' '.join(sorted(pip_updates)))
    if fw['command']:
        cmds.append(fw['command'])
    git_cmds = [x['command'] for x in repos if x['command']]

    if cmds or git_cmds:
        click.echo()
        for cmd in cmds:
            click.secho(f'    {cmd}', fg='green', bold=True)
        if git_cmds:
            if cmds:
                click.echo()
            for cmd in git_cmds:
                click.secho(f'    {cmd}', fg='green', bold=True)
            click.secho('    cd ~/ament_ws && colcon build --symlink-install', fg='green', bold=True)
        click.echo()
        if len(cmds) > 1:
            print_info('Run them in this order — a newer stretch4_body may recommend newer firmware.')
        print_info('Re-run with --check_updates afterwards to confirm.')
    elif not pypi_reachable or fw['error'] or git_unreachable:
        print_warn('No updates found, but the check was incomplete (see warnings above)')
    else:
        click.secho('\n  Everything is up to date — no commands to run.', fg='green', bold=True)
    click.echo()

    # Exit status reflects whether the checks ran, not whether updates were found
    return pypi_reachable and not fw['error'] and not git_unreachable


def check_power_periph():
    print_section('Power & Battery')
    all_pass = True
    ps = r.power_periph.status

    soc = ps.get('battery_soc', 0)
    if soc >= 20:
        print_result(True, f'Battery SOC = {soc:.0f}%')
    elif soc >= 10:
        print_warn(f'Battery SOC = {soc:.0f}% (low — consider charging)')
    else:
        print_result(False, f'Battery SOC = {soc:.0f}% (critically low!)')
        all_pass = False

    soh = ps.get('battery_soh', 0)
    if soh >= 75:
        print_result(True, f'Battery SOH = {soh:.1f}%')
    else:
        print_warn(f'Battery SOH = {soh:.1f}% (degraded battery)')

    for label, key, vmin, vmax in [
        ('Bus Voltage (V)',  'voltage',     20.0, 30.0),
        ('CPU Voltage (V)',  'voltage_cpu', 15.0, 30.0),
        ('12V Rail (V)',     'voltage_12v0', 10.0, 14.0),
        ('5V Rail (V)',      'voltage_5v0',   4.5,  5.5),
    ]:
        val = ps.get(key, 0)
        if val > 0:
            p, msg = val_in_range(label, val, vmin, vmax)
            print_result(p, msg)
            if not p:
                all_pass = False

    runstop = ps.get('runstop_event', False)
    print_warn('Runstop is active') if runstop else print_result(True, 'Runstop not active')

    temp = ps.get('temp', 0)
    p, msg = val_in_range('Board Temp (°C)', temp, 0, 80)
    print_result(p, msg)
    if not p:
        all_pass = False

    if args.verbose:
        print_info(f'Battery Current  : {ps.get("battery_current", 0):.2f} A')
        print_info(f'Adapter present  : {ps.get("adapter_voltage_present", False)}')
        print_info(f'Adapter fault    : {ps.get("adapter_fault", False)}')
        print_info(f'Charging         : {ps.get("charger_is_charging", False)}')

    return all_pass


def check_esp32():
    print_section('ESP32 Connectivity')
    ps = r.power_periph.status

    esp32_present = os.path.exists('/dev/hello-esp32')
    print_result(esp32_present, '/dev/hello-esp32 present')
    if not esp32_present:
        return False

    if args.verbose:
        print_info(f'Aux CPU on  : {ps.get("cpu_on_sts", False)}')

    return True

LINE_SENSOR_MIN_HZ = 25.0
LINE_SENSOR_MEASURE_S = 2.0
LINE_SENSOR_SETTLE_S = 3.0


def check_line_sensors():
    print_section('Line Sensors (hello-pixart-j3)')

    from stretch4_body.subsystem.line_sensor import connect

    subsystems = list(r.params.get('server', {}).get('subsystems', []) or []) if r is not None else []
    line_sensor = getattr(r, 'line_sensor_loop', None) if r is not None else None

    if line_sensor is None and 'line_sensor_loop' in subsystems:
        print_result(False, 'line_sensor_loop is ENABLED in params but the client '
                            'has no handle for it — the subsystem failed to start')
        return False

    opened_here = False
    if line_sensor is not None:
        # Reuse the loop the server is already running — opening the port a
        # second time would just be refused by the one that has it.
        conn = connect.LineSensorConnection(connect.SERVER, line_sensor,
                                            lambda: None, r.pull_status)
    else:
        print_info('line_sensor_loop is not running as a server subsystem — '
                   'reading the board directly for this check.')
        print_info('If you want to use line sensors, enable line_sensor_loop under '
                   'server.subsystems in stretch_user_params.yaml.')
        try:
            conn = connect.open_line_sensors('stretch_system_check', verbose=False)
        except connect.LineSensorUnavailable as exc:
            print_result(False, f'No route to the line sensors: {exc.detail}')
            return False
        opened_here = True

    print_info(conn.describe())
    try:
        return _check_line_sensors_on(conn, just_opened=opened_here)
    finally:
        if opened_here:
            conn.close()


def _check_line_sensors_on(conn, just_opened):
    line_sensor = conn.loop
    conn.pull_status()

    all_pass = True
    lss = line_sensor.status
    health = lss.get('health') or {}
    sensor_names = line_sensor.params.get('sensor_names', [])

    # -- the link ----------------------------------------------------------
    # frame_id > 0 used to be the whole test. It stays true forever after one
    # good frame, so this check passed with the board unplugged.
    port_open = bool(health.get('port_open', False))
    print_result(port_open, 'Serial port open (/dev/hello-pixart-j3)')
    all_pass &= port_open

    if not health.get('streaming', False):
        print_warn('Streaming is OFF — cliff detection is disabled')
        all_pass = False

    # -- which sensors are actually alive ----------------------------------
    dead = list(health.get('sensors_dead', []))
    disabled = list(health.get('disabled_sensors', []))
    ok = [sn for sn in sensor_names if sn not in dead and sn not in disabled]
    print_result(not dead, f'{len(ok)}/{len(sensor_names)} sensors reporting'
                           + (f' — DEAD: {", ".join(dead)}' if dead else ''))
    all_pass &= not dead
    if disabled:
        print_warn(f'DISABLED at runtime (not a fault): {", ".join(disabled)} — '
                   f'{len(disabled)} of {len(sensor_names)} sensors are not looking')
    else:
        print_result(True, f'All {len(sensor_names)} sensors enabled (none disabled)')

  
    from stretch4_body.tools import stretch_line_sensor_hz_check as hz

    active = [sn for sn in sensor_names if sn not in disabled]
    rates, span = {}, 0.0
    if not active:
        print_warn('Every sensor is disabled — nothing to time')
        all_pass = False
    else:
        if just_opened:
            hz.settle(conn, active, max_s=LINE_SENSOR_SETTLE_S)
        rates, span = hz.measure(conn, active, LINE_SENSOR_MEASURE_S)

        slowest = min(rates[sn]['advance_hz'] for sn in active)
        p = slowest >= LINE_SENSOR_MIN_HZ
        print_result(p, f'Frame rate {slowest:.1f} Hz on the slowest sensor '
                        f'(need >= {LINE_SENSOR_MIN_HZ:.0f} Hz, measured over {span:.1f} s)')
        all_pass &= p

    # -- per sensor --------------------------------------------------------
    for sn in sensor_names:
        s = lss.get(sn, {})
        if not isinstance(s, dict):
            print_result(False, f'{sn}: no status block')
            all_pass = False
            continue
        if sn in disabled:
            print_info(f'{sn}: disabled')
            continue
        m = rates.get(sn, {})
        s_rate = m.get('advance_hz', 0.0)
        missed = s.get('missed_frames', 0)
        good = sn not in dead and s_rate >= LINE_SENSOR_MIN_HZ and not m.get('backwards')
        watched_every_frame = m.get('fresh_hz', 0.0) >= 0.9 * s_rate
        print_result(good, f'{sn}: {s_rate:.1f} Hz'
                           + (f', dropped {m["skips"]}x (longest gap {m["max_gap"]} frames)'
                              if m.get('skips') and watched_every_frame else '')
                           + (f', missed {missed} frames' if missed else '')
                           + (f', frame_id went BACKWARDS {m["backwards"]}x' if m.get('backwards') else ''))
        all_pass &= good

        fresh = m.get('fresh_hz', 0.0)
        if s_rate >= LINE_SENSOR_MIN_HZ and fresh < LINE_SENSOR_MIN_HZ:
            print_warn(f'{sn}: only {fresh:.1f} Hz of that reaches a reader — '
                       f'status is being delivered slower than the sensor runs')
        elif args.verbose:
            print_info(f'{sn}: {fresh:.1f} Hz of new frames reaching a reader')

    # -- a sensor missing from every frame -----------------------------------
    # These climb together at the frame rate when a sensor is structurally
    # absent. Rising counters are the signal; a nonzero total may just be
    # history from an earlier fault, so report rather than fail on it.
    incomplete = health.get('frame_not_full_err', 0)
    if incomplete:
        print_warn(f'{incomplete} incomplete frames since startup — a sensor '
                   f'dropped out of frames')

    restarts = health.get('reader_restarts', 0)
    if restarts:
        print_warn(f'Serial port has self-recovered {restarts} time(s) — '
                   f'suspect a flaky cable if this keeps climbing')

    decode = health.get('decode_errors', 0)
    if decode:
        print_warn(f'{decode} decode errors since startup')

    # -- calibration -------------------------------------------------------
    cal = lss.get('calibration') or {}
    loaded, rejected = cal.get('loaded', []), cal.get('rejected', {})
    print_result(len(loaded) == len(sensor_names),
                 f'Calibration: {len(loaded)}/{len(sensor_names)} tares loaded')
    for name, why in sorted(rejected.items()):
        print_info(f'{name}: NO TARE ({str(why).split(":")[0]})')
    all_pass &= len(loaded) == len(sensor_names)

    return all_pass


def check_omnibase():
    print_section('OmniBase (3-Wheel Drive)')
    all_pass = True
    obs = r.omnibase.status

    for i in range(3):
        ws = obs.get(f'wheel_{i}', {})
        pos = ws.get('pos') if ws else None
        if pos is None:
            print_result(False, f'omni-{i}: no status data')
            all_pass = False
            continue
        print_result(True, f'omni-{i}: pos = {pos:.3f} rad')
        if args.verbose:
            effort = ws.get('effort_pct')
            vel    = ws.get('vel')
            if effort is not None:
                print_info(f'omni-{i} effort = {effort:.1f}%')
            if vel is not None:
                print_info(f'omni-{i} vel    = {vel:.3f} rad/s')

    return all_pass


def check_arm():
    print_section('Arm')
    all_pass = True
    arm_s   = r.arm.status
    motor_s = arm_s.get('motor', {})

    if motor_s.get('pos_calibrated', False):
        print_result(True, 'Arm is homed')
    else:
        print_warn('Arm not homed (pos_calibrated = False)')

    pos = arm_s.get('pos')
    if pos is not None:
        p, msg = val_in_range('Arm pos (m)', pos, -0.01, 0.56)
        print_result(p, msg)
        if not p:
            all_pass = False
    else:
        print_result(False, 'Arm pos not available')
        all_pass = False

    return all_pass


def check_lift():
    print_section('Lift')
    all_pass = True
    lift_s  = r.lift.status
    motor_s = lift_s.get('motor', {})

    if motor_s.get('pos_calibrated', False):
        print_result(True, 'Lift is homed')
    else:
        print_warn('Lift not homed (pos_calibrated = False)')

    pos = lift_s.get('pos')
    if pos is not None:
        p, msg = val_in_range('Lift pos (m)', pos, -0.01, 1.12)
        print_result(p, msg)
        if not p:
            all_pass = False
    else:
        print_result(False, 'Lift pos not available')
        all_pass = False

    return all_pass


def check_end_of_arm():
    print_section(f'End-of-Arm ({TOOL_DISPLAY.get(stretch_tool, stretch_tool)})')
    all_pass = True
    eoa    = r.end_of_arm
    joints = getattr(eoa, 'joints', [])

    if not joints:
        print_warn('No joints defined in end-of-arm params')
        return True

    eoa_s = eoa.status
    for joint_name in joints:
        js = eoa_s.get(joint_name, {})
        if not js:
            print_result(False, f'{joint_name}: no status data')
            all_pass = False
            continue

        pos            = js.get('pos')
        pos_calibrated = js.get('pos_calibrated', False)
        temp           = js.get('temp')
        hw_err         = js.get('hardware_error', 0)

        if pos is None:
            print_result(False, f'{joint_name}: no position data')
            all_pass = False
            continue

        joint_ok = (hw_err == 0) and (temp is None or temp < 70)
        temp_str = f'  temp={temp:.0f}°C' if temp is not None else ''
        err_str  = f'  hw_error={hw_err}' if hw_err else ''
        print_result(joint_ok, f'{joint_name}: pos={pos:.3f} rad  homed={int(pos_calibrated)}{temp_str}{err_str}')
        if not joint_ok:
            all_pass = False

        if args.verbose:
            effort  = js.get('effort')
            curr_mA = js.get('current_mA')
            if effort is not None:
                print_info(f'{joint_name} effort = {effort:.1f}%')
            if curr_mA is not None:
                print_info(f'{joint_name} current = {curr_mA:.1f} mA')

    return all_pass


def check_imu():
    print_section('IMU (Base)')
    imu_s = r.power_periph.status.get('imu', {})
    if not imu_s:
        print_result(False, 'IMU status not available')
        return False

    all_pass = True
    az = imu_s.get('az', 0)
    p, msg = val_in_range('IMU az (m/s²)', az, 7.0, 11.0)
    print_result(p, msg)
    if not p:
        all_pass = False

    if args.verbose:
        ax, ay = imu_s.get('ax', 0), imu_s.get('ay', 0)
        print_info(f'ax = {ax:.3f}  ay = {ay:.3f}  az = {az:.3f} m/s²')
        print_info(f'roll = {imu_s.get("roll", 0):.4f} rad  pitch = {imu_s.get("pitch", 0):.4f} rad')
        print_info(f'gravity_tilt = {imu_s.get("gravity_tilt", 0):.4f} rad')

    return all_pass


def check_eye_animations():
    print_section('Eye LED Animations')

    if r is None:
        print_warn('Server offline — cannot check eye animation status')
        return True

    from stretch4_body.core.device import Device
    eye_cfg  = Device(req_params=False).robot_params.get('sentry_eye_animations', {})
    enabled  = bool(eye_cfg.get('enabled', 0))
    behavior = eye_cfg.get('behavior', 'unknown')

    if not enabled:
        print_result(False, 'sentry_eye_animations disabled in robot params')
        return False

    print_result(True, f'sentry_eye_animations enabled  (behavior: {behavior})')

    proto = None
    try:
        proto_str = r.power_periph.status.get('protocol_version')
        if proto_str:
            proto = int(proto_str.lstrip('p'))
    except Exception:
        pass

    if proto is not None:
        ok = proto >= 13
        print_result(ok, f'PowerPeriph protocol: p{proto} (≥p13 required for LED support)')
        return ok
    else:
        print_warn('Could not read PowerPeriph protocol version')
        return True


def check_audio():
    print_section('Audio')
    try:
        res = subprocess.run(
            [sys.executable, '-m', 'stretch4_body.tools.stretch_audio_test', '--check-only']
        )
        return res.returncode == 0
    except Exception as e:
        print_result(False, f'Failed to run audio tests: {e}')
        return False


def check_calibrations():
    print_section('Calibrations Present')
    all_pass = True

    fleet_path = os.environ.get('HELLO_FLEET_PATH', os.path.expanduser('~/stretch_user'))
    fleet_id   = os.environ.get('HELLO_FLEET_ID', stretch_serial_no)
    cal_root   = os.path.join(fleet_path, fleet_id)

    click.secho('    Steppers:', fg='white', bold=True)
    stepper_dir = os.path.join(cal_root, 'calibration_steppers')
    for m in ['hello-motor-arm', 'hello-motor-lift',
              'hello-motor-omni-0', 'hello-motor-omni-1', 'hello-motor-omni-2']:
        files = ([f for f in os.listdir(stepper_dir)
                  if f.startswith(m + '_') and f.endswith('.yaml')]
                 if os.path.isdir(stepper_dir) else [])
        ok = len(files) > 0
        print_result(ok, m, indent=6)
        if not ok:
            all_pass = False

    click.secho('    Cameras:', fg='white', bold=True)
    cam_dir = os.path.join(cal_root, 'calibration_cameras')
    from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

    # Load each camera's calibration the way the rest of the codebase does, rather than looking for
    # files: it checks that the entry exists, parses, and matches the camera's configured size.
    for label, camera_type in (
        ('intrinsics: left',          RGBCameras.left()),
        ('intrinsics: right',         RGBCameras.right()),
        ('intrinsics: center',        RGBCameras.center()),
        ('intrinsics: gripper left',  RGBCameras.gripper_left),
        ('intrinsics: gripper right', RGBCameras.gripper_right),
    ):
        try:
            calibration = camera_type.load_calibration()
            print_result(True, f'{label}  ({calibration.width}x{calibration.height})', indent=6)
        except Exception as e:
            print_result(False, label, indent=6)
            print_info(str(e).strip(), indent=8)
            all_pass = False

    ok = os.path.isfile(os.path.join(cam_dir, 'camera_extrinsics.yaml'))
    print_result(ok, 'extrinsics', indent=6)
    if not ok:
        all_pass = False

    click.secho('    Line Sensors:', fg='white', bold=True)
    ls_dir = os.path.join(cal_root, 'calibration_line_sensors')

    ls_names = (rp.RobotParams._robot_params
                .get('line_sensor_loop', {})
                .get('sensor_names', [f'sensor_{i}' for i in range(6)]))
    tare_dir = calibration_store.tare_dir(ls_dir)
    if os.path.isdir(tare_dir):
        for name in ls_names:
            has_cal = os.path.isfile(calibration_store.tare_path(ls_dir, name))
            print_result(has_cal, name, indent=6)
            if not has_cal:
                all_pass = False
    else:
        print_result(False, f'tare directory missing: {tare_dir}', indent=6)
        all_pass = False

    click.secho('    Lidars:', fg='white', bold=True)
    hesai_dir = os.path.join(cal_root, 'calibration_hesais')
    for side in ('left', 'right'):
        ok = os.path.isfile(os.path.join(hesai_dir, f'{side}_lidar_calibration.dat'))
        print_result(ok, f'{side} lidar', indent=6)
        if not ok:
            all_pass = False

    click.secho('    OmniBase:', fg='white', bold=True)
    imu_dir = os.path.join(cal_root, 'calibration_omnibase_imu')
    has_imu_cal = os.path.isdir(imu_dir) and any(os.listdir(imu_dir))
    print_result(has_imu_cal, 'IMU calibration', indent=6)
    if not has_imu_cal:
        all_pass = False

    return all_pass


def _ptc_call(fn, *args, label='', fail_ok=False):
    """
    Call a stretch4_pyhesai_wrapper.ptc_client helper, suppressing SDK stdout/stderr.
    Returns (value, error_string).  error_string is None on success.
    """
    _o, _e = sys.stdout, sys.stderr
    try:
        logging.disable(logging.CRITICAL)
        sys.stdout = sys.stderr = io.StringIO()
        result = fn(*args)
        sys.stdout, sys.stderr = _o, _e
        logging.disable(logging.NOTSET)
        return result, None
    except Exception as exc:
        sys.stdout, sys.stderr = _o, _e
        logging.disable(logging.NOTSET)
        return None, str(exc)


def _check_lidar_ptc(ip):
    """Report the PTC configuration of a single lidar.

    These settings are advisory: a deviation is worth flagging but does not fail the sensor check,
    unlike the lidar being unreachable.
    """
    from stretch4_pyhesai_wrapper.ptc_client import (
        FILTER_NAMES, FILTER_STRONG,
        PTP_LOCK_OFFSET_US, PTP_STATUS_LOCKED, PTP_STATUS_NAMES,
        RETURN_MODE_LAST_AND_STRONGEST, RETURN_MODE_NAMES,
        get_lidar_ptp_status, get_point_cloud_config,
        get_ptp_lock_offset_us, get_return_mode,
    )

    mode_val, err = _ptc_call(get_return_mode, ip)
    if err:
        print_warn(f'Return mode query failed: {err}')
    else:
        mode_name = RETURN_MODE_NAMES.get(mode_val, f'mode {mode_val}')
        ok = mode_val == RETURN_MODE_LAST_AND_STRONGEST
        suffix = '' if ok else f'  (expected: {RETURN_MODE_NAMES[RETURN_MODE_LAST_AND_STRONGEST]})'
        if ok:
            print_result(True, f'Return mode = {mode_val} ({mode_name})')
        else:
            print_warn(f'Return mode = {mode_val} ({mode_name}){suffix}')

    cfg_val, err = _ptc_call(get_point_cloud_config, ip)
    if err:
        print_warn(f'Point-cloud filter query failed: {err}')
    else:
        _, filt = cfg_val
        filter_name = FILTER_NAMES.get(filt, f'filter {filt}')
        ok = filt == FILTER_STRONG
        suffix = '' if ok else f'  (expected: {FILTER_NAMES[FILTER_STRONG]})'
        if ok:
            print_result(True, f'Filter = {filt} ({filter_name})')
        else:
            print_warn(f'Filter = {filt} ({filter_name}){suffix}')

    offset_val, err = _ptc_call(get_ptp_lock_offset_us, ip)
    if err:
        print_warn(f'PTP lock offset query failed: {err}')
    else:
        ok = offset_val == PTP_LOCK_OFFSET_US
        suffix = '' if ok else f'  (expected: {PTP_LOCK_OFFSET_US} µs)'
        if ok:
            print_result(True, f'PTP lock offset = {offset_val} µs')
        else:
            print_warn(f'PTP lock offset = {offset_val} µs{suffix}')

    ptp_val, err = _ptc_call(get_lidar_ptp_status, ip)
    if err:
        print_warn(f'PTP status query failed: {err}')
    else:
        status      = ptp_val.get('ptp_status')
        status_name = ptp_val.get('ptp_status_name', PTP_STATUS_NAMES.get(status, str(status)))
        ok = status == PTP_STATUS_LOCKED
        if ok:
            print_result(True, f'PTP status = {status_name}')
        else:
            print_warn(f'PTP status = {status_name}  (expected: locked)')


def _check_lidar_streaming(lidars, timeout=5.0):
    """
    Listen on Hesai UDP ports for packets from each lidar.
    Returns dict {side: bool} indicating which lidars sent data.
    """
    import socket, select, time
    LIDAR_PORTS = [2368, 2378]
    ip_to_side  = {cfg['ip']: side for side, cfg in lidars.items()}
    received    = {}

    sockets = []
    try:
        for port in LIDAR_PORTS:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.setblocking(False)
                s.bind(('0.0.0.0', port))
                sockets.append(s)
            except OSError as e:
                print_warn(f'Could not bind port {port}: {e}  (ensure no lidar driver is running)')

        if not sockets:
            return {side: False for side in lidars}

        deadline = time.time() + timeout
        while time.time() < deadline and len(received) < len(lidars):
            ready, _, _ = select.select(sockets, [], [], 0.1)
            for sock in ready:
                try:
                    _, addr = sock.recvfrom(2048)
                    sender_ip = addr[0]
                    side = ip_to_side.get(sender_ip)
                    if side and side not in received:
                        received[side] = True
                except socket.error:
                    pass
    finally:
        for s in sockets:
            s.close()

    return {side: received.get(side, False) for side in lidars}


# ==============================================================================
# Camera checks
# ==============================================================================

CAMERA_FPS_TOLERANCE = 0.50
CAMERA_WARMUP_SECS   = 2.0
CAMERA_MEASURE_SECS  = 3.0

SPINNER_FRAMES = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏']


def _spin(label, stop_event):
    """Animate a braille spinner on the label line until stop_event is set."""
    import itertools, time
    for frame in itertools.cycle(SPINNER_FRAMES):
        if stop_event.is_set():
            break
        sys.stdout.write(f'\r  {frame} {label}  ')
        sys.stdout.flush()
        time.sleep(0.08)
    # Clear the spinner line so the caller can print cleanly
    sys.stdout.write(f'\r{" " * (len(label) + 8)}\r')
    sys.stdout.flush()


def _with_spinner(label, fn):
    """Run fn() while a spinner animates next to label, and return whatever fn() returned."""
    import threading
    stop = threading.Event()
    thread = threading.Thread(target=_spin, args=(label, stop), daemon=True)
    thread.start()
    try:
        return fn()
    finally:
        stop.set()
        thread.join()


def _collect_synced_frames(camera, duration_s, warmup_s=0.0):
    """Pull frames off a SyncedCamera and count them per stream.

    Returns (counts, last_frames, elapsed), both dicts keyed by 'left', 'right', 'center' and 'depth'.
    get_frames() blocks on the device, so it is pumped on a daemon thread: a camera that never
    delivers leaves that thread parked until the caller stops the pipeline, instead of hanging the
    system check.
    """
    import threading, time

    counts, last_frames = {}, {}
    measuring, stop = threading.Event(), threading.Event()

    def pump():
        try:
            for synced in camera.get_frames():
                if stop.is_set():
                    return
                if synced is None or not measuring.is_set():
                    continue
                for name in ('left', 'right', 'center'):
                    frame = getattr(synced, name, None)
                    # The gripper adapter substitutes a zero-timestamp placeholder when a side is
                    # missing from the sync group; those never came off the sensor.
                    if frame is None or frame.timestamp == 0:
                        continue
                    counts[name] = counts.get(name, 0) + 1
                    last_frames[name] = frame
                if synced.depth is not None:
                    counts['depth'] = counts.get('depth', 0) + 1
                    last_frames['depth'] = synced.depth
        except Exception as e:
            logging.debug(f'Stopped pulling frames from the camera: {e}')

    thread = threading.Thread(target=pump, daemon=True)
    thread.start()

    time.sleep(warmup_s)
    measuring.set()
    t0 = time.time()
    time.sleep(duration_s)
    stop.set()
    elapsed = time.time() - t0
    thread.join(timeout=1.0)

    return counts, last_frames, elapsed


def _frame_resolution(frame):
    """(height, width) of a captured frame, decoding it first when the device encoded it to MJPEG.

    Takes either an ImageFrame or a raw depth array, so every stream can be measured the same way.
    """
    image = frame
    if hasattr(image, 'image'):
        image = frame.uncompress() if frame.is_compressed() else frame.image
    if image is None or getattr(image, 'ndim', 0) < 2:
        return None
    return image.shape[0], image.shape[1]


def _check_camera_streams(counts, last_frames, elapsed, streams):
    """Report every stream's activity, frame rate and resolution, grouped by check.

    `streams` holds (name, key, expected_fps, expected_size) with expected_size as (height, width).
    A stream that delivered nothing is reported once under "Stream active" and then left out of the
    other two groups, rather than repeating the same failure three times.
    """
    all_pass = True
    active = []

    print_info('Stream active:')
    for name, key, _, _ in streams:
        seen = counts.get(key, 0) > 0
        print_result(seen, name, indent=6)
        if seen:
            active.append(key)
        else:
            all_pass = False

    print_info('FPS:')
    for name, key, expected_fps, _ in streams:
        if key not in active:
            continue
        actual_fps = counts[key] / elapsed
        ok = actual_fps >= expected_fps * (1 - CAMERA_FPS_TOLERANCE)
        print_result(ok, f'{name}: {actual_fps:.1f}  (target {expected_fps})', indent=6)
        if not ok:
            all_pass = False

    print_info('Resolution:')
    for name, key, _, expected_size in streams:
        if key not in active:
            continue
        resolution = _frame_resolution(last_frames[key])
        if resolution is None:
            print_result(False, f'{name}: frame could not be decoded', indent=6)
            all_pass = False
            continue
        # Frames and CAMERA_CONFIGS both carry (height, width); the report shows width x height.
        ok = resolution == expected_size
        print_result(ok, f'{name}: {resolution[1]}×{resolution[0]}  '
                         f'(expected {expected_size[1]}×{expected_size[0]})', indent=6)
        if not ok:
            all_pass = False

    return all_pass


def _check_camera(label, camera_type, expected_usb_speed, check_depth=False):
    """Open one OAK camera through the adapter the robot streams with, and check its link and streams.

    Going through `RGBCameras.start_synced()` rather than a hand-rolled DepthAI pipeline means this
    check exercises the real pipeline, at the resolutions and frame rates in `CAMERA_CONFIGS`.
    """
    import time
    import depthai as dai

    try:
        camera = _with_spinner(f'{label}: connecting...', camera_type.start_synced)
    except Exception as e:
        click.secho(f'  {label}', fg='cyan', bold=True)
        print_result(False, f'Could not open camera: {e}')
        return False

    all_pass = True
    try:
        click.secho(f'  {label}', fg='cyan', bold=True)

        is_open = camera.is_open()
        print_result(is_open, f'Pipeline running  (id={camera.device.getDeviceId()})')
        if not is_open:
            return False

        speed = camera.device.getUsbSpeed()
        speed_ok = speed == getattr(dai.UsbSpeed, expected_usb_speed)
        print_result(speed_ok, f'USB speed: {speed.name}'
                               if speed_ok else
                               f'USB speed: {speed.name}  (expected {expected_usb_speed} — check cable)')
        if not speed_ok:
            all_pass = False

        measure_secs = CAMERA_WARMUP_SECS + CAMERA_MEASURE_SECS
        counts, last_frames, elapsed = _with_spinner(
            f'{label}: measuring streams ({measure_secs:.0f} s)...',
            lambda: _collect_synced_frames(camera, CAMERA_MEASURE_SECS, warmup_s=CAMERA_WARMUP_SECS),
        )

        def described(name, key, config):
            return name, key, config.fps, tuple(config.image_size)

        streams = [described('Left', 'left', camera.left), described('Right', 'right', camera.right)]
        # Only the head adapter has a center camera; the gripper adapter has no such attribute.
        center = getattr(camera, 'center', None)
        if center is not None:
            streams.append(described('Center', 'center', center))
        if check_depth:
            # The gripper's stereo depth is aligned to its right camera, so it matches it.
            streams.append(described('Depth', 'depth', camera.right))

        if not _check_camera_streams(counts, last_frames, elapsed, streams):
            all_pass = False

    except Exception as e:
        print_result(False, f'Camera check error: {e}')
        all_pass = False
    finally:
        try:
            camera.stop()
        except Exception:
            pass
        # The device needs a moment to become re-enumerable before the next camera is opened.
        time.sleep(1.0)

    return all_pass


def check_cameras():
    """Check the head and gripper OAK cameras through the adapters the robot streams with."""
    try:
        import depthai  # noqa: F401
    except ImportError:
        print_warn('depthai not installed — cannot check OAK cameras')
        return True

    from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

    # That import pulls in Device, whose class body reapplies the fleet logging configuration and
    # turns the root logger back up to INFO. Some adapter lines are logged with a bare
    # logging.info(), so quieten the root logger again to keep them out of this report.
    logging.getLogger().setLevel(logging.WARNING)

    all_pass = True
    # Each board is checked against the USB link speed it is expected to negotiate.
    for label, camera_type, usb_speed, check_depth in (
        ('Head camera (OAK-FFC-3P)',  RGBCameras.synced_left_right_center(), 'SUPER_PLUS', False),
        ('Gripper camera (OAK-D-SR)', RGBCameras.gripper_rgbd,               'HIGH',      True),
    ):
        if not _check_camera(label, camera_type, usb_speed, check_depth=check_depth):
            all_pass = False

    return all_pass


def check_sensors():
    """Check Hesai lidars (PTC config + streaming) and the OAK cameras."""
    import socket, time
    print_section('Sensors')
    all_pass = True

    LIDARS = {
        'left':  {'ip': '192.168.1.202', 'ptc_port': 9347},
        'right': {'ip': '192.168.1.201', 'ptc_port': 9347},
    }
    try:
        import yaml, importlib.util
        spec = importlib.util.find_spec('stretch4_pyhesai_wrapper')
        if spec:
            cfg_path = os.path.join(os.path.dirname(spec.origin), 'config.yaml')
            if os.path.isfile(cfg_path):
                with open(cfg_path) as f:
                    hw_cfg = yaml.safe_load(os.path.expandvars(f.read()))
                LIDARS['left']['ip']        = hw_cfg['left_lidar']['ip']
                LIDARS['right']['ip']       = hw_cfg['right_lidar']['ip']
                LIDARS['left']['ptc_port']  = hw_cfg['left_lidar']['ptc_port']
                LIDARS['right']['ptc_port'] = hw_cfg['right_lidar']['ptc_port']
    except Exception:
        pass

    print_section('Lidars (Hesai)')
    ptc_available = False
    try:
        from stretch4_pyhesai_wrapper.ptc_client import get_return_mode as _test_import  # noqa
        ptc_available = True
    except ImportError:
        print_warn('stretch4_pyhesai_wrapper.ptc_client not available — PTC config checks skipped')

    for side, cfg in LIDARS.items():
        ip, port = cfg['ip'], cfg['ptc_port']
        click.secho(f'\n  {side.capitalize()} lidar ({ip})', fg='cyan', bold=True)

        ping_ok = subprocess.call(
            ['ping', '-c', '3', '-i', '0.2', '-W', '1', ip],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        ) == 0

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            ptc_ok = sock.connect_ex((ip, port)) == 0
            sock.close()
        except Exception:
            ptc_ok = False

        if ping_ok:
            print_result(True, 'Ping reachable')
        elif ptc_ok:
            print_warn('Ping unanswered, but PTC responded -- treating as reachable')
        else:
            print_result(False, 'Ping reachable')

        print_result(ptc_ok, f'PTC reachable ({ip}:{port})')
        if not ptc_ok:
            all_pass = False
            continue

        if ptc_available:
            _check_lidar_ptc(ip)

    click.secho(f'\n  Streaming check (listening 5 s for UDP packets)...', fg='white')
    stream_results = _check_lidar_streaming(LIDARS, timeout=5.0)
    for side, streaming in stream_results.items():
        ip = LIDARS[side]['ip']
        print_result(streaming, f'{side.capitalize()} lidar ({ip}): streaming UDP data')
        if not streaming:
            all_pass = False

    print_section('Cameras')
    if not check_cameras():
        all_pass = False

    return all_pass

# ==============================================================================
# Server lifecycle helpers
# ==============================================================================

def _kill_server():
    click.secho('  Stopping robot server...', fg='yellow')
    ret = subprocess.call(
        ['stretch_body_server', '--kill'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    if ret == 0:
        click.secho('  Server stopped.', fg='yellow')
    else:
        click.secho(f'  Server stop may have failed (exit code {ret}).', fg='yellow')
    return ret == 0


def _restart_server():
    click.secho('  Restarting robot server...', fg='yellow')
    subprocess.Popen(
        ['stretch_body_server', '--restart'],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    click.secho('  Server restarting in background.', fg='yellow')


# ==============================================================================
# Diagnostics export
# ==============================================================================

_TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_TIMEOUT_S = 900


def _run_tool(tool, tool_args, label):
    """Runs one of the sibling tools in a subprocess and returns its combined output."""
    click.secho(f'  Running {label}...', fg='yellow')
    cmd = [sys.executable, os.path.join(_TOOLS_DIR, tool)] + tool_args
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=EXPORT_TIMEOUT_S)
        out = proc.stdout
    except subprocess.TimeoutExpired as e:
        print_warn(f'{label} timed out after {EXPORT_TIMEOUT_S}s')
        out = (e.stdout or '') + f'\n[!] timed out after {EXPORT_TIMEOUT_S}s\n'
    except Exception as e:
        print_warn(f'{label} failed: {e}')
        out = f'[!] {label} failed: {e}\n'
    return f'$ {label}\n\n{out}'


# Shell commands captured into the bundle's commands/ directory:
#   (output file, command, timeout, what the output shows -- also used in the README)
EXPORT_COMMANDS = [
    ('dev_hello_devices.txt', 'ls -la /dev/hello*', 30,
     'The udev symlink for each board and the tty device it currently points at. A board '
     'missing from this listing never enumerated on USB, which explains most "device not '
     'found" failures elsewhere in the bundle.'),
    ('fleet_dir_listing.txt', 'ls -la "$HELLO_FLEET_PATH/$HELLO_FLEET_ID/"', 30,
     "The robot's fleet directory: parameter files, calibration folders and any .bak files, "
     'with their timestamps. Useful for spotting a parameter file that was edited or restored '
     'around the time a problem started.'),
    ('lsusb_verbose.txt', 'lsusb -v', 120,
     'Full USB descriptor dump for every device on the bus, including negotiated link speeds. '
     "\"Couldn't open device\" lines are expected -- the export does not run as root."),
    ('repos_listing.txt', 'ls -la ~/repos', 30,
     "The user's ~/repos checkouts. \"No such file or directory\" simply means this install "
     'has no ~/repos directory.'),
    ('stretch_body_server_status.txt', 'stretch_body_server --status', 180,
     'Server state, control loop rate, loop overruns and daemon status at export time. If no '
     'server was running, this instead shows the tail of the last archived session.'),
]

# Robot parameter files copied into the bundle's robot_params/ directory
EXPORT_PARAM_FILES = [
    'stretch_user_params.yaml',
    'stretch_configuration_params.yaml',
    'stretch_calibration_values.yaml',
]

UDEV_RULES_DIR = '/etc/udev/rules.d'
NUM_EXPORTED_WEB_TELEOP_SESSIONS = 3


def _run_shell(cmd, label, timeout=30):
    """Runs a shell command and returns its combined output, prefixed with the command itself."""
    click.secho(f'  Running {label}...', fg='yellow')
    try:
        proc = subprocess.run(['bash', '-c', cmd], stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, text=True, timeout=timeout)
        out = proc.stdout
        if proc.returncode != 0:
            print_warn(f'{label} exited with code {proc.returncode} (output still captured)')
            out += f'\n[!] exited with code {proc.returncode}\n'
    except subprocess.TimeoutExpired as e:
        print_warn(f'{label} timed out after {timeout}s')
        out = (e.stdout or '') + f'\n[!] timed out after {timeout}s\n'
    except Exception as e:
        print_warn(f'{label} failed: {e}')
        out = f'[!] {label} failed: {e}\n'
    return f'$ {cmd}\n\n{out}'


def _fleet_dir():
    """Path to $HELLO_FLEET_PATH/$HELLO_FLEET_ID, or None if the environment does not define it."""
    fleet_path = os.environ.get('HELLO_FLEET_PATH')
    fleet_id = os.environ.get('HELLO_FLEET_ID')
    if not fleet_path or not fleet_id:
        return None
    return os.path.join(fleet_path, fleet_id)


def _copy_into(src, dest_dir, label, name=None):
    """Copies a file or directory tree into dest_dir, optionally renaming it.

    Returns the name it was copied as, or None if the copy failed.
    """
    import shutil
    try:
        os.makedirs(dest_dir, exist_ok=True)
        name = name or os.path.basename(os.path.normpath(src))
        dest = os.path.join(dest_dir, name)
        if os.path.isdir(src):
            shutil.copytree(src, dest, dirs_exist_ok=True, ignore_dangling_symlinks=True)
        else:
            shutil.copy2(src, dest)
        return name
    except Exception as e:
        print_warn(f'Could not copy {label}: {e}')
        return None


def _recent_web_teleop_sessions(n=NUM_EXPORTED_WEB_TELEOP_SESSIONS):
    """Returns the paths of the n most recent web_teleop session directories, newest first."""
    teleop_dir = hu.get_stretch_directory('log/web_teleop')
    if not os.path.isdir(teleop_dir):
        return []
    sessions = [os.path.join(teleop_dir, d) for d in os.listdir(teleop_dir)
                if os.path.isdir(os.path.join(teleop_dir, d))]
    sessions.sort(key=os.path.getmtime, reverse=True)
    return sessions[:n]


def _bundle_info():
    from datetime import datetime
    return (
        f'Exported      : {datetime.now().isoformat()}\n'
        f'Model         : {_model_display}\n'
        f'Serial Number : {stretch_serial_no}\n'
        f'Batch         : {stretch_batch}\n'
        f'Tool          : {TOOL_DISPLAY.get(stretch_tool, stretch_tool)}\n'
        f'User          : {os.environ.get("USER", "N/A")}\n'
        f'Fleet path    : {os.environ.get("HELLO_FLEET_PATH", "N/A")}\n'
    )

def _bundle_readme(contents):
    """Builds the README shipped inside the bundle, describing the files it actually contains."""
    import textwrap
    from datetime import datetime

    def wrap(text):
        return textwrap.fill(text, width=78)

    def missing(what, where):
        return f'Missing -- {what} could not be collected ({where}).'

    # commands/
    command_sections = []
    for name, cmd, _timeout, description in EXPORT_COMMANDS:
        if name in contents['commands']:
            command_sections.append(f'#### commands/{name}\n\n    $ {cmd}\n\n{wrap(description)}')
    commands_section = '\n\n'.join(command_sections) if command_sections else missing(
        'command output', 'every command failed to run')

    # robot_params/
    if contents['params']:
        param_lines = '\n'.join(f'    {name}' for name in contents['params'])
        params_section = f"""### robot_params/
The robot's parameter files, copied from {contents['fleet_dir']}:

{param_lines}

{wrap('These are plain YAML. stretch_user_params.yaml holds the user overrides, '
      'stretch_configuration_params.yaml the factory configuration, and '
      'stretch_calibration_values.yaml the calibrated joint values. Together with the '
      'installed version in commands/stretch_system_check.txt they define how this robot '
      'was configured at export time. stretch_calibration_values.yaml is absent on a robot '
      'that has not been calibrated.')}"""
    else:
        params_section = '### robot_params/\n' + wrap(missing(
            'the robot parameter files',
            'HELLO_FLEET_PATH/HELLO_FLEET_ID is unset, or the files are absent'))

    # udev_rules.d/
    if contents['udev']:
        udev_section = f"""### udev_rules.d/
{wrap(f'A copy of {UDEV_RULES_DIR} from the robot. The Hello Robot rules file is what '
      'creates the /dev/hello-* symlinks in commands/dev_hello_devices.txt, so compare the '
      'two when a board enumerates on USB but no /dev/hello-* entry appears for it.')}"""
    else:
        udev_section = '### udev_rules.d/\n' + wrap(missing(
            f'{UDEV_RULES_DIR}', 'the directory is absent or unreadable'))

    # web_teleop_logs/
    if contents['web_teleop']:
        teleop_lines = '\n'.join(f'    {name}' for name in contents['web_teleop'])
        teleop_section = f"""### web_teleop_logs/
The {len(contents['web_teleop'])} most recent web teleop session directories:

{teleop_lines}

{wrap('Each directory is one web teleop session, named for its start time. The .txt files '
      'are the captured console output of the processes that session launched (ROS 2, the '
      'web server and the robot browser), and are plain text. These are only present if web '
      'teleop has been run on this robot.')}"""
    else:
        teleop_section = '### web_teleop_logs/\n' + wrap(missing(
            'web teleop session logs', 'web teleop has not been run on this robot'))

    # stretch_status export
    if contents['status_zips']:
        status_name = contents['status_zips'][0]
        status_section = f"""### {status_name}
Robot telemetry history (joint states, currents, voltages, temperatures, ...)
written by `stretch_status --export`. It holds one JSON file per logged run.

Replay it on any machine with stretch4_body installed -- pass the zip itself,
do not unzip it first:

    stretch_status --import {status_name}

Useful variations:

    # Visualize the whole file in Rerun instead of the console
    stretch_status --import {status_name} --rerun

    # Only show some fields
    stretch_status --import {status_name} --fields robot.lift robot.power_periph.voltage

    # Trim the replay window (seconds from the start / from the end of the file)
    stretch_status --import {status_name} --start_seconds_offset 10 --end_seconds_offset 5

{wrap('This is the largest file in the bundle; it is usually where an intermittent hardware '
      'issue is visible.')}"""
    else:
        status_section = """### stretch_status export
Missing -- `stretch_status --export` produced no telemetry archive. This usually
means no status history has been logged yet on this robot
(see $HELLO_FLEET_PATH/log/stretch_status)."""

    # stretch_body_server_logs/
    if contents['session_logs']:
        log_lines = '\n'.join(f'    {name}' for name in contents['session_logs'])
        logs_section = f"""### stretch_body_server_logs/
The {len(contents['session_logs'])} most recent `stretch_body_server` session logs:

{log_lines}

`stretch_body_server.log` (and any `.log.N` rotations) is the session that was
running when this bundle was exported, and is plain text -- open it directly.

Each `stretch_body_server_logs_<YYYYMMDDhhmmss>.tar.gz` is one finished session,
archived when that server shut down; the highest timestamp is the most recent.
Extract one with:

    tar -xzf stretch_body_server_logs_<timestamp>.tar.gz

or read it without extracting:

    tar -xzOf stretch_body_server_logs_<timestamp>.tar.gz stretch_body_server.log | less

These same logs can be exported on their own, without the rest of this bundle:

    stretch_body_server --export [DIR]"""
    else:
        logs_section = """### stretch_body_server_logs/
Missing -- no `stretch_body_server` session logs were found on this robot
(see $HELLO_FLEET_PATH/log/stretch_body_logger). The server may never have been
started on this install."""

    return f"""# Stretch 4 Diagnostics Bundle

Robot         : {_model_display} ({stretch_serial_no})
Tool          : {TOOL_DISPLAY.get(stretch_tool, stretch_tool)}
Exported      : {datetime.now().isoformat()} by {os.environ.get('USER', 'N/A')}
Created with  : stretch_system_check --export

Send this bundle to support@hello-robot.com when reporting an issue. Everything
in it was captured on the robot at export time; nothing here needs the robot to
be present in order to be read back. Every file is plain text unless noted
otherwise.

A section below that says "Missing" means that file could not be collected on
this robot, and says why -- that absence is itself a diagnostic.


## Contents

### report.html  <- start here
An offline summary of everything below: an overall verdict, the failures and
warnings grouped into recommended actions with the commands that address them,
and every captured command rendered with its measurements charted against the
ranges they are held to. Open it in any browser -- it needs no network access
and loads nothing external. Every number in it was parsed from the raw captures
in commands/, which it also embeds verbatim, so the report never says anything
the raw output does not.

### README.md
This file.

### bundle_info.txt
Robot identity at export time: model, serial number, batch, tool, the user who
ran the export, and HELLO_FLEET_PATH.

### commands/
Everything the export ran on the robot, one file per command, each starting
with the command that produced it. A command that failed still has a file
here, holding its error output and exit code. Re-running any of these on the
robot reproduces that file.

#### commands/stretch_system_check.txt

    $ stretch_system_check

Console output of a full system check. One [PASS] / [FAIL] / [SKIP] line per
subsystem plus a summary at the end; it also records the installed software
versions. Firmware shows as [SKIP] here because checking it requires stopping
the server -- see commands/stretch_system_check_updates.txt for the firmware
table.

#### commands/stretch_system_check_sensors.txt

    $ stretch_system_check --sensors

Lidar reachability and streaming, plus camera stream rates, resolutions and
USB link speeds.

#### commands/stretch_system_check_updates.txt

    $ stretch_system_check --check_updates

The installed hello-robot-* pip packages against the latest on PyPI, the git
status of the ROS 2 workspace repos, the firmware version of every board
against the version this software release expects, and the exact commands that
would apply each update. This was the last thing the export ran, because it is
the one capture that stops the robot server; the server was restarted
immediately afterwards, so a matching stop/start in stretch_body_server_logs/
around the export timestamp is expected, not a fault.

{commands_section}

{params_section}

{udev_section}

{teleop_section}

{status_section}

{logs_section}
"""


def export_diagnostics(export_dir):
    """Collects telemetry history, system check output, robot config and logs into one zip."""
    import shutil
    import tempfile
    import zipfile
    from datetime import datetime

    export_dir = os.path.expanduser(export_dir)
    if not os.path.isdir(export_dir):
        click.secho(f'\n[FAIL] Export directory {export_dir} does not exist.', fg='red')
        return False

    print_section('Diagnostics Export')
    staging = tempfile.mkdtemp(prefix='stretch_system_check_export_')
    passthrough = (['--verbose'] if args.verbose else []) + (['--direct'] if args.direct else [])
    fleet_dir = _fleet_dir()
    contents = {'fleet_dir': fleet_dir, 'status_zips': [], 'session_logs': [],
                'commands': [], 'params': [], 'udev': False, 'web_teleop': []}
    captures = {}   # file name in commands/ -> captured text, for the HTML report
    try:
        with open(os.path.join(staging, 'bundle_info.txt'), 'w') as f:
            f.write(_bundle_info())

        # 1. Telemetry history from stretch_status --export (writes its own zip into staging)
        _run_tool('stretch_status.py', ['--export', staging], 'stretch_status --export')
        contents['status_zips'] = sorted(f for f in os.listdir(staging)
                                         if f.startswith('stretch_status_') and f.endswith('.zip'))
        if not contents['status_zips']:
            print_warn('stretch_status --export produced no telemetry archive')

        # 2. The system check itself, run as a subprocess so this export cannot recurse
        command_dir = os.path.join(staging, 'commands')
        os.makedirs(command_dir, exist_ok=True)
        captures['stretch_system_check.txt'] = _run_tool(
            'stretch_system_check.py', passthrough, 'stretch_system_check')
        captures['stretch_system_check_sensors.txt'] = _run_tool(
            'stretch_system_check.py', ['--sensors'] + passthrough, 'stretch_system_check --sensors')
        for name in ('stretch_system_check.txt', 'stretch_system_check_sensors.txt'):
            with open(os.path.join(command_dir, name), 'w') as f:
                f.write(captures[name])

        # 3. Shell commands describing the robot's devices, environment and server state
        for name, cmd, timeout, _description in EXPORT_COMMANDS:
            captures[name] = _run_shell(cmd, cmd, timeout=timeout)
            try:
                with open(os.path.join(command_dir, name), 'w') as f:
                    f.write(captures[name])
                contents['commands'].append(name)
            except OSError as e:
                print_warn(f'Could not save output of `{cmd}`: {e}')

        # 4. The robot's parameter files
        if fleet_dir is None:
            print_warn('HELLO_FLEET_PATH/HELLO_FLEET_ID unset — skipping robot parameter files')
        else:
            click.secho('  Collecting robot parameter files...', fg='yellow')
            param_dir = os.path.join(staging, 'robot_params')
            for name in EXPORT_PARAM_FILES:
                src = os.path.join(fleet_dir, name)
                if not os.path.exists(src):
                    # stretch_calibration_values.yaml is absent on an uncalibrated robot
                    print_warn(f'{src} does not exist')
                    continue
                copied = _copy_into(src, param_dir, src)
                if copied:
                    contents['params'].append(copied)

        # 5. The udev rules that create the /dev/hello-* symlinks
        if not os.path.isdir(UDEV_RULES_DIR):
            print_warn(f'{UDEV_RULES_DIR} does not exist')
        else:
            click.secho(f'  Collecting {UDEV_RULES_DIR}...', fg='yellow')
            contents['udev'] = _copy_into(UDEV_RULES_DIR, staging, UDEV_RULES_DIR,
                                          name='udev_rules.d') is not None

        # 6. The most recent web teleop sessions
        teleop_sessions = _recent_web_teleop_sessions()
        if not teleop_sessions:
            print_warn('No web teleop session logs found')
        else:
            click.secho(f'  Collecting {len(teleop_sessions)} web teleop session logs...', fg='yellow')
            teleop_dir = os.path.join(staging, 'web_teleop_logs')
            for session in teleop_sessions:
                copied = _copy_into(session, teleop_dir, session)
                if copied:
                    contents['web_teleop'].append(copied)

        # 7. The most recent stretch_body_server session logs
        try:
            from stretch4_body.tools.stretch_body_server import get_recent_session_logs
            session_logs = get_recent_session_logs()
        except Exception as e:
            print_warn(f'Could not collect stretch_body_server logs: {e}')
            session_logs = []
        if not session_logs:
            print_warn('No stretch_body_server session logs found')
        else:
            click.secho(f'  Collecting {len(session_logs)} stretch_body_server session logs...', fg='yellow')
            log_dir = os.path.join(staging, 'stretch_body_server_logs')
            os.makedirs(log_dir, exist_ok=True)
            for log in session_logs:
                try:
                    shutil.copy2(log, log_dir)
                except OSError as e:
                    print_warn(f'Could not copy {log}: {e}')
            contents['session_logs'] = sorted(os.listdir(log_dir))

        # 8. The update check. It runs last because it stops the robot server to query
        #    firmware, which would otherwise disturb every capture above.
        print_warn('stretch_system_check --check_updates stops the robot server to query '
                   'firmware; it restarts in the background afterwards')
        captures['stretch_system_check_updates.txt'] = _run_tool(
            'stretch_system_check.py', ['--check_updates'] + passthrough,
            'stretch_system_check --check_updates')
        with open(os.path.join(command_dir, 'stretch_system_check_updates.txt'), 'w') as f:
            f.write(captures['stretch_system_check_updates.txt'])

        # 9. The README, and the HTML report built from everything captured above
        with open(os.path.join(staging, 'README.md'), 'w') as f:
            f.write(_bundle_readme(contents))
        click.secho('  Building report.html...', fg='yellow')
        try:
            with open(os.path.join(staging, 'report.html'), 'w') as f:
                f.write(_report_html(_build_report(contents, captures)))
        except Exception as e:
            print_warn(f'Could not build report.html: {e}')

        # 10. Zip the staging directory
        timestamp = datetime.now().strftime('%Y%m%d%H%M%S')
        zip_path = os.path.join(export_dir, f'stretch_system_check_{stretch_serial_no}_{timestamp}.zip')
        click.secho(f'  Writing {zip_path}...', fg='yellow')
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
            for root, _, files in os.walk(staging):
                for name in sorted(files):
                    full = os.path.join(root, name)
                    if os.path.islink(full) and not os.path.exists(full):
                        continue  # a broken symlink would raise when zipped
                    zf.write(full, os.path.relpath(full, staging))
    finally:
        shutil.rmtree(staging, ignore_errors=True)

    size_mb = os.path.getsize(zip_path) / (1024 * 1024)
    click.echo()
    click.secho(f'Export complete: {zip_path} ({size_mb:.2f} MB)', fg='green', bold=True)
    click.secho('Send this file to support@hello-robot.com when reporting an issue.\n', fg='bright_white')
    return True

# ==============================================================================
# Main
# ==============================================================================

_ALL_CHECKS = [
    'USB Devices', 'Firmware',
    'Power/Battery', 'ESP32', 'Line Sensors',
    'Eye LEDs', 'Calibrations', 'Audio',
    'OmniBase', 'Arm', 'Lift', 'End-of-Arm', 'IMU',
]
_REQUIRE_SERVER = {
    'Power/Battery', 'ESP32', 'OmniBase', 'Arm', 'Lift', 'End-of-Arm', 'IMU'
}


def main():
    global r
    results = {}

    if args.export is not None:
        sys.exit(0 if export_diagnostics(args.export) else 1)

    if args.repos:
        ok = check_repos()
        click.echo()
        sys.exit(0 if ok else 1)

    if args.check_updates:
        ok = check_updates()
        sys.exit(0 if ok else 1)

    if args.firmware:
        click.secho('\n---- Firmware Check Mode ----', fg='cyan', bold=True)
        _kill_server()
        results['USB Devices'] = check_usb_devices()
        results['Firmware']    = check_firmware_versions()
        _restart_server()

        print_section('Summary')
        all_pass = True
        for name in ['USB Devices', 'Firmware']:
            passed = results.get(name)
            print_result(passed, name)
            if not passed:
                all_pass = False
        click.echo()
        click.secho('All firmware checks PASSED.' if all_pass else 'One or more firmware checks FAILED.',
                    fg='green' if all_pass else 'red', bold=True)
        sys.exit(0 if all_pass else 1)

    if args.sensors:
        results['Sensors'] = check_sensors()
        print_section('Summary')
        all_pass = results['Sensors']
        print_result(all_pass, 'Sensors')
        click.echo()
        click.secho('All sensor checks PASSED.' if all_pass else 'One or more sensor checks FAILED.',
                    fg='green' if all_pass else 'red', bold=True)
        sys.exit(0 if all_pass else 1)

    # Full system check
    print_software_versions()
    results['USB Devices'] = check_usb_devices()
    results['Firmware']    = None  # requires --firmware
    results['Calibrations'] = check_calibrations()

    if args.direct:
        from stretch4_body.robot.robot import Robot
        r = Robot()
    else:
        from stretch4_body.robot.robot_client import RobotClient
        r = RobotClient()

    _old_stdout, _old_stderr = sys.stdout, sys.stderr
    sys.stdout = sys.stderr = io.StringIO()
    server_online = r.startup()
    sys.stdout, sys.stderr = _old_stdout, _old_stderr

    if not server_online:
        click.secho(
            '\n[WARN] Could not connect to robot server — live hardware checks skipped.\n'
            '       Launch the server first:  stretch_body_server --launch\n',
            fg='yellow'
        )
        r = None
    else:
        r.pull_status()

    results['Line Sensors'] = check_line_sensors()
    results['Eye LEDs']     = check_eye_animations()
    results['Audio']        = check_audio()

    if r is not None:
        results['Power/Battery'] = check_power_periph()
        results['ESP32']         = check_esp32()
        results['OmniBase']      = check_omnibase()
        results['Arm']           = check_arm()
        results['Lift']          = check_lift()
        results['End-of-Arm']    = check_end_of_arm()
        results['IMU']           = check_imu()
    else:
        for name in _REQUIRE_SERVER:
            results[name] = None

    print_section('Summary')
    all_pass = True
    for name in _ALL_CHECKS:
        passed = results.get(name)
        if passed is None:
            skip_reason = _SKIP_REASONS.get(name) or (
                'run with --firmware to check' if name == 'Firmware' else 'server offline')
            click.secho(f'  [SKIP] {name} ({skip_reason})', fg='yellow')
        else:
            print_result(passed, name)
            if not passed:
                all_pass = False

    click.echo()
    click.secho('All checks PASSED.' if all_pass else 'One or more checks FAILED.',
                fg='green' if all_pass else 'red', bold=True)

    if r is not None:
        r.stop()
    sys.exit(0 if all_pass else 1)


# ==============================================================================
# HTML report
#
# The export writes a self-contained report.html next to the raw captures. The
# captured console output is parsed into structured results here, embedded as
# JSON, and rendered by the page's own JavaScript, so the report opens offline
# with no network access and no bundled libraries.
# ==============================================================================

_RE_SECTION   = re.compile(r'^-{4}\s(?P<title>.+?)\s-{4}$')
_RE_RESULT    = re.compile(r'^(?P<indent>\s*)\[(?P<status>PASS|FAIL|WARN|SKIP)\]\s(?P<message>.*)$')
_RE_RANGE     = re.compile(r'^(?P<label>.+?)\s=\s(?P<value>-?\d+(?:\.\d+)?)\s'
                           r'\(range\s\[(?P<min>-?\d+(?:\.\d+)?),\s*(?P<max>-?\d+(?:\.\d+)?)\]\)$')
_RE_TARGET    = re.compile(r'^(?P<label>.+?):\s(?P<value>-?\d+(?:\.\d+)?)\s+\(target\s(?P<target>-?\d+(?:\.\d+)?)\)$')
_RE_DEV_LINK  = re.compile(r'^l\S*\s.*\s/dev/(?P<name>\S+)\s->\s(?P<target>\S+)$')
_RE_LSUSB     = re.compile(r'^Bus\s(?P<bus>\d+)\sDevice\s(?P<device>\d+):\sID\s(?P<id>\S+)\s*(?P<name>.*)$')
_RE_PIP       = re.compile(r'^\s{2,}(?P<name>[A-Za-z0-9_.-]+)\s+:\s+(?P<current>\S+)'
                           r'(?:\s+→\s+(?P<latest>\S+))?\s*(?P<note>\(.*\))?\s*$')
_RE_SPINNER   = re.compile(r'^[\s' + ''.join(SPINNER_FRAMES) + r']*$')
_RE_SPINNER_PREFIX = re.compile(r'^(\s*)[' + ''.join(SPINNER_FRAMES) + r']\s*')
_RE_SERVICE   = re.compile(r'^\s*Active:\s*(?P<state>\S+)')

# A FAIL/WARN message matching one of these gets a concrete remedy in the
# Recommended actions list: (pattern, title, what to do, commands to run)
_ACTION_HINTS = [
    (re.compile(r'in use by the following process', re.I),
     'A camera is held by another process',
     'Another program has the Luxonis device open. Close the process named in the capture '
     '(or restart the robot), then re-run the sensor check.',
     ['stretch_system_check --sensors']),
    (re.compile(r'/dev/hello-\S+'),
     'A board did not enumerate on USB',
     'The udev symlink for this board is missing, so nothing can talk to it. Check the cable, '
     'then power cycle the robot. If it stays missing, compare udev_rules.d/ against the '
     'device listing.',
     ['ls -la /dev/hello*', 'stretch_system_check --check_updates']),
    (re.compile(r'could not connect to robot server|server offline', re.I),
     'The robot server was not running',
     'Live hardware checks were skipped because nothing could connect to the server.',
     ['stretch_body_server --restart', 'stretch_system_check']),
    (re.compile(r'calibration', re.I),
     'A calibration is missing or stale',
     'Re-run the calibration for the joint named in the capture before trusting its motion.',
     []),
    (re.compile(r'ptp', re.I),
     'Lidar time sync is not locked',
     'The lidars are free-running rather than PTP-locked, so their timestamps drift against '
     'the rest of the robot. Check the PTP service on the robot if point clouds look skewed.',
     []),
]


def _is_noise(line):
    """True for spinner frames and the banner lines that every tool prints."""
    stripped = line.strip()
    if not stripped or _RE_SPINNER.match(line):
        return True
    return stripped.startswith(('For use with S T R E T C H', '---------------------', '========'))


def _measurement(message):
    """Pulls a plottable measurement out of a result message, if there is one."""
    m = _RE_RANGE.match(message)
    if m:
        return {'kind': 'range', 'label': m.group('label'), 'value': float(m.group('value')),
                'min': float(m.group('min')), 'max': float(m.group('max'))}
    m = _RE_TARGET.match(message)
    if m:
        return {'kind': 'target', 'label': m.group('label'), 'value': float(m.group('value')),
                'target': float(m.group('target'))}
    return None


def _parse_check_output(text):
    """Parses captured stretch_system_check console output into sections of results.

    Indentation carries the grouping (camera -> FPS -> per-stream result), so the
    last plain line shallower than a result is kept as that result's group label.
    """
    sections = []
    current = {'title': 'Output', 'results': [], 'rollup': False}
    groups = []  # stack of (indent, label)

    for line in text.splitlines():
        if _is_noise(line):
            continue
        # A spinner frame prefixes the line it animates on; drop it but keep the text
        line = _RE_SPINNER_PREFIX.sub(r'\1', line)

        header = _RE_SECTION.match(line.strip())
        if header:
            if current['results']:
                sections.append(current)
            title = header.group('title')
            # The tool's own Summary block repeats the checks above, so it is kept for
            # display but marked as a recap: counting it would double every result.
            current = {'title': title, 'results': [], 'rollup': title.lower() == 'summary'}
            groups = []
            continue

        result = _RE_RESULT.match(line)
        if result:
            indent = len(result.group('indent'))
            # <= : a heading often sits at the same indent as the results under it
            # ('  Left lidar (...)' then '  [PASS] Ping reachable')
            label = ' › '.join(g[1] for g in groups if g[0] <= indent)
            message = result.group('message').strip()
            current['results'].append({
                'status': result.group('status'),
                'message': message,
                'group': label,
                'measurement': _measurement(message),
            })
            continue

        # A plain line is a group header for everything indented under it
        stripped = line.strip().rstrip(':')
        # '...' marks a transient progress line: it heads nothing, and must not
        # displace the real heading it animates under
        if stripped.endswith('...'):
            continue
        indent = len(line) - len(line.lstrip())
        while groups and groups[-1][0] >= indent:
            groups.pop()
        if stripped and len(stripped) < 80:
            groups.append((indent, stripped))

    if current['results']:
        sections.append(current)
    return sections


def _count_statuses(sections):
    """Totals per status. A recap section contributes only its SKIPs — the tool lists
    skipped checks nowhere else, while its other rows repeat the sections above."""
    counts = {'PASS': 0, 'FAIL': 0, 'WARN': 0, 'SKIP': 0}
    for section in sections:
        for result in section['results']:
            if section.get('rollup') and result['status'] != 'SKIP':
                continue
            counts[result['status']] = counts.get(result['status'], 0) + 1
    return counts


def _parse_dev_links(text):
    """Rows of (device, target) from `ls -la /dev/hello*`."""
    rows = []
    for line in text.splitlines():
        m = _RE_DEV_LINK.match(line.strip())
        if m:
            rows.append({'name': m.group('name'), 'target': m.group('target')})
    return rows


def _parse_lsusb(text):
    """Rows of USB devices from `lsusb -v`, skipping root hubs."""
    rows = []
    for line in text.splitlines():
        m = _RE_LSUSB.match(line.strip())
        if m and 'root hub' not in m.group('name'):
            rows.append({'bus': m.group('bus'), 'device': m.group('device'),
                         'id': m.group('id'), 'name': m.group('name').strip() or 'Unknown device'})
    return rows


def _parse_pip_rows(text):
    """Package rows from the pip section of `stretch_system_check --check_updates`."""
    rows = []
    in_section = False
    for line in text.splitlines():
        header = _RE_SECTION.match(line.strip())
        if header:
            in_section = header.group('title').startswith('Python / pip')
            continue
        if not in_section:
            continue
        m = _RE_PIP.match(line)
        if m and m.group('name').startswith('hello-robot-'):
            note = (m.group('note') or '').strip('()')
            rows.append({'name': m.group('name'), 'current': m.group('current'),
                         'latest': m.group('latest'), 'note': note})
    return rows


def _parse_firmware_table(text):
    """Rows of the recommended-firmware table printed by `--check_updates`."""
    rows = []
    for line in text.splitlines():
        parts = [p.strip() for p in line.split('|')]
        if len(parts) != 4 or parts[0] in ('DEVICE', '') or parts[0].startswith('-'):
            continue
        device, installed, recommended, action = parts
        rows.append({'device': device, 'installed': installed, 'recommended': recommended,
                     'action': action,
                     'status': 'PASS' if action.lower().startswith('at most recent') else 'WARN'})
    return rows


def _parse_commands_to_run(text):
    """The copy-paste commands from the 'Commands To Run' section of --check_updates."""
    prefixes = ('python3', 'pip', 'REx_', 'git', 'cd ', 'sudo', 'stretch_', 'colcon')
    commands, in_section = [], False
    for line in text.splitlines():
        header = _RE_SECTION.match(line.strip())
        if header:
            in_section = header.group('title') == 'Commands To Run'
            continue
        if in_section and line.strip().startswith(prefixes):
            commands.append(line.strip())
    return commands


def _parse_service_state(text):
    """The systemd Active: state from `stretch_body_server --status`, if present."""
    for line in text.splitlines():
        m = _RE_SERVICE.match(line)
        if m:
            return m.group('state')
    return None


# The captures the report renders, in the order they appear in the page:
#   (file name in commands/, card title, how to parse it)
REPORT_CAPTURES = [
    ('stretch_system_check.txt',         'System check',          'check'),
    ('stretch_system_check_sensors.txt', 'Sensors',               'check'),
    ('stretch_system_check_updates.txt', 'Updates & firmware',    'updates'),
    ('dev_hello_devices.txt',            'Robot boards (/dev)',   'devices'),
    ('lsusb_verbose.txt',                'USB bus',               'usb'),
    ('stretch_body_server_status.txt',   'Robot server',          'service'),
    ('fleet_dir_listing.txt',            'Fleet directory',       'listing'),
    ('repos_listing.txt',                '~/repos',               'listing'),
]

_SEVERITY_ORDER = {'critical': 0, 'warning': 1, 'info': 2, 'good': 3}


def _capture_command(raw):
    """The '$ cmd' line that every capture starts with."""
    first = raw.splitlines()[0] if raw else ''
    return first[2:].strip() if first.startswith('$ ') else ''


def _match_hint(messages):
    for pattern, title, detail, commands in _ACTION_HINTS:
        for message in messages:
            if pattern.search(message):
                return {'title': title, 'detail': detail, 'commands': list(commands)}
    return None


def _build_actions(captures):
    """Derives the Recommended actions list from the parsed captures."""
    actions = []

    for capture in captures:
        for section in capture['sections']:
            if section.get('rollup'):
                continue  # its rows repeat the sections above
            for status, severity in (('FAIL', 'critical'), ('WARN', 'warning')):
                hits = [r for r in section['results'] if r['status'] == status]
                if not hits:
                    continue
                # Without the group, two lidars' identical warnings read as duplicates
                messages = [f'{r["group"]} — {r["message"]}' if r['group'] else r['message']
                            for r in hits]
                hint = _match_hint(messages)
                count = len(hits)
                summary = (f'{count} check{"" if count == 1 else "s"} failed' if status == 'FAIL'
                           else f'{count} warning{"" if count == 1 else "s"}')
                actions.append({
                    'severity': severity,
                    'title': hint['title'] if hint else f'{section["title"]}: {summary}',
                    'detail': hint['detail'] if hint else (
                        ('These checks failed. ' if status == 'FAIL' else
                         'These checks passed with a warning. ') +
                        f'The full output is in commands/{capture["file"]}.'),
                    'where': f'{capture["title"]} › {section["title"]}',
                    'items': messages[:8],
                    'commands': (hint['commands'] if hint and hint['commands']
                                 else [capture['command']]),
                })

    # Pending software and firmware updates
    for capture in captures:
        if capture['kind'] == 'updates' and capture.get('update_commands'):
            outdated = [p['name'] for p in capture.get('packages', []) if p.get('latest')]
            actions.append({
                'severity': 'warning',
                'title': 'Software or firmware updates are available',
                'detail': 'The update check found newer versions. Run these in the order shown '
                          '— a newer stretch4_body may recommend newer firmware.',
                'where': capture['title'],
                'items': [f'{name} is out of date' for name in outdated[:8]],
                'commands': capture['update_commands'],
            })

    # Commands that did not complete, so the report says nothing about what they cover
    failed_captures = [c for c in captures if c.get('errors')]
    if failed_captures:
        actions.append({
            'severity': 'warning',
            'title': f'{len(failed_captures)} command'
                     f'{"" if len(failed_captures) == 1 else "s"} did not complete',
            'detail': 'These commands errored or timed out during the export, so this report '
                      'covers nothing they would have reported. Re-run them on the robot.',
            'where': ', '.join(c['title'] for c in failed_captures),
            'items': [f'{c["title"]}: {err}' for c in failed_captures for err in c['errors']][:8],
            'commands': [c['command'] for c in failed_captures if c['command']],
        })

    # Checks that never ran. The firmware check is the exception: the system check
    # skips it, but the bundle's --check_updates capture queries firmware itself, so
    # that table is already here and there is nothing for the reader to go run.
    covered = any(c.get('firmware') for c in captures)
    skipped = [(c, r['message']) for c in captures for s in c['sections']
               for r in s['results'] if r['status'] == 'SKIP'
               and not (covered and 'firmware' in r['message'].lower())]
    if skipped:
        actions.append({
            'severity': 'info',
            'title': f'{len(skipped)} check{"" if len(skipped) == 1 else "s"} did not run',
            'detail': 'These were skipped at export time, so this report says nothing about them '
                      'either way.' + (' The firmware check was skipped too, but its table is in '
                                       'the update check above.' if covered else ''),
            'where': skipped[0][0]['title'],
            'items': [message for _, message in skipped][:8],
            'commands': ['stretch_system_check'],
        })

    actions.sort(key=lambda a: _SEVERITY_ORDER.get(a['severity'], 9))

    if not any(a['severity'] in ('critical', 'warning') for a in actions):
        actions.insert(0, {
            'severity': 'good',
            'title': 'No action needed',
            'detail': 'Every check that ran passed. Nothing in this bundle points at a fault.',
            'where': '', 'items': [], 'commands': [],
        })
    return actions


def _build_report(contents, captures):
    """Turns the raw captures into the JSON the report page renders."""
    from datetime import datetime

    parsed = []
    for file_name, title, kind in REPORT_CAPTURES:
        raw = captures.get(file_name)
        if raw is None:
            continue
        capture = {
            'file': file_name,
            'title': title,
            'kind': kind,
            'command': _capture_command(raw),
            'sections': [],
            'raw': raw,
        }
        if kind in ('check', 'updates'):
            capture['sections'] = _parse_check_output(raw)
        if kind == 'updates':
            capture['packages'] = _parse_pip_rows(raw)
            capture['firmware'] = _parse_firmware_table(raw)
            capture['update_commands'] = _parse_commands_to_run(raw)
        elif kind == 'devices':
            capture['devices'] = _parse_dev_links(raw)
        elif kind == 'usb':
            capture['usb'] = _parse_lsusb(raw)
        elif kind == 'service':
            capture['service_state'] = _parse_service_state(raw)
        # A command that failed to run leaves no PASS/FAIL rows, so its own error
        # lines are what tell the reader this part of the report is blind
        capture['errors'] = [line.strip() for line in raw.splitlines()
                             if line.startswith('[!]')
                             and ('failed:' in line or 'timed out' in line)]
        capture['counts'] = _count_statuses(capture['sections'])
        capture['measurements'] = [
            dict(r['measurement'], status=r['status'], group=r['group'])
            for s in capture['sections'] for r in s['results'] if r['measurement']
        ]
        parsed.append(capture)

    totals = {'PASS': 0, 'FAIL': 0, 'WARN': 0, 'SKIP': 0}
    for capture in parsed:
        for status, count in capture['counts'].items():
            totals[status] += count

    blind = any(capture['errors'] for capture in parsed)
    verdict = ('critical' if totals['FAIL']
               else 'warning' if (totals['WARN'] or blind)
               else 'good')

    return {
        'robot': {
            'model': _model_display,
            'serial': stretch_serial_no,
            'batch': stretch_batch,
            'tool': TOOL_DISPLAY.get(stretch_tool, stretch_tool),
            'user': os.environ.get('USER', 'N/A'),
            'fleet_dir': contents.get('fleet_dir') or 'N/A',
        },
        'exported': datetime.now().isoformat(timespec='seconds'),
        'totals': totals,
        'verdict': verdict,
        'captures': parsed,
        'actions': _build_actions(parsed),
        'bundle': {
            'status_zips': contents.get('status_zips', []),
            'session_logs': contents.get('session_logs', []),
            'params': contents.get('params', []),
            'web_teleop': contents.get('web_teleop', []),
            'udev': bool(contents.get('udev')),
        },
    }


# Status palette, chart chrome and ink. Status hues are fixed in both modes and
# always ship with a glyph + label, so color never carries the meaning alone.
REPORT_STYLE = """
:root {
  color-scheme: light;
  --page: #f9f9f7;  --surface: #fcfcfb;
  --ink: #0b0b0b;   --ink-2: #52514e;  --muted: #898781;
  --grid: #e1e0d9;  --axis: #c3c2b7;   --border: rgba(11,11,11,0.10);
  --good-rgb: 12,163,12;    --warning-rgb: 250,178,25;
  --serious-rgb: 236,131,90; --critical-rgb: 208,59,59;
  --muted-rgb: 137,135,129;
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) {
    color-scheme: dark;
    --page: #0d0d0d;  --surface: #1a1a19;
    --ink: #ffffff;   --ink-2: #c3c2b7;  --muted: #898781;
    --grid: #2c2c2a;  --axis: #383835;   --border: rgba(255,255,255,0.10);
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --page: #0d0d0d;  --surface: #1a1a19;
  --ink: #ffffff;   --ink-2: #c3c2b7;  --muted: #898781;
  --grid: #2c2c2a;  --axis: #383835;   --border: rgba(255,255,255,0.10);
}

* { box-sizing: border-box; }
body {
  margin: 0; background: var(--page); color: var(--ink);
  font: 15px/1.55 ui-sans-serif, system-ui, -apple-system, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
}
.wrap { max-width: 1100px; margin: 0 auto; padding: 24px 16px 64px; }
h1 { font-size: 22px; margin: 0; letter-spacing: -0.01em; }
h2 { font-size: 17px; margin: 40px 0 12px; letter-spacing: -0.01em; }
h3 { font-size: 15px; margin: 0; }
a { color: inherit; }
code, pre, .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }

.topbar { display: flex; flex-wrap: wrap; gap: 12px; align-items: baseline; justify-content: space-between; }
.sub { color: var(--ink-2); font-size: 13px; margin: 6px 0 0; }
.sub b { font-weight: 600; color: var(--ink); }
button.ghost {
  font: inherit; font-size: 13px; color: var(--ink-2); background: var(--surface);
  border: 1px solid var(--border); border-radius: 999px; padding: 4px 12px; cursor: pointer;
}
button.ghost:hover { color: var(--ink); }

.card {
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 12px; padding: 16px; margin-bottom: 12px;
}
.card > header { display: flex; flex-wrap: wrap; gap: 8px 16px; align-items: baseline; justify-content: space-between; }
.cmd { font-size: 12.5px; color: var(--ink-2); margin: 4px 0 0; overflow-wrap: anywhere; }
.cmd::before { content: "$ "; color: var(--muted); }

/* Hero figure + stat tiles */
.hero { display: flex; flex-wrap: wrap; align-items: center; gap: 20px; }
.hero-figure { font-size: 52px; line-height: 1; font-weight: 650; letter-spacing: -0.03em; }
.hero-note { color: var(--ink-2); font-size: 14px; max-width: 46ch; }
.tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(128px, 1fr)); gap: 12px; margin-top: 16px; }
.tile { border: 1px solid var(--border); border-radius: 10px; padding: 10px 12px; }
.tile .label { font-size: 12px; color: var(--ink-2); }
.tile .value { font-size: 24px; font-weight: 600; letter-spacing: -0.02em; }

/* Part-to-whole bar */
.stack { display: flex; gap: 2px; height: 12px; margin: 14px 0 10px; }
.stack span { border-radius: 2px; min-width: 3px; }
.stack.labelled { height: 24px; }
.stack.labelled span {
  display: flex; align-items: center; justify-content: center;
  font-size: 12px; font-weight: 600; font-variant-numeric: tabular-nums;
}
.stack span:first-child { border-top-left-radius: 4px; border-bottom-left-radius: 4px; }
.stack span:last-child { border-top-right-radius: 4px; border-bottom-right-radius: 4px; }
.legend { display: flex; flex-wrap: wrap; gap: 6px 18px; font-size: 13px; color: var(--ink-2); }
.legend .dot { width: 9px; height: 9px; border-radius: 2px; display: inline-block; margin-right: 6px; }
.legend b { color: var(--ink); font-weight: 600; font-variant-numeric: tabular-nums; }

/* Status chips */
.chip {
  display: inline-flex; align-items: center; gap: 6px; font-size: 12px; font-weight: 600;
  padding: 1px 8px; border-radius: 999px; white-space: nowrap;
}
.chip .glyph { font-size: 10px; }

/* Meters */
.meters { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 14px 24px; margin-top: 14px; }
.meter .top { display: flex; justify-content: space-between; align-items: baseline; gap: 10px; font-size: 13px; }
.meter .name { color: var(--ink-2); overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
.meter .val { font-weight: 600; font-variant-numeric: tabular-nums; }
.meter .track {
  position: relative; height: 10px; border-radius: 4px; background: var(--grid); margin: 6px 0 3px;
}
.meter .band { position: absolute; top: 0; bottom: 0; border-radius: 3px; }
.meter .fill { position: absolute; top: 0; bottom: 0; left: 0; border-radius: 4px; }
.meter .mark { position: absolute; top: -3px; bottom: -3px; width: 3px; border-radius: 2px; }
.meter .tick { position: absolute; top: -3px; bottom: -3px; width: 2px; border-radius: 1px; background: var(--axis); }
.meter .scale { display: flex; justify-content: space-between; font-size: 11px; color: var(--muted); font-variant-numeric: tabular-nums; }

/* Results */
.filters { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; margin: 16px 0 4px; font-size: 13px; }
.filters .spacer { flex: 1; }
.section-title { font-size: 12px; text-transform: uppercase; letter-spacing: 0.06em; color: var(--muted); margin: 14px 0 6px; }
.rows { display: grid; gap: 3px; }
.row { display: flex; gap: 10px; align-items: baseline; font-size: 13.5px; padding: 2px 0; }
.row .group { color: var(--muted); font-size: 12px; }
.row .msg { overflow-wrap: anywhere; }
.row.hidden { display: none; }

table { border-collapse: collapse; width: 100%; font-size: 13px; margin-top: 12px; }
th, td { text-align: left; padding: 5px 10px 5px 0; border-bottom: 1px solid var(--grid); }
th { color: var(--muted); font-weight: 600; font-size: 11.5px; text-transform: uppercase; letter-spacing: 0.05em; }
td.mono { font-variant-numeric: tabular-nums; }

/* Actions */
.action { border-left: 3px solid var(--grid); padding: 2px 0 2px 14px; margin-bottom: 18px; }
.action h3 { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; }
.action p { margin: 4px 0 0; color: var(--ink-2); font-size: 13.5px; max-width: 76ch; }
.action ul { margin: 8px 0 0; padding-left: 18px; color: var(--ink-2); font-size: 13px; }
.action .where { color: var(--muted); font-size: 12px; }
pre.cmds {
  background: var(--page); border: 1px solid var(--border); border-radius: 8px;
  padding: 10px 12px; font-size: 12.5px; overflow-x: auto; margin: 10px 0 0;
}
details { margin-top: 14px; }
summary { cursor: pointer; font-size: 13px; color: var(--ink-2); }
details pre {
  background: var(--page); border: 1px solid var(--border); border-radius: 8px;
  padding: 12px; font-size: 12px; max-height: 460px; overflow: auto; white-space: pre-wrap; overflow-wrap: anywhere;
}
footer { margin-top: 40px; color: var(--muted); font-size: 12.5px; }
@media print { .noprint { display: none; } details { display: none; } }
"""

REPORT_SCRIPT = """
const DATA = /*__REPORT_DATA__*/ null;

const STATUS = {
  PASS: { label: 'Pass',    glyph: '\\u25CF', rgb: 'var(--good-rgb)' },
  FAIL: { label: 'Fail',    glyph: '\\u2715', rgb: 'var(--critical-rgb)' },
  WARN: { label: 'Warning', glyph: '\\u25B2', rgb: 'var(--warning-rgb)' },
  SKIP: { label: 'Skipped', glyph: '\\u2013', rgb: 'var(--muted-rgb)' },
};
const VERDICT = {
  good:     { rgb: 'var(--good-rgb)',     word: 'Healthy' },
  warning:  { rgb: 'var(--warning-rgb)',  word: 'Needs attention' },
  critical: { rgb: 'var(--critical-rgb)', word: 'Faults found' },
};
const ORDER = ['PASS', 'WARN', 'FAIL', 'SKIP'];
const solid = (rgb) => 'rgb(' + rgb + ')';
const tint  = (rgb, a) => 'rgba(' + rgb + ',' + a + ')';

function el(tag, cls, text) {
  const node = document.createElement(tag);
  if (cls) node.className = cls;
  if (text !== undefined && text !== null) node.textContent = text;
  return node;
}

function statusChip(status) {
  const meta = STATUS[status] || STATUS.SKIP;
  const chip = el('span', 'chip');
  chip.style.color = solid(meta.rgb);
  chip.style.background = tint(meta.rgb, 0.12);
  chip.appendChild(el('span', 'glyph', meta.glyph));
  chip.appendChild(el('span', null, meta.label));
  return chip;
}

/* Part-to-whole: one stacked bar + a legend that always names every segment. */
const INK_ON_FILL = { WARN: true };   // light fills take ink, the rest take white

function stackedBar(counts, labelled) {
  const total = ORDER.reduce((sum, key) => sum + (counts[key] || 0), 0);
  const frag = document.createDocumentFragment();
  if (!total) return frag;

  const bar = el('div', labelled ? 'stack labelled' : 'stack');
  ORDER.forEach((key) => {
    const value = counts[key] || 0;
    if (!value) return;
    const seg = el('span');
    seg.style.flex = value;
    seg.style.background = solid(STATUS[key].rgb);
    seg.title = value + ' ' + STATUS[key].label.toLowerCase() + ' of ' + total;
    if (labelled && value / total > 0.07) {
      seg.textContent = value;
      seg.style.color = INK_ON_FILL[key] ? '#0b0b0b' : '#ffffff';
    }
    bar.appendChild(seg);
  });
  frag.appendChild(bar);

  const legend = el('div', 'legend');
  ORDER.forEach((key) => {
    const value = counts[key] || 0;
    if (!value) return;
    const item = el('span');
    const dot = el('span', 'dot');
    dot.style.background = solid(STATUS[key].rgb);
    item.appendChild(dot);
    item.appendChild(el('b', null, String(value)));
    item.appendChild(document.createTextNode(' ' + STATUS[key].label.toLowerCase()));
    legend.appendChild(item);
  });
  frag.appendChild(legend);
  return frag;
}

const fmt = (n) => {
  if (Number.isInteger(n)) return String(n);
  const abs = Math.abs(n);
  const text = abs >= 100 ? n.toFixed(1) : abs >= 10 ? n.toFixed(2) : n.toFixed(3);
  return text.indexOf('.') < 0 ? text : text.replace(/0+$/, '').replace(/\\.$/, '');
};

/* Meter: a measured value against the range or target it is held to. */
function meter(m) {
  const rgb = STATUS[m.status] ? STATUS[m.status].rgb : STATUS.SKIP.rgb;
  const wrap = el('div', 'meter');

  const top = el('div', 'top');
  const name = el('span', 'name', m.group ? m.group + ' › ' + m.label : m.label);
  name.title = name.textContent;
  top.appendChild(name);
  const val = el('span', 'val', fmt(m.value));
  val.style.color = solid(rgb);
  top.appendChild(val);
  wrap.appendChild(top);

  const track = el('div', 'track');
  const scale = el('div', 'scale');

  if (m.kind === 'range') {
    const pad = Math.max((m.max - m.min) * 0.12, 1e-9);
    const lo = Math.min(m.min - pad, m.value), hi = Math.max(m.max + pad, m.value);
    const pct = (v) => ((v - lo) / (hi - lo)) * 100;
    const band = el('div', 'band');
    band.style.left = pct(m.min) + '%';
    band.style.width = (pct(m.max) - pct(m.min)) + '%';
    band.style.background = tint(rgb, 0.18);
    band.title = 'Acceptable range ' + fmt(m.min) + ' to ' + fmt(m.max);
    track.appendChild(band);
    const mark = el('div', 'mark');
    mark.style.left = 'calc(' + pct(m.value) + '% - 1.5px)';
    mark.style.background = solid(rgb);
    track.appendChild(mark);
    wrap.title = m.label + ' = ' + fmt(m.value) + ' (acceptable ' + fmt(m.min) + ' to ' + fmt(m.max) + ')';
    scale.appendChild(el('span', null, fmt(m.min)));
    scale.appendChild(el('span', null, fmt(m.max)));
  } else {
    const hi = Math.max(m.value, m.target) * 1.2 || 1;
    const fill = el('div', 'fill');
    fill.style.width = Math.min((m.value / hi) * 100, 100) + '%';
    fill.style.background = tint(rgb, 0.55);
    track.appendChild(fill);
    const tick = el('div', 'tick');
    tick.style.left = 'calc(' + (m.target / hi) * 100 + '% - 1px)';
    tick.title = 'Target ' + fmt(m.target);
    track.appendChild(tick);
    wrap.title = m.label + ' = ' + fmt(m.value) + ' (target ' + fmt(m.target) + ')';
    scale.appendChild(el('span', null, '0'));
    scale.appendChild(el('span', null, 'target ' + fmt(m.target)));
  }

  wrap.appendChild(track);
  wrap.appendChild(scale);
  return wrap;
}

function table(columns, rows, cells) {
  const t = el('table');
  const head = el('tr');
  columns.forEach((c) => head.appendChild(el('th', null, c)));
  t.appendChild(el('thead')).appendChild(head);
  const body = el('tbody');
  rows.forEach((row) => {
    const tr = el('tr');
    cells(row).forEach((cell) => {
      const td = el('td', typeof cell === 'string' ? 'mono' : null);
      if (cell instanceof Node) td.appendChild(cell); else td.textContent = cell;
      tr.appendChild(td);
    });
    body.appendChild(tr);
  });
  t.appendChild(body);
  return t;
}

function renderSummary(root) {
  const verdict = VERDICT[DATA.verdict] || VERDICT.good;
  const card = el('div', 'card');

  const hero = el('div', 'hero');
  const figure = el('div', 'hero-figure');
  const failures = DATA.totals.FAIL || 0;
  const warnings = DATA.totals.WARN || 0;
  figure.textContent = failures ? String(failures) : (warnings ? String(warnings) : 'OK');
  figure.style.color = solid(verdict.rgb);
  hero.appendChild(figure);

  const text = el('div');
  const head = el('h3');
  head.appendChild(statusChip(failures ? 'FAIL' : (warnings ? 'WARN' : 'PASS')));
  head.appendChild(el('span', null, verdict.word));
  text.appendChild(head);
  text.appendChild(el('p', 'hero-note', failures
    ? failures + ' check' + (failures === 1 ? '' : 's') + ' failed. Start with Recommended actions below.'
    : warnings
      ? warnings + ' check' + (warnings === 1 ? '' : 's') + ' raised a warning. Nothing failed outright.'
      : 'Every check that ran passed.'));
  hero.appendChild(text);
  card.appendChild(hero);

  card.appendChild(stackedBar(DATA.totals, true));

  const tiles = el('div', 'tiles');
  DATA.captures.forEach((capture) => {
    const total = ORDER.reduce((sum, key) => sum + (capture.counts[key] || 0), 0);
    if (!total) return;
    const tile = el('div', 'tile');
    tile.appendChild(el('div', 'label', capture.title));
    const value = el('div', 'value');
    const bad = (capture.counts.FAIL || 0), warn = (capture.counts.WARN || 0);
    value.textContent = bad ? bad + ' failed' : warn ? warn + ' warned' : total + ' passed';
    value.style.color = solid(bad ? STATUS.FAIL.rgb : warn ? STATUS.WARN.rgb : STATUS.PASS.rgb);
    tile.appendChild(value);
    tile.appendChild(stackedBar(capture.counts));
    tiles.appendChild(tile);
  });
  card.appendChild(tiles);
  root.appendChild(card);
}

function renderActions(root) {
  DATA.actions.forEach((action) => {
    const meta = STATUS[action.severity === 'critical' ? 'FAIL'
      : action.severity === 'warning' ? 'WARN'
      : action.severity === 'good' ? 'PASS' : 'SKIP'];
    const node = el('div', 'action');
    node.style.borderLeftColor = solid(meta.rgb);

    const title = el('h3');
    const dot = el('span', 'chip');
    dot.style.color = solid(meta.rgb);
    dot.style.background = tint(meta.rgb, 0.12);
    dot.appendChild(el('span', 'glyph', meta.glyph));
    dot.appendChild(el('span', null, action.severity === 'good' ? 'All clear' : meta.label));
    title.appendChild(dot);
    title.appendChild(el('span', null, action.title));
    if (action.where) title.appendChild(el('span', 'where', action.where));
    node.appendChild(title);

    if (action.detail) node.appendChild(el('p', null, action.detail));
    if (action.items && action.items.length) {
      const list = el('ul');
      action.items.forEach((item) => list.appendChild(el('li', null, item)));
      node.appendChild(list);
    }
    if (action.commands && action.commands.length) {
      node.appendChild(el('pre', 'cmds', action.commands.join('\\n')));
    }
    root.appendChild(node);
  });
}

function renderCapture(capture) {
  const card = el('div', 'card');
  const header = el('header');
  const left = el('div');
  left.appendChild(el('h3', null, capture.title));
  left.appendChild(el('p', 'cmd', capture.command || capture.file));
  header.appendChild(left);
  if (capture.service_state) {
    header.appendChild(statusChip(capture.service_state === 'active' ? 'PASS' : 'FAIL'));
  }
  card.appendChild(header);

  const total = ORDER.reduce((sum, key) => sum + (capture.counts[key] || 0), 0);
  if (total) card.appendChild(stackedBar(capture.counts));

  if (capture.measurements && capture.measurements.length) {
    const grid = el('div', 'meters');
    capture.measurements.forEach((m) => grid.appendChild(meter(m)));
    card.appendChild(grid);
  }

  capture.sections.forEach((section) => {
    if (!section.results.length) return;
    card.appendChild(el('div', 'section-title',
      section.rollup ? section.title + ' (recap of the checks above)' : section.title));
    const rows = el('div', 'rows');
    section.results.forEach((result) => {
      const row = el('div', 'row');
      row.dataset.status = result.status;
      row.appendChild(statusChip(result.status));
      const msg = el('span', 'msg', result.message);
      row.appendChild(msg);
      if (result.group) row.appendChild(el('span', 'group', result.group));
      rows.appendChild(row);
    });
    card.appendChild(rows);
  });

  if (capture.devices && capture.devices.length) {
    card.appendChild(table(['Board', 'Device'], capture.devices,
      (row) => ['/dev/' + row.name, row.target]));
  }
  if (capture.usb && capture.usb.length) {
    card.appendChild(el('p', 'sub', capture.usb.length + ' devices on the bus (root hubs excluded)'));
    card.appendChild(table(['Bus', 'Device', 'ID', 'Name'], capture.usb,
      (row) => [row.bus, row.device, row.id, row.name]));
  }
  if (capture.packages && capture.packages.length) {
    card.appendChild(el('div', 'section-title', 'Python packages'));
    card.appendChild(table(['Package', 'Installed', 'Latest', ''], capture.packages,
      (row) => [row.name, row.current, row.latest || '\u2014',
                statusChip(row.latest ? 'WARN' : 'PASS')]));
  }
  if (capture.firmware && capture.firmware.length) {
    card.appendChild(el('div', 'section-title', 'Firmware'));
    card.appendChild(table(['Board', 'Installed', 'Recommended', ''], capture.firmware,
      (row) => [row.device, row.installed, row.recommended, statusChip(row.status)]));
  }

  const details = el('details');
  details.appendChild(el('summary', null, 'Raw output — commands/' + capture.file));
  details.appendChild(el('pre', null, capture.raw));
  card.appendChild(details);
  return card;
}

function renderBundle(root) {
  const bundle = DATA.bundle;
  const lines = [];
  (bundle.status_zips || []).forEach((name) =>
    lines.push(name + ' — telemetry history; replay with: stretch_status --import ' + name));
  if ((bundle.session_logs || []).length)
    lines.push('stretch_body_server_logs/ — ' + bundle.session_logs.length + ' most recent server sessions');
  if ((bundle.params || []).length)
    lines.push('robot_params/ — ' + bundle.params.join(', '));
  if (bundle.udev) lines.push('udev_rules.d/ — copy of /etc/udev/rules.d');
  if ((bundle.web_teleop || []).length)
    lines.push('web_teleop_logs/ — ' + bundle.web_teleop.length + ' most recent web teleop sessions');
  lines.push('README.md — what every file in this bundle is, and how to read it');

  const card = el('div', 'card');
  card.appendChild(el('h3', null, 'Also in this bundle'));
  const list = el('ul');
  lines.forEach((line) => list.appendChild(el('li', null, line)));
  card.appendChild(list);
  root.appendChild(card);
}

function renderFilters(root, captureRoot) {
  const bar = el('div', 'filters noprint');
  const label = el('span', 'sub', 'Show');
  bar.appendChild(label);
  [['all', 'Everything'], ['attention', 'Failures & warnings']].forEach(([mode, text], i) => {
    const button = el('button', 'ghost', text);
    button.addEventListener('click', () => {
      bar.querySelectorAll('button').forEach((b) => (b.style.borderColor = 'var(--border)'));
      button.style.borderColor = solid(STATUS.FAIL.rgb);
      captureRoot.querySelectorAll('.row').forEach((row) => {
        const hide = mode === 'attention' && (row.dataset.status === 'PASS');
        row.classList.toggle('hidden', hide);
      });
    });
    if (i === 0) button.style.borderColor = solid(STATUS.FAIL.rgb);
    bar.appendChild(button);
  });
  root.appendChild(bar);
}

function init() {
  const robot = DATA.robot;
  document.getElementById('title').textContent = robot.model + ' diagnostics — ' + robot.serial;
  document.getElementById('meta').innerHTML = '';
  const meta = document.getElementById('meta');
  [['Exported', DATA.exported.replace('T', ' ')], ['Tool', robot.tool],
   ['Batch', robot.batch], ['User', robot.user]].forEach(([k, v], i) => {
    if (i) meta.appendChild(document.createTextNode('  ·  '));
    meta.appendChild(document.createTextNode(k + ' '));
    meta.appendChild(el('b', null, v));
  });

  renderSummary(document.getElementById('summary'));
  renderActions(document.getElementById('actions'));

  const captures = document.getElementById('captures');
  renderFilters(document.getElementById('filters'), captures);
  DATA.captures.forEach((capture) => captures.appendChild(renderCapture(capture)));
  renderBundle(document.getElementById('bundle'));

  const toggle = document.getElementById('theme');
  toggle.addEventListener('click', () => {
    const dark = document.documentElement.getAttribute('data-theme') === 'dark'
      || (!document.documentElement.hasAttribute('data-theme')
          && window.matchMedia('(prefers-color-scheme: dark)').matches);
    document.documentElement.setAttribute('data-theme', dark ? 'light' : 'dark');
  });
}

document.addEventListener('DOMContentLoaded', init);
"""

REPORT_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>__STYLE__</style>
</head>
<body>
<div class="wrap">
  <div class="topbar">
    <div>
      <h1 id="title">Stretch diagnostics</h1>
      <p class="sub" id="meta"></p>
    </div>
    <button class="ghost noprint" id="theme">Toggle theme</button>
  </div>

  <h2>Diagnostics summary</h2>
  <div id="summary"></div>

  <h2>Recommended actions</h2>
  <div id="actions"></div>

  <h2>Captured commands</h2>
  <div id="filters"></div>
  <div id="captures"></div>

  <h2>Bundle contents</h2>
  <div id="bundle"></div>

  <footer>
    Generated by <code>stretch_system_check --export</code> on the robot. Every number here was
    parsed from the raw output in <code>commands/</code>, which is included verbatim under each card.
    Send the whole bundle to support@hello-robot.com when reporting an issue.
  </footer>
</div>
<script>__SCRIPT__</script>
</body>
</html>
"""


def _report_html(data):
    """Renders the self-contained report page for the parsed capture data."""
    blob = json.dumps(data, ensure_ascii=False).replace('</', '<\\/')
    title = f'{data["robot"]["model"]} diagnostics — {data["robot"]["serial"]}'
    return (REPORT_TEMPLATE
            .replace('__TITLE__', title)
            .replace('__STYLE__', REPORT_STYLE)
            .replace('__SCRIPT__', REPORT_SCRIPT.replace('/*__REPORT_DATA__*/ null', blob)))


if __name__ == '__main__':
    main()
