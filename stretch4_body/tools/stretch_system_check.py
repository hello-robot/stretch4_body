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
"""
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

import os
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
    for label, fname in {
        'intrinsics: center': 'calibration_ros_camera_info_center.yaml',
        'intrinsics: left':   'calibration_ros_camera_info_left.yaml',
        'intrinsics: right':  'calibration_ros_camera_info_right.yaml',
        'extrinsics':         'camera_extrinsics.yaml',
    }.items():
        ok = os.path.isfile(os.path.join(cam_dir, fname))
        print_result(ok, label, indent=6)
        if not ok:
            all_pass = False

    click.secho('    Line Sensors:', fg='white', bold=True)
    ls_dir = os.path.join(cal_root, 'calibration_line_sensors')
    if os.path.isdir(ls_dir):
        for i in range(6):
            sensor_dir = os.path.join(ls_dir, f'sensor_{i}')
            has_cal = (os.path.isdir(sensor_dir) and
                       any(f.endswith('.yaml') for _, _, files in os.walk(sensor_dir) for f in files))
            print_result(has_cal, f'sensor_{i}', indent=6)
            if not has_cal:
                all_pass = False
    else:
        print_result(False, f'calibration directory missing: {ls_dir}', indent=6)
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
    Call a pyhesai_wrapper.ptc_client helper, suppressing SDK stdout/stderr.
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


def _check_lidar_ptc(ip, side, all_pass):
    """Run the full PTC config check for a single lidar. Returns updated all_pass."""
    from pyhesai_wrapper.ptc_client import (
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
            all_pass = False

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
            all_pass = False

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
            all_pass = False

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
            all_pass = False

    return all_pass


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


def check_sensors():
    """Check Hesai lidars (PTC config + streaming) and OAK cameras (via DepthAI)."""
    import socket, time
    print_section('Sensors')
    all_pass = True

    LIDARS = {
        'left':  {'ip': '192.168.1.202', 'ptc_port': 9347},
        'right': {'ip': '192.168.1.201', 'ptc_port': 9347},
    }
    try:
        import yaml, importlib.util
        spec = importlib.util.find_spec('pyhesai_wrapper')
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
        from pyhesai_wrapper.ptc_client import get_return_mode as _test_import  # noqa
        ptc_available = True
    except ImportError:
        print_warn('pyhesai_wrapper.ptc_client not available — PTC config checks skipped')

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
            all_pass = _check_lidar_ptc(ip, side, all_pass)

    click.secho(f'\n  Streaming check (listening 5 s for UDP packets)...', fg='white')
    stream_results = _check_lidar_streaming(LIDARS, timeout=5.0)
    for side, streaming in stream_results.items():
        ip = LIDARS[side]['ip']
        print_result(streaming, f'{side.capitalize()} lidar ({ip}): streaming UDP data')
        if not streaming:
            all_pass = False

    print_section('Cameras')

    HEAD_STREAMS = [
        {'name': 'Center', 'socket_name': 'CAM_A', 'width': 4032, 'height': 3040, 'fps': 5},
        {'name': 'Left',   'socket_name': 'CAM_C', 'width': 1280, 'height':  800, 'fps': 12},
        {'name': 'Right',  'socket_name': 'CAM_B', 'width': 1280, 'height':  800, 'fps': 12},
    ]
    GRIPPER_STREAMS = [
        {'name': 'Left',  'socket_name': 'CAM_C', 'width': 1280, 'height': 800, 'fps': 12},
        {'name': 'Right', 'socket_name': 'CAM_B', 'width': 1280, 'height': 800, 'fps': 12},
    ]
    CAMERA_ROLES = [
        {'label': 'Head camera (OAK-FFC-3P)',   'n_sensors': 3, 'streams': HEAD_STREAMS},
        {'label': 'Gripper camera (OAK-D-SR)',   'n_sensors': 2, 'streams': GRIPPER_STREAMS},
    ]
    FPS_TOLERANCE    = 0.50
    FPS_MEASURE_SECS = 3.0

    SPINNER_FRAMES = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏']

    def _spin(label, stop_event):
        """Animate a braille spinner on the label line until stop_event is set."""
        import itertools
        for frame in itertools.cycle(SPINNER_FRAMES):
            if stop_event.is_set():
                break
            sys.stdout.write(f'\r  {frame} {label}  ')
            sys.stdout.flush()
            time.sleep(0.08)
        # Clear the spinner line so the caller can print cleanly
        sys.stdout.write(f'\r{" " * (len(label) + 8)}\r')
        sys.stdout.flush()

    try:
        import depthai as dai
    except ImportError:
        print_warn('depthai not installed — cannot check OAK cameras')
        return all_pass

    for role in CAMERA_ROLES:
        label    = role['label']
        n_expect = role['n_sensors']
        streams  = role['streams']

        import threading
        stop_spin = threading.Event()
        spin_thread = threading.Thread(target=_spin, args=(label, stop_spin), daemon=True)
        spin_thread.start()

        # Re-enumerate fresh — stale DeviceInfo becomes invalid after the previous
        # camera's pipeline closes, causing the next probe to fail.
        try:
            device_infos = dai.Device.getAllAvailableDevices()
        except Exception as e:
            stop_spin.set()
            spin_thread.join()
            print_warn(f'DepthAI enumeration failed: {e}')
            all_pass = False
            continue

        # Find device with matching sensor count
        target_info = None
        for info in device_infos:
            try:
                tmp = dai.Device(info)
                n = len(tmp.getConnectedCameras())
                tmp.close()
                if n == n_expect:
                    target_info = info
                    break
            except Exception:
                pass

        if target_info is None:
            stop_spin.set()
            spin_thread.join()
            print_result(False, f'No OAK device with {n_expect} sensors found')
            all_pass = False
            continue

        try:
            device = dai.Device(target_info)
        except Exception as e:
            stop_spin.set()
            spin_thread.join()
            print_result(False, f'Could not open device: {e}')
            all_pass = False
            continue

        try:
            mxid  = device.getDeviceId()
            speed = device.getUsbSpeed()
            connected = device.getConnectedCameras()
            n_actual  = len(connected)

            pipeline = dai.Pipeline(defaultDevice=device)
            queues = {}
            socket_map = dai.CameraBoardSocket.__members__

            for cfg in streams:
                sock = socket_map.get(cfg['socket_name'])
                if sock is None or sock not in connected:
                    continue
                cam_node = pipeline.create(dai.node.Camera)
                cam_node.setSensorType(dai.CameraSensorType.COLOR)
                cam_node.build(boardSocket=sock, sensorFps=cfg['fps'])
                cam_out = cam_node.requestOutput(
                    size=(cfg['width'], cfg['height']),
                    fps=cfg['fps'],
                    type=dai.ImgFrame.Type.NV12,
                    resizeMode=dai.ImgResizeMode.CROP,
                    enableUndistortion=False,
                )
                queues[cfg['name']] = cam_out.createOutputQueue(maxSize=4, blocking=False)

            stop_spin.set()
            spin_thread.join()
            click.secho(f'  {label}', fg='cyan', bold=True)

            ok = n_actual == n_expect
            print_result(ok, f'Sensors detected: {n_actual}  (id={mxid})')
            if not ok:
                all_pass = False

            speed_ok = speed == dai.UsbSpeed.SUPER
            if speed_ok:
                print_result(True, f'USB speed: {speed.name}')
            else:
                print_warn(f'USB speed: {speed.name}  (expected SUPER / USB 3 — check cable)')

            # Suppress SDK calibration warnings (C++ threads emit during start + warmup)
            print_info('Starting pipeline...')
            _devnull_fd = os.open(os.devnull, os.O_WRONLY)
            _saved_stderr_fd = os.dup(2)
            os.dup2(_devnull_fd, 2)
            os.close(_devnull_fd)
            try:
                pipeline.start()
                print_info('Warming up streams (2 s)...')
                warmup_end = time.time() + 2.0
                while time.time() < warmup_end:
                    for q in queues.values():
                        if q.has():
                            q.get()
                    time.sleep(0.01)
            finally:
                os.dup2(_saved_stderr_fd, 2)
                os.close(_saved_stderr_fd)

            print_info('Checking stream activity...')
            frames_seen = {name: False for name in queues}
            deadline = time.time() + 2.0
            while time.time() < deadline:
                for name, q in queues.items():
                    if q.has():
                        q.get()
                        frames_seen[name] = True
                if all(frames_seen.values()):
                    break
                time.sleep(0.01)

            for name, seen in frames_seen.items():
                print_result(seen, f'Stream active: {name}')
                if not seen:
                    all_pass = False

            print_info(f'Measuring FPS ({FPS_MEASURE_SECS:.0f} s)...')
            counters = {name: 0 for name in queues}
            for q in queues.values():
                while q.has():
                    q.get()
            t0 = time.time()
            while time.time() - t0 < FPS_MEASURE_SECS:
                for name, q in queues.items():
                    while q.has():
                        q.get()
                        counters[name] += 1
                time.sleep(0.001)
            elapsed = time.time() - t0

            for cfg in streams:
                name = cfg['name']
                if name not in queues:
                    continue
                actual_fps = counters[name] / elapsed
                target_fps = cfg['fps']
                min_fps    = target_fps * (1 - FPS_TOLERANCE)
                fps_ok     = actual_fps >= min_fps
                print_result(fps_ok, f'FPS {name}: {actual_fps:.1f}  (target {target_fps})')
                if not fps_ok:
                    all_pass = False

            print_info('Capturing frames for resolution check...')
            captured = {}
            deadline = time.time() + 2.0
            while time.time() < deadline and len(captured) < len(queues):
                for name, q in queues.items():
                    if name not in captured and q.has():
                        captured[name] = q.get()
                time.sleep(0.01)

            for cfg in streams:
                name = cfg['name']
                if name not in queues:
                    continue
                if name not in captured:
                    print_result(False, f'Resolution {name}: no frame captured')
                    all_pass = False
                    continue
                frame = captured[name]
                w, h  = frame.getWidth(), frame.getHeight()
                res_ok = (w == cfg['width'] and h == cfg['height'])
                print_result(res_ok, f'Resolution {name}: {w}×{h}  (expected {cfg["width"]}×{cfg["height"]})')
                if not res_ok:
                    all_pass = False

            pipeline.stop()

        except Exception as e:
            stop_spin.set()
            spin_thread.join()
            print_result(False, f'Camera check error: {e}')
            all_pass = False
        finally:
            try:
                device.close()
            except Exception:
                pass

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


if __name__ == '__main__':
    main()
