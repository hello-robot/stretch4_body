#!/usr/bin/env python3
"""
Software update checks: hello-robot pip packages against PyPI, ROS 2 workspace repos
against their git remotes, and the robot's firmware against the GitHub release list.

The work is split so the network is touched rarely:

    fetch_remote()  network only. Latest PyPI versions, remote branch SHAs and released
                    firmware. stretch_body_server runs it every few hours, and caches it.
    evaluate()      local only. Compares a fetch (fresh or cached) with what's installed
                    right now, so a cached fetch never reports an update you've already
                    installed.

Available updates are INFO, never a problem with the robot. Nothing here installs anything.
"""
import contextlib
import io
import json
import os
import re
import subprocess
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from importlib.metadata import version as pkg_version

from stretch4_body.behavior.health import health_checks as hc

GROUP_UPDATES = 'updates'

# ==============================================================================
# pip packages vs PyPI
# ==============================================================================

PYPI_TIMEOUT_S = 3.0
INTERNET_PROBE_URL = 'https://pypi.org/simple/'

CORE_PIP = ('hello-robot-stretch4-body', 'hello-robot-stretch4-urdf')

# Command shown to the user for applying pip updates (matches README)
PIP_UPDATE_CMD = 'python3 -m pip install -U'


def internet_available(timeout=PYPI_TIMEOUT_S):
    """One cheap request to PyPI. Every check here needs the internet, so skip them all without it."""
    try:
        req = urllib.request.Request(INTERNET_PROBE_URL, method='HEAD')
        with urllib.request.urlopen(req, timeout=timeout):
            return True
    except Exception:
        return False


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


def _checkable(installed):
    return [n for n in installed if n.lower().startswith('hello-robot-')
            and installed[n] not in (None, '', 'unknown')]


def fetch_pypi_latest(installed):
    """{name: latest version or None} for the hello-robot-* packages in `installed`, queried concurrently."""
    names = _checkable(installed)
    if not names:
        return {}
    try:
        with ThreadPoolExecutor(max_workers=min(8, len(names))) as pool:
            return dict(zip(names, pool.map(_pypi_latest_version, names)))
    except Exception:
        return {}


def pip_updates(installed, latest):
    """{name: latest} for installed packages with a newer release in `latest`."""
    return {n: latest[n] for n in _checkable(installed)
            if latest.get(n) and _is_newer(latest[n], installed[n])}


def check_pypi_updates(installed):
    """
    Query PyPI for newer releases of the hello-robot-* packages in `installed`
    ({name: version}). Fails silently (offline robot).

    Returns (updates, reachable) where updates is {name: latest_version} for
    packages with a newer release, and reachable is False if no query succeeded.
    """
    if not _checkable(installed):
        return {}, True
    latest = fetch_pypi_latest(installed)
    return pip_updates(installed, latest), any(v is not None for v in latest.values())


# ==============================================================================
# ROS2 workspace (~/ament_ws/src) repos vs their git remotes
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
GIT_NOT_CHECKED = 'remote not checked yet'
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


def workspace_repos(src=None):
    """(src_dir, [repo paths]). The list is empty if the workspace directory doesn't exist."""
    src = src or ament_src_dir()
    if not os.path.isdir(src):
        return src, []
    return src, sorted(os.path.join(src, d) for d in os.listdir(src)
                       if os.path.isdir(os.path.join(src, d)))


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


def _upstream(repo):
    """'remote/branch' the checked-out branch tracks, or None."""
    ok, upstream = _git(repo, 'rev-parse', '--abbrev-ref', '--symbolic-full-name', '@{u}')
    return upstream if ok and '/' in upstream else None


def fetch_repo_remote(repo):
    """
    Network half of a repo check: {'upstream', 'sha'} for the branch the repo tracks.
    sha is None if the remote couldn't be reached, '' if it has no such branch.
    Returns None for a repo with no upstream branch. Read-only: `git ls-remote`, never a fetch.
    """
    upstream = _upstream(repo)
    if upstream is None:
        return None
    remote, remote_branch = upstream.split('/', 1)
    ok, ls = _git(repo, 'ls-remote', '--heads', remote, remote_branch)
    if not ok:
        return {'upstream': upstream, 'sha': None}
    return {'upstream': upstream, 'sha': ls.split()[0] if ls else ''}


def fetch_repo_remotes(repos):
    """{repo path: fetch_repo_remote()} for every repo with an upstream, queried concurrently."""
    if not repos:
        return {}
    try:
        with ThreadPoolExecutor(max_workers=min(8, len(repos))) as pool:
            found = dict(zip(repos, pool.map(fetch_repo_remote, repos)))
    except Exception:
        found = {p: fetch_repo_remote(p) for p in repos}
    return {p: v for p, v in found.items() if v is not None}


def check_repo_git_status(repo, remote=False):
    """
    Report the git state of a single workspace repo, and whether the remote has
    a newer commit than the local checkout.

    `remote` is this repo's entry from fetch_repo_remotes(): pass it to judge the repo
    against an earlier fetch without touching the network (None: never fetched). Leave it
    out to query the remote now.

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

    upstream = _upstream(repo)
    if upstream is None:
        out['status'] = GIT_NO_UPSTREAM
        return out

    if remote is False:
        remote = fetch_repo_remote(repo)
    if not remote or remote.get('upstream') != upstream:
        # Never fetched, or the checkout now tracks a different branch than the fetch did
        out['status'] = GIT_NOT_CHECKED
        return out
    remote_sha = remote.get('sha')
    if remote_sha is None:
        out['status'] = GIT_UNREACHABLE
        out['detail'] = f'could not query {upstream.split("/", 1)[0]}'
        return out
    if not remote_sha:
        # The remote answered but has no such branch (deleted upstream, or never pushed)
        out['status'] = GIT_NO_BRANCH
        out['detail'] = f'{upstream} no longer exists'
        return out

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


def check_workspace_repos(remotes=None):
    """
    Check every repo in the ROS2 workspace src directory, concurrently.

    `remotes` is a fetch_repo_remotes() result to judge against without the network;
    None queries every remote now.

    Returns (src_dir, [status dicts sorted by name]). The list is empty if the
    workspace directory doesn't exist.
    """
    src, repos = workspace_repos()
    if not repos:
        return src, []
    if remotes is None:
        check = check_repo_git_status
    else:
        check = lambda p: check_repo_git_status(p, remote=remotes.get(p))
    try:
        with ThreadPoolExecutor(max_workers=min(8, len(repos))) as pool:
            results = list(pool.map(check, repos))
    except Exception:
        results = [check(p) for p in repos]
    return src, sorted(results, key=lambda x: x['name'].lower())


# ==============================================================================
# Firmware vs the GitHub release list
# ==============================================================================

FIRMWARE_TIMEOUT_S = 10.0

# REx_firmware_updater flag for each device
FIRMWARE_INSTALL_ARGS = {'hello-power-periph': '--pimu', 'hello-motor-omni-0': '--wheel_0',
                         'hello-motor-omni-1': '--wheel_1', 'hello-motor-omni-2': '--wheel_2',
                         'hello-motor-lift': '--lift', 'hello-motor-arm': '--arm'}

FW_UPGRADE = 'Upgrade recommended'
FW_DOWNGRADE = 'Downgrade recommended'
FW_CURRENT = 'At most recent version'
FW_UNKNOWN = 'None (might be on dev branch)'


def firmware_boards(robot):
    """
    {device: {'version', 'supported_protocols'}} for the boards a running Robot is talking to.
    The server reads these at startup, so firmware can be checked without stopping it to
    open the USB devices. The ESP32 and line sensor board can't report a version.
    """
    devices = []
    for joint in ('arm', 'lift'):
        motor = getattr(getattr(robot, joint, None), 'motor', None)
        if motor is not None:
            devices.append((motor.name, motor))
    for wheel in getattr(getattr(robot, 'omnibase', None), 'wheels', []) or []:
        devices.append((wheel.name, wheel))
    pp = getattr(robot, 'power_periph', None)
    if pp is not None:
        devices.append(('hello-power-periph', pp))

    boards = {}
    for name, dev in devices:
        info = getattr(dev, 'board_info', None) or {}
        if info.get('firmware_version'):
            boards[name] = {'version': str(info['firmware_version']),
                            'supported_protocols': sorted(getattr(dev, '_supported_protocols', {}) or {})}
    return boards


def fetch_firmware_available(devices, timeout=FIRMWARE_TIMEOUT_S):
    """{device: [released version strings]} from GitHub. Empty lists if it couldn't be reached."""
    from stretch4_body.core.factory.firmware_available import FirmwareAvailable
    with contextlib.redirect_stdout(io.StringIO()):  # It narrates its progress
        available = FirmwareAvailable({d: True for d in devices}, timeout=timeout)
    return {d: [str(v) for v in available.versions.get(d, [])] for d in devices}


def firmware_recommendations(boards, available):
    """
    {device: {'installed', 'recommended', 'action'}}: the newest release that the installed
    stretch4_body supports, the same choice `REx_firmware_updater --recommended` makes.
    """
    from stretch4_body.core.factory.firmware_available import FirmwareAvailable
    from stretch4_body.core.factory.firmware_version import FirmwareVersion
    releases = FirmwareAvailable.__new__(FirmwareAvailable)  # The release list, without fetching it
    releases.versions = {d: [FirmwareVersion(v) for v in vs] for d, vs in (available or {}).items()}
    out = {}
    for dev, board in boards.items():
        installed = FirmwareVersion(board['version'])
        recommended = None
        if releases.versions.get(dev):
            recommended = releases.get_most_recent_version(dev, board['supported_protocols'] or None)
        if recommended is None or not installed.valid:
            action = FW_UNKNOWN
        elif recommended > installed:
            action = FW_UPGRADE
        elif recommended < installed:
            action = FW_DOWNGRADE
        else:
            action = FW_CURRENT
        out[dev] = {'installed': board['version'], 'action': action,
                    'recommended': str(recommended) if recommended is not None else None}
    return out


def firmware_command(recommendations):
    """The REx_firmware_updater line that installs every recommended upgrade, or None."""
    args = [FIRMWARE_INSTALL_ARGS[d] for d, rec in recommendations.items()
            if rec['action'] == FW_UPGRADE and d in FIRMWARE_INSTALL_ARGS]
    return 'REx_firmware_updater --install ' + ' '.join(args) if args else None


def firmware_table(recommendations):
    """The recommendation as text, in the layout REx_firmware_updater --recommended prints."""
    lines = [f'{"DEVICE":<25} | {"INSTALLED":<25} | {"RECOMMENDED":<25} | ACTION', '-' * 100]
    for dev in sorted(recommendations):
        rec = recommendations[dev]
        lines.append(f'{dev.upper():<25} | {rec["installed"]:<25} | '
                     f'{(rec["recommended"] or "—"):<25} | {rec["action"]}')
    return '\n'.join(lines)


# ==============================================================================
# Fetch, evaluate, results
# ==============================================================================

def fetch_remote(boards):
    """
    Network half of every check. Returns a JSON-able dict, stored as the cache:
        {'ts_check', 'online', 'pypi_latest', 'repo_remotes', 'firmware_available'}
    """
    core, extras = discover_pip_packages()
    _, repos = workspace_repos()
    return {
        'ts_check': time.time(),
        'online': True,
        'pypi_latest': fetch_pypi_latest({**core, **extras}),
        'repo_remotes': fetch_repo_remotes(repos),
        'firmware_available': fetch_firmware_available(list(boards)) if boards else {},
    }


def evaluate(remote, boards):
    """
    Local half: judge a fetch_remote() result against what's installed now.
    Returns {'pip': {name: (installed, latest)}, 'pypi_reachable', 'repos': [...],
             'firmware': {device: recommendation}, 'firmware_reachable', 'firmware_command'}.
    """
    core, extras = discover_pip_packages()
    installed = {**core, **extras}
    latest = remote.get('pypi_latest') or {}
    _, repos = check_workspace_repos(remotes=remote.get('repo_remotes') or {})
    available = remote.get('firmware_available') or {}
    firmware = firmware_recommendations(boards, available) if boards else {}
    return {
        'pip': {n: (installed[n], v) for n, v in pip_updates(installed, latest).items()},
        'pypi_reachable': any(v is not None for v in latest.values()),
        'repos': repos,
        'firmware': firmware,
        'firmware_reachable': any(available.values()),
        'firmware_command': firmware_command(firmware),
    }


def update_results(ev):
    """Health results for an evaluate() result. Updates are INFO: worth knowing, not a fault."""
    g = GROUP_UPDATES
    out = []

    def available(id, item, msg, detail=None):
        r = hc.result(g, id, hc.INFO, msg, detail=detail)
        r['summary_group'], r['summary_item'] = 'Updates available', item
        out.append(r)

    if ev['pypi_reachable']:
        for name, (installed, latest) in sorted(ev['pip'].items()):
            available(f'pip_{name}', name, f'{name} {installed} → {latest}',
                      detail=[f'Update with: {PIP_UPDATE_CMD} {name}'])
        if not ev['pip']:
            out.append(hc.result(g, 'pip', hc.OK, 'hello-robot pip packages are up to date'))

    checked = [x for x in ev['repos'] if x['status'] not in (GIT_NOT_CHECKED, GIT_UNREACHABLE, GIT_NOT_A_REPO)]
    for x in ev['repos']:
        if x['status'] in GIT_NEEDS_PULL:
            available(f'repo_{x["name"]}', x['name'], f'{x["name"]}: {x["detail"] or x["status"]}',
                      detail=[f'Update with: {x["command"]}'] if x['command'] else None)
    if checked and not any(x['status'] in GIT_NEEDS_PULL for x in checked):
        out.append(hc.result(g, 'repos', hc.OK, 'ROS2 workspace repos are up to date'))

    if ev['firmware_reachable']:
        upgrades = {d: rec for d, rec in ev['firmware'].items() if rec['action'] == FW_UPGRADE}
        for dev, rec in sorted(upgrades.items()):
            available(f'firmware_{dev}', f'{dev} firmware',
                      f'{dev} firmware {rec["installed"]} → {rec["recommended"]}',
                      detail=[f'Update with: {ev["firmware_command"]}'])
        if not upgrades:
            out.append(hc.result(g, 'firmware', hc.OK, 'Firmware is at the recommended versions'))
    return out


# ==============================================================================
# Schedule and cache
# ==============================================================================

def next_check_time(ts_start, cache, params):
    """
    When the next network check is due. Never sooner than update_check_initial_delay_s
    after the server starts, so it adds no load while the robot comes up. After that,
    update_check_period_s after the last successful check, or update_check_retry_s after
    an attempt that found no internet.
    """
    earliest = ts_start + params['update_check_initial_delay_s']
    ts_check = cache.get('ts_check', 0.0) if cache else 0.0
    ts_attempt = cache.get('ts_attempt', 0.0) if cache else 0.0
    if ts_attempt > ts_check:
        return max(earliest, ts_attempt + params['update_check_retry_s'])
    return max(earliest, ts_check + params['update_check_period_s'])


def cache_path():
    root = os.environ.get('XDG_CACHE_HOME') or os.path.expanduser('~/.cache')
    return os.path.join(root, 'stretch4_body', 'software_updates.json')


def load_cache(path=None):
    try:
        with open(path or cache_path()) as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def save_cache(data, path=None):
    path = path or cache_path()
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f)
        os.replace(tmp, path)  # Never leave a half-written cache behind
        return True
    except Exception:
        return False
