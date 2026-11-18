#!/usr/bin/env python3
"""Sleep, reboot or power off the robot, as the power button does.

Stops the Stretch Body Server, and any other program holding the power board,
then talks to the power board directly, so neither can get in the way.
"""
import os
import pwd
import sys
import time
import signal
import subprocess
import argparse
import click
import stretch4_body.core.hello_utils as hu

hu.print_stretch_re_use()

# option: (power_periph method, what it does)
ACTIONS = {
    'sleep': ('trigger_robot_sleep',
              'This computer shuts down and the robot goes to sleep. The power button\n'
              'pulses yellow, then red. Tap the power button to wake the robot.'),
    'reboot': ('trigger_robot_reboot',
               'This computer shuts down, the robot sleeps for a moment, then it wakes by\n'
               'itself and this computer starts again. The power button pulses yellow\n'
               'until the robot is back up.'),
    'power_off': ('trigger_robot_shutdown',
                  'This computer shuts down, then the robot turns off fully, as after a long\n'
                  'press of the power button. With the charger connected it stays off and\n'
                  'charging. Press the power button to turn it on again.'),
}

KILL_TIMEOUT_S = 30
STOP_WAIT_S = 5     # How long a program gets to exit after SIGTERM, before SIGKILL
CONNECT_TRIES = 3   # The lock can take a moment to free after its holder exits

# Every program that opens the power board holds this lock (see Transport)
POWER_BOARD_LOCK = hu.build_transport_file_lock_path('hello-power-periph')

parser = argparse.ArgumentParser(description='Sleep, reboot or power off the robot')
action = parser.add_mutually_exclusive_group(required=True)
action.add_argument('--sleep', action='store_true', help='Shut down this computer and put the robot to sleep')
action.add_argument('--reboot', action='store_true', help='Shut down this computer, then wake the robot and start it again')
action.add_argument('--power_off', action='store_true', help='Shut down this computer, then turn the robot off fully')
parser.add_argument('-y', '--yes', action='store_true', help="Don't ask for confirmation")
args = parser.parse_args()

name = next(a for a in ACTIONS if getattr(args, a))
method, description = ACTIONS[name]


def lock_holders(path):
    """PIDs of the processes holding the flock on path. Read from /proc/locks,
    which lists the locks of every user's processes."""
    try:
        st = os.stat(path)
    except FileNotFoundError:
        return []
    want = (os.major(st.st_dev), os.minor(st.st_dev), st.st_ino)
    pids = set()
    try:
        with open('/proc/locks') as f:
            for line in f:
                # e.g. '33: FLOCK  ADVISORY  WRITE 134861 103:02:7347862 0 EOF'.
                # A process waiting for the lock has '->' after the index.
                parts = [p for p in line.split() if p != '->']
                if len(parts) < 6 or parts[1] != 'FLOCK':
                    continue
                try:
                    major, minor, inode = parts[5].split(':')
                    if (int(major, 16), int(minor, 16), int(inode)) == want:
                        pids.add(int(parts[4]))
                except ValueError:
                    continue
    except OSError:
        return []
    pids.discard(os.getpid())
    return sorted(pids)


def describe(pid):
    """'PID user command' for a process, as far as it can be read"""
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            cmd = ' '.join(f.read().replace(b'\0', b' ').decode(errors='replace').split())
    except OSError:
        cmd = ''
    if len(cmd) > 100:
        cmd = cmd[:97] + '...'
    try:
        user = pwd.getpwuid(os.stat(f'/proc/{pid}').st_uid).pw_name
    except (OSError, KeyError):
        user = '?'
    return f'{pid} ({user}) {cmd or "?"}'


def is_running(pid):
    """True while pid exists and hasn't exited (a zombie has exited)"""
    try:
        with open(f'/proc/{pid}/stat') as f:
            return f.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except (OSError, IndexError):
        return False


def stop_processes(pids):
    """SIGTERM, then SIGKILL whatever is still running after STOP_WAIT_S.
    Returns the PIDs that couldn't be signalled (e.g. another user's)."""
    denied = []
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        except PermissionError:
            denied.append(pid)
    waiting = [p for p in pids if p not in denied]
    deadline = time.time() + STOP_WAIT_S
    while time.time() < deadline and any(is_running(p) for p in waiting):
        time.sleep(0.1)
    for pid in waiting:
        if is_running(pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
    return denied


def stop_server():
    """Stop the Stretch Body Server so the power board's port is free. If a
    normal stop fails or hangs, force kill its processes."""
    print('Stopping the Stretch Body Server...')
    try:
        if subprocess.run(['stretch_body_server', '--kill'], timeout=KILL_TIMEOUT_S).returncode == 0:
            return
    except subprocess.TimeoutExpired:
        print('Stopping the Stretch Body Server timed out.')
    except FileNotFoundError:
        print('stretch_body_server was not found. Continuing.')
        return
    print('Force killing the Stretch Body Server...')
    try:
        subprocess.run(['stretch_body_server', '--cleanup'], timeout=KILL_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        print('Force killing the Stretch Body Server timed out. Trying the power board anyway.')


def free_power_board():
    """Stop any other program still holding the power board (e.g. a tool run
    with --direct). This computer is about to shut down, which would stop them
    anyway. Returns False if one couldn't be stopped."""
    holders = lock_holders(POWER_BOARD_LOCK)
    if not holders:
        return True
    print('Stopping the other programs using the power board:')
    for pid in holders:
        print('  ' + describe(pid))
    denied = stop_processes(holders)
    time.sleep(0.2)
    left = lock_holders(POWER_BOARD_LOCK)
    if not left:
        return True
    print('These programs are still using the power board:')
    for pid in left:
        print('  ' + describe(pid))
    if denied:
        print('They belong to another user. Close them, or run `sudo kill %s`, then try again.'
              % ' '.join(str(p) for p in denied))
    else:
        print('Close them, then try again.')
    return False


def send_direct():
    from stretch4_body.subsystem.power_periph import PowerPeriph
    for attempt in range(CONNECT_TRIES):
        if attempt:
            time.sleep(1)
        if lock_holders(POWER_BOARD_LOCK):
            continue  # Still held: don't try to open it, which only logs errors
        p = PowerPeriph()
        if not p.startup():
            continue
        try:
            getattr(p, method)()
            p.push_command()
        finally:
            p.stop()
        return True
    return False


def give_up(reason):
    print(reason)
    print('Nothing was changed: the robot is still awake. Try again, or use the power button.')
    sys.exit(1)


print(description)
print('The Stretch Body Server is stopped first, and any other program using the power board.')
others = lock_holders(POWER_BOARD_LOCK)
if others:
    print('Using the power board now:')
    for pid in others:
        print('  ' + describe(pid))
print('')
if not args.yes and not click.confirm('%s the robot now?' % name.replace('_', ' ').capitalize(), default=False):
    print('Cancelled. Nothing was changed.')
    sys.exit(1)

stop_server()
if not free_power_board():
    give_up("Couldn't free the power board.")
print('Sending to the power board...')
if not send_direct():
    give_up('Failed to reach the power board. If this keeps happening, run `stretch_system_check`.')
print('Done. This computer will shut down in a few seconds.')
