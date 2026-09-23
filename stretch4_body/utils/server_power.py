"""Helpers for changing actuator power while the stretch_body_server is running.

Actuator power cannot be changed through the server. The server holds each
motor's serial transport open, and those handles are inherited by its forked
child processes, so dropping a rail underneath it leaves the tty held and the
board cannot re-enumerate when power returns. The server also fails to start
while a motor it expects is unpowered, which puts it into a restart loop.

So every power change goes through the direct API with the server stopped. These
helpers wrap that dance: stop the server, hand back whether it was running, and
start it again afterwards.
"""

import subprocess
import time

import click


def is_server_running(timeout_tries=1):
    """True if a stretch_body_server is reachable, whoever owns it."""
    try:
        from stretch4_body.robot.robot_client import PowerPeriphClient
    except ImportError:
        return False
    for _ in range(timeout_tries):
        p = PowerPeriphClient()
        try:
            if p.startup(verbose=False, allow_different_user_connection=True):
                return True
        except Exception:
            pass
        finally:
            p.stop()
    return False


def kill_server():
    """Stop the running server (and its daemon). Returns True on success."""
    print('Stopping stretch_body_server...')
    subprocess.run(['stretch_body_server', '--kill'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(15):
        if not is_server_running():
            return True
        time.sleep(1.0)
    print('Could not stop the stretch_body_server. Try `stretch_body_server --kill` manually.')
    return False


def start_server(timeout=45.0):
    """Start the server again and wait for it to answer. Returns True on success."""
    print('Starting stretch_body_server...')
    # --restart launches the daemon when one is installed, and otherwise runs the
    # server in the foreground of this child, so detach it and leave it running.
    subprocess.Popen(['stretch_body_server', '--restart'],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     start_new_session=True)
    print('Waiting for stretch_body_server to come back online...')
    ts = time.time()
    while time.time() - ts < timeout:
        if is_server_running():
            print('stretch_body_server is back online.')
            return True
        time.sleep(1.0)
    print('Timed out waiting for stretch_body_server. Run `stretch_body_server --restart` manually.')
    return False


def stop_server_for_power_change(reason='change actuator power', assume_yes=False):
    """Ask to stop the server so a power change can be made directly.

    Returns (ok, was_running). `ok` is False only when the user declined or the
    server would not stop, in which case the caller should not touch the rails.
    Pass `was_running` back to resume_server() afterwards.
    """
    if not is_server_running():
        return True, False

    print("""Stretch Body Server is running and has to be stopped first.
""")
    if not assume_yes and not click.confirm(
            'Stop the stretch_body_server, %s, then start it again?' % reason, default=True):
        return False, True
    if not kill_server():
        return False, True
    return True, True


def resume_server(was_running, assume_yes=False, always_prompt=False):
    """Start the server again after a power change.

    `was_running` is what stop_server_for_power_change() reported, and it sets
    the default: restoring a server that was up is the expected outcome.
    `always_prompt` also offers to start one that was never running, which is
    the usual case after reviving a board the server could not open.
    `assume_yes` skips every prompt and simply restores the prior state.
    """
    if assume_yes:
        return start_server() if was_running else True

    if was_running:
        if not click.confirm('Start the stretch_body_server again?', default=True):
            print('Leaving the stretch_body_server stopped. Run `stretch_body_server --restart` when ready.')
            return True
    elif always_prompt:
        if not click.confirm('Start the stretch_body_server now?', default=False):
            return True
    else:
        return True
    return start_server()
