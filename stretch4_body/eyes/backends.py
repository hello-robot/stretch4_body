"""
Backends for stretch4_body.eyes.Eyes. Each one offers the same small surface:

    connect()                       raise RuntimeError when the robot cannot be reached
    send(payload) -> bool           payload is (left_id, right_id, intensity, r, g, b);
                                    False when the server is known to have rejected it
    is_sentry_active() -> bool/None None when sentry_eye_animations is not installed on this robot
    set_sentry(active, timeout) -> bool  True at once when the sentry is not installed
    robot_status() -> dict          {'runstop': bool or None, 'battery_soc': int or None}
    lease_holder() -> str or None   another client the server names as lease holder
    protocol_version() -> str/None  'p13' and so on, None when it cannot be read
    close()
"""
import logging
import time

from stretch4_body.core.client_server import NotConnectedError

logger = logging.getLogger(__name__)

SENTRY_NAME = 'sentry_eye_animations'
NOP_ID = 0              # EYE_ANIM_NOP: the firmware leaves that eye as it is
# Below the default push priority 0, so an eye push never holds the server lease against
# a motion client: a default-priority push from any other client takes the lease at once
EYE_PUSH_PRIORITY = -1


class FakeBackend:
    """
    In-memory stand-in for a p13 robot. Records every payload in self.sent as
    (left_id, right_id, intensity, r, g, b). Set runstop, battery_soc or sentry_active
    to try the override and sentry paths without a robot; set sentry_restore_fails to
    make every resume (set_sentry(True)) fail the way a rejected unpause does, and
    sentry_installed False for a robot whose params do not list the eye sentry.
    """
    def __init__(self):
        self.sent = []
        self.runstop = False
        self.battery_soc = 100
        self.sentry_active = True
        self.sentry_installed = True
        self.sentry_restore_fails = False
        self.connected = False

    def connect(self):
        self.connected = True

    def send(self, payload):
        self.sent.append(tuple(payload))
        return True

    def is_sentry_active(self):
        return self.sentry_active if self.sentry_installed else None

    def set_sentry(self, active, timeout=1.0):
        if not self.sentry_installed:
            return True
        if active and self.sentry_restore_fails:
            return False
        self.sentry_active = active
        return True

    def robot_status(self):
        return {'runstop': self.runstop, 'battery_soc': self.battery_soc}

    def lease_holder(self):
        return None

    def protocol_version(self):
        return 'p13'

    def close(self):
        self.connected = False


class ServerBackend:
    """
    Sends eye commands through stretch_body_server. Never opens the serial port.

    By default this owns a standalone PowerPeriphClient rather than a RobotClient, because
    RobotClient.stop() queues freewheel and safety commands for every joint, which an eye
    tool must not do to a robot someone else is driving. Pass client= to share an existing
    RobotClient or PowerPeriphClient; it is then never stopped here, and any commands the
    caller has queued on it go out with the next eye command.

    Every push uses ignore_control_lock=True. Without it push_command takes the
    'pusher_client' file lock, and exits the process if another client already holds it,
    which would also stop teleop or a ROS driver from starting while the eyes are in use.

    Every push also goes out at priority EYE_PUSH_PRIORITY (-1). The server gives its
    command lease (client_server.LEASE_TIMEOUT, 1.1 s) to whoever pushes while it is free
    and to any push of higher priority than the holder's, so at -1 an eye client never
    holds the lease against a motion client: stretch_robot_stow, the ROS driver or a
    script pushing at the default 0 takes it at once. The other way round still applies:
    while another client is streaming commands (it pushed within the last 1.1 s) or a
    routine is running, eye pushes are dropped without a reply. send() reports that from
    the latest status the server published, without waiting. That status predates the
    push, and the server only clears an expired lease when the next command arrives, so
    it keeps naming a client that has gone quiet. On the robot itself the published
    lease_expiry (server time.monotonic(), one clock per host) tells a live holder from
    a dead one, and a dead one counts as free; over tcp the clocks differ, so any other
    holder named in the status counts as live. A client that starts pushing in the same
    cycle slips through either way. lease_holder() applies the same rule.

    The server does not publish the PIMU protocol, so protocol_version() returns None.

    A server that dies after connect() is invisible on the happy path: it stops
    publishing, pushes vanish, and the flag StretchBodyClient.connected stays True until
    something pings. The two failure branches (a push about to be reported dropped, a
    sentry change the status never confirmed) ping the admin socket first and raise
    RuntimeError('Lost the connection to stretch_body_server') instead of blaming the
    lease; after that every call raises.
    """
    def __init__(self, client=None):
        self._owns_client = client is None
        if client is None:
            from stretch4_body.robot.robot_client import PowerPeriphClient
            client = PowerPeriphClient()
        self._pp = getattr(client, 'power_periph', client)

    def connect(self):
        if self._owns_client and not self._pp.startup(verbose=False):
            raise RuntimeError(self._connect_failure())

    @staticmethod
    def _connect_failure():
        from stretch4_body.core.client_server import StretchBodyServer
        from stretch4_body.utils.file_access_utils import is_user_in_group
        if not is_user_in_group('users'):
            return ("Cannot connect to stretch_body_server: this user is not in the 'users' group, "
                    "which the server's locks need.")
        try:
            if not StretchBodyServer.is_server_owned_by_current_user():
                return ('Cannot connect to stretch_body_server: its socket belongs to user {!r}. '
                        'Run as that user, or `stretch_body_server --kill` and start it again.'
                        .format(StretchBodyServer.get_server_owning_user()))
        except (OSError, KeyError):
            pass  # no socket file: no server has run since boot
        return 'Could not reach stretch_body_server. Start it with `stretch_body_server`.'

    def close(self):
        if self._owns_client:
            self._pp.stop()

    def _client_id(self):
        return str(getattr(self._pp.client, 'client_id', ''))

    def _guard(self, fn, *args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except NotConnectedError as e:
            raise RuntimeError('Lost the connection to stretch_body_server') from e

    def _push(self):
        self._guard(self._pp.push_command, ignore_control_lock=True, priority=EYE_PUSH_PRIORITY)

    def _status(self):
        self._guard(self._pp.pull_status, blocking=False)
        return self._pp.status

    def send(self, payload):
        """Queue RPC 29 and push it. False when the latest server status says the lease rejects it."""
        left, right, intensity, r, g, b = payload
        self._pp.set_eye_animation(left_idx=left, right_idx=right, intensity=intensity, r=r, g=g, b=b)
        self._push()
        return not self._lease_rejects_us()

    def _lease_rejects_us(self):
        # Read from the status already published, never waited for. A routine keeps its
        # client's lease alive every cycle, and any other live holder outranks priority
        # -1 until its lease expires 1.1 s after its last push.
        s = self._status()
        routine = s.get('routines', {}).get('active_routine', 'routine_nop') != 'routine_nop'
        holder = self._holder(s)
        if routine or (holder is not None and holder != self._client_id()):
            self._check_alive()  # a dead server looks the same from here; raise, do not blame the lease
            return True
        return False

    def _holder(self, status):
        """The published lease holder; None when free, or expired where this host can tell."""
        server = status.get('server', {})
        holder = server.get('lease_holder')
        if holder in (None, 'None', ''):
            return None
        expiry = server.get('lease_expiry')
        if self._pp.client.ip_address is None and expiry is not None and time.monotonic() > float(expiry):
            return None
        return str(holder)

    def lease_holder(self):
        """Another client holding the server lease; None when it is free, expired or this client's."""
        holder = self._holder(self._pp.status)
        return None if holder == self._client_id() else holder

    def _check_alive(self):
        if not self._guard(self._pp.ping_server):
            raise RuntimeError('Lost the connection to stretch_body_server')

    def is_sentry_active(self):
        # The sentry manager publishes one 'active' flag per sentry the robot's params
        # list; no key means sentry_eye_animations is not installed on this robot
        active = self._status().get('safety_layer', {}).get('sentry_manager', {}).get('active', {})
        if SENTRY_NAME not in active:
            return None
        return bool(active[SENTRY_NAME])

    def set_sentry(self, active, timeout=1.0):
        """
        Pause or unpause the eye sentry and wait for the server status to confirm it.
        The command socket keeps only the latest message, so a push that lands in the
        same server cycle as another can be lost; resend until the status agrees. A push
        the lease rejects is never confirmed, so this returns False after timeout. The
        server ignores the command for a sentry it does not have, so a robot without the
        eye sentry returns True at once.
        """
        if self.is_sentry_active() is None:
            return True
        command = 'unpause_sentry' if active else 'pause_sentry'
        ts = time.time()
        while True:
            # Same command RobotClient.pause_sentry sends; the server runs 'robot' commands itself
            self._pp._queue_command('robot', command, SENTRY_NAME)
            self._push()
            t_resend = time.time() + 0.25
            while time.time() < t_resend:
                time.sleep(0.02)
                if self.is_sentry_active() == active:
                    return True
            if time.time() - ts > timeout:
                self._check_alive()  # a dead server never confirms either; raise instead of returning False
                return False

    def robot_status(self):
        s = self._status()
        return {'runstop': bool(s.get('runstop_event', False)), 'battery_soc': s.get('battery_soc')}

    def protocol_version(self):
        # TODO: return status['power_periph']['protocol_version'] once the P14 server publishes it
        return None
