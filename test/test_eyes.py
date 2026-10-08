import array as arr
import copy
import importlib
import subprocess
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from stretch4_body.eyes import Eyes, EyeState, Animation, find_animation, parse_color, parse_intensity
from stretch4_body.eyes.backends import SENTRY_NAME, EYE_PUSH_PRIORITY


def import_robot_module(name):
    """The robot modules need HELLO_FLEET_PATH and the built transport library at import."""
    # Off-robot the import raises KeyError (no fleet directory), not ImportError. Anything
    # else (NameError, SyntaxError, AttributeError) is a broken module and must fail.
    try:
        return importlib.import_module(name)
    except (ImportError, KeyError) as e:
        pytest.skip('needs a robot fleet directory to import {}: {!r}'.format(name, e))


# ---------------- Values ----------------

def test_animation_table_matches_firmware_enum_order():
    eyes = Eyes(backend='fake')
    anims = eyes.animations()
    assert [a.id for a in anims] == list(range(1, 15))
    assert all(isinstance(a, Animation) and a.name == a.name.lower() and ' ' not in a.name for a in anims)
    assert [a.name for a in anims if not a.uses_color] == ['off', 'rainbow_spin']
    assert [a.name for a in anims if not a.uses_intensity] == ['off']


def test_animation_table_matches_power_periph_defn():
    power_periph = import_robot_module('stretch4_body.subsystem.power_periph')
    defn = {k.lower(): v for k, v in power_periph.PowerPeriphDefn.EYE_ANIM_NAME_TO_IDX.items() if k != 'NOP'}
    assert {a.name: a.id for a in Eyes(backend='fake').animations()} == defn


@pytest.mark.parametrize('value', [4, '4', 'look_left', 'LOOK_LEFT', 'look-left', 'Look Left'])
def test_find_animation_accepts_ids_and_names(value):
    assert find_animation(value).id == 4


@pytest.mark.parametrize('value', [0, 15, 'nop', 'wink', True, None, 2.0])
def test_find_animation_rejects_with_valid_names(value):
    with pytest.raises(ValueError, match='idle_glow'):
        find_animation(value)


@pytest.mark.parametrize('value,rgb', [
    ((1, 2, 3), (1, 2, 3)), ([255, 0, 128], (255, 0, 128)), ('#ff8000', (255, 128, 0)),
    ('FF8000', (255, 128, 0)), ('#f80', (255, 136, 0)), ('Hot Pink', (255, 105, 180)),
    ('stretch', (40, 48, 60)),
])
def test_parse_color(value, rgb):
    assert parse_color(value) == rgb


@pytest.mark.parametrize('value', [(1, 2), (0, 0, 256), (1.0, 2, 3), '#12345', 'mauve', 5])
def test_parse_color_rejects(value):
    with pytest.raises(ValueError):
        parse_color(value)


def test_parse_intensity():
    assert parse_intensity(0) == 0
    assert parse_intensity(255) == 255
    assert parse_intensity(1) == 1
    assert parse_intensity(1.0) == 255
    assert parse_intensity(0.5) == 128
    for bad in (-1, 256, 1.5, -0.1, True, '128'):
        with pytest.raises(ValueError):
            parse_intensity(bad)


# ---------------- Eyes on the fake backend ----------------

def test_set_sends_full_payload_and_none_keeps_last_commanded():
    eyes = Eyes(backend='fake')
    eyes.set(left='look_left', right='blink', color='#102030', intensity=200)
    eyes.set(right='happy')
    eyes.set_color((255, 0, 0))
    eyes.set_intensity(0.5)
    assert eyes.backend.sent == [
        (4, 3, 200, 16, 32, 48),
        (4, 8, 200, 16, 32, 48),
        (4, 8, 200, 255, 0, 0),
        (4, 8, 128, 255, 0, 0),
    ]
    s = eyes.state()
    assert (s.left, s.right, s.color, s.intensity, s.source) == ('look_left', 'happy', (255, 0, 0), 128, 'commanded')


def test_uncommanded_eyes_are_sent_as_nop_and_report_none():
    eyes = Eyes(backend='fake')
    s = eyes.state()
    assert (s.left, s.right, s.source) == (None, None, 'assumed')
    eyes.set_color('red')
    assert eyes.backend.sent == [(0, 0, 255, 255, 0, 0)]
    s = eyes.state()
    assert (s.left, s.right, s.color, s.source) == (None, None, (255, 0, 0), 'commanded')
    eyes.set(left='blink')
    assert eyes.backend.sent[-1] == (3, 0, 255, 255, 0, 0)
    s = eyes.state()
    assert (s.left, s.right) == ('blink', None)


def test_off_and_idle():
    eyes = Eyes(backend='fake')
    eyes.set(left='alert', right='alert', color='blue', intensity=10)
    eyes.off()
    eyes.idle()
    assert eyes.backend.sent[1:] == [(1, 1, 10, 0, 0, 255), (2, 2, 255, 40, 48, 60)]


def test_invalid_set_sends_nothing_and_keeps_state():
    eyes = Eyes(backend='fake')
    eyes.set(left='blink', right='blink')
    with pytest.raises(ValueError):
        eyes.set(left='happy', color='nope')
    assert len(eyes.backend.sent) == 1
    assert eyes.state().left == 'blink'


def test_take_control_pauses_and_restores_sentry():
    eyes = Eyes(backend='fake')
    with eyes.take_control() as e:
        assert e is eyes
        assert eyes.backend.sentry_active is False
        assert eyes.state().in_control
        with eyes.take_control():
            pass
        assert eyes.backend.sentry_active is False
    assert eyes.backend.sentry_active is True
    assert not eyes.state().in_control


def test_take_control_leaves_an_inactive_sentry_inactive():
    eyes = Eyes(backend='fake')
    eyes.backend.sentry_active = False
    eyes.take_control()
    assert eyes.release() is True
    assert eyes.backend.sentry_active is False


def test_close_releases_control():
    eyes = Eyes(backend='fake')
    eyes.take_control()
    eyes.take_control()
    assert eyes.close() is True
    assert eyes.backend.sentry_active is True
    assert eyes.backend.connected is False


def test_release_returns_false_and_warns_when_the_resume_is_rejected(caplog):
    eyes = Eyes(backend='fake')
    eyes.backend.sentry_restore_fails = True
    eyes.take_control()
    assert eyes.release() is False
    assert 'still paused' in caplog.text and 'did not confirm' in caplog.text
    s = eyes.state()
    assert (s.sentry_active, s.in_control) == (False, False)
    eyes.backend.sentry_restore_fails = False
    assert eyes.resume_sentry() is True
    assert eyes.backend.sentry_active is True


def test_with_block_never_raises_when_the_resume_is_rejected(caplog):
    eyes = Eyes(backend='fake')
    eyes.backend.sentry_restore_fails = True
    with eyes.take_control():
        pass
    assert eyes.backend.sentry_active is False
    assert SENTRY_NAME in caplog.text
    with pytest.raises(KeyError):
        with eyes.take_control():
            raise KeyError('from the body')


def test_close_closes_the_backend_when_the_resume_fails_or_raises():
    eyes = Eyes(backend='fake')
    eyes.backend.sentry_restore_fails = True
    eyes.take_control()
    assert eyes.close() is False
    assert eyes.backend.connected is False

    eyes = Eyes(backend='fake')
    eyes.take_control()

    def gone(active, timeout=None):
        raise RuntimeError('server gone')
    eyes.backend.set_sentry = gone
    with pytest.raises(RuntimeError, match='server gone'):
        eyes.close()
    assert eyes.backend.connected is False


def test_a_pause_interrupted_before_confirmation_is_undone_by_close():
    eyes = Eyes(backend='fake')
    calls = []

    def interrupted(active, timeout=None):
        calls.append(active)
        if not active:
            raise KeyboardInterrupt
        eyes.backend.sentry_active = True
        return True
    eyes.backend.set_sentry = interrupted
    with pytest.raises(KeyboardInterrupt):
        eyes.take_control()
    assert not eyes.state().in_control
    assert eyes.close() is True
    assert calls == [False, True]


def test_state_reports_robot_overrides():
    eyes = Eyes(backend='fake')
    assert eyes.state().low_soc_override is None
    eyes.backend.battery_soc = 25
    assert eyes.state().low_soc_override == 'yellow'
    eyes.backend.battery_soc = 12
    eyes.backend.runstop = True
    s = eyes.state()
    assert (s.low_soc_override, s.runstop_active) == ('red', True)
    assert isinstance(s, EyeState)
    assert s.to_dict()['color_hex'] == '#28303c'


def test_capabilities():
    caps = Eyes(backend='fake').capabilities()
    assert set(caps) == {'pixels_per_eye', 'per_eye_animation', 'per_eye_color', 'per_pixel', 'readback',
                         'requires_protocol', 'protocol_version', 'sentry_installed', 'sentry_active'}
    assert caps['pixels_per_eye'] == 10
    assert not (caps['per_eye_color'] or caps['per_pixel'] or caps['readback'])
    assert (caps['requires_protocol'], caps['protocol_version']) == ('p13', 'p13')
    assert (caps['sentry_installed'], caps['sentry_active']) == (True, True)


def test_robot_without_the_eye_sentry():
    eyes = Eyes(backend='fake')
    eyes.backend.sentry_installed = False
    eyes.backend.set_sentry = MagicMock(return_value=True)
    caps = eyes.capabilities()
    assert (caps['sentry_installed'], caps['sentry_active']) == (False, None)
    assert eyes.state().sentry_active is None
    with eyes.take_control():
        assert eyes.state().in_control
    assert eyes.resume_sentry() is True
    assert eyes.close() is True
    eyes.backend.set_sentry.assert_not_called()


@pytest.mark.parametrize('name', ['usb', 'direct'])
def test_unknown_backend(name):
    with pytest.raises(ValueError, match='fake'):
        Eyes(backend=name)


def test_cli_on_the_fake_backend():
    cli = [sys.executable, '-m', 'stretch4_body.tools.stretch_eye_animations']
    run = subprocess.run(cli + ['--fake', '--color', 'red'], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
    assert 'left=unchanged right=unchanged color=#ff0000 intensity=255 (commanded)' in run.stdout
    assert '(0, 0, 255, 255, 0, 0)' in run.stdout
    run = subprocess.run(cli + ['--fake'], capture_output=True, text=True)
    assert run.returncode == 2
    assert 'usage:' in run.stdout and 'Nothing to send' in run.stdout


# ---------------- Server backend with a mocked server ----------------

class FakeServer:
    """
    Stands in for StretchBodyClient plus the server, in the server's real order: a push is
    queued, the next status pulled still predates it, and only then does the command go
    through the lease the way dispatch_command_messages does (an expired lease is cleared
    when the next command arrives, a live holder of equal or higher priority rejects a
    foreign push, a running routine rejects everything). 'robot' sentry commands flip the
    sentry flag; power_periph commands go into a PowerPeriph whose transport records RPCs.
    die() makes the server vanish: no new status, ping fails.
    """
    LEASE_TIMEOUT = 1.1

    def __init__(self, power_periph, sentry_active=True, sentry_installed=True):
        self.connected = True
        self.alive = True
        self.client_id = 'client_eyes_test'
        self.ip_address = None
        self.power_periph = power_periph
        self.sent = []
        self.priorities = []
        self.pending = []
        self.holder_priority = None
        # The sentry manager publishes one flag per sentry in the robot's params
        active = {SENTRY_NAME: sentry_active} if sentry_installed else {}
        self.status = {
            'server': {'lease_holder': 'None', 'lease_expiry': 0.0, 'status_id': 0},
            'routines': {'active_routine': 'routine_nop'},
            'safety_layer': {'sentry_manager': {'active': active}},
            'power_periph': {'runstop_event': False, 'battery_soc': 80},
        }

    def startup(self, **kwargs):
        return True

    def stop(self):
        self.connected = False

    def check_connection(self):
        self.connected = self.alive
        return self.alive

    def die(self):
        self.alive = False

    def _do_recv_status(self):
        if not self.alive:
            return None
        self.status['server']['status_id'] += 1
        snapshot = copy.deepcopy(self.status)
        while self.pending:
            self._dispatch(*self.pending.pop(0))
        return snapshot

    def _do_send_cmd(self, cmd_dict, priority=0):
        self.priorities.append(priority)
        self.pending.append((copy.deepcopy(dict(cmd_dict)), priority))

    def _dispatch(self, cmd_dict, priority):
        server = self.status['server']
        now = time.monotonic()
        if server['lease_holder'] != 'None' and (now > server['lease_expiry']
                                                 or (self.holder_priority is not None and priority > self.holder_priority)):
            server['lease_holder'] = 'None'
        if server['lease_holder'] == 'None':
            server['lease_holder'] = self.client_id
            self.holder_priority = priority
        elif server['lease_holder'] != self.client_id:
            return
        server['lease_expiry'] = now + self.LEASE_TIMEOUT
        if self.status['routines']['active_routine'] != 'routine_nop':
            return
        self.sent.append(cmd_dict)
        for subsystem, method, cmd_id, args, kwargs in cmd_dict.values():
            if subsystem == 'robot':
                active = self.status['safety_layer']['sentry_manager']['active']
                active[args[0]] = method == 'unpause_sentry'
            else:
                getattr(self.power_periph, method)(*args, **kwargs)
                self.power_periph.push_command()


def make_power_periph(protocol='p13'):
    """A PowerPeriph that packs RPCs into a mock transport. No serial port is opened."""
    power_periph = import_robot_module('stretch4_body.subsystem.power_periph')
    pp = power_periph.PowerPeriphBase.__new__(power_periph.PowerPeriphBase)
    pp.hw_valid = True
    pp.imu = SimpleNamespace(_dirty_config=False)
    pp._dirty_config = pp._dirty_trigger = pp._dirty_eye_animation = False
    pp.board_info = {'protocol_version': protocol}
    pp.logger = MagicMock()
    pp.transport = MagicMock()
    pp.transport.get_empty_payload.side_effect = lambda: arr.array('B', [0] * 32)
    return pp


def rpc_payloads(pp):
    return [list(c.kwargs['payload']) for c in pp.transport.do_rpc.call_args_list]


def connect_eyes(server):
    """An Eyes on a real PowerPeriphClient whose server connection is the FakeServer."""
    robot_client = import_robot_module('stretch4_body.robot.robot_client')
    client = robot_client.PowerPeriphClient()
    client.client = server
    client.push_lock = MagicMock(is_locked=False)
    client.startup()
    return Eyes(client=client), client


@pytest.fixture
def server_eyes():
    pp = make_power_periph()
    server = FakeServer(pp)
    eyes, client = connect_eyes(server)
    yield eyes, server, pp, client
    try:
        eyes.close()
    except RuntimeError:
        pass  # the server-loss tests leave the fake dead


def test_server_payload_reaches_power_periph_with_color(server_eyes):
    eyes, server, pp, client = server_eyes
    assert eyes.set(left='look_left', right='circle_ccw', color='#ff8000', intensity=0.5).source == 'commanded'
    eyes.set_color('blue')
    assert server.sent[0] == {'power_periph': ['power_periph', 'set_eye_animation', server.sent[0]['power_periph'][2], (),
                                               {'left_idx': 4, 'right_idx': 14, 'intensity': 128, 'r': 255, 'g': 128, 'b': 0}]}
    assert rpc_payloads(pp) == [[29, 4, 14, 128, 255, 128, 0], [29, 4, 14, 128, 0, 0, 255]]
    client.push_lock.acquire.assert_not_called()  # ignore_control_lock


def test_server_pushes_never_outrank_a_motion_client(server_eyes):
    eyes, server, pp, client = server_eyes
    with eyes.take_control():
        eyes.set(left='blink')
    assert EYE_PUSH_PRIORITY == -1
    assert len(server.priorities) >= 3 and set(server.priorities) == {-1}  # pause, eyes, unpause


def test_server_expired_holder_counts_as_free(server_eyes):
    eyes, server, pp, client = server_eyes
    # The server clears an expired lease only when the next command arrives and keeps
    # publishing the old holder; on the same host the published lease_expiry tells
    server.status['server'].update(lease_holder='client_stretch_robot_stow_1', lease_expiry=time.monotonic() - 1)
    assert eyes.state().lease_holder is None
    s = eyes.set(left='alert', color='red')
    assert (s.source, s.lease_holder) == ('commanded', None)  # our own lease is never reported
    assert rpc_payloads(pp) == [[29, 7, 0, 255, 255, 0, 0]]


def test_server_live_holder_means_dropped(server_eyes):
    eyes, server, pp, client = server_eyes
    server.status['server'].update(lease_holder='client_teleop_1', lease_expiry=time.monotonic() + 5)
    s = eyes.set(left='alert', color='red')
    assert (s.source, s.lease_holder) == ('dropped', 'client_teleop_1')
    assert rpc_payloads(pp) == []
    assert (s.left, s.color) == ('alert', (255, 0, 0))  # cached, so the next accepted send carries them
    server.status['server']['lease_holder'] = 'None'
    server.status['routines']['active_routine'] = 'routine_robot_stow'
    assert eyes.set(right='happy').source == 'dropped'
    assert rpc_payloads(pp) == []


def test_server_tcp_client_cannot_tell_an_expired_holder(server_eyes):
    eyes, server, pp, client = server_eyes
    server.ip_address = '10.0.0.5'
    server.status['server'].update(lease_holder='client_stretch_robot_stow_1', lease_expiry=time.monotonic() - 1)
    assert eyes.set(left='alert').source == 'dropped'
    assert rpc_payloads(pp) == [[29, 7, 0, 255, 40, 48, 60]]  # the server itself did take it


def test_server_take_control_pauses_and_restores_sentry(server_eyes):
    eyes, server, pp, client = server_eyes
    active = server.status['safety_layer']['sentry_manager']['active']
    with eyes.take_control():
        assert active[SENTRY_NAME] is False
        assert eyes.state().sentry_active is False
        eyes.off()
    assert active[SENTRY_NAME] is True
    robot_cmds = [c['robot'][1] for c in server.sent if 'robot' in c]
    assert robot_cmds == ['pause_sentry', 'unpause_sentry']
    assert rpc_payloads(pp) == [[29, 1, 1, 255, 40, 48, 60]]


def test_server_without_the_eye_sentry():
    pp = make_power_periph()
    server = FakeServer(pp, sentry_installed=False)
    eyes, client = connect_eyes(server)
    caps = eyes.capabilities()
    assert (caps['sentry_installed'], caps['sentry_active']) == (False, None)
    assert eyes.state().sentry_active is None
    t0 = time.monotonic()
    with eyes.take_control():
        eyes.set(left='blink')
    assert eyes.resume_sentry() is True
    assert eyes.close() is True
    assert time.monotonic() - t0 < 0.5  # nothing waited for a confirmation the server never sends
    assert [c for c in server.sent if 'robot' in c] == []
    assert rpc_payloads(pp) == [[29, 3, 0, 255, 40, 48, 60]]


def test_server_resume_warning_never_names_our_own_client(server_eyes, caplog):
    eyes, server, pp, client = server_eyes
    with eyes.take_control():
        server.status['routines']['active_routine'] = 'routine_robot_stow'  # our unpause is now skipped
    assert eyes.state().sentry_active is False
    assert 'did not confirm the resume' in caplog.text
    assert server.client_id not in caplog.text
    server.status['routines']['active_routine'] = 'routine_nop'  # let the fixture's close() resume at once


def test_server_state_reads_runstop_and_soc(server_eyes):
    eyes, server, pp, client = server_eyes
    server.status['power_periph'].update(runstop_event=True, battery_soc=20)
    s = eyes.state()
    assert (s.runstop_active, s.battery_soc, s.low_soc_override) == (True, 20, 'yellow')


def test_server_protocol_is_not_published(server_eyes):
    eyes, server, pp, client = server_eyes
    caps = eyes.capabilities()
    assert (caps['requires_protocol'], caps['protocol_version']) == ('p13', None)


def test_server_loss_is_caught_before_a_command_is_called_dropped(server_eyes):
    eyes, server, pp, client = server_eyes
    # StretchBodyClient.connected never flips by itself: the fake stops publishing and
    # fails the ping, which is all a dead server looks like from a client
    server.status['server'].update(lease_holder='client_teleop_1', lease_expiry=time.monotonic() + 5)
    eyes.state()
    server.die()
    with pytest.raises(RuntimeError, match='Lost the connection to stretch_body_server'):
        eyes.set_color('red')
    with pytest.raises(RuntimeError, match='Lost the connection to stretch_body_server'):
        eyes.state()


def test_server_loss_is_caught_when_a_sentry_change_is_never_confirmed(server_eyes):
    eyes, server, pp, client = server_eyes
    server.die()
    with pytest.raises(RuntimeError, match='Lost the connection to stretch_body_server'):
        eyes.take_control()  # the pause is never confirmed: the ping decides after the wait
    with pytest.raises(RuntimeError, match='Lost the connection to stretch_body_server'):
        eyes.close()  # the pause went out, so close() tries to undo it; the backend closes anyway


def test_shared_client_is_not_stopped():
    robot_client = import_robot_module('stretch4_body.robot.robot_client')
    client = robot_client.PowerPeriphClient()
    client.client = FakeServer(make_power_periph())
    client.startup()
    Eyes(client=client).close()
    assert client.client.connected


@pytest.mark.parametrize('bad', [dict(intensity=300), dict(r=None), dict(intensity=128.0), dict(g=True),
                                 dict(left_idx=15), dict(b=-1)])
def test_client_rejects_bad_eye_bytes_before_they_reach_the_server(bad):
    robot_client = import_robot_module('stretch4_body.robot.robot_client')
    client = robot_client.PowerPeriphClient()
    kwargs = dict(left_idx=2, right_idx=2)
    kwargs.update(bad)
    with pytest.raises(ValueError, match=next(iter(bad))):
        client.set_eye_animation(**kwargs)
    assert len(client.cmd_dict) == 0


def test_server_side_set_eye_animation_never_raises():
    pp = make_power_periph()
    assert pp.set_eye_animation(2, None, intensity=128.0, r=0, g=255, b=255) is True
    pp.push_command()
    assert rpc_payloads(pp) == [[29, 2, 0, 128, 0, 255, 255]]
    for bad in (dict(intensity=300), dict(r=None), dict(g=128.9), dict(b=True), dict(left_idx='7')):
        kwargs = dict(left_idx=2, right_idx=2, intensity=255, r=255, g=255, b=255)
        kwargs.update(bad)
        assert pp.set_eye_animation(**kwargs) is False
        assert pp._dirty_eye_animation is False
    assert pp.logger.warning.call_count == 5
