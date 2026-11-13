"""RobotServer command and admin dispatch, on a server built without hardware."""
import logging
from unittest.mock import MagicMock, patch

import pytest

from stretch4_body.robot.robot_server import (
    RobotServer,
    LOOP_STATE_RUNNING,
    LOOP_STATE_PAUSED,
    LOOP_STATE_INVALID,
)


def make_server():
    server = RobotServer.__new__(RobotServer)
    server.name = 'robot'
    server.logger = logging.getLogger('robot')
    server.state = LOOP_STATE_RUNNING
    server.do_exit = False
    server.cmd_results = {}
    server.subsystems = {
        'power_periph': MagicMock(spec=['trigger_runstop', 'set_fan_on', 'trigger_motor_sync']),
        'omnibase': MagicMock(spec=['set_velocity']),
        'end_of_arm': MagicMock(),
    }
    server.power_periph = server.subsystems['power_periph']
    server.eoa_loop = MagicMock()
    server.routine_manager = MagicMock()
    server.routine_manager.status = {'active_routine': 'routine_nop'}
    server.sentry_manager = MagicMock()
    server.safe_motion_manager = MagicMock()
    server.loop_mgmt = MagicMock()
    server.server = MagicMock()
    server.status = {'server': {'status_id': 0}}
    return server


def cmd(subsystem, method, *args, **kwargs):
    return [subsystem, method, f'{subsystem}:{method}', args, kwargs]


def test_command_is_called_on_its_subsystem():
    server = make_server()
    server.power_periph.set_fan_on.return_value = 'fan'

    dispatched = server._cb_command_dispatch({'power_periph': cmd('power_periph', 'set_fan_on')})

    assert dispatched == ['power_periph:set_fan_on']
    server.power_periph.set_fan_on.assert_called_once_with()
    assert server.cmd_results['power_periph:set_fan_on']['result'] == 'fan'


def test_command_arguments_are_passed_through():
    server = make_server()

    server._cb_command_dispatch({'omnibase': cmd('omnibase', 'set_velocity', 0.1, 0.0, w=0.2)})

    server.subsystems['omnibase'].set_velocity.assert_called_once_with(0.1, 0.0, w=0.2)


def test_robot_command_is_called_on_the_server():
    server = make_server()

    dispatched = server._cb_command_dispatch({'robot': cmd('robot', 'pause_sentry', 'sentry_overtilt')})

    assert dispatched == ['robot:pause_sentry']
    server.sentry_manager.pause.assert_called_once_with(['sentry_overtilt'])


def test_end_of_arm_command_goes_to_the_end_of_arm_queue():
    server = make_server()
    wrist_cmd = cmd('wrist_yaw.end_of_arm', 'move_by', 'wrist_yaw', 0.1)

    dispatched = server._cb_command_dispatch({'wrist_yaw.end_of_arm': wrist_cmd})

    assert dispatched == ['wrist_yaw.end_of_arm:move_by']
    server.eoa_loop.q_cmd.put.assert_called_once_with(wrist_cmd)
    assert server.cmd_results == {}


@pytest.mark.parametrize('state', [LOOP_STATE_PAUSED, LOOP_STATE_INVALID])
def test_commands_are_ignored_unless_running(state):
    server = make_server()
    server.state = state

    dispatched = server._cb_command_dispatch({'power_periph': cmd('power_periph', 'trigger_runstop')})

    assert dispatched == []
    server.power_periph.trigger_runstop.assert_not_called()


def test_only_runstop_gets_through_while_a_routine_runs():
    server = make_server()
    server.routine_manager.status['active_routine'] = 'routine_robot_home'

    dispatched = server._cb_command_dispatch({
        'omnibase': cmd('omnibase', 'set_velocity', 0.1, 0.0, 0.0),
        'power_periph': cmd('power_periph', 'trigger_runstop'),
    })

    assert dispatched == ['power_periph:trigger_runstop']
    server.subsystems['omnibase'].set_velocity.assert_not_called()
    server.power_periph.trigger_runstop.assert_called_once_with()


def test_routine_command_queues_the_routine():
    server = make_server()

    dispatched = server._cb_command_dispatch({'routines': cmd('routines', 'routine_stow', 1.0)})

    assert dispatched == ['routines:routine_stow']
    server.routine_manager._routine_set_next.assert_called_once_with('routine_stow', 'routines:routine_stow', 1.0)


def test_routine_cancel_gets_through_while_a_routine_runs():
    server = make_server()
    server.routine_manager.status['active_routine'] = 'routine_robot_home'

    dispatched = server._cb_command_dispatch({'routines': cmd('routines', 'cancel')})

    assert dispatched == ['routines:cancel']
    server.routine_manager.cancel.assert_called_once_with()


def test_command_for_a_missing_subsystem_is_dropped(caplog):
    server = make_server()

    dispatched = server._cb_command_dispatch({'lift': cmd('lift', 'move_by', 0.1)})

    assert dispatched == []
    assert 'subsystem lift is not present' in caplog.text


def test_command_with_an_unknown_method_is_dropped(caplog):
    server = make_server()

    dispatched = server._cb_command_dispatch({'power_periph': cmd('power_periph', 'no_such_method')})

    assert dispatched == []
    assert server.cmd_results == {}
    assert 'invalid  cmd power_periph' in caplog.text


def test_command_results_keep_the_newest_5000():
    server = make_server()
    server.cmd_results = {i: {} for i in range(5000)}

    server._cb_command_dispatch({'power_periph': cmd('power_periph', 'set_fan_on')})

    assert len(server.cmd_results) == 5000
    assert 0 not in server.cmd_results
    assert 'power_periph:set_fan_on' in server.cmd_results


def test_admin_ping_is_acknowledged():
    server = make_server()

    assert server._cb_admin_dispatch(b'ping') == b'ping'
    assert server.state == LOOP_STATE_RUNNING


def test_admin_unknown_message_is_ignored():
    server = make_server()

    assert server._cb_admin_dispatch(b'reboot') is None
    assert server.state == LOOP_STATE_RUNNING


def test_admin_unpause_is_ignored_while_running():
    server = make_server()

    assert server._cb_admin_dispatch(b'unpause') is None
    assert server.state == LOOP_STATE_RUNNING


def test_admin_pause_then_unpause():
    server = make_server()
    server.pause_transport = MagicMock()
    server.unpause_transport = MagicMock()

    with patch('stretch4_body.robot.robot_server.hello_utils') as hello_utils:
        hello_utils.acquire_transport_filelock.return_value = True

        assert server._cb_admin_dispatch(b'pause') == b'pause'
        assert server.state == LOOP_STATE_PAUSED
        server.safe_motion_manager.enter_safe_stop.assert_called_once_with()
        server.pause_transport.assert_called_once_with()

        assert server._cb_admin_dispatch(b'unpause') == b'unpause'
        assert server.state == LOOP_STATE_RUNNING
        server.safe_motion_manager.exit_safe_stop.assert_called_once_with()
        server.unpause_transport.assert_called_once_with()


def test_admin_unpause_stays_paused_without_the_transport_lock(caplog):
    server = make_server()
    server.state = LOOP_STATE_PAUSED
    server.unpause_transport = MagicMock()

    with patch('stretch4_body.robot.robot_server.hello_utils') as hello_utils:
        hello_utils.acquire_transport_filelock.return_value = False
        assert server._cb_admin_dispatch(b'unpause') == b'unpause'

    assert server.state == LOOP_STATE_PAUSED
    server.unpause_transport.assert_not_called()
    assert 'Not able to UNPAUSE' in caplog.text


def test_admin_free_up_control_releases_the_lease():
    server = make_server()
    server.server.lease_holder_id = 'client'
    server.server.lease_holder_priority = 3

    with patch('stretch4_body.robot.robot_server.FreeableFileLock') as lock:
        assert server._cb_admin_dispatch(b'free_up_control') == b'free_up_control'

    lock.assert_called_once_with('pusher_client')
    lock.return_value.free.assert_called_once_with()
    assert server.server.lease_holder_id is None
    assert server.server.lease_holder_priority is None


def test_admin_kill_stops_the_server_and_ends_the_loop():
    server = make_server()
    server.stop = MagicMock()

    with patch('stretch4_body.robot.robot_server.hello_utils') as hello_utils:
        assert server._cb_admin_dispatch(b'kill') == b'kill'

    hello_utils.free_transport_filelock.assert_called_once_with('robot')
    server.stop.assert_called_once_with()
    assert server.state == LOOP_STATE_INVALID
    assert server.do_exit is True


def test_published_status_carries_state_and_lease():
    server = make_server()
    server.update_cpu_status = MagicMock()
    server.server.lease_holder_id = 'client'
    server.server.lease_holder_priority = 3
    server.server.lease_expiry = 12.5

    server.publish_status_msg()
    server.publish_status_msg()

    status = server.status['server']
    assert status['status_id'] == 2
    assert status['state'] == LOOP_STATE_RUNNING
    assert status['lease_holder'] == 'client'
    assert status['lease_holder_priority'] == '3'
    assert status['lease_expiry'] == 12.5
    server.server.publish_status.assert_called_with(server.status)


def routine_cycle_server():
    server = make_server()
    for name in ('push_command', 'load_rpc_results', 'publish_status_msg', 'pull_status', '_print_server_state'):
        setattr(server, name, MagicMock())
    return server


def test_routine_cycle_pushes_then_reads_new_commands():
    server = routine_cycle_server()

    assert server.cb_routine_update_controller() is True

    server.safe_motion_manager.step.assert_called_once_with()
    server.push_command.assert_called_once_with(blocking=False)
    server.power_periph.trigger_motor_sync.assert_called_once_with(blocking=False)
    server.sentry_manager.step.assert_called_once_with()
    server.server.dispatch_command_messages.assert_called_once_with(server._cb_command_dispatch, False)


def test_routine_cycle_tells_the_routine_to_stop_when_the_server_is_killed():
    server = routine_cycle_server()
    server.stop = MagicMock()
    server.server.dispatch_admin_messages.side_effect = lambda cb: cb(b'kill')

    with patch('stretch4_body.robot.robot_server.hello_utils'):
        assert server.cb_routine_update_controller() is False

    server.pull_status.assert_not_called()
    server.server.dispatch_command_messages.assert_not_called()


def test_control_loop_runs_routines_until_exit():
    server = make_server()
    server._print_server_state = MagicMock()
    runs = []

    def run_next():
        runs.append(1)
        server.do_exit = len(runs) == 3

    server.routine_manager._routine_run_next.side_effect = run_next

    server.run_controller()

    assert len(runs) == 3


def test_control_loop_serves_admin_and_status_while_paused():
    server = make_server()
    server.state = LOOP_STATE_PAUSED
    server._print_server_state = MagicMock()
    server.publish_status_msg = MagicMock(side_effect=lambda: setattr(server, 'do_exit', True))

    server.run_controller()

    server.routine_manager._routine_run_next.assert_not_called()
    server.server.dispatch_admin_messages.assert_called_once_with(server._cb_admin_dispatch)
    server.publish_status_msg.assert_called_once_with()


def test_control_loop_ends_when_a_cycle_raises(caplog):
    server = make_server()
    server.routine_manager._routine_run_next.side_effect = RuntimeError('lost transport')

    server.run_controller()

    assert 'Control Loop has ended' in caplog.text
    assert 'lost transport' in caplog.text
