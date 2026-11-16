"""The server sends the motor sync only after every stepper command it pushed is acked."""
import array as arr
import logging
from unittest.mock import MagicMock

from stretch4_body.core.stepper import Stepper
from stretch4_body.robot.robot_server import RobotServer, LOOP_STATE_RUNNING


class BoardLink:
    def __init__(self, name, events):
        self.name = name
        self.events = events
        self.acks = True
        self.in_flight = []

    def get_empty_payload(self):
        return arr.array('B', [0] * 64)

    def do_rpc(self, blocking, is_push, payload, rpc_callback):
        self.events.append(f'{self.name} command')
        self.in_flight.append(rpc_callback)
        return 1

    def load_rpc_results(self, wait_on_result=True):
        if self.acks:
            for cb in self.in_flight:
                self.events.append(f'{self.name} ack')
                cb(arr.array('B', [Stepper.RPC_REPLY_COMMAND]))
        self.in_flight = []
        return []


def make_stepper(name, events):
    s = Stepper.__new__(Stepper)
    s.name = name
    s.logger = logging.getLogger(name)
    s.hw_valid = True
    s.status = {'in_sync_mode': True}
    s.ts_last_syncd_motion = 0
    s.command_ack_missing = False
    s._dirty_trigger = False
    s._dirty_gains = False
    s._dirty_command = False
    s._pack_command = lambda payload, sidx: sidx
    s.pull_status = MagicMock()
    s.transport = BoardLink(name, events)
    return s


def make_server(events):
    server = RobotServer.__new__(RobotServer)
    server.state = LOOP_STATE_RUNNING
    server.power_periph = MagicMock(spec=['push_command', 'load_rpc_results', 'pull_status', 'trigger_motor_sync', 'ts_last_motor_sync'])
    server.power_periph.ts_last_motor_sync = 0.0
    server.power_periph.trigger_motor_sync.side_effect = lambda blocking: events.append('sync')
    server.subsystems = {'lift': make_stepper('lift', events), 'arm': make_stepper('arm', events),
                         'power_periph': server.power_periph}
    for name in ('safe_motion_manager', 'loop_mgmt', 'server', 'sentry_manager', 'routine_manager'):
        setattr(server, name, MagicMock())
    server.publish_status_msg = MagicMock()
    server._print_server_state = MagicMock()
    return server


def run_cycle(server, *moving):
    for name in moving:
        server.subsystems[name]._dirty_command = True
    server.cb_routine_update_controller()


def test_sync_goes_out_after_the_command_acks():
    events = []
    server = make_server(events)

    run_cycle(server, 'lift', 'arm')

    assert events == ['lift command', 'arm command', 'lift ack', 'arm ack', 'sync']


def test_no_sync_without_a_command():
    events = []
    server = make_server(events)

    run_cycle(server)

    assert events == []


def test_missing_ack_holds_the_sync_for_one_cycle():
    events = []
    server = make_server(events)
    server.subsystems['arm'].transport.acks = False

    run_cycle(server, 'lift', 'arm')
    assert events == ['lift command', 'arm command', 'lift ack']

    run_cycle(server)
    assert events[3:] == ['sync']


def test_a_joint_that_never_acks_holds_every_sync():
    events = []
    server = make_server(events)
    server.subsystems['arm'].transport.acks = False

    for _ in range(3):
        run_cycle(server, 'lift', 'arm')

    assert 'sync' not in events


def test_a_board_that_drops_out_stops_holding_the_sync():
    events = []
    server = make_server(events)
    server.subsystems['arm'].transport.acks = False
    run_cycle(server, 'lift', 'arm')
    server.subsystems['arm'].hw_valid = False

    run_cycle(server, 'lift', 'arm')

    assert events[-3:] == ['lift command', 'lift ack', 'sync']
