from array import array
from collections import deque

from stretch4_body.robot.robot_server import RobotServer
from stretch4_body.subsystem.power_periph import PowerPeriphAux


class FakePowerPeriph:
    def __init__(self):
        self.status_aux = {
            "current_pwr_state": 3,
            "current_pwr_state_name": "STATE_SLEEP_CHRG",
            "runstop_cause_raw": 4,
            "motor_sync_cnt": 123,
        }
        self.requests = 0

    def pull_status_aux(self, blocking):
        assert blocking is False
        self.requests += 1
        return True


def make_server():
    server = RobotServer.__new__(RobotServer)
    server.power_periph = FakePowerPeriph()
    server.get_subsystem = lambda name: (
        server.power_periph if name == "power_periph" else None
    )
    server._power_aux_interval_s = 0.5
    server._power_aux_pending = False
    server._power_aux_last_request_mono = float("-inf")
    server._power_aux_last_sample_mono = None
    server._power_aux_last_state = None
    server._power_aux_transition_seq = 0
    server._power_aux_transitions = deque(maxlen=64)
    server._power_aux_meta = {
        "available": False,
        "sample_seq": 0,
        "request_errors": 0,
        "transitions": [],
    }
    return server


def test_power_aux_is_queued_nonblocking_and_records_transition():
    server = make_server()
    server._queue_power_periph_aux_status()
    assert server._power_aux_pending
    assert server.power_periph.requests == 1

    server._complete_power_periph_aux_status()
    assert not server._power_aux_pending
    assert server._power_aux_meta["available"]
    assert server._power_aux_meta["sample_seq"] == 1
    assert server._power_aux_meta["transitions"] == [{
        "seq": 1,
        "observed_wall_time_s": server._power_aux_meta["sample_wall_time_s"],
        "observed_monotonic_s": server._power_aux_meta["sample_monotonic_s"],
        "from_state": None,
        "to_state": 3,
        "to_state_name": "STATE_SLEEP_CHRG",
        "runstop_cause_raw": 4,
    }]


def test_power_aux_decoder_exposes_state_machine_and_signed_temperatures():
    power = PowerPeriphAux()
    consumed = power._unpack_status_aux(array(
        "B",
        [3, 4, 0x34, 0x12, 29, 30, 0xFF, 27, 0x80],
    ))
    assert consumed == 9
    assert power.status_aux["current_pwr_state_name"] == "STATE_SLEEP_CHRG"
    assert power.status_aux["runstop_cause_raw"] == 4
    assert power.status_aux["motor_sync_cnt"] == 0x1234
    assert power.status_aux["battery_cell_temp_3"] == -1
    assert power.status_aux["battery_mosfet_temp"] == -128
    assert power.status_aux["_valid"]


def test_power_aux_decoder_rejects_short_packets():
    power = PowerPeriphAux()
    assert power._unpack_status_aux(array("B", [0, 0])) == 0
    assert not power.status_aux["_valid"]
    assert "short aux packet" in power.status_aux["_decode_error"]
