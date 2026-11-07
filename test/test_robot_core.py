from stretch4_body.robot.robot_core import RobotCore
import stretch4_body.core.hello_utils as hello_utils


def test_startup_names_the_robot_by_display_name_then_fleet_id(monkeypatch):
    monkeypatch.setattr(hello_utils, 'acquire_transport_filelock', lambda name: True)
    r = RobotCore()
    r.subsystems = {}
    lines = []
    monkeypatch.setattr(r.logger, 'info', lines.append)

    r.startup()
    assert hello_utils.get_fleet_id() in lines[-1]

    r.params['display_name'] = 'frankie-4'
    r.startup()
    assert 'Starting up Robot frankie-4' in lines[-1]
