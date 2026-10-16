"""The robot client keeps the server's status['firmware_versions'] after pull_status."""
import types
from unittest.mock import patch

from stretch4_body.core.subsystem_client import SubsystemClient

SERVER = {'server': {}, 'routines': {}, 'safety_layer': {},
          'firmware_versions': {'boards': [{'board': 'pimu'}], 'time': 1.0}}


@patch('stretch4_body.core.robot_params.RobotParams.get_params')
def pulled_status(server, mock_get_params):
    mock_get_params.return_value = ({}, {'robot': {'model_name': 'SE4'},
                                         'logging': {'level': 'DEBUG', 'console': True}})
    client = SubsystemClient('robot')
    client.is_valid = True
    client.client = types.SimpleNamespace(connected=True, _do_recv_status=lambda: dict(server))
    assert client.pull_status()
    return client.status


def test_firmware_versions_copied_to_the_root_status():
    assert pulled_status(SERVER)['firmware_versions'] == SERVER['firmware_versions']


def test_older_server_without_firmware_versions():
    server = {k: v for k, v in SERVER.items() if k != 'firmware_versions'}
    assert 'firmware_versions' not in pulled_status(server)
