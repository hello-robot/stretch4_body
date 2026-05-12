
import unittest
from unittest.mock import patch

from stretch4_body.core.client_server import StretchBodyClient, NotConnectedError
from stretch4_body.core.subsystem_client import SubsystemClient


class TestRequireConnection(unittest.TestCase):
    def test_client_throws_exception(self):
        client = StretchBodyClient()
        # Should raise NotConnectedError because startup() was not called
        with self.assertRaises(NotConnectedError):
            client._do_recv_status()

        with self.assertRaises(NotConnectedError):
            client._do_send_cmd({'test': 1})

        with self.assertRaises(NotConnectedError):
            client._do_send_recv_admin_str(b"ping")

    @patch('stretch4_body.core.robot_params.RobotParams.get_params')
    def test_subsystem_client_throws_exception(self, mock_get_params):
        # Mock params
        # get_params returns (user_params, robot_params)
        mock_get_params.return_value = ({}, {'test_subsystem': {'param': 1}, 'logging': {'level': 'DEBUG', 'console': True}})

        sub = SubsystemClient("test_subsystem")

        with self.assertRaises(NotConnectedError):
            sub.pull_status()

        with self.assertRaises(NotConnectedError):
            sub.push_command()

        with self.assertRaises(NotConnectedError):
            sub.kill_server()

        with self.assertRaises(NotConnectedError):
            sub.pause_control_loop()

if __name__ == '__main__':
    unittest.main()
