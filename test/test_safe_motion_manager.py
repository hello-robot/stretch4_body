"""SafeMotionManager safe stop, on a manager built without hardware."""
from unittest.mock import MagicMock

from stretch4_body.behavior.safe_motions.safe_motion_manager import SafeMotionManager


def make_manager(subsystems):
    manager = SafeMotionManager.__new__(SafeMotionManager)
    manager.robot = MagicMock()
    manager.robot.subsystems = {name: MagicMock() for name in subsystems}
    manager.status = {'safe_motions_triggered': [], 'active': {}, 'in_safe_stop': False}
    return manager


def test_safe_stop_freewheels_the_base_and_puts_arm_and_lift_in_safety():
    manager = make_manager(['omnibase', 'arm', 'lift'])

    manager.enter_safe_stop()

    assert manager.status['in_safe_stop'] is True
    manager.robot.subsystems['omnibase'].enable_freewheel_mode.assert_called_once_with()
    manager.robot.subsystems['arm'].enable_safety.assert_called_once_with()
    manager.robot.subsystems['lift'].enable_safety.assert_called_once_with()


def test_safe_stop_skips_subsystems_the_robot_does_not_have():
    manager = make_manager(['lift'])

    manager.enter_safe_stop()

    manager.robot.subsystems['lift'].enable_safety.assert_called_once_with()


def test_exit_safe_stop_clears_the_flag():
    manager = make_manager([])
    manager.enter_safe_stop()

    manager.exit_safe_stop()

    assert manager.status['in_safe_stop'] is False
