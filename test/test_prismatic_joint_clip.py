from unittest.mock import MagicMock

from stretch4_body.core.prismatic_joint import PrismaticJoint


def joint():
    j = PrismaticJoint.__new__(PrismaticJoint)
    j.name = 'lift'
    j.logger = MagicMock()
    j.motor = MagicMock()
    j.motor.status = {'pos_calibrated': True}
    j.soft_motion_limits = {'current': [0.0, 1.0]}
    j.status = {'pos': 0.5, 'in_collision_stop': {'pos': False, 'neg': False}}
    j.stiffness = 1.0
    j.vel_r = j.accel_r = 1.0
    j.contact_sensitivity_pos = j.contact_sensitivity_neg = 1.0
    j.i_feedforward = j.i_feedforward_payload = 0.0
    j.translate_m_to_motor_rad = lambda x: x
    return j


def test_move_to_past_the_limit_warns_and_sends_the_clipped_goal():
    j = joint()
    j.move_to(1.2)
    assert 'Clipping move_to(1.2)' in j.logger.warning.call_args[0][0]
    assert j.motor.set_command.call_args.kwargs['x_des'] == 1.0


def test_move_to_inside_the_limits_does_not_warn():
    j = joint()
    j.move_to(0.7)
    j.logger.warning.assert_not_called()
