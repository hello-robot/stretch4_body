#!/usr/bin/env python3
"""Turning firmware guarded contact off and on with the 'off' profile, and its status. No hardware needed."""
from stretch4_body.subsystem.arm import Arm
from stretch4_body.subsystem.lift import Lift
from stretch4_body.subsystem.omnibase import OmniBase


def test_prismatic_joints():
    for joint in (Lift(), Arm()):
        joint.set_guarded_contact_sensitivity('off')
        assert joint.motor.gains['enable_guarded_mode'] == 0
        assert joint.status['motor']['enable_guarded_mode'] == 0
        joint.set_guarded_contact_sensitivity('medium_strength')
        assert joint.motor.gains['enable_guarded_mode'] == 1
        assert joint.status['motor']['enable_guarded_mode'] == 1


def test_omnibase():
    base = OmniBase()
    base.set_guarded_contact_sensitivity('medium_strength')
    assert all(base.status[f'wheel_{i}']['enable_guarded_mode'] == 1 for i in range(3))
    base.set_guarded_contact_sensitivity('off')
    assert all(w.gains['enable_guarded_mode'] == 0 for w in base.wheels)
    assert all(base.status[f'wheel_{i}']['enable_guarded_mode'] == 0 for i in range(3))


def test_robot_off_profile_names_off_for_every_subsystem():
    from stretch4_body.core.robot_params import RobotParams
    profiles = RobotParams.get_params()[1]['robot']['guarded_contact']
    assert profiles['off'] and set(profiles['off'].values()) == {'off'}
    assert set(profiles['off']) == set(profiles['medium_strength'])
