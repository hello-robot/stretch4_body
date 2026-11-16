#!/usr/bin/env python3

from stretch4_body.core.hello_utils import overwrite_dict


def test_none_default_accepts_any_type():
    params = {'robot': {'server': {'enable_network_access': None}}}
    assert overwrite_dict(params, {'robot': {'server': {'enable_network_access': '*'}}}) is True
    assert params['robot']['server']['enable_network_access'] == '*'


def test_typed_default_still_rejects_none():
    params = {'control_loop_rate_Hz': 100}
    assert overwrite_dict(params, {'control_loop_rate_Hz': None}) is False
    assert params['control_loop_rate_Hz'] == 100


def test_typed_default_still_rejects_other_types():
    params = {'name': 'robot'}
    assert overwrite_dict(params, {'name': 5}) is False
    assert params['name'] == 'robot'


def test_numbers_still_interchangeable():
    params = {'rate': 100}
    assert overwrite_dict(params, {'rate': 50.5}) is True
    assert params['rate'] == 50.5
