#!/usr/bin/env python3

import pytest
import yaml

from stretch4_body.tools import stretch_body_server as sbs


@pytest.fixture
def fleet(monkeypatch, tmp_path):
    """A temporary fleet directory with a user params file, and fixed local addresses."""
    fleet_id = "stretch-se4-test"
    (tmp_path / fleet_id).mkdir()
    params_file = tmp_path / fleet_id / "stretch_user_params.yaml"
    params_file.write_text("robot:\n  tool: eoa_test_tool\n")
    monkeypatch.setenv("HELLO_FLEET_PATH", str(tmp_path))
    monkeypatch.setenv("HELLO_FLEET_ID", fleet_id)
    monkeypatch.setattr(sbs, "_local_ipv4_addresses", lambda: ["127.0.0.1", "192.168.1.20"])
    return params_file


def _read(params_file):
    return yaml.safe_load(params_file.read_text())


def test_enable_all_interfaces(fleet):
    assert sbs.set_network_access('*') is True
    params = _read(fleet)
    assert params['robot']['server']['enable_network_access'] == '*'
    assert params['robot']['tool'] == 'eoa_test_tool'  # other settings kept


def test_enable_specific_address(fleet):
    assert sbs.set_network_access('192.168.1.20') is True
    assert _read(fleet)['robot']['server']['enable_network_access'] == '192.168.1.20'


def test_written_value_is_valid_yaml_string(fleet):
    sbs.set_network_access('*')
    assert "'*'" in fleet.read_text()  # a bare * would be a YAML alias


@pytest.mark.parametrize("address", ["not-an-ip", "300.1.1.1", "::1"])
def test_rejects_invalid_address(fleet, address):
    before = fleet.read_text()
    assert sbs.set_network_access(address) is False
    assert fleet.read_text() == before


def test_rejects_address_not_on_this_machine(fleet):
    before = fleet.read_text()
    assert sbs.set_network_access('10.9.9.9') is False
    assert fleet.read_text() == before


def test_disable_removes_setting_and_empty_sections(fleet):
    fleet.write_text("robot:\n  server:\n    enable_network_access: '*'\n")
    assert sbs.set_network_access(None) is True
    assert _read(fleet) in (None, {})


def test_disable_keeps_other_server_settings(fleet):
    fleet.write_text("robot:\n  server:\n    enable_network_access: '*'\n    subsystems:\n    - line_sensor_loop\n")
    assert sbs.set_network_access(None) is True
    assert _read(fleet) == {'robot': {'server': {'subsystems': ['line_sensor_loop']}}}


def test_disable_when_not_set_leaves_file_alone(fleet):
    before = fleet.read_text()
    assert sbs.set_network_access(None) is True
    assert fleet.read_text() == before


def test_missing_fleet_env_fails(monkeypatch):
    monkeypatch.delenv("HELLO_FLEET_PATH", raising=False)
    monkeypatch.delenv("HELLO_FLEET_ID", raising=False)
    assert sbs.set_network_access('*') is False
