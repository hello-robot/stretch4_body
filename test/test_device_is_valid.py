import pytest
from unittest.mock import patch

from stretch4_body.core.device import Device
from stretch4_body.core.subsystem_client import SubsystemClient
from stretch4_body.core.client_server import NotConnectedError

def test_device_is_valid():
    dev = Device('test_device', req_params=False)
    assert dev.is_valid is True
    
    dev.stop()
    assert dev.is_valid is False
    
    # load_rpc_results should return empty list if not valid
    assert dev.load_rpc_results() == []

@patch('stretch4_body.core.subsystem_client.StretchBodyClient')
def test_subsystem_client_is_valid(mock_client_class):
    # Mock the StretchBodyClient instance
    mock_client = mock_client_class.return_value
    mock_client.startup.return_value = True
    mock_client.server_connected = True
    
    # Use a name that exists in the nominal params to avoid exit
    sub = SubsystemClient("arm")
    # SubsystemClient initializes is_valid=False in its own __init__
    assert sub.is_valid is False
    
    # After startup
    sub.startup()
    assert sub.is_valid is True
    assert sub.connected is True
    
    # After stop
    sub.stop()
    assert sub.is_valid is False
    assert sub.connected is False
    
    # Verify that require_connection decorated methods now raise NotConnectedError
    with pytest.raises(NotConnectedError):
        sub.pull_status()
        
    with pytest.raises(NotConnectedError):
        sub.push_command()

if __name__ == '__main__':
    pytest.main([__file__])
