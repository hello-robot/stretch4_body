import pytest
import time
import logging
from stretch4_body.core.hello_utils import LoggerThrottleFilter
from stretch4_body.core.device import Device

def test_logger_throttle():
    logger = logging.getLogger("test_throttle")
    filter = LoggerThrottleFilter("test_throttle")
    logger.addFilter(filter)
    
    # We need a custom handler to capture logs
    class ListHandler(logging.Handler):
        def __init__(self):
            super().__init__()
            self.log_records = []
        def emit(self, record):
            self.log_records.append(record)
            
    handler = ListHandler()
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)
    
    def log_msg(msg):
        logger.info(msg, extra={'throttle_s': 1})

    log_msg("Message 1")
    log_msg("Message 2") # Should be filtered out
    
    assert len(handler.log_records) == 1
    assert handler.log_records[0].getMessage() == "Message 1"
    
    time.sleep(1.1)
    log_msg("Message 3") # Should be allowed
    assert len(handler.log_records) == 2
    assert handler.log_records[1].getMessage() == "Message 3"
    
    # Also test messages without throttle_s
    logger.info("Message 4")
    assert len(handler.log_records) == 3
    assert handler.log_records[2].getMessage() == "Message 4"

def test_device_filter_not_duplicated():
    dev1 = Device("test_dev_filter", req_params=False)
    logger = logging.getLogger("test_dev_filter")
    
    filters = [f for f in logger.filters if isinstance(f, LoggerThrottleFilter)]
    assert len(filters) == 1
    
    for _ in range(10):
        dev2 = Device("test_dev_filter", req_params=False)
        filters = [f for f in logger.filters if isinstance(f, LoggerThrottleFilter)]
        assert len(filters) == 1

    assert len(dev2.logger.filters) == 1
    
    # Confirm it actually prevents multiple filters of the same name
    assert len([f for f in logger.filters if f.name == "test_dev_filter"]) == 1


if __name__ == "__main__":
    pytest.main([__file__])