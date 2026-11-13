import os
import shutil
import tempfile

import pytest

# Off a robot there is no fleet directory, so run against a copy of the fixture robot.
if 'HELLO_FLEET_PATH' not in os.environ:
    fleet_path = tempfile.mkdtemp(prefix='stretch_fleet_')
    shutil.copytree(os.path.join(os.path.dirname(__file__), 'fixtures', 'stretch-se4-test'),
                    os.path.join(fleet_path, 'stretch-se4-test'))
    os.environ['HELLO_FLEET_PATH'] = fleet_path
    os.environ['HELLO_FLEET_ID'] = 'stretch-se4-test'


def pytest_addoption(parser):
    parser.addoption('--robot', action='store_true', help='also run the tests that need a robot')


def pytest_configure(config):
    config.addinivalue_line('markers', 'robot: needs robot hardware or a running stretch_body_server')


def pytest_collection_modifyitems(config, items):
    if config.getoption('--robot'):
        return
    skip = pytest.mark.skip(reason='needs a robot, run with --robot')
    for item in items:
        if item.get_closest_marker('robot'):
            item.add_marker(skip)
