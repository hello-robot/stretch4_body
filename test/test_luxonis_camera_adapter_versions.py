"""Camera versions written down by the code that opens the camera.

create_pipeline is the one place a camera is opened; it records what the
device reports in luxonis_device_versions_<uid>.json for
stretch_firmware_versions to read. Writing it must never stop a camera.
"""
import json

import pytest

dai = pytest.importorskip('depthai')

from stretch4_body.subsystem.cameras.adapters import luxonis_camera_adapter as lux


class FakeInfo:
    def getDeviceId(self):
        return '1844301011B546F500'


class FakeDevice:
    def getDeviceInfo(self):
        return FakeInfo()

    def getProductName(self):
        return 'OAK-D-WIDE'

    def getDeviceName(self):
        return 'OAK-D-W'

    def getBootloaderVersion(self):
        return dai.Version('0.0.28')

    def getUsbSpeed(self):
        return dai.UsbSpeed.SUPER


def test_record_has_every_version_field():
    rec = lux.device_version_record(FakeDevice(), '1.2.3', 100.0)
    assert rec == {'device_port': '1.2.3', 'time': 100.0, 'depthai_version': dai.__version__,
                   'device_id': '1844301011B546F500', 'product_name': 'OAK-D-WIDE',
                   'device_name': 'OAK-D-W', 'bootloader_version': '0.0.28',
                   'usb_speed': 'SUPER'}


def test_a_getter_that_fails_is_noted_not_raised():
    class NoBootloader(FakeDevice):
        def getBootloaderVersion(self):
            raise RuntimeError('not supported')

    class NoneBootloader(FakeDevice):
        def getBootloaderVersion(self):
            return None

    rec = lux.device_version_record(NoBootloader(), '1.2.3', 100.0)
    assert rec['bootloader_version'] is None
    assert rec['errors'] == {'bootloader_version': 'not supported'}
    assert lux.device_version_record(NoneBootloader(), 'p', 1.0)['bootloader_version'] is None


def test_records_merge_by_device_id(tmp_path, monkeypatch):
    path = tmp_path / 'versions.json'
    monkeypatch.setattr(lux, 'VERSIONS_FILE_PATH', str(path))

    class Other(FakeDevice):
        def getDeviceInfo(self):
            class I:
                def getDeviceId(self):
                    return '19443010A1B2C3D400'
            return I()

    lux.record_device_versions(FakeDevice(), '1.2.3')
    lux.record_device_versions(Other(), '1.2.4')
    lux.record_device_versions(FakeDevice(), '1.2.3')
    data = json.loads(path.read_text())
    assert sorted(data) == ['1844301011B546F500', '19443010A1B2C3D400']
    assert data['19443010A1B2C3D400']['device_port'] == '1.2.4'


def test_an_unwritable_file_does_not_raise(tmp_path, monkeypatch):
    monkeypatch.setattr(lux, 'VERSIONS_FILE_PATH', str(tmp_path / 'missing' / 'v.json'))
    lux.record_device_versions(FakeDevice(), '1.2.3')


def test_a_corrupt_file_is_replaced(tmp_path, monkeypatch):
    path = tmp_path / 'versions.json'
    path.write_text('{not json')
    monkeypatch.setattr(lux, 'VERSIONS_FILE_PATH', str(path))
    lux.record_device_versions(FakeDevice(), '1.2.3')
    assert list(json.loads(path.read_text())) == ['1844301011B546F500']
