import click
import os
import sys
import requests
from stretch4_body.core.factory.firmware_version import FirmwareVersion

class FirmwareAvailable():
    """
    Determine what firmware is available on GitHub
    """
    def __init__(self, use_device):
        self.use_device = use_device #True/False dict for each device name
        self.versions = {}
        for d in self.use_device:
            if self.use_device[d]:
                self.versions[d] = []  # List of available versions for that device
        self.__get_available_firmware_versions()

    def pretty_print(self):
        click.secho(' Currently Tagged Versions of Stretch Firmware on Release Branch '.center(110, '#'), fg="cyan",
                    bold=True)
        for device_name in self.versions:
            click.secho('---- %s ----' % device_name.upper(), fg="white", bold=True)
            for v in self.versions[device_name]:
                print(v)

    def __get_available_firmware_versions(self):
        print('Checking available firmware versions...', end='')
        url = "https://api.github.com/repos/hello-robot/stretch_firmware/releases/tags/All_Release_Binaries"
        try:
            response = requests.get(url)
            if response.status_code != 200:
                print('Failed to fetch from GitHub: Status %d' % response.status_code)
                return
            
            data = response.json()
            assets = data.get('assets', [])
    
            for item in assets:
                name = item['name']
                if name.endswith('.uf2'):
                    version_str = name.replace('.uf2', '').replace('_v', '.v')
                    v = FirmwareVersion(version_str)
                    if v.valid:
                        for device_name in self.versions:
                            #print('Checking',device_name,self.versions[device_name])
                            if (v.device in ['hello-stepper2', 'hello_stepper2'] and device_name in ['hello-motor-lift', 'hello-motor-arm',
                                                                          'hello-motor-left-wheel', 'hello-motor-right-wheel',
                                                                          'hello-motor-omni-0', 'hello-motor-omni-1',
                                                                          'hello-motor-omni-2']) or \
                                    (v.device in ['hello-pimu2', 'hello_pimu2'] and device_name == 'hello-power-periph') or \
                                    (v.device in ['hello-pixart-j3', 'hello_pixart_j3'] and device_name == 'hello-pixart-j3'):
                                self.versions[device_name].append(v)
                if name.endswith('.bin'):
                    version_str = name.replace('.bin', '').replace('_v', '.v')
                    v = FirmwareVersion(version_str)
                    if v.valid:
                        for device_name in self.versions:
                            #print('Checking',device_name,self.versions[device_name])
                            if (v.device in ['hello-esp', 'hello_esp'] and device_name == 'hello-esp32'):
                                self.versions[device_name].append(v)
            print(' Done.')
        except Exception as e:
            print('Failed to fetch from GitHub: %s' % str(e))

    def get_most_recent_version(self, device_name, supported_protocols):
        """
        For the device and supported protocol versions (eg, '['p0','p1']'), return the most recent version (type FirmwareVersion)
        """
        if len(self.versions[device_name]) == 0:
            return None
        recent = None
        if supported_protocols is not None:
            s = [int(x[1:]) for x in supported_protocols]
        else:
            class Everything(object):
                def __contains__(self, other):
                    return True
            s = Everything()
        supported_versions = []
        for v in self.versions[device_name]:
            if v.protocol in s:
                supported_versions.append(v)
        for sv in supported_versions:
            if recent is None or sv > recent:
                recent = sv
        return recent
