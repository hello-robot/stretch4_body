import click
import stretch4_body.core.stepper

import stretch4_body.core.device
import stretch4_body.core.hello_utils
from stretch4_body.core.factory.firmware_version import FirmwareVersion

class FirmwareInstalled():
    """
    Pull the current installed firmware off the robot uCs
    Build config_info of form:
    {'hello-motor-arm': {'board_info': {'board_version': u'Stepper.Irma.V1',
       'firmware_version': u'Stepper.v0.0.1p1',
       'protocol_version': u'p1'},
      'installed_protocol_valid': True,
      'supported_protocols': ['p0', 'p1']}}
    """

    def __init__(self, use_device):
        """
        use_device has form of:
        {'hello-motor-lift': True, 'hello-motor-arm': True, 'hello-motor-right-wheel': True, 'hello-motor-left-wheel': True, 'hello-pimu': True, 'hello-wacc': True}

        config_info is a dict like:

        {'hello-motor-lift': {'board_info': {'board_variant': 'Stepper.0',
           'firmware_version': 'Stepper.v0.3.0p2',
           'protocol_version': 'p2',
           'hardware_id': 0},
          'supported_protocols': ['p0', 'p1', 'p2'],
          'installed_protocol_valid': True,
          'version': 'Stepper.v0.3.0p2'},...}
        """
        self.use_device = use_device
        self.config_info = {'hello-motor-lift': None, 'hello-motor-arm': None, 'hello-motor-omni-0': None,
                            'hello-motor-omni-1': None, 'hello-motor-omni-2': None, 'hello-power-periph': None,
                            'hello-pixart-j3': None, 'hello-esp32': None}
        print('Collecting information...')
        for device in self.use_device.keys():
            if self.use_device[device]:
                if device == 'hello-power-periph':
                    from stretch4_body.subsystem.power_periph import PowerPeriph
                    dd = PowerPeriph(backend=0)
                elif device == 'hello-esp32':
                    self.config_info[device] = self._esp32_config_info()
                    continue
                elif device == 'hello-pixart-j3':
                    self.config_info[device] = self._unknown_config_info()
                    continue
                else:
                    dd = stretch4_body.core.stepper.Stepper('/dev/' + device, backend=0)
                if not dd.startup():
                    click.secho('Unable to communicate with device %s'%device,fg="red", bold=True)
                    dd.stop()
                else:
                    if dd.board_info['firmware_version'] is not None:  # Was able to pull board info from device
                        self.config_info[device] = {}
                        self.config_info[device]['board_info'] = dd.board_info.copy()
                        try:
                            self.config_info[device]['supported_protocols'] = list(dd._supported_protocols.keys())
                        except AttributeError:
                            # Older versions of stretch body used a different represenation
                            if hasattr(dd, 'valid_firmware_protocol'):
                                self.config_info[device]['supported_protocols'] = [dd.valid_firmware_protocol]
                            else:
                                self.config_info[device]['supported_protocols'] = ['p0'] # Fallback or whatever is safe
                        self.config_info[device]['installed_protocol_valid'] = (
                                    dd.board_info['protocol_version'] in self.config_info[device]['supported_protocols'])
                        self.config_info[device]['version'] = FirmwareVersion(self.config_info[device]['board_info']['firmware_version'])
                    else:
                        self.config_info[device] = None
                    dd.stop()

    @staticmethod
    def _unknown_config_info(supported_protocols=None):
        """A device that is present but can't report its firmware version."""
        return {
            'board_info': {
                'board_version': 'Unknown',
                'firmware_version': 'Unknown',
                'protocol_version': 'Unknown',
                'hardware_id': 0
            },
            'supported_protocols': supported_protocols or ['Unknown'],
            'installed_protocol_valid': True,
            'version': FirmwareVersion('Unknown')
        }

    def _esp32_config_info(self):
        """
        Firmware v0.2.0p1 and later report their version over RPC. Older firmware has no
        RPC, so a present ESP32 that doesn't answer is reported as 'Unknown' (p0).
        """
        import os
        from stretch4_body.subsystem.esp32 import Esp32, ESP32_USB_NAME
        if not os.path.exists(ESP32_USB_NAME):
            return None
        dd = Esp32()
        try:
            if dd.startup() or dd.board_info['firmware_version'] is not None:
                info = {'board_info': dd.board_info.copy(),
                        'supported_protocols': list(dd.FIRMWARE_PROTOCOLS),
                        'installed_protocol_valid': dd.board_info['protocol_version'] in dd._supported_protocols,
                        'version': FirmwareVersion(dd.board_info['firmware_version'])}
                return info
        finally:
            dd.stop()
        return self._unknown_config_info(list(Esp32.FIRMWARE_PROTOCOLS))

    def is_version_known(self, device_name):
        """False for a device present on firmware that can't report its version (PixArt J3, ESP32 before RPC)."""
        return self.is_device_valid(device_name) and self.config_info[device_name]['version'].valid

    def get_supported_protocols(self, device_name):
        if self.is_device_valid(device_name):
            return self.config_info[device_name]['supported_protocols']
        return None

    def get_version(self, device_name):
        if self.is_device_valid(device_name):
            return self.config_info[device_name]['version']
        return None
    
    def get_hw_id(self, device_name):
        if self.is_device_valid(device_name):
            return (self.config_info[device_name]['board_info']['hardware_id'])
        return None

    def is_device_valid(self, device_name):
        return self.config_info[device_name] is not None

    def is_protocol_supported(self, device_name, p):
        """
        Provide 'p0', etc
        """
        return self.is_device_valid(device_name) and p in self.config_info[device_name]['supported_protocols']

    def max_protocol_supported(self, device_name):
        if not self.is_version_known(device_name):
            return 'p99'
        x = [int(x[1:]) for x in self.config_info[device_name]['supported_protocols']]
        return 'p' + str(max(x))

    def pretty_print(self):
        click.secho(' Currently Installed Firmware '.center(110, '#'), fg="cyan", bold=True)
        for device in self.config_info:
            if self.use_device[device]:
                click.secho('------------ %s ------------' % device.upper(), fg="white", bold=True)
                if self.config_info[device]:
                    if not self.is_version_known(device):
                        click.secho('Device is present, but current version is not available.', fg="yellow")
                    else:
                        click.echo('Installed Firmware: %s' % self.config_info[device]['board_info']['firmware_version'])
                        x = " , ".join(["{}"] * len(self.config_info[device]['supported_protocols'])).format(
                            *self.config_info[device]['supported_protocols'])
                        click.echo('Installed Stretch Body supports protocols: ' + x)
                        if self.config_info[device]['installed_protocol_valid']:
                            click.secho('Installed protocol %s : VALID' % self.config_info[device]['board_info'][
                                'protocol_version'])
                        else:
                            click.secho('Installed protocol %s : INVALID' % self.config_info[device]['board_info'][
                                'protocol_version'], fg="yellow")
                else:
                    click.secho('Device not found')
