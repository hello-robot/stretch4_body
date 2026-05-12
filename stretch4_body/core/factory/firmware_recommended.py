
import click
from stretch4_body.core.factory.firmware_installed import FirmwareInstalled
from stretch4_body.core.factory.firmware_available import FirmwareAvailable


class FirmwareRecommended():
    """
    Recommend a firmware version to upgrade to based on available / installed
    """
    def __init__(self, use_device, installed=None, available=None):
        print('Collecting information...')
        self.use_device = use_device
        self.fw_installed = FirmwareInstalled(use_device) if installed is None else installed
        self.fw_available = FirmwareAvailable(use_device) if available is None else available
        self.recommended = {}
        self.__get_recommend_updates()

    def __get_recommend_updates(self):
        for device_name in self.use_device.keys():
            if self.use_device[device_name]:
                if self.fw_installed.is_device_valid(device_name):  # Len 0 if device not found
                    supported = self.fw_installed.get_supported_protocols(device_name)
                    if device_name in ['hello-pixart-j3', 'hello-esp32']:
                        supported = None
                    self.recommended[device_name] = self.fw_available.get_most_recent_version(device_name, supported)
                else:
                    self.recommended[device_name] = None

    def pretty_print(self):
        click.secho(' Recommended Firmware Updates '.center(110, '#'), fg="cyan", bold=True)
        print('\n')
        click.secho('%s | %s | %s | %s ' % (
        'DEVICE'.ljust(25), 'INSTALLED'.ljust(25), 'RECOMMENDED'.ljust(25), 'ACTION'.ljust(25)), fg="cyan", bold=True)
        click.secho('-' * 110, fg="cyan", bold=True)

        for device_name in self.recommended: #.keys():
            dev_out = device_name.upper().ljust(25)
            installed_out = ''.ljust(25)
            rec_out = ''.ljust(25)
            action_out = ''.ljust(25)
            if not self.fw_installed.is_device_valid(device_name):
                installed_out = 'No device available'.ljust(25)
            else:
                version = self.fw_installed.get_version(device_name)
                if device_name in ['hello-pixart-j3', 'hello-esp32']:
                    installed_out = 'Unknown'.ljust(25)
                else:
                    installed_out = str(version).ljust(25)
                if self.recommended[device_name] == None:
                    rec_out = 'None (might be on dev branch)'.ljust(25)
                else:
                    rec_out = str(self.recommended[device_name]).ljust(25)
                    if device_name in ['hello-pixart-j3', 'hello-esp32']:
                        action_out = 'Firmware available'.ljust(25)
                    elif self.recommended[device_name] > version:
                        action_out = 'Upgrade recommended'.ljust(25)
                    elif self.recommended[device_name] < version:
                        action_out = 'Downgrade recommended'.ljust(25)
                    else:
                        action_out = 'At most recent version'.ljust(25)
            print('%s | %s | %s | %s ' % (dev_out, installed_out, rec_out, action_out))

    def print_recommended_args(self):
        dev_arg_map = {'hello-power-periph': ' --pimu', 'hello-motor-omni-0': ' --wheel_0',
                       'hello-motor-omni-1': ' --wheel_1', 'hello-motor-omni-2': ' --wheel_2',
                       'hello-motor-lift': ' --lift', 'hello-motor-arm': ' --arm',
                       'hello-pixart-j3': ' --pixart', 'hello-esp32': ' --esp32'}
        rec_args = ''
        for device_name in self.recommended: #.keys():
            if self.fw_installed.is_device_valid(device_name):
                version = self.fw_installed.get_version(device_name)
                if self.recommended[device_name] != None:
                    if device_name in ['hello-pixart-j3', 'hello-esp32'] or self.recommended[device_name] > version:
                        rec_args = rec_args + dev_arg_map[device_name]
        if len(rec_args):
            click.secho('\nRun recommended command: \nREx_firmware_updater --install %s' % rec_args, fg="green",
                        bold=True)
        else:
            click.secho('\nFirmware upgrade not necessary', fg="green", bold=True)

