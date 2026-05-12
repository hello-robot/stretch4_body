import click
import os
from subprocess import Popen, PIPE, call, DEVNULL
import stretch4_body.core.stepper
import stretch4_body.core.device
import yaml
import time
import requests

import stretch4_body.core.hello_utils
from stretch4_body.core.factory.firmware_available import FirmwareAvailable
from stretch4_body.core.factory.firmware_recommended import FirmwareRecommended
from stretch4_body.core.factory.firmware_installed import FirmwareInstalled
from stretch4_body.core.factory.firmware_version import FirmwareVersion
import stretch4_body.core.factory.firmware_utils as fwu
import stretch4_body.core.factory.hello_device_utils as hdu



class FirmwareUpdater():
    def __init__(self, use_device,args):
        self.ready_to_run = False
        self.args = args
        self.home_dir = os.path.expanduser('~')
        self.stepper_type = None

        self.state = {}
        self.state['use_device']=use_device
        self.state['verbose'] = args.verbose
        self.state['no_prompts'] = args.no_prompts
        self.state['install_version'] = args.install_version

        #Check that all devices targeted can be updated
        self.fw_installed = FirmwareInstalled(self.state['use_device'])
        all_valid=True
        for d in self.state['use_device']:
            if self.state['use_device'][d] and not self.fw_installed.is_device_valid(d):
                click.secho('WARNING: Device %s is not valid. Unable to attempt the firmware update. Skipping device.'%d, fg="yellow", bold=True)
                self.state['use_device'][d]=False
                all_valid=False

        self.fw_available = FirmwareAvailable(self.state['use_device'])
        self.fw_recommended = FirmwareRecommended(self.state['use_device'], self.fw_installed, self.fw_available)

        self.ready_to_run = fwu.check_arduino_cli_install(self.state['no_prompts'])

        # Set the target version to flash to recommended for each device
        #This dict has a FirmwareVersion target for each device that has a valid (and desired) update
        self.target = {}
        self.state['repo_path']=None

        #Default target is recommended
        for d in self.state['use_device']:
            if self.state['use_device'][d]:
                if d in self.fw_recommended.recommended and self.fw_recommended.recommended[d] is not None:
                    self.target[d] = self.fw_recommended.recommended[d]

        if args.install_version:
            self.ready_to_run = self.ready_to_run and self.set_target_from_install_version()

        # Count how many updates doing
        num_update = 0
        for device_name in self.target:
            if self.fw_installed.is_device_valid(device_name):
                num_update = num_update + 1
        if self.ready_to_run:
            self.pretty_print_target()
        if not num_update:
            click.secho('No updates to be done', fg="yellow", bold=True)
            self.ready_to_run=False
        #At this point self.state dictionary has all information needed to run an update cycle

# ########################################################################################################3

# ########################################################################################################3


    def run(self):
        if not self.ready_to_run:
            click.secho('WARNING: Unable to complete firmware update...', fg="yellow", bold=True)
            return False

        self.print_upload_warning()

        #First check that all calibration present
        for device_name in self.target:
            sketch_name = fwu.get_sketch_name(device_name)
            if sketch_name == 'hello_stepper2' and not fwu.does_stepper_have_encoder_calibration_YAML(device_name):
                print('Encoder data has not been stored for %s and should be stored first.' % device_name)
                print('First run REx_stepper_calibration_flash_to_YAML '+device_name)
                print('Aborting firmware flash.')
                return False

        #self.pretty_print_state()
        #Advance the state machine
        if self.state['no_prompts'] or click.confirm('Proceed with update??'):
            if not getattr(self.state, 'dummy', False) and not getattr(self.args, 'dummy', False):
                call('sudo echo', shell=True)
            print('\n\n\n')
            #Flash all devices
            for d in self.target:
                click.secho(' %s  '.center(110, '#') % d.upper(), fg="yellow", bold=True)
                click.secho(' %s |  FLASH FIRMWARE... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if self.fw_installed.is_device_valid(d):
                    nretry=3
                    for i in range(nretry):
                        compile_fail, upload_success=self.do_device_flash(d,self.target[d].to_string(),self.state['repo_path'],self.state['verbose'])
                        flash_success = not compile_fail and upload_success
                        if flash_success:
                            break
                        if not upload_success: #Dont retry if compile failure
                            click.secho('WARNING: Failed firmware flash for %s'%d, fg='red', bold=True)
                            break
                else:
                    click.secho('WARNING: Unable to flash %s as device not valid'%d, fg="yellow", bold=True)
                    flash_success = False

                if not flash_success:
                    click.secho('WARNING: Device %s did not flash firmware successfully'%d, fg="red", bold=True)
                    click.secho('WARNING: Power cycle robot and try again.', fg="red", bold=True)
                    return False

                click.secho(' %s |   CHECK #1 IF DEVICE RETURNS TO BUS... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if not self.wait_on_return_to_bus(d):
                    click.secho('WARNING: Device %s did not return to bus successfully'%d, fg="red", bold=True)
                    click.secho('WARNING: Power cycle robot and try again.', fg="red", bold=True)
                    return False

                time.sleep(3.0) #Give a chance for devices to become ready for comms

                click.secho(' %s |   CHECK IF ESTABLISH COMMS... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if not self.verify_establish_comms(d):
                    click.secho('WARNING: Device %s did not establish comms successfully'%d, fg="red", bold=True)
                    click.secho('WARNING: Power cycle robot and try again.', fg="red", bold=True)
                    return False

                click.secho('%s |  CHECK FOR CORRECT VERSION UPDATE... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if not self.verify_firmware_version(d):
                    click.secho('WARNING: Device %s has not updated to target firmware version'%d, fg="red", bold=True)
                    click.secho('WARNING: Power cycle robot and try again.', fg="red", bold=True)
                    return False

                click.secho('%s |  RESTORING CALIBRATION DATA... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if not self.flash_stepper_calibration(d):
                    click.secho('WARNING: Device %s failed on encoder calibration flash'%d, fg="red", bold=True)
                    click.secho('WARNING: Power cycle robot and try again.', fg="red", bold=True)
                    return False

                click.secho('%s |  CHECK #2 IF RETURNED TO BUS... '.center(110, '#')%d.upper(), fg="cyan", bold=True)
                if not self.wait_on_return_to_bus(d):
                    click.secho('WARNING: Device %s did not return to bus successfully'%d, fg="red", bold=True)
                    click.secho('WARNING: Power cycle robot and try again.', fg="red", bold=True)
                    return False
                print('\n\n\n')

            print('')
            click.secho(' CONGRATULATIONS... '.center(110, '#'), fg="cyan", bold=True)
            for d in self.target:
                click.secho('%s | No issues encountered. Firmware updated to %s.'%(d.upper().ljust(25),str(self.target[d])), fg="green", bold=True)
            return True

    # ########################################################################################################3

    def all_completed(self,state_name):
        all_completed=True
        for d in self.target:
            all_completed=all_completed and self.state['completed'][d][state_name]
        return all_completed
    
    def extract_stepper_type(self, device_name):
                if 'hello-motor' in device_name:
                    st = stretch4_body.core.stepper.Stepper('/dev/' + device_name, backend=0)
                    for i in st._supported_protocols.keys():
                        recent_protocol = i.strip('p')
                    if int(recent_protocol) >= 5:
                        if not st.startup():
                            click.secho('FAIL: Unable to establish comms with device %s' % device_name.upper(), fg="red")
                            return False
                        else:
                            if int(st.board_info['protocol_version'].strip('p')) >= 5:
                                self.stepper_type = st.board_info['stepper_type']
                                time.sleep(0.5)
                            st.stop()
                            del st

# ########################################################################################################################
    def do_device_flash(self, device_name, tag, repo_path=None, verbose=False, port_name=None):
        """
        Return compile_fail, upload_success (False, True over UF2 API for upload_success)
        """
        sketch_name=fwu.get_sketch_name(device_name)
            
        dummy = getattr(self.args, 'dummy', False) or getattr(self.state, 'dummy', False)
        if dummy:
            port_name = 'ttyACMDUMMY'
            
        if port_name is None:
            print('Looking for device %s on bus' % device_name)
            if not fwu.wait_on_device(device_name, timeout=5.0):
                print('Failure: Device not on bus.')
                return False, False
            port_name = fwu.get_port_name(device_name)

        if not dummy:
            self.extract_stepper_type(device_name)
        fwu.user_msg_log('Device: %s Port: %s' % (device_name, port_name), user_display=verbose)

        if (port_name is not None or dummy) and sketch_name is not None:
            success = fwu.flash_firmware_update(device_name, tag, f"/dev/{port_name}" if port_name else "/dev/ttyACM0", verbose, dummy=dummy)
            if not success:
                return False, False
            return False, True
        else:
            print('Firmware update %s. Failed to find device %s' % (tag, device_name))
            return False, False

# ########################################################################################################3
    def wait_on_return_to_bus(self,device_name):
        dummy = getattr(self.args, 'dummy', False) or getattr(self.state, 'dummy', False)
        if dummy:
            click.secho(f'Dummy mode enabled. Bypassing wait for {device_name} returning to bus.', fg="yellow")
            return True
            
        click.secho('Checking that device %s returned to bus '%device_name)
        print('It may take several minutes to appear on the USB bus.' )
        ts = time.time()
        found = False
        ntry=30
        for i in range(ntry):
            if not fwu.wait_on_device(device_name, timeout=10.0):
                print('Trying again: %d of %d\n' % (i,ntry))
                # Bit of a hack.Sometimes after a firmware flash the device
                # Doesn't fully present on the USB bus with a serial No for Udev to find
                # In does present as an 'Arduino Zero' product. This will attempt to reset it
                # and re-present to the bus
                time.sleep(1.0)
                click.secho(f'Resetting usb of {device_name} please wait a few seconds', fg="yellow", bold = False)
                call('sudo usbreset \"Arduino Zero\"', shell=True, stdout=DEVNULL)
                time.sleep(2.0)
            else:
                found = True
                break
        if not found:
            click.secho('Device %s failed to return to bus after %f seconds.' % (device_name, time.time() - ts),fg="yellow", bold=True)
            return False
        else:
            click.secho('Device %s returned to bus after %f seconds.' % (device_name, time.time() - ts),fg="green", bold=True)
        return True
# ########################################################################################################3
    def verify_firmware_version(self,device_name):
        if getattr(self.args, 'dummy', False) or getattr(self.state, 'dummy', False):
            click.secho(f'Dummy mode enabled. Bypassing {device_name} firmware version verification.', fg="yellow")
            return True

        if device_name in ['hello-pixart-j3', 'hello-esp32']:
            click.secho(f'PASS: Established firmware verficiation with device {device_name.upper()}', fg="green")
            return True

        fw_installed = FirmwareInstalled({device_name: True})  # Pull the currently installed system from fw
        if not fw_installed.is_device_valid(device_name):  # Device may not have come back on bus
            print('%s | No device available' % device_name.upper().ljust(25))
            print('')
            return False
        else:
            #click.secho(' Confirming Firmware Updates '.center(110, '#'), fg="cyan", bold=True)
            v_curr = fw_installed.get_version(device_name)  # Version that is now on the board
            if v_curr == self.target[device_name]:
                click.secho('PASS: %s | Installed %s | Target %s ' % (device_name.upper().ljust(25), v_curr.to_string().ljust(40),self.target[device_name].to_string().ljust(40)), fg="green")
                return True
            else:
                click.secho('FAIL: %s | Installed %s | Target %s ' % (device_name.upper().ljust(25), v_curr.to_string().ljust(40), self.target[device_name].to_string().ljust(40)),fg="red")
        return False

    def verify_establish_comms(self,device_name):
        if device_name == 'hello-power-periph':
            from stretch4_body.subsystem.power_periph import PowerPeriph
            dd = PowerPeriph(backend=0)
            if not dd.startup():
                click.secho('FAIL: Unable to establish comms with device %s' % device_name.upper(), fg="red")
                dd.stop()
                return False
            dd.pull_status()
            if dd.status['voltage']<24.0:
                click.secho('FAIL: Power Periph voltage is %f' % dd.status['voltage'], fg="red")
                dd.stop()
                return False
            click.secho('PASS: Power Periph voltage is %f' % dd.status['voltage'], fg="green")
            click.secho('PASS: Established comms with device %s ' % device_name.upper(),fg="green")
            dd.stop()
            return True 
        elif device_name == 'hello-pixart-j3':
            from stretch4_body.subsystem.line_sensor.pixart_j3_reader import PixartJ3Reader
            click.secho(f"Reading Pixart J3 sensor rates for 1s to verify comms...")
            pjr = PixartJ3Reader(port_name='/dev/hello-pixart-j3', verbose=False)
            if not pjr.startup():
                click.secho('FAIL: Unable to establish comms with device %s' % device_name.upper(), fg="red")
                return False
            
            t_start = time.time()
            while time.time() - t_start < 1.0:
                pjr.step()
                time.sleep(0.005)
            
            all_good = True
            for i in range(6):
                rate = pjr.status[f'sensor_{i}']['rate_hz']
                print('Frame rate from hello-pixart-j3:  sensor %d is %f' % (i, rate))
                if rate < 20.0:
                    click.secho(f"WARNING: {device_name.upper()} Sensor {i} rate is {rate:.1f} Hz (Expected > 20 Hz)", fg="yellow")
                    all_good = False
            
            pjr.stop()
            
            if not all_good:
                click.secho('Warning: Device %s sensor rates are below expected thresholds' % device_name.upper(), fg="yellow")
                click.secho('Warning: This may be resolved by power cycling device %s' % device_name.upper(),fg="yellow")
                return True
                
            click.secho('PASS: Established comms with device %s ' % device_name.upper(),fg="green")
            return True
        elif device_name == 'hello-esp32':
            return True
        else:
            dd = stretch4_body.core.stepper.Stepper('/dev/' + device_name, backend=0)
        if not dd.startup():
            click.secho('FAIL: Unable to establish comms with device %s' % device_name.upper(), fg="red")
            return False
        else:
            time.sleep(0.5)
            dd.stop()
            del dd
        click.secho('PASS: Established comms with device %s ' % device_name.upper(),fg="green")
        return True
# ########################################################################################################3
    def flash_stepper_calibration(self, device_name):
        if getattr(self.args, 'dummy', False) or getattr(self.state, 'dummy', False):
            click.secho(f'Dummy mode enabled. Bypassing {device_name} stepper calibration flash.', fg="yellow")
            return True

        if device_name in ['hello-motor-arm', 'hello-motor-lift', 'hello-motor-omni-0', 'hello-motor-omni-1', 'hello-motor-omni-2']:
            #click.secho(' Flashing Stepper Calibration: %s '.center(70, '#') % device_name, fg="cyan", bold=True)
            if not fwu.wait_on_device(device_name):
                click.secho('Device %s failed to return to bus.' % device_name, fg="red", bold=True)
                return False
            #time.sleep(1.0)
            motor = stretch4_body.core.stepper.Stepper('/dev/' + device_name, backend=0)
            motor.startup()
            if not motor.hw_valid:
                click.secho('Failed to startup stepper %s' % device_name, fg="red", bold=True)
                return False
            else:
                print('Writing gains to flash...')
                motor.write_gains_to_flash()
                motor.push_command()
                print('Gains written to flash')
                print('')
                print('Reading calibration data from YAML...')
                data = motor.read_encoder_calibration_from_YAML()
                print('Writing calibration data to flash...')
                motor.write_encoder_calibration_to_flash(data)
                print('\n')

                if int(motor.board_info['protocol_version'].strip('p')) >= 5 and self.stepper_type is not None:
                    print('Writing stepper type to flash...')
                    motor.write_stepper_type_to_flash(self.stepper_type)
                    print('Success writing stepper type to Flash')
                    print('\n')

                print('Successful write of FLASH.')
                fwu.wait_on_device(device_name)
                motor.board_reset()
                motor.push_command()
                motor.transport.ser.close()
                time.sleep(2.0) #Give time to return to bus
                return True
        click.secho('Successful flash of device calibration',fg="green")
        return True
# ########################################################################################################3

    def set_target_from_install_version(self):
        # Return True if system was upgraded
        # Return False if systvt = None
        #                     while vt == None:
        #                         id = click.prompt('Please enter desired version id [Recommended]', default=default_id)
        #                         if id >= 0 and id < len(vs):
        #                             vt = vs[id]
        #                         else:
        #                             click.secho('Invalid ID', fg="red")
        #                     print('Selected version %s for device %s' % (vt, device_name))em was not upgraded / error happened
        click.secho(' Select target firmware versions '.center(60, '#'), fg="cyan", bold=True)
        for device_name in self.fw_recommended.recommended.keys():
            if self.state['use_device'][device_name]:
                vs = self.fw_available.versions[device_name]
                if len(vs) and self.fw_recommended.recommended[device_name] is not None:
                    print('')
                    click.secho('---------- %s [%s]-----------' % (
                    device_name.upper(), str(self.fw_installed.get_version(device_name))), fg="blue", bold=True)
                    default_id = 0
                    self.min_allowed_fw_version = {
                        0: '0.3.1p2',
                        1: '0.3.1p2',
                        2: '0.3.1p2',
                        3: '0.7.0p5',
                        4: '0.7.0p5',
                    }
                    ## Checks to hw id to ensure that user can not downgrade fw to far
                    for f_limit in range(len(vs)):
                        if device_name == 'hello-pixart-j3':
                            f_limit = 0
                            break
                        fw_limit = self.min_allowed_fw_version.get(self.fw_installed.get_hw_id(device_name), None)
                        if fw_limit is None:
                            raise ValueError(f'Hardware ID for {device_name.upper()} Exceeds Mapped Version Please Contact Hello Robot Support') # exit out with error message asking user to contact Hello Robot Support
                        fw_version = str(vs[f_limit])
                        fw_version = fw_version[fw_version.index('v') + 1:]
                        if fw_version == fw_limit:
                            break

                    for i in range(f_limit, len(vs)):
                        if vs[i] == self.fw_recommended.recommended[device_name]:
                            default_id = i-f_limit
                        print('%d: %s' % (i-f_limit, vs[i]))


                    valid_id = True
                    while valid_id:
                        id = click.prompt('Please enter desired version id [Recommended]', default=default_id)
                        if id >= 0 and id < len(vs) - f_limit:
                            vt = vs[id+f_limit]
                            valid_id = False
                        else:
                            click.secho('Invalid ID Try Again', fg="red")
                print('Selected version %s for device %s' % (vt, device_name))
                self.target[device_name] = vt

                target_version = vt
                if target_version is None:
                    return False
                self.target[device_name] = target_version
                path_protocol = 'p' + str(target_version.protocol)
                if device_name not in ['hello-pixart-j3', 'hello-esp32'] and not self.fw_installed.is_protocol_supported(device_name, path_protocol):
                    click.secho('---------------------------', fg="yellow")
                    click.secho(
                        'Target firmware path of %s is incompatible with installed Stretch Body for device %s' % (
                        target_version, device_name), fg="yellow")
                    x = " , ".join(["{}"] * len(self.fw_installed.get_supported_protocols(device_name))).format(
                        *self.fw_installed.get_supported_protocols(device_name))
                    click.secho('Installed Stretch Body supports protocols %s' % x, fg="yellow")
                    click.secho('Target path supports protocol %s' % path_protocol, fg="yellow")
                    if path_protocol > self.fw_installed.max_protocol_supported(device_name):
                        click.secho('Upgrade Stretch Body first...', fg="yellow")
                    else:
                        click.secho('Downgrade Stretch Body first...', fg="yellow")
                    return False

        print('')
        print('')
        return True



# ########################################################################################################################



    def pretty_print_target(self):
        click.secho(' UPDATING FIRMWARE TO... '.center(110, '#'), fg="cyan", bold=True)
        for device_name in self.target:
            if self.state['use_device'][device_name]:
                if not self.fw_installed.is_device_valid(device_name):
                    print('%s | No target available' % device_name.upper().ljust(25))
                else:
                    v_curr = self.fw_installed.get_version(device_name)
                    v_targ = self.target[device_name]
                    if v_targ is None:
                        rec = 'No target available'
                    elif v_curr > v_targ:
                        rec = 'Downgrading to %s' % self.target[device_name]
                    elif v_curr < v_targ:
                        rec = 'Upgrading to %s' % self.target[device_name]
                    else:
                        rec = 'Reinstalling %s' % self.target[device_name]
                    print('%s | %s ' % (device_name.upper().ljust(25), rec.ljust(40)))
        print('')

    def print_upload_warning(self):
        click.secho('------------------------------------------------', fg="yellow", bold=True)
        click.secho('WARNING: (1) Updating robot firmware should only be done by experienced users', fg="yellow",
                    bold=True)
        click.secho('WARNING: (2) Do not have other robot processes running during update', fg="yellow", bold=True)
        click.secho('WARNING: (3) Leave robot powered on during update', fg="yellow", bold=True)
        if self.state['use_device']['hello-motor-lift']:
            click.secho('WARNING: (4) Ensure Lift has support clamp in place', fg="yellow", bold=True)
            click.secho('WARNING: (5) Lift may make a loud noise during programming. This is normal.', fg="yellow",
                        bold=True)
        click.secho('------------------------------------------------', fg="yellow", bold=True)








