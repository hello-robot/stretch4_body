import time
from stretch4_body.behavior.sentries.sentry import Sentry
from stretch4_body.core.hello_utils import *


# ######################################################

class SentryBatteryMgmt(Sentry):
    def __init__(self, robot):
        Sentry.__init__(self, name="sentry_battery_mgmt",robot=robot)
        self.status={'ts_last_warning': 0}

    def step(self):
        soc = self.robot.power_periph.status['battery_soc']
        vbatt=self.robot.power_periph.status['voltage']
        is_charging = self.robot.power_periph.status['adapter_voltage_present']

        # Without valid BMS values (NaN) the SOC and voltage warnings can't run. A
        # lost BMS is a fault: on battery the power board shuts the robot down after
        # 60 s; on the adapter the robot keeps running.
        if not self.robot.power_periph.status.get('battery_valid', True):
            if self.robot.power_periph.status.get('battery_status') == 'BMS_LOST':
                if time.time() - self.status['ts_last_warning'] > self.params['alert_period_shutdown']:
                    if is_charging:
                        self.logger.warning('Battery Management Sentry: Warning! Battery not responding. Check the battery and its cable.')
                    else:
                        self.logger.warning('Battery Management Sentry: Warning! Battery not responding. Shutting down soon.')
                        if self.params['enable_audio_alert']:
                            play_sound(get_sounds_dir() + '/shutdown_warning.wav')
                    self.status['ts_last_warning'] = time.time()
            return

        if (soc < self.params['soc_shutdown_warning'] or vbatt < self.params['low_voltage_shutdown_warning']) and not is_charging:
            if time.time() - self.status['ts_last_warning'] > self.params['alert_period_shutdown']:
                self.logger.info('Battery Management Sentry: Warning! Shutting down soon.')
                if self.params['enable_audio_alert']:
                    play_sound(get_sounds_dir() + '/shutdown_warning.wav')
                self.status['ts_last_warning'] = time.time()
        elif  soc < self.params['soc_low_battery_warning'] and not is_charging:
            if time.time() - self.status['ts_last_warning'] > self.params['alert_period_low_battery']:
                self.logger.info('Battery Management Sentry: Warning! Low battery. Please plug in the charger.')
                if self.params['enable_audio_alert']:
                    play_sound(get_sounds_dir() + '/low_battery_warning.wav')
                self.status['ts_last_warning'] = time.time()
