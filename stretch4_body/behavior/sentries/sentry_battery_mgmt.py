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
        if soc < self.params['soc_shutdown_warning'] or vbatt<self.params['low_voltage_shutdown_warning']:
            if time.time() - self.status['ts_last_warning'] > self.params['alert_period_shutdown']:
                self.logger.info('Battery Management Sentry: Warning! Shutting down soon.')
                if self.params['enable_audio_alert']:
                    play_sound(get_sounds_dir() + '/shutdown_warning.wav')
                self.status['ts_last_warning'] = time.time()
        elif  soc < self.params['soc_low_battery_warning'] and not self.robot.power_periph.status['charger_is_charging']:
            if time.time() - self.status['ts_last_warning'] > self.params['alert_period_low_battery']:
                self.logger.info('Battery Management Sentry: Warning! Low battery. Please plug in the charger.')
                if self.params['enable_audio_alert']:
                    play_sound(get_sounds_dir() + '/low_battery_warning.wav')
                self.status['ts_last_warning'] = time.time()
