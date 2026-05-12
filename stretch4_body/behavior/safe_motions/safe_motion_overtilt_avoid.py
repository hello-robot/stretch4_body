from stretch4_body.behavior.safe_motions.safe_motion import SafeMotion
from stretch4_body.core.hello_utils import *
import time
# ######################################################

class VisualizeOverTilt():
    def __init__(self, robot):
        from stretch4_body.core.rerun_plot import RRplot
        self.robot = robot
        self.rrplot = RRplot(name="OvertiltAvoidance", open_browser=False)
        self.rrplot.register(key="gravity_tilt", color_idx=0)
        self.rrplot.setup_blueprint(collapse_panels=False)


    def step(self):
        v=self.robot.power_periph.status['imu']['gravity_tilt']
        self.rrplot.log_scalar(key="gravity_tilt", value=rad_to_deg(v))

class SafeMotionOvertiltAvoid(SafeMotion):
    def __init__(self, robot):
        SafeMotion.__init__(self, name="safe_motion_overtilt_avoid", robot=robot)
        self.ts_last_gravity_tilt = time.time()
        if self.params['enable_rerun_viz']:
            self.vt=VisualizeOverTilt(self.robot)
        self.status={'in_overtilt': False}


    def step(self):
        # Return true  a safety overrided issued
        self.status['in_overtilt'] = False
        if self.params['enabled']:
            if self.params['enable_rerun_viz']:
                self.vt.step()
            t = rad_to_deg(self.robot.power_periph.imu.status['gravity_tilt'])
            if abs(t) > self.params['gravity_tilt_thresh_deg']['default']:
                self.status['in_overtilt']=True
                if time.time()-self.ts_last_gravity_tilt >self.params['alert_period']:
                    #self.robot.power_periph.trigger_beep()
                    self.logger.info('SafeMotionOvertiltAvoid triggered ')
                    if self.params['enable_audio_alert']:
                        play_sound(get_sounds_dir()+'/tilt_warning.wav')
                    self.ts_last_gravity_tilt=time.time()

                if 'omnibase' in self.robot.subsystems:
                    self.robot.subsystems['omnibase'].enable_freewheel_mode()
                if 'arm' in self.robot.subsystems:
                    self.robot.subsystems['arm'].enable_safety()
                if 'lift' in self.robot.subsystems:
                    self.robot.subsystems['lift'].enable_safety()

        return self.status['in_overtilt']

