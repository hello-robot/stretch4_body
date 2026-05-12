from stretch4_body.behavior.sentries.sentry import Sentry
from stretch4_body.core.hello_utils import *
import time

# ######################################################

class SentryJointRunaway(Sentry):
    def __init__(self, robot):
        Sentry.__init__(self, name="sentry_joint_runaway",robot=robot)
        self.status={'runaway_events': []}
        self.last_guarded_event={}
        self.first=True

    def step(self):
        if self.robot.get_subsystem('lift') is not None:
            if abs(self.robot.lift.status['vel'])>self.params['lift_runaway_vel'] and self.robot.lift.motor.hw_valid:
                self.logger.warning('Runaway velocity of lift of %f m/s. Shutting down actuator!'%self.robot.lift.status['vel'])
                self.logger.warning('Stop the server with stretch_body_server --kill')
                self.logger.warning('Run REx_actuator_control --lift to re-enable joint')
                self.robot.power_periph.actuator_control('lift', enable=False)
                self.robot.lift.motor.hw_valid=False
                self.status['runaway_events'].append({'joint':'lift','vel':self.robot.lift.status['vel'],'timestamp':time.time()})

