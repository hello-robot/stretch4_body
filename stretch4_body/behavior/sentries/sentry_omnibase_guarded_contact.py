from stretch4_body.behavior.sentries.sentry import Sentry
from stretch4_body.core.hello_utils import *


# ######################################################

class SentryOmniBaseGuardedContact(Sentry):
    def __init__(self, robot):
        Sentry.__init__(self, name="sentry_omnibase_guarded_contact",robot=robot)
        self.status={'guarded_events': 0}
        self.last_guarded_event={}
        self.first=True


    def step(self):
        # Return true  a safety overrided issued
        #Flag whole base in guarded event if one wheel is
        if self.first:
            for w in self.robot.omnibase.wheels:
                self.last_guarded_event[w.name] = w.status['in_guarded_event']
            self.first=False

        new_contact=False
        pce=self.status['guarded_events']
        if not self.robot.omnibase.params['enable_guarded_mode'] :
            return False

        for w in self.robot.omnibase.wheels:
            if w.status['in_guarded_event'] != self.last_guarded_event[w.name]:
                #print('NEW',w.name,w.status['in_guarded_event'],self.last_guarded_event[w.name])
                new_contact = True
            self.last_guarded_event[w.name]=w.status['in_guarded_event']

        if new_contact:
            self.status['guarded_events']=self.status['guarded_events']+1
            self.robot.omnibase.enable_freewheel_mode() #Put in freewheel on guarded event
            self.logger.info('SentryOmniBaseGuardedContact: New guarded contact event ')

        return self.status['guarded_events'] != pce