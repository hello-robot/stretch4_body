from stretch4_body.behavior.sentries.sentry import Sentry
from stretch4_body.subsystem.power_periph import PowerPeriphDefn
import time
import random

class SentryEyeAnimations(Sentry):
    def __init__(self, robot):
        Sentry.__init__(self, name="sentry_eye_animations", robot=robot)
        self.next_action_time = time.time()
        self.behavior = self.params.get('behavior', 'curious')
        self.status={'active_behavior':'curious'}

    def startup(self):
        if not Sentry.startup(self):
            return False
            
        if self.robot.power_periph and self.robot.power_periph.board_info['protocol_version'] is not None:
            if int(self.robot.power_periph.board_info['protocol_version'][1:]) < 13:
                self.logger.warning(f"Sentry {self.name} disabled. PowerPeriph protocol is below 13.")
                self.is_valid = False
                return False
                
        return True

    def step(self):
        if not self.is_valid:
            return

        ts = time.time()
        
        # Color change logic: white to light pink randomly every 10s or so
        if not hasattr(self, 'next_color_change_time'):
            self.next_color_change_time = ts + random.uniform(8.0, 12.0)
            self.current_rgb = (255, 255, 255)
            
        color_changed = False
        if ts > self.next_color_change_time:
            if random.random() > 0.5:
                self.current_rgb = (255, 255, 255) # white
            else:
                self.current_rgb = (255, 192, 203) # light pink
            self.next_color_change_time = ts + random.uniform(8.0, 12.0)
            color_changed = True

        # Dynamically fetch behavior in case configuration is changed
        self.behavior = self.params.get('behavior', 'curious')

        if self.behavior == 'curious':
            action_triggered = False
            if ts >= self.next_action_time:
                # Circle every 8s or so
                if not hasattr(self, 'next_circle_time'):
                    self.next_circle_time = ts + random.uniform(7.0, 9.0)

                # Look around every 5s or so
                if not hasattr(self, 'next_look_time'):
                    self.next_look_time = ts + random.uniform(4.0, 6.0)

                if ts >= self.next_circle_time:
                    anim = PowerPeriphDefn.EYE_ANIM_CIRCLE_CW
                    delay = 2.0
                    self.next_circle_time = ts + delay + random.uniform(6.0, 10.0)
                    
                    # Force color to medium pink
                    self.current_rgb = (255, 105, 180)
                    color_changed = True
                    # Push next general color change into the future so it doesn't interrupt the animation
                    self.next_color_change_time = ts + delay + random.uniform(8.0, 12.0)
                elif ts >= self.next_look_time:
                    anim = random.choice([
                        PowerPeriphDefn.EYE_ANIM_LEFT_HALF,
                        PowerPeriphDefn.EYE_ANIM_RIGHT_HALF,
                        PowerPeriphDefn.EYE_ANIM_TOP_HALF,
                        PowerPeriphDefn.EYE_ANIM_BOTTOM_HALF
                    ])
                    delay = random.uniform(1.0, 2.5)
                    self.next_look_time = ts + delay + random.uniform(3.0, 6.0)
                else:
                    p = random.random()
                    if p < 0.60:
                        anim = PowerPeriphDefn.EYE_ANIM_IDLE_GLOW
                        delay = random.uniform(2.0, 5.0)
                    elif p < 0.80:
                        anim = PowerPeriphDefn.EYE_ANIM_BLINK
                        delay = random.uniform(0.5, 1.5)
                    elif p < 0.95:
                        anim = PowerPeriphDefn.EYE_ANIM_HAPPY
                        delay = random.uniform(2.0, 4.0)
                    else:
                        anim = PowerPeriphDefn.EYE_ANIM_ALERT
                        delay = random.uniform(1.0, 2.0)
                
                self.current_anim = anim
                self.next_action_time = ts + delay
                action_triggered = True

            if not hasattr(self, 'current_anim'):
                self.current_anim = PowerPeriphDefn.EYE_ANIM_IDLE_GLOW
                action_triggered = True

            if action_triggered or color_changed:
                if self.robot.power_periph:
                    r, g, b = self.current_rgb
                    self.robot.power_periph.set_eye_animation(
                        self.current_anim, self.current_anim,
                        intensity=255, r=r, g=g, b=b
                    )
                    self.robot.power_periph.push_command()
