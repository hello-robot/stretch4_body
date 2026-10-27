import time
from stretch4_body.subsystem.power_periph import PowerPeriphPeriphControl
from stretch4_body.behavior.sentries.sentry_eye_animations import SentryEyeAnimations

class FakePowerPeriph(PowerPeriphPeriphControl):
    def __init__(self):
        self.eye_animation_count = 0
        self.pushes = 0
    def push_command(self):
        self.pushes += 1

class FakeRobot:
    def __init__(self):
        self.power_periph = FakePowerPeriph()

def test_eye_sentry_yields_to_other_eye_commands():
    robot = FakeRobot()
    sentry = SentryEyeAnimations(robot)
    sentry.is_valid = True
    sentry.step()
    assert robot.power_periph.pushes == 1

    robot.power_periph.set_eye_animation(1, 1)
    sentry.step()
    assert robot.power_periph.pushes == 1

    sentry.yield_until = time.time()
    sentry.step()
    assert robot.power_periph.pushes == 2
