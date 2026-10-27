import time
from stretch4_body.subsystem.power_periph import PowerPeriphPeriphControl, PowerPeriphDefn
from stretch4_body.behavior.sentries.sentry_eye_animations import SentryEyeAnimations

class FakePowerPeriph(PowerPeriphPeriphControl):
    def __init__(self):
        self.eye_animation_count = 0
        self._trigger = 0
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

def test_eye_sentry_default_is_a_purple_spin():
    robot = FakeRobot()
    sentry = SentryEyeAnimations(robot)
    sentry.is_valid = True
    sentry.step()
    pp = robot.power_periph
    assert (pp._eye_animation_left, pp._eye_animation_right) == (13, 13)
    assert (pp._eye_animation_r, pp._eye_animation_g, pp._eye_animation_b) == (140, 0, 255)

def test_eye_sentry_hands_the_eyes_over_again_after_a_wake():
    robot = FakeRobot()
    sentry = SentryEyeAnimations(robot)
    sentry.is_valid = True
    sentry.step()
    assert robot.power_periph.pushes == 1

    robot.power_periph.power_diagnostics = {'eye_left': {'animation': PowerPeriphDefn.EYE_ANIM_BOOT_PROGRESS}}
    sentry.step()
    assert robot.power_periph.pushes == 1

    sentry.boot_step_ts -= 4.0
    robot.power_periph._trigger = 0
    sentry.step()
    assert robot.power_periph.pushes == 2
    assert (robot.power_periph._trigger >> 27) & 0xF == PowerPeriphDefn.BOOT_STEP_EYES
