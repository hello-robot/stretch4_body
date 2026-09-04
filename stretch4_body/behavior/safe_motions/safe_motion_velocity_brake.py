from stretch4_body.behavior.safe_motions.safe_motion import SafeMotion
from stretch4_body.core.hello_utils import *
import time

# ######################################################


class SafeMotionVelocityBrake(SafeMotion):
    """
    Monitor joints in velocity mode using the safe_velocity logic, which anticipates collisions with the hardstop.
    Override the command velocity with a safe velocity.
    """

    def __init__(self, robot):
        SafeMotion.__init__(self, name="safe_motion_velocity_brake", robot=robot)
        self.status = {
            'lift': {'in_vel_brake_zone': False, 'in_vel_mode': False},
            'arm': {'in_vel_brake_zone': False, 'in_vel_mode': False},
            'end_of_arm': {'in_vel_brake_zone': False, 'in_vel_mode': False}
        }
        self.t_start = time.time()

    def step(self):
        safe_motion_triggered = False

        if self.robot.get_subsystem('lift') is not None:
            self.status['lift']['in_vel_brake_zone'] = False
            self.status['lift']['in_vel_mode'] = self.robot.lift.in_vel_mode
            if self.robot.lift.in_vel_mode:
                v_curr = self.robot.lift.status['vel']
                v_allowed = self.robot.lift.get_safe_velocity(v_curr,v_deadband=.0005, pad_m=0.003)
                if v_curr != v_allowed:
                    self.robot.lift.set_velocity(v_allowed)  # Override
                    self.status['lift']['in_vel_brake_zone'] = True
                    safe_motion_triggered = True

        if self.robot.get_subsystem('arm') is not None:
            self.status['arm']['in_vel_brake_zone'] = False
            self.status['arm']['in_vel_mode'] = self.robot.arm.in_vel_mode
            if self.robot.arm.in_vel_mode:
                v_curr = self.robot.arm.status['vel']
                v_allowed = self.robot.arm.get_safe_velocity(v_curr,v_deadband=.0005, pad_m=0.003)
                if v_curr != v_allowed:
                    self.robot.arm.set_velocity(v_allowed)
                    self.status['arm']['in_vel_brake_zone'] = True
                    safe_motion_triggered = True

        if self.robot.get_subsystem('end_of_arm') is not None:
            self.robot.end_of_arm.step_safe_motion_velocity_brake(self.robot.status)
            if 'safe_motion_velocity_brake' in self.robot.status['end_of_arm']:
                self.status['end_of_arm']['safe_motion_velocity_brake'] = self.robot.status['end_of_arm']['safe_motion_velocity_brake']
                for j in self.robot.status['end_of_arm']['safe_motion_velocity_brake']:
                    safe_motion_triggered = safe_motion_triggered or self.robot.status['end_of_arm']['safe_motion_velocity_brake'][j]['safe_motion_triggered']

        return safe_motion_triggered
