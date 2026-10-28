import time

import stretch4_body.behavior.routines.routine as routine
from stretch4_body.subsystem.end_of_arm.end_of_arm import GripperHandedness, get_handedness_lift_min

# ###############################################################3


class RoutineWristHandedness(routine.Routine):
    """
    Move the wrist to the opposite handedness (Left/Right) of its current configuration.

    The current handedness is read from the end_of_arm status 'tool_right_handed',
    and the wrist is moved to the other handedness's home configuration (see GripperHandedness.move_to).
    """
    def __init__(self,robot):
        routine.Routine.__init__(self,name='routine_wrist_handedness',robot=robot)

    def run(self,cmd_id,*args, **kwargs):
        if hasattr(self.robot, 'power_periph') and self.robot.is_runstopped():
            self.logger.warning('Not able to change wrist handedness. Robot is runstopped')
            return False

        super().run(cmd_id, *args, **kwargs)

        if not hasattr(self.robot,'eoa_loop') or not self.robot.eoa_loop.is_valid:
            self.logger.warning('Not able to change wrist handedness. End of arm not present')
            return False

        # The wrist swings down through the folded pose, so the lift must be high enough for it to clear the ground and base
        lift_min = get_handedness_lift_min()
        def is_lift_too_low():
            return 'lift' in self.robot.subsystems and self.robot.lift.status['pos'] <= lift_min
        if is_lift_too_low():
            self.logger.warning(f'Not able to change wrist handedness. Lift too low ({self.robot.lift.status["pos"]:.3f} <= {lift_min})')
            return False

        current = GripperHandedness.from_status(self.robot.eoa_loop.status)
        if current is None:
            self.logger.warning('Not able to change wrist handedness. Handedness unknown (wrist_pitch absent or not homed)')
            return False
        target = current.opposite()
        self.logger.info(f'Changing wrist handedness from {current.get_name()} to {target.get_name()}')

        # EndOfArm.change_handedness() moves to the opposite of the same status read above
        self.robot.eoa_loop.q_cmd.put(['end_of_arm', 'change_handedness', cmd_id, (), {}])

        targets = target.get_joint_targets()
        tolerance = self.params.get('pos_tolerance_rad', 0.05)
        t_motion_start = self.params.get('t_motion_start', 0.5) # Ignore arrival until motion has begun
        ts = time.time()
        lift_dropped = False
        def at_target_or_lift_dropped():
            nonlocal lift_dropped
            if is_lift_too_low():
                lift_dropped = True
                return True
            if time.time() - ts < t_motion_start:
                return False
            s = self.robot.eoa_loop.status
            for joint, x_r in targets.items():
                if s[joint]['is_moving'] or abs(s[joint]['pos'] - x_r) > tolerance:
                    return False
            return GripperHandedness.from_status(s) is target

        if not self.wait_on_cb(at_target_or_lift_dropped, timeout=self.params.get('timeout', 15.0)):
            self.logger.warning(f'Wrist failed to reach {target.get_name()} handedness')
            return False

        if lift_dropped:
            self.logger.warning(f'Lift dropped to {self.robot.lift.status["pos"]:.3f} (<= {lift_min}) during wrist handedness change. Stopping wrist')
            for joint in targets:
                self.robot.eoa_loop.q_cmd.put(['end_of_arm', 'quick_stop', cmd_id, (joint,), {}])
            return False

        self.logger.info(f'Wrist handedness changed to {target.get_name()}')
        return True
