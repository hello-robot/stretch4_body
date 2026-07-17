import stretch4_body.behavior.routines.routine as routine
import time

# ###############################################################3


class RoutineRobotStow(routine.Routine):
    def __init__(self,robot):
        routine.Routine.__init__(self,name='routine_robot_stow',robot=robot)


    def run(self,cmd_id,*args, **kwargs):
        """
        Cause the robot to move to its stow position  .
        """

        super().run(cmd_id, *args, **kwargs)
        
        tool=self.robot_params['robot']['tool']
        cfg=self.robot_params[tool]['stow']
        self.logger.info(f'Stowing robot for tool {tool}')


        #self.disable_collision_mgmt()

        if 'lift' in self.robot.subsystems:
            pos_lift = cfg. get('lift_prestow', 0.35) # Enough to clear the tool from hitting the base
            if self.robot.lift.status['pos'] < pos_lift:
                # If the lift is below pos_lift, raise it to avoid the tool hitting the base
                self.logger.info('--------- Pre-Stowing Lift ----')
                self.robot.lift.move_to(pos_lift)
                if not self.wait_until_at_setpoint(self.robot.lift.motor, timeout=10.0):
                    self.logger.warning(f'Lift failed to reach final position when stowing. Expecting {pos_lift} but got {self.robot.lift.status["pos"]:.3f}')
                    return False


        # if 'end_of_arm' in self.robot.subsystems:
        #     # Run pre stow specific to each end of arm
        #     cmd = ['end_of_arm', 'pre_stow', cmd_id, args, kwargs]
        #     # This will cause the EoA process to stop comms while stowing
        #     self.robot.eoa_loop.q_cmd.put(cmd)
        #     self.wait_duration(5.0)

        if 'arm' in self.robot.subsystems:
            pos_arm = cfg['arm']
            threshold = 0.025
            # Bring in arm before bring down
            self.logger.info('--------- Stowing Arm ----')
            self.robot.arm.move_to(pos_arm)
            if not self.wait_until_at_setpoint(self.robot.arm.motor, timeout=4.0):
                actual_pos = self.robot.arm.status['pos']
                within_threshold = abs(actual_pos - pos_arm) < threshold
                self.logger.warning(f'Arm failed to reach final position when stowing. Expecting {pos_arm} but got {actual_pos:.3f}. The arm is {"within" if within_threshold else "not within"} threshold, {"continuing" if within_threshold else "aborting"}.')

                if not within_threshold:
                    return False

        if 'end_of_arm' in self.robot.subsystems:
            cmd = ['end_of_arm', 'stow', cmd_id, args, kwargs]
            # This will cause the EoA process to stop comms while stowing
            self.robot.eoa_loop.q_cmd.put(cmd)
            self.wait_duration(5.0)

        if 'lift' in self.robot.subsystems:
            # Now bring lift down
            pos_lift = cfg['lift']
            self.logger.info('--------- Stowing Lift ----')
            self.robot.lift.move_to(pos_lift)
            if not self.wait_until_at_setpoint(self.robot.lift.motor, timeout=12.0):
                self.logger.warning(f'Lift failed to reach final position when stowing. Expecting {pos_lift} but got {self.robot.lift.status["pos"]:.3f}')
                return False
        # if 'end_of_arm' in self.subsystems:
        #     # Make sure wrist yaw is done before exiting
        #     while self.end_of_arm.motors['wrist_yaw'].motor.is_moving():
        #         time.sleep(0.1)
        #self.enable_collision_mgmt()

        return True



