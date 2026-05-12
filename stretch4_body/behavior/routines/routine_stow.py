
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
        lift_stowed = False
        if 'lift' in self.robot.subsystems:
            self.logger.info('--------- Pre-Stowing Lift ----')
            self.robot.lift.move_to(0.35)
            if not self.wait_until_at_setpoint(self.robot.lift.motor, timeout=10.0):
                self.logger.warning('Lift failed to reach final position when stowing')
                return False


        # if 'end_of_arm' in self.subsystems:
        #     # Run pre stow specific to each end of arm
        #     self.end_of_arm.pre_stow(self)

        if 'arm' in self.robot.subsystems:
            pos_arm = cfg['arm']
            # Bring in arm before bring down
            self.logger.info('--------- Stowing Arm ----')
            self.robot.arm.move_to(pos_arm)
            if not self.wait_until_at_setpoint(self.robot.arm.motor, timeout=6.0):
                self.logger.warning('Arm failed to reach final position when stowing')
                return False

        if 'end_of_arm' in self.robot.subsystems:
            cmd = ['end_of_arm', 'stow', cmd_id, args, kwargs]
            # This will cause the EoA process to stop comms while stowing
            self.robot.eoa_loop.q_cmd.put(cmd)
            self.wait_duration(5.0)

        if 'lift' in self.robot.subsystems:
            # Now bring lift down
            pos_lift = cfg['lift']
            if not lift_stowed:
                self.logger.info('--------- Stowing Lift ----')
                self.robot.lift.move_to(pos_lift)
                if not self.wait_until_at_setpoint(self.robot.lift.motor, timeout=12.0):
                    self.logger.warning('Lift failed to reach final position when stowing')
                    return False
        # if 'end_of_arm' in self.subsystems:
        #     # Make sure wrist yaw is done before exiting
        #     while self.end_of_arm.motors['wrist_yaw'].motor.is_moving():
        #         time.sleep(0.1)
        #self.enable_collision_mgmt()

        return True



