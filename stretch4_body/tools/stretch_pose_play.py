#!/usr/bin/env python3

import logging
import time
import math
import click
import yaml
import stretch4_body.robot.robot_client as rc
from stretch4_body.core.gamepad_enums import MotionProfile

from stretch4_body.utils.stretch_pose_models import RobotJoints, RobotPose

logger = logging.getLogger(__name__)


class KeyframePlayer:
    """As a safety precaution, only joints in the `joints_allowed_to_move` param will move."""
    def __init__(self, *, joints_allowed_to_move:list[RobotJoints], motion_profile:MotionProfile, robot: rc.RobotClient|None = None, ):
        self.logger = logging.getLogger(__name__)

        if robot:
            self.robot = robot
        else:
            self.robot = rc.RobotClient()
            self.robot.startup()
            
        self.poses: list[RobotPose] = []
        self.current_pose_index = 0

        self.last_pose:RobotPose|None = None

        self.joints_allowed_to_move = joints_allowed_to_move
        self.motion_profile = motion_profile


    def load_from_file(self, filename: str):
        try:
            with open(filename, 'r') as f:
                data = yaml.safe_load(f)
            self.poses = [RobotPose.from_dict(p) for p in data]
            self.current_pose_index = 0
            self.logger.info(f"Loaded {len(self.poses)} poses from {filename}")
            self.logger.info(f"Note: Only the following joints will move from the pre-recorded poses: {[j.name for j in self.joints_allowed_to_move]}")
        except FileNotFoundError:
            self.logger.error(f"File {filename} not found.")
            self.poses = []

    def play_pose(self, pose: RobotPose):
        self.logger.info(f"Moving to pose: {pose.name}")

        self.last_pose = pose
        
        for name, joint_pose in pose.joints.items():
            joint = RobotJoints.get_joint_by_name(name)
            if joint is None or joint.value is None:
                continue

            if not joint in self.joints_allowed_to_move:
                continue

            position = joint.to_subsystem_units(joint_pose.position)
            if joint is RobotJoints.arm:
                self.robot.arm.move_to(position, *joint.get_joint_params(self.motion_profile))
            elif joint is RobotJoints.lift:
                self.robot.lift.move_to(position, *joint.get_joint_params(self.motion_profile))
            elif joint in RobotJoints.get_end_of_arm_joints():
                self.robot.end_of_arm.move_to(joint.value, position, *joint.get_joint_params(self.motion_profile))
            else:
                raise NotImplementedError(f"{joint.name} is not a supported joint to move.")
            
        # Move Base
        base_joint = RobotJoints.base
        if pose.base is not None and base_joint in self.joints_allowed_to_move:            
            # self.robot.base.move_by(dx_robot, dy_robot, dtheta, *base_joint.get_base_params(self.motion_profile))
            self.robot.base.rotate_by(pose.base.theta, *base_joint.get_base_params(self.motion_profile)[2:])
            self.robot.base.translate_by(pose.base.x, pose.base.y, *base_joint.get_base_params(self.motion_profile)[:2])

        self.robot.push_command()
        self.robot.wait_command()

    def _play_pose_wait_until_start_time(self, pose:RobotPose):
        """
        Wait to start using the delay_before_start specified in the pose.
        """
        diff = pose.delay_before_start

        if diff > 0.0:
            self.logger.info(f"Waiting {diff:.2f}s")
            time.sleep(diff)
        
        self.play_pose(pose)
        

    def play_poses(self, poses: list[RobotPose], delay_between_frames:float|None, step:bool = False):
        for pose in poses:
            if step:
                input(f"Press Enter to move to pose: {pose.name}...")
                self.play_pose(pose)
            elif delay_between_frames is None:
                self._play_pose_wait_until_start_time(pose)
            else:
                time.sleep(delay_between_frames)
                self.play_pose(pose)

    def play_poses_loop(self, poses: list[RobotPose], delay_between_frames:float|None, delay_between_restart:float, step:bool = False):
        while True:
            self.play_poses(poses, delay_between_frames=delay_between_frames, step=step)
            if not step:
                time.sleep(delay_between_restart)

    def play_next(self, loop:bool = False, wait_until_frame_start_time:bool=False):
        if not self.poses:
            self.logger.warning("No poses loaded.")
            return False

        if self.current_pose_index >= len(self.poses):
            if not loop:
                return False
            self.logger.info("All poses played. Resetting index.")
            self.current_pose_index = 0
            
        pose = self.poses[self.current_pose_index]
        self.play_pose(pose) if not wait_until_frame_start_time else self._play_pose_wait_until_start_time(pose)
        self.current_pose_index += 1
        return True

@click.command(help="Replay robot keyframes from a YAML file.\n\nSee also: stretch_pose_record, stretch_pose_edit")
@click.argument('file')
@click.option('--speed', default=MotionProfile.MEDIUM.name, help=f'One of {[p.name for p in MotionProfile]}. Defaults to {MotionProfile.MEDIUM.name}.')
@click.option('--joints_allowed_to_move', default=','.join([j.name for j in RobotJoints if j is not RobotJoints.base]), help=f'Comma separated values of {[p.name for p in RobotJoints]}. Defaults to {",".join([j.name for j in RobotJoints if j is not RobotJoints.base])}.')
@click.option('--delay_between_frames', help='Delay between frames. If not specified, poses will be played using the delay_before_start field, which is by default the relative timestamps of when the poses were recorded.')
@click.option('--loop', is_flag=True, help='Loop after reaching the last pose.')
@click.option('--step', is_flag=True, help='Step through poses by pressing enter.')
def main(file, speed:str, joints_allowed_to_move:str, delay_between_frames:float|None, loop, step):
    logger.warning(f"""
WARNING: Please proceed carefully.

You are about to replay a set of pre-recorded robot poses.

This keyframe player does not guarantee any safety or precautions.

The robot joints will move to the recorded poses.

These joints will move: {joints_allowed_to_move}.

The robot's environment and surroundings may have changed since the recording.

Neither this keyframe player nor the robot account for the changes in the environment.

Please make sure the robot's surroundings are clear before proceeding.

""")
    
    delay_between_frames = float(delay_between_frames) if delay_between_frames is not None else None
    motion_profile = MotionProfile[speed.upper()]
    
    if input("Type y to continue. The robot will start moving. ").lower() != "y": return

    _joints_allowed_to_move = [
        RobotJoints.get_joint_by_name(j)
        for j in joints_allowed_to_move.split(",")
        if RobotJoints.get_joint_by_name(j) is not None
    ]
    player = KeyframePlayer(joints_allowed_to_move=_joints_allowed_to_move, motion_profile=motion_profile)
    player.load_from_file(file)
    
    try:
        if loop:
            player.play_poses_loop(player.poses,delay_between_frames=delay_between_frames, delay_between_restart=1.0, step=step)
        else:
            player.play_poses(player.poses, delay_between_frames=delay_between_frames, step=step)
        
        time.sleep(0.5)
        player.robot.wait_command()
        
        logger.info("Replay complete. Press Ctrl+C to exit and stop the robot...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Exiting...")
    finally:
        player.robot.stop()

if __name__ == "__main__":
    main()
