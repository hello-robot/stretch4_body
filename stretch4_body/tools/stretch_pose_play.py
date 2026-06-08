#!/usr/bin/env python3

import time
import math
import click
import yaml
import stretch4_body.robot.robot_client as rc
from stretch4_body.core.gamepad_enums import MotionProfile

from stretch4_body.utils.stretch_pose_models import RobotJoints, RobotPose


class KeyframePlayer:
    """As a safety precaution, only joints in the `joints_allowed_to_move` param will move."""
    def __init__(self, *, joints_allowed_to_move:list[RobotJoints], motion_profile:MotionProfile, robot: rc.RobotClient|None = None, ):
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
            print(f"Loaded {len(self.poses)} poses from {filename}")
            print(f"Note: Only the following joints will move from the pre-recorded poses: {[j.name for j in self.joints_allowed_to_move]}")
        except FileNotFoundError:
            print(f"File {filename} not found.")
            self.poses = []

    def play_pose(self, pose: RobotPose):
        print(f"Moving to pose: {pose.name}")

        self.last_pose = pose
        
        for name, joint_pose in pose.joints.items():
            joint = RobotJoints[name]

            if not joint in self.joints_allowed_to_move:
                continue

            position = pose.joints[joint.name].position
            if joint is RobotJoints.arm:
                self.robot.arm.move_to(position, *joint.get_joint_params(self.motion_profile))
            elif joint is RobotJoints.lift:
                self.robot.lift.move_to(position, *joint.get_joint_params(self.motion_profile))
            elif joint in RobotJoints.get_end_of_arm_joints():
                self.robot.end_of_arm.move_to(name, joint_pose.position, *joint.get_joint_params(self.motion_profile))
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
        Wait to start at the relative timestamps of when the poses were recorded.
        For example, if 2 seconds passed between the first pose being recorded and the second pose being recorded, this would wait 2 seconds before playing the second pose.
        """
        diff = (pose.timestamp - self.last_pose.timestamp) if self.last_pose is not None else 0

        if diff > 0.0:
            print(f"Waiting {diff:.2f}s")
            time.sleep(diff)
        
        self.play_pose(pose)
        

    def play_poses(self, poses: list[RobotPose], delay_between_frames:float|None):
        for pose in poses:
            if delay_between_frames is None:
                self._play_pose_wait_until_start_time(pose)
            else:
                time.sleep(delay_between_frames)
                self.play_pose(pose)

    def play_poses_loop(self, poses: list[RobotPose], delay_between_frames:float|None, delay_between_restart:float):
        while True:
            self.play_poses(poses, delay_between_frames=delay_between_frames)
            time.sleep(delay_between_restart)

    def play_next(self, loop:bool = False, wait_until_frame_start_time:bool=False):
        if not self.poses:
            print("No poses loaded.")
            return False

        if self.current_pose_index >= len(self.poses):
            if not loop:
                return False
            print("All poses played. Resetting index.")
            self.current_pose_index = 0
            
        pose = self.poses[self.current_pose_index]
        self.play_pose(pose) if not wait_until_frame_start_time else self._play_pose_wait_until_start_time(pose)
        self.current_pose_index += 1
        return True

@click.command(help="Replay robot keyframes from a YAML file.\n\nSee also: stretch_pose_record, stretch_pose_edit")
@click.option('--file', help='File to load poses from')
@click.option('--speed', default=MotionProfile.MEDIUM.name, help=f'One of {[p.name for p in MotionProfile]}. Defaults to {MotionProfile.MEDIUM.name}.')
@click.option('--joints_allowed_to_move', default=','.join([j.name for j in RobotJoints if j is not RobotJoints.base]), help=f'Comma separated values of {[p.name for p in RobotJoints]}. Defaults to {",".join([j.name for j in RobotJoints if j is not RobotJoints.base])}.')
@click.option('--delay_between_frames', help='Delay between frames. If not specified, poses will be played at the relative timestamps of when they were recorded.')
@click.option('--loop', is_flag=True, help='Loop after reaching the last pose.')
def main(file, speed:str, joints_allowed_to_move:str, delay_between_frames:float|None, loop):
    print(f"""
WARNING: Please proceed carefully.
          
You are about to replay a set of pre-recorded robot poses.
          
This keyframe player does not guarantee any safety or precautions.
          
The robot joints will move in the same way they were recorded.
          
These joints will move: {joints_allowed_to_move}.
          
The robot's environment and surroundings may have changed since the recording.
          
Neither this keyframe player nor the robot account for the changes in the environment.

Please make sure the robot's surroundings are clear before proceeding.

""")
    
    delay_between_frames = float(delay_between_frames) if delay_between_frames is not None else None
    motion_profile = MotionProfile[speed.upper()]
    
    if input("Type y to continue. The robot will start moving. ").lower() != "y": return

    _joints_allowed_to_move = [RobotJoints[j] for j in joints_allowed_to_move.split(",")]
    player = KeyframePlayer(joints_allowed_to_move=_joints_allowed_to_move, motion_profile=motion_profile)
    player.load_from_file(file)
    
    if loop:
        player.play_poses_loop(player.poses,delay_between_frames=delay_between_frames, delay_between_restart=1.0)
    else:
        player.play_poses(player.poses, delay_between_frames=delay_between_frames)
    
    player.robot.stop()

if __name__ == "__main__":
    main()
