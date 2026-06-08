#!/usr/bin/env python3

import time
import click
import json
import stretch4_body.robot.robot_client as rc

from stretch4_body.utils.stretch_pose_models import RobotJoints, RobotPose, JointPose, BasePose

class KeyframeRecorder:
    def __init__(self, filename=None):
        self.robot = rc.RobotClient()
        self.robot.startup()
        self.saved_poses: list[RobotPose] = []
        self.filename = filename or f"poses_{time.time()}.json"
        
    def capture_pose(self, name: str|None=None) -> RobotPose:
        self.robot.pull_status()
        status = self.robot.status

        pose_name = name or f"pose_{len(self.saved_poses)}"
        
        pose = RobotPose(name=pose_name, timestamp=time.time())
        
        # Arm
        if RobotJoints.arm.name in status:
            pose.joints[RobotJoints.arm.name] = JointPose(
                name=RobotJoints.arm.name,
                position=status[RobotJoints.arm.name]['pos'],
                velocity=status[RobotJoints.arm.name]['vel'],
                effort=status[RobotJoints.arm.name]['force']
            )
            
        # Lift
        if RobotJoints.lift.name in status:
            pose.joints[RobotJoints.lift.name] = JointPose(
                name=RobotJoints.lift.name,
                position=status[RobotJoints.lift.name]['pos'],
                velocity=status[RobotJoints.lift.name]['vel'],
                effort=status[RobotJoints.lift.name]['force']
            )
            
        # Wrist (EndOfArm)
        if 'end_of_arm' in status:
            eoa_status = status['end_of_arm']
            # Iterate through known joints in EndOfArm
            for joint in RobotJoints.get_end_of_arm_joints():
                if joint.name in eoa_status:
                    j_status = eoa_status[joint.name]
                    pose.joints[joint.name] = JointPose(
                        name=joint.name,
                        position=j_status['pos'],
                        velocity=j_status['vel'],
                        effort=j_status.get('effort', 0.0)
                    )
                    if joint is RobotJoints.stretch_gripper:
                        pose.joints[joint.name] = JointPose(
                            name=joint.name,
                            position=j_status['gripper_conversion']['finger_rad'],
                            velocity=j_status['gripper_conversion']['finger_vel'],
                            effort=j_status['gripper_conversion'].get('finger_effort', 0.0)
                        )

        # Base
        if RobotJoints.base.name in status:
            b = status[RobotJoints.base.name]
            pose.base = BasePose(x=b['x'], y=b['y'], theta=b['theta'])

        self.saved_poses.append(pose)
            
        return pose

    def save_to_file(self, filename: str):
        data = [p.to_dict() for p in self.saved_poses]
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)
        print(f"Saved {len(self.saved_poses)} poses to {filename}")

    def run(self):
        print(f"""Keyframes recorder has started. 
Use GamepadTeleop to move Stretch.
Press 's' to save the current pose.
Poses will be saved to {self.filename}
Press 'q' to quit. 
""")
        
        while True:
            char = click.getchar()
            if char == 's':
                pose = self.capture_pose()
                print(f"Captured pose: {pose.name}")
            elif char == 'q':
                break
        
        self.save_to_file(self.filename)
        self.robot.stop()
        self.print_poses()

    def print_poses(self):
        print("\n--- Saved Poses ---")
        for i, pose in enumerate(self.saved_poses):
            print(f"Pose {i}: {pose.name}")

@click.command(help="Record robot keyframes by saving the current pose when pressing 's'.\n\nSee also: stretch_pose_play, stretch_pose_edit")
@click.option('--file', '-f', default=None, help='File to save poses to. Defaults to poses_<timestamp>.json')
def main(file):
    recorder = KeyframeRecorder(filename=file)
    recorder.run()

if __name__ == "__main__":
    main()
