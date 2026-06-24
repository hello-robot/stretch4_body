#!/usr/bin/env python3

import yaml
import click
import argparse
import copy
from stretch4_body.core.gamepad_enums import MotionProfile
from stretch4_body.utils.stretch_pose_models import RobotJoints, RobotPose
from stretch4_body.tools.stretch_pose_play import KeyframePlayer
from stretch4_body.core.mujoco_urdf import MujocoURDFCollisionViz


class MujocoURDFCollisionVizAnimate(MujocoURDFCollisionViz):
    def update_from_robot_pose(self, pose: RobotPose, highlight_joint=None, contact_dict=None):
        """
        Translates a RobotPose to the dictionary format expected by MujocoURDFCollisionViz.update()
        """
        urdf_joint_state = {}
        
        for joint_name, joint_pose in pose.joints.items():
            joint = RobotJoints.get_joint_by_name(joint_name)
            if joint is None:
                continue
            if joint is RobotJoints.lift:
                urdf_joint_state['lift_joint'] = joint_pose.position
            elif joint is RobotJoints.arm:
                val = joint_pose.position / 4.0
                urdf_joint_state['arm_l1_joint'] = val
                urdf_joint_state['arm_l2_joint'] = val
                urdf_joint_state['arm_l3_joint'] = val
                urdf_joint_state['arm_l4_joint'] = val
            elif joint in (RobotJoints.wrist_yaw, RobotJoints.wrist_pitch, RobotJoints.wrist_roll):
                urdf_joint_state[f'{joint.name}_joint'] = joint_pose.position
            elif joint is RobotJoints.gripper:
                if joint.value == 'parallel_gripper':
                    joint_val = -joint_pose.position / 2.0
                else:
                    joint_val = joint_pose.position
                for finger_joint in joint.finger_joints:
                    urdf_joint_state[finger_joint] = joint_val
            else:
                # Fallback for any other joints
                urdf_joint_state[f'{joint_name}_joint'] = joint_pose.position

        if contact_dict is None:
            contact_dict = {}

        if highlight_joint:
            h_joint = RobotJoints.get_joint_by_name(highlight_joint)
            if h_joint is RobotJoints.lift:
                contact_dict['lift_link'] = []
            elif h_joint is RobotJoints.arm:
                for i in range(1, 5):
                    contact_dict[f'arm_l{i}_link'] = []
            elif h_joint in (RobotJoints.wrist_yaw, RobotJoints.wrist_pitch, RobotJoints.wrist_roll):
                contact_dict[f'{h_joint.name}_link'] = []
            elif h_joint is RobotJoints.gripper:
                for finger_link in h_joint.finger_links:
                    contact_dict[finger_link] = []

        self.update(urdf_joint_state, contact_dict if contact_dict else None)


class KeyframeEditor:
    def __init__(self, filename):
        self.filename = filename
        
        self.current_pose_idx = 0
        
        # Determine the order of joints we can edit
        self.editable_joints = [
            'lift', 'arm', 'wrist_yaw', 'wrist_pitch', 'wrist_roll'
        ]
        if RobotJoints.gripper.value is not None:
            self.editable_joints.append('gripper')
        self.current_joint_idx = 0
        
        self.viz = MujocoURDFCollisionVizAnimate()

        self.player = KeyframePlayer(joints_allowed_to_move=[RobotJoints[j] for j in self.editable_joints], motion_profile=MotionProfile.SLOW)
        self.player.load_from_file(filename=filename)
        
        self.step_size = 0.05  # Basic step size for increments

    @property
    def poses(self):
        return self.player.poses

    def save_poses(self, output_filename=None):
        if output_filename is None:
            output_filename = self.filename.replace('.yaml', '_edited.yaml')
            if output_filename == self.filename:
                output_filename = "edited_" + self.filename
                
        data = [p.to_dict() for p in self.poses]
        with open(output_filename, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        print(f"\nSaved {len(self.poses)} poses to {output_filename}")

    def display_status(self):
        # Clear screen somewhat for simple CLI feel
        print("\033[H\033[J", end="")
        print("=== Keyframe Editor ===")
        print(f"File: {self.filename}")
        
        if not self.poses:
            print("No poses loaded.")
            return

        pose = self.poses[self.current_pose_idx]
        print(f"Pose: {self.current_pose_idx + 1} / {len(self.poses)}  ({pose.name})")
        
        print("\nJoints:")
        for idx, j_name in enumerate(self.editable_joints):
            pointer = "-->" if idx == self.current_joint_idx else "   "
            val = pose.joints[j_name].position if j_name in pose.joints else 0.0
            print(f"{pointer} {j_name:<16}: {val:.4f}")
            
        print(f"\nStep size: {self.step_size}")
        print("\nControls:")
        print("  [n/p] Next / Previous Pose")
        print("  [j/k] Next / Previous Joint")
        print("  [+/= / -] Increase / Decrease joint value")
        print("  [w/s] Increase / Decrease step size")
        print("  [Space] Duplicate current pose")
        print("  [Delete/x] Delete current pose")
        print("  [P] Play the current pose (Robot will move!)")
        print("  [S] Save to *_edited.json")
        print("  [q] Quit")
        selected_joint = self.editable_joints[self.current_joint_idx]
        self.viz.update_from_robot_pose(pose, highlight_joint=selected_joint)

    def run(self):
        if not self.poses:
            return
            
        self.display_status()
        
        while True:
            char = click.getchar()
            
            if char == 'q':
                break
            elif char == 'n':
                self.current_pose_idx = (self.current_pose_idx + 1) % len(self.poses)
            elif char == 'p':
                self.current_pose_idx = (self.current_pose_idx - 1) % len(self.poses)
            elif char == 'j':
                self.current_joint_idx = (self.current_joint_idx + 1) % len(self.editable_joints)
            elif char == 'k':
                self.current_joint_idx = (self.current_joint_idx - 1) % len(self.editable_joints)
            elif char in ['+', '=']:
                j_name = self.editable_joints[self.current_joint_idx]
                if j_name in self.poses[self.current_pose_idx].joints:
                    self.poses[self.current_pose_idx].joints[j_name].position += self.step_size
            elif char == '-':
                j_name = self.editable_joints[self.current_joint_idx]
                if j_name in self.poses[self.current_pose_idx].joints:
                    self.poses[self.current_pose_idx].joints[j_name].position -= self.step_size
            elif char == 'w':
                self.step_size *= 2.0
            elif char == 's':
                self.step_size /= 2.0
            elif char == 'S':
                self.save_poses()
            elif char == 'P':
                self.player.play_pose(self.poses[self.current_pose_idx])
            elif char == ' ': # Space to duplicate
                new_pose = copy.deepcopy(self.poses[self.current_pose_idx])
                new_pose.name = f"{new_pose.name}_copy"
                self.poses.insert(self.current_pose_idx + 1, new_pose)
                self.current_pose_idx += 1
            elif char in ['\x7f', 'x']: # Backspace/Delete or x
                if len(self.poses) > 1:
                    self.poses.pop(self.current_pose_idx)
                    if self.current_pose_idx >= len(self.poses):
                        self.current_pose_idx = len(self.poses) - 1
            
            self.display_status()
            
        self.viz.stop()

def main():
    parser = argparse.ArgumentParser(
        description="Edit keyframes.\n\nSee also: stretch_pose_play, stretch_pose_record",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("filename", help="YAML file containing poses")
    args = parser.parse_args()
    
    editor = KeyframeEditor(args.filename)
    editor.run()

if __name__ == "__main__":
    main()
