#!/usr/bin/env python3

import os
import shutil
import unittest

import yaml


class TestRobotPose(unittest.TestCase):
    def setUp(self):
        from stretch4_body.core.robot_params import RobotParams
        from stretch4_body.utils.stretch_pose_models import RobotPose

        self.RobotParams = RobotParams
        self.RobotPose = RobotPose
        self.fleet_path = os.environ.get('HELLO_FLEET_PATH', os.path.expanduser('~/stretch_user'))
        self.user_tools_dir = os.path.join(self.fleet_path, "user_tools")
        self.tool_name = "user_eoa_posetool"
        self.tool_dir = os.path.join(self.user_tools_dir, self.tool_name)
        os.makedirs(self.tool_dir, exist_ok=True)
        os.environ['HELLO_FLEET_PATH'] = self.fleet_path
        self.RobotParams.reload()

    def tearDown(self):
        if os.path.exists(self.tool_dir):
            shutil.rmtree(self.tool_dir)
        self.RobotParams.reload()

    def _write_poses(self, poses):
        with open(os.path.join(self.tool_dir, "tool_params.yaml"), 'w') as f:
            yaml.safe_dump({'pose_models': poses}, f)
        self.RobotParams.reload()

    def test_from_dict_keeps_joint_mapping(self):
        # setdefault fills in a missing 'name' without discarding the rest of the joint dict.
        pose = self.RobotPose.from_dict({
            'name': 'stow',
            'timestamp': 0.0,
            'joints': {'arm': {'position': 0.25, 'velocity': 0.0, 'effort': 0.0}},
        })
        self.assertEqual(pose.name, 'stow')
        self.assertEqual(pose.joints['arm'].name, 'arm')
        self.assertAlmostEqual(pose.joints['arm'].position, 0.25)

    def test_from_dict_preserves_explicit_joint_name(self):
        pose = self.RobotPose.from_dict({
            'name': 'zero',
            'timestamp': 0.0,
            'joints': {'lift': {'name': 'lift', 'position': 0.15, 'velocity': 0.0, 'effort': 0.0}},
        })
        self.assertEqual(pose.joints['lift'].name, 'lift')

    def test_round_trip(self):
        source = {
            'name': 'pose_0',
            'timestamp': 12.5,
            'joints': {'arm': {'name': 'arm', 'position': 0.1, 'velocity': 0.0, 'effort': 0.0}},
            'base': {'x': 1.0, 'y': 2.0, 'theta': 0.5},
            'delay_before_start': 1.5,
        }
        pose = self.RobotPose.from_dict(source)
        self.assertEqual(pose.to_dict(), source)

    def test_load_tool_pose_models(self):
        self._write_poses([
            {'name': 'stow', 'timestamp': 0.0,
             'joints': {'arm': {'position': 0.0, 'velocity': 0.0, 'effort': 0.0}}},
            {'name': 'zero', 'timestamp': 0.0,
             'joints': {'lift': {'position': 0.15, 'velocity': 0.0, 'effort': 0.0}}},
        ])
        poses = self.RobotPose.load_tool_pose_models(self.tool_name)
        self.assertEqual(sorted(poses), ['stow', 'zero'])
        self.assertAlmostEqual(poses['zero'].joints['lift'].position, 0.15)

    def test_load_tool_pose_models_empty_list(self):
        self._write_poses([])
        self.assertEqual(self.RobotPose.load_tool_pose_models(self.tool_name), {})

    def test_load_tool_pose_models_without_tool_params(self):
        self.assertEqual(self.RobotPose.load_tool_pose_models(self.tool_name), {})

    def test_load_tool_pose_models_reports_malformed_pose(self):
        self._write_poses([{'name': 'broken', 'timestamp': 0.0,
                            'joints': {'arm': {'position': 0.0}}}])
        with self.assertRaises(ValueError) as ctx:
            self.RobotPose.load_tool_pose_models(self.tool_name)
        self.assertIn('broken', str(ctx.exception))


if __name__ == '__main__':
    unittest.main()
