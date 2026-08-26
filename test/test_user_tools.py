#!/usr/bin/env python3

import os
import unittest
import shutil


class TestUserTools(unittest.TestCase):
    def setUp(self):
        # Setup a temporary custom tool inside stretch_user/user_tools
        self.fleet_path = os.environ.get('HELLO_FLEET_PATH', os.path.expanduser('~/stretch_user'))
        self.fleet_id = os.environ.get('HELLO_FLEET_ID', 'stretch-se4-4024')
        self.user_tools_dir = os.path.join(self.fleet_path, "user_tools")
        self.tool_name = "user_eoa_testtool"
        self.tool_dir = os.path.join(self.user_tools_dir, self.tool_name)
        
        os.makedirs(self.tool_dir, exist_ok=True)
        # Create a dummy python driver
        py_file = os.path.join(self.tool_dir, f"{self.tool_name}.py")
        with open(py_file, 'w') as f:
            f.write("class UserEoaTesttool:\n    def __init__(self):\n        pass\n")
            
        # Create a dummy client driver
        client_py_file = os.path.join(self.tool_dir, f"{self.tool_name}_client.py")
        with open(client_py_file, 'w') as f:
            f.write("class UserEoaTesttool_Client:\n    def __init__(self):\n        pass\n")
            
        # Create a tool_params.yaml to verify merging
        params_file = os.path.join(self.tool_dir, "tool_params.yaml")
        with open(params_file, 'w') as f:
            f.write("tool: eoat_custom\nstow:\n  custom_val: 42\n")

        # Set environment variables
        os.environ['HELLO_FLEET_PATH'] = self.fleet_path
        os.environ['HELLO_FLEET_ID'] = self.fleet_id

    def tearDown(self):
        # Cleanup temporary tool
        if os.path.exists(self.tool_dir):
            shutil.rmtree(self.tool_dir)

    def test_dynamic_loading(self):
        # Reload robot params module to trigger scanning
        from stretch4_body.core.robot_params import RobotParams
        RobotParams.reload()

        # Verify the custom tool is registered
        self.assertIn(self.tool_name, RobotParams._robot_params['supported_eoa'])
        self.assertEqual(RobotParams._robot_params[self.tool_name]['py_class_name'], 'UserEoaTesttool')
        self.assertEqual(RobotParams._robot_params[self.tool_name]['py_module_name'], self.tool_name)
        self.assertEqual(RobotParams._robot_params[self.tool_name]['client_class_name'], 'UserEoaTesttool_Client')
        self.assertEqual(RobotParams._robot_params[self.tool_name]['client_module_name'], f"{self.tool_name}_client")
        
        # Verify custom params were merged and inflated
        self.assertEqual(RobotParams._robot_params[self.tool_name]['tool'], 'eoat_custom')
        self.assertEqual(RobotParams._robot_params[self.tool_name]['stow']['custom_val'], 42)

    def test_dynamic_loading_split_files(self):
        # Setup a secondary custom tool with split files
        split_tool_name = "user_eoa_split_tool"
        split_tool_dir = os.path.join(self.user_tools_dir, split_tool_name)
        os.makedirs(split_tool_dir, exist_ok=True)
        
        # Create end_of_arm.py
        with open(os.path.join(split_tool_dir, "end_of_arm.py"), 'w') as f:
            f.write("class UserEoaSplitTool:\n    def __init__(self):\n        pass\n")
            
        # Create tool.py
        with open(os.path.join(split_tool_dir, "tool.py"), 'w') as f:
            f.write("class UserEoaSplitToolGripper:\n    def __init__(self):\n        pass\n")
            
        # Reload robot params module to trigger scanning
        from stretch4_body.core.robot_params import RobotParams
        RobotParams.reload()

        try:
            # Verify the split tool is registered with end_of_arm.py detected as primary
            self.assertIn(split_tool_name, RobotParams._robot_params['supported_eoa'])
            self.assertEqual(RobotParams._robot_params[split_tool_name]['py_class_name'], 'UserEoaSplitTool')
            self.assertEqual(RobotParams._robot_params[split_tool_name]['py_module_name'], 'end_of_arm')
        finally:
            if os.path.exists(split_tool_dir):
                shutil.rmtree(split_tool_dir)

    def test_builtin_tool_metadata(self):
        from stretch4_body.utils.tool_metadata import (
            get_tool_metadata,
            StretchGripperMetadata,
            ParallelGripperMetadata,
        )

        sg_meta = get_tool_metadata("stretch_gripper")
        self.assertIsInstance(sg_meta, StretchGripperMetadata)
        self.assertEqual(sg_meta.primary_joint, "gripper_finger_left_joint")
        self.assertEqual(sg_meta.tool_joints, ["gripper_finger_left_joint", "gripper_finger_right_joint"])

        pg_meta = get_tool_metadata("parallel_gripper")
        self.assertIsInstance(pg_meta, ParallelGripperMetadata)
        self.assertEqual(pg_meta.primary_joint, "finger_left_joint")
        self.assertEqual(pg_meta.tool_joints, ["finger_left_joint", "finger_right_joint"])

    def test_user_tool_metadata_valid(self):
        valid_tool_name = "user_eoa_validtool"
        valid_tool_dir = os.path.join(self.user_tools_dir, valid_tool_name)
        os.makedirs(valid_tool_dir, exist_ok=True)

        with open(os.path.join(valid_tool_dir, f"{valid_tool_name}.py"), 'w') as f:
            f.write("class UserEoaValidtool:\n    def __init__(self):\n        pass\n")

        with open(os.path.join(valid_tool_dir, f"{valid_tool_name}_client.py"), 'w') as f:
            f.write("class UserEoaValidtoolClient:\n    def __init__(self, ip_address=None):\n        self.poses = {'close': 0.0, 'open': 100.0}\n")

        with open(os.path.join(valid_tool_dir, "tool_params.yaml"), 'w') as f:
            f.write("""
tool_joints: ['joint_left', 'joint_right']
primary_joint: 'joint_left'
tool_links: ['link_left', 'link_right']
actuator_command_range: [0.0, 100.0]
aperture_range: [0.0, 0.08]
urdf_to_actuator_scale: 100.0
client_module_name: user_eoa_validtool_client
client_class_name: UserEoaValidtoolClient
""")

        from stretch4_body.core.robot_params import RobotParams
        RobotParams.reload()

        from stretch4_body.utils.tool_metadata import get_tool_metadata, UserToolMetadata

        try:
            meta = get_tool_metadata(valid_tool_name)
            self.assertIsInstance(meta, UserToolMetadata)
            self.assertEqual(meta.primary_joint, "joint_left")
            self.assertEqual(meta.tool_joints, ["joint_left", "joint_right"])
            self.assertEqual(meta.tool_links, ["link_left", "link_right"])
            self.assertEqual(meta.actuator_command_range, (0.0, 100.0))
            self.assertEqual(meta.aperture_range, (0.0, 0.08))

            # Test conversions
            self.assertAlmostEqual(meta.normalized_to_actuator(0.5), 50.0)
            self.assertAlmostEqual(meta.actuator_to_aperture(50.0), 0.04)
        finally:
            if os.path.exists(valid_tool_dir):
                shutil.rmtree(valid_tool_dir)

    def test_user_tool_metadata_fail_fast(self):
        invalid_tool_name = "user_eoa_invalidtool"
        invalid_tool_dir = os.path.join(self.user_tools_dir, invalid_tool_name)
        os.makedirs(invalid_tool_dir, exist_ok=True)

        with open(os.path.join(invalid_tool_dir, f"{invalid_tool_name}.py"), 'w') as f:
            f.write("class UserEoaInvalidtool:\n    def __init__(self):\n        pass\n")

        # Missing actuator_command_range and aperture_range
        with open(os.path.join(invalid_tool_dir, "tool_params.yaml"), 'w') as f:
            f.write("tool_joints: ['joint_a']\ntool_links: ['link_a']\nclient_module_name: dummy\nclient_class_name: Dummy\n")

        from stretch4_body.core.robot_params import RobotParams
        RobotParams.reload()
        from stretch4_body.utils.tool_metadata import get_tool_metadata, ToolConfigurationError

        try:
            with self.assertRaises(ToolConfigurationError):
                get_tool_metadata(invalid_tool_name)
        finally:
            if os.path.exists(invalid_tool_dir):
                shutil.rmtree(invalid_tool_dir)


if __name__ == "__main__":
    unittest.main()
