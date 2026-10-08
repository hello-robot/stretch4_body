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
        # Create a dummy driver file under a deliberately arbitrary filename (not
        # `{tool_name}.py`/`tool.py`/`end_of_arm.py`) to prove resolution comes from
        # tool_params.yaml alone, never from filename conventions.
        py_file = os.path.join(self.tool_dir, "driver_impl.py")
        with open(py_file, 'w') as f:
            f.write("class UserEoaTesttool:\n    def __init__(self):\n        pass\n")

        # Same for the client -- an arbitrary filename, not `{tool_name}_client.py`/`tool_client.py`.
        client_py_file = os.path.join(self.tool_dir, "remote_impl.py")
        with open(client_py_file, 'w') as f:
            f.write("class UserEoaTesttool_Client:\n    def __init__(self):\n        pass\n")

        # tool_params.yaml is the single source of truth for driver/client wiring -- it
        # points explicitly at the (arbitrarily-named) files above.
        params_file = os.path.join(self.tool_dir, "tool_params.yaml")
        with open(params_file, 'w') as f:
            f.write(
                "tool: eoat_custom\n"
                "stow:\n"
                "  custom_val: 42\n"
                "py_module_name: driver_impl\n"
                "py_class_name: UserEoaTesttool\n"
                "client_module_name: remote_impl\n"
                "client_class_name: UserEoaTesttool_Client\n"
            )

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

        # Verify the custom tool is registered, and that driver/client resolve entirely
        # from tool_params.yaml's py_module_name/py_class_name/client_module_name/
        # client_class_name -- not from the (deliberately non-conventional) filenames
        # created in setUp.
        self.assertIn(self.tool_name, RobotParams._robot_params['supported_eoa'])
        self.assertEqual(RobotParams._robot_params[self.tool_name]['py_class_name'], 'UserEoaTesttool')
        self.assertEqual(RobotParams._robot_params[self.tool_name]['py_module_name'], 'driver_impl')
        self.assertEqual(RobotParams._robot_params[self.tool_name]['client_class_name'], 'UserEoaTesttool_Client')
        self.assertEqual(RobotParams._robot_params[self.tool_name]['client_module_name'], 'remote_impl')

        # Verify custom params were merged and inflated
        self.assertEqual(RobotParams._robot_params[self.tool_name]['tool'], 'eoat_custom')
        self.assertEqual(RobotParams._robot_params[self.tool_name]['stow']['custom_val'], 42)

    def test_dynamic_loading_ignores_conventional_filenames(self):
        # A tool that provides conventionally-named files (end_of_arm.py, tool.py -- the
        # historical auto-detection patterns) but no tool_params.yaml driver/client keys
        # must NOT have them picked up: there is no filename-based detection any more, so
        # it falls back to the passive NIL driver and no bespoke client.
        split_tool_name = "user_eoa_split_tool"
        split_tool_dir = os.path.join(self.user_tools_dir, split_tool_name)
        os.makedirs(split_tool_dir, exist_ok=True)

        with open(os.path.join(split_tool_dir, "end_of_arm.py"), 'w') as f:
            f.write("class UserEoaSplitTool:\n    def __init__(self):\n        pass\n")

        with open(os.path.join(split_tool_dir, "tool.py"), 'w') as f:
            f.write("class UserEoaSplitToolGripper:\n    def __init__(self):\n        pass\n")

        # Reload robot params module to trigger scanning
        from stretch4_body.core.robot_params import RobotParams
        RobotParams.reload()

        try:
            self.assertIn(split_tool_name, RobotParams._robot_params['supported_eoa'])
            self.assertEqual(
                RobotParams._robot_params[split_tool_name]['py_class_name'], 'EOA_Wrist_DW4_Tool_NIL'
            )
            self.assertEqual(
                RobotParams._robot_params[split_tool_name]['py_module_name'],
                'stretch4_body.subsystem.end_of_arm.end_of_arm_tools',
            )
            self.assertNotIn('client_class_name', RobotParams._robot_params[split_tool_name])
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

        from stretch4_body.utils.tool_metadata import get_tool_metadata, LinearToolMetadata

        try:
            meta = get_tool_metadata(valid_tool_name)
            self.assertIsInstance(meta, LinearToolMetadata)
            self.assertEqual(meta.primary_joint, "joint_left")
            self.assertEqual(meta.tool_joints, ["joint_left", "joint_right"])
            self.assertEqual(meta.tool_links, ["link_left", "link_right"])
            self.assertEqual(meta.actuator_range, (0.0, 100.0))
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
