#!/usr/bin/env python3

import os
import sys
import unittest
import shutil

class TestUserTools(unittest.TestCase):
    def setUp(self):
        # Setup a temporary custom tool inside stretch_user/stretch-se4-4024/user_tools
        self.fleet_path = "/home/shehab/stretch_user"
        self.fleet_id = "stretch-se4-4024"
        self.user_tools_dir = os.path.join(self.fleet_path, self.fleet_id, "user_tools")
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
        for mod in list(sys.modules.keys()):
            if 'robot_params' in mod:
                del sys.modules[mod]
            
        from stretch4_body.core.robot_params import RobotParams
        
        # Verify the custom tool is registered
        self.assertIn(self.tool_name, RobotParams._robot_params['supported_eoa'])
        self.assertEqual(RobotParams._robot_params[self.tool_name]['py_class_name'], 'UserEoaTesttool')
        self.assertEqual(RobotParams._robot_params[self.tool_name]['py_module_name'], self.tool_name)
        self.assertEqual(RobotParams._robot_params[self.tool_name]['client_class_name'], 'UserEoaTesttool_Client')
        self.assertEqual(RobotParams._robot_params[self.tool_name]['client_module_name'], f"{self.tool_name}_client")
        
        # Verify custom params were merged and inflated
        self.assertEqual(RobotParams._robot_params[self.tool_name]['tool'], 'eoat_custom')
        self.assertEqual(RobotParams._robot_params[self.tool_name]['stow']['custom_val'], 42)

if __name__ == "__main__":
    unittest.main()
