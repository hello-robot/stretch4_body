#Robot parameters for Stretch 4

# ######################### USER PARAMS ##################################################
#Template for the generated file: stretch_user_params.yaml
user_params_header='#User parameters\n' \
                   '#You can override nominal settings here\n' \
                   '#USE WITH CAUTION. IT IS POSSIBLE TO CAUSE UNSAFE BEHAVIOR OF THE ROBOT \n'

user_params_template={
    'robot': {'NA': 0}} #Include this just as an example

# ###################### CONFIGURATION PARAMS #####################################################
#Template for the generated file: stretch_configuration_params.yaml
#Configuration parameters may have variation across the fleet of  robots
configuration_params_header='#Parameters that are specific to this robot\n' \
                            '#Do not edit, instead edit stretch_user_params.yaml\n'

configuration_params_template={
    'hello-motor-arm':{'serial_no': 'NA'},
    'hello-motor-lift':{'serial_no': 'NA'},
    'hello-motor-omni-0':{'serial_no': 'NA'},
    'hello-motor-omni-1':{'serial_no': 'NA'},
    'hello-motor-omni-2': {'serial_no': 'NA'},
    'power-periph':{
         'firebase':{
           'url': "NA",
           'api_key': "NA",
           'user_email': "NA",
           'user_password': "NA",
           'network_ssid': "NA",
           'network_password': "NA"}},
    'robot':{
        'batch_name': 'NA',
        'serial_no': 'NA',
        'model_name':'SE4'}}

# ###################### NOMINAL PARAMS #####################################################
#Parameters that are common across the SE4 fleet



# ######## EOA Joints ######
# We use a modular design of dictionaries so that different parameter sets
# can be easily managed depending on the configuation of the end-of-arm
# Eg, which joints and tools and versions of hardware are present


SE4_wrist_yaw_DW4={
    'eeprom_cfg': {
        'temperature_limit': 72,
        'max_voltage_limit': 29,
        'min_voltage_limit': 11,
        'pid': [32,32,0],
        'return_delay_time': 0,
        'angular_resolution': 1.0,
        'phase': 61, #61 for multi-turn, 45 for normal
        'max_pos_limit': 0,#0 for multi-turn, 4095 for normal
        'min_pos_limit': 0,
        'max_load_limit_pct': 48.0,
        'overload_safe': 25.0,
        'overload_time_ms': 1000,
        'overload_thresh': 48.0,
        'overcurrent': 150,
        'overcurrent_time_ms': 200,
        'protection_torque': 20,
        'overload_protection_time': 200,
        'enable_protection_overload':0,
        'enable_protection_current':1,
        'enable_protection_temp':1,
        'enable_protection_sensor':0,
        'enable_protection_voltage':0
    },
    'motion': {
        # Predefined motion profiles (vel: rad/s, accel: rad/s^2)
        'default': {'accel': 7.0, 'vel': 7.0},
        'fast': {'accel': 9.0, 'vel': 9.0},
        'max': {'accel': 12.0, 'vel': 12.0},
        'slow': {'accel': 4.0, 'vel': 4.0},
        'vel_brakezone_factor': 1,
        'vel_is_moving_thresh': 0.01
    },
        'id': 20,
        'set_safe_velocity':1,
        'req_calibration': 1,
        'gr': 2.0,
        'usb_name': '/dev/hello-feetech-wrist',
        'retry_on_comm_failure': 1,
        'baud': 1000000,
        'range_pad_deg': [ 0.0, 0.0 ],
        'range_deg': [-65, 245],
        'homing_offset_bias_t': -133,
        'homing_to_neg_limit': 1,
        'homing_pwm': -200,
        'flip_encoder_polarity': 1,
        # Stall detection limits. Tune these for force-sensitive tasks.
        'stall_backoff': 0.017,
        'stall_max_effort': 20.0,
        'stall_max_time': 1.0,
        'stall_min_vel': 0.1,
        'disable_torque_on_runstop': 1,
        'enable_torque_after_runstop':1,
        'enable_runstop':1}

SE4_stretch_gripper_DW4={

    'gripper_conversion': {
        'finger_length_m': 0.18205,# Straight length from `link_gripper_finger_right.STL`, ignoring the bend in the metal
        'aperture_open_m': 0.150,  # Measured by hand, from fingertip to fingertip
        'aperture_closed_m': 0.0
    },
    'eeprom_cfg': {
        'temperature_limit': 72,
        'max_voltage_limit': 29,
        'min_voltage_limit': 11,
        'pid': [32,32,0],
        'return_delay_time': 0,
        'angular_resolution': 1.0,
        'phase': 61, #61 for multi-turn, 45 for normal
        'max_pos_limit': 0,#0 for multi-turn, 4095 for normal
        'min_pos_limit': 0,
        'max_load_limit_pct': 48.0,
        'overload_safe': 25.0,
        'overload_time_ms': 1000,
        'overload_thresh': 48.0,
        'overcurrent': 150,
        'overcurrent_time_ms': 200,
        'protection_torque': 20,
        'overload_protection_time': 200,
        'enable_protection_overload':0,
        'enable_protection_current':1,
        'enable_protection_temp':1,
        'enable_protection_sensor':0,
        'enable_protection_voltage':0
    },
    'motion': {
        # Predefined motion profiles (vel: rad/s, accel: rad/s^2)
        'default': {'accel': 6.0, 'vel': 6.0},
        'fast': {'accel': 6.0, 'vel': 6.0},
        'max': {'accel': 6.0, 'vel': 6.0},
        'slow': {'accel': 4.0, 'vel': 1.0},
        'vel_brakezone_factor': 1,
        'vel_is_moving_thresh': 0.01
    },
        'id': 23,
        'set_safe_velocity': 1,
        'req_calibration': 1,
        'gr': 1.0,
        'usb_name': '/dev/hello-feetech-wrist',
        'retry_on_comm_failure': 1,
        'baud': 1000000,
        'range_pad_deg': [ 0.0, 0.0 ],
        'range_deg': [-100,300],
        'homing_offset_bias_t': 0,  
        'homing_to_neg_limit': 1,
        'homing_pwm': 150,
        'flip_encoder_polarity': 0,
        'stall_backoff': 0.017,
        'stall_max_effort': 20.0,
        'stall_max_time': 1.0,
        'stall_min_vel': 0.1,
        'disable_torque_on_runstop': 0,
        'enable_torque_after_runstop': 1,
        'enable_runstop':1}

SE4_parallel_gripper_DW4={
    'eeprom_cfg': {
        'temperature_limit': 72,
        'max_voltage_limit': 29,
        'min_voltage_limit': 11,
        'pid': [32,32,0],
        'return_delay_time': 0,
        'angular_resolution': 1.0,
        'phase': 45,#61, #61 for multi-turn, 45 for normal
        'max_pos_limit': 4095,#0 for multi-turn, 4095 for normal
        'min_pos_limit': 0,
        'max_load_limit_pct': 48.0,
        'overload_safe': 25.0,
        'overload_time_ms': 1000,
        'overload_thresh': 48.0,
        'overcurrent': 150,
        'overcurrent_time_ms': 200,
        'protection_torque': 20,
        'overload_protection_time': 200,
        'enable_protection_overload':0,
        'enable_protection_current':1,
        'enable_protection_temp':1,
        'enable_protection_sensor':0,
        'enable_protection_voltage':0
    },
    'motion': {
        # Predefined motion profiles (vel: rad/s, accel: rad/s^2)
        'default': {'accel': 6.0, 'vel': 6.0},
        'fast': {'accel': 6.0, 'vel': 6.0},
        'max': {'accel': 6.0, 'vel': 6.0},
        'slow': {'accel': 4.0, 'vel': 1.0},
        'vel_brakezone_factor': 1,
        'vel_is_moving_thresh': 0.01
    },
        'id': 24,
        'set_safe_velocity': 1,
        'req_calibration': 1,
        'gr': 1.0,
        'usb_name': '/dev/hello-feetech-wrist',
        'retry_on_comm_failure': 1,
        'baud': 1000000,
        'range_pad_deg': [ 0.0, 0.0 ],
        'range_mm':80.0,
        'range_deg': [0,116.5],
        'homing_offset_bias_t': 0,      
        'homing_to_neg_limit': 1,
        'homing_pwm': -150,
        'flip_encoder_polarity': 1,
        'kL':30.25,#mm
        'kR':22.0,#mm
        'kT0':44.0,#deg
        'kX0': 10.5, #mm
        'stall_backoff': 0.017,
        'stall_max_effort': 20.0,
        'stall_max_time': 1.0,
        'stall_min_vel': 0.1,
        'disable_torque_on_runstop': 0,
        'enable_torque_after_runstop': 1,
        'enable_runstop':1}

SE4_wrist_pitch_DW4={
    'eeprom_cfg': {
        'temperature_limit': 72,
        'max_voltage_limit': 29,
        'min_voltage_limit': 11,
        'pid': [32,32,0],
        'return_delay_time': 0,
        'angular_resolution': 1.0,
        'phase': 61, #61 for multi-turn, 45 for normal
        'max_pos_limit': 0,#0 for multi-turn, 4095 for normal
        'min_pos_limit': 0,
        'max_load_limit_pct': 48.0,
        'overload_safe': 25.0,
        'overload_time_ms': 1000,
        'overload_thresh': 48.0,
        'overcurrent': 150,
        'overcurrent_time_ms': 200,
        'protection_torque': 20,
        'overload_protection_time': 200,
        'enable_protection_overload':0,
        'enable_protection_current':1,
        'enable_protection_temp':1,
        'enable_protection_sensor':0,
        'enable_protection_voltage':0
    },
    'motion': {
        # Predefined motion profiles (vel: rad/s, accel: rad/s^2)
        'default': {'accel': 7.0, 'vel': 7.0},
        'fast': {'accel': 9.0, 'vel': 9.0},
        'max': {'accel': 12.0, 'vel': 12.0},
        'slow': {'accel': 4.0, 'vel': 4.0},
        'vel_brakezone_factor': 1,
        'vel_is_moving_thresh': 0.01
    },
    'id': 21,
    'set_safe_velocity': 1,
    'req_calibration': 1,
    'gr': 2.0,
    'usb_name': '/dev/hello-feetech-wrist',
    'retry_on_comm_failure': 1,
    'baud': 1000000,
    'range_pad_deg': [1.0, 1.0],
    'range_deg': [-65, 245],
    'homing_offset_bias_t': -160,          
    'homing_to_neg_limit': 1,
    'homing_pwm': -200,
    'flip_encoder_polarity': 1,
    'stall_backoff': 0.017,
    'stall_max_effort': 20.0,
    'stall_max_time': 1.0,
    'stall_min_vel': 0.1,
    'disable_torque_on_runstop': 1,
    'enable_torque_after_runstop': 1,
    'enable_runstop':1}

SE4_wrist_roll_DW4={
    'eeprom_cfg': {
        'temperature_limit': 72,
        'max_voltage_limit': 29,
        'min_voltage_limit': 11,
        'pid': [32,32,0],
        'return_delay_time': 0,
        'angular_resolution': 1.0,
        'phase': 61, #61 for multi-turn, 45 for normal
        'max_pos_limit': 0,#0 for multi-turn, 4095 for normal
        'min_pos_limit': 0,
        'max_load_limit_pct': 48.0,
        'overload_safe': 25.0,
        'overload_time_ms': 1000,
        'overload_thresh': 48.0,
        'overcurrent': 150,
        'overcurrent_time_ms': 200,
        'protection_torque': 20,
        'overload_protection_time': 200,
        'enable_protection_overload':0,
        'enable_protection_current':1,
        'enable_protection_temp':1,
        'enable_protection_sensor':0,
        'enable_protection_voltage':0
    },
    'motion': {
        # Predefined motion profiles (vel: rad/s, accel: rad/s^2)
        'default': {'accel': 7.0, 'vel': 7.0},
        'fast': {'accel': 9.0, 'vel': 9.0},
        'max': {'accel': 12.0, 'vel': 12.0},
        'slow': {'accel': 4.0, 'vel': 4.0},
        'vel_brakezone_factor': 1,
        'vel_is_moving_thresh': 0.01
    },
    'id': 22,
    'set_safe_velocity': 1,
    'req_calibration': 1,
    'gr': 2.0,
    'usb_name': '/dev/hello-feetech-wrist',
    'retry_on_comm_failure': 1,
    'baud': 1000000,
    'range_pad_deg': [1.0, 1.0],
    'range_deg': [-245, 65],
    'homing_offset_bias_t': -127,              
    'homing_to_neg_limit': 1,
    'homing_pwm': -200,
    'flip_encoder_polarity': 1,
    'stall_backoff': 0.017,
    'stall_max_effort': 20.0,
    'stall_max_time': 1.0,
    'stall_min_vel': 0.1,
    'disable_torque_on_runstop': 1,
    'enable_torque_after_runstop': 1,
    'enable_runstop':1}


# ######### EndOfArm Defn ##############
"""
Define the EndOfArm DynamixelXChain parameters
Point to which joint devices & parameters to load for the chain
"""

SE4_eoa_wrist_dw4_tool_nil={
        'py_class_name': 'EOA_Wrist_DW4_Tool_NIL',
        'py_module_name': 'stretch4_body.subsystem.end_of_arm.end_of_arm_tools',
        'use_group_sync_read': 0,
        'use_group_sync_write':0,
        'retry_on_comm_failure': 1,
        'baud': 1000000,
        'i_feedforward_payload':0.0,
        #'dxl_latency_timer': 64,
        'wrist': 'eoaw_dw4',
        'tool': 'eoat_nil',
        'stow': {
            'arm': 0.0,
            'lift': 0.15,
            'wrist_pitch': 0.0,
            'wrist_roll': 0.0,
            'wrist_yaw': 3.14,
        },

        'devices': {
            'wrist_pitch': {
                'py_class_name': 'WristPitch',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_pitch',
                'device_params': 'SE4_wrist_pitch_DW4'
            },
            'wrist_roll': {
                'py_class_name': 'WristRoll',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_roll',
                'device_params': 'SE4_wrist_roll_DW4'
            },
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_yaw',
                'device_params': 'SE4_wrist_yaw_DW4'
            }
            }
            }


SE4_eoa_wrist_dw4_tool_sg4={
        'py_class_name': 'EOA_Wrist_DW4_Tool_SG4',
        'py_module_name': 'stretch4_body.subsystem.end_of_arm.end_of_arm_tools',
        'use_group_sync_read': 0, #1
        'use_group_sync_write':0, #1
        'retry_on_comm_failure': 1,
        'baud': 1000000,
        'i_feedforward_payload':0.3,
        #'dxl_latency_timer': 64,
        'wrist': 'eoaw_dw4',
        'tool': 'eoat_sg4',
        'stow': {
            'arm': 0.0,
            'lift': 0.15,
            'wrist_pitch': 0.0,
            'wrist_roll': 0.0,
            'wrist_yaw': 3.14,
            'stretch_gripper': 0
        },

        'devices': {
            'wrist_pitch': {
                'py_class_name': 'WristPitch',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_pitch',
                'device_params': 'SE4_wrist_pitch_DW4'
            },
            'wrist_roll': {
                'py_class_name': 'WristRoll',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_roll',
                'device_params': 'SE4_wrist_roll_DW4'
            },
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_yaw',
                'device_params': 'SE4_wrist_yaw_DW4'
            },
            'stretch_gripper': {
                'py_class_name': 'StretchGripper4',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.stretch_gripper',
                'device_params': 'SE4_stretch_gripper_DW4'
            }
            },
        'ros': {
            'joints': 
            [{
                'py_module_name': 'stretch_core.command_groups',
                'py_class_name': 'GripperCommandGroup',
            }]
            }
        }

SE4_eoa_wrist_dw4_tool_pg4={
        'py_class_name': 'EOA_Wrist_DW4_Tool_PG4',
        'py_module_name': 'stretch4_body.subsystem.end_of_arm.end_of_arm_tools',
        'use_group_sync_read': 0, #1
        'use_group_sync_write':0, #1
        'retry_on_comm_failure': 1,
        'baud': 1000000,
        'i_feedforward_payload':0.3,
        #'dxl_latency_timer': 64,
        'wrist': 'eoaw_dw4',
        'tool': 'eoat_pg4',
        'stow': {
            'arm': 0.0,
            'lift': 0.15,
            'wrist_pitch': 0.0,
            'wrist_roll': 0.0,
            'wrist_yaw': 3.14,
            'parallel_gripper': 0
        },
        'collision_mgmt': {
            'k_brake_distance': {'wrist_pitch': 0.25, 'wrist_yaw': 0.25, 'wrist_roll': 0.25},
            'collision_pairs': {
                'link_wrist_pitch_TO_base_link': {'link_pts': 'link_wrist_pitch', 'link_cube': 'base_link','detect_as': 'pts'},
                'link_wrist_yaw_bottom_TO_base_link': {'link_pts': 'link_wrist_yaw_bottom', 'link_cube': 'base_link','detect_as': 'pts'}},
            'joints': {'lift': [{'motion_dir': 'neg', 'collision_pair': 'link_wrist_pitch_TO_base_link'},
                                {'motion_dir': 'neg', 'collision_pair': 'link_wrist_yaw_bottom_TO_base_link'}]}},

        'devices': {
            'wrist_pitch': {
                'py_class_name': 'WristPitch',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_pitch',
                'device_params': 'SE4_wrist_pitch_DW4'
            },
            'wrist_roll': {
                'py_class_name': 'WristRoll',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_roll',
                'device_params': 'SE4_wrist_roll_DW4'
            },
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_yaw',
                'device_params': 'SE4_wrist_yaw_DW4'
            },
            'parallel_gripper': {
                'py_class_name': 'ParallelGripper',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.parallel_gripper',
                'device_params': 'SE4_parallel_gripper_DW4'
            }
            },
        'ros': {'joints': 
            [{
                'py_module_name': 'stretch_core.command_groups',
                'py_class_name': 'ParallelGripperCommandGroup',
            }]
            }
        }

SE4_eoa_wrist_dw4_tool_tablet={
        'py_class_name': 'EOA_Wrist_DW4_Tool_Tablet',
        'py_module_name': 'stretch4_body.subsystem.end_of_arm.end_of_arm_tools',
        'use_group_sync_read': 0,
        'use_group_sync_write':0,
        'retry_on_comm_failure': 1,
        'baud': 1000000,
        'i_feedforward_payload':0.0,
        #'dxl_latency_timer': 64,
        'wrist': 'eoaw_dw4',
        'tool': 'eoat_tablet',
        'stow': {
            'arm': 0.0,
            'lift': 0.30,
            'wrist_pitch': 0.784,
            'wrist_roll': 0.0,
            'wrist_yaw': 3.633,
        },
        'devices': {
            'wrist_pitch': {
                'py_class_name': 'WristPitch',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_pitch',
                'device_params': 'SE4_wrist_pitch_DW4'
            },
            'wrist_roll': {
                'py_class_name': 'WristRoll',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_roll',
                'device_params': 'SE4_wrist_roll_DW4'
            },
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_yaw',
                'device_params': 'SE4_wrist_yaw_DW4'
            }
            }
            }


SE4_eoa_wrist_dw4_tool_calibration={
        'py_class_name': 'EOA_Wrist_DW4_Tool_Calibration',
        'py_module_name': 'stretch4_body.subsystem.end_of_arm.end_of_arm_tools',
        'use_group_sync_read': 0,
        'use_group_sync_write':0,
        'retry_on_comm_failure': 1,
        'baud': 1000000,
        'i_feedforward_payload':0.2,
        #'dxl_latency_timer': 64,
        'wrist': 'eoaw_dw4',
        'tool': 'eoat_calibration',
        'stow': {
            'arm': 0.0,
            'lift': 0.30,
            'wrist_pitch': -0.883,
            'wrist_roll': 0.123,
            'wrist_yaw': 3.633,
        },
        'devices': {
            'wrist_pitch': {
                'py_class_name': 'WristPitch',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_pitch',
                'device_params': 'SE4_wrist_pitch_DW4'
            },
            'wrist_roll': {
                'py_class_name': 'WristRoll',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_roll',
                'device_params': 'SE4_wrist_roll_DW4'
            },
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_yaw',
                'device_params': 'SE4_wrist_yaw_DW4'
            }
            }
            }

# ###################################33
# Baseline Nominal Params
nominal_params={
    # #################################
    #Each EOA will get expanded at runtime into its full parameter dictionary
    # Eg, supported_eoa.tool_none --> adds the wrist_yaw param dict to nominal_params
    # Add all formally supported EOA to this list
    'supported_eoa': [
        'eoa_wrist_dw4_tool_nil',
        'eoa_wrist_dw4_tool_sg4',
        'eoa_wrist_dw4_tool_pg4',
        'eoa_wrist_dw4_tool_tablet',
        'eoa_wrist_dw4_tool_calibration'
    ],
    'supported_eoa_metadata': {
        'eoa_wrist_dw4_tool_nil': {
            'name': 'No Tool',
            'description': 'No tool attached to the robot.'
        },
        'eoa_wrist_dw4_tool_sg4': {
            'name': 'Stretch Gripper',
            'description': 'The standard Stretch 4 compliant gripper.'
        },
        'eoa_wrist_dw4_tool_pg4': {
            'name': 'Parallel Jaw Gripper',
            'description': 'A parallel jaw gripper for the Stretch 4.'
        },
        'eoa_wrist_dw4_tool_tablet': {
            'name': 'Tablet Holder',
            'description': 'A holder for a tablet device.'
        },
        'eoa_wrist_dw4_tool_calibration': {
            'name': 'Calibration Tool',
            'description': 'A tool for calibrating the robot\'s head sensors.'
        }
    },
    'eoa_wrist_dw4_tool_nil': SE4_eoa_wrist_dw4_tool_nil,
    'eoa_wrist_dw4_tool_sg4': SE4_eoa_wrist_dw4_tool_sg4,
    'eoa_wrist_dw4_tool_pg4': SE4_eoa_wrist_dw4_tool_pg4,
    'eoa_wrist_dw4_tool_tablet': SE4_eoa_wrist_dw4_tool_tablet,
    'eoa_wrist_dw4_tool_calibration': SE4_eoa_wrist_dw4_tool_calibration,
    # 'line_sensor_vel_limit':{
    #     'sensor_normals':{ #CCW from robot forward
    #         'hello-gs2-0':180,
    #         'hello-gs2-1':120,
    #         'hello-gs2-2':60,
    #         'hello-gs2-3':0,
    #         'hello-gs2-4':300,
    #         'hello-gs2-5':240},
    #     'phase_adj':330.0, #adjust to forward
    #     'dropoff_deg':20.0
    # },
    'line_sensor_loop':{
        'loop_rate_Hz': 250, #Run fast as polling serial, 6 ch, 30hz
        'cpu_affinity': 16,
        'flip_range_ordering':True,
        'sensor_names': ['sensor_0', 'sensor_1', 'sensor_2', 'sensor_3', 'sensor_4', 'sensor_5'], #CW from robot forward
        'bus_sensor_map': [ [ 1, 0 ],[ 3, 2 ],[ 5, 4 ] ], #Remap if cables plugged in differently
        
        'line_sensor_geometry':{
            'pixart_report_num': 320, #Num range samples per sensors
            'sensor_horizontal_fov_degrees': 103.0,
            'sensor_vertical_fov_degrees': 84.0,
            'sensor_height_above_floor_mm':79.24, #?? Height of the  sensor above the floor, per CAD
            'sensor_pitch_diameter_mm': 378.28, #??Per quentin/cad
            'emitter_height_above_floor_mm':100.67, # Height of the emitter above the floor, per CAD
            'emitter_horizontal_fov_degrees': 110.5,
            'emitter_pitch_diameter_mm': 404.04, #Per quentin/cad
            'sensor_angle_down_deg': 26.0, #Angle of the sensor below the horizontal.
            'sensor_angles_deg':[10.18, 39.64, 80.36, 39.64, 80.36, 39.64], #How arranged on base, CW from robot forward
            'sensor_normals_deg':[0.0, 60.0, 120.0, 180.0, 240.0, 300.0], #How arranged on base, CW from robot forward
        },
        'line_sensor_cost_map':{
            'base_radius_mm': 170.0,
            'inflation_mm': 20.0
        },
        'line_sensor_cluster_tracker':{
             'match_thresh_m': 0.1,
             'max_age_s': 1.0,
             'thresh_cliff_mm': 10,
             'thresh_obstacle_mm': 10,
             'cluster_eps': 0.03,
             'cluster_min_points': 10,
             'min_width': 0.01
        }
    },
    'end_of_arm_loop': {
        'loop_rate_Hz': 50,
        'cpu_affinity': 16
    },
    'omnibase': {
        'forward_dir': 'calder',
        'gr': 6,
        'use_vel_traj': 1,
        'motion': {
            # Base motion profiles. w_r: rotation (rad/s), xy_m: translation (m/s)
            'default': {
                'accel_w_r': 2.0,
                'vel_w_r': 2.0,
                'accel_xy_m': 0.25,
                'vel_xy_m': 0.3},
            'fast': {
                'accel_w_r': 3.0,
                'vel_w_r': 3.0,
                'accel_xy_m': 0.4,
                'vel_xy_m': 0.4},
            'max': {
                'accel_w_r': 4.0,
                'vel_w_r': 4.0,
                'accel_xy_m': 0.5,
                'vel_xy_m': 0.6},
            'slow': {
                'accel_w_r': 1.0,
                'vel_w_r': 1.0,
                'accel_xy_m': 0.1,
                'vel_xy_m': 0.1}},
        'sentry_fast_motion_allowed_on_stow':{
            # Safety limits applied when the robot is NOT in a compact stow position
            'limit_accel_m': 0.15,
            'limit_vel_m': 0.1,
            'max_arm_extension_m': 0.03,
            'max_lift_height_m': 0.3,
            'min_wrist_yaw_rad': 2.54},
        'wheel_diameter_m': 0.200, #Per quentin/cad
        'base_radius_m': 0.174,  # Per quentin/cad, to centerline of wheel
        # Flag to enable/disable collision detection (stops movement on unexpected effort)
        'enable_guarded_mode': 1},
    'arm':{
        'usb_name': '/dev/hello-motor-arm',
        'use_vel_traj': 1,
        'chain_pitch': .009525,
        'chain_sprocket_teeth': 15,
        'gr_spur': 2.33333,
        'i_feedforward': 0,
        'calibration_range_bounds':[0.547, 0.558],
        # Absolute hardstop limits. Tune carefully to prevent physical damage.
        'range_m': [0.0, 0.55],#0.555], clipp last 5mm to protect hardstops
        # Homing procedure settings. contact_sensitivity adjusts how hard the arm hits the hardstop.
        'homing': {'contact_sensitivity': 0.3, 'end_pos': 0.1, 'v_m': 0.2, 'a_m': 0.3, 'to_positive_stop': False,'safety_hold':1,'safety_stiffness':1.0},
        'motion':{
            # Predefined motion profiles (m/s, m/s^2)
            'default':{
                'accel_m': 0.4,
                'vel_m': 0.4},
            'fast':{
                'accel_m': 0.6,
                'vel_m': 0.6},
            'max':{
                'accel_m': 0.7,
                'vel_m': 0.7},
            'slow':{
                'accel_m': 0.1,
                'vel_m': 0.1},
                'vel_brakezone_factor': 0.03},
                'set_safe_velocity': 1},
    'fee_comm_errors': {
        'warn_every_s': 1.0,
        'warn_above_rate': 0.1,
        'verbose': 0},
    'end_of_arm':{
        'usb_name': '/dev/hello-feetech-wrist',
        'devices':{},
        'use_group_sync_read': 0,
        'use_group_sync_write':0,
        'retry_on_comm_failure': 1,
        'baud': 1000000,
        'dxl_latency_timer': 64,
        'ros': {'joints': []}
        },
    'hello-motor-omni-2': {
        'gains': {
            'drv8262_min_vref':25, #76: 10, 56: 25
            'effort_LPF': 2.0,
            'enable_guarded_mode': 0,
            'enable_runstop': 1,
            'enable_sync_mode': 1,
            'enable_vel_watchdog': 1,
            'flip_effort_polarity': 0,
            'flip_encoder_polarity': 0,
            'k_calibration_step': 0.3,
            'iMax_neg': -7.7,
            'iMax_pos': 7.7,
            'i_contact_neg': -2.0,
            'i_contact_pos': 2.0,
            'i_safety_feedforward': 0.0,
            'toff_setting': 0 ,
            'decay_setting': 0,
            'pKd_d': 0.05,
            'pKi_d': 5.0,
            'pKi_limit': 200.0,
            'pKp_d': 100.0,
            'pLPF': 100,
            'voltage_LPF': 1.0,
            'phase_advance_d': 1.8,
            'pos_near_setpoint_d': 6.0,
            'safety_hold': 0,
            'safety_stiffness': 1.0,
            'vKd_d': 0,
            'vKi_d': 3.5,
            'vKi_limit': 2000,
            'vKp_d': 0.6,
            'vLPF': 30,
            'vTe_d': 50,
            'vel_near_setpoint_d': 3.5,
            'vel_status_LPF': 10.0,
            'coeff_acc_pos': -1.69, 
            'coeff_intercept_pos': 1269.28,
            'coeff_acc_neg': 2.96, 
            'coeff_intercept_neg': -1454.33,
            },
        'guarded_contact':{
            # Guarded contact sensitivities. Lower coefficients make the robot stop more easily upon contact.
            'off': 0,
          'sensitivity_default':{
              'coeff_sensitivity_pos': 0.5,
              'coeff_sensitivity_neg': 0.5
          },
            'sensitivity_high':{
              'coeff_sensitivity_pos': 0.0,
              'coeff_sensitivity_neg': 0.0
          },
            'sensitivity_low':{
              'coeff_sensitivity_pos': 1.0,
              'coeff_sensitivity_neg': 1.0
          }
        },
        'holding_torque': 1.26,
        'motion': {
            'accel': 15,
            'vel': 25},
        'rated_current': 2.8,
        'transport':{
            'qid': 2,
            'default_backend': 1
        },
    },
    'hello-motor-omni-1': {
        'gains': {
            'drv8262_min_vref':25, #76: 10, 56: 25
            'effort_LPF': 2.0,
            'enable_guarded_mode': 0,
            'enable_runstop': 1,
            'enable_sync_mode': 1,
            'enable_vel_watchdog': 1,
            'flip_effort_polarity': 0,
            'flip_encoder_polarity': 0,
            'k_calibration_step': 0.3,
            'iMax_neg': -7.7,
            'iMax_pos': 7.7,
            'i_contact_neg': -2.0,
            'i_contact_pos': 2.0,
            'i_safety_feedforward': 0.0,
            'toff_setting': 0 ,
            'decay_setting': 0,
            'pKd_d': 0.05,
            'pKi_d': 5.0,
            'pKi_limit': 200.0,
            'pKp_d': 100.0,
            'pLPF': 100,
            'voltage_LPF': 1.0,
            'phase_advance_d': 1.8,
            'pos_near_setpoint_d': 6.0,
            'safety_hold': 0,
            'safety_stiffness': 1.0,
            'vKd_d': 0,
            'vKi_d': 3.5,
            'vKi_limit': 2000,
            'vKp_d': 0.6,
            'vLPF': 30,
            'vTe_d': 50,
            'vel_near_setpoint_d': 3.5,
            'vel_status_LPF': 10.0,
            'coeff_acc_pos':0.89, 
            'coeff_intercept_pos':741.12,
            'coeff_acc_neg':0.99, 
            'coeff_intercept_neg':-985.91,
            },
        'guarded_contact':{
            # Guarded contact sensitivities. Lower coefficients make the robot stop more easily upon contact.
            'off':0,
          'sensitivity_default':{
              'coeff_sensitivity_pos': 0.5,
              'coeff_sensitivity_neg': 0.5
          },
            'sensitivity_high':{
              'coeff_sensitivity_pos': 0.0,
              'coeff_sensitivity_neg': 0.0
          },
            'sensitivity_low':{
              'coeff_sensitivity_pos': 1.0,
              'coeff_sensitivity_neg': 1.0
          }
        },
        'holding_torque': 1.26,
        'motion': {
            'accel': 15,
            'vel': 25},
        'rated_current': 2.8,
        'transport':{
            'qid': 1,
            'default_backend': 1
        }},
    'hello-motor-omni-0': {
        'gains': {
            'drv8262_min_vref':25, #76: 10, 56: 25
            'effort_LPF': 2.0,
            'enable_guarded_mode': 0,
            'enable_runstop': 1,
            'enable_sync_mode': 1,
            'enable_vel_watchdog': 1,
            'flip_effort_polarity': 0,
            'flip_encoder_polarity': 0,
            'k_calibration_step': 0.3,
            'iMax_neg': -7.7,
            'iMax_pos': 7.7,
            'i_contact_neg': -2.0,
            'i_contact_pos': 2.0,
            'i_safety_feedforward': 0.0,
            'toff_setting': 0 ,
            'decay_setting': 0,
            'pKd_d': 0.05,
            'pKi_d': 5.0,
            'pKi_limit': 200.0,
            'pKp_d': 100.0,
            'pLPF': 100,
            'voltage_LPF': 1.0,
            'phase_advance_d': 1.8,
            'pos_near_setpoint_d': 6.0,
            'safety_hold': 0,
            'safety_stiffness': 1.0,
            'vKd_d': 0,
            'vKi_d': 3.5,
            'vKi_limit': 2000,
            'vKp_d': 0.6,
            'vLPF': 30,
            'vTe_d': 50,
            'vel_near_setpoint_d': 3.5,
            'vel_status_LPF': 10.0,
            'coeff_acc_pos': 0.73, 
            'coeff_intercept_pos': 1132.0,
            'coeff_acc_neg': -0.07, 
            'coeff_intercept_neg': -864.24,
            },
        'guarded_contact':{
            # Guarded contact sensitivities. Lower coefficients make the robot stop more easily upon contact.
            'off':0,
          'sensitivity_default':{
              'coeff_sensitivity_pos': 0.5,
              'coeff_sensitivity_neg': 0.5
          },
            'sensitivity_high':{
              'coeff_sensitivity_pos': 0.0,
              'coeff_sensitivity_neg': 0.0
          },
            'sensitivity_low':{
              'coeff_sensitivity_pos': 1.0,
              'coeff_sensitivity_neg': 1.0
          }
        },
        'holding_torque': 1.26,
        'motion': {
            'accel': 15,
            'vel': 25},
        'rated_current': 2.8,
        'transport':{
            'qid': 0,
            'default_backend': 1
        }},
    'hello-motor-arm':{
        'gains':{
            'drv8262_min_vref':25, #76: 10, 56: 25
            'effort_LPF': 10.0,
            'enable_guarded_mode': 1,
            'enable_runstop': 1,
            'enable_sync_mode': 1,
            'enable_vel_watchdog': 1,
            'flip_effort_polarity': 1,
            'flip_encoder_polarity': 1,
            'iMax_neg': -7.7,
            'iMax_pos': 7.7,
            'i_contact_neg': -2.0,
            'i_contact_pos': 2.0,
            'i_safety_feedforward': 0.0,
            'k_calibration_step':0.3,
            'toff_setting': 0 ,
            'decay_setting': 0,
            'pKd_d': 0.05,
            'pKi_d': 10,
            'pKi_limit': 200.0,
            'pKp_d': 125.0,
            'pLPF': 100,
            'voltage_LPF':1.0,
            'phase_advance_d': 1.8,
            'pos_near_setpoint_d': 6.0,
            'safety_hold': 0,
            'safety_stiffness': 0.0,
            'vKd_d': 0,
            'vKi_d': 0.005,
            'vKi_limit': 200,
            'vKp_d': 0.2,
            'vLPF': 30,
            'vTe_d': 50,
            'vel_near_setpoint_d': 3.5,
            'vel_status_LPF': 10.0,
            'coeff_acc_pos': -0.15, 
            'coeff_intercept_pos': 798.37,
            'coeff_acc_neg': -5.31, 
            'coeff_intercept_neg': -800.62,
            },
        'holding_torque': 1.26,
        'motion':{
            'accel': 15,
            'vel': 25},
        'rated_current': 2.8,
        'transport':{
            'qid': 3,
            'default_backend': 1
        },
        'guarded_contact':{
            # Guarded contact sensitivities. Lower coefficients make the robot stop more easily upon contact.
            'off': 0,
          'sensitivity_default':{
              'coeff_sensitivity_pos': 0.5,
              'coeff_sensitivity_neg': 0.5
          },
            'sensitivity_high':{
              'coeff_sensitivity_pos': 0.0,
              'coeff_sensitivity_neg': 0.0
          },
            'sensitivity_low':{
              'coeff_sensitivity_pos': 1.0,
              'coeff_sensitivity_neg': 1.0
          }
        }},
    'hello-motor-lift':{
            'gains':{
            'drv8262_min_vref':10, #76: 10, 56: 25
            'effort_LPF': 2.0,
            'enable_guarded_mode': 1,
            'enable_runstop': 1,
            'enable_sync_mode': 1,
            'enable_vel_watchdog': 1,
            'flip_effort_polarity': 0,
            'flip_encoder_polarity': 0,
            'k_calibration_step': 0.3,
            'iMax_neg': -7.7,
            'iMax_pos': 7.7,
            'i_contact_neg': -2.0,
            'i_contact_pos': 2.0,
            'i_safety_feedforward': 1.5,
            'toff_setting': 0 ,
            'decay_setting': 1,
            'pKd_d': 0.05,
            'pKi_d': 50.0,
            'pKi_limit': 200.0,
            'pKp_d': 125.0,
            'pLPF': 100,
            'voltage_LPF':1.0,
            'phase_advance_d': 1.8,
            'pos_near_setpoint_d': 6.0,
            'safety_hold': 1,
            'safety_stiffness': 0.0,
           'vKd_d': 0,
            'vKi_d': 3.5,
            'vKi_limit': 2000,
            'vKp_d': 0.3,
            'vLPF': 30,
            'vTe_d': 50,
            'vel_near_setpoint_d': 3.5,
            'vel_status_LPF': 10.0,
            'coeff_acc_pos': 50.99,
            'coeff_intercept_pos': 1024.65,
            'coeff_acc_neg': -4.37,
            'coeff_intercept_neg': 200.90,
            },
        'holding_torque': 1.9,
        'motion':{
            'accel': 15,
            'vel': 12},
        'rated_current': 2.95,
        'transport':{
            'qid': 4,
            'default_backend': 1
        },
        'guarded_contact':{
            # Guarded contact sensitivities. Lower coefficients make the robot stop more easily upon contact.
            'off': 0,
          'sensitivity_default':{
              'coeff_sensitivity_pos': 0.5,
              'coeff_sensitivity_neg': 0.5
          },
            'sensitivity_high':{
              'coeff_sensitivity_pos': 0.0,
              'coeff_sensitivity_neg': 0.0
          },
            'sensitivity_low':{
              'coeff_sensitivity_pos': 1.0,
              'coeff_sensitivity_neg': 1.0
          }
        }},
    'lift':{
        'usb_name': '/dev/hello-motor-lift',
        'use_vel_traj': 1,
        'calibration_range_bounds': [1.197,1.203 ],
        'i_feedforward': 1.5,
        # Absolute hardstop limits. Tune carefully to prevent physical damage.
        'range_m' : [0.0, 1.20], #Calder no shells, [0.0, 1.21], #Dali w/ shells, [0.0, 1.20]
        # Homing procedure settings. contact_sensitivity adjusts how hard the lift hits the hardstop.
        'homing': {'contact_sensitivity': 0.2,'end_pos':0.5,'v_m':0.15,'a_m':0.3,'to_positive_stop':True,'safety_hold':1,'safety_stiffness':0.7},
        'belt_pitch_m': 0.005,
          'motion':{
            # Predefined motion profiles (m/s, m/s^2)
            'default':{
              'accel_m': 0.3,
              'vel_m': 0.3},
            'fast':{
              'accel_m': 0.5,
              'vel_m': 0.4},
            'max':{
              'accel_m': 1.0,
              'vel_m': 0.5},
            'slow':{
              'accel_m': 0.2,
              'vel_m': 0.15},
        'vel_brakezone_factor': 0.01},
        'set_safe_velocity': 1,
          'pinion_t': 22},
    'imu':{
        'config': {
            'gyro_zero_offsets': [0.0, 0.0, 0.0],
            'gravity_vector_scale': 1.0,
            'mag_offsets': [0.0, 0.0, 0.0],
            'mag_softiron_matrix': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],#[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            'rate_gyro_vector_scale': 1.0,
            'accel_LPF': 100.0}},
    'power_periph':{
      'usb_name': '/dev/hello-power-periph',
      'transport':{
          'qid': 5,
          'default_backend': 1,
          },
      'base_fan_off': 70,
      'base_fan_on': 82,

      'config':{
        # Pitch/Roll derivative threshold to detect a base bump
        'bump_thresh': 20.0,
        'current_LPF': 10.0,
        'high_current_eoa_alert': 8.0,
        'runstop_at_high_current_eoa': 1,
        'disable_eoa_at_high_current_eoa': 1,
        'low_soc_alert': 10,
        'runstop_at_low_soc': 1,
        'high_current_alert': 9.5,
        'runstop_at_high_current': 0,
        'stop_at_runstop': 1,
        'temp_LPF': 1.0,
        'voltage_LPF': 1.0,
        'nuc_safe_shutdown':1},
      'firebase':{
        'url': 'NA',
        'api_key':'NA',
        'user_email':'NA',
        'user_password': 'NA',
        'network_ssid': 'NA',
        'network_password':'NA'}},
    'robot_server':{
        # 'control_loop_rate_Hz': 100,
        # 'network_loop_rate_Hz':50,
        # 'cpu_affinity': 17
        },
    # ########### ROUTINES ################################
    'routine_manager':{'controllers': ['routine_nop', 'routine_blind_dock', 'routine_lift_home','routine_arm_home',
                         'routine_wrist_joint_home','routine_end_of_arm_home','routine_robot_home','routine_robot_stow'],},
    'routine_nop':{
        'py_module_name': 'stretch4_body.behavior.routines.routine',
        'py_class_name': 'RoutineNOP',
        'enabled': 1
    },
    'routine_robot_stow':{
        'py_module_name': 'stretch4_body.behavior.routines.routine_stow',
        'py_class_name': 'RoutineRobotStow',
        'enabled': 1
    },
    'routine_lift_home': {
        'py_module_name': 'stretch4_body.behavior.routines.routine_homing',
        'py_class_name': 'RoutineLiftHome',
        'required_subsystems': ['lift'],
        'enabled': 1
    },
    'routine_blind_dock': {
        'py_module_name': 'stretch4_body.behavior.routines.routine_blind_dock',
        'py_class_name': 'RoutineBlindDock',
        'required_subsystems': ['omnibase','power_periph'],
        'num_retries': 3,
        't_settle': 1.0,
        'enabled': 1
    },
    'routine_arm_home': {
        'py_module_name': 'stretch4_body.behavior.routines.routine_homing',
        'py_class_name': 'RoutineArmHome',
        'required_subsystems': ['arm'],
        'enabled': 1
    },
    'routine_wrist_joint_home': {
        'py_module_name': 'stretch4_body.behavior.routines.routine_homing',
        'py_class_name': 'RoutineWristJointHome',
        'enabled': 1
    },
    'routine_end_of_arm_home': {
        'py_module_name': 'stretch4_body.behavior.routines.routine_homing',
        'py_class_name': 'RoutineEndOfArmHome',
        'enabled': 1
    },
    'routine_robot_home': {
        'py_module_name': 'stretch4_body.behavior.routines.routine_homing',
        'py_class_name': 'RoutineRobotHome',
        'exclude_sentry_pause': ['sentry_status_logger','sentry_ubuntu_power_management','sentry_cpu_temp'],
        'exclude_safe_motion_pause': ['safe_motion_overtilt_avoid'],
        'enabled':1
    },

    ############## SENTRIES #############################
    'sentry_manager': {'controllers':[
        'sentry_status_logger',
        'sentry_self_collision', 
        'sentry_ubuntu_power_management', 
        'sentry_battery_mgmt',
        'sentry_joint_runaway',
        'sentry_limit_vel_on_pose',
        'sentry_omnibase_guarded_contact', 
        'sentry_cpu_temp',
        'sentry_eye_animations']},
    'sentry_eye_animations': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_eye_animations',
        'py_class_name': 'SentryEyeAnimations',
        'behavior': 'curious',
        'required_subsystems': ['power_periph'],
        'enabled': 1,
    },
    'sentry_limit_vel_on_pose': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_limit_vel_on_pose',
        'py_class_name': 'SentryLimitVelOnPose',
        # Height below which the lift is considered "low" for speed limit calculations
        'lift_lower_safe_height_m': 0.2,
        # Flags to reduce base speed depending on how extended the lift/arm are (to prevent tipping)
        'limit_omnibase_translation_by_lift': 1,
        'limit_omnibase_rotation_by_arm': 0,
        'limit_omnibase_rotation_by_lift': 1,
        'required_subsystems': ['omnibase', 'arm', 'lift'],
        'enabled':1,
    },
    'sentry_self_collision': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_self_collision',
        'py_class_name': 'SentrySelfCollision',
        'urdf_joints_to_sentry':['joint_lift','joint_arm_l0','joint_arm_l1','joint_arm_l2','joint_arm_l3','joint_wrist_yaw','joint_wrist_pitch','joint_wrist_roll'],
        'required_subsystems': ['arm', 'lift','end_of_arm'],
        'enabled': 1,
    },
    'sentry_omnibase_guarded_contact': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_omnibase_guarded_contact',
        'py_class_name': 'SentryOmniBaseGuardedContact',
        'required_subsystems': ['omnibase', 'power_periph'],
        'enabled': 1,
    },
    'sentry_joint_runaway': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_joint_runaway',
        'py_class_name': 'SentryJointRunaway',
        'required_subsystems': ['power_periph'],
        'enabled': 1,
        'lift_runaway_vel':1.25
    },
    'sentry_battery_mgmt': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_battery_mgmt',
        'py_class_name': 'SentryBatteryMgmt',
        'required_subsystems': ['power_periph'],
        'enabled': 1,
        'enable_audio_alert': 1,
        'alert_period_low_battery': 20.0,
        'alert_period_shutdown': 5.0,
        'soc_low_battery_warning':20.0,
        'soc_shutdown_warning':1.0, #Will shutdown at 0 SOC
        'low_voltage_shutdown_warning': 25.0 #Will shutdown at 24.85V
    },
    'sentry_ubuntu_power_management': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_ubuntu_power_management',
        'py_class_name': 'SentryUbuntuPowerManagement',
        'required_subsystems': [],
        'enabled': 1,
        'check_rate_seconds': 30
    },
    'sentry_status_logger': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_status_logger',
        'py_class_name': 'SentryStatusLogger',
        'required_subsystems': [],
        'enabled': 1,
        'check_rate': 100,
        'maximum_log_size_mb': 5000 # 5GB folder in ~/stretch_user/logs/status_logs
    },
    'sentry_cpu_temp': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_cpu_temp',
        'py_class_name': 'SentryCPUTemp',
        'required_subsystems': ['power_periph'],
        'enabled': 1,
        'loop_rate_Hz': 0.5,
        'base_fan_on': 60.0,
        'base_fan_off': 50.0,
        'fan_control': 1
    },
    ############## SAFE MOTIONS #############################
    'safe_motion_manager': {'controllers': ['safe_motion_overtilt_avoid']},
    'safe_motion_line_sensor_vel_limit': {
        'py_module_name': 'stretch4_body.behavior.safe_motions.safe_motion_line_sensor_vel_limit',
        'py_class_name': 'SafeMotionLineSensorVelLimit',
        'required_subsystems': ['omnibase'],
        'enabled':0},
    'safe_motion_overtilt_avoid': {
        'py_module_name': 'stretch4_body.behavior.safe_motions.safe_motion_overtilt_avoid',
        'py_class_name': 'SafeMotionOvertiltAvoid',
        # Angles (deg) before the robot pauses motion to prevent tipping over
        'gravity_tilt_thresh_deg':{'default':6.0,'conservative':9.0,'aggressive':3.0},
        'enable_gravity_tilt':1,
        'required_subsystems': ['omnibase','power_periph'],
        'enabled':1,
        'enable_rerun_viz':0,
        'enable_audio_alert':1,
        'alert_period': 2.0
    },
    # ##################### Robot #################################
    'robot': {
        'batch_name': 'NA',
        'serial_no': 'NA',
        'model_name': 'SE4',
        'subsystems': ['lift', 'arm', 'omnibase', 'end_of_arm', 'power_periph'],
        'tool': 'eoa_wrist_dw4_tool_sg4',
        'enable_rate_log':1,
        'max_rate_log_samples':10000,
        'guarded_contact':{
            # Profiles defining how sensitive the robot should be to unexpected forces (collisions) during different activities
            'off':0,
            'default':{
                    'lift':'sensitivity_default',
                    'arm':'sensitivity_default',
                    'omnibase':'sensitivity_default'
            },
            'high_sensitivity_nav':{
                    'lift':'sensitivity_default',
                    'arm':'sensitivity_default',
                    'omnibase':'sensitivity_high'
            },
            'high_sensitivity_manipulation':{
                    'lift':'sensitivity_high',
                    'arm':'sensitivity_high',
                    'omnibase':'sensitivity_high'
            },
            'strong_manipulation':{
                    'lift':'sensitivity_low',
                    'arm':'sensitivity_low',
                    'omnibase':'sensitivity_default'
            },
        },
        'direct':{
            'start_body_thread':True,
            'start_eoa_thread':True,
            'start_sys_mon_thread':True,
            'EOAStatusThread_Hz': 15.0,
            'BodyStatusThread_Hz': 25.0,
            'SystemMonitorThread_Hz': 15.0,
            'SystemMonitorThread_monitor_downrate_int': 2,
            'SystemMonitorThread_trace_downrate_int': 1,
            'SystemMonitorThread_sentry_downrate_int': 1,
            'use_monitor': 0,
            'use_trace': 0,
            'use_sentries': 1},
        'server': {
            'control_loop_rate_Hz': 100,
            'max_push_command_rate_Hz':     100,
            'subsystems': [],#'line_sensor_loop'],
        },
    },
    'robot_monitor':{
        'monitor_runstop': 1,},
    'robot_sentry':{
        'omnibase':{
            'fast_motion_allowed_on_stow':0},
        'power_periph':{'fan_control': 1}},
    'robot_trace':{
        'n_samples_per_file':100,
        'duration_limit_minutes':10.0
    },
    'cameras': {
        'head_left': {
            "config": {
                "camera_device": "OAK-FFC-3P",
                "image_size": (1200, 1920),
                "fps": 30,
                "rotate_number_of_times": 1,
                "buffer_size": 2,
                "is_compressed": False,
                "is_lossless": False, # Only used if is_compressed is true
                "jpeg_quality": 90, # Only used if is_compressed is true and is_lossless is False
                "distortion_model": "DistortionModels.equidistant_with_recompute_extrinsics",
                "sensor_pixel_size_mm": 3.0/1000.0,
                "use_auto_exposure": True,
                "limit_max": None, # Only used if use_auto_exposure is True
                "exposure_time": None, # Only used if use_auto_exposure is False
                "iso": None # Only used if use_auto_exposure is False
            }
        },
        'head_right': {
            "config": {
                "camera_device": "OAK-FFC-3P",
                "image_size": (1200, 1920),
                "fps": 30,
                "rotate_number_of_times": -1,
                "buffer_size": 2,
                "is_compressed": False,
                "is_lossless": False, # Only used if is_compressed is true
                "jpeg_quality": 90, # Only used if is_compressed is true and is_lossless is False
                "distortion_model": "DistortionModels.equidistant_with_recompute_extrinsics",
                "sensor_pixel_size_mm": 3.0/1000.0,
                "use_auto_exposure": True,
                "limit_max": None, # Only used if use_auto_exposure is True
                "exposure_time": None, # Only used if use_auto_exposure is False
                "iso": None # Only used if use_auto_exposure is False
            }
        },
        'head_center': {
            "config": {
                "camera_device": "OAK-FFC-3P",
                # "image_size": (3040, 4056),  # Full 12MP resolution
                "image_size": (3040, 4032),  # Almost full 12MP resolution, 24 pixels subtracted to be divisible by 16 for compression
                "fps": 10,
                "rotate_number_of_times": -1,
                "buffer_size": 2,
                "is_compressed": False,
                "is_lossless": False, # Only used if is_compressed is true
                "jpeg_quality": 90, # Only used if is_compressed is true and is_lossless is False
                "distortion_model": "DistortionModels.wide_angle",
                "sensor_pixel_size_mm": 1.55/1000.0,
                "use_auto_exposure": True,
                "limit_max": None, # Only used if use_auto_exposure is True
                "exposure_time": None, # Only used if use_auto_exposure is False
                "iso": None # Only used if use_auto_exposure is False
            }
        },
        'gripper_left': {
            "config": {
                "camera_device": "OAK-D-SR",
                # Options for full FOV: (640x400), (800x500), (960x600), (1024x640), (1280x800)
                "image_size": (400, 640),
                # "image_size": (800, 1280),
                "fps": 30,
                "rotate_number_of_times": 0,
                "buffer_size": 2,
                "is_compressed": False,
                "is_lossless": False, # Only used if is_compressed is true
                "jpeg_quality": 90, # Only used if is_compressed is true and is_lossless is False
                "distortion_model": None,
                "use_auto_exposure": True,
                "limit_max": None, # Only used if use_auto_exposure is True
                "exposure_time": None, # Only used if use_auto_exposure is False
                "iso": None # Only used if use_auto_exposure is False
            }
        },
        'gripper_right': {
            "config": {
                "camera_device": "OAK-D-SR",
                # Options for full FOV: (640x400), (800x500), (960x600), (1024x640), (1280x800)
                "image_size": (400, 640),
                # "image_size": (800, 1280),
                "fps": 30,
                "rotate_number_of_times": 0,
                "buffer_size": 2,
                "is_compressed": False,
                "is_lossless": False, # Only used if is_compressed is true
                "jpeg_quality": 90, # Only used if is_compressed is true and is_lossless is False
                "distortion_model": None,
                "use_auto_exposure": True,
                "limit_max": None, # Only used if use_auto_exposure is True
                "exposure_time": None, # Only used if use_auto_exposure is False
                "iso": None # Only used if use_auto_exposure is False
            }
        },
    },
    'self_collision_mujoco':{
        'SE4':{'k_brake_distance': {'lift': 1.1, 'arm': 1.1,'wrist_pitch':1.1,'wrist_yaw':1.1,'wrist_roll':1.1},
               # 'ignore_links': ['link_wheel_0','link_wheel_1', 'link_wheel_2', 'link_wrist','gripper_camera_link','link_tool_attachment_site','link_camera_right',
               #                  'link_camera_left','link_camera_center'],
               'ignore_links': ['link_wheel_0','link_wheel_1', 'link_wheel_2','link_tool_attachment_site', 'link_grasp_center'],
               'exclusions':[
                   ["link_head", "link_lift"],
                   ["base_link", "link_mast"],
                   ["link_lift", "link_mast"],
                   ["link_lift", "link_head"],
                   ["link_arm_l4","link_head"],

                   ["link_wrist", "link_arm_l4"],
                   ["link_wrist", "link_arm_l3"],
                   ["link_wrist", "link_arm_l2"],
                   ["link_wrist", "link_arm_l1"],
                   ["link_wrist", "link_arm_l0"],
                   ["link_wrist", "link_lift"],

                   ["link_arm_l0", "link_arm_l1"],
                   ["link_arm_l0", "link_arm_l2"],
                   ["link_arm_l0", "link_arm_l3"],
                   ["link_arm_l0", "link_arm_l4"],
                   ["link_arm_l0", "link_lift"],

                   ["link_arm_l1", "link_arm_l2"],
                   ["link_arm_l1", "link_arm_l3"],
                   ["link_arm_l1", "link_arm_l4"],
                   ["link_arm_l1", "link_lift"],


                   ["link_arm_l2", "link_arm_l3"],
                   ["link_arm_l2", "link_arm_l4"],
                   ["link_arm_l2", "link_lift"],

                   ["link_arm_l3", "link_arm_l4"],
                   ["link_arm_l3", "link_lift"],

                   ["link_arm_l4", "link_lift"],
                   ["link_arm_l4", "link_mast"],

                   ["link_wrist_yaw", "link_arm_l0"],
                   ["link_wrist_yaw", "link_arm_l1"],
                   ["link_wrist_yaw", "link_arm_l2"],
                   ["link_wrist_yaw", "link_arm_l3"],
                   ["link_wrist_yaw", "link_arm_l4"],


                    ]},
        'eoa_wrist_dw4_tool_sg4':{'k_brake_distance': {},
               'exclusions':[
                   ["link_gripper_finger_left", "link_wrist_pitch"],
                   ["link_gripper_finger_left", "link_gripper_finger_right"],
                   ["link_gripper_finger_left", "link_gripper_fingertip_right"],
                   ["link_gripper_finger_right", "link_wrist_pitch"],
                   ["link_gripper_finger_right","link_gripper_fingertip_left"],
                   ["link_gripper_fingertip_left","link_gripper_fingertip_right"],
                   ["link_aruco_fingertip_left","link_aruco_fingertip_right"],
                   ["link_aruco_fingertip_left","link_gripper_finger_right"],
                   ["link_aruco_fingertip_left","link_gripper_fingertip_right"],
                   ["link_aruco_fingertip_right","link_gripper_finger_left"],
                   ["link_aruco_fingertip_right","link_gripper_fingertip_left"],

               ]},
        'eoa_wrist_dw4_tool_pg4':{'k_brake_distance': {},
               'exclusions':[
                   ["link_finger_left", "link_finger_right"],
               ]},
        'eoa_wrist_dw4_tool_nil':{'k_brake_distance': {},
               'exclusions':[

               ]},
        'eoa_wrist_dw4_tool_tablet':{'k_brake_distance': {},
               'exclusions':[

               ]},
        'eoa_wrist_dw4_tool_calibration':{'k_brake_distance': {},
               'exclusions':[

               ]}},
    'self_collision_loop': {
        'loop_rate_Hz': 60.0},
    'stretch_gamepad':{
        'enable_fn_button': 0,
        'function_cmd':'',
        'press_time_span':5},
    'params':['stretch_body.robot_params_SE4_eoa'],
    'ros': {
        'joints': [{
            'py_module_name': 'stretch_core.command_groups',
            'py_class_name': 'LiftCommandGroup',
        },
        {
            'py_module_name': 'stretch_core.command_groups',
            'py_class_name': 'ArmCommandGroup',
        },
        {
            'py_module_name': 'stretch_core.command_groups',
            'py_class_name': 'MobileBaseCommandGroup',
        },
        {
            'py_module_name': 'stretch_core.command_groups',
            'py_class_name': 'WristYawCommandGroup',
        },
        {
            'py_module_name': 'stretch_core.command_groups',
            'py_class_name': 'WristPitchCommandGroup',
        },
        {
            'py_module_name': 'stretch_core.command_groups',
            'py_class_name': 'WristRollCommandGroup',
        }],
    }
    }

