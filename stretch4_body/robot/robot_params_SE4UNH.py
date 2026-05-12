#Robot parameters for Stretch 4 UNH

# ######################### USER PARAMS ##################################################
#Template for the generated file: stretch_user_params.yaml
user_params_header='#User parameters\n' \
                   '#You can override nominal settings here\n' \
                   '#USE WITH CAUTION. IT IS POSSIBLE TO CAUSE UNSAFE BEHAVIOR OF THE ROBOT \n'

user_params_template={
    'robot': {'NA': 0}} #Include this just as an example

# ###################### CONFIGURATION PARAMS #####################################################
#Template for the generated file: stretch_configuration_params.yaml
#Configuration parameters may have variation across the fleet of robots
configuration_params_header='#Parameters that are specific to this robot\n' \
                            '#Do not edit, instead edit stretch_user_params.yaml\n'

configuration_params_template={
    'hello-motor-lift':{'serial_no': 'NA'},
    'hello-motor-omni-0':{'serial_no': 'NA'},
    'hello-motor-omni-1':{'serial_no': 'NA'},
    'hello-motor-omni-2': {'serial_no': 'NA'},
    'power-periph':{
        'firebase': {
            'url': 'NA',
            'api_key': 'NA',
            'user_email': 'NA',
            'user_password': 'NA',
            'network_ssid': 'NA',
            'network_password': 'NA'}},
    'robot':{
        'batch_name': 'NA',
        'serial_no': 'NA',
        'model_name':'SE4UNH'}}

# ###################### NOMINAL PARAMS #####################################################
#Parameters that are common across the SE4 fleet

# ###################################33
# Baseline Nominal Params
nominal_params={
    'supported_eoa': ['eoa_wrist_nil_tool_unh'],
    'supported_eoa_metadata': {
        'eoa_wrist_nil_tool_unh': {
            'name': 'No Tool',
            'description': 'No tool attached to the robot.'
        }
    },
    'eoa_wrist_nil_tool_unh': {
        'devices': {}, # intentionally empty
        'i_feedforward_payload': 0.0,
        'stow': {
            'lift': 0.33
        },
    },
    'omnibase': {
        'forward_dir': 'calder',
        'gr': 6,
        'use_vel_traj': 1,
        'motion': {
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
            'limit_accel_m': 0.15,
            'limit_vel_m': 0.1,
            'max_arm_extension_m': 0.03,
            'max_lift_height_m': 0.3,
            'min_wrist_yaw_rad': 2.54},
        'wheel_diameter_m': 0.200, #Per quentin/cad
        'base_radius_m': 0.174,  # Per quentin/cad, to centerline of wheel
        'enable_guarded_mode': 1},


    'fee_comm_errors': {
        'warn_every_s': 1.0,
        'warn_above_rate': 0.1,
        'verbose': 0},
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
            'qid': 0,
            'default_backend': 1
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
            'i_safety_feedforward': 0.8,
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
            'coeff_acc_pos': 3.69, 
            'coeff_intercept_pos': 832.59,
            'coeff_acc_neg': -0.54, 
            'coeff_intercept_neg': 1.49,
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
        'calibration_range_bounds': [1.094, 1.106],
        'i_feedforward': 0.8,
        'range_m' : [0.0, 1.1],
        'homing': {'contact_sensitivity': 0.7, 'end_pos': 0.5, 'v_m': 0.25, 'a_m': 0.3, 'to_positive_stop': True,'safety_hold':1,'safety_stiffness':0.7},
        'belt_pitch_m': 0.005,
          'motion':{
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
            'accel_LPF':100.0}},
    'power_periph':{
      'usb_name': '/dev/hello-power-periph',
      'transport':{
          'qid': 5,
          'default_backend': 1,
          },
      'base_fan_off': 70,
      'base_fan_on': 82,
      'config':{
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
        'nuc_safe_shutdown':0},
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

    'routine_manager': {'controllers': ['routine_nop', 'routine_blind_dock', 'routine_lift_home',
                                        'routine_robot_stow','routine_robot_home'], },


    'routine_nop': {
        'py_module_name': 'stretch4_body.behavior.routines.routine',
        'py_class_name': 'RoutineNOP',
        'enabled': 1
    },
    'routine_robot_stow': {
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
    'routine_robot_home': {
        'py_module_name': 'stretch4_body.behavior.routines.routine_homing',
        'py_class_name': 'RoutineRobotHome',
        'enabled':1
    },
    ############## SENTRIES #############################
    'sentry_manager': {'controllers': [
        'sentry_omnibase_guarded_contact',
        'sentry_limit_vel_on_pose',
        'sentry_battery_mgmt',
        'sentry_ubuntu_power_management',
        'sentry_status_logger',
        'sentry_cpu_temp',
        'sentry_joint_runaway',]},
    'sentry_limit_vel_on_pose': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_limit_vel_on_pose',
        'py_class_name': 'SentryLimitVelOnPose',
        'lift_lower_safe_height_m': 0.2,
        'limit_omnibase_translation_by_lift': 1,
        'limit_omnibase_rotation_by_arm': 0,
        'limit_omnibase_rotation_by_lift': 1,
        'required_subsystems': ['omnibase', 'lift'],
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
        'lift_runaway_vel': 1.25
    },
    'sentry_battery_mgmt': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_battery_mgmt',
        'py_class_name': 'SentryBatteryMgmt',
        'required_subsystems': ['power_periph'],
        'enabled': 1,
        'enable_audio_alert': 1,
        'alert_period_low_battery': 20.0,
        'alert_period_shutdown': 5.0,
        'soc_low_battery_warning': 20.0,
        'soc_shutdown_warning': 1.0,  # Will shutdown at 0 SOC
        'low_voltage_shutdown_warning': 25.0  # Will shutdown at 24.85V
    },
    'sentry_ubuntu_power_management': {
        'py_module_name': 'stretch4_body.behavior.sentries.sentry_ubuntu_power_management',
        'py_class_name': 'SentryUbuntuPowerManagement',
        'required_subsystems': [],
        'enabled': 1,
        'check_rate_seconds':30
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
        'enabled':0
        },
    'safe_motion_overtilt_avoid': {
        'py_module_name': 'stretch4_body.behavior.safe_motions.safe_motion_overtilt_avoid',
        'py_class_name': 'SafeMotionOvertiltAvoid',
        'gravity_tilt_thresh_deg':{'default':6.0,'conservative':9.0,'aggressive':3.0},
        'enable_gravity_tilt':1,
        'required_subsystems': ['omnibase','power_periph'],
        'enabled':1,
        'enable_rerun_viz': 0,
        'enable_audio_alert': 1,
        'alert_period': 2.0
    },

    #################### ROBOT ########################
    'robot': {
        'batch_name': 'NA',
        'serial_no': 'NA',
        'model_name': 'SE4UNH',
        'subsystems': ['lift', 'omnibase', 'power_periph'],
        'tool': 'eoa_wrist_nil_tool_unh',
        'enable_rate_log':1,
        'max_rate_log_samples':10000,
        'guarded_contact':{
            'off':0,
            'default':{
                    'lift':'sensitivity_default',
                    'omnibase':'sensitivity_default'
            },
            'high_sensitivity_nav':{
                    'lift':'sensitivity_default',
                    'omnibase':'sensitivity_high'
            },
            'high_sensitivity_manipulation':{
                    'lift':'sensitivity_high',
                    'omnibase':'sensitivity_default'
            },
            'strong_manipulation':{
                    'lift':'sensitivity_low',
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
            'max_push_command_rate_Hz': 100,
            'subsystems': [],  # ['line_sensor_loop'],
        }
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
                "image_size": (3040, 4056),  # Full 12MP resolution
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
    },
    'self_collision_mujoco':{
        'SE4UNH':{'k_brake_distance': {'lift': 1.1},
               'ignore_links': ['link_wheel_0','link_wheel_1', 'link_wheel_2'],
               'exclusions':[
                   ["link_head", "link_lift"],
                   ["base_link", "link_mast"],
                   ["link_lift", "link_mast"],
                   ["link_lift", "link_head"],

                    ]},
        'eoa_wrist_nil_tool_unh':{'k_brake_distance': {},
               'exclusions':[

               ]},
               },
    'self_collision_loop': {
        'loop_rate_Hz': 60.0},
    'stretch_gamepad':{
        'enable_fn_button': 0,
        'function_cmd':'',
        'press_time_span':5},
    'params':[],
    'ros': {
        'joints': [{
            'py_module_name': 'stretch_core.command_groups',
            'py_class_name': 'LiftCommandGroup',
        },
        {
            'py_module_name': 'stretch_core.command_groups',
            'py_class_name': 'MobileBaseCommandGroup',
        }],
    }
    }
