#!/usr/bin/env python3
import argparse
import datetime
import os
import shutil
from pathlib import Path
import numpy as np
import yaml
from dataclasses import asdict

from stretch4_body.subsystem.cameras.models.camera_calibration import (
    DEFAULT_CALIBRATION_FOLDER_PATH,
    RGBCameraCalibrationFile,
    CalibrateCameraResults,
    RGBCameraCalibration
)
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels
from stretch4_body.subsystem.cameras.adapters.luxonis_gripper_camera_adapter import GripperCameraLuxonis


def save_to_user_yaml(calibration: CalibrateCameraResults, camera_type: RGBCameras):
    """Saves the calibration results under the specific camera name key in the user-level yaml."""
    calibration_results = calibration.get_serializable()
    calibration_results["fleet_id"] = os.environ.get("HELLO_FLEET_ID", "")

    user_calib_path = RGBCameraCalibrationFile.USER.get_camera_calibration_file_path()
    
    existing_stretch_user_calibration_file = {}
    try:
        if os.path.exists(user_calib_path):
            with open(user_calib_path, "r") as file:
                existing_stretch_user_calibration_file = yaml.safe_load(file) or {}
    except Exception as e:
        print(f"Warning: could not read {user_calib_path}: {e}")

    if os.path.exists(user_calib_path):
        mod_time = int(os.path.getmtime(user_calib_path))
        p = Path(user_calib_path)
        backup_path = p.with_name(f"{p.stem}_backup_{mod_time}{p.suffix}")
        shutil.copy2(user_calib_path, backup_path)
        print(f"Backed up {user_calib_path} to {backup_path}")

    existing_stretch_user_calibration_file[camera_type.name] = calibration_results
    
    with open(user_calib_path, "w") as file:
        yaml.dump(existing_stretch_user_calibration_file, file, sort_keys=True)
    
    print(f"Successfully saved {camera_type.name} to {user_calib_path}")


def REx_gripper_camera_calibration():
    parser = argparse.ArgumentParser(
        description="Calibrate gripper camera by reading on-device factory intrinsics and saving them."
    )
    args = parser.parse_args()

    print("Reading gripper configurations...")
    left_config = RGBCameras.gripper_left.config
    right_config = RGBCameras.gripper_right.config

    print("Initializing GripperCameraLuxonis to read factory intrinsics...")
    camera = GripperCameraLuxonis(left_config, right_config, enable_pointcloud=False)

    try:
        for cam_type in [RGBCameras.gripper_left, RGBCameras.gripper_right]:
            print(f"\n--- Processing {cam_type.name} ---")
            M, D = camera.get_gripper_intrinsics(cam_type)
            if M is None or D is None:
                print(f"Error: Could not read calibration for {cam_type.name} from device.")
                continue

            print(f"Successfully read intrinsics for {cam_type.name}.")
            print(f"Camera Matrix (M):\n{M}")
            print(f"Distortion Coefficients (D):\n{D}")

            # Construct CalibrateCameraResults
            calibration = CalibrateCameraResults(
                camera_name=cam_type.name,
                calibration_date=datetime.datetime.now(),
                image_size=list(cam_type.config.image_size) + [3],  # [height, width, channels] e.g. [400, 640, 3]
                number_of_images_processed=0,
                number_of_images_used=0,
                reprojection_error=0.0,
                camera_matrix=M,
                distortion_coefficients=D,
                projection_matrix=np.dot(M, np.eye(3, 4)),
                distortion_model=DistortionModels.wide_angle,
                rectification_matrix=np.zeros((3, 3)),
                rotation_vectors=[],
                translation_vectors=[],
                focal_length_mm=(
                    RGBCameraCalibration.get_focal_length_mm(
                        M, pixel_size_mm=cam_type.config.sensor_pixel_size_mm
                    )
                    if cam_type.config.sensor_pixel_size_mm is not None
                    else None
                ),
            )

            # Save user-level format
            save_to_user_yaml(calibration, cam_type)

        print("\nGripper camera calibration processing completed.")

    finally:
        print("Stopping camera...")
        camera.stop()


if __name__ == "__main__":
    REx_gripper_camera_calibration()
