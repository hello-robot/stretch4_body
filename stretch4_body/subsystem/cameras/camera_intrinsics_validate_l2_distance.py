#!/usr/bin/env python3
import time
import argparse
import sys
import os
import yaml
import datetime

from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import RGBPipelineController
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.image_frame import SyncedImageFrame
from stretch4_body.subsystem.cameras.detectors.detector_aruco import find_all_aruco_markers, get_aruco_pose
from stretch4_body.subsystem.cameras.enums.charuco_dictionary import CharucoBoards
from stretch4_body.subsystem.cameras.cv_utils import solve_pnp
from stretch4_body.subsystem.cameras.models.camera_calibration import RGBCameraCalibration, DEFAULT_CALIBRATION_FOLDER_PATH
from stretch4_body.subsystem.cameras.detectors.detector_frame_settled import DetectFrameSettled
import rerun as rr
import rerun.blueprint as rrb
import cv2
import numpy as np

def _parse_args():
    parser = argparse.ArgumentParser(
        description="Helps the user manually validate camera intrinsics calibration by detecting an ArUco marker from multiple robot poses using a tape measure."
    )
    parser.add_argument(
        "--charuco_board_name",
        type=str,
        default="BOARD_5x7_37mm_27mm_4x4_start_id_0",
        help="Name of the CharucoBoards enum to use for calibration.",
    )
    parser.add_argument(
        "--skip_user_prompt",
        action="store_true",
        help="Skip the user prompt for validation.",
    )
    return parser.parse_known_args()[0]


class CalibrationValidator:
    def __init__(self, charuco_board_name: str):
        self.charuco_board_config = CharucoBoards[charuco_board_name].get_board_config(use_high_MP_corner_refinement=True)
        self.poses = [
            {'lift': 0.8, 'arm': 0.15, 'wrist_pitch': -0.49, 'known_distance_m': [0.61, 0.61, 0.6]},
            {'lift': 0.8 , 'arm': 0.25, 'wrist_pitch': -0.49, 'known_distance_m': [0.70, 0.69, 0.68]},
            {'lift': 0.8, 'arm': 0.45, 'wrist_pitch': -0.49, 'known_distance_m': [0.850, 0.850, 0.84]},
            {'lift': 1.1, 'arm': 0.15, 'wrist_pitch': -0.49, 'known_distance_m': [0.451, 0.451, 0.441]},
            {'lift': 1.1, 'arm': 0.25, 'wrist_pitch': -0.49, 'known_distance_m': [0.563, 0.553, 0.543]},
            {'lift': 1.1, 'arm': 0.45, 'wrist_pitch': -0.49, 'known_distance_m': [0.744, 0.744, 0.734]},
        ]

        self.robot = RobotClient()
        if not self.robot.startup():
            print("Failed to start robot client.")
            sys.exit(1)

        if self.robot.params.get('tool') != 'eoa_wrist_dw4_tool_calibration':
            print("WARNING: This script is intended to be run with the 'eoa_wrist_dw4_tool_calibration' tool.")
            print("Make sure your tool parameter in your robot geometry is correctly set.")
            raise Exception("Tool is mpt correct. Expecting the eoa_wrist_dw4_tool_calibration tool.")

        self.camera_type = RGBCameras.synced_left_right_center()
        
        self.pipeline = RGBPipelineController(
            camera_type=self.camera_type,
            recording_directory=None,
            show_image_in=None, # We'll handle rerun ourselves
            is_rotate=True,
            is_rectify=False,
            is_crop=False,
            ai_models_to_use=[],
            detect_aruco_marker_size=None,
        )

        self.pipeline.set_calibration_exposure_preset()

        self.calibrations = {}
        for cam in [RGBCameras.left(), RGBCameras.right(), RGBCameras.center()]:
            try:
                self.calibrations[cam.name] = cam.load_calibration()
            except Exception as e:
                print(f"Failed to load calibration for {cam.name}: {e}")

        # Metrics state for running average
        self.distances = {RGBCameras.left().name: [], RGBCameras.right().name: [], RGBCameras.center().name: []}
        
        self.frame_settled_detectors_val = {
            RGBCameras.left().name: DetectFrameSettled(),
            RGBCameras.right().name: DetectFrameSettled(),
            RGBCameras.center().name: DetectFrameSettled()
        }
        
    def setup_rerun_blueprint(self):
        rr.spawn()
        # We define a simple side-by-side view
        views = []
        for cam in [RGBCameras.left(), RGBCameras.center(), RGBCameras.right()]:
            views.append(rrb.Spatial2DView(name=f"{cam.name.upper()} Camera", origin=f"world/{cam.name}"))
            
        text_log_view = rrb.TextDocumentView(name="Instructions & Validation Status", origin="Validation/Instructions")

        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Horizontal(*views),
                rrb.Vertical(
                    text_log_view,
                ),
                column_shares=[3, 1]
            ),
            rrb.BlueprintPanel(state="collapsed"),
            rrb.SelectionPanel(state="collapsed"),
        )
        try:
            rr.send_blueprint(blueprint)
        except Exception as e:
            print(f"Failed to send rerun blueprint: {e}")

    def update_instructions(self, message: str):
        rr.log("Validation/Instructions", rr.TextDocument(message, media_type=rr.MediaType.MARKDOWN))

    def move_to_pose(self, pose):
        print(f"Moving to pose: {pose}")
        self.robot.lift.move_to(pose['lift'])
        self.robot.arm.move_to(pose['arm'])
        self.robot.end_of_arm.move_to('wrist_pitch', pose['wrist_pitch'])
        self.robot.end_of_arm.move_to('wrist_roll', 0)
        self.robot.end_of_arm.move_to('wrist_yaw', 0)
        self.robot.push_command()
        self.robot.wait_command()
        time.sleep(1) # Let the camera settle

    def _get_distances_from_image(self, frame_name: str, color_image: np.ndarray, calibration: RGBCameraCalibration):
        dist_avg = None
        labeled_img = color_image.copy()
        
        gray = cv2.cvtColor(labeled_img, cv2.COLOR_BGR2GRAY)
        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            self.charuco_board_config.charuco_detector.detectBoard(gray)
        )

        board = self.charuco_board_config.charuco_detector.getBoard()

        object_points = []
        image_points = []
        if charuco_ids is not None and len(charuco_ids) > 0:
            cv2.aruco.drawDetectedCornersCharuco(labeled_img, charuco_corners, charuco_ids, (0, 255, 0))

            valid = self.charuco_board_config.check_valid_detection(charuco_ids, marker_ids)
            enough = self.charuco_board_config.check_enough_corners_detected(charuco_ids, minimum_percentage_of_corners_required=0.15)

            if valid and enough:
                (
                    object_points,
                    image_points,
                ) = board.matchImagePoints(charuco_corners, charuco_ids, None, None)
                
                if object_points is not None and len(object_points) >= 6:
                    success, rvec, tvec = solve_pnp(
                        object_points=object_points,
                        image_points=image_points,
                        camera_matrix=calibration.camera_matrix,
                        distortion_coefficients=calibration.distortion_coefficients,
                        distortion_model=calibration.distortion_model,
                    )

                    if success:
                        board_width = self.charuco_board_config.size[0] * self.charuco_board_config.square_length
                        board_height = self.charuco_board_config.size[1] * self.charuco_board_config.square_length
                        center_3d = np.array([[board_width / 2.0, board_height / 2.0, 0.0]], dtype=float)
                        
                        R, _ = cv2.Rodrigues(rvec)
                        center_cam = (R @ center_3d.T) + tvec
                        dist = np.linalg.norm(center_cam)
                        
                        self.distances[frame_name].append(dist)
            
        if len(self.distances[frame_name]) > 0:
            dist_avg = np.mean(self.distances[frame_name][-15:])

        if dist_avg is not None:
            text = f"Avg Dist over 15 frames: {dist_avg:.3f}m"
        else:
            text = "No Charuco detected"
            
        labeled_img = cv2.putText(labeled_img, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        rr.log(f"world/{frame_name}", rr.Image(labeled_img, color_model="BGR").compress())
        return dist_avg

    def process_frame(self, synced_frame: SyncedImageFrame | None):
        if synced_frame is None:
            raise Exception("Synced frame is None")

        if synced_frame.center is None:
            raise Exception("Center frame is None")

        cam_frames = {
            RGBCameras.left().name: synced_frame.left.image,
            RGBCameras.right().name: synced_frame.right.image,
            RGBCameras.center().name: synced_frame.center.image
        }

        dist_texts = []
        for cam_name, img in cam_frames.items():
            calib = self.calibrations[cam_name]
            avg_dist = self._get_distances_from_image(cam_name, img, calib)
            dist_texts.append(f"- **{cam_name.capitalize()}**: {f'{avg_dist:.3f}m' if avg_dist else 'Not Detected'}")

        instructions = f"""
# Validation Instructions

To validate calibration for this pose:
1. Use a physical tape measure to measure the distance from the lens of each camera to the center of the Charuco board.
2. Compare your physical measurement to the automated distances shown below.

### Live Detected Average Distances (rolling)
{chr(10).join(dist_texts)}

**Note:** If the difference exceeds your tolerance, you may need to recalibrate. 
Press CTRL+C in the terminal to go to the next pose or abort.
"""
        self.update_instructions(instructions)


    def _get_settled_frame(self, timeout:float):
        for det in self.frame_settled_detectors_val.values():
            det.reset()

        start_t = time.monotonic()
        for frame in self.pipeline.get_frame_synced(is_run_pipeline=False):
            time.sleep(0.25)
            
            left_s = True
            if RGBCameras.left().name in self.frame_settled_detectors_val.keys():
                left_s = self.frame_settled_detectors_val[RGBCameras.left().name].check_stability_diff(frame.left.image if frame.left else None)
            
            right_s = True
            if RGBCameras.right().name in self.frame_settled_detectors_val.keys():
                right_s = self.frame_settled_detectors_val[RGBCameras.right().name].check_stability_diff(frame.right.image if frame.right else None)
                
            cen_s = True
            if RGBCameras.center().name in self.frame_settled_detectors_val.keys():
                cen_s = self.frame_settled_detectors_val[RGBCameras.center().name].check_stability_diff(frame.center.image if frame.center else None)
        
            if left_s and right_s and cen_s:
                return frame
            
            if time.monotonic() - start_t > timeout:
                return None

        return None


    def run(self, skip_user_prompt: bool, interactive:bool):
        if not skip_user_prompt:
            ans = input("This script will move the robot to validation poses. Do you wish to proceed? [y/N]: ")
            if ans.lower() != 'y':
                print("Exiting.")
                raise Exception("User aborted validation.")

        rr.init("Calibration_Validation", spawn=False,)
        if interactive:
            self.setup_rerun_blueprint()


        print("Starting non-interactive validation with known distances.")
        total_errors = {RGBCameras.left().name: [], RGBCameras.center().name: [], RGBCameras.right().name: []}


        for idx, pose in enumerate(self.poses):
            self.move_to_pose(pose)
            
            print(f"Pose {idx+1}/{len(self.poses)} reached. Waiting for camera to settle...")
            self.distances = {RGBCameras.left().name: [], RGBCameras.right().name: [], RGBCameras.center().name: []}
            
            
            settled_frame = self._get_settled_frame(timeout=5.0)
            if settled_frame is None:
                raise RuntimeError("Camera image did not settle.")
            else:
                print("Camera image settled.")

            # Report errors
            known_dists = pose['known_distance_m'] # [left, center, right]
            cam_order = [RGBCameras.left().name, RGBCameras.center().name, RGBCameras.right().name]

            self.process_frame(settled_frame)
            
            print(f"--- Results for Pose {idx+1} ---")
            for c_idx, cam_name in enumerate(cam_order):
                if len(self.distances[cam_name]) > 0:
                    dist_avg = np.mean(self.distances[cam_name])
                    known_d = known_dists[c_idx]
                    err = abs(dist_avg - known_d)
                    total_errors[cam_name].append(err)
                    print(f"{cam_name.capitalize()}: Measured={dist_avg:.3f}m, Known={known_d:.3f}m, Error={err:.3f}m")
                else:
                    print(f"{cam_name.capitalize()}: No Charuco detected.")
        
        print("--- Final Averaged Errors ---")
        avg_errors = []
        for cam_name in [RGBCameras.left().name, RGBCameras.center().name, RGBCameras.right().name]:
            errors = total_errors[cam_name]
            if len(errors) > 0:
                mean_err = float(np.mean(errors))
                print(f"{cam_name.capitalize()}: {mean_err:.3f}m")
                avg_errors.append(mean_err)
            else:
                print(f"{cam_name.capitalize()}: N/A")
                avg_errors.append(float('inf'))

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fleet_id = os.environ.get("HELLO_FLEET_ID", "")
        results = {
            "timestamp": timestamp,
            "fleet_id": fleet_id,
            "avg_errors": avg_errors
        }
        
        save_dir = os.path.join(DEFAULT_CALIBRATION_FOLDER_PATH, "intrinsics_validation", timestamp)
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, "camera_instrinsics_validation.yaml")
        
        with open(save_path, "w") as f:
            yaml.dump(results, f, default_flow_style=False)
            
        print(f"Saved validation results to {save_path}")
            
        return avg_errors


def REx_validate_intrinsics(interactive:bool):
    args = _parse_args()
    validator = CalibrationValidator(args.charuco_board_name)
    
    return validator.run(skip_user_prompt=not interactive, interactive=interactive)

if __name__ == "__main__":
    REx_validate_intrinsics(interactive=True)
