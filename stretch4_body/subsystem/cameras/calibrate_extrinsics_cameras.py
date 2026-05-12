#!/usr/bin/env python3
import argparse
import glob
import os
import yaml
import cv2
import numpy as np
import datetime
import numpy as np
from dataclasses import dataclass
from typing import Optional, Any
from pathlib import Path
import rerun as rr


from scipy.spatial.transform import Rotation

from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.image_write_to_disk import (
    get_recording_subdirectory,
)
from stretch4_body.subsystem.cameras.enums.charuco_dictionary import CharucoBoardConfig, CharucoBoards
from stretch4_body.subsystem.cameras.cv_utils import solve_pnp
from stretch4_body.subsystem.cameras.models.camera_calibration import (
    DEFAULT_IMAGES_SAVE_PATH,
    RGBCameraCalibration,
)

_fleet_path = os.environ["HELLO_FLEET_PATH"]
_fleet_id = os.environ["HELLO_FLEET_ID"]
CAMERA_EXTRINSICS_YAML_PATH = os.path.join(_fleet_path, _fleet_id, "calibration_cameras", "camera_extrinsics.yaml")


@dataclass
class CameraDetection:
    image: np.ndarray
    charuco_ids: np.ndarray
    marker_ids: np.ndarray
    obj_pts: np.ndarray
    img_pts: np.ndarray
    T_camera_board: np.ndarray


@dataclass
class FrameCorrespondences:
    center: Optional[CameraDetection] = None
    left: Optional[CameraDetection] = None
    right: Optional[CameraDetection] = None


def average_transforms(transforms):
    """Average a list of 4x4 homogenous transformation matrices."""
    if not transforms:
        return None

    t_vecs_all = [T[:3, 3] for T in transforms]
    R_mats_all = [T[:3, :3] for T in transforms]

    t_median = np.median(t_vecs_all, axis=0)

    t_vecs = []
    R_mats = []
    for i, t in enumerate(t_vecs_all):
        # Robust outlier rejection: solvePnP suffers from planar ambiguity (flipping).
        # Discard any transforms whose translation is further than 10cm from median.
        if np.linalg.norm(t - t_median) < 0.1:
            t_vecs.append(t)
            R_mats.append(R_mats_all[i])

    if not t_vecs:
        return None

    # Simple mean for translation
    t_mean = np.mean(t_vecs, axis=0)

    # Robust averaging for rotation matrices on SO(3)
    rotations = Rotation.from_matrix(R_mats)
    R_mean = rotations.mean().as_matrix()

    T_mean = np.eye(4)
    T_mean[:3, :3] = R_mean
    T_mean[:3, 3] = t_mean
    return T_mean


def process_camera_image(
    image_path: str, board: cv2.aruco.CharucoBoard, board_config: CharucoBoardConfig, calibration: RGBCameraCalibration
) -> CameraDetection|None:
    """Detects Charuco board and computes camera pose."""
    if not image_path:
        return None

    img_col = cv2.imread(image_path)
    if img_col is None:
        return None

    charuco_corners, charuco_ids, marker_corners, marker_ids = (
        board_config.charuco_detector.detectBoard(img_col)
    )

    if charuco_ids is None or len(charuco_ids) < 6:
        return None

    obj_pts, img_pts = board.matchImagePoints(charuco_corners, charuco_ids, None, None)
    if obj_pts is None or len(obj_pts) < 6:
        return None

    success, rvec, tvec = solve_pnp(
        obj_pts,
        img_pts,
        calibration.camera_matrix,
        calibration.distortion_coefficients,
        distortion_model=calibration.distortion_model,
    )

    if not success:
        return None

    R, _ = cv2.Rodrigues(rvec)
    T_camera_board = np.eye(4)
    T_camera_board[:3, :3] = R
    T_camera_board[:3, 3] = tvec.flatten()

    return CameraDetection(
        image=img_col,
        charuco_ids=charuco_ids,
        marker_ids=marker_ids,
        obj_pts=obj_pts,
        img_pts=img_pts,
        T_camera_board=T_camera_board,
    )


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate Extrinsics between Left, Right, and Center cameras"
    )

    parser.add_argument(
        "-d",
        "--recording_directory",
        type=str,
        default=DEFAULT_IMAGES_SAVE_PATH,
        help=f"Directory used to record the data, if provided, images will be saved to disk in this directory. Otherwise {DEFAULT_IMAGES_SAVE_PATH} is used.",
    )

    parser.add_argument(
        "--charuco_board_names",
        type=str,
        default="BOARD_5x7_37mm_27mm_4x4_start_id_0",
        help=f"Name of the CharucoBoards enum to use for calibration. Comma separated values of {[c.name for c in CharucoBoards]}",
    )

    parser.add_argument(
        "-t", "--timestamp", help="Timestamp of the recording to process"
    )

    parser.add_argument(
        "-last",
        "--use_last_recording",
        action="store_true",
        help="Use the last recorded folder timestamp inside the provided recording dir. This will load existing images and 'append' new saves to this folder.",
    )

    parser.add_argument(
        "--visualize",
        action="store_false",
        help="Visualize the correspondences used to calculate the transform in rerun.",
    )

    return parser.parse_known_args()[0]


def REx_calibrate_extrinsics_cameras(interactive: bool):
    args = _parse_args()
    return calibrate_extrinsics_camera_camera(
        recording_directory=args.recording_directory,
        charuco_board_names=args.charuco_board_names,
        timestamp=args.timestamp,
        use_last_recording=True,
        visualize=interactive,
    )

def calibrate_extrinsics_camera_camera(
    recording_directory: str,
    charuco_board_names: str,
    timestamp: Optional[str] = None,
    use_last_recording: bool = False,
    visualize: bool = False,
):
    charuco_board_name = charuco_board_names.split(",")[0]

    if not timestamp and not use_last_recording:
        print("Please provide exactly one of --timestamp or --use_last_recording.")
        return

    if visualize:
        rr.init("Calibrate Extrinsics Correspondences", spawn=True)
        rr.spawn(memory_limit="2GiB")
        rr.log(
            "Instructions",
            rr.TextDocument(
                "### Correspondences View\nReview the 2D detected ArUco/Charuco markers overlayed perfectly onto the images to ensure valid parsing.",
                media_type="text/markdown",
            ),
            static=True,
        )

    # Load cameras and their intrinsics
    print("Loading intrinsics...")
    cam_c = RGBCameras.center()
    cam_l = RGBCameras.left()
    cam_r = RGBCameras.right()

    cal_c: RGBCameraCalibration = cam_c.load_calibration()
    cal_l: RGBCameraCalibration = cam_l.load_calibration()
    cal_r: RGBCameraCalibration = cam_r.load_calibration()

    # Get directories for each camera
    dir_c = get_recording_subdirectory(
        recording_directory, cam_c.recording_folder_name, timestamp
    )
    dir_l = get_recording_subdirectory(
        recording_directory, cam_l.recording_folder_name, timestamp
    )
    dir_r = get_recording_subdirectory(
        recording_directory, cam_r.recording_folder_name, timestamp
    )

    images_c = (
        glob.glob(os.path.join(dir_c, "*.png"))
        if dir_c and os.path.isdir(dir_c)
        else []
    )
    images_l = (
        glob.glob(os.path.join(dir_l, "*.png"))
        if dir_l and os.path.isdir(dir_l)
        else []
    )
    images_r = (
        glob.glob(os.path.join(dir_r, "*.png"))
        if dir_r and os.path.isdir(dir_r)
        else []
    )

    print(
        f"Found {len(images_c)} center images, {len(images_l)} left images, {len(images_r)} right images."
    )

    if len(images_c) == 0:
        print(
            "No center calibration images found. Center camera is required as the reference frame."
        )
        return

    center_board_config = CharucoBoards[charuco_board_name].get_board_config(use_high_MP_corner_refinement=True)
    center_board = center_board_config.charuco_detector.getBoard()
    left_right_board_config = CharucoBoards[charuco_board_name].get_board_config(use_high_MP_corner_refinement=False)
    left_right_board = left_right_board_config.charuco_detector.getBoard()

    transforms_center = []
    transforms_left = []
    transforms_right = []
    frame_data_vis = []

    # Time tolerance in seconds for images to be considered synchronized
    SYNC_TOLERANCE_SEC = 0.01

    print("Computing extrinsics from valid frames...")

    for c_img in images_c:
        # c_time = os.path.getmtime(c_img)
        c_time = float(Path(c_img).stem)
        l_img, r_img = None, None

        # Find matching left image
        if images_l:
            # l_dists = [(abs(os.path.getmtime(l) - c_time), l) for l in images_l]
            l_dists = [(abs(float(Path(l).stem) - c_time), l) for l in images_l]
            l_dists.sort()
            if l_dists[0][0] < SYNC_TOLERANCE_SEC:
                l_img = l_dists[0][1]

        # Find matching right image
        if images_r:
            # r_dists = [(abs(os.path.getmtime(r) - c_time), r) for r in images_r]
            r_dists = [(abs(float(Path(r).stem) - c_time), r) for r in images_r]
            r_dists.sort()
            if r_dists[0][0] < SYNC_TOLERANCE_SEC:
                r_img = r_dists[0][1]

        if not l_img and not r_img:
            continue

        center_detection = process_camera_image(c_img, center_board, center_board_config, cal_c)
        if center_detection is None:
            continue

        frame_corr = FrameCorrespondences(center=center_detection)
        T_center_board = center_detection.T_camera_board
        transforms_center.append(T_center_board)

        if visualize:
            rr.set_time(
                "frame_idx", sequence=(len(transforms_left) + len(transforms_right))
            )
            rr.log(
                "correspondences/center/image",
                rr.Image(center_detection.image, color_model="BGR"),
            )
            rr.log(
                "correspondences/center/image/points",
                rr.Points2D(
                    center_detection.img_pts.reshape(-1, 2), colors=[0, 255, 0], radii=3
                ),
            )

        if l_img:
            left_detection = process_camera_image(l_img, left_right_board, left_right_board_config, cal_l)
            if left_detection is not None:
                # T_center_left transforms from left camera frame to center camera frame
                T_center_left = T_center_board @ np.linalg.inv(
                    left_detection.T_camera_board
                )
                transforms_left.append(T_center_left)
                frame_corr.left = left_detection

                if visualize:
                    rr.log(
                        "correspondences/left/image",
                        rr.Image(left_detection.image, color_model="BGR"),
                    )
                    rr.log(
                        "correspondences/left/image/points",
                        rr.Points2D(
                            left_detection.img_pts.reshape(-1, 2),
                            colors=[255, 0, 0],
                            radii=3,
                        ),
                    )

        if r_img:
            right_detection = process_camera_image(r_img, left_right_board, left_right_board_config, cal_r)
            if right_detection is not None:
                # T_center_right transforms from right camera frame to center camera frame
                T_center_right = T_center_board @ np.linalg.inv(
                    right_detection.T_camera_board
                )
                transforms_right.append(T_center_right)
                frame_corr.right = right_detection

                if visualize:
                    rr.log(
                        "correspondences/right/image",
                        rr.Image(right_detection.image, color_model="BGR"),
                    )
                    rr.log(
                        "correspondences/right/image/points",
                        rr.Points2D(
                            right_detection.img_pts.reshape(-1, 2),
                            colors=[0, 0, 255],
                            radii=3,
                        ),
                    )

        frame_data_vis.append(frame_corr)

    print(
        f"Successfully computed {len(transforms_left)} left-to-center transforms and {len(transforms_right)} right-to-center transforms."
    )

    T_mean_l = average_transforms(transforms_left)
    T_mean_r = average_transforms(transforms_right)

    if visualize:
        rr.log("error_cloud", rr.ViewCoordinates.RDF, static=True)

        for idx, fd in enumerate(frame_data_vis):
            rr.set_time("frame_idx", sequence=idx)

            obj_c_flat = fd.center.obj_pts.reshape(-1, 3)
            P_board_c = np.hstack([obj_c_flat, np.ones((len(obj_c_flat), 1))])
            P_c = (fd.center.T_camera_board @ P_board_c.T).T[:, :3]
            rr.log(
                "error_cloud/center_points",
                rr.Points3D(P_c, colors=[0, 255, 0], radii=0.002),
            )

            ids_c_flat = fd.center.charuco_ids.flatten()

            if T_mean_l is not None and fd.left is not None:
                obj_l_flat = fd.left.obj_pts.reshape(-1, 3)
                P_board_l = np.hstack([obj_l_flat, np.ones((len(obj_l_flat), 1))])
                P_l_in_l = (fd.left.T_camera_board @ P_board_l.T).T
                P_l_in_c = (T_mean_l @ P_l_in_l.T).T[:, :3]

                labels = []
                for num, id_l in enumerate(fd.left.charuco_ids.flatten()):
                    if id_l in ids_c_flat:
                        idx_c = np.where(ids_c_flat == id_l)[0][0]
                        err = np.linalg.norm(P_l_in_c[num] - P_c[idx_c]) * 1000
                        labels.append(f"{err:.4f}mm")
                    else:
                        labels.append("")

                rr.log(
                    "error_cloud/left_points",
                    rr.Points3D(
                        P_l_in_c, colors=[255, 0, 0], labels=labels, radii=0.002
                    ),
                )

            if T_mean_r is not None and fd.right is not None:
                obj_r_flat = fd.right.obj_pts.reshape(-1, 3)
                P_board_r = np.hstack([obj_r_flat, np.ones((len(obj_r_flat), 1))])
                P_r_in_r = (fd.right.T_camera_board @ P_board_r.T).T
                P_r_in_c = (T_mean_r @ P_r_in_r.T).T[:, :3]

                labels = []
                for num, id_r in enumerate(fd.right.charuco_ids.flatten()):
                    if id_r in ids_c_flat:
                        idx_c = np.where(ids_c_flat == id_r)[0][0]
                        err = np.linalg.norm(P_r_in_c[num] - P_c[idx_c]) * 1000
                        labels.append(f"{err:.4f}mm")
                    else:
                        labels.append("")

                rr.log(
                    "error_cloud/right_points",
                    rr.Points3D(
                        P_r_in_c, colors=[0, 0, 255], labels=labels, radii=0.002
                    ),
                )

    left_errors = []
    right_errors = []

    for fd in frame_data_vis:
        obj_c_flat = fd.center.obj_pts.reshape(-1, 3)
        P_board_c = np.hstack([obj_c_flat, np.ones((len(obj_c_flat), 1))])
        P_c = (fd.center.T_camera_board @ P_board_c.T).T[:, :3]
        ids_c_flat = fd.center.charuco_ids.flatten()

        if T_mean_l is not None and fd.left is not None:
            obj_l_flat = fd.left.obj_pts.reshape(-1, 3)
            P_board_l = np.hstack([obj_l_flat, np.ones((len(obj_l_flat), 1))])
            P_l_in_l = (fd.left.T_camera_board @ P_board_l.T).T
            P_l_in_c = (T_mean_l @ P_l_in_l.T).T[:, :3]

            for num, id_l in enumerate(fd.left.charuco_ids.flatten()):
                if id_l in ids_c_flat:
                    idx_c = np.where(ids_c_flat == id_l)[0][0]
                    err = np.linalg.norm(P_l_in_c[num] - P_c[idx_c])
                    left_errors.append(float(err))

        if T_mean_r is not None and fd.right is not None:
            obj_r_flat = fd.right.obj_pts.reshape(-1, 3)
            P_board_r = np.hstack([obj_r_flat, np.ones((len(obj_r_flat), 1))])
            P_r_in_r = (fd.right.T_camera_board @ P_board_r.T).T
            P_r_in_c = (T_mean_r @ P_r_in_r.T).T[:, :3]

            for num, id_r in enumerate(fd.right.charuco_ids.flatten()):
                if id_r in ids_c_flat:
                    idx_c = np.where(ids_c_flat == id_r)[0][0]
                    err = np.linalg.norm(P_r_in_c[num] - P_c[idx_c])
                    right_errors.append(float(err))

    # Build dictionary to save
    output_data = {}
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    fleet_id = os.environ.get("HELLO_FLEET_ID", "")
    output_data["timestamp"] = timestamp_str
    output_data["fleet_id"] = fleet_id

    if T_mean_l is not None:
        output_data["left_to_center"] = T_mean_l.tolist()
        output_data["left_errors_m"] = np.mean(left_errors)
    if T_mean_r is not None:
        output_data["right_to_center"] = T_mean_r.tolist()
        output_data["right_errors_m"] = np.mean(right_errors)

    if not output_data:
        print("No paired transforms could be computed. Exiting without saving.")
        return

    out_yaml = CAMERA_EXTRINSICS_YAML_PATH
    out_dir = os.path.dirname(out_yaml)
    os.makedirs(out_dir, exist_ok=True)

    if os.path.exists(out_yaml):
        import shutil
        mod_time = int(os.path.getmtime(out_yaml))
        p = Path(out_yaml)
        backup_path = p.with_name(f"{p.stem}_backup_{mod_time}{p.suffix}")
        shutil.copy2(out_yaml, backup_path)
        print(f"Backed up {out_yaml} to {backup_path}")

    # Read existing data to append
    if os.path.exists(out_yaml):
        with open(out_yaml, "r") as f:
            existing_data = yaml.safe_load(f) or {}
    else:
        existing_data = {}

    # Update with new data
    existing_data.update(output_data)

    with open(out_yaml, "w") as f:
        yaml.dump(existing_data, f, default_flow_style=False)
    
    print(f"Saved camera extrinsics to {out_yaml}")


def main():
    args = _parse_args()
    calibrate_extrinsics_camera_camera(
        recording_directory=args.recording_directory,
        charuco_board_names=args.charuco_board_names,
        timestamp=args.timestamp,
        use_last_recording=args.use_last_recording,
        visualize=args.visualize,
    )


if __name__ == "__main__":
    main()
