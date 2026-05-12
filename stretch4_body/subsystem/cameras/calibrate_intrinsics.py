"""
The `CalibrateIntrinsics` handles the logic for performing camera instrinsic calibration on images saved to disk.

It also supports capturing images that are useful for camera calibration from live camera streams.

Images are only saved if they pass a few critera that aid in achieving a good calibration quality.

You can run this file to do manual calibration. This script assumes it is running on the robot.

You could also use `calibrate_intrinsics_robot_move.py` to control the robot while doing calibration.
"""

import argparse
import datetime
import glob
import os
from pathlib import Path
import threading
import time
from dataclasses import asdict
from typing import Callable

import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import (
    RGBPipelineController,
    create_directory_if_it_does_not_exist,
)
from stretch4_body.subsystem.cameras.cv_utils import camera_calibrate, draw_frame_axes, solve_pnp
from stretch4_body.subsystem.cameras.detectors.detector_frame_settled import (
    DetectFrameSettled,
)
from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels
from stretch4_body.subsystem.cameras.enums.log_levels import LogLevels
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.image_write_to_disk import (
    get_recording_subdirectory,
)
from stretch4_body.subsystem.cameras.models.image_frame import (
    ImageFrame,
    SyncedImageFrame,
)
from stretch4_body.subsystem.cameras.models.image_write_to_disk import (
    get_recording_subdirectory,
)
from stretch4_body.subsystem.cameras.models.camera_calibration import (
    RGBCameraCalibration,
    RGBCameraCalibrationFile,
    DEFAULT_IMAGES_SAVE_PATH,
    CameraPoseVisualizer,
    CalibrateCameraResults,
    calculate_per_view_reprojection_error,
)

from stretch4_body.subsystem.cameras.enums.charuco_dictionary import (
    CharucoBoardConfig,
    CharucoBoards,
)


AUTO_SAVE_SLEEP = 5  # seconds

RECOMMENDED_REPROJECTION_ERROR = 1.0  # This or less is good
RECOMMENDED_MINIMUM_NUMBER_OF_IMAGES_TO_CAPTURE = 50

LAST_SAVED_WINDOW_NAME = "Last Saved"


class CalibrateIntrinsics:
    """A class that provides a way to run calibration on images captured during interactive calibration or loaded from disk."""

    def __init__(
        self,
        recording_directory: str,
        camera_type: RGBCameras,
        timestamp: str | None,
        is_use_last_recording: bool,
        charuco_board_names: list[str],
        time_between_image_captures: float | None,
        not_interactive: bool = False,
    ) -> None:
        """If `time_between_image_captures` is not None, images will be captured at regular intervals."""
        self.last_save_time = 0
        self.camera_type = camera_type
        self.time_between_image_captures = time_between_image_captures
        self.capture_requested = False
        self.not_interactive = not_interactive

        self.visualizer = CameraPoseVisualizer(self.camera_type.name, not_interactive=self.not_interactive)

        self.frame_settled_detector = DetectFrameSettled(required_stable_frames=3)

        self.charuco_boards: list[CharucoBoardConfig] = []

        for charuco_board_name in charuco_board_names:
            # Setup Charuco instance variables based on the enum string
            charuco_board = CharucoBoards[charuco_board_name]

            self.charuco_boards.append(charuco_board.get_board_config(use_high_MP_corner_refinement=self.camera_type.is_center()))

        data_directory = None
        if is_use_last_recording or timestamp is not None:
            data_directory = get_recording_subdirectory(
                recording_directory, camera_type.recording_folder_name, timestamp
            )
        if data_directory is None or not os.path.isdir(data_directory):
            self.log_message(f"Data Directory {data_directory if data_directory is not None else ""} was not found. Creating it.", LogLevels.WARN)

            data_directory, time_string = create_directory_if_it_does_not_exist(
                recording_directory, self.camera_type
            )

        self.data_directory = data_directory

        self.log_message(
            f"Calibrating {self.camera_type.name} camera. Data directory: {self.data_directory}", LogLevels.INFO
        )

        self.all_object_points = []
        self.all_image_points = []
        self.all_timestamps: list[float] = []
        self.image_size = None

        self.last_calibration: CalibrateCameraResults | None = None

        self._load_images_and_do_calibration(data_directory)

    def _get_calibration_points(
        self, color_image: np.ndarray, board_config: CharucoBoardConfig
    ):

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            board_config.charuco_detector.detectBoard(gray)
        )  # from docs: If markerCorners and markerIds are empty, the function will detect aruco markers and ids.

        board = board_config.charuco_detector.getBoard()

        if marker_ids is not None:
            """We want to filter out marker_id's not belonging to this board config
            (the detector returns ALL markerId's in the scene.)"""
            valid_board_ids = set(np.array(board.getIds()).flatten())

            filtered_marker_corners = []
            filtered_marker_ids = []

            for corner, m_id in zip(marker_corners, marker_ids.flatten()):
                if m_id in valid_board_ids:
                    filtered_marker_corners.append(corner)
                    filtered_marker_ids.append([m_id])  # Keep the (N, 1) shape

            # Overwrite the original variables with the filtered data
            marker_corners = tuple(filtered_marker_corners)
            marker_ids = (
                np.array(filtered_marker_ids, dtype=np.int32)
                if filtered_marker_ids
                else None
            )

        object_points = []
        image_points = []

        if charuco_ids is not None and len(charuco_ids) > 0:
            (
                object_points,
                image_points,
            ) = board.matchImagePoints(charuco_corners, charuco_ids, None, None)

        object_points = np.array(object_points, dtype=np.float32)
        image_points = np.array(image_points, dtype=np.float32)

        return (
            object_points,
            image_points,
            charuco_corners,
            charuco_ids,
            marker_corners,
            marker_ids,
        )

    def _load_images_and_do_calibration(self, image_directory: str):
        """Loads images from the save folder, and runs them through the calibration pipeline."""
        file_name_pattern = image_directory + "*.png"
        file_names = glob.glob(file_name_pattern)

        self.log_message(f"Found {len(file_names)} calibration images for {self.camera_type.name} in {self.data_directory}.", LogLevels.INFO)

        frame_number = 0
        for f in file_names:
            self.log_message(f"Reading {f}", LogLevels.INFO)
            color_image = cv2.imread(f)

            if color_image is None:
                raise Exception(f"Could not read {f}")
            
            frame_number += 1

            self.process_image_frame(
                ImageFrame(timestamp=float(Path(f).stem), frame_number=frame_number, image=color_image),
                save_image_to_disk=False,
                use_stable_frames_only=False,
            )

    def request_capture(self):
        """Requests the capture of the next viable frame."""
        self.capture_requested = True
        msg = f"Capture requested for {self.camera_type.name}."
        self.log_message(msg, LogLevels.INFO)

    def log_message(self, text: str, level: LogLevels):
        self.visualizer.log_message(text, level.value)
        print(f"{level}: {text}")

    def log_instructions(self, text: str):
        self.visualizer.log_instructions(text)
        print(f"{text}")

    def print_calibration_results(
        self, calibration_results: CalibrateCameraResults | None = None
    ):
        calibration_results = calibration_results or self.last_calibration

        if calibration_results is None:
            return f"Calibration results are empty for {self.camera_type.name}. Please continue calibrating."

        msg = f"""{calibration_results}
{"Please continue capturing images" if calibration_results.number_of_images_used < RECOMMENDED_MINIMUM_NUMBER_OF_IMAGES_TO_CAPTURE or calibration_results.reprojection_error > RECOMMENDED_REPROJECTION_ERROR else "You may stop calibrating your camera now." }
"""
        self.log_message(msg, LogLevels.INFO)

    def do_calibration(
        self,
        object_points: list | None = None,
        image_points: list | None = None,
        frame_number: int | None = None,
    ):
        try:
            if self.image_size is None:
                raise ValueError("Image Size cannot be empty.")

            calibration_results = _do_calibration(
                self.all_object_points if object_points is None else object_points,
                self.all_image_points if image_points is None else image_points,
                self.image_size,
                self.camera_type,
                frame_number=frame_number or len(self.all_timestamps),
            )

            self.print_calibration_results(calibration_results)

            return calibration_results
        except Exception as e:
            msg = f"Calibration failed. {e=}"
            self.log_message(msg, LogLevels.ERROR)
            return None

    def calculate_per_image_rmse(self):
        if self.last_calibration is None:
            msg = "No calibration data available to filter. Please calibrate first."
            self.log_message(msg, LogLevels.WARN)
            return

        per_image_errors, board_colors = calculate_per_view_reprojection_error(
            self.all_object_points, self.all_image_points, self.last_calibration
        )

        self.visualizer.update_pointcloud(
            self.last_calibration,
            self.all_object_points,
            self.all_timestamps,
            per_image_errors=per_image_errors,
            board_colors=board_colors,
        )

    def save_calibration(self):
        if self.last_calibration is None:
            raise Exception(f"No calibration data to save for {self.camera_type.name}.")
        save_calibration(
            calibration=self.last_calibration,
            image_directory=self.data_directory,
            camera_type=self.camera_type,
            log_callback=self.log_message,
        )

    def _save_image(self, color_image: np.ndarray, timestamp: str):
        base_filename = self.data_directory + f"{timestamp}"
        rgb_filename = base_filename + ".png"
        cv2.imwrite(
            rgb_filename,
            color_image,
        )

    def _annotate_img(
        self,
        img,
        object_points,
        image_points,
        charuco_corners,
        charuco_ids,
        marker_corners,
        marker_ids,
        color,
    ):
        annotated_img = cv2.UMat(img)
        try:
            if charuco_corners is not None and len(charuco_corners) > 0:
                annotated_img = aruco.drawDetectedCornersCharuco(
                    annotated_img, charuco_corners, charuco_ids, color
                )
            # annotated_img = aruco.drawDetectedMarkers(
            #     annotated_img, marker_corners, marker_ids, color
            # )
            distortion_model=self.camera_type.config.distortion_model
            if distortion_model is None:
                raise RuntimeError("Distortion model cannot be none")
            if self.last_calibration is not None:
                annotated_img = draw_3d_axis(
                    annotated_img,
                    object_points,
                    image_points,
                    self.last_calibration.camera_matrix,
                    self.last_calibration.distortion_coefficients,
                    distortion_model=distortion_model
                    
                )
        finally:
            if isinstance(annotated_img, cv2.UMat):
                annotated_img = annotated_img.get()

        return annotated_img

    def process_image_frame(
        self,
        frame: ImageFrame | None,
        save_image_to_disk: bool,
        use_stable_frames_only: bool,
    ):
        """This is typically used by the interactive calibration image capture pipeline; when a new image comes in, this callback checks if it meet the criteria to save, and if it does, it saves the image to disk and calls do_calibation()."""
        if frame is None:
            raise ValueError("No frame in the image callback.")

        color_image, timestamp = frame.image, frame.timestamp

        object_points_all_boards_in_scene = []
        image_points_all_boards_in_scene = []
        annotated_img = color_image

        # 1. Evaluate capture trigger conditions once per frame
        auto_capture_triggered = False
        if self.time_between_image_captures is not None:
            auto_capture_triggered = (
                time.time() - self.last_save_time > self.time_between_image_captures
            )

        capture_triggered = auto_capture_triggered or self.capture_requested

        valid_board_detected = False

        is_frame_settled = (
            self.frame_settled_detector.check_stability_diff(color_image, threshold=3)
            if use_stable_frames_only
            else True
        )

        if is_frame_settled:
            for charuc_board in self.charuco_boards:
                # Loop through all the boards in the scene
                (
                    object_points,
                    image_points,
                    charuco_corners,
                    charuco_ids,
                    marker_corners,
                    marker_ids,
                ) = self._get_calibration_points(color_image, charuc_board)

                color = [255, 0, 0]

                this_board_valid = charuc_board.check_valid_detection(
                    charuco_ids, marker_ids
                ) and charuc_board.check_enough_corners_detected(charuco_ids)

                if this_board_valid:
                    color = [0, 255, 0]

                    # 2. Only append points if the board was actually detected
                    if object_points is not None and len(object_points) > 0:
                        object_points_all_boards_in_scene.append(object_points)
                        image_points_all_boards_in_scene.append(image_points)

                valid_board_detected = valid_board_detected or this_board_valid

                annotated_img = self._annotate_img(
                    annotated_img,
                    object_points,
                    image_points,
                    charuco_corners,
                    charuco_ids,
                    marker_corners,
                    marker_ids,
                    color,
                )
        
        if not is_frame_settled:
            font = cv2.FONT_HERSHEY_SIMPLEX
            color = (255, 0, 255)
            annotated_img = cv2.putText(
                annotated_img,
                f"Waiting for image to settle",
                (10, 80),
                font,
                2.5,
                color,
                4,
            )

        # 4. Handle logging and state updates outside the board loop
        is_valid_to_save = False
        if not save_image_to_disk or (capture_triggered and valid_board_detected):
            is_valid_to_save = True

            msg = f"Accepted {self.camera_type.name}."
            self.log_message(msg, LogLevels.INFO)

        if is_frame_settled and capture_triggered:
            self.capture_requested = False  # Reset manual flag, when frame has been settled and even if there is no board.

        self.visualizer.show_detected_corners(annotated_img)
        self.visualizer.log_capture_status(
            self.time_between_image_captures is not None,
            self.is_capture_request_pending(),
        )

        if not is_valid_to_save:
            return

        if self.image_size is None:
            # Note: OpenCV calibrateCamera usually expects (width, height).
            # Normally we should do color_image.shape[:2][::-1], but we'll keep it (height, width) here and invert the input to calibrateCamera later so we're not confused by the OpenCV one-off.
            self.image_size = color_image.shape

        if len(object_points_all_boards_in_scene) == 0:
            msg2 = "Skipping this image; object_points array is empty. Are there no markers in the scene?"
            self.log_message(msg2, LogLevels.WARN)
            return

        # Append this frame's array to the historical list of all frames
        combined_object_points = (
            self.all_object_points + object_points_all_boards_in_scene
        )
        combined_image_points = self.all_image_points + image_points_all_boards_in_scene

        calibration_results = self.do_calibration(
            object_points=combined_object_points, image_points=combined_image_points
        )

        if calibration_results is None:
            msg2 = "Calibration failed, not saving this image."
            self.log_message(msg2, LogLevels.ERROR)
            return

        self.all_object_points = combined_object_points
        self.all_image_points = combined_image_points
        self.all_timestamps.append(timestamp)

        self.last_calibration = calibration_results

        # self.visualizer.update_pointcloud(
        #     self.last_calibration, self.all_object_points, self.all_timestamps
        # )

        self.last_save_time = time.time()
        self.visualizer.show_last_saved(color_image)

        if save_image_to_disk:
            self._save_image(color_image, timestamp=str(timestamp))

    def show_interactive_windows(self):
        self.visualizer.show_detected_corners(np.zeros((400, 400, 3), dtype=np.uint8))
        self.visualizer.show_last_saved(np.zeros((400, 400, 3), dtype=np.uint8))
        self.visualizer.log_capture_status(
            self.time_between_image_captures is not None,
            self.is_capture_request_pending(),
        )
        self.visualizer.setup_blueprint([self.camera_type.name])

    def has_frame_been_stable(self):
        return self.frame_settled_detector.has_frame_been_stable()

    def is_capture_request_pending(self):
        return self.capture_requested

    def show_dropped_frame(self):
        self.visualizer.show_dropped_frame()


class CalibrateIntrinsicsThreeHeadCameras(CalibrateIntrinsics):

    def __init__(
        self,
        recording_directory: str,
        timestamp: str | None,
        is_use_last_recording: bool,
        charuco_board_names: list[str],
        time_between_image_captures: float | None,
        use_center_camera: bool,
        not_interactive: bool = False,
    ) -> None:

        self.calibration_instances = [
            CalibrateIntrinsics(
                recording_directory=recording_directory,
                camera_type=RGBCameras.left(),
                timestamp=timestamp,
                is_use_last_recording=is_use_last_recording,
                charuco_board_names=charuco_board_names,
                time_between_image_captures=time_between_image_captures,
                not_interactive=not_interactive,
            ),
            CalibrateIntrinsics(
                recording_directory=recording_directory,
                camera_type=RGBCameras.right(),
                timestamp=timestamp,
                is_use_last_recording=is_use_last_recording,
                charuco_board_names=charuco_board_names,
                time_between_image_captures=time_between_image_captures,
                not_interactive=not_interactive,
            ),
        ]
        if use_center_camera:
            self.calibration_instances.append(
                CalibrateIntrinsics(
                    recording_directory=recording_directory,
                    camera_type=RGBCameras.center(),
                    timestamp=timestamp,
                    is_use_last_recording=is_use_last_recording,
                    charuco_board_names=charuco_board_names,
                    time_between_image_captures=time_between_image_captures,
                    not_interactive=not_interactive,
                )
            )

    def save_calibration(self):
        [c.save_calibration() for c in self.calibration_instances]

    def log_message(self, text: str, level: LogLevels):
        # [c.log_message(text, level) for c in self.calibration_instances]
        self.calibration_instances[0].log_message(text, level)

    def log_instructions(self, text: str):
        # [c.log_instructions(text) for c in self.calibration_instances]
        self.calibration_instances[0].log_instructions(text)

    def print_calibration_results(
        self, calibration_results: CalibrateCameraResults | None = None
    ):
        if calibration_results is not None:
            raise NotImplementedError(
                "Passing calibration_results to print_calibration_results is not supported."
            )
        [c.print_calibration_results() for c in self.calibration_instances]

    def do_calibration(
        self,
        object_points: list | None = None,
        image_points: list | None = None,
        frame_number: int | None = None,
    ):
        if (
            object_points is not None
            or image_points is not None
            or frame_number is not None
        ):
            raise NotImplementedError(
                "Passing object_points or image_points or frame_number to do_calibration is not supported."
            )

        [c.do_calibration() for c in self.calibration_instances]

    def request_capture(self):
        [c.request_capture() for c in self.calibration_instances]

    def process_image_frame(
        self,
        frame: ImageFrame | None,
        save_image_to_disk: bool,
        use_stable_frames_only: bool,
    ):
        raise NotImplementedError("Use process_synced_image_frame instead.")

    def process_synced_image_frame(
        self,
        frame: SyncedImageFrame | None,
        save_image_to_disk: bool,
        use_stable_frames_only: bool,
    ):
        if frame is None:
            raise ValueError("Frames are empty.")
        
        if frame.center is None:
            # self.show_dropped_frame()
            return # Center frame is not synced, skip this set of images.

        for c in self.calibration_instances:
            frame_to_process = frame.get_frame_by_camera_type(c.camera_type)
            frame_to_process.timestamp = frame.timestamp # Overwrite the timestamp with the synced timestamp so they save to disk with the same name.
            c.process_image_frame(
                frame=frame_to_process,
                save_image_to_disk=save_image_to_disk,
                use_stable_frames_only=use_stable_frames_only,
            )

    def calculate_per_image_rmse(self):
        [c.calculate_per_image_rmse() for c in self.calibration_instances]

    def show_interactive_windows(self):
        [c.show_interactive_windows() for c in self.calibration_instances]
        
        camera_names = [c.camera_type.name for c in self.calibration_instances]
        self.calibration_instances[-1].visualizer.setup_blueprint(camera_names)

    def has_frame_been_stable(self):
        return all([c.has_frame_been_stable() for c in self.calibration_instances])

    def is_capture_request_pending(self):
        return any([c.is_capture_request_pending() for c in self.calibration_instances])

    def show_dropped_frame(self):
        [c.show_dropped_frame() for c in self.calibration_instances]


def _do_calibration(
    all_object_points,
    all_image_points,
    image_size: tuple[int, int],
    camera_type: RGBCameras,
    frame_number: int,
):
    """Peforms a camera's calibration"""

    # Perform Calibration
    width = image_size[1]
    height = image_size[0]

    distortion_model = camera_type.config.distortion_model

    if distortion_model is None:
        raise ValueError(
            "Distortion model cannot be none, make sure to update the camera type config with the distortion model to use."
        )
    
    camera_matrix, distortion_coefficients, reprojection_error, rotation_vectors, translation_vectors = camera_calibrate(all_object_points, all_image_points, width, height, distortion_model)

    projection_matrix = np.dot(camera_matrix, np.eye(3, 4)).tolist()

    calibration = CalibrateCameraResults(
        camera_name=camera_type.name,
        calibration_date=datetime.datetime.now(),
        image_size=list(image_size),
        number_of_images_processed=frame_number,
        number_of_images_used=frame_number,
        reprojection_error=reprojection_error,
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_coefficients,
        projection_matrix=projection_matrix,
        distortion_model=distortion_model,
        rectification_matrix=np.zeros(9).reshape((3, 3)),
        rotation_vectors=list(rotation_vectors),
        translation_vectors=list(translation_vectors),
        focal_length_mm=(
            RGBCameraCalibration.get_focal_length_mm(
                camera_matrix, pixel_size_mm=camera_type.config.sensor_pixel_size_mm
            )
            if camera_type.config.sensor_pixel_size_mm is not None
            else None
        ),
    )

    return calibration


def _save_calibration_to_ros_yaml(calibration: CalibrateCameraResults, filepath: str):
    data_dict = asdict(calibration)

    for key in [
        "camera_matrix",
        "distortion_coefficients",
        "projection_matrix",
        "rectification_matrix",
    ]:
        if key in data_dict:
            matrix = data_dict[key]
            if matrix is None:
                data_dict[key] = None
                continue

            if isinstance(matrix, list):
                m = np.array(matrix)
            else:
                m = matrix

            if not hasattr(m, "shape"):
                data_dict[key] = m
                continue

            if m.ndim == 1:
                rows = 1
                cols = m.shape[0]
                data = m.tolist()
            elif m.ndim == 2:
                rows = m.shape[0]
                cols = m.shape[1]
                data = m.flatten().tolist()
            else:
                print(
                    f"Warning: Matrix '{key}' with unexpected shape {m.shape} encountered."
                )
                data_dict[key] = m.tolist()
                continue

            data_dict[key] = {"rows": rows, "cols": cols, "data": data}

    serialized_dict = CalibrateCameraResults._serialize(data_dict)
    serialized_dict["camera_type"] = serialized_dict["camera_name"]

    camera_name = calibration.camera_name
    if "left" in calibration.camera_name:
        camera_name = "/left"
    elif "right" in calibration.camera_name:
        camera_name = "/right"
    elif "center" in calibration.camera_name:
        camera_name = "/center"
    serialized_dict["camera_name"] = camera_name

    serialized_dict["image_width"] = calibration.image_size[1]
    serialized_dict["image_height"] = calibration.image_size[0]
    serialized_dict["distortion_model"] = calibration.distortion_model.get_model_name()
    serialized_dict["fleet_id"] = os.environ.get("HELLO_FLEET_ID", "")

    try:
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        if os.path.exists(filepath):
            import shutil
            mod_time = int(os.path.getmtime(filepath))
            p = Path(filepath)
            backup_path = p.with_name(f"{p.stem}_backup_{mod_time}{p.suffix}")
            shutil.copy2(filepath, backup_path)
            print(f"Backed up {filepath} to {backup_path}")

        with open(filepath, "w") as yaml_file:
            yaml.safe_dump(
                serialized_dict, yaml_file, default_flow_style=None, sort_keys=False
            )
        print(f"Successfully saved calibration file to: {filepath}")

    except IOError as e:
        print(f"Error: Could not write to file {filepath}. {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


def save_calibration(
    calibration: CalibrateCameraResults,
    image_directory: str | None,
    camera_type: RGBCameras,
    log_callback: Callable[[str, LogLevels], None],
):
    calibration_results = calibration.get_serializable()

    ros_yaml_file_to_save = (
        RGBCameraCalibrationFile.LEFT.get_camera_calibration_file_path()
    )
    if camera_type.is_right():
        ros_yaml_file_to_save = (
            RGBCameraCalibrationFile.RIGHT.get_camera_calibration_file_path()
        )
    elif camera_type.is_center():
        ros_yaml_file_to_save = (
            RGBCameraCalibrationFile.CENTER.get_camera_calibration_file_path()
        )

    _save_calibration_to_ros_yaml(calibration, ros_yaml_file_to_save)

    if image_directory is not None:
        results_file_time = time.strftime("%Y%m%d%H%M%S")
        results_file_name = (
            image_directory
            + "camera_calibration_results_"
            + results_file_time
            + ".yaml"
        )
        
        calibration_results["fleet_id"] = os.environ.get("HELLO_FLEET_ID", "")

        with open(results_file_name, "w") as file:
            yaml.dump(calibration_results, file, sort_keys=True)

        log_callback(
            f"saved calibration results to {results_file_name}", LogLevels.INFO
        )

    existing_stretch_user_calibration_file = {}
    try:
        with open(
            RGBCameraCalibrationFile.USER.get_camera_calibration_file_path(), "r"
        ) as file:
            existing_stretch_user_calibration_file = yaml.safe_load(file)
    except FileNotFoundError:
        log_callback(
            f"The file '{RGBCameraCalibrationFile.USER.get_camera_calibration_file_path()}' was not found, creating it.",
            LogLevels.ERROR,
        )

    user_calib_path = RGBCameraCalibrationFile.USER.get_camera_calibration_file_path()
    if os.path.exists(user_calib_path):
        import shutil
        mod_time = int(os.path.getmtime(user_calib_path))
        p = Path(user_calib_path)
        backup_path = p.with_name(f"{p.stem}_backup_{mod_time}{p.suffix}")
        shutil.copy2(user_calib_path, backup_path)
        log_callback(f"Backed up {user_calib_path} to {backup_path}", LogLevels.INFO)

    existing_stretch_user_calibration_file[camera_type.name] = calibration_results
    with open(
        user_calib_path, "w"
    ) as file:
        yaml.dump(existing_stretch_user_calibration_file, file, sort_keys=True)

    log_callback(
        f"and {user_calib_path}",
        LogLevels.INFO,
    )


def show_image(image, title, wait_key: int):
    window_name = f"{title}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, image)
    cv2.waitKey(wait_key)


def draw_3d_axis(
    img, object_points, image_points, camera_matrix, distortion_coefficients, distortion_model:DistortionModels
) -> cv2.UMat:
    if len(image_points) >= 6:
        success, rvec, tvec = solve_pnp(
            object_points,
            image_points,
            camera_matrix,
            distortion_coefficients,
            distortion_model=distortion_model,
        )

        if success:
            draw_frame_axes(
                img, camera_matrix, distortion_coefficients, rvec, tvec, 0.1, distortion_model
            )
    return img


def calibration_pipeline(
    calibration: CalibrateIntrinsics, camera_type: RGBCameras, instructions: str
):

    calibration.log_instructions(instructions)

    calibration.show_interactive_windows()

    rgb_pipeline_controller = RGBPipelineController(
        camera_type=camera_type,
        recording_directory=None,
        show_image_in=None,
        is_rotate=False,
        is_rectify=False,
        is_crop=False,
        ai_models_to_use=[],
        detect_aruco_marker_size=None,
    )

    calibration.log_instructions("""
    These exposure settings work best for 450-650 lux ambient lighting.
    This was tested by using the max brightness and the white light setting on the 
    2800-6500K Dimmable Photography Light Panels set 2ft horizontally from the mast on either side of the robot.
    The height of the bottom of each light panel is 5ft from the floor.
    The panels are angled 45 degrees toward the charuco board vertically, about 10 degrees toward the floor.
    """)
    rgb_pipeline_controller.set_calibration_exposure_preset()

    if isinstance(calibration, CalibrateIntrinsicsThreeHeadCameras):
        for frame in rgb_pipeline_controller.get_frame_synced(is_run_pipeline=True):
            calibration.process_synced_image_frame(
                frame, save_image_to_disk=True, use_stable_frames_only=True
            )
    else:
        for frame in rgb_pipeline_controller.get_frame(is_run_pipeline=True):
            calibration.process_image_frame(
                frame, save_image_to_disk=True, use_stable_frames_only=True
            )


def _calibrate_intrinsics(
    recording_directory: str,
    timestamp: str | None,
    is_use_last_recording: bool,
    is_not_interactive: bool,
    camera_type: RGBCameras,
    charuco_board_names: list[str],
    auto_capture_interval: float | None,
):

    if camera_type.is_synced_camera_type():
        calibration = CalibrateIntrinsicsThreeHeadCameras(
            recording_directory=recording_directory,
            timestamp=timestamp,
            is_use_last_recording=is_use_last_recording,
            charuco_board_names=charuco_board_names,
            use_center_camera=camera_type == RGBCameras.synced_left_right_center(),
            time_between_image_captures=auto_capture_interval,
        )
    else:
        calibration = CalibrateIntrinsics(
            recording_directory=recording_directory,
            camera_type=camera_type,
            timestamp=timestamp,
            is_use_last_recording=is_use_last_recording,
            charuco_board_names=charuco_board_names,
            time_between_image_captures=auto_capture_interval,
        )

    if is_not_interactive:
        if timestamp is None and not is_use_last_recording:
            raise Exception(
                "You must specify a --replay_from_folder or --replay_last flag if you are not capturing from a camera device."
            )

        calibration.do_calibration()
        calibration.save_calibration()

        return

    instructions = f"""
You are about to perform camera instrinsic calibration.

A rerun.io window should open up showing you the live feed from the camera(s).

Please read the following instructions on how to perform a good calibration.

You may need to return to the terminal to save the calibration yaml file by pressing 's', after you are done with the calibration process.

Before you begin, please consider the following:
1. Make sure your lens is focused. You can use the `REx_camera_focus` script to check if the lens is focused. 
    A good focus will allow you to read text up to ~3ft away with the fisheye lenses.
2. Lighting is very important; avoid direct sunlight on the lens or the ChArUco board. 
    Balanced and diffuse ambient lighting yields the best results.
3. Capture at least {RECOMMENDED_MINIMUM_NUMBER_OF_IMAGES_TO_CAPTURE} images 
    and produce a Project Error of less than {RECOMMENDED_REPROJECTION_ERROR} for a good calibration.
4. It might be helpful to do calibration in a room with no movement around the robot. 
    This calibration uses motion detection to decide if the image is stable enough before capturing, to minimize blurry images. 
    Warning: If there is motion in the camera frame, you may notice that the auto-capture is not triggering for a long time.

    
The capture of a calibration frame happens automatically when the following conditions are met:
1. This program identifies corners from your CharUco board. The markers overlay will turn green, indicating a valid pose. 
    A blue overlay or an overlay not appearing at all will mean an invalid pose. 
    NOTE: you may have a valid pose with a green overlay, but error messages may appear in the console, indicating an error with calibration. 
    Please be aware of the console messages while you are performing calibration.
2. {auto_capture_interval} seconds have passed since the last saved image. 
    Please move to the next pose within {auto_capture_interval} seconds after an image is captured.  
    * Note: You may choose to trigger capture manually by setting "--auto_capture_interval None", and using the 'c' key in the terminal to capture a frame.


This program is expecting the following ChArUco board(s):
{[CharucoBoards[charuco_board_name].get_board_config(use_high_MP_corner_refinement=camera_type.is_center()) for charuco_board_name in charuco_board_names]}
* Note: You can change the charuco board(s) by passing comma separated values to the --charuco_board_names flag.
If you use multiple boards, make sure the id's of the ArUco markers do not overlap on any of the boards being used in tandem.


After calibration is finished, you can verify the calibration by using an aruco marker of a known size, 
    and running `stretch_camera_show --detect_aruco_marker_size KNOWN_SIZE --left_right_center --opencv`.

When you are happy with a calibration, press 's' and enter, to save a few yaml files. 

One yaml file will be saved to your recording directory, and copies of it will be saved in the HELLO_FLEET_PATH/HELLO_FLEET_ID directory to 
    be used by ROS2 and other scripts that require camera calibration.

All your images are saved in the recording directory: {recording_directory}.

You may rerun the calibration on those images by using the --not_interactive flag, 
    and passing either the timestamp of the recordings via --replay_from_folder or the --replay_last flag. 

You an quit the script safely at any time using CTRL+C. Your images will remain. To resume recording to the same folder, you can use the --replay_last flag.

Interactive Controls:
- Press 's' to save calibration and exit.
- Press 't' to toggle between Auto-Capture and Manual-Capture modes.
- Press 'c' to manually capture an image (only works when Auto-Capture is disabled).
- Press 'o' to remove outliers and recalibrate.
- Press 'CTRL + C' to quit without saving.
"""

    def keyboard_controller():
        key = (
            input(
                "Press 's' (save), 't' (toggle auto-capture), 'c' (manual-capture), 'CTRL+C' (quit): "
            )
            .strip()
            .lower()
        )
        if key == "s":
            print("Saving...")
            calibration.save_calibration()
            print("Saved!")
            return
        elif key == "c":
            calibration.request_capture()
        else:
            print("Unknown command:", key)

    def keyboard_listener():
        while True:
            keyboard_controller()

    listener_thread = threading.Thread(target=keyboard_listener, daemon=True)
    listener_thread.start()

    calibration_pipeline(
        calibration=calibration, camera_type=camera_type, instructions=instructions
    )

    listener_thread.join()


def _parse_args(parser: argparse.ArgumentParser | None = None):

    parser = parser or argparse.ArgumentParser(
        prog="""
The `CalibrateIntrinsics` handles the logic for performing camera instrinsic calibration on images saved to disk.

It also supports capturing images that are useful for camera calibration from live camera streams. 

Images are only saved if they pass a few critera that aid in achieving a good calibration quality.

You can run this file to do manual calibration. This script assumes it is running on the robot.

You could also use `calibrate_intrinsics_robot_move.py` to control the robot while doing calibration.
"""
    )

    parser.add_argument(
        "-d",
        "--recording_directory",
        type=str,
        default=DEFAULT_IMAGES_SAVE_PATH,
        help=f"Directory used to record the data, if provided, images will be saved to disk in this directory. Otherwise {DEFAULT_IMAGES_SAVE_PATH} is used.",
    )

    parser.add_argument(
        "--not_interactive",
        action="store_true",
        help="If this is true, a --replay_from_folder or --replay_last must be passed in. The camera device will not be used, and calibration will be done on previously recorded images.",
    )

    parser.add_argument(
        "--charuco_board_names",
        type=str,
        # default="BOARD_5x7_37mm_27mm_4x4_start_id_20",
        default="BOARD_5x7_37mm_27mm_4x4_start_id_0",
        # default="BOARD_5x7_37mm_27mm_4x4_start_id_0,BOARD_5x7_37mm_27mm_4x4_start_id_20",
        # default="BOARD_5x7_37mm_27mm_4x4_start_id_0,BOARD_5x7_37mm_27mm_4x4_start_id_20,BOARD_5x7_37mm_27mm_4x4_start_id_40",
        help=f"Name of the CharucoBoards enum to use for calibration. Comma separated values of {[c.name for c in CharucoBoards]}",
    )

    parser.add_argument(
        "--auto_capture_interval",
        type=str,
        default=str(AUTO_SAVE_SLEEP),
        help=f"Intervals between auto capture. Default: {AUTO_SAVE_SLEEP}s. Pass 'None' to disable auto-capture.",
    )

    parser.add_argument(
        "-l", "--left", action="store_true", help="Use the left RGB camera."
    )
    parser.add_argument(
        "-r", "--right", action="store_true", help="Use the right RGB camera."
    )
    parser.add_argument(
        "-c", "--center", action="store_true", help="Use the center RGB."
    )
    parser.add_argument(
        "-lr",
        "--left_right",
        action="store_true",
        help="Use the left and right RGB cameras.",
    )
    parser.add_argument(
        "-lrc",
        "--left_right_center",
        action="store_true",
        help="Use all three RGB cameras: left, right and center.",
    )
    parser.add_argument(
        "--replay_from_folder", help="Timestamp of the recording to process"
    )
    parser.add_argument(
        "--replay_last",
        action="store_true",
        help="Use the last recorded folder timestamp inside the provided recording dir. This will load existing images and 'append' new saves to this folder.",
    )
    args, _ = parser.parse_known_args()

    recording_directory = args.recording_directory
    timestamp = args.replay_from_folder
    is_use_last_recording = args.replay_last
    is_not_interactive = args.not_interactive
    charuco_board_names = args.charuco_board_names.split(",")
    auto_capture_interval = args.auto_capture_interval
    auto_capture_interval = (
        None
        if auto_capture_interval.lower() == "none"
        else float(auto_capture_interval)
    )

    if timestamp is not None and is_use_last_recording:
        raise ValueError(
            "You cannot pass a timestamp via --replay_from_folder and use the --replay_last flag."
        )

    camera_type = None
    if args.left:
        camera_type = RGBCameras.left()
    elif args.right:
        camera_type = RGBCameras.right()
    elif args.center:
        camera_type = RGBCameras.center()
    elif args.left_right:
        camera_type = RGBCameras.synced_left_right()
    elif args.left_right_center:
        camera_type = RGBCameras.synced_left_right_center()
    else:
        raise Exception(
            "You must specify one of --left, --right, --center, --left_right or --left_right_center."
        )

    return (
        recording_directory,
        timestamp,
        is_use_last_recording,
        is_not_interactive,
        camera_type,
        charuco_board_names,
        auto_capture_interval,
    )


def REx_calibrate_intrinsics(interactive: bool):
    (
        recording_directory,
        timestamp,
        is_use_last_recording,
        _,
        camera_type,
        charuco_board_names,
        auto_capture_interval,
    ) = _parse_args()

    _calibrate_intrinsics(
        recording_directory=recording_directory,
        timestamp=timestamp,
        is_use_last_recording=is_use_last_recording,
        is_not_interactive=not interactive,
        camera_type=camera_type,
        charuco_board_names=charuco_board_names,
        auto_capture_interval=auto_capture_interval,
    )


if __name__ == "__main__":
    params = _parse_args()
    _calibrate_intrinsics(*params)
