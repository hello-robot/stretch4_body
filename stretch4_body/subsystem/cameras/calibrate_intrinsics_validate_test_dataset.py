"""
This script helps the user collect a test dataset and validate camera intrinsics calibration by 
calculating the RMSE projection error for this new dataset that was not used to train the calibration model.
This tool is mostly for sanity checking and quick validation, it is not part of the calibration pipeline.
"""
import time

import cv2
import numpy as np

from stretch4_body.subsystem.cameras.calibrate_intrinsics import (
    AUTO_SAVE_SLEEP,
    CalibrateIntrinsics,
    _parse_args,
    calibration_pipeline,
)
from stretch4_body.subsystem.cameras.cv_utils import project_points, solve_pnp
from stretch4_body.subsystem.cameras.enums.log_levels import LogLevels
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame


class RGBCalibrationValidateRMSE(CalibrateIntrinsics):
    """This class utilizes the underlying logic of RGBCalibration to load and detect ChArUco boards."""

    def __init__(
        self,
        recording_directory: str,
        camera_type: RGBCameras,
        timestamp: str | None,
        is_use_last_recording: bool,
        charuco_board_names: list[str],
        time_between_image_captures: float | None,
    ) -> None:

        if camera_type.is_synced_camera_type():
            raise NotImplementedError(
                "Multiple cameras are not supported, use --left, --right or --center instead."
            )

        self.loaded_calibration = camera_type.load_calibration()

        self.total_squared_error = 0
        self.total_points = 0

        # Init will init the charuco board detector(s) and load the images, and then call process_image_frame as the images are being loaded.
        super().__init__(
            recording_directory,
            camera_type,
            timestamp,
            is_use_last_recording,
            charuco_board_names,
            time_between_image_captures=time_between_image_captures,
        )

    def process_image_frame(
        self, frame: ImageFrame | None, save_image_to_disk: bool, use_stable_frames_only:bool
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
            self.frame_settled_detector.check_stability_diff(color_image, threshold=2)
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

                if charuc_board.check_valid_detection(
                    charuco_ids, marker_ids
                ) and charuc_board.check_enough_corners_detected(charuco_ids):

                    valid_board_detected = True
                    color = [0, 255, 0]

                    # 2. Only append points if the board was actually detected
                    if object_points is not None and len(object_points) > 0:
                        object_points_all_boards_in_scene.append(object_points)
                        image_points_all_boards_in_scene.append(image_points)

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

        # This is a hack for the placeholder to support multiple boards later.
        object_points_all_boards_in_scene = object_points_all_boards_in_scene[0]
        image_points_all_boards_in_scene = image_points_all_boards_in_scene[0]

        if len(self.charuco_boards) > 1:
            raise NotImplementedError(
                "Calibration validation only works with one board in charuco_board_names right now."
            )

        MIN_NUMBER_OBJECT_POINTS_FOR_SOLVE_PNP = 4
        if (
            len(object_points_all_boards_in_scene)
            <= MIN_NUMBER_OBJECT_POINTS_FOR_SOLVE_PNP
        ):
            msg2 = f"Skipping this image; object_points array is empty or less than {MIN_NUMBER_OBJECT_POINTS_FOR_SOLVE_PNP}. Are there no markers in the scene?"
            print(msg2)
            self.log_message(msg2, LogLevels.WARN)
            return

        rmse_result = self.calculate_rmse(
            object_points=object_points_all_boards_in_scene,
            image_points=image_points_all_boards_in_scene,
        )

        if not rmse_result:
            msg2 = "RMSE calculation failed, not saving this image."
            print(msg2)
            self.log_message(msg2, LogLevels.ERROR)
            return

        self.last_save_time = time.time()
        self.visualizer.show_last_saved(color_image)

        if save_image_to_disk:
            self._save_image(color_image, timestamp=str(timestamp))

    def calculate_rmse(
        self,
        object_points,
        image_points,
    ):
        # 1. Find extrinsics (pose) using the FIXED intrinsics
        success, rvec, tvec = solve_pnp(
            object_points,
            image_points,
            self.loaded_calibration.camera_matrix,
            self.loaded_calibration.distortion_coefficients,
            distortion_model=self.loaded_calibration.distortion_model,
        )

        if not success:
            return False

        # 2. Project the 3D points back onto the 2D image plane
        projected_points = project_points(
            object_points,
            rvec,
            tvec,
            self.loaded_calibration.camera_matrix,
            self.loaded_calibration.distortion_coefficients,
            distortion_model=self.loaded_calibration.distortion_model,
        )

        # 3. Calculate the L2 Norm (Euclidean distance) between detected and projected points
        error = cv2.norm(image_points, projected_points, cv2.NORM_L2)

        # Store for overall calculation
        self.total_squared_error += error * error
        num_points = len(image_points)
        self.total_points += num_points

        # Image-specific RMSE
        img_rmse = np.sqrt((error * error) / num_points)
        self.log_message(f"{img_rmse=}", LogLevels.INFO)
        self.log_message(
            f"OVERALL VALIDATION RMSE: {self.get_overall_rmse():.4f} pixels across {self.total_points} total points.",
            LogLevels.INFO,
        )

        return True

    def get_overall_rmse(self):
        if self.total_points == 0:
            raise ValueError("There are no RMSE results yet.")

        overall_rmse = np.sqrt(self.total_squared_error / self.total_points)

        return overall_rmse


def main():
    (
        recording_directory,
        timestamp,
        is_use_last_recording,
        is_not_interactive,
        camera_type,
        charuco_board_names,
        auto_capture_interval,
    ) = _parse_args()

    instructions = """
Run this flag after you have completed and saved a calibration. This mode will guide you to collect new images, and use the camera matrix and distortion coefficients to compute the reprojection error of your calibration
"""

    calibration_validation = RGBCalibrationValidateRMSE(
        recording_directory=recording_directory,
        camera_type=camera_type,
        timestamp=timestamp,
        is_use_last_recording=is_use_last_recording,
        charuco_board_names=charuco_board_names,
        time_between_image_captures=AUTO_SAVE_SLEEP,
    )

    calibration_pipeline(
        calibration=calibration_validation,
        camera_type=camera_type,
        instructions=instructions,
    )


if __name__ == "__main__":
    """
    Example usage:
    python3 calibrate_intrinsics_validate.py --center --recording_dir ./recordings_calibration_validation --charuco_board_names BOARD_5x7_30mm_22mm_4x4_start_id_0
    """
    main()
