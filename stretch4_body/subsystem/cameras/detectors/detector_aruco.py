"""
Usage of the methods in this file:
```
detected_markers = find_all_aruco_markers(input_image, ArucoDictionary.DICT_4X4_100)
output_image = draw_aruco_detections(input_image, detected_markers)
```
"""

import cv2
import numpy as np

from stretch4_body.subsystem.cameras.cv_utils import draw_frame_axes, solve_pnp
from stretch4_body.subsystem.cameras.models.camera_calibration import (
    RGBCameraCalibration,
)
from stretch4_body.subsystem.cameras.enums.aruco_dictionary import ArucoDictionary


def find_all_aruco_markers(image, dictionaries: list[ArucoDictionary]):
    """
    Detects ArUco markers from various dictionaries in a given image.

    """
    all_detections = {}
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    for aruco_dict in dictionaries:
        detector = aruco_dict.get_aruco_detector()
        (corners, ids, rejected) = detector.detectMarkers(gray_image)

        if ids is not None and len(ids) > 0:
            all_detections[aruco_dict.name] = (corners, ids)

    return all_detections



def get_aruco_pose(corners, marker_length, calibration: RGBCameraCalibration):
    """
    Returns the 4x4 matrix T_C tracking the Aruco marker pose in camera frame,
    and the 2D image center.
    """
    marker_points = np.array(
        [
            [-marker_length / 2, marker_length / 2, 0],
            [marker_length / 2, marker_length / 2, 0],
            [marker_length / 2, -marker_length / 2, 0],
            [-marker_length / 2, -marker_length / 2, 0],
        ],
        dtype=np.float32,
    )

    success, rvec, tvec = solve_pnp(
        object_points=marker_points,
        image_points=corners,
        camera_matrix=calibration.camera_matrix,
        distortion_coefficients=calibration.distortion_coefficients,
        distortion_model=calibration.distortion_model,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )

    if success:
        return rvec, tvec

    return None, None


def draw_aruco_pose(
    image,
    corners,
    marker_length: float,
    marker_ids,
    camera_calibration: RGBCameraCalibration,
):
    rvec, tvec = get_aruco_pose(
        corners,
        marker_length,
        camera_calibration
    )

    if rvec is None or tvec is None:
        return

    # Draw axis for each marker (x=red, y=green, z=blue)
    draw_frame_axes(
        image,
        camera_calibration.camera_matrix,
        camera_calibration.distortion_coefficients,
        rvec,
        tvec,
        marker_length * 0.5,
        distortion_model=camera_calibration.distortion_model,
    )

    distance = np.linalg.norm(tvec)

    corner_points = corners.reshape((4, 2)).astype(int)
    top_left = corner_points[0]
    cv2.putText(
        image,
        f"ID: {marker_ids} Dist: {distance:.3f}m",
        (top_left[0], top_left[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )


def draw_aruco_detections(
    image,
    detections,
    camera_calibration: RGBCameraCalibration | None,
    marker_length: float,
):
    """
    Runs ArUco detection if a float >= 0.0 is provided. If length is 0.0, the ArUco markers will be detected, but distance will not be printed. If calibration is available, ArUco pose will be displayed. If length > 0.0 and calibration is available, distance to the marker will be displayed.
    """
    marked_image = image.copy()

    if not detections:
        return marked_image  # Return original image if no detections

    # Loop through each dictionary that had a detection
    for aruco_name, (corners, marker_ids) in detections.items():
        marker_ids = marker_ids.flatten()

        # Loop through each detected marker for this dictionary
        for marker_corner, marker_id in zip(corners, marker_ids):

            if camera_calibration is not None and marker_length > 0.0:
                draw_aruco_pose(
                        marked_image,
                        marker_corner,
                        marker_length,
                        marker_id,
                    camera_calibration,
                )

            # Extract the corner points
            corner_points = marker_corner.reshape((4, 2)).astype(int)
            top_left = corner_points[0]

            # 1. Draw the bounding box around the detected marker
            cv2.polylines(
                img=marked_image,
                pts=[corner_points],
                isClosed=True,
                color=(0, 255, 0),  # Green color
                thickness=2,
            )

            # 2. Draw the ArUco marker ID on the image
            text = f"ID: {marker_id}"
            cv2.putText(
                img=marked_image,
                text=text,
                org=(top_left[0], top_left[1] - 15),  # Position text above the marker
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.6,
                color=(0, 0, 255),  # Red color
                thickness=2,
            )

    return marked_image


def do_aruco_detection(
    color_image: np.ndarray,
    camera_calibration: RGBCameraCalibration | None,
    marker_length: float,
    dictionaries_to_detect: list[ArucoDictionary],
):

    detected_markers = find_all_aruco_markers(
        color_image, dictionaries=dictionaries_to_detect
    )
    output_image = draw_aruco_detections(
        color_image, detected_markers, camera_calibration, marker_length
    )

    return output_image
