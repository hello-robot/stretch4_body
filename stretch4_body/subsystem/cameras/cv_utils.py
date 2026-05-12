import cv2
import numpy as np
from dataclasses import dataclass
from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels


def _project_points_omnidir_model(
    object_points, rvec, tvec, camera_matrix, distortion_coefficients
):
    obj_pts_formatted = np.array(object_points, dtype=np.float64).reshape(-1, 1, 3)

    # 2. Ensure rvec and tvec are also float arrays
    rvec_formatted = np.array(rvec, dtype=np.float64)
    tvec_formatted = np.array(tvec, dtype=np.float64)
    D_array = distortion_coefficients[:-1]
    xi_scalar = float(distortion_coefficients[-1])
    return cv2.omnidir.projectPoints(
        obj_pts_formatted,
        rvec_formatted,
        tvec_formatted,
        camera_matrix,
        xi_scalar,
        D_array,
    )[0]


def project_points(
    object_points,
    rvec,
    tvec,
    camera_matrix,
    distortion_coefficients,
    distortion_model: DistortionModels,
):
    if distortion_model == DistortionModels.omnidir:
        return _project_points_omnidir_model(
            object_points, rvec, tvec, camera_matrix, distortion_coefficients
        )

    # Fisheye requires a specific shape: (N, 1, 3)
    object_points = (
        object_points.reshape(-1, 1, 3)
        if distortion_model.is_fisheye()
        else object_points
    )
    if distortion_model.is_fisheye():
        return cv2.fisheye.projectPoints(
            object_points, rvec, tvec, camera_matrix, distortion_coefficients
        )[0]
    else:
        # return _project_points_omnidir_model(object_points, rvec, tvec, camera_matrix, distortion_coefficients)
        return cv2.projectPoints(
            object_points, rvec, tvec, camera_matrix, distortion_coefficients
        )[0]


def _solve_pnp_omnidir_model(
    object_points,
    image_points,
    camera_matrix,
    distortion_coefficients,
    *,
    flags=0,
    rvec: np.ndarray | None = None,
    tvec: np.ndarray | None = None
) -> tuple[bool, np.ndarray, np.ndarray]:
    undistorted_image_points = _undistort_points_omnidir_model(
        image_points, camera_matrix, distortion_coefficients
    )

    # Because we already flattened the points, we tell solvePnP that
    # the camera matrix is a blank Identity matrix, and distortion is zero.
    blank_camera_matrix = np.eye(3, dtype=np.float64)
    blank_dist_coeffs = np.zeros((4, 1), dtype=np.float64)

    success, rvecs, tvecs = cv2.solvePnP(
        object_points,
        undistorted_image_points,
        blank_camera_matrix,
        blank_dist_coeffs,
        # flags=cv2.SOLVEPNP_ITERATIVE # SQPNP or EPnP also work well here
        flags=flags,
        rvec=rvec,
        tvec=tvec,
    )

    return success, rvecs, tvecs


def _undistort_points_omnidir_model(
    pts_2d,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
):

    # 1. Setup your calibration parameters (using our fix from before)
    D_array = distortion_coefficients[:-1]
    xi_array = np.array([distortion_coefficients[-1]], dtype=np.float64)

    # 2. Undistort the 2D pixels to ideal normalized image coordinates.
    # By passing an Identity matrix for R and P, we are telling the function:
    # "Map these pixels to a perfect, distortion-free pinhole plane located at Z=1"
    R_identity = np.eye(3, dtype=np.float64)
    P_identity = np.eye(3, dtype=np.float64)

    return cv2.omnidir.undistortPoints(
        pts_2d.astype(np.float32),
        camera_matrix,
        D_array,
        xi_array,
        R_identity,
        P_identity,
    )


def undistort_points(
    pts_2d,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    distortion_model: DistortionModels,
):
    if distortion_model == DistortionModels.omnidir:
        return _undistort_points_omnidir_model(
            pts_2d, camera_matrix, distortion_coefficients
        )
    if distortion_model.is_fisheye():
        return cv2.fisheye.undistortPoints(
            pts_2d, camera_matrix, distortion_coefficients
        )
    else:
        return cv2.undistortPoints(pts_2d, camera_matrix, distortion_coefficients)


def solve_pnp(
    object_points,
    image_points,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray,
    distortion_model: DistortionModels,
    *,
    flags=0,
    rvec: np.ndarray | None = None,
    tvec: np.ndarray | None = None
) -> tuple[bool, np.ndarray, np.ndarray]:
    """
    A helper to resolve cv2.solvePnP and cv2.fisheye.solvePnP without needing extra logic.
    """
    if distortion_model == DistortionModels.omnidir:
        return _solve_pnp_omnidir_model(
            object_points,
            image_points,
            camera_matrix,
            distortion_coefficients,
            flags=flags,
            rvec=rvec,
            tvec=tvec,
        )
        
    obj_pts = np.array(object_points)
    img_pts = np.array(image_points)

    if distortion_model.is_fisheye():
        obj_pts = obj_pts.reshape(-1, 1, 3).astype(np.float64)
        img_pts = img_pts.reshape(-1, 1, 2).astype(np.float64)
        return cv2.fisheye.solvePnP(
            obj_pts,
            img_pts,
            camera_matrix,
            distortion_coefficients,
            flags=flags,
        )
    else:
        # return _solve_pnp_omnidir_model(object_points, image_points, camera_matrix, distortion_coefficients)
        return cv2.solvePnP(
            obj_pts,
            img_pts,
            camera_matrix,
            distortion_coefficients,
            rvec=rvec,
            tvec=tvec,
            useExtrinsicGuess=(rvec is not None and tvec is not None),
            flags=flags,
        )


def draw_frame_axes(
    image, K, D, rvec, tvec, length, distortion_model: DistortionModels
):
    """
    An implementation of cv2.drawFrameAxes that supports a fisheye model.

    Example usage:
    ```
    success, rvec, tvec = solve_pnp(
            object_points,
            image_points,
            camera_matrix,
            distortion_coefficients,
            is_fisheye
        )

        if success:
            draw_frame_axes(img, camera_matrix, distortion_coefficients, rvec, tvec, 0.1, is_fisheye)
    ```
    """
    # 1. Define the 3D points for the origin and the ends of the X, Y, Z axes
    axis_points_3D = np.array(
        [
            [0, 0, 0],  # Origin
            [length, 0, 0],  # X-axis end
            [0, length, 0],  # Y-axis end
            [0, 0, length],  # Z-axis end
        ],
        dtype=np.float32,
    ).reshape(-1, 1, 3)

    # 2. Project these 3D points into 2D image space using the FISHEYE function
    axis_points_2D = project_points(
        object_points=axis_points_3D,
        rvec=rvec,
        tvec=tvec,
        camera_matrix=K,
        distortion_coefficients=D,
        distortion_model=distortion_model,
    )
    axis_points_2D = np.int32(axis_points_2D).reshape(-1, 2)

    # 3. Draw the lines (Origin to X, Y, Z)
    origin = tuple(axis_points_2D[0])
    image = cv2.line(
        image, origin, tuple(axis_points_2D[1]), (0, 0, 255), 3
    )  # X is Red
    image = cv2.line(
        image, origin, tuple(axis_points_2D[2]), (0, 255, 0), 3
    )  # Y is Green
    image = cv2.line(
        image, origin, tuple(axis_points_2D[3]), (255, 0, 0), 3
    )  # Z is Blue

    return image


@dataclass
class RectifyMaps:
    map1: np.ndarray
    map2: np.ndarray
    new_K: np.ndarray


def get_recify_maps(
    color_image: np.ndarray,
    *,
    sim_cam_matrix,
    sim_cam_distortion_coeffs,
    balance: float,
    fov_scale: float
):
    image_size = color_image.shape[:2][::-1]
    # from https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-part-2-13990f1b157f
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        sim_cam_matrix,
        sim_cam_distortion_coeffs,
        image_size,
        np.eye(3),
        new_size=image_size,
        balance=balance,
        fov_scale=fov_scale,
    )
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        sim_cam_matrix,
        sim_cam_distortion_coeffs,
        np.eye(3),
        new_K,
        image_size,
        cv2.CV_16SC2,
    )

    return RectifyMaps(map1, map2, new_K)


def rectify(
    color_image,
    camera_matrix,
    distortion_coefficients,
    distortion_model: DistortionModels,
    *,
    rectify_maps: RectifyMaps | None
):

    if distortion_model == DistortionModels.omnidir:
        D_array = distortion_coefficients[:-1]
        xi_scalar = distortion_coefficients[-1]

        xi_array = np.array([xi_scalar], dtype=np.float64)
        return cv2.omnidir.undistortImage(
            color_image,
            camera_matrix,
            D_array,
            xi_array,
            flags=cv2.omnidir.RECTIFY_PERSPECTIVE,
        )

    if not distortion_model.is_fisheye():
        # Non-fisheye camera undistort:
        return cv2.undistort(color_image, camera_matrix, distortion_coefficients)

    # Fisheye rectify:
    if rectify_maps is None:
        raise RuntimeError("The rectify_maps paramete is required")

    def fisheye_rectify(
        color_image: np.ndarray, depth_image: np.ndarray | None, map1, map2
    ):
        """https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html
        `balance`	Sets the new focal length in range between the min focal length and the max focal length. Balance is in range of [0, 1].
        `fov_scale`	Divisor for new focal length.

        Undistortion guide: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html#:~:text=%2C%20None)-,Undistortion,-Now%2C%20we%20can

        fyi, this did not work:
        rectified = cv2.fisheye.undistortImage(color_image, K=sim_cam_matrix, D=sim_cam_distortion_coeffs.flatten())
        undistortImage() is not working, but under the hood does the same thing as our implementation below below https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/fisheye.cpp#L622
        https://github.com/opencv/opencv/blob/4.x/modules/calib3d/src/fisheye.cpp#L622
        """

        def do_fisheye_undistort(image):
            return cv2.remap(
                image,
                map1,
                map2,
                interpolation=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
            )

        rectified = do_fisheye_undistort(color_image)

        rectified_depth_image = None
        if depth_image is not None:
            rectified_depth_image = do_fisheye_undistort(depth_image)

        return rectified, rectified_depth_image

    color_image, _ = fisheye_rectify(
        color_image=color_image,
        depth_image=None,
        map1=rectify_maps.map1,
        map2=rectify_maps.map2,
    )
    return color_image


def camera_calibrate(all_object_points, all_image_points, width, height, distortion_model:DistortionModels):

    distortion_coefficients = distortion_model.get_initial_distortion_coefficients()

    K_initial = distortion_model.get_initial_intrinsic_guess(width, height)
    D_initial = np.array([0.0] * 4)

    if distortion_model == DistortionModels.omnidir:
        # cv2.omnidir.calibrate requires np.float64 points and can fail if output arrays are pre-allocated with the wrong dimensions.
        all_object_points_64 = [pts.astype(np.float64) for pts in all_object_points]
        all_image_points_64 = [pts.astype(np.float64) for pts in all_image_points]

        (
            reprojection_error,
            camera_matrix,
            xi,
            distortion_coefficients,
            rotation_vectors,
            translation_vectors,
            idx,
        ) = cv2.omnidir.calibrate(
            all_object_points_64,
            all_image_points_64,
            (width, height),
            None,
            None,
            None,
            flags=distortion_model.get_flags(),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
        )
        if distortion_coefficients is not None and xi is not None:
            distortion_coefficients = np.append(distortion_coefficients.flatten(), xi.flatten())
    else:
        # Normal fisheye and non-fisheye:
        (
            reprojection_error,
            camera_matrix,
            distortion_coefficients,
            rotation_vectors,
            translation_vectors,
        ) = (cv2.fisheye.calibrate if distortion_model.is_fisheye() else cv2.calibrateCamera)(
            all_object_points,
            all_image_points,
            (width, height),  # OpenCV wants it in (width, height) format
            K_initial,
            D_initial,
            rvecs=None,
            tvecs=None,
            flags=distortion_model.get_flags(),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6),
        )
    return camera_matrix, distortion_coefficients, reprojection_error, rotation_vectors, translation_vectors