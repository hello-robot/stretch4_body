from dataclasses import asdict, dataclass, field
import datetime
import os
import numpy as np
import yaml
from enum import Enum, auto
import logging
import cv2
import cv2.aruco as aruco
import rerun as rr
import rerun.blueprint as rrb
from typing import TYPE_CHECKING

from stretch4_body.subsystem.cameras.cv_utils import project_points, undistort_points
from stretch4_body.subsystem.cameras.enums.distortion_models import DistortionModels

if TYPE_CHECKING:
    from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras


DEFAULT_CALIBRATION_FOLDER_PATH = f"{os.getenv('HELLO_FLEET_PATH')}/{ os.getenv('HELLO_FLEET_ID')}/calibration_cameras"
DEFAULT_IMAGES_SAVE_PATH = f"{DEFAULT_CALIBRATION_FOLDER_PATH}/calibration_images"


class RGBCameraCalibrationFile(Enum):
    LEFT = auto()
    CENTER = auto()
    RIGHT = auto()
    USER = auto()

    @property
    def camera_name(self):
        return self.name.lower()

    def _calibration_file_name(self) -> str:
        """Returns the camera calibration file from ~/stretch_user/stretch-se4-xxxx/calibration_cameras/{camera_name}.yaml"""
        if self.camera_name in ["left", "right", "center"]:
            return f"calibration_ros_camera_info_{self.camera_name}.yaml"
        if self.camera_name == "user":
            return "calibration_rgb_head_camera.yaml"
        raise ValueError(f"{self.camera_name} is not a valid camera name")

    def get_camera_calibration_file_path(self) -> str:
        """Get the calibration file path for the given camera name from HELLO_FLEET_PATH environment variable."""
        return f"{DEFAULT_CALIBRATION_FOLDER_PATH}/{self._calibration_file_name()}"


@dataclass
class RGBCameraCalibration:
    width: int
    height: int
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    distortion_model: DistortionModels
    max_half_fov: float | None = field(init=False)

    def __post_init__(self):
        """Dynamically computes the max field of view from the distortion model."""
        w, h = self.width, self.height

        # 1. Sample pixels slightly inside the boundary to avoid invalid black corners
        mx = w * 0.02  # 2% horizontal margin
        my = h * 0.02  # 2% vertical margin
        xs = np.linspace(mx, w - mx, 10)
        ys = np.linspace(my, h - my, 10)

        pts = []
        for x in xs:
            pts.append([x, my])
            pts.append([x, h - my])
        for y in ys:
            pts.append([mx, y])
            pts.append([w - mx, y])

        # Format for OpenCV: shape (N, 1, 2)
        pts_2d = np.array(pts, dtype=np.float32).reshape(-1, 1, 2)

        try:
            # 2. Push boundary pixels through the inverse distortion model
            undist = undistort_points(
                    pts_2d, self.camera_matrix, self.distortion_coefficients, self.distortion_model
                )

            # Filter out NaNs and Infs (Fisheye polynomials can break here)
            valid_mask = ~np.isnan(undist[:, 0, 0]) & ~np.isinf(undist[:, 0, 0])
            undist = undist[valid_mask]

            if len(undist) > 0:
                x = undist[:, 0, 0]
                y = undist[:, 0, 1]

                r = np.sqrt(x**2 + y**2)
                
                # Filter out astronomically large coordinates from invalid extrapolation
                # r = 20 roughly corresponds to an 87.1 degree half-FOV. 
                valid_r_mask = r < 20.0
                r = r[valid_r_mask]
                
                if len(r) > 0:
                    thetas = np.arctan(r)  # Z is normalized to 1 by undistortPoints

                    # Take the max angle, add a 5% safety buffer
                    # Cap at 89 degrees to prevent infinity projection math explosions
                    self.max_half_fov = min(np.max(thetas) * 1.05, np.radians(89.0))
                else:
                    self.max_half_fov = np.radians(75.0)  # Fallback if remaining r-values were all invalid
            else:
                self.max_half_fov = np.radians(75.0)  # Fallback

        except Exception as e:
            self.max_half_fov = None

    @staticmethod
    def get_focal_length_pixels(camera_matrix):
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        focal_length_pixels = (fx + fy) / 2.0
        return focal_length_pixels

    @staticmethod
    def get_focal_length_mm(camera_matrix:np.ndarray, pixel_size_mm:float):
        return RGBCameraCalibration.get_focal_length_pixels(camera_matrix) * pixel_size_mm

    @staticmethod
    def load_calibration_from_fleet_path(
        camera_type: "RGBCameras", is_flip_width_and_height: bool
    ) -> "RGBCameraCalibration":
        """Loads a camera's calibration from ~/stretch_user/stretch-se4-xxxx/calibration_cameras/calibration_rgb_head_camera.yaml"""
        stretch_user_calibration_file_path = (
            RGBCameraCalibrationFile.USER.get_camera_calibration_file_path()
        )
        with open(stretch_user_calibration_file_path, "r") as f:
            calibration = yaml.load(f, Loader=yaml.FullLoader)
            if not camera_type.name in calibration:
                raise ValueError(
                    f"{camera_type.name} was not found in {stretch_user_calibration_file_path}. Please run `REx_camera_calibrate --intrinsics` first."
                )

            calibration = calibration[camera_type.name]

            # pprint(calibration)

        im_h, im_w, _ = calibration["image_size"]
        
        req_h, req_w = camera_type.config.image_size
        camera_matrix = np.array(calibration["camera_matrix"])
        
        if req_w != im_w or req_h != im_h:
            logging.warning(f"Requested image size ({req_w}, {req_h}) for {camera_type.name} differs from calibration image size ({im_w}, {im_h}). Adjusting camera_matrix to account for cropping.")
            crop_x = (im_w - req_w) / 2.0
            crop_y = (im_h - req_h) / 2.0
            
            camera_matrix[0, 2] -= crop_x
            camera_matrix[1, 2] -= crop_y
            
            im_w = req_w
            im_h = req_h

        width = im_w if not is_flip_width_and_height else im_h
        height = im_h if not is_flip_width_and_height else im_w

        distortion_coefficients = np.array(calibration["distortion_coefficients"])
        if "is_fisheye" in calibration:
            raise RuntimeError(f"""
You are using an old calibration file. Please modify {stretch_user_calibration_file_path} with the following:
is_fisheye: True -> distortion_model: equidistant_with_recompute_extrinsics
is_fisheye: False -> distortion_model: wide_angle
""")
        distortion_model = DistortionModels[calibration["distortion_model"]]

        return RGBCameraCalibration(
            width, height, camera_matrix, distortion_coefficients, distortion_model=distortion_model
        )


@dataclass
class CalibrateCameraResults:
    """Stores the results of a camera calibration."""

    camera_name: str
    calibration_date: datetime.datetime
    image_size: list[int]  # e.g., [1200, 1920]
    number_of_images_processed: int
    number_of_images_used: int
    reprojection_error: float
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    projection_matrix: np.ndarray
    distortion_model: DistortionModels
    rectification_matrix: np.ndarray
    rotation_vectors: list[np.ndarray] | tuple
    translation_vectors: list[np.ndarray] | tuple
    focal_length_mm:float|None = None

    def __repr__(self) -> str:
        focal_length = "" if not self.focal_length_mm else f"Focal Length: {self.focal_length_mm:.2f}mm"
        return f"""
==========================
Camera: {self.camera_name}. {focal_length}. Distortion Model: {self.distortion_model}.
Number of images: {self.number_of_images_used} / {self.number_of_images_processed} used.
Reprojection Error: {self.reprojection_error:.6f}.
"""

    @staticmethod
    def _serialize(dictionary: dict):
        def parse(value):
            if isinstance(value, DistortionModels):
                return value.get_model_name()
            if "tolist" in dir(value):
                return value.tolist()
            return value

        return {
            (k): (parse(v))
            for k, v in dictionary.items()
            if k
            not in (
                "rotation_vectors",
                "translation_vectors",
                "per_image_errors",
                "board_colors",
            )
        }

    def get_serializable(self):
        return CalibrateCameraResults._serialize(asdict(self))


class CameraPoseVisualizer:
    def __init__(self, camera_name: str, not_interactive: bool = False):
        self.camera_name = camera_name
        self.is_initialized = False
        self.not_interactive = not_interactive

    def init_rerun(self):
        if self.not_interactive:
            return
        if not self.is_initialized:
            try:
                rr.init(application_id=f"Camera Calibration", spawn=False)
                rr.spawn(memory_limit="4GB")
                self.is_initialized = True
            except Exception as e:
                print(f"Failed to initialize rerun: {e}")

    def update_pointcloud(
        self,
        calibration: "CalibrateCameraResults",
        all_object_points: list[np.ndarray],
        timestamps: list[float],
        per_image_errors: list[float] | None = None,
        board_colors: list[tuple[int, int, int]] | None = None,
    ):
        if not self.is_initialized:
            self.init_rerun()
            return

        try:
            # Setup camera
            width, height = calibration.image_size[1], calibration.image_size[0]
            rr.log(
                f"world/{self.camera_name}/image",
                rr.Pinhole(
                    resolution=[width, height],
                    image_from_camera=calibration.camera_matrix,
                ),
                static=True
            )

            if (
                calibration.rotation_vectors is None
                or calibration.translation_vectors is None
            ):
                return

            # board_colors = calibration.board_colors
            # Plot checkerboards and rays
            for i, (rvec, tvec, obj_pts) in enumerate(
                zip(
                    calibration.rotation_vectors,
                    calibration.translation_vectors,
                    all_object_points,
                )
            ):
                obj_pts = obj_pts.reshape(-1, 3)  # ensure (N, 3)
                # board_color = board_colors[i] if board_colors and i < len(board_colors) else [0, 255, 0]
                # image_error = calibration.per_image_errors[i] if calibration.per_image_errors and i < len(calibration.per_image_errors) else 0.0

                R, _ = cv2.Rodrigues(rvec)

                # Setup board transform
                rr.log(
                    f"world/{self.camera_name}/board_{i}",
                    rr.Transform3D(translation=tvec.flatten(), mat3x3=R),
                    static=True
                )

                # Log board error as a scalar metric
                # rr.log(
                #     f"world/{self.camera_name}/board_{i}/error",
                #     rr.Scalars(image_error)
                # )

                # Plot board points with error details in hover label
                rr.log(
                    f"world/{self.camera_name}/board_{i}/points",
                    rr.Points3D(
                        obj_pts,
                        colors=board_colors,
                        radii=0.005,
                        labels=(
                            [f"Error: {per_image_errors[i]:.4f}"]
                            if per_image_errors is not None
                            else [f"Image timestamp: {timestamps[i]}"]
                        )
                        * len(obj_pts),
                        show_labels=False,
                    ),
                    static=True
                )

                # # Plot rays from camera origin (in camera frame)
                # corners_cam = (R @ obj_pts.T).T + tvec.flatten()

                # # Draw rays: from origin to each corner
                # rays = np.zeros((len(corners_cam), 2, 3))
                # rays[:, 1, :] = corners_cam

                # rr.log(
                #     f"world/{self.camera_name}/rays_{i}",
                #     rr.LineStrips3D(rays, colors=[[0, 0, 255, 50]] * len(rays))
                # )

        except Exception as e:
            print(f"Error in rerun visualization: {e}")

    def show_detected_corners(self, image: np.ndarray):
        if not self.is_initialized:
            self.init_rerun()
            return
        try:
            image_array = np.asarray(image)
            if (
                image_array is not None
                and len(image_array.shape) >= 2
                and image_array.size > 0
            ):
                rr.log(
                    f"Detected\\ Charuco\\ Corners\\ {self.camera_name}",
                    rr.Image(image_array, color_model="BGR").compress(),
                )
        except Exception as e:
            print(f"Error in rerun visualization: {e}")

    def show_dropped_frame(self):
        if not self.is_initialized:
            self.init_rerun()
            return
        try:
            image = np.zeros((480, 640, 3), dtype=np.uint8)
            text = "Frame dropped"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            thickness = 2
            text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
            
            text_x = (image.shape[1] - text_size[0]) // 2
            text_y = (image.shape[0] + text_size[1]) // 2
            
            cv2.putText(image, text, (text_x, text_y), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
            rr.log(
                f"Detected\\ Charuco\\ Corners\\ {self.camera_name}",
                rr.Image(image, color_model="BGR").compress(),
            )
        except Exception as e:
            print(f"Error in rerun visualization: {e}")

    def show_last_saved(self, image: np.ndarray):
        if not self.is_initialized:
            self.init_rerun()
            return
        try:
            image_array = np.asarray(image)
            if (
                image_array is not None
                and len(image_array.shape) >= 2
                and image_array.size > 0
            ):
                rr.log(
                    f"Last\\ Saved\\ {self.camera_name}",
                    rr.Image(image_array, color_model="BGR").compress(),
                )
        except Exception as e:
            print(f"Error in rerun visualization: {e}")

    def log_instructions(self, text: str):
        if not self.is_initialized:
            self.init_rerun()
            return
        try:
            rr.log(
                f"Instructions\\ {self.camera_name}",
                rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN),
            )
        except Exception as e:
            print(f"Error in rerun visualization: {e}")

    def log_message(self, text: str, level: str = "INFO"):
        if not self.is_initialized:
            self.init_rerun()
            return
        try:
            rr.log(f"Console\\ {self.camera_name}", rr.TextLog(text, level=level))
        except Exception as e:
            print(f"Error in rerun visualization: {e}")

    def log_capture_status(self, is_auto: bool, is_capture_requested: bool):
        if not self.is_initialized:
            self.init_rerun()
            return
        status = (
            "🟢 AUTO-CAPTURE ENABLED"
            if is_auto
            else "🔴 WAITING FOR CAPTURE SIGNAL" if not is_capture_requested else "🟢 CAPTURE REQUESTED"
        )
        text = f"""
### Capture Mode
*{status}*
"""
        try:
            rr.log(
                f"Status\\ {self.camera_name}",
                rr.TextDocument(text, media_type=rr.MediaType.MARKDOWN),
            )
        except Exception as e:
            print(f"Error in rerun visualization: {e}")

    @staticmethod
    def setup_blueprint(camera_names: list[str]):
        last_saved_views = [
            rrb.Spatial2DView(name=f"Last Saved: {cam}", origin=f"Last\\ Saved\\ {cam}") 
            for cam in camera_names
        ]
        
        detected_corners_views = [
            rrb.Spatial2DView(name=f"Detected Corners: {cam}", origin=f"Detected\\ Charuco\\ Corners\\ {cam}")
            for cam in camera_names
        ]
        
        instructions_views = [
            rrb.TextDocumentView(name=f"Instructions: {cam}", origin=f"Instructions\\ {cam}")
            for cam in camera_names
        ]
        status_views = [
            rrb.TextDocumentView(name=f"Status: {cam}", origin=f"Status\\ {cam}")
            for cam in camera_names
        ]
        console_views = [
            rrb.TextLogView(name=f"Log: {cam}", origin=f"Console\\ {cam}")
            for cam in camera_names
        ]
        
        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Vertical(
                    rrb.Horizontal(*last_saved_views),
                    rrb.Horizontal(*detected_corners_views),
                ),
                rrb.Vertical(
                    rrb.Tabs(*instructions_views, active_tab=instructions_views[0].name if instructions_views else None),
                    rrb.Vertical(*status_views),
                    rrb.Vertical(*console_views)
                ),
                column_shares=[2, 1]
            ),
            rrb.BlueprintPanel(state="collapsed"),
            rrb.SelectionPanel(state="collapsed"),
            collapse_panels=True,
        )
        try:
            rr.send_blueprint(blueprint)
        except Exception as e:
            print(f"Failed to send blueprint: {e}")


def calculate_per_view_reprojection_error(
    all_object_points, all_image_points, calibration: CalibrateCameraResults
):
    # Calculate per-view reprojection errors and map to colors (Green -> Red)
    board_colors = []
    per_image_errors = []
    if (
        calibration.rotation_vectors is not None
        and calibration.translation_vectors is not None
    ):
        for i in range(len(all_object_points)):

            imgpoints2, _ = project_points(
                object_points=all_object_points[i].reshape(1, -1, 3),
                rvec=calibration.rotation_vectors[i],
                tvec=calibration.translation_vectors[i],
                camera_matrix=calibration.camera_matrix,
                distortion_coefficients=calibration.distortion_coefficients,
                distortion_model=calibration.distortion_model
            )

            pts1 = np.asarray(all_image_points[i]).reshape(-1, 2)
            pts2 = np.asarray(imgpoints2).reshape(-1, 2)
            error = float(np.linalg.norm(pts1 - pts2)) / len(pts1)
            per_image_errors.append(error)

            # Map error to color gradient:
            max_thresh = 100
            ratio = min(error / max_thresh, 1.0)
            red = int(255 * ratio)
            green = int(255 * (1 - ratio))
            board_colors.append([red, green, 0])

    return per_image_errors, board_colors
