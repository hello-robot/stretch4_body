import logging

logger = logging.getLogger(__name__)
from collections.abc import Generator

from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import RGBPipelineController, RGBPipelineControllerROS

from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame, SyncedImageFrame
from stretch4_body.subsystem.cameras.models.rgb_camera_config import RGBCameraConfig
from stretch4_body.subsystem.cameras.detectors.detector_ai_models import AIModelWrapper

def _start_camera(camera_type:RGBCameras, is_rotate:bool, is_rectify:bool, is_crop: bool, ai_models_to_use: list[AIModelWrapper]|None, detect_aruco_marker_size: float|None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True, config:RGBCameraConfig|None=None) -> Generator[ImageFrame, None, None]:
    cls = RGBPipelineControllerROS if use_ros_for_cameras else RGBPipelineController
    rgb_pipeline_controller = cls(
        camera_type=camera_type,
        recording_directory=None,
        show_image_in=None,
        is_rotate=is_rotate,
        is_rectify=is_rectify,
        is_crop=is_crop,
        ai_models_to_use=ai_models_to_use or [],
        detect_aruco_marker_size=detect_aruco_marker_size,
        camera_configs={camera_type.base: config} if config is not None else None,
    )

    return rgb_pipeline_controller.get_frame(is_run_pipeline=is_run_pipeline)

def _start_synced_camera(camera_type:RGBCameras, is_rotate:bool, is_rectify:bool, is_crop: bool, ai_models_to_use: list[AIModelWrapper]|None, detect_aruco_marker_size: float|None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True, enable_pointcloud:bool=False, configs:dict[RGBCameras, RGBCameraConfig]|None=None) -> Generator[SyncedImageFrame, None, None]:
    cls = RGBPipelineControllerROS if use_ros_for_cameras else RGBPipelineController
    rgb_pipeline_controller = cls(
        camera_type=camera_type,
        recording_directory=None,
        show_image_in=None,
        is_rotate=is_rotate,
        is_rectify=is_rectify,
        is_crop=is_crop,
        ai_models_to_use=ai_models_to_use or [],
        detect_aruco_marker_size=detect_aruco_marker_size,
        enable_pointcloud=enable_pointcloud,
        camera_configs=configs,
    )

    return rgb_pipeline_controller.get_frame_synced(is_run_pipeline=is_run_pipeline)


def stream_left_camera(*, config:RGBCameraConfig|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[ImageFrame, None, None]:
    """Stream the left head camera"""
    return _start_camera(camera_type=RGBCameras.head_left, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, config=config)

def stream_right_camera(*, config:RGBCameraConfig|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[ImageFrame, None, None]:
    """Stream the right head camera"""
    return _start_camera(camera_type=RGBCameras.head_right, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, config=config)

def stream_center_camera(*, config:RGBCameraConfig|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[ImageFrame, None, None]:
    """Stream the right head camera"""
    return _start_camera(camera_type=RGBCameras.head_center, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, config=config)

def stream_left_right_camera(*, configs:dict[RGBCameras, RGBCameraConfig]|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[SyncedImageFrame, None, None]:
    """Stream the left and right head cameras"""
    return _start_synced_camera(camera_type=RGBCameras.head_left_right, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, configs=configs)

def stream_left_right_center_camera(*, configs:dict[RGBCameras, RGBCameraConfig]|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[SyncedImageFrame, None, None]:
    """Stream the center, left and right head cameras"""
    return _start_synced_camera(camera_type=RGBCameras.head_left_right_center, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, configs=configs)

def stream_gripper_camera(*, configs:dict[RGBCameras, RGBCameraConfig]|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True, enable_pointcloud:bool=False) -> Generator[SyncedImageFrame, None, None]:
    """Stream the gripper RGBD camera"""
    return _start_synced_camera(camera_type=RGBCameras.gripper_rgbd, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, configs=configs, enable_pointcloud=enable_pointcloud)


# The MJPEG variants below stream the same cameras, but the device encodes to JPEG on-chip. Frames
# arrive compressed (`ImageFrame.is_compressed()`), so they cost a fraction of the bandwidth of raw
# BGR to move between processes. `is_run_pipeline=True` decodes them for you; pass False to keep the
# JPEG bitstream (that is what the ROS 2 camera node does before republishing it).

def stream_left_camera_compressed(*, config:RGBCameraConfig|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[ImageFrame, None, None]:
    """Stream the left head camera, MJPEG encoded on-device"""
    return _start_camera(camera_type=RGBCameras.head_left_compressed, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, config=config)

def stream_right_camera_compressed(*, config:RGBCameraConfig|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[ImageFrame, None, None]:
    """Stream the right head camera, MJPEG encoded on-device"""
    return _start_camera(camera_type=RGBCameras.head_right_compressed, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, config=config)

def stream_center_camera_compressed(*, config:RGBCameraConfig|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[ImageFrame, None, None]:
    """Stream the center head camera, MJPEG encoded on-device"""
    return _start_camera(camera_type=RGBCameras.head_center_compressed, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, config=config)

def stream_left_right_camera_compressed(*, configs:dict[RGBCameras, RGBCameraConfig]|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[SyncedImageFrame, None, None]:
    """Stream the left and right head cameras, MJPEG encoded on-device"""
    return _start_synced_camera(camera_type=RGBCameras.head_left_right_compressed, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, configs=configs)

def stream_left_right_center_camera_compressed(*, configs:dict[RGBCameras, RGBCameraConfig]|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[SyncedImageFrame, None, None]:
    """Stream the center, left and right head cameras, MJPEG encoded on-device"""
    return _start_synced_camera(camera_type=RGBCameras.head_left_right_center_compressed, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, configs=configs)

def stream_gripper_camera_compressed(*, configs:dict[RGBCameras, RGBCameraConfig]|None=None, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True, enable_pointcloud:bool=False) -> Generator[SyncedImageFrame, None, None]:
    """Stream the gripper RGBD camera, MJPEG encoded on-device. The depth map is never compressed."""
    return _start_synced_camera(camera_type=RGBCameras.gripper_rgbd_compressed, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline, configs=configs, enable_pointcloud=enable_pointcloud)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    logger.info("Stream only the left camera.")
    for image_frame in stream_left_camera():
        if image_frame is None: 
            logger.info("No frame returned")
            continue
        logger.info(f"Got image: {image_frame.image.shape=}, {image_frame.timestamp=}")
        break


    logger.info("Stream both the left and right cameras.")
    for image_frame in stream_left_right_camera():
        if image_frame is None: 
            logger.info("No frame returned")
            continue
        logger.info(f"Got left image: {image_frame.left.image.shape=}, {image_frame.left.timestamp=}")
        logger.info(f"Got right image: {image_frame.right.image.shape=}, {image_frame.right.timestamp=}")
        break


    logger.info("Stream from the gripper RGBD camera.")
    for image_frame in stream_gripper_camera():
        if image_frame is None: 
            logger.info("No frame returned")
            continue
        logger.info(f"Got left image: {image_frame.left.image.shape=}, {image_frame.left.timestamp=}")
        if image_frame.pointcloud is not None:
            logger.info(f"Got pointcloud image: {image_frame.pointcloud.shape=}")
        break
