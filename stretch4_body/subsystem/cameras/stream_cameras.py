from collections.abc import Generator

from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import RGBPipelineController, RGBPipelineControllerROS

from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame, SyncedImageFrame
from stretch4_body.subsystem.cameras.detectors.detector_ai_models import AIModelWrapper

def _start_camera(camera_type:RGBCameras, is_rotate:bool, is_rectify:bool, is_crop: bool, ai_models_to_use: list[AIModelWrapper]|None, detect_aruco_marker_size: float|None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[ImageFrame, None, None]:
    cls = RGBPipelineControllerROS if use_ros_for_cameras else RGBPipelineController
    rgb_pipeline_controller = cls(
        camera_type=camera_type,
        recording_directory=None,
        show_image_in=None,
        is_rotate=is_rotate,
        is_rectify=is_rectify,
        is_crop=is_crop,
        ai_models_to_use=ai_models_to_use or [],
        detect_aruco_marker_size=detect_aruco_marker_size
    )

    return rgb_pipeline_controller.get_frame(is_run_pipeline=is_run_pipeline)

def _start_synced_camera(camera_type:RGBCameras, is_rotate:bool, is_rectify:bool, is_crop: bool, ai_models_to_use: list[AIModelWrapper]|None, detect_aruco_marker_size: float|None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[SyncedImageFrame, None, None]:
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
    )

    return rgb_pipeline_controller.get_frame_synced(is_run_pipeline=is_run_pipeline)


def stream_left_camera(*, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[ImageFrame, None, None]:
    """Stream the left head camera"""
    return _start_camera(camera_type=RGBCameras.head_left, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline)

def stream_right_camera(*, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[ImageFrame, None, None]:
    """Stream the right head camera"""
    return _start_camera(camera_type=RGBCameras.head_right, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline)

def stream_center_camera(*, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[ImageFrame, None, None]:
    """Stream the right head camera"""
    return _start_camera(camera_type=RGBCameras.head_center, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline)

def stream_left_right_camera(*, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[SyncedImageFrame, None, None]:
    """Stream the left and right head cameras"""
    return _start_synced_camera(camera_type=RGBCameras.head_left_right, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline)

def stream_left_right_center_camera(*, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[SyncedImageFrame, None, None]:
    """Stream the center, left and right head cameras"""
    return _start_synced_camera(camera_type=RGBCameras.head_left_right_center, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline)

def stream_gripper_camera(*, is_rotate:bool=True, is_rectify:bool=False, is_crop: bool=False, ai_models_to_use: list[AIModelWrapper]|None=None, detect_aruco_marker_size: float|None=None, use_ros_for_cameras:bool=False, is_run_pipeline:bool=True) -> Generator[SyncedImageFrame, None, None]:
    """Stream the gripper RGBD camera"""
    return _start_synced_camera(camera_type=RGBCameras.gripper_rgbd, is_rotate=is_rotate, is_rectify=is_rectify, is_crop=is_crop, ai_models_to_use=ai_models_to_use, detect_aruco_marker_size=detect_aruco_marker_size, use_ros_for_cameras=use_ros_for_cameras, is_run_pipeline=is_run_pipeline)


if __name__ == "__main__":

    print("Stream only the left camera.")
    for image_frame in stream_left_camera():
        if image_frame is None: 
            print("No frame returned")
            continue
        print(f"Got image: {image_frame.image.shape=}, {image_frame.timestamp=}")
        break


    print("Stream both the left and right cameras.")
    for image_frame in stream_left_right_camera():
        if image_frame is None: 
            print("No frame returned")
            continue
        print(f"Got left image: {image_frame.left.image.shape=}, {image_frame.left.timestamp=}")
        print(f"Got right image: {image_frame.right.image.shape=}, {image_frame.right.timestamp=}")
        break


    print("Stream from the gripper RGBD camera.")
    for image_frame in stream_gripper_camera():
        if image_frame is None: 
            print("No frame returned")
            continue
        print(f"Got left image: {image_frame.left.image.shape=}, {image_frame.left.timestamp=}")
        if image_frame.pointcloud is not None:
            print(f"Got pointcloud image: {image_frame.pointcloud.shape=}")
        break