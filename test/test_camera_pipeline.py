import pytest
import numpy as np
import cv2
import time
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import RGBPipelineController
from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

def test_run_pipeline_rotation():
    controller = RGBPipelineController(
        camera_type=RGBCameras.head_left,
        recording_directory=None,
        show_image_in=None,
        is_rotate=True,
        is_rectify=False,
        is_crop=False,
        ai_models_to_use=[],
        detect_aruco_marker_size=None,
        is_open_camera=False
    )
    
    # 100x200 image
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    image[0:50, 0:100, :] = 255
    
    frame = ImageFrame(
        timestamp=time.time(),
        frame_number=0,
        image=image
    )
    
    controller.run_pipeline(frame)
    
    # Left camera config rotation number of times is 1 (so rot90 once)
    assert frame.image.shape == (200, 100, 3)
    assert frame.image_raw.shape == (100, 200, 3)

def test_run_pipeline_aruco_detection(monkeypatch):
    controller = RGBPipelineController(
        camera_type=RGBCameras.head_left,
        recording_directory=None,
        show_image_in=None,
        is_rotate=False,
        is_rectify=False,
        is_crop=False,
        ai_models_to_use=[],
        detect_aruco_marker_size=0.1,
        is_open_camera=False
    )
    
    # Mock do_aruco_detection to modify the image so we don't depend on actual ArUco markers in the test image
    called = []
    def mock_do_aruco_detection(color_image, camera_calibration, marker_length, dictionaries_to_detect):
        called.append(True)
        # return a modified copy of the image
        res = color_image.copy()
        res[0, 0, :] = [1, 2, 3]
        return res
        
    monkeypatch.setattr(
        "stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller.do_aruco_detection",
        mock_do_aruco_detection
    )
    
    image = np.zeros((100, 100, 3), dtype=np.uint8)
    frame = ImageFrame(
        timestamp=time.time(),
        frame_number=0,
        image=image
    )
    
    controller.run_pipeline(frame)
    
    assert called == [True]
    assert frame.image[0, 0, 0] == 1
    assert frame.image[0, 0, 1] == 2
    assert frame.image[0, 0, 2] == 3
    assert frame.image_raw[0, 0, 0] == 0

def test_run_pipeline_sequential_multiple(monkeypatch):
    # Test that rotate and then aruco are applied sequentially
    controller = RGBPipelineController(
        camera_type=RGBCameras.head_left,
        recording_directory=None,
        show_image_in=None,
        is_rotate=True,
        is_rectify=False,
        is_crop=False,
        ai_models_to_use=[],
        detect_aruco_marker_size=0.1,
        is_open_camera=False
    )
    
    def mock_do_aruco_detection(color_image, camera_calibration, marker_length, dictionaries_to_detect):
        # We expect the image to be rotated already
        assert color_image.shape == (200, 100, 3)
        res = color_image.copy()
        res[0, 0, :] = [1, 2, 3]
        return res
        
    monkeypatch.setattr(
        "stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller.do_aruco_detection",
        mock_do_aruco_detection
    )
    
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    frame = ImageFrame(
        timestamp=time.time(),
        frame_number=0,
        image=image
    )
    
    controller.run_pipeline(frame)
    
    assert frame.image.shape == (200, 100, 3)
    assert frame.image[0, 0, 0] == 1
def test_show_rgb_cli_defaults(monkeypatch):
    from stretch4_body.subsystem.cameras.show_rgb import show_rgb
    from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import RecordRgbShowImageIn
    import argparse

    # We mock parse_args to return custom values
    class MockArgs:
        def __init__(self, left=False, right=False, center=False, left_right=False, left_right_center=False, gripper=False, camera_name=None, rerun=False, opencv=False, no_rotate=False, recording_directory=None, show_fps=False, use_ros_for_cameras=False, rectify=False, crop=False, detect_aruco_marker_size=None):
            self.left = left
            self.right = right
            self.center = center
            self.left_right = left_right
            self.left_right_center = left_right_center
            self.gripper = gripper
            self.camera_name = camera_name
            self.rerun = rerun
            self.opencv = opencv
            self.no_rotate = no_rotate
            self.recording_directory = recording_directory
            self.show_fps = show_fps
            self.use_ros_for_cameras = use_ros_for_cameras
            self.rectify = rectify
            self.crop = crop
            self.detect_aruco_marker_size = detect_aruco_marker_size

    instantiated_args = []

    class MockController:
        def __init__(self, **kwargs):
            instantiated_args.append(kwargs)
        def get_frame(self, is_run_pipeline=True):
            return []
        def get_frame_synced(self, is_run_pipeline=True):
            return []

    monkeypatch.setattr(
        "stretch4_body.subsystem.cameras.show_rgb.RGBPipelineController",
        MockController
    )

    # 1. Test defaults (neither rerun nor opencv, no no-rotate)
    monkeypatch.setattr(
        argparse.ArgumentParser,
        "parse_args",
        lambda self: MockArgs(left=True, rerun=False, opencv=False, no_rotate=False)
    )
    show_rgb()
    assert len(instantiated_args) == 1
    assert instantiated_args[0]["show_image_in"] == RecordRgbShowImageIn.RERUN
    assert instantiated_args[0]["is_rotate"] is True
    assert instantiated_args[0]["camera_type"] == RGBCameras.left()

    # 2. Test opencv
    instantiated_args.clear()
    monkeypatch.setattr(
        argparse.ArgumentParser,
        "parse_args",
        lambda self: MockArgs(left=True, rerun=False, opencv=True, no_rotate=False)
    )
    show_rgb()
    assert len(instantiated_args) == 1
    assert instantiated_args[0]["show_image_in"] == RecordRgbShowImageIn.CVIMSHOW
    assert instantiated_args[0]["is_rotate"] is True
    assert instantiated_args[0]["camera_type"] == RGBCameras.left()

    # 3. Test no-rotate
    instantiated_args.clear()
    monkeypatch.setattr(
        argparse.ArgumentParser,
        "parse_args",
        lambda self: MockArgs(left=True, rerun=False, opencv=False, no_rotate=True)
    )
    show_rgb()
    assert len(instantiated_args) == 1
    assert instantiated_args[0]["show_image_in"] == RecordRgbShowImageIn.RERUN
    assert instantiated_args[0]["is_rotate"] is False
    assert instantiated_args[0]["camera_type"] == RGBCameras.left()

    # 4. Test default camera type (no camera argument supplied)
    instantiated_args.clear()
    monkeypatch.setattr(
        argparse.ArgumentParser,
        "parse_args",
        lambda self: MockArgs()
    )
    show_rgb()
    assert len(instantiated_args) == 1
    assert instantiated_args[0]["camera_type"] == RGBCameras.synced_left_right_center()


if __name__ == '__main__':
    pytest.main([__file__])
