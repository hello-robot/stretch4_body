import pytest
import time
from stretch4_body.subsystem.cameras.stream_cameras import stream_left_camera, stream_left_right_camera
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import RGBPipelineController
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

def test_stream_camera_left_for_loop():
    count = 0
    for frame in stream_left_camera(is_run_pipeline=False):
        if frame is not None:
            count += 1
        if count >= 5:
            break
    assert count == 5
    time.sleep(3) # Wait for the usb device to come back on the USB bus after disconnecting

def test_stream_camera_left_break_resume():
    # stream, break, stream again, break, stream again
    for _ in range(3):
        count = 0
        for frame in stream_left_camera(is_run_pipeline=False):
            if frame is not None:
                count += 1
            time.sleep(3) # Wait for the usb device to come back on the USB bus after disconnecting
            if count >= 3:
                break
        assert count == 3

def test_stream_camera_left_right_for_loop():
    count = 0
    for frame in stream_left_right_camera(is_run_pipeline=False):
        if frame is not None:
            assert frame.left is not None
            assert frame.right is not None
            count += 1
        time.sleep(3) # Wait for the usb device to come back on the USB bus after disconnecting
        if count >= 5:
            break
    assert count == 5

def test_stream_camera_left_right_break_resume():
    # stream, break, stream again, break, stream again
    for _ in range(3):
        count = 0
        for frame in stream_left_right_camera(is_run_pipeline=False):
            if frame is not None:
                assert frame.left is not None
                assert frame.right is not None
                count += 1
            time.sleep(3) # Wait for the usb device to come back on the USB bus after disconnecting
            if count >= 3:
                break
        assert count == 3

def test_rgb_pipeline_controller_get_frame_left_for_loop():
    controller = RGBPipelineController(
        camera_type=RGBCameras.head_left,
        recording_directory=None,
        show_image_in=None,
        is_rotate=False,
        is_rectify=False,
        is_crop=False,
        ai_models_to_use=[],
        detect_aruco_marker_size=None
    )
    count = 0
    try:
        for frame in controller.get_frame(is_run_pipeline=False):
            if frame is not None:
                count += 1
            if count >= 5:
                break
        assert count == 5
    finally:
        controller.stop()
    time.sleep(3) # Wait for the usb device to come back on the USB bus after disconnecting

def test_rgb_pipeline_controller_get_frame_left_break_resume():
    # stream, break, stream again, break, stream again
    for _ in range(3):
        controller = RGBPipelineController(
            camera_type=RGBCameras.head_left,
            recording_directory=None,
            show_image_in=None,
            is_rotate=False,
            is_rectify=False,
            is_crop=False,
            ai_models_to_use=[],
            detect_aruco_marker_size=None
        )
        try:
            for _ in range(3):
                count = 0
                for frame in controller.get_frame(is_run_pipeline=False):
                    if frame is not None:
                        count += 1
                    if count >= 3:
                        break
                assert count == 3
        finally:
            controller.stop()
        time.sleep(3) # Wait for the usb device to come back on the USB bus after disconnecting

def test_rgb_pipeline_controller_get_frame_synced_left_right_for_loop():
    controller = RGBPipelineController(
        camera_type=RGBCameras.head_left_right,
        recording_directory=None,
        show_image_in=None,
        is_rotate=False,
        is_rectify=False,
        is_crop=False,
        ai_models_to_use=[],
        detect_aruco_marker_size=None
    )
    count = 0
    try:
        for frame in controller.get_frame_synced(is_run_pipeline=False):
            if frame is not None:
                assert frame.left is not None
                assert frame.right is not None
                count += 1
            if count >= 5:
                break
        assert count == 5
    finally:
        controller.stop()
    time.sleep(3) # Wait for the usb device to come back on the USB bus after disconnecting

def test_rgb_pipeline_controller_get_frame_synced_left_right_break_resume():
    # stream, break, stream again, break, stream again
    for _ in range(3):
        controller = RGBPipelineController(
            camera_type=RGBCameras.head_left_right,
            recording_directory=None,
            show_image_in=None,
            is_rotate=False,
            is_rectify=False,
            is_crop=False,
            ai_models_to_use=[],
            detect_aruco_marker_size=None
        )
        try:
            for _ in range(3):
                count = 0
                for frame in controller.get_frame_synced(is_run_pipeline=False):
                    if frame is not None:
                        assert frame.left is not None
                        assert frame.right is not None
                        count += 1
                    if count >= 3:
                        break
                assert count == 3
        finally:
            controller.stop()
        time.sleep(3) # Wait for the usb device to come back on the USB bus after disconnecting

if __name__ == "__main__":
    pytest.main([__file__])