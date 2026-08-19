import pytest
import numpy as np
import cv2
import time
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import RGBPipelineController
from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.models.image_write_to_disk import DEFAULT_RECORDING_CHUNK_SECONDS

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
        def __init__(self, left=False, right=False, center=False, left_right=False, left_right_center=False, gripper=False, camera_name=None, rerun=False, opencv=False, no_rotate=False, recording_directory=None, record_format=".mp4", show_fps=False, use_ros_for_cameras=False, rectify=False, crop=False, detect_aruco_marker_size=None, recording_chunk_seconds=DEFAULT_RECORDING_CHUNK_SECONDS):
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
            self.record_format = record_format
            self.show_fps = show_fps
            self.use_ros_for_cameras = use_ros_for_cameras
            self.rectify = rectify
            self.crop = crop
            self.recording_chunk_seconds = recording_chunk_seconds
            self.detect_aruco_marker_size = detect_aruco_marker_size

    instantiated_args = []

    class MockController:
        def __init__(self, **kwargs):
            instantiated_args.append(kwargs)
        def get_frame(self, is_run_pipeline=True):
            return []
        def get_frame_synced(self, is_run_pipeline=True):
            return []
        def stop(self):
            ...

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


def test_show_rgb_cli_recording_options(monkeypatch, tmp_path):
    from stretch4_body.subsystem.cameras.show_rgb import show_rgb
    from stretch4_body.subsystem.cameras.enums.recording_file_format import RecordingFileFormat
    import argparse

    class MockArgs:
        def __init__(self, recording_directory=None, record_format=".mp4"):
            self.left = True
            self.right = False
            self.center = False
            self.left_right = False
            self.left_right_center = False
            self.gripper = False
            self.camera_name = None
            self.rerun = False
            self.opencv = False
            self.no_rotate = False
            self.show_fps = False
            self.use_ros_for_cameras = False
            self.rectify = False
            self.crop = False
            self.detect_aruco_marker_size = None
            self.recording_directory = recording_directory
            self.record_format = record_format
            self.recording_chunk_seconds = DEFAULT_RECORDING_CHUNK_SECONDS

    instantiated_args = []

    class MockController:
        def __init__(self, **kwargs):
            instantiated_args.append(kwargs)
        def get_frame(self, is_run_pipeline=True):
            return []
        def get_frame_synced(self, is_run_pipeline=True):
            return []
        def stop(self):
            ...

    monkeypatch.setattr(
        "stretch4_body.subsystem.cameras.show_rgb.RGBPipelineController",
        MockController
    )

    recording_directory = str(tmp_path)

    # The format is accepted with or without the leading dot, and defaults to mp4.
    for record_format, expected in [(".mp4", RecordingFileFormat.mp4), ("png", RecordingFileFormat.png), (".jpg", RecordingFileFormat.jpg)]:
        instantiated_args.clear()
        monkeypatch.setattr(
            argparse.ArgumentParser,
            "parse_args",
            lambda self, record_format=record_format: MockArgs(recording_directory=recording_directory, record_format=record_format)
        )
        show_rgb()
        assert len(instantiated_args) == 1
        assert instantiated_args[0]["recording_directory"] == recording_directory
        assert instantiated_args[0]["recording_file_format"] == expected

    # Without a recording directory nothing is written, whatever the format is.
    instantiated_args.clear()
    monkeypatch.setattr(argparse.ArgumentParser, "parse_args", lambda self: MockArgs())
    show_rgb()
    assert instantiated_args[0]["recording_directory"] is None
    assert instantiated_args[0]["recording_file_format"] == RecordingFileFormat.mp4

    # An unsupported format exits with argparse's usage error.
    monkeypatch.setattr(
        argparse.ArgumentParser,
        "parse_args",
        lambda self: MockArgs(recording_directory=recording_directory, record_format=".gif")
    )
    with pytest.raises(SystemExit):
        show_rgb()


def test_recording_file_formats_write_to_disk(tmp_path):
    import queue
    import threading
    from stretch4_body.subsystem.cameras.enums.recording_file_format import RecordingFileFormat
    from stretch4_body.subsystem.cameras.models.image_write_to_disk import add_image_to_save_queue, saver_thread

    directory = str(tmp_path) + "/"
    image = np.zeros((100, 200, 3), dtype=np.uint8)

    for file_format, expected_filename in [
        (RecordingFileFormat.png, "1.000000.png"),
        (RecordingFileFormat.jpg, "1.000000.jpg"),
        # A chunked recording numbers its files, the first chunk is 0000.
        (RecordingFileFormat.mp4, "video_0000.mp4"),
    ]:
        save_rgb_queue = queue.Queue()
        for frame_number in range(3):
            add_image_to_save_queue(
                color_image=image,
                rgb_timestamp=1.0 + frame_number,
                directory=directory,
                camera_type=RGBCameras.head_left,
                frame_number=frame_number,
                save_rgb_queue=save_rgb_queue,
                file_format=file_format,
            )

        stop_event = threading.Event()
        stop_event.set()  # The saver drains the queue before it stops.
        saver_thread(stop_event, save_rgb_queue, file_format, video_fps=30.0, chunk_seconds=300)

        written = tmp_path / expected_filename
        assert written.exists(), f"{file_format} did not write {written}"
        assert written.stat().st_size > 0

        # A video holds every frame in one file, the image formats write one file per frame.
        expected_number_of_files = 1 if file_format.is_video() else 3
        assert len(list(tmp_path.glob("*" + file_format.extension))) == expected_number_of_files


def test_stop_writes_out_the_whole_recording(tmp_path):
    """Stopping has to drain the queue and close the video, otherwise the file cannot be played back."""
    from stretch4_body.subsystem.cameras.enums.recording_file_format import RecordingFileFormat

    controller = RGBPipelineController(
        camera_type=RGBCameras.head_left,
        recording_directory=str(tmp_path),
        show_image_in=None,
        is_rotate=False,
        is_rectify=False,
        is_crop=False,
        ai_models_to_use=[],
        detect_aruco_marker_size=None,
        is_open_camera=False,
        recording_file_format=RecordingFileFormat.mp4,
    )
    controller.save_thread.start()

    number_of_frames = 20
    for frame_number in range(number_of_frames):
        controller.frame_number = frame_number
        controller.run_pipeline(ImageFrame(
            timestamp=time.time() + frame_number,
            frame_number=frame_number,
            image=np.full((60, 80, 3), frame_number * 5, dtype=np.uint8),
        ))

    controller.stop()

    videos = list(tmp_path.glob("*/*/video_0000.mp4"))
    assert len(videos) == 1

    capture = cv2.VideoCapture(str(videos[0]))
    frames_read = 0
    while capture.read()[0]:
        frames_read += 1
    capture.release()

    assert frames_read == number_of_frames


def _write_frames_to_video(directory, number_of_frames, fps, chunk_seconds, size=(240, 320),
                           feed_in_real_time=False):
    """Encodes moving frames through a VideoFileWriter and returns the chunks it wrote."""
    from stretch4_body.subsystem.cameras.models.image_write_to_disk import VideoFileWriter

    writer = VideoFileWriter(str(directory / "video.mp4"), fps, chunk_seconds=chunk_seconds)
    image = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    for frame_number in range(number_of_frames):
        # The frames have to differ, or the encoder compresses the whole run into almost nothing.
        image = np.roll(image, 7, axis=1)
        image[:, :20] = frame_number % 256
        writer.write(image)
        if feed_in_real_time:
            # Chunks rotate on elapsed real time, so a test of chunking has to spend it.
            time.sleep(1 / fps)
    writer.release()

    return sorted(directory.glob("video_*.mp4")), directory / "video.mp4"


def test_recording_is_split_into_chunks(tmp_path):
    """A long recording is written as chunks, so an interruption does not cost the whole file."""
    fps = 20
    chunk_seconds = 1
    expected_chunks = 3
    number_of_frames = fps * chunk_seconds * expected_chunks

    chunks, single_file = _write_frames_to_video(
        tmp_path, number_of_frames, fps, chunk_seconds, feed_in_real_time=True
    )

    assert not single_file.exists(), "the unchunked name should not be written when chunking"
    # Feeding frames costs a little more than the sleep, so the last chunk may only just have opened.
    assert expected_chunks <= len(chunks) <= expected_chunks + 1, [c.name for c in chunks]
    assert [c.name for c in chunks] == [f"video_{i:04d}.mp4" for i in range(len(chunks))]

    frames_read_in_total = 0
    for chunk in chunks:
        capture = cv2.VideoCapture(str(chunk))
        frames_read = 0
        while capture.read()[0]:
            frames_read += 1
        capture.release()
        # Every chunk has to be playable on its own, that is the point of chunking.
        assert frames_read > 0, f"{chunk.name} is empty"
        frames_read_in_total += frames_read

    assert frames_read_in_total == number_of_frames


def test_recording_chunks_rotate_on_real_time_not_recorded_time(tmp_path):
    """A camera running below its configured rate still gets chunks of the requested length.

    ffmpeg's own segmenting measures the recording's timeline, so a camera delivering a third of its
    configured rate produced chunks three times longer than asked for - or, over a short run, one
    unsplit file.
    """
    configured_fps = 30
    delivered_fps = 10
    chunk_seconds = 1
    seconds_to_record = 3
    number_of_frames = delivered_fps * seconds_to_record

    from stretch4_body.subsystem.cameras.models.image_write_to_disk import VideoFileWriter

    writer = VideoFileWriter(str(tmp_path / "video.mp4"), configured_fps, chunk_seconds=chunk_seconds)
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    for frame_number in range(number_of_frames):
        image = np.roll(image, 7, axis=1)
        image[:, :20] = frame_number % 256
        writer.write(image)
        time.sleep(1 / delivered_fps)
    writer.release()

    chunks = sorted(tmp_path.glob("video_*.mp4"))
    assert len(chunks) >= seconds_to_record, (
        f"{seconds_to_record}s at {chunk_seconds}s chunks should not fit in {len(chunks)} file(s)"
    )


def test_recording_chunk_seconds_of_zero_writes_a_single_file(tmp_path):
    number_of_frames = 20
    chunks, single_file = _write_frames_to_video(tmp_path, number_of_frames, 10, chunk_seconds=0)

    assert chunks == [], "chunking should be off"
    assert single_file.exists()

    capture = cv2.VideoCapture(str(single_file))
    frames_read = 0
    while capture.read()[0]:
        frames_read += 1
    capture.release()
    assert frames_read == number_of_frames


def test_recording_is_encoded_far_smaller_than_the_old_mpeg4_encoder(tmp_path):
    """The reason for moving off OpenCV's mpeg4: it wrote ~112 Mbps for the 12MP center camera."""
    fps = 10
    number_of_frames = 40
    height, width = 480, 640

    rng = np.random.default_rng(0)
    frames = []
    image = rng.integers(0, 255, (height, width, 3), dtype=np.uint8)
    for _ in range(number_of_frames):
        image = np.roll(image, 7, axis=1)
        frames.append(image.copy())

    from stretch4_body.subsystem.cameras.models.image_write_to_disk import VideoFileWriter

    writer = VideoFileWriter(str(tmp_path / "video.mp4"), fps, chunk_seconds=0)
    for frame in frames:
        writer.write(frame)
    writer.release()
    new_size = (tmp_path / "video.mp4").stat().st_size

    mpeg4 = cv2.VideoWriter(
        str(tmp_path / "mpeg4.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    assert mpeg4.isOpened()
    for frame in frames:
        mpeg4.write(frame)
    mpeg4.release()
    mpeg4_size = (tmp_path / "mpeg4.mp4").stat().st_size

    assert new_size < mpeg4_size, f"{new_size} bytes is not smaller than mpeg4's {mpeg4_size}"


def test_frames_below_the_hardware_encoders_minimum_still_record(tmp_path):
    """The GPU refuses frames under 128 pixels, which a heavily cropped image can hit."""
    from stretch4_body.subsystem.cameras.models.image_write_to_disk import (
        MINIMUM_HARDWARE_ENCODER_DIMENSION,
    )

    size = (MINIMUM_HARDWARE_ENCODER_DIMENSION // 2, MINIMUM_HARDWARE_ENCODER_DIMENSION // 2)
    number_of_frames = 20
    _, single_file = _write_frames_to_video(
        tmp_path, number_of_frames, 10, chunk_seconds=0, size=size
    )

    assert single_file.exists()
    capture = cv2.VideoCapture(str(single_file))
    frames_read = 0
    while capture.read()[0]:
        frames_read += 1
    capture.release()
    assert frames_read == number_of_frames


if __name__ == '__main__':
    pytest.main([__file__])
