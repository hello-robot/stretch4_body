
import logging

logger = logging.getLogger(__name__)
from dataclasses import dataclass
import glob
import os
import platform
import queue
import threading
import cv2
import numpy as np
import yaml
from stretch4_body.core.hello_utils import create_time_string
from stretch4_body.subsystem.cameras.enums.recording_file_format import RecordingFileFormat
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

VIDEO_FILENAME = "video"


@dataclass
class RgbImageToWriteToDisk:
    """A helper dataclass to help store captured frames in a queue to be written to disk."""

    rgb_filename: str
    color_image: np.ndarray
    camera_type: RGBCameras
    frame_number: int


def add_image_to_save_queue(
    color_image: np.ndarray,
    rgb_timestamp: float,
    directory: str,
    camera_type: RGBCameras,
    frame_number: int,
    save_rgb_queue: queue.Queue[RgbImageToWriteToDisk],
    file_format: RecordingFileFormat = RecordingFileFormat.png,
):
    if file_format.is_video():
        # Every frame of this camera goes into the same file, so it cannot be named after a timestamp.
        rgb_filename = directory + VIDEO_FILENAME + file_format.extension
    else:
        rgb_filename = directory + "{:f}".format(rgb_timestamp) + file_format.extension

    save_rgb_queue.put(
        RgbImageToWriteToDisk(
            rgb_filename=rgb_filename,
            color_image=color_image,
            camera_type=camera_type,
            frame_number=frame_number,
        )
    )


def get_last_file_or_folder_in_directory(path_with_regex:str):
    directory = glob.glob(path_with_regex)
    if len(directory) < 1:
        return None
    directory.sort()
    directory = directory[-1]
    return directory

def get_recording_subdirectory(recording_directory, data_type, timestamp:str|None = None):
    if timestamp is not None:
        return f"{recording_directory}/{data_type}/{timestamp}/"

    return get_last_file_or_folder_in_directory(recording_directory + '/' + data_type + '/*[0-9]/')


class VideoFileWriter:
    """Writes frames into one video file.

    The file is opened on the first frame, because OpenCV needs the frame size up front and only the
    captured imagery knows it - rotating and cropping in the pipeline change it.
    """

    def __init__(self, filename: str, fps: float):
        self.filename = filename
        self.fps = fps
        self._writer: cv2.VideoWriter | None = None
        self._frame_size: tuple[int, int] | None = None

    def write(self, color_image: np.ndarray):
        # Rotating in the pipeline returns a view with negative strides, which OpenCV cannot write.
        color_image = np.ascontiguousarray(color_image)

        if color_image.ndim == 2:
            color_image = cv2.cvtColor(color_image, cv2.COLOR_GRAY2BGR)

        height, width = color_image.shape[:2]

        if self._writer is None:
            self._frame_size = (width, height)
            self._writer = cv2.VideoWriter(
                self.filename,
                cv2.VideoWriter_fourcc(*"mp4v"),
                self.fps,
                self._frame_size,
            )
            if not self._writer.isOpened():
                raise RuntimeError(f"OpenCV could not open {self.filename} to write video to.")

            logger.info(f"Writing video to {self.filename} at {self.fps} fps, {width}x{height}.")

        if (width, height) != self._frame_size:
            logger.warning(
                f"Skipping a {width}x{height} frame, {self.filename} is being written at {self._frame_size[0]}x{self._frame_size[1]}."
            )
            return

        self._writer.write(color_image)

    def release(self):
        """Closes the file. Without this the video has no index and cannot be played back."""
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            logger.info(f"Finished writing {self.filename}.")


def get_imwrite_parameters(file_format: RecordingFileFormat) -> list[int]:
    if file_format is RecordingFileFormat.jpg:
        # 0 is the worst quality, 100 is the best, 95 is the default.
        return [cv2.IMWRITE_JPEG_QUALITY, 95]

    # 0 is no compression, 9 is maximum compression, [] is default
    # return [cv2.IMWRITE_PNG_COMPRESSION, 9]
    return []


def saver_thread(
    stop_event: threading.Event,
    save_rgb_queue: queue.Queue[RgbImageToWriteToDisk],
    file_format: RecordingFileFormat = RecordingFileFormat.png,
    video_fps: float = 30.0,
    abandon_event: threading.Event | None = None,
    finished_event: threading.Event | None = None,
):
    """Writes captured frames to disk until `stop_event` is set and the queue has been drained.

    Setting `abandon_event` drops whatever is still queued and closes the files right away, for a user
    who would rather quit than wait for a long recording to be written out.

    `finished_event` is set once every file has been closed, so that whoever is quitting knows that
    the recording is complete and it is safe to exit.
    """

    imwrite_parameters = get_imwrite_parameters(file_format)
    video_writers: dict[str, VideoFileWriter] = {}

    def is_abandoned():
        return abandon_event is not None and abandon_event.is_set()

    try:
        while (not stop_event.is_set() or not save_rgb_queue.empty()) and not is_abandoned():
            try:
                rgb_image_to_write = save_rgb_queue.get(timeout=1 / 30)

                if file_format.is_video():
                    filename = rgb_image_to_write.rgb_filename
                    if filename not in video_writers:
                        video_writers[filename] = VideoFileWriter(filename, video_fps)
                    video_writers[filename].write(rgb_image_to_write.color_image)
                else:
                    cv2.imwrite(
                        rgb_image_to_write.rgb_filename,
                        rgb_image_to_write.color_image,
                        imwrite_parameters,
                    )

                logger.debug(f"Camera {rgb_image_to_write.camera_type.name} capture: {rgb_image_to_write.frame_number} {save_rgb_queue.qsize()=}")
            except queue.Empty:
                ...
    finally:
        for video_writer in video_writers.values():
            video_writer.release()

        if finished_event is not None:
            finished_event.set()


def get_camera_recording_directory(
    recording_directory: str, camera_type: RGBCameras, time_string: str | None = None
):

    time_string = time_string or create_time_string()

    directory = (
        recording_directory
        + "/"
        + camera_type.recording_folder_name
        + "/"
        + time_string
        + "/"
    )

    return directory


def create_directory_if_it_does_not_exist(
    recording_directory: str, camera_type: RGBCameras, time_string: str | None = None
):

    time_string = time_string or create_time_string()

    directory = get_camera_recording_directory(
        recording_directory, camera_type, time_string
    )

    if not os.path.exists(directory):
        os.makedirs(directory)

        info = {}
        info["robot"] = platform.node()

        with open(os.path.join(directory, "info.yaml"), "w") as f:
            yaml.dump(info, f)

    return directory, time_string
