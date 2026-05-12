
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
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras


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
):
    base_filename = directory + "{:f}".format(rgb_timestamp)
    rgb_filename = base_filename + ".png"
    # rgb_filename = base_filename + ".jpg"
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


def saver_thread(
    stop_event: threading.Event, save_rgb_queue: queue.Queue[RgbImageToWriteToDisk]
):

    while not stop_event.is_set() or not save_rgb_queue.empty():
        if stop_event.is_set():
            print(
                f"record_rgb: Stop event has been set, waiting to finish writing data. {save_rgb_queue.qsize()} left."
            )
        try:
            rgb_image_to_write = save_rgb_queue.get(timeout=1 / 30)

            # 0 is no compression, 9 is maximum compression, [] is default
            # compression_level =  [cv2.IMWRITE_PNG_COMPRESSION, 9]
            compression_level = []
            cv2.imwrite(
                rgb_image_to_write.rgb_filename,
                rgb_image_to_write.color_image,
                compression_level,
            )

            if rgb_image_to_write.frame_number % 10 == 0:
                print(
                    f"Camera {rgb_image_to_write.camera_type.name} capture: {rgb_image_to_write.frame_number} {save_rgb_queue.qsize()=}"
                )
        except queue.Empty:
            ...


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
