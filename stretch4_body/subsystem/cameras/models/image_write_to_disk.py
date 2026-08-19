
import logging

logger = logging.getLogger(__name__)
from dataclasses import dataclass
import functools
import glob
import os
import platform
import queue
import shutil
import subprocess
import threading
import time
import cv2
import numpy as np
import yaml
from stretch4_body.core.hello_utils import create_time_string
from stretch4_body.subsystem.cameras.enums.recording_file_format import RecordingFileFormat
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

VIDEO_FILENAME = "video"

# The render node of the integrated GPU, used to encode video in hardware.
VAAPI_RENDER_NODE = "/dev/dri/renderD128"

# Constant-quantizer level for the hardware HEVC encoder. 30 measures ~12x smaller than the mpeg4
# this used to write, at SSIM 0.95 against the same source. Lower is better quality and bigger.
HEVC_CONSTANT_QUANTIZER = 30

# Rate factor for the software H.264 fallback, chosen to land near the hardware encoder's output.
H264_CONSTANT_RATE_FACTOR = 28

# The GPU's HEVC encoder refuses frames smaller than this, so they are encoded in software instead.
# Full frames are far larger, but a heavily cropped one can land under it.
MINIMUM_HARDWARE_ENCODER_DIMENSION = 128

# How long each chunk of a recording is, when chunking is not configured explicitly.
DEFAULT_RECORDING_CHUNK_SECONDS = 300

# How often a keyframe is written, in seconds of recorded video. Players can only seek to a keyframe,
# and a damaged frame corrupts playback until the next one, so recordings are not written as one long
# run of predicted frames. Measured on 12MP center footage: 5s costs ~7% over an unbounded interval,
# 1s costs ~47%.
KEYFRAME_INTERVAL_SECONDS = 5

ENCODER_LOG_FILENAME = "video_encoder.log"


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


@functools.cache
def get_hardware_encoder_or_none() -> str | None:
    """Returns the hardware HEVC encoder to use, or None when the machine cannot encode in hardware.

    The probe actually encodes a frame, because a present render node does not by itself mean the
    driver will hand out an encoding context. Cached so that opening several cameras probes once.
    """
    if shutil.which("ffmpeg") is None:
        return None

    if not os.path.exists(VAAPI_RENDER_NODE):
        logger.info(f"{VAAPI_RENDER_NODE} is not present, video will be encoded in software.")
        return None

    probe = subprocess.run(
        [
            "ffmpeg", "-hide_banner", "-v", "error",
            "-init_hw_device", f"vaapi=hw:{VAAPI_RENDER_NODE}", "-filter_hw_device", "hw",
            "-f", "lavfi", "-i", "color=black:s=320x240:r=1", "-frames:v", "1",
            "-vf", "format=nv12,hwupload", "-c:v", "hevc_vaapi", "-f", "null", "-",
        ],
        capture_output=True,
        timeout=30,
    )

    if probe.returncode != 0:
        logger.info(
            f"The GPU at {VAAPI_RENDER_NODE} would not encode HEVC, video will be encoded in "
            f"software: {probe.stderr.decode(errors='replace').strip()}"
        )
        return None

    return "hevc_vaapi"


class VideoFileWriter:
    """Encodes frames into video, one file per chunk of `chunk_seconds`.

    Frames are piped to ffmpeg, which encodes HEVC on the GPU when the machine has one and falls back
    to H.264 on the CPU when it does not. The previous mpeg4 encoder wrote ~112 Mbps for the 12MP
    center camera, about 50 GB an hour; HEVC measures ~10 Mbps on the same footage.

    Chunking keeps an interrupted recording usable. A single file that never gets closed has no index
    and will not play back at all, so an hour-long recording used to be an hour at risk.

    The encoder is started on the first frame, because it needs the frame size up front and only the
    captured imagery knows it - rotating and cropping in the pipeline change it.
    """

    def __init__(
        self,
        filename: str,
        fps: float,
        chunk_seconds: float | None = DEFAULT_RECORDING_CHUNK_SECONDS,
    ):
        self.filename = filename
        self.fps = fps
        self.chunk_seconds = chunk_seconds
        self._process: subprocess.Popen | None = None
        self._encoder_log = None
        self._frame_size: tuple[int, int] | None = None
        self._fallback_writer: cv2.VideoWriter | None = None
        self._chunk_index = 0
        self._chunk_started_at = 0.0

        # Encoders draining their last frames after a chunk was rotated away. They are not waited on
        # at rotation time, so a rotation does not stall the frames still arriving behind it.
        self._finishing_processes: list[subprocess.Popen] = []

    @property
    def _is_chunked(self) -> bool:
        return self.chunk_seconds is not None and self.chunk_seconds > 0

    def _current_chunk_filename(self) -> str:
        if not self._is_chunked:
            return self.filename

        root, extension = os.path.splitext(self.filename)
        return f"{root}_{self._chunk_index:04d}{extension}"

    def _output_arguments(self) -> list[str]:
        return [
            "-g", str(max(1, round(self.fps * KEYFRAME_INTERVAL_SECONDS))),
            "-movflags", "+faststart",
            self._current_chunk_filename(),
        ]

    def _ffmpeg_command(self, width: int, height: int) -> list[str]:
        command = ["ffmpeg", "-hide_banner", "-v", "error", "-y"]

        hardware_encoder = get_hardware_encoder_or_none()

        if hardware_encoder is not None and min(width, height) < MINIMUM_HARDWARE_ENCODER_DIMENSION:
            logger.info(
                f"{width}x{height} is below the {MINIMUM_HARDWARE_ENCODER_DIMENSION} pixel minimum "
                f"of the GPU's encoder, {self.filename} will be encoded in software."
            )
            hardware_encoder = None

        if hardware_encoder is not None:
            command += [
                "-init_hw_device", f"vaapi=hw:{VAAPI_RENDER_NODE}",
                "-filter_hw_device", "hw",
            ]

        command += [
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}",
            "-r", str(self.fps),
            "-i", "pipe:0",
        ]

        if hardware_encoder is not None:
            command += [
                "-vf", "format=nv12,hwupload",
                "-c:v", hardware_encoder,
                "-rc_mode", "CQP",
                "-global_quality", str(HEVC_CONSTANT_QUANTIZER),
                # Without hvc1 the track is tagged hev1, which QuickTime and Safari refuse to play.
                "-tag:v", "hvc1",
            ]
        else:
            command += [
                "-vf", "format=yuv420p",
                "-c:v", "libx264",
                "-preset", "veryfast",
                "-crf", str(H264_CONSTANT_RATE_FACTOR),
            ]

        return command + self._output_arguments()

    def _start(self, width: int, height: int):
        command = self._ffmpeg_command(width, height)

        # ffmpeg's diagnostics go to a file rather than a pipe nobody reads, which would fill up and
        # block the encoder partway through a long recording.
        if self._encoder_log is None:
            log_path = os.path.join(os.path.dirname(self.filename), ENCODER_LOG_FILENAME)
            self._encoder_log = open(log_path, "ab")
        self._encoder_log.write((" ".join(command) + "\n").encode())
        self._encoder_log.flush()

        self._process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=self._encoder_log,
            # Ctrl-C reaches every process in the terminal's group, which would kill the encoder
            # before it has written the index and leave the recording unplayable. Its own session
            # keeps it alive until release() closes stdin and it finishes on its own.
            start_new_session=True,
        )

        self._chunk_started_at = time.monotonic()

        if self._is_chunked:
            logger.info(
                f"Writing {self._current_chunk_filename()} at {self.fps} fps, {width}x{height}, "
                f"rotating after {self.chunk_seconds:g}s."
            )
        else:
            logger.info(
                f"Writing video to {self.filename} at {self.fps} fps, {width}x{height}."
            )

    def _start_fallback(self, width: int, height: int):
        """Encodes with OpenCV when ffmpeg is missing, so that recording still works."""
        logger.warning(
            "ffmpeg was not found, falling back to OpenCV's mpeg4 encoder. Recordings will be "
            "roughly ten times larger and will not be split into chunks. Install ffmpeg to fix this."
        )
        self._fallback_writer = cv2.VideoWriter(
            self.filename, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (width, height)
        )
        if not self._fallback_writer.isOpened():
            raise RuntimeError(f"OpenCV could not open {self.filename} to write video to.")

    def write(self, color_image: np.ndarray):
        # Rotating in the pipeline returns a view with negative strides, which cannot be written.
        color_image = np.ascontiguousarray(color_image)

        if color_image.ndim == 2:
            color_image = cv2.cvtColor(color_image, cv2.COLOR_GRAY2BGR)

        height, width = color_image.shape[:2]

        if self._process is None and self._fallback_writer is None:
            self._frame_size = (width, height)
            if shutil.which("ffmpeg") is None:
                self._start_fallback(width, height)
            else:
                self._start(width, height)

        if (width, height) != self._frame_size:
            logger.warning(
                f"Skipping a {width}x{height} frame, {self.filename} is being written at {self._frame_size[0]}x{self._frame_size[1]}."
            )
            return

        if self._fallback_writer is not None:
            self._fallback_writer.write(color_image)
            return

        # Chunks are rotated on elapsed real time rather than by ffmpeg's segment muxer, which
        # measures the recording's own timeline. A camera that delivers below its configured rate
        # makes that timeline run slower than the clock, so a "300 second" chunk would have taken
        # however long it took to capture 300 seconds worth of frames.
        if self._is_chunked and time.monotonic() - self._chunk_started_at >= self.chunk_seconds:
            self._rotate_chunk(width, height)

        try:
            self._process.stdin.write(color_image.tobytes())
        except (BrokenPipeError, ValueError):
            raise RuntimeError(
                f"The encoder writing {self._current_chunk_filename()} exited early, see "
                f"{os.path.join(os.path.dirname(self.filename), ENCODER_LOG_FILENAME)}."
            )

    def _rotate_chunk(self, width: int, height: int):
        """Closes the chunk being written and opens the next one."""
        process, self._process = self._process, None
        if process is not None:
            try:
                process.stdin.close()
            except BrokenPipeError:
                ...
            self._finishing_processes.append(process)

        # Chunks closed earlier have had a whole chunk's worth of time to finish, so this only
        # collects their exit status rather than waiting on them.
        self._reap_finished_processes()

        self._chunk_index += 1
        self._start(width, height)

    def _reap_finished_processes(self, wait: bool = False):
        still_finishing = []
        for process in self._finishing_processes:
            if wait:
                try:
                    process.wait(timeout=120)
                except subprocess.TimeoutExpired:
                    logger.error(f"An encoder for {self.filename} did not exit, killing it.")
                    process.kill()
                    process.wait()
            elif process.poll() is None:
                still_finishing.append(process)
                continue

            if process.returncode != 0:
                logger.error(
                    f"An encoder for {self.filename} exited with {process.returncode}, see "
                    f"{os.path.join(os.path.dirname(self.filename), ENCODER_LOG_FILENAME)}."
                )
        self._finishing_processes = still_finishing

    def release(self):
        """Closes the file. Without this the video has no index and cannot be played back."""
        if self._fallback_writer is not None:
            self._fallback_writer.release()
            self._fallback_writer = None
            logger.info(f"Finished writing {self.filename}.")
            return

        if self._process is None:
            self._reap_finished_processes(wait=True)
            return

        # Closing stdin tells ffmpeg the stream ended, so it writes out the index and exits.
        try:
            self._process.stdin.close()
        except BrokenPipeError:
            ...

        try:
            return_code = self._process.wait(timeout=120)
        except subprocess.TimeoutExpired:
            logger.error(f"The encoder writing {self.filename} did not exit, killing it.")
            self._process.kill()
            return_code = self._process.wait()

        self._process = None

        self._reap_finished_processes(wait=True)

        if self._encoder_log is not None:
            self._encoder_log.close()
            self._encoder_log = None

        if return_code != 0:
            logger.error(
                f"The encoder writing {self._current_chunk_filename()} exited with {return_code}, "
                f"see {os.path.join(os.path.dirname(self.filename), ENCODER_LOG_FILENAME)}."
            )
            return

        if self._is_chunked:
            logger.info(
                f"Finished writing {self._chunk_index + 1} chunk(s) of {self.filename}."
            )
        else:
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
    chunk_seconds: float | None = DEFAULT_RECORDING_CHUNK_SECONDS,
):
    """Writes captured frames to disk until `stop_event` is set and the queue has been drained.

    Setting `abandon_event` drops whatever is still queued and closes the files right away, for a user
    who would rather quit than wait for a long recording to be written out.

    `finished_event` is set once every file has been closed, so that whoever is quitting knows that
    the recording is complete and it is safe to exit.

    `chunk_seconds` splits a video recording into files of that length, so that an interrupted
    recording is still playable up to the last completed chunk. 0 or None writes a single file.
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
                        video_writers[filename] = VideoFileWriter(
                            filename, video_fps, chunk_seconds=chunk_seconds
                        )
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
