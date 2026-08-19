from enum import Enum


class RecordingFileFormat(Enum):
    """The file formats that a camera recording can be written to disk as.

    `png` and `jpg` write one file per frame, `mp4` writes every frame of a camera into one video file.
    """

    png = "png"
    jpg = "jpg"
    mp4 = "mp4"

    @staticmethod
    def all_extensions() -> list[str]:
        return [file_format.extension for file_format in RecordingFileFormat]

    @staticmethod
    def from_string(file_format: str) -> "RecordingFileFormat":
        """Parses a user supplied format, with or without the leading dot, e.g. `.png`, `png` or `JPEG`."""
        name = file_format.strip().lower().lstrip(".")

        if name == "jpeg":
            name = RecordingFileFormat.jpg.value

        try:
            return RecordingFileFormat(name)
        except ValueError:
            raise ValueError(
                f"{file_format} is not a supported recording file format. Supported formats: {', '.join(RecordingFileFormat.all_extensions())}."
            )

    @property
    def extension(self) -> str:
        return "." + self.value

    def is_video(self) -> bool:
        """Video formats accumulate every frame into one file, instead of writing one file per frame."""
        return self is RecordingFileFormat.mp4
