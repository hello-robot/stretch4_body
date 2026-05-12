from enum import Enum


class LogLevels(str, Enum):
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"