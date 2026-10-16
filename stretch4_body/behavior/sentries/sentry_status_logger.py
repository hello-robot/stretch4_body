from stretch4_body.behavior.sentries.sentry import Sentry
from stretch4_body.robot.robot_client import RobotClient
import multiprocessing
import time
import os
import json
import logging
import signal
from typing import Optional

from stretch4_body.core.status_log import StatusLogTrimmer

# A temp file this old at daemon start is left over from a writer that died mid-write.
STALE_TEMP_S = 600.0


def write_batch(log_dir: str, encoded_samples, stamp: Optional[str] = None) -> str:
    """
    Write JSON-encoded samples as one status_<stamp>.json array and return its path.
    Written to a temp name, fsynced, then renamed, so a reader or a power cut never sees
    a partial file. Never replaces an existing file: two batches in the same second (the
    exit flush right after a timed write) get status_<stamp>_1.json.
    """
    stamp = stamp or time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(log_dir, f"status_{stamp}.json")
    n = 1
    while os.path.exists(path):
        path = os.path.join(log_dir, f"status_{stamp}_{n}.json")
        n += 1
    tmp = f"{path}.{os.getpid()}.tmp"
    try:
        with open(tmp, "w") as f:
            f.write("[")
            for i, sample in enumerate(encoded_samples):
                if i:
                    f.write(", ")
                f.write(sample)
            f.write("]")
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.remove(tmp)
        except OSError:
            pass
        raise
    return path


def remove_stale_temp_files(log_dir: str, logger: logging.Logger):
    now = time.time()
    for f in os.listdir(log_dir):
        if not f.endswith(".tmp"):
            continue
        path = os.path.join(log_dir, f)
        try:
            if now - os.path.getmtime(path) > STALE_TEMP_S:
                os.remove(path)
                logger.warning(f"Removed partial status batch {f}")
        except OSError:
            pass


def writer_daemon(log_dir: str, check_rate: float, do_exit: multiprocessing.Event, maximum_log_size_mb: int,
                  logger: logging.Logger, batch_period_s: float = 60.0, client_factory=RobotClient):
    """
    Background daemon that polls RobotClient status and writes it to disk in batches,
    managing the directory size. The file format (aliases and per-subsystem server
    copies dropped, slow power keys written only when they change) is in
    stretch4_body/core/status_log.py.
    """
    def manage_directory_size(max_size_bytes=maximum_log_size_mb * 1024 * 1024):
        try:
            files = [
                os.path.join(log_dir, f)
                for f in os.listdir(log_dir)
                if f.endswith(".json")
            ]
            if not files:
                return

            # Decorate sorting with os.path.getmtime for sort
            files.sort(key=os.path.getmtime)

            total_size = sum(os.path.getsize(f) for f in files)

            while total_size > max_size_bytes and len(files) > 0:
                oldest = files.pop(0)
                size = os.path.getsize(oldest)
                os.remove(oldest)
                total_size -= size
        except Exception as e:
            logger.error(f"Error managing status dir size: {e}")

    def flush(what):
        try:
            write_batch(log_dir, batch)
        except Exception as e:
            logger.error(f"Error writing {what}: {e}")
        trimmer.reset()   # every file starts with a complete sample
        manage_directory_size()

    remove_stale_temp_files(log_dir, logger)

    # Set up client to communicate with RobotServer asynchronously without blocking its loops
    r = client_factory()
    if not r.startup():
        logger.error("SentryStatusLogger daemon failed to start RobotClient")
        return

    trimmer = StatusLogTrimmer()
    batch = []
    last_batch_time = time.time()
    sleep_time = 1.0 / float(check_rate)

    try:
        while not do_exit.is_set():
            r.pull_status()
            # Shallow: the subsystem dicts are RobotClient's own and pull_status updates
            # them in place, so the sample is encoded now, not at write time.
            rs = r.status.copy()
            if "timestamp" not in rs:
                rs["timestamp"] = time.time()

            # Drop line sensor data entirely for now
            rs.pop('line_sensor_loop', None)

            batch.append(json.dumps(trimmer.trim(rs), default=str))

            if time.time() - last_batch_time >= batch_period_s:
                if batch:
                    flush("status batch")
                    batch = []
                last_batch_time = time.time()

            # Slight yield to avoid spinning CPU but fast enough to poll
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        # Final flush on exit
        if batch:
            flush("final status batch")
        r.stop()


class SentryStatusLogger(Sentry):
    def __init__(self, robot):
        Sentry.__init__(self, "sentry_status_logger", robot)
        self.status = {}
        self.writer_process: Optional[multiprocessing.Process] = None
        self.do_exit = multiprocessing.Event()

    def startup(self) -> bool:
        check_rate = self.params["check_rate"]
        maximum_log_size_mb = self.params["maximum_log_size_mb"]

        fleet_path = os.getenv("HELLO_FLEET_PATH", os.path.expanduser("~"))
        log_dir = os.path.join(fleet_path, "log", "stretch_status")
        os.makedirs(log_dir, exist_ok=True)

        self.writer_process = multiprocessing.Process(
            target=writer_daemon, args=(log_dir, check_rate, self.do_exit, maximum_log_size_mb, self.logger)
        )
        self.writer_process.daemon = False
        self.writer_process.start()
        self.logger.info(f"Started SentryStatusLogger daemon process {self.writer_process.pid}")
        return Sentry.startup(self)

    def step(self):
        # We process nothing here to avoid deepcopy lags in the synchronous control loop
        pass

    def _manage_ctrlC(self, *args):
        self.do_exit.set()

    def stop(self) -> bool:
        self.logger.info('Stopping SentryStatusLogger process')
        
        # Safely capture main thread termination similar to end_of_arm_loop
        original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._manage_ctrlC)
        self.do_exit.set()
        
        if self.writer_process is not None:
            self.writer_process.join()
            self.writer_process = None
            
        signal.signal(signal.SIGINT, original_sigint)
        return True
