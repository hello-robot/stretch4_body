from stretch4_body.behavior.sentries.sentry import Sentry
from stretch4_body.robot.robot_client import RobotClient
import multiprocessing
import time
import os
import json
import logging
import signal
from typing import Optional


def writer_daemon(log_dir: str, check_rate: float, do_exit: multiprocessing.Event, maximum_log_size_mb: int, logger:logging.Logger):
    """
    Background daemon that consumes batched statuses from IPC
    and writes them to disk, managing the directory size.
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

    # Set up client to communicate with RobotServer asynchronously without blocking its loops
    r = RobotClient()
    if not r.startup():
        logger.error("SentryStatusLogger daemon failed to start RobotClient")
        return

    batch = []
    last_batch_time = time.time()
    sleep_time = 1.0 / float(check_rate)

    try:
        while not do_exit.is_set():
            r.pull_status()
            rs = r.status.copy()
            if "timestamp" not in rs:
                rs["timestamp"] = time.time()
            batch.append(rs)

            # Batch for 1 minute
            if time.time() - last_batch_time >= 60.0:
                if batch:
                    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
                    filename = os.path.join(log_dir, f"status_{timestamp_str}.json")
                    try:
                        with open(filename, "w") as f:
                            json.dump(batch, f, default=str)
                    except Exception as e:
                        logger.error(f"Error writing status batch: {e}")

                    batch = []
                    manage_directory_size()

                last_batch_time = time.time()

            # Slight yield to avoid spinning CPU but fast enough to poll
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        # Final flush on exit
        if batch:
            timestamp_str = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(log_dir, f"status_{timestamp_str}.json")
            try:
                with open(filename, "w") as f:
                    json.dump(batch, f, default=str)
            except Exception as e:
                logger.error(f"Error writing final status batch: {e}")
            manage_directory_size()
            
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
