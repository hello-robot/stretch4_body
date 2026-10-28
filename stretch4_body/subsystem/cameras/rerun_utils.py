import threading
import queue
import cv2
import rerun as rr
import numpy as np

class RerunAsyncLogger:
    """
    An asynchronous logger for Rerun that offloads CPU-intensive tasks like
    image compression to a background thread to maintain high frame rates.
    """
    def __init__(self, camera_name: str, queue_size: int = 10):
        self.camera_name = camera_name.upper()
        self.queue = queue.Queue(maxsize=queue_size)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _worker(self):
        while not self.stop_event.is_set():
            try:
                # Task format: (entity_path, data, type)
                task = self.queue.get(timeout=0.1)
                if task is None:
                    continue
                
                path, data, task_type = task
                
                if task_type == "image":
                    # Use OpenCV for faster encoding. Quality 80 is a good balance.
                    success, jpeg_bytes = cv2.imencode('.jpg', data, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if success:
                        rr.log(
                            path,
                            rr.EncodedImage(contents=jpeg_bytes.tobytes(), media_type="image/jpeg"),
                        )
                elif task_type == "direct":
                    # For DepthImage, Points3D, etc. that don't need host-side compression
                    rr.log(path, data)
                
            except queue.Empty:
                continue
            except Exception as e:
                import logging
                logging.error(f"Error in RerunAsyncLogger worker for {self.camera_name}: {e}")

    def log_image(self, path: str, image: np.ndarray):
        """Queue an image for asynchronous compression and logging."""
        try:
            self.queue.put( (path, image, "image"), block=True)
        except Exception:
            pass

    def log_any(self, path: str, archetype: "Any"):
        """Queue any Rerun archetype for asynchronous logging."""
        try:
            self.queue.put( (path, archetype, "direct"), block=True)
        except Exception:
            pass

    def stop(self, timeout: float = 2.0):
        """Stop the background thread."""
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=timeout)
