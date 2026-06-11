import time
import cv2
import numpy as np
from typing import Callable

class DetectFrameSettled:
    """
    This is used to prevent processing blurry or moving images (Motion Gating). It supports two primary detection methods:
    1. Frame Differencing: Detects physical motion/displacement.
    2. Laplacian Variance: Detects changes in sharpness or vibration.
    """

    def __init__(self, required_stable_frames: int = 5):
        self.required_stable_frames = required_stable_frames
        
        # Internal State Management
        self.stable_frame_count = 0
        self.prev_gray = None
        self.prev_variance = None
        self.prev_gray_ema = None
        
    def reset(self):
        """Resets the internal frame counters and anchors."""
        self.stable_frame_count = 0
        self.prev_gray = None
        self.prev_variance = None
        self.prev_gray_ema = None

    def check_stability_diff(self, frame:np.ndarray|None, threshold=3.0, timeout_blocking: float | None = None):
        """
        Gates based on the Mean Absolute Difference (MAD).
        If timeout_blocking is not None, this call will block.
        """
        return self._execute_check(frame, timeout_blocking, lambda f: self._compute_diff(f, threshold))

    def check_stability_sharpness(self, frame:np.ndarray|None, threshold=5.0, timeout_blocking: float | None = None):
        """
        Gates based on the stability of the Laplacian Variance.
        If timeout_blocking is not None, this call will block.
        """
        return self._execute_check(frame, timeout_blocking, lambda f: self._compute_laplacian(f, threshold))

    def check_stability_ema(self, frame:np.ndarray|None, threshold=3.0, alpha=0.1, timeout_blocking: float | None = None):
        """
        Gates based on 8x8 block-based maximum difference against an EMA running average.
        If timeout_blocking is not None, this call will block.
        """
        return self._execute_check(frame, timeout_blocking, lambda f: self._compute_ema(f, threshold, alpha))

    def _execute_check(self, frame:np.ndarray|None, timeout_blocking: float | None, algorithm_func: Callable):
        """
        Handles the logic for both instantaneous (None) and blocking (float) checks.
        """
        # NON-BLOCKING: Check once and return status immediately
        if timeout_blocking is None:
            return algorithm_func(frame)

        # BLOCKING: Loop until settled or timeout reached
        start_time = time.monotonic()
        while (time.monotonic() - start_time) < timeout_blocking:
            if algorithm_func(frame):
                return True
            # Small sleep to prevent CPU spiking in the blocking loop
            time.sleep(0.01) 
            
        return False
    
    def has_frame_been_stable(self):
        return self.stable_frame_count >= self.required_stable_frames

    def _compute_diff(self, frame:np.ndarray|None, threshold):
        if frame is None: return False
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        is_stable = False
        if self.prev_gray is not None:
            diff = cv2.absdiff(self.prev_gray, gray)
            if np.mean(diff) < threshold:
                self.stable_frame_count += 1
            else:
                self.stable_frame_count = 0 
            
            if self.has_frame_been_stable():
                is_stable = True

        self.prev_gray = gray
        return is_stable

    def _compute_laplacian(self, frame:np.ndarray|None, threshold):
        if frame is None: return False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        current_variance = cv2.Laplacian(gray, cv2.CV_64F).var()

        is_stable = False
        if self.prev_variance is not None:
            if abs(current_variance - self.prev_variance) < threshold:
                self.stable_frame_count += 1
            else:
                self.stable_frame_count = 0 

            if self.has_frame_been_stable():
                is_stable = True

        self.prev_variance = current_variance
        return is_stable

    def _compute_ema(self, frame:np.ndarray|None, threshold, alpha):
        if frame is None: return False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        gray_f = gray.astype(np.float32)

        is_stable = False
        if self.prev_gray_ema is not None:
            diff = np.abs(self.prev_gray_ema - gray_f)
            diff_blocks = cv2.resize(diff, (8, 8), interpolation=cv2.INTER_AREA)
            max_block_diff = np.max(diff_blocks)

            if max_block_diff < threshold:
                self.stable_frame_count += 1
            else:
                self.stable_frame_count = 0

            if self.has_frame_been_stable():
                is_stable = True

            # Update running average (EMA)
            self.prev_gray_ema = alpha * gray_f + (1.0 - alpha) * self.prev_gray_ema
        else:
            self.prev_gray_ema = gray_f

        return is_stable