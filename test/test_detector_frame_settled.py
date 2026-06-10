import pytest
import numpy as np
import cv2
import time
from stretch4_body.subsystem.cameras.detectors.detector_frame_settled import DetectFrameSettled

def test_frame_differencing_gating():
    # Original behavior test (anchoring)
    detector = DetectFrameSettled(required_stable_frames=3)
    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    
    assert not detector.check_stability_diff(frame1)
    assert not detector.check_stability_diff(frame1)
    assert not detector.check_stability_diff(frame1)
    assert detector.check_stability_diff(frame1)
    
    # Large change should reset stability
    frame2 = np.ones((480, 640, 3), dtype=np.uint8) * 100
    assert not detector.check_stability_diff(frame2)
    assert not detector.check_stability_diff(frame2)
    assert not detector.check_stability_diff(frame2)
    assert detector.check_stability_diff(frame2)

def test_sharpness_gating():
    # Original behavior test (anchoring variance)
    detector = DetectFrameSettled(required_stable_frames=2)
    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.line(frame1, (10, 10), (100, 100), (255, 255, 255), 2)
    
    assert not detector.check_stability_sharpness(frame1)
    assert not detector.check_stability_sharpness(frame1)
    assert detector.check_stability_sharpness(frame1)
    
    frame2 = cv2.GaussianBlur(frame1, (15, 15), 0)
    assert not detector.check_stability_sharpness(frame2)
    assert not detector.check_stability_sharpness(frame2)
    assert detector.check_stability_sharpness(frame2)

def test_slow_continuous_motion_original_diff():
    # Verify that original check_stability_diff eventually detects slow drift/motion due to anchoring
    detector = DetectFrameSettled(required_stable_frames=3)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Establish stability
    for _ in range(4):
        detector.check_stability_diff(frame, threshold=2.0)
    assert detector.has_frame_been_stable()
    
    # Increment slowly. After 2 increments of 1.0, the total difference from the anchor (0) is 2.0,
    # which is not < threshold (2.0), so it should reset.
    frame = frame.copy() + 1
    assert detector.check_stability_diff(frame, threshold=2.0) # difference is 1.0 < 2.0 (stable)
    
    frame = frame.copy() + 1
    assert not detector.check_stability_diff(frame, threshold=2.0) # difference is 2.0 >= 2.0 (unstable!)

def test_blocking_timeout():
    detector = DetectFrameSettled(required_stable_frames=3)
    frame1 = np.zeros((480, 640, 3), dtype=np.uint8)
    
    start = time.time()
    assert detector.check_stability_diff(frame1, timeout_blocking=0.5)
    end = time.time()
    assert (end - start) < 0.1, "Should have returned immediately due to passing the same frame"

def test_ema_gating_stability_and_local_motion():
    detector = DetectFrameSettled(required_stable_frames=3)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # 1. Establish stability
    assert not detector.check_stability_ema(frame, threshold=3.0)
    assert not detector.check_stability_ema(frame, threshold=3.0)
    assert not detector.check_stability_ema(frame, threshold=3.0)
    assert detector.check_stability_ema(frame, threshold=3.0)
    
    # 2. Slow lighting drift (should remain stable)
    frame_f = frame.astype(np.float32)
    for _ in range(10):
        frame_f = frame_f + 0.2 # drift of 0.2 per frame
        frame_uint8 = np.clip(frame_f, 0, 255).astype(np.uint8)
        assert detector.check_stability_ema(frame_uint8, threshold=3.0, alpha=0.1)
        
    # 3. Local motion (changing a 50x50 block in the center to 255)
    motion_frame = frame_uint8.copy()
    motion_frame[200:250, 200:250, :] = 255
    assert not detector.check_stability_ema(motion_frame, threshold=3.0)
    assert not detector.has_frame_been_stable()
