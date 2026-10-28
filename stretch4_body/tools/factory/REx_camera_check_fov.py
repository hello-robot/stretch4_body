#!/usr/bin/env python3

import time
import cv2
import numpy as np
import argparse
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.core.hello_utils import print_stretch_re_use
from stretch4_body.subsystem.cameras.detectors.detector_frame_settled import DetectFrameSettled
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import RGBPipelineController
from stretch4_body.subsystem.cameras.cv_utils import undistort_points

# --- Constants ---
EXPECTED_FOV_LEFT = (1200, 1655)
EXPECTED_FOV_RIGHT = (1200, 1655)
EXPECTED_FOV_CENTER = (3040, 4032)
TOLERANCE = 20

class Colors:
    GREEN = '\033[92m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

def print_valid(text):
    print(f"{Colors.GREEN}{text}{Colors.ENDC}")

def print_invalid(text):
    print(f"{Colors.RED}{text}{Colors.ENDC}")

def calculate_hv_fov(calibration, x_min, x_max, y_min, y_max):
    """Calculate Horizontal and Vertical FOV in degrees using calibration and visible bounds."""
    if calibration is None or not hasattr(calibration, 'camera_matrix'):
        return None, None
    
    # Points at the center of each edge of the visible region
    pts = np.array([
        [x_min, (y_min+y_max)/2.0], # Left
        [x_max, (y_min+y_max)/2.0], # Right
        [(x_min+x_max)/2.0, y_min], # Top
        [(x_min+x_max)/2.0, y_max]  # Bottom
    ], dtype=np.float32).reshape(-1, 1, 2)
    
    try:
        undist = undistort_points(pts, calibration.camera_matrix, calibration.distortion_coefficients, calibration.distortion_model)
        
        # undist returns (x, y) coordinates on the Z=1 plane
        # angle = 2 * arctan(distance)
        h_dist = np.linalg.norm(undist[1] - undist[0])
        v_dist = np.linalg.norm(undist[3] - undist[2])
        
        h_fov = np.degrees(2 * np.arctan(h_dist / 2.0))
        v_fov = np.degrees(2 * np.arctan(v_dist / 2.0))
        
        return h_fov, v_fov
    except Exception:
        return None, None

def measure_visible_region(image, visualize=False):
    """Measure the width and height of the non-black visible region."""
    if image is None:
        return 0, 0, None
    
    # 1. Switch to LAB Color Space and isolate the L channel (Luminance)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    L = lab[:, :, 0]
    h, w = L.shape
    
    # Apply slight blur to ignore isolated sensor noise
    blurred = cv2.GaussianBlur(L, (5, 5), 0)
    
    # 2. Use a Relative Intensity Threshold
    # Find the brightest point in the center 10% area
    center_roi = blurred[int(h * 0.45):int(h * 0.55), int(w * 0.45):int(w * 0.55)]
    max_L = np.max(center_roi)
    
    # Threshold to capture the darker edges of the visible circle
    thresh = max_L * 0.15
    
    # Scan a range of columns/rows in the center to find the absolute peaks of the dome.
    col_scan_range = range(int(w * 0.45), int(w * 0.55), 2)
    row_scan_range = range(int(h * 0.45), int(h * 0.55), 2)
    
    # 1. Find Top Edge 
    top_pts = []
    for x in col_scan_range:
        for y in range(h):
            if blurred[y, x] > thresh:
                top_pts.append(y)
                break
    y_min = int(np.min(top_pts)) if top_pts else 0
    
    # 2. Find Bottom Edge
    bottom_pts = []
    for x in col_scan_range:
        for y in range(h - 1, -1, -1):
            if blurred[y, x] > thresh:
                bottom_pts.append(y)
                break
    y_max = int(np.max(bottom_pts)) if bottom_pts else h - 1
    
    # 3. Find Left Edge
    left_pts = []
    for y in row_scan_range:
        for x in range(w):
            if blurred[y, x] > thresh:
                left_pts.append(x)
                break
    x_min = int(np.min(left_pts)) if left_pts else 0
    
    # 4. Find Right Edge
    right_pts = []
    for y in row_scan_range:
        for x in range(w - 1, -1, -1):
            if blurred[y, x] > thresh:
                right_pts.append(x)
                break
    x_max = int(np.max(right_pts)) if right_pts else w - 1

    rect_w = x_max - x_min + 1
    rect_h = y_max - y_min + 1
    
    viz_img = None
    if visualize:
        viz_img = image.copy()
        # Draw the final bounding box
        cv2.rectangle(viz_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.putText(viz_img, f"Measured: {rect_w}x{rect_h}", (x_min + 5, y_min + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    return x_min, x_max, y_min, y_max, viz_img

def perform_check(name, last_frame, expected_fov, camera_type, visualize=False):
    """Perform measurement and FOV calculation on a single stabilized frame."""
    if last_frame is None:
        print_invalid(f"Failed to grab a frame from {name} camera.")
        return None, False

    x_min, x_max, y_min, y_max, viz_img = measure_visible_region(last_frame, visualize=visualize)
    w = x_max - x_min + 1
    h = y_max - y_min + 1
    exp_w, exp_h = expected_fov
    
    # Check calibration
    calibration = None
    h_fov, v_fov = None, None
    try:
        calibration = camera_type.load_calibration()
        if calibration:
            h_fov, v_fov = calculate_hv_fov(calibration, x_min, x_max, y_min, y_max)
    except Exception:
        pass

    print(f"\n--- {name.upper()} Camera Results ---")
    print(f"Measured Visible Region: {w}x{h}")
    if h_fov and v_fov:
        print(f"Calculated FOV: H={h_fov:.1f} deg, V={v_fov:.1f} deg")
    
    print(f"Expected Visible Region: {exp_w}x{exp_h}")
    
    is_valid = (abs(w - exp_w) <= TOLERANCE) and (abs(h - exp_h) <= TOLERANCE)
    
    if is_valid:
        print_valid("FOV is VALID")
    else:
        print_invalid("FOV is INVALID")

    if visualize and viz_img is not None:
        # Draw the FOV calculation points (Left, Right, Top, Bottom)
        points = [
            (int(x_min), int((y_min + y_max) / 2)),
            (int(x_max), int((y_min + y_max) / 2)),
            (int((x_min + x_max) / 2), int(y_min)),
            (int((x_min + x_max) / 2), int(y_max))
        ]
        for p in points:
            cv2.circle(viz_img, p, 5, (255, 0, 255), -1)

        # Add validity text to image
        status_text = "VALID" if is_valid else "INVALID"
        color = (0, 255, 0) if is_valid else (0, 0, 255)
        # Position text further down (100 instead of 80) to avoid clipping
        cv2.putText(viz_img, f"{name.upper()}: {status_text}", (int(viz_img.shape[1] - 400), viz_img.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 5)
        if h_fov and v_fov:
             cv2.putText(viz_img, f"FOV: H={h_fov:.1f}, V={v_fov:.1f}", (20, viz_img.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)

    return viz_img, is_valid

def main():
    description = (
        "REx Camera FOV Check\n"
        "Measures the visible (non-vignetted) region of the head cameras.\n"
        "Note: Ensure cameras are NOT facing direct sunlight or strong illumination "
        "to avoid overexposure, which can interfere with the measurement."
    )
    parser = argparse.ArgumentParser(description=description, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument("--visualize", action="store_true", help="Visualize the camera stream and the measured FOV region.")
    args = parser.parse_args()

    print_stretch_re_use()
    print("Camera FOV Check")
    print("====================")
    
    print("\nStarting synced stream for all head cameras...")
    
    # Instantiate the controller directly to access exposure presets while keeping synced parallel capture
    controller = RGBPipelineController(
        camera_type=RGBCameras.head_left_right_center,
        recording_directory=None,
        show_image_in=None,
        is_rotate=True,
        is_rectify=False,
        is_crop=False,
        ai_models_to_use=[],
        detect_aruco_marker_size=None
    )
    
    # Apply calibration exposure settings
    # controller.set_calibration_exposure_preset()
    
    detectors = {
        "left": DetectFrameSettled(required_stable_frames=15),
        "right": DetectFrameSettled(required_stable_frames=15),
        "center": DetectFrameSettled(required_stable_frames=5)
    }
    
    stable_frames = {"left": None, "right": None, "center": None}
    settled_at = {"left": None, "right": None, "center": None}
    
    start_time = time.time()
    stream_gen = controller.get_frame_synced(is_run_pipeline=True)
    
    try:
        for synced_frame in stream_gen:
            if synced_frame is None:
                continue
            
            # Check stability for each camera that hasn't settled yet
            if synced_frame.left is not None and stable_frames["left"] is None:
                if detectors["left"].check_stability_diff(synced_frame.left.image):
                    stable_frames["left"] = synced_frame.left.image
                    settled_at["left"] = time.time() - start_time
                    print(f"LEFT camera settled after {settled_at['left']:.2f}s")
                    
            if synced_frame.right is not None and stable_frames["right"] is None:
                if detectors["right"].check_stability_diff(synced_frame.right.image):
                    stable_frames["right"] = synced_frame.right.image
                    settled_at["right"] = time.time() - start_time
                    print(f"RIGHT camera settled after {settled_at['right']:.2f}s")
                    
            if synced_frame.center is not None and stable_frames["center"] is None:
                if detectors["center"].check_stability_diff(synced_frame.center.image, threshold=5):
                    stable_frames["center"] = synced_frame.center.image
                    settled_at["center"] = time.time() - start_time
                    print(f"CENTER camera settled after {settled_at['center']:.2f}s")
            
            if args.visualize:
                # Show all three cameras h-stacked during stabilization
                target_h_prev = 1920
                preview_images = []
                # Use a common width based on Left camera aspect ratio (1.6)
                target_w_prev = 1200

                for cam_frame in [synced_frame.left, synced_frame.right, synced_frame.center]:
                    if cam_frame is not None:
                        preview_images.append(cv2.resize(cam_frame.image, (target_w_prev, target_h_prev)))
                    else:
                        # Placeholder for missing frames
                        preview_images.append(np.zeros((target_h_prev, target_w_prev, 3), dtype=np.uint8))
                
                stacked_preview = cv2.hconcat(preview_images)
                cv2.namedWindow("Stabilizing cameras (L | R | C)...", cv2.WINDOW_NORMAL)
                cv2.imshow("Stabilizing cameras (L | R | C)...", stacked_preview)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            # Break if all settled or timeout
            if all(f is not None for f in stable_frames.values()):
                break
            
            if time.time() - start_time > 15.0:
                print("Timeout waiting for all cameras to stabilize. Capturing remaining frames.")
                # Fill in any missing frames with the current ones
                if stable_frames["left"] is None: stable_frames["left"] = synced_frame.left.image
                if stable_frames["right"] is None: stable_frames["right"] = synced_frame.right.image
                if stable_frames["center"] is None: stable_frames["center"] = synced_frame.center.image
                break
                
    except Exception as e:
        print(f"Error during parallel streaming: {e}")
    finally:
        controller.stop()
        if args.visualize:
            cv2.destroyAllWindows()
            
    # Perform measurements and collect visualization images
    results = []
    res_l, _ = perform_check("left", stable_frames["left"], EXPECTED_FOV_LEFT, RGBCameras.left(), visualize=args.visualize)
    res_r, _ = perform_check("right", stable_frames["right"], EXPECTED_FOV_RIGHT, RGBCameras.right(), visualize=args.visualize)
    res_c, _ = perform_check("center", stable_frames["center"], EXPECTED_FOV_CENTER, RGBCameras.center(), visualize=args.visualize)
    
    if args.visualize:
        # H-stack the results for comparison
        # Base everything on the Left camera's dimensions for a uniform look
        target_h = 1920
        target_w = 1200 # Default fallback
        if res_l is not None:
            h_orig, w_orig = res_l.shape[:2]
            target_w = int(w_orig * (target_h / h_orig))
            
        stacked_images = []
        for img in [res_l, res_r, res_c]:
            if img is not None:
                # Use INTER_AREA for higher quality downsampling
                stacked_images.append(cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_AREA))
        
        if stacked_images:
            combined = cv2.hconcat(stacked_images)
            cv2.namedWindow("REx Camera FOV Results (Left | Right | Center)", cv2.WINDOW_NORMAL)
            cv2.imshow("REx Camera FOV Results (Left | Right | Center)", combined)
            print("\nDisplaying combined results. Press any key to exit...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
