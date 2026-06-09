"""
Before using this script, please take a minute to understand its underlying assumptions:

This script estimates the "bluriness" of an image by the quantity of edges (gradient color changes) across the image.
A really blurry image will not have many edge thresholds when scanning across or down an image. 
As the image becomes sharper, there will be more distinct details, forming more edges.

This script uses OpenCV to convolve the Laplacian kernel across a grayscale image, creating a response with the second derivative of its values.
This response is analogous to the quanitiy of edges in an image.
Then it computes the variance of the response to determine the blurriness of an image.

More variance would mean more spread of edges, so the higher the variance score, the more focused an image is - at least compared to its blurry counterpart.

This method is talked about in this paper by So et al 
https://doi.org/10.48550/arXiv.2405.11490, https://arxiv.org/html/2405.11490v1 and "The Laplacian operator has also been applied as an auto-focusing technique in microscopy due to its reliability and speed (Pech-Pacheco et al., 2000)" https://ieeexplore.ieee.org/document/903548 

          
Please follow these instructions for the best results:

1. Point the camera towards a scene with many objects or details.
Do not use a blank white wall, or a sparse scene, because this script uses changes in color gradients across the image to determine bluriness.
2. Press 'r' to reset the focus score before starting to adjust your lens.
3. Do your best to keep your hand out of the camera's view and slowly rotate the lens.
4. Observe the score - when you go past the best focus, you will notice the score will drop. Rotate in the opposite direction and go back to the top score.
5. Press 'q' to quit.
"""

import argparse
import cv2
import numpy as np
import time
from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.subsystem.cameras.detectors.detector_frame_settled import DetectFrameSettled
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import RGBPipelineController
from stretch4_body.subsystem.cameras.models.image_frame import ImageFrame
from colorama import Fore, Style

max_score = 0
last_score = 0

frame_settled_detector = DetectFrameSettled()

def focus_assistant(frame:ImageFrame|None):
    global max_score, last_score, frame_settled_detector

    if frame is None: return

    color_image, timestamp  = frame.image, frame.timestamp

    is_settled = frame_settled_detector.check_stability_diff(color_image)

    cv2.namedWindow('Focus Assistant', cv2.WINDOW_NORMAL)
    
    # 1. Convert to grayscale (Focus is best measured on luminance)
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)


    # 2. Calculate the Laplacian Variance (The "Focus Score")
    # We use a 64-bit float to avoid overflow/truncation issues
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    score = laplacian.var()

    # Update peak score

    if is_settled:
        last_score = score   
        if score > max_score:
            max_score = score     
    
    # Text Settings
    font = cv2.FONT_HERSHEY_SIMPLEX
    color = (0, 255, 0) # Green
    if score < 100: color = (0, 0, 255) # Red if very blurry

    color = color if is_settled else (255,0,255)
    

    color_image_overlayed = cv2.UMat(color_image) 
    # Display Current Score
    cv2.putText(color_image_overlayed, f"Focus Score: {int(score)}" if is_settled else "Focus Score: Waiting for image to settle", (10, 30), 
                font, 0.8, color, 2)
    
    # Display Max Score (Peak Hold)
    cv2.putText(color_image_overlayed, f"Peak: {int(max_score)}", (10, 60), 
                font, 0.8, (255, 255, 0), 2)
    
    cv2.putText(color_image_overlayed, f"Press 'r' to reset. 'q' to quit.", (10, 90), 
                font, 0.8, (170, 170, 170), 2)

    # Draw a dynamic bar at the bottom to visualize focus
    # We normalize the bar based on the Peak Score to make it relative
    if max_score > 0:
        score_to_use = score if is_settled else last_score
        bar_width = int((score_to_use / max_score) * (color_image.shape[1] - 20))
        cv2.rectangle(color_image_overlayed, (10, color_image.shape[0] - 30), 
                        (10 + bar_width, color_image.shape[0] - 10), color, -1)

    # Show the frame
    cv2.namedWindow('Focus Assistant', cv2.WINDOW_NORMAL)
    cv2.imshow('Focus Assistant', color_image_overlayed)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        exit(0)
    elif key == ord('r'):
        max_score = 0
        print("Peak score reset.")
    
def calibrate_focus():
    global frame_settled_detector
    parser = argparse.ArgumentParser(
        prog="Retrieve camera calibration from the Luxonis module and save them to disk."
    )

    parser.add_argument(
        "-l", "--left", action="store_true", help="Use the left RGB camera."
    )
    parser.add_argument(
        "-r", "--right", action="store_true", help="Use the right RGB camera."
    )

    parser.add_argument(
        "--detect_aruco_marker_size",
        type=float,
        default=0.0,
        help="Runs ArUco detection if a float >= 0.0 is provided. If length is 0.0, the ArUco markers will be detected, but distance will not be printed. If length > 0.0 and calibration is available, ArUco pose and L2 distance to the marker will be displayed.",
    )
    
    args = parser.parse_args()
    detect_aruco_marker_size:float|None = args.detect_aruco_marker_size
    camera_type = None
    if args.left:
        camera_type = RGBCameras.left()
    elif args.right:
        camera_type = RGBCameras.right()
    else:
        raise Exception(
            "You must specify one of --left, --right to specify the rgb camera to use."
        )
    print("""
You are about to focus a camera lens. 
          
Please follow these instructions for the best results:

1. Point the camera towards a scene with many objects or details.
Do not use a blank white wall, or a sparse scene, because this script uses changes in color gradients across the image to determine bluriness.
It is recommended to put a ChArUco board with marker size 27mm at 20in away from a stowed wrist at a 3ft height for left/right camera lens focus. 
2. Press 'r' to reset the focus score before starting to adjust your lens.
3. Do your best to keep your hand out of the camera's view and slowly rotate the lens.
4. Observe the score - when you go past the best focus, you will notice the score will drop. Rotate in the opposite direction and go back to the top score.
5. Press 'q' to quit.
          """)

    robot = RobotClient()
    if robot.startup():
        if robot.params.get('tool') == 'eoa_wrist_dw4_tool_calibration':
            ans = input(f"You are using the calibration board tool. Do you want to move arm to calibration pose? {Fore.RED} WARNING: the robot arm and wrist will move. {Style.RESET_ALL} [y/N]: ")
            if ans.lower() == 'y':
                print("Moving to calibration pose...")
                robot.lift.move_to(0.7)
                robot.arm.move_to(0.25)
                robot.end_of_arm.move_to('wrist_pitch', -0.49)
                robot.end_of_arm.move_to('wrist_roll', 0)
                robot.end_of_arm.move_to('wrist_yaw', 0)
                robot.push_command()
                robot.wait_command()

    rgb_pipeline_controller = RGBPipelineController(
        camera_type=camera_type,
        recording_directory=None,
        show_image_in=None,
        is_rotate=True,
        is_rectify=False,
        is_crop=False,
        ai_models_to_use=[],
        detect_aruco_marker_size=detect_aruco_marker_size
    )
    
    print("""
    These exposure settings work best for 450-650 lux ambient lighting.
    This was tested by using the max brightness and the white light setting on the 
    2800-6500K Dimmable Photography Light Panels set 2ft horizontally from the mast on either side of the robot.
    The height of the bottom of each light panel is 5ft from the floor.
    The panels are angled 45 degrees toward the charuco board vertically, about 10 degrees toward the floor.
    """)
    rgb_pipeline_controller.set_calibration_exposure_preset()

    frame_settled_detector.reset()

    for frame in rgb_pipeline_controller.get_frame(is_run_pipeline=True):
        focus_assistant(frame)


if __name__ == "__main__":
    calibrate_focus()

    