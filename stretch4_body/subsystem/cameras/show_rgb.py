from stretch4_body.core.hello_utils import LoopTimer
import argparse
from stretch4_body.subsystem.cameras.controllers.camera_pipeline_controller import (
    RGBPipelineController,
    RGBPipelineControllerROS,
    RecordRgbShowImageIn,
)
from stretch4_body.subsystem.cameras.enums.rgb_camera import RGBCameras

def show_rgb():

    parser = argparse.ArgumentParser(
        prog="Show images from the RGB cameras.",
        description="Displays or saves RGB images from a camera.",
    )
    parser.add_argument(
        "-d",
        "--recording_directory",
        type=str,
        default=None,
        help="Directory used to record the data, if provided, images will be saved to disk in this directory.",
    )
    parser.add_argument(
        "--rerun",
        action="store_true",
        help="Display the recording in a rerun window. Note: this may adversly affect performance. Default: False.",
    )
    parser.add_argument(
        "--opencv",
        action="store_true",
        help="Display the recording in an opencv window. Note: this may adversly affect performance. Default: False.",
    )
    parser.add_argument(
        "-l", "--left", action="store_true", help="Use the left RGB camera."
    )
    parser.add_argument(
        "-r", "--right", action="store_true", help="Use the right RGB camera."
    )
    parser.add_argument(
        "-c", "--center", action="store_true", help="Use the center RGB."
    )
    parser.add_argument(
        "-lr",
        "--left_right",
        action="store_true",
        help="Use the synced RGB left and right cameras.",
    )
    parser.add_argument(
        "-lrc",
        "--left_right_center",
        action="store_true",
        help="Use the synced RGB left and right cameras with the center camera.",
    )
    parser.add_argument(
        "-g", "--gripper", action="store_true", help="Use the gripper camera."
    )
    parser.add_argument(
        "--camera_name",
        type=str,
        default=None,
        help="Use the specified camera name. This name should match a name key in the RGBCameras enum.",
    )
    parser.add_argument(
        "--rectify",
        action="store_true",
        help="Rectify the RGB imagery. Default: False.",
    )
    parser.add_argument(
        "--crop",
        action="store_true",
        help="Crop the RGB imagery. Default: False.",
    )
    parser.add_argument(
        "--detect_aruco_marker_size",
        type=float,
        default=None,
        help="Runs ArUco detection if a float >= 0.0 is provided. Default: None. If length is 0.0, the ArUco markers will be detected, but distance will not be printed. If length > 0.0 and calibration is available, ArUco pose and L2 distance to the marker will be displayed.",
    )
    parser.add_argument(
        "--show_fps",
        action="store_true",
        help="Show the FPS of the stream. Default: False.",
    )
    parser.add_argument(
        "--use_ros_for_cameras",
        action="store_true",
        help="Use ros2 to subscribe to camera images, instead of using the python camera API. (Default: False)",
    )

    args = parser.parse_args()

    recording_directory = args.recording_directory

    show_fps = args.show_fps

    camera_type = None
    if args.camera_name:
        camera_type = RGBCameras[args.camera_name]
    elif args.left:
        camera_type = RGBCameras.left()
    elif args.right:
        camera_type = RGBCameras.right()
    elif args.center:
        camera_type = RGBCameras.center()
    elif args.left_right:
        camera_type = RGBCameras.synced_left_right()
    elif args.left_right_center:
        camera_type = RGBCameras.synced_left_right_center()
    elif args.gripper:
        camera_type = RGBCameras.gripper_rgbd
    else:
        raise Exception(
            "You must specify one of --left, --right, --center, --left_right, --left_right_center, --gripper, or --camera_name to specify the rgb camera to record."
        )
    is_record_to_rerun = args.rerun
    is_record_to_cvimshow = args.opencv
    is_crop = args.crop
    is_rectify = args.rectify

    detect_aruco_marker_size = args.detect_aruco_marker_size

    show_image_in = None
    if is_record_to_rerun:
        show_image_in = RecordRgbShowImageIn.RERUN
    elif is_record_to_cvimshow:
        show_image_in = RecordRgbShowImageIn.CVIMSHOW
    
    if not is_record_to_rerun and not is_record_to_cvimshow and not recording_directory:
        raise Exception("You should specify one of the following: --opencv, --rerun or --recording_directory to specify an output destination for the camera stream.")

    if args.use_ros_for_cameras:
        controller_class = RGBPipelineControllerROS
    else:
        controller_class = RGBPipelineController

    rgb_pipeline_controller = controller_class(
        camera_type=camera_type,
        recording_directory=recording_directory,
        show_image_in=show_image_in,
        is_rotate=True,
        is_rectify=is_rectify,
        is_crop=is_crop,
        ai_models_to_use=[],
        detect_aruco_marker_size=detect_aruco_marker_size,
    )

    loop_timer = LoopTimer()
    loop_timer.start_of_iteration()
    def print_loop_timer():
        if not show_fps:
            return
        loop_timer.end_of_iteration()
        loop_timer.pretty_print(minimum=True)
        loop_timer.start_of_iteration()
    if camera_type.is_synced_camera_type():
        for _ in rgb_pipeline_controller.get_frame_synced(is_run_pipeline=True):
            print_loop_timer()
            pass # do nothing, the pipeline will handle the user's pipeline configs
    else:
        for _ in rgb_pipeline_controller.get_frame(is_run_pipeline=True):
            print_loop_timer()
            pass # do nothing, the pipeline will handle the user's pipeline configs


if __name__ == "__main__":
    show_rgb()
