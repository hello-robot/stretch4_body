#!/usr/bin/env python3
from stretch4_body.core.hello_utils import LoopTimer
import argparse
import os
import rerun as rr
import rerun.blueprint as rrb

from stretch4_body.subsystem.cameras.emulated_rgbd import (
    stream_left_rgbd,
    stream_right_rgbd,
    stream_center_rgbd,
    stream_left_right_rgbd,
    stream_left_right_center_rgbd,
    RGBDFrame,
    EmulatedRGBDStreamer,
)


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize colored point clouds and depth images from Stretch lidars and cameras in rerun."
    )

    # Camera selection flags
    parser.add_argument("-l", "--left", action="store_true", help="Display RGBD stream from left camera")
    parser.add_argument("-r", "--right", action="store_true", help="Display RGBD stream from right camera")
    parser.add_argument("-c", "--center", action="store_true", help="Display RGBD stream from center camera")
    parser.add_argument("-lr", "--left_right", action="store_true", help="Display RGBD streams from left and right cameras")
    parser.add_argument("-lrc", "--left_right_center", action="store_true", help="Display RGBD streams from all cameras")

    # Lidar selection flags
    parser.add_argument("--lidar_left", action="store_true", help="Use left lidar")
    parser.add_argument("--lidar_right", action="store_true", help="Use right lidar")

    parser.add_argument(
        "--show_fps",
        action="store_true",
        help="Show the FPS of the stream. Default: False.",
    )

    parser.add_argument(
        "--use_ros_for_lidars",
        action="store_true",
        help="Use ros2 to subscribe to lidar points. (Default: False)",
    )

    parser.add_argument(
        "--use_ros_for_cameras",
        action="store_true",
        help="Use ros2 to subscribe to camera images, instead of using the python camera API. (Default: False)",
    )

    return parser.parse_args()


def render_rgbd(c_name: str, frame: RGBDFrame):
    rr.log(f"Cameras/{c_name}_rotated", rr.Image(frame.image_frame.image, color_model="BGR").compress())
    rr.log(f"Cameras/{c_name}/rgb_raw", rr.Image(frame.image_frame.image_raw, color_model="BGR").compress())
    
    if frame.depth_image is not None and frame.depth_image.shape[0] > 0:
        rr.log(f"Cameras/{c_name}/depth", rr.DepthImage(frame.depth_image, meter=1.0))
        
    if len(frame.pointcloud) > 0:
        rr.log(
            f"Pointclouds/camera_frame/{c_name}",
            rr.Points3D(frame.pointcloud, colors=frame.pointcloud_colors, radii=[0.0025]),
        )
    if len(frame.pointcloud_base) > 0:
        rr.log(
            f"Pointclouds/base_frame/{c_name}",
            rr.Points3D(frame.pointcloud_base, colors=frame.pointcloud_colors, radii=[0.0025]),
        )
    ...

def main():
    args = _parse_args()

    show_fps = args.show_fps

    # Resolve camera flags
    use_left = args.left
    use_right = args.right
    use_center = args.center
    use_left_right = args.left_right
    use_left_right_center = args.left_right_center

    use_ros_for_cameras = args.use_ros_for_cameras
    use_ros_for_lidars = args.use_ros_for_lidars

    if not (use_left or use_right or use_center or use_left_right or use_left_right_center):
        use_left_right = True

    # Resolve lidar flags (both by default)
    use_both_lidars_default = not (args.lidar_left or args.lidar_right)
    use_left_lidar = args.lidar_left or use_both_lidars_default
    use_right_lidar = args.lidar_right or use_both_lidars_default

    print("Initializing RGBD Streamer with Lidars...")

    rr.init("Stretch RGBD Show", spawn=False)
    rr.spawn(memory_limit="2GiB")

    if use_left_right_center:
        blueprint = rrb.Blueprint(
            rrb.Vertical(
                rrb.Spatial3DView(name="Base Frame", origin="/", contents=["+ Pointclouds/base_frame/**"]),
                rrb.Horizontal(
                    rrb.Spatial2DView(name="Left Camera Rotated", origin="Cameras/left_rotated"),
                    rrb.Spatial2DView(name="Center Camera Rotated", origin="Cameras/center_rotated"),
                    rrb.Spatial2DView(name="Right Camera Rotated", origin="Cameras/right_rotated"),
                ),
                rrb.Horizontal(
                    rrb.Spatial2DView(name="Left Camera", origin="Cameras/left"),
                    rrb.Spatial2DView(name="Center Camera", origin="Cameras/center"),
                    rrb.Spatial2DView(name="Right Camera", origin="Cameras/right"),
                    visible=True
                ),
                row_shares=[3, 1, 1]
            ),
            collapse_panels=True
        )
    elif use_left_right:
        blueprint = rrb.Blueprint(
            rrb.Vertical(
                rrb.Horizontal(
                    rrb.Vertical(
                    rrb.Spatial2DView(name="Left Camera Rotated", origin="Cameras/left_rotated"),
                    rrb.Spatial2DView(name="Right Camera Rotated", origin="Cameras/right_rotated"),
                    ),
                    rrb.Vertical(
                    rrb.Spatial2DView(name="Left Camera", origin="Cameras/left"),
                    rrb.Spatial2DView(name="Right Camera", origin="Cameras/right"),
                    visible=False
                    ),
                    rrb.Spatial3DView(name="Base Frame", origin="/", contents=["+ Pointclouds/base_frame/**"]),
                column_shares=[1,1,5]
                ),
                row_shares=[3, 1]
            ),
            collapse_panels=True
        )
    else:
        camera_name = "left"
        if use_center:
            camera_name = "center"
        elif use_right:
            camera_name = "right"
        blueprint = rrb.Blueprint(
            rrb.Horizontal(
                rrb.Vertical(
                rrb.Spatial2DView(name="Camera Rotated", origin=f"Cameras/{camera_name}_rotated"),
                rrb.Spatial2DView(name="Depth Camera", origin=f"Cameras/{camera_name}"),
                ),
                rrb.Spatial3DView(name="Base Frame", origin="/", contents=["+ Pointclouds/base_frame/**"]),
            column_shares=[1,5]
            ),
            collapse_panels=True
        )


    rr.send_blueprint(blueprint)

    print("Streaming started. Ctrl+C to exit.")

    loop_timer = LoopTimer()
    loop_timer.start_of_iteration()
    def print_loop_timer():
        if not show_fps:
            return
        loop_timer.end_of_iteration()
        loop_timer.pretty_print(minimum=True)
        loop_timer.start_of_iteration()
    try:
        if use_left_right:
            for synced_frame in stream_left_right_rgbd(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_cameras=use_ros_for_cameras, use_ros_for_lidars=use_ros_for_lidars):
                if synced_frame.left: render_rgbd("left", synced_frame.left)
                if synced_frame.right: render_rgbd("right", synced_frame.right)
                print_loop_timer()
        elif use_left_right_center:
            for synced_frame in stream_left_right_center_rgbd(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_cameras=use_ros_for_cameras, use_ros_for_lidars=use_ros_for_lidars):
                if synced_frame.left: render_rgbd("left", synced_frame.left)
                if synced_frame.right: render_rgbd("right", synced_frame.right)
                if synced_frame.center: render_rgbd("center", synced_frame.center)
                print_loop_timer()
        elif use_left:
            for frame in stream_left_rgbd(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_cameras=use_ros_for_cameras, use_ros_for_lidars=use_ros_for_lidars):
                render_rgbd("left", frame)
                print_loop_timer()

        elif use_right:
            for frame in stream_right_rgbd(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_cameras=use_ros_for_cameras, use_ros_for_lidars=use_ros_for_lidars):
                render_rgbd("right", frame)
                print_loop_timer()

        elif use_center:
            for frame in stream_center_rgbd(use_left_lidar=use_left_lidar, use_right_lidar=use_right_lidar, use_ros_for_cameras=use_ros_for_cameras, use_ros_for_lidars=use_ros_for_lidars):
                # The center camera logging in rerun is really slow,
                # when logging is disabled, you should be able to get full 10hz rgbd
                # but with logging, this may drop to 3hz-5hz
                render_rgbd("center", frame)
                print_loop_timer()

    except KeyboardInterrupt:
        print("Stopping... (Force quitting due to background threads)")
        os._exit(0)
    except Exception as e:
        print(f"Stopping due to error: {e=}")
        raise e
    finally:
        EmulatedRGBDStreamer.get_instance().stop()


if __name__ == "__main__":
    main()
