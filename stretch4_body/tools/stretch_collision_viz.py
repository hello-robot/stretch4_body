#!/usr/bin/env python3
import argparse

import stretch4_body.core.hello_utils as hu
from stretch4_body.core.mujoco_urdf import MujocoURDFCollisionViz
from stretch4_body.behavior.sentries.self_collision.self_collision_loop import SelfCollisionLoop
from stretch4_body.robot.robot_client import RobotClient as Robot
import time

hu.print_stretch_re_use()
import argparse

def get_collisions(robot):
    try:
        return robot.status['safety_layer']['sentry_manager']['sentry_self_collision']['collisions']
    except:
        hu.qprint("Could not get collision information. Is the self-collision sentry runnig?", fg="yellow")
        return {}

def cb_loop(collisions):
    jcfg = SelfCollisionLoop.get_urdf_joint_configuration(r.status)

    viz.update(jcfg, collisions)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize Stretch collision system ')
    #parser.add_argument("--mesh", help="View actual mesh models", action="store_true")
    parser.add_argument('-g', "--gamepad", help="Use gamepad to control pose", action="store_true")
    parser.add_argument('-i', "--show-ignored", help="Show ignored links", action="store_true")

    args = parser.parse_args()

    viz = MujocoURDFCollisionViz(robot_rgba=[1.0, 1.0, 1.0, 1.0], bg_rgba=[0.2, 0.2, 0.2, 1.0], show_ignored=args.show_ignored)
    r = Robot()
    try:
        if r.startup():
            print("Starting visualization loop...")
            r.pull_status()
            if not r.is_homed():
                print('Warning. Visualization may be inaccurate because the robot has not been calibrated')
                # exit()
            gamepad=None
            if args.gamepad:
                from stretch4_body.core.gamepad_teleop import GamePadTeleop
                gamepad_teleop = GamePadTeleop(robot=r, use_server=True,cb_loop=cb_loop)
                gamepad_teleop.startup()
                gamepad_teleop.mainloop()
            else:
                try:
                    while True:
                        r.pull_status()
                        hu.qprint(r.status['server']['cpu'])
                        collisions = get_collisions(r)
                        if len(collisions) > 0:
                            print(f"{collisions=}")
                        cb_loop(collisions)
                        time.sleep(0.01)
                except KeyboardInterrupt:
                    pass
    finally:
        r.stop()
        viz.stop()


