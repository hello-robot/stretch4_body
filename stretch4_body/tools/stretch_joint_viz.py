#!/usr/bin/env python3
import argparse
import time
import io
import rerun as rr
import yourdfpy

import stretch4_urdf
import stretch4_body.core.hello_utils as hu
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.robot.robot_client import RobotClient as Robot
from stretch4_body.behavior.sentries.self_collision.self_collision_loop import SelfCollisionLoop

def main():
    hu.print_stretch_re_use()
    
    parser = argparse.ArgumentParser(description='Visualize Stretch joint poses in rerun in real-time')
    parser.add_argument('--hide-joints', nargs='+', default=[], help='List of joints to hide from visualization')
    args = parser.parse_args()

    r = Robot()
    if not r.startup():
        print("Failed to start RobotClient.")
        return

    # Fetch URDF using RobotParams
    _, robot_params = RobotParams.get_params()
    model_name = robot_params['robot']['model_name']
    batch_name = robot_params['robot']['batch_name']
    eoa_name = robot_params['robot']['tool']

    try:
        urdf_contents = stretch4_urdf.get_urdf(model_name, batch_name, eoa_name, do_add_file_prefix_to_absolute_paths=False)
    except Exception as e:
        print(f"Failed to fetch URDF: {e}")
        r.stop()
        return

    # Parse URDF
    f = io.StringIO(urdf_contents)
    urdf = yourdfpy.URDF.load(f)

    # Filter out joints to hide
    joints_to_hide = set(args.hide_joints)
    visible_joints = []
    for j_name, joint in urdf.joint_map.items():
        if j_name not in joints_to_hide:
            visible_joints.append(joint)

    rr.init("stretch_joint_viz")
    rr.spawn(memory_limit="2GB")

    print("Starting visualization loop... Press Ctrl+C to stop.")
    
    # We will log arrows at each frame to visualize it
    arrows_origins = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    arrows_vectors = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]
    arrows_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]

    # Statically log meshes, labels, and frames
    for joint in visible_joints:
        # Log axes and label statically relative to the joint
        rr.log(f"robot/joints/{joint.name}/frame", rr.Arrows3D(
            origins=arrows_origins,
            vectors=arrows_vectors,
            colors=arrows_colors
        ), static=True)
        
        rr.log(f"robot/joints/{joint.name}/label", rr.Points3D(
            positions=[[0, 0, 0]],
            labels=[joint.name]
        ), static=True)

        # Log meshes
        link_name = joint.child
        if link_name in urdf.link_map:
            link = urdf.link_map[link_name]
            for i, visual in enumerate(link.visuals):
                if visual.geometry and visual.geometry.mesh and visual.geometry.mesh.filename:
                    mesh_path = f"robot/joints/{joint.name}/mesh_{i}"
                    if visual.origin is not None:
                        v_trans = visual.origin[0:3, 3]
                        v_mat = visual.origin[0:3, 0:3]
                        rr.log(mesh_path, rr.Transform3D(translation=v_trans, mat3x3=v_mat), static=True)
                    
                    try:
                        rr.log(mesh_path, rr.Asset3D(path=visual.geometry.mesh.filename, albedo_factor=[200, 200, 200, 100]), static=True)
                    except Exception as e:
                        print(f"Failed to log mesh {visual.geometry.mesh.filename}: {e}")

    try:
        while True:
            r.pull_status()
            
            # Use same utility as collision viz to get dictionary of joint states
            urdf_joint_state = SelfCollisionLoop.get_urdf_joint_configuration(r.status)
            
            # Update kinematics using yourdfpy
            try:
                urdf.update_cfg(urdf_joint_state)
            except Exception as e:
                pass
                
            # Log transform for each visible joint's child link
            for joint in visible_joints:
                link_name = joint.child
                try:
                    matrix, _ = urdf.scene.graph.get(link_name)
                    translation = matrix[0:3, 3]
                    mat3x3 = matrix[0:3, 0:3]
                    
                    rr.log(f"robot/joints/{joint.name}", rr.Transform3D(translation=translation, mat3x3=mat3x3))
                except Exception as e:
                    pass
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("Stopping visualization.")
    finally:
        r.stop()

if __name__ == "__main__":
    main()
