#!/usr/bin/env python3
"""
This script is responsible for implementing the solver-wrapper for collision checking for the Stretch robot using MuJoCo.

You can run this script independently by using `robot_collision_mujoco.py --visualize` to visualize the robot in the given joint configuration. Use `--help` for more options.

Note: this typically requires `stretch4_mujoco` or `stretch4_urdf` to be installed in order to load the robot model. You can provide a custom model path using `--model` argument.

"""
import mujoco
import numpy as np
from stretch4_body.core.device import Device
import copy
from stretch4_body.core.mujoco_urdf import *


class SelfCollisionMujoco(Device):
    def __init__(self):
        Device.__init__(self, 'self_collision_mujoco')
        self.valid=False
        self._last_joint_states = None

    def startup(self):
        try:
            self._load_model()
            self.valid=True
            return True
        except:
            self.logger.error('SelfCollisionMujoco failed to load model')
            return False

    def _load_model(self) -> None:
        self.model = get_mujoco_collision_model_stretch4_urdf()
        self.data = mujoco.MjData(self.model)

        self._build_joint_qpos_indices()
        
    def _build_joint_qpos_indices(self) -> None:
        # Build joint name to qpos index mapping and dof index mapping
        self._joint_qpos_indices = {}
        self._joint_dof_indices = {}
        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name:
                qpos_adr = self.model.jnt_qposadr[i]
                dof_adr = self.model.jnt_dofadr[i]
                self._joint_qpos_indices[joint_name] = qpos_adr
                self._joint_dof_indices[joint_name] = dof_adr
    
    def _set_joint_states_in_mujoco(self, joint_states: MujocoJointStates) -> None:
        for joint_name, position in joint_states.to_dict().items():
            if joint_name in self._joint_qpos_indices:
                qpos_idx = self._joint_qpos_indices[joint_name]
                self.data.qpos[qpos_idx] = position

    def _update_mujoco_state(self, joint_states: MujocoJointStates) -> None:
        """
        Updates MuJoCo state if the joint configuration has changed.
        """
        if self._last_joint_states == joint_states:
            return

        self._set_joint_states_in_mujoco(joint_states)
        mujoco.mj_forward(self.model, self.data)
        self._last_joint_states = copy.deepcopy(joint_states)

    
    def get_collisions(self, joint_states: MujocoJointStates) -> dict[str, list[str]]:
        """
        Returns a dictionary of every link name that is in collision with another link and the list of links it is in collision with.
        """
        if not self.valid:
            return {}
        
        self._update_mujoco_state(joint_states)

        collisions: dict[str, list[str]] = {}

        # Iterate through all contacts
        for i in range(self.data.ncon):
            contact = self.data.contact[i]

            # Get the geom IDs involved in the contact
            geom1_id = contact.geom1
            geom2_id = contact.geom2
            body1_id = self.model.geom_bodyid[geom1_id]
            body2_id = self.model.geom_bodyid[geom2_id]
            body1_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body1_id)
            body2_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body2_id)

            if body1_name and body2_name:
                if body1_name == "world" or body2_name == "world":
                    continue

                # Add collision pair (bidirectional)
                if body1_name not in collisions:
                    collisions[body1_name] = []
                if body2_name not in collisions[body1_name]:
                    collisions[body1_name].append(body2_name)

                if body2_name not in collisions:
                    collisions[body2_name] = []
                if body1_name not in collisions[body2_name]:
                    collisions[body2_name].append(body1_name)
        #print('###########################################')
        #print('Collisions: ', collisions)
        return collisions

    def get_collision_directions(self, joint_states: MujocoJointStates) -> dict:
        """
        Returns a nested dictionary of collision gradients for each colliding pair.
        The gradient indicates the direction in joint space that moves body1 AWAY from body2.
        Result is { body1: { body2: { joint_name: gradient_value } } }
        """
        if not self.valid:
            return {}

        self._update_mujoco_state(joint_states)

        collision_directions = {}

        # Pre-allocate Jacobian arrays (3 x nv)
        # Note: self.model.nv is the number of degrees of freedom
        jac1 = np.zeros((3, self.model.nv))
        jac2 = np.zeros((3, self.model.nv))

        for i in range(self.data.ncon):
            contact = self.data.contact[i]

            # Get bodies
            geom1_id = contact.geom1
            geom2_id = contact.geom2
            body1_id = self.model.geom_bodyid[geom1_id]
            body2_id = self.model.geom_bodyid[geom2_id]
            body1_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body1_id)
            body2_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body2_id)

            if not body1_name or not body2_name:
                continue
            if body1_name == "world" or body2_name == "world":
                continue

            # Compute Jacobians for the contact point
            # usage: mj_jac(model, data, jacp, jacr, point, body)
            mujoco.mj_jac(self.model, self.data, jac1, None, contact.pos, body1_id)
            mujoco.mj_jac(self.model, self.data, jac2, None, contact.pos, body2_id)

            # Contact normal points from geom1 to geom2
            normal = contact.frame[:3]

            # Gradient of separation w.r.t q: (J2 - J1)^T * n
            grad = jac2.T @ normal - jac1.T @ normal

            # Function to accumulate gradients
            def add_direction(b_source, b_target, g):
                if b_source not in collision_directions:
                    collision_directions[b_source] = {}
                if b_target not in collision_directions[b_source]:
                    collision_directions[b_source][b_target] = {}

                for joint_name, dof_idx in self._joint_dof_indices.items():
                    # Check component corresponding to this joint
                    val = g[dof_idx]
                    if abs(val) > 1e-6:
                        current = collision_directions[b_source][b_target].get(joint_name, 0.0)
                        collision_directions[b_source][b_target][joint_name] = current + val

            # Add for both directions (symmetric)
            add_direction(body1_name, body2_name, grad)
            add_direction(body2_name, body1_name, grad)
        #print('CCC',collision_directions)
        #print('Collision directions: ', SelfCollisionMujoco.extract_collision_dirs(collision_directions))

        return collision_directions


    def visualize(self, joint_states: MujocoJointStates, highlight_collisions: bool = True, timeout:float|None = None, callback: callable = None, highlight_collision_directions: bool = False) -> None:
        """
        Visualize the robot in the given joint configuration using MuJoCo viewer.
        """
        import mujoco.viewer
        import numpy as np
        import time

        collisions = self.get_collisions(joint_states) if highlight_collisions else {}
        colliding_bodies = set(collisions.keys())

        if highlight_collisions:
             # Store original colors
            original_colors = {}
            for i in range(self.model.ngeom):
                original_colors[i] = self.model.geom_rgba[i].copy()

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                if callback:
                     joint_states=callback(joint_states)
                     if joint_states is not None:
                        self._update_mujoco_state(joint_states)

                        if highlight_collisions:
                            collisions = self.get_collisions(joint_states)
                            colliding_bodies = set(collisions.keys())
                            
                            # Update colors based on new collisions
                            for i in range(self.model.ngeom):
                                body_id = self.model.geom_bodyid[i]
                                body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                                if body_name in colliding_bodies:
                                    self.model.geom_rgba[i] = [1.0, 0.0, 0.0, 0.8] # Red for collision
                                else:
                                    # Restore original color
                                    self.model.geom_rgba[i] = original_colors[i]

                        if highlight_collision_directions:
                            viewer.user_scn.ngeom = 0
                            dirs = self.get_collision_directions(joint_states)
                            
                            # Aggregate gradients per joint
                            joint_grads = {}
                            for b1 in dirs:
                                for b2 in dirs[b1]:
                                    for jn, val in dirs[b1][b2].items():
                                        joint_grads[jn] = joint_grads.get(jn, 0.0) + val
                            
                            for jn, val in joint_grads.items():
                                if abs(val) < 1e-6:
                                    continue
                                    
                                # Find joint info
                                try:
                                    jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                                    if jid == -1: continue
                                    
                                    # Axis and anchor
                                    # For slide/hinge joints, xaxis is the direction
                                    axis = self.data.xaxis[jid]
                                    anchor = self.data.xanchor[jid]
                                    
                                    # Direction of the arrow: sign(val) * axis
                                    direction = np.sign(val) * axis
                                    
                                    # Align arrow with direction
                                    # mjv_initGeom requires a rotation matrix (9 floats)
                                    # We can assume standard arrow points up (Z) or X?
                                    # Usually mjGEOM_ARROW points along Z.
                                    # We need to construct rotation from Z to direction.
                                    
                                    # Simple approach: use zaxis as default, rotate to match direction
                                    z_vec = np.array([0, 0, 1.0])
                                    if np.linalg.norm(direction) < 1e-6: continue
                                    
                                    # Helper to compute rotation matrix from a to b
                                    def rotation_matrix_from_vectors(vec1, vec2):
                                        """ Find the rotation matrix that aligns vec1 to vec2
                                        :param vec1: A 3d "source" vector
                                        :param vec2: A 3d "destination" vector
                                        :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
                                        """
                                        a, b = (vec1 / np.linalg.norm(vec1)), (vec2 / np.linalg.norm(vec2))
                                        v = np.cross(a, b)
                                        c = np.dot(a, b)
                                        s = np.linalg.norm(v)
                                        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
                                        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2 + 1e-9))
                                        return rotation_matrix

                                    if np.allclose(direction, z_vec):
                                        mat = np.eye(3)
                                    elif np.allclose(direction, -z_vec):
                                        mat = np.diag([1, -1, -1]) # Flip Z
                                    else:
                                        mat = rotation_matrix_from_vectors(z_vec, direction)
                                    
                                    # Add arrow to scene
                                    mujoco.mjv_initGeom(
                                        viewer.user_scn.geoms[viewer.user_scn.ngeom],
                                        type=mujoco.mjtGeom.mjGEOM_ARROW,
                                        size=np.array([0.02, 0.2, 0.05]), # Radius, length
                                        pos=anchor,
                                        mat=mat.flatten(),
                                        rgba=np.array([0.0, 1.0, 0.0, 1.0]) # Green
                                    )
                                    viewer.user_scn.ngeom += 1
                                    
                                except Exception as e:
                                    self.logger.error(f"Error visualizing arrow for {jn}: {e}")


                viewer.sync()
                time.sleep(timeout or 0.01) # Faster update than 0.1 for smooth animation
                if timeout:
                    break

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Check collisions for Stretch robot")
    parser.add_argument("--model", type=str, default=None,
                        help="Path to stretch.xml model")
    parser.add_argument("--lift", type=float, default=0.5, help="Lift position [0.0, 1.1]")
    parser.add_argument("--arm", type=float, default=0.0, help="Arm extension [0, 0.52]")
    parser.add_argument("--wrist_yaw", type=float, default=0.0, help="Wrist yaw [-1.39, 4.42]")
    parser.add_argument("--wrist_pitch", type=float, default=0.0, help="Wrist pitch [-1.57, 0.56]")
    parser.add_argument("--wrist_roll", type=float, default=0.0, help="Wrist roll [-3.14, 3.14]")
    parser.add_argument("--visualize", "-v", action="store_true",
                        help="Open MuJoCo viewer to visualize the configuration")
    args = parser.parse_args()


    solver = SelfCollisionMujoco()
    solver.startup()

    # Create joint states (arm segments share the same extension)
    arm_per_segment = args.arm / 4.0  # Total arm divided by 4 segments
    joint_states = MujocoJointStates(
        joint_lift=args.lift,
        joint_arm_l0=arm_per_segment,
        joint_arm_l1=arm_per_segment,
        joint_arm_l2=arm_per_segment,
        joint_arm_l3=arm_per_segment,
        joint_wrist_yaw=args.wrist_yaw,
        joint_wrist_pitch=args.wrist_pitch,
        joint_wrist_roll=args.wrist_roll,
        joint_gripper_slide=0.0,
        joint_gripper_finger_left=0.0,
        joint_gripper_finger_right=0.0,
    )

    joint_states = MujocoJointStates.from_urdf_joint_state(
      {'joint_lift': 0.42120704119986957, 'joint_arm_l0': 0.0008689873049044256, 'joint_arm_l1': 0.0008689873049044256, 'joint_arm_l2': 0.0008689873049044256, 'joint_arm_l3': 0.0008689873049044256, 'joint_wrist_yaw': 1.4035924209153616, 'joint_wrist_pitch': 2.8723790253158628, 'joint_wrist_roll': -3.0833013836501384, 'gripper': 0.48013598660820567}
    )


    print(f"""Checking collisions with joint states:
          
{joint_states}
""")

    collisions = solver.get_collisions(joint_states)

    if collisions:
        print("Collisions detected:")
        for body, colliding_with in collisions.items():
            print(f"  {body} <-> {', '.join(colliding_with)}")
    else:
        print("No collisions detected")

    if args.visualize:
        solver.visualize(joint_states, highlight_collisions=True)

