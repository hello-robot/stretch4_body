#!/usr/bin/env python3
import os
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
from stretch4_urdf import get_urdf

from stretch4_body.core.device import Device
from stretch4_body.subsystem.end_of_arm.gripper_conversion import *
from stretch4_body.utils.file_access_utils import (acquire_lock_if_available,
                                                     is_file_in_use,
                                                     setup_shared_directory)


@dataclass
class MujocoJointStates:
    """Represent joint states in Mujoco convention
    Facilitate conversion to/from URDF convention"""
    joint_lift: float
    joint_arm_l0: float
    joint_arm_l1: float
    joint_arm_l2: float
    joint_arm_l3: float

    joint_wrist_yaw: float
    joint_wrist_pitch: float
    joint_wrist_roll: float

    joint_gripper_slide: float = 0.0
    joint_gripper_finger_left: float = 0.0
    joint_gripper_finger_right: float = 0.0
    joint_finger_left: float = 0.0
    joint_finger_right: float = 0.0


    def to_dict(self) -> dict[str, float]:
        return asdict(self)

    def __repr__(self) -> str:
        fields = []
        for k, v in self.to_dict().items():
           if v is None: 
            fields.append(f"{k}={v}")
           else:
            fields.append(f"{k}={v:.2f}")   
        return "\n".join(fields)

    def from_urdf_joint_state(state: dict, robot_params=None):
        """
        Take in urdf joint state and convert to mujoco joint state convention
        eg state =
        {'joint_lift': 0.11316090153133451, 'joint_arm_l0': 0.0014814796174020114,
        'joint_arm_l1': 0.0014814796174020114, 'joint_arm_l2': 0.0014814796174020114,
        'joint_arm_l3': 0.0014814796174020114, 'joint_wrist_yaw': 0.06366020269725411,
        'joint_wrist_pitch': 0.12425244381873693, 'joint_wrist_roll': 0.07363107781851078}

        """
        if robot_params is None:
            d = Device(req_params=False)
            robot_params=d.robot_params

        jgfl = 0.0
        jgfr = 0.0
        jfl = 0.0
        jfr = 0.0
        if robot_params['robot']['tool']=='eoa_wrist_dw4_tool_sg4':
            jgfl = state.get("joint_gripper_finger_left")
            jgfr = state.get("joint_gripper_finger_right")
        elif robot_params['robot']['tool']=='eoa_wrist_dw4_tool_pg4':
            jfl = state.get("joint_finger_left")
            jfr = state.get("joint_finger_right")

        mujoco_state = MujocoJointStates(
            joint_lift=state.get("joint_lift", 0.0),
            joint_arm_l0=state.get("joint_arm_l0", 0.0),
            joint_arm_l1=state.get("joint_arm_l1", 0.0),
            joint_arm_l2=state.get("joint_arm_l2", 0.0),
            joint_arm_l3=state.get("joint_arm_l3", 0.0),
            joint_wrist_yaw=state.get("joint_wrist_yaw", 0.0),
            joint_wrist_pitch=state.get("joint_wrist_pitch", 0.0), #+math.radians(-45.0),#
            # Temporary pitch inversion until the URDF is changed to match stretch4_body joints
            joint_wrist_roll=state.get("joint_wrist_roll", 0.0), #+math.radians(-45.0)
            # Temporary pitch inversion until the URDF is changed to match stretch4_body joints
            joint_gripper_slide=0.0,  # state.get("joint_gripper_slide", 0.0),
            joint_gripper_finger_left=jgfl,
            joint_gripper_finger_right=jgfr,
            joint_finger_left=jfl,
            joint_finger_right=jfr,
        )
        return mujoco_state


def get_mujoco_collision_model_stretch4_urdf():
    """Required stretch4_urdf to be pip installed in the same python environment.
    If stretch4_body is installed, it will be used to get the batch and model name to load the correct URDF.
    Return path to built collision model
    """
    d = Device(req_params=False)
    
    model_name=d.robot_params['robot']['model_name']
    batch_name=d.robot_params['robot']['batch_name']
    eoa_name=d.robot_params['robot']['tool']

    urdf_contents = get_urdf(model_name, batch_name, eoa_name, do_add_file_prefix_to_absolute_paths=False)
    urdf_contents = urdf_contents.replace("<robot name=\"stretch\">", f'''
            <robot name="stretch">
            <mujoco>
            <compiler strippath="false" fusestatic="false"/>
            </mujoco>''', 1)


    robot_exclusions=d.robot_params['self_collision_mujoco'][model_name]['exclusions']
    eoa_exclusions=d.robot_params['self_collision_mujoco'][eoa_name]['exclusions']
    exclusions = "<contact>\n"

    for e in robot_exclusions+eoa_exclusions:
        exclusions = exclusions + f"<exclude body1=\"{e[0]}\" body2=\"{e[1]}\"/>\n"
    exclusions = exclusions +"</contact>\n"

    # Handle ignored links (disable collision)
    ignore_links = d.robot_params['self_collision_mujoco'][model_name].get('ignore_links', []) + \
                   d.robot_params['self_collision_mujoco'][eoa_name].get('ignore_links', [])

    spec = mujoco.MjSpec.from_string(urdf_contents)
    spec.compile()

    # Modify XML to disable collisions for ignored links
    root = ET.fromstring(spec.to_xml())
    
    # Handle ignored links (disable collision)
    ignore_links = d.robot_params['self_collision_mujoco'][model_name].get('ignore_links', []) + \
                    d.robot_params['self_collision_mujoco'][eoa_name].get('ignore_links', [])
    print(f"{ignore_links=}")
    for link_name in ignore_links:
        # Find body with name=link_name in the MJCF
        # MJCF structure: <worldbody> <body name="..."> ...
        # Search recursively
        found = False
        for body in root.iter('body'):
            if body.get('name') == link_name:
                found = True
                # Set contype=0 conaffinity=0 for all geoms in this body
                for geom in body.findall('geom'):
                    geom.set('contype', '0')
                    geom.set('conaffinity', '0')
        
        if not found:
            print(f"Warning: Ignored link '{link_name}' not found in MJCF.")

    xml_content = ET.tostring(root, encoding='unicode', method='xml')
    
    xml_content = xml_content.replace('</mujoco>', exclusions + '\n</mujoco>')

    return mujoco.MjModel.from_xml_string(xml_content)

class MujocoURDFCollisionViz(Device):
    def __init__(self, robot_params=None, robot_rgba=None, bg_rgba=None, show_ignored=False):
        Device.__init__(self, 'mujoco_urdf_viz', req_params=False)
        if robot_params:
            self.robot_params = robot_params
        
        # Color settings
        self.robot_rgba = robot_rgba if robot_rgba else [0.95, 0.85, 1.0, 1.0] # Lighter violet
        self.bg_rgba = bg_rgba if bg_rgba else [0.95, 0.95, 0.95, 1.0] # Off-white
        self.show_ignored = show_ignored
        
        self.model = None
        self.data = None
        self.viewer = None
        self._last_joint_states = None
        self._joint_qpos_indices = {}
        self.valid = False

        self.startup()

    def startup(self):
        self._load_model()
        self.valid = True
        
        # Launch viewer in background
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data, show_left_ui=False, show_right_ui=False)
        
        # Set camera view
        self.viewer.cam.lookat = [0.0, 0.0, 0.5]
        self.viewer.cam.distance = 4.0
        self.viewer.cam.azimuth = 140
        self.viewer.cam.elevation = -25
        
        return True


    def _load_model(self) -> None:
        self.model = get_mujoco_collision_model_stretch4_urdf()
        self.data = mujoco.MjData(self.model)
        
        # Store original colors
        self._original_colors = {}
        
        # Set background color
        self.model.vis.rgba.haze[:] = self.bg_rgba

        # Retrieve ignore links
        d = Device(req_params=False)
        model_name = d.robot_params['robot']['model_name']
        eoa_name = d.robot_params['robot']['tool']
        ignore_links = d.robot_params['self_collision_mujoco'][model_name].get('ignore_links', []) + \
                       d.robot_params['self_collision_mujoco'][eoa_name].get('ignore_links', [])
        ignore_links_set = set(ignore_links)
        
        for i in range(self.model.ngeom):
            # Set robot color to configurable value
            self.model.geom_rgba[i] = self.robot_rgba
            
            # Check if this geom belongs to an ignored link
            if not self.show_ignored:
                body_id = self.model.geom_bodyid[i]
                body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                if body_name in ignore_links_set:
                    self.model.geom_rgba[i] = [0, 0, 0, 0] # Invisible

            self._original_colors[i] = self.model.geom_rgba[i].copy()

        self._build_joint_qpos_indices()

    def _build_joint_qpos_indices(self) -> None:
        self._joint_qpos_indices = {}
        for i in range(self.model.njnt):
            joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if joint_name:
                self._joint_qpos_indices[joint_name] = self.model.jnt_qposadr[i]

    def _set_joint_states_in_mujoco(self, joint_states: MujocoJointStates) -> None:
        for joint_name, position in joint_states.to_dict().items():
            if joint_name in self._joint_qpos_indices:
                qpos_idx = self._joint_qpos_indices[joint_name]
                self.data.qpos[qpos_idx] = position

    def update(self, urdf_joint_state, contact_dict=None):
        """
        urdf_joint_state: dict of joint positions matching urdf joint state convention
             (e.g. {'joint_lift': 0.5, ...})
        contact_dict: dict of collisions, e.g. {'link_a': ['link_b'], ...}
                      Links in this dict will be highlighted red.
        """
        if not self.valid or not self.viewer.is_running():
            return

        # Update joint state
        try:
             # Adapt incoming dist to MujocoJointStates
            mjs = MujocoJointStates.from_urdf_joint_state(urdf_joint_state)
            self._set_joint_states_in_mujoco(mjs)
            mujoco.mj_forward(self.model, self.data)
        except Exception as e:
             print(f"MujocoURDFViz: Error updating state: {e}")



        # Highlight collisions
        if contact_dict:
            colliding_bodies = set(contact_dict.keys())
            #print('COLLIDING',colliding_bodies)
            # The contact_dict keys are BODY names in mujoco.
            # We need to find geoms belonging to these bodies.
            # However, typically simple mapping: verify body names.
            
            for i in range(self.model.ngeom):
                body_id = self.model.geom_bodyid[i]
                body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                
                if body_name in colliding_bodies:
                    self.model.geom_rgba[i] = [1.0, 0.5, 0.0, 1.0] # Orange
                else:
                    self.model.geom_rgba[i] = self._original_colors[i]
        else:
             # Reset all if no dict provided or empty
             for i in range(self.model.ngeom):
                self.model.geom_rgba[i] = self._original_colors[i]

        self.viewer.sync()

    def stop(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None


if __name__ == "__main__":
    import math

    # Verify with custom colors to check if params are working
    viz = MujocoURDFCollisionViz(robot_rgba=[1.0, 1.0, 1.0, 1.0], bg_rgba=[0.2, 0.2, 0.2, 1.0])
    
    print("Starting visualization loop...")
    try:
        t = 0
        while True:
            t += 0.01
            
            # Simulate some motion
            val = {
                'joint_lift': 0.5 + 0.3 * math.sin(t),
                'joint_arm_l0': 0.1, # Just base
                # Other arm joints will be zeroed if not set or handled by from_urdf_joint_state defaults
                'joint_wrist_yaw': math.cos(t),
                'gripper': 0.0
            }
            
            # Simulate collisions periodically
            collisions = {}
            if (int(t) % 2) == 0:
                collisions = {'link_lift': ['link_base'], 'link_head': ['link_mast']} # Dummy names
            
            viz.update(val, collisions)
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        viz.stop()
