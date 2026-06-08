#!/usr/bin/env python3
import time
import sys
import os
import argparse
import numpy as np
import stretch4_body

try:
    import open3d as o3d
except ImportError:
    print("Error: Open3D not installed. Please install it to use this tool.")
    sys.exit(1)

from stretch4_body.subsystem.line_sensor.line_sensor_loop import LineSensorLoop
from stretch4_body.subsystem.line_sensor.line_sensor_utils import LineSensorGeometry, LineSensorCalibration, LineSensorClusterTracker, LineSensorCostMap

class LineSensorVisualizer3D:
    def __init__(self, show_annotations=False, sensors=None, show_clusters=False, show_cost_map=False, use_calibration=True,
                 nice_viz=False, turntable=False, use_odom=False, bg_color=[0.2, 0.22, 0.3], sensor_color=[1.0, 1.0, 0.0], thickness=6.0):
        self.show_annotations = show_annotations
        self.sensors_to_show = sensors
        self.show_clusters = show_clusters
        self.show_cost_map = show_cost_map
        self.use_calibration = use_calibration
        self.nice_viz = nice_viz
        self.turntable = turntable
        self.use_odom = use_odom
        self.bg_color = bg_color
        self.sensor_color = sensor_color
        self.thickness = thickness

        if self.use_odom and self.nice_viz:
            from stretch4_body.robot.robot_client import RobotClient
            self.robot_client = RobotClient()
            if not self.robot_client.startup():
                print("Failed to start RobotClient for odometry")
                sys.exit(1)

        # Start LineSensorLoop
        self.lsl = LineSensorLoop()
        if not self.lsl.startup():
            print("Failed to start LineSensorLoop")
            sys.exit(1)
            
        # Update Params
        p = self.lsl.params
        ls_geom = p.get('line_sensor_geometry', {})
        ls_cost = p.get('line_sensor_cost_map', {})
        ls_tracker = p.get('line_sensor_cluster_tracker', {})
        
        self.param_height_cm = ls_geom.get('sensor_height_above_floor_mm', 69.4) / 10.0
        self.param_diameter_cm = ls_geom.get('sensor_pitch_diameter_mm', 400.0) / 10.0
        
        self.param_emitter_height_cm = ls_geom.get('emitter_height_above_floor_mm', 100.67) / 10.0
        self.param_emitter_diameter_cm = ls_geom.get('emitter_pitch_diameter_mm', 404.04) / 10.0
        
        # Geometry Helper
        self.geom = LineSensorGeometry(ls_geom)

        # Calibration Helper
        self.calibration = LineSensorCalibration(self.lsl)
        self.calibration.load_latest_tare()
        
        # Tracker for Clustering
        self.tracker = LineSensorClusterTracker(ls_tracker)
        
        # Traversability / CostMap
        self.cost_map = LineSensorCostMap(ls_cost)
        self.donut_geometry = o3d.geometry.TriangleMesh()
        
        self.max_cost_map_dist = 0.5
        # Donut Map State (Linear Ramp)
        # List of 72 wedges (360 / 5)
        # Each: current_dist (float)
        self.wedge_state = [self.max_cost_map_dist] * int(360 / 5)
        self.last_loop_time = time.time()
        
        # Determine sensors to show
        all_sensors = p.get('sensor_names', [])
        if self.sensors_to_show is None:
            self.sensors_to_show = all_sensors
        else:
            # Validate
            self.sensors_to_show = [s for s in self.sensors_to_show if s in all_sensors]
            if not self.sensors_to_show:
                print("Warning: No valid sensors specified. Showing all.")
                self.sensors_to_show = all_sensors
        
        # Initialize Visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='Stretch Line Sensor 3D', width=1280, height=720)
        
        opt = self.vis.get_render_option()
        if self.nice_viz:
            opt.background_color = np.asarray(self.bg_color)
            opt.point_size = self.thickness
            opt.line_width = self.thickness
        else:
            opt.background_color = np.asarray([1.0, 1.0, 1.0])
            opt.point_size = self.thickness / 2.0
            opt.line_width = self.thickness / 2.0
        
        if self.show_cost_map:
            self.vis.add_geometry(self.donut_geometry)
        
        # Rotation Matrices
        self.R_ccw = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, np.pi / 2))
        
        # Add Robot to Scene
        self.add_robot_to_scene()

    def add_robot_to_scene(self):
        p = self.lsl.params
        
        # 1. Define Visualizaton Rotation
        # We want to rotate the robot system 180 degrees (flip back) AND 
        # rotate it -90 degrees (CW) for the view, as per original code.
        # Combined: 180 - 90 = 90 degrees (CCW).
        R_viz = self.R_ccw
        
        # Robot Translation Offset (x, y, z)
        # Raises robot 27mm off the ground
        robot_translate = (0, 0, 0.027)

        # 2. Add Base Link
        stl_path = os.path.join(os.path.dirname(stretch4_body.__file__), 'media', 'base_link.STL')
        if os.path.exists(stl_path):
            print(f"Loading STL from: {stl_path}")
            robot_mesh = o3d.io.read_triangle_mesh(stl_path)
            robot_mesh.compute_vertex_normals()
            robot_mesh.paint_uniform_color([0.8, 0.9, 1.0]) # Very Light Blue
            # Apply Viz Rotation
            robot_mesh.rotate(R_viz, center=(0, 0, 0))
            # Apply Translation Offset
            robot_mesh.translate(robot_translate)
            self.vis.add_geometry(robot_mesh)
        else:
            print(f"Warning: STL not found at {stl_path}")

        # 3. Add Wheels (URDF)
        # URDF Transforms
        wheel_configs = [
            {
                'name': 'wheel_0_link.STL',
                'xyz': [0.150688420258492, 0.0869999999999999, 0.0715000000000003],
                'rpy': [-1.5707963267949, 0, -1.0471975511966]
            },
            {
                'name': 'wheel_1_link.STL',
                'xyz': [-0.150688420258491, 0.0870000000000015, 0.0715000000000002],
                'rpy': [-1.5707963267949, 0, 1.04719755119659]
            },
            {
                'name': 'wheel_2_link.STL',
                'xyz': [0, -0.174, 0.0715],
                'rpy': [-1.5707963267949, 0, -3.14159265358979]
            }
        ]

        self.wheel_data = []
        for i, wc in enumerate(wheel_configs):
            w_name = wc['name']
            w_path = os.path.join(os.path.dirname(stretch4_body.__file__), 'media', w_name)
            if os.path.exists(w_path):
                 w_mesh = o3d.io.read_triangle_mesh(w_path)
                 w_mesh.compute_vertex_normals()
                 w_mesh.paint_uniform_color([0.15, 0.15, 0.15]) # Dark Charcoal
                 
                 # 1. Apply Joint Rotation (RPY)
                 # standard URDF rpy is Rz(y) * Ry(p) * Rx(r)
                 roll, pitch, yaw = wc['rpy']
                 Rx = o3d.geometry.get_rotation_matrix_from_xyz((roll, 0, 0))
                 Ry = o3d.geometry.get_rotation_matrix_from_xyz((0, pitch, 0))
                 Rz = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, yaw))
                 R_joint = Rz @ Ry @ Rx
                 
                 w_mesh.rotate(R_joint, center=(0, 0, 0))
                 
                 # 2. Apply Joint Translation (XYZ)
                 w_mesh.translate(wc['xyz'])
                 
                 # 3. Apply Visualization Rotation (Matches Base Link)
                 # Rotate the entire assembly (including position) around global origin
                 w_mesh.rotate(R_viz, center=(0, 0, 0))
                 
                 # Apply Translation Offset
                 w_mesh.translate(robot_translate)

                 self.vis.add_geometry(w_mesh)
                 
                 # Save wheel data for animation
                 center = R_viz @ np.array(wc['xyz']) + np.array(robot_translate)
                 axis = R_viz @ R_joint @ np.array([0.0, 0.0, 1.0])
                 self.wheel_data.append({
                     'mesh': w_mesh,
                     'center': center,
                     'axis': axis,
                     'name': f'wheel_{i}'
                 })
            else:
                 print(f"Warning: STL not found at {w_path}")
            
        # Initialize Sensor Point Clouds
        self.sensor_clouds = []
        for i in range(6):
            pcd = o3d.geometry.PointCloud()
            # Initialize with empty points to add to visualizer
            pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))
            if self.nice_viz:
                pcd.paint_uniform_color(self.sensor_color)
            else:
                pcd.paint_uniform_color([0, 0, 1]) # 3D points in blue
            
            # Check if this sensor should be visible
            s_name = p.get('sensor_names', [])[i]
            if s_name in self.sensors_to_show:
                self.vis.add_geometry(pcd)
            
            self.sensor_clouds.append(pcd)
        
        self.setup_sensors()
        if self.nice_viz:
            self.setup_studio_plane()
        self.setup_grid()
        
        # Helper for cluster visualization
        self.cluster_cloud = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.cluster_cloud)

        if self.show_annotations:
            self.add_annotations()

        
    def add_annotations(self):
        # Params for labels
        p = self.lsl.params
        
        # Add Coordinate Frame for Reference

        # X=Red, Y=Green, Z=Blue
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
        self.vis.add_geometry(mesh_frame)
        
        # Add Annotations at z=0.3m
        z_annot = 0.3
        
        # 1. Circle (Robot Base)
        # Radius 0.2m (Diameter 40cm)
        radius = self.param_diameter_cm / 200.0 # cm -> m
        num_circle_pts = 64
        circle_pts = []
        for i in range(num_circle_pts):
            theta = 2 * np.pi * i / num_circle_pts
            x = radius * np.cos(theta)
            y = radius * np.sin(theta)
            circle_pts.append([x, y, z_annot])
        
        circle_lines = [[i, (i + 1) % num_circle_pts] for i in range(num_circle_pts)]
        circle_ls = o3d.geometry.LineSet()
        circle_ls.points = o3d.utility.Vector3dVector(circle_pts)
        circle_ls.lines = o3d.utility.Vector2iVector(circle_lines)
        circle_ls.paint_uniform_color([0.2, 0.2, 0.2]) # Dark Grey
        circle_ls.rotate(self.R_ccw, center=(0, 0, 0))
        self.vis.add_geometry(circle_ls)
        
        # 2. X Arrow (Forward)
        arrow_len = 0.3
        # Line
        x_pts = [[0, 0, z_annot], [arrow_len, 0, z_annot]]
        x_lines = [[0, 1]]
        x_ls = o3d.geometry.LineSet()
        x_ls.points = o3d.utility.Vector3dVector(x_pts)
        x_ls.lines = o3d.utility.Vector2iVector(x_lines)
        x_ls.paint_uniform_color([0, 0, 0]) # Black
        x_ls.rotate(self.R_ccw, center=(0, 0, 0))
        self.vis.add_geometry(x_ls)
        
        # Arrowhead X
        # Simple triangle or lines? Lines are easier to construct
        tip = np.array([arrow_len, 0, z_annot])
        head_len = 0.05
        # Wings at +/- 135 deg
        wing1 = np.array([arrow_len - head_len, head_len/2, z_annot])
        wing2 = np.array([arrow_len - head_len, -head_len/2, z_annot])
        
        head_pts = [tip, wing1, wing2]
        head_lines = [[0, 1], [0, 2]]
        head_ls = o3d.geometry.LineSet()
        head_ls.points = o3d.utility.Vector3dVector(head_pts)
        head_ls.lines = o3d.utility.Vector2iVector(head_lines)
        head_ls.paint_uniform_color([0, 0, 0])
        head_ls.rotate(self.R_ccw, center=(0, 0, 0))
        self.vis.add_geometry(head_ls)
        
        # Label 'X'
        # Draw lines relative to tip
        char_size = 0.05
        # Center of char
        xc = arrow_len + char_size
        yc = 0
        
        # X shape: 2 crossed lines
        x_char_pts = [
            [xc - char_size/2, yc - char_size/2, z_annot],
            [xc + char_size/2, yc + char_size/2, z_annot],
            [xc - char_size/2, yc + char_size/2, z_annot],
            [xc + char_size/2, yc - char_size/2, z_annot]
        ]
        x_char_lines = [[0, 1], [2, 3]]
        x_char_ls = o3d.geometry.LineSet()
        x_char_ls.points = o3d.utility.Vector3dVector(x_char_pts)
        x_char_ls.lines = o3d.utility.Vector2iVector(x_char_lines)
        x_char_ls.paint_uniform_color([0, 0, 0])
        x_char_ls.rotate(self.R_ccw, center=(0, 0, 0))
        self.vis.add_geometry(x_char_ls)
        
        # 3. Y Arrow (Left)
        # Line
        y_pts = [[0, 0, z_annot], [0, arrow_len, z_annot]]
        y_lines = [[0, 1]]
        y_ls = o3d.geometry.LineSet()
        y_ls.points = o3d.utility.Vector3dVector(y_pts)
        y_ls.lines = o3d.utility.Vector2iVector(y_lines)
        y_ls.paint_uniform_color([0, 0, 0]) # Black
        y_ls.rotate(self.R_ccw, center=(0, 0, 0))
        self.vis.add_geometry(y_ls)
        
        # Arrowhead Y
        tip_y = np.array([0, arrow_len, z_annot])
        wing1_y = np.array([head_len/2, arrow_len - head_len, z_annot])
        wing2_y = np.array([-head_len/2, arrow_len - head_len, z_annot])
        
        head_y_pts = [tip_y, wing1_y, wing2_y]
        head_y_lines = [[0, 1], [0, 2]]
        head_y_ls = o3d.geometry.LineSet()
        head_y_ls.points = o3d.utility.Vector3dVector(head_y_pts)
        head_y_ls.lines = o3d.utility.Vector2iVector(head_y_lines)
        head_y_ls.paint_uniform_color([0, 0, 0])
        head_y_ls.rotate(self.R_ccw, center=(0, 0, 0))
        self.vis.add_geometry(head_y_ls)

        # Label 'Y'
        yc_l = arrow_len + char_size
        xc_l = 0
        
        # Y shape
        y_char_pts = [
            [xc_l - char_size/2, yc_l + char_size/2, z_annot], # Top Left
            [xc_l, yc_l, z_annot], # Center
            [xc_l + char_size/2, yc_l + char_size/2, z_annot], # Top Right
            [xc_l, yc_l - char_size/2, z_annot] # Bottom
        ]
        y_char_lines = [[0, 1], [2, 1], [1, 3]]
        y_char_ls = o3d.geometry.LineSet()
        y_char_ls.points = o3d.utility.Vector3dVector(y_char_pts)
        y_char_ls.lines = o3d.utility.Vector2iVector(y_char_lines)
        y_char_ls.rotate(self.R_ccw, center=(0, 0, 0))
        self.vis.add_geometry(y_char_ls)
        
        # --- 4. Sensor Ground Frames ---
        sensor_names = p.get('sensor_names', [])
        for i, s_name in enumerate(sensor_names):
            if s_name not in self.sensors_to_show:
                continue
                
            # Get Transform
            T = self.geom.get_sensor_ground_frame(i)
            
            # Create Frame
            # X=Red, Y=Green, Z=Blue
            # origin=[0,0,0] creates frame at origin. We transform it to T.
            mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.15, origin=[0, 0, 0])
            
            # Apply T
            mesh_frame.transform(T)
            
            # Apply Global Viz Rotation
            mesh_frame.rotate(self.R_ccw, center=(0, 0, 0))
            
            self.vis.add_geometry(mesh_frame)

        
        
    def setup_grid(self):
        self.grid_size = 6.0 # meters (3m in all directions)
        self.grid_spacing = 0.20 # 200mm
        self.grid_z_height = 0.001 if self.nice_viz else 0.0
        
        self.grid_ls = o3d.geometry.LineSet()
        self._grid_added = False
        self.update_grid(0.0, 0.0, 0.0)
        self.grid_ls.paint_uniform_color([0.8, 0.8, 0.8]) # Light Grey
        self.vis.add_geometry(self.grid_ls)
        self._grid_added = True
        
    def update_grid(self, odom_x, odom_y, odom_theta):
        half_size = self.grid_size / 2.0
        spacing = self.grid_spacing
        
        # Center of the generated grid in world frame
        cx = np.round(odom_x / spacing) * spacing
        cy = np.round(odom_y / spacing) * spacing
        
        grid_pts_w = []
        grid_lines = []
        
        num_lines = int(self.grid_size / spacing) + 1
        
        # X terms
        for i in range(num_lines):
            x_val = cx - half_size + i * spacing
            idx = len(grid_pts_w)
            grid_pts_w.append([x_val, cy - half_size, self.grid_z_height])
            grid_pts_w.append([x_val, cy + half_size, self.grid_z_height])
            grid_lines.append([idx, idx+1])
            
        # Y terms
        for i in range(num_lines):
            y_val = cy - half_size + i * spacing
            idx = len(grid_pts_w)
            grid_pts_w.append([cx - half_size, y_val, self.grid_z_height])
            grid_pts_w.append([cx + half_size, y_val, self.grid_z_height])
            grid_lines.append([idx, idx+1])
            
        pts_w = np.array(grid_pts_w)
        
        # Transform to robot frame
        pts_w[:, 0] -= odom_x
        pts_w[:, 1] -= odom_y
        
        cos_t = np.cos(-odom_theta)
        sin_t = np.sin(-odom_theta)
        
        pts_r = np.zeros_like(pts_w)
        pts_r[:, 0] = pts_w[:, 0] * cos_t - pts_w[:, 1] * sin_t
        pts_r[:, 1] = pts_w[:, 0] * sin_t + pts_w[:, 1] * cos_t
        pts_r[:, 2] = pts_w[:, 2]
        
        self.grid_ls.points = o3d.utility.Vector3dVector(pts_r)
        
        # If this is the first time, set the lines
        if len(np.asarray(self.grid_ls.lines)) == 0:
            self.grid_ls.lines = o3d.utility.Vector2iVector(grid_lines)
            
        # Apply visualization coordinate rotation
        self.grid_ls.rotate(self.R_ccw, center=(0, 0, 0))
        
        if getattr(self, '_grid_added', False):
            self.vis.update_geometry(self.grid_ls)

    def setup_studio_plane(self):
        # A nicely shaded plane that the robot sits on with a light gradient
        res = 60
        size = 6.0
        depth = 0.1 # 10cm depth floor
        vertices = []
        triangles = []
        colors = []
        
        c_center = np.array([0.7, 0.72, 0.75]) # Light grey with a touch of blue
        c_edge = np.array(self.bg_color)       # Fade into background
        c_side = c_edge * 0.9 # Slightly darker for sides
        
        for i in range(res):
            for j in range(res):
                x = (i / (res - 1) - 0.5) * size
                y = (j / (res - 1) - 0.5) * size
                z = 0.0 # sit exactly at z=0
                vertices.append([x, y, z])
                
                # Distance from center
                dist = np.sqrt(x**2 + y**2)
                max_dist = size / 2.0
                
                # Smoothstep blending
                t = np.clip(dist / max_dist, 0.0, 1.0)
                # Apply a slight curve to gradient
                factor = t * t * (3.0 - 2.0 * t) 
                
                c = c_center * (1.0 - factor) + c_edge * factor
                colors.append(c)
                
        for i in range(res - 1):
            for j in range(res - 1):
                idx = i * res + j
                triangles.append([idx, idx + 1, idx + res])
                triangles.append([idx + 1, idx + res + 1, idx + res])
                
        # Add side walls to give the floor depth
        def add_wall(p1, p2, p3, p4, color):
            # Assumes p1, p2, p3, p4 are in CCW order when viewed from outside
            base_idx = len(vertices)
            vertices.extend([p1, p2, p3, p4])
            colors.extend([color]*4)
            triangles.append([base_idx, base_idx+1, base_idx+2])
            triangles.append([base_idx, base_idx+2, base_idx+3])

        hs = size / 2.0
        # -Y face
        add_wall([hs, -hs, -depth], [hs, -hs, 0], [-hs, -hs, 0], [-hs, -hs, -depth], c_side)
        # +Y face
        add_wall([-hs, hs, -depth], [-hs, hs, 0], [hs, hs, 0], [hs, hs, -depth], c_side)
        # -X face
        add_wall([-hs, -hs, -depth], [-hs, -hs, 0], [-hs, hs, 0], [-hs, hs, -depth], c_side)
        # +X face
        add_wall([hs, hs, -depth], [hs, hs, 0], [hs, -hs, 0], [hs, -hs, -depth], c_side)
                
        plane = o3d.geometry.TriangleMesh()
        plane.vertices = o3d.utility.Vector3dVector(vertices)
        plane.triangles = o3d.utility.Vector3iVector(triangles)
        plane.vertex_colors = o3d.utility.Vector3dVector(colors)
        plane.compute_vertex_normals()
        
        plane.rotate(self.R_ccw, center=(0, 0, 0))
        self.vis.add_geometry(plane)

        
    def setup_sensors(self):
        p = self.lsl.params
        ls_geom = p.get('line_sensor_geometry', {})

        R_m = (self.param_diameter_cm / 2.0) / 100.0
        z_s = self.param_height_cm / 100.0
        
        # Params
        sensor_angles = ls_geom.get('sensor_angles_deg', [10.18, 39.64, 80.36, 39.64, 80.36, 39.64])
        sensor_normals = ls_geom.get('sensor_normals_deg', [0.0, 60.0, 120.0, 180.0, 240.0, 300.0])
        
        current_pos_angle_deg = 0.0
        
        sensor_names = p.get('sensor_names', [])
        
        for i in range(6):
            # Check visibility
            if i < len(sensor_names):
                s_name = sensor_names[i]
                if s_name not in self.sensors_to_show:
                    continue
            
            # 1. Calculate Position Angle (Location on Ring)
            # Cumulative sum of increments
            angle_incr = sensor_angles[i]
            current_pos_angle_deg += angle_incr
            
            # Robot Frame: CW from Forward -> Negative angle in Standard Math
            pos_angle_rad = -np.deg2rad(current_pos_angle_deg)
            
            # 2. Calculate Orientation Angle (Facing Direction)
            # Absolute angle CW from Forward
            rot_angle_deg = sensor_normals[i]
            rot_angle_rad = -np.deg2rad(rot_angle_deg)
            
            # Position in Robot Frame
            x = R_m * np.cos(pos_angle_rad)
            y = R_m * np.sin(pos_angle_rad)
            z = z_s
            
            # 1. Calculate points in Sensor Local Frame (X is Normal/Forward for sensor)
            # Emitter is at Sensor Pos.
            # We want to draw lines starting from Sensor Pos, oriented by Rot Angle.
            
            # Emitter Params
            R_m_emitter = (self.param_emitter_diameter_cm / 2.0) / 100.0
            z_s_emitter = self.param_emitter_height_cm / 100.0
            
            # Recalculate Emitter Position based on Emitter Diameter
            # (Use same angle as sensor, assuming concentric alignment)
            sx_e = R_m_emitter * np.cos(pos_angle_rad)
            sy_e = R_m_emitter * np.sin(pos_angle_rad)
            z_e = z_s_emitter

            # Pitch Angle
            angle_down_deg = ls_geom.get('sensor_angle_down_deg', 26.0)
            angle_down_rad = np.deg2rad(angle_down_deg)
            
            # Emitter FOV for Geometry
            emitter_fov_deg = ls_geom.get('emitter_horizontal_fov_degrees', 110.5)
            fov_rad_emitter = np.deg2rad(emitter_fov_deg)
            
            # Calculate floor points in "Face Frame" (Origin at Sensor Pos, X aligned with Normal)
            
            # Dist along floor normal
            dist_f = z_s_emitter / np.tan(angle_down_rad)
            
            # Slant range
            slant_dist = z_s_emitter / np.sin(angle_down_rad)
            
            # Half width on floor (Emitter FOV)
            w_half = slant_dist * np.tan(fov_rad_emitter / 2.0)
            
            # Vector to Center Impact (relative to Emitter):
            # Down by z_s, Forward by dist_f.
            # In Sensor Frame (X=Forward, Z=Up):
            v_center = np.array([dist_f, 0, -z_s_emitter])
            
            # Vector to Left Edge:
            v_left = np.array([dist_f, w_half, -z_s_emitter])
            
            # Vector to Right Edge:
            v_right = np.array([dist_f, -w_half, -z_s_emitter])
            
            # Rotate vectors by Sensor Orientation (rot_angle_rad)
            def rot_z_vec(v, angle):
                xr = v[0] * np.cos(angle) - v[1] * np.sin(angle)
                yr = v[0] * np.sin(angle) + v[1] * np.cos(angle)
                zr = v[2]
                return np.array([xr, yr, zr])
                
            v_center_r = rot_z_vec(v_center, rot_angle_rad)
            v_left_r = rot_z_vec(v_left, rot_angle_rad)
            v_right_r = rot_z_vec(v_right, rot_angle_rad)
            
            # Absolute P_emitter (Calculated above as x,y,z)
            p_emitter = np.array([sx_e, sy_e, z_e])
            
            # Absolute Points on Floor
            p_center = p_emitter + v_center_r
            p_left = p_emitter + v_left_r
            p_right = p_emitter + v_right_r
            
            # Create Lines
            points = [p_emitter, p_center, p_left, p_right]
            lines = [
                [0, 1], # Emitter -> Center
                [0, 2], # Emitter -> Left
                [0, 3], # Emitter -> Right
                [2, 1], [1, 3] # Left -> Center -> Right
            ]
            
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector(points)
            line_set.lines = o3d.utility.Vector2iVector(lines)
            line_set.paint_uniform_color([1, 0, 0]) # Red
            
            # Apply Global Viz Rotation
            line_set.rotate(self.R_ccw, center=(0, 0, 0))
            
            self.vis.add_geometry(line_set)
            
            # --- Sensor Label (S{i}) ---
            if self.show_annotations:
                # Position: Radially outward at z_annot
                z_annot = 0.3
                # Radius slightly larger than diameter
                label_R = R_m + 0.12
                lx = label_R * np.cos(pos_angle_rad)
                ly = label_R * np.sin(pos_angle_rad)
                lz = z_annot
                
                # Draw 'S' and '{i}'
                # Scale 60% of X/Y label (0.05) -> 0.03
                lbl_scale = 0.03
                spacing = lbl_scale * 1.5 # Increase spacing to avoid overlap
                
                # S center
                sx_c = lx - spacing/2
                sy_c = ly
                self.draw_char('S', [sx_c, sy_c, lz], lbl_scale)
                
                # Num center
                nx_c = lx + spacing/2
                ny_c = ly
                self.draw_char(str(i), [nx_c, ny_c, lz], lbl_scale)


    def draw_char(self, char, center, scale):
        # Draw character using lines on XY plane at Z=center[2]
        cx, cy, cz = center
        s = scale / 2.0
        
        # Grid: 
        # TL: -1, 1; TR: 1, 1
        # BL: -1,-1; BR: 1,-1
        
        pts = []
        lines = []
        
        # Helper to add line
        def add_seg(p1, p2): # p1, p2 are tuples (x_local, y_local)
            idx = len(pts)
            pts.append([cx + p1[0]*s, cy + p1[1]*s, cz])
            pts.append([cx + p2[0]*s, cy + p2[1]*s, cz])
            lines.append([idx, idx+1])

        if char == 'S':
            # Boxy S
            add_seg((1, 1), (-1, 1)) # Top
            add_seg((-1, 1), (-1, 0)) # TL vert
            add_seg((-1, 0), (1, 0)) # Mid
            add_seg((1, 0), (1, -1)) # BR vert
            add_seg((1, -1), (-1, -1)) # Bot
        elif char == '0':
            add_seg((-1, 1), (1, 1))
            add_seg((1, 1), (1, -1))
            add_seg((1, -1), (-1, -1))
            add_seg((-1, -1), (-1, 1))
            add_seg((-1, -1), (1, 1)) # Diagonal
        elif char == '1':
            add_seg((0, 1), (0, -1))
        elif char == '2':
            add_seg((-1, 1), (1, 1))
            add_seg((1, 1), (1, 0))
            add_seg((1, 0), (-1, 0))
            add_seg((-1, 0), (-1, -1))
            add_seg((-1, -1), (1, -1))
        elif char == '3':
            add_seg((-1, 1), (1, 1))
            add_seg((1, 1), (1, -1)) # Right vert
            add_seg((-1, -1), (1, -1))
            add_seg((-1, 0), (1, 0)) # Mid
        elif char == '4':
            add_seg((-1, 1), (-1, 0))
            add_seg((-1, 0), (1, 0))
            add_seg((1, 1), (1, -1))
        elif char == '5':
            add_seg((1, 1), (-1, 1)) 
            add_seg((-1, 1), (-1, 0)) 
            add_seg((-1, 0), (1, 0)) 
            add_seg((1, 0), (1, -1)) 
            add_seg((1, -1), (-1, -1)) 

        if not pts:
            return

        ls = o3d.geometry.LineSet()
        ls.points = o3d.utility.Vector3dVector(pts)
        ls.lines = o3d.utility.Vector2iVector(lines)
        ls.paint_uniform_color([0, 0, 0])
        ls.rotate(self.R_ccw, center=(0, 0, 0))
        self.vis.add_geometry(ls)

    def update_pcd(self):
        self.lsl.pull_status()
        
        sensor_names = self.lsl.params['sensor_names']
        
        frame_points = []
        
        for i, sensor_name in enumerate(sensor_names):
            if sensor_name not in self.sensors_to_show:
                continue

            if sensor_name not in self.lsl.status:
                if not hasattr(self, 'printed_missing_status'):
                     print(f"DEBUG: {sensor_name} not in status. Keys: {list(self.lsl.status.keys())}")
                     self.printed_missing_status = True
                continue
                
            status = self.lsl.status[sensor_name]
            ranges = np.array(status['ranges'])
            
            if len(ranges) == 0:
                continue
            
            # Apply Calibration (Tare)
            if self.use_calibration:
                ranges = self.calibration.apply_tare(ranges, sensor_name)
            
            # Get Points via Geometry Helper
            points = self.geom.get_sensor_points_in_robot_frame(i, ranges)
            
            if len(points) == 0:
                continue
            
            # Accumulate
            frame_points.append(points)
            
            if not self.show_clusters:
                # Update individual sensor clouds if not clustering
                self.sensor_clouds[i].points = o3d.utility.Vector3dVector(points)
                self.sensor_clouds[i].rotate(self.R_ccw, center=(0, 0, 0))
                self.vis.update_geometry(self.sensor_clouds[i])
            else:
                # Clear individual clouds if clustering
                if len(self.sensor_clouds[i].points) > 0:
                    self.sensor_clouds[i].points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
                    self.vis.update_geometry(self.sensor_clouds[i])

        # Feature: Clusters
        current_clusters_pcd = None
        if self.show_clusters:
            # Aggregate points
            if frame_points:
                all_raw_points = np.vstack(frame_points)
                # Process via Tracker (Filter -> Cluster -> Track)
                merged_pcd = self.tracker.process_frame(all_raw_points)
                
                # Update cluster cloud
                if len(merged_pcd.points) > 0:
                    current_clusters_pcd = merged_pcd # Keep reference for traversability
                    self.cluster_cloud.points = merged_pcd.points
                    self.cluster_cloud.colors = merged_pcd.colors
                    self.cluster_cloud.rotate(self.R_ccw, center=(0, 0, 0))
                    self.vis.update_geometry(self.cluster_cloud)
                else:
                    self.cluster_cloud.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
                    self.vis.update_geometry(self.cluster_cloud)
            else:
                 self.cluster_cloud.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
                 self.vis.update_geometry(self.cluster_cloud)
        else:
             # Clear cluster cloud if not showing
             if len(self.cluster_cloud.points) > 0:
                 self.cluster_cloud.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
                 self.vis.update_geometry(self.cluster_cloud)

        # Feature: Traversability Donut
        if self.show_cost_map:
            self.update_donut_ramp(current_clusters_pcd, frame_points)
        
    def update_donut_ramp(self, current_clusters_pcd, frame_points):
        # Prepare obstacles
        obstacles = None
        if current_clusters_pcd:
            obstacles = current_clusters_pcd
        elif frame_points:
             obstacles = o3d.geometry.PointCloud()
             obstacles.points = o3d.utility.Vector3dVector(np.vstack(frame_points))
             
        max_dist = self.max_cost_map_dist
        r_safe_m = self.cost_map.r_safe
        z_height = -0.020 if self.nice_viz else -0.001 # 20mm below the floor

        c_green = [0.2, 0.8, 0.2] if self.nice_viz else [0, 1, 0]
        c_red = [0.9, 0.2, 0.2] if self.nice_viz else [1, 0, 0]

        vertices = []
        triangles = []
        colors = []
        
        # 360 degrees in 5 degree steps
        step_deg = 5
        angles = np.arange(0, 360, step_deg)
        
        # Half step for wedge span
        half_step_rad = np.deg2rad(step_deg / 2.0)
        
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        
        # Ramp Speed (m/s)
        # Go from 0 to max_dist in 1.0s -> speed = max_dist
        ramp_speed = max_dist
        
        for i, angle_deg in enumerate(angles):
            angle_rad = np.deg2rad(angle_deg)
            vx = np.cos(angle_rad)
            vy = np.sin(angle_rad)
            
            check_dist = None
            if obstacles:
                check_dist = self.cost_map.check_traversability([vx, vy], obstacles, max_dist_m=max_dist)
            
            # Instantaneous Reading
            current_reading = max_dist
            if check_dist is not None:
                current_reading = check_dist
            
            # Linear Ramp Logic
            stored_dist = self.wedge_state[i]
            
            if current_reading < stored_dist:
                # Immediate Decrease (Safety)
                stored_dist = current_reading
            else:
                # Linear Increase
                step = ramp_speed * dt
                if stored_dist < current_reading:
                    stored_dist = min(current_reading, stored_dist + step)
            
            # Update State
            self.wedge_state[i] = stored_dist
            
            # Use stored value for viz
            dist_to_viz = stored_dist
            
            # Render Logic (Inverted)
            # Green Segment: From r_safe to r_safe + dist_to_viz
            # Red Segment: From r_safe + dist_to_viz to r_safe + max_dist (if obstructed)
            
            # Define Wedge Vertices Helper
            def add_wedge_segment(r_start, r_end, color):
                theta_start = angle_rad - half_step_rad
                theta_end = angle_rad + half_step_rad
                
                v0 = [r_start * np.cos(theta_start), r_start * np.sin(theta_start), z_height]
                v1 = [r_end * np.cos(theta_start), r_end * np.sin(theta_start), z_height]
                v2 = [r_end * np.cos(theta_end), r_end * np.sin(theta_end), z_height]
                v3 = [r_start * np.cos(theta_end), r_start * np.sin(theta_end), z_height]
                
                base_idx = len(vertices)
                vertices.extend([v0, v1, v2, v3])
                triangles.append([base_idx, base_idx+1, base_idx+2])
                triangles.append([base_idx, base_idx+2, base_idx+3])
                colors.extend([color] * 4)

            # Green (Traversable)
            green_r_end = r_safe_m + dist_to_viz
            if green_r_end > r_safe_m + 0.001:
                add_wedge_segment(r_safe_m, green_r_end, c_green)
            
            # Red (Blocked)
            if dist_to_viz < max_dist - 0.01:
                red_r_start = green_r_end
                red_r_end = r_safe_m + max_dist
                add_wedge_segment(red_r_start, red_r_end, c_red)
             
        # Update Mesh
        self.donut_geometry.vertices = o3d.utility.Vector3dVector(vertices)
        if len(triangles) > 0:
            self.donut_geometry.triangles = o3d.utility.Vector3iVector(triangles)
        else:
             self.donut_geometry.triangles = o3d.utility.Vector3iVector(np.zeros((0, 3), dtype=int))
             
        self.donut_geometry.vertex_colors = o3d.utility.Vector3dVector(colors)
        self.donut_geometry.compute_vertex_normals()
        
        self.donut_geometry.rotate(self.R_ccw, center=(0,0,0))
        
        self.vis.update_geometry(self.donut_geometry)


    def update_camera_pose(self):
        ctr = self.vis.get_view_control()
        pos = np.array([1.5 * np.cos(self.turntable_angle), 
                        1.5 * np.sin(self.turntable_angle), 
                        0.3]) # 300mm high, 1.5m radius
        lookat = np.array([0.0, 0.0, 0.15]) # top of the base
        front = pos - lookat
        
        ctr.set_lookat(lookat)
        ctr.set_front(front)
        ctr.set_up([0.0, 0.0, 1.0])

    def set_initial_camera(self):
        self.turntable_angle = np.pi / 2.0 # View mostly the side of the base
        self.update_camera_pose()
        
        ctr = self.vis.get_view_control()
        ctr.set_zoom(0.3) # Adjust zoom to roughly match 1.5m distance visually

    def update_turntable(self):
        now = time.time()
        if not hasattr(self, 'last_turntable_time'):
            self.last_turntable_time = now
            return
            
        dt = now - self.last_turntable_time
        self.last_turntable_time = now
        
        # 1 rev per 20s = 360/20 = 18 deg/sec = 0.314159 rad/sec
        self.turntable_angle += 0.314159 * dt/4
        
        self.update_camera_pose()

    def update_wheels(self, dt):
        if not hasattr(self, 'robot_client'):
            return
        if 'omnibase' not in self.robot_client.status:
            return
            
        odom = self.robot_client.status['omnibase']
        gr = self.robot_client.omnibase.params.get('gr', 6.0)
        
        for w_data in getattr(self, 'wheel_data', []):
            w_name = w_data['name']
            if w_name in odom:
                vel = odom[w_name].get('vel', 0.0) / gr
                if abs(vel) < 0.001:
                    continue
                    
                d_theta = vel * dt
                rot_vec = w_data['axis'] * d_theta
                R = o3d.geometry.get_rotation_matrix_from_axis_angle(rot_vec)
                
                w_data['mesh'].rotate(R, center=w_data['center'])
                if hasattr(self, 'vis'):
                    self.vis.update_geometry(w_data['mesh'])

    def run(self):
        print("Visualization initialized. Press 'Q' or 'Esc' in the window to exit.")
        first_frame = True
        last_time = time.time()
        try:
            while self.vis.poll_events():
                now = time.time()
                dt = now - last_time
                last_time = now
                
                if first_frame:
                    self.set_initial_camera()
                    first_frame = False
                    
                self.update_pcd()
                
                if self.use_odom and self.nice_viz:
                    self.robot_client.pull_status()
                    if 'omnibase' in self.robot_client.status:
                        odom = self.robot_client.status['omnibase']
                        x = odom.get('x', 0.0)
                        y = odom.get('y', 0.0)
                        theta = odom.get('theta', 0.0)
                        self.update_grid(x, y, theta)
                        self.update_wheels(dt)
                
                if self.turntable:
                    self.update_turntable()
                    
                self.vis.update_renderer()
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            self.lsl.stop()
            if self.use_odom and self.nice_viz and hasattr(self, 'robot_client'):
                self.robot_client.stop()
            self.vis.destroy_window()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='3D Visualization of Stretch Line Sensors')

    parser.add_argument('-a', '--annotations', action='store_true', help='Show annotations (grid, labels)')
    parser.add_argument('-s', '--sensors', nargs='+', help='List of sensors to show (e.g. sensor_0 sensor_1)')
    parser.add_argument('--cluster', action='store_true', help='Enable Spatial Clustering')
    parser.add_argument('--cost_map', action='store_true', help='Enable Cost Map Visualization')
    parser.add_argument('--no_calib', action='store_true', help='Disable Calibration (Show Raw Data)')
    
    parser.add_argument('--nice_viz', action='store_true', help='Enable nice visualization (photo studio background, no grid)')
    parser.add_argument('--turntable', action='store_true', help='Slowly spin the robot around its Z axis')
    parser.add_argument('--odom', action='store_true', help='Shift the grid to simulate driving based on base odometry (requires --nice_viz)')
    parser.add_argument('--bg_color', type=float, nargs=3, default=[0.2, 0.22, 0.3], help='Background color (RGB 0-1)')
    parser.add_argument('--sensor_color', type=float, nargs=3, default=[1.0, 1.0, 0.0], help='Sensor points color (RGB 0-1)')
    parser.add_argument('--thickness', type=float, default=6.0, help='Point size and line thickness (starts twice current)')
    
    args = parser.parse_args()
    
    # Cost Map implies Clustering
    if args.cost_map:
        args.cluster = True
    
    viz = LineSensorVisualizer3D(
        show_annotations=args.annotations, 
        sensors=args.sensors, 
        show_clusters=args.cluster,
        show_cost_map=args.cost_map,
        use_calibration=not args.no_calib,
        nice_viz=args.nice_viz,
        turntable=args.turntable,
        use_odom=args.odom,
        bg_color=args.bg_color,
        sensor_color=args.sensor_color,
        thickness=args.thickness
    )
    viz.run()
