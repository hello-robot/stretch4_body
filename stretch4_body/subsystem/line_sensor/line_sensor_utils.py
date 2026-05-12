#!/usr/bin/env python3

import numpy as np
import time
import yaml
import os
import glob
import datetime
import tqdm
import pprint 
import stretch4_body.core.hello_utils as hu
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False
from stretch4_body.subsystem.line_sensor.line_sensor_loop import LineSensorLoop

class LineSensorGeometry:
    def __init__(self, params):
        self.params = params
        self.param_height_cm = params.get('emitter_height_above_floor_mm', 100.67) / 10.0
        self.param_diameter_cm = params.get('emitter_pitch_diameter_mm', 404.04) / 10.0
        self.sensor_angles = params.get('sensor_angles_deg', [10.18, 39.64, 80.36, 39.64, 80.36, 39.64])
        self.sensor_normals = params.get('sensor_normals_deg', [0.0, 60.0, 120.0, 180.0, 240.0, 300.0])
        
        # Params previously in pixart_j3_parameters
        self.pixart_report_num = params.get('pixart_report_num', 320)
        self.horizontal_fov_degrees = params.get('sensor_horizontal_fov_degrees', 103.0)
        self.horizontal_fov_rad = np.deg2rad(self.horizontal_fov_degrees)
        self.angle_down_deg = params.get('sensor_angle_down_deg', 26.0)
        
        print(f"LineSensorGeometry Init:")
        print(f"  Diameter (cm): {self.param_diameter_cm}")
        print(f"  R_m: {(self.param_diameter_cm / 2.0) / 100.0}")
        print(f"  Sensor Angles (deg): {self.sensor_angles}")
        print(f"  Sensor Normals (deg): {self.sensor_normals}")

    def get_angles(self):
        return np.deg2rad(90) - np.linspace(-self.horizontal_fov_rad/2, self.horizontal_fov_rad/2, self.pixart_report_num)

    def to_floor_coordinate_system(self, x, y):
         # y_b = -x
         # hypotenuse_m = y
         # angle = self.angle_down_rad
         # z_b = floor_y - (hypotenuse_m * math.sin(angle))
         # x_b = hypotenuse_m * math.cos(angle)
         
         y_b = -x
         hypotenuse_m = y
         angle = np.deg2rad(self.angle_down_deg)
         floor_y = self.param_height_cm / 100.0
         
         z_b = floor_y - (hypotenuse_m * np.sin(angle))
         x_b = hypotenuse_m * np.cos(angle)
         
         return x_b, y_b, z_b

    def get_sensor_points_in_robot_frame(self, sensor_idx, ranges):
        """
        Convert sensor ranges to global robot frame points (XYZ).
        """
        R_m = (self.param_diameter_cm / 2.0) / 100.0
        # sensor_height_m = self.param_height_cm / 100.0 # Used internally now

        if len(ranges) == 0:
             return np.zeros((0, 3))

        # Filter out invalid readings (Meters)
        # Assuming max valid range ~4m
        # Note: Input ranges are in Meters (e.g. 0.2m)
        valid_mask = (ranges < 4.0) & (ranges > 0)
        ranges_m = ranges[valid_mask]
        
        if len(ranges_m) == 0:
            return np.zeros((0, 3))

        # 1. Get Sensor Plane Coords (Meters)
        # Re-implementing get_cartesian_points to avoid hardcoded limits in library
        # y = range, x = y / tan(angles)
        angles = self.get_angles()
        
        if len(ranges) == len(angles):
            # We need to mask angles to match ranges_m
            angles_masked = angles[valid_mask]
            y_s = ranges_m
            x_s = y_s / np.tan(angles_masked)
        else:
             # Fallback: library call (might have limits, but better than crash)
             # But library call expects full array.
             # If we pass filtered array, it crashes.
             # So we must return empty or fail gracefully.
             print(f"Warning: ranges len {len(ranges)} != angles len {len(angles)}")
             return np.zeros((0, 3))

        # 2. To Floor/Base Frame (Meters)
        # x_b: Forward from sensor, y_b: Left from sensor, z_b: Up from floor
        x_b, y_b, z_b = self.to_floor_coordinate_system(x_s, y_s)
        
        # 3. Rotate to Robot Global based on sensor idx
        # Position Angle:
        current_pos_angle_deg = sum(self.sensor_angles[:sensor_idx+1])
        pos_angle_rad = -np.deg2rad(current_pos_angle_deg)
        
        # Orientation Angle:
        rot_angle_deg = self.sensor_normals[sensor_idx]
        rot_angle_rad = -np.deg2rad(rot_angle_deg)
        
        # 4a. Rotate Point Cloud (Sensor Frame -> Aligned with Robot X)
        # Original logic: x_b forward, y_b left.
        # We want to rotate these points by rot_angle_rad about Z.
        x_rot = x_b * np.cos(rot_angle_rad) - y_b * np.sin(rot_angle_rad)
        y_rot = x_b * np.sin(rot_angle_rad) + y_b * np.cos(rot_angle_rad)
        z_rot = z_b
        
        # 4b. Translate to Sensor Position (Robot Frame)
        # Sensor Pos:
        sx = R_m * np.cos(pos_angle_rad)
        sy = R_m * np.sin(pos_angle_rad)
        
        x_robot = x_rot + sx
        y_robot = y_rot + sy
        z_robot = z_rot
        
        # Stack
        points = np.stack((x_robot, y_robot, z_robot), axis=1)
        return points

    def get_sensor_ground_frame(self, sensor_idx):
        """
        Returns the 4x4 Homogenous Transform for the Sensor Ground Frame.
        Origin: (ex, ey, 0) - Emitter XY projected to ground.
        Y-Axis: Radial Outward (Sensor Normal).
        Z-Axis: Up (0,0,1).
        X-Axis: Orthogonal Right (Y x Z).
        """
        R_m = (self.param_diameter_cm / 2.0) / 100.0
        
        # Position Angle (Calculates geometric position of emitter on circumference)
        current_pos_angle_deg = sum(self.sensor_angles[:sensor_idx+1])
        pos_angle_rad = -np.deg2rad(current_pos_angle_deg)
        
        ex = R_m * np.cos(pos_angle_rad)
        ey = R_m * np.sin(pos_angle_rad)
        
        # Origin
        origin = np.array([ex, ey, 0.0])
        
        # Sensor Normal (Yaw)
        # Note: sensor_normals are CW from Robot X = Forward.
        # But we need Vector in Robot Frame.
        # Robot X = [1,0,0].
        # Yaw is rotation around Z.
        # Angle is given CW (positive). standard math is CCW (negative).
        yaw_deg = self.sensor_normals[sensor_idx]
        yaw_rad = -np.deg2rad(yaw_deg)
        
        # Y-Axis (Green) = Points Outward
        # This matches the sensor normal direction.
        y_axis = np.array([np.cos(yaw_rad), np.sin(yaw_rad), 0.0])
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # Z-Axis (Blue) = Up
        z_axis = np.array([0.0, 0.0, 1.0])
        
        # X-Axis (Red) = Right (Orthogonal to Outward)
        # In a right-handed system: X = Y x Z?
        # Let's check:
        # If Y is forward (0,1,0), Z is Up (0,0,1).
        # X = Y x Z = (1, 0, 0) -> Right. Yes.
        # Wait, usually X is Forward in robotics?
        # User requested: "Y direction points along the direction of the central emitter ray".
        # So this frame is: Y-Forward.
        
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Build 4x4 Matrix
        T = np.eye(4)
        T[:3, 0] = x_axis
        T[:3, 1] = y_axis
        T[:3, 2] = z_axis
        T[:3, 3] = origin
        
        return T

    def get_ground_intersect_dist_y(self):
        """
        Returns the distance along the Sensor Ground Frame Y-axis
        where the emitter central ray intersects the ground.
        dist = height / tan(pitch)
        """
        sensor_height_m = self.param_height_cm / 100.0
        angle_down_deg = self.params.get('sensor_angle_down_deg', 26.0)
        angle_down_rad = np.deg2rad(angle_down_deg)
        
        if np.isclose(angle_down_rad, 0):
             return float('inf')
        
        dist = sensor_height_m / np.tan(angle_down_rad)
        return dist




class LineSensorCalibration:
    """
    Calibrates line sensors by assuming the robot is on a flat floor.
    Features:
    - Record raw data to timestamped directories.
    - Load recorded data.
    - Compute 'model' (range adjustments) to match ideal floor geometry.
    - Save model to YAML.
    """
    def __init__(self, line_sensor_loop):
        self.lsl = line_sensor_loop
        self.params = self.lsl.params
        self.sensor_names = self.params['sensor_names']
        
        # Data storage: {sensor_name: [list of range arrays]}
        self.data_samples = {name: [] for name in self.sensor_names}
        
        # Calibration results (Tare Offsets): {sensor_name: adjustment_array}
        self.tare_offsets = {}
        
        # Current session directory (set during record)
        self.session_directory = None

    def get_calibration_base_dir(self):
        return hu.get_fleet_directory() + 'calibration_line_sensors/'

    def get_sensor_dir(self, sensor_name):
        return os.path.join(self.get_calibration_base_dir(), sensor_name)

    def create_timestamp_string(self):
        return datetime.datetime.now().strftime("%Y%m%d%H%M%S")

    def record_data(self, itrs=100, sensors=None):
        """
        Record 'itrs' samples for specified sensors (or all) to a new timestamped directory.
        Structure: .../calibration_line_sensors/<sensor_name>/<timestamp>/<timestamp_ms>_j3_ranges.npy
        """
        target_sensors = sensors if sensors else self.sensor_names
        print(f"Recording {itrs} samples for sensors: {target_sensors}...")
        timestamp = self.create_timestamp_string()
        
        # Prepare directories
        sensor_dirs = {}
        # Prepare directories
        sensor_dirs = {}
        for name in target_sensors:
            s_dir = os.path.join(self.get_sensor_dir(name), timestamp)
            if not os.path.exists(s_dir):
                os.makedirs(s_dir)
            sensor_dirs[name] = s_dir
            
        # Reset local data samples
        # Reset local data samples for target sensors
        for name in target_sensors:
            self.data_samples[name] = []
        
        for i in tqdm.tqdm(range(itrs), desc="Recording Data"):
            self.lsl.pull_status()
            curr_time = time.time()
            
            for name in target_sensors:
                if name in self.lsl.status:
                    ranges = np.array(self.lsl.status[name]['ranges'])
                    
                    if len(ranges) == 0:
                        continue
                        
                    # Store in memory
                    self.data_samples[name].append(ranges)
                    
                    # Save to disk
                    filename = os.path.join(sensor_dirs[name], '{:.6f}_j3_ranges.npy'.format(curr_time))
                    np.save(filename, ranges)
            
            time.sleep(0.02) # ~50Hz max poll rate from loop
            
        self.session_timestamp = timestamp
        print(f"Recording complete. Session ID: {timestamp}")

    def load_data(self, timestamp=None, sensors=None):
        """
        Load data from a specific timestamp session, or the most recent one if None.
        """
        target_sensors = sensors if sensors else self.sensor_names
        
        for name in target_sensors:
            self.data_samples[name] = [] # Clear existing
            
            base_dir = self.get_sensor_dir(name)
            if not os.path.exists(base_dir):
                print(f"No calibration data found for {name}")
                continue
                
            # Find session dir
            if timestamp:
                session_dir = os.path.join(base_dir, timestamp)
            else:
                # Find most recent
                dirs = glob.glob(os.path.join(base_dir, '*'))
                dirs = [d for d in dirs if os.path.isdir(d)]
                if not dirs:
                    print(f"No sessions found for {name}")
                    continue
                dirs.sort()
                session_dir = dirs[-1]
            
            if not os.path.exists(session_dir):
                print(f"Session directory not found: {session_dir}")
                continue
                
            print(f"Loading data for {name} from {session_dir}")
            files = glob.glob(os.path.join(session_dir, '*_j3_ranges.npy'))
            files.sort()
            
            for f in files:
                try:
                    ranges = np.load(f)
                    self.data_samples[name].append(ranges)
                except Exception as e:
                    print(f"Error loading {f}: {e}")
                    
            print(f"  Loaded {len(self.data_samples[name])} samples.")

    def compute_ideal_range(self):
        # Calculate expected range for a flat floor given sensor geometry
        geom_params = self.params.get('line_sensor_geometry', {})
        h_m = geom_params.get('emitter_height_above_floor_mm', 100.67) / 1000.0
        angle_down_deg = geom_params.get('sensor_angle_down_deg', 26.0)
        angle_down_rad = np.deg2rad(angle_down_deg)
        r_ideal = h_m / np.sin(angle_down_rad)
        return r_ideal

    def compute_tare(self, sensors=None):
        """
        Compute the tare (adjustments) based on loaded data.
        """
        r_ideal = self.compute_ideal_range()
        print(f"Computing tare with Ideal Range: {r_ideal:.4f} m")
        
        target_sensors = sensors if sensors else self.sensor_names
        
        self.tare_offsets = {}
        
        for name in target_sensors:
            if name not in self.data_samples:
                continue
                
            samples = self.data_samples[name]
            if not samples:
                print(f"No data for {name}, skipping model computation.")
                continue
                
            # Filter out inconsistent shapes
            if len(samples) == 0:
                print(f"No data for {name}, skipping model computation.")
                continue

            # Determine mode shape
            shapes = [s.shape for s in samples]
            # Find most common shape
            from collections import Counter
            common_shape = Counter(shapes).most_common(1)[0][0]
            
            # Filter
            valid_samples = [s for s in samples if s.shape == common_shape]
            if len(valid_samples) < len(samples):
                print(f"Warning: Dropped {len(samples) - len(valid_samples)} samples with inconsistent shapes for {name}")
            
            if not valid_samples:
                print(f"No valid samples for {name} with shape {common_shape}")
                continue

            samples_np = np.stack(valid_samples)
            
            # Error = Measured - Ideal
            # Adjustment = Measured - Ideal
            # So Corrected = Measured - Adjustment = Ideal
            errors = samples_np - r_ideal
            
            median_adjustments = np.median(errors, axis=0)
            self.tare_offsets[name] = median_adjustments
            
            avg_adj = np.mean(median_adjustments)
            print(f"Sensor {name}: Avg Tare = {avg_adj:.4f} m")

    def save_tare(self, timestamp=None, sensors=None):
        """
        Save the computed tare to the session directory (or most recent).
        """
        target_sensors = sensors if sensors else self.sensor_names
        
        for name in target_sensors:
            if name not in self.tare_offsets:
                continue
                
            # Determine save directory
            base_dir = self.get_sensor_dir(name)
            if timestamp:
                session_dir = os.path.join(base_dir, timestamp)
            elif hasattr(self, 'session_timestamp'):
                 session_dir = os.path.join(base_dir, self.session_timestamp)
            else:
                # Find most recent to save into
                dirs = glob.glob(os.path.join(base_dir, '*'))
                dirs = [d for d in dirs if os.path.isdir(d)]
                if not dirs:
                    print(f"No directory to save model for {name}")
                    continue
                dirs.sort()
                session_dir = dirs[-1]
            
            if not os.path.exists(session_dir):
                os.makedirs(session_dir)
                
            filename = os.path.join(session_dir, 'calibration_tare.yaml')
            
            # Prepare data
            data = {
                'sensor_name': name,
                'tare_offsets': self.tare_offsets[name].tolist(),
                'ideal_range_m': float(self.compute_ideal_range()),
                'timestamp': datetime.datetime.now().isoformat()
            }
            
            with open(filename, 'w') as f:
                yaml.dump(data, f)
            print(f"Saved tare for {name} to {filename}")

    def load_latest_tare(self):
        """
        Load the latest tare (adjustment array) for each sensor.
        If multiple sessions exist, uses the one with the latest timestamp.
        """
        print("Loading latest tare...")
        loaded_count = 0
        for name in self.sensor_names:
            base_dir = self.get_sensor_dir(name)
            if not os.path.exists(base_dir):
                continue
                
            # Find all session dirs containing 'calibration_tare.yaml'
            session_dirs = glob.glob(os.path.join(base_dir, '*'))
            session_dirs.sort(reverse=True) # Newest first
            
            for s_dir in session_dirs:
                if not os.path.isdir(s_dir):
                    continue
                    
                model_path = os.path.join(s_dir, 'calibration_tare.yaml')
                if os.path.exists(model_path):
                    try:
                        with open(model_path, 'r') as f:
                            data = yaml.safe_load(f)
                        
                        if 'tare_offsets' in data:
                            self.tare_offsets[name] = np.array(data['tare_offsets'])
                            print(f"  Loaded tare for {name} from {s_dir}")
                            loaded_count += 1
                            break # Found latest for this sensor
                    except Exception as e:
                        print(f"  Error loading tare for {name} from {s_dir}: {e}")
        
        if loaded_count == 0:
            print("  No tare found.")
            
    def apply_tare(self, ranges, sensor_name):
        """
        Apply tare adjustments to raw ranges.
        New Range = Raw Range - Tare
        """
        if sensor_name not in self.tare_offsets:
            return ranges
            
        adjustment = self.tare_offsets[sensor_name]
        
        # Check shape compatibility
        if ranges.shape != adjustment.shape:
             # Try capturing shape mismatch only once to avoid spam?
             # For now, just return raw if mismatch
             return ranges
             
        return ranges - adjustment



class LineSensorClusterTracker:
    """
    Tracks spatial clusters of line sensor points over time.
    
    Processing Steps:
    1. Ground Filtering: Removes points within a Z-range (cliffs/obstacles) defined by thresh_cliff_mm and thresh_obstacle_mm.
    2. Clustering: Uses DBSCAN to group remaining points into clusters based on proximity (cluster_eps).
    3. Filtering: Removes random noise or small clusters based on size (min_width) and point count (cluster_min_points).
    4. Tracking: Matches new clusters to existing tracks based on centroid distance (match_thresh_m) to maintain consistent IDs.
    """
    def __init__(self, params):
        self.params = params
        self.tracks = {} # {id: {'centroid': np.array, 'last_seen': time, 'pcd': o3d.geometry.PointCloud, 'color': [r,g,b]}}
        self.next_id = 0
        
        # Tracking Params
        # match_thresh_m: Max distance between a new cluster centroid and an existing track to consider it a match.
        self.match_thresh_m = params.get('match_thresh_m', 0.1)
        # max_age_s: How long to keep a track alive (in seconds) if it's not seen in recent frames.
        self.max_age_s = params.get('max_age_s', 1.0)
        
        # Ground Filtering Params
        # thresh_cliff_mm: Z threshold for "cliffs". Points below -10mm (default) are considered cliffs/ground imperfections and ignored.
        thresh_cliff_mm = params.get('thresh_cliff_mm', 10)
        # thresh_obstacle_mm: Z threshold for "obstacles". Points above 10mm (default) are considered valid obstacles. 
        # Points between -thresh_cliff_mm and thresh_obstacle_mm are filtered out as "ground".
        thresh_obstacle_mm = params.get('thresh_obstacle_mm', 10)
        
        self.z_min_exclude = -thresh_cliff_mm / 1000.0
        self.z_max_exclude = thresh_obstacle_mm / 1000.0
        
        # Clustering Params (DBSCAN)
        # cluster_eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other. (meters)
        self.cluster_eps = params.get('cluster_eps', 0.03)
        # cluster_min_points: The number of samples (or total weight) in a neighborhood for a point to be considered as a core point.
        self.cluster_min_points = params.get('cluster_min_points', 3)
        # min_width: Minimum physical width (largest dimension) of a cluster to be considered valid. Helps filter out single stray pixels.
        self.min_width = params.get('min_width', 0.01)
        
        # Colors
        self.cluster_colors = [
            [1, 0, 0], [0, 1, 0], [0, 0, 1], 
            [1, 1, 0], [1, 0, 1], [0, 1, 1], 
            [0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5],
            [1, 0.5, 0], [0.5, 0, 1], [0, 1, 0.5]
        ]

    def _get_spatial_clusters(self, points):
        """
        Internal: Cluster raw points (numpy Nx3).
        Returns list of Open3D PointClouds.
        """
        if len(points) == 0:
            return []
            
        # 1. Ground Filter
        # Remove points where z_min < z < z_max
        mask_ground = (points[:, 2] > self.z_min_exclude) & (points[:, 2] < self.z_max_exclude)
        mask_keep = ~mask_ground
        
        filtered_points = points[mask_keep]
        
        if len(filtered_points) == 0:
            return []
            
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(filtered_points)
            
        # 2. Euclidean Clustering
        labels = np.array(pcd.cluster_dbscan(eps=self.cluster_eps, min_points=self.cluster_min_points, print_progress=False))
        
        if len(labels) == 0:
            return []
            
        max_label = labels.max()
        if max_label < 0:
            return []
            
        clusters = []
        for i in range(max_label + 1):
            mask_cluster = (labels == i)
            
            # Extract points for this cluster
            # Note: labels align with filtered_points
            c_points = filtered_points[mask_cluster]
            
            c_pcd = o3d.geometry.PointCloud()
            c_pcd.points = o3d.utility.Vector3dVector(c_points)
            
            # 3. Filter Small Clusters
            bbox = c_pcd.get_axis_aligned_bounding_box()
            extent = bbox.get_extent()
            max_extent = np.max(extent)
            
            if max_extent >= self.min_width:
                 clusters.append(c_pcd)
                 
        return clusters

    def process_frame(self, points):
        """
        Full pipeline: Raw Points -> Filter -> Cluster -> Track.
        Returns: One merged Open3D PointCloud with colors.
        """
        # 1. Cluster
        clusters = self._get_spatial_clusters(points)
        
        # 2. Update Tracks
        now = time.time()
        
        # Compute Centroids
        new_centroids = []
        for c in clusters:
             new_centroids.append(c.get_center())

        # Match (Greedy)
        active_track_ids = list(self.tracks.keys())
        matches = {} # {new_idx: track_id}
        used_tracks = set()
        
        candidates = []
        for i, center in enumerate(new_centroids):
            for tid in active_track_ids:
                track_center = self.tracks[tid]['centroid']
                dist = np.linalg.norm(center - track_center)
                if dist < self.match_thresh_m:
                    candidates.append((dist, i, tid))
        
        candidates.sort(key=lambda x: x[0])
        
        for dist, new_idx, tid in candidates:
            if new_idx not in matches and tid not in used_tracks:
                matches[new_idx] = tid
                used_tracks.add(tid)
        
        # Update State & Prepare Output
        merged_points = []
        merged_colors = []
        
        for i, cluster in enumerate(clusters):
            tid = None
            if i in matches:
                tid = matches[i]
                self.tracks[tid]['centroid'] = new_centroids[i]
                self.tracks[tid]['last_seen'] = now
                self.tracks[tid]['pcd'] = cluster
            else:
                tid = self.next_id
                self.next_id += 1
                # Assign color based on ID creation
                col = self.cluster_colors[tid % len(self.cluster_colors)]
                self.tracks[tid] = {
                    'centroid': new_centroids[i],
                    'last_seen': now,
                    'pcd': cluster,
                    'color': col
                }
            
            # Use color from track
            color = self.tracks[tid]['color']
            pts = np.asarray(cluster.points)
            cols = np.tile(color, (len(pts), 1))
            
            merged_points.append(pts)
            merged_colors.append(cols)

        # Prune
        prune_ids = [tid for tid, t in self.tracks.items() if now - t['last_seen'] > self.max_age_s]
        for tid in prune_ids:
            del self.tracks[tid]
            
        # Return merged result
        out_pcd = o3d.geometry.PointCloud()
        if merged_points:
            out_pcd.points = o3d.utility.Vector3dVector(np.vstack(merged_points))
            out_pcd.colors = o3d.utility.Vector3dVector(np.vstack(merged_colors))
            
        return out_pcd

class LineSensorCostMap:
    def __init__(self, params):
        self.params = params
        base_radius_mm = params.get('base_radius_mm', 170.0)
        inflation_mm = params.get('inflation_mm', 20.0)
        
        # Robot Base Radius ~170mm (340mm diameter)
        # Inflation: extra buffer
        self.r_safe = (base_radius_mm + inflation_mm) / 1000.0 # meters
        
    def check_traversability(self, velocity_vector, clusters, max_dist_m=0.2):
        """
        Check how far the robot can move in velocity_vector direction.
        velocity_vector: (vx, vy)
        clusters: list of points (N, 3) or Open3D PointClouds
        max_dist_m: maximum distance to check/return
        
        Returns: safe_distance (float) or None if safe >= max_dist_m
        """
        # Normalize velocity vector
        norm = np.linalg.norm(velocity_vector)
        if norm < 0.001:
            return max_dist_m # No movement, technically safe? Or undefined.
            
        v_hat = np.array(velocity_vector) / norm
        
        # Flatten clusters into a single set of obstacle points
        # We assume 2D check on XY plane.
        obstacle_points = []
        
        # Determine input type: list of PCDs or something else?
        # The tracker processes frame and returns a single merged PCD now? 
        # Or should we pass the list of clusters from the tracker?
        # The tracker returns a merged PCD. We can use that.
        
        if hasattr(clusters, 'points'): # Open3D PointCloud
            pts = np.asarray(clusters.points)
            if len(pts) > 0:
                obstacle_points = pts[:, :2] # XY only
        elif isinstance(clusters, list):
            # List of PCDs (if we were using old API, but now tracker returns merged)
            # Handle just in case
            all_pts = []
            for c in clusters:
                if hasattr(c, 'points'):
                    pts = np.asarray(c.points)
                    if len(pts) > 0:
                        all_pts.append(pts[:, :2])
            if all_pts:
                obstacle_points = np.vstack(all_pts)
                
        if len(obstacle_points) == 0:
            return None # No obstacles
            
        # Ray-Circle Intersection formulation
        # Robot is circle Radius R at P(t) = t * v_hat
        # Obstacle is Point C
        # Condition: || C - t * v_hat || < R
        # Squared: ||C||^2 - 2t(C . v_hat) + t^2 < R^2
        # t^2 - 2(C.v)t + (|C|^2 - R^2) < 0
        # Roots of t^2 - 2(C.v)t + (|C|^2 - R^2) = 0
        # t = [2(C.v) +/- sqrt(4(C.v)^2 - 4(|C|^2 - R^2))] / 2
        # t = (C.v) +/- sqrt((C.v)^2 - |C|^2 + R^2)
        
        # Let d_proj = C . v_hat
        # Let d_sq = |C|^2
        # Discriminant D = d_proj^2 - d_sq + R^2
        # If D < 0: No intersection (Line does not hit circle)
        
        # Wait, if D < 0, it means the LINE defined by ray doesn't intersect circle.
        # But we are checking if Point C is within distance R of line segment from 0 to max_dist?
        
        # Alternative Logic (Geometric):
        # 1. Project C onto Line: t_closest = C . v_hat
        # 2. Dist of C to Line: h = sqrt(|C|^2 - t_closest^2)
        # 3. If h > R_safe: No collision ever on this infinite line.
        # 4. If h <= R_safe: potential collision.
        #    Collision starts when robot center is at t_coll = t_closest - sqrt(R_safe^2 - h^2)
        #    Check if 0 < t_coll < max_dist.
        
        # Vectorized implementation
        C = obstacle_points # (N, 2)
        
        # 1. Dot Product (Projection)
        # v_hat is (2,)
        t_closest = np.dot(C, v_hat) # (N,)
        
        # Filter 1: Clusters "behind" the robot are not immediate threats for forward collision?
        # Actually, if t_closest is negative, obstacle is behind.
        # But if R_safe is large, we might overlap with it at t=0.
        # Let's check overlap at t=0 first?
        # Dist to origin |C|. If |C| < R_safe, we are already in collision.
        
        C_sq_norm = np.sum(C**2, axis=1) # |C|^2
        
        # Check initial collision
        if np.any(C_sq_norm < self.r_safe**2):
            return 0.0 # Collision at start
            
        # 2. Dist to Line squared: h^2 = |C|^2 - t_closest^2
        # Note: mathematically h^2 must be >= 0. roundoff might make it negative slightly.
        h_sq = C_sq_norm - t_closest**2
        h_sq = np.maximum(h_sq, 0) # Clip negative
        
        # 3. Check if h < R_safe ( h^2 < R_safe^2 )
        mask_potential = h_sq < self.r_safe**2
        
        if not np.any(mask_potential):
            return None
            
        # 4. Calculate Collision Distances for potential obstacles
        safe_sq = self.r_safe**2
        dt = np.sqrt(safe_sq - h_sq[mask_potential])
        
        # t_coll = t_closest - dt
        t_coll = t_closest[mask_potential] - dt
        
        # Filter for valid collisions in range [0, max_dist]
        # We only care about positive t_coll (forward).
        # What if t_coll < 0? It means we passed the intersection?
        # Or checking overlaps? We handled t=0 overlap above.
        
        mask_valid = (t_coll >= 0) & (t_coll <= max_dist_m)
        
        valid_t = t_coll[mask_valid]
        
        if len(valid_t) == 0:
            return None
            
        # Return first collision
        return np.min(valid_t)


if __name__ == "__main__":
    print("Testing Line Sensor Calibration...")
    
    # Initialize Loop
    lsl = LineSensorLoop()
    if not lsl.startup():
        print("Failed to start LineSensorLoop")
        exit(1)
        
    try:
        calib = LineSensorCalibration(lsl)
        
        # 1. Record
        print("\n--- Recording Data ---")
        calib.record_data(itrs=50) # 50 samples
        
        # 2. Load (Verify loading works)
        print("\n--- Loading Data ---")
        calib.load_data(calib.session_timestamp)
        
        # 3. Compute Model
        print("\n--- Computing Model ---")
        calib.compute_tare()
        
        # 4. Save Model
        print("\n--- Saving Model ---")
        calib.save_tare()
        
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        lsl.stop()
