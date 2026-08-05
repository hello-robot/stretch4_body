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
from stretch4_body.subsystem.line_sensor import calibration
from stretch4_body.subsystem.line_sensor import calibration_store

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

        # Keep only bins carrying a distance (meters). Status codes arrive as
        # NaN from protocol.decode_distances_mm, so there is no magnitude
        # threshold here: the chip never reports a large distance, only a code.
        valid_mask = np.isfinite(ranges) & (ranges > 0)
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
    """Records flat-floor sessions and loads the tare they produce.

    The maths lives in calibration.py and the on-disk layout in calibration_store.py.
    """

    def __init__(self, line_sensor_loop, base_dir=None):
        self.lsl = line_sensor_loop
        self.params = self.lsl.params
        self.sensor_names = list(self.params['sensor_names'])
        self.n_bins = int(self.params['line_sensor_geometry']['pixart_report_num'])
        self._base_dir = base_dir
        # name -> LoadedTare, populated by load_tares()
        self.tares = {}
        # name -> TareRejected, so a refusal is inspectable rather than a print
        self.rejected = {}

    # -- paths -------------------------------------------------------------

    def get_calibration_base_dir(self):
        if self._base_dir is not None:
            return self._base_dir
        return os.path.join(hu.get_fleet_directory(), 'calibration_line_sensors')

    def fingerprint_for(self, sensor_name):
        return calibration.config_fingerprint(
            sensor_name, self.sensor_names.index(sensor_name), self.params)

    def compute_ideal_range(self):
        """Flat-floor AXIAL DEPTH, constant across the fan.
        """
        geom = self.params['line_sensor_geometry']
        h = geom['emitter_height_above_floor_mm'] / 1000.0
        return h / np.sin(np.deg2rad(geom['sensor_angle_down_deg']))

    # -- recording ---------------------------------------------------------

    def record_session(self, n_frames=300, sensors=None, timeout_s=None,
                       poll_period_s=0.002, progress=True):
        """Capture n_frames DISTINCT frames per sensor.
        """
        targets = list(sensors) if sensors else list(self.sensor_names)
        unknown = [s for s in targets if s not in self.sensor_names]
        if unknown:
            raise ValueError(f'unknown sensors {unknown}; configured: {self.sensor_names}')
        if timeout_s is None:
            timeout_s = max(30.0, n_frames / 20.0)   # 20 Hz worst case, floor 30 s

        session = calibration.RecordingSession(
            session_id=calibration_store.new_session_id(),
            started_at=datetime.datetime.now().isoformat(),
            requested_frames=int(n_frames),
            stretch_body_version=getattr(hu, '__version__', ''),
            loop_params_snapshot=_plain(self.params))
        for name in targets:
            fp = self.fingerprint_for(name)
            session.fingerprints[name] = {
                'fingerprint': fp, 'sha256': calibration.fingerprint_hash(fp)}

        buf = {n: {'ranges': [], 'codes': [], 'frame_id': [], 'ts': [],
                   'missed': []} for n in targets}
        last_id = {n: None for n in targets}
        dupes = {n: 0 for n in targets}
        regressions = {n: 0 for n in targets}
        ever_dead = {n: False for n in targets}
        max_missed = {n: 0 for n in targets}

        t0 = time.time()
        polls = 0
        bar = tqdm.tqdm(total=n_frames, disable=not progress,
                        desc='recording', unit='frame')
        try:
            while time.time() - t0 < timeout_s:
                if all(len(buf[n]['ranges']) >= n_frames for n in targets):
                    break
                self.lsl.pull_status()
                polls += 1
                status = self.lsl.status
                # Liveness lives in the health block now that status messages
                dead = set(status.get('sensors_dead', ()) or ())
                now = time.time()
                for name in targets:
                    if name in dead:
                        ever_dead[name] = True
                        continue
                    entry = status.get(name) or {}
                    ranges = entry.get('ranges')
                    codes = entry.get('codes')
                    fid = entry.get('frame_id')
                    if ranges is None or codes is None or fid is None:
                        continue
                    if len(ranges) != self.n_bins or len(codes) != self.n_bins:
                        continue
                    if last_id[name] is not None:
                        if fid == last_id[name]:
                            dupes[name] += 1
                            continue          # same frame; not a new observation
                        if fid < last_id[name]:
                            regressions[name] += 1
                    last_id[name] = fid
                    if len(buf[name]['ranges']) >= n_frames:
                        continue
                    buf[name]['ranges'].append(np.asarray(ranges, dtype=np.float64))
                    buf[name]['codes'].append(np.asarray(codes, dtype=np.uint8))
                    buf[name]['frame_id'].append(int(fid))
                    buf[name]['ts'].append(now)
                    m = int(entry.get('missed_frames', 0))
                    buf[name]['missed'].append(m)
                    max_missed[name] = max(max_missed[name], m)
                if progress:
                    bar.n = min(len(buf[n]['ranges']) for n in targets)
                    bar.refresh()
                time.sleep(poll_period_s)
        finally:
            bar.close()

        elapsed = time.time() - t0
        session.ended_at = datetime.datetime.now().isoformat()
        session.poll_iterations = polls

        for i, name in enumerate(targets):
            b = buf[name]
            got = len(b['ranges'])
            empty2d = np.zeros((0, self.n_bins))
            rec = calibration.SensorRecording(
                sensor_name=name, sensor_index=self.sensor_names.index(name),
                ranges=np.stack(b['ranges']) if got else empty2d,
                codes=(np.stack(b['codes']) if got
                       else np.zeros((0, self.n_bins), np.uint8)),
                frame_id=np.asarray(b['frame_id'], dtype=np.int64),
                ts=np.asarray(b['ts'], dtype=np.float64),
                missed_frames=np.asarray(b['missed'], dtype=np.int32))
            rec.stats = {
                'requested_frames': int(n_frames),
                'distinct_frames_captured': int(got),
                'poll_iterations': int(polls),
                'duplicate_frames_skipped': int(dupes[name]),
                'frame_id_min': int(min(b['frame_id'])) if got else None,
                'frame_id_max': int(max(b['frame_id'])) if got else None,
                'frame_id_regressions': int(regressions[name]),
                'achieved_frames_per_s': round(got / elapsed, 2) if elapsed else 0.0,
                'max_missed_frames': int(max_missed[name]),
                'ever_in_sensors_dead': bool(ever_dead[name]),
                'wall_clock_s': round(elapsed, 2),
            }
            if got == 0:
                rec.status = 'DEAD' if ever_dead[name] else 'NO_DATA'
                rec.notes.append('no frames captured')
            elif got < n_frames:
                rec.status = 'TIMEOUT'
                rec.notes.append(f'captured {got}/{n_frames} frames in {elapsed:.1f}s')
            if ever_dead[name] and rec.status == 'OK':
                rec.status = 'DEAD'
                rec.notes.append('appeared in sensors_dead during the run')
            session.recordings[name] = rec

        return session

    # -- tare loading ------------------------------------------------------

    def load_tares(self, sensors=None, verbose=True):
        """Load and validate the current tare for each sensor.

        A refusal is recorded in self.rejected and the sensor is left
        uncalibrated. It is never downgraded to a warning and never falls back
        to an older file: running uncalibrated is recoverable, running on a
        tare belonging to a different robot configuration is not.
        """
        targets = list(sensors) if sensors else list(self.sensor_names)
        base = self.get_calibration_base_dir()
        self.tares, self.rejected = {}, {}
        for name in targets:
            path = calibration_store.tare_path(base, name)
            try:
                self.tares[name] = calibration_store.load_validated_tare(
                    path, self.fingerprint_for(name), self.n_bins)
            except calibration_store.TareRejected as exc:
                self.rejected[name] = exc
                if verbose:
                    print(f'  {name}: NO TARE ({exc.reason}) -- {exc.detail}')
        return self.tares

    def apply_tare(self, ranges, sensor_name, codes=None):
        """Tare one sensor's ranges. Uncalibrated sensors pass through."""
        t = self.tares.get(sensor_name)
        if t is None:
            return np.asarray(ranges, dtype=np.float64)
        return calibration.apply_tare_array(ranges, t.offsets, t.valid_mask, codes)

    def bin_reliable(self):
        """{sensor_name: bool array} -- bins with a trustworthy tare."""
        return {n: t.valid_mask for n, t in self.tares.items()}

    def bin_null_rate(self):
        """{sensor_name: float array} -- per-bin no-return rate on clear floor."""
        return {n: t.null_rate_per_bin for n, t in self.tares.items()}


def _plain(obj):
    """Params snapshots go into YAML and JSON, so strip numpy and tuples."""
    if isinstance(obj, dict):
        return {str(k): _plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple, np.ndarray)):
        return [_plain(v) for v in list(obj)]
    if isinstance(obj, (bool, np.bool_)):
        return bool(obj)
    if isinstance(obj, (int, np.integer)):
        return int(obj)
    if isinstance(obj, (float, np.floating)):
        return float(obj)
    return obj



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
