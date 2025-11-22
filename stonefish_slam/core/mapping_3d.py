#!/usr/bin/env python3
"""
3D Point Cloud Generation from Sonar Images with Probabilistic Mapping

This module accumulates multiple sonar frames and updates voxel probabilities
using log-odds Bayesian updates to create a probabilistic 3D map.

Ported from krit_slam feature_extraction_3d.py with modifications for ROS2 Humble
and integration with stonefish_slam SLAM system.

References:
- Sonar 3D reconstruction: octree-based probabilistic mapping
- Log-odds Bayesian update for occupancy grid mapping
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import defaultdict
import gtsam


class SimpleOctree:
    """
    Sparse voxel storage using dictionary with dynamic expansion
    Stores log-odds values for each voxel
    """

    def __init__(self, resolution=0.03, dynamic_expansion=True):
        """
        Initialize octree with given resolution

        Args:
            resolution: Size of each voxel in meters
            dynamic_expansion: Enable dynamic map expansion
        """
        self.resolution = resolution
        self.voxels = defaultdict(float)  # Store log-odds values
        self.dynamic_expansion = dynamic_expansion

        # Map bounds (for dynamic expansion)
        self.min_bounds = np.array([float('inf')] * 3)
        self.max_bounds = np.array([-float('inf')] * 3)

        # Log-odds parameters (will be set from config)
        self.log_odds_occupied = 1.5      # Log-odds increment for occupied
        self.log_odds_free = -2.0         # Log-odds decrement for free space (strong)
        self.log_odds_min = -2.0         # Minimum log-odds (clamping)
        self.log_odds_max = 3.5          # Maximum log-odds (clamping)
        self.log_odds_threshold = 0.0    # Threshold for considering occupied
        self.adaptive_update = True       # Enable adaptive updating by default
        self.adaptive_threshold = 0.5     # Protection threshold (will be set from config)
        self.adaptive_max_ratio = 0.5     # Maximum update ratio at adaptive_threshold

    def get_voxel_key(self, point):
        """
        Get voxel key from 3D point

        Args:
            point: [x, y, z] numpy array

        Returns:
            Tuple (ix, iy, iz) as voxel index
        """
        voxel_idx = np.floor(point / self.resolution).astype(int)
        return tuple(voxel_idx)

    def world_to_key(self, x, y, z):
        """
        Convert world coordinates to voxel key

        Args:
            x, y, z: World coordinates

        Returns:
            Tuple (ix, iy, iz) as voxel index
        """
        return self.get_voxel_key(np.array([x, y, z]))

    def update_voxel(self, point, log_odds_update, adaptive=True):
        """
        Update voxel log-odds value with optional adaptive updating

        Args:
            point: [x, y, z] numpy array
            log_odds_update: Log-odds increment/decrement
            adaptive: If True, reduce occupied updates for free space voxels
        """
        key = self.get_voxel_key(point)

        # Adaptive update: reduce occupied updates for voxels that are likely free
        if adaptive and log_odds_update > 0:  # If updating as occupied
            current_log_odds = self.voxels.get(key, 0.0)
            current_prob = 1.0 / (1.0 + np.exp(-current_log_odds))

            # Protection for free space (prob <= adaptive_threshold)
            # Maximum adaptive_max_ratio update rate for voxels with prob <= adaptive_threshold
            if current_prob <= self.adaptive_threshold:
                # Linear interpolation with adaptive_max_ratio maximum
                # prob = 0.0 -> scale = 0.0 (no update)
                # prob = adaptive_threshold -> scale = adaptive_max_ratio
                # Linear scaling: scale = (current_prob / adaptive_threshold) * adaptive_max_ratio
                update_scale = (current_prob / self.adaptive_threshold) * self.adaptive_max_ratio
                log_odds_update *= update_scale

        # Apply update
        if key not in self.voxels:
            self.voxels[key] = 0.0
        self.voxels[key] += log_odds_update
        # Clamp to prevent overflow
        self.voxels[key] = np.clip(self.voxels[key],
                                   self.log_odds_min,
                                   self.log_odds_max)

        # Update bounds for dynamic expansion
        if self.dynamic_expansion:
            self.min_bounds = np.minimum(self.min_bounds, point)
            self.max_bounds = np.maximum(self.max_bounds, point)

    def get_probability(self, point):
        """
        Get probability from log-odds value

        Args:
            point: [x, y, z] numpy array

        Returns:
            Probability [0, 1]
        """
        key = self.get_voxel_key(point)
        log_odds = self.voxels.get(key, 0.0)
        # Convert log-odds to probability
        return 1.0 / (1.0 + np.exp(-log_odds))

    def get_occupied_voxels(self, min_probability=0.5):
        """
        Get all occupied voxels above probability threshold

        Args:
            min_probability: Minimum probability to consider occupied

        Returns:
            List of (point, probability) tuples
        """
        occupied = []
        # Handle edge cases for min_probability
        if min_probability >= 1.0:
            # Probability 1.0 means only return maximum log-odds voxels
            min_log_odds = self.log_odds_max - 0.01  # Just below max
        elif min_probability <= 0.0:
            min_log_odds = self.log_odds_min
        else:
            min_log_odds = np.log(min_probability / (1.0 - min_probability))

        # Debug: Count voxels at different probability ranges
        prob_ranges = {"0.5-0.7": 0, "0.7-0.9": 0, "0.9-0.95": 0, "0.95-0.99": 0, "0.99+": 0}

        for key, log_odds in self.voxels.items():
            probability = 1.0 / (1.0 + np.exp(-log_odds))

            # Count probability ranges
            if probability >= 0.99:
                prob_ranges["0.99+"] += 1
            elif probability >= 0.95:
                prob_ranges["0.95-0.99"] += 1
            elif probability >= 0.9:
                prob_ranges["0.9-0.95"] += 1
            elif probability >= 0.7:
                prob_ranges["0.7-0.9"] += 1
            elif probability >= 0.5:
                prob_ranges["0.5-0.7"] += 1

            if log_odds > min_log_odds:
                # Convert voxel index back to 3D point (center of voxel)
                point = np.array(key) * self.resolution + self.resolution / 2
                occupied.append((point, probability))

        # Print debug info
        if len(self.voxels) > 0 and len(occupied) > 0:
            print(f"\nVoxel probability distribution (min_prob={min_probability:.3f}):")
            print(f"  Min log-odds threshold: {min_log_odds:.3f}")
            print(f"  Total voxels: {len(self.voxels)}")
            for range_name, count in prob_ranges.items():
                if count > 0:
                    print(f"  {range_name}: {count} voxels")
            print(f"  Returned as occupied: {len(occupied)} voxels")
            if len(occupied) > 0:
                probs = [p[1] for p in occupied]
                log_odds_vals = [self.voxels.get(self.get_voxel_key(p[0]), 0) for p in occupied[:5]]
                print(f"  Occupied prob range: [{min(probs):.3f}, {max(probs):.3f}]")
                print(f"  Sample log-odds: {log_odds_vals[:3]}")

        return occupied

    def get_all_voxels_classified(self, min_probability=0.7):
        """
        Get all voxels classified as free, unknown, or occupied

        Args:
            min_probability: Minimum probability to consider occupied (0.0-1.0)

        Returns:
            Dictionary with 'free', 'unknown', 'occupied' lists
        """
        free = []
        unknown = []
        occupied = []

        # Convert 0.3 probability to log-odds for free threshold (prob < 0.3 = free)
        free_threshold = np.log(0.3 / (1.0 - 0.3))  # log-odds < this -> free
        # Convert min_probability to log-odds threshold for occupied
        if min_probability >= 1.0:
            occupied_threshold = self.log_odds_max - 0.01
        elif min_probability <= 0.0:
            occupied_threshold = self.log_odds_min
        else:
            occupied_threshold = np.log(min_probability / (1.0 - min_probability))

        for key, log_odds in self.voxels.items():
            # Convert voxel index back to 3D point (center of voxel)
            point = np.array(key) * self.resolution + self.resolution / 2
            probability = 1.0 / (1.0 + np.exp(-log_odds))

            if log_odds < free_threshold:
                free.append((point, probability))
            elif log_odds > occupied_threshold:  # Only if above min_probability threshold
                occupied.append((point, probability))
            else:
                unknown.append((point, probability))

        return {
            'free': free,
            'unknown': unknown,
            'occupied': occupied
        }

    def clear(self):
        """Clear all voxels"""
        self.voxels.clear()


class SonarMapping3D:
    """
    Convert sonar images to 3D point clouds with probabilistic mapping
    Accumulates multiple frames and updates voxel probabilities

    Integrated with stonefish_slam SLAM system using keyframe-based updates.
    """

    def __init__(self, config=None):
        """
        Initialize with sonar parameters

        Args:
            config: Optional dictionary with custom parameters
        """
        # Default sonar parameters for BlueROV2 with FLS
        default_config = {
            'horizontal_fov': 130.0,      # degrees (-65 to +65)
            'vertical_aperture': 20.0,     # degrees (vertical aperture)
            'max_range': 30.0,             # meters (updated from 20.0)
            'min_range': 0.5,              # meters (updated from 0.2)
            'intensity_threshold': 50,     # 0-255 scale (match 2D mapping)
            'image_width': 512,            # Sonar image width (bearings)
            'image_height': 500,          # Sonar image height (ranges)
            # Sonar mounting (BlueROV2 actual mounting, FRD frame)
            'sonar_position': [0.25, 0.0, 0.08],  # meters from base_link
            'sonar_tilt': 0.5236,          # 30° downward (radians)
            # Octree parameters
            'voxel_resolution': 0.05,      # 5cm voxels (recommended default)
            'min_probability': 0.6,        # Minimum probability for occupied
            'max_frames': 0,               # 0=unlimited
            'dynamic_expansion': True,     # Enable dynamic map expansion
            'adaptive_update': True,       # Enable adaptive updating (linear protection)
        }

        # Update with provided config if any
        if config:
            default_config.update(config)

        # Store parameters
        self.horizontal_fov = np.radians(default_config['horizontal_fov'])
        self.vertical_aperture = np.radians(default_config['vertical_aperture'])
        self.max_range = default_config['max_range']
        self.min_range = default_config['min_range']
        self.intensity_threshold = default_config['intensity_threshold']
        self.image_width = default_config['image_width']
        self.image_height = default_config['image_height']
        self.voxel_resolution = default_config['voxel_resolution']
        self.min_probability = default_config['min_probability']
        self.max_frames = default_config['max_frames']
        self.dynamic_expansion = default_config['dynamic_expansion']

        # Sonar mounting transform (updated to use tilt instead of full RPY)
        self.sonar_position = np.array(default_config['sonar_position'])
        self.sonar_tilt = default_config['sonar_tilt']

        # Pre-compute sonar to base_link transform
        self.T_sonar_to_base = self.create_transform_matrix(
            self.sonar_position,
            self.sonar_tilt
        )

        # Pre-compute angles for efficiency
        self.bearing_angles = np.linspace(
            -self.horizontal_fov/2,
            self.horizontal_fov/2,
            self.image_width
        )

        # Range resolution
        self.range_resolution = self.max_range / self.image_height

        # Get adaptive update settings from config
        self.adaptive_update = default_config.get('adaptive_update', True)
        self.adaptive_threshold = default_config.get('adaptive_threshold', 0.5)
        self.adaptive_max_ratio = default_config.get('adaptive_max_ratio', 0.5)

        # Get log-odds parameters from config
        self.log_odds_occupied = default_config.get('log_odds_occupied', 1.5)
        self.log_odds_free = default_config.get('log_odds_free', -2.0)
        self.log_odds_min = default_config.get('log_odds_min', -10.0)
        self.log_odds_max = default_config.get('log_odds_max', 10.0)

        # Initialize octree for voxel storage with dynamic expansion
        self.octree = SimpleOctree(self.voxel_resolution, dynamic_expansion=self.dynamic_expansion)

        # Pass all settings to octree
        self.octree.adaptive_update = self.adaptive_update
        self.octree.adaptive_threshold = self.adaptive_threshold
        self.octree.adaptive_max_ratio = self.adaptive_max_ratio
        self.octree.log_odds_occupied = self.log_odds_occupied
        self.octree.log_odds_free = self.log_odds_free
        self.octree.log_odds_min = self.log_odds_min
        self.octree.log_odds_max = self.log_odds_max

        # Debug: Print actual values being used
        print(f"  Octree log_odds_occupied: {self.octree.log_odds_occupied}")
        print(f"  Octree log_odds_free: {self.octree.log_odds_free}")
        print(f"  Octree adaptive_threshold: {self.octree.adaptive_threshold}")
        print(f"  Octree adaptive_max_ratio: {self.octree.adaptive_max_ratio}")

        # Frame counter
        self.frame_count = 0

        print(f"SonarMapping3D initialized:")
        print(f"  Horizontal FOV: {default_config['horizontal_fov']}°")
        print(f"  Vertical aperture: {default_config['vertical_aperture']}°")
        print(f"  Range: {self.min_range}-{self.max_range}m")
        print(f"  Intensity threshold: {self.intensity_threshold}")
        print(f"  Voxel resolution: {self.voxel_resolution}m")
        print(f"  Min probability: {self.min_probability}")
        # Debug: Show what log-odds threshold this corresponds to
        if self.min_probability >= 1.0:
            print(f"    -> Using max log-odds threshold: {self.log_odds_max - 0.01:.2f}")
        elif self.min_probability > 0 and self.min_probability < 1.0:
            min_log_odds = np.log(self.min_probability / (1.0 - self.min_probability))
            print(f"    -> Log-odds threshold: {min_log_odds:.2f}")
        print(f"  Dynamic expansion: {self.dynamic_expansion}")
        if hasattr(self, 'adaptive_update'):
            print(f"  Adaptive update: {self.adaptive_update} (linear protection up to prob={self.adaptive_threshold}, max_ratio={self.adaptive_max_ratio})")

    def create_transform_matrix(self, position, tilt_rad):
        """
        Create 4x4 transform with tilt angle (pitch rotation)

        Args:
            position: [x, y, z] translation
            tilt_rad: Tilt angle in radians (pitch rotation around Y axis)

        Returns:
            4x4 numpy array transform matrix
        """
        T = np.eye(4)
        # Rotation: tilt downward (pitch rotation around Y axis)
        rot = R.from_euler('y', tilt_rad)  # Pitch around Y axis
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = position
        return T

    def pose_msg_to_transform(self, pose_msg):
        """
        Convert pose (dict or ROS message) to 4x4 transform matrix

        Args:
            pose_msg: Dict with 'position' and 'orientation', or geometry_msgs/Pose

        Returns:
            4x4 numpy array transform matrix
        """
        # Handle dict format (from our SLAM integration)
        if isinstance(pose_msg, dict):
            position = [
                pose_msg['position']['x'],
                pose_msg['position']['y'],
                pose_msg['position']['z']
            ]
            quaternion = [
                pose_msg['orientation']['x'],
                pose_msg['orientation']['y'],
                pose_msg['orientation']['z'],
                pose_msg['orientation']['w']
            ]
        else:
            # Handle ROS message format
            if hasattr(pose_msg, 'pose'):
                pose = pose_msg.pose
            else:
                pose = pose_msg

            position = [pose.position.x, pose.position.y, pose.position.z]
            quaternion = [pose.orientation.x, pose.orientation.y,
                         pose.orientation.z, pose.orientation.w]

        # Create transform using scipy
        T = np.eye(4)
        rot = R.from_quat(quaternion)  # scipy uses [x, y, z, w] format
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = position

        return T

    def process_sonar_ray(self, bearing_angle, intensity_profile, T_sonar_to_world, voxel_updates):
        """
        Process a single sonar ray (bearing) and accumulate voxel updates

        Args:
            bearing_angle: Horizontal angle in radians
            intensity_profile: 1D array of intensities along range
            T_sonar_to_world: 4x4 transform matrix from sonar to world
            voxel_updates: Dictionary to accumulate updates per voxel
        """
        # Find all high intensity (occupied) regions
        first_hit_idx = -1
        last_hit_idx = -1
        high_intensity_indices = []

        for r_idx, intensity in enumerate(intensity_profile):
            if intensity > self.intensity_threshold:
                if first_hit_idx == -1:
                    first_hit_idx = r_idx
                last_hit_idx = r_idx
                high_intensity_indices.append(r_idx)

        # If no hit found, entire ray is free space up to max range
        if first_hit_idx == -1:
            first_hit_idx = len(intensity_profile)  # Process entire ray as free

        # Calculate vertical aperture parameters
        half_aperture = self.vertical_aperture / 2

        # Update free space before first hit (with sparse sampling)
        free_sampling_step = 10  # Could be made configurable if needed
        for r_idx in range(0, first_hit_idx, free_sampling_step):
            range_m = r_idx * self.range_resolution
            if range_m < self.min_range:
                continue

            # Calculate vertical spread at this range
            vertical_spread = range_m * np.tan(half_aperture)
            # Sparse vertical sampling for free space
            free_vertical_factor = 4.0  # Could be made configurable
            num_vertical_steps = max(1, int(vertical_spread / (self.voxel_resolution * free_vertical_factor)))

            for v_step in range(-num_vertical_steps, num_vertical_steps + 1):
                # Calculate vertical angle within aperture
                vertical_angle = (v_step / max(1, num_vertical_steps)) * half_aperture

                # Calculate 3D position in sonar frame
                # Sonar coordinate system: X=forward, Y=right, Z=down (FRD)
                # CRITICAL: Keep the MINUS sign for y_sonar (coordinate system convention)
                x_sonar = range_m * np.cos(vertical_angle) * np.cos(bearing_angle)
                y_sonar = -range_m * np.cos(vertical_angle) * np.sin(bearing_angle)
                z_sonar = range_m * np.sin(vertical_angle)

                # Transform to world frame
                pt_sonar = np.array([x_sonar, y_sonar, z_sonar, 1.0])
                pt_world = T_sonar_to_world @ pt_sonar

                # Get voxel key and accumulate update
                voxel_key = self.octree.world_to_key(pt_world[0], pt_world[1], pt_world[2])

                # Accumulate free space updates
                if voxel_key not in voxel_updates:
                    voxel_updates[voxel_key] = {'point': pt_world[:3], 'sum': 0.0, 'count': 0, 'type': 'free'}
                voxel_updates[voxel_key]['sum'] += self.octree.log_odds_free
                voxel_updates[voxel_key]['count'] += 1

        # Update occupied regions ONLY
        # Process only the high intensity (occupied) regions we found
        for r_idx in high_intensity_indices:
            range_m = r_idx * self.range_resolution
            if range_m < self.min_range or range_m > self.max_range:
                continue

            # Calculate vertical spread at this range
            vertical_spread = range_m * np.tan(half_aperture)
            # Denser vertical sampling for occupied space
            occupied_vertical_factor = 1.5  # Could be made configurable
            num_vertical_steps = max(2, int(vertical_spread / (self.voxel_resolution * occupied_vertical_factor)))

            # This is a high intensity region: mark as occupied
            log_odds_update = self.octree.log_odds_occupied

            for v_step in range(-num_vertical_steps, num_vertical_steps + 1):
                # Calculate vertical angle within aperture
                vertical_angle = (v_step / max(1, num_vertical_steps)) * half_aperture

                # Calculate 3D position
                x_sonar = range_m * np.cos(vertical_angle) * np.cos(bearing_angle)
                y_sonar = -range_m * np.cos(vertical_angle) * np.sin(bearing_angle)
                z_sonar = range_m * np.sin(vertical_angle)

                # Transform to world frame
                pt_sonar = np.array([x_sonar, y_sonar, z_sonar, 1.0])
                pt_world = T_sonar_to_world @ pt_sonar

                # Get voxel key and accumulate update
                voxel_key = self.octree.world_to_key(pt_world[0], pt_world[1], pt_world[2])

                # Accumulate occupied updates
                if voxel_key not in voxel_updates:
                    voxel_updates[voxel_key] = {'point': pt_world[:3], 'sum': 0.0, 'count': 0, 'type': 'occupied'}
                elif voxel_updates[voxel_key]['type'] == 'free':
                    # If previously marked as free, switch to occupied (occupied has priority)
                    voxel_updates[voxel_key]['type'] = 'occupied'
                    voxel_updates[voxel_key]['sum'] = 0.0  # Reset sum for occupied
                    voxel_updates[voxel_key]['count'] = 0

                voxel_updates[voxel_key]['sum'] += log_odds_update
                voxel_updates[voxel_key]['count'] += 1

    def process_sonar_image(self, polar_image, robot_pose):
        """
        Process sonar image and update probabilistic map

        Args:
            polar_image: 2D numpy array (height x width) with intensity values
            robot_pose: Robot pose (dict with 'position'/'orientation' or ROS Pose message)
        """
        # Ensure image is numpy array
        if not isinstance(polar_image, np.ndarray):
            polar_image = np.array(polar_image)

        # Get image dimensions
        range_bins, bearing_bins = polar_image.shape

        # Check if dimensions match expected
        if bearing_bins != self.image_width:
            # Resize bearing angles if needed
            self.bearing_angles = np.linspace(
                -self.horizontal_fov/2,
                self.horizontal_fov/2,
                bearing_bins
            )

        if range_bins != self.image_height:
            # Update range resolution if needed
            self.range_resolution = self.max_range / range_bins

        # Get robot transform
        T_base_to_world = self.pose_msg_to_transform(robot_pose)
        T_sonar_to_world = T_base_to_world @ self.T_sonar_to_base

        # Initialize voxel update accumulator for this frame
        # Dictionary to accumulate updates: key -> (sum_updates, count)
        voxel_updates = {}  # Will store accumulated updates per voxel

        # Process subset of bearings for efficiency
        bearing_divisor = 256  # Could be made configurable
        bearing_step = max(1, bearing_bins // bearing_divisor)

        # Debug: Check if we have any data to process
        max_intensity = np.max(polar_image)
        if max_intensity > 0:
            print(f"Processing image with max intensity: {max_intensity}, threshold: {self.intensity_threshold}")

        for b_idx in range(0, bearing_bins, bearing_step):
            bearing_angle = self.bearing_angles[b_idx]
            intensity_profile = polar_image[:, b_idx]

            # Process this ray and accumulate updates
            self.process_sonar_ray(bearing_angle, intensity_profile, T_sonar_to_world, voxel_updates)

        # Apply averaged updates to all voxels
        for voxel_key, update_info in voxel_updates.items():
            # Calculate average update
            avg_update = update_info['sum'] / update_info['count']

            # Apply the averaged update
            if update_info['type'] == 'occupied':
                self.octree.update_voxel(update_info['point'], avg_update, adaptive=self.octree.adaptive_update)
            else:  # free
                self.octree.update_voxel(update_info['point'], avg_update, adaptive=False)

            # Debug: Log some statistics
            if np.random.random() < 0.001:  # Sample 0.1% for better debugging
                print(f"Voxel {voxel_key}: {update_info['count']} rays, avg_update={avg_update:.4f}, type={update_info['type']}, log_odds_occupied={self.octree.log_odds_occupied}")

        # Debug: Print summary statistics
        if self.frame_count % 100 == 0:
            occupied_voxels = sum(1 for v in voxel_updates.values() if v['type'] == 'occupied')
            free_voxels = sum(1 for v in voxel_updates.values() if v['type'] == 'free')
            multi_ray_voxels = sum(1 for v in voxel_updates.values() if v['count'] > 1)
            print(f"Frame {self.frame_count}: {occupied_voxels} occupied, {free_voxels} free, {multi_ray_voxels} multi-ray voxels")

        # Increment frame counter
        self.frame_count += 1

        # Optional: Clear old voxels if frame limit reached
        if self.max_frames > 0 and self.frame_count > self.max_frames:
            # Simple approach: clear and restart
            # More sophisticated: implement sliding window or decay
            self.octree.clear()
            self.frame_count = 0
            print(f"Reached max frames ({self.max_frames}), clearing map")

    def update_map_from_slam(self, new_keyframes, all_slam_keyframes=None):
        """
        Update 3D map from SLAM keyframes (similar to mapping_2d.update_global_map_from_slam)

        Args:
            new_keyframes: List of new Keyframe objects
            all_slam_keyframes: Complete list of keyframes (for bounds - not used in 3D)
        """
        for kf in new_keyframes:
            # Keyframe.image contains the polar sonar image
            polar_img = kf.image

            # Skip if no image available
            if polar_img is None:
                continue

            # Convert gtsam.Pose2 to simple pose dict for processing
            # (we'll use x, y, yaw from Pose2, assume z=0, roll=0, pitch=0)
            pose_dict = {
                'position': {'x': kf.pose.x(),
                            'y': kf.pose.y(),
                            'z': 0.0},  # Assume planar motion
                'orientation': {'x': 0.0, 'y': 0.0,
                               'z': np.sin(kf.pose.theta()/2),
                               'w': np.cos(kf.pose.theta()/2)}
            }

            self.process_sonar_image(polar_img, pose_dict)

    def get_point_cloud(self, include_free=False):
        """
        Get current point cloud from probabilistic map

        Args:
            include_free: Whether to include free space voxels

        Returns:
            Dictionary containing classified voxels and statistics
        """
        if include_free:
            # Get all classified voxels with min_probability filter
            classified = self.octree.get_all_voxels_classified(self.min_probability)

            result = {
                'occupied': classified['occupied'],
                'free': classified['free'],
                'unknown': classified['unknown'],
                'num_voxels': len(self.octree.voxels),
                'num_occupied': len(classified['occupied']),
                'num_free': len(classified['free']),
                'num_unknown': len(classified['unknown']),
                'frame_count': self.frame_count,
                'bounds': {
                    'min': self.octree.min_bounds.copy() if self.octree.dynamic_expansion else None,
                    'max': self.octree.max_bounds.copy() if self.octree.dynamic_expansion else None
                }
            }
        else:
            # Get only occupied voxels (backward compatibility)
            occupied_voxels = self.octree.get_occupied_voxels(self.min_probability)

            if occupied_voxels:
                points = np.array([v[0] for v in occupied_voxels])
                probabilities = np.array([v[1] for v in occupied_voxels])
            else:
                points = np.empty((0, 3))
                probabilities = np.empty(0)

            result = {
                'points': points,
                'probabilities': probabilities,
                'num_voxels': len(self.octree.voxels),
                'num_occupied': len(occupied_voxels),
                'frame_count': self.frame_count
            }

        return result

    def get_pointcloud2_msg(self, frame_id='world_ned', stamp=None):
        """
        Generate ROS2 PointCloud2 message for visualization

        Args:
            frame_id: Coordinate frame (default 'world_ned')
            stamp: ROS timestamp (optional)

        Returns:
            sensor_msgs/PointCloud2 message
        """
        import struct
        from sensor_msgs.msg import PointCloud2, PointField
        from std_msgs.msg import Header

        # Get occupied voxels
        result = self.get_point_cloud(include_free=False)
        points = result['points']
        probs = result['probabilities']

        if len(points) == 0:
            # Return empty point cloud
            msg = PointCloud2()
            msg.header = Header()
            msg.header.frame_id = frame_id
            if stamp:
                msg.header.stamp = stamp
            msg.height = 1
            msg.width = 0
            return msg

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = frame_id
        if stamp:
            msg.header.stamp = stamp

        # Define fields: x, y, z, intensity
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 16  # 4 floats * 4 bytes
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.height = 1
        msg.width = len(points)

        # Pack data: probability -> intensity (0.0-1.0)
        data = []
        for i in range(len(points)):
            x, y, z = points[i]
            intensity = float(probs[i])  # Already 0-1 range
            data.append(struct.pack('ffff', x, y, z, intensity))

        msg.data = b''.join(data)

        return msg

    def reset_map(self):
        """Reset the probabilistic map"""
        self.octree.clear()
        self.frame_count = 0
        print("Map reset")
