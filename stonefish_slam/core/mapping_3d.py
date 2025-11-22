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


class OctNode:
    """
    Single node in hierarchical octree (internal or leaf)

    Uses lazy initialization: children only created when needed
    Stores log-odds occupancy value at this node
    """

    def __init__(self, center, size):
        """
        Initialize octree node

        Args:
            center: [x, y, z] center position of this node
            size: Edge length of this cubic node
        """
        self.center = np.array(center, dtype=np.float64)
        self.size = float(size)
        self.log_odds = 0.0  # Occupancy log-odds value
        self.children = [None] * 8  # 8 children (lazy init)

    def is_leaf(self):
        """Check if this is a leaf node (no children created yet)"""
        return self.children[0] is None

    def get_octant_index(self, point):
        """
        Find which octant (0-7) the point belongs to

        Octant indexing:
        - Bit 0 (value 1): X >= center.x
        - Bit 1 (value 2): Y >= center.y
        - Bit 2 (value 4): Z >= center.z

        Args:
            point: [x, y, z] position

        Returns:
            Octant index 0-7
        """
        index = 0
        if point[0] >= self.center[0]:
            index |= 1  # X bit
        if point[1] >= self.center[1]:
            index |= 2  # Y bit
        if point[2] >= self.center[2]:
            index |= 4  # Z bit
        return index

    def subdivide(self):
        """
        Create 8 children nodes (lazy subdivision)

        Each child inherits parent's log-odds value and has size/2
        """
        half_size = self.size / 2.0
        quarter_size = self.size / 4.0

        for i in range(8):
            # Calculate octant offset based on bit pattern
            x_offset = quarter_size if (i & 1) else -quarter_size
            y_offset = quarter_size if (i & 2) else -quarter_size
            z_offset = quarter_size if (i & 4) else -quarter_size

            child_center = self.center + np.array([x_offset, y_offset, z_offset])
            self.children[i] = OctNode(child_center, half_size)
            # Inherit parent's log-odds value
            self.children[i].log_odds = self.log_odds


class HierarchicalOctree:
    """
    Hierarchical octree for efficient 3D occupancy mapping

    Uses recursive 8-way tree structure for O(log n) update/query complexity
    Implements lazy initialization for sparse memory usage
    Compatible with existing SimpleOctree API
    """

    def __init__(self, resolution=0.1, max_depth=9, center=None, size=None):
        """
        Initialize hierarchical octree

        Args:
            resolution: Minimum voxel size in meters (leaf node size)
            max_depth: Maximum tree depth (9 → 2^9 * resolution = 51.2m for 0.1m res)
            center: Center of root node (default [size/2, size/2, size/2])
            size: Size of root node (default 2^max_depth * resolution)
        """
        self.resolution = float(resolution)
        self.max_depth = int(max_depth)

        # Calculate required size and depth
        if size is None:
            # Auto-size: 2^max_depth * resolution
            size = (2 ** max_depth) * resolution

        if center is None:
            # Default center at world origin to cover negative coordinates
            center = [0.0, 0.0, 0.0]

        self.root = OctNode(center, size)

        # Log-odds parameters (same as SimpleOctree)
        self.log_odds_occupied = 1.5
        self.log_odds_free = -2.0
        self.log_odds_min = -10.0
        self.log_odds_max = 10.0
        self.log_odds_threshold = 0.0
        self.adaptive_update = True
        self.adaptive_threshold = 0.5
        self.adaptive_max_ratio = 0.5

        # Bounds tracking (for compatibility)
        self.min_bounds = np.array([float('inf')] * 3)
        self.max_bounds = np.array([-float('inf')] * 3)
        self.dynamic_expansion = True

    def update_voxel(self, point, log_odds_update, adaptive=True):
        """
        Update voxel at point with log-odds increment/decrement

        Args:
            point: [x, y, z] position (or longer array, only first 3 used)
            log_odds_update: Log-odds change to apply
            adaptive: If True, apply adaptive update for occupied voxels
        """
        point = np.array(point[:3], dtype=np.float64)  # Ensure 3D

        # Adaptive update logic (same as SimpleOctree)
        if adaptive and log_odds_update > 0:
            current_log_odds = self.query_voxel(point)
            current_prob = 1.0 / (1.0 + np.exp(-current_log_odds))

            if current_prob <= self.adaptive_threshold:
                # Linear scaling: reduce update for free space voxels
                update_scale = (current_prob / self.adaptive_threshold) * self.adaptive_max_ratio
                log_odds_update *= update_scale

        # Update recursively
        self._update_recursive(self.root, point, log_odds_update, depth=0)

        # Update bounds
        if self.dynamic_expansion:
            self.min_bounds = np.minimum(self.min_bounds, point)
            self.max_bounds = np.maximum(self.max_bounds, point)

    def _update_recursive(self, node, point, log_odds_update, depth):
        """
        Recursive update to target voxel

        Args:
            node: Current OctNode
            point: Target position
            log_odds_update: Log-odds change
            depth: Current tree depth
        """
        # Check if we've reached resolution or max depth
        if depth >= self.max_depth or node.size <= self.resolution * 2:
            # Leaf node: update log-odds
            node.log_odds += log_odds_update
            node.log_odds = np.clip(node.log_odds, self.log_odds_min, self.log_odds_max)
            return

        # Internal node: subdivide if needed and recurse
        if node.is_leaf():
            node.subdivide()

        child_index = node.get_octant_index(point)
        self._update_recursive(node.children[child_index], point, log_odds_update, depth + 1)

    def query_voxel(self, point):
        """
        Query log-odds value at point

        Args:
            point: [x, y, z] position

        Returns:
            Log-odds value at this position
        """
        point = np.array(point[:3], dtype=np.float64)
        return self._query_recursive(self.root, point, depth=0)

    def _query_recursive(self, node, point, depth):
        """
        Recursive query for log-odds value

        Args:
            node: Current OctNode
            point: Target position
            depth: Current tree depth

        Returns:
            Log-odds value
        """
        if depth >= self.max_depth or node.is_leaf():
            return node.log_odds

        child_index = node.get_octant_index(point)
        return self._query_recursive(node.children[child_index], point, depth + 1)

    def get_occupied_voxels(self, min_probability=0.5):
        """
        Get all occupied voxels above threshold

        Args:
            min_probability: Minimum probability to consider occupied

        Returns:
            List of (point, probability) tuples
        """
        occupied = []

        # Convert probability to log-odds threshold
        if min_probability >= 1.0:
            min_log_odds = self.log_odds_max - 0.01
        elif min_probability <= 0.0:
            min_log_odds = self.log_odds_min
        else:
            min_log_odds = np.log(min_probability / (1.0 - min_probability))

        # Traverse tree and collect occupied leaves
        self._collect_occupied(self.root, min_log_odds, occupied, depth=0)

        return occupied

    def _collect_occupied(self, node, min_log_odds, occupied, depth):
        """
        Recursively collect occupied voxels

        Args:
            node: Current OctNode
            min_log_odds: Minimum log-odds threshold
            occupied: List to append results to
            depth: Current tree depth
        """
        # If leaf or at max depth
        if depth >= self.max_depth or node.is_leaf():
            # Only check threshold at leaf nodes
            if node.log_odds > min_log_odds:
                probability = 1.0 / (1.0 + np.exp(-node.log_odds))
                occupied.append((node.center.copy(), probability))
            return

        # Recurse into children (internal nodes don't have meaningful log_odds)
        for child in node.children:
            if child is not None:
                self._collect_occupied(child, min_log_odds, occupied, depth + 1)

    def get_all_voxels_classified(self, min_probability=0.7):
        """
        Get free/unknown/occupied classification (compatibility method)

        Args:
            min_probability: Minimum probability to consider occupied

        Returns:
            Dictionary with 'free', 'unknown', 'occupied' lists
        """
        free = []
        unknown = []
        occupied = []

        free_threshold = np.log(0.3 / 0.7)  # prob < 0.3 = free
        occupied_threshold = np.log(min_probability / (1.0 - min_probability))

        self._classify_voxels(self.root, free_threshold, occupied_threshold,
                             free, unknown, occupied, depth=0)

        return {'free': free, 'unknown': unknown, 'occupied': occupied}

    def _classify_voxels(self, node, free_thresh, occ_thresh, free, unknown, occupied, depth):
        """
        Recursively classify voxels

        Args:
            node: Current OctNode
            free_thresh: Free space threshold
            occ_thresh: Occupied space threshold
            free, unknown, occupied: Lists to append results to
            depth: Current tree depth
        """
        if depth >= self.max_depth or node.is_leaf():
            probability = 1.0 / (1.0 + np.exp(-node.log_odds))

            if node.log_odds < free_thresh:
                free.append((node.center.copy(), probability))
            elif node.log_odds > occ_thresh:
                occupied.append((node.center.copy(), probability))
            else:
                unknown.append((node.center.copy(), probability))
            return

        # Recurse into children
        for child in node.children:
            if child is not None:
                self._classify_voxels(child, free_thresh, occ_thresh,
                                    free, unknown, occupied, depth + 1)

    def clear(self):
        """Clear all data (recreate root)"""
        self.root = OctNode(self.root.center, self.root.size)
        self.min_bounds = np.array([float('inf')] * 3)
        self.max_bounds = np.array([-float('inf')] * 3)

    def world_to_key(self, x, y, z):
        """
        Compatibility method (not used in hierarchical octree)

        Returns quantized voxel key for compatibility with existing code
        """
        ix = int(np.floor(x / self.resolution))
        iy = int(np.floor(y / self.resolution))
        iz = int(np.floor(z / self.resolution))
        return (ix, iy, iz)

    def get_voxel_key(self, point):
        """Compatibility method"""
        return self.world_to_key(point[0], point[1], point[2])

    @property
    def voxels(self):
        """
        Compatibility: return dict-like view of octree

        Returns object with __len__ that counts all leaf nodes
        """
        count = {'total': 0}
        self._count_leaves(self.root, count, depth=0)

        # Return a fake dict-like object
        class VoxelDict:
            def __init__(self, total):
                self._total = total
            def __len__(self):
                return self._total

        return VoxelDict(count['total'])

    def _count_leaves(self, node, count, depth):
        """Count all leaf nodes"""
        if depth >= self.max_depth or node.is_leaf():
            count['total'] += 1
            return

        for child in node.children:
            if child is not None:
                self._count_leaves(child, count, depth + 1)


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
            'min_range': 0.5,              # meters
            'intensity_threshold': 50,     # 0-255 scale (match 2D mapping)
            'image_width': 512,            # Sonar image width (bearings)
            'image_height': 500,          # Sonar image height (ranges)
            # Sonar mounting (BlueROV2 actual mounting, FRD frame)
            'sonar_position': [0.25, 0.0, 0.08],  # meters from base_link
            'sonar_tilt': -0.5236,         # 30° downward (negative pitch in FRD)
            # Octree parameters
            'voxel_resolution': 0.1,       # 10cm voxels (hierarchical octree optimized)
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

        # Range resolution (bin size in meters)
        self.range_resolution = (self.max_range - self.min_range) / self.image_height

        # Get adaptive update settings from config
        self.adaptive_update = default_config.get('adaptive_update', True)
        self.adaptive_threshold = default_config.get('adaptive_threshold', 0.5)
        self.adaptive_max_ratio = default_config.get('adaptive_max_ratio', 0.5)

        # Get log-odds parameters from config
        self.log_odds_occupied = default_config.get('log_odds_occupied', 1.5)
        self.log_odds_free = default_config.get('log_odds_free', -2.0)
        self.log_odds_min = default_config.get('log_odds_min', -10.0)
        self.log_odds_max = default_config.get('log_odds_max', 10.0)

        # Initialize hierarchical octree for voxel storage
        self.octree = HierarchicalOctree(
            resolution=self.voxel_resolution,
            max_depth=9,  # Optimal for 30m space (2^9 * 0.1m = 51.2m)
            center=None,  # Auto-calculated
            size=None  # Auto-calculated
        )

        # Pass all settings to octree
        self.octree.adaptive_update = self.adaptive_update
        self.octree.adaptive_threshold = self.adaptive_threshold
        self.octree.adaptive_max_ratio = self.adaptive_max_ratio
        self.octree.log_odds_occupied = self.log_odds_occupied
        self.octree.log_odds_free = self.log_odds_free
        self.octree.log_odds_min = self.log_odds_min
        self.octree.log_odds_max = self.log_odds_max

        # Frame counter
        self.frame_count = 0

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
        # tilt_rad is negative for downward tilt in FRD frame
        rot = R.from_euler('y', tilt_rad)  # Pitch around Y axis (tilt_rad already negative)
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

        # DEBUG: Print ray processing info for first frame, first few bearings
        if self.frame_count == 0 and abs(bearing_angle) < 0.5:  # Central bearings only
            max_intensity = np.max(intensity_profile)
            print(f"\n=== Ray bearing={np.degrees(bearing_angle):.1f}°, max_intensity={max_intensity} ===")
            print(f"  Threshold: {self.intensity_threshold}")
            print(f"  First hit index: {first_hit_idx}")
            print(f"  High intensity indices: {high_intensity_indices[:5] if len(high_intensity_indices) > 0 else 'None'}")

        # If no hit found, skip this ray entirely (no information)
        if first_hit_idx == -1:
            # No reflection: don't update as free space (we don't know if it's free or just out of range)
            if self.frame_count == 0 and abs(bearing_angle) < 0.5:
                print(f"  → SKIPPED: No reflection detected (no map update)")
            return

        # Calculate vertical aperture parameters
        half_aperture = self.vertical_aperture / 2

        # DEBUG: Print free space processing
        if self.frame_count == 0 and abs(bearing_angle) < 0.5:
            print(f"  Free space: Processing r_idx 0 to {first_hit_idx} (before first hit)")

        # Update free space before first hit (with sparse sampling)
        free_sampling_step = 2  # Reduced from 10 to 2 for better coverage (0.12m intervals)
        for r_idx in range(0, first_hit_idx, free_sampling_step):
            # Calculate actual range (FLS image: row 0 = far, row max = near)
            range_m = self.max_range - r_idx * self.range_resolution

            # Calculate vertical spread at this range
            vertical_spread = range_m * np.tan(half_aperture)
            # Sparse vertical sampling for free space
            free_vertical_factor = 8.0  # Increased from 4.0 to reduce point count
            num_vertical_steps = max(1, int(vertical_spread / (self.voxel_resolution * free_vertical_factor)))

            for v_step in range(-num_vertical_steps, num_vertical_steps + 1):
                # Calculate vertical angle within aperture
                vertical_angle = (v_step / max(1, num_vertical_steps)) * half_aperture

                # Calculate 3D position in sonar frame
                # Sonar coordinate system: X=forward, Y=right, Z=down (FRD)
                # Bearing: negative=left, positive=right
                x_sonar = range_m * np.cos(vertical_angle) * np.cos(bearing_angle)
                y_sonar = range_m * np.cos(vertical_angle) * np.sin(bearing_angle)
                z_sonar = range_m * np.sin(vertical_angle)

                # Transform to world frame
                pt_sonar = np.array([x_sonar, y_sonar, z_sonar, 1.0])
                pt_world = T_sonar_to_world @ pt_sonar

                # CRITICAL: Check actual range after transform
                sonar_origin_world = T_sonar_to_world[:3, 3]
                actual_range = np.linalg.norm(pt_world[:3] - sonar_origin_world)
                if actual_range <= self.min_range:
                    continue  # Skip points at or below min_range

                # Get voxel key and accumulate update
                voxel_key = self.octree.world_to_key(pt_world[0], pt_world[1], pt_world[2])

                # Accumulate log-odds updates (no type distinction)
                if voxel_key not in voxel_updates:
                    voxel_updates[voxel_key] = {'point': pt_world[:3], 'sum': 0.0, 'count': 0}
                voxel_updates[voxel_key]['sum'] += self.octree.log_odds_free
                voxel_updates[voxel_key]['count'] += 1

        # Update occupied regions ONLY
        # Process only the high intensity (occupied) regions we found
        for r_idx in high_intensity_indices:
            # Calculate actual range (FLS image: row 0 = far, row max = near)
            range_m = self.max_range - r_idx * self.range_resolution

            # Skip out-of-range (should not happen with correct calculation)
            if range_m > self.max_range:
                continue

            # Calculate vertical spread at this range
            vertical_spread = range_m * np.tan(half_aperture)
            # Denser vertical sampling for occupied space
            occupied_vertical_factor = 3.0  # Increased from 1.5 to reduce point count
            num_vertical_steps = max(2, int(vertical_spread / (self.voxel_resolution * occupied_vertical_factor)))

            # DEBUG: Print vertical sampling for first ray
            if self.frame_count == 0 and abs(bearing_angle) < 0.5 and r_idx == high_intensity_indices[0]:
                print(f"    r_idx={r_idx}, range_m={range_m:.2f}m, vertical_spread={vertical_spread:.3f}m, num_vertical_steps={num_vertical_steps}")

            # This is a high intensity region: mark as occupied
            log_odds_update = self.octree.log_odds_occupied

            for v_step in range(-num_vertical_steps, num_vertical_steps + 1):
                # Calculate vertical angle within aperture
                vertical_angle = (v_step / max(1, num_vertical_steps)) * half_aperture

                # Calculate 3D position in sonar frame
                # Bearing: negative=left, positive=right
                x_sonar = range_m * np.cos(vertical_angle) * np.cos(bearing_angle)
                y_sonar = range_m * np.cos(vertical_angle) * np.sin(bearing_angle)
                z_sonar = range_m * np.sin(vertical_angle)

                # Transform to world frame
                pt_sonar = np.array([x_sonar, y_sonar, z_sonar, 1.0])
                pt_world = T_sonar_to_world @ pt_sonar

                # CRITICAL: Check actual range after transform
                sonar_origin_world = T_sonar_to_world[:3, 3]
                actual_range = np.linalg.norm(pt_world[:3] - sonar_origin_world)
                if actual_range <= self.min_range:
                    continue  # Skip points at or below min_range

                # Get voxel key and accumulate update
                voxel_key = self.octree.world_to_key(pt_world[0], pt_world[1], pt_world[2])

                # Accumulate log-odds updates (no type distinction - just add)
                if voxel_key not in voxel_updates:
                    voxel_updates[voxel_key] = {'point': pt_world[:3], 'sum': 0.0, 'count': 0}
                voxel_updates[voxel_key]['sum'] += log_odds_update
                voxel_updates[voxel_key]['count'] += 1

        # DEBUG: Print voxel summary (NOTE: voxel_updates accumulates across ALL rays in this frame)
        if self.frame_count == 0 and abs(bearing_angle) < 0.5:
            print(f"  Total voxels accumulated so far (all rays): {len(voxel_updates)}")

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
            self.range_resolution = (self.max_range - self.min_range) / range_bins

        # Get robot transform
        T_base_to_world = self.pose_msg_to_transform(robot_pose)
        T_sonar_to_world = T_base_to_world @ self.T_sonar_to_base

        # Initialize voxel update accumulator for this frame
        # Dictionary to accumulate updates: key -> (sum_updates, count)
        voxel_updates = {}  # Will store accumulated updates per voxel

        # Process subset of bearings for efficiency
        bearing_divisor = 128  # Reduced from 256 to decrease point count
        bearing_step = max(1, bearing_bins // bearing_divisor)


        for b_idx in range(0, bearing_bins, bearing_step):
            bearing_angle = self.bearing_angles[b_idx]
            intensity_profile = polar_image[:, b_idx]

            # Process this ray and accumulate updates
            self.process_sonar_ray(bearing_angle, intensity_profile, T_sonar_to_world, voxel_updates)

        # Apply averaged updates to all voxels
        for voxel_key, update_info in voxel_updates.items():
            # Calculate average update
            avg_update = update_info['sum'] / update_info['count']

            # Apply update (adaptive only if positive log-odds = occupied)
            adaptive = (avg_update > 0)
            self.octree.update_voxel(update_info['point'], avg_update, adaptive=adaptive)

            # Debug: Log some statistics (commented out - too verbose)
            # if np.random.random() < 0.001:  # Sample 0.1% for better debugging
            #     print(f"Voxel {voxel_key}: {update_info['count']} rays, avg_update={avg_update:.4f}, type={update_info['type']}, log_odds_occupied={self.octree.log_odds_occupied}")


        # Increment frame counter
        self.frame_count += 1

        # Optional: Clear old voxels if frame limit reached
        if self.max_frames > 0 and self.frame_count > self.max_frames:
            # Simple approach: clear and restart
            # More sophisticated: implement sliding window or decay
            self.octree.clear()
            self.frame_count = 0

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

            # Convert gtsam.Pose2/Pose3 to simple pose dict for processing
            # Use x, y, yaw from Pose2, z from Pose3, assume roll=0, pitch=0
            pose_dict = {
                'position': {'x': kf.pose.x(),
                            'y': kf.pose.y(),
                            'z': kf.pose3.z()},  # Use actual depth from Pose3
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
