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
import time  # For performance profiling


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
            # Bearing propagation parameters (NEW)
            'enable_propagation': True,    # Enable bearing propagation (optimized for performance)
            'propagation_radius': 1,       # Number of adjacent bearings to propagate to (reduced from 2)
            'propagation_sigma': 1.0,      # Gaussian decay sigma for propagation weight (reduced from 1.5)
            'enable_profiling': True,      # Enable performance profiling
            # Gaussian weighting for vertical aperture (NEW)
            'enable_gaussian_weighting': False,  # Use Gaussian vs Uniform intensity distribution
            'gaussian_sigma_factor': 2.5,        # Sigma = aperture / factor (smaller = wider spread)
            # Range weighting parameters (distance-dependent decay)
            'use_range_weighting': True,         # Enable range-dependent weight decay
            'max_effective_range': 15.0,         # Maximum effective range in meters
            'lambda_decay': 0.3,                 # Exponential decay rate
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

        # Bearing propagation settings
        self.enable_propagation = default_config.get('enable_propagation', False)
        self.propagation_radius = default_config.get('propagation_radius', 2)
        self.propagation_sigma = default_config.get('propagation_sigma', 1.5)

        # Performance profiling
        self.enable_profiling = default_config.get('enable_profiling', True)
        self.performance_stats = {
            'frame_times': [],           # Processing times per frame
            'voxel_updates': [],          # Number of voxel updates per frame
            'ray_times': [],              # Processing times per ray
            'total_frames': 0,
            'total_voxels_updated': 0,
            'propagation_times': []       # Times for propagation operations
        }

        # Gaussian weighting settings for vertical aperture
        self.enable_gaussian_weighting = default_config.get('enable_gaussian_weighting', False)
        self.gaussian_sigma_factor = default_config.get('gaussian_sigma_factor', 2.5)

        # Range weighting parameters (distance-dependent decay)
        self.use_range_weighting = default_config.get('use_range_weighting', True)
        self.max_effective_range = default_config.get('max_effective_range', 15.0)
        self.lambda_decay = default_config.get('lambda_decay', 0.3)

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

    def compute_range_weight(self, range_m, max_range=None, lambda_decay=None):
        """
        Compute distance-dependent weight using exponential decay.

        Closer measurements are more reliable, so they receive higher weight.

        Args:
            range_m: Measured range in meters
            max_range: Maximum effective range (use self.max_effective_range if None)
            lambda_decay: Decay rate (use self.lambda_decay if None)

        Returns:
            weight: [0.0, 1.0], exponential decay with hard cutoff
        """
        if max_range is None:
            max_range = self.max_effective_range
        if lambda_decay is None:
            lambda_decay = self.lambda_decay

        if range_m > max_range:
            return 0.0  # Hard cutoff beyond effective range

        return np.exp(-lambda_decay * range_m / max_range)

    def propagate_bearing_updates_optimized(self, voxel_updates, sampled_bearing_idx, bearing_bins, T_sonar_to_world):
        """
        Optimized bearing propagation with minimal computation

        This method implements spatial coherence: adjacent bearings often observe
        similar surfaces, so we can propagate updates with decreasing weight.
        Optimized version with reduced computation.

        Args:
            voxel_updates: Dictionary of voxel updates from sampled bearing
            sampled_bearing_idx: Index of the sampled bearing (0, 2, 4, ...)
            bearing_bins: Total number of bearings (512)
            T_sonar_to_world: 4x4 transform matrix from sonar to world

        Returns:
            Dictionary of propagated voxel updates
        """
        if not self.enable_propagation:
            return {}

        propagated_updates = {}

        # Get sonar origin in world frame once
        sonar_origin_world = T_sonar_to_world[:3, 3]
        sampled_bearing_angle = self.bearing_angles[sampled_bearing_idx]

        # Calculate propagation range (now only ±1 bearing)
        for offset in range(-self.propagation_radius, self.propagation_radius + 1):
            if offset == 0:
                continue  # Skip original bearing (already processed)

            adjacent_bearing_idx = sampled_bearing_idx + offset

            # Boundary check
            if adjacent_bearing_idx < 0 or adjacent_bearing_idx >= bearing_bins:
                continue

            # Calculate Gaussian weight (faster with radius=1)
            if abs(offset) == 1:
                weight = np.exp(-0.5 / (self.propagation_sigma**2))  # Pre-computable for radius=1
            else:
                weight = np.exp(-0.5 * (offset / self.propagation_sigma)**2)

            # Get bearing angle difference
            adjacent_bearing_angle = self.bearing_angles[adjacent_bearing_idx]
            angle_diff = adjacent_bearing_angle - sampled_bearing_angle

            # Pre-compute rotation values
            cos_diff = np.cos(angle_diff)
            sin_diff = np.sin(angle_diff)

            # For each voxel update in the original bearing
            for voxel_key, update_info in voxel_updates.items():
                # Get original point in world frame
                px, py, pz = update_info['point']

                # Transform to sonar-relative coordinates
                rel_x = px - sonar_origin_world[0]
                rel_y = py - sonar_origin_world[1]
                # Note: pz stays the same (no vertical rotation)

                # Apply 2D rotation (Z-axis rotation only, much faster)
                new_rel_x = rel_x * cos_diff - rel_y * sin_diff
                new_rel_y = rel_x * sin_diff + rel_y * cos_diff

                # Transform back to world coordinates
                new_px = new_rel_x + sonar_origin_world[0]
                new_py = new_rel_y + sonar_origin_world[1]
                # pz unchanged (vertical aperture same for all bearings)

                # Create new voxel key for adjusted position
                adjacent_voxel_key = self.octree.world_to_key(new_px, new_py, pz)

                # Apply weighted update
                weighted_sum = update_info['sum'] * weight

                if adjacent_voxel_key not in propagated_updates:
                    propagated_updates[adjacent_voxel_key] = {
                        'point': np.array([new_px, new_py, pz]),
                        'sum': 0.0,
                        'count': 0
                    }

                propagated_updates[adjacent_voxel_key]['sum'] += weighted_sum
                propagated_updates[adjacent_voxel_key]['count'] += update_info['count']

        return propagated_updates

    def propagate_bearing_updates(self, voxel_updates, sampled_bearing_idx, bearing_bins, T_sonar_to_world):
        """
        Wrapper that calls optimized version
        Kept for backward compatibility
        """
        return self.propagate_bearing_updates_optimized(
            voxel_updates, sampled_bearing_idx, bearing_bins, T_sonar_to_world
        )

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

        # DEBUG: Print ray processing info for first frame, first few bearings (DISABLED)
        if False:  # Disabled for performance
            max_intensity = np.max(intensity_profile)
            print(f"\n=== Ray bearing={np.degrees(bearing_angle):.1f}°, max_intensity={max_intensity} ===")
            print(f"  Threshold: {self.intensity_threshold}")
            print(f"  First hit index: {first_hit_idx}")
            print(f"  High intensity indices: {high_intensity_indices[:5] if len(high_intensity_indices) > 0 else 'None'}")

        # If no hit found, skip this ray entirely (no information)
        if first_hit_idx == -1:
            # No reflection: don't update as free space (we don't know if it's free or just out of range)
            if False:  # Disabled for performance
                print(f"  → SKIPPED: No reflection detected (no map update)")
            return

        # Calculate vertical aperture parameters
        half_aperture = self.vertical_aperture / 2

        # DEBUG: Print free space processing (DISABLED)
        if False:  # Disabled for performance
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

            # Base log-odds for free space before weighting
            base_log_odds_free = self.octree.log_odds_free

            # Apply range weighting if enabled
            if self.use_range_weighting:
                range_weight = self.compute_range_weight(range_m)
                base_log_odds_free = base_log_odds_free * range_weight

            for v_step in range(-num_vertical_steps, num_vertical_steps + 1):
                # Calculate vertical angle within aperture
                vertical_angle = (v_step / max(1, num_vertical_steps)) * half_aperture

                # Apply Gaussian weighting if enabled
                if self.enable_gaussian_weighting:
                    # Normalized angle: -1 to +1
                    normalized_angle = vertical_angle / half_aperture if half_aperture > 0 else 0
                    # Gaussian: exp(-0.5 * (x/σ)^2), σ = 1/gaussian_sigma_factor
                    gaussian_weight = np.exp(-0.5 * (normalized_angle * self.gaussian_sigma_factor)**2)
                    log_odds_update = base_log_odds_free * gaussian_weight
                else:
                    # Uniform weighting (default)
                    log_odds_update = base_log_odds_free

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
                voxel_updates[voxel_key]['sum'] += log_odds_update  # Use weighted value
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

            # DEBUG: Print vertical sampling for first ray (DISABLED)
            if False:  # Disabled for performance
                print(f"    r_idx={r_idx}, range_m={range_m:.2f}m, vertical_spread={vertical_spread:.3f}m, num_vertical_steps={num_vertical_steps}")

            # Base log-odds for occupied space before weighting
            base_log_odds_occupied = self.octree.log_odds_occupied

            # Apply range weighting if enabled
            if self.use_range_weighting:
                range_weight = self.compute_range_weight(range_m)
                base_log_odds_occupied = base_log_odds_occupied * range_weight

            for v_step in range(-num_vertical_steps, num_vertical_steps + 1):
                # Calculate vertical angle within aperture
                vertical_angle = (v_step / max(1, num_vertical_steps)) * half_aperture

                # Apply Gaussian weighting if enabled
                if self.enable_gaussian_weighting:
                    # Normalized angle: -1 to +1
                    normalized_angle = vertical_angle / half_aperture if half_aperture > 0 else 0
                    # Gaussian: exp(-0.5 * (x/σ)^2), σ = 1/gaussian_sigma_factor
                    gaussian_weight = np.exp(-0.5 * (normalized_angle * self.gaussian_sigma_factor)**2)
                    log_odds_update = base_log_odds_occupied * gaussian_weight
                else:
                    # Uniform weighting (default)
                    log_odds_update = base_log_odds_occupied

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

        # DEBUG: Print voxel summary (NOTE: voxel_updates accumulates across ALL rays in this frame) (DISABLED)
        if False:  # Disabled for performance
            print(f"  Total voxels accumulated so far (all rays): {len(voxel_updates)}")

    def process_sonar_image(self, polar_image, robot_pose):
        """
        Process sonar image and update probabilistic map

        Args:
            polar_image: 2D numpy array (height x width) with intensity values
            robot_pose: Robot pose (dict with 'position'/'orientation' or ROS Pose message)
        """
        # Start timing if profiling enabled
        if self.enable_profiling:
            frame_start_time = time.time()

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
        all_voxel_updates = {}  # Combined updates (original + propagated)

        # Process subset of bearings for efficiency
        # With propagation radius=1, we need denser sampling to avoid gaps
        bearing_divisor = 256  # Increased from 128 for better coverage with propagation
        bearing_step = max(1, bearing_bins // bearing_divisor)

        # Track processed bearings for propagation
        processed_bearings = []

        for b_idx in range(0, bearing_bins, bearing_step):
            if self.enable_profiling:
                ray_start = time.time()

            bearing_angle = self.bearing_angles[b_idx]
            intensity_profile = polar_image[:, b_idx]

            # Clear voxel_updates for this bearing
            voxel_updates.clear()

            # Process this ray and accumulate updates
            self.process_sonar_ray(bearing_angle, intensity_profile, T_sonar_to_world, voxel_updates)

            # Add to all updates
            for voxel_key, update_info in voxel_updates.items():
                if voxel_key not in all_voxel_updates:
                    all_voxel_updates[voxel_key] = {'point': update_info['point'], 'sum': 0.0, 'count': 0}
                all_voxel_updates[voxel_key]['sum'] += update_info['sum']
                all_voxel_updates[voxel_key]['count'] += update_info['count']

            # Propagate to adjacent bearings if enabled
            if self.enable_propagation and len(voxel_updates) > 0:
                if self.enable_profiling:
                    prop_start = time.time()

                propagated = self.propagate_bearing_updates(
                    voxel_updates, b_idx, bearing_bins, T_sonar_to_world
                )

                # Merge propagated updates
                for voxel_key, update_info in propagated.items():
                    if voxel_key not in all_voxel_updates:
                        all_voxel_updates[voxel_key] = {'point': update_info['point'], 'sum': 0.0, 'count': 0}
                    all_voxel_updates[voxel_key]['sum'] += update_info['sum']
                    all_voxel_updates[voxel_key]['count'] += update_info['count']

                if self.enable_profiling:
                    prop_time = time.time() - prop_start
                    self.performance_stats['propagation_times'].append(prop_time)

            processed_bearings.append(b_idx)

            if self.enable_profiling:
                ray_time = time.time() - ray_start
                self.performance_stats['ray_times'].append(ray_time)

        # Apply averaged updates to all voxels
        num_voxels_updated = 0
        for voxel_key, update_info in all_voxel_updates.items():
            # Calculate average update
            avg_update = update_info['sum'] / update_info['count']

            # Apply update (adaptive only if positive log-odds = occupied)
            adaptive = (avg_update > 0)
            self.octree.update_voxel(update_info['point'], avg_update, adaptive=adaptive)
            num_voxels_updated += 1

        # Increment frame counter
        self.frame_count += 1

        # Performance profiling statistics
        if self.enable_profiling:
            frame_time = time.time() - frame_start_time
            self.performance_stats['frame_times'].append(frame_time)
            self.performance_stats['voxel_updates'].append(num_voxels_updated)
            self.performance_stats['total_frames'] += 1
            self.performance_stats['total_voxels_updated'] += num_voxels_updated

            # Print statistics every 10 frames
            if self.frame_count % 10 == 0:
                avg_frame_time = np.mean(self.performance_stats['frame_times'][-10:])
                avg_voxels = np.mean(self.performance_stats['voxel_updates'][-10:])
                total_voxels = len(self.octree.voxels)

                print(f"\n[3D Mapping Performance - Frame {self.frame_count}]")
                print(f"  Avg frame time (last 10): {avg_frame_time:.3f}s")
                print(f"  Avg voxels updated/frame: {avg_voxels:.0f}")
                print(f"  Total voxels in map: {total_voxels}")
                print(f"  Bearings processed: {len(processed_bearings)} (step={bearing_step})")

                if self.enable_propagation:
                    print(f"  Propagation: ENABLED (radius={self.propagation_radius}, sigma={self.propagation_sigma:.1f})")
                    if len(self.performance_stats['propagation_times']) > 0:
                        avg_prop_time = np.mean(self.performance_stats['propagation_times'][-10:])
                        prop_percent = (avg_prop_time / avg_frame_time) * 100
                        print(f"    Propagation time: {avg_prop_time:.4f}s ({prop_percent:.1f}% of frame time)")
                else:
                    print(f"  Propagation: DISABLED")

                # Memory usage estimate (rough)
                memory_mb = (total_voxels * 100) / (1024 * 1024)  # ~100 bytes per voxel estimate
                print(f"  Estimated memory usage: {memory_mb:.1f} MB")

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
        # Reset performance stats
        if self.enable_profiling:
            self.performance_stats = {
                'frame_times': [],
                'voxel_updates': [],
                'ray_times': [],
                'total_frames': 0,
                'total_voxels_updated': 0,
                'propagation_times': []
            }

    def get_performance_summary(self):
        """
        Get performance statistics summary

        Returns:
            Dictionary with performance metrics
        """
        if not self.enable_profiling or len(self.performance_stats['frame_times']) == 0:
            return {'profiling_enabled': False}

        stats = {
            'profiling_enabled': True,
            'total_frames': self.performance_stats['total_frames'],
            'total_voxels_updated': self.performance_stats['total_voxels_updated'],
            'avg_frame_time': np.mean(self.performance_stats['frame_times']) if len(self.performance_stats['frame_times']) > 0 else 0,
            'avg_voxels_per_frame': np.mean(self.performance_stats['voxel_updates']) if len(self.performance_stats['voxel_updates']) > 0 else 0,
            'avg_ray_time': np.mean(self.performance_stats['ray_times']) if len(self.performance_stats['ray_times']) > 0 else 0,
            'total_rays_processed': len(self.performance_stats['ray_times']),
        }

        if self.enable_propagation and len(self.performance_stats['propagation_times']) > 0:
            stats['propagation_enabled'] = True
            stats['avg_propagation_time'] = np.mean(self.performance_stats['propagation_times'])
            stats['propagation_radius'] = self.propagation_radius
            stats['propagation_sigma'] = self.propagation_sigma
        else:
            stats['propagation_enabled'] = False

        # Add percentiles for frame times
        if len(self.performance_stats['frame_times']) > 0:
            stats['frame_time_p50'] = np.percentile(self.performance_stats['frame_times'], 50)
            stats['frame_time_p90'] = np.percentile(self.performance_stats['frame_times'], 90)
            stats['frame_time_p99'] = np.percentile(self.performance_stats['frame_times'], 99)

        return stats
