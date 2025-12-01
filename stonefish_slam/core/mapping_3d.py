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

from stonefish_slam.utils.profiler import MappingProfiler
from stonefish_slam.core.octree import HierarchicalOctree, OctNode

# C++ Ray Processor import (with fallback)
try:
    from stonefish_slam.ray_processor import RayProcessor, RayProcessorConfig
    CPP_RAY_PROCESSOR_AVAILABLE = True
except ImportError as e:
    print(f"[WARNING] C++ RayProcessor not available: {e}")
    CPP_RAY_PROCESSOR_AVAILABLE = False


class SonarMapping3D:
    """
    Convert sonar images to 3D point clouds with probabilistic mapping
    Accumulates multiple frames and updates voxel probabilities

    Integrated with stonefish_slam SLAM system using keyframe-based updates.
    """

    def __init__(self, config):
        """
        Initialize with sonar parameters (config required)

        Args:
            config: Dictionary with all required parameters
        """

        # Store parameters
        self.horizontal_fov = np.radians(config['horizontal_fov'])
        self.vertical_fov = np.radians(config['vertical_fov'])
        self.range_max = config['range_max']
        self.range_min = config['range_min']
        self.intensity_threshold = config['intensity_threshold']
        self.num_beams = config['num_beams']
        self.num_bins = config['num_bins']
        self.voxel_resolution = config['voxel_resolution']
        self.min_probability = config['min_probability']
        self.max_frames = config['max_frames']
        self.dynamic_expansion = config['dynamic_expansion']

        # Sonar mounting transform (convert tilt from degrees to radians, negative for FRD)
        self.sonar_position = np.array(config['sonar_position'])
        self.sonar_tilt = -np.deg2rad(config['sonar_tilt_deg'])

        # Pre-compute sonar to base_link transform
        self.T_sonar_to_base = self.create_transform_matrix(
            self.sonar_position,
            self.sonar_tilt
        )

        # Pre-compute angles for efficiency
        self.bearing_angles = np.linspace(
            -self.horizontal_fov/2,
            self.horizontal_fov/2,
            self.num_beams
        )

        # Range resolution (bin size in meters)
        self.range_resolution = (self.range_max - self.range_min) / self.num_bins

        # Get adaptive update settings from config
        self.adaptive_update = config['adaptive_update']
        self.adaptive_threshold = config['adaptive_threshold']
        self.adaptive_max_ratio = config['adaptive_max_ratio']

        # Get log-odds parameters from config
        self.log_odds_occupied = config['log_odds_occupied']
        self.log_odds_free = config['log_odds_free']
        self.log_odds_min = config['log_odds_min']
        self.log_odds_max = config['log_odds_max']

        # C++ backend initialization
        self.use_cpp_backend = config['use_cpp_backend']

        if self.use_cpp_backend:
            try:
                from stonefish_slam import octree_mapping
                self.cpp_octree = octree_mapping.OctreeMapping(resolution=self.voxel_resolution)

                # Configure adaptive protection (unidirectional: Free → Occupied only)
                self.cpp_octree.set_adaptive_params(
                    enable=self.adaptive_update,
                    threshold=self.adaptive_threshold,
                    max_ratio=self.adaptive_max_ratio
                )

                # Configure clamping thresholds (prevent fast saturation)
                # Convert log-odds to probability: p = 1 / (1 + exp(-log_odds))
                prob_min = 1.0 / (1.0 + np.exp(-self.log_odds_min))
                prob_max = 1.0 / (1.0 + np.exp(-self.log_odds_max))
                self.cpp_octree.set_clamping_thresholds(prob_min, prob_max)

                print(f"[INFO] Using C++ OctoMap backend (resolution: {self.voxel_resolution}m, "
                      f"adaptive={self.adaptive_update}, threshold={self.adaptive_threshold}, "
                      f"max_ratio={self.adaptive_max_ratio}, clamp=[{self.log_odds_min}, {self.log_odds_max}])")
                # No Python octree needed
                self.octree = None
            except ImportError as e:
                print(f"[WARNING] C++ OctoMap not available, falling back to Python: {e}")
                self.use_cpp_backend = False
                self.cpp_octree = None
                # Fall through to Python octree initialization

        # C++ Ray Processor initialization (depends on C++ octree)
        # C++ ray processor: Fixed OpenMP/GIL issue with scoped release pattern (2025-11-24)
        self.use_cpp_ray_processor = config.get('use_cpp_ray_processor', True)
        self.cpp_ray_processor = None

        if self.use_cpp_ray_processor and CPP_RAY_PROCESSOR_AVAILABLE and self.use_cpp_backend:
            try:
                # Create RayProcessorConfig
                ray_config = RayProcessorConfig()
                ray_config.range_max = self.range_max
                ray_config.range_min = self.range_min
                ray_config.range_resolution = self.range_resolution
                ray_config.vertical_fov = self.vertical_fov
                # horizontal_fov: degrees for C++ angle calculation
                ray_config.horizontal_fov = np.degrees(self.horizontal_fov)
                # bearing_resolution: radians (proper interval calculation for np.linspace)
                ray_config.bearing_resolution = self.horizontal_fov / (self.num_beams - 1)
                ray_config.log_odds_occupied = self.log_odds_occupied
                ray_config.log_odds_free = self.log_odds_free
                ray_config.use_range_weighting = config['use_range_weighting']
                ray_config.lambda_decay = config['lambda_decay']
                ray_config.enable_gaussian_weighting = config['enable_gaussian_weighting']
                ray_config.voxel_resolution = self.voxel_resolution
                ray_config.bearing_step = config['bearing_step']
                ray_config.intensity_threshold = self.intensity_threshold

                # Store bearing_step for profiling
                self.cpp_bearing_step = ray_config.bearing_step

                # Create RayProcessor with shared octree
                self.cpp_ray_processor = RayProcessor(self.cpp_octree, ray_config)
                print(f"[INFO] C++ RayProcessor initialized (OpenMP enabled, bearing_step={ray_config.bearing_step})")
            except Exception as e:
                print(f"[WARNING] Failed to initialize C++ RayProcessor: {e}")
                self.use_cpp_ray_processor = False
                self.cpp_ray_processor = None
        else:
            if self.use_cpp_ray_processor:
                if not CPP_RAY_PROCESSOR_AVAILABLE:
                    print(f"[INFO] C++ RayProcessor disabled (module not available)")
                elif not self.use_cpp_backend:
                    print(f"[INFO] C++ RayProcessor disabled (requires C++ octree backend)")
            self.use_cpp_ray_processor = False

        # Initialize Python octree if C++ backend not used
        if not self.use_cpp_backend:
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
        self.enable_propagation = config['enable_propagation']
        self.propagation_radius = config.get('propagation_radius', 2)
        self.propagation_sigma = config.get('propagation_sigma', 1.5)

        # Performance profiling
        self.enable_profiling = config.get('enable_profiling', True)
        self.performance_stats = {
            'frame_times': [],           # Processing times per frame
            'voxel_updates': [],          # Number of voxel updates per frame
            'ray_times': [],              # Processing times per ray
            'total_frames': 0,
            'total_voxels_updated': 0,
            'propagation_times': []       # Times for propagation operations
        }

        # Detailed profiling infrastructure (7 measurement points)
        self.profiling_enabled = True  # Enable detailed profiling
        self.profiling_data = {
            'frame_total': [],
            'ray_processing': [],
            'dda_traversal': [],
            'dict_merge': [],
            'occupied_processing': [],
            'bearing_propagation': [],
            'octree_updates': [],
            'voxels_per_frame': [],
            'rays_per_frame': []
        }

        # CSV profiling infrastructure (P3.1/P3.2)
        self.csv_sample_interval = config.get('frame_interval', 10)  # Use frame_interval
        self.csv_path = '/tmp/mapping_profiling.csv'
        self.profiler = MappingProfiler(
            csv_path=self.csv_path,
            sample_interval=self.csv_sample_interval
        )
        if self.enable_profiling:
            self.profiler.start()

        # Gaussian weighting settings for vertical aperture
        self.enable_gaussian_weighting = config['enable_gaussian_weighting']
        self.gaussian_sigma_factor = config.get('gaussian_sigma_factor', 2.5)

        # Range weighting parameters (distance-dependent decay)
        self.use_range_weighting = config['use_range_weighting']
        self.lambda_decay = config['lambda_decay']

        # DDA voxel traversal initialization (after all parameters are set)
        self.use_dda = config['use_dda_traversal']
        if self.use_dda:
            try:
                from stonefish_slam import dda_traversal
                self.dda_traverser = dda_traversal.DDATraversal(self.voxel_resolution)

                # Create config object for batch processing
                self.dda_config = dda_traversal.SonarRayConfig()
                self.dda_config.voxel_size = self.voxel_resolution
                self.dda_config.log_odds_free = self.log_odds_free
                self.dda_config.range_max = self.range_max
                self.dda_config.range_min = self.range_min
                self.dda_config.vertical_fov = self.vertical_fov
                self.dda_config.use_range_weighting = self.use_range_weighting
                self.dda_config.lambda_decay = self.lambda_decay
                self.dda_config.enable_gaussian_weighting = self.enable_gaussian_weighting
                self.dda_config.gaussian_sigma_factor = self.gaussian_sigma_factor

                print(f"[INFO] Using C++ DDA batch processing (voxel_size={self.voxel_resolution}m)")
            except ImportError as e:
                self.use_dda = False
                print(f"[WARN] C++ DDA module not found ({e}), using Python traversal")

    def get_voxel_count(self):
        """
        Get total number of voxels in the map (backend-agnostic).

        Returns:
            int: Total number of voxels
        """
        if self.use_cpp_backend:
            return self.cpp_octree.get_num_nodes()
        else:
            return len(self.octree.voxels) if self.octree else 0

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

    def compute_range_weight(self, range_m, lambda_decay=None):
        """
        Compute distance-dependent weight using exponential decay.

        Uses sonar's range_max as the reference for decay calculation.
        No hard cutoff - weight decays smoothly from 1.0 (near) to ~0.9 (range_max).

        Args:
            range_m: Measured range in meters
            lambda_decay: Decay rate (use self.lambda_decay if None)

        Returns:
            weight: (0.0, 1.0], exponential decay w(r) = exp(-λ * r / range_max)

        Literature:
            - Fairfield 2007: σ(r) = σ₀ + k·r
            - Pinto 2015: SNR decreases with range
        """
        if lambda_decay is None:
            lambda_decay = self.lambda_decay

        # Exponential decay: w(r) = exp(-λ * r / r_max)
        # At r=0: w=1.0
        # At r=range_max: w=exp(-λ) (e.g., λ=0.1 → w≈0.90)
        return np.exp(-lambda_decay * range_m / self.range_max)

    def _compute_first_hit_map(self, polar_image):
        """
        Compute first hit range for each bearing

        Args:
            polar_image: 2D numpy array (height x width) with intensity values

        Returns:
            np.ndarray: shape (n_bearings,), first hit range in meters for each bearing
                       Returns range_max if no hit found
        """
        range_bins, bearing_bins = polar_image.shape
        first_hit_map = np.full(bearing_bins, self.range_max, dtype=np.float32)

        for b_idx in range(bearing_bins):
            intensity_profile = polar_image[:, b_idx]

            # Find first pixel above threshold
            for r_idx, intensity in enumerate(intensity_profile):
                if intensity > self.intensity_threshold:
                    # Calculate actual range (FLS image: row 0 = far, row max = near)
                    first_hit_map[b_idx] = self.range_max - r_idx * self.range_resolution
                    break

        return first_hit_map

    def _voxel_to_sonar_coords(self, voxel_world, T_world_to_sonar):
        """
        Transform voxel world coordinates to sonar frame coordinates

        Args:
            voxel_world: np.ndarray [x, y, z] in world frame
            T_world_to_sonar: 4x4 inverse transform matrix (world → sonar)

        Returns:
            tuple: (bearing_rad, range_m, elevation_rad)
        """
        # Transform to sonar frame
        voxel_world_homo = np.array([voxel_world[0], voxel_world[1], voxel_world[2], 1.0])
        voxel_sonar = T_world_to_sonar @ voxel_world_homo

        x_s, y_s, z_s = voxel_sonar[:3]

        # Convert to spherical coordinates
        # Use HORIZONTAL range only (matching first_hit_map calculation)
        range_m = np.sqrt(x_s**2 + y_s**2)  # Horizontal range (ignore Z)

        # Bearing: horizontal angle (atan2(y, x))
        bearing_rad = np.arctan2(y_s, x_s)

        # Elevation: vertical angle (asin(z / range))
        elevation_rad = np.arcsin(z_s / range_m) if range_m > 1e-6 else 0.0

        return bearing_rad, range_m, elevation_rad

    def _is_voxel_in_shadow(self, voxel_world, T_world_to_sonar, first_hit_map, exclude_bearing_rad=None):
        """
        Check if voxel is in ANY bearing's shadow cone

        Redesigned to match C++ logic: iterate all bearings and check if voxel falls
        within each bearing's angular cone and is beyond its first hit.

        Args:
            voxel_world: np.ndarray [x, y, z] in world frame
            T_world_to_sonar: 4x4 inverse transform (world → sonar)
            first_hit_map: np.ndarray (n_bearings,) first hit ranges
            exclude_bearing_rad: Bearing angle to exclude from shadow check (for occupied voxels)
                                If None, check all bearings (for free space)

        Returns:
            bool: True if voxel is in shadow (should skip update)
        """
        # 1. Convert voxel to sonar coordinates
        bearing_rad, range_m, elevation_rad = self._voxel_to_sonar_coords(voxel_world, T_world_to_sonar)

        # 2. Calculate angular width for each bearing's cone
        bearing_bins = len(first_hit_map)
        actual_bearing_resolution = self.horizontal_fov / (bearing_bins - 1)  # radians
        # Use vertical aperture for shadow cone width (matches beam physical footprint)
        bearing_half_width = self.vertical_fov / 2.0  # Half of vertical aperture in radians

        # 3. Calculate exclude bearing index if specified (for occupied voxels)
        exclude_idx = -1
        if exclude_bearing_rad is not None:
            exclude_normalized = (exclude_bearing_rad + self.horizontal_fov / 2) / self.horizontal_fov
            exclude_idx = int(exclude_normalized * bearing_bins)

        # 4. Check EACH bearing's shadow cone
        for b_idx in range(bearing_bins):
            # Skip excluded bearing (for occupied voxels)
            if b_idx == exclude_idx:
                continue

            # Calculate this bearing's angle
            bearing_angle = -self.horizontal_fov / 2 + b_idx * actual_bearing_resolution

            # Check if voxel is beyond this bearing's first hit
            first_hit_range = first_hit_map[b_idx]
            epsilon = 0.01  # 1cm tolerance
            if range_m < first_hit_range + epsilon:
                continue  # Before first hit, not shadow

            # Check if voxel is in this bearing's angular cone
            angle_diff = abs(bearing_rad - bearing_angle)

            # Handle wraparound (angle difference should be within [-π, π])
            if angle_diff > np.pi:
                angle_diff = 2 * np.pi - angle_diff

            if angle_diff <= bearing_half_width:
                return True  # In shadow

        return False  # Not in any bearing's shadow

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
                if self.use_cpp_backend:
                    adjacent_voxel_key = (
                        int(np.floor(new_px / self.voxel_resolution)),
                        int(np.floor(new_py / self.voxel_resolution)),
                        int(np.floor(pz / self.voxel_resolution))
                    )
                else:
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

    def process_sonar_ray(self, bearing_angle, intensity_profile, T_sonar_to_world, voxel_updates, timing_accumulators=None, T_world_to_sonar=None, first_hit_map=None):
        """
        Process a single sonar ray (bearing) and accumulate voxel updates

        Args:
            bearing_angle: Horizontal angle in radians
            intensity_profile: 1D array of intensities along range
            T_sonar_to_world: 4x4 transform matrix from sonar to world
            voxel_updates: Dictionary to accumulate updates per voxel
            timing_accumulators: Optional dict with 'dda', 'merge', 'occupied' keys for profiling
            T_world_to_sonar: Optional 4x4 inverse transform (for shadow validation)
            first_hit_map: Optional np.ndarray (n_bearings,) for shadow validation
        """
        # Find first hit and all high intensity regions after first hit
        first_hit_idx = -1
        high_intensity_indices = []

        for r_idx, intensity in enumerate(intensity_profile):
            if intensity > self.intensity_threshold:
                if first_hit_idx == -1:
                    first_hit_idx = r_idx  # Mark first hit
                high_intensity_indices.append(r_idx)  # Collect all high intensity after first
            # Low intensity after first hit: shadow region, don't update

        # If no hit found, treat entire range as free space
        if first_hit_idx == -1:
            # No reflection within range_max → entire measured range is free space
            first_hit_idx = len(intensity_profile)  # Treat entire range as free

        # Calculate vertical aperture parameters
        half_aperture = self.vertical_fov / 2

        # Update free space before first hit (with sparse sampling)
        # Use C++ DDA batch processing if available
        if self.use_dda and first_hit_idx > 0:
            # Calculate range to first hit
            range_to_first_hit = self.range_max - (first_hit_idx - 1) * self.range_resolution

            # Horizontal ray direction at bearing_angle
            ray_direction_horizontal = np.array([
                np.cos(bearing_angle),
                np.sin(bearing_angle),
                0.0
            ])

            # Transform to world frame
            sonar_origin_world = T_sonar_to_world[:3, 3]
            ray_direction_world = T_sonar_to_world[:3, :3] @ ray_direction_horizontal
            ray_direction_world = ray_direction_world / np.linalg.norm(ray_direction_world)

            # Compute vertical steps
            mid_range = self.range_max - (first_hit_idx / 2) * self.range_resolution
            vertical_spread = mid_range * np.tan(half_aperture)
            free_vertical_factor = 8.0
            num_vertical_steps = max(1, int(vertical_spread / (self.voxel_resolution * free_vertical_factor)))

            # SINGLE C++ call for entire vertical fan
            try:
                # [C] DDA traversal timing
                if timing_accumulators is not None:
                    t_dda = time.perf_counter()

                voxel_updates_list = self.dda_traverser.process_free_space_ray(
                    sonar_origin_world,
                    ray_direction_world,
                    range_to_first_hit,
                    num_vertical_steps,
                    self.dda_config
                )

                if timing_accumulators is not None:
                    timing_accumulators['dda'] += (time.perf_counter() - t_dda)

                # [D] Dictionary merge timing
                if timing_accumulators is not None:
                    t_merge = time.perf_counter()

                # Merge results into voxel_updates dict
                for update in voxel_updates_list:
                    key = tuple(update.key)
                    voxel_center = np.array(update.key, dtype=float) * self.voxel_resolution

                    # Shadow validation (same as Python fallback path)
                    if T_world_to_sonar is not None and first_hit_map is not None:
                        if self._is_voxel_in_shadow(voxel_center, T_world_to_sonar, first_hit_map):
                            continue  # Skip voxel in shadow region

                    if key not in voxel_updates:
                        voxel_updates[key] = {'point': voxel_center, 'sum': 0.0, 'count': 0}
                    voxel_updates[key]['sum'] += update.log_odds_sum
                    voxel_updates[key]['count'] += update.count

                if timing_accumulators is not None:
                    timing_accumulators['merge'] += (time.perf_counter() - t_merge)

            except Exception as e:
                print(f"[ERROR] DDA batch processing failed: {e}")
                # Fall through to Python fallback below
        else:
            # Fallback to original Python traversal
            free_sampling_step = 2  # Reduced from 10 to 2 for better coverage (0.12m intervals)
            for r_idx in range(0, first_hit_idx, free_sampling_step):
                # Calculate actual range (FLS image: row 0 = far, row max = near)
                range_m = self.range_max - r_idx * self.range_resolution

                # Calculate vertical spread at this range
                vertical_spread = range_m * np.tan(half_aperture)
                # Sparse vertical sampling for free space
                free_vertical_factor = 8.0  # Increased from 4.0 to reduce point count
                num_vertical_steps = max(1, int(vertical_spread / (self.voxel_resolution * free_vertical_factor)))

                # Base log-odds for free space before weighting
                base_log_odds_free = self.log_odds_free

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

                    # Shadow validation: check if voxel is in another bearing's shadow
                    if T_world_to_sonar is not None and first_hit_map is not None:
                        if self._is_voxel_in_shadow(pt_world[:3], T_world_to_sonar, first_hit_map):
                            continue  # Skip voxel in shadow region

                    # Get voxel key and accumulate update
                    if self.use_cpp_backend:
                        # Use resolution-based quantization for C++ backend
                        voxel_key = (
                            int(np.floor(pt_world[0] / self.voxel_resolution)),
                            int(np.floor(pt_world[1] / self.voxel_resolution)),
                            int(np.floor(pt_world[2] / self.voxel_resolution))
                        )
                    else:
                        voxel_key = self.octree.world_to_key(pt_world[0], pt_world[1], pt_world[2])

                    # Accumulate log-odds updates (no type distinction)
                    if voxel_key not in voxel_updates:
                        voxel_updates[voxel_key] = {'point': pt_world[:3], 'sum': 0.0, 'count': 0}
                    voxel_updates[voxel_key]['sum'] += log_odds_update  # Use weighted value
                    voxel_updates[voxel_key]['count'] += 1

        # [E] Occupied voxel processing timing
        if timing_accumulators is not None:
            t_occupied = time.perf_counter()

        # Update occupied regions: all high intensity after first hit
        # Process all high intensity regions after first reflection
        # Low intensity regions after first hit are shadow/unknown (not updated)
        for r_idx in high_intensity_indices:
            # Calculate actual range (FLS image: row 0 = far, row max = near)
            range_m = self.range_max - r_idx * self.range_resolution

            # Skip out-of-range (should not happen with correct calculation)
            if range_m > self.range_max:
                continue

            # Calculate vertical spread at this range
            vertical_spread = range_m * np.tan(half_aperture)
            # Denser vertical sampling for occupied space
            occupied_vertical_factor = 3.0  # Increased from 1.5 to reduce point count
            num_vertical_steps = max(2, int(vertical_spread / (self.voxel_resolution * occupied_vertical_factor)))

            # Base log-odds for occupied space before weighting
            base_log_odds_occupied = self.log_odds_occupied

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

                # Shadow validation: exclude current bearing (occupied voxels should not be blocked by their own bearing)
                if T_world_to_sonar is not None and first_hit_map is not None:
                    if self._is_voxel_in_shadow(pt_world[:3], T_world_to_sonar, first_hit_map,
                                               exclude_bearing_rad=bearing_angle):
                        continue  # Skip voxel in other bearings' shadow

                # Get voxel key and accumulate update
                if self.use_cpp_backend:
                    # Use resolution-based quantization for C++ backend
                    voxel_key = (
                        int(np.floor(pt_world[0] / self.voxel_resolution)),
                        int(np.floor(pt_world[1] / self.voxel_resolution)),
                        int(np.floor(pt_world[2] / self.voxel_resolution))
                    )
                else:
                    voxel_key = self.octree.world_to_key(pt_world[0], pt_world[1], pt_world[2])

                # Accumulate log-odds updates (no type distinction - just add)
                if voxel_key not in voxel_updates:
                    voxel_updates[voxel_key] = {'point': pt_world[:3], 'sum': 0.0, 'count': 0}
                voxel_updates[voxel_key]['sum'] += log_odds_update
                voxel_updates[voxel_key]['count'] += 1

        if timing_accumulators is not None:
            timing_accumulators['occupied'] += (time.perf_counter() - t_occupied)

    def process_sonar_image(self, polar_image, robot_pose):
        """
        Process sonar image and update probabilistic map

        Args:
            polar_image: 2D numpy array (height x width) with intensity values
            robot_pose: Robot pose (dict with 'position'/'orientation' or ROS Pose message)
        """
        # [A] Frame start timing
        if self.profiling_enabled:
            t_frame_start = time.perf_counter()

        # Start timing if profiling enabled (legacy)
        if self.enable_profiling:
            frame_start_time = time.time()

        # Ensure image is numpy array
        if not isinstance(polar_image, np.ndarray):
            polar_image = np.array(polar_image)

        # Get image dimensions
        range_bins, bearing_bins = polar_image.shape

        # Check if dimensions match expected
        if bearing_bins != self.num_beams:
            # Resize bearing angles if needed
            self.bearing_angles = np.linspace(
                -self.horizontal_fov/2,
                self.horizontal_fov/2,
                bearing_bins
            )

        if range_bins != self.num_bins:
            # Update range resolution if needed
            self.range_resolution = (self.range_max - self.range_min) / range_bins

        # Get robot transform
        T_base_to_world = self.pose_msg_to_transform(robot_pose)
        T_sonar_to_world = T_base_to_world @ self.T_sonar_to_base

        # Compute inverse transform for shadow validation (cache for efficiency)
        T_world_to_sonar = np.linalg.inv(T_sonar_to_world)

        # Compute first-hit map for shadow validation
        first_hit_map = self._compute_first_hit_map(polar_image)

        # Initialize voxel update accumulator for this frame
        # Dictionary to accumulate updates: key -> (sum_updates, count)
        voxel_updates = {}  # Will store accumulated updates per voxel
        all_voxel_updates = {}  # Combined updates (original + propagated)

        # Process subset of bearings for efficiency
        # Reduced divisor since propagation is disabled
        bearing_divisor = 128  # Reduced from 256 (50% fewer bearings processed)
        bearing_step = max(1, bearing_bins // bearing_divisor)

        # Track processed bearings for propagation
        processed_bearings = []

        # [B] Ray processing timing
        if self.profiling_enabled:
            t_ray_start = time.perf_counter()
            timing_accumulators = {'dda': 0.0, 'merge': 0.0, 'occupied': 0.0}
        else:
            timing_accumulators = {'dda': 0.0, 'merge': 0.0, 'occupied': 0.0}  # Always initialize for profiling compatibility

        # C++ vs Python ray processing decision
        use_cpp_path = self.use_cpp_ray_processor and self.cpp_ray_processor is not None

        if use_cpp_path:
            # C++ Ray Processing Path
            try:
                if self.profiling_enabled:
                    t_cpp_start = time.perf_counter()

                # Process entire sonar image with C++ (direct octree update)
                self.cpp_ray_processor.process_sonar_image(polar_image, T_sonar_to_world)

                if self.profiling_enabled:
                    t_cpp_total = time.perf_counter() - t_cpp_start
                    # Store in ray_processing slot for compatibility
                    t_ray_total = t_cpp_total

                # C++ updates octree directly, no Python merge needed
                # Calculate processed rays from C++ bearing_step (not Python's bearing_step!)
                cpp_step = self.cpp_bearing_step if hasattr(self, 'cpp_bearing_step') else 2
                num_rays_processed = bearing_bins // cpp_step
                for b_idx in range(0, bearing_bins, cpp_step):
                    processed_bearings.append(b_idx)

                # Estimate voxel count (C++ doesn't report exact count yet)
                # Typical: ~100-500 voxels per ray depending on scene
                num_voxels_updated = num_rays_processed * 200  # Rough estimate

            except Exception as e:
                print(f"[ERROR] C++ RayProcessor failed: {e}")
                print(f"[INFO] Falling back to Python implementation for this frame")
                # Disable C++ for this frame and fall through to Python
                use_cpp_path = False

        if not use_cpp_path:
            # Python Ray Processing Path (existing code)
            for b_idx in range(0, bearing_bins, bearing_step):
                if self.enable_profiling:
                    ray_start = time.time()

                bearing_angle = self.bearing_angles[b_idx]
                intensity_profile = polar_image[:, b_idx]

                # Clear voxel_updates for this bearing
                voxel_updates.clear()

                # Process this ray and accumulate updates (with shadow validation)
                self.process_sonar_ray(bearing_angle, intensity_profile, T_sonar_to_world, voxel_updates, timing_accumulators, T_world_to_sonar, first_hit_map)

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

                    propagated = self.propagate_bearing_updates_optimized(
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

            # [B] Ray processing timing end
            if self.profiling_enabled:
                t_ray_total = time.perf_counter() - t_ray_start

        # Apply averaged updates to all voxels (only if Python path was used)
        if not use_cpp_path:
            # [G] Octree updates timing
            if self.profiling_enabled:
                t_octree_start = time.perf_counter()

            num_voxels_updated = 0

            if self.use_cpp_backend:
                # C++ batch update path
                if len(all_voxel_updates) > 0:
                    # Prepare batch arrays
                    points_list = []
                    log_odds_list = []

                    for voxel_key, update_info in all_voxel_updates.items():
                        # Calculate average update
                        avg_update = update_info['sum'] / update_info['count']
                        points_list.append(update_info['point'])
                        log_odds_list.append(avg_update)

                    # Convert to numpy arrays
                    points = np.array(points_list, dtype=np.float64)
                    log_odds = np.array(log_odds_list, dtype=np.float64)

                    # Get sensor origin from transform
                    sensor_origin = T_sonar_to_world[:3, 3]

                    # Single batch insert call
                    self.cpp_octree.insert_point_cloud(points, log_odds, sensor_origin)
                    num_voxels_updated = len(points)
            else:
                # Python update path (existing)
                for voxel_key, update_info in all_voxel_updates.items():
                    # Calculate average update
                    avg_update = update_info['sum'] / update_info['count']

                    # Apply update (adaptive protection for both occupied AND free)
                    # CRITICAL FIX: Free space should also be protected from rapid changes
                    adaptive = True  # Always use adaptive protection
                    self.octree.update_voxel(update_info['point'], avg_update, adaptive=adaptive)
                    num_voxels_updated += 1

            if self.profiling_enabled:
                t_octree_total = time.perf_counter() - t_octree_start
        else:
            # C++ path already updated octree, skip Python merge
            if self.profiling_enabled:
                t_octree_total = 0.0  # No additional octree update needed

        # [A] Frame total timing end
        if self.profiling_enabled:
            t_frame_total = time.perf_counter() - t_frame_start

            # Store profiling data
            self.profiling_data['frame_total'].append(t_frame_total)
            self.profiling_data['ray_processing'].append(t_ray_total)
            self.profiling_data['dda_traversal'].append(timing_accumulators['dda'])
            self.profiling_data['dict_merge'].append(timing_accumulators['merge'])
            self.profiling_data['occupied_processing'].append(timing_accumulators['occupied'])
            self.profiling_data['bearing_propagation'].append(0.0)  # Not used in C++ path
            self.profiling_data['octree_updates'].append(t_octree_total)
            self.profiling_data['voxels_per_frame'].append(num_voxels_updated)
            self.profiling_data['rays_per_frame'].append(len(processed_bearings))

        # Increment frame counter
        self.frame_count += 1

        # Print detailed profiling statistics every csv_sample_interval frames
        if self.profiling_enabled and self.frame_count % self.csv_sample_interval == 0:
            if len(self.profiling_data['frame_total']) >= self.csv_sample_interval:
                self._print_profiling_stats()
                # Keep only last csv_sample_interval frames
                for key in self.profiling_data:
                    self.profiling_data[key] = self.profiling_data[key][-self.csv_sample_interval:]

        # Performance profiling statistics (legacy - keep for compatibility)
        if self.enable_profiling:
            frame_time = time.time() - frame_start_time
            self.performance_stats['frame_times'].append(frame_time)
            self.performance_stats['voxel_updates'].append(num_voxels_updated)
            self.performance_stats['total_frames'] += 1
            self.performance_stats['total_voxels_updated'] += num_voxels_updated

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
        if self.use_cpp_backend:
            # C++ backend path
            # Convert probability threshold to log-odds
            # C++ backend expects probability threshold (0-1), not log-odds
            threshold = np.clip(self.min_probability, 0.0, 1.0)

            # Get occupied cells from C++ backend (now returns Nx4: x, y, z, log_odds)
            occupied_cells = self.cpp_octree.get_occupied_cells(threshold=threshold)

            if len(occupied_cells) > 0:
                # Validate shape
                if occupied_cells.shape[1] != 4:
                    raise ValueError(f"Expected Nx4 array from C++ backend, got {occupied_cells.shape}")

                points = occupied_cells[:, :3]  # x, y, z
                log_odds_values = occupied_cells[:, 3]  # log-odds

                # Convert log-odds to probability: p = 1 / (1 + exp(-log_odds))
                probs = 1.0 / (1.0 + np.exp(-log_odds_values))
            else:
                points = np.empty((0, 3))
                probs = np.empty(0)

            result = {
                'points': points,
                'probabilities': probs,
                'num_points': len(points),
                'num_occupied': len(points),
                'frame_count': self.frame_count
            }

            # Note: include_free not supported in C++ backend yet
            if include_free:
                print("[WARNING] include_free=True not supported with C++ backend")

            return result

        else:
            # Python backend path (existing)
            if include_free:
                # Get all classified voxels with min_probability filter
                classified = self.octree.get_all_voxels_classified(self.min_probability)

                result = {
                    'occupied': classified['occupied'],
                    'free': classified['free'],
                    'unknown': classified['unknown'],
                    'num_voxels': self.get_voxel_count(),
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
                    'num_voxels': self.get_voxel_count(),
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
        if self.use_cpp_backend:
            self.cpp_octree.clear()
        else:
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

    def _print_profiling_stats(self):
        """Print concise profiling statistics and save to CSV (called every 10 frames)"""
        import numpy as np

        # Calculate averages (convert to milliseconds)
        avg_frame = np.mean(self.profiling_data['frame_total']) * 1000  # ms
        avg_ray = np.mean(self.profiling_data['ray_processing']) * 1000
        avg_octree = np.mean(self.profiling_data['octree_updates']) * 1000
        avg_voxels = np.mean(self.profiling_data['voxels_per_frame'])

        fps = 1000.0 / avg_frame if avg_frame > 0 else 0

        # Concise console output (1 line)
        print(f"Frame {self.frame_count}: {avg_frame:.1f}ms ({fps:.1f} FPS) | {int(avg_voxels)} voxels")

        # CSV sampling using MappingProfiler
        if self.enable_profiling:
            # Get C++ stats (P3.1/P3.2)
            exp_calls = 0
            exp_ms = 0.0
            map_voxels = 0
            memory_mb = 0.0

            if self.use_cpp_ray_processor and self.cpp_ray_processor is not None:
                try:
                    ray_stats = self.cpp_ray_processor.get_ray_stats()
                    exp_calls = ray_stats.exp_calls
                    exp_ms = ray_stats.exp_time_ms
                except Exception:
                    pass

            if self.use_cpp_backend and hasattr(self.cpp_octree, 'get_map_stats'):
                try:
                    map_stats = self.cpp_octree.get_map_stats()
                    map_voxels = map_stats.num_leaf_nodes
                    memory_mb = map_stats.memory_mb
                except Exception:
                    pass

            # Dedup count (from Python octree or estimate)
            dedup_count = avg_voxels if hasattr(self, 'octree') else 0

            # Record frame metrics with profiler
            self.profiler.record_frame(
                frame_id=self.frame_count,
                timestamp=time.time(),
                total_ms=avg_frame,
                ray_ms=avg_ray,
                octree_ms=avg_octree,
                exp_calls=exp_calls,
                exp_ms=exp_ms,
                map_voxels=map_voxels,
                memory_mb=memory_mb,
                dedup_count=int(dedup_count)
            )

            # Reset C++ stats for next sampling window
            if self.use_cpp_ray_processor and self.cpp_ray_processor is not None:
                try:
                    self.cpp_ray_processor.reset_ray_stats()
                except Exception:
                    pass

    def __del__(self):
        """Cleanup resources on destruction"""
        if hasattr(self, 'profiler'):
            self.profiler.close()
