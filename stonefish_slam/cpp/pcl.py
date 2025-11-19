"""
Simple Python implementation of PCL functions for SLAM
Replaces the C++ pybind11 module when libpointmatcher is not available
"""

import numpy as np
from scipy.spatial import KDTree
from sklearn.neighbors import NearestNeighbors


def remove_outlier(points, radius, min_points):
    """
    Remove outlier points based on radius search

    Args:
        points: (N, 2) or (N, 3) numpy array
        radius: search radius
        min_points: minimum number of neighbors required

    Returns:
        filtered points array
    """
    if len(points) == 0:
        return points

    # Build KDTree for efficient neighbor search
    tree = KDTree(points)

    # Count neighbors within radius for each point
    inliers = []
    for i, point in enumerate(points):
        neighbors = tree.query_ball_point(point, radius)
        if len(neighbors) >= min_points:
            inliers.append(i)

    return points[inliers]


def density_filter(points, descriptors=None, knn=10, min_density=0.0, max_density=float('inf')):
    """
    Filter points based on local density

    Args:
        points: (N, 2) or (N, 3) numpy array
        descriptors: optional (N, D) descriptors
        knn: number of neighbors for density estimation
        min_density: minimum density threshold
        max_density: maximum density threshold

    Returns:
        filtered points (and descriptors if provided)
    """
    if len(points) == 0:
        if descriptors is not None:
            return points, descriptors
        return points

    # Estimate density using knn
    nbrs = NearestNeighbors(n_neighbors=min(knn, len(points)), algorithm='kd_tree')
    nbrs.fit(points)
    distances, indices = nbrs.kneighbors(points)

    # Density = 1 / average distance to knn neighbors
    avg_distances = np.mean(distances, axis=1)
    densities = 1.0 / (avg_distances + 1e-10)

    # Filter by density
    mask = (densities >= min_density) & (densities <= max_density)

    if descriptors is not None:
        return points[mask], descriptors[mask]
    return points[mask]


def downsample(points, descriptors_or_resolution=None, resolution=None):
    """
    Downsample point cloud using voxel grid

    Args:
        points: (N, 2) or (N, 3) numpy array
        descriptors_or_resolution: either descriptors (N, D) array or resolution float
        resolution: voxel size (only needed if descriptors are provided)

    Returns:
        downsampled points (and descriptors if provided)
    """
    # Parse arguments to handle both call signatures:
    # downsample(points, resolution) or downsample(points, descriptors, resolution)
    if descriptors_or_resolution is None:
        descriptors = None
        voxel_resolution = 0.5  # default
    elif isinstance(descriptors_or_resolution, (int, float)):
        # Called as downsample(points, resolution)
        descriptors = None
        voxel_resolution = float(descriptors_or_resolution)
    else:
        # Called as downsample(points, descriptors, resolution)
        descriptors = descriptors_or_resolution
        voxel_resolution = resolution if resolution is not None else 0.5

    if len(points) == 0:
        if descriptors is not None:
            return points, descriptors
        return points

    # Compute voxel indices for each point
    voxel_indices = np.floor(points / voxel_resolution).astype(np.int32)

    # Create unique voxel keys
    if points.shape[1] == 2:
        voxel_keys = voxel_indices[:, 0] * 1000000 + voxel_indices[:, 1]
    else:  # 3D
        voxel_keys = (voxel_indices[:, 0] * 1000000 +
                      voxel_indices[:, 1] * 1000 +
                      voxel_indices[:, 2])

    # Get unique voxels
    unique_keys, inverse_indices = np.unique(voxel_keys, return_inverse=True)

    # Average points in each voxel
    downsampled_points = []
    downsampled_descriptors = [] if descriptors is not None else None

    for i in range(len(unique_keys)):
        mask = inverse_indices == i
        # Take centroid of points in this voxel
        downsampled_points.append(np.mean(points[mask], axis=0))

        if descriptors is not None:
            # Take mean of descriptors
            downsampled_descriptors.append(np.mean(descriptors[mask], axis=0))

    downsampled_points = np.array(downsampled_points, dtype=np.float32)

    if descriptors is not None:
        downsampled_descriptors = np.array(downsampled_descriptors, dtype=descriptors.dtype)
        return downsampled_points, downsampled_descriptors

    return downsampled_points


def match(reference_points, query_points, knn=1, max_dist=float('inf')):
    """
    Find nearest neighbors from reference to query points

    Args:
        reference_points: (N, D) reference point cloud
        query_points: (M, D) query point cloud
        knn: number of nearest neighbors
        max_dist: maximum matching distance

    Returns:
        indices: (M, knn) indices in reference cloud (-1 if no match)
        distances: (M, knn) distances to matches
    """
    if len(reference_points) == 0 or len(query_points) == 0:
        indices = -np.ones((len(query_points), knn), dtype=np.int32)
        distances = np.full((len(query_points), knn), float('inf'), dtype=np.float32)
        return indices, distances

    # Build KDTree for reference points
    tree = KDTree(reference_points)

    # Query knn neighbors
    distances, indices = tree.query(query_points, k=knn, distance_upper_bound=max_dist)

    # Handle the case when distance_upper_bound is exceeded
    # KDTree returns len(reference_points) as index for points beyond max_dist
    invalid_mask = indices >= len(reference_points)
    indices = indices.astype(np.int32)
    indices[invalid_mask] = -1
    distances[invalid_mask] = float('inf')

    # Ensure output shape consistency
    if knn == 1:
        indices = indices.reshape(-1, 1)
        distances = distances.reshape(-1, 1)

    return indices, distances.astype(np.float32)


class ICP:
    """
    Simple ICP implementation for 2D point clouds
    """

    def __init__(self):
        self.max_iterations = 40
        self.tolerance = 0.01
        self.max_correspondence_distance = 3.0
        self.outlier_ratio = 0.8

    def loadFromYaml(self, config_file):
        """
        Load ICP configuration from YAML file
        For now, we use default parameters
        """
        # TODO: Parse YAML config if needed
        print(f"[ICP] Using default configuration (YAML parsing not implemented)")

    def compute(self, source_points, target_points, initial_guess):
        """
        Compute ICP alignment

        Args:
            source_points: (N, 2) source point cloud
            target_points: (M, 2) target point cloud
            initial_guess: (3, 3) initial transformation matrix

        Returns:
            ("success" or error message, transformation matrix)
        """
        if len(source_points) == 0 or len(target_points) == 0:
            return "Empty point cloud", initial_guess

        # Apply initial transformation
        T = initial_guess.copy()

        prev_error = float('inf')

        for iteration in range(self.max_iterations):
            # Transform source points
            ones = np.ones((len(source_points), 1))
            source_homogeneous = np.hstack([source_points, ones])
            transformed_source = (T @ source_homogeneous.T).T[:, :2]

            # Find correspondences
            indices, distances = match(target_points, transformed_source,
                                      knn=1, max_dist=self.max_correspondence_distance)

            # Filter outliers
            valid_mask = (indices[:, 0] != -1)
            if np.sum(valid_mask) < 3:
                return "Not enough correspondences", T

            # Keep best matches (trim outliers)
            num_keep = int(np.sum(valid_mask) * self.outlier_ratio)
            if num_keep < 3:
                return "Not enough inliers", T

            valid_distances = distances[valid_mask, 0]
            sorted_indices = np.argsort(valid_distances)[:num_keep]

            source_matched = transformed_source[valid_mask][sorted_indices]
            target_matched = target_points[indices[valid_mask, 0]][sorted_indices]

            # Compute transformation using SVD
            source_center = np.mean(source_matched, axis=0)
            target_center = np.mean(target_matched, axis=0)

            source_centered = source_matched - source_center
            target_centered = target_matched - target_center

            H = source_centered.T @ target_centered
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T

            # Handle reflection case
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T

            t = target_center - R @ source_center

            # Build transformation matrix
            T_delta = np.eye(3, dtype=np.float32)
            T_delta[:2, :2] = R
            T_delta[:2, 2] = t

            # Update total transformation
            T = T_delta @ T

            # Check convergence
            error = np.mean(valid_distances[sorted_indices])

            if abs(prev_error - error) < self.tolerance:
                return "success", T

            prev_error = error

        return "success", T

    def getCovariance(self):
        """
        Get covariance estimate (simplified - returns identity)
        """
        return np.eye(3, dtype=np.float32) * 0.1
