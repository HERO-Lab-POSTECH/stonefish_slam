"""
Hierarchical Octree implementation for 3D occupancy mapping.

Note: This is a legacy Python octree. The main system now uses
C++ OctoMap backend (see cpp/octree_mapping.cpp).
"""
import numpy as np
from typing import Optional, Tuple, List, Dict


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

        # Adaptive update logic (unidirectional protection: Free → Occupied only)
        # Rationale: Strongly protect against false positives (free→occupied)
        #            but allow occupied→free updates for genuine free space
        if adaptive:
            current_log_odds = self.query_voxel(point)
            current_prob = 1.0 / (1.0 + np.exp(-current_log_odds))

            # Only protect Free → Occupied direction (prevent false positives)
            if log_odds_update > 0:
                # Occupied update: protect free voxels
                if current_prob <= self.adaptive_threshold:
                    update_scale = (current_prob / self.adaptive_threshold) * self.adaptive_max_ratio
                    log_odds_update *= update_scale
            # No protection for Occupied → Free (allow genuine free space updates)

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
