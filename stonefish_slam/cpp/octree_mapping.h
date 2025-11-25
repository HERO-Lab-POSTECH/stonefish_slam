#pragma once

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace py = pybind11;

/**
 * @brief Map statistics for profiling P3.2 (map size)
 */
struct MapStats {
    size_t num_nodes;       // Total number of nodes
    size_t num_leaf_nodes;  // Number of leaf nodes (actual voxels)
    double memory_mb;       // Memory usage in MB
};

/**
 * @brief High-performance Octree mapping using OctoMap library
 *
 * This class provides a Python-accessible C++ wrapper around OctoMap's OcTree
 * for efficient voxel-based occupancy mapping. Designed to replace Python-based
 * Octree implementations with significant performance improvements.
 *
 * Key features:
 * - Batch point cloud insertion (zero-copy with NumPy)
 * - Log-odds occupancy probability updates
 * - Automatic clamping to prevent overflow
 * - Fast occupied voxel queries
 *
 * Reference:
 * - Hornung et al. (2013) "OctoMap: An Efficient Probabilistic 3D Mapping
 *   Framework Based on Octrees", Autonomous Robots
 */
class OctreeMapping {
public:
    /**
     * @brief Construct Octree with specified resolution
     * @param resolution Voxel size in meters (default: 0.1m)
     */
    explicit OctreeMapping(double resolution = 0.1);

    /**
     * @brief Destructor - cleans up OcTree memory
     */
    ~OctreeMapping();

    /**
     * @brief Batch insert point cloud with log-odds updates
     *
     * Efficiently processes entire point clouds in a single call, avoiding
     * Python-C++ boundary overhead. Uses OctoMap's optimized update logic.
     *
     * @param points Nx3 NumPy array of world coordinates [x, y, z]
     * @param log_odds N-length array of log-odds updates per point
     * @param sensor_origin 3-length array [x, y, z] of sensor position
     *
     * Example:
     *   points = np.array([[1.0, 0.0, 0.5], [1.1, 0.1, 0.5]])
     *   log_odds = np.array([0.85, 0.85])  # occupied
     *   origin = np.array([0.0, 0.0, 0.0])
     *   tree.insert_point_cloud(points, log_odds, origin)
     */
    void insert_point_cloud(
        py::array_t<double> points,
        py::array_t<double> log_odds,
        py::array_t<double> sensor_origin
    );

    /**
     * @brief Get all occupied voxels above threshold
     *
     * @param threshold Occupancy probability threshold (0.0 to 1.0, default: 0.5)
     * @return Nx4 NumPy array of occupied voxel centers [x, y, z, log_odds]
     */
    py::array_t<double> get_occupied_cells(double threshold = 0.5);

    /**
     * @brief Query occupancy probability at specific location
     *
     * @param x World X coordinate
     * @param y World Y coordinate
     * @param z World Z coordinate
     * @return Occupancy probability (0.0 to 1.0), or 0.5 if unknown
     */
    double query_cell(double x, double y, double z);

    /**
     * @brief Clear all voxels from the tree
     */
    void clear();

    /**
     * @brief Get total number of nodes in the tree
     * @return Node count (includes both occupied and free nodes)
     */
    size_t get_num_nodes() const;

    /**
     * @brief Set log-odds thresholds for occupied/free classification
     *
     * @param occupied Log-odds value for occupied updates (e.g., 0.85)
     * @param free Log-odds value for free space updates (e.g., -0.4)
     */
    void set_log_odds_thresholds(double occupied, double free);

    /**
     * @brief Set probability clamping limits
     *
     * Prevents extreme probabilities that could cause numerical issues.
     *
     * @param min Minimum probability (e.g., 0.12)
     * @param max Maximum probability (e.g., 0.97)
     */
    void set_clamping_thresholds(double min, double max);

    /**
     * @brief Get map statistics (P3.2 profiling)
     *
     * @return MapStats with node counts and memory usage
     */
    MapStats get_map_stats() const;

private:
    std::unique_ptr<octomap::OcTree> tree_;  // OctoMap tree instance
    double resolution_;                       // Voxel size in meters
    double log_odds_occupied_;                // Default log-odds for occupied
    double log_odds_free_;                    // Default log-odds for free space
};
