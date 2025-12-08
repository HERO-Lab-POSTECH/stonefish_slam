#pragma once

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <unordered_map>
#include <array>

namespace py = pybind11;

/**
 * @brief Update method for voxel probability updates
 *
 * - LOG_ODDS: Standard log-odds Bayesian update (default)
 * - WEIGHTED_AVERAGE: Weighted average based on intensity (voxelmap_fusion style)
 * - IWLO: Intensity-Weighted Log-Odds with adaptive learning rate
 */
enum class UpdateMethod {
    LOG_ODDS = 0,          // Standard log-odds update (default)
    WEIGHTED_AVERAGE = 1,  // Weighted average based on intensity
    IWLO = 2               // Intensity-Weighted Log-Odds
};

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
     * @brief Batch insert point cloud with intensity and pre-computed log-odds
     *
     * Primary API for IWLO/WEIGHTED_AVG update methods. ray_processor computes
     * the log-odds (ΔL) including range/gaussian weights, octree applies alpha decay.
     *
     * @param points Nx3 NumPy array of world coordinates [x, y, z]
     * @param intensities N-length array of intensity values (for WEIGHTED_AVG mode)
     * @param log_odds N-length array of pre-computed log-odds (ΔL from ray_processor)
     * @param sensor_origin 3-length array [x, y, z] of sensor position
     *
     * Update behavior:
     * - IWLO: L_new = clamp(L_old + ΔL × α(n), L_min, L_max)
     * - WEIGHTED_AVERAGE: P_new = (n*P_old + w(I))/(n+1)
     */
    void insert_point_cloud_with_intensity_and_logodds(
        py::array_t<double> points,
        py::array_t<double> intensities,
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
     * @brief Set adaptive update parameters (unidirectional protection)
     *
     * Protects free space voxels from being easily converted to occupied.
     * Only applies protection for Free → Occupied updates, not Occupied → Free.
     *
     * @param enable Enable adaptive protection
     * @param threshold Probability threshold for protection (default: 0.5)
     * @param max_ratio Maximum update ratio for protected voxels (default: 0.3)
     */
    void set_adaptive_params(bool enable, double threshold, double max_ratio);

    /**
     * @brief Set update method (LOG_ODDS, WEIGHTED_AVERAGE, IWLO)
     *
     * @param method Update method enum (0=LOG_ODDS, 1=WEIGHTED_AVG, 2=IWLO)
     */
    void set_update_method(int method);

    /**
     * @brief Set intensity parameters for WEIGHTED_AVERAGE and IWLO
     *
     * @param threshold Intensity threshold (e.g., 35.0)
     * @param max_val Maximum intensity value (e.g., 255.0)
     */
    void set_intensity_params(double threshold, double max_val);

    /**
     * @brief Set IWLO-specific parameters
     *
     * @param sharpness Sigmoid sharpness (default: 3.0)
     * @param decay_rate Learning rate decay (default: 0.1)
     * @param min_alpha Minimum learning rate (default: 0.1)
     * @param L_min Log-odds saturation lower bound (default: -2.0)
     * @param L_max Log-odds saturation upper bound (default: 3.5)
     */
    void set_iwlo_params(double sharpness, double decay_rate, double min_alpha,
                         double L_min, double L_max);

    /**
     * @brief Get map statistics (P3.2 profiling)
     *
     * @return MapStats with node counts and memory usage
     */
    MapStats get_map_stats() const;

    /**
     * @brief Serialize OcTree to binary data for octomap_msgs
     * @return Binary data as Python bytes object
     */
    py::bytes serialize_to_binary();

    /**
     * @brief C++ native batch insert (zero NumPy overhead)
     *
     * Internal-only method for use by C++ components (e.g., ray_processor).
     * Bypasses NumPy array creation and Python binding overhead.
     *
     * @param points Vector of voxel centers in world coordinates
     * @param log_odds Vector of log-odds updates per point
     * @param sensor_origin Sensor position [x, y, z]
     *
     * Note: This method is NOT exposed to Python. It is identical in logic
     *       to insert_point_cloud() but uses pure C++ containers.
     */
    void insert_voxels_batch_native(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<double>& log_odds,
        const Eigen::Vector3d& sensor_origin
    );

private:
    std::unique_ptr<octomap::OcTree> tree_;  // OctoMap tree instance
    double resolution_;                       // Voxel size in meters
    double log_odds_occupied_;                // Default log-odds for occupied
    double log_odds_free_;                    // Default log-odds for free space

    // Adaptive protection parameters (unidirectional: Free → Occupied only)
    bool adaptive_update_;                    // Enable adaptive protection
    double adaptive_threshold_;               // Probability threshold for protection
    double adaptive_max_ratio_;               // Max update ratio for protected voxels

    // Update method and intensity parameters
    UpdateMethod update_method_;              // Current update method
    double intensity_threshold_;              // Intensity threshold (e.g., 35.0)
    double intensity_max_;                    // Maximum intensity value (255.0)

    // IWLO parameters
    double sharpness_;                        // Sigmoid sharpness (3.0)
    double decay_rate_;                       // Learning rate decay (0.1)
    double min_alpha_;                        // Minimum learning rate (0.1)
    double L_min_;                            // Saturation lower bound (-2.0)
    double L_max_;                            // Saturation upper bound (3.5)

    // Observation count tracking for IWLO and WEIGHTED_AVERAGE
    std::unordered_map<uint64_t, int> observation_counts_;

    // Intensity weight LUT for performance optimization
    std::array<double, 256> intensity_weight_lut_;
    bool lut_initialized_ = false;

    // Helper functions
    uint64_t key_to_hash(const octomap::OcTreeKey& key) const;
    double intensity_to_weight(double intensity) const;
    double compute_alpha(int obs_count) const;
    void initialize_lut();
};
