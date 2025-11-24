#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <cmath>
#include <memory>

namespace py = pybind11;

// Forward declarations
class OctreeMapping;

/**
 * @brief Configuration for sonar ray processing
 *
 * Contains all parameters needed for processing sonar rays including
 * range parameters, log-odds updates, and weighting factors.
 */
struct RayProcessorConfig {
    // Range parameters
    double max_range;                   // Maximum sonar range (m)
    double min_range;                   // Minimum valid range (m)
    double range_resolution;            // Range bin resolution (m)

    // Angular parameters
    double vertical_aperture;           // Vertical beam aperture (radians)
    double horizontal_fov;              // Horizontal field of view (degrees)
    double bearing_resolution;          // Horizontal angular resolution (radians)

    // Vertical sampling parameters
    double free_vertical_factor;        // Vertical sampling sparsity for free space (default: 8.0)
    double occupied_vertical_factor;    // Vertical sampling density for occupied (default: 3.0)

    // Log-odds parameters
    double log_odds_occupied;           // Log-odds update for occupied voxels (e.g., 0.85)
    double log_odds_free;               // Log-odds update for free space (e.g., -0.4)

    // Weighting parameters
    bool use_range_weighting;           // Enable range-based weighting
    double lambda_decay;                // Exponential decay factor for range weighting
    bool enable_gaussian_weighting;     // Enable Gaussian weighting on vertical angle
    double gaussian_sigma_factor;       // Sigma factor for Gaussian weighting

    // Processing parameters
    double voxel_resolution;            // Voxel size (m)
    int bearing_step;                   // Bearing sampling step (e.g., 2 = every 2nd bearing)
    uint8_t intensity_threshold;        // Intensity threshold for detection (0-255)

    // Constructor with default values
    RayProcessorConfig()
        : max_range(30.0),
          min_range(0.5),
          range_resolution(0.06),
          vertical_aperture(20.0 * M_PI / 180.0),
          horizontal_fov(130.0),
          bearing_resolution(0.0175),
          free_vertical_factor(8.0),
          occupied_vertical_factor(3.0),
          log_odds_occupied(0.85),
          log_odds_free(-0.4),
          use_range_weighting(true),
          lambda_decay(2.0),
          enable_gaussian_weighting(false),
          gaussian_sigma_factor(3.0),
          voxel_resolution(0.2),
          bearing_step(2),
          intensity_threshold(30)
    {}
};

/**
 * @brief High-performance sonar ray processor for 3D mapping
 *
 * Processes sonar polar images and updates 3D occupancy map using:
 * - DDA traversal for free space (fast ray casting)
 * - Vertical fan processing for occupied space (accounting for beam spread)
 * - Range weighting and Gaussian vertical weighting
 * - Direct OctoMap updates (eliminates Python dictionary overhead)
 *
 * Performance target: 828.7ms (Python) → 100-150ms (C++) = 5-8× speedup
 *
 * Key optimizations:
 * - Single C++ call for entire sonar image (minimize Python ↔ C++ boundary crossing)
 * - Eigen batch transformations (SIMD vectorization)
 * - OpenMP parallelization across bearings
 * - Direct octree updates (no intermediate dictionary)
 *
 * References:
 * - mapping_3d.py lines 830-1017 (Python implementation)
 * - dda_traversal.cpp (free space processing)
 * - octree_mapping.cpp (OctoMap interface)
 */
class RayProcessor {
public:
    /**
     * @brief Constructor
     * @param octree Pointer to OctreeMapping instance (must remain valid)
     * @param config Processing configuration
     */
    RayProcessor(
        OctreeMapping* octree,
        const RayProcessorConfig& config
    );

    /**
     * @brief Process entire sonar polar image
     *
     * Main entry point from Python. Processes all bearings and updates octree.
     *
     * @param polar_image 2D NumPy array (num_range_bins × num_bearings), uint8
     * @param T_sonar_to_world 4×4 transformation matrix (sonar → world frame)
     *
     * Performance: Processes ~128 bearings × 512 range bins in 100-150ms
     *
     * Example usage from Python:
     *   processor = RayProcessor(octree, dda, config)
     *   processor.process_sonar_image(polar_img, T_sonar_world)
     */
    void process_sonar_image(
        py::array_t<uint8_t> polar_image,
        const Eigen::Matrix4d& T_sonar_to_world
    );

    /**
     * @brief Update configuration parameters
     * @param config New configuration
     */
    void set_config(const RayProcessorConfig& config);

    /**
     * @brief Get current configuration
     * @return Current configuration
     */
    RayProcessorConfig get_config() const { return config_; }

private:
    /**
     * @brief Structure to hold voxel updates before octree insertion
     *
     * Used internally to collect voxel data in OpenMP parallel regions
     * without calling Python API (GIL-free).
     */
    struct VoxelUpdate {
        double x, y, z;
        double log_odds;
    };

    /**
     * @brief Process single ray - internal version (GIL-free, OpenMP-safe)
     *
     * Collects voxel updates in C++ buffer instead of calling Python API.
     * Used by process_sonar_image() in OpenMP parallel region.
     *
     * @param bearing_idx Bearing index in polar image
     * @param num_bearings Total number of bearings (for angle calculation)
     * @param intensity_profile Intensity values along range (1D array)
     * @param T_sonar_to_world Transformation matrix
     * @param sonar_origin_world Sonar origin in world frame (cached)
     * @param voxel_updates Output buffer to collect voxel updates
     */
    void process_single_ray_internal(
        int bearing_idx,
        int num_bearings,
        const std::vector<uint8_t>& intensity_profile,
        const Eigen::Matrix4d& T_sonar_to_world,
        const Eigen::Vector3d& sonar_origin_world,
        std::vector<VoxelUpdate>& voxel_updates
    );

    /**
     * @brief Process single ray - legacy version for Python API
     *
     * Maintains backward compatibility with external Python calls.
     * Uses internal version and inserts to octree immediately.
     *
     * @param bearing_idx Bearing index in polar image
     * @param num_bearings Total number of bearings (for angle calculation)
     * @param intensity_profile Intensity values along range (1D array)
     * @param T_sonar_to_world Transformation matrix
     */
    void process_single_ray(
        int bearing_idx,
        int num_bearings,
        const std::vector<uint8_t>& intensity_profile,
        const Eigen::Matrix4d& T_sonar_to_world
    );

    /**
     * @brief Process occupied voxels - internal version (GIL-free, OpenMP-safe)
     *
     * Collects occupied voxel updates in C++ buffer.
     * Used by process_single_ray_internal() in OpenMP parallel region.
     *
     * @param hit_indices Indices of range bins with high intensity
     * @param bearing_angle Horizontal bearing angle (radians)
     * @param T_sonar_to_world Transformation matrix
     * @param sonar_origin_world Sonar origin in world frame (cached)
     * @param voxel_updates Output buffer to collect voxel updates
     */
    void process_occupied_voxels_internal(
        const std::vector<int>& hit_indices,
        double bearing_angle,
        const Eigen::Matrix4d& T_sonar_to_world,
        const Eigen::Vector3d& sonar_origin_world,
        std::vector<VoxelUpdate>& voxel_updates
    );

    /**
     * @brief Process occupied voxels - legacy version for Python API
     *
     * For each range bin with high intensity:
     * 1. Create vertical fan of points (±num_vertical_steps)
     * 2. Apply range weighting: exp(-λ × r / r_max)
     * 3. Apply Gaussian vertical weighting (optional)
     * 4. Transform to world frame (Eigen batch operation)
     * 5. Update octree directly
     *
     * This replaces Python lines 942-1017 (occupied processing loop)
     *
     * @param hit_indices Indices of range bins with high intensity
     * @param bearing_angle Horizontal bearing angle (radians)
     * @param T_sonar_to_world Transformation matrix
     */
    void process_occupied_voxels(
        const std::vector<int>& hit_indices,
        double bearing_angle,
        const Eigen::Matrix4d& T_sonar_to_world
    );

    /**
     * @brief Find first intensity peak above threshold
     * @param intensity_profile Intensity values (0-255)
     * @return Index of first hit, or -1 if none found
     */
    int find_first_hit(const std::vector<uint8_t>& intensity_profile) const;

    /**
     * @brief Find last intensity peak above threshold
     * @param intensity_profile Intensity values (0-255)
     * @return Index of last hit, or -1 if none found
     */
    int find_last_hit(const std::vector<uint8_t>& intensity_profile) const;

    /**
     * @brief Extract all indices with intensity above threshold
     * @param intensity_profile Intensity values (0-255)
     * @param start_idx Start search from this index
     * @param end_idx End search at this index
     * @return Vector of indices with high intensity
     */
    std::vector<int> extract_hit_indices(
        const std::vector<uint8_t>& intensity_profile,
        int start_idx,
        int end_idx
    ) const;

    /**
     * @brief Compute range-based weighting factor
     *
     * Exponential decay: w(r) = exp(-λ × r / r_max)
     * Python equivalent: mapping_3d.py lines 756-758
     *
     * @param range_m Range in meters
     * @return Weight factor (0.0 to 1.0)
     */
    double compute_range_weight(double range_m) const;

    /**
     * @brief Compute vertical angle for given step
     *
     * Linear interpolation within vertical aperture:
     * θ_v = (v_step / num_steps) × (aperture / 2)
     *
     * Python equivalent: mapping_3d.py lines 974-976
     *
     * @param v_step Vertical step index (e.g., -5 to +5)
     * @param num_vertical_steps Total number of steps (e.g., 5)
     * @return Vertical angle in radians
     */
    double compute_vertical_angle(int v_step, int num_vertical_steps) const;

    /**
     * @brief Compute bearing angle from bearing index
     * @param bearing_idx Bearing index in polar image
     * @return Bearing angle in radians (centered at 0)
     */
    double compute_bearing_angle(int bearing_idx, int num_bearings) const;

    /**
     * @brief Compute 3D ray direction from bearing angle
     *
     * Sonar frame convention: X=forward, Y=right, Z=down (FRD)
     * Bearing: 0=forward, positive=right, negative=left
     *
     * @param bearing_angle Horizontal angle (radians)
     * @return Unit direction vector in sonar frame
     */
    Eigen::Vector3d compute_ray_direction(double bearing_angle) const;

    /**
     * @brief Compute number of vertical steps based on range and vertical spread
     *
     * Adaptive sampling: denser near sonar, sparser far away
     * num_steps = max(min_steps, vertical_spread / (voxel_size × factor))
     *
     * @param range_m Range in meters
     * @param vertical_factor Sampling factor (free: 8.0, occupied: 3.0)
     * @return Number of vertical steps (±N)
     */
    int compute_num_vertical_steps(double range_m, double vertical_factor) const;

private:
    /**
     * @brief Internal DDA voxel traversal
     *
     * Simplified version of DDATraversal for internal use only.
     * Avoids inter-module symbol dependencies.
     */
    std::vector<Eigen::Vector3d> traverse_ray_dda(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        int max_voxels = 500
    ) const;

    /**
     * @brief Convert world coordinates to voxel key
     */
    std::array<int, 3> world_to_voxel_key(const Eigen::Vector3d& point) const;

private:
    // External dependencies (non-owning pointers)
    OctreeMapping* octree_;     // Octree for voxel updates

    // Configuration
    RayProcessorConfig config_;

    // Cached values for performance
    double half_aperture_;      // Cached vertical_aperture / 2
};
