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
#include <chrono>
#include <atomic>

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
    double range_max;                   // Maximum sonar range (m)
    double range_min;                   // Minimum valid range (m)
    double range_resolution;            // Range bin resolution (m)

    // Angular parameters
    double vertical_fov;           // Vertical beam aperture (radians)
    double horizontal_fov;              // Horizontal field of view (degrees)
    double bearing_resolution;          // Horizontal angular resolution (radians)

    // Log-odds parameters
    double log_odds_occupied;           // Log-odds update for occupied voxels (default: 0.5)
    double log_odds_free;               // Log-odds update for free space (default: -5.0)

    // Weighting parameters
    bool use_range_weighting;           // Enable range-based weighting
    double lambda_decay;                // Exponential decay factor for range weighting
    bool enable_gaussian_weighting;     // Enable Gaussian weighting on vertical angle
    double gaussian_sigma_factor;       // Sigma factor for Gaussian weighting

    // Processing parameters
    double voxel_resolution;            // Voxel size (m)
    int bearing_step;                   // Bearing sampling step (e.g., 2 = every 2nd bearing)
    uint8_t intensity_threshold;        // Intensity threshold for detection (0-255)

    // Update method parameters
    int update_method;                  // 0=LOG_ODDS, 1=WEIGHTED_AVG, 2=IWLO
    double sharpness;                   // IWLO sigmoid sharpness
    double decay_rate;                  // IWLO decay rate
    double min_alpha;                   // IWLO minimum alpha
    double L_min;                       // IWLO saturation lower
    double L_max;                       // IWLO saturation upper
    double intensity_max;               // Maximum intensity value

    // Constructor with default values
    RayProcessorConfig()
        : range_max(40.0),
          range_min(0.5),
          range_resolution(0.06),
          vertical_fov(20.0 * M_PI / 180.0),
          horizontal_fov(130.0),
          bearing_resolution(0.0175),
          log_odds_occupied(0.5),
          log_odds_free(-5.0),  // Strong free space clearing (balanced with occupied)
          use_range_weighting(true),
          lambda_decay(2.0),
          enable_gaussian_weighting(false),
          gaussian_sigma_factor(3.0),
          voxel_resolution(0.2),
          bearing_step(2),
          intensity_threshold(30),
          update_method(0),
          sharpness(3.0),
          decay_rate(0.1),
          min_alpha(0.1),
          L_min(-2.0),
          L_max(3.5),
          intensity_max(255.0)
    {}
};

/**
 * @brief Ray processing statistics for profiling P3.1 (exp() calls)
 */
struct RayStats {
    size_t exp_calls;      // Number of exp() calls
    double exp_time_ms;    // Total time spent in exp() (milliseconds)
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
     * @param polar_image 2D NumPy array (num_range_bins × num_beams), uint8
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

    /**
     * @brief Get ray processing statistics (profiling P3.1)
     * @return RayStats with exp() call count and timing
     */
    RayStats get_ray_stats() const;

    /**
     * @brief Reset ray processing statistics
     *
     * Called after CSV sampling to reset counters for next measurement window.
     */
    void reset_ray_stats();

    /**
     * @brief Compute first hit range map from polar image
     *
     * For each bearing, find the first range bin with intensity above threshold
     * and compute its horizontal range in meters.
     *
     * Python equivalent: mapping_3d.py lines 372-396 (_compute_first_hit_map)
     *
     * @param polar_image 2D NumPy array (num_range_bins × num_beams), uint8
     * @return Vector of first hit ranges (one per bearing, in meters)
     */
    std::vector<double> compute_first_hit_map(
        const py::array_t<uint8_t>& polar_image
    ) const;

    /**
     * @brief Generate hit map visualization image
     *
     * Creates RGB visualization image with same logic as actual voxel update
     * (process_single_ray_internal). Each pixel is colored based on:
     * - Black (0, 0, 0): Invalid range
     * - Red (255, 0, 0): First hit
     * - Yellow (255, 255, 0): Occupied (hit but not first)
     * - Green (0, 255, 0): Free space
     * - Blue (0, 0, 255): Shadow region
     *
     * @param polar_image 2D NumPy array (num_range_bins × num_beams), uint8
     * @return RGB image (num_range_bins × num_beams × 3), uint8
     */
    py::array_t<uint8_t> generate_hit_map_visualization(
        const py::array_t<uint8_t>& polar_image
    ) const;

    /**
     * @brief Check if voxel is in shadow region (simplified: global minimum first_hit)
     *
     * Algorithm:
     * 1. Transform voxel to sonar frame
     * 2. Calculate horizontal range (ignore Z)
     * 3. Compare with global minimum first_hit across all bearings
     * 4. If voxel_range >= global_min_first_hit, it's in shadow
     *
     * This simplified approach uses a single global minimum instead of per-bearing checks,
     * reducing computational complexity from O(num_beams) to O(1) per voxel.
     *
     * Example:
     *   Global min first_hit = 5m (closest obstacle across all bearings)
     *   Voxel at horizontal range 6m → 6m >= 5m → Shadow ✓
     *   Voxel at horizontal range 4m → 4m < 5m → Not shadow ✓
     *
     * @param voxel_world Voxel center in world frame
     * @param T_world_to_sonar Inverse transformation (world → sonar)
     * @param global_min_first_hit Global minimum first hit range across all bearings (meters)
     * @return True if voxel is in shadow (should skip update)
     */
    bool is_voxel_in_shadow(
        const Eigen::Vector3d& voxel_world,
        const Eigen::Matrix4d& T_world_to_sonar,
        double global_min_first_hit
    ) const;

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
        double intensity;  // Intensity value for IWLO/Weighted Average
    };

    /**
     * @brief Process single ray - internal version (GIL-free, OpenMP-safe)
     *
     * Collects voxel updates in C++ buffer instead of calling Python API.
     * Used by process_sonar_image() in OpenMP parallel region.
     *
     * @param bearing_idx Bearing index in polar image
     * @param num_beams Total number of bearings (for angle calculation)
     * @param polar_image Pointer to 2D polar image (num_range_bins x num_beams)
     * @param num_range_bins Number of range bins in polar image
     * @param T_sonar_to_world Transformation matrix
     * @param sonar_origin_world Sonar origin in world frame (cached)
     * @param first_hit_map First hit ranges for all bearings (meters)
     * @param voxel_updates Output buffer to collect voxel updates
     */
    void process_single_ray_internal(
        int bearing_idx,
        int num_beams,
        const uint8_t* polar_image,
        int num_range_bins,
        const Eigen::Matrix4d& T_sonar_to_world,
        const Eigen::Vector3d& sonar_origin_world,
        const std::vector<double>& first_hit_map,
        std::vector<VoxelUpdate>& voxel_updates
    );


    /**
     * @brief Find first intensity peak above threshold
     * @param intensity_profile Intensity values (0-255)
     * @return Index of first hit, or -1 if none found
     */
    int find_first_hit(const std::vector<uint8_t>& intensity_profile) const;

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
    double compute_bearing_angle(int bearing_idx, int num_beams) const;

    /**
     * @brief Compute intensity-based weighting factor
     *
     * Sigmoid function: w(I) = sigmoid((I - I_mid) / (sharpness * scale))
     * High intensity (>127) → weight > 0.5 → stronger occupied update
     * Low intensity (<127) → weight < 0.5 → weaker occupied update
     *
     * @param intensity Intensity value (0-255)
     * @return Weight factor (0.0 to 1.0)
     */
    double compute_intensity_weight(uint8_t intensity) const;

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
     * @brief Compute number of vertical steps for full voxel coverage
     *
     * Full coverage: sample every voxel in vertical aperture
     * vertical_spread = range × tan(half_aperture)
     * num_steps = ceil(vertical_spread / voxel_resolution)
     *
     * @param range_m Range in meters
     * @return Number of vertical steps (±N)
     */
    int compute_num_vertical_steps(double range_m) const;

    /**
     * @brief Compute number of horizontal steps for full voxel coverage
     *
     * Each bearing covers angular width: (horizontal_fov / num_beams) * bearing_step
     * horizontal_spread = range × tan(bearing_spread / 2)
     * num_steps = ceil(horizontal_spread / voxel_resolution)
     *
     * @param range_m Range in meters
     * @param bearing_step Bearing sampling step (e.g., 2 = every 2nd bearing)
     * @param num_beams Total number of bearings
     * @return Number of horizontal steps (±N), 0 means single ray
     */
    int compute_num_horizontal_steps(double range_m, int bearing_step, int num_beams) const;

private:
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
    double half_aperture_;      // Cached vertical_fov / 2

    // Profiling counters (P3.1: exp() measurement)
    // Using mutable atomic for thread-safe updates in const methods
    mutable std::atomic<size_t> exp_call_count_{0};
    mutable std::atomic<int64_t> exp_time_ns_{0};  // Nanoseconds for precision
};
