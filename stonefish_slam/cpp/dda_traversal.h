#pragma once

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <algorithm>
#include <unordered_map>

namespace py = pybind11;

// Configuration for sonar ray processing
struct SonarRayConfig {
    double voxel_size;
    double log_odds_free;
    double range_max;
    double range_min;
    double vertical_fov;
    bool use_range_weighting;
    double lambda_decay;
    bool enable_gaussian_weighting;
    double gaussian_sigma_factor;
};

// Voxel update result
struct VoxelUpdate {
    std::array<int, 3> key;
    double log_odds_sum;
    int count;
};

/**
 * @brief 3D DDA (Digital Differential Analyzer) voxel traversal
 *
 * Implements Amanatides & Woo (1987) algorithm for fast ray-voxel intersection.
 * Traverses voxels along a ray from start to end point with O(n) complexity
 * where n = number of voxels intersected.
 *
 * Reference: "A Fast Voxel Traversal Algorithm for Ray Tracing", Eurographics 1987
 */
class DDATraversal {
public:
    /**
     * @param voxel_size Voxel resolution in meters (default: 0.2m)
     */
    explicit DDATraversal(double voxel_size = 0.2);

    /**
     * @brief Convert world coordinates to voxel key (grid index)
     * @param point World coordinates (x, y, z)
     * @return Voxel key [ix, iy, iz]
     */
    std::array<int, 3> world_to_key(const Eigen::Vector3d& point);

    /**
     * @brief Traverse voxels from start to end using DDA algorithm
     *
     * @param start Start point in world coordinates
     * @param end End point in world coordinates
     * @param max_voxels Maximum voxels to traverse (safety limit)
     * @return Vector of voxel keys [x, y, z] as integer grid indices
     */
    std::vector<std::array<int, 3>> traverse(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        int max_voxels = 10000
    );

    /**
     * @brief Process entire vertical fan for free space ray
     *
     * Performs DDA traversal for multiple vertical angles, accumulates voxel updates.
     * This minimizes Python ↔ C++ boundary crossings (1 call instead of 10).
     *
     * @param sonar_origin Origin point in world frame
     * @param ray_direction_horizontal Horizontal ray direction (unit vector)
     * @param range_to_first_hit Maximum range to traverse
     * @param num_vertical_steps Number of vertical samples (±N steps)
     * @param config Sonar configuration parameters
     * @return Vector of accumulated voxel updates
     */
    std::vector<VoxelUpdate> process_free_space_ray(
        const Eigen::Vector3d& sonar_origin,
        const Eigen::Vector3d& ray_direction_horizontal,
        double range_to_first_hit,
        int num_vertical_steps,
        const SonarRayConfig& config
    );

private:
    double voxel_size_;

    // Helper: Convert voxel array to hashable key
    struct VoxelKey {
        int x, y, z;
        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash {
        size_t operator()(const VoxelKey& k) const {
            return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1) ^ (std::hash<int>()(k.z) << 2);
        }
    };

    VoxelKey voxel_to_key(const std::array<int, 3>& arr) const;
};
