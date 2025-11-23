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
    double max_range;
    double min_range;
    double vertical_aperture;
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
    explicit DDATraversal(double voxel_size = 0.2) : voxel_size_(voxel_size) {
        if (voxel_size_ <= 0.0) {
            throw std::invalid_argument("Voxel size must be positive");
        }
    }

    /**
     * @brief Convert world coordinates to voxel key (grid index)
     * @param point World coordinates (x, y, z)
     * @return Voxel key [ix, iy, iz]
     */
    std::array<int, 3> world_to_key(const Eigen::Vector3d& point) {
        return {
            static_cast<int>(std::floor(point.x() / voxel_size_)),
            static_cast<int>(std::floor(point.y() / voxel_size_)),
            static_cast<int>(std::floor(point.z() / voxel_size_))
        };
    }

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
    ) {
        std::vector<std::array<int, 3>> voxel_list;

        // Input validation
        if (max_voxels <= 0) {
            return voxel_list;
        }

        // Check for NaN or Inf
        if (!start.allFinite() || !end.allFinite()) {
            return voxel_list;
        }

        // Calculate ray direction
        Eigen::Vector3d ray = end - start;
        double ray_length = ray.norm();

        // Handle zero-length ray (start == end)
        if (ray_length < 1e-10) {
            voxel_list.push_back(world_to_key(start));
            return voxel_list;
        }

        // Normalize ray direction
        ray.normalize();

        // Get starting voxel
        std::array<int, 3> current_voxel = world_to_key(start);
        std::array<int, 3> end_voxel = world_to_key(end);

        // Add starting voxel
        voxel_list.push_back(current_voxel);

        // Calculate step direction for each axis
        std::array<int, 3> step;
        for (int i = 0; i < 3; ++i) {
            if (ray[i] > 0) {
                step[i] = 1;
            } else if (ray[i] < 0) {
                step[i] = -1;
            } else {
                step[i] = 0;
            }
        }

        // Calculate tMax and tDelta for DDA algorithm
        // tMax[i] = parameter t at which ray crosses next voxel boundary on axis i
        // tDelta[i] = parametric distance along ray to cross one voxel on axis i
        Eigen::Vector3d tMax, tDelta;

        for (int i = 0; i < 3; ++i) {
            if (std::abs(ray[i]) < 1e-10) {
                // Ray parallel to this axis
                tMax[i] = std::numeric_limits<double>::max();
                tDelta[i] = std::numeric_limits<double>::max();
            } else {
                // Calculate next voxel boundary on this axis
                double next_boundary;
                if (step[i] > 0) {
                    next_boundary = (current_voxel[i] + 1) * voxel_size_;
                } else {
                    next_boundary = current_voxel[i] * voxel_size_;
                }

                // Distance from start to next boundary divided by ray direction
                tMax[i] = (next_boundary - start[i]) / ray[i];

                // Distance to cross one voxel
                tDelta[i] = voxel_size_ / std::abs(ray[i]);
            }
        }

        // DDA main loop
        while (voxel_list.size() < static_cast<size_t>(max_voxels)) {
            // Check if we reached the end voxel
            if (current_voxel[0] == end_voxel[0] &&
                current_voxel[1] == end_voxel[1] &&
                current_voxel[2] == end_voxel[2]) {
                break;
            }

            // Find axis with minimum tMax (next voxel boundary to cross)
            int min_axis = 0;
            if (tMax[1] < tMax[0]) min_axis = 1;
            if (tMax[2] < tMax[min_axis]) min_axis = 2;

            // Advance along that axis
            current_voxel[min_axis] += step[min_axis];
            tMax[min_axis] += tDelta[min_axis];

            // Add new voxel to list
            voxel_list.push_back(current_voxel);
        }

        return voxel_list;
    }

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
    ) {
        std::unordered_map<VoxelKey, VoxelUpdate, VoxelKeyHash> updates_map;

        double half_aperture = config.vertical_aperture / 2.0;

        // Vertical fan loop (moved from Python to C++)
        for (int v_step = -num_vertical_steps; v_step <= num_vertical_steps; ++v_step) {
            // Compute vertical angle
            double vertical_angle = (num_vertical_steps > 0) ?
                (v_step / (double)num_vertical_steps) * half_aperture : 0.0;

            // Rotate horizontal direction by vertical angle
            // Simple pitch rotation:
            // x' = x * cos(v) - z * sin(v)
            // y' = y
            // z' = x * sin(v) + z * cos(v)
            double cos_v = std::cos(vertical_angle);
            double sin_v = std::sin(vertical_angle);

            Eigen::Vector3d tilted_direction(
                ray_direction_horizontal[0] * cos_v - ray_direction_horizontal[2] * sin_v,
                ray_direction_horizontal[1],
                ray_direction_horizontal[0] * sin_v + ray_direction_horizontal[2] * cos_v
            );
            tilted_direction.normalize();

            // End point for this vertical angle
            Eigen::Vector3d end_point = sonar_origin + tilted_direction * range_to_first_hit;

            // DDA traversal
            std::vector<std::array<int, 3>> voxels = traverse(sonar_origin, end_point, 500);

            // Compute Gaussian weight once for this vertical angle
            double gaussian_weight = 1.0;
            if (config.enable_gaussian_weighting && half_aperture > 0) {
                double normalized_angle = vertical_angle / half_aperture;
                gaussian_weight = std::exp(-0.5 * std::pow(normalized_angle * config.gaussian_sigma_factor, 2));
            }

            // Process each voxel
            for (const auto& voxel_arr : voxels) {
                // Compute voxel center
                Eigen::Vector3d voxel_center(
                    voxel_arr[0] * config.voxel_size,
                    voxel_arr[1] * config.voxel_size,
                    voxel_arr[2] * config.voxel_size
                );

                // Compute range
                double range = (voxel_center - sonar_origin).norm();

                // Skip if below min_range
                if (range <= config.min_range) continue;

                // Range weighting
                double range_weight = 1.0;
                if (config.use_range_weighting) {
                    range_weight = std::exp(-config.lambda_decay * range / config.max_range);
                }

                // Total log-odds update
                double log_odds_update = config.log_odds_free * range_weight * gaussian_weight;

                // Accumulate in map (handle duplicates)
                VoxelKey key = voxel_to_key(voxel_arr);
                if (updates_map.find(key) == updates_map.end()) {
                    updates_map[key] = {voxel_arr, 0.0, 0};
                }
                updates_map[key].log_odds_sum += log_odds_update;
                updates_map[key].count++;
            }
        }

        // Convert map to vector
        std::vector<VoxelUpdate> result;
        result.reserve(updates_map.size());
        for (const auto& [key, update] : updates_map) {
            result.push_back(update);
        }

        return result;
    }

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

    VoxelKey voxel_to_key(const std::array<int, 3>& arr) const {
        return {arr[0], arr[1], arr[2]};
    }
};

// Pybind11 module definition
PYBIND11_MODULE(dda_traversal, m) {
    m.doc() = "DDA 3D voxel traversal (Amanatides-Woo 1987)";

    // Add struct bindings
    py::class_<SonarRayConfig>(m, "SonarRayConfig")
        .def(py::init<>())
        .def_readwrite("voxel_size", &SonarRayConfig::voxel_size)
        .def_readwrite("log_odds_free", &SonarRayConfig::log_odds_free)
        .def_readwrite("max_range", &SonarRayConfig::max_range)
        .def_readwrite("min_range", &SonarRayConfig::min_range)
        .def_readwrite("vertical_aperture", &SonarRayConfig::vertical_aperture)
        .def_readwrite("use_range_weighting", &SonarRayConfig::use_range_weighting)
        .def_readwrite("lambda_decay", &SonarRayConfig::lambda_decay)
        .def_readwrite("enable_gaussian_weighting", &SonarRayConfig::enable_gaussian_weighting)
        .def_readwrite("gaussian_sigma_factor", &SonarRayConfig::gaussian_sigma_factor);

    py::class_<VoxelUpdate>(m, "VoxelUpdate")
        .def(py::init<>())
        .def_readwrite("key", &VoxelUpdate::key)
        .def_readwrite("log_odds_sum", &VoxelUpdate::log_odds_sum)
        .def_readwrite("count", &VoxelUpdate::count);

    py::class_<DDATraversal>(m, "DDATraversal")
        .def(py::init<double>(),
             py::arg("voxel_size") = 0.2,
             "Initialize DDA traversal with voxel size")
        .def("traverse",
             &DDATraversal::traverse,
             py::arg("start"),
             py::arg("end"),
             py::arg("max_voxels") = 10000,
             "Traverse voxels from start to end point using DDA")
        .def("world_to_key",
             &DDATraversal::world_to_key,
             py::arg("point"),
             "Convert world coordinates to voxel key")
        .def("process_free_space_ray",
             &DDATraversal::process_free_space_ray,
             py::arg("sonar_origin"),
             py::arg("ray_direction_horizontal"),
             py::arg("range_to_first_hit"),
             py::arg("num_vertical_steps"),
             py::arg("config"),
             "Process entire vertical fan for free space (batch processing)");
}
