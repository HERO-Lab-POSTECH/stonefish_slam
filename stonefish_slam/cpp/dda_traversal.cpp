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

namespace py = pybind11;

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

private:
    double voxel_size_;
};

// Pybind11 module definition
PYBIND11_MODULE(dda_traversal, m) {
    m.doc() = "DDA 3D voxel traversal (Amanatides-Woo 1987)";

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
             "Convert world coordinates to voxel key");
}
