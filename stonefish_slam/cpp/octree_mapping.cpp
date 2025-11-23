#include "octree_mapping.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pybind11/eigen.h>
#include <stdexcept>
#include <cmath>

OctreeMapping::OctreeMapping(double resolution)
    : resolution_(resolution),
      log_odds_occupied_(0.85),
      log_odds_free_(-0.4)
{
    if (resolution_ <= 0.0) {
        throw std::invalid_argument("Resolution must be positive");
    }

    // Create OctoMap tree with specified resolution
    tree_ = std::make_unique<octomap::OcTree>(resolution_);

    // Set default clamping thresholds (prevent extreme probabilities)
    tree_->setClampingThresMin(0.12);  // Min probability ~12%
    tree_->setClampingThresMax(0.97);  // Max probability ~97%

    // Enable automatic pruning for memory efficiency
    tree_->enableChangeDetection(true);
}

OctreeMapping::~OctreeMapping() {
    // unique_ptr automatically deletes tree_
}

void OctreeMapping::insert_point_cloud(
    py::array_t<double> points,
    py::array_t<double> log_odds,
    py::array_t<double> sensor_origin
) {
    // Input validation
    auto points_buf = points.request();
    auto log_odds_buf = log_odds.request();
    auto origin_buf = sensor_origin.request();

    // Check dimensions
    if (points_buf.ndim != 2 || points_buf.shape[1] != 3) {
        throw std::runtime_error("Points must be Nx3 array");
    }
    if (log_odds_buf.ndim != 1) {
        throw std::runtime_error("Log odds must be 1D array");
    }
    if (origin_buf.ndim != 1 || origin_buf.shape[0] != 3) {
        throw std::runtime_error("Sensor origin must be 3-element array");
    }

    size_t num_points = points_buf.shape[0];
    if (log_odds_buf.shape[0] != static_cast<ssize_t>(num_points)) {
        throw std::runtime_error("Points and log_odds must have same length");
    }

    // Zero-copy access to NumPy data
    const double* points_ptr = static_cast<const double*>(points_buf.ptr);
    const double* log_odds_ptr = static_cast<const double*>(log_odds_buf.ptr);
    const double* origin_ptr = static_cast<const double*>(origin_buf.ptr);

    // Convert sensor origin to OctoMap point
    octomap::point3d sensor_pos(origin_ptr[0], origin_ptr[1], origin_ptr[2]);

    // Batch insert all points
    for (size_t i = 0; i < num_points; ++i) {
        // Get point coordinates (row-major: [x, y, z])
        double x = points_ptr[i * 3 + 0];
        double y = points_ptr[i * 3 + 1];
        double z = points_ptr[i * 3 + 2];
        octomap::point3d endpoint(x, y, z);

        // Get log-odds update for this point
        double log_odds_update = log_odds_ptr[i];

        // Update node using OctoMap's built-in log-odds logic
        // updateNode internally converts log-odds to probability, updates, and clamps
        octomap::OcTreeKey key;
        if (tree_->coordToKeyChecked(endpoint, key)) {
            // Direct log-odds update (OctoMap handles clamping)
            tree_->updateNode(key, log_odds_update > 0.0);

            // For more precise control, manually set log-odds
            octomap::OcTreeNode* node = tree_->search(key);
            if (node) {
                // Get current log-odds
                double current_log_odds = node->getLogOdds();
                // Add update
                double new_log_odds = current_log_odds + log_odds_update;
                // Clamp to prevent overflow (cast to double for type matching)
                double min_log = static_cast<double>(tree_->getClampingThresMinLog());
                double max_log = static_cast<double>(tree_->getClampingThresMaxLog());
                new_log_odds = std::max(min_log, std::min(new_log_odds, max_log));
                node->setLogOdds(static_cast<float>(new_log_odds));
            }
        }
    }

    // Optional: prune tree to reduce memory (can be expensive)
    // tree_->prune();
}

py::array_t<double> OctreeMapping::get_occupied_cells(double threshold) {
    if (threshold < 0.0 || threshold > 1.0) {
        throw std::invalid_argument("Threshold must be in [0, 1]");
    }

    // First pass: count occupied cells
    size_t occupied_count = 0;
    for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs();
         it != tree_->end_leafs(); ++it) {
        if (tree_->isNodeOccupied(*it)) {
            double occupancy = it->getOccupancy();
            if (occupancy >= threshold) {
                occupied_count++;
            }
        }
    }

    // Allocate NumPy array (N x 3)
    py::array_t<double> result(std::vector<ssize_t>{static_cast<ssize_t>(occupied_count), 3});
    auto result_buf = result.request();
    double* result_ptr = static_cast<double*>(result_buf.ptr);

    // Second pass: fill array with voxel centers
    size_t idx = 0;
    for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs();
         it != tree_->end_leafs(); ++it) {
        if (tree_->isNodeOccupied(*it)) {
            double occupancy = it->getOccupancy();
            if (occupancy >= threshold) {
                // Get voxel center coordinates
                octomap::point3d coord = it.getCoordinate();
                result_ptr[idx * 3 + 0] = coord.x();
                result_ptr[idx * 3 + 1] = coord.y();
                result_ptr[idx * 3 + 2] = coord.z();
                idx++;
            }
        }
    }

    return result;
}

double OctreeMapping::query_cell(double x, double y, double z) {
    octomap::point3d query_point(x, y, z);
    octomap::OcTreeNode* node = tree_->search(query_point);

    if (node) {
        return node->getOccupancy();  // Returns probability in [0, 1]
    } else {
        return 0.5;  // Unknown = 0.5 probability
    }
}

void OctreeMapping::clear() {
    tree_->clear();
}

size_t OctreeMapping::get_num_nodes() const {
    return tree_->size();
}

void OctreeMapping::set_log_odds_thresholds(double occupied, double free) {
    log_odds_occupied_ = occupied;
    log_odds_free_ = free;
}

void OctreeMapping::set_clamping_thresholds(double min, double max) {
    if (min <= 0.0 || min >= 1.0 || max <= 0.0 || max >= 1.0 || min >= max) {
        throw std::invalid_argument("Invalid clamping thresholds");
    }
    tree_->setClampingThresMin(min);
    tree_->setClampingThresMax(max);
}

// Pybind11 module definition
PYBIND11_MODULE(octree_mapping, m) {
    m.doc() = "High-performance Octree mapping using OctoMap library";

    py::class_<OctreeMapping>(m, "OctreeMapping")
        .def(py::init<double>(),
             py::arg("resolution") = 0.1,
             "Initialize Octree with specified voxel resolution (meters)")
        .def("insert_point_cloud",
             &OctreeMapping::insert_point_cloud,
             py::arg("points"),
             py::arg("log_odds"),
             py::arg("sensor_origin"),
             "Batch insert point cloud with log-odds updates\n\n"
             "Args:\n"
             "    points: Nx3 NumPy array of [x, y, z] coordinates\n"
             "    log_odds: N-length array of log-odds updates\n"
             "    sensor_origin: [x, y, z] sensor position")
        .def("get_occupied_cells",
             &OctreeMapping::get_occupied_cells,
             py::arg("threshold") = 0.5,
             "Get all occupied voxels above probability threshold\n\n"
             "Args:\n"
             "    threshold: Occupancy probability threshold (0.0 to 1.0)\n\n"
             "Returns:\n"
             "    Nx3 NumPy array of occupied voxel centers")
        .def("query_cell",
             &OctreeMapping::query_cell,
             py::arg("x"),
             py::arg("y"),
             py::arg("z"),
             "Query occupancy probability at specific location\n\n"
             "Returns:\n"
             "    Occupancy probability (0.0 to 1.0), or 0.5 if unknown")
        .def("clear",
             &OctreeMapping::clear,
             "Clear all voxels from the tree")
        .def("get_num_nodes",
             &OctreeMapping::get_num_nodes,
             "Get total number of nodes in the tree")
        .def("set_log_odds_thresholds",
             &OctreeMapping::set_log_odds_thresholds,
             py::arg("occupied"),
             py::arg("free"),
             "Set log-odds thresholds for occupied/free updates")
        .def("set_clamping_thresholds",
             &OctreeMapping::set_clamping_thresholds,
             py::arg("min"),
             py::arg("max"),
             "Set probability clamping limits (prevents extreme values)");
}
