#include "octree_mapping.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pybind11/eigen.h>
#include <stdexcept>
#include <cmath>

#ifdef _OPENMP
#include <omp.h>
#endif

OctreeMapping::OctreeMapping(double resolution)
    : resolution_(resolution),
      log_odds_occupied_(0.85),
      log_odds_free_(-3.5)  // Extreme free space decrease for testing (clamping min)
{
    if (resolution_ <= 0.0) {
        throw std::invalid_argument("Resolution must be positive");
    }

    // Create OctoMap tree with specified resolution
    tree_ = std::make_unique<octomap::OcTree>(resolution_);

    // Set default clamping thresholds (prevent extreme probabilities)
    // Fixed: Symmetric clamping ±3.5 log-odds for balanced free/occupied updates
    tree_->setClampingThresMin(0.03);  // log-odds -3.5 (was -2.0, asymmetric)
    tree_->setClampingThresMax(0.97);  // log-odds +3.5

    // Enable automatic pruning for memory efficiency
    tree_->enableChangeDetection(true);

#ifdef _OPENMP
    int max_threads = omp_get_max_threads();
    std::cout << "[OctreeMapping] OpenMP enabled: " << max_threads << " threads available" << std::endl;
#else
    std::cout << "[OctreeMapping] OpenMP not available, running single-threaded" << std::endl;
#endif
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

    // Batch insert all points with OpenMP parallelization
    // Strategy: Each thread accumulates updates in local buffer, then applies with mutex
    // This minimizes lock contention and maximizes parallelism

#ifdef _OPENMP
    // Thread-local storage for batched updates
    struct UpdateBatch {
        std::vector<octomap::OcTreeKey> keys;
        std::vector<double> log_odds_updates;

        void reserve(size_t n) {
            keys.reserve(n);
            log_odds_updates.reserve(n);
        }

        void add(const octomap::OcTreeKey& key, double log_odds) {
            keys.push_back(key);
            log_odds_updates.push_back(log_odds);
        }
    };

    // Parallel phase: compute keys and prepare updates (no tree access)
    std::vector<UpdateBatch> thread_batches(omp_get_max_threads());
    for (auto& batch : thread_batches) {
        batch.reserve(num_points / omp_get_max_threads() + 100);
    }

    #pragma omp parallel
    {
        int thread_id = omp_get_thread_num();
        UpdateBatch& my_batch = thread_batches[thread_id];

        #pragma omp for schedule(dynamic, 100) nowait
        for (size_t i = 0; i < num_points; ++i) {
            // Get point coordinates (row-major: [x, y, z])
            double x = points_ptr[i * 3 + 0];
            double y = points_ptr[i * 3 + 1];
            double z = points_ptr[i * 3 + 2];
            octomap::point3d endpoint(x, y, z);

            // Get log-odds update for this point
            double log_odds_update = log_odds_ptr[i];

            // Compute key (no tree modification)
            octomap::OcTreeKey key;
            if (tree_->coordToKeyChecked(endpoint, key)) {
                my_batch.add(key, log_odds_update);
            }
        }
    }

    // Sequential phase: merge all updates into tree
    // Use OctoMap's updateNode() API for incremental updates (enables pruning)
    for (const auto& batch : thread_batches) {
        for (size_t i = 0; i < batch.keys.size(); ++i) {
            const auto& key = batch.keys[i];
            double log_odds_update = batch.log_odds_updates[i];

            // Use updateNode() API for incremental update
            // This ensures pruning is performed automatically
            tree_->updateNode(key, static_cast<float>(log_odds_update), true);
        }
    }

#else
    // Single-threaded fallback
    for (size_t i = 0; i < num_points; ++i) {
        // Get point coordinates (row-major: [x, y, z])
        double x = points_ptr[i * 3 + 0];
        double y = points_ptr[i * 3 + 1];
        double z = points_ptr[i * 3 + 2];
        octomap::point3d endpoint(x, y, z);

        // Get log-odds update for this point
        double log_odds_update = log_odds_ptr[i];

        // Use updateNode() API for incremental update (enables pruning)
        octomap::OcTreeKey key;
        if (tree_->coordToKeyChecked(endpoint, key)) {
            tree_->updateNode(key, static_cast<float>(log_odds_update), true);
        }
    }
#endif

    // Optional: prune tree to reduce memory (can be expensive)
    // tree_->prune();
}

py::array_t<double> OctreeMapping::get_occupied_cells(double threshold) {
    if (threshold < 0.0 || threshold > 1.0) {
        throw std::invalid_argument("Threshold must be in [0, 1]");
    }

    // First pass: count cells above threshold
    // Fixed: Removed isNodeOccupied() double-filtering, use threshold only (occupied voxels)
    size_t occupied_count = 0;
    for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs();
         it != tree_->end_leafs(); ++it) {
        double occupancy = it->getOccupancy();
        if (occupancy >= threshold) {
            occupied_count++;
        }
    }

    // Allocate NumPy array (N x 4: x, y, z, log_odds)
    py::array_t<double> result(std::vector<ssize_t>{static_cast<ssize_t>(occupied_count), 4});
    auto result_buf = result.request();
    double* result_ptr = static_cast<double*>(result_buf.ptr);

    // Second pass: fill array with voxel centers and log-odds
    // Fixed: Removed isNodeOccupied() double-filtering, use threshold only (occupied voxels)
    size_t idx = 0;
    for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs();
         it != tree_->end_leafs(); ++it) {
        double occupancy = it->getOccupancy();
        if (occupancy >= threshold) {
            // Get voxel center coordinates using keyToCoord (more reliable)
            octomap::point3d coord = tree_->keyToCoord(it.getKey());
            result_ptr[idx * 4 + 0] = coord.x();
            result_ptr[idx * 4 + 1] = coord.y();
            result_ptr[idx * 4 + 2] = coord.z();
            result_ptr[idx * 4 + 3] = it->getLogOdds();  // Add log-odds value
            idx++;
        }
    }

    return result;
}

double OctreeMapping::query_cell(double x, double y, double z) {
    octomap::point3d query_point(x, y, z);

    // Convert point to voxel key (handles arbitrary coordinates)
    octomap::OcTreeKey key;
    if (!tree_->coordToKeyChecked(query_point, key)) {
        return 0.5;  // Out of bounds
    }

    // Search using key (not coordinate)
    octomap::OcTreeNode* node = tree_->search(key);

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
             "    Nx4 NumPy array of occupied voxel centers [x, y, z, log_odds]")
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
