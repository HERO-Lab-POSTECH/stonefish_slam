#include "octree_mapping.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pybind11/eigen.h>
#include <stdexcept>
#include <cmath>
#include <algorithm>

#ifdef _OPENMP
#include <omp.h>
#endif

OctreeMapping::OctreeMapping(double resolution)
    : resolution_(resolution),
      log_odds_occupied_(0.5),
      log_odds_free_(-5.0),  // Strong free space clearing (balanced with occupied)
      adaptive_update_(true),
      adaptive_threshold_(0.5),
      adaptive_max_ratio_(0.3),
      update_method_(UpdateMethod::LOG_ODDS),  // Default to LOG_ODDS
      intensity_threshold_(35.0),
      intensity_max_(255.0),
      sharpness_(3.0),
      decay_rate_(0.1),
      min_alpha_(0.1),
      L_min_(-2.0),
      L_max_(3.5),
      lut_initialized_(false)
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

    // Initialize intensity weight LUT
    initialize_lut();

// #ifdef _OPENMP
//     int max_threads = omp_get_max_threads();
//     std::cout << "[OctreeMapping] OpenMP enabled: " << max_threads << " threads available" << std::endl;
// #else
//     std::cout << "[OctreeMapping] OpenMP not available, running single-threaded" << std::endl;
// #endif
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
    // Optimization: Call updateNode first, then use returned node for adaptive adjustment
    for (const auto& batch : thread_batches) {
        for (size_t i = 0; i < batch.keys.size(); ++i) {
            const auto& key = batch.keys[i];
            double log_odds_update = batch.log_odds_updates[i];

            // IWLO: Apply alpha decay and saturation
            if (update_method_ == UpdateMethod::IWLO) {
                uint64_t hash = key_to_hash(key);
                int obs_count = observation_counts_[hash];
                observation_counts_[hash] = obs_count + 1;

                // Apply alpha decay to both occupied and free space updates
                double alpha = compute_alpha(obs_count);
                log_odds_update *= alpha;

                // Apply saturation
                octomap::OcTreeNode* node = tree_->search(key);
                double current_log_odds = node ? node->getLogOdds() : 0.0;
                double new_log_odds = std::max(L_min_, std::min(L_max_, current_log_odds + log_odds_update));
                log_odds_update = new_log_odds - current_log_odds;
            }

            // Adaptive protection (unidirectional: Free → Occupied only)
            double final_log_odds = log_odds_update;
            if (adaptive_update_ && log_odds_update > 0.0) {
                // Check current probability BEFORE update
                octomap::OcTreeNode* node = tree_->search(key);
                double current_prob = node ? node->getOccupancy() : 0.5;

                // Apply protection if needed
                if (current_prob <= adaptive_threshold_) {
                    double update_scale = (current_prob / adaptive_threshold_) * adaptive_max_ratio_;
                    final_log_odds = log_odds_update * update_scale;
                }
            }
            // Single update with final log_odds
            tree_->updateNode(key, static_cast<float>(final_log_odds), false);
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

        // Use updateNode() API for incremental update (enables pruning, lazy_eval=false)
        octomap::OcTreeKey key;
        if (tree_->coordToKeyChecked(endpoint, key)) {
            // IWLO: Apply alpha decay and saturation
            if (update_method_ == UpdateMethod::IWLO) {
                uint64_t hash = key_to_hash(key);
                int obs_count = observation_counts_[hash];
                observation_counts_[hash] = obs_count + 1;

                // Apply alpha decay to both occupied and free space updates
                double alpha = compute_alpha(obs_count);
                log_odds_update *= alpha;

                // Apply saturation
                octomap::OcTreeNode* node = tree_->search(key);
                double current_log_odds = node ? node->getLogOdds() : 0.0;
                double new_log_odds = std::max(L_min_, std::min(L_max_, current_log_odds + log_odds_update));
                log_odds_update = new_log_odds - current_log_odds;
            }

            // Adaptive protection (unidirectional: Free → Occupied only)
            double final_log_odds = log_odds_update;
            if (adaptive_update_ && log_odds_update > 0.0) {
                // Check current probability BEFORE update
                octomap::OcTreeNode* node = tree_->search(key);
                double current_prob = node ? node->getOccupancy() : 0.5;

                // Apply protection if needed
                if (current_prob <= adaptive_threshold_) {
                    double update_scale = (current_prob / adaptive_threshold_) * adaptive_max_ratio_;
                    final_log_odds = log_odds_update * update_scale;
                }
            }
            // Single update with final log_odds
            tree_->updateNode(key, static_cast<float>(final_log_odds), false);
        }
    }
#endif

    // Optional: prune tree to reduce memory (can be expensive)
    // tree_->prune();
}

void OctreeMapping::insert_point_cloud_with_intensity_and_logodds(
    py::array_t<double> points,
    py::array_t<double> intensities,
    py::array_t<double> log_odds,
    py::array_t<double> sensor_origin
) {
    // Input validation
    auto points_buf = points.request();
    auto intensities_buf = intensities.request();
    auto log_odds_buf = log_odds.request();
    auto origin_buf = sensor_origin.request();

    // Check dimensions
    if (points_buf.ndim != 2 || points_buf.shape[1] != 3) {
        throw std::runtime_error("Points must be Nx3 array");
    }
    if (intensities_buf.ndim != 1) {
        throw std::runtime_error("Intensities must be 1D array");
    }
    if (log_odds_buf.ndim != 1) {
        throw std::runtime_error("Log odds must be 1D array");
    }
    if (origin_buf.ndim != 1 || origin_buf.shape[0] != 3) {
        throw std::runtime_error("Sensor origin must be 3-element array");
    }

    size_t num_points = points_buf.shape[0];

    if (intensities_buf.shape[0] != static_cast<ssize_t>(num_points)) {
        throw std::runtime_error("Points and intensities must have same length");
    }
    if (log_odds_buf.shape[0] != static_cast<ssize_t>(num_points)) {
        throw std::runtime_error("Points and log_odds must have same length");
    }

    // Zero-copy access to NumPy data
    const double* points_ptr = static_cast<const double*>(points_buf.ptr);
    const double* intensities_ptr = static_cast<const double*>(intensities_buf.ptr);
    const double* log_odds_ptr = static_cast<const double*>(log_odds_buf.ptr);

    // For LOG_ODDS method, fall back to standard insert_point_cloud
    if (update_method_ == UpdateMethod::LOG_ODDS) {
        insert_point_cloud(points, log_odds, sensor_origin);
        return;
    }

    // For WEIGHTED_AVERAGE and IWLO: process with intensity and log-odds
#ifdef _OPENMP
    // Parallel phase: compute keys and prepare updates
    struct UpdateBatch {
        std::vector<octomap::OcTreeKey> keys;
        std::vector<double> intensities;
        std::vector<double> log_odds_vals;

        void reserve(size_t n) {
            keys.reserve(n);
            intensities.reserve(n);
            log_odds_vals.reserve(n);
        }

        void add(const octomap::OcTreeKey& key, double intensity, double log_odds_val) {
            keys.push_back(key);
            intensities.push_back(intensity);
            log_odds_vals.push_back(log_odds_val);
        }
    };

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
            double x = points_ptr[i * 3 + 0];
            double y = points_ptr[i * 3 + 1];
            double z = points_ptr[i * 3 + 2];
            octomap::point3d endpoint(x, y, z);

            double intensity = intensities_ptr[i];
            double log_odds_val = log_odds_ptr[i];

            octomap::OcTreeKey key;
            if (tree_->coordToKeyChecked(endpoint, key)) {
                my_batch.add(key, intensity, log_odds_val);
            }
        }
    }

    // Sequential phase: apply updates based on method
    for (const auto& batch : thread_batches) {
        for (size_t i = 0; i < batch.keys.size(); ++i) {
            const auto& key = batch.keys[i];
            double intensity = batch.intensities[i];
            double passed_log_odds = batch.log_odds_vals[i];

            uint64_t hash = key_to_hash(key);

            if (update_method_ == UpdateMethod::WEIGHTED_AVERAGE) {
                // Weighted Average: P_new = (n*P_old + w(I)) / (n+1)
                int obs_count = observation_counts_[hash];
                observation_counts_[hash] = obs_count + 1;

                octomap::OcTreeNode* node = tree_->search(key);
                double current_prob = node ? node->getOccupancy() : 0.5;

                double weight = intensity_to_weight(intensity);
                double new_prob = (obs_count * current_prob + weight) / (obs_count + 1);

                double new_log_odds = std::log(new_prob / (1.0 - new_prob + 1e-10));
                tree_->updateNode(key, static_cast<float>(new_log_odds), false);

            } else if (update_method_ == UpdateMethod::IWLO) {
                // IWLO: Apply alpha decay to ray_processor's log-odds
                int obs_count = observation_counts_[hash];
                observation_counts_[hash] = obs_count + 1;

                double alpha = compute_alpha(obs_count);
                double delta_L = passed_log_odds * alpha;

                // Adaptive protection (Free → Occupied only)
                // Only protect voxels that are already FREE (P < threshold)
                // Unknown voxels (P = 0.5) should NOT be protected
                if (adaptive_update_ && delta_L > 0.0) {
                    octomap::OcTreeNode* check_node = tree_->search(key);
                    if (check_node) {
                        double current_prob = check_node->getOccupancy();

                        if (current_prob < adaptive_threshold_) {
                            double update_scale = (current_prob / adaptive_threshold_) * adaptive_max_ratio_;
                            delta_L *= update_scale;
                        }
                    }
                    // New voxels (check_node == nullptr) are NOT protected
                }

                // Saturation
                octomap::OcTreeNode* node = tree_->search(key);
                double current_log_odds = node ? node->getLogOdds() : 0.0;
                double new_log_odds = std::max(L_min_, std::min(L_max_, current_log_odds + delta_L));

                tree_->updateNode(key, static_cast<float>(new_log_odds - current_log_odds), false);
            }
        }
    }

#else
    // Single-threaded fallback
    for (size_t i = 0; i < num_points; ++i) {
        double x = points_ptr[i * 3 + 0];
        double y = points_ptr[i * 3 + 1];
        double z = points_ptr[i * 3 + 2];
        octomap::point3d endpoint(x, y, z);

        double intensity = intensities_ptr[i];
        double passed_log_odds = log_odds_ptr[i];

        octomap::OcTreeKey key;
        if (tree_->coordToKeyChecked(endpoint, key)) {
            uint64_t hash = key_to_hash(key);

            if (update_method_ == UpdateMethod::WEIGHTED_AVERAGE) {
                int obs_count = observation_counts_[hash];
                observation_counts_[hash] = obs_count + 1;

                octomap::OcTreeNode* node = tree_->search(key);
                double current_prob = node ? node->getOccupancy() : 0.5;

                double weight = intensity_to_weight(intensity);
                double new_prob = (obs_count * current_prob + weight) / (obs_count + 1);

                double new_log_odds = std::log(new_prob / (1.0 - new_prob + 1e-10));
                tree_->updateNode(key, static_cast<float>(new_log_odds), false);

            } else if (update_method_ == UpdateMethod::IWLO) {
                // IWLO: Apply alpha decay to ray_processor's log-odds
                int obs_count = observation_counts_[hash];
                observation_counts_[hash] = obs_count + 1;

                double alpha = compute_alpha(obs_count);
                double delta_L = passed_log_odds * alpha;

                // Adaptive protection (Free → Occupied only)
                // Only protect voxels that are already FREE (P < threshold)
                // Unknown voxels (P = 0.5) should NOT be protected
                if (adaptive_update_ && delta_L > 0.0) {
                    octomap::OcTreeNode* check_node = tree_->search(key);
                    if (check_node) {
                        double current_prob = check_node->getOccupancy();

                        if (current_prob < adaptive_threshold_) {
                            double update_scale = (current_prob / adaptive_threshold_) * adaptive_max_ratio_;
                            delta_L *= update_scale;
                        }
                    }
                    // New voxels (check_node == nullptr) are NOT protected
                }

                // Saturation
                octomap::OcTreeNode* node = tree_->search(key);
                double current_log_odds = node ? node->getLogOdds() : 0.0;
                double new_log_odds = std::max(L_min_, std::min(L_max_, current_log_odds + delta_L));

                tree_->updateNode(key, static_cast<float>(new_log_odds - current_log_odds), false);
            }
        }
    }
#endif
}

py::array_t<double> OctreeMapping::get_occupied_cells(double threshold) {
    if (threshold < 0.0 || threshold > 1.0) {
        throw std::invalid_argument("Threshold must be in [0, 1]");
    }

    // Single pass: collect occupied cells directly into vector
    // Optimization: avoid double iteration (count + fill)
    std::vector<double> cells_data;
    cells_data.reserve(tree_->getNumLeafNodes() / 2);  // Estimate: ~50% occupancy

    for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs();
         it != tree_->end_leafs(); ++it) {
        double occupancy = it->getOccupancy();
        if (occupancy >= threshold) {
            // Get voxel center coordinates using keyToCoord (more reliable)
            octomap::point3d coord = tree_->keyToCoord(it.getKey());
            cells_data.push_back(coord.x());
            cells_data.push_back(coord.y());
            cells_data.push_back(coord.z());
            cells_data.push_back(it->getLogOdds());
        }
    }

    // Calculate final count and create NumPy array
    size_t occupied_count = cells_data.size() / 4;
    py::array_t<double> result(std::vector<ssize_t>{static_cast<ssize_t>(occupied_count), 4});
    auto result_buf = result.request();
    double* result_ptr = static_cast<double*>(result_buf.ptr);

    // Copy data from vector to NumPy array
    std::copy(cells_data.begin(), cells_data.end(), result_ptr);

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
    observation_counts_.clear();
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

void OctreeMapping::set_adaptive_params(bool enable, double threshold, double max_ratio) {
    adaptive_update_ = enable;
    adaptive_threshold_ = threshold;
    adaptive_max_ratio_ = max_ratio;
}

// Get map statistics (P3.2 profiling)
MapStats OctreeMapping::get_map_stats() const {
    size_t nodes = tree_->calcNumNodes();
    size_t leaves = tree_->getNumLeafNodes();
    double mem_mb = tree_->memoryUsage() / (1024.0 * 1024.0);

    return {nodes, leaves, mem_mb};
}

// Set update method
void OctreeMapping::set_update_method(int method) {
    if (method < 0 || method > 2) {
        throw std::invalid_argument("Invalid update method (0=LOG_ODDS, 1=WEIGHTED_AVG, 2=IWLO)");
    }
    update_method_ = static_cast<UpdateMethod>(method);
}

// Set intensity parameters
void OctreeMapping::set_intensity_params(double threshold, double max_val) {
    if (threshold < 0 || max_val <= threshold) {
        throw std::invalid_argument("Invalid intensity parameters");
    }
    intensity_threshold_ = threshold;
    intensity_max_ = max_val;
    lut_initialized_ = false;  // LUT needs recalculation
    initialize_lut();
}

// Set IWLO parameters
void OctreeMapping::set_iwlo_params(double sharpness, double decay_rate, double min_alpha,
                                     double L_min, double L_max) {
    if (sharpness <= 0 || decay_rate < 0 || min_alpha <= 0 || min_alpha > 1.0 || L_min >= L_max) {
        throw std::invalid_argument("Invalid IWLO parameters");
    }
    sharpness_ = sharpness;
    decay_rate_ = decay_rate;
    min_alpha_ = min_alpha;
    L_min_ = L_min;
    L_max_ = L_max;
    lut_initialized_ = false;  // LUT needs recalculation if sharpness changed
    initialize_lut();
}

// Convert OcTreeKey to hash for observation counting
uint64_t OctreeMapping::key_to_hash(const octomap::OcTreeKey& key) const {
    // Simple hash: combine x, y, z using bit shifting
    return (static_cast<uint64_t>(key[0]) << 32) |
           (static_cast<uint64_t>(key[1]) << 16) |
            static_cast<uint64_t>(key[2]);
}

// Initialize intensity weight LUT (256 entries for 0-255 intensity range)
void OctreeMapping::initialize_lut() {
    if (lut_initialized_) return;

    for (int i = 0; i < 256; ++i) {
        if (i <= static_cast<int>(intensity_threshold_)) {
            intensity_weight_lut_[i] = 0.0;
        } else {
            double normalized = (static_cast<double>(i) - intensity_threshold_)
                               / (intensity_max_ - intensity_threshold_);
            double x = sharpness_ * (normalized - 0.5);
            intensity_weight_lut_[i] = 1.0 / (1.0 + std::exp(-x));
        }
    }
    lut_initialized_ = true;
}

// IWLO intensity to weight conversion (LUT-optimized)
double OctreeMapping::intensity_to_weight(double intensity) const {
    int idx = static_cast<int>(std::clamp(intensity, 0.0, 255.0));
    return intensity_weight_lut_[idx];
}

// IWLO adaptive learning rate
double OctreeMapping::compute_alpha(int obs_count) const {
    return std::max(min_alpha_, 1.0 / (1.0 + decay_rate_ * obs_count));
}

// Serialize OcTree to binary data for octomap_msgs
py::bytes OctreeMapping::serialize_to_binary() {
    std::stringstream ss;
    tree_->writeBinaryData(ss);
    std::string data = ss.str();
    return py::bytes(data);
}

// C++ native batch insert (zero NumPy overhead)
void OctreeMapping::insert_voxels_batch_native(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<double>& log_odds,
    const Eigen::Vector3d& sensor_origin
) {
    // Input validation
    if (points.size() != log_odds.size()) {
        throw std::runtime_error("Points and log_odds must have same length");
    }

    size_t num_points = points.size();
    if (num_points == 0) {
        return;  // No work to do
    }

    // Convert sensor origin to OctoMap point
    octomap::point3d sensor_pos(sensor_origin.x(), sensor_origin.y(), sensor_origin.z());

    // Identical logic to insert_point_cloud(), but using std::vector instead of NumPy
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
            const Eigen::Vector3d& pt = points[i];
            octomap::point3d endpoint(pt.x(), pt.y(), pt.z());

            double log_odds_update = log_odds[i];

            octomap::OcTreeKey key;
            if (tree_->coordToKeyChecked(endpoint, key)) {
                my_batch.add(key, log_odds_update);
            }
        }
    }

    // Sequential phase: merge all updates into tree
    for (const auto& batch : thread_batches) {
        for (size_t i = 0; i < batch.keys.size(); ++i) {
            const auto& key = batch.keys[i];
            double log_odds_update = batch.log_odds_updates[i];

            // Adaptive protection (unidirectional: Free → Occupied only)
            double final_log_odds = log_odds_update;
            if (adaptive_update_ && log_odds_update > 0.0) {
                // Check current probability BEFORE update
                octomap::OcTreeNode* node = tree_->search(key);
                double current_prob = node ? node->getOccupancy() : 0.5;

                // Apply protection if needed
                if (current_prob <= adaptive_threshold_) {
                    double update_scale = (current_prob / adaptive_threshold_) * adaptive_max_ratio_;
                    final_log_odds = log_odds_update * update_scale;
                }
            }
            // Single update with final log_odds
            tree_->updateNode(key, static_cast<float>(final_log_odds), false);
        }
    }

#else
    // Single-threaded fallback
    for (size_t i = 0; i < num_points; ++i) {
        const Eigen::Vector3d& pt = points[i];
        octomap::point3d endpoint(pt.x(), pt.y(), pt.z());

        double log_odds_update = log_odds[i];

        octomap::OcTreeKey key;
        if (tree_->coordToKeyChecked(endpoint, key)) {
            // Adaptive protection (unidirectional: Free → Occupied only)
            double final_log_odds = log_odds_update;
            if (adaptive_update_ && log_odds_update > 0.0) {
                // Check current probability BEFORE update
                octomap::OcTreeNode* node = tree_->search(key);
                double current_prob = node ? node->getOccupancy() : 0.5;

                // Apply protection if needed
                if (current_prob <= adaptive_threshold_) {
                    double update_scale = (current_prob / adaptive_threshold_) * adaptive_max_ratio_;
                    final_log_odds = log_odds_update * update_scale;
                }
            }
            // Single update with final log_odds
            tree_->updateNode(key, static_cast<float>(final_log_odds), false);
        }
    }
#endif
}

// Pybind11 module definition
PYBIND11_MODULE(octree_mapping, m) {
    m.doc() = "High-performance Octree mapping using OctoMap library";

    // MapStats struct (P3.2 profiling)
    py::class_<MapStats>(m, "MapStats")
        .def(py::init<>())
        .def_readwrite("num_nodes", &MapStats::num_nodes, "Total number of nodes")
        .def_readwrite("num_leaf_nodes", &MapStats::num_leaf_nodes, "Number of leaf nodes")
        .def_readwrite("memory_mb", &MapStats::memory_mb, "Memory usage (MB)");

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
        .def("insert_point_cloud_with_intensity_and_logodds",
             &OctreeMapping::insert_point_cloud_with_intensity_and_logodds,
             py::arg("points"),
             py::arg("intensities"),
             py::arg("log_odds"),
             py::arg("sensor_origin"),
             "Batch insert point cloud with intensity and pre-computed log-odds\n\n"
             "Args:\n"
             "    points: Nx3 NumPy array of [x, y, z] coordinates\n"
             "    intensities: N-length array of intensity values (0-255)\n"
             "    log_odds: N-length array of pre-computed log-odds for free space\n"
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
             "Set probability clamping limits (prevents extreme values)")
        .def("set_adaptive_params",
             &OctreeMapping::set_adaptive_params,
             py::arg("enable"),
             py::arg("threshold"),
             py::arg("max_ratio"),
             "Set adaptive update parameters (unidirectional Free -> Occupied protection)")
        .def("set_update_method",
             &OctreeMapping::set_update_method,
             py::arg("method"),
             "Set update method (0=LOG_ODDS, 1=WEIGHTED_AVG, 2=IWLO)")
        .def("set_intensity_params",
             &OctreeMapping::set_intensity_params,
             py::arg("threshold"),
             py::arg("max_val"),
             "Set intensity parameters for WEIGHTED_AVG and IWLO")
        .def("set_iwlo_params",
             &OctreeMapping::set_iwlo_params,
             py::arg("sharpness"),
             py::arg("decay_rate"),
             py::arg("min_alpha"),
             py::arg("L_min"),
             py::arg("L_max"),
             "Set IWLO-specific parameters")
        .def("get_map_stats",
             &OctreeMapping::get_map_stats,
             "Get map statistics (P3.2 profiling)")
        .def("serialize_to_binary",
             &OctreeMapping::serialize_to_binary,
             "Serialize OcTree to binary data for octomap_msgs");
}
