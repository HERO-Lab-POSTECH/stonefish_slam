#include "ray_processor.h"
#include "octree_mapping.h"
#include <iostream>
#include <algorithm>
#include <unordered_map>
#include <limits>

#ifdef _OPENMP
#include <omp.h>
#endif

// Constructor
RayProcessor::RayProcessor(
    OctreeMapping* octree,
    const RayProcessorConfig& config
)
    : octree_(octree),
      config_(config),
      half_aperture_(config.vertical_aperture / 2.0)
{
    if (!octree_) {
        throw std::invalid_argument("OctreeMapping pointer cannot be null");
    }

#ifdef _OPENMP
    std::cout << "[RayProcessor] OpenMP enabled: " << omp_get_max_threads()
              << " threads available" << std::endl;
#else
    std::cout << "[RayProcessor] OpenMP not available, single-threaded" << std::endl;
#endif
}

// Set configuration
void RayProcessor::set_config(const RayProcessorConfig& config) {
    config_ = config;
    half_aperture_ = config_.vertical_aperture / 2.0;
}

// Process entire sonar image
void RayProcessor::process_sonar_image(
    py::array_t<uint8_t> polar_image,
    const Eigen::Matrix4d& T_sonar_to_world
) {
    // Step 1: Extract data while holding GIL
    auto img_buf = polar_image.request();
    if (img_buf.ndim != 2) {
        throw std::runtime_error("Polar image must be 2D array (num_range_bins × num_bearings)");
    }

    int num_range_bins = img_buf.shape[0];
    int num_bearings = img_buf.shape[1];
    const uint8_t* img_ptr = static_cast<const uint8_t*>(img_buf.ptr);

    // Process bearings with step (e.g., every 2nd bearing for performance)
    std::vector<int> bearing_indices;
    for (int b = 0; b < num_bearings; b += config_.bearing_step) {
        bearing_indices.push_back(b);
    }

    // Step 2: Prepare C++ buffers for each thread
    int num_threads = 1;
#ifdef _OPENMP
    num_threads = omp_get_max_threads();
#endif
    std::vector<std::vector<VoxelUpdate>> thread_updates(num_threads);

    // Pre-allocate reasonable capacity per thread (estimate: ~1000 voxels per ray)
    for (auto& updates : thread_updates) {
        updates.reserve(bearing_indices.size() * 1000 / num_threads);
    }

    Eigen::Vector3d sonar_origin_world = T_sonar_to_world.block<3, 1>(0, 3);

    // Step 3: Release GIL and process in parallel (pure C++ operations)
    {
        py::gil_scoped_release release;

        // Parallel processing across bearings (each bearing is independent)
#ifdef _OPENMP
        #pragma omp parallel for schedule(dynamic)
#endif
        for (size_t i = 0; i < bearing_indices.size(); ++i) {
            int tid = 0;
#ifdef _OPENMP
            tid = omp_get_thread_num();
#endif
            std::vector<VoxelUpdate>& local_updates = thread_updates[tid];

            int b_idx = bearing_indices[i];

            // Extract intensity profile for this bearing (column-major access)
            std::vector<uint8_t> intensity_profile(num_range_bins);
            for (int r = 0; r < num_range_bins; ++r) {
                intensity_profile[r] = img_ptr[r * num_bearings + b_idx];
            }

            // Process this ray (collect voxel updates in local_updates)
            process_single_ray_internal(b_idx, num_bearings, intensity_profile,
                                       T_sonar_to_world, sonar_origin_world,
                                       local_updates);
        }
    }  // GIL automatically reacquired here

    // Step 4: Merge all thread results and update octree (with GIL)
    size_t total_updates = 0;
    for (const auto& updates : thread_updates) {
        total_updates += updates.size();
    }

    if (total_updates > 0) {
        // Merge all updates into single batch
        std::vector<VoxelUpdate> all_updates;
        all_updates.reserve(total_updates);

        for (const auto& updates : thread_updates) {
            all_updates.insert(all_updates.end(), updates.begin(), updates.end());
        }

        // Convert to NumPy arrays and insert
        std::vector<py::ssize_t> points_shape = {static_cast<py::ssize_t>(total_updates), 3};
        py::array_t<double> points_np(points_shape);
        py::array_t<double> log_odds_np(static_cast<py::ssize_t>(total_updates));
        py::array_t<double> origin_np(static_cast<py::ssize_t>(3));

        auto points_buf = points_np.request();
        auto log_odds_buf = log_odds_np.request();
        auto origin_buf = origin_np.request();

        double* points_ptr = static_cast<double*>(points_buf.ptr);
        double* log_odds_ptr = static_cast<double*>(log_odds_buf.ptr);
        double* origin_ptr = static_cast<double*>(origin_buf.ptr);

        // Fill data
        for (size_t i = 0; i < total_updates; ++i) {
            points_ptr[i * 3 + 0] = all_updates[i].x;
            points_ptr[i * 3 + 1] = all_updates[i].y;
            points_ptr[i * 3 + 2] = all_updates[i].z;
            log_odds_ptr[i] = all_updates[i].log_odds;
        }

        origin_ptr[0] = sonar_origin_world[0];
        origin_ptr[1] = sonar_origin_world[1];
        origin_ptr[2] = sonar_origin_world[2];

        // Single batch insert to octree
        octree_->insert_point_cloud(points_np, log_odds_np, origin_np);
    }
}

// Internal version: collect voxel updates in C++ buffer (GIL-free, OpenMP-safe)
void RayProcessor::process_single_ray_internal(
    int bearing_idx,
    int num_bearings,
    const std::vector<uint8_t>& intensity_profile,
    const Eigen::Matrix4d& T_sonar_to_world,
    const Eigen::Vector3d& sonar_origin_world,
    std::vector<VoxelUpdate>& voxel_updates
) {
    // 1. Find first/last hit
    int first_hit_idx = find_first_hit(intensity_profile);

    // No hit found: skip this ray (no map update)
    if (first_hit_idx < 0) {
        return;
    }

    int last_hit_idx = find_last_hit(intensity_profile);

    // 2. Free space processing (DDA traversal to first hit)
    if (first_hit_idx > 0) {
        // Compute bearing angle (use num_bearings, not intensity_profile.size())
        double bearing_angle = compute_bearing_angle(bearing_idx, num_bearings);

        // Horizontal ray direction (in sonar frame)
        Eigen::Vector3d ray_direction_sonar = compute_ray_direction(bearing_angle);

        // Transform to world frame
        Eigen::Vector3d ray_direction_world = T_sonar_to_world.block<3, 3>(0, 0) * ray_direction_sonar;
        ray_direction_world.normalize();

        // Range to first hit (FLS convention: row 0 = far, row max = near)
        double range_to_first_hit = config_.max_range - (first_hit_idx - 1) * config_.range_resolution;
        if (range_to_first_hit < config_.min_range) {
            range_to_first_hit = config_.min_range;
        }

        // Compute vertical steps for free space (adaptive based on mid-range)
        double mid_range = config_.max_range - (first_hit_idx / 2.0) * config_.range_resolution;
        int num_vertical_steps = compute_num_vertical_steps(mid_range, config_.free_vertical_factor);

        // Process free space with vertical fan (internal DDA traversal)
        for (int v_step = -num_vertical_steps; v_step <= num_vertical_steps; ++v_step) {
            // Compute vertical angle
            double vertical_angle = compute_vertical_angle(v_step, num_vertical_steps);

            // Rotate ray direction by vertical angle (simple pitch rotation)
            double cos_v = std::cos(vertical_angle);
            double sin_v = std::sin(vertical_angle);
            Eigen::Vector3d tilted_direction_world(
                ray_direction_world[0] * cos_v - ray_direction_world[2] * sin_v,
                ray_direction_world[1],
                ray_direction_world[0] * sin_v + ray_direction_world[2] * cos_v
            );
            tilted_direction_world.normalize();

            // End point for this vertical angle
            Eigen::Vector3d end_point = sonar_origin_world + tilted_direction_world * range_to_first_hit;

            // DDA traversal (pure C++)
            std::vector<Eigen::Vector3d> voxels = traverse_ray_dda(sonar_origin_world, end_point, 500);

            // Add to voxel updates with range weighting
            for (const auto& voxel : voxels) {
                // Compute range from sonar origin
                double range = (voxel - sonar_origin_world).norm();

                // Range weighting
                double log_odds_update = config_.log_odds_free;
                if (config_.use_range_weighting) {
                    double range_weight = compute_range_weight(range);
                    log_odds_update *= range_weight;
                }

                voxel_updates.push_back({voxel.x(), voxel.y(), voxel.z(), log_odds_update});
            }
        }
    }

    // 3. Occupied space processing (vertical fan for each high-intensity range bin)
    std::vector<int> hit_indices = extract_hit_indices(intensity_profile, first_hit_idx, last_hit_idx);
    if (!hit_indices.empty()) {
        double bearing_angle = compute_bearing_angle(bearing_idx, num_bearings);
        process_occupied_voxels_internal(hit_indices, bearing_angle, T_sonar_to_world,
                                        sonar_origin_world, voxel_updates);
    }
}

// Legacy version: for external Python calls (maintains API compatibility)
void RayProcessor::process_single_ray(
    int bearing_idx,
    int num_bearings,
    const std::vector<uint8_t>& intensity_profile,
    const Eigen::Matrix4d& T_sonar_to_world
) {
    // Use internal version with temporary buffer, then insert to octree
    std::vector<VoxelUpdate> voxel_updates;
    voxel_updates.reserve(1000);  // Typical ray size

    Eigen::Vector3d sonar_origin_world = T_sonar_to_world.block<3, 1>(0, 3);

    process_single_ray_internal(bearing_idx, num_bearings, intensity_profile,
                                T_sonar_to_world, sonar_origin_world, voxel_updates);

    // Convert to NumPy and insert to octree
    if (!voxel_updates.empty()) {
        std::vector<py::ssize_t> points_shape = {static_cast<py::ssize_t>(voxel_updates.size()), 3};
        py::array_t<double> points_np(points_shape);
        py::array_t<double> log_odds_np(static_cast<py::ssize_t>(voxel_updates.size()));
        py::array_t<double> origin_np(static_cast<py::ssize_t>(3));

        auto points_buf = points_np.request();
        auto log_odds_buf = log_odds_np.request();
        auto origin_buf = origin_np.request();

        double* points_ptr = static_cast<double*>(points_buf.ptr);
        double* log_odds_ptr = static_cast<double*>(log_odds_buf.ptr);
        double* origin_ptr = static_cast<double*>(origin_buf.ptr);

        for (size_t i = 0; i < voxel_updates.size(); ++i) {
            points_ptr[i * 3 + 0] = voxel_updates[i].x;
            points_ptr[i * 3 + 1] = voxel_updates[i].y;
            points_ptr[i * 3 + 2] = voxel_updates[i].z;
            log_odds_ptr[i] = voxel_updates[i].log_odds;
        }

        origin_ptr[0] = sonar_origin_world[0];
        origin_ptr[1] = sonar_origin_world[1];
        origin_ptr[2] = sonar_origin_world[2];

        octree_->insert_point_cloud(points_np, log_odds_np, origin_np);
    }
}

// Internal version: collect occupied voxels in C++ buffer (GIL-free, OpenMP-safe)
void RayProcessor::process_occupied_voxels_internal(
    const std::vector<int>& hit_indices,
    double bearing_angle,
    const Eigen::Matrix4d& T_sonar_to_world,
    const Eigen::Vector3d& sonar_origin_world,
    std::vector<VoxelUpdate>& voxel_updates
) {
    // Process each range bin with high intensity
    for (int r_idx : hit_indices) {
        // Calculate range (FLS convention: row 0 = far, row max = near)
        double range_m = config_.max_range - r_idx * config_.range_resolution;

        // Skip out-of-range
        if (range_m > config_.max_range || range_m <= config_.min_range) {
            continue;
        }

        // Compute number of vertical steps (adaptive based on range)
        int num_vertical_steps = compute_num_vertical_steps(range_m, config_.occupied_vertical_factor);

        // Total number of vertical points
        int num_vertical_points = 2 * num_vertical_steps + 1;

        // Batch create all vertical points in sonar frame (Eigen SIMD optimization)
        Eigen::Matrix3Xd points_sonar(3, num_vertical_points);

        for (int v_step = -num_vertical_steps; v_step <= num_vertical_steps; ++v_step) {
            int col_idx = v_step + num_vertical_steps;

            // Compute vertical angle
            double vertical_angle = compute_vertical_angle(v_step, num_vertical_steps);

            // Sonar frame coordinates (FRD: X=forward, Y=right, Z=down)
            // Bearing: 0=forward, positive=right, negative=left
            double x_sonar = range_m * std::cos(vertical_angle) * std::cos(bearing_angle);
            double y_sonar = range_m * std::cos(vertical_angle) * std::sin(bearing_angle);
            double z_sonar = range_m * std::sin(vertical_angle);

            points_sonar.col(col_idx) << x_sonar, y_sonar, z_sonar;
        }

        // Batch transform to world frame (Eigen SIMD optimization)
        Eigen::Matrix3Xd points_world =
            T_sonar_to_world.block<3, 3>(0, 0) * points_sonar +
            sonar_origin_world.replicate(1, num_vertical_points);

        // Compute base log-odds update with range weighting
        double base_log_odds = config_.log_odds_occupied;
        if (config_.use_range_weighting) {
            double range_weight = compute_range_weight(range_m);
            base_log_odds *= range_weight;
        }

        // Fill voxel updates with Gaussian weighting if enabled
        for (int v_step = -num_vertical_steps; v_step <= num_vertical_steps; ++v_step) {
            int idx = v_step + num_vertical_steps;

            // Compute log-odds with optional Gaussian weighting
            double log_odds_update = base_log_odds;
            if (config_.enable_gaussian_weighting && num_vertical_steps > 0) {
                double vertical_angle = compute_vertical_angle(v_step, num_vertical_steps);
                double normalized_angle = vertical_angle / half_aperture_;
                double gaussian_weight = std::exp(-0.5 * std::pow(normalized_angle * config_.gaussian_sigma_factor, 2));
                log_odds_update *= gaussian_weight;
            }

            voxel_updates.push_back({
                points_world(0, idx),
                points_world(1, idx),
                points_world(2, idx),
                log_odds_update
            });
        }
    }
}

// Legacy version: for external Python calls (maintains API compatibility)
void RayProcessor::process_occupied_voxels(
    const std::vector<int>& hit_indices,
    double bearing_angle,
    const Eigen::Matrix4d& T_sonar_to_world
) {
    std::vector<VoxelUpdate> voxel_updates;
    Eigen::Vector3d sonar_origin_world = T_sonar_to_world.block<3, 1>(0, 3);

    process_occupied_voxels_internal(hit_indices, bearing_angle, T_sonar_to_world,
                                     sonar_origin_world, voxel_updates);

    // Convert to NumPy and insert to octree
    if (!voxel_updates.empty()) {
        std::vector<py::ssize_t> points_shape = {static_cast<py::ssize_t>(voxel_updates.size()), 3};
        py::array_t<double> points_np(points_shape);
        py::array_t<double> log_odds_np(static_cast<py::ssize_t>(voxel_updates.size()));
        py::array_t<double> origin_np(static_cast<py::ssize_t>(3));

        auto points_buf = points_np.request();
        auto log_odds_buf = log_odds_np.request();
        auto origin_buf = origin_np.request();

        double* points_ptr = static_cast<double*>(points_buf.ptr);
        double* log_odds_ptr = static_cast<double*>(log_odds_buf.ptr);
        double* origin_ptr = static_cast<double*>(origin_buf.ptr);

        for (size_t i = 0; i < voxel_updates.size(); ++i) {
            points_ptr[i * 3 + 0] = voxel_updates[i].x;
            points_ptr[i * 3 + 1] = voxel_updates[i].y;
            points_ptr[i * 3 + 2] = voxel_updates[i].z;
            log_odds_ptr[i] = voxel_updates[i].log_odds;
        }

        origin_ptr[0] = sonar_origin_world[0];
        origin_ptr[1] = sonar_origin_world[1];
        origin_ptr[2] = sonar_origin_world[2];

        octree_->insert_point_cloud(points_np, log_odds_np, origin_np);
    }
}

// Find first hit
int RayProcessor::find_first_hit(const std::vector<uint8_t>& intensity_profile) const {
    for (size_t i = 0; i < intensity_profile.size(); ++i) {
        if (intensity_profile[i] > config_.intensity_threshold) {
            return static_cast<int>(i);
        }
    }
    return -1;  // No hit found
}

// Find last hit
int RayProcessor::find_last_hit(const std::vector<uint8_t>& intensity_profile) const {
    for (int i = static_cast<int>(intensity_profile.size()) - 1; i >= 0; --i) {
        if (intensity_profile[i] > config_.intensity_threshold) {
            return i;
        }
    }
    return -1;  // No hit found
}

// Extract all hit indices
std::vector<int> RayProcessor::extract_hit_indices(
    const std::vector<uint8_t>& intensity_profile,
    int start_idx,
    int end_idx
) const {
    std::vector<int> hit_indices;
    hit_indices.reserve(end_idx - start_idx + 1);

    for (int i = start_idx; i <= end_idx && i < static_cast<int>(intensity_profile.size()); ++i) {
        if (intensity_profile[i] > config_.intensity_threshold) {
            hit_indices.push_back(i);
        }
    }

    return hit_indices;
}

// Compute range weight
double RayProcessor::compute_range_weight(double range_m) const {
    // Exponential decay: w(r) = exp(-λ × r / r_max)
    // Python: mapping_3d.py lines 756-758
    return std::exp(-config_.lambda_decay * range_m / config_.max_range);
}

// Compute vertical angle
double RayProcessor::compute_vertical_angle(int v_step, int num_vertical_steps) const {
    // Linear interpolation within vertical aperture
    // Python: mapping_3d.py lines 974-976
    if (num_vertical_steps == 0) {
        return 0.0;
    }
    return (static_cast<double>(v_step) / num_vertical_steps) * half_aperture_;
}

// Compute bearing angle
double RayProcessor::compute_bearing_angle(int bearing_idx, int num_bearings) const {
    // Match Python's np.linspace(-fov/2, +fov/2, num_bearings)
    // np.linspace includes endpoints, so there are (num_bearings - 1) intervals
    // Example: num_bearings=512, bearing_idx=0 → -65°, bearing_idx=511 → +65°
    double fov_rad = config_.horizontal_fov * M_PI / 180.0;
    double start_angle = -fov_rad / 2.0;
    double step = fov_rad / (num_bearings - 1);
    return start_angle + bearing_idx * step;
}

// Compute ray direction
Eigen::Vector3d RayProcessor::compute_ray_direction(double bearing_angle) const {
    // Sonar frame: X=forward, Y=right, Z=down (FRD)
    // Bearing: 0=forward, positive=right, negative=left
    return Eigen::Vector3d(
        std::cos(bearing_angle),  // X component
        std::sin(bearing_angle),  // Y component
        0.0                       // Z component (horizontal ray)
    );
}

// Compute number of vertical steps
int RayProcessor::compute_num_vertical_steps(double range_m, double vertical_factor) const {
    // Adaptive sampling based on vertical spread at this range
    // vertical_spread = range × tan(half_aperture)
    // num_steps = max(min_steps, vertical_spread / (voxel_size × factor))

    double vertical_spread = range_m * std::tan(half_aperture_);
    int num_steps = static_cast<int>(vertical_spread / (config_.voxel_resolution * vertical_factor));

    // Minimum steps to ensure coverage
    int min_steps = (vertical_factor < 5.0) ? 2 : 1;  // Occupied: min 2, Free: min 1

    return std::max(min_steps, num_steps);
}

// Internal DDA voxel traversal (simplified version)
std::vector<Eigen::Vector3d> RayProcessor::traverse_ray_dda(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    int max_voxels
) const {
    std::vector<Eigen::Vector3d> voxel_centers;

    // Input validation
    if (max_voxels <= 0 || !start.allFinite() || !end.allFinite()) {
        return voxel_centers;
    }

    // Calculate ray direction and length
    Eigen::Vector3d ray = end - start;
    double ray_length = ray.norm();

    if (ray_length < 1e-10) {
        voxel_centers.push_back(start);
        return voxel_centers;
    }

    ray.normalize();

    // Get starting and ending voxel keys
    std::array<int, 3> current_voxel = world_to_voxel_key(start);
    std::array<int, 3> end_voxel = world_to_voxel_key(end);

    // Add starting voxel center
    Eigen::Vector3d voxel_center(
        current_voxel[0] * config_.voxel_resolution,
        current_voxel[1] * config_.voxel_resolution,
        current_voxel[2] * config_.voxel_resolution
    );
    voxel_centers.push_back(voxel_center);

    // Calculate step direction for each axis
    std::array<int, 3> step;
    for (int i = 0; i < 3; ++i) {
        step[i] = (ray[i] > 0) ? 1 : ((ray[i] < 0) ? -1 : 0);
    }

    // Calculate tMax and tDelta for DDA algorithm
    Eigen::Vector3d tMax, tDelta;

    for (int i = 0; i < 3; ++i) {
        if (std::abs(ray[i]) < 1e-10) {
            tMax[i] = std::numeric_limits<double>::max();
            tDelta[i] = std::numeric_limits<double>::max();
        } else {
            double next_boundary = (step[i] > 0) ?
                                   (current_voxel[i] + 1) * config_.voxel_resolution :
                                   current_voxel[i] * config_.voxel_resolution;
            tMax[i] = (next_boundary - start[i]) / ray[i];
            tDelta[i] = config_.voxel_resolution / std::abs(ray[i]);
        }
    }

    // DDA main loop
    while (voxel_centers.size() < static_cast<size_t>(max_voxels)) {
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

        // Add new voxel center
        Eigen::Vector3d new_center(
            current_voxel[0] * config_.voxel_resolution,
            current_voxel[1] * config_.voxel_resolution,
            current_voxel[2] * config_.voxel_resolution
        );
        voxel_centers.push_back(new_center);
    }

    return voxel_centers;
}

// Convert world coordinates to voxel key
std::array<int, 3> RayProcessor::world_to_voxel_key(const Eigen::Vector3d& point) const {
    return {
        static_cast<int>(std::floor(point.x() / config_.voxel_resolution)),
        static_cast<int>(std::floor(point.y() / config_.voxel_resolution)),
        static_cast<int>(std::floor(point.z() / config_.voxel_resolution))
    };
}

// Pybind11 module definition
PYBIND11_MODULE(ray_processor, m) {
    m.doc() = "High-performance sonar ray processor for 3D mapping";

    // RayProcessorConfig struct
    py::class_<RayProcessorConfig>(m, "RayProcessorConfig")
        .def(py::init<>())
        .def_readwrite("max_range", &RayProcessorConfig::max_range)
        .def_readwrite("min_range", &RayProcessorConfig::min_range)
        .def_readwrite("range_resolution", &RayProcessorConfig::range_resolution)
        .def_readwrite("vertical_aperture", &RayProcessorConfig::vertical_aperture)
        .def_readwrite("horizontal_fov", &RayProcessorConfig::horizontal_fov)
        .def_readwrite("bearing_resolution", &RayProcessorConfig::bearing_resolution)
        .def_readwrite("free_vertical_factor", &RayProcessorConfig::free_vertical_factor)
        .def_readwrite("occupied_vertical_factor", &RayProcessorConfig::occupied_vertical_factor)
        .def_readwrite("log_odds_occupied", &RayProcessorConfig::log_odds_occupied)
        .def_readwrite("log_odds_free", &RayProcessorConfig::log_odds_free)
        .def_readwrite("use_range_weighting", &RayProcessorConfig::use_range_weighting)
        .def_readwrite("lambda_decay", &RayProcessorConfig::lambda_decay)
        .def_readwrite("enable_gaussian_weighting", &RayProcessorConfig::enable_gaussian_weighting)
        .def_readwrite("gaussian_sigma_factor", &RayProcessorConfig::gaussian_sigma_factor)
        .def_readwrite("voxel_resolution", &RayProcessorConfig::voxel_resolution)
        .def_readwrite("bearing_step", &RayProcessorConfig::bearing_step)
        .def_readwrite("intensity_threshold", &RayProcessorConfig::intensity_threshold);

    // RayProcessor class
    py::class_<RayProcessor>(m, "RayProcessor")
        .def(py::init<OctreeMapping*, const RayProcessorConfig&>(),
             py::arg("octree"),
             py::arg("config"),
             "Initialize ray processor with octree and configuration")
        .def("process_sonar_image",
             &RayProcessor::process_sonar_image,
             py::arg("polar_image"),
             py::arg("T_sonar_to_world"),
             "Process entire sonar polar image and update octree\n\n"
             "Args:\n"
             "    polar_image: 2D NumPy array (num_range_bins × num_bearings), uint8\n"
             "    T_sonar_to_world: 4×4 transformation matrix (sonar → world frame)")
        .def("set_config",
             &RayProcessor::set_config,
             py::arg("config"),
             "Update configuration parameters")
        .def("get_config",
             &RayProcessor::get_config,
             "Get current configuration");
}
