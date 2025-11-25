#include "ray_processor.h"
#include "octree_mapping.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <unordered_map>

#ifdef _OPENMP
#include <omp.h>
#endif

// Hash function for Eigen::Vector3i (used for voxel keys in unordered_map)
namespace std {
    template<>
    struct hash<Eigen::Vector3i> {
        std::size_t operator()(const Eigen::Vector3i& key) const {
            std::size_t h1 = std::hash<int>{}(key.x());
            std::size_t h2 = std::hash<int>{}(key.y());
            std::size_t h3 = std::hash<int>{}(key.z());
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };
}

// VoxelKey for deduplication (sorted vector approach)
struct VoxelKey {
    int x, y, z;  // Voxel grid coordinates (integer)

    VoxelKey(double fx, double fy, double fz, double resolution) {
        // Convert floating-point coordinates to integer grid (voxel center index)
        x = static_cast<int>(std::floor(fx / resolution));
        y = static_cast<int>(std::floor(fy / resolution));
        z = static_cast<int>(std::floor(fz / resolution));
    }

    // Comparison for sorting (x → y → z order)
    bool operator<(const VoxelKey& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

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

    // Compute first-hit map for shadow validation (while holding GIL)
    std::vector<double> first_hit_map = compute_first_hit_map(polar_image);

    // Compute inverse transform for shadow validation
    Eigen::Matrix4d T_world_to_sonar = T_sonar_to_world.inverse();

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
                                       T_sonar_to_world, T_world_to_sonar,
                                       sonar_origin_world, first_hit_map,
                                       local_updates);
        }
    }  // GIL automatically reacquired here

    // Step 4: Merge all thread results and update octree (with GIL)
    size_t total_updates = 0;
    for (const auto& updates : thread_updates) {
        total_updates += updates.size();
    }

    if (total_updates > 0) {
        // Deduplicate voxel updates using sorted vector approach
        // VoxelUpdate structure for sorting
        struct VoxelUpdateSorted {
            VoxelKey key;
            double log_odds;

            bool operator<(const VoxelUpdateSorted& other) const {
                return key < other.key;
            }
        };

        // Collect all updates
        std::vector<VoxelUpdateSorted> all_updates;
        all_updates.reserve(total_updates);

        for (const auto& updates : thread_updates) {
            for (const auto& update : updates) {
                VoxelKey key(update.x, update.y, update.z, config_.voxel_resolution);
                all_updates.push_back({key, update.log_odds});
            }
        }

        // Sort by key (cache-friendly sequential access)
        std::sort(all_updates.begin(), all_updates.end());

        // Merge adjacent duplicates
        std::vector<VoxelUpdateSorted> unique_updates;
        unique_updates.reserve(all_updates.size() / 2);  // Rough estimate

        for (size_t i = 0; i < all_updates.size(); ) {
            VoxelKey current_key = all_updates[i].key;
            double sum = 0.0;

            // Accumulate all updates for same voxel
            while (i < all_updates.size() && all_updates[i].key == current_key) {
                sum += all_updates[i].log_odds;
                i++;
            }

            unique_updates.push_back({current_key, sum});
        }

        size_t unique_count = unique_updates.size();

        // NED frame Z filtering: exclude voxels above robot
        // In NED: Z = Down (positive = underwater)
        // Filter condition: voxel_z < robot_z → above robot → exclude
        double robot_z = sonar_origin_world[2];  // Robot Z position in NED frame
        int robot_grid_z = static_cast<int>(std::floor(robot_z / config_.voxel_resolution));

        // First pass: count valid voxels (at or below robot)
        size_t filtered_count = 0;
        for (size_t i = 0; i < unique_count; ++i) {
            // VoxelKey.z is already in grid units, compare directly
            if (unique_updates[i].key.z >= robot_grid_z) {  // Keep voxels at or below robot (NED)
                filtered_count++;
            }
        }

        // Allocate NumPy arrays for filtered voxels only
        std::vector<py::ssize_t> points_shape = {static_cast<py::ssize_t>(filtered_count), 3};
        py::array_t<double> points_np(points_shape);
        py::array_t<double> log_odds_np(static_cast<py::ssize_t>(filtered_count));
        py::array_t<double> origin_np(static_cast<py::ssize_t>(3));

        auto points_buf = points_np.request();
        auto log_odds_buf = log_odds_np.request();
        auto origin_buf = origin_np.request();

        double* points_ptr = static_cast<double*>(points_buf.ptr);
        double* log_odds_ptr = static_cast<double*>(log_odds_buf.ptr);
        double* origin_ptr = static_cast<double*>(origin_buf.ptr);

        // Second pass: fill only valid voxels
        size_t write_idx = 0;
        for (size_t i = 0; i < unique_count; ++i) {
            // VoxelKey is in grid units, compare directly
            if (unique_updates[i].key.z >= robot_grid_z) {  // Keep voxels at or below robot
                const auto& update = unique_updates[i];
                // Convert grid coordinates to world coordinates (voxel center)
                points_ptr[write_idx * 3 + 0] = (update.key.x + 0.5) * config_.voxel_resolution;
                points_ptr[write_idx * 3 + 1] = (update.key.y + 0.5) * config_.voxel_resolution;
                points_ptr[write_idx * 3 + 2] = (update.key.z + 0.5) * config_.voxel_resolution;
                log_odds_ptr[write_idx] = update.log_odds;
                write_idx++;
            }
        }

        std::cout << "Deduplication: " << total_updates << " → " << unique_count
                  << " → " << filtered_count << " voxels (Z-filtered: "
                  << (unique_count - filtered_count) << " above robot)" << std::endl;

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
    const Eigen::Matrix4d& T_world_to_sonar,
    const Eigen::Vector3d& sonar_origin_world,
    const std::vector<double>& first_hit_map,
    std::vector<VoxelUpdate>& voxel_updates
) {
    // 1. Find first hit only (ignore anything after first reflection)
    int first_hit_idx = find_first_hit(intensity_profile);

    // 2. Free space processing (DDA traversal to first hit or max range)
    double range_to_first_hit;
    if (first_hit_idx < 0) {
        // No reflection within max_range → entire measured range is free space
        first_hit_idx = static_cast<int>(intensity_profile.size());
    }
    {
        // Normal case: free space up to first hit
        // Range to first hit (FLS convention: row 0 = far, row max = near)
        // CRITICAL FIX: Use (first_hit_idx + 1) to prevent free space from overlapping
        // with occupied bin. DDA includes end voxel, so free space must end BEFORE
        // occupied bin's far boundary: [max_range - (idx+1)*res, max_range - idx*res]
        range_to_first_hit = config_.max_range - (first_hit_idx + 1) * config_.range_resolution;
        if (range_to_first_hit < config_.min_range) {
            range_to_first_hit = config_.min_range;
        }
    }

    // Always perform free space processing (even if no hit found)
    {
        // Compute bearing angle (use num_bearings, not intensity_profile.size())
        double bearing_angle = compute_bearing_angle(bearing_idx, num_bearings);

        // Horizontal ray direction (in sonar frame)
        Eigen::Vector3d ray_direction_sonar = compute_ray_direction(bearing_angle);

        // Transform to world frame
        Eigen::Vector3d ray_direction_world = T_sonar_to_world.block<3, 3>(0, 0) * ray_direction_sonar;
        ray_direction_world.normalize();

        // Compute vertical steps for free space (full coverage)
        // CRITICAL FIX: Use range_to_first_hit (not mid_range) to match occupied vertical sampling
        // This ensures identical vertical angle distribution between free/occupied updates
        int num_vertical_steps = compute_num_vertical_steps(range_to_first_hit);

        // Safety margin: end 1 voxel before occupied to prevent overlap
        double safe_range = range_to_first_hit - config_.voxel_resolution;
        if (safe_range <= 0.0) {
            return;  // Skip this bearing if too close
        }

        // Pre-compute constants (outside vertical loop for efficiency)
        const double cos_bear = std::cos(bearing_angle);
        const double sin_bear = std::sin(bearing_angle);
        // 실제 bearing 해상도 기반 cone width 계산 (0.5 = 각 bearing 책임 영역만)
        double actual_bearing_resolution = (config_.horizontal_fov * M_PI / 180.0) / (num_bearings - 1);
        const double bearing_half_width = actual_bearing_resolution * config_.bearing_step * 0.5;
        const double cos_half_width = std::cos(bearing_half_width);
        const Eigen::Matrix3d R_world_to_sonar = T_sonar_to_world.block<3, 3>(0, 0).transpose();

        // Process free space with vertical fan (internal DDA traversal)
        // CRITICAL FIX: Use same coordinate calculation as occupied space (sonar frame)
        for (int v_step = -num_vertical_steps; v_step <= num_vertical_steps; ++v_step) {
            // Compute vertical angle
            double vertical_angle = compute_vertical_angle(v_step, num_vertical_steps);

            // Compute end point in sonar frame (same as occupied calculation)
            // This ensures coordinate consistency when sonar is tilted
            // Optimized: pre-computed cos_bear/sin_bear outside loop
            double cos_vert = std::cos(vertical_angle);
            double horizontal_range = safe_range * cos_vert;
            double x_sonar = horizontal_range * cos_bear;
            double y_sonar = horizontal_range * sin_bear;
            double z_sonar = safe_range * std::sin(vertical_angle);
            Eigen::Vector3d end_point_sonar(x_sonar, y_sonar, z_sonar);

            // Transform to world frame (same transform as occupied)
            Eigen::Vector3d end_point = T_sonar_to_world.block<3, 3>(0, 0) * end_point_sonar + sonar_origin_world;

            // DDA traversal (pure C++)
            std::vector<Eigen::Vector3d> voxels = traverse_ray_dda(sonar_origin_world, end_point, 500);

            // End voxel 제외로 occupied 영역 보호
            if (!voxels.empty()) {
                voxels.pop_back();
            }

            // Add to voxel updates with range weighting
            for (const auto& voxel : voxels) {
                // SHADOW VALIDATION: Skip voxels in shadow region
                if (is_voxel_in_shadow(voxel, T_world_to_sonar, first_hit_map, num_bearings)) {
                    continue;  // Voxel is beyond first hit, skip free space update
                }

                // Compute range from sonar origin
                double range = (voxel - sonar_origin_world).norm();

                // CRITICAL FIX: Only update voxels within current bearing's angular cone
                // Optimized: Use cos/sin dot product instead of atan2 for 50% performance gain
                Eigen::Vector3d voxel_in_sonar = R_world_to_sonar * (voxel - sonar_origin_world);

                // Compute horizontal distance (ignore Z for bearing calculation)
                double horiz_dist = std::sqrt(voxel_in_sonar.x() * voxel_in_sonar.x() +
                                               voxel_in_sonar.y() * voxel_in_sonar.y());

                if (horiz_dist < 1e-6) {
                    continue;  // Skip voxels at origin
                }

                // Normalize to get unit direction vector
                double voxel_cos = voxel_in_sonar.x() / horiz_dist;
                double voxel_sin = voxel_in_sonar.y() / horiz_dist;

                // Dot product: cos(angle_diff) = cos(a)*cos(b) + sin(a)*sin(b)
                double cos_diff = voxel_cos * cos_bear + voxel_sin * sin_bear;

                // Check if within cone: cos(angle_diff) > cos(half_width)
                // (larger cos value = smaller angle)
                if (cos_diff < cos_half_width) {
                    continue;  // Voxel outside current bearing's angular cone
                }

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

    // 3. Occupied space processing: all high intensity after first hit
    // After first reflection: only update high intensity (> threshold) as occupied
    // Low intensity regions after first hit = shadow/unknown (NO UPDATE)
    if (first_hit_idx >= 0) {
        std::vector<int> hit_indices;
        for (size_t i = first_hit_idx; i < intensity_profile.size(); ++i) {
            if (intensity_profile[i] > config_.intensity_threshold) {
                hit_indices.push_back(static_cast<int>(i));
            }
            // Low intensity (≤ threshold) after first hit: NO UPDATE (shadow region)
        }

        if (!hit_indices.empty()) {
            double bearing_angle = compute_bearing_angle(bearing_idx, num_bearings);
            process_occupied_voxels_internal(hit_indices, bearing_angle, T_sonar_to_world,
                                            sonar_origin_world, voxel_updates);
        }
    }
}

// Legacy version: for external Python calls (maintains API compatibility)
void RayProcessor::process_single_ray(
    int bearing_idx,
    int num_bearings,
    const std::vector<uint8_t>& intensity_profile,
    const Eigen::Matrix4d& T_sonar_to_world
) {
    // NOTE: Legacy API does NOT support shadow validation
    // For full shadow validation, use process_sonar_image() instead

    // Use internal version with temporary buffer, then insert to octree
    std::vector<VoxelUpdate> voxel_updates;
    voxel_updates.reserve(1000);  // Typical ray size

    Eigen::Vector3d sonar_origin_world = T_sonar_to_world.block<3, 1>(0, 3);
    Eigen::Matrix4d T_world_to_sonar = T_sonar_to_world.inverse();

    // Create dummy first_hit_map (no shadow validation for single ray)
    std::vector<double> first_hit_map(num_bearings, config_.max_range);

    process_single_ray_internal(bearing_idx, num_bearings, intensity_profile,
                                T_sonar_to_world, T_world_to_sonar,
                                sonar_origin_world, first_hit_map, voxel_updates);

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

        // Compute number of vertical steps (full coverage)
        int num_vertical_steps = compute_num_vertical_steps(range_m);

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

    // P3.1 profiling: measure exp() calls
    exp_call_count_.fetch_add(1, std::memory_order_relaxed);

    auto t1 = std::chrono::high_resolution_clock::now();
    double weight = std::exp(-config_.lambda_decay * range_m / config_.max_range);
    auto t2 = std::chrono::high_resolution_clock::now();

    int64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    exp_time_ns_.fetch_add(elapsed_ns, std::memory_order_relaxed);

    return weight;
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

// Compute number of vertical steps (full voxel coverage)
int RayProcessor::compute_num_vertical_steps(double range_m) const {
    // Full voxel coverage: sample every voxel in vertical aperture
    // vertical_spread = range × tan(half_aperture)
    // num_steps = ceil(vertical_spread / voxel_resolution)

    double vertical_spread = range_m * std::tan(half_aperture_);
    int num_steps = static_cast<int>(std::ceil(vertical_spread / config_.voxel_resolution));

    // At least 1 step to ensure coverage
    return std::max(1, num_steps);
}

// Compute first hit range map
std::vector<double> RayProcessor::compute_first_hit_map(
    const py::array_t<uint8_t>& polar_image
) const {
    // Extract image dimensions
    auto img_buf = polar_image.request();
    if (img_buf.ndim != 2) {
        throw std::runtime_error("Polar image must be 2D array (num_range_bins × num_bearings)");
    }

    int num_range_bins = img_buf.shape[0];
    int num_bearings = img_buf.shape[1];
    const uint8_t* img_ptr = static_cast<const uint8_t*>(img_buf.ptr);

    // Initialize with max_range (no hit by default)
    std::vector<double> first_hit_map(num_bearings, config_.max_range);

    // Process each bearing
    for (int b_idx = 0; b_idx < num_bearings; ++b_idx) {
        // Find first pixel above threshold
        for (int r_idx = 0; r_idx < num_range_bins; ++r_idx) {
            uint8_t intensity = img_ptr[r_idx * num_bearings + b_idx];

            if (intensity > config_.intensity_threshold) {
                // Calculate horizontal range (FLS: row 0 = far, row max = near)
                first_hit_map[b_idx] = config_.max_range - r_idx * config_.range_resolution;
                break;
            }
        }
    }

    return first_hit_map;
}

// Check if voxel is in shadow region
bool RayProcessor::is_voxel_in_shadow(
    const Eigen::Vector3d& voxel_world,
    const Eigen::Matrix4d& T_world_to_sonar,
    const std::vector<double>& first_hit_map,
    int num_bearings
) const {
    // Transform voxel to sonar frame
    Eigen::Vector4d voxel_world_homo(voxel_world.x(), voxel_world.y(), voxel_world.z(), 1.0);
    Eigen::Vector4d voxel_sonar = T_world_to_sonar * voxel_world_homo;

    double x_s = voxel_sonar.x();
    double y_s = voxel_sonar.y();
    // double z_s = voxel_sonar.z();  // Not needed for horizontal range

    // Calculate HORIZONTAL range only (ignore Z, matching Python)
    double range_m = std::sqrt(x_s * x_s + y_s * y_s);

    // Calculate bearing angle (horizontal angle)
    double bearing_rad = std::atan2(y_s, x_s);

    // Convert bearing to index
    // bearing_angles range: [-fov/2, +fov/2]
    // Normalize: bearing_rad to [0, 1] relative to FOV
    double fov_rad = config_.horizontal_fov * M_PI / 180.0;
    double bearing_normalized = (bearing_rad + fov_rad / 2.0) / fov_rad;

    // Clamp to valid range [0, 0.9999] to prevent out-of-bounds
    bearing_normalized = std::clamp(bearing_normalized, 0.0, 0.9999);

    int bearing_idx = static_cast<int>(bearing_normalized * num_bearings);

    // Boundary check
    if (bearing_idx < 0 || bearing_idx >= num_bearings) {
        return true;  // Out of FOV, treat as shadow
    }

    // Get first hit range for this bearing
    double first_hit_range = first_hit_map[bearing_idx];

    // Shadow condition: voxel range >= first hit range
    // Add small epsilon to avoid numerical issues (1cm tolerance)
    constexpr double epsilon = 0.01;
    return range_m >= (first_hit_range + epsilon);
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

    // Add starting voxel center (grid index → world center)
    Eigen::Vector3d voxel_center(
        (current_voxel[0] + 0.5) * config_.voxel_resolution,
        (current_voxel[1] + 0.5) * config_.voxel_resolution,
        (current_voxel[2] + 0.5) * config_.voxel_resolution
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

        // Add new voxel center (grid index → world center)
        Eigen::Vector3d new_center(
            (current_voxel[0] + 0.5) * config_.voxel_resolution,
            (current_voxel[1] + 0.5) * config_.voxel_resolution,
            (current_voxel[2] + 0.5) * config_.voxel_resolution
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

// Get ray processing statistics (P3.1 profiling)
RayStats RayProcessor::get_ray_stats() const {
    return {
        exp_call_count_.load(std::memory_order_relaxed),
        exp_time_ns_.load(std::memory_order_relaxed) / 1e6  // ns → ms
    };
}

// Reset ray processing statistics
void RayProcessor::reset_ray_stats() {
    exp_call_count_.store(0, std::memory_order_relaxed);
    exp_time_ns_.store(0, std::memory_order_relaxed);
}

// Pybind11 module definition
PYBIND11_MODULE(ray_processor, m) {
    m.doc() = "High-performance sonar ray processor for 3D mapping";

    // RayStats struct (P3.1 profiling)
    py::class_<RayStats>(m, "RayStats")
        .def(py::init<>())
        .def_readwrite("exp_calls", &RayStats::exp_calls, "Number of exp() calls")
        .def_readwrite("exp_time_ms", &RayStats::exp_time_ms, "Total exp() time (ms)");

    // RayProcessorConfig struct
    py::class_<RayProcessorConfig>(m, "RayProcessorConfig")
        .def(py::init<>())
        .def_readwrite("max_range", &RayProcessorConfig::max_range)
        .def_readwrite("min_range", &RayProcessorConfig::min_range)
        .def_readwrite("range_resolution", &RayProcessorConfig::range_resolution)
        .def_readwrite("vertical_aperture", &RayProcessorConfig::vertical_aperture)
        .def_readwrite("horizontal_fov", &RayProcessorConfig::horizontal_fov)
        .def_readwrite("bearing_resolution", &RayProcessorConfig::bearing_resolution)
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
        .def("compute_first_hit_map",
             &RayProcessor::compute_first_hit_map,
             py::arg("polar_image"),
             "Compute first hit range for each bearing\n\n"
             "Args:\n"
             "    polar_image: 2D NumPy array (num_range_bins × num_bearings), uint8\n"
             "Returns:\n"
             "    List of first hit ranges (meters) for each bearing")
        .def("is_voxel_in_shadow",
             &RayProcessor::is_voxel_in_shadow,
             py::arg("voxel_world"),
             py::arg("T_world_to_sonar"),
             py::arg("first_hit_map"),
             py::arg("num_bearings"),
             "Check if voxel is in shadow region\n\n"
             "Args:\n"
             "    voxel_world: Voxel position in world frame (3D)\n"
             "    T_world_to_sonar: Inverse transformation matrix (world → sonar)\n"
             "    first_hit_map: List of first hit ranges for all bearings\n"
             "    num_bearings: Total number of bearings\n"
             "Returns:\n"
             "    True if voxel is in shadow (should skip update)")
        .def("set_config",
             &RayProcessor::set_config,
             py::arg("config"),
             "Update configuration parameters")
        .def("get_config",
             &RayProcessor::get_config,
             "Get current configuration")
        .def("get_ray_stats",
             &RayProcessor::get_ray_stats,
             "Get ray processing statistics (P3.1 profiling)")
        .def("reset_ray_stats",
             &RayProcessor::reset_ray_stats,
             "Reset ray processing statistics");
}
