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
      half_aperture_(config.vertical_fov / 2.0)
{
    if (!octree_) {
        throw std::invalid_argument("OctreeMapping pointer cannot be null");
    }
}

// Set configuration
void RayProcessor::set_config(const RayProcessorConfig& config) {
    config_ = config;
    half_aperture_ = config_.vertical_fov / 2.0;
}

// Process entire sonar image
void RayProcessor::process_sonar_image(
    py::array_t<uint8_t> polar_image,
    const Eigen::Matrix4d& T_sonar_to_world
) {
    // Step 1: Extract data while holding GIL
    auto img_buf = polar_image.request();
    if (img_buf.ndim != 2) {
        throw std::runtime_error("Polar image must be 2D array (num_range_bins × num_beams)");
    }

    int num_range_bins = img_buf.shape[0];
    int num_beams = img_buf.shape[1];
    const uint8_t* img_ptr = static_cast<const uint8_t*>(img_buf.ptr);

    // Compute first-hit map for shadow validation (while holding GIL)
    std::vector<double> first_hit_map = compute_first_hit_map(polar_image);

    // Process bearings with step (e.g., every 2nd bearing for performance)
    std::vector<int> bearing_indices;
    for (int b = 0; b < num_beams; b += config_.bearing_step) {
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

            // Process this ray (collect voxel updates in local_updates)
            // Pass polar image pointer directly for per-voxel pixel lookup
            process_single_ray_internal(b_idx, num_beams, img_ptr, num_range_bins,
                                       T_sonar_to_world, sonar_origin_world,
                                       first_hit_map, local_updates);
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
            double intensity_avg;  // Average intensity for this voxel

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
                all_updates.push_back({key, update.log_odds, update.intensity});
            }
        }

        // Sort by key (cache-friendly sequential access)
        std::sort(all_updates.begin(), all_updates.end());

        // Merge adjacent duplicates
        std::vector<VoxelUpdateSorted> unique_updates;
        unique_updates.reserve(all_updates.size() / 2);  // Rough estimate

        for (size_t i = 0; i < all_updates.size(); ) {
            VoxelKey current_key = all_updates[i].key;
            double free_sum = 0.0;
            double occupied_sum = 0.0;
            double max_occupied_intensity = 0.0;
            bool has_occupied = false;

            // Separate free and occupied updates for same voxel
            while (i < all_updates.size() && all_updates[i].key == current_key) {
                double intensity = all_updates[i].intensity_avg;

                if (intensity >= config_.intensity_threshold) {
                    // Occupied update (high intensity)
                    occupied_sum += all_updates[i].log_odds;
                    max_occupied_intensity = std::max(max_occupied_intensity, intensity);
                    has_occupied = true;
                } else {
                    // Free update (low intensity)
                    free_sum += all_updates[i].log_odds;
                }
                i++;
            }

            // Merge log_odds: Bayesian update uses additive log-odds
            // L_total = L_free + L_occupied (log-space addition = probability multiplication)
            double final_log_odds = free_sum + occupied_sum;
            double final_intensity = has_occupied ? max_occupied_intensity : 1.0;
            unique_updates.push_back({current_key, final_log_odds, final_intensity});
        }

        size_t unique_count = unique_updates.size();

        // Allocate NumPy arrays for all unique voxels (no Z filtering)
        std::vector<py::ssize_t> points_shape = {static_cast<py::ssize_t>(unique_count), 3};
        py::array_t<double> points_np(points_shape);
        py::array_t<double> origin_np(static_cast<py::ssize_t>(3));

        auto points_buf = points_np.request();
        auto origin_buf = origin_np.request();

        double* points_ptr = static_cast<double*>(points_buf.ptr);
        double* origin_ptr = static_cast<double*>(origin_buf.ptr);

        // Prepare data arrays based on update method
        py::array_t<double> log_odds_np;
        py::array_t<double> intensities_np;
        double* log_odds_ptr = nullptr;
        double* intensities_ptr = nullptr;

        if (config_.update_method == 1 || config_.update_method == 2) {
            // Weighted Average OR IWLO: need both intensity and log_odds arrays
            intensities_np = py::array_t<double>(static_cast<py::ssize_t>(unique_count));
            log_odds_np = py::array_t<double>(static_cast<py::ssize_t>(unique_count));
            auto intensities_buf = intensities_np.request();
            auto log_odds_buf = log_odds_np.request();
            intensities_ptr = static_cast<double*>(intensities_buf.ptr);
            log_odds_ptr = static_cast<double*>(log_odds_buf.ptr);
        } else {
            // Log-odds only: use log_odds_sum directly
            log_odds_np = py::array_t<double>(static_cast<py::ssize_t>(unique_count));
            auto log_odds_buf = log_odds_np.request();
            log_odds_ptr = static_cast<double*>(log_odds_buf.ptr);
        }

        // Fill all unique voxels
        for (size_t i = 0; i < unique_count; ++i) {
            const auto& update = unique_updates[i];
            // Convert grid coordinates to world coordinates (voxel center)
            points_ptr[i * 3 + 0] = (update.key.x + 0.5) * config_.voxel_resolution;
            points_ptr[i * 3 + 1] = (update.key.y + 0.5) * config_.voxel_resolution;
            points_ptr[i * 3 + 2] = (update.key.z + 0.5) * config_.voxel_resolution;

            if (config_.update_method == 1 || config_.update_method == 2) {
                // Weighted Average OR IWLO: use both intensity and log_odds
                intensities_ptr[i] = update.intensity_avg;
                log_odds_ptr[i] = update.log_odds;
            } else {
                // Log-odds only: use log_odds_sum
                log_odds_ptr[i] = update.log_odds;
            }
        }

        origin_ptr[0] = sonar_origin_world[0];
        origin_ptr[1] = sonar_origin_world[1];
        origin_ptr[2] = sonar_origin_world[2];

        // Single batch insert to octree (select API based on update method)
        if (config_.update_method == 1 || config_.update_method == 2) {
            // Weighted Average OR IWLO: use new API with intensity and log-odds
            octree_->insert_point_cloud_with_intensity_and_logodds(
                points_np, intensities_np, log_odds_np, origin_np
            );
        } else {
            // LOG_ODDS ONLY: use log_odds_sum directly
            octree_->insert_point_cloud(points_np, log_odds_np, origin_np);
        }
    }
}

// Internal version: collect voxel updates in C++ buffer (GIL-free, OpenMP-safe)
void RayProcessor::process_single_ray_internal(
    int bearing_idx,
    int num_beams,
    const uint8_t* polar_image,
    int num_range_bins,
    const Eigen::Matrix4d& T_sonar_to_world,
    const Eigen::Vector3d& sonar_origin_world,
    const std::vector<double>& first_hit_map,
    std::vector<VoxelUpdate>& voxel_updates
) {

    // Extract intensity profile for this bearing
    std::vector<uint8_t> intensity_profile(num_range_bins);
    for (int r = 0; r < num_range_bins; ++r) {
        intensity_profile[r] = polar_image[r * num_beams + bearing_idx];
    }

    // Compute bearing angle for this ray
    double bearing_angle = compute_bearing_angle(bearing_idx, num_beams);

    // Pixel-based unified processing: iterate over all range bins
    // For each (range, bearing) pixel:
    //   - Check pixel intensity
    //   - Generate vertical fan
    //   - Update as occupied/free based on pixel value and shadow region

    // Get first hit range for shadow validation
    double first_hit_range = first_hit_map[bearing_idx];

    // Calculate consistent vertical steps based on first_hit_range
    // All pixels in this bearing will use the same vertical fan size
    int max_vertical_steps = compute_num_vertical_steps(first_hit_range);

    // Process all range bins
    for (int r_idx = 0; r_idx < num_range_bins; ++r_idx) {
        uint8_t pixel_intensity = intensity_profile[r_idx];

        // Calculate range (FLS convention: row 0 = far, row max = near)
        double range_m = config_.range_max - r_idx * config_.range_resolution;

        // Skip invalid range
        if (range_m <= config_.range_min || range_m > config_.range_max) {
            continue;
        }

        // Determine pixel status
        bool is_hit = (pixel_intensity > config_.intensity_threshold);
        bool is_shadow = (!is_hit && range_m >= first_hit_range);

        // Skip shadow region (no update)
        if (is_shadow) {
            continue;
        }

        // Use consistent vertical steps based on first_hit_range (not current range)
        int num_vertical_steps = max_vertical_steps;

        // Generate horizontal spread based on first_hit_range for consistency
        int num_horizontal_steps = compute_num_horizontal_steps(first_hit_range, config_.bearing_step, num_beams);

        // Compute bearing spread for horizontal offset calculation
        double bearing_spread_rad = (config_.horizontal_fov * M_PI / 180.0) / num_beams * config_.bearing_step;

        // Compute base log-odds update
        double base_log_odds;
        if (is_hit) {
            // Occupied update (no intensity weighting)
            base_log_odds = config_.log_odds_occupied;
        } else {
            // Free space update
            base_log_odds = config_.log_odds_free;
        }

        // Apply range weighting if enabled
        if (config_.use_range_weighting) {
            base_log_odds *= compute_range_weight(range_m);
        }

        // Process horizontal spread: cover full bearing width [-spread/2, +spread/2]
        // h_step range: [-N, +N] inclusive (2N+1 steps)
        // When N=0, single center ray; when N=1, steps -1,0,+1 (3 rays covering full width)
        for (int h_step = -num_horizontal_steps; h_step <= num_horizontal_steps; ++h_step) {
            // Compute horizontal offset angle
            double horizontal_offset = 0.0;
            if (num_horizontal_steps > 0) {
                // Map h_step [-N, +N] to offset [-spread/2, +spread/2]
                horizontal_offset = (static_cast<double>(h_step) / num_horizontal_steps) * (bearing_spread_rad / 2.0);
            }
            double adjusted_bearing = bearing_angle + horizontal_offset;

            // Process vertical fan
            for (int v_step = -num_vertical_steps; v_step <= num_vertical_steps; ++v_step) {
                // Compute vertical angle
                double vertical_angle = compute_vertical_angle(v_step, num_vertical_steps);

                // Calculate voxel position in sonar frame with adjusted bearing
                double cos_vert = std::cos(vertical_angle);
                double x_sonar = range_m * cos_vert * std::cos(adjusted_bearing);
                double y_sonar = range_m * cos_vert * std::sin(adjusted_bearing);
                double z_sonar = range_m * std::sin(vertical_angle);

                // Transform to world frame
                Eigen::Vector3d point_sonar(x_sonar, y_sonar, z_sonar);
                Eigen::Vector3d point_world = T_sonar_to_world.block<3, 3>(0, 0) * point_sonar + sonar_origin_world;

                // Apply Gaussian weighting for occupied voxels (optional)
                double log_odds_update = base_log_odds;
                if (is_hit && config_.enable_gaussian_weighting && num_vertical_steps > 0) {
                    double normalized_angle = vertical_angle / half_aperture_;
                    double gaussian_weight = std::exp(-0.5 * std::pow(normalized_angle * config_.gaussian_sigma_factor, 2));
                    log_odds_update *= gaussian_weight;
                }

                // Add voxel update
                voxel_updates.push_back({
                    point_world.x(),
                    point_world.y(),
                    point_world.z(),
                    log_odds_update,
                    static_cast<double>(pixel_intensity)
                });
            }
        }
    }
}



// Find first hit in intensity profile (image scan order: far to near)
// Returns the first index with intensity above threshold
int RayProcessor::find_first_hit(const std::vector<uint8_t>& intensity_profile) const {
    for (size_t i = 0; i < intensity_profile.size(); ++i) {
        if (intensity_profile[i] > config_.intensity_threshold) {
            return static_cast<int>(i);
        }
    }
    return -1;  // No hit found
}

// Compute range weight
double RayProcessor::compute_range_weight(double range_m) const {
    // Exponential decay: w(r) = exp(-λ × r / r_max)
    // Python: mapping_3d.py lines 756-758

    // P3.1 profiling: measure exp() calls
    exp_call_count_.fetch_add(1, std::memory_order_relaxed);

    auto t1 = std::chrono::high_resolution_clock::now();
    double weight = std::exp(-config_.lambda_decay * range_m / config_.range_max);
    auto t2 = std::chrono::high_resolution_clock::now();

    int64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    exp_time_ns_.fetch_add(elapsed_ns, std::memory_order_relaxed);

    return weight;
}

// Compute intensity weight using sigmoid function
// w(I) = sigmoid((I - I_mid) / (sharpness * scale))
// High intensity (>127) → weight > 0.5 → stronger occupied update
// Low intensity (<127) → weight < 0.5 → weaker occupied update
double RayProcessor::compute_intensity_weight(uint8_t intensity) const {
    double I_mid = config_.intensity_max / 2.0;  // 127.5
    double scale = config_.intensity_max / 5.0;   // 51.0
    double x = (static_cast<double>(intensity) - I_mid) / (config_.sharpness * scale);
    return 1.0 / (1.0 + std::exp(-x));
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
double RayProcessor::compute_bearing_angle(int bearing_idx, int num_beams) const {
    // Match Python's np.linspace(-fov/2, +fov/2, num_beams)
    // np.linspace includes endpoints, so there are (num_beams - 1) intervals
    // Example: num_beams=512, bearing_idx=0 → -65°, bearing_idx=511 → +65°
    double fov_rad = config_.horizontal_fov * M_PI / 180.0;
    double start_angle = -fov_rad / 2.0;
    double step = fov_rad / (num_beams - 1);
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

// Compute number of horizontal steps for full voxel coverage
int RayProcessor::compute_num_horizontal_steps(double range_m, int bearing_step, int num_beams) const {
    // Each bearing covers angular width: (horizontal_fov / num_beams) * bearing_step
    double bearing_spread_rad = (config_.horizontal_fov * M_PI / 180.0) / num_beams * bearing_step;

    // Total horizontal distance between adjacent processed bearings
    // This is the full arc length that needs to be covered
    double total_horizontal_spread = range_m * bearing_spread_rad;  // arc length ≈ r * θ

    // Number of voxels to cover on each side of center
    // 2x oversampling to ensure all OctoMap grid cells receive at least one sample
    // Without this, floor(coord/res) quantization causes checkerboard gaps
    int num_steps = static_cast<int>(std::ceil(total_horizontal_spread / config_.voxel_resolution));

    // At least 1 to ensure minimum coverage
    return std::max(1, num_steps);
}

// Compute first hit range map
std::vector<double> RayProcessor::compute_first_hit_map(
    const py::array_t<uint8_t>& polar_image
) const {
    // Extract image dimensions
    auto img_buf = polar_image.request();
    if (img_buf.ndim != 2) {
        throw std::runtime_error("Polar image must be 2D array (num_range_bins × num_beams)");
    }

    int num_range_bins = img_buf.shape[0];
    int num_beams = img_buf.shape[1];
    const uint8_t* img_ptr = static_cast<const uint8_t*>(img_buf.ptr);

    // Initialize with range_max (no hit by default)
    std::vector<double> first_hit_map(num_beams, config_.range_max);

    // Process each bearing
    for (int b_idx = 0; b_idx < num_beams; ++b_idx) {
        // Find first hit from near to far (closest object first)
        // FLS convention: row 0 = far, row max = near
        // So we iterate from row max (near) to row 0 (far)
        for (int r_idx = num_range_bins - 1; r_idx >= 0; --r_idx) {
            uint8_t intensity = img_ptr[r_idx * num_beams + b_idx];

            if (intensity > config_.intensity_threshold) {
                // Calculate horizontal range
                first_hit_map[b_idx] = config_.range_max - r_idx * config_.range_resolution;
                break;
            }
        }
    }

    return first_hit_map;
}

// Generate hit map visualization image
py::array_t<uint8_t> RayProcessor::generate_hit_map_visualization(
    const py::array_t<uint8_t>& polar_image
) const {
    // Extract image dimensions
    auto img_buf = polar_image.request();
    if (img_buf.ndim != 2) {
        throw std::runtime_error("Polar image must be 2D array (num_range_bins × num_beams)");
    }

    int num_range_bins = img_buf.shape[0];
    int num_beams = img_buf.shape[1];
    const uint8_t* img_ptr = static_cast<const uint8_t*>(img_buf.ptr);

    // Compute first_hit_map (same as actual voxel update)
    std::vector<double> first_hit_map = compute_first_hit_map(polar_image);

    // Create RGB image (num_range_bins x num_beams x 3)
    py::array_t<uint8_t> vis_image({num_range_bins, num_beams, 3});
    auto vis_buf = vis_image.mutable_unchecked<3>();

    // Process each bearing
    for (int b_idx = 0; b_idx < num_beams; ++b_idx) {
        double first_hit_range = first_hit_map[b_idx];

        // Process each range bin (same logic as process_single_ray_internal)
        for (int r_idx = 0; r_idx < num_range_bins; ++r_idx) {
            uint8_t pixel_intensity = img_ptr[r_idx * num_beams + b_idx];

            // Calculate range (FLS convention: row 0 = far, row max = near)
            double range_m = config_.range_max - r_idx * config_.range_resolution;

            uint8_t r = 0, g = 0, b = 0;

            // Skip invalid range → Black (0, 0, 0)
            if (range_m <= config_.range_min || range_m > config_.range_max) {
                // Keep black
            } else {
                // SAME LOGIC as process_single_ray_internal
                bool is_hit = (pixel_intensity > config_.intensity_threshold);
                bool is_shadow = (!is_hit && range_m >= first_hit_range);

                // Check if this is the first hit
                // First hit is the range bin closest to first_hit_range
                bool is_first_hit = is_hit &&
                    std::abs(range_m - first_hit_range) < config_.range_resolution;

                if (is_first_hit) {
                    // Red: First hit
                    r = 255; g = 0; b = 0;
                } else if (is_hit) {
                    // Yellow: Occupied (hit but not first)
                    r = 255; g = 255; b = 0;
                } else if (is_shadow) {
                    // Blue: Shadow region (behind first hit)
                    r = 0; g = 0; b = 255;
                } else {
                    // Green: Free space
                    r = 0; g = 255; b = 0;
                }
            }

            vis_buf(r_idx, b_idx, 0) = r;
            vis_buf(r_idx, b_idx, 1) = g;
            vis_buf(r_idx, b_idx, 2) = b;
        }
    }

    return vis_image;
}

// Check if voxel is in shadow region (simplified: global minimum first_hit)
bool RayProcessor::is_voxel_in_shadow(
    const Eigen::Vector3d& voxel_world,
    const Eigen::Matrix4d& T_world_to_sonar,
    double global_min_first_hit
) const {
    // Transform voxel to sonar frame
    Eigen::Vector4d voxel_world_homo(voxel_world.x(), voxel_world.y(), voxel_world.z(), 1.0);
    Eigen::Vector4d voxel_sonar = T_world_to_sonar * voxel_world_homo;

    double x_s = voxel_sonar[0];
    double y_s = voxel_sonar[1];

    // Calculate HORIZONTAL range only (ignore Z for 2D horizontal range)
    double voxel_range = std::sqrt(x_s * x_s + y_s * y_s);

    // Simple: if voxel is beyond global minimum first_hit, it's in shadow
    return voxel_range >= global_min_first_hit;
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
        .def_readwrite("range_max", &RayProcessorConfig::range_max)
        .def_readwrite("range_min", &RayProcessorConfig::range_min)
        .def_readwrite("range_resolution", &RayProcessorConfig::range_resolution)
        .def_readwrite("vertical_fov", &RayProcessorConfig::vertical_fov)
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
        .def_readwrite("intensity_threshold", &RayProcessorConfig::intensity_threshold)
        .def_readwrite("update_method", &RayProcessorConfig::update_method)
        .def_readwrite("sharpness", &RayProcessorConfig::sharpness)
        .def_readwrite("decay_rate", &RayProcessorConfig::decay_rate)
        .def_readwrite("min_alpha", &RayProcessorConfig::min_alpha)
        .def_readwrite("L_min", &RayProcessorConfig::L_min)
        .def_readwrite("L_max", &RayProcessorConfig::L_max)
        .def_readwrite("intensity_max", &RayProcessorConfig::intensity_max);

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
             "    polar_image: 2D NumPy array (num_range_bins × num_beams), uint8\n"
             "    T_sonar_to_world: 4×4 transformation matrix (sonar → world frame)")
        .def("compute_first_hit_map",
             &RayProcessor::compute_first_hit_map,
             py::arg("polar_image"),
             "Compute first hit range for each bearing\n\n"
             "Args:\n"
             "    polar_image: 2D NumPy array (num_range_bins × num_beams), uint8\n"
             "Returns:\n"
             "    List of first hit ranges (meters) for each bearing")
        .def("generate_hit_map_visualization",
             &RayProcessor::generate_hit_map_visualization,
             py::arg("polar_image"),
             "Generate hit map visualization image with same logic as actual voxel update\n\n"
             "Args:\n"
             "    polar_image: 2D NumPy array (num_range_bins × num_beams), uint8\n"
             "Returns:\n"
             "    RGB image (num_range_bins × num_beams × 3), uint8\n"
             "Color codes:\n"
             "    Black (0,0,0): Invalid range\n"
             "    Red (255,0,0): First hit\n"
             "    Yellow (255,255,0): Occupied (hit but not first)\n"
             "    Green (0,255,0): Free space\n"
             "    Blue (0,0,255): Shadow region")
        .def("is_voxel_in_shadow",
             &RayProcessor::is_voxel_in_shadow,
             py::arg("voxel_world"),
             py::arg("T_world_to_sonar"),
             py::arg("global_min_first_hit"),
             "Check if voxel is in shadow region (simplified: global minimum first_hit)\n\n"
             "Args:\n"
             "    voxel_world: Voxel position in world frame (3D)\n"
             "    T_world_to_sonar: Inverse transformation matrix (world → sonar)\n"
             "    global_min_first_hit: Global minimum first hit range across all bearings (meters)\n"
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
