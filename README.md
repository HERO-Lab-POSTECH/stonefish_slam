# Stonefish SLAM

📖 **Full documentation site (Korean): https://hero-lab-postech.github.io/stonefish_slam/**

Sonar-based SLAM (Simultaneous Localization and Mapping) for underwater robots using the Stonefish simulator.

## Features

- Forward-Looking Sonar (FLS) based feature extraction using CFAR
- 2D occupancy grid mapping
- 3D OctoMap-based probabilistic mapping
- Sequential Scan Matching (SSM) for localization
- Loop closure with factor graph optimization (GTSAM)
- Multiple update methods: log-odds, weighted average, IWLO

## Coordinate Frames

- **Global frame: `world_ned` (NED — North-East-Down).** The Stonefish simulator publishes its world frame as NED, so SLAM output messages (`pose`, `traj`, `cloud`, `constraint`, 2D/3D maps) all use `world_ned`. This is an intentional departure from REP-103/105 (which prescribe ENU) to stay consistent with the simulator.
- **Local TF chain: `odom → base_link` (ENU, REP-105).** Dead reckoning keeps the standard REP-105 ENU body frames. The global↔local TF publishers are identity (no rotation), so only frame naming differs.

See `docs/CONVENTIONS.md` §2.0 for the rationale.

## Requirements

This is a mixed C++/Python ROS 2 (Humble) package. It needs a C++ toolchain and native
libraries (for the pybind11 extensions) **in addition to** the Python packages.

### Dependencies

**ROS 2 packages** (from `package.xml`): `rclcpp`, `rclpy`, `std_msgs`, `sensor_msgs`,
`geometry_msgs`, `nav_msgs`, `visualization_msgs`, `octomap_msgs`, `tf2_ros`,
`tf2_geometry_msgs`, `cv_bridge`, `message_filters`, `sensor_msgs_py`, `tf_transformations`,
and `stonefish_msgs` (provided by the stonefish_sim repo).

**System / C++ build dependencies** (from `CMakeLists.txt`):

| Dependency | Required? | Used by |
|------------|-----------|---------|
| `pybind11` | required | all 5 C++ extension modules |
| Eigen3 (`libeigen3-dev`, `eigen3_cmake_module`) | required | all C++ extensions |
| OctoMap (`octomap`) | required | `octree_mapping`, `ray_processor` |
| GTSAM (`ros-humble-gtsam`) | required (C++ lib) | factor-graph backend — but **Python needs pip, see warning** |
| OpenMP | optional | parallel ray processing (falls back to single-threaded) |
| libpointmatcher | optional (`QUIET`) | C++ ICP (`pcl` module); Python fallback if absent |
| PCL (`common`, `filters`) | optional (`QUIET`) | C++ ICP (`pcl` module); Python fallback if absent |

**Python packages** (from `package.xml` / source imports): `numpy`, `scipy`,
`opencv-python` (`cv2`), `pyyaml`, `scikit-learn` (`sklearn`), `shapely`, `matplotlib`,
and `gtsam`.

> ⚠️ **GTSAM Python bindings require pip — apt is not enough.**
> `sudo apt install ros-humble-gtsam` installs **only the C++ library**, so `import gtsam`
> from Python (`core/factor_graph.py`, `core/localization.py`, `core/types.py`, etc.) fails.
> You **must** also run `pip install gtsam` to get the Python bindings. Without it the SLAM
> node will not start.

### Docker (recommended for deployment / reproduction)

The team Docker environment lives in a dedicated repo:
[**stonefish_bringup**](https://github.com/HERO-Lab-POSTECH/stonefish_bringup).
`stonefish_slam` is built into the **same image** as the simulator (the
multi-stage Dockerfile bakes Stonefish core + `stonefish_sim` + this repo via
`stonefish.repos`), so all C++ dependencies (GTSAM, OctoMap, pybind11) and the
5 `.so` extensions are compiled at image build time — no per-machine setup.

```bash
git clone https://github.com/HERO-Lab-POSTECH/stonefish_bringup.git
cd stonefish_bringup
docker compose build       # first build ~15-30 min
```

Requirements (NVIDIA GPU + nvidia-container-toolkit, host X display `:0`) and
the run commands are documented in the
[stonefish_bringup README](https://github.com/HERO-Lab-POSTECH/stonefish_bringup#readme).
For host development, continue with the Native install below.

### Native install

Source ROS 2 Humble first, then install the system and Python dependencies.

```bash
# 0. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 1. System / C++ build dependencies (apt)
sudo apt install \
    ros-humble-gtsam \
    ros-humble-libpointmatcher \
    ros-humble-octomap \
    ros-humble-tf-transformations \
    pybind11-dev \
    libeigen3-dev

# 2. Python GTSAM bindings (REQUIRED — apt provides C++ only)
pip install gtsam
```

### Build

Build the package with `--merge-install` (the existing install tree uses the merged layout),
then source it.

```bash
colcon build --merge-install --packages-select stonefish_slam
source install/setup.bash
```

### Verify the C++ extensions (.so) built

A successful build installs 5 pybind11 modules (`cfar`, `dda_traversal`, `octree_mapping`,
`ray_processor`, `pcl`). Confirm the `.so` files are present (Python 3.10 path shown):

```bash
ls install/local/lib/python3.10/dist-packages/stonefish_slam/*.so
```

If a C++ extension is missing, the package logs a warning and runs the pure-Python fallback
(the libpointmatcher ICP fallback is less precise — build the extensions for production accuracy).

## Quick Start

```bash
# Terminal 1: Run simulator
ros2 launch stonefish_ros2 bluerov2.launch.py

# Terminal 2: Run path following (includes hybrid controller)
ros2 launch stonefish_trajectory_manager path.launch.py

# Terminal 3: Run SLAM
ros2 launch stonefish_slam slam.launch.py vehicle_name:=bluerov2
```

### Other Modes

```bash
# Mapping only (no scan matching, maps from odometry)
ros2 launch stonefish_slam slam.launch.py mode:=mapping

# Localization only (ICP on, no loop closure, no maps)
ros2 launch stonefish_slam slam.launch.py mode:=localization

# Replay a recorded bag instead of the simulator (the bag publishes /clock)
ros2 launch stonefish_slam slam.launch.py use_sim_time:=true rviz:=false
ros2 bag play <bag> --clock
```

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `slam.launch.py` | SLAM node — `mode:=slam` (default) / `localization` / `mapping` |
| `slam_real_bag.launch.py` | `slam.launch.py` + `config/real_bag_overrides.yaml` + odometry→TF bridge for sea-trial bags |
| `dead_reckoning.launch.py` | Dead reckoning node |
| `mapping_3d_standalone.launch.py` | Standalone 3D mapping |
| `mapping_2d_standalone.launch.py` | Standalone 2D mapping |
| `mapping_combined_standalone.launch.py` | Standalone 2D + 3D mapping |

### Launch Arguments

```bash
ros2 launch stonefish_slam slam.launch.py \
    vehicle_name:=bluerov2 \
    mode:=slam \                    # slam | localization | mapping
    enable_2d_mapping:=true \       # empty (default) = value from config/slam.yaml
    enable_3d_mapping:=true \
    update_method:=iwlo \           # log_odds | weighted_avg | iwlo (default from config/slam.yaml)
    icp_config_file:=icp.yaml \     # or icp_real_bag.yaml
    override_config:='' \           # optional profile yaml loaded after config/slam.yaml
    evaluate:=true \                # accuracy monitor + 2D trajectory-error accumulator
    rviz:=true \
    use_sim_time:=false             # true only with `ros2 bag play --clock`
```

Precedence, last wins: `config/slam.yaml` < `config/mapping/method_<update_method>.yaml`
< `override_config` < `mode` preset < explicit `enable_*_mapping` / `update_method` arguments.
Finer switches (`ssm.enable`, `nssm.enable`, `fft_localization.enable`, …) live in
`config/slam.yaml`.

## ROS2 Topics

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle}/fls/image` | sensor_msgs/Image | Forward-Looking Sonar |
| `/{vehicle}/odometry` | nav_msgs/Odometry | Ground truth (simulator) |
| `/{vehicle}/imu` | sensor_msgs/Imu | IMU data |
| `/{vehicle}/dvl` | stonefish_msgs/DVL | DVL velocity |
| `/{vehicle}/pressure` | sensor_msgs/FluidPressure | Depth sensor |

### Publications

| Topic | Type | Description |
|-------|------|-------------|
| `/stonefish_slam/slam/pose` | geometry_msgs/PoseWithCovarianceStamped | SLAM pose estimate |
| `/stonefish_slam/slam/odom` | nav_msgs/Odometry | SLAM odometry |
| `/stonefish_slam/slam/traj` | sensor_msgs/PointCloud2 | Trajectory |
| `/stonefish_slam/mapping/map_2d_image` | sensor_msgs/Image | 2D occupancy grid |
| `/stonefish_slam/mapping/map_3d_octomap` | octomap_msgs/Octomap | 3D OctoMap |
| `/stonefish_slam/slam/constraint` | visualization_msgs/Marker | Loop-closure constraints |
| `/stonefish_slam/slam/cloud` | sensor_msgs/PointCloud2 | Aggregated point cloud |

## Configuration Files

Located in `config/`:

| File | Description |
|------|-------------|
| `slam.yaml` | Every `slam_node` parameter, in sections: data source, module switches, sonar hardware, CFAR feature extraction, keyframes/noise, SSM (ICP), NSSM/PCM loop closure, FFT localization, 2D/3D mapping |
| `mapping/method_*.yaml` | Per-method 3D probability-update values; `update_method` picks one |
| `icp.yaml`, `icp_real_bag.yaml` | ICP (libpointmatcher) chains; `icp_config_file` picks one |
| `real_bag_overrides.yaml` | Real sea-trial profile (bag topics, compressed FLS, relaxed gates); `override_config` |
| `dead_reckoning.yaml` | Dead-reckoning node (DVL/IMU) settings |

`sonar.sonar_tilt_deg` is the depression angle below the horizontal plane and must equal
the simulator's FLS mounting (`bluerov2.scn`, where the `rpy` roll is measured from nadir:
tilt = 90° − roll); `test_sonar_tilt_matches_sim_scenario.py` checks both repos.

### Update Methods

- **log_odds**: Standard log-odds update (config: `config/mapping/method_log_odds.yaml`)
- **weighted_avg**: Weighted average update (config: `config/mapping/method_weighted_avg.yaml`)
- **iwlo**: Inverse-Weighted Log-Odds (config: `config/mapping/method_iwlo.yaml`)

## Nodes

| Node | Description |
|------|-------------|
| `slam_node` | Main SLAM node (feature extraction + localization + mapping) |
| `dead_reckoning_node` | Dead reckoning from DVL/IMU |
| `mapping_3d_standalone` | Standalone 3D mapping |
| `mapping_2d_standalone` | Standalone 2D mapping |

## C++ Modules (pybind11)

High-performance modules:
- `cfar`: CFAR feature extraction
- `dda_traversal`: DDA ray traversal for 3D mapping
- `octree_mapping`: OctoMap-based 3D mapping (OpenMP)
- `ray_processor`: Sonar ray processing
- `pcl_module`: ICP with libpointmatcher

If a C++ extension is missing (not built), the package logs a warning naming the
absent module and what degrades, then runs the pure-Python fallback. The ICP
fallback is less precise than the libpointmatcher C++ path — build the extensions
for production accuracy.

## Testing

```bash
# stage the built pybind11 extensions into the source tree (once per build)
cp <ws>/build/stonefish_slam/*.so stonefish_slam/
python3 -m pytest        # from the repo root
```

Without the staged `.so` files, collection fails at
`stonefish_slam.cpp` imports — this is expected, not a broken checkout.

## Contributing

Branch/commit/PR rules live in [CONTRIBUTING.md](CONTRIBUTING.md)
(GitHub Flow · Conventional Commits · 1-reviewer PR gate). Run
`python3 -m pytest` before opening a PR.

## License

GPL-3.0
