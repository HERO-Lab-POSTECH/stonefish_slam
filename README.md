# Stonefish SLAM

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

### Dependencies

- stonefish_sim packages (stonefish_ros2, stonefish_msgs)
- Python: numpy, scipy, opencv-python, scikit-learn, shapely, matplotlib
- ROS: tf_transformations
- C++ build: pybind11, Eigen3, PCL, libpointmatcher
- GTSAM (required — factor graph optimization; unguarded module-top import)

### Build

```bash
cd /workspace/colcon_ws
colcon build --packages-select stonefish_slam
source install/setup.bash
```

## Quick Start

```bash
# Terminal 1: Run simulator
ros2 launch stonefish_ros2 bluerov2.launch.py

# Terminal 2: Run path following (includes hybrid controller)
ros2 launch stonefish_trajectory_manager path_following.launch.py

# Terminal 3: Run SLAM
ros2 launch stonefish_slam slam.launch.py vehicle_name:=bluerov2
```

### Other Modes

```bash
# Mapping only (no localization)
ros2 launch stonefish_slam mapping.launch.py vehicle_name:=bluerov2

# Localization only (no mapping)
ros2 launch stonefish_slam localization.launch.py vehicle_name:=bluerov2
```

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `slam.launch.py` | Full SLAM with configurable modes |
| `mapping.launch.py` | Mapping only (SSM/NSSM disabled) |
| `localization.launch.py` | Localization only (mapping disabled) |
| `dead_reckoning.launch.py` | Dead reckoning node |
| `mapping_3d_standalone.launch.py` | Standalone 3D mapping |
| `mapping_2d_standalone.launch.py` | Standalone 2D mapping |
| `mapping_combined_standalone.launch.py` | Standalone 2D + 3D mapping |

### Launch Arguments

```bash
ros2 launch stonefish_slam slam.launch.py \
    vehicle_name:=bluerov2 \
    mode:=slam \                    # slam, localization-only, mapping-only
    enable_2d_mapping:=true \
    enable_3d_mapping:=true \
    update_method:=iwlo \           # log_odds, weighted_avg, iwlo
    ssm_enable:=true \
    nssm_enable:=false \
    rviz:=true
```

## ROS2 Topics

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `/{vehicle}/fls/image` | sensor_msgs/Image | Forward-Looking Sonar |
| `/{vehicle}/odometry` | nav_msgs/Odometry | Ground truth (simulator) |
| `/{vehicle}/imu` | sensor_msgs/Imu | IMU data |
| `/{vehicle}/dvl_sim` | stonefish_msgs/DVL | DVL velocity |
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
| `sonar.yaml` | Sonar hardware parameters (FOV, range, beams) |
| `feature.yaml` | CFAR feature extraction settings |
| `mapping.yaml` | 2D/3D mapping parameters |
| `localization.yaml` | SSM and keyframe settings |
| `factor_graph.yaml` | NSSM loop closure parameters |
| `icp.yaml` | ICP (libpointmatcher) configuration |
| `dead_reckoning.yaml` | Dead-reckoning (DVL/IMU) settings |
| `slam.yaml` | Global SLAM settings |

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

## License

GPL-3.0
