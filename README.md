# Stonefish SLAM

Sonar-based SLAM (Simultaneous Localization and Mapping) for underwater robots using the Stonefish simulator.

## Features

- Forward-Looking Sonar (FLS) based feature extraction using CFAR
- 2D occupancy grid mapping
- 3D OctoMap-based probabilistic mapping
- Sequential Scan Matching (SSM) for localization
- Loop closure with factor graph optimization (GTSAM)
- Multiple update methods: log-odds, weighted average, IWLO

## Requirements

### Dependencies

- stonefish_sim packages (stonefish_ros2, stonefish_msgs)
- Python: numpy, scipy, opencv-python, transforms3d
- C++ build: pybind11, Eigen3, PCL, libpointmatcher
- Optional: GTSAM (for factor graph optimization)

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

# Terminal 2: Run controller
ros2 launch stonefish_control controller.launch.py controller_type:=position

# Terminal 3: Run path following (robot needs to move for SLAM)
ros2 launch stonefish_trajectory_manager path_following.launch.py

# Terminal 4: Run SLAM
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
| `/stonefish_slam/feature_extraction/feature` | sensor_msgs/PointCloud2 | Extracted features |

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
| `slam.yaml` | Global SLAM settings |

### Update Methods

- **log_odds**: Standard log-odds update (config: `method_log_odds.yaml`)
- **weighted_avg**: Weighted average update (config: `method_weighted_avg.yaml`)
- **iwlo**: Inverse-Weighted Log-Odds (config: `method_iwlo.yaml`)

## Nodes

| Node | Description |
|------|-------------|
| `slam_node` | Main SLAM node (feature extraction + localization + mapping) |
| `dead_reckoning_node` | Dead reckoning from DVL/IMU |
| `kalman_node` | Kalman filter state estimation |
| `mapping_3d_standalone` | Standalone 3D mapping |
| `mapping_2d_standalone` | Standalone 2D mapping |

## C++ Modules (pybind11)

High-performance modules:
- `cfar`: CFAR feature extraction
- `dda_traversal`: DDA ray traversal for 3D mapping
- `octree_mapping`: OctoMap-based 3D mapping (OpenMP)
- `ray_processor`: Sonar ray processing
- `pcl_module`: ICP with libpointmatcher

## License

GPL-3.0
