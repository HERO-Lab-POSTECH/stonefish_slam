"""
Topics for the stonefish_slam project (ROS2)
"""

# Stonefish simulator topics
IMU_TOPIC = "/bluerov2/imu"
DVL_TOPIC = "/bluerov2/dvl_sim"
DEPTH_TOPIC = "/bluerov2/pressure"
SONAR_TOPIC = "/bluerov2/sonar"  # To be confirmed
GYRO_TOPIC = "/bluerov2/gyro"

# SLAM namespace
SLAM_NS = "/stonefish_slam/"

# SLAM-related topics
GYRO_INTEGRATION_TOPIC = SLAM_NS + "gyro_integrated"
SONAR_FUSION_TOPIC = SLAM_NS + "sonar_fusion"
# Use simulator's odometry directly instead of dead_reckoning/kalman output
LOCALIZATION_ODOM_TOPIC = "/bluerov2/odometry"  # Changed from SLAM_NS + "localization/odom"
LOCALIZATION_TRAJ_TOPIC = SLAM_NS + "localization/traj"
SLAM_POSE_TOPIC = SLAM_NS + "slam/pose"
SLAM_ODOM_TOPIC = SLAM_NS + "slam/odom"
SLAM_TRAJ_TOPIC = SLAM_NS + "slam/traj"
SLAM_CLOUD_TOPIC = SLAM_NS + "slam/cloud"
SLAM_CONSTRAINT_TOPIC = SLAM_NS + "slam/constraint"
SLAM_ISAM2_TOPIC = SLAM_NS + "slam/isam2"
SLAM_PREDICT_SLAM_UPDATE_SERVICE = SLAM_NS + "slam/predict_slam_update"

# Mapping topics
MAPPING_INTENSITY_TOPIC = SLAM_NS + "mapping/intensity"
MAPPING_OCCUPANCY_TOPIC = SLAM_NS + "mapping/occupancy"
MAPPING_GET_MAP_SERVICE = SLAM_NS + "mapping/get_map"

# Feature extraction topics
SONAR_FEATURE_TOPIC = SLAM_NS + "feature_extraction/feature"
SONAR_FEATURE_IMG_TOPIC = SLAM_NS + "feature_extraction/feature_img"
