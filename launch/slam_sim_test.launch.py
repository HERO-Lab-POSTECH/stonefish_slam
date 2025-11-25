#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for SLAM simulation with 5-frame sampling test mode

    This launch file starts:
    1. Feature extraction from sonar (CFAR-based)
    2. SLAM node with test configuration (5-frame sampling, coarser resolution)
    3. Static TF publishers

    Test mode configuration:
    - frame_sampling_interval: 5 (process every 5th frame)
    - voxel_resolution: 0.3m (reduced from 0.2m for faster processing)
    - Reduces computation by ~80% for testing purposes
    """

    # Declare launch arguments
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM processing'
    )

    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Vehicle name for topic namespacing'
    )

    # Get package directories
    pkg_share = get_package_share_directory('stonefish_slam')

    # Config files
    feature_config = os.path.join(pkg_share, 'config', 'feature.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam.yaml')
    test_config = os.path.join(pkg_share, 'config', 'test_mapping.yaml')  # Test config
    icp_config = os.path.join(pkg_share, 'config', 'icp.yaml')

    # Feature extraction node
    feature_extraction_node = Node(
        package='stonefish_slam',
        executable='feature_extraction_node_sim',
        name='feature_extraction_sim_node',
        output='screen',
        parameters=[
            feature_config,
            {'vehicle_name': LaunchConfiguration('vehicle_name')}
        ]
    )

    # SLAM node with test configuration
    slam_node = Node(
        package='stonefish_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[
            slam_config,  # Base config
            test_config,  # Test overrides (frame_sampling_interval, voxel_resolution)
            {
                'enable_slam': LaunchConfiguration('enable_slam'),
                'icp_config': icp_config,
            }
        ]
    )

    # Static TF: world_ned -> {vehicle_name}_map (identity transform)
    world_ned_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_ned_to_vehicle_map_tf_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world_ned',
            '--child-frame-id', [LaunchConfiguration('vehicle_name'), TextSubstitution(text='_map')]
        ]
    )

    return LaunchDescription([
        # Arguments
        enable_slam_arg,
        vehicle_name_arg,

        # Nodes
        feature_extraction_node,
        slam_node,
        world_ned_to_map_tf,
    ])
