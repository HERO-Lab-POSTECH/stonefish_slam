#!/usr/bin/env python3
"""Launch file for independent 3D mapping standalone node."""

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    # Get package share directory
    pkg_share = Path(get_package_share_directory('stonefish_slam'))
    config_dir = pkg_share / 'config'

    # Read update_method from mapping.yaml
    mapping_yaml_path = config_dir / 'mapping.yaml'
    with open(mapping_yaml_path, 'r') as f:
        mapping_config = yaml.safe_load(f)
    default_update_method = mapping_config.get('slam_node', {}).get('ros__parameters', {}).get('mapping_3d', {}).get('update_method', 'log_odds')

    # Declare launch arguments
    update_method_arg = DeclareLaunchArgument(
        'update_method',
        default_value=default_update_method,
        description='3D mapping probability update method: log_odds, weighted_avg, iwlo'
    )

    # Build method config path dynamically
    update_method = LaunchConfiguration('update_method')
    method_config_path = PathJoinSubstitution([
        FindPackageShare('stonefish_slam'),
        'config', 'mapping',
        ['method_', update_method, '.yaml']
    ])

    return LaunchDescription([
        update_method_arg,
        Node(
            package='stonefish_slam',
            executable='mapping_3d_standalone',
            name='slam_node',  # Must match yaml namespace (slam_node.ros__parameters)
            output='screen',
            parameters=[
                str(config_dir / 'sonar.yaml'),    # Sonar parameters
                str(config_dir / 'mapping.yaml'),  # Mapping parameters
                str(config_dir / 'slam.yaml'),     # General SLAM params
                method_config_path,                # Dynamic: method_{log_odds|weighted_avg|iwlo}.yaml
                {
                    # Standalone-specific settings only
                    'resolution': 0.2,
                    'frame_interval': 10,
                    'odom_topic': '/bluerov2/odometry',
                    'sonar_topic': '/bluerov2/fls/image',
                    'update_method': update_method,
                }
            ],
            remappings=[
                # No remappings needed (direct topic subscription)
            ]
        ),
    ])
