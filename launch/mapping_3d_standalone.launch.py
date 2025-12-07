#!/usr/bin/env python3
"""Launch file for independent 3D mapping standalone node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    # Get package share directory
    pkg_share = Path(get_package_share_directory('stonefish_slam'))
    config_dir = pkg_share / 'config'

    # Declare launch arguments
    update_method_arg = DeclareLaunchArgument(
        'update_method',
        default_value='log_odds',
        description='3D mapping probability update method: log_odds, weighted_avg, iwlo'
    )

    # Build method config path
    update_method = LaunchConfiguration('update_method')

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
                # Method-specific config - use method_iwlo.yaml for IWLO testing
                str(config_dir / 'mapping' / 'method_iwlo.yaml'),
                {
                    # Standalone-specific settings only
                    'resolution': 0.2,
                    'frame_interval': 5,
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
