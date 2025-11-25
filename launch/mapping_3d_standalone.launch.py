#!/usr/bin/env python3
"""Launch file for independent 3D mapping standalone node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    # Get package share directory
    pkg_share = Path(get_package_share_directory('stonefish_slam'))
    config_dir = pkg_share / 'config'

    return LaunchDescription([
        Node(
            package='stonefish_slam',
            executable='mapping_3d_standalone',
            name='slam_node',  # Match namespace in slam.yaml
            output='screen',
            parameters=[
                str(config_dir / 'slam.yaml'),  # Load slam.yaml for all params
                {
                    # Standalone-specific settings only
                    'resolution': 0.2,  # Test 3: 0.3 → 0.2
                    'frame_interval': 20,
                    'odom_topic': '/bluerov2/odometry',
                    'sonar_topic': '/bluerov2/fls/image',
                }
            ],
            remappings=[
                # No remappings needed (direct topic subscription)
            ]
        ),
    ])
