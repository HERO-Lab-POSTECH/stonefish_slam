#!/usr/bin/env python3
"""Launch file for independent 3D mapping test node."""

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
            executable='mapping_3d_test',
            name='mapping_3d_test_node',
            output='screen',
            parameters=[
                str(config_dir / 'slam.yaml'),  # Load slam.yaml for sonar/mapping_3d params
                {
                    # Test-specific overrides
                    'resolution': 0.5,
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
