#!/usr/bin/env python3
"""Launch file for independent 3D mapping test node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stonefish_slam',
            executable='mapping_3d_test',
            name='mapping_3d_test_node',
            output='screen',
            parameters=[{
                'resolution': 0.3,
                'frame_interval': 10,
                'odom_topic': '/bluerov2/odometry',
                'sonar_topic': '/bluerov2/fls/image',
            }],
            remappings=[
                # No remappings needed (direct topic subscription)
            ]
        ),
    ])
