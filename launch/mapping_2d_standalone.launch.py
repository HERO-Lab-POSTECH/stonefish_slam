#!/usr/bin/env python3
"""Launch file for independent 2D mapping standalone node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('stonefish_slam'))
    config_dir = pkg_share / 'config'

    # Declare launch arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Vehicle name for topic namespacing'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    vehicle_name = LaunchConfiguration('vehicle_name')

    mapping_2d_node = Node(
        package='stonefish_slam',
        executable='mapping_2d_standalone',
        name='slam_node',  # Must match yaml namespace (slam_node.ros__parameters)
        parameters=[
            str(config_dir / 'sonar.yaml'),    # Sonar parameters
            str(config_dir / 'mapping.yaml'),  # Mapping parameters
            str(config_dir / 'slam.yaml'),     # General SLAM params
            {
                'frame_interval': 10,
                'odom_topic': ['/', vehicle_name, '/odometry'],
                'sonar_topic': ['/', vehicle_name, '/fls/image'],
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        vehicle_name_arg,
        use_sim_time_arg,
        mapping_2d_node,
    ])
