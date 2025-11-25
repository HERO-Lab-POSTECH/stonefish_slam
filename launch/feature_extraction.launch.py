#!/usr/bin/env python3
"""
ROS2 Launch file for Feature Extraction Sim node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('stonefish_slam').find('stonefish_slam')

    # Config file path
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'feature.yaml'
    ])

    # Declare launch arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Name of the vehicle (used for topic namespacing)'
    )

    sonar_topic_arg = DeclareLaunchArgument(
        'sonar_topic',
        default_value='/bluerov2/fls/image',
        description='Sonar image topic name'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Feature extraction sim node
    feature_extraction_node = Node(
        package='stonefish_slam',
        executable='feature_extraction_node',
        name='feature_extraction_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'vehicle_name': LaunchConfiguration('vehicle_name'),
                'sonar_image_topic': LaunchConfiguration('sonar_topic')
            }
        ]
    )

    return LaunchDescription([
        vehicle_name_arg,
        sonar_topic_arg,
        use_sim_time_arg,
        feature_extraction_node,
    ])
