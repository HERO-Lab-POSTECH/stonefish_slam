#!/usr/bin/env python3
"""
ROS2 Launch file for Dead Reckoning node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('stonefish_slam').find('stonefish_slam')

    # Config file path
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'dead_reckoning.yaml'
    ])

    # Declare launch arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Name of the vehicle (used for topic namespacing)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Dead reckoning node
    dead_reckoning_node = Node(
        package='stonefish_slam',
        executable='dead_reckoning_node',
        name='dead_reckoning_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'vehicle_name': LaunchConfiguration('vehicle_name')
            }
        ],
        remappings=[
            # Add any additional topic remappings here if needed
        ]
    )

    # Static TF publishers - aligned with Stonefish (world_ned is the base frame)
    # Note: Stonefish publishes world_ned -> vehicle/base_link
    tf_world_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world_ned', '--child-frame-id', 'map'
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        vehicle_name_arg,
        use_sim_time_arg,
        dead_reckoning_node,
        tf_world_to_map,
    ])
