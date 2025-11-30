#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Launch file for mapping-only mode

    This launch file starts SLAM in mapping-only mode:
    - SSM (Sequential Scan Matching) disabled
    - NSSM (Non-Sequential Scan Matching / Loop Closure) disabled
    - Dead reckoning poses used directly for mapping
    - 2D mapping enabled by default (can be overridden)

    Use case: Generate sonar maps using accurate external localization (e.g., DVL + IMU)
    """

    # Declare launch arguments
    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Vehicle name for topic namespacing'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )

    enable_2d_mapping_arg = DeclareLaunchArgument(
        'enable_2d_mapping',
        default_value='true',
        description='Enable 2D mapping'
    )

    enable_3d_mapping_arg = DeclareLaunchArgument(
        'enable_3d_mapping',
        default_value='false',
        description='Enable 3D mapping'
    )

    # Get package share directory
    pkg_share = FindPackageShare('stonefish_slam').find('stonefish_slam')

    # Include slam.launch.py with mapping-only settings
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'mode': 'mapping-only',
            'enable_slam': 'false',
            'enable_2d_mapping': LaunchConfiguration('enable_2d_mapping'),
            'enable_3d_mapping': LaunchConfiguration('enable_3d_mapping'),
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )

    return LaunchDescription([
        # Arguments
        vehicle_name_arg,
        rviz_arg,
        enable_2d_mapping_arg,
        enable_3d_mapping_arg,

        # Include SLAM launch
        slam_launch,
    ])
