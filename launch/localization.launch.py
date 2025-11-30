#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Launch file for localization-only mode

    This launch file starts SLAM in localization-only mode:
    - SSM (Sequential Scan Matching) enabled
    - NSSM (Non-Sequential Scan Matching / Loop Closure) disabled
    - Mapping disabled (no 2D/3D map generation)

    Use case: Real-time localization without loop closures or mapping overhead
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

    # Get package share directory
    pkg_share = FindPackageShare('stonefish_slam').find('stonefish_slam')

    # Include slam.launch.py with localization-only settings
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'mode': 'localization-only',
            'ssm.enable': 'true',
            'nssm.enable': 'false',
            'enable_2d_mapping': 'false',
            'enable_3d_mapping': 'false',
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )

    return LaunchDescription([
        # Arguments
        vehicle_name_arg,
        rviz_arg,

        # Include SLAM launch
        slam_launch,
    ])
