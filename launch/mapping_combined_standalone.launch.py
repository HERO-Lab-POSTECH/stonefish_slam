#!/usr/bin/env python3
"""Launch file for combined 3D and 2D mapping standalone nodes with namespaces."""

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('stonefish_slam')

    # 3D mapping with namespace
    mapping_3d = GroupAction([
        PushRosNamespace('mapping_3d'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'mapping_3d_standalone.launch.py')
            )
        )
    ])

    # 2D mapping with namespace
    mapping_2d = GroupAction([
        PushRosNamespace('mapping_2d'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'mapping_2d_standalone.launch.py')
            )
        )
    ])

    return LaunchDescription([
        mapping_3d,
        mapping_2d
    ])
