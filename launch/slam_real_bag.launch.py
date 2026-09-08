#!/usr/bin/env python3
"""SLAM on a real sea-trial bag (KMU/LIG dataset) with the real-bag profile.

Wraps slam.launch.py with config/real_bag_overrides.yaml (bag topics,
compressed FLS decode, real-data tuning) and adds the odometry→TF bridge so
RViz can anchor views on the vehicle frame.

Usage:
    ros2 launch stonefish_slam slam_real_bag.launch.py
    ros2 bag play <bag> --clock
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('stonefish_slam')
    override_config = os.path.join(pkg_share, 'config', 'real_bag_overrides.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz with the real-bag display config')

    bridge_odom_topic_arg = DeclareLaunchArgument(
        'bridge_odom_topic',
        default_value='/vehicle_synced/dynamics/odometry',
        description='Bag odometry topic republished as TF by odom_tf_bridge')

    bridge_child_frame_arg = DeclareLaunchArgument(
        'bridge_child_frame',
        default_value='girona500/base_link',
        description='TF child frame for the bridged odometry ("" = msg.child_frame_id)')

    # SLAM with the real-bag parameter profile; RViz disabled here because the
    # sim-oriented default config lacks the bag displays (started below instead).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'override_config': override_config,
            'icp_config_file': 'icp_real_bag.yaml',
            'rviz': 'false',
        }.items()
    )

    # Bags stream the vehicle pose only as Odometry; republishing it as TF lets
    # RViz anchor views on the vehicle frame.
    odom_tf_bridge = Node(
        package='stonefish_slam',
        executable='odom_tf_bridge',
        name='odom_tf_bridge',
        output='screen',
        parameters=[{
            'odom_topic': LaunchConfiguration('bridge_odom_topic'),
            'child_frame': LaunchConfiguration('bridge_child_frame'),
            'use_sim_time': True,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'real_bag.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        rviz_arg,
        bridge_odom_topic_arg,
        bridge_child_frame_arg,
        slam,
        odom_tf_bridge,
        rviz_node,
    ])
