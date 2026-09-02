#!/usr/bin/env python3
"""Launch file for the opt-in standalone feature extraction node.

Feature extraction is internal to slam_node by default; this launch provides
the optional standalone variant that only publishes feature points.

config/slam.yaml is loaded first (its '/**' selector matches this node too), so
CFAR.*, filter.* and sonar.* come from the same file as slam_node; the dict
after it only sets the topics derived from vehicle_name.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
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

    feature_node = Node(
        package='stonefish_slam',
        executable='feature_extraction_node',
        name='feature_extraction_node',
        parameters=[
            os.path.join(get_package_share_directory('stonefish_slam'), 'config', 'slam.yaml'),
            {
                'sonar_topic': ['/', vehicle_name, '/fls/image'],
                'feature_topic': '/feature_extraction/points',
                'vehicle_name': vehicle_name,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        vehicle_name_arg,
        use_sim_time_arg,
        feature_node,
    ])
