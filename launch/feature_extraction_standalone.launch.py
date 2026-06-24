#!/usr/bin/env python3
"""Launch file for the opt-in standalone feature extraction node.

Feature extraction is internal to slam_node by default; this launch provides
the optional standalone variant that only publishes feature points.

Parameters are passed directly as a dict because the node name
('feature_extraction_node') does not match the config YAML namespace
('slam_node'). The node's declare_parameter defaults already mirror
config/feature.yaml and config/sonar.yaml, so these values keep the
launch self-documenting and parameterized by vehicle_name.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
