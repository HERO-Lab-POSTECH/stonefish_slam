#!/usr/bin/env python3
"""Launch file for the opt-in standalone FFT localization node.

FFT localization is internal to slam_node by default; this launch provides
the optional standalone variant that only publishes raw consecutive-frame
transforms (no odom validation, no DR fallback).

Parameters are passed directly as a dict because the node name
('fft_localization_node') does not match the config YAML namespace
('slam_node'). The node's declare_parameter defaults already mirror
config/slam.yaml (fft_localization.*) and config/sonar.yaml (sonar.*), so
these values keep the launch self-documenting and parameterized by vehicle_name.
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

    fft_node = Node(
        package='stonefish_slam',
        executable='fft_localization_node',
        name='fft_localization_node',
        parameters=[
            {
                'sonar_topic': ['/', vehicle_name, '/fls/image'],
                'pose_topic': '/fft_localization/transform',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        vehicle_name_arg,
        use_sim_time_arg,
        fft_node,
    ])
