#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for SLAM simulation with Stonefish

    This launch file starts:
    1. Feature extraction from sonar (CFAR-based)
    2. SLAM node (uses simulator's odometry directly)
    3. Static TF publishers
    4. (Optional) RViz visualization
    """

    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )

    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='true',
        description='Enable SLAM processing'
    )

    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Vehicle name for topic namespacing'
    )

    # Get package directories
    pkg_share = get_package_share_directory('stonefish_slam')

    # Config files
    feature_config = os.path.join(pkg_share, 'config', 'feature.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam.yaml')
    icp_config = os.path.join(pkg_share, 'config', 'icp.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    # Feature extraction node
    feature_extraction_node = Node(
        package='stonefish_slam',
        executable='feature_extraction_node_sim',
        name='feature_extraction_sim_node',  # Must match the name in feature.yaml
        output='screen',
        parameters=[
            feature_config,
            {'vehicle_name': LaunchConfiguration('vehicle_name')}
        ]
    )

    # SLAM node
    slam_node = Node(
        package='stonefish_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[
            slam_config,
            {
                'enable_slam': LaunchConfiguration('enable_slam'),
                'icp_config': icp_config,
            }
        ]
    )

    # Static TF: world_ned -> map (identity transform)
    # Stonefish simulator uses world_ned as the root frame
    # SLAM uses map as the global frame, so we connect them with identity transform
    world_ned_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_ned_to_map_tf_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'world_ned', '--child-frame-id', 'map']
    )

    # RViz node (conditional)
    # TODO: Create a proper RViz config for ROS2
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     arguments=['-d', rviz_config],
    #     condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    return LaunchDescription([
        # Arguments
        rviz_arg,
        enable_slam_arg,
        vehicle_name_arg,

        # Nodes
        feature_extraction_node,
        slam_node,
        world_ned_to_map_tf,
        # rviz_node,  # Uncomment when RViz config is ready
    ])
