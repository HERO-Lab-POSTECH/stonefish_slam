#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
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

    vehicle_name_arg = DeclareLaunchArgument(
        'vehicle_name',
        default_value='bluerov2',
        description='Vehicle name for topic namespacing'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Operating mode: slam, localization-only, mapping-only'
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

    update_method_arg = DeclareLaunchArgument(
        'update_method',
        default_value='log_odds',
        description='3D mapping probability update method: log_odds, weighted_avg, iwlo'
    )

    # Note: ssm.enable and nssm.enable are now loaded from slam.yaml only
    # To override, use command line: ros2 launch ... ssm.enable:=true
    # (Removed from default launch arguments to respect yaml config)

    # Get package directories
    pkg_share = get_package_share_directory('stonefish_slam')

    # Config files (modular structure)
    # NOTE: Load order matters - base configs first, module overrides last
    sonar_config = os.path.join(pkg_share, 'config', 'sonar.yaml')
    feature_config = os.path.join(pkg_share, 'config', 'feature.yaml')
    localization_config = os.path.join(pkg_share, 'config', 'localization.yaml')
    factor_graph_config = os.path.join(pkg_share, 'config', 'factor_graph.yaml')
    mapping_config = os.path.join(pkg_share, 'config', 'mapping.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam.yaml')
    icp_config = os.path.join(pkg_share, 'config', 'icp.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')

    # Method-specific config (dynamic loading based on update_method argument)
    method_config = PathJoinSubstitution([
        FindPackageShare('stonefish_slam'),
        'config', 'mapping',
        PythonExpression(["'method_' + '", LaunchConfiguration('update_method'), "' + '.yaml'"])
    ])

    # SLAM node with integrated feature extraction
    # NOTE: Feature extraction is now INTERNAL to slam_node
    slam_node = Node(
        package='stonefish_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[
            sonar_config,         # Sonar hardware + common parameters (vehicle_name, topic)
            feature_config,       # Feature extraction params (CFAR, filters)
            localization_config,  # SLAM keyframes, noise models, SSM, ICP config path
            factor_graph_config,  # Loop closure (NSSM) and consistency verification (PCM)
            mapping_config,       # 2D/3D mapping parameters
            method_config,        # Method-specific config (log_odds, weighted_avg, iwlo)
            slam_config,          # Integration settings (ssm.enable, nssm.enable)
            {
                'icp_config': icp_config,
                'mode': LaunchConfiguration('mode'),
                'enable_2d_mapping': LaunchConfiguration('enable_2d_mapping'),
                'enable_3d_mapping': LaunchConfiguration('enable_3d_mapping'),
                'update_method': LaunchConfiguration('update_method'),
                # ssm.enable and nssm.enable are now loaded from slam.yaml
                'vehicle_name': LaunchConfiguration('vehicle_name')
            }
        ]
    )

    # Static TF: world_ned -> {vehicle_name}_map (identity transform)
    # Stonefish simulator uses world_ned as the root frame
    # SLAM uses {vehicle_name}_map as the global frame, so we connect them with identity transform
    world_ned_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_ned_to_vehicle_map_tf_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world_ned',
            '--child-frame-id', [LaunchConfiguration('vehicle_name'), TextSubstitution(text='_map')]
        ]
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
        vehicle_name_arg,
        mode_arg,
        enable_2d_mapping_arg,
        enable_3d_mapping_arg,
        update_method_arg,
        # ssm.enable and nssm.enable now loaded from slam.yaml (not launch args)

        # Nodes
        slam_node,
        world_ned_to_map_tf,
        # rviz_node,  # Uncomment when RViz config is ready
    ])
