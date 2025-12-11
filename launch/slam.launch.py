#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """
    Launch setup function for SLAM node with conditional parameters.

    This function is called by OpaqueFunction to allow runtime evaluation
    of launch arguments before creating nodes.
    """

    # Get package directories
    pkg_share = get_package_share_directory('stonefish_slam')

    # Config files (modular structure)
    sonar_config = os.path.join(pkg_share, 'config', 'sonar.yaml')
    feature_config = os.path.join(pkg_share, 'config', 'feature.yaml')
    localization_config = os.path.join(pkg_share, 'config', 'localization.yaml')
    factor_graph_config = os.path.join(pkg_share, 'config', 'factor_graph.yaml')
    mapping_config = os.path.join(pkg_share, 'config', 'mapping.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam.yaml')
    icp_config = os.path.join(pkg_share, 'config', 'icp.yaml')

    # Get launch argument values
    vehicle_name = LaunchConfiguration('vehicle_name').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    enable_2d_mapping = LaunchConfiguration('enable_2d_mapping').perform(context)
    enable_3d_mapping = LaunchConfiguration('enable_3d_mapping').perform(context)
    update_method = LaunchConfiguration('update_method').perform(context)
    ssm_enable = LaunchConfiguration('ssm_enable').perform(context)
    nssm_enable = LaunchConfiguration('nssm_enable').perform(context)

    # Method-specific config
    method_config = os.path.join(pkg_share, 'config', 'mapping', f'method_{update_method}.yaml')

    # Build parameter dict with conditional ssm/nssm override
    param_dict = {
        'icp_config': icp_config,
        'mode': mode,
        'enable_2d_mapping': enable_2d_mapping.lower() == 'true',
        'enable_3d_mapping': enable_3d_mapping.lower() == 'true',
        'update_method': update_method,
        'vehicle_name': vehicle_name
    }

    # Only add ssm.enable/nssm.enable if explicitly set (override yaml)
    if ssm_enable.lower() in ['true', 'false']:
        param_dict['ssm.enable'] = ssm_enable.lower() == 'true'
    if nssm_enable.lower() in ['true', 'false']:
        param_dict['nssm.enable'] = nssm_enable.lower() == 'true'

    # SLAM node with integrated feature extraction
    slam_node = Node(
        package='stonefish_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[
            sonar_config,         # Sonar hardware + common parameters
            feature_config,       # Feature extraction params (CFAR, filters)
            localization_config,  # SLAM keyframes, noise models, SSM, ICP config path
            factor_graph_config,  # Loop closure (NSSM) and consistency verification (PCM)
            mapping_config,       # 2D/3D mapping parameters
            method_config,        # Method-specific config (log_odds, weighted_avg, iwlo)
            slam_config,          # Integration settings (ssm.enable, nssm.enable defaults)
            param_dict            # Runtime overrides
        ]
    )

    # Static TF: world_ned -> {vehicle_name}_map (identity transform)
    world_ned_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_ned_to_vehicle_map_tf_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world_ned',
            '--child-frame-id', f'{vehicle_name}_map'
        ]
    )

    return [slam_node, world_ned_to_map_tf]


def generate_launch_description():
    """
    Launch file for SLAM simulation with Stonefish

    This launch file starts:
    1. Feature extraction from sonar (CFAR-based)
    2. SLAM node (uses simulator's odometry directly)
    3. Static TF publishers
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
        default_value='true',
        description='Enable 3D mapping'
    )

    update_method_arg = DeclareLaunchArgument(
        'update_method',
        default_value='log_odds',
        description='3D mapping probability update method: log_odds, weighted_avg, iwlo'
    )

    ssm_enable_arg = DeclareLaunchArgument(
        'ssm_enable',
        default_value='',  # Empty = use yaml default
        description='Override ssm.enable (true/false, empty = use yaml)'
    )

    nssm_enable_arg = DeclareLaunchArgument(
        'nssm_enable',
        default_value='',  # Empty = use yaml default
        description='Override nssm.enable (true/false, empty = use yaml)'
    )

    return LaunchDescription([
        # Arguments
        rviz_arg,
        vehicle_name_arg,
        mode_arg,
        enable_2d_mapping_arg,
        enable_3d_mapping_arg,
        update_method_arg,
        ssm_enable_arg,
        nssm_enable_arg,

        # Nodes (created via OpaqueFunction for conditional param handling)
        OpaqueFunction(function=launch_setup),
    ])
