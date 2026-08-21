#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import tempfile

import yaml


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
    icp_config = os.path.join(
        pkg_share, 'config',
        LaunchConfiguration('icp_config_file').perform(context))
    # Optional profile overriding the yaml defaults above (e.g.
    # real_bag_overrides.yaml). Empty string = no override.
    override_config = LaunchConfiguration('override_config').perform(context)

    # Get launch argument values
    vehicle_name = LaunchConfiguration('vehicle_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
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
        # Dot notation: the node declares mapping_3d.update_method, not a
        # bare top-level 'update_method' (which would be dropped).
        'mapping_3d.update_method': update_method,
        'vehicle_name': vehicle_name,
        'use_sim_time': use_sim_time.lower() == 'true'
    }

    # Only add ssm.enable/nssm.enable if explicitly set (override yaml)
    if ssm_enable.lower() in ['true', 'false']:
        param_dict['ssm.enable'] = ssm_enable.lower() == 'true'
    if nssm_enable.lower() in ['true', 'false']:
        param_dict['nssm.enable'] = nssm_enable.lower() == 'true'

    # SLAM node with integrated feature extraction
    parameters = [
        sonar_config,         # Sonar hardware + common parameters
        feature_config,       # Feature extraction params (CFAR, filters)
        localization_config,  # SLAM keyframes, noise models, SSM, ICP config path
        factor_graph_config,  # Loop closure (NSSM) and consistency verification (PCM)
        mapping_config,       # 2D/3D mapping parameters
        method_config,        # Method-specific config (log_odds, weighted_avg, iwlo)
        slam_config,          # Integration settings (ssm.enable, nssm.enable defaults)
    ]
    if override_config:
        # Profile overrides beat the yaml defaults; explicit launch arguments
        # (param_dict below) still win over the profile.
        parameters.append(override_config)
    parameters.append(param_dict)  # Runtime overrides

    slam_node = Node(
        package='stonefish_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=parameters
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

    # RViz visualization (gated by the 'rviz' launch argument).
    # The shipped config is authored for vehicle_name='bluerov2'. For any other
    # vehicle, rewrite the 'bluerov2' tokens (topic prefixes + TF frame ids) to
    # the requested vehicle_name in a temp copy and load that — equivalent to
    # nav2's ReplaceString, but with no extra dependency. At the default vehicle
    # the original file is loaded untouched (so opening it directly still works).
    rviz_config = os.path.join(pkg_share, 'rviz', 'stonefish_slam.rviz')
    if vehicle_name and vehicle_name != 'bluerov2':
        with open(rviz_config) as f:
            rewritten = f.read().replace('bluerov2', vehicle_name)
        tmp = tempfile.NamedTemporaryFile(
            mode='w', prefix=f'stonefish_slam_{vehicle_name}_', suffix='.rviz',
            delete=False)
        tmp.write(rewritten)
        tmp.close()
        rviz_config = tmp.name
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time.lower() == 'true'}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return [slam_node, world_ned_to_map_tf, rviz_node]


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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
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

    # Default comes from mapping.yaml so the config file is the single source
    # of truth for the update method (same resolution as
    # mapping_3d_standalone.launch.py); the argument still overrides it.
    mapping_yaml_path = os.path.join(
        get_package_share_directory('stonefish_slam'), 'config', 'mapping.yaml'
    )
    with open(mapping_yaml_path, 'r') as f:
        _mapping_config = yaml.safe_load(f)
    default_update_method = (
        _mapping_config.get('slam_node', {})
        .get('ros__parameters', {})
        .get('mapping_3d', {})
        .get('update_method', 'log_odds')
    )

    update_method_arg = DeclareLaunchArgument(
        'update_method',
        default_value=default_update_method,
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

    icp_config_file_arg = DeclareLaunchArgument(
        'icp_config_file',
        default_value='icp.yaml',
        description='ICP config file name under config/ (e.g. icp_real_bag.yaml)'
    )

    override_config_arg = DeclareLaunchArgument(
        'override_config',
        default_value='',
        description='Optional yaml path overriding config defaults '
                    '(e.g. config/real_bag_overrides.yaml). Empty = none.'
    )

    accuracy_monitor_arg = DeclareLaunchArgument(
        'enable_accuracy_monitor',
        default_value='true',
        description='Enable SLAM vs ground-truth accuracy monitor'
    )

    accuracy_log_every_n_arg = DeclareLaunchArgument(
        'accuracy_log_every_n',
        default_value='1',
        description='Accuracy monitor: log stats every N samples (0 disables periodic logs)'
    )

    traj_error_arg = DeclareLaunchArgument(
        'enable_traj_error_accumulator',
        default_value='true',
        description='Enable 2D error accumulator (SLAM pose vs ground truth)'
    )

    traj_error_log_every_n_arg = DeclareLaunchArgument(
        'traj_error_log_every_n',
        default_value='1',
        description='Trajectory 2D error: log every N samples (0 disables periodic logs)'
    )

    traj_error_pose_topic_arg = DeclareLaunchArgument(
        'traj_error_slam_pose_topic',
        default_value='/stonefish_slam/slam/pose',
        description='SLAM pose topic for 2D error accumulator'
    )

    traj_error_ground_truth_topic_arg = DeclareLaunchArgument(
        'traj_error_ground_truth_topic',
        default_value='',
        description='Ground truth topic (empty => /{vehicle_name}/odometry)'
    )

    # Evaluation observers (read-only; they never feed back into SLAM)
    accuracy_monitor_node = Node(
        package='stonefish_slam',
        executable='slam_accuracy_monitor',
        name='slam_accuracy_monitor',
        output='screen',
        parameters=[{
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'log_every_n': LaunchConfiguration('accuracy_log_every_n'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_accuracy_monitor'))
    )

    traj_error_node = Node(
        package='stonefish_slam',
        executable='traj_2d_error_accumulator',
        name='traj_2d_error_accumulator',
        output='screen',
        parameters=[{
            'vehicle_name': LaunchConfiguration('vehicle_name'),
            'slam_pose_topic': LaunchConfiguration('traj_error_slam_pose_topic'),
            'ground_truth_topic': LaunchConfiguration('traj_error_ground_truth_topic'),
            'log_every_n': LaunchConfiguration('traj_error_log_every_n'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_traj_error_accumulator'))
    )

    return LaunchDescription([
        # Arguments
        rviz_arg,
        vehicle_name_arg,
        use_sim_time_arg,
        mode_arg,
        enable_2d_mapping_arg,
        enable_3d_mapping_arg,
        update_method_arg,
        ssm_enable_arg,
        nssm_enable_arg,
        icp_config_file_arg,
        override_config_arg,
        accuracy_monitor_arg,
        accuracy_log_every_n_arg,
        traj_error_arg,
        traj_error_log_every_n_arg,
        traj_error_pose_topic_arg,
        traj_error_ground_truth_topic_arg,

        # Nodes (created via OpaqueFunction for conditional param handling)
        OpaqueFunction(function=launch_setup),
        accuracy_monitor_node,
        traj_error_node,
    ])
