#!/usr/bin/env python3
"""SLAM launch — one node, one config file, three modes.

    ros2 launch stonefish_slam slam.launch.py                     # full SLAM: config/slam.yaml as is
    ros2 launch stonefish_slam slam.launch.py mode:=localization  # ICP on, loop closure + mapping off
    ros2 launch stonefish_slam slam.launch.py mode:=mapping       # scan matching off, maps from odometry

Parameter precedence, last wins:
    config/slam.yaml
    < config/mapping/method_<update_method>.yaml   (3D probability-update method)
    < override_config:=<profile.yaml>              (e.g. config/real_bag_overrides.yaml)
    < mode preset < explicit launch arguments (enable_2d_mapping, enable_3d_mapping, update_method)

mode:=localization → ICP on, loop closure off, no maps; mode:=mapping → no scan matching,
2D map on / 3D map off (what the deleted wrappers passed). Both default to rviz:=false.

Two read-only evaluation observers (SLAM-vs-ground-truth accuracy monitor and the
2D trajectory-error accumulator) start alongside the node unless evaluate:=false.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_SHARE = get_package_share_directory('stonefish_slam')
SLAM_YAML = os.path.join(PKG_SHARE, 'config', 'slam.yaml')

# What each mode overrides on top of config/slam.yaml. 'mode' is the node's own
# parameter (it gates the Localization module); the flags mirror what the old
# localization.launch.py / mapping.launch.py wrappers passed.
MODE_PRESETS = {
    'slam': {'mode': 'slam'},
    'localization': {
        'mode': 'localization-only',
        'ssm.enable': True,
        'nssm.enable': False,
        'enable_2d_mapping': False,
        'enable_3d_mapping': False,
    },
    'mapping': {
        'mode': 'mapping-only',
        'ssm.enable': False,
        'nssm.enable': False,
        'enable_2d_mapping': True,
        'enable_3d_mapping': False,
    },
}


def _yaml_default_update_method():
    """config/slam.yaml is the single source of truth for the 3D update method."""
    with open(SLAM_YAML) as fh:
        params = yaml.safe_load(fh)['/**']['ros__parameters']
    return params['mapping_3d']['update_method']


def _as_bool(text):
    return text.strip().lower() == 'true'


def launch_setup(context, *args, **kwargs):
    vehicle_name = LaunchConfiguration('vehicle_name').perform(context)
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODE_PRESETS:
        raise ValueError(f"mode:={mode!r} — expected one of {sorted(MODE_PRESETS)}")
    # The deleted localization/mapping wrappers defaulted to no RViz; keep that.
    rviz_arg = LaunchConfiguration('rviz').perform(context)
    rviz_on = _as_bool(rviz_arg) if rviz_arg else mode == 'slam'
    update_method = LaunchConfiguration('update_method').perform(context)
    override_config = LaunchConfiguration('override_config').perform(context)
    icp_config = os.path.join(
        PKG_SHARE, 'config', LaunchConfiguration('icp_config_file').perform(context))

    # Runtime overrides: mode preset first, explicit arguments on top (an empty
    # argument leaves the yaml value alone).
    param_dict = {
        'vehicle_name': vehicle_name,
        'use_sim_time': use_sim_time,
        'icp_config': icp_config,
        # The node declares mapping_3d.update_method; a bare 'update_method' is dropped.
        'mapping_3d.update_method': update_method,
        **MODE_PRESETS[mode],
    }
    for arg in ('enable_2d_mapping', 'enable_3d_mapping'):
        value = LaunchConfiguration(arg).perform(context)
        if value:
            param_dict[arg] = _as_bool(value)

    # The node parameter is nested, so it cannot ride the loop above.
    semantic = LaunchConfiguration('semantic').perform(context)
    if semantic:
        param_dict['semantic.enable'] = _as_bool(semantic)

    parameters = [
        SLAM_YAML,
        os.path.join(PKG_SHARE, 'config', 'mapping', f'method_{update_method}.yaml'),
    ]
    if override_config:
        parameters.append(override_config)
    parameters.append(param_dict)

    slam_node = Node(
        package='stonefish_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=parameters,
    )

    # Static TF: world_ned -> {vehicle_name}_map (identity)
    world_ned_to_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_ned_to_vehicle_map_tf_publisher',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world_ned',
            '--child-frame-id', f'{vehicle_name}_map',
        ],
    )

    # RViz. The shipped config is authored for vehicle_name='bluerov2'; for any
    # other vehicle rewrite the 'bluerov2' tokens (topic prefixes + TF frame ids)
    # into a temp copy — nav2's ReplaceString without the dependency.
    rviz_config = os.path.join(PKG_SHARE, 'rviz', 'stonefish_slam.rviz')
    if vehicle_name and vehicle_name != 'bluerov2':
        with open(rviz_config) as f:
            rewritten = f.read().replace('bluerov2', vehicle_name)
        tmp = tempfile.NamedTemporaryFile(
            mode='w', prefix=f'stonefish_slam_{vehicle_name}_', suffix='.rviz', delete=False)
        tmp.write(rewritten)
        tmp.close()
        rviz_config = tmp.name
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(str(rviz_on).lower()),
    )

    # Evaluation observers — read-only, never feed back into SLAM.
    observer_params = [{'vehicle_name': vehicle_name, 'use_sim_time': use_sim_time}]
    accuracy_monitor = Node(
        package='stonefish_slam',
        executable='slam_accuracy_monitor',
        name='slam_accuracy_monitor',
        output='screen',
        parameters=observer_params,
        condition=IfCondition(LaunchConfiguration('evaluate')),
    )
    traj_error = Node(
        package='stonefish_slam',
        executable='traj_2d_error_accumulator',
        name='traj_2d_error_accumulator',
        output='screen',
        parameters=observer_params,
        condition=IfCondition(LaunchConfiguration('evaluate')),
    )

    return [slam_node, world_ned_to_map_tf, rviz_node, accuracy_monitor, traj_error]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_name', default_value='bluerov2',
            description='Vehicle name: topic prefix /{vehicle_name}/… and TF frame ids'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true only when something publishes /clock (ros2 bag play --clock); '
                        'the Stonefish simulator does not'),
        DeclareLaunchArgument(
            'rviz', default_value='',
            description='true/false; empty = true for mode:=slam, false for localization/mapping '
                        '(what the old wrapper launches did)'),
        DeclareLaunchArgument(
            'mode', default_value='slam',
            description='slam | localization (ICP only, no loop closure, no maps) | '
                        'mapping (no scan matching, maps from odometry)'),
        DeclareLaunchArgument(
            'enable_2d_mapping', default_value='',
            description='true/false; empty = config/slam.yaml (or the mode preset)'),
        DeclareLaunchArgument(
            'enable_3d_mapping', default_value='',
            description='true/false; empty = config/slam.yaml (or the mode preset)'),
        DeclareLaunchArgument(
            'semantic', default_value='',
            description='true/false; empty = config/slam.yaml. true subscribes to the '
                        'detection topic, labels cloud points and adds landmark factors'),
        DeclareLaunchArgument(
            'update_method', default_value=_yaml_default_update_method(),
            description='3D probability update: log_odds | weighted_avg | iwlo '
                        '(loads config/mapping/method_<name>.yaml)'),
        DeclareLaunchArgument(
            'icp_config_file', default_value='icp.yaml',
            description='libpointmatcher chain under config/ (icp.yaml | icp_real_bag.yaml)'),
        DeclareLaunchArgument(
            'override_config', default_value='',
            description='Optional yaml profile loaded after config/slam.yaml '
                        '(e.g. config/real_bag_overrides.yaml)'),
        DeclareLaunchArgument(
            'evaluate', default_value='true',
            description='Start the accuracy monitor and 2D trajectory-error accumulator'),
        OpaqueFunction(function=launch_setup),
    ])
