from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    pkg_share = Path(get_package_share_directory('stonefish_slam'))
    config_dir = pkg_share / 'config'

    mapping_2d_node = Node(
        package='stonefish_slam',
        executable='mapping_2d_standalone',
        name='slam_node',  # Must match yaml namespace (slam_node.ros__parameters)
        parameters=[
            str(config_dir / 'sonar.yaml'),    # Sonar parameters
            str(config_dir / 'mapping.yaml'),  # Mapping parameters
            str(config_dir / 'slam.yaml'),     # General SLAM params
            {
                'frame_interval': 10,
                'odom_topic': '/bluerov2/odometry',
                'sonar_topic': '/bluerov2/fls/image',
            }
        ],
        output='screen'
    )

    return LaunchDescription([mapping_2d_node])
