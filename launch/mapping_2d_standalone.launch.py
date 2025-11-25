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
        name='mapping_2d_standalone_node',
        parameters=[
            str(config_dir / 'slam.yaml'),
            {
                'frame_interval': 1,
                'odom_topic': '/bluerov2/odometry',
                'sonar_topic': '/bluerov2/fls/image',
            }
        ],
        output='screen'
    )

    return LaunchDescription([mapping_2d_node])
