#!/usr/bin/env python3
"""
Independent 3D mapping standalone node.

Subscribes to odometry and sonar images directly (time-synchronized),
processes every Nth frame for 3D occupancy mapping.

Usage:
    ros2 run stonefish_slam mapping_3d_standalone
    ros2 launch stonefish_slam mapping_3d_standalone.launch.py
"""

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np

from stonefish_slam.core.mapping_3d import SonarMapping3D


class Mapping3DStandaloneNode(Node):
    """3D Mapping Standalone Node - Time-synchronized sonar + odometry processing"""

    def __init__(self):
        super().__init__('mapping_3d_standalone_node')

        # Declare parameters
        self.declare_parameter('resolution', 0.3)
        self.declare_parameter('frame_interval', 10)
        self.declare_parameter('odom_topic', '/bluerov2/odometry')
        self.declare_parameter('sonar_topic', '/bluerov2/fls/image')

        # Sonar hardware parameters (from slam.yaml sonar section)
        self.declare_parameter('sonar.max_range', 40.0)
        self.declare_parameter('sonar.min_range', 0.5)
        self.declare_parameter('sonar.horizontal_fov', 130.0)
        self.declare_parameter('sonar.vertical_aperture', 20.0)
        self.declare_parameter('sonar.image_width', 918)
        self.declare_parameter('sonar.image_height', 512)
        self.declare_parameter('sonar.sonar_position', [0.25, 0.0, 0.08])
        self.declare_parameter('sonar.sonar_tilt_deg', 10.0)

        # 3D Mapping parameters (from slam.yaml mapping_3d section)
        self.declare_parameter('mapping_3d.voxel_resolution', 0.2)
        self.declare_parameter('mapping_3d.min_probability', 0.6)
        self.declare_parameter('mapping_3d.log_odds_occupied', 1.5)
        self.declare_parameter('mapping_3d.log_odds_free', -2.0)
        self.declare_parameter('mapping_3d.log_odds_min', -10.0)
        self.declare_parameter('mapping_3d.log_odds_max', 10.0)
        self.declare_parameter('mapping_3d.adaptive_update', True)
        self.declare_parameter('mapping_3d.adaptive_threshold', 0.5)
        self.declare_parameter('mapping_3d.adaptive_max_ratio', 0.5)
        self.declare_parameter('mapping_3d.use_cpp_backend', True)
        self.declare_parameter('mapping_3d.enable_propagation', False)
        self.declare_parameter('mapping_3d.use_range_weighting', True)
        self.declare_parameter('mapping_3d.lambda_decay', 0.1)
        self.declare_parameter('mapping_3d.enable_gaussian_weighting', False)
        self.declare_parameter('mapping_3d.use_dda_traversal', True)
        self.declare_parameter('mapping_3d.bearing_step', 2)

        # Additional parameters
        self.declare_parameter('intensity_threshold', 50)
        self.declare_parameter('max_frames', 0)  # 0 = unlimited (was 1000, caused hang at frame #1300)
        self.declare_parameter('dynamic_expansion', True)
        self.declare_parameter('gaussian_sigma_factor', 2.5)
        self.declare_parameter('propagation_radius', 2)
        self.declare_parameter('propagation_sigma', 1.5)
        self.declare_parameter('enable_profiling', True)
        self.declare_parameter('use_cpp_ray_processor', True)

        # Get parameters
        resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.frame_interval = self.get_parameter('frame_interval').get_parameter_value().integer_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        sonar_topic = self.get_parameter('sonar_topic').get_parameter_value().string_value

        # Build config from parameters (matching slam_ros.py structure)
        config = {
            # Node-level parameters
            'frame_interval': self.frame_interval,
            # Sonar hardware (from sonar.* parameters)
            'max_range': self.get_parameter('sonar.max_range').value,
            'min_range': self.get_parameter('sonar.min_range').value,
            'horizontal_fov': self.get_parameter('sonar.horizontal_fov').value,
            'vertical_aperture': self.get_parameter('sonar.vertical_aperture').value,
            'image_width': self.get_parameter('sonar.image_width').value,
            'image_height': self.get_parameter('sonar.image_height').value,
            'sonar_position': self.get_parameter('sonar.sonar_position').value,
            'sonar_tilt_deg': self.get_parameter('sonar.sonar_tilt_deg').value,

            # 3D mapping (from mapping_3d.* parameters)
            'voxel_resolution': resolution,  # Use test-specific override
            'min_probability': self.get_parameter('mapping_3d.min_probability').value,
            'intensity_threshold': self.get_parameter('intensity_threshold').value,
            'max_frames': self.get_parameter('max_frames').value,
            'dynamic_expansion': self.get_parameter('dynamic_expansion').value,
            'log_odds_occupied': self.get_parameter('mapping_3d.log_odds_occupied').value,
            'log_odds_free': self.get_parameter('mapping_3d.log_odds_free').value,
            'log_odds_min': self.get_parameter('mapping_3d.log_odds_min').value,
            'log_odds_max': self.get_parameter('mapping_3d.log_odds_max').value,
            'adaptive_update': self.get_parameter('mapping_3d.adaptive_update').value,
            'adaptive_threshold': self.get_parameter('mapping_3d.adaptive_threshold').value,
            'adaptive_max_ratio': self.get_parameter('mapping_3d.adaptive_max_ratio').value,
            'use_cpp_backend': self.get_parameter('mapping_3d.use_cpp_backend').value,
            'use_cpp_ray_processor': self.get_parameter('use_cpp_ray_processor').value,
            'use_range_weighting': self.get_parameter('mapping_3d.use_range_weighting').value,
            'lambda_decay': self.get_parameter('mapping_3d.lambda_decay').value,
            'enable_gaussian_weighting': self.get_parameter('mapping_3d.enable_gaussian_weighting').value,
            'gaussian_sigma_factor': self.get_parameter('gaussian_sigma_factor').value,
            'bearing_step': self.get_parameter('mapping_3d.bearing_step').value,
            'use_dda_traversal': self.get_parameter('mapping_3d.use_dda_traversal').value,
            'enable_propagation': self.get_parameter('mapping_3d.enable_propagation').value,
            'propagation_radius': self.get_parameter('propagation_radius').value,
            'propagation_sigma': self.get_parameter('propagation_sigma').value,
            'enable_profiling': self.get_parameter('enable_profiling').value,
        }

        self.mapper_3d = SonarMapping3D(config)
        self.get_logger().info(
            f'Mapper initialized: resolution={resolution}m, '
            f'max_range={config["max_range"]}m, tilt={config["sonar_tilt_deg"]}°'
        )

        # Frame counter
        self.frame_count = 0

        # CV bridge
        self.bridge = CvBridge()

        # Time-synchronized subscribers
        self.get_logger().info(f'Subscribing to: {odom_topic}, {sonar_topic}')
        odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)
        sonar_sub = message_filters.Subscriber(self, Image, sonar_topic)

        # Approximate time sync (1 second tolerance, larger queue)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, sonar_sub],
            queue_size=50,
            slop=1.0
        )
        self.ts.registerCallback(self.sync_callback)

        # Publisher
        self.pc_pub = self.create_publisher(PointCloud2, '/mapping_test/pointcloud', 10)

        self.get_logger().info(
            f'3D Mapping Test Node ready: '
            f'resolution={resolution}m, frame_interval={self.frame_interval}'
        )

    def sync_callback(self, odom_msg, sonar_msg):
        """Time-synchronized callback for odometry + sonar"""

        self.frame_count += 1

        # Debug: First callback confirmation
        if self.frame_count == 1:
            self.get_logger().info('First synchronized message received!')

        # Frame sampling: only process every Nth frame
        if self.frame_count % self.frame_interval != 0:
            return

        self.get_logger().info(f'Processing frame #{self.frame_count}')

        try:
            # Convert sonar image
            sonar_image = self.bridge.imgmsg_to_cv2(sonar_msg, desired_encoding='mono8')

            # Process with mapper
            self.mapper_3d.process_sonar_image(sonar_image, odom_msg.pose.pose)

            # Publish point cloud
            pc_msg = self.mapper_3d.get_pointcloud2_msg(
                frame_id='world_ned',
                stamp=self.get_clock().now().to_msg()
            )

            if pc_msg.width > 0:
                self.pc_pub.publish(pc_msg)
                self.get_logger().info(
                    f'Published {pc_msg.width} points, '
                    f'{self.mapper_3d.get_voxel_count()} total voxels'
                )
            else:
                self.get_logger().warn(
                    f'Empty point cloud (frame #{self.frame_count})'
                )

        except Exception as e:
            import traceback
            self.get_logger().error(f'Processing failed: {e}\n{traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)
    node = Mapping3DStandaloneNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
