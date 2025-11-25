#!/usr/bin/env python3
"""
Independent 3D mapping test node.

Subscribes to odometry and sonar images directly (time-synchronized),
processes every 5th frame for testing purposes.

Usage:
    ros2 run stonefish_slam mapping_3d_test
    ros2 launch stonefish_slam mapping_3d_test.launch.py
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


class MappingTestNode(Node):
    """3D Mapping Test Node - Time-synchronized sonar + odometry processing"""

    def __init__(self):
        super().__init__('mapping_3d_test_node')

        # Parameters
        self.declare_parameter('resolution', 0.3)
        self.declare_parameter('frame_interval', 5)
        self.declare_parameter('odom_topic', '/bluerov2/odometry')
        self.declare_parameter('sonar_topic', '/bluerov2/fls/image')

        resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.frame_interval = self.get_parameter('frame_interval').get_parameter_value().integer_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        sonar_topic = self.get_parameter('sonar_topic').get_parameter_value().string_value

        # Initialize mapper
        config = {
            'voxel_resolution': resolution,
            'use_cpp_backend': True,
            'horizontal_fov': 130.0,
            'vertical_aperture': 20.0,
            'min_range': 0.5,
            'max_range': 40.0,
            'image_height': 500,
            'image_width': 512,
            'intensity_threshold': 50,
        }

        self.mapper_3d = SonarMapping3D(config)
        self.get_logger().info(f'Mapper initialized with resolution={resolution}m')

        # Frame counter
        self.frame_count = 0

        # CV bridge
        self.bridge = CvBridge()

        # Time-synchronized subscribers
        self.get_logger().info(f'Subscribing to: {odom_topic}, {sonar_topic}')
        odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)
        sonar_sub = message_filters.Subscriber(self, Image, sonar_topic)

        # Approximate time sync (100ms tolerance)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, sonar_sub],
            queue_size=10,
            slop=0.1
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
    node = MappingTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
