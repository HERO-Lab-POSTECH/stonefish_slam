#!/usr/bin/env python3
"""
Independent 2D mapping standalone node.

Subscribes to odometry and sonar images (time-synchronized),
generates 2D occupancy grid map using SonarMapping2D keyframe interface.

Usage:
    ros2 run stonefish_slam mapping_2d_standalone
    ros2 launch stonefish_slam mapping_2d_standalone.launch.py
"""

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry
from cv_bridge import CvBridge
import numpy as np
import gtsam

from stonefish_slam.core.mapping_2d import SonarMapping2D


class Mapping2DStandaloneNode(Node):
    """2D Mapping Standalone Node - Time-synchronized sonar + odometry"""

    def __init__(self):
        super().__init__('slam_node')

        # Declare parameters
        self.declare_parameter('frame_interval', 1)
        self.declare_parameter('odom_topic', '/bluerov2/odometry')
        self.declare_parameter('sonar_topic', '/bluerov2/fls/image')

        # Mapping 2D parameters
        self.declare_parameter('map_2d_resolution', 0.1)
        self.declare_parameter('map_size', [4000, 4000])
        self.declare_parameter('intensity_threshold', 50)

        # Sonar parameters
        self.declare_parameter('sonar.range_max', 20.0)
        self.declare_parameter('sonar.range_min', 0.5)
        self.declare_parameter('sonar.horizontal_fov', 130.0)
        self.declare_parameter('sonar.num_beams', 918)
        self.declare_parameter('sonar.num_bins', 512)
        self.declare_parameter('sonar.sonar_position', [0.25, 0.0, 0.08])
        self.declare_parameter('sonar.sonar_tilt_deg', 10.0)

        # Get parameters
        self.frame_interval = self.get_parameter('frame_interval').value
        odom_topic = self.get_parameter('odom_topic').value
        sonar_topic = self.get_parameter('sonar_topic').value

        map_resolution = self.get_parameter('map_2d_resolution').value
        map_size = self.get_parameter('map_size').value
        sonar_range = self.get_parameter('sonar.range_max').value
        sonar_fov = self.get_parameter('sonar.horizontal_fov').value
        range_min = self.get_parameter('sonar.range_min').value
        sonar_tilt_deg = self.get_parameter('sonar.sonar_tilt_deg').value
        intensity_threshold = self.get_parameter('intensity_threshold').value

        # Initialize mapper with SonarMapping2D signature
        self.mapper_2d = SonarMapping2D(
            map_resolution=map_resolution,
            map_size=tuple(map_size),
            sonar_range=sonar_range,
            sonar_fov=sonar_fov,
            fan_pixel_resolution=0.05,
            sonar_tilt_deg=sonar_tilt_deg,
            range_min=range_min,
            keyframe_sample_threshold=50,
            intensity_threshold=intensity_threshold,
            ros_logger=self.get_logger()
        )
        self.get_logger().info(
            f'2D Mapper initialized: resolution={map_resolution}m, '
            f'range={sonar_range}m, tilt={sonar_tilt_deg}°'
        )

        # Frame counter for keyframe ID
        self.frame_count = 0

        # CV bridge
        self.bridge = CvBridge()

        # Time-synchronized subscribers
        self.get_logger().info(f'Subscribing to: {odom_topic}, {sonar_topic}')
        odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)
        sonar_sub = message_filters.Subscriber(self, Image, sonar_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [odom_sub, sonar_sub],
            queue_size=50,
            slop=1.0
        )
        self.ts.registerCallback(self.sync_callback)

        # Publisher (Image for now, OccupancyGrid conversion TODO)
        self.map_pub = self.create_publisher(Image, '/map_2d', 10)

        self.get_logger().info('2D Mapping Standalone Node ready')

    def sync_callback(self, odom_msg, sonar_msg):
        """Time-synchronized callback"""
        self.frame_count += 1

        if self.frame_count % self.frame_interval != 0:
            return

        self.get_logger().info(f'Processing frame #{self.frame_count}')

        try:
            # Convert sonar image
            sonar_image = self.bridge.imgmsg_to_cv2(sonar_msg, 'mono8')

            # Convert Odometry Pose to gtsam.Pose2 (2D: x, y, yaw)
            pose_msg = odom_msg.pose.pose
            x = pose_msg.position.x
            y = pose_msg.position.y

            # Extract yaw from quaternion
            from tf_transformations import euler_from_quaternion
            q = pose_msg.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            pose_2d = gtsam.Pose2(x, y, yaw)

            # Add keyframe with unique key
            self.mapper_2d.add_keyframe(self.frame_count, pose_2d, sonar_image)

            # Update global map (standalone mode - no SLAM integration)
            self.mapper_2d.update_global_map()

            # Get map image and publish as Image message
            map_image = self.mapper_2d.get_map_image()
            if map_image is not None:
                map_msg = self.bridge.cv2_to_imgmsg(map_image, encoding='mono8')
                map_msg.header.stamp = self.get_clock().now().to_msg()
                map_msg.header.frame_id = 'world_ned'
                self.map_pub.publish(map_msg)
                self.get_logger().info(f'Published 2D map (frame #{self.frame_count})')

        except Exception as e:
            import traceback
            self.get_logger().error(f'Processing failed: {e}\n{traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)
    node = Mapping2DStandaloneNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
