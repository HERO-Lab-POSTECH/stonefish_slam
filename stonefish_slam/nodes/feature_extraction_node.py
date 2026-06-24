#!/usr/bin/env python3
"""
Independent feature extraction standalone node.

Subscribes to polar sonar images, runs CFAR-based feature extraction
(reusing core.feature_extraction.FeatureExtraction), and publishes the
detected cartesian feature points as a PointCloud2 (z=0.0).

This node ONLY publishes features. It is an opt-in standalone alternative
to the feature extraction that is internal to slam_node; slam_node does
NOT consume this topic.

Usage:
    ros2 run stonefish_slam feature_extraction_node
    ros2 launch stonefish_slam feature_extraction_standalone.launch.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2 as pc2

from stonefish_slam.core.feature_extraction import FeatureExtraction


class FeatureExtractionNode(Node):
    """Standalone feature extraction node - sonar image -> feature PointCloud2."""

    def __init__(self):
        super().__init__('feature_extraction_node')

        # Topic parameters
        self.declare_parameter('sonar_topic', '/bluerov2/fls/image')
        self.declare_parameter('feature_topic', '/feature_extraction/points')

        # Vehicle name (read by FeatureExtraction.init_params)
        self.declare_parameter('vehicle_name', 'bluerov2')

        # CFAR parameters (defaults match config/feature.yaml: CFAR.*)
        self.declare_parameter('CFAR.Ntc', 20)
        self.declare_parameter('CFAR.Ngc', 10)
        self.declare_parameter('CFAR.Pfa', 0.01)
        self.declare_parameter('CFAR.rank', 10)
        self.declare_parameter('CFAR.alg', 'SOCA')

        # Filter parameters (defaults match config/feature.yaml: filter.*)
        self.declare_parameter('filter.threshold', 80)
        self.declare_parameter('filter.resolution', 0.5)
        self.declare_parameter('filter.radius', 1.0)
        self.declare_parameter('filter.min_points', 5)
        self.declare_parameter('filter.skip', 5)

        # Visualization parameters (defaults match config/feature.yaml: visualization.*)
        self.declare_parameter('visualization.coordinates', 'cartesian')
        self.declare_parameter('visualization.radius', 2.0)
        self.declare_parameter('visualization.color', 'green')

        # Sonar parameters (defaults match config/sonar.yaml: sonar.*)
        self.declare_parameter('sonar.horizontal_fov', 130.0)
        self.declare_parameter('sonar.vertical_fov', 20.0)
        self.declare_parameter('sonar.num_beams', 512)
        self.declare_parameter('sonar.num_bins', 500)
        self.declare_parameter('sonar.range_min', 0.5)
        self.declare_parameter('sonar.range_max', 40.0)
        self.declare_parameter('sonar.sonar_tilt_deg', 30.0)

        sonar_topic = self.get_parameter('sonar_topic').value
        feature_topic = self.get_parameter('feature_topic').value

        # Reuse the shared feature extraction module (borrows this node's
        # ROS context for logging/parameters; no data coupling).
        self.feature_extractor = FeatureExtraction(self)

        # Subscriber: polar sonar image only (extract_features needs the image alone).
        # QoS must be BEST_EFFORT to match the simulator's FLS publisher; a RELIABLE
        # subscriber would never connect to a BEST_EFFORT publisher (see core/slam.py:434-438).
        qos_sub_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )
        self.get_logger().info(f'Subscribing to sonar topic: {sonar_topic}')
        self.create_subscription(Image, sonar_topic, self.sonar_callback, qos_sub_profile)

        # Publisher: feature points as PointCloud2.
        self.feature_pub = self.create_publisher(PointCloud2, feature_topic, 10)

        self.get_logger().info('Feature Extraction Standalone Node ready')

    def sonar_callback(self, sonar_msg):
        """Extract features from a sonar image and publish them as PointCloud2."""
        try:
            points = self.feature_extractor.extract_features(sonar_msg)
        except Exception as e:
            import traceback
            self.get_logger().error(f'Feature extraction failed: {e}\n{traceback.format_exc()}')
            return

        # points: (N, 2) cartesian [x, y]. Add z=0.0 -> (N, 3).
        if points.shape[0] == 0:
            points_xyz = np.zeros((0, 3), dtype=np.float32)
        else:
            points_xyz = np.zeros((points.shape[0], 3), dtype=np.float32)
            points_xyz[:, 0:2] = points

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = sonar_msg.header.frame_id

        cloud_msg = pc2.create_cloud_xyz32(header, points_xyz)
        self.feature_pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FeatureExtractionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
