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
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

from stonefish_slam.core.mapping_3d import SonarMapping3D


class Mapping3DStandaloneNode(Node):
    """3D Mapping Standalone Node - Time-synchronized sonar + odometry processing"""

    def __init__(self):
        super().__init__('slam_node')  # Use same namespace as slam.yaml

        # Declare parameters
        self.declare_parameter('resolution', 0.2)  # Test 3: 0.3 → 0.2
        self.declare_parameter('frame_interval', 10)
        self.declare_parameter('odom_topic', '/bluerov2/odometry')
        self.declare_parameter('sonar_topic', '/bluerov2/fls/image')

        # Sonar hardware parameters (defaults match sonar.yaml)
        self.declare_parameter('sonar.range_max', 15.0)
        self.declare_parameter('sonar.range_min', 0.5)
        self.declare_parameter('sonar.horizontal_fov', 130.0)
        self.declare_parameter('sonar.vertical_fov', 20.0)
        self.declare_parameter('sonar.num_beams', 512)
        self.declare_parameter('sonar.num_bins', 500)
        self.declare_parameter('sonar.sonar_position', [0.0, 0.0, 0.0])
        self.declare_parameter('sonar.sonar_tilt_deg', 10.0)

        # 3D Mapping parameters (from slam.yaml mapping_3d section)
        self.declare_parameter('mapping_3d.map_3d_voxel_size', 0.2)
        self.declare_parameter('mapping_3d.min_probability', 0.6)
        self.declare_parameter('mapping_3d.log_odds_occupied', 1.5)
        self.declare_parameter('mapping_3d.log_odds_free', -2.0)
        self.declare_parameter('mapping_3d.log_odds_min', -10.0)
        self.declare_parameter('mapping_3d.log_odds_max', 10.0)
        self.declare_parameter('mapping_3d.adaptive_update', True)
        self.declare_parameter('mapping_3d.adaptive_threshold', 0.5)
        self.declare_parameter('mapping_3d.adaptive_max_ratio', 0.5)
        self.declare_parameter('mapping_3d.use_cpp_backend', False)  # Test 2: Python backend for adaptive test
        self.declare_parameter('mapping_3d.enable_propagation', False)
        self.declare_parameter('mapping_3d.use_range_weighting', True)
        self.declare_parameter('mapping_3d.lambda_decay', 0.1)
        self.declare_parameter('mapping_3d.enable_gaussian_weighting', False)
        self.declare_parameter('mapping_3d.use_dda_traversal', True)
        self.declare_parameter('mapping_3d.bearing_step', 2)

        # IWLO parameters
        self.declare_parameter('mapping_3d.sharpness', 1.0)
        self.declare_parameter('mapping_3d.decay_rate', 0.05)
        self.declare_parameter('mapping_3d.min_alpha', 0.3)
        self.declare_parameter('mapping_3d.update_method', 'iwlo')

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
            'range_max': self.get_parameter('sonar.range_max').value,
            'range_min': self.get_parameter('sonar.range_min').value,
            'horizontal_fov': self.get_parameter('sonar.horizontal_fov').value,
            'vertical_fov': self.get_parameter('sonar.vertical_fov').value,
            'num_beams': self.get_parameter('sonar.num_beams').value,
            'num_bins': self.get_parameter('sonar.num_bins').value,
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
            # IWLO parameters
            'sharpness': self.get_parameter('mapping_3d.sharpness').value,
            'decay_rate': self.get_parameter('mapping_3d.decay_rate').value,
            'min_alpha': self.get_parameter('mapping_3d.min_alpha').value,
            'update_method': self.get_parameter('mapping_3d.update_method').value,
        }

        # Debug: Check parameter loading
        self.get_logger().info(
            f'DEBUG: mapping_3d.use_cpp_backend={self.get_parameter("mapping_3d.use_cpp_backend").value}, '
            f'use_cpp_ray_processor={self.get_parameter("use_cpp_ray_processor").value}'
        )

        self.mapper_3d = SonarMapping3D(config)
        self.get_logger().info(
            f'Mapper initialized: resolution={resolution}m, '
            f'sonar_tilt={config["sonar_tilt_deg"]}°, '
            f'range_max={config["range_max"]}m, '
            f'adaptive_max_ratio={config["adaptive_max_ratio"]}'
        )

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
            queue_size=20,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, '/mapping_3d_standalone/pointcloud', 10)
        self.threshold_debug_pub = self.create_publisher(
            Image,
            '/mapping_3d_standalone/threshold_debug',
            10
        )

        # Log C++ backend status
        self.get_logger().info(
            f'C++ Backend: use_cpp_backend={self.mapper_3d.use_cpp_backend}, '
            f'use_cpp_ray_processor={self.mapper_3d.use_cpp_ray_processor}, '
            f'cpp_ray_processor={self.mapper_3d.cpp_ray_processor is not None}'
        )

        self.get_logger().info(
            f'3D Mapping Test Node ready: '
            f'resolution={resolution}m, frame_interval={self.frame_interval}'
        )

    def sync_callback(self, odom_msg, sonar_msg):
        """Time-synchronized callback for odometry + sonar"""

        self.frame_count += 1

        # Debug: First callback confirmation + time sync check
        if self.frame_count == 1:
            odom_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
            sonar_time = sonar_msg.header.stamp.sec + sonar_msg.header.stamp.nanosec * 1e-9
            time_diff = abs(odom_time - sonar_time)
            self.get_logger().info(
                f'First synchronized message received! Time diff: {time_diff*1000:.1f}ms'
            )

        # Frame sampling: only process every Nth frame
        if self.frame_count % self.frame_interval != 0:
            return

        self.get_logger().info(f'Processing frame #{self.frame_count}')

        try:
            # Convert sonar image
            sonar_image = self.bridge.imgmsg_to_cv2(sonar_msg, desired_encoding='mono8')

            # Process with mapper (맵 업데이트 먼저)
            self.mapper_3d.process_sonar_image(sonar_image, odom_msg.pose.pose)

            # --- Threshold debugging image generation (맵 업데이트 후) ---
            threshold_value = self.mapper_3d.intensity_threshold

            # Black background (B, G, R)
            threshold_debug = np.zeros((sonar_image.shape[0], sonar_image.shape[1], 3), dtype=np.uint8)

            # Color code each bearing column
            for bearing_idx in range(sonar_image.shape[1]):
                intensity_column = sonar_image[:, bearing_idx]

                # Bottom→Top scan (near→far, row max → row 0)
                # Find first hit where intensity > threshold
                first_hit_idx = -1
                for row_idx in range(len(intensity_column) - 1, -1, -1):
                    if intensity_column[row_idx] > threshold_value:
                        first_hit_idx = row_idx
                        break

                if first_hit_idx >= 0:
                    # Green: Free space (bottom to first_hit)
                    # 실제 업데이트와 일치: first_hit보다 1 row 아래까지만 free로 마킹
                    free_end = min(first_hit_idx + 1, len(intensity_column) - 1)
                    for row_idx in range(len(intensity_column) - 1, free_end, -1):
                        threshold_debug[row_idx, bearing_idx] = [0, 255, 0]  # Green (BGR)

                    # Red: Occupied (first_hit and beyond where intensity > threshold)
                    for row_idx in range(first_hit_idx, -1, -1):
                        if intensity_column[row_idx] > threshold_value:
                            threshold_debug[row_idx, bearing_idx] = [0, 0, 255]  # Red (BGR)
                        # else: Black (shadow) - already initialized
                else:
                    # No hit: entire range is free space (simulator environment)
                    # ray_processor.cpp line 264-266: range_to_first_hit = range_max
                    threshold_debug[:, bearing_idx] = [0, 255, 0]  # Full column Green

            # ROS2 Image 메시지로 변환 및 publish (맵 업데이트와 동기화)
            try:
                threshold_msg = self.bridge.cv2_to_imgmsg(threshold_debug, encoding='bgr8')
                threshold_msg.header = sonar_msg.header
                self.threshold_debug_pub.publish(threshold_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'CV Bridge Error (threshold debug): {e}')

            # Publish point cloud
            pc_msg = self.mapper_3d.get_pointcloud2_msg(
                frame_id='world_ned',
                stamp=self.get_clock().now().to_msg()
            )

            if pc_msg.width > 0:
                self.pc_pub.publish(pc_msg)

                # DEBUG: Check voxel z range vs robot z
                result = self.mapper_3d.get_point_cloud(include_free=False)
                if len(result['points']) > 0:
                    z_min = result['points'][:, 2].min()
                    z_max = result['points'][:, 2].max()
                    robot_z = odom_msg.pose.pose.position.z
                    self.get_logger().info(
                        f'Published {pc_msg.width} points, {self.mapper_3d.get_voxel_count()} total voxels | '
                        f'Robot z={robot_z:.2f}m, Voxel z=[{z_min:.2f}, {z_max:.2f}]m'
                    )
                else:
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
