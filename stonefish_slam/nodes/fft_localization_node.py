#!/usr/bin/env python3
"""
Independent FFT localization standalone node.

Subscribes to polar sonar images, runs FFT-based phase-correlation registration
between consecutive frames (reusing core.localization_fft.FFTLocalizer), and
publishes the estimated frame-to-frame transform as PoseWithCovarianceStamped.

This node ONLY publishes raw consecutive-frame transforms. It is an opt-in
standalone alternative to the FFT localization that is internal to slam_node;
slam_node does NOT consume this topic.

Unlike the integrated SLAM path, this node does NOT validate against odometry,
does NOT fall back to dead reckoning, and does NOT override rotation with DR
(all of those depend on the SLAM keyframe / factor-graph state, which is not
available in an independent node). Every incoming polar frame is compared to
the immediately preceding one via estimate_transform(prev, curr,
rotation_override=None); the result is published as-is.

Usage:
    ros2 run stonefish_slam fft_localization_node
    ros2 launch stonefish_slam fft_localization_standalone.launch.py
"""

import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped

from stonefish_slam.utils.sonar import OculusProperty
from stonefish_slam.core.localization_fft import FFTLocalizer


class FFTLocalizationNode(Node):
    """Standalone FFT localization node - sonar image -> frame-to-frame PoseWithCovarianceStamped."""

    def __init__(self):
        super().__init__('fft_localization_node')

        # Topic parameters
        self.declare_parameter('sonar_topic', '/bluerov2/fls/image')
        self.declare_parameter('pose_topic', '/fft_localization/transform')

        # FFT localizer parameters (defaults match config/slam.yaml: fft_localization.*).
        # Only the pure-estimation parameters are declared here; the SLAM validation
        # parameters (validate_with_odom, max_position_error, max_rotation_error,
        # min_ppr, reject_on_failure, use_dr_rotation) are intentionally omitted
        # because this node publishes raw transforms without odom validation.
        self.declare_parameter('fft_localization.range_min', 0.5)
        self.declare_parameter('fft_localization.verbose', False)
        self.declare_parameter('fft_localization.trans_erosion_iterations', 4)
        self.declare_parameter('fft_localization.trans_gaussian_sigma', 4.0)
        self.declare_parameter('fft_localization.trans_gaussian_truncate', 4.0)

        # Sonar parameters (defaults match config/sonar.yaml: sonar.*).
        self.declare_parameter('sonar.horizontal_fov', 130.0)
        self.declare_parameter('sonar.vertical_fov', 20.0)
        self.declare_parameter('sonar.num_beams', 512)
        self.declare_parameter('sonar.num_bins', 500)
        self.declare_parameter('sonar.range_max', 40.0)
        self.declare_parameter('sonar.sonar_tilt_deg', 30.0)

        sonar_topic = self.get_parameter('sonar_topic').value
        pose_topic = self.get_parameter('pose_topic').value

        # Assemble OculusProperty exactly as slam.py does (core/slam.py:245-251
        # for the 7 sonar fields, plus core/slam.py:394-397 for the tilt angle).
        # FFTLocalizer reads range_max, range_resolution, num_ranges, num_beams,
        # horizontal_fov, vertical_fov, angular_resolution, and tilt_angle_rad.
        sonar_range_max = self.get_parameter('sonar.range_max').value
        sonar_horizontal_fov = self.get_parameter('sonar.horizontal_fov').value
        sonar_vertical_fov = self.get_parameter('sonar.vertical_fov').value
        sonar_num_bins = self.get_parameter('sonar.num_bins').value
        sonar_num_beams = self.get_parameter('sonar.num_beams').value
        sonar_tilt_deg = self.get_parameter('sonar.sonar_tilt_deg').value

        oculus = OculusProperty(tilt_angle_deg=sonar_tilt_deg)
        oculus.range_max = sonar_range_max
        oculus.range_resolution = sonar_range_max / sonar_num_bins
        oculus.num_ranges = sonar_num_bins
        oculus.horizontal_fov = np.radians(sonar_horizontal_fov)
        oculus.vertical_fov = np.radians(sonar_vertical_fov)
        oculus.num_beams = sonar_num_beams
        oculus.angular_resolution = np.radians(sonar_horizontal_fov) / sonar_num_beams

        # Build the FFT localizer with the same estimation parameters slam.py uses
        # (core/slam.py:403-410).
        fft_range_min = self.get_parameter('fft_localization.range_min').value
        fft_verbose = self.get_parameter('fft_localization.verbose').value
        fft_erosion = self.get_parameter('fft_localization.trans_erosion_iterations').value
        fft_gaussian_sigma = self.get_parameter('fft_localization.trans_gaussian_sigma').value
        fft_gaussian_truncate = self.get_parameter('fft_localization.trans_gaussian_truncate').value
        self.fft_localizer = FFTLocalizer(
            oculus=oculus,
            range_min=fft_range_min,
            verbose=fft_verbose,
            trans_erosion_iterations=fft_erosion,
            trans_gaussian_sigma=fft_gaussian_sigma,
            trans_gaussian_truncate=fft_gaussian_truncate
        )

        # Previous polar sonar image (consecutive-frame pairing; first frame is skipped).
        self.prev_polar_sonar = None

        # Subscriber: polar sonar image only.
        # QoS must be BEST_EFFORT to match the simulator's FLS publisher; a RELIABLE
        # subscriber would never connect to a BEST_EFFORT publisher (see core/slam.py:434-438).
        qos_sub_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )
        self.get_logger().info(f'Subscribing to sonar topic: {sonar_topic}')
        self.create_subscription(Image, sonar_topic, self.sonar_callback, qos_sub_profile)

        # Publisher: frame-to-frame transform as PoseWithCovarianceStamped.
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)

        self.get_logger().info(
            f'FFT Localization Standalone Node ready (tilt={sonar_tilt_deg}deg)'
        )

    def sonar_callback(self, sonar_msg):
        """Estimate the transform from the previous frame to this one and publish it."""
        # Extract polar sonar image identically to slam.py:642-645.
        polar_curr = np.frombuffer(sonar_msg.data, dtype=np.uint8).reshape(
            sonar_msg.height, sonar_msg.width
        )

        # First frame: no previous to compare against; store and skip.
        if self.prev_polar_sonar is None:
            self.prev_polar_sonar = polar_curr.copy()
            return

        try:
            fft_result = self.fft_localizer.estimate_transform(
                self.prev_polar_sonar.copy(), polar_curr.copy(),
                rotation_override=None
            )
        except Exception as e:
            self.get_logger().error(f'FFT localization failed: {e}\n{traceback.format_exc()}')
            self.prev_polar_sonar = polar_curr.copy()
            return

        # Advance the reference frame every frame (raw consecutive-frame pairing).
        self.prev_polar_sonar = polar_curr.copy()

        # success=False -> skip publishing (raw publishing has nothing to emit).
        if not fft_result['success']:
            self.get_logger().warn('FFT estimate unsuccessful; skipping publish',
                                   throttle_duration_sec=2.0)
            return

        self.publish_transform(fft_result, sonar_msg.header.stamp)

    def publish_transform(self, fft_result, stamp):
        """Map an FFT result dict to PoseWithCovarianceStamped and publish."""
        tx, ty = fft_result['translation'][0], fft_result['translation'][1]
        yaw = np.radians(fft_result['rotation'])  # rotation is in degrees

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'world_ned'

        pose_msg.pose.pose.position.x = float(tx)
        pose_msg.pose.pose.position.y = float(ty)
        pose_msg.pose.pose.position.z = 0.0

        # Yaw-only quaternion (roll=pitch=0): qz=sin(yaw/2), qw=cos(yaw/2).
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = float(np.sin(yaw / 2.0))
        pose_msg.pose.pose.orientation.w = float(np.cos(yaw / 2.0))

        # Map the 3x3 FFT covariance (x, y, theta order) into the 6x6
        # PoseWithCovariance covariance (x, y, z, roll, pitch, yaw order).
        # Rows/cols (0, 1, 5) correspond to (x, y, yaw); same placement
        # slam.py:899 uses for the SLAM transform covariance.
        # This is a 2-DOF + yaw planar estimate, so z/roll/pitch carry no
        # information: give them a large variance (1e6) so a downstream fuser
        # (e.g. an EKF) ignores them rather than treating them as ~0 (a small
        # default like slam.py's 1e-4 would wrongly assert confident near-zero).
        cov6 = np.diag(np.array([1e6, 1e6, 1e6, 1e6, 1e6, 1e6], dtype=np.float32))
        fft_cov = fft_result.get('covariance', None)
        if fft_cov is not None:
            cov6[np.ix_((0, 1, 5), (0, 1, 5))] = fft_cov
        pose_msg.pose.covariance = cov6.ravel().tolist()

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FFTLocalizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
