#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Image
import cv_bridge
import time

from stonefish_slam.utils.io import *
from stonefish_slam.utils.topics import *
from stonefish_slam.utils.conversions import *
from stonefish_slam.utils.visualization import apply_custom_colormap
# from stonefish_slam import pcl  # TODO: Re-enable when pybind11 is fixed
import matplotlib.pyplot as plt

from stonefish_slam.core.cfar import CFAR


class FeatureExtraction:
    '''Feature extraction module using CFAR for sonar images.

    Designed as a composable module (not a Node).
    Instantiated by a parent node (e.g., SLAM node).
    '''

    def __init__(self, parent_node):
        """Initialize feature extraction module.

        Args:
            parent_node: Parent ROS2 node that provides logging and parameter access
        """
        self.node = parent_node
        self.node.get_logger().info("Feature extraction module initializing...")

        # Default parameters for CFAR
        self.Ntc = 20
        self.Ngc = 10
        self.Pfa = 1e-2
        self.rank = None
        self.alg = "SOCA"
        self.detector = None
        self.threshold = 0

        # Default parameters for point cloud
        self.resolution = 0.5
        self.outlier_filter_radius = 1.0
        self.outlier_filter_min_points = 5
        self.skip = 5

        # Stonefish FLS sonar parameters (from bluerov2.scn)
        self.horizontal_fov = 130.0  # degrees
        self.vertical_fov = 20.0  # degrees
        self.num_beams = 512  # number of beams (columns)
        self.num_bins = 500  # number of range bins (rows)
        self.range_min = 0.5  # meters
        self.range_max = 20.0  # meters

        # Frame counter for skip
        self.frame_count = 0

        # Cache for polar_to_cartesian conversion (performance optimization)
        self.p2c_cache = None

        # CV bridge
        self.BridgeInstance = cv_bridge.CvBridge()

        # Initialize parameters
        self.init_params()

    def configure(self):
        '''Calls the CFAR class constructor for the featureExtraction class
        '''
        self.detector = CFAR(self.Ntc, self.Ngc, self.Pfa, self.rank)

    def init_params(self):
        """Initialize parameters from parent node (parameters must be already declared by parent)"""

        # Get parameters (parent node MUST have declared these already)
        self.vehicle_name = self.node.get_parameter('vehicle_name').get_parameter_value().string_value

        # CFAR parameters
        self.Ntc = self.node.get_parameter('CFAR.Ntc').get_parameter_value().integer_value
        self.Ngc = self.node.get_parameter('CFAR.Ngc').get_parameter_value().integer_value
        self.Pfa = self.node.get_parameter('CFAR.Pfa').get_parameter_value().double_value
        self.rank = self.node.get_parameter('CFAR.rank').get_parameter_value().integer_value
        self.alg = self.node.get_parameter('CFAR.alg').get_parameter_value().string_value

        # Filter parameters
        self.threshold = self.node.get_parameter('filter.threshold').get_parameter_value().integer_value
        self.resolution = self.node.get_parameter('filter.resolution').get_parameter_value().double_value
        self.outlier_filter_radius = self.node.get_parameter('filter.radius').get_parameter_value().double_value
        self.outlier_filter_min_points = self.node.get_parameter('filter.min_points').get_parameter_value().integer_value
        self.skip = self.node.get_parameter('filter.skip').get_parameter_value().integer_value

        # Visualization parameters
        self.coordinates = self.node.get_parameter('visualization.coordinates').get_parameter_value().string_value
        self.radius = self.node.get_parameter('visualization.radius').get_parameter_value().double_value
        self.color = self.node.get_parameter('visualization.color').get_parameter_value().string_value

        # Sonar parameters (Stonefish FLS)
        self.horizontal_fov = self.node.get_parameter('sonar.horizontal_fov').get_parameter_value().double_value
        self.vertical_fov = self.node.get_parameter('sonar.vertical_fov').get_parameter_value().double_value
        self.num_beams = self.node.get_parameter('sonar.num_beams').get_parameter_value().integer_value
        self.num_bins = self.node.get_parameter('sonar.num_bins').get_parameter_value().integer_value
        self.range_min = self.node.get_parameter('sonar.min_range').get_parameter_value().double_value
        self.range_max = self.node.get_parameter('sonar.max_range').get_parameter_value().double_value

        self.node.get_logger().info(f'FLS Parameters: {self.num_bins}x{self.num_beams}, FOV={self.horizontal_fov}°, Range={self.range_min}-{self.range_max}m')

        # Configure CFAR
        self.configure()

        self.node.get_logger().info("Feature extraction module initialized")

    def polar_to_cartesian(self, polar_image):
        """
        Convert Stonefish FLS polar image to cartesian coordinates
        Based on Oculus sonar polar-to-cartesian conversion methodology

        Reference: Oculus SDK and similar FLS processing pipelines

        Coordinate System (ROS REP-103 convention):
            - X-axis: forward (range direction, 0 to range_max)
            - Y-axis: lateral/port-starboard (negative=left/port, positive=right/starboard)
            - Z-axis: up (not used for 2D FLS)
            - bearing angle: measured from X-axis, counterclockwise positive

        Input: polar_image (num_bins × num_beams)
            - Rows: range bins (row=0 → range_max FAR, row=max → range_min NEAR)
            - Cols: bearing bins (col=0 → -FOV/2 left, col=max → +FOV/2 right)

        Output: cartesian image with proper x,y coordinates
            - X: forward (range direction)
            - Y: lateral (left-right direction)
        """
        rows = polar_image.shape[0]  # num_bins
        cols = polar_image.shape[1]  # num_beams

        # Build or use cached transformation maps (performance optimization)
        if self.p2c_cache is None:
            self.node.get_logger().info("Building polar-to-cartesian transformation maps (one-time setup)...")

            # Calculate range resolution
            range_resolution = (self.range_max - self.range_min) / rows

            # Maximum lateral extent based on FOV
            max_lateral = self.range_max * np.sin(np.radians(self.horizontal_fov / 2.0))

            # Cartesian image dimensions (same resolution as range)
            cart_width = int(np.ceil(2 * max_lateral / range_resolution))
            cart_height = rows

            # Create bearing angle array for each column
            # col=0 → -FOV/2 (left), col=num_beams-1 → +FOV/2 (right)
            bearing_angles = np.radians(
                np.linspace(-self.horizontal_fov / 2.0,
                           self.horizontal_fov / 2.0,
                           cols)
            )

            # Create interpolation function: bearing → column index
            from scipy.interpolate import interp1d
            f_bearings = interp1d(
                bearing_angles,
                range(cols),
                kind='linear',
                bounds_error=False,
                fill_value=-1,
                assume_sorted=True
            )

            # Build cartesian meshgrid (pixel indices)
            XX, YY = np.meshgrid(range(cart_width), range(cart_height))

            # Convert pixel indices to metric coordinates
            # IMPORTANT: Stonefish FLS row convention is OPPOSITE of Oculus
            # Stonefish: row=0 (top) is FAR range, row=max (bottom) is NEAR range
            # Cartesian: YY=0 (top) should be FAR, YY=max (bottom) should be NEAR
            x_meters = self.range_max - range_resolution * YY

            # Y: lateral distance - centered at 0
            y_meters = range_resolution * (-cart_width / 2.0 + XX + 0.5)

            # Convert cartesian (x, y) to polar (r, bearing)
            r_polar = np.sqrt(np.square(x_meters) + np.square(y_meters))
            bearing_polar = np.arctan2(y_meters, x_meters)

            # Map polar coordinates to image indices
            # Range to row index (distance in meters to pixel row)
            # STONEFISH: row=0 is FAR (range_max), row=rows-1 is NEAR (range_min)
            map_y = np.asarray((self.range_max - r_polar) / range_resolution, dtype=np.float32)

            # Bearing to column index (using interpolation)
            map_x = np.asarray(f_bearings(bearing_polar), dtype=np.float32)

            # Create valid region mask (where both map_x and map_y are valid)
            valid_mask = (map_x >= 0) & (map_x < cols - 1) & (map_y >= 0) & (map_y < rows - 1)

            # Cache the transformation maps
            self.p2c_cache = {
                'map_x': map_x,
                'map_y': map_y,
                'cart_height': cart_height,
                'cart_width': cart_width,
                'valid_mask': valid_mask.astype(np.uint8) * 255  # Cache valid region mask
            }

            self.node.get_logger().info(f"Transformation maps built: {cart_height}x{cart_width} cartesian image")

        # Use cached maps for fast remapping
        cartesian_image = cv2.remap(
            polar_image,
            self.p2c_cache['map_x'],
            self.p2c_cache['map_y'],
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0  # Black border instead of white
        )

        return cartesian_image

    def extract_features(self, sonar_msg):
        '''Extract features from sonar image using CFAR.

        Args:
            sonar_msg: Image message in polar coordinates (range × bearing)

        Returns:
            numpy array: Nx2 array of [x, y] points in cartesian coordinates (meters)
                         Returns empty array (0, 2) if no features detected
        '''

        # Skip frames
        self.frame_count += 1
        if self.frame_count % self.skip != 0:
            return np.zeros((0, 2), dtype=np.float32)

        # Decode the image
        polar_img = self.BridgeInstance.imgmsg_to_cv2(sonar_msg, desired_encoding="passthrough")

        # Ensure grayscale
        if len(polar_img.shape) == 3:
            polar_img = cv2.cvtColor(polar_img, cv2.COLOR_BGR2GRAY)

        # Detect targets using CFAR (in polar coordinates)
        start_time = time.time()
        peaks_cfar = self.detector.detect(polar_img, self.alg)

        # Apply threshold for feature extraction (point cloud)
        peaks = peaks_cfar.copy()
        if self.threshold > 0:
            peaks &= polar_img > self.threshold

        end_time = time.time()
        self.node.get_logger().debug(f"CFAR elapsed time: {end_time - start_time:.4f}s")

        # Extract peak locations in polar image (row, col)
        peak_locs = np.argwhere(peaks)  # Returns [[row, col], ...]

        if len(peak_locs) == 0:
            self.node.get_logger().debug("No features detected")
            return np.zeros((0, 2), dtype=np.float32)

        # Convert polar coordinates to cartesian
        # peak_locs: [[row, col], ...] where row=range_bin, col=beam
        # FLS coordinate system (following ROS REP-103):
        #   X: forward (range direction)
        #   Y: lateral (left-right direction)
        #   bearing: angle from X-axis, negative=left, positive=right
        points_cartesian = []

        for row, col in peak_locs:
            # Row to range (meters)
            # STONEFISH: row=0 → FAR (range_max), row=num_bins-1 → NEAR (range_min)
            range_m = self.range_max - (row / (self.num_bins - 1)) * (self.range_max - self.range_min)

            # Col to bearing angle (radians)
            # col=0 → -FOV/2 (left edge), col=num_beams-1 → +FOV/2 (right edge)
            bearing_rad = np.radians(-self.horizontal_fov / 2.0 +
                                     (col / (self.num_beams - 1)) * self.horizontal_fov)

            # Polar to Cartesian conversion
            # Based on: bearing = arctan2(y, x) where x=forward, y=lateral
            # Therefore: x = r*cos(bearing), y = r*sin(bearing)
            x = range_m * np.cos(bearing_rad)  # forward component
            y = range_m * np.sin(bearing_rad)  # lateral component

            points_cartesian.append([x, y])

        points = np.array(points_cartesian)

        self.node.get_logger().debug(f"Extracted {len(points)} feature points")

        return points


# Standalone Node wrapper (for backward compatibility if needed)
class FeatureExtractionNode(Node):
    """Standalone ROS2 node wrapper for FeatureExtraction module.

    DEPRECATED: This wrapper is for backward compatibility only.
    The recommended approach is to use FeatureExtraction as an internal module in SLAM node.
    """

    def __init__(self):
        super().__init__('feature_extraction_node')
        self.get_logger().warn("FeatureExtractionNode standalone mode is DEPRECATED. Use SLAM node integration instead.")

        # Instantiate feature extraction module
        self.feature_extractor = FeatureExtraction(self)

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Sonar subscriber
        self.sonar_sub = self.create_subscription(
            Image,
            f'/{self.feature_extractor.vehicle_name}/fls/image',
            self.callback,
            qos
        )

        # Feature publisher
        self.feature_pub = self.create_publisher(
            PointCloud2,
            SONAR_FEATURE_TOPIC,
            10
        )

        self.get_logger().info(f"Feature extraction node initialized (STANDALONE MODE)")

    def callback(self, sonar_msg):
        """ROS2 callback wrapper"""
        # Extract features
        points = self.feature_extractor.extract_features(sonar_msg)

        # Publish as PointCloud2
        if len(points) > 0:
            # Add z=0 for 2D sonar in FRD frame: [x=forward, y=lateral, z=0]
            points_3d = np.c_[points[:, 0], points[:, 1], np.zeros(len(points))]

            # Convert to a pointcloud
            feature_msg = n2r(points_3d, "PointCloudXYZ")
            feature_msg.header.stamp = sonar_msg.header.stamp
            feature_msg.header.frame_id = f"{self.feature_extractor.vehicle_name}/base_link_frd"
            self.feature_pub.publish(feature_msg)
        else:
            # Publish empty cloud
            empty_points = np.zeros((0, 3))
            feature_msg = n2r(empty_points, "PointCloudXYZ")
            feature_msg.header.stamp = sonar_msg.header.stamp
            feature_msg.header.frame_id = f"{self.feature_extractor.vehicle_name}/base_link_frd"
            self.feature_pub.publish(feature_msg)


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


if __name__ == "__main__":
    main()
