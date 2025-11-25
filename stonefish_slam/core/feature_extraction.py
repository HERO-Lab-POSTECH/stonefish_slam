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


class FeatureExtractionNode(Node):
    '''Class to handle extracting features from Sonar images using CFAR
    subscribes to the sonar driver and publishes a point cloud
    '''

    def __init__(self):
        super().__init__('feature_extraction_node')
        self.get_logger().info("Feature extraction node initializing...")

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

        # Initialize node
        self.init_node_params()

    def configure(self):
        '''Calls the CFAR class constructor for the featureExtraction class
        '''
        self.detector = CFAR(self.Ntc, self.Ngc, self.Pfa, self.rank)

    def init_node_params(self):
        """Initialize ROS2 parameters and create publishers/subscribers"""

        # Declare parameters with default values
        self.declare_parameter('vehicle_name', 'bluerov2')
        self.declare_parameter('sonar_image_topic', '/bluerov2/fls/image')

        # CFAR parameters
        self.declare_parameter('CFAR.Ntc', 20)
        self.declare_parameter('CFAR.Ngc', 10)
        self.declare_parameter('CFAR.Pfa', 1e-2)
        self.declare_parameter('CFAR.rank', 10)
        self.declare_parameter('CFAR.alg', 'SOCA')

        # Filter parameters
        self.declare_parameter('filter.threshold', 0)
        self.declare_parameter('filter.resolution', 0.5)
        self.declare_parameter('filter.radius', 1.0)
        self.declare_parameter('filter.min_points', 5)
        self.declare_parameter('filter.skip', 5)

        # Visualization parameters
        self.declare_parameter('visualization.coordinates', 'cartesian')
        self.declare_parameter('visualization.radius', 0.1)
        self.declare_parameter('visualization.color', 'green')

        # Sonar parameters (Stonefish FLS)
        self.declare_parameter('sonar.horizontal_fov', 130.0)
        self.declare_parameter('sonar.vertical_fov', 20.0)
        self.declare_parameter('sonar.num_beams', 512)
        self.declare_parameter('sonar.num_bins', 500)
        self.declare_parameter('sonar.min_range', 0.5)
        self.declare_parameter('sonar.max_range', 20.0)

        # Get parameters
        self.vehicle_name = self.get_parameter('vehicle_name').get_parameter_value().string_value
        sonar_topic = self.get_parameter('sonar_image_topic').get_parameter_value().string_value

        # CFAR parameters
        self.Ntc = self.get_parameter('CFAR.Ntc').get_parameter_value().integer_value
        self.Ngc = self.get_parameter('CFAR.Ngc').get_parameter_value().integer_value
        self.Pfa = self.get_parameter('CFAR.Pfa').get_parameter_value().double_value
        self.rank = self.get_parameter('CFAR.rank').get_parameter_value().integer_value
        self.alg = self.get_parameter('CFAR.alg').get_parameter_value().string_value

        # Filter parameters
        self.threshold = self.get_parameter('filter.threshold').get_parameter_value().integer_value
        self.resolution = self.get_parameter('filter.resolution').get_parameter_value().double_value
        self.outlier_filter_radius = self.get_parameter('filter.radius').get_parameter_value().double_value
        self.outlier_filter_min_points = self.get_parameter('filter.min_points').get_parameter_value().integer_value
        self.skip = self.get_parameter('filter.skip').get_parameter_value().integer_value

        # Visualization parameters
        self.coordinates = self.get_parameter('visualization.coordinates').get_parameter_value().string_value
        self.radius = self.get_parameter('visualization.radius').get_parameter_value().double_value
        self.color = self.get_parameter('visualization.color').get_parameter_value().string_value

        # Sonar parameters (Stonefish FLS)
        self.horizontal_fov = self.get_parameter('sonar.horizontal_fov').get_parameter_value().double_value
        self.vertical_fov = self.get_parameter('sonar.vertical_fov').get_parameter_value().double_value
        self.num_beams = self.get_parameter('sonar.num_beams').get_parameter_value().integer_value
        self.num_bins = self.get_parameter('sonar.num_bins').get_parameter_value().integer_value
        self.range_min = self.get_parameter('sonar.min_range').get_parameter_value().double_value
        self.range_max = self.get_parameter('sonar.max_range').get_parameter_value().double_value

        # CV bridge
        self.BridgeInstance = cv_bridge.CvBridge()

        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.get_logger().info(f'Subscribing to sonar topic: {sonar_topic}')
        self.get_logger().info(f'FLS Parameters: {self.num_bins}x{self.num_beams}, FOV={self.horizontal_fov}°, Range={self.range_min}-{self.range_max}m')

        # Sonar subscriber
        self.sonar_sub = self.create_subscription(
            Image,
            sonar_topic,
            self.callback,
            qos
        )

        # Feature publisher
        self.feature_pub = self.create_publisher(
            PointCloud2,
            SONAR_FEATURE_TOPIC,
            10
        )

        # Visualization publisher
        self.feature_img_pub = self.create_publisher(
            Image,
            SONAR_FEATURE_IMG_TOPIC,
            10
        )

        # Configure CFAR
        self.configure()

        self.get_logger().info("Feature extraction sim node initialized")

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
            self.get_logger().info("Building polar-to-cartesian transformation maps (one-time setup)...")

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

            self.get_logger().info(f"Transformation maps built: {cart_height}x{cart_width} cartesian image")

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

    def publish_features(self, ping, points):
        '''Publish the feature message using the provided parameters in an Image message
        ping: Image message
        points: points to be converted to a ros point cloud, in cartesian meters (Nx2 array: [x, y])
        '''

        # Add z=0 for 2D sonar in FRD frame: [x=forward, y=lateral, z=0]
        points = np.c_[points[:, 0], points[:, 1], np.zeros(len(points))]

        # Convert to a pointcloud
        feature_msg = n2r(points, "PointCloudXYZ")

        # Give the feature message the same time stamp as the source sonar image
        # This is CRITICAL to good time sync downstream
        feature_msg.header.stamp = ping.header.stamp
        # Use FRD (Forward-Right-Down) frame for correct orientation
        # FLS sonar scans forward (X+) in horizontal plane (XY)
        feature_msg.header.frame_id = f"{self.vehicle_name}/base_link_frd"

        # Publish the point cloud, to be used by SLAM
        self.feature_pub.publish(feature_msg)

    def publish_sonar_image(self, peaks_mask):
        """Publish visualization of CFAR detection results ONLY

        OPTIMIZED: Only convert mask to cartesian (not original image)

        Args:
            peaks_mask: Binary mask of CFAR detections (polar coordinates, boolean)
        """
        # Convert CFAR mask to cartesian (ONLY 1 conversion!)
        peaks_img = (peaks_mask.astype(np.uint8)) * 255
        peaks_cart = self.polar_to_cartesian(peaks_img)

        # Black background + green CFAR detections
        vis_img = np.zeros((peaks_cart.shape[0], peaks_cart.shape[1], 3), dtype=np.uint8)
        vis_img[peaks_cart > 128] = [0, 255, 0]

        # Publish
        vis_msg = self.BridgeInstance.cv2_to_imgmsg(vis_img, encoding="bgr8")
        self.feature_img_pub.publish(vis_msg)

    def callback(self, sonar_msg):
        '''Feature extraction callback
        sonar_msg: an Image message, in polar coordinates (range × bearing)
        '''

        # Skip frames
        self.frame_count += 1
        if self.frame_count % self.skip != 0:
            return

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
        self.get_logger().debug(f"CFAR elapsed time: {end_time - start_time:.4f}s")

        # Extract peak locations in polar image (row, col)
        peak_locs = np.argwhere(peaks)  # Returns [[row, col], ...]

        if len(peak_locs) == 0:
            self.get_logger().debug("No features detected")
            # Publish empty cloud
            empty_points = np.zeros((0, 3))
            feature_msg = n2r(empty_points, "PointCloudXYZ")
            feature_msg.header.stamp = sonar_msg.header.stamp
            feature_msg.header.frame_id = f"{self.vehicle_name}/base_link_frd"
            self.feature_pub.publish(feature_msg)
            # Publish visualization (empty)
            self.publish_sonar_image(peaks)
            return

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

        self.get_logger().debug(f"Extracted {len(points)} feature points")

        # Publish the feature message
        self.publish_features(sonar_msg, points)

        # Publish visualization (OPTIMIZED: mask only, 1 conversion)
        self.publish_sonar_image(peaks)


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
