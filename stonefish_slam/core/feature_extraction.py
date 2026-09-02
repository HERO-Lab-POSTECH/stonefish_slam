#!/usr/bin/env python3
import numpy as np
import cv2
import cv_bridge
import time

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

        # Stonefish FLS sonar parameters (loaded from YAML)
        self.horizontal_fov = None
        self.vertical_fov = None
        self.num_beams = None  # number of beams (columns)
        self.num_bins = None  # number of range bins (rows)
        self.range_min = None
        self.range_max = None
        self.sonar_tilt_deg = None
        self.sonar_tilt_rad = None

        # Frame counter for skip
        self.frame_count = 0

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
        self.range_min = self.node.get_parameter('sonar.range_min').get_parameter_value().double_value
        self.range_max = self.node.get_parameter('sonar.range_max').get_parameter_value().double_value
        self.sonar_tilt_deg = self.node.get_parameter('sonar.sonar_tilt_deg').get_parameter_value().double_value
        self.sonar_tilt_rad = np.radians(self.sonar_tilt_deg)

        # 경사거리 → 수평거리 투영 (sonar.projection). 아래 _project_range 참고.
        self.projection = (
            self.node.get_parameter('sonar.projection').get_parameter_value().string_value or 'legacy')
        cos_tilt = float(np.cos(self.sonar_tilt_rad))
        self._inv_cos_tilt = 1.0 / cos_tilt if cos_tilt > 1e-6 else 1.0
        self.proj_dropped = 0       # r <= h 라 수평 성분이 없어 버린 점
        self.proj_alt_missing = 0   # altitude 모드인데 고도계 값이 아직 없던 프레임

        self.node.get_logger().info(f'FLS Parameters: {self.num_bins}x{self.num_beams}, FOV={self.horizontal_fov}°, Range={self.range_min}-{self.range_max}m, Tilt={self.sonar_tilt_deg}°')

        # Configure CFAR
        self.configure()

        self.node.get_logger().info("Feature extraction module initialized")

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

        # Decode the image (supports both Image and CompressedImage; sensor_msgs
        # is imported lazily to keep this module path-loadable without ROS)
        from sensor_msgs.msg import CompressedImage as _CompressedImage
        if isinstance(sonar_msg, _CompressedImage):
            polar_img = self.BridgeInstance.compressed_imgmsg_to_cv2(sonar_msg, desired_encoding="passthrough")
        else:
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
            # 경사거리를 수평거리로 투영한 뒤에 극→직교 변환한다. Bruce SLAM 은
            # 소나가 수평 장착이라 이 단계가 없었다(φ=0 가정) — 하향 틸트로 바닥을
            # 보면 그 가정이 깨지고 ICP 병진이 체계적으로 축소된다.
            proj_m = self._project_range(range_m)
            if proj_m is None:
                continue

            x = proj_m * np.cos(bearing_rad)  # forward component
            y = proj_m * np.sin(bearing_rad)  # lateral component

            points_cartesian.append([x, y])

        points = np.array(points_cartesian)

        self.node.get_logger().debug(f"Extracted {len(points)} feature points")

        return points

    def _project_range(self, range_m):
        '''경사거리 r 을 수평 평면 좌표로 투영한다.

        바닥의 한 점은 고도 h·수평거리 rho 에 대해 r = sqrt(rho^2 + h^2) 로 재진다.
        따라서 전진 d 에 대한 r 의 변화는 d·cos(theta) 로 d 보다 작고, r 을 그대로
        쓰면 ICP 가 병진을 그만큼 적게 본다. theta 는 고도각이며 틸트 tau 근처다.

        - ``legacy``       : r 그대로. Bruce SLAM 원본(수평 장착 가정).
        - ``inv_cos_tilt`` : r / cos(tau). dC/drho = cos(theta)/cos(tau) 로 상수 근사.
        - ``altitude``     : sqrt(r^2 - h^2). 평평한 바닥 가정에서 정확하다.

        Returns:
            투영된 거리(m). 수평 성분이 없는 점(r <= h)이면 None.
        '''
        if self.projection == 'altitude':
            h = getattr(self.node, 'altitude_m', None)
            if h is None or h <= 0.0:
                self.proj_alt_missing += 1
                return range_m * self._inv_cos_tilt
            if range_m <= h:
                self.proj_dropped += 1
                return None
            return float(np.sqrt(range_m * range_m - h * h))
        if self.projection == 'inv_cos_tilt':
            return range_m * self._inv_cos_tilt
        return range_m
