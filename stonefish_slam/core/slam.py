# python imports
import numpy as np
import struct
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import cv_bridge
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped

# stonefish_slam imports
from stonefish_slam.utils.io import *
from stonefish_slam.utils.conversions import *
from stonefish_slam.utils.visualization import *
from stonefish_slam.core.factor_graph import FactorGraph
from stonefish_slam.core.localization import Localization
from stonefish_slam.core.types import Keyframe, STATUS, ICPResult
from stonefish_slam.core.mapping_2d import SonarMapping2D
from stonefish_slam.core.mapping_3d import SonarMapping3D
from stonefish_slam.core.feature_extraction import FeatureExtraction
from stonefish_slam.core.localization_fft import FFTLocalizer
from stonefish_slam.cpp import pcl
from stonefish_slam.utils.topics import *


def pointcloud2_to_xyz_array(cloud_msg):
    """
    Convert PointCloud2 message to numpy array of XYZ points

    Args:
        cloud_msg: sensor_msgs.msg.PointCloud2

    Returns:
        numpy array of shape (N, 3) with xyz coordinates
    """
    # Get point step and create dtype
    point_step = cloud_msg.point_step

    # Parse fields to find x, y, z offsets
    field_names = [field.name for field in cloud_msg.fields]

    # Find offsets for x, y, z
    x_offset = None
    y_offset = None
    z_offset = None

    for field in cloud_msg.fields:
        if field.name == 'x':
            x_offset = field.offset
        elif field.name == 'y':
            y_offset = field.offset
        elif field.name == 'z':
            z_offset = field.offset

    if x_offset is None or y_offset is None or z_offset is None:
        raise ValueError("PointCloud2 must have x, y, z fields")

    # Convert data to numpy array
    num_points = cloud_msg.width * cloud_msg.height
    points = []

    for i in range(num_points):
        offset = i * point_step

        # Unpack x, y, z as floats
        x = struct.unpack('f', cloud_msg.data[offset + x_offset:offset + x_offset + 4])[0]
        y = struct.unpack('f', cloud_msg.data[offset + y_offset:offset + y_offset + 4])[0]
        z = struct.unpack('f', cloud_msg.data[offset + z_offset:offset + z_offset + 4])[0]

        points.append([x, y, z])

    # Ensure we return a 2D array even when empty
    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    return np.array(points, dtype=np.float32)


class SLAMNode(Node):
    """ROS2 SLAM node using modular factor graph and localization components.
    """

    def __init__(self):
        # Initialize ROS2 Node first
        Node.__init__(self, 'slam_node')

        # Declare mode parameter BEFORE using it
        self.declare_parameter('mode', 'slam')
        self.mode = self.get_parameter('mode').value
        self.get_logger().info(f"Operating mode: {self.mode}")

        # Validate mode
        valid_modes = ['slam', 'localization-only', 'mapping-only']
        if self.mode not in valid_modes:
            self.get_logger().error(f"Invalid mode '{self.mode}'. Must be one of {valid_modes}")
            raise ValueError(f"Invalid mode: {self.mode}")

        # ===== Declare all parameters from YAML files (BEFORE init_node) =====

        # Sonar parameters (sonar.yaml)
        self.declare_parameter('vehicle_name', 'bluerov2')
        # Note: sonar_image_topic is constructed from vehicle_name
        self.declare_parameter('sonar.horizontal_fov', 130.0)
        self.declare_parameter('sonar.vertical_fov', 20.0)
        self.declare_parameter('sonar.num_beams', 512)
        self.declare_parameter('sonar.num_bins', 500)
        self.declare_parameter('sonar.range_min', 0.5)
        self.declare_parameter('sonar.range_max', 30.0)
        self.declare_parameter('sonar.sonar_position', [0.0, 0.0, 0.0])
        self.declare_parameter('sonar.sonar_tilt_deg', 10.0)

        # Feature extraction parameters (feature.yaml)
        self.declare_parameter('CFAR.Ntc', 20)
        self.declare_parameter('CFAR.Ngc', 10)
        self.declare_parameter('CFAR.Pfa', 0.01)
        self.declare_parameter('CFAR.rank', 10)
        self.declare_parameter('CFAR.alg', 'SOCA')
        self.declare_parameter('filter.threshold', 80)
        self.declare_parameter('filter.resolution', 0.5)
        self.declare_parameter('filter.radius', 1.0)
        self.declare_parameter('filter.min_points', 5)
        self.declare_parameter('filter.skip', 5)
        self.declare_parameter('visualization.coordinates', 'cartesian')
        self.declare_parameter('visualization.radius', 2.0)
        self.declare_parameter('visualization.color', 'green')

        # Localization parameters (localization.yaml)
        self.declare_parameter('keyframe_duration', 1.0)
        self.declare_parameter('keyframe_translation', 3.0)
        self.declare_parameter('keyframe_rotation', 0.5236)
        self.declare_parameter('slam_prior_noise', [0.1, 0.1, 0.01])
        self.declare_parameter('slam_odom_noise', [0.2, 0.2, 0.02])
        self.declare_parameter('slam_icp_noise', [0.1, 0.1, 0.01])
        self.declare_parameter('point_downsample_resolution', 0.5)
        self.declare_parameter('ssm.enable', False)
        self.declare_parameter('ssm.min_points', 50)
        self.declare_parameter('ssm.max_translation', 3.0)
        self.declare_parameter('ssm.max_rotation', 0.5236)
        self.declare_parameter('ssm.target_frames', 3)
        self.declare_parameter('icp_config', '/workspace/colcon_ws/src/stonefish_slam/config/icp.yaml')

        # Factor graph parameters (factor_graph.yaml)
        self.declare_parameter('nssm.enable', False)
        self.declare_parameter('nssm.min_st_sep', 15)
        self.declare_parameter('nssm.min_points', 150)
        self.declare_parameter('nssm.max_translation', 5.0)
        self.declare_parameter('nssm.max_rotation', 0.5236)
        self.declare_parameter('nssm.source_frames', 5)
        self.declare_parameter('nssm.cov_samples', 30)
        self.declare_parameter('pcm_queue_size', 5)
        self.declare_parameter('min_pcm', 3)

        # Mapping parameters (mapping.yaml)
        self.declare_parameter('mapping_2d.map_2d_resolution', 0.1)
        self.declare_parameter('mapping_2d.map_size', [4000, 4000])
        self.declare_parameter('mapping_2d.map_update_interval', 1)
        self.declare_parameter('mapping_2d.intensity_threshold', 10)
        self.declare_parameter('mapping_3d.map_3d_voxel_size', 0.2)
        self.declare_parameter('mapping_3d.min_probability', 0.6)
        self.declare_parameter('mapping_3d.log_odds_occupied', 2.0)
        self.declare_parameter('mapping_3d.log_odds_free', -3.0)
        self.declare_parameter('mapping_3d.log_odds_min', -30.0)
        self.declare_parameter('mapping_3d.log_odds_max', 20.0)
        self.declare_parameter('mapping_3d.adaptive_update', True)
        self.declare_parameter('mapping_3d.adaptive_threshold', 0.5)
        self.declare_parameter('mapping_3d.adaptive_max_ratio', 0.3)
        self.declare_parameter('mapping_3d.use_range_weighting', True)
        self.declare_parameter('mapping_3d.lambda_decay', 0.1)
        self.declare_parameter('mapping_3d.use_cpp_backend', True)
        self.declare_parameter('mapping_3d.enable_propagation', False)
        self.declare_parameter('mapping_3d.propagation_radius', 2)
        self.declare_parameter('mapping_3d.propagation_sigma', 1.5)
        self.declare_parameter('mapping_3d.enable_gaussian_weighting', False)
        self.declare_parameter('mapping_3d.use_dda_traversal', True)
        self.declare_parameter('mapping_3d.bearing_step', 1)

        # SLAM integration parameters (slam.yaml)
        self.declare_parameter('enable_2d_mapping', False)
        self.declare_parameter('enable_3d_mapping', True)

        # FFT localization parameters
        self.declare_parameter('fft_localization.enable', False)
        self.declare_parameter('fft_localization.range_min', 0.5)

        # Initialize SLAM modules (composition instead of inheritance)
        self.fg = FactorGraph()

        # Conditional localization instantiation
        if self.mode != 'mapping-only':
            self.localization = Localization(self.fg)
            self.get_logger().info("Localization module enabled")
        else:
            self.localization = None
            self.get_logger().info("Localization module disabled (mapping-only mode)")

        # Feature extraction module (integrated internally)
        self.feature_extractor = FeatureExtraction(self)
        self.get_logger().info("Feature extraction module integrated")

        # No lock needed (synchronous processing)

        # TF2 buffer for timestamp synchronization (30s cache for delayed mapping)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Mapping statistics
        self.mapping_stats = {
            'keyframes_total': 0,
            'maps_published': 0
        }

        # Mapping initialization (configured in init_node)
        self.mapper = None
        self.enable_2d_mapping = False
        self.map_update_interval = 1  # 매 키프레임마다 업데이트
        self.last_map_update_kf = 0
        self.bridge = cv_bridge.CvBridge()

        # Initialize node parameters and subscribers/publishers
        self.init_node()

    def init_node(self, ns="~") -> None:
        """Configures the SLAM node

        Args:
            ns (str, optional): The namespace of the node. Defaults to "~".
        """
        # keyframe paramters, how often to add them (loaded from localization.yaml)
        keyframe_duration_sec = self.get_parameter('keyframe_duration').value
        keyframe_duration = Duration(seconds=keyframe_duration_sec)
        keyframe_translation = self.get_parameter('keyframe_translation').value
        keyframe_rotation = self.get_parameter('keyframe_rotation').value

        # Set keyframe criteria in localization module (skip if mapping-only mode)
        if self.localization is not None:
            self.localization.keyframe_duration = keyframe_duration
            self.localization.keyframe_translation = keyframe_translation
            self.localization.keyframe_rotation = keyframe_rotation

        # noise models (loaded from localization.yaml)
        prior_sigmas = self.get_parameter('slam_prior_noise').value
        odom_sigmas = self.get_parameter('slam_odom_noise').value
        icp_odom_sigmas = self.get_parameter('slam_icp_noise').value

        # Store noise sigmas for later noise model creation
        self.prior_sigmas = prior_sigmas
        self.odom_sigmas = odom_sigmas
        self.icp_odom_sigmas = icp_odom_sigmas

        # Set noise sigmas in localization module (skip if mapping-only mode)
        if self.localization is not None:
            self.localization.odom_sigmas = odom_sigmas
            self.localization.icp_odom_sigmas = icp_odom_sigmas

        # resultion for map downsampling (loaded from localization.yaml)
        point_resolution = self.get_parameter('point_downsample_resolution').value
        if self.localization is not None:
            self.localization.point_resolution = point_resolution

        # Sonar configuration (loaded from sonar.yaml)
        if self.localization is not None:
            # Configure oculus object with parameters from sonar.yaml
            # Note: In original code, this was done via oculus.configure(ping) message callback
            # Now we use ROS2 parameters instead
            sonar_range_max = self.get_parameter('sonar.range_max').value
            sonar_range_min = self.get_parameter('sonar.range_min').value
            sonar_horizontal_fov = self.get_parameter('sonar.horizontal_fov').value
            sonar_vertical_fov = self.get_parameter('sonar.vertical_fov').value
            # Get sonar dimensions from sonar.yaml
            sonar_num_bins = self.get_parameter('sonar.num_bins').value
            sonar_num_beams = self.get_parameter('sonar.num_beams').value

            # Set critical parameters manually (configure() requires ping message)
            self.localization.oculus.range_max = sonar_range_max
            self.localization.oculus.range_resolution = sonar_range_max / sonar_num_bins
            self.localization.oculus.num_ranges = sonar_num_bins
            self.localization.oculus.horizontal_fov = np.radians(sonar_horizontal_fov)
            self.localization.oculus.vertical_fov = np.radians(sonar_vertical_fov)
            self.localization.oculus.num_beams = sonar_num_beams
            self.localization.oculus.angular_resolution = np.radians(sonar_horizontal_fov) / sonar_num_beams

            self.get_logger().info(f"Sonar configured: range_max={sonar_range_max}m, "
                                   f"resolution={self.localization.oculus.range_resolution:.3f}m, "
                                   f"FOV={sonar_horizontal_fov}deg")

        # sequential scan matching parameters (SSM) (loaded from localization.yaml)
        if self.localization is not None:
            # SSM is disabled in mapping-only mode
            if self.mode == 'mapping-only':
                self.localization.ssm_params.enable = False
            else:
                self.localization.ssm_params.enable = self.get_parameter('ssm.enable').value
            self.localization.ssm_params.min_points = self.get_parameter('ssm.min_points').value
            self.localization.ssm_params.max_translation = self.get_parameter('ssm.max_translation').value
            self.localization.ssm_params.max_rotation = self.get_parameter('ssm.max_rotation').value
            self.localization.ssm_params.target_frames = self.get_parameter('ssm.target_frames').value
            self.get_logger().info(f"SSM: {self.localization.ssm_params.enable}")

        # non sequential scan matching parameters (NSSM) aka loop closures (loaded from localization.yaml)
        if self.localization is not None:
            # NSSM is disabled in localization-only and mapping-only modes
            if self.mode in ['localization-only', 'mapping-only']:
                self.localization.nssm_params.enable = False
            else:
                self.localization.nssm_params.enable = self.get_parameter('nssm.enable').value
            self.localization.nssm_params.min_st_sep = self.get_parameter('nssm.min_st_sep').value
            self.localization.nssm_params.min_points = self.get_parameter('nssm.min_points').value
            self.localization.nssm_params.max_translation = self.get_parameter('nssm.max_translation').value
            self.localization.nssm_params.max_rotation = self.get_parameter('nssm.max_rotation').value
            self.localization.nssm_params.source_frames = self.get_parameter('nssm.source_frames').value
            self.localization.nssm_params.cov_samples = self.get_parameter('nssm.cov_samples').value
            self.get_logger().info(f"NSSM: {self.localization.nssm_params.enable}")

        # pairwise consistency maximization parameters for loop closure (loaded from localization.yaml)
        self.fg.pcm_queue_size = self.get_parameter('pcm_queue_size').value
        self.fg.min_pcm = self.get_parameter('min_pcm').value

        # ===== Sonar Hardware Parameters ===== (loaded from sonar.yaml)
        # ===== 2D Mapping Parameters ===== (loaded from mapping.yaml)
        # ===== 3D Mapping Parameters ===== (loaded from mapping.yaml)
        # ===== Mapping Enable Flags ===== (loaded from slam.yaml)

        self.enable_2d_mapping = self.get_parameter('enable_2d_mapping').value
        self.map_update_interval = self.get_parameter('mapping_2d.map_update_interval').value

        # Build sonar config dict (unified for 2D and 3D)
        sonar_config = {
            'range_max': self.get_parameter('sonar.range_max').value,
            'range_min': self.get_parameter('sonar.range_min').value,
            'horizontal_fov': self.get_parameter('sonar.horizontal_fov').value,
            'vertical_fov': self.get_parameter('sonar.vertical_fov').value,
            'num_beams': self.get_parameter('sonar.num_beams').value,
            'num_bins': self.get_parameter('sonar.num_bins').value,
            'sonar_position': self.get_parameter('sonar.sonar_position').value,
            'sonar_tilt_deg': self.get_parameter('sonar.sonar_tilt_deg').value,
        }

        # Fixed map resolution for DDS message size compatibility
        # Auto-calculated: sonar_range / sonar_bins = 40/512 = 0.078m → 5120x5120 pixels (26MB)
        # Fixed: 0.2m → 1000x1000 pixels (1MB) - reduces message size by 26x
        map_resolution = 0.2  # Fixed resolution for 200x200m map

        if self.enable_2d_mapping:
            map_size = tuple(self.get_parameter('mapping_2d.map_size').value)
            intensity_threshold = self.get_parameter('mapping_2d.intensity_threshold').value

            self.mapper = SonarMapping2D(
                map_resolution=map_resolution,
                map_size=map_size,
                sonar_range=sonar_config['range_max'],
                sonar_fov=sonar_config['horizontal_fov'],
                sonar_tilt_deg=sonar_config['sonar_tilt_deg'],
                intensity_threshold=intensity_threshold
            )
            self.get_logger().info(
                f"2D Mapping enabled: resolution={map_resolution}m/px, "
                f"range_max={sonar_config['range_max']}m, tilt={sonar_config['sonar_tilt_deg']}°, "
                f"intensity_threshold={intensity_threshold}"
            )

        # Store sonar parameters for compatibility
        self.sonar_fov = sonar_config['horizontal_fov']
        self.sonar_range = sonar_config['range_max']
        self.intensity_threshold = self.get_parameter('mapping_2d.intensity_threshold').value

        # Initialize 3D mapper
        self.enable_3d_mapping = self.get_parameter('enable_3d_mapping').value
        self.mapper_3d = None
        self.map_3d_pub = None

        if self.enable_3d_mapping:
            self.get_logger().info("Initializing 3D mapping...")

            # Build 3D mapping config dict (includes sonar + 3D-specific params)
            mapping_3d_config = {
                # Sonar parameters (shared from sonar_config)
                'range_max': sonar_config['range_max'],
                'range_min': sonar_config['range_min'],
                'horizontal_fov': sonar_config['horizontal_fov'],
                'vertical_fov': sonar_config['vertical_fov'],
                'num_beams': sonar_config['num_beams'],
                'num_bins': sonar_config['num_bins'],
                'sonar_position': sonar_config['sonar_position'],
                'sonar_tilt_deg': sonar_config['sonar_tilt_deg'],
                'intensity_threshold': self.intensity_threshold,

                # 3D mapping specific
                'voxel_resolution': self.get_parameter('mapping_3d.map_3d_voxel_size').value,
                'min_probability': self.get_parameter('mapping_3d.min_probability').value,
                'log_odds_occupied': self.get_parameter('mapping_3d.log_odds_occupied').value,
                'log_odds_free': self.get_parameter('mapping_3d.log_odds_free').value,
                'log_odds_min': self.get_parameter('mapping_3d.log_odds_min').value,
                'log_odds_max': self.get_parameter('mapping_3d.log_odds_max').value,
                'adaptive_update': self.get_parameter('mapping_3d.adaptive_update').value,
                'adaptive_threshold': self.get_parameter('mapping_3d.adaptive_threshold').value,
                'adaptive_max_ratio': self.get_parameter('mapping_3d.adaptive_max_ratio').value,
                'use_cpp_backend': self.get_parameter('mapping_3d.use_cpp_backend').value,
                'enable_propagation': self.get_parameter('mapping_3d.enable_propagation').value,
                'use_range_weighting': self.get_parameter('mapping_3d.use_range_weighting').value,
                'lambda_decay': self.get_parameter('mapping_3d.lambda_decay').value,
                'enable_gaussian_weighting': self.get_parameter('mapping_3d.enable_gaussian_weighting').value,
                'use_dda_traversal': self.get_parameter('mapping_3d.use_dda_traversal').value,
                'bearing_step': self.get_parameter('mapping_3d.bearing_step').value,

                # Fixed parameters
                'max_frames': 0,
                'dynamic_expansion': True,
            }

            # Create mapper
            self.mapper_3d = SonarMapping3D(config=mapping_3d_config)
            self.get_logger().info(
                f"3D Mapper initialized: resolution={mapping_3d_config['voxel_resolution']}m, "
                f"range_max={mapping_3d_config['range_max']}m, tilt={mapping_3d_config['sonar_tilt_deg']}°"
            )

        # FFT localizer initialization (optional)
        self.fft_enable = self.get_parameter('fft_localization.enable').value
        if self.fft_enable:
            if self.localization is not None:
                fft_range_min = self.get_parameter('fft_localization.range_min').value
                self.fft_localizer = FFTLocalizer(
                    oculus=self.localization.oculus,
                    range_min=fft_range_min
                )
                self.get_logger().info("FFT localization enabled")

                # Previous polar sonar image storage
                self.prev_polar_sonar = None
            else:
                self.get_logger().warn("FFT localization disabled: requires localization module (not available in mapping-only mode)")
                self.fft_enable = False
                self.fft_localizer = None
        else:
            self.fft_localizer = None
            self.get_logger().info("FFT localization disabled")

        # max delay between an incoming point cloud and dead reckoning
        self.feature_odom_sync_max_delay = 0.5

        # QoS profile for subscriptions (matching simulator's BEST_EFFORT)
        qos_sub_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        # QoS profile for publishers
        qos_pub_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for image publishers (BEST_EFFORT for compatibility with image viewers)
        qos_image_pub_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for PointCloud2 (RELIABLE for RViz compatibility)
        qos_pointcloud_pub_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to sonar image and odometry
        # NOTE: Feature extraction is now INTERNAL - no external feature topic subscription
        vehicle_name = self.get_parameter('vehicle_name').value
        sonar_image_topic = f'/{vehicle_name}/fls/image'
        odom_topic = f'/{vehicle_name}/odometry'
        self.sonar_sub = Subscriber(self, Image, sonar_image_topic, qos_profile=qos_sub_profile)
        self.odom_sub = Subscriber(self, Odometry, odom_topic, qos_profile=qos_sub_profile)

        # Add debug prints for topic names
        self.get_logger().info(f"Subscribing to sonar image: {sonar_image_topic} (internal feature extraction)")
        self.get_logger().info(f"Subscribing to odom topic: {odom_topic}")

        # Define sync policy: sonar image + odometry (2-way synchronization)
        self.time_sync = ApproximateTimeSynchronizer(
            [self.sonar_sub, self.odom_sub],
            20,
            self.feature_odom_sync_max_delay
        )
        self.time_sync.registerCallback(self.SLAM_callback_integrated)
        self.get_logger().info("Using 2-way synchronizer (sonar + odom) with integrated feature extraction")

        self.get_logger().info(f"Created time synchronizer with max delay: {self.feature_odom_sync_max_delay}")

        # pose publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, SLAM_POSE_TOPIC, 10)

        # dead reckoning topic
        self.odom_pub = self.create_publisher(Odometry, SLAM_ODOM_TOPIC, 10)

        # SLAM trajectory topic
        self.traj_pub = self.create_publisher(
            PointCloud2, SLAM_TRAJ_TOPIC, qos_pub_profile)

        # constraints between poses
        self.constraint_pub = self.create_publisher(
            Marker, SLAM_CONSTRAINT_TOPIC, qos_pub_profile)

        # point cloud publisher topic
        self.cloud_pub = self.create_publisher(
            PointCloud2, SLAM_CLOUD_TOPIC, qos_pub_profile)

        # 2D map publisher
        if self.enable_2d_mapping:
            self.map_2d_pub = self.create_publisher(
                Image,
                SLAM_NS + 'mapping/map_2d_image',
                qos_profile=qos_image_pub_profile
            )
            self.get_logger().info(f"Publishing 2D map to: {SLAM_NS}mapping/map_2d_image (QoS: BEST_EFFORT)")

        # 3D map publisher
        if self.enable_3d_mapping:
            self.map_3d_pub = self.create_publisher(
                PointCloud2,
                SLAM_NS + 'mapping/map_3d_pointcloud',
                qos_profile=qos_pointcloud_pub_profile
            )
            self.get_logger().info(f"Publishing 3D map to: {SLAM_NS}mapping/map_3d_pointcloud (QoS: RELIABLE)")

        # tf broadcaster to show pose
        self.tf = TransformBroadcaster(self)

        # cv bridge object
        self.CVbridge = cv_bridge.CvBridge()

        # get the ICP configuration from the yaml file (loaded from slam.yaml)
        icp_config = self.get_parameter('icp_config').value
        if icp_config and self.localization is not None:
            self.localization.icp.loadFromYaml(icp_config)

        # Extract robot ID from odometry topic
        # LOCALIZATION_ODOM_TOPIC = "/bluerov2/odometry" → rov_id = "bluerov2"
        if LOCALIZATION_ODOM_TOPIC.startswith('/'):
            parts = LOCALIZATION_ODOM_TOPIC.split('/')
            self.rov_id = parts[1] if len(parts) > 1 else ""
        else:
            self.rov_id = ""

        self.get_logger().info(f"Detected vehicle ID: '{self.rov_id}' from topic {LOCALIZATION_ODOM_TOPIC}")

        # call the configure function
        self.configure()
        self.get_logger().info("SLAM node is initialized")

        # 2D mapping uses synchronous processing (no thread needed)
        if self.enable_2d_mapping:
            self.get_logger().info("2D mapping enabled (synchronous mode)")

    def configure(self) -> None:
        """Configure SLAM noise models."""
        import gtsam

        # Create noise models using GTSAM
        prior_model = gtsam.noiseModel.Diagonal.Sigmas(np.r_[self.prior_sigmas])
        odom_model = gtsam.noiseModel.Diagonal.Sigmas(np.r_[self.odom_sigmas])
        icp_odom_model = gtsam.noiseModel.Diagonal.Sigmas(np.r_[self.icp_odom_sigmas])

        # Set noise models in factor graph
        self.fg.set_noise_models(prior_model, odom_model, icp_odom_model)

    def SLAM_callback_integrated(self, sonar_msg: Image, odom_msg: Odometry) -> None:
        """Integrated SLAM callback with internal feature extraction.

        Replaces the old 3-way synchronization (feature + odom + sonar).
        Now uses 2-way sync (sonar + odom) with internal feature extraction.

        Args:
            sonar_msg (Image): Sonar image message (polar coordinates)
            odom_msg (Odometry): Dead reckoning odometry
        """
        # 1. Extract features internally using FeatureExtraction module
        try:
            points = self.feature_extractor.extract_features(sonar_msg)
            self.get_logger().info(
                f"Callback: extracted {len(points)} features",
                throttle_duration_sec=1.0
            )
        except Exception as e:
            self.get_logger().error(f"Feature extraction failed: {e}")
            return

        # 2. Convert sonar image for mapping and FFT localization
        sonar_image = None
        polar_sonar = None
        if self.enable_2d_mapping or self.enable_3d_mapping or self.fft_enable:
            try:
                sonar_image = self.bridge.imgmsg_to_cv2(sonar_msg, desired_encoding="mono8")

                # FFT localization (polar sonar image 필요)
                if self.fft_enable:
                    # Polar image는 sonar_msg.data를 직접 변환
                    polar_sonar = np.frombuffer(sonar_msg.data, dtype=np.uint8).reshape(
                        sonar_msg.height, sonar_msg.width
                    )

                    # FFT 실행 (이전 프레임과 비교)
                    if self.prev_polar_sonar is not None:
                        try:
                            # 깊은 복사로 ICP와 완전 분리
                            polar_prev = self.prev_polar_sonar.copy()
                            polar_curr = polar_sonar.copy()

                            fft_result = self.fft_localizer.estimate_transform(polar_prev, polar_curr)

                            if fft_result['success']:
                                self.get_logger().info(
                                    f"FFT: rotation={fft_result['rotation']:.2f}deg, "
                                    f"translation=({fft_result['translation'][0]:.2f}, {fft_result['translation'][1]:.2f})m",
                                    throttle_duration_sec=1.0
                                )
                                # TODO: FFT 결과를 어디에 저장/사용할지는 추후 결정
                            else:
                                self.get_logger().warn("FFT localization failed", throttle_duration_sec=2.0)

                        except Exception as e:
                            self.get_logger().error(f"FFT localization error: {e}")

                    # 현재 polar image 저장 (다음 프레임용)
                    self.prev_polar_sonar = polar_sonar.copy()

            except Exception as e:
                self.get_logger().error(f"Failed to convert sonar image: {e}")

        # 3. Standard SLAM processing (unified for all modes)
        time = sonar_msg.header.stamp
        dr_pose3 = r2g(odom_msg.pose.pose)
        frame = Keyframe(False, time, dr_pose3)

        # Check if valid points (feature extraction may return empty on skip frames)
        if len(points) == 0 or (len(points) > 0 and np.isnan(points[0, 0])):
            frame.status = False
        else:
            if self.localization is not None:
                frame.status = self.localization.is_keyframe(frame)
            else:
                # In mapping-only mode, use simple time-based keyframe decision
                frame.status = True

        # Set frame twist
        frame.twist = odom_msg.twist.twist

        # Update keyframe pose from dead reckoning
        if self.fg.keyframes:
            dr_odom = self.fg.current_keyframe.dr_pose.between(frame.dr_pose)
            pose = self.fg.current_keyframe.pose.compose(dr_odom)
            frame.update(pose)

        # 4. Process keyframe
        if frame.status:
            # Add points
            frame.points = points

            # Add sonar image to frame
            if sonar_image is not None:
                frame.image = sonar_image
                frame.sonar_time = sonar_msg.header.stamp

            # Conditional localization processing
            if self.mode != 'mapping-only':
                # Sequential scan matching
                if not self.fg.keyframes:
                    self.fg.add_prior_factor(frame)
                else:
                    self.add_sequential_scan_matching(frame)

                # Update factor graph
                self.fg.update_graph(frame)
                self.get_logger().info(
                    f"Keyframe added: #{len(self.fg.keyframes)}, "
                    f"pose=({frame.pose.x():.2f}, {frame.pose.y():.2f}, {frame.pose.theta():.3f}), "
                    f"points={len(points)}, has_image={frame.image is not None}"
                )

                # Loop closure (slam mode only)
                if self.mode == 'slam' and self.localization.nssm_params.enable and self.add_nonsequential_scan_matching():
                    self.fg.update_graph()
            else:
                # mapping-only mode: use DR pose directly
                frame.pose = frame.dr_pose
                self.fg.keyframes.append(frame)
                self.get_logger().info(
                    f"Keyframe added (mapping-only): #{len(self.fg.keyframes)}, "
                    f"DR pose=({frame.dr_pose.x():.2f}, {frame.dr_pose.y():.2f}, {frame.dr_pose.theta():.3f}), "
                    f"points={len(points)}, has_image={frame.image is not None}"
                )

            # 5. Update 2D/3D maps immediately (synchronous)
            if (self.enable_2d_mapping or self.enable_3d_mapping) and (len(self.fg.keyframes) - self.last_map_update_kf >= self.map_update_interval):
                new_keyframes = list(self.fg.keyframes[self.last_map_update_kf:])

                if new_keyframes:
                    try:
                        # Update 2D map if enabled
                        if self.enable_2d_mapping and self.mapper:
                            source_frame = 'base_link_frd' if self.rov_id == "" else f"{self.rov_id}/base_link_frd"

                            self.mapper.update_global_map_from_slam(
                                new_keyframes,
                                tf2_buffer=None,
                                target_frame='world_ned',
                                source_frame=source_frame,
                                all_slam_keyframes=self.fg.keyframes
                            )

                            map_image = self.mapper.get_map_image()
                            if map_image is not None and map_image.size > 0:
                                image_msg = self.bridge.cv2_to_imgmsg(map_image, encoding="mono8")
                                image_msg.header.stamp = self.get_clock().now().to_msg()
                                image_msg.header.frame_id = "map"
                                self.map_2d_pub.publish(image_msg)
                                self.get_logger().info(
                                    f"Published 2D map: {map_image.shape[1]}x{map_image.shape[0]} pixels, "
                                    f"{len(new_keyframes)} new keyframes"
                                )

                        # Update 3D map if enabled
                        if self.enable_3d_mapping and self.mapper_3d:
                            try:
                                self.mapper_3d.update_map_from_slam(
                                    new_keyframes,
                                    all_slam_keyframes=self.fg.keyframes
                                )

                                pc_msg = self.mapper_3d.get_pointcloud2_msg(
                                    frame_id='world_ned',
                                    stamp=self.get_clock().now().to_msg()
                                )

                                if pc_msg.width > 0:
                                    self.map_3d_pub.publish(pc_msg)
                                    self.get_logger().info(
                                        f"Published 3D point cloud: {pc_msg.width} points"
                                    )
                            except Exception as e:
                                import traceback
                                self.get_logger().error(f"3D mapping update failed: {e}\n{traceback.format_exc()}")

                        # Update counter AFTER both 2D and 3D mapping
                        self.last_map_update_kf = len(self.fg.keyframes)

                    except Exception as e:
                        import traceback
                        self.get_logger().error(f"Synchronous mapping failed: {e}\n{traceback.format_exc()}")

            # Track keyframe count
            self.mapping_stats['keyframes_total'] = len(self.fg.keyframes)

        # Update current frame and publish
        if self.localization is not None:
            self.localization.current_frame = frame
        self.publish_all()

    def SLAM_callback(self, feature_msg: PointCloud2, odom_msg: Odometry) -> None:
        """SLAM call back. Subscibes to the feature msg point cloud and odom msg
            Handles the whole SLAM system and publishes map, poses and constraints

        Args:
            feature_msg (PointCloud2): the incoming sonar point cloud
            odom_msg (Odometry): the incoming DVL/IMU state estimate
        """

        self.get_logger().info("SLAM_callback", throttle_duration_sec=1.0)

        # get rostime from the point cloud
        time = feature_msg.header.stamp

        # get the dead reckoning pose from the odom msg, GTSAM pose object
        dr_pose3 = r2g(odom_msg.pose.pose)

        # init a new key frame
        frame = Keyframe(False, time, dr_pose3)

        # convert the point cloud message to a numpy array of 2D
        points = pointcloud2_to_xyz_array(feature_msg)
        # Extract [x, y] from FRD frame point cloud (original Oculus used [x, -z])
        points = np.c_[points[:, 0], points[:, 1]]

        # In case feature extraction is skipped in this frame
        if len(points) and np.isnan(points[0, 0]):
            frame.status = False
        else:
            if self.localization is not None:
                frame.status = self.localization.is_keyframe(frame)
            else:
                # In mapping-only mode, use simple time-based keyframe decision
                frame.status = True

        # set the frames twist
        frame.twist = odom_msg.twist.twist

        # update the keyframe with pose information from dead reckoning
        if self.fg.keyframes:
            dr_odom = self.fg.current_keyframe.dr_pose.between(frame.dr_pose)
            pose = self.fg.current_keyframe.pose.compose(dr_odom)
            frame.update(pose)

        # check frame staus, are we actually adding a keyframe?
        if frame.status:

            # add the point cloud to the frame
            frame.points = points

            # Conditional localization processing
            if self.mode != 'mapping-only':
                # perform seqential scan matching
                # if this is the first frame do not
                if not self.fg.keyframes:
                    self.fg.add_prior_factor(frame)
                else:
                    self.add_sequential_scan_matching(frame)

                # update the factor graph with the new frame
                self.fg.update_graph(frame)

                # if loop closures are enabled (slam mode only)
                # nonsequential scan matching is True (a loop closure occured) update graph again
                if self.mode == 'slam' and self.localization.nssm_params.enable and self.add_nonsequential_scan_matching():
                    self.fg.update_graph()
            else:
                # mapping-only mode: use DR pose directly (no localization)
                # Just add frame to keyframes list for mapping
                frame.pose = frame.dr_pose  # Use DR pose as-is
                self.fg.keyframes.append(frame)

        # update current time step and publish the topics
        if self.localization is not None:
            self.localization.current_frame = frame
        self.publish_all()

    def SLAM_callback_with_mapping(self, feature_msg: PointCloud2, odom_msg: Odometry, sonar_msg: Image) -> None:
        """SLAM callback with 2D mapping support (3-way synchronization).

        Integrates sonar image acquisition into the SLAM pipeline for 2D map generation.

        Args:
            feature_msg (PointCloud2): the incoming sonar point cloud
            odom_msg (Odometry): the incoming DVL/IMU state estimate
            sonar_msg (Image): the incoming sonar image
        """
        # 1. Convert sonar image
        try:
            sonar_image = self.bridge.imgmsg_to_cv2(sonar_msg, desired_encoding="mono8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert sonar image: {e}")
            sonar_image = None

        # 2. Standard SLAM processing
        # Use sonar timestamp for consistency (feature + mapping use same time)
        time = sonar_msg.header.stamp
        dr_pose3 = r2g(odom_msg.pose.pose)
        frame = Keyframe(False, time, dr_pose3)

        # Convert point cloud
        points = pointcloud2_to_xyz_array(feature_msg)
        points = np.c_[points[:, 0], points[:, 1]]

        # Check if valid points
        if len(points) and np.isnan(points[0, 0]):
            frame.status = False
        else:
            if self.localization is not None:
                frame.status = self.localization.is_keyframe(frame)
            else:
                # In mapping-only mode, use simple time-based keyframe decision
                frame.status = True

        # Set frame twist
        frame.twist = odom_msg.twist.twist

        # Update keyframe pose from dead reckoning
        if self.fg.keyframes:
            dr_odom = self.fg.current_keyframe.dr_pose.between(frame.dr_pose)
            pose = self.fg.current_keyframe.pose.compose(dr_odom)
            frame.update(pose)

        # 3. Process keyframe
        if frame.status:
            # Add points
            frame.points = points

            # Add sonar image to frame with acquisition timestamp
            if sonar_image is not None:
                frame.image = sonar_image
                frame.sonar_time = sonar_msg.header.stamp  # Store sonar acquisition time for mapping

            # Conditional localization processing
            if self.mode != 'mapping-only':
                # Sequential scan matching
                if not self.fg.keyframes:
                    self.fg.add_prior_factor(frame)
                else:
                    self.add_sequential_scan_matching(frame)

                # Update factor graph
                self.fg.update_graph(frame)
                self.get_logger().info(
                    f"Keyframe added: #{len(self.fg.keyframes)}, "
                    f"pose=({frame.pose.x():.2f}, {frame.pose.y():.2f}, {frame.pose.theta():.3f}), "
                    f"has_image={frame.image is not None}"
                )

                # Loop closure (slam mode only)
                if self.mode == 'slam' and self.localization.nssm_params.enable and self.add_nonsequential_scan_matching():
                    self.fg.update_graph()
            else:
                # mapping-only mode: use DR pose directly
                frame.pose = frame.dr_pose
                self.fg.keyframes.append(frame)
                self.get_logger().info(
                    f"Keyframe added (mapping-only): #{len(self.fg.keyframes)}, "
                    f"DR pose=({frame.dr_pose.x():.2f}, {frame.dr_pose.y():.2f}, {frame.dr_pose.theta():.3f}), "
                    f"has_image={frame.image is not None}"
                )

            # 4. Update 2D/3D maps immediately (synchronous)
            if (self.enable_2d_mapping or self.enable_3d_mapping) and (len(self.fg.keyframes) - self.last_map_update_kf >= self.map_update_interval):
                # Get new keyframes for incremental update
                new_keyframes = list(self.fg.keyframes[self.last_map_update_kf:])

                if new_keyframes:
                    try:
                        # Update 2D map if enabled
                        if self.enable_2d_mapping and self.mapper:
                            # Construct source_frame with namespace (use FRD for correct NED mapping)
                            source_frame = 'base_link_frd' if self.rov_id == "" else f"{self.rov_id}/base_link_frd"

                            # Update map immediately (synchronous - no queue, no thread)
                            self.mapper.update_global_map_from_slam(
                                new_keyframes,  # New keyframes to process
                                tf2_buffer=None,  # Disabled: use keyframe.pose
                                target_frame='world_ned',
                                source_frame=source_frame,
                                all_slam_keyframes=self.fg.keyframes  # Complete list for bounds calculation
                            )

                            # Get and publish map image immediately
                            map_image = self.mapper.get_map_image()
                            self.get_logger().info(
                                f"get_map_image() returned: shape={map_image.shape if map_image is not None else None}, "
                                f"size={map_image.size if map_image is not None else 0}"
                            )

                            if map_image is not None and map_image.size > 0:
                                self.get_logger().info(f"Converting to ROS Image message...")
                                image_msg = self.bridge.cv2_to_imgmsg(map_image, encoding="mono8")
                                image_msg.header.stamp = self.get_clock().now().to_msg()
                                image_msg.header.frame_id = "map"

                                self.get_logger().info(
                                    f"Publishing to {self.map_2d_pub.topic_name if hasattr(self.map_2d_pub, 'topic_name') else 'unknown'}"
                                )
                                self.map_2d_pub.publish(image_msg)
                                self.get_logger().info("publish() called successfully")

                                self.get_logger().info(
                                    f"Published 2D map: {map_image.shape[1]}x{map_image.shape[0]} pixels, "
                                    f"{len(new_keyframes)} new keyframes, "
                                    f"bounds=X[{self.mapper.min_x:.1f},{self.mapper.max_x:.1f}] "
                                    f"Y[{self.mapper.min_y:.1f},{self.mapper.max_y:.1f}]"
                                )
                            else:
                                self.get_logger().error(f"Map image is None or empty! Cannot publish.")

                        # 5. Update 3D map (independent of 2D mapping)
                        if self.enable_3d_mapping and self.mapper_3d:
                            try:
                                # Update 3D map from same new keyframes
                                self.mapper_3d.update_map_from_slam(
                                    new_keyframes,
                                    all_slam_keyframes=self.fg.keyframes
                                )

                                # Get and publish PointCloud2
                                pc_msg = self.mapper_3d.get_pointcloud2_msg(
                                    frame_id='world_ned',
                                    stamp=self.get_clock().now().to_msg()
                                )

                                if pc_msg.width > 0:  # Only publish if not empty
                                    self.map_3d_pub.publish(pc_msg)
                                    self.get_logger().info(
                                        f"Published 3D point cloud: {pc_msg.width} points, "
                                        f"frame {self.mapper_3d.frame_count}"
                                    )
                                else:
                                    # Debug: Why is point cloud empty?
                                    self.get_logger().warn(
                                        f"3D point cloud is empty after {self.mapper_3d.frame_count} frames, "
                                        f"{self.mapper_3d.get_voxel_count()} total voxels"
                                    )
                            except Exception as e:
                                import traceback
                                self.get_logger().error(f"3D mapping update failed: {e}\n{traceback.format_exc()}")

                        # Update counter AFTER both 2D and 3D mapping
                        self.last_map_update_kf = len(self.fg.keyframes)

                    except Exception as e:
                        import traceback
                        self.get_logger().error(f"Synchronous mapping failed: {e}\n{traceback.format_exc()}")

            # Track keyframe count
            self.mapping_stats['keyframes_total'] = len(self.fg.keyframes)

        # Update current frame and publish
        if self.localization is not None:
            self.localization.current_frame = frame
        self.publish_all()

    def publish_all(self) -> None:
        """Publish to all ouput topics
            trajectory, contraints, point cloud and the full GTSAM instance
        """
        if not self.fg.keyframes:
            return

        self.publish_pose()

        # Get current frame status
        if self.localization is not None:
            current_frame_status = self.localization.current_frame.status
        else:
            # mapping-only mode: use last keyframe status
            current_frame_status = self.fg.keyframes[-1].status if self.fg.keyframes else False

        if current_frame_status:
            self.publish_trajectory()
            self.publish_constraint()
            self.publish_point_cloud()

    def publish_pose(self) -> None:
        """Append dead reckoning from Localization to SLAM estimate to achieve realtime TF.
        """
        # Get current frame (either from localization or factor graph)
        if self.localization is not None:
            current_frame = self.localization.current_frame
        else:
            # mapping-only mode: use last keyframe
            if not self.fg.keyframes:
                return
            current_frame = self.fg.keyframes[-1]

        # define a pose with covariance message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = current_frame.time
        if self.rov_id == "":
            pose_msg.header.frame_id = "map"
        else:
            pose_msg.header.frame_id = self.rov_id + "_map"
        pose_msg.pose.pose = g2r(current_frame.pose3)

        cov = 1e-4 * np.identity(6, np.float32)
        # FIXME Use cov in current_frame
        if self.fg.current_keyframe is not None:
            cov[np.ix_((0, 1, 5), (0, 1, 5))] = self.fg.current_keyframe.transf_cov
        pose_msg.pose.covariance = cov.ravel().tolist()
        self.pose_pub.publish(pose_msg)

        # Note: TF broadcast removed (not used in SLAM calculations)
        # Simulator provides world_ned → base_link_frd directly
        # If navigation stack integration needed, re-add map → odom TF

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.pose.pose = pose_msg.pose.pose
        if self.rov_id == "":
            odom_msg.child_frame_id = "base_link"
        else:
            odom_msg.child_frame_id = self.rov_id + "_base_link"
        odom_msg.twist.twist = current_frame.twist
        self.odom_pub.publish(odom_msg)

    def publish_constraint(self) -> None:
        """Publish constraints between poses in the factor graph,
        either sequential or non-sequential.
        """

        # define a list of all the constraints
        links = []

        # iterate over all the keframes
        for x, kf in enumerate(self.fg.keyframes[1:], 1):

            # append each SSM factor in blue
            p1 = self.fg.keyframes[x - 1].pose3.x(), self.fg.keyframes[x - 1].pose3.y(), self.fg.keyframes[x - 1].dr_pose3.z()
            p2 = self.fg.keyframes[x].pose3.x(), self.fg.keyframes[x].pose3.y(), self.fg.keyframes[x].dr_pose3.z()
            links.append((p1, p2, "blue"))

            # loop over all loop closures in this keyframe and append them in red
            for k, _ in self.fg.keyframes[x].constraints:
                p0 = self.fg.keyframes[k].pose3.x(), self.fg.keyframes[k].pose3.y(), self.fg.keyframes[k].dr_pose3.z()
                links.append((p0, p2, "red"))

        # if nothing, do nothing
        if links:

            # conver this list to a series of multi-colored lines and publish
            link_msg = ros_constraints(links)
            link_msg.header.stamp = self.fg.current_keyframe.time
            if self.rov_id != "":
                link_msg.header.frame_id = self.rov_id + "_map"
            self.constraint_pub.publish(link_msg)

    def publish_trajectory(self) -> None:
        """Publish 3D trajectory as point cloud in [x, y, z, roll, pitch, yaw, index] format.
        """

        # get all the poses from each keyframe
        poses = np.array([g2n(kf.pose3) for kf in self.fg.keyframes])

        # convert to a ros color line
        traj_msg = ros_colorline_trajectory(poses)
        traj_msg.header.stamp = self.fg.current_keyframe.time
        if self.rov_id == "":
            traj_msg.header.frame_id = "map"
        else:
            traj_msg.header.frame_id = self.rov_id + "_map"
        self.traj_pub.publish(traj_msg)

    def publish_point_cloud(self) -> None:
        """Publish downsampled 3D point cloud with z = 0.
        The last column represents keyframe index at which the point is observed.
        """
        # 1. Collect point clouds from all keyframes
        all_points = [np.zeros((0, 2), np.float32)]

        # List of keyframe ids
        all_keys = []

        # 2. Transform each keyframe's points to global coordinate system
        for key in range(len(self.fg.keyframes)):

            # get the pose
            pose = self.fg.keyframes[key].pose

            # get the registered point cloud
            transf_points = self.fg.keyframes[key].transf_points

            # append
            all_points.append(transf_points)
            all_keys.append(key * np.ones((len(transf_points), 1)))

        # 3. Merge point clouds
        all_points = np.concatenate(all_points)
        all_keys = np.concatenate(all_keys)

        # 4. Downsample point cloud using PCL
        if self.localization is not None:
            point_resolution = self.localization.point_resolution
        else:
            point_resolution = 0.5  # Default resolution for mapping-only mode
        sampled_points, sampled_keys = pcl.downsample(
            all_points, all_keys, point_resolution
        )

        # 5. Convert to ROS message and publish
        sampled_xyzi = np.c_[sampled_points, np.zeros_like(sampled_keys), sampled_keys]

        # if there are no points return and do nothing
        if len(sampled_xyzi) == 0:
            return

        # convert the point cloud to a ros message and publish
        cloud_msg = n2r(sampled_xyzi, "PointCloudXYZI")
        cloud_msg.header.stamp = self.fg.current_keyframe.time
        if self.rov_id == "":
            cloud_msg.header.frame_id = "map"
        else:
            cloud_msg.header.frame_id = self.rov_id + "_map"
        self.cloud_pub.publish(cloud_msg)

    def add_sequential_scan_matching(self, keyframe: Keyframe) -> None:
        """Perform sequential scan matching and add to factor graph.

        Args:
            keyframe: Current keyframe to match
        """
        # Initialize SSM
        ret = self.localization.initialize_sequential_scan_matching(keyframe)

        # If initialization failed, add odometry factor only
        if not ret.status:
            self.fg.add_odometry_factor(keyframe)
            return

        # Create ICP result
        ret2 = ICPResult(ret, self.localization.ssm_params.cov_samples > 0)

        # Compute ICP
        with CodeTimer("SLAM - sequential scan matching - ICP"):
            if self.localization.ssm_params.initialization and self.localization.ssm_params.cov_samples > 0:
                message, odom, cov, sample_transforms = self.localization.compute_icp_with_cov(
                    ret2.source_points,
                    ret2.target_points,
                    ret2.initial_transforms[: self.localization.ssm_params.cov_samples],
                )

                if message != "success":
                    ret2.status = STATUS.NOT_CONVERGED
                    ret2.status.description = message
                else:
                    ret2.estimated_transform = odom
                    ret2.cov = cov
                    ret2.sample_transforms = sample_transforms
                    ret2.status.description = f"{len(ret2.sample_transforms)} samples"
            else:
                message, odom = self.localization.compute_icp(
                    ret2.source_points, ret2.target_points, ret2.initial_transform
                )

                if message != "success":
                    ret2.status = STATUS.NOT_CONVERGED
                    ret2.status.description = message
                else:
                    ret2.estimated_transform = odom
                    ret2.status.description = ""

        # Verify transform is reasonable
        if ret2.status:
            delta = ret2.initial_transform.between(ret2.estimated_transform)
            delta_translation = np.linalg.norm(delta.translation())
            delta_rotation = abs(delta.theta())
            if (
                delta_translation > self.localization.ssm_params.max_translation
                or delta_rotation > self.localization.ssm_params.max_rotation
            ):
                ret2.status = STATUS.LARGE_TRANSFORMATION
                ret2.status.description = f"trans {delta_translation:.2f} rot {delta_rotation:.2f}"

        # Check overlap
        if ret2.status:
            overlap = self.localization.get_overlap(
                ret2.source_points, ret2.target_points, ret2.estimated_transform
            )
            if overlap < self.localization.ssm_params.min_points:
                ret2.status = STATUS.NOT_ENOUGH_OVERLAP
            ret2.status.description = f"overlap {overlap}"

        # Add to graph if successful
        if ret2.status:
            self.fg.add_icp_factor(
                ret2.source_key,
                ret2.target_key,
                ret2.estimated_transform,
                ret2.cov
            )

            # Add initial guess for new pose
            target_pose = self.fg.keyframes[ret2.target_key].pose
            self.fg.values.insert(
                X(ret2.source_key), target_pose.compose(ret2.estimated_transform)
            )
            ret2.inserted = True
        else:
            # Fall back to odometry
            self.get_logger().warn(
                f"[SSM] ICP failed ({ret2.status.description}), using odometry instead. "
                f"DR delta: tx={ret.initial_transform.x():.2f}m, ty={ret.initial_transform.y():.2f}m, rot={np.degrees(ret.initial_transform.theta()):.1f}deg"
            )
            self.fg.add_odometry_factor(keyframe)

    def add_nonsequential_scan_matching(self) -> bool:
        """Perform non-sequential scan matching (loop closure detection).

        Returns:
            True if loop closure was added
        """
        # Check if we have enough keyframes
        if self.fg.current_key < self.localization.nssm_params.min_st_sep:
            return False

        # Initialize NSSM
        ret = self.localization.initialize_nonsequential_scan_matching()

        if not ret.status:
            return False

        # Create ICP result
        ret2 = ICPResult(ret, self.localization.nssm_params.cov_samples > 0)

        # Compute ICP
        with CodeTimer("SLAM - nonsequential scan matching - ICP"):
            if self.localization.nssm_params.initialization and self.localization.nssm_params.cov_samples > 0:
                message, odom, cov, sample_transforms = self.localization.compute_icp_with_cov(
                    ret2.source_points,
                    ret2.target_points,
                    ret2.initial_transforms[: self.localization.nssm_params.cov_samples],
                )

                if message != "success":
                    ret2.status = STATUS.NOT_CONVERGED
                    ret2.status.description = message
                else:
                    ret2.estimated_transform = odom
                    ret2.cov = cov
                    ret2.sample_transforms = sample_transforms
                    ret2.status.description = f"{len(ret2.sample_transforms)} samples"
            else:
                message, odom = self.localization.compute_icp(
                    ret2.source_points, ret2.target_points, ret2.initial_transform
                )

                if message != "success":
                    ret2.status = STATUS.NOT_CONVERGED
                    ret2.status.description = message
                else:
                    ret2.estimated_transform = odom
                    ret.status.description = ""

        # Verify transform
        if ret2.status:
            delta = ret2.initial_transform.between(ret2.estimated_transform)
            delta_translation = np.linalg.norm(delta.translation())
            delta_rotation = abs(delta.theta())
            if (
                delta_translation > self.localization.nssm_params.max_translation
                or delta_rotation > self.localization.nssm_params.max_rotation
            ):
                ret2.status = STATUS.LARGE_TRANSFORMATION
                ret2.status.description = f"trans {delta_translation:.2f} rot {delta_rotation:.2f}"

        # Check overlap
        if ret2.status:
            overlap = self.localization.get_overlap(
                ret2.source_points, ret2.target_points[:, :2], ret2.estimated_transform
            )
            if overlap < self.localization.nssm_params.min_points:
                ret2.status = STATUS.NOT_ENOUGH_OVERLAP
            ret2.status.description = str(overlap)

        # Add to loop closure queue for PCM verification
        if ret2.status:
            self.fg.add_loop_closure(ret2)
            return True

        return False

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        super().destroy_node()


def main(args=None):
    """Main function for SLAM node"""
    rclpy.init(args=args)

    node = SLAMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    main()
