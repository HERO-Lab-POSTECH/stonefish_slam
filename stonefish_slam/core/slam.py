# python imports
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster
import cv_bridge
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import PointCloud2, Image, CompressedImage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from octomap_msgs.msg import Octomap
from ament_index_python.packages import get_package_share_directory

# stonefish_slam imports
from stonefish_slam.utils.io import CodeTimer
from stonefish_slam.utils.conversions import X, g2n, g2r, n2g, n2r, r2g
from stonefish_slam.utils.visualization import ros_colorline_trajectory, ros_constraints
from stonefish_slam.core.factor_graph import FactorGraph
from stonefish_slam.core.localization import Localization
from stonefish_slam.core.types import Keyframe, STATUS, ICPResult
from stonefish_slam.core.mapping_2d import SonarMapping2D
from stonefish_slam.core.mapping_3d import SonarMapping3D
from stonefish_slam.core.feature_extraction import FeatureExtraction
from stonefish_slam.core.localization_fft import FFTLocalizer
from stonefish_slam.core.semantic import (
    PendingSemantic, aligned_labels, detection_rows, labels_from_detections,
    pixel_to_bearing_range)
from stonefish_slam.cpp import pcl
from stonefish_slam.utils.topics import (
    LOCALIZATION_ODOM_TOPIC, SLAM_CLOUD_TOPIC, SLAM_CONSTRAINT_TOPIC,
    SLAM_NS, SLAM_ODOM_TOPIC, SLAM_POSE_TOPIC, SLAM_TRAJ_TOPIC)


def _stamp_to_ns(stamp) -> int:
    """builtin_interfaces/Time → 나노초 정수.

    Args:
        stamp: a `builtin_interfaces.msg.Time` (a message header's stamp).

    Returns:
        int: nanoseconds since epoch, the key the pending-semantic queue is
        indexed by. Integers, so two headers copied from the same image compare
        exactly.
    """
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


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

        # Sonar parameters (slam.yaml: sonar section)
        self.declare_parameter('vehicle_name', 'bluerov2')
        # Note: sonar_image_topic is constructed from vehicle_name unless
        # sonar_topic/odom_topic override it (real-bag replay: e.g.
        # /vehicle_synced/image/fls2/image/compressed + /lig_nav).
        self.declare_parameter('sonar_topic', '')       # '' = /{vehicle_name}/fls/image
        self.declare_parameter('odom_topic', '')        # '' = /{vehicle_name}/odometry
        self.declare_parameter('sonar_compressed', False)  # True: subscribe CompressedImage
        self.declare_parameter('sonar.horizontal_fov', 130.0)
        self.declare_parameter('sonar.vertical_fov', 20.0)
        self.declare_parameter('sonar.num_beams', 512)
        self.declare_parameter('sonar.num_bins', 500)
        self.declare_parameter('sonar.range_min', 0.5)
        self.declare_parameter('sonar.range_max', 40.0)
        self.declare_parameter('sonar.sonar_position', [0.0, 0.0, 0.0])
        self.declare_parameter('sonar.sonar_tilt_deg', 30.0)

        # Feature extraction parameters (slam.yaml: feature section)
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

        # Localization parameters (slam.yaml: localization section)
        self.declare_parameter('keyframe_duration', 1.0)
        # 0.0 disables the forced-keyframe cadence; >0 forces a keyframe after
        # this many seconds even without motion (steady cadence in slow segments)
        self.declare_parameter('keyframe_duration_max', 0.0)
        self.declare_parameter('keyframe_translation', 3.0)
        self.declare_parameter('keyframe_rotation', 0.5236)
        self.declare_parameter('slam_prior_noise', [0.1, 0.1, 0.01])
        self.declare_parameter('slam_odom_noise', [0.2, 0.2, 0.02])
        self.declare_parameter('slam_icp_noise', [0.1, 0.1, 0.01])
        self.declare_parameter('slam_loop_robust_c', 3.0)
        self.declare_parameter('point_downsample_resolution', 0.5)
        self.declare_parameter('ssm.enable', False)
        self.declare_parameter('ssm.min_points', 50)
        self.declare_parameter('ssm.max_translation', 3.0)
        self.declare_parameter('ssm.max_rotation', 0.5236)
        self.declare_parameter('ssm.target_frames', 3)
        default_icp_config = os.path.join(
            get_package_share_directory('stonefish_slam'), 'config', 'icp.yaml')
        self.declare_parameter('icp_config', default_icp_config)

        # Factor graph parameters (slam.yaml: factor_graph section)
        self.declare_parameter('nssm.enable', False)
        self.declare_parameter('nssm.min_st_sep', 15)
        self.declare_parameter('nssm.min_points', 150)
        self.declare_parameter('nssm.max_translation', 5.0)
        self.declare_parameter('nssm.max_rotation', 0.5236)
        self.declare_parameter('nssm.source_frames', 5)
        self.declare_parameter('nssm.cov_samples', 30)
        # Try a loop closure only every N keyframes (1 = every keyframe).
        # NSSM cost grows O(N) with graph size; >1 bounds it on long runs.
        self.declare_parameter('nssm.try_interval', 1)
        self.declare_parameter('pcm_queue_size', 5)
        self.declare_parameter('min_pcm', 3)

        # Mapping parameters (slam.yaml: mapping section)
        self.declare_parameter('mapping_2d.map_2d_resolution', 0.2)
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
        # Probability update method + its method-specific parameters
        # (config/mapping/method_*.yaml). Defaults mirror mapping_3d.py's own
        # fallbacks so an undeclared method file cannot shift behaviour.
        self.declare_parameter('mapping_3d.update_method', 'log_odds')
        self.declare_parameter('mapping_3d.intensity_threshold', 35)
        self.declare_parameter('mapping_3d.sharpness', 1.0)
        self.declare_parameter('mapping_3d.decay_rate', 0.05)
        self.declare_parameter('mapping_3d.min_alpha', 0.3)
        self.declare_parameter('mapping_3d.use_cpp_ray_processor', True)

        # SLAM integration parameters (slam.yaml)
        self.declare_parameter('enable_2d_mapping', False)
        self.declare_parameter('enable_3d_mapping', True)
        # publish_point_cloud() rebuilds every keyframe's cloud per call
        # (O(N*P)); on dense real-bag data this was measured blocking the
        # callback thread ~30s at N=45 keyframes. Disable when the cloud
        # topic is not consumed.
        self.declare_parameter('publish_point_cloud', True)

        # FFT localization parameters
        self.declare_parameter('fft_localization.enable', False)
        self.declare_parameter('fft_localization.range_min', 0.5)
        self.declare_parameter('fft_localization.verbose', False)
        self.declare_parameter('fft_localization.trans_erosion_iterations', 4)
        self.declare_parameter('fft_localization.trans_gaussian_sigma', 4.0)
        self.declare_parameter('fft_localization.trans_gaussian_truncate', 4.0)

        # FFT validation parameters
        self.declare_parameter('fft_localization.validate_with_odom', True)
        self.declare_parameter('fft_localization.max_position_error', 2.0)  # meters
        self.declare_parameter('fft_localization.max_rotation_error', 0.35)  # radians (~20 deg)
        self.declare_parameter('fft_localization.use_dr_rotation', False)

        # Semantic (object-detection) parameters — slam.yaml `semantic:` section.
        # Every one of these is inert while `semantic.enable` is false: no
        # subscription, no extra publisher, no extra [INSTR] line, no
        # vision_msgs import, and /slam/cloud keeps its 4-field XYZI schema.
        # That run is the A/B baseline, so its OUTPUT must match the
        # pre-semantic node (the parameter list itself does grow by these five).
        self.declare_parameter('semantic.enable', False)
        self.declare_parameter('semantic.detection_topic', '')   # '' = /sonar_yolo/detections
        self.declare_parameter('semantic.max_stamp_delta', 0.05)  # s
        self.declare_parameter('semantic.pending_timeout', 3.0)   # s
        self.declare_parameter('semantic.min_conf', 0.25)
        # Landmark factors: the path by which a detection actually constrains
        # the pose graph. Sigmas are deliberately loose — the point is that the
        # detection is used, not that it dominates ICP and odometry.
        self.declare_parameter('semantic.landmark.enable', True)
        self.declare_parameter('semantic.landmark.assoc_radius', 3.0)      # m
        self.declare_parameter('semantic.landmark.range_sigma', 1.0)       # m
        self.declare_parameter('semantic.landmark.bearing_sigma_deg', 10.0)
        self.declare_parameter('semantic.landmark.robust_c', 3.0)
        # 3D voxel labels + the labelled cloud topic they are published on.
        self.declare_parameter('semantic.label_3d', True)

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

        # Mapping statistics
        self.mapping_stats = {
            'keyframes_total': 0
        }

        # 계측 카운터 (I1~I6). 위치 추정 파이프라인이 실제로 어느 경로를 탔는지
        # 세는 것이 목적이다 — "icp 0%" 같은 보고가 분모 없이 나오던 상태를 끝낸다.
        # `[INSTR]` 태그로 키프레임마다 한 줄 요약을 낸다(grep 으로 뽑아 쓴다).
        # 계측 전용이고 어떤 판정에도 쓰이지 않는다.
        self.instr = {
            'icp_attempted': 0,         # I2 — ICP 호출 횟수(분모)
            'icp_converged': 0,         # I2 — message == "success"
            'icp_factor_added': 0,      # I3 — factor graph 에 ICP factor 가 들어간 횟수
            'odom_factor_fallback': 0,  # I3 — ICP 를 돌렸으나 실패해 odometry 로 떨어진 횟수
            'ssm_init_failed': 0,       # I3 — SSM 초기화 실패로 ICP 에 도달조차 못한 횟수
            'seed_fft': 0,              # I4 — FFT 가 실제로 시드를 준 횟수
            'seed_dr_fallback': 0,      # I4 — DR fallback 이 시드로 쓰인 횟수
            'reject_pos': 0,            # I6 — 위치 오차로 기각
            'reject_rot': 0,            # I6 — 회전 오차로 기각
        }

        # Mapping initialization (configured in init_node)
        self.mapper = None
        # Set before init_node() so every consumer can read it; _init_semantic
        # re-reads the parameter and is the only place that turns it on.
        self.semantic_enable = False
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
        self._init_keyframe_and_noise_params()
        self._init_sonar_and_scan_matching_params()
        self._init_mappers()
        self._init_fft_localizer()
        self._init_subscribers_and_publishers()
        self._init_semantic()
        self._finalize_node_config()

    def _init_keyframe_and_noise_params(self) -> None:
        """Configures keyframe criteria and noise models (loaded from slam.yaml)."""
        # keyframe paramters, how often to add them (loaded from slam.yaml)
        keyframe_duration_sec = self.get_parameter('keyframe_duration').value
        keyframe_duration = Duration(seconds=keyframe_duration_sec)
        keyframe_duration_max_sec = self.get_parameter('keyframe_duration_max').value
        keyframe_duration_max = (
            Duration(seconds=keyframe_duration_max_sec)
            if keyframe_duration_max_sec > 0.0 else None)
        keyframe_translation = self.get_parameter('keyframe_translation').value
        keyframe_rotation = self.get_parameter('keyframe_rotation').value

        # Set keyframe criteria in localization module (skip if mapping-only mode)
        if self.localization is not None:
            self.localization.keyframe_duration = keyframe_duration
            self.localization.keyframe_duration_max = keyframe_duration_max
            self.localization.keyframe_translation = keyframe_translation
            self.localization.keyframe_rotation = keyframe_rotation

        # noise models (loaded from slam.yaml)
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

    def _init_sonar_and_scan_matching_params(self) -> None:
        """Configures sonar hardware, SSM, NSSM, and PCM parameters (loaded from slam.yaml)."""
        # resultion for map downsampling (loaded from slam.yaml)
        point_resolution = self.get_parameter('point_downsample_resolution').value
        if self.localization is not None:
            self.localization.point_resolution = point_resolution

        # Sonar configuration (loaded from slam.yaml)
        if self.localization is not None:
            # Configure oculus object with parameters from slam.yaml (sonar section)
            # Note: In original code, this was done via oculus.configure(ping) message callback
            # Now we use ROS2 parameters instead
            sonar_range_max = self.get_parameter('sonar.range_max').value
            sonar_range_min = self.get_parameter('sonar.range_min').value
            sonar_horizontal_fov = self.get_parameter('sonar.horizontal_fov').value
            sonar_vertical_fov = self.get_parameter('sonar.vertical_fov').value
            # Get sonar dimensions from slam.yaml (sonar section)
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

        # sequential scan matching parameters (SSM) (loaded from slam.yaml)
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

        # non sequential scan matching parameters (NSSM) aka loop closures (loaded from slam.yaml)
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
        self.nssm_try_interval = max(1, self.get_parameter('nssm.try_interval').value)

        # pairwise consistency maximization parameters for loop closure (loaded from slam.yaml)
        self.fg.pcm_queue_size = self.get_parameter('pcm_queue_size').value
        self.fg.min_pcm = self.get_parameter('min_pcm').value

    def _init_mappers(self) -> None:
        """Configures and creates the 2D and 3D sonar mappers (loaded from slam.yaml)."""
        # ===== Sonar Hardware Parameters ===== (loaded from slam.yaml)
        # ===== 2D Mapping Parameters ===== (slam.yaml: mapping_2d)
        # ===== 3D Mapping Parameters ===== (slam.yaml: mapping_3d)
        # ===== Mapping Enable Flags ===== (slam.yaml)

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

        # 0.2 m/px keeps the grid message ~1 MB (sonar_range/sonar_bins ≈ 0.078 m
        # would be 26x). The yaml key used to advertise 0.1 while this line
        # hard-coded 0.2; the parameter is the only source now.
        map_resolution = self.get_parameter('mapping_2d.map_2d_resolution').value

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
                # 3D threshold, not self.intensity_threshold (that one is the
                # 2D occupancy grid's, mapping_2d.intensity_threshold).
                'intensity_threshold': self.get_parameter('mapping_3d.intensity_threshold').value,

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
                'propagation_radius': self.get_parameter('mapping_3d.propagation_radius').value,
                'propagation_sigma': self.get_parameter('mapping_3d.propagation_sigma').value,
                'use_range_weighting': self.get_parameter('mapping_3d.use_range_weighting').value,
                'lambda_decay': self.get_parameter('mapping_3d.lambda_decay').value,
                'enable_gaussian_weighting': self.get_parameter('mapping_3d.enable_gaussian_weighting').value,
                'use_dda_traversal': self.get_parameter('mapping_3d.use_dda_traversal').value,
                'bearing_step': self.get_parameter('mapping_3d.bearing_step').value,

                # Update method + its method-specific params. Without these
                # SonarMapping3D falls back to log_odds regardless of what
                # config/slam.yaml and the method_*.yaml file select.
                'update_method': self.get_parameter('mapping_3d.update_method').value,
                'sharpness': self.get_parameter('mapping_3d.sharpness').value,
                'decay_rate': self.get_parameter('mapping_3d.decay_rate').value,
                'min_alpha': self.get_parameter('mapping_3d.min_alpha').value,
                'use_cpp_ray_processor': self.get_parameter('mapping_3d.use_cpp_ray_processor').value,

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

    def _init_fft_localizer(self) -> None:
        """Configures the optional FFT localizer (loaded from fft_localization.* params)."""
        # FFT localizer initialization (optional)
        self.fft_enable = self.get_parameter('fft_localization.enable').value
        if self.fft_enable:
            if self.localization is not None:
                fft_range_min = self.get_parameter('fft_localization.range_min').value

                # Pass sonar tilt angle to OculusProperty
                sonar_tilt_deg = self.get_parameter('sonar.sonar_tilt_deg').value
                self.localization.oculus.tilt_angle_deg = sonar_tilt_deg
                self.localization.oculus.tilt_angle_rad = np.deg2rad(sonar_tilt_deg)

                fft_verbose = self.get_parameter('fft_localization.verbose').value
                fft_erosion = self.get_parameter('fft_localization.trans_erosion_iterations').value
                fft_gaussian_sigma = self.get_parameter('fft_localization.trans_gaussian_sigma').value
                fft_gaussian_truncate = self.get_parameter('fft_localization.trans_gaussian_truncate').value
                self.fft_localizer = FFTLocalizer(
                    oculus=self.localization.oculus,
                    range_min=fft_range_min,
                    verbose=fft_verbose,
                    trans_erosion_iterations=fft_erosion,
                    trans_gaussian_sigma=fft_gaussian_sigma,
                    trans_gaussian_truncate=fft_gaussian_truncate
                )
                self.get_logger().info(f"FFT localization enabled (tilt={sonar_tilt_deg}°)")

                # FFT validation parameters
                self.fft_validate = self.get_parameter('fft_localization.validate_with_odom').value
                self.fft_max_pos_error = self.get_parameter('fft_localization.max_position_error').value
                self.fft_max_rot_error = self.get_parameter('fft_localization.max_rotation_error').value
                self.fft_use_dr_rotation = self.get_parameter('fft_localization.use_dr_rotation').value

                # Previous polar sonar image storage
                self.prev_polar_sonar = None
            else:
                self.get_logger().warn("FFT localization disabled: requires localization module (not available in mapping-only mode)")
                self.fft_enable = False
                self.fft_localizer = None
        else:
            self.fft_localizer = None
            self.get_logger().info("FFT localization disabled")

    def _init_subscribers_and_publishers(self) -> None:
        """Sets up QoS profiles, subscribers, the time synchronizer, and all publishers."""
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
        # Topic overrides support real-bag replay (e.g. compressed FLS + external nav).
        vehicle_name = self.get_parameter('vehicle_name').value
        sonar_image_topic = self.get_parameter('sonar_topic').value or f'/{vehicle_name}/fls/image'
        odom_topic = self.get_parameter('odom_topic').value or f'/{vehicle_name}/odometry'
        self.sonar_compressed = self.get_parameter('sonar_compressed').value
        sonar_msg_type = CompressedImage if self.sonar_compressed else Image
        self.sonar_sub = Subscriber(self, sonar_msg_type, sonar_image_topic, qos_profile=qos_sub_profile)
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
        self.time_sync.registerCallback(self.slam_callback_integrated)
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
                Octomap,
                SLAM_NS + 'mapping/map_3d_octomap',
                qos_profile=qos_pointcloud_pub_profile
            )
            self.get_logger().info(f"Publishing 3D map to: {SLAM_NS}mapping/map_3d_octomap (QoS: RELIABLE)")
        # The labelled 3D cloud is created in _init_semantic, not here: with the
        # feature off the topic must not exist at all.
        self.cloud_3d_pub = None
        self.qos_pointcloud_pub_profile = qos_pointcloud_pub_profile

        # tf broadcaster to show pose
        self.tf = TransformBroadcaster(self)

        # cv bridge object

    def _init_semantic(self) -> None:
        """Wires the object-detection input, or leaves the node untouched.

        Returns early — before importing vision_msgs, before creating any
        subscription or counter — when `semantic.enable` is false, so a machine
        without `vision_msgs` runs exactly the node it ran before.
        """
        self.semantic_enable = self.get_parameter('semantic.enable').value
        if not self.semantic_enable:
            return

        # Imported here, not at module scope: the dependency must not be
        # required by a run that has the feature switched off.
        from vision_msgs.msg import Detection2DArray

        self.semantic_min_conf = float(self.get_parameter('semantic.min_conf').value)
        self.landmark_enable = self.get_parameter('semantic.landmark.enable').value
        self.label_3d = self.get_parameter('semantic.label_3d').value
        # Keyframes whose detections are confirmed but whose voxels have not been
        # labelled yet. NOT the same set as `new_keyframes`: a detection can land
        # on a keyframe the mapper already consumed, and
        # `keyframes[last_map_update_kf:]` never hands that one back.
        self.pending_semantic_map = []
        if self.landmark_enable:
            self.fg.landmark_assoc_radius = float(
                self.get_parameter('semantic.landmark.assoc_radius').value)
            self.fg.landmark_sigmas = np.array([
                np.radians(float(self.get_parameter('semantic.landmark.bearing_sigma_deg').value)),
                float(self.get_parameter('semantic.landmark.range_sigma').value)])
            self.fg.robust_landmark_c = float(
                self.get_parameter('semantic.landmark.robust_c').value)
        max_delta_s = float(self.get_parameter('semantic.max_stamp_delta').value)
        timeout_s = float(self.get_parameter('semantic.pending_timeout').value)
        self.pending_semantic = PendingSemantic(
            int(max_delta_s * 1e9), int(timeout_s * 1e9))

        # 계측 카운터 (semantic). `[INSTR] semantic` 로 따로 나간다 — 기존
        # `[INSTR] counters` 줄은 off/on 어느 쪽에서도 형식이 그대로여야 한다.
        self.semantic_instr = {
            'det_received': 0,          # 구독한 Detection2DArray 메시지 수
            'det_empty': 0,             # 그중 **발행자가** 탐지 0건으로 보낸 메시지 수
                                        # (필터로 비워진 것은 아래 두 카운터가 센다)
            'det_below_conf': 0,        # min_conf 미만이라 버린 탐지 수
            'det_bad_class': 0,         # class_id 가 정수 문자열이 아니라 버린 탐지 수
            'det_duplicate': 0,         # 같은 stamp 의 **미결** 검출을 덮어쓴 횟수
                                        # (이미 소비된 stamp 의 재전송은 여기 안 잡힌다)
            'kf_stamp_collision': 0,    # 같은 stamp 의 미결 키프레임을 덮어쓴 횟수
            'det_missing': 0,           # 워터마크까지 검출이 안 온 키프레임 수
            'det_expired': 0,           # 짝지을 키프레임 없이 만료된 검출 수
            'det_matched': 0,           # 키프레임과 짝지어진 검출 메시지 수
            'det_no_labeled_peaks': 0,  # 짝은 맞았으나 bbox 안 CFAR 피크가 0
            'landmark_factors_added': 0,  # ★ "검출이 위치 추정에 쓰였다"의 유일한 증거
            'landmarks_created': 0,     # 새로 만들어진 랜드마크 변수 수
            'voxels_labeled': 0,        # 역투영으로 라벨이 붙은 복셀 수(누적)
        }

        if self.label_3d and self.enable_3d_mapping:
            self.cloud_3d_pub = self.create_publisher(
                PointCloud2, SLAM_NS + 'mapping/cloud_3d',
                qos_profile=self.qos_pointcloud_pub_profile)
            self.get_logger().info(
                f"Publishing labelled 3D cloud to: {SLAM_NS}mapping/cloud_3d")

        topic = self.get_parameter('semantic.detection_topic').value or '/sonar_yolo/detections'
        self.detection_sub = self.create_subscription(
            Detection2DArray, topic, self.semantic_detection_callback, 10)
        self.get_logger().info(
            f"Semantic labelling enabled: subscribing to {topic} "
            f"(max_stamp_delta={max_delta_s}s, pending_timeout={timeout_s}s, "
            f"min_conf={self.semantic_min_conf}, landmark={self.landmark_enable})")

    def _detections_from_msg(self, msg) -> np.ndarray:
        """Detection2DArray → Kx6 `[class, conf, x1, y1, x2, y2]` 픽셀 배열.

        Args:
            msg (vision_msgs.msg.Detection2DArray): 구독한 메시지.

        Returns:
            np.ndarray: (K, 6) float32. `min_conf` 미만이거나 `class_id` 가
            정수 문자열이 아닌 탐지는 카운터를 올리고 버린다 — `class_id` 는
            `vision_msgs` 규격상 문자열이라 발행자가 이름을 실을 수도 있고,
            그건 이 파이프라인이 라벨로 쓸 수 없는 값이다.
        """
        items = [
            (det.results[0].hypothesis.class_id,
             det.results[0].hypothesis.score,
             det.bbox.center.position.x, det.bbox.center.position.y,
             det.bbox.size_x, det.bbox.size_y)
            for det in msg.detections if det.results
        ]
        rows, n_below_conf, n_bad_class = detection_rows(items, self.semantic_min_conf)
        self.semantic_instr['det_below_conf'] += n_below_conf
        self.semantic_instr['det_bad_class'] += n_bad_class
        return rows

    def semantic_detection_callback(self, msg) -> None:
        """Detection2DArray 를 받아 같은 소나 프레임의 키프레임에 붙인다.

        검출은 추론 지연 때문에 그 키프레임보다 늦게 오는 것이 정상이므로,
        짝이 아직 없으면 큐에 남겨 두고 키프레임 쪽에서 집어 간다.

        Args:
            msg (vision_msgs.msg.Detection2DArray): 발행자가 이미지 header 를
                복사해 둔 메시지.
        """
        stamp_ns = _stamp_to_ns(msg.header.stamp)
        self.semantic_instr['det_received'] += 1
        if not msg.detections:
            # 발행자가 "추론했고 아무것도 없었다"로 보낸 것. 신뢰도·class_id
            # 필터로 비워진 것과 구분해야 A/B 에서 검출기 상태를 읽을 수 있다.
            self.semantic_instr['det_empty'] += 1
        dets = self._detections_from_msg(msg)

        if self.pending_semantic.has_detection(stamp_ns):
            self.semantic_instr['det_duplicate'] += 1
        pending = self.pending_semantic.offer_detection(stamp_ns, dets)
        if pending is not None:
            frame, peak_locs, pose_key = pending
            self._apply_semantic(frame, peak_locs, pose_key, dets)
            # 늦게 온 검출은 slam 콜백 밖에서 소비되므로, 여기서 요약을 내지
            # 않으면 `det_matched` 증가가 로그에 영영 안 나타난다.
            self._log_instrumentation()

        # 키프레임이 더 안 생기는 구간(정지·특징 없음)에서도 큐가 자라지 않도록
        # 검출 쪽에서도 워터마크를 돌린다.
        self._expire_semantic(stamp_ns)

    def _apply_semantic(self, frame: Keyframe, peak_locs, pose_key: int, dets) -> None:
        """검출을 키프레임의 점 라벨과 랜드마크 factor 로 굳힌다.

        Args:
            frame (Keyframe): 짝이 맞은 키프레임.
            peak_locs: 그 키프레임의 CFAR 피크 픽셀 (N, 2) `[row, col]`.
            pose_key (int): 그 키프레임이 factor graph 에서 갖는 X 인덱스.
            dets: `_detections_from_msg` 의 (K, 6) 배열.
        """
        self.semantic_instr['det_matched'] += 1
        frame.detections = dets
        frame.labels = labels_from_detections(peak_locs, dets)
        if len(dets) > 0 and not frame.labels.any():
            # bbox 는 왔는데 그 안에 CFAR 피크가 하나도 없다 — 3D 라벨이
            # 비는 원인이라 따로 센다(랜드마크 factor 와는 무관하다).
            self.semantic_instr['det_no_labeled_peaks'] += 1
        if self.landmark_enable and self.mode != 'mapping-only':
            self._add_landmark_factors(frame, pose_key, dets)
        if self.label_3d and len(dets) > 0:
            self.pending_semantic_map.append(frame)

    def _add_landmark_factors(self, frame: Keyframe, pose_key: int, dets) -> None:
        """검출 하나마다 BearingRange factor 하나를 factor graph 에 넣는다.

        측정값은 **bbox 중심 픽셀**이다. 라벨이 붙은 CFAR 점의 centroid 를 쓰면
        bbox 안에 피크가 하나도 없는 프레임에서 factor 가 아예 안 생기는데, 그러면
        "검출이 위치 추정에 쓰였는가"의 답이 검출기 성능에 의존해 버린다. bbox
        중심을 쓰면 검출 1건 ⇒ factor 1건이 구성상 보장되고, factor 가 0 이 되는
        원인은 검출 부재·stamp 불일치·ISAM2 실패 셋뿐이라 카운터로 갈린다.

        Args:
            frame (Keyframe): 짝이 맞은 키프레임.
            pose_key (int): 그 키프레임의 X 인덱스.
            dets: (K, 6) `[class, conf, x1, y1, x2, y2]`.
        """
        for cls, _conf, x1, y1, x2, y2 in dets:
            col_c, row_c = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
            bearing, rng = pixel_to_bearing_range(
                row_c, col_c,
                num_bins=self.feature_extractor.num_bins,
                num_beams=self.feature_extractor.num_beams,
                range_min=self.feature_extractor.range_min,
                range_max=self.feature_extractor.range_max,
                horizontal_fov_deg=self.feature_extractor.horizontal_fov)
            world_guess = frame.pose.transformFrom(
                np.array([rng * np.cos(bearing), rng * np.sin(bearing)]))
            try:
                _j, is_new = self.fg.add_landmark_factor(
                    pose_key, int(cls), float(bearing), float(rng), world_guess)
            except Exception as e:  # noqa: BLE001 — 계측이 붙은 부가 경로다
                # A landmark factor must never take the node down: it is the
                # formal "detection was used" path, not the estimator itself.
                self.get_logger().error(f"landmark factor failed: {e}")
                continue
            self.semantic_instr['landmark_factors_added'] += 1
            if is_new:
                self.semantic_instr['landmarks_created'] += 1

    def _expire_semantic(self, now_ns: int) -> None:
        """워터마크를 넘긴 미결 항목을 정리하고 센다.

        키프레임 콜백과 검출 콜백 **양쪽에서** 부른다. 한쪽에서만 부르면 다른
        쪽만 들어오는 구간(차량 정지로 키프레임이 안 생기거나, 특징이 없어
        키프레임 판정이 계속 false 인 구간)에서 큐가 무한히 자란다.

        Args:
            now_ns (int): 방금 처리한 소나 프레임의 stamp(ns).
        """
        stale_kfs, stale_dets = self.pending_semantic.expire(now_ns)
        self.semantic_instr['det_missing'] += len(stale_kfs)
        self.semantic_instr['det_expired'] += len(stale_dets)

    def _offer_keyframe_semantic(self, frame: Keyframe, peak_locs, stamp_ns: int) -> None:
        """키프레임을 미결 큐에 넣고, 워터마크를 넘긴 항목을 정리한다.

        Args:
            frame (Keyframe): 방금 만들어진 키프레임.
            peak_locs: 그 키프레임의 CFAR 피크 픽셀 (N, 2).
            stamp_ns (int): 그 소나 프레임의 stamp(ns).
        """
        # The keyframe is offered before update_graph() appends it, so the X
        # index it will get is the current length. Carrying it in the payload
        # keeps the key stable for a detection that arrives several frames later.
        pose_key = len(self.fg.keyframes)
        if self.pending_semantic.has_keyframe(stamp_ns):
            self.semantic_instr['kf_stamp_collision'] += 1
        dets = self.pending_semantic.offer_keyframe(
            stamp_ns, (frame, peak_locs, pose_key))
        if dets is not None:
            self._apply_semantic(frame, peak_locs, pose_key, dets)

        self._expire_semantic(stamp_ns)

    def _label_and_publish_cloud_3d(self) -> None:
        """미결 키프레임의 검출을 복셀 라벨로 굳히고 라벨 점군을 발행한다.

        `new_keyframes` 로는 부족하다 — 검출이 늦게 도착한 키프레임은 이미
        `last_map_update_kf` 뒤로 지나가 다시는 목록에 안 들어온다. 그래서
        검출이 확정된 키프레임을 따로 모아 두었다가 여기서 비운다.
        """
        try:
            for frame in self.pending_semantic_map:
                self.semantic_instr['voxels_labeled'] += \
                    self.mapper_3d.label_voxels_from_keyframe(
                        self.mapper_3d.keyframe_pose_dict(frame), frame.detections)
            self.pending_semantic_map.clear()

            points, probs, labels = self.mapper_3d.get_labeled_point_cloud()
            if len(points) == 0:
                return
            cloud = np.c_[points, probs.reshape(-1, 1),
                          labels.reshape(-1, 1).astype(np.float64)]
            msg = n2r(cloud, "PointCloudXYZPL")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world_ned"
            self.cloud_3d_pub.publish(msg)
        except Exception as e:  # noqa: BLE001 — 부가 산출물이 매핑을 못 죽인다
            import traceback
            self.get_logger().error(
                f"labelled 3D cloud failed: {e}\n{traceback.format_exc()}")

    def _finalize_node_config(self) -> None:
        """Loads ICP config, extracts the robot ID, and calls configure() to finish init."""
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
        self.fg.robust_loop_c = self.get_parameter('slam_loop_robust_c').value

    def validate_fft_with_odom(self, fft_result: dict, dr_transform):
        """
        Validate FFT result against dead reckoning odometry.

        Args:
            fft_result: FFT estimate_transform result with covariance
            dr_transform: Dead reckoning transform (gtsam.Pose2)

        Returns:
            (is_valid, validation_info dict)
        """
        # FFT transform
        fft_tx = fft_result['translation'][0]
        fft_ty = fft_result['translation'][1]
        fft_theta = np.radians(fft_result['rotation'])

        # DR transform
        dr_tx = dr_transform.x()
        dr_ty = dr_transform.y()
        dr_theta = dr_transform.theta()

        # Position error
        pos_error = np.sqrt((fft_tx - dr_tx)**2 + (fft_ty - dr_ty)**2)

        # Rotation error (wrap to [-pi, pi])
        rot_error = abs(fft_theta - dr_theta)
        rot_error = min(rot_error, 2*np.pi - rot_error)

        # Validation info
        info = {
            'fft': (fft_tx, fft_ty, np.degrees(fft_theta)),
            'dr': (dr_tx, dr_ty, np.degrees(dr_theta)),
            'pos_err': pos_error,
            'rot_err_deg': np.degrees(rot_error),
            'pos_threshold': self.fft_max_pos_error,
            'rot_threshold_deg': np.degrees(self.fft_max_rot_error),
            'reasons': []
        }

        # Validation checks
        if pos_error > self.fft_max_pos_error:
            info['reasons'].append('pos')
        if rot_error > self.fft_max_rot_error:
            info['reasons'].append('rot')

        # I6 — 기각 사유를 나눠 센다. `use_dr_rotation: true` 에서는 회전 오차가
        # 항등 0 이라 'rot' 이 영영 0 으로 나와야 한다(회전 게이트 사문화 가설).
        # 그게 실측되면 17% 기각의 분모와 사유가 확정된다.
        # 키를 f-string 으로 만들지 않는 이유는 두 가지다 — 선언에 없는 사유가
        # 추가되면 KeyError 로 죽고, 정적 검증(테스트)이 닿지 않는다.
        if 'pos' in info['reasons']:
            self.instr['reject_pos'] += 1
        if 'rot' in info['reasons']:
            self.instr['reject_rot'] += 1

        # I7~I10 — 판정에 쓰지 않고 기록만 하는 값들. FFT 가 이미 계산해 반환
        # dict 에 실어 보내는데 아무도 읽지 않던 것들이라 소비만 추가한다.
        #   I7  dr_ty  : 기각이 |ty| 큰 구간에 몰리면 ty 부호 오류, 균등하면 게이트가 빡빡한 것
        #   I8  peak   : 품질 게이트 임계값을 정하려면 분포가 먼저 필요하다(P1-5)
        #   I9  cov    : FFT 자체 불확실도가 DR 불일치를 예측하는지 상관 분석용
        #   I10 rot_fft: `use_dr_rotation` 을 끌 수 있는지 판단할 유일한 근거
        # np.diag 는 입력 차원에 따라 길이가 달라진다 — 2x2 면 길이 2, 1-D 면 정사각
        # 행렬. 로그 한 줄 때문에 IndexError 가 나면 caller 의 broad except 가 그걸
        # FFT 실패로 바꾸므로 "계측은 판정을 바꾸지 않는다" 는 전제가 깨진다.
        # 길이를 3 으로 못 박아 그 경로 자체를 없앤다(적대 검증 NIT).
        cov = fft_result.get('covariance')
        cov_diag = tuple(np.ravel(np.diag(np.atleast_2d(cov)))) if cov is not None else ()
        cov_diag = cov_diag + (np.nan,) * 3
        self.get_logger().info(
            f"[INSTR] gate pos_err={pos_error:.3f} rot_err_deg={np.degrees(rot_error):.2f} "
            f"dr_ty={dr_ty:.3f} "
            f"rot_peak={fft_result.get('rot_peak', float('nan')):.4f} "
            f"trans_peak={fft_result.get('trans_peak', float('nan')):.4f} "
            f"cov=({cov_diag[0]:.4f},{cov_diag[1]:.4f},{cov_diag[2]:.4f}) "
            f"rot_fft={fft_result.get('rotation_fft', float('nan')):.2f} "
            f"rot_used={fft_result['rotation']:.2f} "
            f"dr_rot_override={fft_result.get('rotation_override_used', False)} "
            f"reasons={'|'.join(info['reasons']) or 'none'}"
        )

        return len(info['reasons']) == 0, info

    def slam_callback_integrated(self, sonar_msg, odom_msg: Odometry) -> None:
        """Integrated SLAM callback with internal feature extraction.

        Replaces the old 3-way synchronization (feature + odom + sonar).
        Now uses 2-way sync (sonar + odom) with internal feature extraction.

        Args:
            sonar_msg (Image | CompressedImage): Sonar image message (polar
                coordinates); type follows the sonar_compressed parameter
            odom_msg (Odometry): Dead reckoning odometry
        """
        # 1. Extract features internally using FeatureExtraction module
        # (ICP, mapping, keyframe 판단 등에 모두 필요 - 항상 수행)
        try:
            if self.semantic_enable:
                points, peak_locs = \
                    self.feature_extractor.extract_features_with_pixels(sonar_msg)
            else:
                # Unchanged call on the off path — extract_features delegates,
                # so the baseline run executes exactly what it did before.
                points = self.feature_extractor.extract_features(sonar_msg)
                peak_locs = None
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
                if self.sonar_compressed:
                    sonar_image = self.bridge.compressed_imgmsg_to_cv2(
                        sonar_msg, desired_encoding="passthrough")
                    if sonar_image.ndim == 3:
                        import cv2 as _cv2
                        sonar_image = _cv2.cvtColor(sonar_image, _cv2.COLOR_BGR2GRAY)

                    # FFT localization reuses the decoded mono8 polar image
                    # (CompressedImage has no raw height/width buffer to reshape)
                    if self.fft_enable:
                        polar_sonar = sonar_image
                else:
                    sonar_image = self.bridge.imgmsg_to_cv2(sonar_msg, desired_encoding="mono8")

                    # FFT localization (polar sonar image 필요)
                    if self.fft_enable:
                        # Polar image는 sonar_msg.data를 직접 변환
                        polar_sonar = np.frombuffer(sonar_msg.data, dtype=np.uint8).reshape(
                            sonar_msg.height, sonar_msg.width
                        )

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
            # Keep the label array the same length as the points it indexes;
            # the Keyframe was constructed before the points were known.
            frame.labels = np.zeros(len(points), np.uint8)

            # Add sonar image to frame
            if sonar_image is not None:
                frame.image = sonar_image
                frame.sonar_time = sonar_msg.header.stamp

            # Pair this keyframe with its detection. The detection normally
            # arrives later (YOLO inference lag), so this only queues it; the
            # detection callback finishes the job when it lands.
            if self.semantic_enable:
                self._offer_keyframe_semantic(
                    frame, peak_locs, _stamp_to_ns(sonar_msg.header.stamp))

            # FFT localization (only for keyframes)
            if self.fft_enable and self.prev_polar_sonar is not None and polar_sonar is not None:
                try:
                    polar_prev = self.prev_polar_sonar.copy()
                    polar_curr = polar_sonar.copy()

                    # DR 회전 추출 (use_dr_rotation 설정 시)
                    rotation_override = None
                    if self.fft_use_dr_rotation and self.fg.keyframes:
                        dr_transform = self.fg.current_keyframe.dr_pose.between(frame.dr_pose)
                        rotation_override = np.degrees(dr_transform.theta())

                    fft_result = self.fft_localizer.estimate_transform(
                        polar_prev, polar_curr,
                        rotation_override=rotation_override
                    )

                    if fft_result['success']:
                        # Validate with odometry if enabled
                        fft_valid = True
                        val_info = None

                        if self.fft_validate and self.fg.keyframes:
                            # Get DR transform between keyframes
                            dr_transform = self.fg.current_keyframe.dr_pose.between(frame.dr_pose)
                            fft_valid, val_info = self.validate_fft_with_odom(fft_result, dr_transform)

                        if fft_valid:
                            # FFT valid - use FFT result
                            frame.fft_transform = n2g(
                                (fft_result['translation'][0],
                                 fft_result['translation'][1],
                                 np.radians(fft_result['rotation'])),
                                "Pose2"
                            )
                            frame.fft_covariance = fft_result.get('covariance', None)
                            frame.fft_success = True
                            frame.fft_is_dr_fallback = False

                            # Log: USE FFT [DR_ROT] | FFT(...) DR(...) | Δ(...)
                            if val_info:
                                mode_str = "[DR_ROT]" if fft_result.get('rotation_override_used', False) else "[FFT_ROT]"
                                fft = val_info['fft']
                                dr = val_info['dr']
                                self.get_logger().info(
                                    f"USE FFT {mode_str} | FFT({fft[0]:.2f},{fft[1]:.2f},{fft[2]:.1f}°) "
                                    f"DR({dr[0]:.2f},{dr[1]:.2f},{dr[2]:.1f}°) | "
                                    f"Δ({val_info['pos_err']:.2f}m,{val_info['rot_err_deg']:.1f}°)"
                                )
                        else:
                            # Log: USE DR | FFT(...) DR(...) | reason
                            fft = val_info['fft']
                            dr = val_info['dr']
                            reasons = []
                            if 'pos' in val_info['reasons']:
                                reasons.append(f"pos {val_info['pos_err']:.2f}>{val_info['pos_threshold']:.2f}m")
                            if 'rot' in val_info['reasons']:
                                reasons.append(f"rot {val_info['rot_err_deg']:.1f}>{val_info['rot_threshold_deg']:.1f}°")
                            self.get_logger().warn(
                                f"USE DR  | FFT({fft[0]:.2f},{fft[1]:.2f},{fft[2]:.1f}°) "
                                f"DR({dr[0]:.2f},{dr[1]:.2f},{dr[2]:.1f}°) | "
                                f"{', '.join(reasons)}",
                                throttle_duration_sec=1.0
                            )
                            frame.fft_transform = dr_transform
                            frame.fft_covariance = None
                            frame.fft_success = True  # Still mark as success to use DR transform
                            frame.fft_is_dr_fallback = True  # Flag for DR fallback
                    else:
                        self.get_logger().warn("FFT localization failed", throttle_duration_sec=2.0)
                        frame.fft_success = False

                except Exception as e:
                    self.get_logger().error(f"FFT localization error: {e}")
                    frame.fft_success = False

            # Update prev_polar_sonar for next keyframe (FFT only)
            if self.fft_enable and polar_sonar is not None:
                self.prev_polar_sonar = polar_sonar.copy()

            # Conditional localization processing
            if self.mode != 'mapping-only':
                # Sequential scan matching
                if not self.fg.keyframes:
                    self.fg.add_prior_factor(frame)
                else:
                    self.add_sequential_scan_matching(frame)

                # Update factor graph
                self.fg.update_graph(frame)

                # Loop closure (slam mode only) — throttled via nssm.try_interval
                # to bound NSSM's O(N) candidate search on long runs
                if self.mode == 'slam' and self.localization.nssm_params.enable \
                        and (self.fg.current_key % self.nssm_try_interval == 0) \
                        and self.add_nonsequential_scan_matching():
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
                                source_frame=source_frame
                            )

                            map_image = self.mapper.get_map_image()
                            if map_image is not None and map_image.size > 0:
                                image_msg = self.bridge.cv2_to_imgmsg(map_image, encoding="mono8")
                                image_msg.header.stamp = self.get_clock().now().to_msg()
                                image_msg.header.frame_id = "world_ned"
                                self.map_2d_pub.publish(image_msg)
                                self.get_logger().info(
                                    f"Published 2D map: {map_image.shape[1]}x{map_image.shape[0]} pixels, "
                                    f"{len(new_keyframes)} new keyframes"
                                )

                        # Update 3D map if enabled
                        if self.enable_3d_mapping and self.mapper_3d:
                            try:
                                self.mapper_3d.update_map_from_slam(
                                    new_keyframes
                                )

                                octomap_msg = self.mapper_3d.get_octomap_msg(
                                    frame_id='world_ned',
                                    stamp=self.get_clock().now().to_msg()
                                )

                                if len(octomap_msg.data) > 0:
                                    self.map_3d_pub.publish(octomap_msg)
                                    self.get_logger().info(
                                        f"Published 3D octomap: {len(octomap_msg.data)} bytes"
                                    )

                                if self.semantic_enable and self.label_3d:
                                    self._label_and_publish_cloud_3d()
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
            if self.get_parameter('publish_point_cloud').value:
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
        # Stamp in the global frame so RViz (Fixed Frame world_ned) can render it.
        # The former {rov_id}_world_ned frame had no TF link to world_ned and no
        # consumer other than RViz, so visualization silently failed.
        pose_msg.header.frame_id = "world_ned"
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

        # Single-topic SLAM output:
        #   header.frame_id        : global frame  (world_ned)
        #   pose.pose              : position + orientation (global frame)
        #   pose.covariance        : 6x6 SLAM pose uncertainty
        #   child_frame_id         : vehicle frame ({rov_id}_base_link)
        #   twist.twist            : linear/angular velocity (vehicle frame)
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.pose.pose = pose_msg.pose.pose
        odom_msg.pose.covariance = pose_msg.pose.covariance
        if self.rov_id == "":
            odom_msg.child_frame_id = "base_link"
        else:
            odom_msg.child_frame_id = self.rov_id + "_base_link"
        odom_msg.twist.twist = current_frame.twist
        # Twist covariance: left as default zeros — SLAM does not estimate velocity uncertainty.
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
            # Global frame for RViz (see publish pose note). Previously only set
            # when rov_id != "" — left empty otherwise — so it never matched the
            # world_ned Fixed Frame.
            link_msg.header.frame_id = "world_ned"
            self.constraint_pub.publish(link_msg)

    def publish_trajectory(self) -> None:
        """Publish 3D trajectory as point cloud in [x, y, z, roll, pitch, yaw, index] format.
        """

        # get all the poses from each keyframe
        poses = np.array([g2n(kf.pose3) for kf in self.fg.keyframes])

        # convert to a ros color line
        traj_msg = ros_colorline_trajectory(poses)
        traj_msg.header.stamp = self.fg.current_keyframe.time
        # Global frame for RViz (see publish pose note).
        traj_msg.header.frame_id = "world_ned"
        self.traj_pub.publish(traj_msg)

    def publish_point_cloud(self) -> None:
        """Publish downsampled 3D point cloud with z = 0.
        The last column represents keyframe index at which the point is observed.
        """
        # 1. Collect point clouds from all keyframes
        all_points = [np.zeros((0, 2), np.float32)]

        # List of keyframe ids
        all_keys = []

        # Semantic labels, collected in lock-step with the keys (on only).
        all_labels = []

        # 2. Transform each keyframe's points to global coordinate system
        for key in range(len(self.fg.keyframes)):

            # get the registered point cloud
            transf_points = self.fg.keyframes[key].transf_points
            if transf_points is None:
                # mapping-only keyframes are appended but never registered
                # (no factor-graph update), so they carry no global-frame
                # points — publishing crashed on len(None) at the first one.
                continue

            # append
            all_points.append(transf_points)
            all_keys.append(key * np.ones((len(transf_points), 1)))
            if self.semantic_enable:
                all_labels.append(
                    aligned_labels(self.fg.keyframes[key].labels, len(transf_points))
                    .reshape(-1, 1).astype(np.float64))

        if not all_keys:
            return

        # 3. Merge point clouds
        all_points = np.concatenate(all_points)
        all_keys = np.concatenate(all_keys)

        # 4. Downsample point cloud using PCL
        if self.localization is not None:
            point_resolution = self.localization.point_resolution
        else:
            point_resolution = 0.5  # Default resolution for mapping-only mode
        if self.semantic_enable:
            # The label rides through the same descriptor channel as the
            # keyframe index. downsample() picks a medoid, not a centroid, so
            # an integer label survives as that integer (probe 2026-09-02).
            sampled_points, sampled_desc = pcl.downsample(
                all_points, np.c_[all_keys, np.concatenate(all_labels)],
                point_resolution
            )
            sampled_cloud = np.c_[
                sampled_points, np.zeros((len(sampled_points), 1)), sampled_desc]
            cloud_type = "PointCloudXYZIL"
        else:
            sampled_points, sampled_keys = pcl.downsample(
                all_points, all_keys, point_resolution
            )
            sampled_cloud = np.c_[
                sampled_points, np.zeros_like(sampled_keys), sampled_keys]
            cloud_type = "PointCloudXYZI"

        # 5. Convert to ROS message and publish
        # if there are no points return and do nothing
        if len(sampled_cloud) == 0:
            return

        # convert the point cloud to a ros message and publish
        cloud_msg = n2r(sampled_cloud, cloud_type)
        cloud_msg.header.stamp = self.fg.current_keyframe.time
        # Global frame for RViz (see publish pose note).
        cloud_msg.header.frame_id = "world_ned"
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
            # I3 — 이 경로는 ICP 에 **도달조차 못 한** odometry factor 다.
            # `ssm.enable: false` 면 매 키프레임 여기로 빠지므로, 여기서 요약을
            # 내지 않으면 I1(ssm_disabled_count)이 정확히 그 상황에서 침묵한다 —
            # 계측의 출발점이 자기가 답해야 할 질문에서만 안 보이는 셈이 된다.
            self.instr['ssm_init_failed'] += 1
            self.fg.add_odometry_factor(keyframe)
            self._log_instrumentation()
            return

        # Create ICP result
        ret2 = ICPResult(ret, self.localization.ssm_params.cov_samples > 0)

        # If FFT succeeded, seed ICP with the FFT transform as the initial guess.
        # ICP then refines both rotation and translation starting from that seed
        # (previously the FFT transform replaced ICP entirely and skipped the
        # SSM validation below; refining + always validating proved more robust).
        fft_seeded = False
        seed_is_dr = False
        if self.fft_enable \
                and hasattr(keyframe, 'fft_success') and keyframe.fft_success \
                and keyframe.fft_transform is not None:
            ret2.initial_transform = keyframe.fft_transform
            if ret2.initial_transforms is not None and len(ret2.initial_transforms) > 0:
                # Replace every sampled initial guess with the FFT seed
                ret2.initial_transforms = [keyframe.fft_transform for _ in ret2.initial_transforms]
            fft_seeded = True
            # I4 — `fft_is_dr_fallback` 은 여태 write-only 였다. 여기서 읽는 이 한 줄이
            # 관찰 가능성 자체를 만든다: 시드가 FFT 에서 왔는지 DR fallback 에서 왔는지
            # 구분되지 않으면 아래 태그가 거짓을 말한다(둘 다 [FFT_SEED] 로 찍혔다).
            seed_is_dr = bool(getattr(keyframe, 'fft_is_dr_fallback', False))
            if seed_is_dr:
                self.instr['seed_dr_fallback'] += 1
            else:
                self.instr['seed_fft'] += 1

        # Always compute ICP (both rotation and translation come from ICP,
        # seeded with FFT when available)
        with CodeTimer("SLAM - sequential scan matching - ICP"):
            self.instr['icp_attempted'] += 1   # I2 — 비율 보고의 분모
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

        if ret2.status:
            self.instr['icp_converged'] += 1   # I2 — message == "success" 인 경우만 여기 온다

        if ret2.status and fft_seeded:
            # I5 — 태그를 시드 출처로 분기한다. 종전에는 DR fallback 에도 [FFT_SEED]
            # 가 붙어 로그가 거짓이었다.
            desc = ret2.status.description or ""
            tag = "[DR_SEED]" if seed_is_dr else "[FFT_SEED]"
            ret2.status.description = (desc + " " + tag).strip()

        # I11 — tilt 스케일 편향의 결정 계측. ICP 가 시드 대비 병진을 얼마나
        # 늘리거나 줄였는지의 단일 스칼라다. 중앙값이 1 에서 유의하게 벗어나면
        # 점군 척도가 어긋나 있다는 뜻이다. 방향까지 예단하지 않고 값만 남긴다 —
        # 예측치는 LOC-3 교정에 따라 압축(<1)일 수도 팽창(>1)일 수도 있다.
        if ret2.status:
            init_norm = float(np.linalg.norm(ret2.initial_transform.translation()))
            est_norm = float(np.linalg.norm(ret2.estimated_transform.translation()))
            if init_norm > 1e-6:
                # 시드 출처는 세 가지다. `fft_seeded` 가드 없이 seed_is_dr 만 보면
                # FFT 를 아예 안 쓴 경우(`fft_localization.enable: false`, FFT 실패)
                # 까지 'fft' 로 찍혀 I5 에서 고친 것과 같은 종류의 거짓이 된다.
                if not fft_seeded:
                    seed_src = 'none'      # 순수 DR — FFT 가 시드를 주지 않았다
                elif seed_is_dr:
                    seed_src = 'dr'        # FFT 가 DR fallback 을 시드로 넘겼다
                else:
                    seed_src = 'fft'
                self.get_logger().info(
                    f"[INSTR] scale init={init_norm:.4f} est={est_norm:.4f} "
                    f"ratio={est_norm / init_norm:.4f} seed={seed_src}"
                )

        # ICP validation — verify transform is reasonable
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
            self.instr['icp_factor_added'] += 1   # I3 — factor graph 의 실제 구성비
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
            self.instr['odom_factor_fallback'] += 1   # I3
            self.get_logger().warn(
                f"[SSM] Localization failed ({ret2.status.description}), using odometry instead. "
                f"DR delta: tx={ret2.initial_transform.x():.2f}m, ty={ret2.initial_transform.y():.2f}m, rot={np.degrees(ret2.initial_transform.theta()):.1f}deg"
            )
            self.fg.add_odometry_factor(keyframe)

        self._log_instrumentation()

    def _log_instrumentation(self) -> None:
        """계측 카운터 한 줄 요약 (I1~I6).

        키프레임마다 나가므로 `grep '\\[INSTR\\] counters'` 로 시계열을 그대로 뽑을
        수 있다. `ssm_disabled` 는 `Localization` 이 세므로 여기서 읽어 온다 — 이
        값이 키프레임 총수와 같으면 "icp 0%" 의 원인이 알고리즘이 아니라
        `ssm.enable: false` 라는 **설정**임이 확정된다(I1).
        """
        i = self.instr
        attempted = i['icp_attempted']
        rate = (i['icp_converged'] / attempted) if attempted else float('nan')
        self.get_logger().info(
            f"[INSTR] counters ssm_disabled={getattr(self.localization, 'ssm_disabled_count', -1)} "
            f"icp_attempted={attempted} icp_converged={i['icp_converged']} "
            f"icp_rate={rate:.3f} "
            f"factor_icp={i['icp_factor_added']} factor_odom={i['odom_factor_fallback']} "
            f"ssm_init_failed={i['ssm_init_failed']} "
            f"seed_fft={i['seed_fft']} seed_dr={i['seed_dr_fallback']} "
            f"reject_pos={i['reject_pos']} reject_rot={i['reject_rot']}"
        )
        if self.semantic_enable:
            # A separate line: the counters line above is the A/B baseline's
            # format and must not move when semantic labelling is switched on.
            si = self.semantic_instr
            self.get_logger().info(
                "[INSTR] semantic " + " ".join(f"{k}={v}" for k, v in si.items())
                + f" pending={len(self.pending_semantic)}"
            )

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
                    ret2.status.description = ""

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
