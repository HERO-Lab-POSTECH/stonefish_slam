"""2D Sonar Image Mosaic Mapping Module.

ROS2-independent core class for creating 2D sonar image mosaics.
Converts polar sonar images to fan-shaped cartesian and accumulates them into a global map
using simple overlay (latest-write-wins).

Algorithm:
    1. Polar → Cartesian conversion with cv2.remap (cached transformation)
    2. NED coordinate transformation with sonar tilt correction
    3. Simple overlay: each new frame overwrites previous values at the same location
       - No preprocessing (raw sonar intensity)
       - No weighted averaging
       - Incremental update: only new keyframes processed
       - No full map rebuild: map persists across updates

Reference:
    slam_2d.py (krit_slam):
        - Line 942-986: polar_to_cartesian_image()
        - Line 988-1118: publish_2d_map()
        - Line 1240-1377: save_2d_map()
    mapping.py (krit_slam):
        - Line 11-40: Submap class structure
        - Line 145-252: add_keyframe()
"""

import numpy as np
import cv2
import logging
from typing import Optional, Tuple, List, Dict, Any, Union
import gtsam
from scipy.interpolate import interp1d
from rclpy.duration import Duration

# Type alias for duck typing: SLAM Keyframe objects
# Must have .pose (gtsam.Pose2) and .image (np.ndarray) attributes
KeyframeType = Any


class Mapping2D:
    """2D Sonar Image Mosaic Mapper (Simple Overlay).

    Creates a global 2D map by accumulating sonar images from multiple keyframes.
    Converts polar sonar images to fan-shaped cartesian images and transforms them
    to a global coordinate frame using pose estimates.

    **Implementation**:
    - Simple overlay: latest sonar data overwrites previous values
    - No image preprocessing (raw intensity)
    - Incremental update: processes only new keyframes (no full rebuild)
    - Map persistence: map data preserved across updates

    This class supports two operation modes:
    1. **Independent mode**: Store keyframes internally via add_keyframe() and use
       update_global_map() for map generation.
    2. **SLAM integration mode**: Reference SLAM's keyframes directly via
       update_global_map_from_slam() to avoid data duplication.

    Attributes:
        map_resolution (float): Map resolution in meters per pixel
        sonar_range (float): Maximum sonar range in meters
        sonar_fov (float): Sonar field of view in degrees
        fan_pixel_resolution (float): Resolution of fan-shaped image (m/pixel)
        keyframes (List[Dict]): List of keyframes with pose and sonar image (independent mode)
        global_map_accum (np.ndarray): Accumulated weighted intensity values
        global_map_weight (np.ndarray): Accumulated weight sum per pixel
        global_map_count (np.ndarray): Observation count per pixel (debugging)
        processed_keyframe_keys (set): Set of processed keyframe keys (incremental tracking)

    Example (SLAM integration):
        >>> from stonefish_slam.core.slam import SLAM
        >>> slam = SLAM(...)
        >>> mapper = Mapping2D(map_resolution=0.1, sonar_range=20.0)
        >>> # After SLAM processing
        >>> mapper.update_global_map_from_slam(slam.keyframes)
        >>> map_image = mapper.get_map_image()
        >>> mapper.save_map('/path/to/map.png')

    Example (Independent mode):
        >>> mapper = Mapping2D()
        >>> for pose, sonar_image in data:
        >>>     mapper.add_keyframe(key=i, pose=pose, sonar_image=sonar_image)
        >>> mapper.update_global_map()
        >>> map_image = mapper.get_map_image()
    """

    def __init__(
        self,
        map_resolution: float = 0.1,
        map_size: Tuple[int, int] = (4000, 4000),
        sonar_range: float = 20.0,
        sonar_fov: float = 130.0,
        fan_pixel_resolution: float = 0.05,
        sonar_tilt_deg: float = 30.0,
        range_min: float = 0.1,
        keyframe_sample_threshold: int = 50,
        intensity_threshold: int = 50,
        ros_logger=None
    ):
        """Initialize 2D mapping parameters.

        Args:
            map_resolution: Global map resolution in meters per pixel (default: 0.1)
            map_size: Maximum map size in pixels (width, height) (default: 2000x2000)
            sonar_range: Maximum sonar range in meters (default: 20.0)
            sonar_fov: Sonar field of view in degrees (default: 130.0)
            fan_pixel_resolution: Fan-shaped image resolution in m/pixel (default: 0.05)
            sonar_tilt_deg: Sonar tilt angle in degrees (0=vertical down, 30=tilted) (default: 30.0)
            range_min: Minimum sonar range in meters (default: 0.1)
            keyframe_sample_threshold: Threshold for adaptive sampling (default: 50)
            intensity_threshold: Minimum intensity for pixels to affect map (default: 50)
        """
        self.map_resolution = map_resolution
        self.max_map_size = map_size
        self.sonar_range = sonar_range
        self.sonar_fov = sonar_fov
        self.fan_pixel_resolution = fan_pixel_resolution
        self.sonar_tilt_rad = np.deg2rad(sonar_tilt_deg)
        self.range_min = range_min
        self.keyframe_sample_threshold = keyframe_sample_threshold
        self.intensity_threshold = intensity_threshold

        # Logger for debugging (supports both ROS and Python logging)
        self.ros_logger = ros_logger
        if ros_logger is not None:
            self.logger = ros_logger
        else:
            self.logger = logging.getLogger('Mapping2D')
            self.logger.setLevel(logging.INFO)

        # tf2 lookup statistics
        self.tf2_stats = {'success': 0, 'failed': 0, 'no_timestamp': 0}

        # Polar-to-cartesian transformation cache
        self.p2c_cache = None

        # Keyframe storage: List of {'key': int, 'pose': gtsam.Pose2, 'image': np.ndarray}
        self.keyframes: List[Dict] = []

        # SLAM keyframe reference (for SLAM integration mode)
        # Stores reference to all SLAM keyframes for bounds calculation
        self.all_slam_keyframes: List[KeyframeType] = []

        # Global map buffers (initialized when first keyframe is added)
        self.global_map_accum: Optional[np.ndarray] = None  # Latest-write-wins overlay map
        self.global_map_count: Optional[np.ndarray] = None  # Observation count (for debugging)

        # Track processed keyframes for incremental updates (Option 2)
        self.processed_keyframe_keys: set = set()

        # Map bounds (updated when keyframes are added)
        self.min_x: float = 0.0
        self.max_x: float = 0.0
        self.min_y: float = 0.0
        self.max_y: float = 0.0
        self.map_width: int = 0
        self.map_height: int = 0

    def add_keyframe(
        self,
        key: int,
        pose: gtsam.Pose2,
        sonar_image: np.ndarray
    ) -> None:
        """Add a keyframe with sonar image.

        No preprocessing applied - raw sonar intensity is stored.

        Args:
            key: Keyframe index
            pose: Robot pose as gtsam.Pose2 (x, y, theta in NED frame)
            sonar_image: Polar sonar image (num_bins × num_beams), uint8 or float
        """
        # Convert to uint8 if needed
        if sonar_image.dtype != np.uint8:
            sonar_image = np.clip(sonar_image, 0, 255).astype(np.uint8)

        keyframe = {
            'key': key,
            'pose': pose,
            'image': sonar_image
        }
        self.keyframes.append(keyframe)

    def polar_to_cartesian_image(
        self,
        polar_img: np.ndarray,
        max_range: Optional[float] = None,
        fov_deg: Optional[float] = None
    ) -> np.ndarray:
        """Convert polar sonar image to fan-shaped cartesian image.

        Reference: feature_extraction_sim.py Line 172-277 (cv2.remap-based implementation)

        Maps polar coordinates (range × bearing) to cartesian fan-shaped image.
        Uses cached transformation maps with cv2.remap() for high performance.

        Coordinate System:
            - Stonefish FLS convention: row=0 is FAR (range_max), row=max is NEAR (range_min)
            - X-axis: forward (range direction)
            - Y-axis: lateral/port-starboard (negative=left/port, positive=right/starboard)

        Args:
            polar_img: Input polar image (range × bearing), shape (num_bins, num_beams)
            max_range: Maximum sonar range in meters (default: self.sonar_range)
            fov_deg: Field of view in degrees (default: self.sonar_fov)

        Returns:
            Fan-shaped cartesian image (height × width), uint8
        """
        if max_range is None:
            max_range = self.sonar_range
        if fov_deg is None:
            fov_deg = self.sonar_fov

        rows, cols = polar_img.shape

        # Build or use cached transformation maps (performance optimization)
        if self.p2c_cache is None:
            # Calculate range resolution
            range_resolution = (max_range - self.range_min) / rows

            # Maximum lateral extent based on FOV
            max_lateral = max_range * np.sin(np.radians(fov_deg / 2.0))

            # Cartesian image dimensions (same resolution as range)
            cart_width = int(np.ceil(2 * max_lateral / range_resolution))
            cart_height = rows

            # Create bearing angle array for each column
            # col=0 → -FOV/2 (left), col=num_beams-1 → +FOV/2 (right)
            bearing_angles = np.radians(
                np.linspace(-fov_deg / 2.0, fov_deg / 2.0, cols)
            )

            # Create interpolation function: bearing → column index
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
            # CRITICAL: Stonefish FLS row convention is OPPOSITE of Oculus
            # Stonefish: row=0 (top) is FAR range, row=max (bottom) is NEAR range
            # Cartesian: YY=0 (top) should be FAR, YY=max (bottom) should be NEAR
            x_meters = max_range - range_resolution * YY

            # Y: lateral distance - centered at 0
            y_meters = range_resolution * (-cart_width / 2.0 + XX + 0.5)

            # Convert cartesian (x, y) to polar (r, bearing)
            r_polar = np.sqrt(np.square(x_meters) + np.square(y_meters))
            bearing_polar = np.arctan2(y_meters, x_meters)

            # Map polar coordinates to image indices
            # Range to row index (distance in meters to pixel row)
            # STONEFISH: row=0 is FAR (range_max), row=rows-1 is NEAR (range_min)
            map_y = np.asarray((max_range - r_polar) / range_resolution, dtype=np.float32)

            # Bearing to column index (using interpolation)
            map_x = np.asarray(f_bearings(bearing_polar), dtype=np.float32)

            # Cache the transformation maps
            self.p2c_cache = {
                'map_x': map_x,
                'map_y': map_y,
                'cart_height': cart_height,
                'cart_width': cart_width
            }

        # Use cached maps for fast remapping
        cartesian_img = cv2.remap(
            polar_img,
            self.p2c_cache['map_x'],
            self.p2c_cache['map_y'],
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0
        )

        # Flip horizontally to correct left-right orientation
        cartesian_img = np.fliplr(cartesian_img)

        return cartesian_img

    def get_map_bounds(
        self,
        keyframes: Optional[Union[List[Dict], List[KeyframeType]]] = None,
        buffer_m: float = 30.0
    ) -> Tuple[float, float, float, float]:
        """Calculate map bounds from keyframe poses.

        Args:
            keyframes: List of keyframes (dict or SLAM Keyframe objects).
                      If None, uses self.keyframes (default: None)
            buffer_m: Buffer margin in meters (default: 30.0)

        Returns:
            Tuple of (min_x, max_x, min_y, max_y) in meters
        """
        if keyframes is None:
            keyframes = self.keyframes

        if not keyframes:
            return 0.0, 0.0, 0.0, 0.0

        # Extract poses from either dict or object format
        xs = []
        ys = []
        for kf in keyframes:
            if isinstance(kf, dict):
                pose = kf['pose']
            else:
                pose = kf.pose
            xs.append(pose.x())
            ys.append(pose.y())

        min_x = min(xs) - buffer_m
        max_x = max(xs) + buffer_m
        min_y = min(ys) - buffer_m
        max_y = max(ys) + buffer_m

        return min_x, max_x, min_y, max_y

    def _process_keyframes_to_map(
        self,
        keyframes: Union[List[Dict], List[KeyframeType]],
        buffer_m: float = 30.0,
        tf2_buffer=None,
        target_frame: str = 'world_ned',
        source_frame: str = 'base_link'
    ) -> None:
        """Process keyframes and accumulate into global map (core logic).

        This is a private method that contains the common logic for map generation.
        Used by both update_global_map() and update_global_map_from_slam().

        **Simple Overlay Implementation**:
        - Only processes new keyframes (tracked via processed_keyframe_keys)
        - Map initialized once and preserved across updates
        - Latest-write-wins: each pixel shows most recent sonar observation
        - No preprocessing or weighting

        Coordinate Transformation Convention:
            - NED frame: X=North, Y=East, Z=Down, yaw=clockwise(+)
            - gtsam.Pose2: yaw is counterclockwise(+) → negated for NED
            - Image mapping: rows=X(North), cols=Y(East)
            - Y-axis negation applied for correct East direction

        Args:
            keyframes: List of keyframes (dict or SLAM Keyframe objects)
            buffer_m: Buffer margin around trajectory in meters (default: 30.0)
            tf2_buffer: tf2_ros.Buffer for timestamp-based pose lookup (optional)
            target_frame: Target frame for tf2 lookup (default: 'world_ned')
            source_frame: Source frame for tf2 lookup (default: 'base_link')
        """
        if not keyframes:
            return

        # Initialize map ONCE with fixed bounds centered at first keyframe
        if self.global_map_accum is None:
            # Get first keyframe pose to center the map
            first_kf = keyframes[0]
            if isinstance(first_kf, dict):
                first_pose = first_kf['pose']
            else:
                first_pose = first_kf.pose
            x0, y0 = first_pose.x(), first_pose.y()

            # Set initial bounds centered at first keyframe (50x50m, will expand dynamically)
            INITIAL_MAP_SIZE = 50.0
            self.min_x = x0 - INITIAL_MAP_SIZE/2
            self.max_x = x0 + INITIAL_MAP_SIZE/2
            self.min_y = y0 - INITIAL_MAP_SIZE/2
            self.max_y = y0 + INITIAL_MAP_SIZE/2

            # Calculate initial map dimensions
            new_map_width = int((self.max_y - self.min_y) / self.map_resolution)
            new_map_height = int((self.max_x - self.min_x) / self.map_resolution)

            # Initialize map (will expand dynamically as needed)
            self.global_map_accum = np.zeros((new_map_height, new_map_width), dtype=np.float64)
            self.global_map_count = np.zeros((new_map_height, new_map_width), dtype=np.float64)
            self.map_width = new_map_width
            self.map_height = new_map_height

            self.logger.info(
                f"[Dynamic Map] Initialized: {new_map_width}x{new_map_height} pixels ({INITIAL_MAP_SIZE}x{INITIAL_MAP_SIZE}m), "
                f"centered at first keyframe pose ({x0:.2f}, {y0:.2f}), "
                f"bounds X=[{self.min_x:.1f}, {self.max_x:.1f}], Y=[{self.min_y:.1f}, {self.max_y:.1f}]"
            )


        # 3. Transform and add each keyframe's sonar image
        # Adaptive sampling based on number of keyframes
        sample_step = 4 if len(keyframes) > self.keyframe_sample_threshold else 2

        # Track newly processed keyframes in this update
        newly_processed_count = 0

        for kf in keyframes:
            # Extract keyframe key for tracking (Option 2: incremental update)
            if isinstance(kf, dict):
                kf_key = kf['key']
            else:
                kf_key = kf.key if hasattr(kf, 'key') else None

            # Skip already processed keyframes (incremental update)
            if kf_key is not None and kf_key in self.processed_keyframe_keys:
                continue

            # Extract image and pose (duck typing: dict or object)
            if isinstance(kf, dict):
                polar_img = kf['image']
                pose = kf['pose']
                timestamp = kf.get('time', None)  # For dict mode
            else:
                # SLAM Keyframe object
                if not hasattr(kf, 'image') or kf.image is None:
                    continue
                polar_img = kf.image
                pose = kf.pose  # Default pose
                # Use sonar acquisition time if available, otherwise fall back to feature time
                timestamp = kf.sonar_time if hasattr(kf, 'sonar_time') and kf.sonar_time is not None else kf.time

            # No preprocessing - use raw sonar intensity

            # Debug log for keyframe processing
            if timestamp is not None:
                from rclpy.time import Time
                self.logger.debug(
                    f"Processing keyframe: pose=({pose.x():.2f}, {pose.y():.2f}), "
                    f"timestamp_type={type(timestamp).__name__}, "
                    f"tf2_available={tf2_buffer is not None}"
                )

            # Use tf2 to get accurate pose at sonar acquisition time
            if tf2_buffer is not None and timestamp is not None:
                try:
                    # Lookup transform at the exact sonar acquisition time
                    transform = tf2_buffer.lookup_transform(
                        target_frame,
                        source_frame,
                        timestamp,
                        timeout=Duration(seconds=0.1)
                    )

                    # Convert transform to gtsam.Pose2
                    from stonefish_slam.utils.conversions import r2g, pose322
                    from geometry_msgs.msg import Pose
                    ros_pose = Pose()
                    ros_pose.position.x = transform.transform.translation.x
                    ros_pose.position.y = transform.transform.translation.y
                    ros_pose.position.z = transform.transform.translation.z
                    ros_pose.orientation = transform.transform.rotation

                    pose3 = r2g(ros_pose)
                    pose = pose322(pose3)  # Extract 2D pose (x, y, yaw)

                    # Update statistics
                    self.tf2_stats['success'] += 1

                    if self.tf2_stats['success'] % 10 == 0:
                        self.logger.info(
                            f"tf2 stats: success={self.tf2_stats['success']}, "
                            f"failed={self.tf2_stats['failed']}"
                        )

                except Exception as e:
                    # Update statistics
                    self.tf2_stats['failed'] += 1

                    # Log tf2 lookup failures (use debug for common extrapolation errors)
                    error_msg = str(e)
                    if 'Lookup would require extrapolation' in error_msg:
                        # Common when mapping old keyframes, use debug level
                        self.logger.debug(
                            f"tf2 extrapolation needed (using keyframe pose): {error_msg[:80]}"
                        )
                    elif 'Could not find transform' in error_msg:
                        self.logger.warning(
                            f"Transform not available ({target_frame} → {source_frame}). "
                            f"Using keyframe pose."
                        )
                    else:
                        self.logger.warning(
                            f"tf2 lookup failed: {error_msg[:200]}. Using keyframe pose."
                        )
                    # Fallback to keyframe pose (pose already set above)
            else:
                if timestamp is None:
                    self.tf2_stats['no_timestamp'] += 1

            # ===== STEP 1: Convert polar sonar to fan-shaped cartesian =====
            fan_img = self.polar_to_cartesian_image(polar_img, self.sonar_range, self.sonar_fov)
            fan_h, fan_w = fan_img.shape

            # ===== STEP 2: Prepare rotation matrix (NED convention) =====
            # NED uses clockwise yaw (navigation), gtsam uses counterclockwise (math) → negate
            theta = -pose.theta()
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)

            # ===== STEP 3: Downsample and extract valid pixels (vectorized) =====
            fan_sampled = fan_img[::sample_step, ::sample_step]
            sampled_h, sampled_w = fan_sampled.shape

            # Create pixel coordinate grids
            yy, xx = np.meshgrid(
                np.arange(0, fan_h, sample_step),
                np.arange(0, fan_w, sample_step),
                indexing='ij'
            )

            # Filter low-intensity pixels (intensity threshold)
            mask = fan_sampled > self.intensity_threshold
            if not np.any(mask):
                continue

            # ===== Check if map expansion is needed (using only valid pixels) =====
            yy_valid = yy[mask]
            xx_valid = xx[mask]

            # Convert VALID pixels to local coordinates
            local_x_raw = (fan_h - yy_valid) * self.fan_pixel_resolution
            local_x = local_x_raw * np.cos(self.sonar_tilt_rad)
            local_y = (xx_valid - fan_w / 2.0) * self.fan_pixel_resolution

            # Transform to global coordinates
            theta = -pose.theta()
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            global_x_preview = local_x * cos_theta - local_y * sin_theta + pose.x()
            global_y_preview = -(local_x * sin_theta + local_y * cos_theta) + pose.y()

            # Get min/max extents of VALID sonar data only
            sonar_min_x = np.min(global_x_preview)
            sonar_max_x = np.max(global_x_preview)
            sonar_min_y = np.min(global_y_preview)
            sonar_max_y = np.max(global_y_preview)

            # Check if expansion is needed
            needs_expansion = (
                sonar_min_x < self.min_x or sonar_max_x > self.max_x or
                sonar_min_y < self.min_y or sonar_max_y > self.max_y
            )

            if needs_expansion:
                self._expand_map(sonar_min_x, sonar_max_x, sonar_min_y, sonar_max_y)

            intensities = fan_sampled[mask]

            # ===== STEP 4 & 5: Use already computed global coordinates =====
            # (Already computed in expansion check above)
            global_x = global_x_preview
            global_y = global_y_preview

            # Log tilt info once
            if not hasattr(self, '_tilt_logged'):
                self.logger.info(
                    f"Sonar tilt correction: {np.rad2deg(self.sonar_tilt_rad):.1f}° "
                    f"(range correction factor: {np.cos(self.sonar_tilt_rad):.3f})"
                )
                self._tilt_logged = True

            # ===== STEP 6: Convert to map pixel coordinates =====
            # NED→Image mapping: X(North)→rows, Y(East)→cols
            map_row = ((global_x - self.min_x) / self.map_resolution).astype(np.int32)
            map_col = ((global_y - self.min_y) / self.map_resolution).astype(np.int32)

            # Boundary check (vectorized)
            valid_idx = (
                (map_row >= 0) & (map_row < self.map_height) &
                (map_col >= 0) & (map_col < self.map_width)
            )

            # Extract valid points
            map_row_valid = map_row[valid_idx]
            map_col_valid = map_col[valid_idx]
            intensities_valid = intensities[valid_idx].astype(np.float32)

            # Debug: log mapping statistics
            total_sampled_pixels = sampled_h * sampled_w
            total_pixels_after_intensity_filter = len(intensities)
            valid_pixels = len(intensities_valid)
            filtered_by_intensity = total_sampled_pixels - total_pixels_after_intensity_filter

            if total_pixels_after_intensity_filter > 0:
                if valid_pixels > 0:
                    self.logger.info(
                        f"Keyframe mapping: sampled={total_sampled_pixels}, "
                        f"filtered_by_intensity={filtered_by_intensity} (threshold>{self.intensity_threshold}), "
                        f"after_filter={total_pixels_after_intensity_filter}, "
                        f"valid_in_bounds={valid_pixels} ({100*valid_pixels/total_pixels_after_intensity_filter:.1f}%), "
                        f"intensity_range=[{intensities_valid.min():.1f}, {intensities_valid.max():.1f}]"
                    )
                else:
                    self.logger.warning(
                        f"Keyframe mapping: sampled={total_sampled_pixels}, "
                        f"filtered_by_intensity={filtered_by_intensity} (threshold>{self.intensity_threshold}), "
                        f"after_filter={total_pixels_after_intensity_filter}, "
                        f"valid_in_bounds=0 (0.0%) - ALL PIXELS OUT OF BOUNDS! "
                        f"map_bounds=({self.min_x:.1f}, {self.max_x:.1f}, {self.min_y:.1f}, {self.max_y:.1f}), "
                        f"pose=({pose.x():.1f}, {pose.y():.1f})"
                    )

            # Latest image overlay - just overwrite with newest sonar data
            if len(intensities_valid) > 0:
                linear_indices = map_row_valid * self.map_width + map_col_valid
                map_flat = self.global_map_accum.ravel()
                count_flat = self.global_map_count.ravel()

                # Simple overwrite (latest-write-wins)
                map_flat[linear_indices] = intensities_valid
                np.add.at(count_flat, linear_indices, 1)  # Track observation count

            # Mark this keyframe as processed (incremental update)
            if kf_key is not None:
                self.processed_keyframe_keys.add(kf_key)
                newly_processed_count += 1

        # Log incremental update statistics
        if newly_processed_count > 0:
            self.logger.info(
                f"Incremental map update: processed {newly_processed_count} new keyframes "
                f"(total processed: {len(self.processed_keyframe_keys)})"
            )

    def update_global_map(self, buffer_m: float = 30.0) -> None:
        """Update global map from self.keyframes (independent operation mode).

        Reference: slam_2d.py Line 1000-1118

        This method uses the internally stored keyframes (added via add_keyframe()).
        For SLAM integration, use update_global_map_from_slam() instead.

        Args:
            buffer_m: Buffer margin around trajectory in meters (default: 30.0)
        """
        self._process_keyframes_to_map(self.keyframes, buffer_m)

    def update_global_map_from_slam(
        self,
        slam_keyframes: List[KeyframeType],
        buffer_m: float = 30.0,
        tf2_buffer=None,
        target_frame: str = 'world_ned',
        source_frame: str = 'base_link',
        all_slam_keyframes: Optional[List[KeyframeType]] = None
    ) -> None:
        """Update global map directly from SLAM keyframes (SLAM integration mode).

        This method allows direct use of SLAM's Keyframe objects without duplication.
        The SLAM manages keyframe lifecycle, and this mapper only references them.

        Args:
            slam_keyframes: List of new SLAM Keyframe objects to process
                           Each must have .pose (gtsam.Pose2) and .image (np.ndarray)
            buffer_m: Buffer margin around trajectory in meters (default: 30.0)
            tf2_buffer: tf2_ros.Buffer for timestamp-based pose lookup (optional)
            target_frame: Target frame for tf2 lookup (default: 'world_ned')
            source_frame: Source frame for tf2 lookup (default: 'base_link')
            all_slam_keyframes: Complete list of all SLAM keyframes (for bounds calculation)
                               If None, uses slam_keyframes (default: None)

        Example:
            >>> from stonefish_slam.core.slam import SLAM
            >>> slam = SLAM(...)
            >>> mapper = Mapping2D(...)
            >>> # After SLAM processes data
            >>> mapper.update_global_map_from_slam(
            >>>     new_keyframes,
            >>>     all_slam_keyframes=slam.keyframes,
            >>>     tf2_buffer=tf_buffer
            >>> )
            >>> map_img = mapper.get_map_image()
        """
        # Update all keyframes reference (for bounds calculation)
        if all_slam_keyframes is not None:
            self.all_slam_keyframes = all_slam_keyframes
        else:
            # Backward compatibility: use slam_keyframes if all not provided
            self.all_slam_keyframes = slam_keyframes

        # Filter new keyframes with valid images (for processing)
        valid_keyframes = [
            kf for kf in slam_keyframes
            if hasattr(kf, 'image') and kf.image is not None
        ]

        if not valid_keyframes:
            return

        self._process_keyframes_to_map(valid_keyframes, buffer_m, tf2_buffer, target_frame, source_frame)

    def get_map_image(self) -> np.ndarray:
        """Get current global map as uint8 image.

        Latest image overlay: Each pixel shows the most recent sonar observation.
        Simple overwrite - no normalization, no maximum, just latest data.

        Returns:
            Global map image (height × width), uint8, vertically flipped
        """
        if self.global_map_accum is None:
            return np.zeros((100, 100), dtype=np.uint8)

        # Simple direct conversion (latest-write-wins, no processing)
        global_map_uint8 = np.clip(self.global_map_accum, 0, 255).astype(np.uint8)

        # Flip vertically for correct orientation in ROS/RViz
        # Note: OpenCV images have origin at top-left, but ROS maps have origin at bottom-left
        global_map_flipped = np.flipud(global_map_uint8)

        return global_map_flipped

    def _expand_map(self, sonar_min_x: float, sonar_max_x: float,
                    sonar_min_y: float, sonar_max_y: float) -> None:
        """Expand map using numpy padding to accommodate new sonar data.

        This method expands the map boundaries when new sonar data extends beyond
        current map limits. Uses grid alignment to ensure bounds are always exact
        multiples of map_resolution, eliminating cumulative floating-point errors.

        Args:
            sonar_min_x: Minimum global X coordinate of sonar image
            sonar_max_x: Maximum global X coordinate of sonar image
            sonar_min_y: Minimum global Y coordinate of sonar image
            sonar_max_y: Maximum global Y coordinate of sonar image
        """
        # Calculate target bounds with 5m buffer
        target_min_x = sonar_min_x - 5.0
        target_max_x = sonar_max_x + 5.0
        target_min_y = sonar_min_y - 5.0
        target_max_y = sonar_max_y + 5.0

        # Snap to resolution grid (eliminates cumulative error)
        # floor for min, ceil for max ensures coverage
        new_min_x = np.floor(target_min_x / self.map_resolution) * self.map_resolution
        new_max_x = np.ceil(target_max_x / self.map_resolution) * self.map_resolution
        new_min_y = np.floor(target_min_y / self.map_resolution) * self.map_resolution
        new_max_y = np.ceil(target_max_y / self.map_resolution) * self.map_resolution

        # Calculate required padding in meters (always exact multiples of resolution)
        pad_north_m = max(0, self.min_x - new_min_x)
        pad_south_m = max(0, new_max_x - self.max_x)
        pad_west_m = max(0, self.min_y - new_min_y)
        pad_east_m = max(0, new_max_y - self.max_y)

        # Convert to pixels (exact division, no rounding needed)
        pad_top_px = int(pad_north_m / self.map_resolution)
        pad_bottom_px = int(pad_south_m / self.map_resolution)
        pad_left_px = int(pad_west_m / self.map_resolution)
        pad_right_px = int(pad_east_m / self.map_resolution)

        # If no expansion needed, return early
        if pad_top_px == 0 and pad_bottom_px == 0 and pad_left_px == 0 and pad_right_px == 0:
            return

        # Store old dimensions for logging
        old_height, old_width = self.map_height, self.map_width
        old_bounds = (self.min_x, self.max_x, self.min_y, self.max_y)

        # Expand both accumulator and count maps
        self.global_map_accum = np.pad(
            self.global_map_accum,
            ((pad_top_px, pad_bottom_px), (pad_left_px, pad_right_px)),
            mode='constant', constant_values=0
        )
        self.global_map_count = np.pad(
            self.global_map_count,
            ((pad_top_px, pad_bottom_px), (pad_left_px, pad_right_px)),
            mode='constant', constant_values=0
        )

        # Update bounds (CRITICAL for world-to-pixel consistency)
        # With grid alignment, these are exact
        self.min_x -= pad_top_px * self.map_resolution
        self.max_x += pad_bottom_px * self.map_resolution
        self.min_y -= pad_left_px * self.map_resolution
        self.max_y += pad_right_px * self.map_resolution

        # Update map dimensions
        self.map_height, self.map_width = self.global_map_accum.shape

        # Log expansion with bounds info
        if self.logger:
            self.logger.info(
                f"[Mapping] Map expanded: {old_height}x{old_width} -> {self.map_height}x{self.map_width} "
                f"(padding: top={pad_top_px}, bottom={pad_bottom_px}, left={pad_left_px}, right={pad_right_px})"
            )
            self.logger.info(
                f"[Mapping] Bounds updated: X=[{old_bounds[0]:.2f}, {old_bounds[1]:.2f}] -> [{self.min_x:.2f}, {self.max_x:.2f}], "
                f"Y=[{old_bounds[2]:.2f}, {old_bounds[3]:.2f}] -> [{self.min_y:.2f}, {self.max_y:.2f}]"
            )

    def get_colored_map(self) -> np.ndarray:
        """Get colored visualization of the global map.

        Reference: slam_2d.py Line 1366-1367

        Returns:
            Colored map image (height × width × 3), uint8, BGR format
        """
        grayscale_map = self.get_map_image()
        colored_map = cv2.applyColorMap(grayscale_map, cv2.COLORMAP_JET)
        return colored_map

    def save_map(self, filepath: str, save_colored: bool = True) -> None:
        """Save global map to file.

        Reference: slam_2d.py Line 1362-1368

        Args:
            filepath: Path to save grayscale map (e.g., '/path/to/2d_map.png')
            save_colored: Whether to also save colored version (default: True)
        """
        grayscale_map = self.get_map_image()
        cv2.imwrite(filepath, grayscale_map)

        if save_colored:
            # Save colored version
            colored_map = self.get_colored_map()
            colored_filepath = filepath.replace('.png', '_colored.png')
            cv2.imwrite(colored_filepath, colored_map)

    def clear(self) -> None:
        """Clear all keyframes and reset the map."""
        self.keyframes.clear()
        self.global_map_accum = None
        self.global_map_count = None
        self.processed_keyframe_keys.clear()  # Reset processed keyframe tracking
        self.min_x = 0.0
        self.max_x = 0.0
        self.min_y = 0.0
        self.max_y = 0.0
        self.map_width = 0
        self.map_height = 0

    def get_map_info(self) -> Dict:
        """Get map metadata.

        Returns:
            Dictionary containing map parameters and statistics
        """
        valid_pixels = 0
        if self.global_map_count is not None:
            valid_pixels = int(np.sum(self.global_map_count > 0))

        return {
            'num_keyframes': len(self.keyframes),
            'map_width': self.map_width,
            'map_height': self.map_height,
            'map_resolution': self.map_resolution,
            'valid_pixels': valid_pixels,
            'bounds': {
                'min_x': self.min_x,
                'max_x': self.max_x,
                'min_y': self.min_y,
                'max_y': self.max_y
            }
        }
