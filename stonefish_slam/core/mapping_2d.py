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
        keyframe_sample_threshold: int = 50
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
        """
        self.map_resolution = map_resolution
        self.max_map_size = map_size
        self.sonar_range = sonar_range
        self.sonar_fov = sonar_fov
        self.fan_pixel_resolution = fan_pixel_resolution
        self.sonar_tilt_rad = np.deg2rad(sonar_tilt_deg)
        self.range_min = range_min
        self.keyframe_sample_threshold = keyframe_sample_threshold

        # Logger for debugging
        self.logger = logging.getLogger('Mapping2D')
        self.logger.setLevel(logging.INFO)

        # tf2 lookup statistics
        self.tf2_stats = {'success': 0, 'failed': 0, 'no_timestamp': 0}

        # Polar-to-cartesian transformation cache
        self.p2c_cache = None

        # Keyframe storage: List of {'key': int, 'pose': gtsam.Pose2, 'image': np.ndarray}
        self.keyframes: List[Dict] = []

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

        # 1. Initialize bounds once (no expansion - use very large fixed bounds)
        if self.global_map_accum is None:
            # Calculate initial center from first keyframes
            init_bounds = self.get_map_bounds(keyframes, buffer_m)
            center_x = (init_bounds[0] + init_bounds[1]) / 2.0
            center_y = (init_bounds[2] + init_bounds[3]) / 2.0

            # Create very large fixed map (1km × 1km centered on initial position)
            map_half_size = 500.0
            self.min_x = center_x - map_half_size
            self.max_x = center_x + map_half_size
            self.min_y = center_y - map_half_size
            self.max_y = center_y + map_half_size

        # 2. Calculate map dimensions
        new_map_width = int((self.max_y - self.min_y) / self.map_resolution)
        new_map_height = int((self.max_x - self.min_x) / self.map_resolution)

        # 3. Resize map if dimensions changed (fast padding)
        if self.global_map_accum is None:
            # First initialization
            self.global_map_accum = np.zeros((new_map_height, new_map_width), dtype=np.float64)
            self.global_map_count = np.zeros((new_map_height, new_map_width), dtype=np.float64)
            self.map_width = new_map_width
            self.map_height = new_map_height
            self.logger.info(f"Map initialized: ({new_map_height}, {new_map_width})")
        elif self.global_map_accum.shape != (new_map_height, new_map_width):
            # Expand map with zero-padding (faster than creating new array)
            pad_height = new_map_height - self.map_height
            pad_width = new_map_width - self.map_width

            if pad_height > 0 or pad_width > 0:
                pad_h = max(0, pad_height)
                pad_w = max(0, pad_width)
                self.global_map_accum = np.pad(self.global_map_accum, ((0, pad_h), (0, pad_w)), mode='constant')
                self.global_map_count = np.pad(self.global_map_count, ((0, pad_h), (0, pad_w)), mode='constant')
                self.map_width = new_map_width
                self.map_height = new_map_height
                self.logger.info(f"Map auto-expanded to ({new_map_height}, {new_map_width})")


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

            # Filter zero-intensity pixels
            mask = fan_sampled > 0
            if not np.any(mask):
                continue

            yy_valid = yy[mask]
            xx_valid = xx[mask]
            intensities = fan_sampled[mask]

            # ===== STEP 4: Convert pixels to local coordinates (body frame) =====
            # Stonefish FLS: row=0 is FAR, row=max is NEAR
            local_x_raw = (fan_h - yy_valid) * self.fan_pixel_resolution

            # Apply sonar tilt correction (downward tilt projects range to horizontal plane)
            local_z = local_x_raw * np.sin(self.sonar_tilt_rad)  # Depth (ignored in 2D)
            local_x = local_x_raw * np.cos(self.sonar_tilt_rad)  # Horizontal range
            local_y = (xx_valid - fan_w / 2.0) * self.fan_pixel_resolution

            # Log tilt info once
            if not hasattr(self, '_tilt_logged'):
                self.logger.info(
                    f"Sonar tilt correction: {np.rad2deg(self.sonar_tilt_rad):.1f}° "
                    f"(range correction factor: {np.cos(self.sonar_tilt_rad):.3f})"
                )
                self._tilt_logged = True

            # ===== STEP 5: Transform to global NED frame =====
            # Standard 2D rotation + translation
            global_x = local_x * cos_theta - local_y * sin_theta + pose.x()
            # Y-axis negation required for correct NED East direction mapping
            global_y = -(local_x * sin_theta + local_y * cos_theta) + pose.y()

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
            total_pixels = len(intensities)
            valid_pixels = len(intensities_valid)
            if total_pixels > 0 and valid_pixels > 0:
                self.logger.info(
                    f"Keyframe mapping: total={total_pixels}, valid={valid_pixels} "
                    f"({100*valid_pixels/total_pixels:.1f}%), "
                    f"intensity=[{intensities_valid.min():.1f}, {intensities_valid.max():.1f}]",
                    throttle_duration_sec=2.0
                )

            # Latest image overlay - just overwrite with newest sonar data
            if len(intensities_valid) > 0:
                linear_indices = map_row_valid * self.map_width + map_col_valid
                map_flat = self.global_map_accum.ravel()
                count_flat = self.global_map_count.ravel()

                # Simple overwrite (latest-write-wins)
                map_flat[linear_indices] = intensities_valid
                np.add.at(count_flat, linear_indices, 1)  # Track observation count
            else:
                self.logger.warning(
                    f"No valid pixels in keyframe: map_bounds=({self.min_x:.1f}, {self.max_x:.1f}, {self.min_y:.1f}, {self.max_y:.1f}), "
                    f"pose=({pose.x():.1f}, {pose.y():.1f})"
                )

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
        source_frame: str = 'base_link'
    ) -> None:
        """Update global map directly from SLAM keyframes (SLAM integration mode).

        This method allows direct use of SLAM's Keyframe objects without duplication.
        The SLAM manages keyframe lifecycle, and this mapper only references them.

        Args:
            slam_keyframes: List of SLAM Keyframe objects
                           Each must have .pose (gtsam.Pose2) and .image (np.ndarray)
            buffer_m: Buffer margin around trajectory in meters (default: 30.0)
            tf2_buffer: tf2_ros.Buffer for timestamp-based pose lookup (optional)
            target_frame: Target frame for tf2 lookup (default: 'world_ned')
            source_frame: Source frame for tf2 lookup (default: 'base_link')

        Example:
            >>> from stonefish_slam.core.slam import SLAM
            >>> slam = SLAM(...)
            >>> mapper = Mapping2D(...)
            >>> # After SLAM processes data
            >>> mapper.update_global_map_from_slam(slam.keyframes, tf2_buffer=tf_buffer)
            >>> map_img = mapper.get_map_image()
        """
        # Filter keyframes with valid images
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
