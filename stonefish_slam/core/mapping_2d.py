"""2D Sonar Image Mosaic Mapping Module.

ROS2-independent core class for creating 2D sonar image mosaics.
Converts polar sonar images to fan-shaped cartesian and accumulates them into a global map.

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
from typing import Optional, Tuple, List, Dict, Any, Union
import gtsam

# Type alias for duck typing: SLAM Keyframe objects
# Must have .pose (gtsam.Pose2) and .image (np.ndarray) attributes
KeyframeType = Any


class Mapping2D:
    """2D Sonar Image Mosaic Mapper.

    Creates a global 2D map by accumulating sonar images from multiple keyframes.
    Converts polar sonar images to fan-shaped cartesian images and transforms them
    to a global coordinate frame using pose estimates.

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
        global_map_accum (np.ndarray): Accumulated intensity values
        global_map_count (np.ndarray): Observation count per pixel
        global_map_max (np.ndarray): Maximum intensity per pixel

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
        map_size: Tuple[int, int] = (2000, 2000),
        sonar_range: float = 20.0,
        sonar_fov: float = 130.0,
        fan_pixel_resolution: float = 0.05
    ):
        """Initialize 2D mapping parameters.

        Args:
            map_resolution: Global map resolution in meters per pixel (default: 0.1)
            map_size: Maximum map size in pixels (width, height) (default: 2000x2000)
            sonar_range: Maximum sonar range in meters (default: 20.0)
            sonar_fov: Sonar field of view in degrees (default: 130.0)
            fan_pixel_resolution: Fan-shaped image resolution in m/pixel (default: 0.05)
        """
        self.map_resolution = map_resolution
        self.max_map_size = map_size
        self.sonar_range = sonar_range
        self.sonar_fov = sonar_fov
        self.fan_pixel_resolution = fan_pixel_resolution

        # Keyframe storage: List of {'key': int, 'pose': gtsam.Pose2, 'image': np.ndarray}
        self.keyframes: List[Dict] = []

        # Global map buffers (initialized when first keyframe is added)
        self.global_map_accum: Optional[np.ndarray] = None
        self.global_map_count: Optional[np.ndarray] = None
        self.global_map_max: Optional[np.ndarray] = None

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

        Args:
            key: Keyframe index
            pose: Robot pose as gtsam.Pose2 (x, y, theta in NED frame)
            sonar_image: Polar sonar image (num_bins × num_beams), uint8 or float
        """
        keyframe = {
            'key': key,
            'pose': pose,
            'image': sonar_image.astype(np.uint8) if sonar_image.dtype != np.uint8 else sonar_image
        }
        self.keyframes.append(keyframe)

    def polar_to_cartesian_image(
        self,
        polar_img: np.ndarray,
        max_range: Optional[float] = None,
        fov_deg: Optional[float] = None
    ) -> np.ndarray:
        """Convert polar sonar image to fan-shaped cartesian image.

        Reference: slam_2d.py Line 942-986

        Maps polar coordinates (range × bearing) to cartesian fan-shaped image.
        Uses inverse mapping (cartesian → polar lookup) for efficiency.

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

        img_h, img_w = polar_img.shape

        # Calculate output dimensions for fan-shaped image
        # slam_2d.py Line 955-958
        fov_rad = np.radians(fov_deg)
        output_width = int(2 * max_range * np.sin(fov_rad / 2) / self.fan_pixel_resolution)
        output_height = int(max_range / self.fan_pixel_resolution)

        # Create output image
        cartesian_img = np.zeros((output_height, output_width), dtype=np.uint8)

        # Create mapping from cartesian to polar
        # slam_2d.py Line 964-985
        # Sample every 2 pixels for speed
        for y in range(0, output_height, 2):
            for x in range(0, output_width, 2):
                # Convert output pixel to physical coordinates
                x_m = (x - output_width / 2) * self.fan_pixel_resolution
                y_m = (output_height - y) * self.fan_pixel_resolution  # y increases downward in image

                # Convert to polar coordinates
                r = np.sqrt(x_m**2 + y_m**2)
                theta = np.arctan2(x_m, y_m)  # angle from forward direction

                # Check if within sonar FOV
                if r > max_range or r < 0.1 or abs(theta) > fov_rad / 2:
                    continue

                # Map to input image coordinates
                # In polar image: row index corresponds to range
                row = int((r / max_range) * (img_h - 1))
                col = int((theta / (fov_rad / 2) * 0.5 + 0.5) * (img_w - 1))

                if 0 <= row < img_h and 0 <= col < img_w:
                    cartesian_img[y, x] = polar_img[row, col]

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
        buffer_m: float = 30.0
    ) -> None:
        """Process keyframes and accumulate into global map (core logic).

        This is a private method that contains the common logic for map generation.
        Used by both update_global_map() and update_global_map_from_slam().

        Args:
            keyframes: List of keyframes (dict or SLAM Keyframe objects)
            buffer_m: Buffer margin around trajectory in meters (default: 30.0)
        """
        if not keyframes:
            return

        # 1. Calculate global map bounds
        self.min_x, self.max_x, self.min_y, self.max_y = self.get_map_bounds(keyframes, buffer_m)

        # 2. Create global map
        self.map_width = int((self.max_x - self.min_x) / self.map_resolution)
        self.map_height = int((self.max_y - self.min_y) / self.map_resolution)

        # Limit map size to prevent memory issues
        max_w, max_h = self.max_map_size
        if self.map_width > max_w or self.map_height > max_h:
            self.map_width = min(self.map_width, max_w)
            self.map_height = min(self.map_height, max_h)

        # Initialize map with float for better blending
        self.global_map_accum = np.zeros((self.map_height, self.map_width), dtype=np.float64)
        self.global_map_count = np.zeros((self.map_height, self.map_width), dtype=np.float64)
        self.global_map_max = np.zeros((self.map_height, self.map_width), dtype=np.float32)

        # 3. Transform and add each keyframe's sonar image
        # Adaptive sampling based on number of keyframes
        sample_step = 4 if len(keyframes) > 50 else 2

        for kf in keyframes:
            # Extract image and pose (duck typing: dict or object)
            if isinstance(kf, dict):
                polar_img = kf['image']
                pose = kf['pose']
            else:
                # SLAM Keyframe object
                if not hasattr(kf, 'image') or kf.image is None:
                    continue
                polar_img = kf.image
                pose = kf.pose

            # Convert polar to fan-shaped cartesian image
            fan_img = self.polar_to_cartesian_image(polar_img, self.sonar_range, self.sonar_fov)
            fan_h, fan_w = fan_img.shape

            # Get pose parameters
            theta = pose.theta()
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)

            # Process fan-shaped image with sampling for performance
            for y in range(0, fan_h, sample_step):
                for x in range(0, fan_w, sample_step):
                    intensity = fan_img[y, x]
                    if intensity == 0:  # Skip empty pixels
                        continue

                    # Convert pixel to local coordinates (robot frame)
                    local_x = (fan_h - y) * self.fan_pixel_resolution
                    local_y = (x - fan_w / 2) * self.fan_pixel_resolution

                    # Transform to global frame (NED convention)
                    global_x = local_x * cos_theta - local_y * sin_theta + pose.x()
                    global_y = local_x * sin_theta + local_y * cos_theta + pose.y()

                    # Convert to map coordinates
                    map_x = int((global_x - self.min_x) / self.map_resolution)
                    map_y = int((global_y - self.min_y) / self.map_resolution)

                    # Add to map if within bounds
                    if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
                        self.global_map_accum[map_y, map_x] += float(intensity)
                        self.global_map_count[map_y, map_x] += 1
                        self.global_map_max[map_y, map_x] = max(
                            self.global_map_max[map_y, map_x], float(intensity)
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
        buffer_m: float = 30.0
    ) -> None:
        """Update global map directly from SLAM keyframes (SLAM integration mode).

        This method allows direct use of SLAM's Keyframe objects without duplication.
        The SLAM manages keyframe lifecycle, and this mapper only references them.

        Args:
            slam_keyframes: List of SLAM Keyframe objects
                           Each must have .pose (gtsam.Pose2) and .image (np.ndarray)
            buffer_m: Buffer margin around trajectory in meters (default: 30.0)

        Example:
            >>> from stonefish_slam.core.slam import SLAM
            >>> slam = SLAM(...)
            >>> mapper = Mapping2D(...)
            >>> # After SLAM processes data
            >>> mapper.update_global_map_from_slam(slam.keyframes)
            >>> map_img = mapper.get_map_image()
        """
        # Filter keyframes with valid images
        valid_keyframes = [
            kf for kf in slam_keyframes
            if hasattr(kf, 'image') and kf.image is not None
        ]

        if not valid_keyframes:
            return

        self._process_keyframes_to_map(valid_keyframes, buffer_m)

    def get_map_image(self, normalize: bool = True) -> np.ndarray:
        """Get current global map as uint8 image.

        Reference: slam_2d.py Line 1082-1111

        Blends average and max intensity (alpha=0.7 for average),
        normalizes using percentile clipping, and flips vertically
        for ROS coordinate convention.

        Args:
            normalize: Whether to normalize to 0-255 range (default: True)

        Returns:
            Global map image (height × width), uint8, vertically flipped
        """
        if self.global_map_accum is None:
            return np.zeros((100, 100), dtype=np.uint8)

        # 4. Process the accumulated values
        # slam_2d.py Line 1083-1092
        mask = self.global_map_count > 0

        # Calculate average intensity
        global_map_avg = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        global_map_avg[mask] = self.global_map_accum[mask] / self.global_map_count[mask]

        # Blend average and max for better visualization (preserve strong reflections)
        # slam_2d.py Line 1089-1092
        alpha = 0.7  # Weight for average (0.7 average, 0.3 max)
        global_map_blended = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        global_map_blended[mask] = alpha * global_map_avg[mask] + (1 - alpha) * self.global_map_max[mask]

        # Normalize to 0-255 range for better visualization
        # slam_2d.py Line 1094-1105
        if normalize and np.any(mask):
            # Use percentile for robust normalization (ignore outliers)
            low_percentile = np.percentile(global_map_blended[mask], 5)
            high_percentile = np.percentile(global_map_blended[mask], 95)

            if high_percentile > low_percentile:
                global_map_blended[mask] = np.clip(
                    (global_map_blended[mask] - low_percentile) / (high_percentile - low_percentile) * 255,
                    0, 255
                )

        global_map_uint8 = global_map_blended.astype(np.uint8)

        # Flip vertically for correct orientation in ROS/RViz
        # slam_2d.py Line 1111
        # Note: OpenCV images have origin at top-left, but ROS maps have origin at bottom-left
        global_map_flipped = np.flipud(global_map_uint8)

        return global_map_flipped

    def get_colored_map(self) -> np.ndarray:
        """Get colored visualization of the global map.

        Reference: slam_2d.py Line 1366-1367

        Returns:
            Colored map image (height × width × 3), uint8, BGR format
        """
        grayscale_map = self.get_map_image(normalize=True)
        colored_map = cv2.applyColorMap(grayscale_map, cv2.COLORMAP_JET)
        return colored_map

    def save_map(self, filepath: str, save_colored: bool = True) -> None:
        """Save global map to file.

        Reference: slam_2d.py Line 1362-1368

        Args:
            filepath: Path to save grayscale map (e.g., '/path/to/2d_map.png')
            save_colored: Whether to also save colored version (default: True)
        """
        grayscale_map = self.get_map_image(normalize=True)
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
        self.global_map_max = None
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
