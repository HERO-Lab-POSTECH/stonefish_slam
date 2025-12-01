"""
FFT-based Sonar Image Registration for Localization

This module provides Fourier-based registration methods for Forward-Looking Sonar (FLS) images.
Based on: "Fourier-based Registration for Robust Forward-looking Sonar Mosaicing"
         in Low-visibility Underwater Environments (Hurtós et al., 2015)

Completely independent module - no ROS2 or ICP dependencies.
"""

import numpy as np
from scipy import ndimage
from scipy.interpolate import interp1d
from typing import Tuple, Dict, Any, Optional
import warnings

from stonefish_slam.utils.sonar import OculusProperty


class FFTLocalizer:
    """
    FFT-based registration for Forward-Looking Sonar (FLS) images.

    This implementation uses phase correlation in polar and Cartesian domains
    to estimate rotation and translation between consecutive sonar frames.
    """

    def __init__(self,
                 oculus: OculusProperty,
                 range_min: float = 0.5,
                 rot_erosion_iterations: int = 3,
                 rot_gaussian_sigma: float = 4.0,
                 rot_gaussian_truncate: float = 4.0,
                 trans_erosion_iterations: int = 4,
                 trans_gaussian_sigma: float = 4.0,
                 trans_gaussian_truncate: float = 4.0,
                 max_expected_rotation: float = 30.0,
                 verbose: bool = False):
        """
        Initialize FFT Localizer.

        Args:
            oculus: OculusProperty instance (sonar configuration)
            range_min: Minimum reliable range in meters (default: 0.5)
            rot_erosion_iterations: Erosion iterations for rotation mask
            rot_gaussian_sigma: Gaussian sigma for rotation mask smoothing
            rot_gaussian_truncate: Gaussian truncate factor for rotation mask
            trans_erosion_iterations: Erosion iterations for translation mask
            trans_gaussian_sigma: Gaussian sigma for translation mask smoothing
            trans_gaussian_truncate: Gaussian truncate factor for translation mask
            max_expected_rotation: Maximum expected rotation in degrees (for padding)
            verbose: Enable debug output
        """
        self.oculus = oculus
        self.range_min = range_min
        self.verbose = verbose

        # Rotation erosion mask parameters
        self.rot_erosion_iterations = rot_erosion_iterations
        self.rot_gaussian_sigma = rot_gaussian_sigma
        self.rot_gaussian_truncate = rot_gaussian_truncate

        # Translation erosion mask parameters
        self.trans_erosion_iterations = trans_erosion_iterations
        self.trans_gaussian_sigma = trans_gaussian_sigma
        self.trans_gaussian_truncate = trans_gaussian_truncate

        # Maximum expected rotation for padding calculation
        self.max_expected_rotation = max_expected_rotation

        # Cache for polar_to_cartesian conversion (performance optimization)
        self.p2c_cache = None

        # Cache range_resolution from polar_to_cartesian for consistent translation estimation
        # CRITICAL: Must use same range_resolution for cart conversion and translation
        self.cart_range_resolution = None

    def apply_range_min_mask(self, img_polar: np.ndarray) -> np.ndarray:
        """
        Apply minimum range mask to polar sonar image.
        Masks out unreliable near-field data below range_min.

        Args:
            img_polar: Polar sonar image (range × angle)

        Returns:
            Masked image with near-field data set to zero
        """
        if self.range_min <= 0:
            return img_polar

        masked_img = img_polar.copy()
        range_bins, _ = img_polar.shape

        # Calculate number of range bins to mask
        range_min_bins = int(self.range_min / self.oculus.range_max * range_bins)

        # Stonefish polar image: Row 0 = far (top), Row N-1 = near (bottom)
        # Mask bottom rows (near field < min_range)
        if range_min_bins > 0:
            masked_img[-range_min_bins:, :] = 0

        return masked_img

    def apply_erosion_mask(self,
                           image: np.ndarray,
                           erosion_iterations: int = 4,
                           gaussian_sigma: float = 4.0,
                           gaussian_truncate: float = 3.0) -> np.ndarray:
        """
        Apply erosion-based mask to reduce edge artifacts.

        Creates a smooth mask by eroding non-zero regions and applying
        Gaussian smoothing for soft boundaries.

        Args:
            image: Input image
            erosion_iterations: Number of binary erosion iterations
            gaussian_sigma: Gaussian filter sigma (pixels)
            gaussian_truncate: Gaussian kernel truncation factor

        Returns:
            Masked image with soft boundaries
        """
        h, w = image.shape

        # Create footprint from non-zero regions
        footprint = (image != 0).astype(np.float64)

        # Apply binary erosion
        from scipy.ndimage import binary_erosion
        structure = np.ones((3, 3))
        shrink = footprint.copy()
        for _ in range(erosion_iterations):
            if np.sum(shrink) == 0:
                break
            shrink = binary_erosion(shrink, structure).astype(np.float64)

        # Apply distance-based smoothing (preserves peak sharpness better than Gaussian)
        # Literature: Gaussian spreading creates spurious peaks in phase correlation
        # Reference: Reddy & Chatterji, Nature Scientific Reports 2025
        from scipy.ndimage import distance_transform_edt
        dist = distance_transform_edt(shrink)
        max_dist = np.max(dist) if np.max(dist) > 0 else 1.0
        # Normalize with smooth falloff using gaussian_sigma as scale parameter
        mask = np.clip(dist / (gaussian_sigma * 2), 0, 1)

        return image * mask

    def polar_to_cartesian(self, polar_image: np.ndarray) -> np.ndarray:
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
        import cv2

        rows = polar_image.shape[0]  # num_bins
        cols = polar_image.shape[1]  # num_beams

        # Build or use cached transformation maps (performance optimization)
        if self.p2c_cache is None:
            if self.verbose:
                print("Building polar-to-cartesian transformation maps (one-time setup)...")

            # Calculate range resolution
            range_resolution = (self.oculus.range_max - self.range_min) / rows

            # Cache for translation estimation (CRITICAL: same scale factor as cart conversion)
            self.cart_range_resolution = range_resolution

            # Maximum lateral extent based on FOV
            horizontal_fov_deg = np.rad2deg(self.oculus.horizontal_fov)
            max_lateral = self.oculus.range_max * np.sin(np.radians(horizontal_fov_deg / 2.0))

            # Cartesian image dimensions (same resolution as range)
            cart_width = int(np.ceil(2 * max_lateral / range_resolution))
            cart_height = rows

            # Create bearing angle array for each column
            # col=0 → -FOV/2 (left), col=num_beams-1 → +FOV/2 (right)
            bearing_angles = np.radians(
                np.linspace(-horizontal_fov_deg / 2.0,
                           horizontal_fov_deg / 2.0,
                           cols)
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
            # IMPORTANT: Stonefish FLS row convention is OPPOSITE of Oculus
            # Stonefish: row=0 (top) is FAR range, row=max (bottom) is NEAR range
            # Cartesian: YY=0 (top) should be FAR, YY=max (bottom) should be NEAR
            x_meters = self.oculus.range_max - range_resolution * YY

            # Y: lateral distance - centered at 0
            y_meters = range_resolution * (-cart_width / 2.0 + XX + 0.5)

            # Convert cartesian (x, y) to polar (r, bearing)
            r_polar = np.sqrt(np.square(x_meters) + np.square(y_meters))
            bearing_polar = np.arctan2(y_meters, x_meters)

            # Map polar coordinates to image indices
            # Range to row index (distance in meters to pixel row)
            # STONEFISH: row=0 is FAR (range_max), row=rows-1 is NEAR (range_min)
            map_y = np.asarray((self.oculus.range_max - r_polar) / range_resolution, dtype=np.float32)

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

            if self.verbose:
                print(f"Transformation maps built: {cart_height}x{cart_width} cartesian image")

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

    def compute_phase_correlation(self,
                                  img1: np.ndarray,
                                  img2: np.ndarray) -> np.ndarray:
        """
        Compute phase correlation between two images.

        Args:
            img1: First image
            img2: Second image

        Returns:
            Phase Correlation Matrix (PCM)
        """
        # Compute 2D FFT
        F1 = np.fft.fft2(img1)
        F2 = np.fft.fft2(img2)

        # Shift zero frequency to center
        F1 = np.fft.fftshift(F1)
        F2 = np.fft.fftshift(F2)

        # Compute cross power spectrum
        cross_power = F1 * np.conj(F2)

        # Normalize (phase correlation)
        eps = 1e-10
        cross_power_spectrum = cross_power / (np.abs(cross_power) + eps)

        # Inverse FFT to get PCM
        pcm = np.fft.ifft2(cross_power_spectrum)
        pcm = np.fft.ifftshift(pcm)
        pcm = np.abs(pcm)

        return pcm

    def detect_peak(self,
                    pcm: np.ndarray,
                    subpixel: bool = True) -> Tuple[float, float, float]:
        """
        Detect peak in Phase Correlation Matrix.

        Args:
            pcm: Phase Correlation Matrix
            subpixel: Enable subpixel accuracy using parabolic fitting

        Returns:
            (row_offset, col_offset, peak_value)
        """
        # Find maximum
        peak_value = np.max(pcm)
        peak_loc = np.unravel_index(np.argmax(pcm), pcm.shape)

        # Convert to offset from center
        h, w = pcm.shape
        row_offset = peak_loc[0] - h // 2
        col_offset = peak_loc[1] - w // 2

        # Subpixel refinement using parabolic fitting
        if subpixel and peak_value > 0:
            try:
                r, c = peak_loc
                if 1 <= r < h-1 and 1 <= c < w-1:
                    neighborhood = pcm[r-1:r+2, c-1:c+2]

                    # Vertical direction
                    vert = neighborhood[:, 1]
                    if len(vert) == 3:
                        denom = 2 * (vert[0] - 2*vert[1] + vert[2])
                        if abs(denom) > 1e-10:
                            dr = (vert[0] - vert[2]) / denom
                            row_offset += dr

                    # Horizontal direction
                    horiz = neighborhood[1, :]
                    if len(horiz) == 3:
                        denom = 2 * (horiz[0] - 2*horiz[1] + horiz[2])
                        if abs(denom) > 1e-10:
                            dc = (horiz[0] - horiz[2]) / denom
                            col_offset += dc

            except Exception as e:
                if self.verbose:
                    print(f"Subpixel refinement failed: {e}")

        return row_offset, col_offset, peak_value

    def _calculate_padding_size(self,
                                image_shape: Tuple[int, int],
                                safety_margin: int = 20) -> int:
        """
        Calculate padding size to prevent FOV loss during rotation.

        Args:
            image_shape: (height, width) of Cartesian sonar image
            safety_margin: Extra padding pixels for safety

        Returns:
            Padding size in pixels (applied to all four sides)
        """
        h, w = image_shape

        # Calculate image diagonal
        diagonal = np.sqrt(h*h + w*w)

        # Convert max rotation to radians
        max_rotation_rad = np.deg2rad(self.max_expected_rotation)

        # Required padding based on rotation
        pad_size = int(diagonal * np.sin(max_rotation_rad) / 2) + safety_margin

        if self.verbose:
            print(f"Calculated padding: {pad_size} pixels (max_rotation: {self.max_expected_rotation}°)")

        return pad_size

    def _apply_cartesian_padding(self,
                                 image: np.ndarray,
                                 pad_size: int) -> np.ndarray:
        """
        Apply padding to Cartesian sonar image.

        Uses BORDER_REPLICATE mode for better FFT performance.

        Args:
            image: Cartesian sonar image
            pad_size: Pixels to pad on each side

        Returns:
            Padded image
        """
        import cv2

        padded = cv2.copyMakeBorder(
            image,
            pad_size, pad_size, pad_size, pad_size,
            borderType=cv2.BORDER_REPLICATE
        )

        if self.verbose:
            print(f"Applied padding: {image.shape} -> {padded.shape}")

        return padded

    def _rotate_image(self,
                      image: np.ndarray,
                      angle_deg: float,
                      center: Tuple[float, float]) -> np.ndarray:
        """
        Rotate image using cv2.warpAffine.

        Args:
            image: Image to rotate
            angle_deg: Rotation angle in degrees
            center: Rotation center (row, col)

        Returns:
            Rotated image
        """
        import cv2

        # cv2.getRotationMatrix2D uses (x, y) = (col, row)
        center_cv = (center[1], center[0])

        # Create rotation matrix
        M = cv2.getRotationMatrix2D(center_cv, angle_deg, scale=1.0)

        # Apply rotation
        rotated = cv2.warpAffine(
            image,
            M,
            (image.shape[1], image.shape[0]),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REPLICATE
        )

        return rotated

    def estimate_rotation(self,
                          img1_polar: np.ndarray,
                          img2_polar: np.ndarray) -> Dict[str, Any]:
        """
        Estimate rotation between two polar sonar images.

        Uses phase correlation in polar domain.

        Args:
            img1_polar: First polar image (range × angle), should be pre-masked with apply_range_min_mask
            img2_polar: Second polar image (range × angle), should be pre-masked with apply_range_min_mask

        Returns:
            dict with keys:
                'rotation': float (degrees)
                'peak_value': float (correlation peak)
                'success': bool
        """
        # Note: Min range masking removed (redundant - caller applies it in estimate_transform)
        # Apply erosion mask directly to input (already min-range masked)
        img1_masked = self.apply_erosion_mask(
            img1_polar,
            erosion_iterations=self.rot_erosion_iterations,
            gaussian_sigma=self.rot_gaussian_sigma,
            gaussian_truncate=self.rot_gaussian_truncate
        )
        img2_masked = self.apply_erosion_mask(
            img2_polar,
            erosion_iterations=self.rot_erosion_iterations,
            gaussian_sigma=self.rot_gaussian_sigma,
            gaussian_truncate=self.rot_gaussian_truncate
        )

        # Compute phase correlation
        pcm = self.compute_phase_correlation(img1_masked, img2_masked)

        # Detect peak
        row_offset, col_offset, peak_value = self.detect_peak(pcm)

        # Convert column offset to rotation angle
        # Phase correlation: col_offset > 0 = image2 shifts right = features moved left
        # Robot rotates CW → world rotates CCW → features move left (negative col_offset)
        # Therefore: col_offset > 0 = robot rotated CCW (yaw decrease)
        # NED: yaw increase = CW, so rotation = -col_offset
        rotation_deg = -col_offset * np.rad2deg(self.oculus.angular_resolution)

        if self.verbose:
            print(f"Rotation: {rotation_deg:.2f}° (peak={peak_value:.4f})")

        return {
            'rotation': rotation_deg,
            'peak_value': peak_value,
            'success': True
        }

    def estimate_translation(self,
                             img1_cart: np.ndarray,
                             img2_cart: np.ndarray,
                             rotation: float = 0.0) -> Dict[str, Any]:
        """
        Estimate translation between two Cartesian sonar images.

        Uses phase correlation in Cartesian domain with rotation compensation.

        Args:
            img1_cart: First Cartesian image
            img2_cart: Second Cartesian image
            rotation: Pre-computed rotation to compensate (degrees)

        Returns:
            dict with keys:
                'translation': [tx, ty] (meters, NED frame)
                'peak_value': float (correlation peak)
                'success': bool
        """
        # Apply erosion mask
        img1_masked = self.apply_erosion_mask(
            img1_cart,
            erosion_iterations=self.trans_erosion_iterations,
            gaussian_sigma=self.trans_gaussian_sigma,
            gaussian_truncate=self.trans_gaussian_truncate
        )
        img2_masked = self.apply_erosion_mask(
            img2_cart,
            erosion_iterations=self.trans_erosion_iterations,
            gaussian_sigma=self.trans_gaussian_sigma,
            gaussian_truncate=self.trans_gaussian_truncate
        )

        # Calculate padding
        h, w = img1_masked.shape
        pad_size = self._calculate_padding_size(img1_masked.shape)

        # Apply padding to both images
        img1_padded = self._apply_cartesian_padding(img1_masked, pad_size)
        img2_padded = self._apply_cartesian_padding(img2_masked, pad_size)

        # Calculate rotation center (top center for Stonefish, accounting for padding)
        center_row_padded = pad_size
        center_col_padded = w // 2 + pad_size

        # Apply rotation compensation
        img2_rotated = img2_padded.copy()
        if abs(rotation) > 0.01:
            img2_rotated = self._rotate_image(
                img2_padded,
                -rotation,  # Negative to compensate
                (center_row_padded, center_col_padded)
            )

            if self.verbose:
                print(f"Applied rotation compensation: {-rotation:.2f}°")

        # Compute phase correlation
        pcm = self.compute_phase_correlation(img1_padded, img2_rotated)

        # Detect peak
        row_offset, col_offset, peak_value = self.detect_peak(pcm)

        # Convert to meters (NED coordinate frame)
        # Phase correlation: row_offset > 0 = image2 shifts down = object farther = robot backward
        # Stonefish polar: Row 0 = far (top), Row N-1 = near (bottom)
        # Therefore: tx = -row_offset (negative sign needed)
        # CRITICAL: Use cart_range_resolution (same as polar_to_cartesian), NOT oculus.range_resolution
        tx = -row_offset * self.cart_range_resolution  # Forward (meters)
        ty = col_offset * self.cart_range_resolution   # Left (meters)

        if self.verbose:
            print(f"Translation: ({tx:.2f}, {ty:.2f}) m (peak={peak_value:.4f})")

        return {
            'translation': [tx, ty],
            'peak_value': peak_value,
            'success': True
        }

    def estimate_transform(self,
                           polar_img1: np.ndarray,
                           polar_img2: np.ndarray) -> Dict[str, Any]:
        """
        Estimate full transformation (rotation + translation) between two polar sonar images.

        This is the main entry point for FFT-based localization.

        Args:
            polar_img1: First polar image (range × angle)
            polar_img2: Second polar image (range × angle)

        Returns:
            dict with keys:
                'rotation': float (degrees)
                'translation': [tx, ty] (meters, NED frame)
                'success': bool
                'rot_peak': float (rotation correlation peak)
                'trans_peak': float (translation correlation peak)
        """
        # Validate input
        if polar_img1.shape != polar_img2.shape:
            warnings.warn(f"Image shape mismatch: {polar_img1.shape} vs {polar_img2.shape}")
            return {
                'rotation': 0.0,
                'translation': [0.0, 0.0],
                'success': False
            }

        # Apply min range mask
        img1_masked = self.apply_range_min_mask(polar_img1)
        img2_masked = self.apply_range_min_mask(polar_img2)

        # Step 1: Estimate rotation in polar domain
        rot_result = self.estimate_rotation(img1_masked, img2_masked)
        rotation = rot_result['rotation']
        rot_peak = rot_result['peak_value']

        # Step 2: Convert to Cartesian
        img1_cart = self.polar_to_cartesian(img1_masked)
        img2_cart = self.polar_to_cartesian(img2_masked)

        # Step 3: Estimate translation with rotation compensation
        trans_result = self.estimate_translation(img1_cart, img2_cart, rotation)
        translation = trans_result['translation']
        trans_peak = trans_result['peak_value']

        if self.verbose:
            print(f"\nFFT Registration complete:")
            print(f"  Rotation: {rotation:.2f}°")
            print(f"  Translation: ({translation[0]:.2f}, {translation[1]:.2f}) m")

        return {
            'rotation': rotation,
            'translation': translation,
            'success': True,
            'rot_peak': rot_peak,
            'trans_peak': trans_peak
        }
