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
                 rot_erosion_iterations: int = 1,  # Match reference implementation
                 rot_gaussian_sigma: float = 4.0,
                 rot_gaussian_truncate: float = 2.0,  # Match reference implementation
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

        # Apply Gaussian smoothing for soft mask edges (reference implementation)
        # Based on krit_fft working implementation (lines 297-318)
        from scipy.ndimage import gaussian_filter
        mask = gaussian_filter(shrink, sigma=gaussian_sigma, truncate=gaussian_truncate)

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
            # CRITICAL: Stonefish polar images cover FULL range from 0 to range_max
            # range_min is only used for masking unreliable near-field data
            range_resolution = self.oculus.range_max / rows

            # Cache for translation estimation
            # CRITICAL: Cartesian image is horizontal projection, so use horizontal range resolution
            # horizontal_range = slant_range * cos(tilt), so horizontal_resolution = slant_resolution * cos(tilt)
            self.cart_range_resolution = range_resolution * np.cos(self.oculus.tilt_angle_rad)

            # Apply tilt correction: project slant range to horizontal range
            # When sonar is tilted down, measured range is slant range
            # Horizontal range = slant range * cos(tilt_angle)
            horizontal_range_max = self.oculus.range_max * np.cos(self.oculus.tilt_angle_rad)

            # Maximum lateral extent based on FOV (use horizontal range)
            horizontal_fov_deg = np.rad2deg(self.oculus.horizontal_fov)
            max_lateral = horizontal_range_max * np.sin(np.radians(horizontal_fov_deg / 2.0))

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

            # Convert pixel indices to metric coordinates (projected horizontal plane)
            # IMPORTANT: Stonefish FLS row convention is OPPOSITE of Oculus
            # Stonefish: row=0 (top) is FAR range, row=max (bottom) is NEAR range
            # Cartesian: YY=0 (top) should be FAR, YY=max (bottom) should be NEAR
            # These are PROJECTED coordinates on horizontal plane
            x_proj = horizontal_range_max - range_resolution * YY
            y_proj = range_resolution * (-cart_width / 2.0 + XX + 0.5)

            # Compute horizontal range and bearing from projected coordinates
            horizontal_range = np.sqrt(np.square(x_proj) + np.square(y_proj))
            bearing_polar = np.arctan2(y_proj, x_proj)

            # Convert horizontal range back to slant range (reverse tilt projection)
            # slant_range = horizontal_range / cos(tilt)
            cos_tilt = np.cos(self.oculus.tilt_angle_rad)
            r_polar = horizontal_range / cos_tilt if cos_tilt > 1e-6 else horizontal_range

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
                    subpixel: bool = True) -> Tuple[float, float, float, float]:
        """
        Detect peak in Phase Correlation Matrix.

        Args:
            pcm: Phase Correlation Matrix
            subpixel: Enable subpixel accuracy using parabolic fitting

        Returns:
            (row_offset, col_offset, peak_value, ppr)
        """
        # Find maximum (1st peak)
        peak_value = np.max(pcm)
        peak_loc = np.unravel_index(np.argmax(pcm), pcm.shape)

        # Detect 2nd peak for PPR calculation
        pcm_masked = pcm.copy()
        r, c = peak_loc
        h, w = pcm.shape
        # Mask 5x5 region around 1st peak
        r_min, r_max = max(0, r-2), min(h, r+3)
        c_min, c_max = max(0, c-2), min(w, c+3)
        pcm_masked[r_min:r_max, c_min:c_max] = 0
        second_peak = np.max(pcm_masked)

        # Calculate PPR (Peak-to-Peak Ratio)
        ppr = peak_value / (second_peak + 1e-10)

        # Convert to offset from center
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

        return row_offset, col_offset, peak_value, ppr

    def compute_peak_variance(self,
                              pcm: np.ndarray,
                              peak_loc: Tuple[int, int],
                              resolution: float) -> Tuple[float, float]:
        """
        Compute variance around peak using weighted second moment.

        Based on Hurtós et al. (2015) methodology.

        Args:
            pcm: Phase correlation matrix
            peak_loc: (row, col) of peak
            resolution: meters per pixel (or degrees per pixel for rotation)

        Returns:
            (variance_row, variance_col) in resolution^2 units
        """
        r, c = peak_loc
        h, w = pcm.shape

        # Extract 7x7 neighborhood (or smaller at edges)
        size = 3  # 7x7 window
        r_min, r_max = max(0, r-size), min(h, r+size+1)
        c_min, c_max = max(0, c-size), min(w, c+size+1)

        neighborhood = pcm[r_min:r_max, c_min:c_max]

        # Threshold at 50% of peak (Hurtós method)
        threshold = 0.5 * np.max(neighborhood)
        mask = neighborhood > threshold

        if np.sum(mask) < 3:
            # Not enough points, return default variance
            return (resolution * 2) ** 2, (resolution * 2) ** 2

        # Weighted variance calculation
        weights = neighborhood[mask]
        weights = weights / np.sum(weights)

        rows, cols = np.where(mask)
        center_r = r - r_min
        center_c = c - c_min

        var_r = np.sum(weights * (rows - center_r) ** 2) * (resolution ** 2)
        var_c = np.sum(weights * (cols - center_c) ** 2) * (resolution ** 2)

        # Minimum variance (prevent zero)
        min_var = (resolution * 0.5) ** 2
        return max(var_r, min_var), max(var_c, min_var)

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
                'ppr': float (peak-to-peak ratio)
                'variance_theta': float (radians^2)
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

        # Detect peak (with PPR)
        row_offset, col_offset, peak_value, ppr = self.detect_peak(pcm)

        # Convert column offset to rotation angle
        # Phase correlation: col_offset > 0 means CW rotation (positive angle)
        # Reference: krit_fft line 873
        rotation_deg = col_offset * np.rad2deg(self.oculus.angular_resolution)

        # Compute rotation variance
        peak_loc = np.unravel_index(np.argmax(pcm), pcm.shape)
        _, var_col = self.compute_peak_variance(
            pcm,
            peak_loc,
            np.rad2deg(self.oculus.angular_resolution)
        )
        # Convert from degrees^2 to radians^2
        var_theta = np.deg2rad(np.sqrt(var_col)) ** 2

        if self.verbose:
            # Detailed debug output for rotation estimation
            print(f"[Rotation Debug]")
            print(f"  PCM shape: {pcm.shape}")
            print(f"  Peak loc: row={peak_loc[0]}, col={peak_loc[1]}")
            print(f"  Peak value: {peak_value:.4f}")
            print(f"  col_offset: {col_offset:.2f} pixels")
            print(f"  angular_resolution: {np.rad2deg(self.oculus.angular_resolution):.4f}°/pixel")
            print(f"  PPR: {ppr:.2f}")
            print(f"  rotation_deg: {rotation_deg:.2f}°")

        return {
            'rotation': rotation_deg,
            'peak_value': peak_value,
            'ppr': ppr,
            'variance_theta': var_theta,
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
                'ppr': float (peak-to-peak ratio)
                'variance_x': float (meters^2)
                'variance_y': float (meters^2)
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

        # Calculate rotation center (bottom center - sonar position, accounting for padding)
        # Reference: krit_fft line 928-931
        center_row_padded = h - 1 + pad_size
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

        # Detect peak (with PPR)
        row_offset, col_offset, peak_value, ppr = self.detect_peak(pcm)

        # Compute translation variance
        peak_loc = np.unravel_index(np.argmax(pcm), pcm.shape)
        var_row, var_col = self.compute_peak_variance(
            pcm,
            peak_loc,
            self.cart_range_resolution
        )

        # Convert to meters (NED coordinate frame)
        # Phase correlation: row_offset > 0 = image2 shifts down = object farther = robot backward
        # Stonefish polar: Row 0 = far (top), Row N-1 = near (bottom)
        # Therefore: tx = -row_offset (negative sign needed)
        # CRITICAL: Use cart_range_resolution (same as polar_to_cartesian), NOT oculus.range_resolution
        # NOTE: Tilt correction already applied in polar_to_cartesian projection
        # Cartesian image already in horizontal plane, no additional tilt correction needed
        tx = -row_offset * self.cart_range_resolution  # Forward (meters)
        ty = col_offset * self.cart_range_resolution   # Left (meters)

        if self.verbose:
            print(f"Translation: ({tx:.2f}, {ty:.2f}) m (peak={peak_value:.4f}, ppr={ppr:.2f})")

        return {
            'translation': [tx, ty],
            'peak_value': peak_value,
            'ppr': ppr,
            'variance_x': var_row,
            'variance_y': var_col,
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
                'covariance': np.ndarray (3×3, diagonal: [var_x, var_y, var_theta])
                'ppr_rot': float (rotation peak-to-peak ratio)
                'ppr_trans': float (translation peak-to-peak ratio)
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
                'covariance': np.eye(3),
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

        # Step 4: Build covariance matrix (3×3 diagonal)
        covariance = np.diag([
            trans_result['variance_x'],      # var(x) in meters^2
            trans_result['variance_y'],      # var(y) in meters^2
            rot_result['variance_theta']     # var(theta) in radians^2
        ])

        if self.verbose:
            print(f"\nFFT Registration complete:")
            print(f"  Rotation: {rotation:.2f}° (PPR: {rot_result['ppr']:.2f})")
            print(f"  Translation: ({translation[0]:.2f}, {translation[1]:.2f}) m (PPR: {trans_result['ppr']:.2f})")
            print(f"  Covariance diag: [{trans_result['variance_x']:.4f}, {trans_result['variance_y']:.4f}, {rot_result['variance_theta']:.6f}]")

        return {
            'rotation': rotation,
            'translation': translation,
            'covariance': covariance,
            'ppr_rot': rot_result['ppr'],
            'ppr_trans': trans_result['ppr'],
            'success': True,
            'rot_peak': rot_peak,
            'trans_peak': trans_peak
        }
