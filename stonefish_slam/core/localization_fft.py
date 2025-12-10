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
                 dft_upsample_factor: int = 100,
                 dft_refinement_enable: bool = True,
                 periodic_decomp_enable: bool = True,
                 roi_threshold: float = 10.0,
                 use_roi: bool = False,
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
            dft_upsample_factor: Upsampling factor for DFT subpixel refinement (default: 100)
            dft_refinement_enable: Enable DFT subpixel refinement (default: True)
            periodic_decomp_enable: Enable periodic decomposition (Moisan 2011, default: True)
            roi_threshold: Pixel intensity threshold for ROI computation (default: 10.0)
            use_roi: Enable ROI-based FFT processing (default: True)
            verbose: Enable debug output
        """
        self.oculus = oculus
        self.range_min = range_min
        self.verbose = verbose

        if self.verbose:
            print(f"[FFTLocalizer] Initialized with verbose=True, tilt={oculus.tilt_angle_deg}°", flush=True)

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

        # DFT subpixel refinement parameters
        self.dft_upsample_factor = dft_upsample_factor
        self.dft_refinement_enable = dft_refinement_enable

        # Periodic decomposition (Moisan 2011)
        self.periodic_decomp_enable = periodic_decomp_enable

        # ROI-based FFT parameters
        self.roi_threshold = roi_threshold
        self.use_roi = use_roi

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

    def _compute_roi(self, image: np.ndarray, threshold: float = None) -> Tuple[int, int, int, int]:
        """
        Compute bounding box containing all pixels above threshold.

        Args:
            image: Input image
            threshold: Pixel intensity threshold (default: self.roi_threshold)

        Returns:
            (row_min, row_max, col_min, col_max)
        """
        if threshold is None:
            threshold = self.roi_threshold

        mask = image > threshold
        rows = np.any(mask, axis=1)
        cols = np.any(mask, axis=0)

        if not np.any(rows) or not np.any(cols):
            # No valid pixels, return full image bounds
            return 0, image.shape[0], 0, image.shape[1]

        row_indices = np.where(rows)[0]
        col_indices = np.where(cols)[0]
        row_min, row_max = row_indices[0], row_indices[-1] + 1
        col_min, col_max = col_indices[0], col_indices[-1] + 1

        return row_min, row_max, col_min, col_max

    def _apply_roi(self, img1: np.ndarray, img2: np.ndarray,
                   threshold: float = None) -> Tuple[np.ndarray, np.ndarray, Tuple[int, int, int, int]]:
        """
        Compute common ROI for two images and crop both.

        Args:
            img1: First image
            img2: Second image
            threshold: Pixel intensity threshold

        Returns:
            (img1_roi, img2_roi, roi_bounds)
        """
        roi1 = self._compute_roi(img1, threshold)
        roi2 = self._compute_roi(img2, threshold)

        # Union of both ROIs (include both)
        row_min = min(roi1[0], roi2[0])
        row_max = max(roi1[1], roi2[1])
        col_min = min(roi1[2], roi2[2])
        col_max = max(roi1[3], roi2[3])

        img1_roi = img1[row_min:row_max, col_min:col_max]
        img2_roi = img2[row_min:row_max, col_min:col_max]

        return img1_roi, img2_roi, (row_min, row_max, col_min, col_max)

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

            # Calculate range resolution (slant range basis)
            # CRITICAL: Stonefish polar images cover FULL range from 0 to range_max
            # range_min is only used for masking unreliable near-field data
            range_resolution = self.oculus.range_max / rows

            # Calculate horizontal range resolution (for cartesian coordinate calculation)
            horizontal_range_resolution = range_resolution * np.cos(self.oculus.tilt_angle_rad)

            # Cache for translation estimation (horizontal plane)
            # NOTE: Use slant range_resolution for translation calculation
            # The Cartesian remap preserves polar row indexing, so pixel movement
            # corresponds to slant range, not horizontal range
            self.cart_range_resolution = range_resolution

            # Apply tilt correction: project slant range to horizontal range
            # When sonar is tilted down, measured range is slant range
            # Horizontal range = slant range * cos(tilt_angle)
            horizontal_range_max = self.oculus.range_max * np.cos(self.oculus.tilt_angle_rad)

            # Maximum lateral extent based on FOV (use horizontal range)
            horizontal_fov_deg = np.rad2deg(self.oculus.horizontal_fov)
            max_lateral = horizontal_range_max * np.sin(np.radians(horizontal_fov_deg / 2.0))

            # Cartesian image dimensions (same resolution as horizontal range)
            cart_width = int(np.ceil(2 * max_lateral / horizontal_range_resolution))
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
            x_proj = horizontal_range_max - horizontal_range_resolution * YY
            y_proj = horizontal_range_resolution * (-cart_width / 2.0 + XX + 0.5)

            # Compute horizontal range and bearing from projected coordinates
            horizontal_range = np.sqrt(np.square(x_proj) + np.square(y_proj))
            bearing_polar = np.arctan2(y_proj, x_proj)

            # Convert horizontal range back to slant range for polar image indexing
            # Polar image rows are indexed by slant range, not horizontal range
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

        # Apply valid mask to explicitly mark FOV boundary
        # This ensures erosion mask correctly identifies FOV edges
        valid_mask_float = self.p2c_cache['valid_mask'].astype(np.float32) / 255.0
        cartesian_image = cartesian_image * valid_mask_float

        return cartesian_image

    def _periodic_decomposition(self, image: np.ndarray) -> np.ndarray:
        """
        Moisan (2011) periodic-plus-smooth decomposition.
        경계 불연속으로 인한 spectral leakage 제거.

        Reference: Moisan, L. "Periodic Plus Smooth Image Decomposition"
                   J Math Imaging Vis 39, 161-179 (2011)
        """
        u = image.astype(np.float64)
        M, N = u.shape

        # Step 1: Compute boundary jump image v
        v = np.zeros_like(u)
        v[0, :] = u[-1, :] - u[0, :]      # Top row: bottom - top
        v[-1, :] = u[0, :] - u[-1, :]     # Bottom row: top - bottom
        v[:, 0] += u[:, -1] - u[:, 0]     # Left col: right - left
        v[:, -1] += u[:, 0] - u[:, -1]    # Right col: left - right

        # Step 2: Solve Poisson equation via FFT
        v_fft = np.fft.fft2(v)

        q = np.arange(M).reshape(M, 1)
        r = np.arange(N).reshape(1, N)
        divisor = 2 * np.cos(2 * np.pi * q / M) + 2 * np.cos(2 * np.pi * r / N) - 4

        with np.errstate(divide='ignore', invalid='ignore'):
            s_fft = np.divide(v_fft, divisor, out=np.zeros_like(v_fft), where=divisor != 0)
        s_fft[0, 0] = 0  # DC component is zero

        # Step 3: Periodic = original - smooth
        s = np.real(np.fft.ifft2(s_fft))
        return u - s

    def compute_phase_correlation(self,
                                  img1: np.ndarray,
                                  img2: np.ndarray,
                                  return_cross_power: bool = False,
                                  apply_periodic_decomp: bool = None):
        """
        Compute phase correlation between two images.

        Args:
            img1: First image
            img2: Second image
            return_cross_power: If True, return (pcm, cross_power_spectrum) tuple
            apply_periodic_decomp: Apply periodic decomposition (default: self.periodic_decomp_enable)

        Returns:
            Phase Correlation Matrix (PCM), or (PCM, cross_power_spectrum) if return_cross_power=True
        """
        # Apply periodic decomposition to reduce spectral leakage
        if apply_periodic_decomp is None:
            apply_periodic_decomp = self.periodic_decomp_enable

        if apply_periodic_decomp:
            img1 = self._periodic_decomposition(img1)
            img2 = self._periodic_decomposition(img2)

        # Compute 2D FFT (no fftshift - standard phase correlation)
        F1 = np.fft.fft2(img1)
        F2 = np.fft.fft2(img2)

        # Compute cross power spectrum
        cross_power = F1 * np.conj(F2)

        # Normalize (phase correlation)
        eps = 1e-10
        cross_power_spectrum = cross_power / (np.abs(cross_power) + eps)

        # Inverse FFT to get PCM
        # fftshift applied AFTER ifft2 for peak detection at center
        pcm = np.fft.ifft2(cross_power_spectrum)
        pcm = np.abs(pcm)
        pcm = np.fft.fftshift(pcm)

        if return_cross_power:
            # Return cross_power_spectrum BEFORE ifftshift (needed for DFT refinement)
            return pcm, cross_power_spectrum
        return pcm

    def detect_peak(self,
                    pcm: np.ndarray,
                    subpixel: bool = True,
                    cross_power_spectrum: Optional[np.ndarray] = None,
                    use_dft_refinement: Optional[bool] = None) -> Tuple[float, float, float]:
        """
        Detect peak in Phase Correlation Matrix.

        Args:
            pcm: Phase Correlation Matrix
            subpixel: Enable subpixel accuracy using parabolic fitting
            cross_power_spectrum: Normalized cross-power spectrum for DFT refinement (optional)
            use_dft_refinement: Enable DFT subpixel refinement (default: self.dft_refinement_enable)

        Returns:
            (row_offset, col_offset, peak_value)
        """
        # Find maximum peak
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
                    print(f"Parabolic refinement failed: {e}")

        # DFT subpixel refinement (after parabolic fitting)
        if use_dft_refinement is None:
            use_dft_refinement = self.dft_refinement_enable

        if use_dft_refinement and cross_power_spectrum is not None and peak_value > 0:
            try:
                row_offset, col_offset = self._dft_subpixel_refinement(
                    cross_power_spectrum,
                    row_offset,
                    col_offset
                )
            except Exception as e:
                if self.verbose:
                    print(f"DFT refinement failed, using parabolic result: {e}")

        return row_offset, col_offset, peak_value

    def _upsampled_dft(self,
                       cross_power_spectrum: np.ndarray,
                       upsample_factor: int,
                       row_offset: float,
                       col_offset: float) -> np.ndarray:
        """
        Upsampled DFT by matrix multiplication (Guizar-Sicaros et al. 2008).

        Computes upsampled DFT in a small region around the specified offset.
        This is much faster than upsampling the entire cross-power spectrum.

        Args:
            cross_power_spectrum: Normalized cross-power spectrum (non-shifted)
            upsample_factor: Upsampling factor (e.g., 100)
            row_offset: Row offset from center (pixels)
            col_offset: Column offset from center (pixels)

        Returns:
            Upsampled region (1.5 × 1.5 pixels at upsampled resolution)
        """
        h, w = cross_power_spectrum.shape

        # Define upsampled region size (1.5 pixels at original resolution)
        region_size = int(np.ceil(1.5 * upsample_factor))

        # Generate upsampled sample positions centered around initial offset
        # Guizar-Sicairos 2008: sample DFT at positions around the initial estimate
        upsampled_grid = np.arange(region_size) - region_size // 2

        # Sample positions in original pixel coordinates (centered at offset)
        row_positions = row_offset + upsampled_grid / upsample_factor
        col_positions = col_offset + upsampled_grid / upsample_factor

        # Frequency indices for non-shifted spectrum (0, 1, ..., N/2, -N/2+1, ..., -1)
        row_freq_idx = np.fft.fftfreq(h) * h  # Returns: 0, 1, ..., h/2, -h/2+1, ..., -1
        col_freq_idx = np.fft.fftfreq(w) * w

        # DFT kernel: exp(-2πi * position * freq_idx / N)
        # For each sample position, compute contribution from all frequency components
        row_kernel = np.exp(-1j * 2 * np.pi * np.outer(row_positions, row_freq_idx) / h)
        col_kernel = np.exp(-1j * 2 * np.pi * np.outer(col_positions, col_freq_idx) / w)

        # Matrix multiply: upsampled_region[i,j] = sum over all freq of kernel * spectrum
        upsampled_region = row_kernel @ cross_power_spectrum @ col_kernel.T

        return upsampled_region

    def _dft_subpixel_refinement(self,
                                 cross_power_spectrum: np.ndarray,
                                 row_offset_init: float,
                                 col_offset_init: float) -> Tuple[float, float]:
        """
        Refine subpixel offset using upsampled DFT (Guizar-Sicaros et al. 2008).

        Args:
            cross_power_spectrum: Normalized cross-power spectrum (non-shifted)
            row_offset_init: Initial row offset from parabolic fitting (pixels)
            col_offset_init: Initial column offset from parabolic fitting (pixels)

        Returns:
            (refined_row_offset, refined_col_offset) in pixels
        """
        # Compute upsampled DFT around initial offset
        upsampled_region = self._upsampled_dft(
            cross_power_spectrum,
            self.dft_upsample_factor,
            row_offset_init,
            col_offset_init
        )

        # Find peak in upsampled region
        upsampled_abs = np.abs(upsampled_region)
        peak_idx = np.unravel_index(np.argmax(upsampled_abs), upsampled_abs.shape)

        region_size = upsampled_region.shape[0]
        region_center = region_size // 2

        # Compute offset from region center (in upsampled pixels)
        row_shift_upsampled = peak_idx[0] - region_center
        col_shift_upsampled = peak_idx[1] - region_center

        # Convert to original pixel units
        row_offset_refined = row_offset_init + row_shift_upsampled / self.dft_upsample_factor
        col_offset_refined = col_offset_init + col_shift_upsampled / self.dft_upsample_factor

        if self.verbose:
            print(f"  DFT refinement: ({row_offset_init:.2f}, {col_offset_init:.2f}) "
                  f"→ ({row_offset_refined:.4f}, {col_offset_refined:.4f})")

        return row_offset_refined, col_offset_refined

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

        # Apply ROI to reduce computation
        if self.use_roi:
            img1_masked, img2_masked, roi_bounds = self._apply_roi(img1_masked, img2_masked)
            if self.verbose:
                print(f"[ROI] Rotation: {img1_polar.shape} -> {img1_masked.shape} "
                      f"(rows: {roi_bounds[0]}:{roi_bounds[1]}, cols: {roi_bounds[2]}:{roi_bounds[3]})")

        # Normalize images to 0~255 uint8 for CLAHE
        import cv2
        img1_u8 = (img1_masked / img1_masked.max() * 255).astype(np.uint8) if img1_masked.max() > 0 else img1_masked.astype(np.uint8)
        img2_u8 = (img2_masked / img2_masked.max() * 255).astype(np.uint8) if img2_masked.max() > 0 else img2_masked.astype(np.uint8)

        # Apply CLAHE
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        img1_clahe = clahe.apply(img1_u8)
        img2_clahe = clahe.apply(img2_u8)

        # Normalize to 0~1 for FFT
        img1_norm = img1_clahe.astype(np.float64) / 255.0
        img2_norm = img2_clahe.astype(np.float64) / 255.0

        # Phase correlation with DFT refinement
        pcm, cross_power = self.compute_phase_correlation(img1_norm, img2_norm, return_cross_power=True)
        row_offset, col_offset, peak_value = self.detect_peak(pcm, cross_power_spectrum=cross_power)

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
            import os
            print(f"[Rotation Debug]", flush=True)
            print(f"  PCM shape: {pcm.shape}", flush=True)
            print(f"  Peak loc: row={peak_loc[0]}, col={peak_loc[1]}", flush=True)
            print(f"  Peak value: {peak_value:.4f}", flush=True)
            print(f"  col_offset: {col_offset:.2f} pixels", flush=True)
            print(f"  angular_resolution: {np.rad2deg(self.oculus.angular_resolution):.4f}°/pixel", flush=True)
            print(f"  rotation_deg: {rotation_deg:.2f}°", flush=True)
            # Save debug images
            debug_dir = "/tmp/fft_debug"
            os.makedirs(debug_dir, exist_ok=True)
            cv2.imwrite(f"{debug_dir}/rot_polar1.png", (img1_polar / img1_polar.max() * 255).astype(np.uint8) if img1_polar.max() > 0 else img1_polar.astype(np.uint8))
            cv2.imwrite(f"{debug_dir}/rot_polar2.png", (img2_polar / img2_polar.max() * 255).astype(np.uint8) if img2_polar.max() > 0 else img2_polar.astype(np.uint8))
            cv2.imwrite(f"{debug_dir}/rot_norm1.png", (img1_norm * 255).astype(np.uint8))
            cv2.imwrite(f"{debug_dir}/rot_norm2.png", (img2_norm * 255).astype(np.uint8))
            cv2.imwrite(f"{debug_dir}/rot_pcm.png", (pcm / pcm.max() * 255).astype(np.uint8) if pcm.max() > 0 else pcm.astype(np.uint8))
            np.save(f"{debug_dir}/rot_img1.npy", img1_norm)
            np.save(f"{debug_dir}/rot_img2.npy", img2_norm)
            print(f"  Debug images saved to {debug_dir}", flush=True)

        return {
            'rotation': rotation_deg,
            'peak_value': peak_value,
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
                'variance_x': float (meters^2)
                'variance_y': float (meters^2)
                'success': bool
        """
        import cv2

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

        # Apply ROI to reduce computation
        if self.use_roi:
            img1_roi, img2_roi, roi_bounds = self._apply_roi(img1_padded, img2_rotated)
            if self.verbose:
                print(f"[ROI] Translation: {img1_padded.shape} -> {img1_roi.shape} "
                      f"(rows: {roi_bounds[0]}:{roi_bounds[1]}, cols: {roi_bounds[2]}:{roi_bounds[3]})")
        else:
            img1_roi = img1_padded
            img2_roi = img2_rotated

        # Normalize images to 0~255 uint8 for CLAHE
        img1_u8 = (img1_roi / img1_roi.max() * 255).astype(np.uint8) if img1_roi.max() > 0 else img1_roi.astype(np.uint8)
        img2_u8 = (img2_roi / img2_roi.max() * 255).astype(np.uint8) if img2_roi.max() > 0 else img2_roi.astype(np.uint8)

        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        img1_clahe = clahe.apply(img1_u8)
        img2_clahe = clahe.apply(img2_u8)

        # Normalize to 0~1 for FFT
        img1_norm = img1_clahe.astype(np.float64) / 255.0
        img2_norm = img2_clahe.astype(np.float64) / 255.0

        # Phase correlation with DFT refinement
        # NOTE: Disable periodic decomposition for Cartesian images (fan-shaped with large zero regions)
        # Moisan (2011) assumes full image data; zero-padded regions cause artifacts
        pcm, cross_power = self.compute_phase_correlation(
            img1_norm, img2_norm,
            return_cross_power=True,
            apply_periodic_decomp=False
        )
        row_offset, col_offset, peak_value = self.detect_peak(pcm, cross_power_spectrum=cross_power)

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

        return {
            'translation': [tx, ty],
            'peak_value': peak_value,
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
            print(f"  Rotation: {rotation:.2f}° (peak: {rot_peak:.4f})")
            print(f"  Translation: ({translation[0]:.2f}, {translation[1]:.2f}) m (peak: {trans_peak:.4f})")
            print(f"  Covariance diag: [{trans_result['variance_x']:.4f}, {trans_result['variance_y']:.4f}, {rot_result['variance_theta']:.6f}]")

        return {
            'rotation': rotation,
            'translation': translation,
            'covariance': covariance,
            'success': True,
            'rot_peak': rot_peak,
            'trans_peak': trans_peak
        }
