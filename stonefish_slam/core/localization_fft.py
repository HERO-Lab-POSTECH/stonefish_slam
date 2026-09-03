"""
FFT-based Sonar Image Registration for Localization

This module provides Fourier-based registration methods for Forward-Looking Sonar (FLS) images.
Based on: "Fourier-based Registration for Robust Forward-looking Sonar Mosaicing"
         in Low-visibility Underwater Environments (Hurtós et al., 2015)

Completely independent module - no ROS2 or ICP dependencies.
"""

import numpy as np
from scipy.interpolate import interp1d
import scipy.fft as _sfft
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
                 rotation_candidates: int = 1,
                 rotation_tilt_compensation: bool = False,
                 rotation_tilt_var_compensation: bool = True,
                 use_roi: bool = False,
                 remove_radial_mean: bool = False,
                 trans_lowpass: float = 0.0,
                 trans_clahe: bool = True,
                 trans_window: str = '',
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
            rotation_candidates: How many polar-correlation peaks to try as rotation
                hypotheses. 1 keeps the historical behaviour (global maximum only).
                >1 hands the choice to estimate_transform, which picks the hypothesis
                with the strongest translation correlation (default: 1)
            use_roi: Enable ROI-based FFT processing (default: False)
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

        # 회전 후보 개수. tilt 30° 에서 극좌표 상관면의 전역 최대가 정답인 비율은
        # 43% 뿐이고, 정답은 상위 피크 안에 들어 있다 — 어느 것인지는 병진 상관이
        # 답한다. 근거는 .hq/community/posts/finding/019.
        self.rotation_candidates = max(1, int(rotation_candidates))

        # 방위 컬럼 이동을 요각으로 바꿀 때 하향 틸트를 나눌지. 상수 cos τ 는
        # 보어사이트에서만 정확하다 — 시야 가장자리의 잔차 약 1% 는 남는다.
        self.rotation_tilt_compensation = bool(rotation_tilt_compensation)
        self.rotation_tilt_var_compensation = bool(rotation_tilt_var_compensation)

        # ROI-based FFT parameters
        self.roi_threshold = roi_threshold
        self.use_roi = use_roi

        # 몸체 고정 거리 띠 제거 (아래 remove_band_envelope 참고)
        self.remove_radial_mean = remove_radial_mean
        # 병진 상관 전처리 3종 (회전 경로에는 걸지 않는다 — 측정된 적이 없다).
        # tilt 30° 오프라인 188 쌍: 대조군 83.0%/0.152 m/척도 0.912 →
        # 0.50+clahe off+hann 이 89.9%/0.127 m/0.929 (finding/024).
        self.trans_lowpass = float(trans_lowpass)
        self.trans_clahe = bool(trans_clahe)
        self.trans_window = str(trans_window or '')
        self._lp_cache = {}
        self._win_cache = {}

        # Cache for polar_to_cartesian conversion (performance optimization)
        self.p2c_cache = None

        # Cache range_resolution from polar_to_cartesian for consistent translation estimation
        # CRITICAL: Must use same range_resolution for cart conversion and translation
        self.cart_range_resolution = None

    def remove_band_envelope(self, img_polar: np.ndarray) -> np.ndarray:
        """거리 행마다 방위각 평균을 빼서 몸체 고정 띠 성분을 없앤다.

        하향 틸트 FLS 가 바닥을 보면 반사는 소나 기하가 정하는 좁은 거리 띠 안에서만
        나온다. 그 띠는 차량과 함께 움직이므로 두 프레임 사이에서 **이동하지 않는다** —
        위상 상관에서 0 시프트 성분으로 들어가 지형이 만드는 진짜 peak 와 경쟁한다.
        극좌표에서 한 행은 등거리이므로, 행 평균이 곧 그 띠의 포락선이다.

        평균 이하 화소는 0 으로 자른다: 뒤따르는 정규화·CLAHE·erosion 마스크가 모두
        비음수 강도를 전제한다.

        Args:
            img_polar: 극좌표 소나 영상 (거리 × 방위)

        Returns:
            같은 shape 의 float32 영상.
        """
        a = img_polar.astype(np.float32)
        return np.clip(a - a.mean(axis=1, keepdims=True), 0.0, None)

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
        # 투영 방식(sonar.projection)과 고도계 값. altitude 모드는 고도가 바뀌면
        # 사상이 달라지므로 0.25 m 격자로 양자화해 캐시 키에 넣는다 — 매 프레임
        # 재구축하면 remap 맵 생성 비용이 그대로 들어온다.
        projection = getattr(self.oculus, 'projection', 'legacy')
        altitude = getattr(self.oculus, 'altitude_m', None)
        cache_key = (projection,
                     round(altitude / 0.25) if projection == 'altitude' and altitude else None)

        if self.p2c_cache is None or self.p2c_cache.get('key') != cache_key:
            if self.verbose:
                print(f"Building polar-to-cartesian transformation maps (key={cache_key})...")

            # Calculate range resolution (slant range basis)
            # CRITICAL: Stonefish polar images cover FULL range from 0 to range_max
            # range_min is only used for masking unreliable near-field data
            range_resolution = self.oculus.range_max / rows

            # Cache for translation estimation (horizontal plane)
            # NOTE: Use slant range_resolution for true XY projection
            # Cartesian image represents horizontal plane coordinates
            self.cart_range_resolution = range_resolution

            # 경사거리 r 을 어떤 좌표 C 로 그릴지가 이 변환의 전부다. 전진 d 에 대해
            # dC/drho 가 1 이어야 직교영상에서 잰 병진이 실제 병진과 같아진다.
            #   legacy       C = r*cos(tau)      원본. dC/drho = cos(theta)cos(tau) 로 압축된다
            #   inv_cos_tilt C = r/cos(tau)      dC/drho = cos(theta)/cos(tau), tau 근처에서 1
            #   altitude     C = sqrt(r^2-h^2)   평평한 바닥에서 정확히 1
            # feature_extraction._project_range 와 같은 규약을 쓴다.
            cos_tilt = float(np.cos(self.oculus.tilt_angle_rad))
            if projection == 'altitude' and altitude and altitude > 0.0:
                h_sq = float(altitude) ** 2
                fwd = lambda r: np.sqrt(np.maximum(np.square(r) - h_sq, 0.0))
                inv = lambda c: np.sqrt(np.square(c) + h_sq)
            elif projection == 'inv_cos_tilt' and cos_tilt > 1e-6:
                fwd = lambda r: r / cos_tilt
                inv = lambda c: c * cos_tilt
            else:
                scale = cos_tilt if cos_tilt > 1e-6 else 1.0
                fwd = lambda r: r * scale
                inv = lambda c: c / scale

            horizontal_range_max = float(fwd(self.oculus.range_max))

            # Maximum lateral extent based on FOV (use horizontal range)
            horizontal_fov_deg = np.rad2deg(self.oculus.horizontal_fov)
            max_lateral = horizontal_range_max * np.sin(np.radians(horizontal_fov_deg / 2.0))

            # Cartesian image dimensions (use slant range resolution for true projection)
            cart_width = int(np.ceil(2 * max_lateral / range_resolution))
            cart_height = int(np.ceil(horizontal_range_max / range_resolution))

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
            # These are PROJECTED coordinates on horizontal plane (use slant range resolution)
            x_proj = horizontal_range_max - range_resolution * YY
            y_proj = range_resolution * (-cart_width / 2.0 + XX + 0.5)

            # Compute horizontal range and bearing from projected coordinates
            horizontal_range = np.sqrt(np.square(x_proj) + np.square(y_proj))
            bearing_polar = np.arctan2(y_proj, x_proj)

            # Convert horizontal range back to slant range for polar image indexing
            # Polar image rows are indexed by slant range, not horizontal range
            r_polar = inv(horizontal_range)

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
                'key': cache_key,
                'map_x': map_x,
                'map_y': map_y,
                'cart_height': cart_height,
                'cart_width': cart_width,
                'valid_mask': valid_mask.astype(np.uint8) * 255  # Cache valid region mask
            }

            if self.verbose:
                print(f"Transformation maps built: {cart_height}x{cart_width} cartesian image")
                print(f"  range_max: {self.oculus.range_max}m, horizontal_range_max: {horizontal_range_max:.2f}m")
                print(f"  range_resolution: {range_resolution:.6f} m/pixel")
                print(f"  cart_range_resolution: {self.cart_range_resolution:.6f} m/pixel")
                print(f"  cart_height: {cart_height} (was {rows}), cart_width: {cart_width}")

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
        v_fft = _sfft.fft2(v, workers=-1)

        q = np.arange(M).reshape(M, 1)
        r = np.arange(N).reshape(1, N)
        divisor = 2 * np.cos(2 * np.pi * q / M) + 2 * np.cos(2 * np.pi * r / N) - 4

        with np.errstate(divide='ignore', invalid='ignore'):
            s_fft = np.divide(v_fft, divisor, out=np.zeros_like(v_fft), where=divisor != 0)
        s_fft[0, 0] = 0  # DC component is zero

        # Step 3: Periodic = original - smooth
        s = np.real(_sfft.ifft2(s_fft, workers=-1))
        return u - s

    def compute_phase_correlation(self,
                                  img1: np.ndarray,
                                  img2: np.ndarray,
                                  return_cross_power: bool = False,
                                  apply_periodic_decomp: bool = None,
                                  f1: np.ndarray = None,
                                  lowpass: bool = False):
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
        # scipy.fft 는 numpy.fft 와 같은 결과를 내면서 스레드를 쓴다 — 1028x1422
        # 에서 49.9 → 3.9 ms (실측 2026-09-02). 회전 후보 K 개마다 도는 변환이라
        # 이 한 줄이 K=9 의 온라인 성립 여부를 갈랐다.
        # f1 이 주어지면 img1 의 변환을 건너뛴다. 회전 후보 K 개를 시험하는 동안
        # img1 쪽은 한 번도 바뀌지 않으므로 K-1 회가 순수 낭비다.
        F1 = _sfft.fft2(img1, workers=-1) if f1 is None else f1
        F2 = _sfft.fft2(img2, workers=-1)

        # Compute cross power spectrum
        cross_power = F1 * np.conj(F2)
        # 고주파(스페클) 억제. 병진 경로에서만 호출자가 켠다 — 회전 경로에
        # 거는 것은 측정된 적이 없다.
        if lowpass and self.trans_lowpass > 0.0:
            cross_power = cross_power * self._lowpass_mask(cross_power.shape)

        # Normalize (phase correlation)
        eps = 1e-10
        cross_power_spectrum = cross_power / (np.abs(cross_power) + eps)

        # Inverse FFT to get PCM
        # fftshift applied AFTER ifft2 for peak detection at center
        pcm = _sfft.ifft2(cross_power_spectrum, workers=-1)
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

    @property
    def deg_per_col(self) -> float:
        """방위 컬럼 1 칸이 소나 방위 몇 도인가. 이미지가 실제로 도는 각이다."""
        return float(np.rad2deg(self.oculus.angular_resolution))

    @property
    def _cos_tilt(self) -> float:
        """요각↔방위 환산 계수. 보정이 꺼져 있으면 1 이라 모든 환산이 항등이다."""
        if not self.rotation_tilt_compensation:
            return 1.0
        c = float(np.cos(self.oculus.tilt_angle_rad))
        return c if c > 1e-6 else 1.0

    def yaw_from_bearing(self, deg: float) -> float:
        """상관면이 준 방위 시프트를 차량 요각으로 되돌린다.

        하향 틸트 τ 인 소나에서 요잉 Δψ 는 방위축을 cos τ·Δψ 만 움직인다. 이미지
        정렬에는 방위 각을 그대로 써야 하고, 팩터그래프에 넣는 측정치만 이 환산을
        거친다 — 둘을 같이 건드리면 정렬 워프가 과회전한다.
        """
        return deg / self._cos_tilt

    def bearing_from_yaw(self, deg: float) -> float:
        """차량 요각을 이미지가 실제로 도는 각(방위 시프트)으로 바꾼다."""
        return deg * self._cos_tilt

    def polar_warp(self,
                   polar_img: np.ndarray,
                   tx: float, ty: float, dyaw_deg: float) -> np.ndarray:
        """프레임2 극좌표 영상을 프레임1 의 극좌표 격자로 리샘플한다.

        1 패스 추정 (tx, ty, dyaw) 을 되감아 두 영상을 대략 겹쳐 놓는다. 남은
        불일치를 다시 상관하면 1 패스에서 크게 틀린 쌍이 구제된다 — 이미 맞은
        쌍에는 잡음만 더하므로 **게이트가 기각한 쌍에만** 쓴다(finding/023).

        행=경사거리(row 0 이 far), 열=방위. 지면거리 rho=sqrt(r^2-h^2) 로 평탄
        해저를 가정한다 — `sonar.projection: altitude` 와 같은 가정이다.
        프레임1 의 (rho,theta) 에 보이는 점은 프레임2 에서 R(-dyaw)(p1 - t) 에 있다.
        """
        import cv2
        alt = float(getattr(self.oculus, 'altitude_m', 0.0) or 0.0)
        rows, cols = polar_img.shape
        res = self.oculus.range_resolution
        fov = self.oculus.horizontal_fov
        th = np.linspace(-fov / 2, fov / 2, cols)[None, :]
        r = ((rows - np.arange(rows, dtype=np.float64)) * res)[:, None]
        rho = np.sqrt(np.maximum(r ** 2 - alt ** 2, 0.0))

        x1, y1 = rho * np.cos(th), rho * np.sin(th)
        dx, dy = x1 - tx, y1 - ty
        c, s = np.cos(np.radians(-dyaw_deg)), np.sin(np.radians(-dyaw_deg))
        x2, y2 = c * dx - s * dy, s * dx + c * dy

        rho2 = np.hypot(x2, y2)
        th2 = np.arctan2(y2, x2)
        r2 = np.sqrt(rho2 ** 2 + alt ** 2)
        map_y = (rows - r2 / res).astype(np.float32)
        map_x = ((th2 + fov / 2) / fov * (cols - 1)).astype(np.float32)
        return cv2.remap(polar_img.astype(np.float32), map_x, map_y,
                         cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                         borderValue=0.0)

    def _rotation_peaks(self,
                        pcm: np.ndarray,
                        deg_per_col: float,
                        separation: int = 4) -> list:
        """상관면의 상위 피크들을 회전 가설로 뽑는다.

        각 항목은 (rotation_deg, peak_value, variance_theta). 분산을 여기서 같이
        계산하는 이유는 선택된 가설의 분산이 공분산에 들어가야 하기 때문이다 —
        전역 최대의 분산을 쓰면 다른 피크를 골랐을 때 팩터 그래프가 엉뚱한
        신뢰도를 받는다.

        서브픽셀 보간은 하지 않는다. 오프라인 측정에서 정수 후보만으로 이미
        이겼고(finding/019), 보간을 더하려면 선택 후 병진을 한 번 더 풀어야 한다.
        """
        work = pcm.copy()
        height, width = work.shape
        floor = work.min()
        peaks = []
        for _ in range(self.rotation_candidates):
            loc = np.unravel_index(np.argmax(work), work.shape)
            _, var_col = self.compute_peak_variance(pcm, loc, deg_per_col)
            peaks.append(((loc[1] - width // 2) * deg_per_col,
                          float(pcm[loc]),
                          np.deg2rad(np.sqrt(var_col)) ** 2))
            r0, r1 = max(0, loc[0] - separation), min(height, loc[0] + separation + 1)
            c0, c1 = max(0, loc[1] - separation), min(width, loc[1] + separation + 1)
            work[r0:r1, c0:c1] = floor
        return peaks

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
        rotation_deg = col_offset * self.deg_per_col

        # Compute rotation variance
        peak_loc = np.unravel_index(np.argmax(pcm), pcm.shape)
        _, var_col = self.compute_peak_variance(
            pcm,
            peak_loc,
            self.deg_per_col
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
            print(f"  deg_per_col: {self.deg_per_col:.4f}°/pixel", flush=True)
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

        result = {
            'rotation': rotation_deg,
            'peak_value': peak_value,
            'variance_theta': var_theta,
            'success': True
        }
        if self.rotation_candidates > 1:
            result['candidates'] = self._rotation_peaks(
                pcm, self.deg_per_col)
        return result

    def _lowpass_mask(self, shape):
        """정규화 반경 <= trans_lowpass 만 남기는 이진 마스크(fftshift 안 된 배치).

        이진이라 F1·M 과 conj(F2·M) 의 곱이 F1·conj(F2)·M 과 같다 — 그래서
        두 영상을 각각 역FFT 로 필터하는 대신 교차스펙트럼에 한 번만 곱한다.
        FFT 왕복이 후보당 4 회 늘지 않으므로 처리량을 깎지 않는다.
        """
        m = self._lp_cache.get(shape)
        if m is None:
            h, w = shape
            fy = np.fft.fftfreq(h)[:, None]
            fx = np.fft.fftfreq(w)[None, :]
            r = np.sqrt(fy ** 2 + fx ** 2) / 0.5          # 0 .. ~1.41
            m = (r <= self.trans_lowpass).astype(np.float64)
            self._lp_cache[shape] = m
        return m

    def _trans_prep(self, padded):
        """병진 상관 직전 전처리 — 정규화 + (선택) CLAHE + (선택) 창함수."""
        import cv2                                    # 이 파일 관행: 메서드 안 지역 import
        u8 = (padded / padded.max() * 255).astype(np.uint8) \
            if padded.max() > 0 else padded.astype(np.uint8)
        if self.trans_clahe:
            u8 = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(u8)
        n = u8.astype(np.float64) / 255.0
        if self.trans_window == 'hann':
            win = self._win_cache.get(n.shape)
            if win is None:
                win = np.outer(np.hanning(n.shape[0]), np.hanning(n.shape[1]))
                self._win_cache[n.shape] = win
            n = n * win
        return n

    def prepare_translation(self, img1_cart: np.ndarray, img2_cart: np.ndarray) -> dict:
        """회전 후보 사이에서 바뀌지 않는 준비 작업을 한 번만 해 둔다.

        `estimate_translation` 은 후보마다 두 이미지의 침식 마스크·패딩·CLAHE·정규화와
        img1 의 2D 변환을 다시 한다. 그런데 회전은 패딩 *이후* 에 img2 에만 걸리므로
        (`_rotate_image(img2_padded, ...)`) 그 앞 단계는 전부 후보와 무관하다.
        K=9 에서 후보당 96 ms 중 약 24 ms 가 이 낭비였다(실측 2026-09-02).

        `use_roi` 가 켜져 있으면 ROI 가 회전된 img2 에 의존해 img1 쪽도 후보마다
        달라지므로 이 경로를 쓰지 않는다 — 호출자가 None 을 받으면 옛 경로로 간다.
        """
        if self.use_roi:
            return None
        import cv2
        kw = dict(erosion_iterations=self.trans_erosion_iterations,
                  gaussian_sigma=self.trans_gaussian_sigma,
                  gaussian_truncate=self.trans_gaussian_truncate)
        m1 = self.apply_erosion_mask(img1_cart, **kw)
        m2 = self.apply_erosion_mask(img2_cart, **kw)
        h, w = m1.shape
        pad = self._calculate_padding_size(m1.shape)
        p1 = self._apply_cartesian_padding(m1, pad)
        p2 = self._apply_cartesian_padding(m2, pad)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        n1 = self._trans_prep(p1)
        return {
            'img1_norm': n1,
            'f1': _sfft.fft2(n1, workers=-1),
            'img2_padded': p2,
            'center': (h - 1 + pad, w // 2 + pad),
            'clahe': clahe,
        }

    def estimate_translation(self,
                             img1_cart: np.ndarray,
                             img2_cart: np.ndarray,
                             rotation: float = 0.0,
                             refine: bool = True,
                             prep: dict = None) -> Dict[str, Any]:
        """
        Estimate translation between two Cartesian sonar images.

        Uses phase correlation in Cartesian domain with rotation compensation.

        Args:
            img1_cart: First Cartesian image
            img2_cart: Second Cartesian image
            rotation: Pre-computed rotation to compensate (degrees)
            refine: Run subpixel refinement and peak variance. False is for scoring a
                rotation hypothesis that may be discarded — `peak_value` is the maximum
                of the correlation surface and is taken *before* refinement, so the score
                is identical either way while the 100x-upsampled DFT is not paid for.
                The chosen hypothesis is re-run with refine=True.

        Returns:
            dict with keys:
                'translation': [tx, ty] (meters, NED frame)
                'peak_value': float (correlation peak)
                'variance_x': float (meters^2)
                'variance_y': float (meters^2)
                'success': bool
        """
        import cv2

        if prep is None:
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
        else:
            # prepare_translation() 이 이미 해 둔 것들. 회전 후보 사이에서 안 바뀐다.
            img1_padded = None                      # img1_norm 을 직접 쓴다
            img2_padded = prep['img2_padded']
            center_row_padded, center_col_padded = prep['center']

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

        clahe = prep['clahe'] if prep is not None else cv2.createCLAHE(
            clipLimit=2.0, tileGridSize=(8, 8))

        # img1 쪽은 회전이 안 걸리므로 prep 이 있으면 이미 만들어져 있다.
        if prep is not None:
            img1_norm = prep['img1_norm']
        else:
            img1_norm = self._trans_prep(img1_roi)

        img2_norm = self._trans_prep(img2_roi)

        # Phase correlation with DFT refinement
        # NOTE: Disable periodic decomposition for Cartesian images (fan-shaped with large zero regions)
        # Moisan (2011) assumes full image data; zero-padded regions cause artifacts
        pcm, cross_power = self.compute_phase_correlation(
            img1_norm, img2_norm,
            return_cross_power=True,
            apply_periodic_decomp=False,
            f1=prep['f1'] if prep is not None else None,
            lowpass=True
        )
        row_offset, col_offset, peak_value = self.detect_peak(
            pcm, subpixel=refine,
            cross_power_spectrum=cross_power if refine else None)

        # Compute translation variance
        if refine:
            peak_loc = np.unravel_index(np.argmax(pcm), pcm.shape)
            var_row, var_col = self.compute_peak_variance(
                pcm,
                peak_loc,
                self.cart_range_resolution
            )
        else:
            var_row = var_col = 0.0

        # Convert to meters (NED coordinate frame)
        # Phase correlation: row_offset > 0 = image2 shifts down = object farther = robot backward
        # Stonefish polar: Row 0 = far (top), Row N-1 = near (bottom)
        # Therefore: tx = -row_offset (negative sign needed)
        # CRITICAL: Use cart_range_resolution (same as polar_to_cartesian), NOT oculus.range_resolution
        # NOTE: Tilt correction already applied in polar_to_cartesian projection
        # Cartesian image already in horizontal plane, no additional tilt correction needed
        tx = -row_offset * self.cart_range_resolution  # Forward (meters)
        ty = col_offset * self.cart_range_resolution   # Starboard (+y, NED body) (meters)

        if self.verbose:
            print(f"[Translation Debug]")
            print(f"  row_offset: {row_offset:.4f}, col_offset: {col_offset:.4f}")
            print(f"  cart_range_resolution: {self.cart_range_resolution:.6f} m/pixel")
            print(f"  tx = -{row_offset:.4f} * {self.cart_range_resolution:.6f} = {tx:.4f} m")
            print(f"  ty = {col_offset:.4f} * {self.cart_range_resolution:.6f} = {ty:.4f} m")

        return {
            'translation': [tx, ty],
            'peak_value': peak_value,
            'variance_x': var_row,
            'variance_y': var_col,
            'success': True
        }

    def estimate_transform(self,
                           polar_img1: np.ndarray,
                           polar_img2: np.ndarray,
                           rotation_override: Optional[float] = None) -> Dict[str, Any]:
        """
        Estimate full transformation (rotation + translation) between two polar sonar images.

        This is the main entry point for FFT-based localization.

        Args:
            polar_img1: First polar image (range × angle)
            polar_img2: Second polar image (range × angle)
            rotation_override: Optional rotation to use instead of FFT estimation (degrees)

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

        # 몸체 고정 띠 제거 (회전·병진 추정 양쪽의 입력에 적용한다)
        if self.remove_radial_mean:
            img1_masked = self.remove_band_envelope(img1_masked)
            img2_masked = self.remove_band_envelope(img2_masked)

        # Step 1: Estimate rotation in polar domain (or use override)
        rot_result = self.estimate_rotation(img1_masked, img2_masked)

        # Step 2: Convert to Cartesian
        img1_cart = self.polar_to_cartesian(img1_masked)
        img2_cart = self.polar_to_cartesian(img2_masked)

        # Step 3: Pick a rotation, then estimate translation under it.
        #
        # 전역 최대만 쓰면 tilt 30° 에서 57% 의 쌍이 엉뚱한 열을 고른다 — 극좌표
        # 상관면이 거리 변위와 방위 변위를 동시에 설명해야 해서 피크가 끌려간다.
        # 후보를 여러 개 두고 병진 상관이 가장 강한 것을 고르면 그 결합을 푼다.
        candidates = rot_result.get('candidates')
        if rotation_override is not None:
            rotation, rot_peak = rotation_override, rot_result['peak_value']
            var_theta = rot_result['variance_theta']
            if self.verbose:
                print(f"[FFT] Using rotation override: {rotation:.2f}° (FFT estimate: {rot_result['rotation']:.2f}°)")
            trans_result = self.estimate_translation(
                img1_cart, img2_cart, self.bearing_from_yaw(rotation))
        elif candidates:
            # 점수만 필요한 동안은 서브픽셀 보간을 끈다 — peak_value 는 보간 전에
            # 정해지므로 선택 결과가 같고, 버릴 후보 K-1 개에 100배 업샘플 DFT 를
            # 물리지 않는다. 이긴 후보만 refine=True 로 한 번 더 푼다.
            # 회전과 무관한 준비 작업(두 이미지의 침식 마스크·패딩, img1 의 CLAHE 와
            # 2D 변환)을 한 번만 한다. 후보마다 다시 하면 K-1 회가 낭비다.
            prep = self.prepare_translation(img1_cart, img2_cart)
            best_rot = best_peak = best_var = None
            best_score = -np.inf
            for cand_rot, cand_peak, cand_var in candidates:
                score = self.estimate_translation(
                    img1_cart, img2_cart, cand_rot, refine=False, prep=prep)['peak_value']
                if score > best_score:
                    best_score, best_rot, best_peak, best_var = score, cand_rot, cand_peak, cand_var
            rot_peak, var_theta = best_peak, best_var
            trans_result = self.estimate_translation(img1_cart, img2_cart, best_rot, prep=prep)
            rotation = self.yaw_from_bearing(best_rot)
            if self.verbose:
                print(f"[FFT] Rotation {rotation:.2f}° chosen from {len(candidates)} candidates "
                      f"by translation peak {trans_result['peak_value']:.4f} "
                      f"(global max was {rot_result['rotation']:.2f}°)")
        else:
            rot_peak = rot_result['peak_value']
            var_theta = rot_result['variance_theta']
            trans_result = self.estimate_translation(
                img1_cart, img2_cart, rot_result['rotation'])
            rotation = self.yaw_from_bearing(rot_result['rotation'])

        # 분산도 요각 공간으로 옮긴다(각이 1/cos τ 배면 분산은 1/cos²τ 배).
        # 이게 옳은 전파지만 팩터를 33% 약화시킨다 — 이득 보정과 잡음 팽창을
        # 갈라 보려면 rotation_tilt_var_compensation 을 끈다(진단 전용).
        if self.rotation_tilt_var_compensation:
            var_theta = var_theta / self._cos_tilt ** 2

        translation = trans_result['translation']
        trans_peak = trans_result['peak_value']

        # Step 4: Build covariance matrix (3×3 diagonal)
        covariance = np.diag([
            trans_result['variance_x'],      # var(x) in meters^2
            trans_result['variance_y'],      # var(y) in meters^2
            var_theta                        # var(theta) in radians^2, of the CHOSEN peak
        ])

        if self.verbose:
            print(f"\nFFT Registration complete:")
            print(f"  Rotation: {rotation:.2f}° (peak: {rot_peak:.4f})")
            print(f"  Translation: ({translation[0]:.2f}, {translation[1]:.2f}) m (peak: {trans_peak:.4f})")
            print(f"  Covariance diag: [{trans_result['variance_x']:.4f}, {trans_result['variance_y']:.4f}, {rot_result['variance_theta']:.6f}]")

        return {
            'rotation': rotation,
            'rotation_fft': self.yaw_from_bearing(rot_result['rotation']),
            'rotation_override_used': rotation_override is not None,
            'translation': translation,
            'covariance': covariance,
            # P1-5: 여기까지 왔다는 것 외에 어떤 품질 판정도 거치지 않은 리터럴이다.
            # `rot_peak`·`trans_peak` 는 계산만 되고 어떤 임계값과도 비교되지 않는다
            # — 임계값을 정하려면 분포가 먼저 필요해서(I8) 이번 사이클은 로깅만 하고
            # 게이트는 다음 사이클로 미뤘다. 현재 유일한 품질 방어선은 slam.py 의
            # `validate_fft_with_odom` (DR 비교) 하나뿐이다.
            'success': True,
            'rot_peak': rot_peak,
            'trans_peak': trans_peak
        }
