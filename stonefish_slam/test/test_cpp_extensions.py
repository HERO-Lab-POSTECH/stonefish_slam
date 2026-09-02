"""Characterization tests for the compiled pybind11 extensions (P0-4, P1-9, P1-18).

The .so files are gitignored build artifacts, so these tests skip unless the
extensions have been staged into the source tree:

    colcon build --merge-install --packages-select stonefish_slam
    cp build/stonefish_slam/*.so src/stonefish_slam/stonefish_slam/

These load the extension by file path rather than by package import, for the
same reason conftest.py's load_module fixture does: the package __init__ pulls
in rclpy/cv_bridge at import time.
"""
import importlib
import math
import sys

import numpy as np
import pytest

# WEIGHTED_AVG / IWLO intensity weight, mirroring OctreeMapping::initialize_lut.
# Defaults from the OctreeMapping constructor.
INTENSITY_THRESHOLD = 35.0
INTENSITY_MAX = 255.0
SHARPNESS = 3.0


def _load_extension(name):
    """Reach the extension through the package, never by file path.

    stonefish_slam/__init__.py already imports these (swallowing ImportError
    when a .so is missing), and a pybind11 module registers its types
    process-globally — loading the same .so a second time under a different
    module name aborts with 'type ... is already registered'.
    """
    try:
        return importlib.import_module(f"stonefish_slam.{name}")
    except ImportError:
        pytest.skip(f"{name} extension not staged (see module docstring)")


def _intensity_weight(intensity):
    normalized = (intensity - INTENSITY_THRESHOLD) / (INTENSITY_MAX - INTENSITY_THRESHOLD)
    return 1.0 / (1.0 + math.exp(-SHARPNESS * (normalized - 0.5)))


def _weighted_avg_tree(octree_mapping):
    tree = octree_mapping.OctreeMapping(0.1)
    tree.set_update_method(1)  # WEIGHTED_AVG
    return tree


def _observe(tree, intensity, times):
    """Insert the same endpoint `times` times, returning the probability after each."""
    points = np.array([[1.0, 1.0, 1.0]])
    intensities = np.array([intensity])
    log_odds = np.array([0.0])
    origin = np.array([0.0, 0.0, 0.0])
    probs = []
    for _ in range(times):
        tree.insert_point_cloud_with_intensity_and_logodds(
            points, intensities, log_odds, origin)
        probs.append(tree.query_cell(1.0, 1.0, 1.0))
    return probs


# ---------------------------------------------------------------------------
# P0-4: WEIGHTED_AVERAGE passed an ABSOLUTE log-odds to the additive
# updateNode(key, float, bool) API.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("intensity", [255.0, 40.0])
def test_weighted_average_holds_its_target_probability(intensity):
    """A repeated identical observation must HOLD at w(I), not walk away from it.

    WEIGHTED_AVERAGE's own formula is a running mean of w(I): starting from the
    unknown prior 0.5, the first observation lands exactly on w and every later
    identical observation reproduces it. Measured before the fix (2026-09-01),
    the absolute log-odds accumulated instead: I=255 went
    0.8176 -> 0.9526 -> 0.97 (upper clamp) and I=40 went
    0.1928 -> 0.054 -> 0.03 (lower clamp).
    """
    octree_mapping = _load_extension("octree_mapping")
    tree = _weighted_avg_tree(octree_mapping)

    probs = _observe(tree, intensity, times=10)
    expected = _intensity_weight(intensity)

    for i, p in enumerate(probs):
        assert p == pytest.approx(expected, abs=1e-3), (
            f"observation {i} drifted to {p:.4f}, target {expected:.4f}"
        )


def test_weighted_average_does_not_diverge_to_the_clamping_bounds():
    """Divergence, not saturation, is the failure mode — pin both directions.

    The constructor clamps probability to [0.03, 0.97]; a monotone-convergence
    check alone would not catch alternating intensities, so this asserts the
    high and low cases stay strictly inside their respective bounds.
    """
    octree_mapping = _load_extension("octree_mapping")

    high = _observe(_weighted_avg_tree(octree_mapping), 255.0, times=20)[-1]
    low = _observe(_weighted_avg_tree(octree_mapping), 40.0, times=20)[-1]

    assert high < 0.95, f"high intensity reached {high:.4f}, near the 0.97 clamp"
    assert low > 0.05, f"low intensity reached {low:.4f}, near the 0.03 clamp"


def test_weighted_average_tracks_a_changed_intensity():
    """The running mean must still move when the observation changes — the delta
    conversion must not freeze the voxel at its first value."""
    octree_mapping = _load_extension("octree_mapping")
    tree = _weighted_avg_tree(octree_mapping)

    _observe(tree, 40.0, times=1)
    after_low = tree.query_cell(1.0, 1.0, 1.0)
    _observe(tree, 255.0, times=1)
    after_high = tree.query_cell(1.0, 1.0, 1.0)

    assert after_high > after_low


# ---------------------------------------------------------------------------
# P1-9: set_adaptive_params was the one setter without input validation, and
# adaptive_threshold_ is a divisor in six places.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("threshold", [0.0, -0.1, 1.5])
def test_set_adaptive_params_rejects_an_out_of_range_threshold(threshold):
    octree_mapping = _load_extension("octree_mapping")
    tree = octree_mapping.OctreeMapping(0.1)

    with pytest.raises(ValueError):
        tree.set_adaptive_params(enable=True, threshold=threshold, max_ratio=0.3)


def test_set_adaptive_params_rejects_a_negative_max_ratio():
    octree_mapping = _load_extension("octree_mapping")
    tree = octree_mapping.OctreeMapping(0.1)

    with pytest.raises(ValueError):
        tree.set_adaptive_params(enable=True, threshold=0.5, max_ratio=-1.0)


def test_set_adaptive_params_accepts_the_shipped_values():
    """config/mapping/method_iwlo.yaml and method_log_odds.yaml both ship
    threshold=0.5, max_ratio=0.3 — the guard must not reject production."""
    octree_mapping = _load_extension("octree_mapping")
    tree = octree_mapping.OctreeMapping(0.1)

    tree.set_adaptive_params(enable=True, threshold=0.5, max_ratio=0.3)


# ---------------------------------------------------------------------------
# P1-18: RayProcessor holds OctreeMapping as a non-owning raw pointer.
# ---------------------------------------------------------------------------

def test_ray_processor_keeps_its_octree_alive():
    """Without py::keep_alive<1, 2> the Python octree can be collected while the
    processor still points at it, making process_sonar_image a use-after-free.

    Refcount is the deterministic observable: keep_alive makes the RayProcessor
    hold a reference, so constructing one must raise the octree's refcount.
    """
    octree_mapping = _load_extension("octree_mapping")
    ray_processor = _load_extension("ray_processor")

    tree = octree_mapping.OctreeMapping(0.1)
    config = ray_processor.RayProcessorConfig()

    before = sys.getrefcount(tree)
    processor = ray_processor.RayProcessor(octree=tree, config=config)
    after = sys.getrefcount(tree)

    assert after > before, (
        "RayProcessor took no reference to the octree; py::keep_alive<1, 2> is missing"
    )
    del processor


# ---------------------------------------------------------------------------
# Semantic labelling: the two places the Python side has to agree with C++.
# ---------------------------------------------------------------------------

def test_downsample_preserves_an_integer_descriptor_pair():
    """`/slam/cloud` 의 keyframe index 와 semantic label 이 정수로 살아남는가.

    라벨은 이 descriptor 채널을 타고 나간다. `downsample` 이 대표점을 평균으로
    잡으면 라벨 1 과 2 가 섞여 1.5 가 되고, uint8 로 잘리면서 조용히 다른
    클래스가 된다. 실제로는 medoid(입력 점 하나)를 고르므로 정수가 정수로
    남는다 — 그 성질에 기대고 있으니 여기서 못 박는다.

    `test_pcl.py` 는 순수 파이썬 fallback(`cpp/pcl.py`, centroid + descriptor
    평균)을 path-load 하므로 이 질문의 증거가 되지 못한다.
    """
    pcl = _load_extension("pcl")

    points = np.array([[0.20, 0.15], [0.21, 0.16], [0.22, 0.14], [5.0, 5.0]])
    descriptors = np.array([[0.0, 3.0], [1.0, 2.0], [1.0, 2.0], [7.0, 9.0]])

    sampled_points, sampled_desc = pcl.downsample(points, descriptors, 0.5)

    assert len(sampled_points) == 2, "한 복셀의 점 3개가 하나로 줄어야 한다"
    # 대표 descriptor 는 입력 중 하나와 정확히 같아야 한다(평균이 아니다).
    for row in sampled_desc:
        assert any(np.array_equal(row, d) for d in descriptors), (
            f"descriptor {row} 가 입력에 없다 — 평균이 섞였다"
        )
    assert np.all(sampled_desc == np.round(sampled_desc)), "정수 라벨이 깨졌다"


def test_ray_processor_voxels_project_back_to_the_pixel_that_made_them(load_module):
    """C++ 이 놓은 복셀을 파이썬 역투영이 같은 픽셀로 되돌리는가.

    3D 라벨은 이 왕복 위에 서 있다. C++ 이 `range_max − r_idx·range_resolution`
    (경사거리)로 복셀을 놓으므로 역산도 **경사거리**여야 한다 —
    `mapping_3d._voxel_to_sonar_coords` 의 수평거리로는 앙각이 0 이 아닌 복셀에서
    range bin 이 어긋난다.

    오차는 복셀 격자화 몫이므로 허용치는 복셀 반대각선(라벨 경로가 쓰는 pad)이다.
    """
    octree_mapping = _load_extension("octree_mapping")
    ray_processor = _load_extension("ray_processor")
    sem = load_module("stonefish_slam/core/semantic.py", "semantic_for_cpp_roundtrip")

    num_beams, num_bins = 64, 50
    fov_deg, range_min, range_max = 120.0, 0.5, 10.0
    voxel = 0.1
    range_resolution = (range_max - range_min) / num_bins

    tree = octree_mapping.OctreeMapping(voxel)
    config = ray_processor.RayProcessorConfig()
    config.horizontal_fov = fov_deg            # degrees (C++ converts)
    config.vertical_fov = math.radians(20.0)   # radians (C++ does NOT convert)
    config.range_min, config.range_max = range_min, range_max
    config.range_resolution = range_resolution
    config.voxel_resolution = voxel
    config.bearing_step = 1
    config.intensity_threshold = 35
    config.log_odds_occupied = 3.0
    processor = ray_processor.RayProcessor(octree=tree, config=config)

    r_idx, b_idx = 20, 32
    polar = np.zeros((num_bins, num_beams), np.uint8)
    polar[r_idx, b_idx] = 255
    processor.process_sonar_image(polar, np.eye(4))   # sonar frame == world frame

    cells = tree.get_occupied_cells(threshold=0.7)
    assert len(cells) > 0, "밝은 픽셀 하나가 점유 복셀을 하나도 안 만들었다"

    slant, bearing = sem.slant_range_bearing(cells[:, :3])
    row, col = sem.sonar_to_pixel(
        slant, bearing, num_bins=num_bins, num_beams=num_beams,
        range_min=range_min, range_max=range_max, horizontal_fov_deg=fov_deg)

    pad_m = voxel * math.sqrt(3.0) / 2.0
    pad_row = pad_m / range_resolution
    pad_col = math.degrees(pad_m / slant.min()) / fov_deg * (num_beams - 1)

    assert np.all(np.abs(row - r_idx) <= pad_row), (
        f"range bin 이 격자 오차를 넘어 어긋났다: {row}"
    )
    assert np.all(np.abs(col - b_idx) <= pad_col), (
        f"bearing bin 이 격자 오차를 넘어 어긋났다: {col}"
    )
