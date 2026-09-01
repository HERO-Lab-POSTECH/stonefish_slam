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
