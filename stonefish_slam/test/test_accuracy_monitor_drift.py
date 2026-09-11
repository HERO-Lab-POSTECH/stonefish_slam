"""The drift metric's numerator and denominator must span the same samples.

est_buf/gt_buf are rolling deques (maxlen = ate_window_size, default 500) and
are re-aligned with umeyama_se2 on every call, so the RMSE they produce is a
WINDOWED quantity. It used to be divided by self.total_distance, which
accumulates for the whole mission and is never reset. The longer the vehicle
runs, the smaller that ratio gets — regardless of how badly the estimate
drifts — so the node built for sea-trial evaluation reported drift trending to
0% and accuracy trending to 100% exactly when it should have reported the
opposite.
"""
import sys
import types
from collections import deque

import numpy as np
import pytest

REL = "stonefish_slam/core/slam_accuracy_monitor.py"

# slam_accuracy_monitor imports rclpy/tf2_ros/ROS msgs at module top; the metric
# under test uses none of them.
_ROS_STUBS = {
    "rclpy": {},
    "rclpy.node": {"Node": object},
    "rclpy.qos": {"QoSProfile": object, "HistoryPolicy": object,
                  "ReliabilityPolicy": object},
    "rclpy.duration": {"Duration": object},
    "sensor_msgs": {},
    "sensor_msgs.msg": {"PointCloud2": object},
    "sensor_msgs_py": {},
    "sensor_msgs_py.point_cloud2": {},
    "nav_msgs": {},
    "nav_msgs.msg": {"Path": object},
    "tf2_ros": {},
}


@pytest.fixture
def monitor_module(load_module):
    preexisting = set(sys.modules)
    for name, attrs in _ROS_STUBS.items():
        if name not in sys.modules:
            mod = types.ModuleType(name)
            for k, v in attrs.items():
                setattr(mod, k, v)
            sys.modules[name] = mod
    sys.modules["rclpy"].node = sys.modules["rclpy.node"]
    sys.modules["rclpy"].qos = sys.modules["rclpy.qos"]
    sys.modules["rclpy"].duration = sys.modules["rclpy.duration"]
    sys.modules["sensor_msgs_py"].point_cloud2 = sys.modules["sensor_msgs_py.point_cloud2"]
    try:
        yield load_module(REL, "slam_accuracy_monitor_under_test")
    finally:
        for name in set(sys.modules) - preexisting:
            del sys.modules[name]


def _monitor(module, est, gt, total_distance):
    """(rmse, drift, acc) from an instance carrying only what the metric reads."""
    m = module.TrajPathGtAteMonitor.__new__(module.TrajPathGtAteMonitor)
    m.est_buf = deque(est)
    m.gt_buf = deque(gt)
    m.ate_min_samples = 20
    m.ate_use_se2_align = True
    m.distance_epsilon = 1e-6
    m.total_distance = total_distance
    return m._compute_ate_and_accuracy()


def _straight_leg(n, step=1.0):
    """n GT points along +x, one metre apart."""
    return [(i * step, 0.0) for i in range(n)]


def _with_drift(gt, amplitude):
    """Estimate that bends away from GT, growing quadratically along the path.

    A CONSTANT lateral offset is useless here: umeyama_se2 is a rigid transform
    and absorbs it exactly, leaving rmse == 0. Real drift accumulates, so the
    error must be a bend the alignment cannot remove.
    """
    n = max(len(gt) - 1, 1)
    return [(x, y + amplitude * (i / n) ** 2) for i, (x, y) in enumerate(gt)]


def test_drift_does_not_shrink_as_the_mission_gets_longer(monitor_module):
    """Same window, same error — a longer past must not improve the score.

    This is the defect: total_distance grows without bound while the window does
    not, so the reported drift fell purely with elapsed distance.
    """
    gt = _straight_leg(50)
    est = _with_drift(gt, 2.0)

    _, drift_early, acc_early = _monitor(monitor_module, est, gt, total_distance=49.0)
    _, drift_late, acc_late = _monitor(monitor_module, est, gt, total_distance=5000.0)

    assert drift_early == pytest.approx(drift_late), (
        f"drift changed with mission length alone: {drift_early:.3f}% vs "
        f"{drift_late:.3f}% — the denominator is not the window"
    )
    assert acc_early == pytest.approx(acc_late)


def test_drift_is_rmse_over_the_window_path_length(monitor_module):
    """Pin the definition: 100 * RMSE / (GT path length within the window)."""
    gt = _straight_leg(50)          # 49 m of GT path inside the window
    est = _with_drift(gt, 2.0)

    rmse, drift, acc = _monitor(monitor_module, est, gt, total_distance=5000.0)

    # Assert against the measured rmse rather than assuming what survives the
    # SE(2) alignment.
    assert drift == pytest.approx(100.0 * rmse / 49.0, rel=1e-6)
    assert acc == pytest.approx(max(0.0, 100.0 - drift))


def test_larger_error_reports_larger_drift(monitor_module):
    """Sanity: the metric must still respond to the thing it measures."""
    gt = _straight_leg(50)

    _, small, _ = _monitor(monitor_module, _with_drift(gt, 0.5), gt, 100.0)
    _, large, _ = _monitor(monitor_module, _with_drift(gt, 5.0), gt, 100.0)

    assert large > small


def test_station_keeping_reports_nan_not_zero_accuracy(monitor_module):
    """A window with no GT motion has no drift RATE to report.

    This is the hazard the windowed denominator introduces and the cumulative
    one hid: hovering drives the GT path length to zero, so any nonzero RMSE
    would divide out to a huge drift and clamp to 0% accuracy — a confident
    wrong answer about a vehicle that is simply holding position.
    """
    stationary_gt = [(0.0, 0.0)] * 50
    est = [(0.0, 0.05 * (i % 2)) for i in range(50)]  # jitter, no net motion

    rmse, drift, acc = _monitor(monitor_module, est, stationary_gt, total_distance=500.0)

    assert not np.isnan(rmse), "RMSE is still well defined while hovering"
    assert np.isnan(drift) and np.isnan(acc), (
        f"station keeping reported drift={drift}, acc={acc} instead of NaN"
    )


def test_too_few_samples_stays_nan(monitor_module):
    gt = _straight_leg(5)
    rmse, drift, acc = _monitor(monitor_module, _with_drift(gt, 1.0), gt, 10.0)
    assert np.isnan(rmse) and np.isnan(drift) and np.isnan(acc)


# ---------------------------------------------------------------------------
# Estimated path length vs GT (len_ratio).
#
# Nothing else in the pipeline sees the length of the SLAM trajectory itself:
# `[INSTR] scale ratio` in slam.py compares ICP against its own seed, and
# total_distance accumulates GT. A translation estimate that comes out short
# every keyframe is therefore invisible in the logs. Worse, it is invisible in
# the DRIFT metric too — umeyama_se2 is a rigid alignment, so a uniformly
# shrunken path still rotates/translates onto GT and reports a modest RMSE.
# ---------------------------------------------------------------------------


def test_polyline_length_of_a_straight_leg_is_its_span(monitor_module):
    xy = np.asarray(_straight_leg(11, step=1.0), dtype=np.float64)
    assert monitor_module.polyline_length_2d(xy) == pytest.approx(10.0)


def test_polyline_length_is_undefined_below_two_points(monitor_module):
    """NaN, not 0.0 — a zero would read as 'the vehicle did not move'."""
    assert np.isnan(monitor_module.polyline_length_2d(np.zeros((1, 2))))
    assert np.isnan(monitor_module.polyline_length_2d(np.zeros((0, 2))))


def test_uniform_translation_compression_shows_up_as_the_length_ratio(monitor_module):
    """A 7% short step every keyframe must read as len_ratio 0.93.

    This is the quantity the tilt30 programme could not see. The estimate here
    is a straight line like GT, just scaled — exactly the failure mode that a
    rigid-alignment drift score under-reports.
    """
    gt = np.asarray(_straight_leg(101, step=1.0), dtype=np.float64)
    est = gt * 0.93

    ratio = monitor_module.polyline_length_2d(est) / monitor_module.polyline_length_2d(gt)
    assert ratio == pytest.approx(0.93, abs=1e-9)

    # And the guard this metric exists for: the drift score does NOT flag it.
    rmse, drift, _acc = _monitor(monitor_module, [tuple(p) for p in est],
                                 [tuple(p) for p in gt], total_distance=100.0)
    assert drift < 100.0 * (1.0 - ratio), (
        f"drift={drift:.2f}% already exposes a {100 * (1 - ratio):.0f}% "
        "compression — then len_ratio would be redundant"
    )
