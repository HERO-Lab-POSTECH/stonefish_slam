import numpy as np
import pytest

pytest.importorskip("sklearn")  # pcl.py module-top import; 없는 머신은 모듈 전체 skip

REL = "stonefish_slam/cpp/pcl.py"


def _pcl(load_module):
    return load_module(REL, "pcl_under_test")


def _square():
    return np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0],
                     [0.5, 0.5], [0.2, 0.8], [0.8, 0.2]], dtype=float)


def test_icp_recovers_known_translation(load_module):
    # P4 T-A5: was xfail (strict). Root cause of the miss was NOT float32 but
    # the fixed outlier_ratio=0.8 trimming 20% of a perfect-overlap cloud
    # (asymmetric drop biases the centroid) plus a convergence test on the
    # pre-update residual. After the fix, perfect-overlap recovers the exact
    # translation.
    m = _pcl(load_module)
    src = _square()
    shift = np.array([0.3, -0.2])
    status, T = m.ICP().compute(src, src + shift, np.eye(3))
    assert status == "success"
    np.testing.assert_allclose(T[:2, 2], shift, atol=0.05)


def test_icp_perfect_overlap_recovers_exactly(load_module):
    # Perfect-overlap (overlap ratio = 1.0): with correct trimming the
    # translation must be recovered to near machine precision, not merely
    # within 0.05. Pins that no inliers are spuriously discarded.
    m = _pcl(load_module)
    src = _square()
    shift = np.array([0.3, -0.2])
    status, T = m.ICP().compute(src, src + shift, np.eye(3))
    assert status == "success"
    np.testing.assert_allclose(T[:2, 2], shift, atol=1e-6)


def test_icp_recovers_small_rotation(load_module):
    m = _pcl(load_module)
    src = _square()
    th = np.deg2rad(10.0)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    status, T = m.ICP().compute(src, src @ R.T, np.eye(3))
    assert status == "success"
    recovered = np.arctan2(T[1, 0], T[0, 0])
    assert np.isclose(recovered, th, atol=np.deg2rad(2.0))


def test_match_returns_zero_distance_for_identical_clouds(load_module):
    m = _pcl(load_module)
    pts = _square()
    idx, dist = m.match(pts, pts, knn=1)
    np.testing.assert_allclose(np.asarray(dist).ravel(), 0.0, atol=1e-9)
