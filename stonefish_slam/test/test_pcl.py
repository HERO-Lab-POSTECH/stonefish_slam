import logging
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("sklearn")  # pcl.py module-top import; 없는 머신은 모듈 전체 skip

REL = "stonefish_slam/cpp/pcl.py"
CONFIG_ICP_YAML = Path(__file__).resolve().parents[2] / "config" / "icp.yaml"


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


def test_loadfromyaml_applies_the_two_supported_keys(load_module, tmp_path):
    # P0-1: loadFromYaml was a print-only no-op, so operator ICP tuning was
    # silently discarded on the pure-Python path. The values here deliberately
    # differ from ICP.__init__'s defaults (3.0 / 40) — shipped config/icp.yaml
    # happens to repeat the defaults, so it cannot tell parsing from a no-op.
    cfg = tmp_path / "icp.yaml"
    cfg.write_text(
        "outlierFilters:\n"
        "  - MaxDistOutlierFilter:\n"
        "      maxDist: 7.5\n"
        "transformationCheckers:\n"
        "  - CounterTransformationChecker:\n"
        "      maxIterationCount: 12\n"
    )
    m = _pcl(load_module)
    icp = m.ICP()
    icp.loadFromYaml(str(cfg))
    assert icp.max_correspondence_distance == pytest.approx(7.5)
    assert icp.max_iterations == 12


def test_loadfromyaml_parses_the_shipped_config(load_module):
    # Structure compatibility against the REAL config/icp.yaml — the nested
    # libpointmatcher layout (list-of-single-key-mappings, bare component name
    # for errorMinimizer) must resolve to its stated values.
    #
    # NOT a discriminating test: shipped icp.yaml repeats the defaults, so this
    # also passes on a no-op loadFromYaml. The discriminator is the tmp_path
    # test above; this one pins that the nested layout does not raise.
    m = _pcl(load_module)
    icp = m.ICP()
    icp.loadFromYaml(str(CONFIG_ICP_YAML))
    # outlierFilters[MaxDistOutlierFilter].maxDist
    assert icp.max_correspondence_distance == pytest.approx(3.0)
    # transformationCheckers[CounterTransformationChecker].maxIterationCount
    assert icp.max_iterations == 40


def test_loadfromyaml_keeps_outlier_ratio_at_one(load_module):
    # TrimmedDistOutlierFilter.ratio: 0.8 is present in config/icp.yaml and must
    # NOT be mapped — a fixed trim below the true overlap biases the Kabsch
    # centroid on this path (the bug P4a fixed; see ICP.__init__).
    #
    # A regression pin, not a discriminator: it also holds on a no-op
    # loadFromYaml. Its job is to fail if someone later "completes" the mapping
    # by wiring ratio through.
    m = _pcl(load_module)
    icp = m.ICP()
    icp.loadFromYaml(str(CONFIG_ICP_YAML))
    assert icp.outlier_ratio == 1.0


def test_loadfromyaml_names_the_settings_it_ignores(load_module, caplog):
    m = _pcl(load_module)
    with caplog.at_level(logging.WARNING):
        m.ICP().loadFromYaml(str(CONFIG_ICP_YAML))
    warned = "\n".join(r.getMessage() for r in caplog.records)
    for name in ("TrimmedDistOutlierFilter", "KDTreeMatcher",
                 "DifferentialTransformationChecker"):
        assert name in warned


def test_loadfromyaml_missing_file_keeps_defaults(load_module, caplog):
    # Mirrors the C++ binding (cpp/pcl.cpp:108), which falls back to defaults
    # when the file cannot be opened.
    m = _pcl(load_module)
    icp = m.ICP()
    with caplog.at_level(logging.WARNING):
        icp.loadFromYaml("/nonexistent/icp.yaml")
    assert icp.max_correspondence_distance == 3.0
    assert icp.max_iterations == 40
    assert icp.outlier_ratio == 1.0


def test_match_returns_zero_distance_for_identical_clouds(load_module):
    m = _pcl(load_module)
    pts = _square()
    idx, dist = m.match(pts, pts, knn=1)
    np.testing.assert_allclose(np.asarray(dist).ravel(), 0.0, atol=1e-9)


def test_downsample_fallback_keeps_descriptors_integral(load_module):
    """descriptor 는 평균이 아니라 대표점의 것이어야 한다.

    이 채널로 keyframe index 와 semantic 라벨이 나간다. 두 라벨 1·2 를 평균내면
    1.5 가 되고, 소비자가 정수로 자르면 경고 없이 클래스 1 이 된다. C++ 경로
    (libpointmatcher OctreeGrid, samplingMethod=3)는 medoid 를 고르므로 정수가
    보존되는데, 이 순수 파이썬 fallback 만 다르면 `.so` 유무로 라벨이 갈린다.
    """
    pcl = load_module("stonefish_slam/cpp/pcl.py", "pcl_fallback_downsample")

    points = np.array([[0.20, 0.15], [0.21, 0.16], [0.22, 0.14], [5.0, 5.0]])
    descriptors = np.array([[0.0, 3.0], [1.0, 2.0], [1.0, 2.0], [7.0, 9.0]])

    sampled_points, sampled_desc = pcl.downsample(points, descriptors, 0.5)

    assert len(sampled_points) == 2
    for row in sampled_desc:
        assert any(np.array_equal(row, d) for d in descriptors), (
            f"descriptor {row} 가 입력에 없다 — 평균이 섞였다"
        )
