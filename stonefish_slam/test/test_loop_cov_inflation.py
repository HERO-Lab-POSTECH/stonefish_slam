"""slam_loop_along_sigma_scale — 루프 인자 공분산의 전진축만 키우는 변환."""
import gtsam
import numpy as np


def test_inflates_only_the_along_axis(load_factor_graph):
    inflate = load_factor_graph.FactorGraph.inflate_loop_cov
    cov = np.diag([0.1, 0.2, 0.01])

    # 전진 방향이 +x 인 인자: xx 만 scale^2 배, 나머지 불변
    out = inflate(cov, gtsam.Pose2(5.0, 0.0, 0.0), 3.0)
    assert np.allclose(np.diag(out), [0.1 * 9, 0.2, 0.01])

    # 전진 방향이 +y 면 yy 가 커진다
    out = inflate(cov, gtsam.Pose2(0.0, 5.0, 0.0), 3.0)
    assert np.allclose(np.diag(out), [0.1, 0.2 * 9, 0.01])

    # scale <= 1, 병진 0, cov None 은 원본 그대로
    assert inflate(cov, gtsam.Pose2(5.0, 0.0, 0.0), 1.0) is cov
    assert inflate(cov, gtsam.Pose2(0.0, 0.0, 0.0), 3.0) is cov
    assert inflate(None, gtsam.Pose2(5.0, 0.0, 0.0), 3.0) is None


def test_stays_symmetric_positive_definite_for_a_diagonal_factor(load_factor_graph):
    cov = np.array([[0.1, 0.02, 0.0], [0.02, 0.2, 0.0], [0.0, 0.0, 0.01]])
    out = load_factor_graph.FactorGraph.inflate_loop_cov(
        cov, gtsam.Pose2(3.0, 4.0, 0.0), 2.5)
    assert np.allclose(out, out.T)
    assert np.all(np.linalg.eigvalsh(out) > 0)
