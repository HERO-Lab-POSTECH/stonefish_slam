# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""검출을 pose graph 에 넣는 경로(BearingRange 랜드마크)의 특성 테스트.

`load_factor_graph` fixture 로 실제 gtsam 을 쓴다 — 여기서 확인해야 하는 것은
"우리 코드가 gtsam 을 부른다"가 아니라 "gtsam 이 실제로 그 그래프를 받아 푼다"
이기 때문이다.

가장 중요한 회귀는 `update_graph` 의 키프레임 개수 세는 법이다. 랜드마크는 pose
와 같은 `Values` 에 들어가므로 `values.size()` 는 더 이상 키프레임 수가 아니고,
`atPose2(X(size-1))` 은 존재하지 않는 키를 묻게 된다.
"""
import numpy as np
import gtsam
import pytest


class _FlakyIsam:
    """`isam.update` 만 갈아끼우는 얇은 프록시.

    pybind 객체의 메서드는 read-only 라 monkeypatch 로 못 바꾼다. `fg.isam` 은
    평범한 파이썬 속성이므로 프록시로 통째로 갈아끼운다.
    """

    def __init__(self, real, fail_first: int):
        self._real, self._fail_left = real, fail_first

    def update(self, *args, **kwargs):
        if self._fail_left > 0:
            self._fail_left -= 1
            raise RuntimeError("Indeterminant linear system")
        return self._real.update(*args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._real, name)


def _fg(load_factor_graph):
    """노이즈 모델만 채운 FactorGraph."""
    fg = load_factor_graph.FactorGraph()
    diag = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.01]))
    fg.set_noise_models(prior_model=diag, odom_model=diag, icp_odom_model=diag)
    return fg


def _keyframe(load_factor_graph, x, y=0.0, theta=0.0):
    """dead-reckoning pose 만 가진 최소 Keyframe."""
    types = __import__("sys").modules["stonefish_slam.core.types"]
    return types.Keyframe(True, None, gtsam.Pose3(
        gtsam.Rot3.Yaw(theta), gtsam.Point3(x, y, 0.0)))


def _two_keyframes_with_a_landmark(load_factor_graph):
    """키프레임 2개 + 랜드마크 1개를 그래프에 밀어 넣는다."""
    fg = _fg(load_factor_graph)

    kf0 = _keyframe(load_factor_graph, 0.0)
    fg.add_prior_factor(kf0)
    fg.add_landmark_factor(0, cls=1, bearing=0.2, rng=5.0,
                           world_guess=np.array([4.9, 1.0]))
    fg.update_graph(kf0)

    kf1 = _keyframe(load_factor_graph, 1.0)
    fg.add_odometry_factor(kf1)
    fg.add_landmark_factor(1, cls=1, bearing=0.1, rng=4.2,
                           world_guess=np.array([5.1, 1.1]))
    fg.update_graph(kf1)
    return fg


# ------------------------------------------------------------- noise model


def test_landmark_noise_model_is_robust_and_independent_of_the_loop_kernel(
        load_factor_graph):
    """loop closure 용 c 와 semantic 용 c 는 다른 실패율을 표현한다."""
    fg = _fg(load_factor_graph)
    fg.robust_loop_c = 1.0
    fg.robust_landmark_c = 5.0

    model = fg.create_landmark_noise_model()

    assert isinstance(model, gtsam.noiseModel.Robust)
    # c=5.0 의 Cauchy 는 c=1.0 보다 3σ 에서 훨씬 덜 깎는다.
    assert (gtsam.noiseModel.mEstimator.Cauchy.Create(5.0).weight(3.0)
            > gtsam.noiseModel.mEstimator.Cauchy.Create(1.0).weight(3.0))


def test_landmark_sigmas_are_bearing_then_range(load_factor_graph):
    """`BearingRange2D` 의 측정 순서가 (bearing, range) 라 시그마도 그 순서다.

    순서가 뒤집히면 10° 오차와 1 m 오차의 가중이 서로 바뀐다 — 값은 그대로라
    로그로는 안 보이고, 궤적이 미묘하게 끌릴 뿐이다. 그래서 **같은 크기의
    1σ 오차 두 개가 같은 error 를 내는지**로 확인한다.
    """
    X, L = load_factor_graph.X, load_factor_graph.L
    fg = _fg(load_factor_graph)
    fg.landmark_sigmas = np.array([0.1, 10.0])   # 1σ = 0.1 rad / 10 m
    noise = fg.create_landmark_noise_model()

    values = gtsam.Values()
    values.insert(X(0), gtsam.Pose2(0.0, 0.0, 0.0))
    values.insert(L(0), np.array([5.0, 0.0]))

    bearing_1sigma = gtsam.BearingRangeFactor2D(
        X(0), L(0), gtsam.Rot2(0.1), 5.0, noise)     # 0.1 rad 어긋남
    range_1sigma = gtsam.BearingRangeFactor2D(
        X(0), L(0), gtsam.Rot2(0.0), 15.0, noise)    # 10 m 어긋남

    assert bearing_1sigma.error(values) == pytest.approx(range_1sigma.error(values))
    assert bearing_1sigma.error(values) > 0


# ------------------------------------------------------- graph with a landmark


def test_isam2_solves_a_graph_that_has_both_poses_and_a_landmark(load_factor_graph):
    fg = _two_keyframes_with_a_landmark(load_factor_graph)

    values = fg.isam.calculateEstimate()
    assert values.size() == 3, "pose 2 + landmark 1"
    assert len(fg.keyframes) == 2
    assert len(fg.landmarks) == 1


def test_keyframe_count_is_not_values_size(load_factor_graph):
    """이 테스트가 바로 그 회귀다.

    `update_graph` 가 `values.size()` 로 키프레임을 세면 랜드마크가 하나만 들어와도
    `atPose2(X(2))` 를 묻게 되고 gtsam 이 RuntimeError 를 던진다 — 즉 검출이
    한 번이라도 쓰이는 순간 노드가 죽는다.
    """
    fg = _two_keyframes_with_a_landmark(load_factor_graph)

    values = fg.isam.calculateEstimate()
    assert values.size() > len(fg.keyframes), "테스트 전제(랜드마크가 있다)가 깨졌다"
    with pytest.raises(RuntimeError):
        values.atPose2(load_factor_graph.X(values.size() - 1))

    # 그럼에도 두 키프레임 모두 최적화된 pose 를 받았어야 한다.
    for kf in fg.keyframes:
        assert kf.pose is not None
    assert fg.keyframes[-1].cov is not None, "marginal 이 죽지 않고 실려야 한다"


def test_same_class_within_the_radius_reuses_one_landmark(load_factor_graph):
    fg = _two_keyframes_with_a_landmark(load_factor_graph)
    assert len(fg.landmarks) == 1
    assert fg.landmarks[0]['n_obs'] == 2


def test_same_class_outside_the_radius_makes_a_second_landmark(load_factor_graph):
    fg = _fg(load_factor_graph)
    kf0 = _keyframe(load_factor_graph, 0.0)
    fg.add_prior_factor(kf0)
    fg.landmark_assoc_radius = 3.0
    j0, new0 = fg.add_landmark_factor(0, 1, 0.2, 5.0, np.array([5.0, 0.0]))
    j1, new1 = fg.add_landmark_factor(0, 1, -0.9, 12.0, np.array([-5.0, 0.0]))
    assert (new0, new1) == (True, True)
    assert j0 != j1


def test_a_different_class_never_associates(load_factor_graph):
    fg = _fg(load_factor_graph)
    kf0 = _keyframe(load_factor_graph, 0.0)
    fg.add_prior_factor(kf0)
    j0, _ = fg.add_landmark_factor(0, 1, 0.2, 5.0, np.array([5.0, 0.0]))
    j1, new1 = fg.add_landmark_factor(0, 2, 0.2, 5.0, np.array([5.0, 0.0]))
    assert new1 is True and j0 != j1


def test_two_detections_in_one_tick_insert_the_variable_only_once(load_factor_graph):
    """같은 새 랜드마크에 두 번 연관되면 `values.insert` 가 중복돼 죽는다."""
    fg = _fg(load_factor_graph)
    kf0 = _keyframe(load_factor_graph, 0.0)
    fg.add_prior_factor(kf0)
    j0, _ = fg.add_landmark_factor(0, 1, 0.2, 5.0, np.array([5.0, 0.0]))
    j1, new1 = fg.add_landmark_factor(0, 1, 0.21, 5.05, np.array([5.05, 0.0]))
    assert (j0, new1) == (j0, False)
    fg.update_graph(kf0)          # 여기서 죽지 않아야 한다
    assert len(fg.landmarks) == 1


# ------------------------------------------------------------------ atomicity


def test_a_failed_update_does_not_commit_the_landmark(load_factor_graph):
    """ISAM2 가 거부한 tick 의 랜드마크 id 가 남으면 다음 관측이 유령에 붙는다."""
    fg = _fg(load_factor_graph)
    fg.isam = _FlakyIsam(fg.isam, fail_first=1)
    kf0 = _keyframe(load_factor_graph, 0.0)
    fg.add_prior_factor(kf0)
    fg.add_landmark_factor(0, 1, 0.2, 5.0, np.array([5.0, 0.0]))

    fg.update_graph(kf0)          # 예외가 밖으로 새지 않는다

    assert fg.landmarks == {}, "실패한 update 의 랜드마크가 commit 됐다"
    assert fg.pending_landmarks == {}


def test_the_next_observation_recovers_after_a_failed_update(load_factor_graph):
    """실패 뒤 같은 클래스가 다시 보이면 'key already exists' 없이 복구돼야 한다."""
    fg = _fg(load_factor_graph)
    fg.isam = _FlakyIsam(fg.isam, fail_first=1)
    kf0 = _keyframe(load_factor_graph, 0.0)
    fg.add_prior_factor(kf0)
    fg.add_landmark_factor(0, 1, 0.2, 5.0, np.array([5.0, 0.0]))

    fg.update_graph(kf0)
    assert fg.landmarks == {}

    # 두 번째 관측: prior 를 다시 넣고(첫 tick 의 값이 버려졌으므로) 랜드마크도 다시.
    fg.keyframes.clear()
    kf1 = _keyframe(load_factor_graph, 0.0)
    fg.add_prior_factor(kf1)
    fg.add_landmark_factor(0, 1, 0.2, 5.0, np.array([5.0, 0.0]))
    fg.update_graph(kf1)

    assert len(fg.landmarks) == 1
    # id 는 재사용하지 않는다 — 버려진 tick 의 id 를 되쓰면 부분적으로 남은
    # 상태와 충돌할 수 있다. 그래서 0 이 아니라 '실제로 커밋된 것'을 본다.
    (committed_id,) = fg.landmarks
    assert committed_id != 0, "실패한 tick 의 id 를 그대로 되썼다"
    assert fg.isam.calculateEstimate().exists(load_factor_graph.L(committed_id))


def test_a_real_indeterminate_update_leaves_no_landmark_metadata(load_factor_graph):
    """가짜 프록시가 아니라 **실제 gtsam 실패**로 확인한다.

    `_FlakyIsam` 은 진짜 `update()` 를 부르기 전에 던지므로 GTSAM 내부 상태가
    어떻게 되는지는 증명하지 못한다. 여기서는 prior 없는 그래프(gauge 자유도)로
    실제 `Indeterminant linear system` 을 일으킨다.

    프로브 실측(2026-09-02): 실패한 update 뒤에도 변수는 estimate 에 남고, 같은
    키를 다시 `insert` 하면 "key already exists" 로 죽는다. 그러니 메타데이터를
    버리는 것만으로는 부족하고 — id 를 재사용하지 않는 것과 insert 전
    `exists()` 확인이 같이 있어야 다음 관측이 복구된다.
    """
    X, L = load_factor_graph.X, load_factor_graph.L
    fg = _fg(load_factor_graph)

    kf0 = _keyframe(load_factor_graph, 0.0)
    kf1 = _keyframe(load_factor_graph, 1.0)
    # prior 를 일부러 넣지 않는다 — 이것이 불확정의 원인이다.
    fg.keyframes.append(kf0)
    fg.values.insert(X(0), kf0.pose)
    fg.add_odometry_factor(kf1)
    fg.add_landmark_factor(0, cls=1, bearing=0.0, rng=5.0,
                           world_guess=np.array([5.0, 0.0]))

    fg.update_graph(kf1)          # 예외가 밖으로 새지 않아야 한다

    assert fg.landmarks == {} and fg.pending_landmarks == {}
    # 실측: 변수는 남는다. 그래서 id 재사용이 금지된다.
    assert fg.isam.calculateEstimate().exists(L(0))

    # 다음 관측은 새 id 를 받아 "key already exists" 없이 진행돼야 한다.
    j, is_new = fg.add_landmark_factor(0, cls=1, bearing=0.0, rng=5.0,
                                       world_guess=np.array([5.0, 0.0]))
    assert (j, is_new) == (1, True), "실패한 tick 의 id 를 되썼다"
