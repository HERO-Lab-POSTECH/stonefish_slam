"""Tests for FactorGraph robust noise model wiring (T-C2).

Tests verify:
1. create_robust_full_noise_model uses c=3.0 (not 1.0)
2. add_icp_factor accepts robust=True flag and applies robust model
3. add_loop_closure calls add_icp_factor with robust=True
"""
from types import SimpleNamespace

import numpy as np
import gtsam


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_fg(load_factor_graph):
    """Instantiate FactorGraph with minimal noise models set."""
    fg_mod = load_factor_graph
    FactorGraph = fg_mod.FactorGraph

    fg = FactorGraph()

    # Minimal noise models so add_icp_factor doesn't crash on icp_odom_model
    diag = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.01]))
    fg.set_noise_models(
        prior_model=diag,
        odom_model=diag,
        icp_odom_model=diag,
    )
    return fg


def _identity_cov():
    """3x3 identity covariance."""
    return np.eye(3) * 0.01


# ---------------------------------------------------------------------------
# Test 1: c=3.0 and Robust type
# ---------------------------------------------------------------------------

def test_robust_noise_model_uses_configured_c(load_factor_graph):
    """create_robust_full_noise_model must use c=3.0 and return Robust model."""
    fg = _make_fg(load_factor_graph)
    cov = _identity_cov()

    model = fg.create_robust_full_noise_model(cov)

    # Must be Robust type
    assert isinstance(model, gtsam.noiseModel.Robust), (
        f"Expected gtsam.noiseModel.Robust, got {type(model)}"
    )

    # fg must expose robust_loop_c attribute = 3.0
    assert hasattr(fg, "robust_loop_c"), "FactorGraph missing robust_loop_c attribute"
    assert fg.robust_loop_c == 3.0, (
        f"Expected robust_loop_c=3.0, got {fg.robust_loop_c}"
    )

    # Verify Cauchy weight values at c=3.0:
    #   weight(1.0) ≈ 0.9,  weight(3.0) ≈ 0.5
    cauchy = gtsam.noiseModel.mEstimator.Cauchy.Create(3.0)
    assert abs(cauchy.weight(1.0) - 0.9) < 1e-3, (
        f"Cauchy(c=3.0).weight(1.0) expected ~0.9, got {cauchy.weight(1.0)}"
    )
    assert abs(cauchy.weight(3.0) - 0.5) < 1e-3, (
        f"Cauchy(c=3.0).weight(3.0) expected ~0.5, got {cauchy.weight(3.0)}"
    )


# ---------------------------------------------------------------------------
# Test 2: add_icp_factor robust=True flag
# ---------------------------------------------------------------------------

def test_add_icp_factor_robust_flag_returns_robust_model(load_factor_graph):
    """add_icp_factor must accept robust=True and add a factor with Robust noise model."""
    fg = _make_fg(load_factor_graph)
    cov = _identity_cov()
    transform = gtsam.Pose2(0.0, 0.0, 0.0)

    # robust kwarg must be accepted (TypeError if not)
    fg.add_icp_factor(
        source_key=0,
        target_key=1,
        transform=transform,
        cov=cov,
        robust=True,
    )

    # Inspect the last added factor's noise model
    factor = fg.graph.at(0)
    noise_model = factor.noiseModel()
    assert isinstance(noise_model, gtsam.noiseModel.Robust), (
        f"Expected Robust noise model when robust=True, got {type(noise_model)}"
    )


def test_add_icp_factor_default_is_nonrobust(load_factor_graph):
    """The default (no robust kwarg) MUST stay non-robust — this is the SSM /
    odometry invariant: only loop closures opt into the robust kernel. Pins the
    default so a future signature change can't silently make SSM factors robust."""
    fg = _make_fg(load_factor_graph)
    cov = _identity_cov()

    # No robust kwarg — exercises the production SSM call shape (positional cov).
    fg.add_icp_factor(0, 1, gtsam.Pose2(0.0, 0.0, 0.0), cov)

    noise_model = fg.graph.at(fg.graph.size() - 1).noiseModel()
    assert not isinstance(noise_model, gtsam.noiseModel.Robust), (
        f"Default add_icp_factor must be non-robust, got {type(noise_model)}"
    )


# ---------------------------------------------------------------------------
# Test 3: add_loop_closure calls add_icp_factor with robust=True
# ---------------------------------------------------------------------------

def test_loop_closure_path_uses_robust(load_factor_graph, monkeypatch):
    """add_loop_closure must call add_icp_factor with robust=True."""
    fg = _make_fg(load_factor_graph)

    captured = {}

    original_add_icp = fg.add_icp_factor

    def mock_add_icp(source_key, target_key, transform, cov=None, robust=False):
        captured["robust"] = robust
        original_add_icp(source_key, target_key, transform, cov=cov, robust=robust)

    monkeypatch.setattr(fg, "add_icp_factor", mock_add_icp)

    # Build a minimal ICPResult-like object that PCM will pass through.
    # We bypass PCM by monkeypatching verify_pcm to always return [0].
    class FakeICPResult:
        source_key = 0
        target_key = 1
        source_pose = gtsam.Pose2(0.0, 0.0, 0.0)
        target_pose = gtsam.Pose2(1.0, 0.0, 0.0)
        estimated_transform = gtsam.Pose2(1.0, 0.0, 0.0)
        cov = _identity_cov()
        inserted = False

    fake = FakeICPResult()
    fg.nssm_queue = [fake]

    # Force PCM to accept the candidate
    monkeypatch.setattr(fg, "verify_pcm", lambda queue, min_pcm: [0])

    # Add a keyframe so keyframes[0] and keyframes[1] exist
    fake_kf = type("KF", (), {"constraints": [], "pose": gtsam.Pose2()})()
    fg.keyframes = [fake_kf, fake_kf]

    # Call add_loop_closure — it will internally call add_icp_factor
    fg.add_loop_closure(fake)

    assert "robust" in captured, "add_icp_factor was never called from add_loop_closure"
    assert captured["robust"] is True, (
        f"add_loop_closure must call add_icp_factor(robust=True), got robust={captured['robust']}"
    )


# ---------------------------------------------------------------------------
# Test 4 (P0-3): ISAM2 failure must not kill the node, and must not poison the
# pending factor queue.
# ---------------------------------------------------------------------------

def _keyframe(fg_mod, sec, x):
    """Minimal real Keyframe at (x, 0, 0). time only needs sec/nanosec."""
    return fg_mod.Keyframe(
        status=True,
        time=SimpleNamespace(sec=sec, nanosec=0),
        dr_pose3=gtsam.Pose3(gtsam.Rot3(), np.array([float(x), 0.0, 0.0])),
    )


class _FailOnceISAM:
    """Delegates to the real ISAM2 but raises on the first update() call.

    Mirrors what GTSAM does on a degenerate linearization: the pybind layer
    surfaces IndeterminantLinearSystemException as RuntimeError.
    """

    def __init__(self, inner):
        self._inner = inner
        self.raised = False

    def update(self, graph, values):
        if not self.raised:
            self.raised = True
            raise RuntimeError("Indeterminant linear system detected (injected)")
        return self._inner.update(graph, values)

    def __getattr__(self, name):
        return getattr(self._inner, name)


def test_isam_failure_is_survivable_and_leaves_no_poisoned_queue(load_factor_graph):
    """P0-3: a failed ISAM2 update must clear the pending graph/values.

    A bare try/except would be worse than the crash: the offending factors stay
    queued and every later keyframe re-pushes them, so ISAM2 fails for the rest
    of the node's life. This pins both halves — no exception escapes, AND the
    next update actually succeeds.
    """
    fg = _make_fg(load_factor_graph)

    # Production order (slam.py:841-846): factors first, then update_graph(frame)
    # appends the keyframe.
    kf0 = _keyframe(load_factor_graph, 0, 0.0)
    fg.add_prior_factor(kf0)

    fg.isam = _FailOnceISAM(fg.isam)

    # Must not raise.
    fg.update_graph(kf0)

    assert fg.isam.raised, "the injected failure never fired"
    assert fg.graph.size() == 0, "pending factors survived the failed update"
    assert fg.values.size() == 0, "pending values survived the failed update"

    # The next tick must work. Re-seed the prior the failed update dropped.
    fg.add_prior_factor(kf0)
    fg.update_graph()

    assert fg.keyframes[-1].cov is not None, "recovered update left no covariance"
    assert fg.graph.size() == 0 and fg.values.size() == 0


# ---------------------------------------------------------------------------
# Test 5 (P1-3): verify_pcm must not crash on a candidate without covariance
# ---------------------------------------------------------------------------

def _candidate(source_key, target_key, cov):
    """A mutually consistent PCM candidate: the transform agrees with the poses,
    so md == 0 and the whole set forms one clique when cov is present."""
    source_pose = gtsam.Pose2(float(source_key), 0.0, 0.0)
    target_pose = gtsam.Pose2(float(target_key), 0.0, 0.0)
    return SimpleNamespace(
        source_key=source_key,
        target_key=target_key,
        source_pose=source_pose,
        target_pose=target_pose,
        estimated_transform=target_pose.between(source_pose),
        cov=cov,
        inserted=False,
    )


def test_verify_pcm_skips_candidates_without_covariance(load_factor_graph):
    """P1-3: cov is None whenever nssm.cov_samples == 0, and np.linalg.solve
    raises LinAlgError on it. Claim no consistency instead of crashing."""
    fg = _make_fg(load_factor_graph)
    queue = [_candidate(i, 0, None) for i in range(3)]

    assert fg.verify_pcm(queue, 3) == []


def test_verify_pcm_still_matches_when_covariance_is_present(load_factor_graph):
    """The None guard must not swallow real candidates."""
    fg = _make_fg(load_factor_graph)
    queue = [_candidate(i, 0, _identity_cov()) for i in range(3)]

    assert fg.verify_pcm(queue, 3) != []


# ---------------------------------------------------------------------------
# Test 6 (P1-14): the NSSM queue must age out without a new loop closure
# ---------------------------------------------------------------------------

def test_update_graph_ages_out_stale_loop_closure_candidates(load_factor_graph):
    """P1-14: trimming used to happen only when a NEW candidate arrived, so a
    vehicle leaving a loopy area kept the queue non-empty forever — pinning
    update_graph's pose refresh to the O(N) full-history path."""
    fg = _make_fg(load_factor_graph)

    kf0 = _keyframe(load_factor_graph, 0, 0.0)
    fg.add_prior_factor(kf0)
    fg.update_graph(kf0)

    # A candidate from long ago, and no new loop closure will ever arrive.
    fg.nssm_queue = [_candidate(0, 0, _identity_cov())]

    for i in range(1, fg.pcm_queue_size + 3):
        kf = _keyframe(load_factor_graph, i, float(i))
        fg.add_odometry_factor(kf)
        fg.update_graph(kf)

    assert fg.nssm_queue == [], (
        "stale candidate never aged out; the pose refresh stays on the O(N) path"
    )


# ---------------------------------------------------------------------------
# Test 4: verify_pcm Mahalanobis gate (inv→solve numerical stability, T-C3)
# ---------------------------------------------------------------------------

def _make_consistent_pair(consistent: bool):
    """Build two FakeICPResult candidates whose composed cycle error is
    near-zero (consistent) or large (inconsistent).

    PCM composes: pjk2 = pj.between(pi * pil * plk); md = ||Logmap(pjk1.between(pjk2))||²_Σ.
    With all poses at identity and matching transforms, the cycle closes (md≈0,
    consistent). Offsetting one estimated_transform breaks consistency (md large).
    """
    import gtsam as _g

    class C:
        source_pose = _g.Pose2(0.0, 0.0, 0.0)
        target_pose = _g.Pose2(0.0, 0.0, 0.0)
        estimated_transform = _g.Pose2(0.0, 0.0, 0.0)
        cov = np.eye(3) * 0.01
        inserted = False

    a, b = C(), C()
    if not consistent:
        # Large disagreement → md far above chi2.ppf(0.99,3)=11.34.
        b.estimated_transform = _g.Pose2(10.0, 10.0, 1.0)
    return [a, b]


def test_verify_pcm_accepts_consistent_rejects_inconsistent(load_factor_graph):
    """verify_pcm gate must accept a self-consistent pair and reject a grossly
    inconsistent one — this pins the Mahalanobis gate behavior so the inv→solve
    refactor is proven behavior-preserving."""
    fg = _make_fg(load_factor_graph)

    # Consistent pair: cycle closes, md≈0 < 11.34 → both in a clique of size 2.
    consistent = _make_consistent_pair(consistent=True)
    res_c = fg.verify_pcm(consistent, min_pcm_value=2)
    assert len(res_c) == 2, f"consistent pair must form a clique of 2, got {res_c}"

    # Inconsistent pair: md ≫ 11.34 → no edge → no clique of size 2.
    inconsistent = _make_consistent_pair(consistent=False)
    res_i = fg.verify_pcm(inconsistent, min_pcm_value=2)
    assert len(res_i) < 2, f"inconsistent pair must NOT form a clique of 2, got {res_i}"


def test_verify_pcm_mahalanobis_solve_matches_inv(load_factor_graph):
    """The quadratic form error·Σ⁻¹·error must equal error·solve(Σ,error) on a
    well-conditioned covariance — the identity that makes inv→solve safe."""
    cov = np.array([[0.04, 0.001, 0.0],
                    [0.001, 0.04, 0.0],
                    [0.0, 0.0, 0.0004]])
    error = np.array([0.05, -0.03, 0.01])
    md_inv = error.dot(np.linalg.inv(cov)).dot(error)
    md_solve = error.dot(np.linalg.solve(cov, error))
    assert np.isclose(md_inv, md_solve, rtol=1e-9), (
        f"inv and solve quadratic forms must match: {md_inv} vs {md_solve}"
    )
