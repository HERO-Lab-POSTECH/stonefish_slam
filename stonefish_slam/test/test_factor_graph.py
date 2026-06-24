"""Tests for FactorGraph robust noise model wiring (T-C2).

Tests verify:
1. create_robust_full_noise_model uses c=3.0 (not 1.0)
2. add_icp_factor accepts robust=True flag and applies robust model
3. add_loop_closure calls add_icp_factor with robust=True
"""
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
