"""NSSM global-search bounds must come from the SELECTED target's covariance.

`initialize_nonsequential_scan_matching` picks a source covariance and stores
it on the result (`ret.cov = keyframes[ret.source_key].cov`), but the shgo
search bounds were derived from a bare `cov` left over from the
field-of-view filter loop above. That loop iterates source frames newest to
oldest, so the leaked value is always the OLDEST frame's covariance rather
than the one the result actually carries — a stale, unrelated uncertainty
sizing the global search box.

The bounds are 5 sigma wide with
    sigma_translation = sqrt(max(eigvals(cov[:2, :2])))
    sigma_rotation    = sqrt(cov[2, 2])
so a test can read the intended box straight off `ret.cov` and compare.
"""
import numpy as np
import pytest

import gtsam


def _diag_cov(trans_var, rot_var):
    """A 3x3 pose covariance with the given translation/rotation variances."""
    return np.diag([trans_var, trans_var, rot_var]).astype(float)


def _expected_bounds(cov):
    """The 5-sigma search box the code documents, computed from one cov."""
    translation_std = np.sqrt(np.max(np.linalg.eigvals(cov[:2, :2])))
    rotation_std = np.sqrt(cov[2, 2])
    stds = np.array([[translation_std, translation_std, rotation_std]]).T
    return 5.0 * np.c_[-stds, stds]


class _Frame:
    """Minimal stand-in for a Keyframe: only pose and cov are read here."""

    def __init__(self, pose, cov):
        self.pose = pose
        self.cov = cov


class _FactorGraph:
    """Minimal stand-in for FactorGraph: only keyframes and current_key."""

    def __init__(self, keyframes, current_key):
        self.keyframes = keyframes
        self.current_key = current_key


@pytest.fixture
def nssm_case(load_localization, monkeypatch):
    """Build a Localization driven to the shgo call, capturing the bounds.

    Source frames span keys 5..1 (newest first), so the loop variable `cov`
    ends on key 1 — the OLDEST — while `ret.cov` is key 5's, the newest. The
    two covariances are deliberately far apart so a swap is unmissable.
    """
    module = load_localization

    newest_cov = _diag_cov(4.0, 0.25)   # key 5 -> what ret.cov must hold
    oldest_cov = _diag_cov(100.0, 9.0)  # key 1 -> the value that leaked

    keyframes = {}
    for key in range(0, 6):
        if key == 5:
            cov = newest_cov
        elif key == 1:
            cov = oldest_cov
        else:
            cov = _diag_cov(1.0, 0.01)
        keyframes[key] = _Frame(gtsam.Pose2(0.0, 0.0, 0.0), cov)

    loc = module.Localization.__new__(module.Localization)
    loc.fg = _FactorGraph(keyframes, current_key=6)
    loc.current_frame = _Frame(gtsam.Pose2(0.0, 0.0, 0.0), newest_cov)
    loc.oculus = module.OculusProperty()
    loc.oculus.range_max = 30.0
    loc.oculus.horizontal_fov = np.deg2rad(130.0)
    loc.save_data = False
    loc.point_noise = 0.5

    loc.nssm_params = module.SMParams()
    loc.nssm_params.initialization = True
    loc.nssm_params.initialization_params = (8, 1, 0.01)
    loc.nssm_params.min_points = 1
    loc.nssm_params.min_st_sep = 1
    loc.nssm_params.source_frames = 5

    # Points: a small cloud, all attributed to target key 0 so the overlap
    # selection is unambiguous and > 10 points survive the count filter.
    n_points = 40
    points = np.zeros((n_points, 2), dtype=np.float64)
    points[:, 0] = np.linspace(0.5, 2.0, n_points)
    keys = np.zeros(n_points, dtype=np.int32)

    def fake_get_points(frames, key=None, return_keys=False):
        if return_keys:
            return points.copy(), keys.copy()
        return points.copy()

    loc.get_points = fake_get_points

    def fake_subroutine(*args, **kwargs):
        return (lambda x: 0.0), []

    loc.get_matching_cost_subroutine1 = fake_subroutine

    captured = {}

    class _Result:
        success = True
        x = np.zeros(3)
        fun = 0.0
        message = "ok"

    def fake_shgo(func, bounds, **kwargs):
        captured["bounds"] = np.asarray(bounds, dtype=float)
        return _Result()

    monkeypatch.setattr(module, "shgo", fake_shgo)

    ret = loc.initialize_nonsequential_scan_matching()
    return ret, captured, newest_cov, oldest_cov


def test_bounds_follow_the_results_covariance(nssm_case):
    """shgo bounds must be derived from ret.cov, not the leaked loop value."""
    ret, captured, _newest, _oldest = nssm_case
    assert "bounds" in captured, "shgo was never reached; test setup is wrong"
    np.testing.assert_allclose(
        captured["bounds"], _expected_bounds(ret.cov), rtol=1e-12, atol=1e-12
    )


def test_bounds_are_not_the_oldest_source_frames_covariance(nssm_case):
    """Guard the specific regression: the oldest frame's cov must not size it."""
    _ret, captured, _newest, oldest_cov = nssm_case
    stale = _expected_bounds(oldest_cov)
    assert not np.allclose(captured["bounds"], stale), (
        "search bounds still come from the field-of-view loop's leaked cov"
    )


def test_result_covariance_is_the_source_keys(nssm_case):
    """ret.cov itself is unchanged: the newest source frame's covariance."""
    ret, _captured, newest_cov, _oldest = nssm_case
    np.testing.assert_allclose(ret.cov, newest_cov)
