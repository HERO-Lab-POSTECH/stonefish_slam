"""compute_icp_with_cov must not leak a linear-algebra failure into the callback.

`MinCovDet(...).fit(...)` was guarded with `except ValueError:` alone, but
`numpy.linalg.LinAlgError` derives from Exception, not ValueError — a singular
scatter matrix therefore propagated straight out of the ROS2 subscription
callback. The function already has a failure return for exactly this case
("Failed to calculate covariance"); the guard just never reached it.
"""
import numpy as np
import pytest

import gtsam


class _SucceedingICP:
    """Stub ICP that converges, with a little spread between calls.

    Identical samples make the scatter matrix rank-deficient and MinCovDet
    itself raises — which is the very failure this module is about, so the
    success-path test needs samples that actually span three dimensions.
    """

    def __init__(self):
        self._rng = np.random.default_rng(0)

    def compute(self, source_points, target_points, guess_matrix):
        T = np.eye(3)
        T[:2, 2] = self._rng.normal(0.0, 0.05, size=2)
        theta = self._rng.normal(0.0, 0.01)
        T[:2, :2] = [[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]]
        return "success", T


def _localization(module):
    loc = module.Localization.__new__(module.Localization)
    loc.icp = _SucceedingICP()
    loc.icp_odom_sigmas = [0.1, 0.1, 0.01]
    return loc


def _guesses(n):
    return [gtsam.Pose2(0.0, 0.0, 0.0) for _ in range(n)]


def _cloud():
    return np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], dtype=np.float32)


@pytest.mark.parametrize("error", [np.linalg.LinAlgError("singular"), ValueError("bad")])
def test_covariance_failure_returns_instead_of_raising(load_localization, monkeypatch, error):
    module = load_localization

    class _FailingMinCovDet:
        def __init__(self, *args, **kwargs):
            pass

        def fit(self, samples):
            raise error

    monkeypatch.setattr(module, "MinCovDet", _FailingMinCovDet)

    message, transform, cov, samples = _localization(module).compute_icp_with_cov(
        _cloud(), _cloud(), _guesses(20)
    )

    assert message == "Failed to calculate covariance"
    assert transform is None and cov is None and samples is None


def test_covariance_success_path_is_untouched(load_localization):
    """The widened except must not swallow a healthy estimation."""
    module = load_localization

    message, transform, cov, samples = _localization(module).compute_icp_with_cov(
        _cloud(), _cloud(), _guesses(20)
    )

    assert message == "success"
    assert cov.shape == (3, 3)
    assert len(samples) == 20
