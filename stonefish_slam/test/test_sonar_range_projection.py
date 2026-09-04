"""경사거리 → 수평거리 투영이 병진을 보존하는지 본다.

하향 틸트 FLS 가 평평한 바닥을 볼 때, 수평거리 rho 인 점은 경사거리
r = sqrt(rho^2 + h^2) 로 재진다. 점군을 r 로 그리면 전진 d 에 대한 점의 이동이
d·cos(theta) 라서 ICP 가 병진을 축소해 본다. 이 파일이 지키는 계약은 하나다 —
``altitude`` 투영은 rho 를 정확히 되돌려야 한다. 되돌리지 못하면 SLAM 궤적이
그 비율만큼 조여든다.

feature_extraction.py 는 import-time 에 cv2/cv_bridge/cfar(.so) 를 끌어오므로
파일 경로 로드 + stub 으로 우회하고, 클래스는 __new__ 로 만들어 _project_range 가
읽는 속성만 채운다.
"""
import sys
import types

import numpy as np
import pytest

REL = "stonefish_slam/core/feature_extraction.py"

_STUBS = {
    "cv2": {},
    "cv_bridge": {"CvBridge": object},
    "stonefish_slam": {},
    "stonefish_slam.core": {},
    "stonefish_slam.core.cfar": {"CFAR": object},
}


@pytest.fixture
def extractor(load_module):
    """_project_range 가 읽는 속성만 가진 FeatureExtraction."""
    preexisting = set(sys.modules)
    for name, attrs in _STUBS.items():
        if name not in sys.modules:
            mod = types.ModuleType(name)
            for k, v in attrs.items():
                setattr(mod, k, v)
            sys.modules[name] = mod
    try:
        module = load_module(REL, "feature_extraction_projection")

        def make(projection, tilt_deg=30.0, altitude=None):
            fe = module.FeatureExtraction.__new__(module.FeatureExtraction)
            fe.projection = projection
            cos_tilt = float(np.cos(np.radians(tilt_deg)))
            fe._inv_cos_tilt = 1.0 / cos_tilt
            fe.sonar_tilt_rad = float(np.radians(tilt_deg))
            fe.proj_dropped = 0
            fe.proj_alt_missing = 0
            fe.node = types.SimpleNamespace(altitude_m=altitude)
            return fe

        yield make
    finally:
        for name in set(sys.modules) - preexisting:
            del sys.modules[name]


def test_altitude_projection_recovers_horizontal_range(extractor):
    """h 를 알면 바닥 점의 수평거리를 정확히 되돌려야 한다."""
    h = 4.6
    fe = extractor("altitude", altitude=h)
    for rho in (5.0, 8.0, 12.0, 20.0):
        slant = np.hypot(rho, h)
        assert fe._project_range(slant) == pytest.approx(rho, abs=1e-9), (
            f"rho={rho} 를 되돌리지 못했다 — 병진이 {fe._project_range(slant)/rho:.3f} 배로 나온다"
        )


def test_legacy_projection_compresses_translation(extractor):
    """legacy 는 r 을 그대로 쓴다 — 이 테스트는 축소가 실재함을 고정한다."""
    h, rho, d = 4.6, 8.0, 0.5
    fe = extractor("legacy", altitude=h)
    moved = fe._project_range(np.hypot(rho - d, h)) - fe._project_range(np.hypot(rho, h))
    assert abs(moved) < d, "경사거리 그대로 쓰는데 병진이 축소되지 않았다"
    assert abs(moved) / d == pytest.approx(rho / np.hypot(rho, h), abs=0.02)


def test_inv_cos_tilt_is_a_constant_approximation(extractor):
    """r/cos(tau) 는 고도각이 틸트와 같은 거리에서만 정확하다."""
    tilt = 30.0
    h = 4.6
    fe = extractor("inv_cos_tilt", tilt_deg=tilt, altitude=h)
    rho_exact = h / np.tan(np.radians(tilt))          # theta == tau 인 지점
    d = 0.2
    slant = lambda r: np.hypot(r, h)
    moved = fe._project_range(slant(rho_exact - d)) - fe._project_range(slant(rho_exact))
    assert abs(moved) / d == pytest.approx(1.0, abs=0.02)


def test_points_closer_than_the_altitude_are_dropped(extractor):
    """r <= h 는 바닥 반사가 아니다 — 수평 성분이 없으므로 버리고 센다."""
    fe = extractor("altitude", altitude=4.6)
    assert fe._project_range(3.0) is None
    assert fe.proj_dropped == 1


def test_missing_altitude_falls_back_without_crashing(extractor):
    """고도계가 아직 안 왔을 때 altitude 모드는 상수 근사로 떨어진다."""
    fe = extractor("altitude", altitude=None)
    assert fe._project_range(10.0) == pytest.approx(10.0 / np.cos(np.radians(30.0)))
    assert fe.proj_alt_missing == 1


# --- 빔 방위 -> 수평면 방위 --------------------------------------------------
# 거리 사상이 맞아도 방위가 틀리면 병진은 여전히 축소된다. 빔은 소나 자신의
# 기울어진 프레임에서 정의되므로 바닥의 점 (X, Y, h) 은
#   beta = atan2(Y, X*cos(tau) + h*sin(tau))
# 로 관측된다. 아래 계약은 _beam_azimuth 가 그 beta 를 참 방위 atan2(Y, X) 로
# 되돌린다는 것 하나다.

def _true_beam_bearing(x, y, h, tilt_deg):
    """참 지면 좌표에서 소나가 실제로 보게 되는 빔 방위(순방향 사상)."""
    tau = np.radians(tilt_deg)
    return np.arctan2(y, x * np.cos(tau) + h * np.sin(tau))


@pytest.mark.parametrize("x,y", [(10.0, 10.0), (14.0, 5.0), (5.0, 12.0),
                                 (20.0, 0.0), (8.0, -9.0), (30.0, -2.0)])
def test_beam_azimuth_recovers_the_true_horizontal_bearing(extractor, x, y):
    h, tilt_deg = 6.4, 30.0
    fe = extractor("altitude", tilt_deg=tilt_deg, altitude=h)
    rho = float(np.hypot(x, y))
    beta = _true_beam_bearing(x, y, h, tilt_deg)
    assert fe._beam_azimuth(beta, rho) == pytest.approx(np.arctan2(y, x), abs=1e-9)


def test_beam_azimuth_is_identity_without_tilt(extractor):
    """틸트가 0 이면 빔 프레임과 수평면이 같으므로 보정이 없어야 한다."""
    fe = extractor("altitude", tilt_deg=0.0, altitude=6.4)
    for beta in np.radians([-65.0, -20.0, 0.0, 33.0, 65.0]):
        assert fe._beam_azimuth(beta, 15.0) == pytest.approx(beta, abs=1e-12)


def test_beam_azimuth_leaves_bearing_alone_without_altitude(extractor):
    """고도를 모르면 rho 가 참 수평거리가 아니라 보정 근거가 없다."""
    fe = extractor("altitude", tilt_deg=30.0, altitude=None)
    assert fe._beam_azimuth(0.5, 12.0) == pytest.approx(0.5)
    fe_legacy = extractor("legacy", tilt_deg=30.0, altitude=6.4)
    assert fe_legacy._beam_azimuth(0.5, 12.0) == pytest.approx(0.5)
