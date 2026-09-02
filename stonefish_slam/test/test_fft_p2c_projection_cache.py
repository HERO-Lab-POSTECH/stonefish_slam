"""FFT 직교변환 캐시는 투영 규약과 고도가 바뀌면 다시 지어져야 한다.

`polar_to_cartesian` 은 cv2.remap 사상표를 한 번 짓고 `p2c_cache` 에 넣는다.
사상표는 경사거리 r 을 어떤 좌표로 그릴지(`oculus.projection`)와, altitude
모드에서는 고도 h 에 통째로 의존한다. 키 비교 없이 재사용하면 고도가 바뀐 뒤에도
옛 사상표로 remap 해 픽셀 거리와 `cart_range_resolution` 이 어긋나고, 그 결과가
그대로 병진(m)이 된다 — 조용히 틀린다.

같은 결함군의 mapping_2d 쪽 가드는 test_mapping_2d_p2c_cache.py 에 있다.
"""
import sys
import types

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

REL = "stonefish_slam/core/localization_fft.py"


@pytest.fixture
def localizer(load_module):
    """polar_to_cartesian 이 읽는 속성만 가진 FFTLocalizer."""
    preexisting = set(sys.modules)
    for name in ("stonefish_slam", "stonefish_slam.utils", "stonefish_slam.utils.sonar"):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
    sys.modules["stonefish_slam.utils.sonar"].OculusProperty = object
    try:
        module = load_module(REL, "localization_fft_p2c")

        def make(projection="legacy", altitude=None, tilt_deg=30.0):
            f = module.FFTLocalizer.__new__(module.FFTLocalizer)
            f.verbose = False
            f.p2c_cache = None
            f.cart_range_resolution = None
            f.oculus = types.SimpleNamespace(
                range_max=40.0,
                horizontal_fov=np.radians(130.0),
                tilt_angle_rad=np.radians(tilt_deg),
                projection=projection,
                altitude_m=altitude,
            )
            return f

        yield make
    finally:
        for name in set(sys.modules) - preexisting:
            del sys.modules[name]


def _polar(rows=100, cols=64):
    img = np.zeros((rows, cols), dtype=np.uint8)
    img[rows // 2, cols // 2] = 255
    return img


def test_projection_change_rebuilds_the_maps(localizer):
    """legacy 와 inv_cos_tilt 는 직교영상 크기부터 달라야 한다."""
    f = localizer("legacy")
    legacy = f.polar_to_cartesian(_polar())
    f.oculus.projection = "inv_cos_tilt"
    inv = f.polar_to_cartesian(_polar())
    assert inv.shape[0] > legacy.shape[0], (
        f"투영을 바꿨는데 사상표가 그대로다 (legacy {legacy.shape} → inv {inv.shape})"
    )
    # C = r/cos(tau) 대 r·cos(tau) 이므로 세로 길이 비는 1/cos²(tau)
    assert inv.shape[0] / legacy.shape[0] == pytest.approx(
        1.0 / np.cos(np.radians(30.0)) ** 2, rel=0.02)


def test_altitude_change_rebuilds_the_maps(localizer):
    """고도가 달라지면 사상표도 달라져야 한다 — 캐시 키에 고도가 들어 있다."""
    f = localizer("altitude", altitude=4.0)
    shallow = f.polar_to_cartesian(_polar())
    f.oculus.altitude_m = 20.0
    deep = f.polar_to_cartesian(_polar())
    assert deep.shape[0] < shallow.shape[0], (
        f"고도를 {4.0}→{20.0} m 로 바꿨는데 직교영상 크기가 그대로다 "
        f"({shallow.shape} → {deep.shape})"
    )


def test_small_altitude_drift_reuses_the_cache(localizer):
    """0.25 m 격자 안의 흔들림으로 매 프레임 사상표를 다시 짓지는 않는다."""
    f = localizer("altitude", altitude=4.55)   # 4.55, 4.60 은 같은 0.25 m 칸
    f.polar_to_cartesian(_polar())
    first = f.p2c_cache["map_x"]
    f.oculus.altitude_m = 4.60
    f.polar_to_cartesian(_polar())
    assert f.p2c_cache["map_x"] is first, "같은 칸 안의 0.05 m 차이로 사상표를 다시 지었다"
