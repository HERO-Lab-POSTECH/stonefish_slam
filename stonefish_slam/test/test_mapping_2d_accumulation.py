"""Characterization: _accumulate_keyframe_into_map의 수치 배선.

`ema_fusion` 함수 자체는 test_fusion.py가 수치로 고정하지만, 호출부 배선
(alpha=0.3·threshold=0.0 인자, intensity 정규화식, 빈 mask 조기종료)은
어떤 테스트도 잡지 못했다 — mutation 4종이 전부 67 passed로 생존
(PR #10 리뷰 실측, main에도 동일한 선행 공백). 이 파일이 그 배선을 고정한다.

god-method 분해(PR #10)로 이 로직이 bound method 하나
(`_accumulate_keyframe_into_map`)가 되어 `__new__` 인스턴스로 직접 호출할
수 있다. fixture는 test_mapping_2d_p2c_cache.py의 stub 패턴을 따른다.
"""
import sys
import types

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

REL = "stonefish_slam/core/mapping_2d.py"

# test_mapping_2d_p2c_cache.py와 동일한 stub 집합 — mapping_2d.py의
# ROS 의존(cv_bridge·rclpy 등)만 막고 cv2/gtsam/scipy는 실제 import.
_ROS_STUBS = {
    "cv_bridge": {"CvBridge": object},
    "sensor_msgs_py": {},
    "sensor_msgs_py.point_cloud2": {},
    "rclpy": {},
    "rclpy.duration": {"Duration": object},
    "geometry_msgs": {},
    "geometry_msgs.msg": {"Pose": object},
    "std_msgs": {},
    "std_msgs.msg": {"Header": object},
}

THRESHOLD = 100.0


class _Pose:
    """gtsam.Pose2 대역 — 헬퍼가 읽는 x()/y()/theta()만 제공."""

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self._x, self._y, self._theta = x, y, theta

    def x(self):
        return self._x

    def y(self):
        return self._y

    def theta(self):
        return self._theta


@pytest.fixture
def make_mapper(load_module):
    """_accumulate_keyframe_into_map이 읽는 속성만 채운 인스턴스 팩토리."""
    preexisting = set(sys.modules)
    for name, attrs in _ROS_STUBS.items():
        if name not in sys.modules:
            mod = types.ModuleType(name)
            for k, v in attrs.items():
                setattr(mod, k, v)
            sys.modules[name] = mod
    sys.modules["sensor_msgs_py"].point_cloud2 = sys.modules[
        "sensor_msgs_py.point_cloud2"
    ]
    try:
        module = load_module(REL, "mapping_2d_accum")

        def _make():
            m = module.SonarMapping2D.__new__(module.SonarMapping2D)
            m.sonar_range = 15.0
            m.sonar_fov = 130.0
            m.range_min = 0.5
            m.p2c_cache = None
            m.intensity_threshold = THRESHOLD
            m.sonar_tilt_rad = 0.0
            m.map_resolution = 0.1
            # 소나 범위(15 m)보다 훨씬 넓은 고정 경계 — _expand_map 미발동
            m.min_x, m.max_x = -50.0, 50.0
            m.min_y, m.max_y = -50.0, 50.0
            m.map_height = m.map_width = 1000
            m.global_map_accum = np.zeros((1000, 1000), dtype=np.float32)
            m.global_map_count = np.zeros((1000, 1000), dtype=np.int32)
            m.logger = types.SimpleNamespace(
                info=lambda *a, **k: None, warning=lambda *a, **k: None)
            return m

        yield _make
    finally:
        for name in set(sys.modules) - preexisting:
            del sys.modules[name]


def _uniform_polar(value):
    """전 픽셀 동일 강도의 polar 프레임 — remap 보간 후 fan 내부도 동일값."""
    return np.full((100, 64), value, dtype=np.uint8)


def _normalized(raw):
    """호출부 정규화식: [threshold, 255] → [0, 255]."""
    return (raw - THRESHOLD) / (255.0 - THRESHOLD) * 255.0


def test_all_below_threshold_returns_false_without_side_effects(make_mapper):
    """전 픽셀이 threshold 이하면 False 반환, 맵은 무변화 (조기종료 배선)."""
    m = make_mapper()
    processed = m._accumulate_keyframe_into_map(
        _Pose(), _uniform_polar(0), sample_step=1)
    assert processed is False, '빈 mask는 False (호출부 continue와 동치)'
    assert m.global_map_count.sum() == 0, '스킵된 keyframe이 count를 올림'
    assert m.global_map_accum.sum() == 0.0, '스킵된 keyframe이 맵을 씀'


def test_first_observation_writes_normalized_intensity(make_mapper):
    """첫 관측은 정규화 강도를 그대로 기록 — 정규화식 배선 고정."""
    m255 = make_mapper()
    assert m255._accumulate_keyframe_into_map(
        _Pose(), _uniform_polar(255), sample_step=1) is True
    assert m255.global_map_accum.max() == pytest.approx(255.0, abs=0.01), \
        'raw 255 → 정규화 255.0 (배율 회귀)'

    m178 = make_mapper()
    assert m178._accumulate_keyframe_into_map(
        _Pose(), _uniform_polar(178), sample_step=1) is True
    assert m178.global_map_accum.max() == pytest.approx(
        _normalized(178.0), abs=0.1), \
        f'raw 178 → 정규화 {_normalized(178.0):.2f} (255 배율·threshold 오프셋 회귀)'


def test_second_observation_blends_with_ema_alpha(make_mapper):
    """재관측은 0.3·new + 0.7·old — alpha·threshold 인자 배선 고정.

    alpha가 바뀌면 가중치가, threshold가 커지면 old(255>threshold)가
    first-observation으로 오분류돼 new 직접 대입이 되므로 둘 다 이 값에서
    벗어난다.
    """
    m = make_mapper()
    pose = _Pose()
    m._accumulate_keyframe_into_map(pose, _uniform_polar(255), sample_step=1)
    m._accumulate_keyframe_into_map(pose, _uniform_polar(178), sample_step=1)
    expected = 0.3 * _normalized(178.0) + 0.7 * 255.0
    assert m.global_map_accum.max() == pytest.approx(expected, abs=0.1), \
        f'EMA 재관측 값은 {expected:.2f} (alpha=0.3·threshold=0.0 배선)'


def test_farthest_mapped_point_reaches_range_max(make_mapper):
    """P1-1: local_x_raw에 range_min 항이 빠져 전 맵 점이 차량 쪽으로 밀렸다.

    polar_to_cartesian_image는 행을 x = range_max - range_resolution*YY로 만들고
    range_resolution = (range_max - range_min)/rows다. 따라서 최상단 행은
    range_max에 있어야 하는데, range_min 항이 없으면 최원거리가
    rows*range_resolution = range_max - range_min에서 멈춘다.

    tilt=0, pose=원점이므로 global_x가 곧 slant range다.
    실측(range_max=15.0, range_min=0.5, rows=100 -> res=0.145):
        수정 후 99.5*0.145 + 0.5 = 14.9275  (map_row 649)
        수정 전 99.5*0.145       = 14.4275  (map_row 644)
    """
    m = make_mapper()
    assert m._accumulate_keyframe_into_map(
        _Pose(), _uniform_polar(255), sample_step=1) is True

    rows = np.nonzero(m.global_map_count.sum(axis=1))[0]
    farthest_x = rows.max() * m.map_resolution + m.min_x

    range_resolution = (m.sonar_range - m.range_min) / 100
    expected = m.sonar_range - 0.5 * range_resolution

    # 맵 격자 한 칸(0.1 m) 오차 허용 — map_row가 int로 잘린다.
    assert farthest_x == pytest.approx(expected, abs=m.map_resolution), \
        f'최원거리가 {farthest_x:.3f} m, range_max 부근({expected:.3f} m)이어야 한다'
