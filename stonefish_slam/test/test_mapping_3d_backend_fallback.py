# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""3D 매퍼의 백엔드 폴백과 `max_frames` 리셋 — 두 AttributeError 회귀 가드.

둘 다 `self.octree` 가 `use_cpp_backend` 가 켜져 있을 때 `None` 이라는 같은
사실에서 나온 크래시였고, 둘 다 `config/slam.yaml` 의 노브만으로 도달했다.

1. `use_cpp_backend: true` + `use_cpp_ray_processor: false` → 파이썬 광선
   경로가 `self.octree.update_voxel` 을 부르는데 그 옥트리가 없어 **첫
   프레임**에서 죽었다.
2. `use_cpp_backend: true` + `max_frames > 0` → 리셋이 무조건 파이썬 옥트리를
   지우려 들어 **`max_frames + 1` 번째 프레임**에서 죽었다.

여기 케이스는 C++ 확장 유무와 무관하게 판정이 서도록 썼다 — 확장이 없으면
생성자가 어차피 파이썬으로 떨어지므로 (1) 은 그때도 참이어야 하고, (2) 는
스텁 옥트리로 분기만 본다(확장 불필요).
"""
import sys

import numpy as np
import pytest

FOV_DEG = 120.0
NUM_BEAMS, NUM_BINS = 64, 40
RANGE_MIN, RANGE_MAX = 0.5, 10.0


def _config(**over):
    cfg = {
        "range_min": RANGE_MIN, "range_max": RANGE_MAX,
        "range_resolution": (RANGE_MAX - RANGE_MIN) / NUM_BINS,
        "horizontal_fov": np.radians(FOV_DEG), "vertical_fov": np.radians(20.0),
        "num_beams": NUM_BEAMS, "num_bins": NUM_BINS,
        "voxel_resolution": 0.5,
        "sonar_position": [0.0, 0.0, 0.0], "sonar_tilt_deg": 0.0,
        "intensity_threshold": 50,
        "log_odds_occupied": 0.85, "log_odds_free": -0.40,
        "log_odds_min": -6.0, "log_odds_max": 6.0,
        "adaptive_update": False, "adaptive_threshold": 0.7,
        "adaptive_max_ratio": 3.0,
        "use_range_weighting": False, "lambda_decay": 0.1,
        "enable_gaussian_weighting": False, "gaussian_sigma_factor": 2.5,
        "use_cpp_backend": False, "use_cpp_ray_processor": False,
        "use_dda_traversal": False,
        "enable_propagation": False, "propagation_radius": 2,
        "propagation_sigma": 1.5,
        "update_method": "log_odds", "frame_interval": 10,
        "min_probability": 0.7, "max_frames": 0, "dynamic_expansion": False,
        "sharpness": 1.0, "decay_rate": 0.05, "min_alpha": 0.3,
        "bearing_step": 1,
    }
    cfg.update(over)
    return cfg


IDENTITY_POSE = {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}


class _StubOctree:
    """`clear()` 가 실제로 자기에게 왔는지만 기록하는 최소 스텁."""

    def __init__(self):
        self.cleared = 0

    def clear(self):
        self.cleared += 1


def _frame():
    img = np.zeros((NUM_BINS, NUM_BEAMS), np.uint8)
    img[NUM_BINS // 2, NUM_BEAMS // 2:NUM_BEAMS // 2 + 4] = 200
    return img


def test_the_backend_never_survives_without_its_ray_processor(load_mapping_3d):
    """C++ 옥트리만 남고 광선 처리기가 빠지는 조합은 생성자를 못 빠져나온다."""
    m = load_mapping_3d.SonarMapping3D(
        _config(use_cpp_backend=True, use_cpp_ray_processor=False))

    assert not (m.use_cpp_backend and not m.use_cpp_ray_processor), (
        "C++ 백엔드가 켜진 채 광선 처리기만 꺼지면 파이썬 갱신 경로가 "
        "존재하지 않는 self.octree 에 쓴다")
    # 그 불변식의 다른 얼굴: 파이썬 옥트리는 백엔드가 꺼졌을 때만 있다.
    assert (m.octree is None) == m.use_cpp_backend


def test_that_combination_no_longer_dies_on_the_first_frame(load_mapping_3d):
    """회귀: 예전엔 프레임 1 에서 `NoneType.update_voxel` 로 죽었다."""
    m = load_mapping_3d.SonarMapping3D(
        _config(use_cpp_backend=True, use_cpp_ray_processor=False))
    for _ in range(2):
        m.process_sonar_image(_frame(), IDENTITY_POSE)   # 예전 크래시 지점
    assert m.frame_count == 2


def test_the_frame_limit_reset_clears_whichever_octree_is_live(load_mapping_3d):
    """C++ 백엔드에서는 C++ 옥트리가 지워진다 — 예전 크래시 지점."""
    m = load_mapping_3d.SonarMapping3D(_config(max_frames=2))
    stub = _StubOctree()
    m.use_cpp_backend, m.cpp_octree, m.octree = True, stub, None
    m.voxel_labels = {(1, 2, 3): 7}
    m.frame_count = 3

    assert m._reset_map_if_frame_limit() is True
    assert stub.cleared == 1
    assert m.voxel_labels == {}
    assert m.frame_count == 0


def test_the_python_backend_still_clears_its_own_octree(load_mapping_3d):
    m = load_mapping_3d.SonarMapping3D(_config(max_frames=2))
    stub = _StubOctree()
    m.use_cpp_backend, m.octree, m.cpp_octree = False, stub, None
    m.voxel_labels = {(1, 2, 3): 7}
    m.frame_count = 3

    assert m._reset_map_if_frame_limit() is True
    assert stub.cleared == 1
    assert m.voxel_labels == {}


@pytest.mark.parametrize("max_frames,frame_count", [(0, 999), (5, 5)])
def test_the_reset_is_a_no_op_at_or_below_the_limit(
        load_mapping_3d, max_frames, frame_count):
    """`max_frames: 0` 은 무제한이고, 경계값에서는 아직 안 지운다."""
    m = load_mapping_3d.SonarMapping3D(_config(max_frames=max_frames))
    stub = _StubOctree()
    m.use_cpp_backend, m.cpp_octree, m.octree = True, stub, None
    m.voxel_labels = {(1, 2, 3): 7}
    m.frame_count = frame_count

    assert m._reset_map_if_frame_limit() is False
    assert stub.cleared == 0
    assert m.voxel_labels == {(1, 2, 3): 7}
    assert m.frame_count == frame_count


class _FakeCppOctree:
    """생성자가 부르는 설정 메서드를 전부 삼키는 C++ 옥트리 대역."""

    def __init__(self, *_a, **_kw):
        self.cleared = 0

    def clear(self):
        self.cleared += 1

    def __getattr__(self, _name):        # set_adaptive_params 등 전부 no-op
        return lambda *a, **kw: None


class _FakeOctreeModule:
    OctreeMapping = _FakeCppOctree


def test_the_guard_holds_even_where_the_cpp_octree_imports(
        load_mapping_3d, monkeypatch):
    """CI 는 `.so` 가 없어 생성자가 어차피 파이썬으로 떨어진다 — 그러면 폴백
    가드를 지워도 위 두 케이스가 초록으로 통과한다(agy 적대 검증 NIT 4).

    그래서 여기서는 C++ 옥트리 import 만 대역으로 성공시키고 RayProcessor 는
    없는 상태를 만든다. 확장 유무와 무관하게 **가드 자신**이 실행되는 유일한
    케이스다.
    """
    monkeypatch.setattr(load_mapping_3d, 'CPP_RAY_PROCESSOR_AVAILABLE', False)
    monkeypatch.setattr(sys.modules['stonefish_slam'], 'octree_mapping',
                        _FakeOctreeModule, raising=False)

    m = load_mapping_3d.SonarMapping3D(
        _config(use_cpp_backend=True, use_cpp_ray_processor=True))

    # 대역이 실제로 물렸는지 먼저 확인한다 — 안 물렸으면 이 케이스는 위의 두
    # 케이스와 같은 vacuous 통과가 되고, 그걸 모르고 지나가는 게 함정이었다.
    assert m.octree is not None, "C++ 옥트리 대역이 안 물렸다 — 케이스가 무력하다"
    assert m.use_cpp_backend is False
    assert m.cpp_octree is None


def test_a_runtime_ray_processor_failure_does_not_kill_the_node(load_mapping_3d):
    """프레임 처리 중 C++ 광선 처리기가 던져도 노드는 살아야 한다.

    예전에는 그 `except` 가 파이썬 경로로 폴백했는데, C++ 백엔드에서는 flush 할
    `self.octree` 가 없어 `NoneType.update_voxel` 로 죽었다 — "이 프레임만 실패"가
    "노드 사망"이 됐다(agy 적대 검증 MAJOR 2). 이제 그 프레임의 지도 갱신만 버린다.
    """
    m = load_mapping_3d.SonarMapping3D(_config())
    # 백엔드는 C++, 파이썬 옥트리는 없음 — 실제 배포 기본값의 상태.
    m.use_cpp_backend, m.octree, m.cpp_octree = True, None, _FakeCppOctree()

    class _ExplodingRayProcessor:
        def process_sonar_image(self, *_a, **_kw):
            raise RuntimeError("C++ 광선 처리기가 이 프레임에서 던졌다")

    m.use_cpp_ray_processor = True
    m.cpp_ray_processor = _ExplodingRayProcessor()

    m.process_sonar_image(_frame(), IDENTITY_POSE)   # 예전 크래시 지점
    assert m.frame_count == 1
