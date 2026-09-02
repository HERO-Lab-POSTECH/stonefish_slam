# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""3D 복셀 라벨 역투영 — **순수 파이썬 백엔드에서만** 도는 특성 테스트.

C++ OctoMap 이 있는 환경에서는 `get_point_cloud()` 가 C++ 경로를 타지만, 여기서
확인하려는 것은 백엔드가 아니라 **역투영 기하와 라벨 수명 규칙**이라 파이썬
백엔드로 고정한다(`use_cpp_backend: False`). C++ 확장이 실제로 같은 배치를
내는지는 `test_cpp_extensions.py` 쪽 왕복 케이스가 본다.

핵심은 `_voxel_to_sonar_coords` 를 쓰지 않는다는 것이다 — 그 헬퍼는 수평거리를
내는데 `ray_processor.cpp` 는 복셀을 **경사거리**로 놓는다. 앙각이 0 이 아니면
두 값이 다르고, 수평거리로 역투영하면 라벨이 엉뚱한 range bin 에 붙는다.
"""
import numpy as np
import pytest

FOV_DEG = 120.0
NUM_BEAMS, NUM_BINS = 256, 100
RANGE_MIN, RANGE_MAX = 0.5, 10.0
VOXEL = 0.5


def _config():
    """순수 파이썬 경로만 타는 최소 config."""
    return {
        "horizontal_fov": FOV_DEG, "vertical_fov": 20.0,
        "range_max": RANGE_MAX, "range_min": RANGE_MIN,
        "num_beams": NUM_BEAMS, "num_bins": NUM_BINS,
        "voxel_resolution": VOXEL,
        "sonar_position": [0.0, 0.0, 0.0], "sonar_tilt_deg": 0.0,
        "intensity_threshold": 50.0,
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
        # 0.7: 명시적으로 채운 복셀(log-odds 3.0 → p≈0.95)만 점유로 세게 한다.
        # 0.1 이면 한 번 스친 셀(log-odds 0 → p=0.5)까지 다 딸려 나온다.
        "min_probability": 0.7, "max_frames": 100, "dynamic_expansion": False,
        "sharpness": 1.0, "decay_rate": 0.05, "min_alpha": 0.3,
        "bearing_step": 1,
    }


@pytest.fixture
def mapper(load_mapping_3d):
    return load_mapping_3d.SonarMapping3D(_config())


IDENTITY_POSE = {'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}}


def _cpp_placement(r_idx, b_idx, elevation_deg=0.0):
    """`ray_processor.cpp` 가 그 픽셀의 복셀을 놓는 자리(격자화 전)."""
    res = (RANGE_MAX - RANGE_MIN) / NUM_BINS
    fov = np.radians(FOV_DEG)
    rng = RANGE_MAX - r_idx * res
    bearing = -fov / 2.0 + b_idx * fov / (NUM_BEAMS - 1)
    v = np.radians(elevation_deg)
    return np.array([rng * np.cos(v) * np.cos(bearing),
                     rng * np.cos(v) * np.sin(bearing),
                     rng * np.sin(v)])


def _occupy(mapper, points, log_odds=3.0):
    """주어진 world 점들을 점유 복셀로 만든다(파이썬 octree 직접 갱신)."""
    for p in points:
        mapper.octree.update_voxel(np.asarray(p, dtype=float), log_odds)


# ------------------------------------------------------------- 역투영 기하


@pytest.mark.parametrize("r_idx,b_idx,elev", [(0, 0, 0.0), (37, 130, 7.5),
                                              (80, 255, -9.0)])
def test_ungridded_placement_round_trips_to_the_same_pixel(load_mapping_3d,
                                                           r_idx, b_idx, elev):
    """격자화 **전**의 점은 픽셀로 정확히 되돌아와야 한다."""
    sem = __import__("sys").modules["stonefish_slam.core.semantic"]
    p = _cpp_placement(r_idx, b_idx, elev).reshape(1, 3)
    slant, bearing = sem.slant_range_bearing(p)
    row, col = sem.sonar_to_pixel(
        slant, bearing, num_bins=NUM_BINS, num_beams=NUM_BEAMS,
        range_min=RANGE_MIN, range_max=RANGE_MAX, horizontal_fov_deg=FOV_DEG)
    assert row[0] == pytest.approx(r_idx, abs=1e-6)
    assert col[0] == pytest.approx(b_idx, abs=1e-6)


def test_a_voxel_centre_still_lands_inside_the_box_thanks_to_the_pad(mapper):
    """복셀 **중심**은 격자에 스냅된 자리라 정확한 역산이 없다 — pad 로 흡수한다."""
    target = _cpp_placement(40, 128, 5.0)
    _occupy(mapper, [target])
    # 그 복셀의 실제 중심(octree 가 돌려주는 값)으로 검사한다.
    centre = mapper.get_point_cloud()['points'][0]
    assert not np.allclose(centre, target), "격자화가 안 일어났다 — 전제가 깨졌다"

    dets = np.array([[0, 0.9, 120.0, 35.0, 136.0, 45.0]])   # col 120~136, row 35~45
    assert mapper.label_voxels_from_keyframe(IDENTITY_POSE, dets) == 1
    _points, _probs, labels = mapper.get_labeled_point_cloud()
    assert list(labels) == [1]


def test_a_voxel_outside_the_box_is_not_labelled(mapper):
    _occupy(mapper, [_cpp_placement(40, 128), _cpp_placement(10, 20)])
    dets = np.array([[0, 0.9, 120.0, 35.0, 136.0, 45.0]])

    mapper.label_voxels_from_keyframe(IDENTITY_POSE, dets)

    _p, _q, labels = mapper.get_labeled_point_cloud()
    assert sorted(labels) == [0, 1]


def test_a_voxel_behind_the_sonar_is_never_labelled(mapper):
    """FOV 밖이면 픽셀 좌표가 우연히 bbox 안에 들어도 라벨이 붙으면 안 된다."""
    _occupy(mapper, [np.array([-5.0, 0.0, 0.0])])     # 정후방
    dets = np.array([[0, 0.9, 0.0, 0.0, 255.0, 99.0]])  # 이미지 전체

    assert mapper.label_voxels_from_keyframe(IDENTITY_POSE, dets) == 0


def test_no_detections_labels_nothing(mapper):
    _occupy(mapper, [_cpp_placement(40, 128)])
    assert mapper.label_voxels_from_keyframe(IDENTITY_POSE, np.zeros((0, 6))) == 0
    _p, _q, labels = mapper.get_labeled_point_cloud()
    assert list(labels) == [0]


def test_a_later_detection_overwrites_the_label(mapper):
    _occupy(mapper, [_cpp_placement(40, 128)])
    box = [120.0, 35.0, 136.0, 45.0]
    mapper.label_voxels_from_keyframe(IDENTITY_POSE, np.array([[0, 0.9] + box]))
    mapper.label_voxels_from_keyframe(IDENTITY_POSE, np.array([[4, 0.9] + box]))
    _p, _q, labels = mapper.get_labeled_point_cloud()
    assert list(labels) == [5]


# --------------------------------------------------------------- 라벨 수명


def test_labels_are_dropped_when_the_map_resets(mapper):
    """`max_frames` 리셋으로 지도가 비면 라벨도 같이 가야 한다.

    안 그러면 같은 셀을 나중에 다시 점유한 복셀이, 로봇이 이미 떠난 곳의 라벨을
    물려받는다.
    """
    _occupy(mapper, [_cpp_placement(40, 128)])
    mapper.label_voxels_from_keyframe(
        IDENTITY_POSE, np.array([[0, 0.9, 120.0, 35.0, 136.0, 45.0]]))
    assert mapper.voxel_labels

    mapper.max_frames = 1
    mapper.frame_count = 2
    # 리셋 분기만 타면 되므로 최소 크기 이미지로 부른다 — 정식 크기의 빈
    # 이미지를 순수 파이썬 경로로 돌리면 이 한 케이스가 수 초를 먹는다.
    mapper.process_sonar_image(np.zeros((8, 16), np.uint8), IDENTITY_POSE)

    assert mapper.voxel_labels == {}, "지도는 비었는데 라벨이 남았다"


def test_only_currently_occupied_voxels_come_out(mapper):
    """점유가 풀린 셀의 라벨은 출력에 나오면 안 된다."""
    _occupy(mapper, [_cpp_placement(40, 128)])
    mapper.label_voxels_from_keyframe(
        IDENTITY_POSE, np.array([[0, 0.9, 120.0, 35.0, 136.0, 45.0]]))

    # 라벨 dict 은 그대로 두고 지도만 비운다.
    stale = dict(mapper.voxel_labels)
    mapper.octree.clear()
    mapper.voxel_labels = stale

    points, _probs, labels = mapper.get_labeled_point_cloud()
    assert len(points) == 0 and len(labels) == 0


def test_prefetched_points_are_used_instead_of_walking_the_octree_again(mapper):
    """맵 틱 하나에서 키프레임이 여럿이면 옥트리를 M+1 번 걷게 된다 — 한 번만."""
    target = _cpp_placement(40, 128)
    _occupy(mapper, [target])
    points = mapper.get_point_cloud()['points']
    dets = np.array([[0, 0.9, 120.0, 35.0, 136.0, 45.0]])

    calls = {'n': 0}
    real = mapper.get_point_cloud

    def _counting(*a, **k):
        calls['n'] += 1
        return real(*a, **k)
    mapper.get_point_cloud = _counting

    assert mapper.label_voxels_from_keyframe(IDENTITY_POSE, dets, points=points) == 1
    assert calls['n'] == 0, "points 를 넘겼는데도 옥트리를 다시 걸었다"
    assert list(mapper.labels_for_points(points)) == [1]


def test_empty_map_gives_empty_arrays(mapper):
    points, probs, labels = mapper.get_labeled_point_cloud()
    assert len(points) == 0 and len(probs) == 0 and len(labels) == 0
    assert labels.dtype == np.uint8
