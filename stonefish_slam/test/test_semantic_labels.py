#!/usr/bin/env python3
"""`core/semantic.py` — 라벨 부여·픽셀 변환·미결 큐의 특성 테스트.

`feature_extraction.py` 는 상단에서 `cv_bridge`·`cfar` 를 끌어오므로 `load_module`
로 열 수 없다. 그래서 그 모듈의 변환식은 여기서 **오라클로 다시 적어** 대조한다
(출처: `stonefish_slam/core/feature_extraction.py` `extract_features()` 의
range/bearing 계산). 식이 갈리면 이 테스트가 깨진다.
"""

import numpy as np
import pytest

REL = "stonefish_slam/core/semantic.py"

# bluerov2.scn 의 FLS 와 config/slam.yaml `sonar:` 기본값.
NUM_BINS, NUM_BEAMS = 500, 512
RANGE_MIN, RANGE_MAX, FOV_DEG = 0.5, 40.0, 130.0


@pytest.fixture
def sem(load_module):
    return load_module(REL, "stonefish_slam_semantic")


def _feature_extraction_oracle(row, col):
    """`extract_features()` 의 픽셀 → (x, y) 변환을 그대로 옮긴 것."""
    range_m = RANGE_MAX - (row / (NUM_BINS - 1)) * (RANGE_MAX - RANGE_MIN)
    bearing = np.radians(-FOV_DEG / 2.0 + (col / (NUM_BEAMS - 1)) * FOV_DEG)
    return range_m * np.cos(bearing), range_m * np.sin(bearing)


# ---------------------------------------------------------------- labels


def test_label_is_class_plus_one_inside_the_box(sem):
    peaks = np.array([[10, 20], [300, 400]])
    dets = np.array([[0, 0.9, 15.0, 5.0, 25.0, 15.0]])  # col 15~25, row 5~15
    assert list(sem.labels_from_detections(peaks, dets)) == [1, 0]


def test_box_edges_are_inclusive(sem):
    peaks = np.array([[5, 15], [15, 25], [4, 15], [15, 26]])
    dets = np.array([[0, 0.9, 15.0, 5.0, 25.0, 15.0]])
    assert list(sem.labels_from_detections(peaks, dets)) == [1, 1, 0, 0]


def test_later_box_wins_on_overlap(sem):
    peaks = np.array([[10, 20]])
    dets = np.array([
        [0, 0.9, 0.0, 0.0, 100.0, 100.0],
        [3, 0.5, 0.0, 0.0, 100.0, 100.0],
    ])
    assert sem.labels_from_detections(peaks, dets)[0] == 4


def test_no_detections_gives_all_zeros(sem):
    peaks = np.array([[1, 2], [3, 4]])
    out = sem.labels_from_detections(peaks, np.zeros((0, 6)))
    assert out.dtype == np.uint8 and list(out) == [0, 0]


def test_no_peaks_gives_empty_labels(sem):
    out = sem.labels_from_detections(np.zeros((0, 2), int),
                                     np.array([[0, 0.9, 0.0, 0.0, 9.0, 9.0]]))
    assert out.shape == (0,) and out.dtype == np.uint8


# ------------------------------------------------------- detection rows


def test_detection_rows_converts_centre_and_size_to_corners(sem):
    rows, low, bad = sem.detection_rows([("3", 0.8, 100.0, 50.0, 20.0, 10.0)], 0.25)
    assert (low, bad) == (0, 0)
    assert list(rows[0]) == [3.0, pytest.approx(0.8), 90.0, 45.0, 110.0, 55.0]


def test_detection_rows_drops_low_confidence_and_counts_it(sem):
    rows, low, bad = sem.detection_rows(
        [("0", 0.9, 1.0, 1.0, 2.0, 2.0), ("0", 0.1, 1.0, 1.0, 2.0, 2.0)], 0.25)
    assert len(rows) == 1 and (low, bad) == (1, 0)


def test_detection_rows_drops_non_numeric_class_id_and_counts_it(sem):
    """`class_id` 는 표준상 문자열 키다 — 이름을 실어 보내는 발행자가 있을 수 있다."""
    rows, low, bad = sem.detection_rows([("sofa", 0.9, 1.0, 1.0, 2.0, 2.0)], 0.25)
    assert len(rows) == 0 and (low, bad) == (0, 1)


def test_detection_rows_of_nothing_is_an_empty_kx6(sem):
    rows, low, bad = sem.detection_rows([], 0.25)
    assert rows.shape == (0, 6) and (low, bad) == (0, 0)


def test_detection_rows_feed_labels_from_detections(sem):
    """두 함수의 열 규약(x=col, y=row)이 맞물리는지."""
    rows, _, _ = sem.detection_rows([("0", 0.9, 100.0, 50.0, 20.0, 10.0)], 0.25)
    peaks = np.array([[50, 100], [50, 89], [44, 100]])  # 중심, 좌측 밖, 상단 밖
    assert list(sem.labels_from_detections(peaks, rows)) == [1, 0, 0]


# ------------------------------------------------- pixel <-> bearing/range


def test_pixel_to_bearing_range_endpoints(sem):
    kw = dict(num_bins=NUM_BINS, num_beams=NUM_BEAMS, range_min=RANGE_MIN,
              range_max=RANGE_MAX, horizontal_fov_deg=FOV_DEG)
    b0, r0 = sem.pixel_to_bearing_range(0, 0, **kw)
    b1, r1 = sem.pixel_to_bearing_range(NUM_BINS - 1, NUM_BEAMS - 1, **kw)
    assert r0 == pytest.approx(RANGE_MAX)       # row 0 = 먼 쪽
    assert r1 == pytest.approx(RANGE_MIN)
    assert np.degrees(b0) == pytest.approx(-FOV_DEG / 2.0)
    assert np.degrees(b1) == pytest.approx(+FOV_DEG / 2.0)


@pytest.mark.parametrize("row,col", [(0, 0), (137, 42), (250, 256), (499, 511)])
def test_pixel_to_bearing_range_matches_feature_extraction(sem, row, col):
    """factor 측정값이 `Keyframe.points` 와 같은 기하에 놓이는지."""
    bearing, rng = sem.pixel_to_bearing_range(
        row, col, num_bins=NUM_BINS, num_beams=NUM_BEAMS,
        range_min=RANGE_MIN, range_max=RANGE_MAX, horizontal_fov_deg=FOV_DEG)
    x, y = rng * np.cos(bearing), rng * np.sin(bearing)
    ox, oy = _feature_extraction_oracle(row, col)
    assert (x, y) == (pytest.approx(ox), pytest.approx(oy))


def test_slant_range_is_the_norm_not_the_horizontal_range(sem):
    """`_voxel_to_sonar_coords`(수평거리)로는 역투영이 안 된다는 증거."""
    p = np.array([[3.0, 4.0, 12.0]])
    slant, bearing = sem.slant_range_bearing(p)
    assert slant[0] == pytest.approx(13.0)
    assert np.hypot(p[0, 0], p[0, 1]) == pytest.approx(5.0)   # 수평거리
    assert bearing[0] == pytest.approx(np.arctan2(4.0, 3.0))


def test_sonar_to_pixel_inverts_the_cpp_voxel_placement(sem):
    """`ray_processor.cpp` 가 복셀을 놓은 식을 정확히 되짚는가.

    C++: `range_m = range_max − r_idx·(range_max−range_min)/num_bins`,
    `bearing = −fov/2 + b_idx·fov/(num_beams−1)`,
    `x = r·cos(v)·cos(b)`, `y = r·cos(v)·sin(b)`, `z = r·sin(v)`.
    앙각 v 가 0 이 아니어도 `|P| = r` 이므로 왕복이 정확해야 한다.
    """
    res = (RANGE_MAX - RANGE_MIN) / NUM_BINS
    fov = np.radians(FOV_DEG)
    for r_idx, b_idx, v_deg in [(0, 0, 0.0), (123, 77, 7.5), (400, 511, -9.0)]:
        rng = RANGE_MAX - r_idx * res
        bearing = -fov / 2.0 + b_idx * fov / (NUM_BEAMS - 1)
        v = np.radians(v_deg)
        p = np.array([[rng * np.cos(v) * np.cos(bearing),
                       rng * np.cos(v) * np.sin(bearing),
                       rng * np.sin(v)]])
        slant, back_bearing = sem.slant_range_bearing(p)
        row, col = sem.sonar_to_pixel(
            slant, back_bearing, num_bins=NUM_BINS, num_beams=NUM_BEAMS,
            range_min=RANGE_MIN, range_max=RANGE_MAX, horizontal_fov_deg=FOV_DEG)
        assert row[0] == pytest.approx(r_idx, abs=1e-6)
        assert col[0] == pytest.approx(b_idx, abs=1e-6)


def test_the_two_range_conventions_differ_by_at_most_one_bin(sem):
    """두 관습이 갈려 있다는 사실 자체를 고정한다.

    `feature_extraction` 은 `/(num_bins−1)`, `ray_processor` 는 `/num_bins` 로
    나눈다. 한쪽으로 통일하는 것은 이 변경의 범위가 아니므로, 차이가 한 bin
    안이라는 것만 못 박아 둔다 — 누가 한쪽을 '고치면' 여기서 걸린다.
    """
    res = (RANGE_MAX - RANGE_MIN) / NUM_BINS
    rows = np.arange(NUM_BINS)
    fe_range = RANGE_MAX - (rows / (NUM_BINS - 1)) * (RANGE_MAX - RANGE_MIN)
    cpp_range = RANGE_MAX - rows * res
    # 최대 차이는 마지막 bin 에서 정확히 한 bin(res) 이다.
    assert np.max(np.abs(fe_range - cpp_range)) == pytest.approx(res)


# ------------------------------------------------------------ pending queue


def _queue(sem, delta_ms=50, timeout_s=3.0):
    return sem.PendingSemantic(int(delta_ms * 1e6), int(timeout_s * 1e9))


def test_detection_arriving_after_its_keyframe_is_matched(sem):
    q = _queue(sem)
    assert q.offer_keyframe(1_000_000_000, "kf0") is None
    assert q.offer_detection(1_000_000_000, "det0") == "kf0"
    assert len(q) == 0


def test_detection_arriving_before_its_keyframe_is_matched(sem):
    q = _queue(sem)
    assert q.offer_detection(2_000_000_000, "det0") is None
    assert q.offer_keyframe(2_000_000_000, "kf0") == "det0"
    assert len(q) == 0


def test_two_keyframes_of_lag_still_matches_the_right_one(sem):
    q = _queue(sem)
    q.offer_keyframe(1_000_000_000, "kf0")
    q.offer_keyframe(2_000_000_000, "kf1")
    q.offer_keyframe(3_000_000_000, "kf2")
    assert q.offer_detection(1_000_000_000, "det") == "kf0"
    assert len(q) == 2


def test_stamp_outside_the_tolerance_does_not_match(sem):
    q = _queue(sem, delta_ms=50)
    q.offer_keyframe(1_000_000_000, "kf0")
    assert q.offer_detection(1_000_000_000 + 60_000_000, "det") is None
    assert len(q) == 2  # 둘 다 미결로 남는다


def test_empty_detection_is_a_normal_match(sem):
    """검출 0건도 소비돼야 '없었다'와 '안 왔다'가 갈린다."""
    q = _queue(sem)
    q.offer_keyframe(1_000_000_000, "kf0")
    assert q.offer_detection(1_000_000_000, []) == "kf0"


def test_duplicate_stamp_is_visible_before_it_overwrites(sem):
    q = _queue(sem)
    q.offer_detection(1_000_000_000, "det_a")
    assert q.has_detection(1_000_000_000)
    q.offer_detection(1_000_000_000, "det_b")
    assert q.offer_keyframe(1_000_000_000, "kf0") == "det_b"


def test_watermark_expires_unmatched_entries_on_both_sides(sem):
    q = _queue(sem, timeout_s=3.0)
    q.offer_keyframe(1_000_000_000, "kf_old")
    q.offer_detection(1_100_000_000, "det_old")   # 허용오차 밖 → 짝 안 맞음
    q.offer_keyframe(9_000_000_000, "kf_new")
    kfs, dets = q.expire(9_000_000_000)
    assert kfs == ["kf_old"] and dets == ["det_old"]
    assert len(q) == 1


def test_nothing_expires_before_the_watermark(sem):
    q = _queue(sem, timeout_s=3.0)
    q.offer_keyframe(1_000_000_000, "kf0")
    assert q.expire(3_500_000_000) == ([], [])
    assert len(q) == 1
