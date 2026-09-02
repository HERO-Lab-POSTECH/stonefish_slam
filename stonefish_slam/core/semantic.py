#!/usr/bin/env python3
"""검출(bbox) ↔ 소나 픽셀 ↔ 방위/거리 변환과, 검출을 기다리는 미결 큐.

ROS·gtsam·OpenCV 를 import 하지 않는다. 같은 일을 `feature_extraction.py` 안에
두면 그 모듈이 상단에서 `cv_bridge`·`cfar` 를 끌어오는 탓에 루트 `conftest.py` 의
`load_module` fixture 로 열 수 없고, ROS 없는 CI 러너에서 테스트가 통째로 빠진다.

**거리↔행 변환이 두 가지인 것은 실수가 아니다.** 이 repo 에는 서로 다른 두
관습이 이미 공존한다:

* `feature_extraction.extract_features` — `range_max − row/(num_bins−1)·(range_max−range_min)`
* `cpp/ray_processor.cpp` · `core/mapping_3d.py` — `range_max − r_idx·(range_max−range_min)/num_bins`

500 bin·0.5~40 m 에서 둘의 차이는 최대 한 bin(≈0.079 m)이다. 랜드마크 factor 의
측정값은 pose graph·ICP 가 사는 기하(=`Keyframe.points`)와 같아야 하므로 전자를,
3D 복셀 라벨의 역투영은 C++ 가 복셀을 놓은 기하를 되짚어야 하므로 후자를 쓴다.
둘을 하나로 합치는 것은 이 계획의 범위가 아니다 — 어느 쪽을 정본으로 삼을지는
`Keyframe.points` 와 OctoMap 을 같이 쓰는 소비자가 정할 문제다.
"""

import numpy as np


def labels_from_detections(peak_locs: np.ndarray, dets: np.ndarray) -> np.ndarray:
    """CFAR 피크 픽셀마다 그것을 덮는 bbox 의 클래스 라벨을 붙인다.

    Args:
        peak_locs (np.ndarray): (N, 2) `[row, col]` 픽셀 좌표
            (`np.argwhere` 출력 순서 그대로 — `Keyframe.points` 와 1:1).
        dets (np.ndarray): (K, 6) `[class, conf, x1, y1, x2, y2]`.
            x 는 col(bearing 축), y 는 row(range bin 축). 경계 포함.

    Returns:
        np.ndarray: (N,) uint8. bbox 안이면 `class + 1`, 밖이면 0 — 0 이
        "라벨 없음"이라 클래스 0 과 겹치지 않게 1 을 더한다. bbox 가 겹치면
        `dets` 의 **나중 것이 이긴다**.
    """
    n = len(peak_locs)
    labels = np.zeros(n, dtype=np.uint8)
    if n == 0 or len(dets) == 0:
        return labels

    rows = np.asarray(peak_locs)[:, 0]
    cols = np.asarray(peak_locs)[:, 1]
    # ponytail: K(프레임당 검출 수)는 한 자릿수라 K 루프 × N 벡터화면 충분하다.
    for cls, _conf, x1, y1, x2, y2 in np.asarray(dets, dtype=np.float64):
        inside = (cols >= x1) & (cols <= x2) & (rows >= y1) & (rows <= y2)
        labels[inside] = np.uint8(int(cls) + 1)
    return labels


def detection_rows(items, min_conf: float):
    """검출 메시지에서 뽑은 값들을 Kx6 배열로 — 필터링 규칙이 사는 곳.

    ROS 메시지를 직접 받지 않는다(이 모듈은 ROS 를 모른다). 호출자가
    `vision_msgs/Detection2D` 에서 필요한 여섯 값만 뽑아 넘긴다.

    Args:
        items: `(class_id, score, center_x, center_y, size_x, size_y)` 의 시퀀스.
            `class_id` 는 문자열이다 — `vision_msgs` 4.1.1 의 `ObjectHypothesis`
            가 그렇게 정의돼 있다.
        min_conf (float): 이 미만 신뢰도는 버린다.

    Returns:
        tuple: `(rows, n_below_conf, n_bad_class)`.
        `rows` 는 (K, 6) float32 `[class, conf, x1, y1, x2, y2]`(bbox 모서리).
        `class_id` 가 정수 문자열이 아니면 버리고 `n_bad_class` 로 센다 —
        표준상 그 필드는 `VisionInfo` DB 의 키라 발행자가 이름을 실을 수도
        있고, 그런 값은 이 파이프라인이 정수 라벨로 쓸 수 없다.
    """
    rows, n_below_conf, n_bad_class = [], 0, 0
    for class_id, score, cx, cy, size_x, size_y in items:
        if float(score) < min_conf:
            n_below_conf += 1
            continue
        try:
            cls = int(class_id)
        except (TypeError, ValueError):
            n_bad_class += 1
            continue
        half_x, half_y = float(size_x) / 2.0, float(size_y) / 2.0
        rows.append([cls, float(score),
                     float(cx) - half_x, float(cy) - half_y,
                     float(cx) + half_x, float(cy) + half_y])
    if not rows:
        return np.zeros((0, 6), np.float32), n_below_conf, n_bad_class
    return np.array(rows, np.float32), n_below_conf, n_bad_class


def pixel_to_bearing_range(
    row, col, num_bins: int, num_beams: int,
    range_min: float, range_max: float, horizontal_fov_deg: float,
):
    """소나 픽셀을 (방위, 거리) 로 — `feature_extraction` 과 같은 기하.

    랜드마크 factor 의 측정값이 `Keyframe.points` 와 같은 좌표계에 놓이도록
    `extract_features` 의 변환식을 그대로 쓴다(모듈 docstring 참조).

    Args:
        row: range bin 인덱스. row=0 이 range_max(먼 쪽), row=num_bins−1 이 range_min.
        col: beam 인덱스. col=0 이 −FOV/2, col=num_beams−1 이 +FOV/2.
        num_bins (int): 이미지 높이(range bin 수).
        num_beams (int): 이미지 폭(beam 수).
        range_min (float): m.
        range_max (float): m.
        horizontal_fov_deg (float): 도.

    Returns:
        tuple: `(bearing_rad, range_m)`. 입력이 배열이면 배열로 나온다.
    """
    range_m = range_max - (np.asarray(row, dtype=np.float64) / (num_bins - 1)) * (
        range_max - range_min
    )
    bearing_rad = np.radians(
        -horizontal_fov_deg / 2.0
        + (np.asarray(col, dtype=np.float64) / (num_beams - 1)) * horizontal_fov_deg
    )
    return bearing_rad, range_m


def slant_range_bearing(points_sonar: np.ndarray):
    """소나 프레임 3D 점을 (경사거리, 방위) 로.

    `cpp/ray_processor.cpp` 는 복셀을
    `x = r·cos(v)·cos(b)`, `y = r·cos(v)·sin(b)`, `z = r·sin(v)` 로 놓는다.
    따라서 `|P| = r`(수평거리가 아니라 **경사거리**)이고 `b = atan2(y, x)` 다.
    `mapping_3d._voxel_to_sonar_coords` 는 `sqrt(x²+y²)`(수평거리)를 내므로
    앙각이 0 이 아닌 복셀에서는 다른 값이며, 역투영에 쓰면 안 된다.

    Args:
        points_sonar (np.ndarray): (..., 3) 소나 프레임 좌표.

    Returns:
        tuple: `(slant_range, bearing_rad)`, 각각 (...,) 배열.
    """
    p = np.asarray(points_sonar, dtype=np.float64)
    slant = np.linalg.norm(p, axis=-1)
    bearing = np.arctan2(p[..., 1], p[..., 0])
    return slant, bearing


def sonar_to_pixel(
    slant_range, bearing_rad, num_bins: int, num_beams: int,
    range_min: float, range_max: float, horizontal_fov_deg: float,
):
    """(경사거리, 방위) 를 소나 픽셀로 — `cpp/ray_processor.cpp` 와 같은 기하.

    3D 복셀 라벨 역투영 전용이다. C++ 가 `range_resolution =
    (range_max − range_min)/num_bins` 로 복셀을 놓았으므로 그 식을 되짚는다
    (`feature_extraction` 의 `/(num_bins−1)` 이 아니다 — 모듈 docstring 참조).

    Args:
        slant_range: m, `slant_range_bearing` 의 출력.
        bearing_rad: rad, 같음.
        num_bins (int): range bin 수.
        num_beams (int): beam 수.
        range_min (float): m.
        range_max (float): m.
        horizontal_fov_deg (float): 도.

    Returns:
        tuple: `(row, col)` 실수 픽셀 좌표(반올림하지 않는다 — 호출자가
        허용오차와 함께 쓴다). FOV 밖·거리 밖 여부는 호출자가 판정한다.
    """
    range_resolution = (range_max - range_min) / num_bins
    row = (range_max - np.asarray(slant_range, dtype=np.float64)) / range_resolution
    fov_rad = np.radians(horizontal_fov_deg)
    col = (np.asarray(bearing_rad, dtype=np.float64) + fov_rad / 2.0) / fov_rad * (
        num_beams - 1
    )
    return row, col


class PendingSemantic:
    """검출과 키프레임을 stamp 로 **정확히 한 번** 짝지어 주는 큐.

    YOLO 노드는 추론 중 들어온 프레임을 드랍하고(`busy`), 추론 지연 때문에
    검출이 그 키프레임보다 늦게 도착한다. 그래서 어느 쪽이 먼저 오든 상관없게
    양쪽을 각각 담아 두고, 짝이 맞는 순간 둘 다 꺼낸다.

    시간은 나노초 정수로만 다룬다 — ROS 메시지 타입을 이 모듈에 들이지 않기
    위해서다(변환은 호출자 몫).

    ponytail: 짝 찾기는 dict 선형 스캔이다. 큐에는 `timeout` 초 동안의
    키프레임만 남으므로(기본 3 s × 1 Hz) 항목이 한 자릿수다 — 정렬 구조가
    필요해지면 그때 바꾼다.
    """

    def __init__(self, max_stamp_delta_ns: int, timeout_ns: int):
        """Class constructor.

        Args:
            max_stamp_delta_ns (int): 같은 프레임으로 볼 stamp 차이 상한(ns).
            timeout_ns (int): 이 시간이 지나도록 짝을 못 찾으면 만료(ns).
        """
        self.max_stamp_delta_ns = int(max_stamp_delta_ns)
        self.timeout_ns = int(timeout_ns)
        self._keyframes = {}
        self._detections = {}

    def offer_keyframe(self, stamp_ns: int, keyframe):
        """키프레임을 넣는다.

        Args:
            stamp_ns (int): 그 키프레임의 소나 프레임 stamp(ns).
            keyframe: 짝이 맞을 때 돌려받을 값.

        Returns:
            먼저 도착해 있던 검출. 없으면 None(키프레임이 큐에 남는다).
        """
        det = self._pop_closest(self._detections, stamp_ns)
        if det is not None:
            return det
        self._keyframes[stamp_ns] = keyframe
        return None

    def offer_detection(self, stamp_ns: int, detections):
        """검출을 넣는다.

        Args:
            stamp_ns (int): 검출이 나온 소나 프레임 stamp(ns).
            detections: 짝이 맞을 때 돌려받을 값.

        Returns:
            먼저 도착해 있던 키프레임. 없으면 None(검출이 큐에 남는다).
        """
        kf = self._pop_closest(self._keyframes, stamp_ns)
        if kf is not None:
            return kf
        self._detections[stamp_ns] = detections
        return None

    def has_detection(self, stamp_ns: int) -> bool:
        """같은 stamp 의 미결 검출이 이미 큐에 있는가 (중복 계수용)."""
        return stamp_ns in self._detections

    def expire(self, now_ns: int):
        """워터마크를 넘긴 미결 항목을 빼낸다.

        Args:
            now_ns (int): 현재 소나 프레임 stamp(ns).

        Returns:
            tuple: `(만료된 키프레임 목록, 만료된 검출 목록)`. 키프레임 쪽은
            "검출이 끝내 안 왔다"(det_missing), 검출 쪽은 "짝지을 키프레임이
            없었다"(det_expired)를 뜻한다.
        """
        cutoff = int(now_ns) - self.timeout_ns
        kfs = [self._keyframes.pop(k) for k in
               [k for k in self._keyframes if k < cutoff]]
        dets = [self._detections.pop(k) for k in
                [k for k in self._detections if k < cutoff]]
        return kfs, dets

    def __len__(self) -> int:
        """미결 항목 총수(키프레임 + 검출)."""
        return len(self._keyframes) + len(self._detections)

    def _pop_closest(self, store: dict, stamp_ns: int):
        """`stamp_ns` 에 가장 가까운 항목을 허용오차 안에서 꺼낸다."""
        best_key, best_delta = None, self.max_stamp_delta_ns
        for key in store:
            delta = abs(key - stamp_ns)
            if delta <= best_delta:
                best_key, best_delta = key, delta
        if best_key is None:
            return None
        return store.pop(best_key)
