# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""역워프 재시도가 쓰는 기하 변환을 고정한다.

`FFTLocalizer.polar_warp` 는 1 패스 추정 (tx, ty, dyaw) 을 되감아 프레임2 극좌표
영상을 프레임1 격자로 리샘플한다. 게이트가 기각한 쌍에만 쓰이고, 그 쌍의 38% 를
구제한다(측정은 .hq/community/posts/finding/023).

부호 규약이 틀리면 워프가 오차를 **키우는데** 그 형태가 "재시도가 아무것도 못
건짐"으로만 드러난다 — 조용한 실패라 여기서 못 박는다. 검사는 셋이다.
  1. 영-변환 워프는 항등이다.
  2. 전방 병진 tx>0 은 표적을 **더 멀리** 놓는다. 직관과 반대라 못 박는다 —
     워프는 "프레임2 내용이 프레임1 격자에서 어디 놓이나"를 답한다. 프레임2(나중,
     가까움)에서 16 m 인 표적은 3 m 전진 전인 프레임1 에서 19 m 였다. 행 0 이 far
     이므로 행 인덱스가 **작아진다**. 이 방향을 뒤집으면 워프가 오차를 키운다.
  3. 요각 회전은 방위(열)를 옮기고, +요각과 -요각이 반대쪽으로 간다.
"""
import sys
import types
from pathlib import Path

import numpy as np
import pytest

REL = "stonefish_slam/core/localization_fft.py"


class _Oculus:
    """polar_warp 가 읽는 기하 파라미터만 가진 최소 스텁."""
    range_resolution = 40.0 / 500
    horizontal_fov = np.radians(130.0)
    altitude_m = 6.4


@pytest.fixture
def warper(load_module):
    preexisting = set(sys.modules)
    for name in ("stonefish_slam", "stonefish_slam.utils", "stonefish_slam.utils.sonar"):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
    sys.modules["stonefish_slam.utils.sonar"].OculusProperty = object
    try:
        cls = load_module(REL, "localization_fft_polarwarp").FFTLocalizer
        f = cls.__new__(cls)
        f.oculus = _Oculus()
        yield f
    finally:
        for name in set(sys.modules) - preexisting:
            del sys.modules[name]


def _blob(rows=500, cols=512, row=300, col=256, half=6):
    img = np.zeros((rows, cols), dtype=np.float32)
    img[row - half:row + half, col - half:col + half] = 1.0
    return img


def _centroid(img):
    m = img > 0.05
    if not m.any():
        return None
    rr, cc = np.nonzero(m)
    return rr.mean(), cc.mean()


def test_zero_warp_is_identity(warper):
    img = _blob()
    assert np.allclose(warper.polar_warp(img, 0.0, 0.0, 0.0), img)


def test_forward_translation_places_target_farther_in_frame1(warper):
    """전진했으면 그 표적은 이전 프레임에서 더 멀리 있었다 → 행 인덱스가 작아진다."""
    img = _blob()
    r0, _ = _centroid(img)
    r1, _ = _centroid(warper.polar_warp(img, 3.0, 0.0, 0.0))
    assert r1 < r0 - 1.0, (
        f"전진 3 m 인데 행이 {r0:.1f}→{r1:.1f} — 커졌다면 병진 부호가 뒤집혔다")


def test_yaw_shifts_bearing_column_and_sign_matches_bearing_from_yaw(warper):
    """요각은 방위 열을 옮기고, +요각과 -요각은 반대쪽으로 가야 한다."""
    img = _blob()
    _, c0 = _centroid(img)
    _, cp = _centroid(warper.polar_warp(img, 0.0, 0.0, +8.0))
    _, cn = _centroid(warper.polar_warp(img, 0.0, 0.0, -8.0))
    assert abs(cp - c0) > 2.0, f"요각 +8° 인데 열이 {c0:.1f}→{cp:.1f}"
    assert (cp - c0) * (cn - c0) < 0, (
        f"+8°/-8° 가 같은 쪽으로 갔다 (열 {c0:.1f} → {cp:.1f} / {cn:.1f}) — 부호 규약 오류")
