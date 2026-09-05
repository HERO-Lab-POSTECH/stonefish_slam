# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""몸체 고정 거리 띠를 빼는 전처리가 실제로 정지 성분만 없애는지 본다.

하향 틸트 FLS 의 바닥 반사는 좁은 거리 띠 안에서만 나오고, 그 띠는 차량과 함께
움직이므로 두 프레임 사이에서 이동하지 않는다. 위상 상관에서는 0 시프트 성분으로
들어가 지형이 만드는 진짜 peak 와 경쟁한다. 극좌표에서 한 행은 등거리이므로 행 평균이
그 포락선이고, 빼면 지형 텍스처만 남아야 한다.
"""
import sys
import types

import numpy as np
import pytest

REL = "stonefish_slam/core/localization_fft.py"


@pytest.fixture
def localizer(load_module):
    preexisting = set(sys.modules)
    for name in ("stonefish_slam", "stonefish_slam.utils", "stonefish_slam.utils.sonar"):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
    sys.modules["stonefish_slam.utils.sonar"].OculusProperty = object
    try:
        module = load_module(REL, "localization_fft_band")
        f = module.FFTLocalizer.__new__(module.FFTLocalizer)
        yield f
    finally:
        for name in set(sys.modules) - preexisting:
            del sys.modules[name]


def _frame(rows=120, cols=64, band=(40, 70), texture_shift=0, rng=None):
    """등거리 행에 걸친 띠 포락선 + 그 위에 얹힌 지형 텍스처."""
    rng = rng or np.random.default_rng(0)
    img = np.zeros((rows, cols), np.float32)
    img[band[0]:band[1], :] += 120.0                      # 정지한 띠
    tex = rng.uniform(0, 60, (rows, cols)).astype(np.float32)
    img[band[0]:band[1], :] += np.roll(tex, texture_shift, axis=0)[band[0]:band[1], :]
    return img


def test_the_stationary_band_is_removed(localizer):
    """행 평균을 빼면 띠의 일정 성분이 사라진다."""
    img = _frame()
    out = localizer.remove_band_envelope(img)
    inside = out[45:65].mean()
    # 띠 안팎의 평균 차가 크게 줄어야 한다 (원본은 120 이상 차이)
    outside = out[10:30].mean()
    assert abs(inside - outside) < 10.0, (
        f"띠가 남아 있다 — 안 {inside:.1f} vs 밖 {outside:.1f}")


def test_texture_survives(localizer):
    """지형 텍스처는 남아야 한다 — 전부 0 으로 만들면 상관이 불가능하다."""
    out = localizer.remove_band_envelope(_frame())
    assert out[40:70].std() > 5.0, "띠 안 텍스처가 통째로 지워졌다"
    assert out.max() > 0


def test_output_is_non_negative(localizer):
    """뒤따르는 정규화·CLAHE·erosion 마스크가 비음수를 전제한다."""
    out = localizer.remove_band_envelope(_frame())
    assert out.min() >= 0.0
    assert out.dtype == np.float32


def test_a_uniform_frame_becomes_empty(localizer):
    """방위각으로 균일한 프레임(정보 0)은 0 이 된다 — 빼는 대상이 맞다는 확인."""
    img = np.tile(np.linspace(0, 200, 120, dtype=np.float32)[:, None], (1, 64))
    assert localizer.remove_band_envelope(img).max() < 1e-3   # float32 반올림 여유
