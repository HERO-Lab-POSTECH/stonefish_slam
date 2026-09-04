# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""DFT 서브픽셀 정제가 상관 표면의 *역* 변환 위에서 도는지 못 박는다.

`_upsampled_dft` 는 비이동 상호전력 스펙트럼에 음지수 커널을 곱한다. 그것만으로는
**순**변환이라 상관 표면의 거울상이 나오고, 정제가 +offset 근방을 뒤지면 실제 피크가
없는 자리를 뒤진다. conj(F(conj(x))) = N·F^-1(x) 이므로 입출력을 켤레로 감싸야 한다
(scikit-image 의 `_phase_cross_correlation` 이 같은 커널에 같은 켤레를 쓰는 이유).

이 결함은 조용하다 — 예외도 경고도 없고 정제된 값이 그냥 0.55 px 짧아진다. 오프라인
합성 실측으로 `dft_refinement_enable=true` 만 -0.042~-0.049 m 의 덧셈 편향을 내고
켤레를 넣으면 off 와 자릿수까지 같아졌다(.hq finding/037).

여기서는 이상적인 상호전력 스펙트럼(알려진 서브픽셀 이동)을 만들어, 정제 결과가
평범한 역FFT 피크와 **같은 방향으로** 그 이동을 회복하는지 본다. 부호 규약은
하드코딩하지 않고 역FFT 피크에서 읽는다 — 규약이 바뀌어도 이 테스트는 결함만 잡는다.
"""
import sys
import types

import numpy as np
import pytest

REL = "stonefish_slam/core/localization_fft.py"
H, W = 64, 96
TRUE_DR, TRUE_DC = 7.3, -4.6


@pytest.fixture
def loc(load_module):
    preexisting = set(sys.modules)
    for name in ("stonefish_slam", "stonefish_slam.utils", "stonefish_slam.utils.sonar"):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
    sys.modules["stonefish_slam.utils.sonar"].OculusProperty = object
    try:
        cls = load_module(REL, "localization_fft_dft").FFTLocalizer
        f = cls.__new__(cls)
        f.dft_upsample_factor = 100
        f.verbose = False
        yield f
    finally:
        for name in set(sys.modules) - preexisting:
            del sys.modules[name]


def _ideal_cps():
    """(TRUE_DR, TRUE_DC) 만큼 이동한 쌍의 정규화 상호전력 스펙트럼."""
    u = np.fft.fftfreq(H)[:, None] * H
    v = np.fft.fftfreq(W)[None, :] * W
    return np.exp(2j * np.pi * (u * TRUE_DR / H + v * TRUE_DC / W))


def _integer_peak(cps):
    """detect_peak 와 같은 방식으로 정수 격자 피크를 읽는다."""
    pcm = np.fft.fftshift(np.abs(np.fft.ifft2(cps)))
    r, c = np.unravel_index(np.argmax(pcm), pcm.shape)
    return r - H // 2, c - W // 2


def test_refinement_recovers_subpixel_shift(loc):
    """정제가 정수 시작점에서 참 서브픽셀 위치로 수렴해야 한다."""
    cps = _ideal_cps()
    r0, c0 = _integer_peak(cps)
    # 부호 규약은 역FFT 가 정한다 — 그것을 기준으로 참값을 맞춘다.
    srow = 1.0 if r0 == round(TRUE_DR) else -1.0
    scol = 1.0 if c0 == round(TRUE_DC) else -1.0

    rr, cc = loc._dft_subpixel_refinement(cps, float(r0), float(c0))

    step = 1.0 / loc.dft_upsample_factor
    assert abs(rr - srow * TRUE_DR) < 2 * step, (
        f"행 정제가 빗나갔다: {rr} vs {srow * TRUE_DR} "
        "(켤레 누락이면 거울상 표면을 뒤져 창 가장자리로 밀린다)")
    assert abs(cc - scol * TRUE_DC) < 2 * step, (
        f"열 정제가 빗나갔다: {cc} vs {scol * TRUE_DC}")


def test_refinement_beats_the_integer_start(loc):
    """정제는 최소한 정수 시작점보다 참값에 가까워야 한다 — 멀어지면 방향이 뒤집힌 것이다."""
    cps = _ideal_cps()
    r0, c0 = _integer_peak(cps)
    srow = 1.0 if r0 == round(TRUE_DR) else -1.0
    scol = 1.0 if c0 == round(TRUE_DC) else -1.0

    rr, cc = loc._dft_subpixel_refinement(cps, float(r0), float(c0))

    assert abs(rr - srow * TRUE_DR) < abs(r0 - srow * TRUE_DR), "행 정제가 오히려 멀어졌다"
    assert abs(cc - scol * TRUE_DC) < abs(c0 - scol * TRUE_DC), "열 정제가 오히려 멀어졌다"


def test_upsampled_dft_equals_scaled_inverse_fft(loc):
    """정수 위치에서 `_upsampled_dft` 는 H*W*ifft2 와 **정확히** 같아야 한다.

    conj(F(conj(x))) = N*F^-1(x) 이므로 등식이 성립한다. 켤레가 빠지면 같은 자리에서
    순변환 값이 나와 등식이 깨진다 — 정제 경로를 타지 않고 변환 방향만 보는 검사다.
    배율 1 이면 표본 위치가 정수라 역FFT 격자와 곧바로 맞댈 수 있다.
    """
    cps = _ideal_cps()
    r0, c0 = _integer_peak(cps)
    region = loc._upsampled_dft(cps, 1, float(r0), float(c0))

    size = region.shape[0]
    grid = np.arange(size) - size // 2
    rows = (r0 + grid).astype(int) % H
    cols = (c0 + grid).astype(int) % W
    expected = (np.fft.ifft2(cps) * (H * W))[np.ix_(rows, cols)]

    assert np.allclose(region, expected, atol=1e-8), (
        "업샘플 DFT 가 역FFT 와 다르다 — 켤레가 빠져 순변환을 계산하고 있다")
