# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""병진 위상상관 저역통과의 두 성질을 못 박는다.

`trans_lowpass` 는 교차스펙트럼에 이진 마스크를 **한 번** 곱해 고주파(스페클)를
죽인다. 두 영상을 각각 역FFT 로 필터하는 것과 동치인데, 그 동치는 마스크가
이진이라 `F1·M · conj(F2·M) = F1·conj(F2)·M²= F1·conj(F2)·M` 이기 때문이다.
마스크가 이진이 아니게 되는 순간(가장자리 완화 같은 것) 그 동치가 깨지고,
온라인 결과가 오프라인에서 채택한 수치와 조용히 갈라진다 — 그래서 여기서 잰다.

측정 근거는 .hq/community/posts/finding/024 (tilt 30° 188 쌍: 대조군
83.0%/0.152 m → 0.50 에서 89.9%/0.132 m).
"""
import sys
import types

import numpy as np
import pytest

REL = "stonefish_slam/core/localization_fft.py"


@pytest.fixture
def loc(load_module):
    preexisting = set(sys.modules)
    for name in ("stonefish_slam", "stonefish_slam.utils", "stonefish_slam.utils.sonar"):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
    sys.modules["stonefish_slam.utils.sonar"].OculusProperty = object
    try:
        cls = load_module(REL, "localization_fft_lowpass").FFTLocalizer
        f = cls.__new__(cls)
        f._lp_cache = {}
        yield f
    finally:
        for name in set(sys.modules) - preexisting:
            del sys.modules[name]


def test_mask_is_binary_and_idempotent(loc):
    """이진이라야 교차스펙트럼 한 번 곱하기가 두 영상 필터링과 동치다."""
    loc.trans_lowpass = 0.5
    m = loc._lowpass_mask((64, 96))
    assert set(np.unique(m)) <= {0.0, 1.0}, "마스크가 이진이 아니면 동치가 깨진다"
    assert np.array_equal(m * m, m)
    assert m[0, 0] == 1.0, "DC 는 항상 통과해야 한다"
    assert m.any() and not m.all(), "0.5 에서는 통과·차단이 모두 있어야 한다"


def test_cutoff_is_monotonic_in_kept_fraction(loc):
    """차단주파수를 올리면 통과 대역이 넓어진다 — 반경 규약이 뒤집히지 않았는지."""
    kept = []
    for c in (0.2, 0.5, 0.9):
        loc.trans_lowpass = c
        loc._lp_cache.clear()
        kept.append(loc._lowpass_mask((64, 96)).mean())
    assert kept[0] < kept[1] < kept[2], f"통과 비율이 단조가 아니다: {kept}"
