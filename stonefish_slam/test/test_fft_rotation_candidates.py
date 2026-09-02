# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""회전 가설 여러 개 중 병진 상관이 가장 강한 것을 고르는 경로를 고정한다.

tilt 30° 에서 극좌표 상관면의 전역 최대가 정답 회전인 쌍은 43% 뿐이고, 정답은 상위
피크 안에 있다 — 거리 변위와 방위 변위를 한 상관면이 동시에 설명해야 해서 피크가
끌려가기 때문이다. 어느 후보가 옳은지는 그 회전으로 병진을 풀어 봐야 답이 나온다.
측정은 .hq/community/posts/finding/019.

여기서 고정하는 것은 두 가지다.
  1. `rotation_candidates=1` 이면 옛 동작(전역 최대)과 **정확히** 같아야 한다.
  2. `rotation_candidates>1` 이면 병진 피크가 가장 큰 후보가 선택되고, 공분산의
     회전 분산도 **그 후보의 것**이어야 한다 — 전역 최대의 분산을 쓰면 다른 피크를
     골랐을 때 팩터 그래프가 엉뚱한 신뢰도를 받는다.

배선(yaml → 생성자)은 AST 로 따로 본다. slam.py 는 import-time 에 rclpy 를 끌어와
path-load 가 닿지 않는다(CONVENTIONS §2.8).
"""
import ast
import sys
import types
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
REL = "stonefish_slam/core/localization_fft.py"
PARAM = "fft_localization.rotation_candidates"


# ──────────────────────────── 동작 ────────────────────────────

@pytest.fixture
def localizer(load_module):
    preexisting = set(sys.modules)
    for name in ("stonefish_slam", "stonefish_slam.utils", "stonefish_slam.utils.sonar"):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
    sys.modules["stonefish_slam.utils.sonar"].OculusProperty = object
    try:
        yield load_module(REL, "localization_fft_rotcand").FFTLocalizer
    finally:
        for name in set(sys.modules) - preexisting:
            del sys.modules[name]


def _stub(cls, candidates, translations):
    """estimate_rotation·estimate_translation 을 대체한 최소 인스턴스.

    translations 는 회전각 -> (병진, 병진피크) 표. 실제 상관 대신 이 표를 읽으므로
    선택 논리만 남는다.
    """
    f = cls.__new__(cls)
    f.rotation_candidates = len(candidates)
    f.verbose = False
    f.remove_radial_mean = False
    f.apply_range_min_mask = lambda img: img
    f.polar_to_cartesian = lambda img: img
    f.estimate_rotation = lambda a, b: {
        'rotation': candidates[0][0], 'peak_value': candidates[0][1],
        'variance_theta': candidates[0][2], 'success': True,
        **({'candidates': candidates} if len(candidates) > 1 else {}),
    }

    def _trans(c1, c2, rotation=0.0):
        t, peak = translations[round(rotation, 6)]
        return {'translation': list(t), 'peak_value': peak,
                'variance_x': 0.1, 'variance_y': 0.2, 'success': True}
    f.estimate_translation = _trans
    return f


IMG = np.zeros((8, 8), np.float32)


def test_one_candidate_keeps_the_global_maximum(localizer):
    """옛 동작 보존 — 후보가 하나면 전역 최대를 그대로 쓴다."""
    f = _stub(localizer, [(3.0, 0.9, 0.04)], {3.0: ((1.0, 2.0), 0.5)})
    out = f.estimate_transform(IMG, IMG)
    assert out['rotation'] == pytest.approx(3.0)
    assert out['translation'] == [1.0, 2.0]
    assert out['covariance'][2, 2] == pytest.approx(0.04)


def test_the_strongest_translation_peak_wins(localizer):
    """전역 최대(3.0°)보다 2번째 후보(-1.5°)의 병진 상관이 강하면 그쪽이 이긴다."""
    cands = [(3.0, 0.9, 0.04), (-1.5, 0.8, 0.01), (7.25, 0.7, 0.09)]
    trans = {3.0: ((9.0, 9.0), 0.20),
             -1.5: ((1.0, 2.0), 0.77),
             7.25: ((5.0, 5.0), 0.31)}
    out = _stub(localizer, cands, trans).estimate_transform(IMG, IMG)
    assert out['rotation'] == pytest.approx(-1.5)
    assert out['translation'] == [1.0, 2.0]
    assert out['trans_peak'] == pytest.approx(0.77)
    assert out['rot_peak'] == pytest.approx(0.8), "선택된 후보의 상관값이어야 한다"
    assert out['covariance'][2, 2] == pytest.approx(0.01), \
        "회전 분산이 전역 최대(0.04)의 것이면 팩터 그래프가 틀린 신뢰도를 받는다"
    assert out['rotation_fft'] == pytest.approx(3.0), "FFT 원본 추정은 따로 남아야 한다"


def test_the_override_still_bypasses_the_search(localizer):
    """rotation_override(=DR 회전)는 후보 탐색을 건너뛴다."""
    cands = [(3.0, 0.9, 0.04), (-1.5, 0.8, 0.01)]
    trans = {3.0: ((9.0, 9.0), 0.20), -1.5: ((1.0, 2.0), 0.77), 0.5: ((4.0, 4.0), 0.05)}
    out = _stub(localizer, cands, trans).estimate_transform(IMG, IMG, rotation_override=0.5)
    assert out['rotation'] == pytest.approx(0.5)
    assert out['translation'] == [4.0, 4.0]
    assert out['rotation_override_used'] is True


def test_peaks_are_separated_and_ordered(localizer):
    """_rotation_peaks 는 서로 떨어진 봉우리를 값 내림차순으로 준다."""
    f = localizer.__new__(localizer)
    f.rotation_candidates = 3
    f.compute_peak_variance = lambda pcm, loc, res: (0.0, 1.0)
    pcm = np.zeros((21, 41), np.float64)
    pcm[10, 20] = 0.5           # 중앙 = 0°
    pcm[10, 30] = 0.9           # +10 열
    pcm[3, 8] = 0.7
    peaks = f._rotation_peaks(pcm, deg_per_col=0.25, separation=4)
    assert [round(p[0], 6) for p in peaks] == [2.5, -3.0, 0.0]
    assert [p[1] for p in peaks] == sorted((p[1] for p in peaks), reverse=True)


# ──────────────────────────── 배선 ────────────────────────────

def _tree(rel):
    return ast.parse((ROOT / rel).read_text(encoding="utf-8"), filename=rel)


def _string_args(tree, attr):
    return {n.args[0].value for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == attr and n.args
            and isinstance(n.args[0], ast.Constant)}


def _call_kwargs(tree, name):
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == name:
            return {k.arg: k.value for k in n.keywords}
    raise AssertionError(f"{name}(...) 호출이 없다")


@pytest.mark.parametrize("rel", ["core/slam.py", "nodes/fft_localization_node.py"])
def test_the_parameter_is_declared_read_and_passed(rel):
    """선언·읽기만으로는 부족하다 — 생성자에 안 넘기면 기본값 1 로 조용히 남는다."""
    tree = _tree(rel)
    assert PARAM in _string_args(tree, "declare_parameter"), f"{rel} 이 {PARAM} 을 선언하지 않는다"
    assert PARAM in _string_args(tree, "get_parameter"), f"{rel} 이 {PARAM} 을 읽지 않는다"
    passed = _call_kwargs(tree, "FFTLocalizer")
    assert "rotation_candidates" in passed, f"{rel} 이 rotation_candidates 를 안 넘긴다"
    assert isinstance(passed["rotation_candidates"], ast.Name)


def test_the_yaml_turns_it_on():
    """기본값 1 은 옛 동작이므로, yaml 이 켜지 않으면 개선이 배포되지 않는다."""
    cfg = yaml.safe_load((ROOT.parent / "config/slam.yaml").read_text(encoding="utf-8"))
    node = next(iter(cfg.values()))["ros__parameters"]
    assert node["fft_localization"]["rotation_candidates"] > 1
