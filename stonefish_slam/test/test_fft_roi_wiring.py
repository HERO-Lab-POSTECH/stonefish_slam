# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""ROI 크롭이 yaml 에서 FFTLocalizer 생성자까지 실제로 닿는지 AST 로 고정한다.

`_apply_roi`/`_compute_roi` 는 처음부터 구현돼 있었지만 `use_roi` 기본값이 False 였고
slam.py 가 그 인자를 **아예 넘기지 않아** 죽은 경로였다. docstring 은 "(default: True)"
라고 반대로 적혀 있어서, 코드를 읽는 쪽이 켜져 있다고 믿기 좋은 상태였다.

같은 함정이 `sonar.projection` 에서 이미 한 번 실험 런을 태웠다 — 배선 한 줄이 없어도
아무것도 실패하지 않고 로그는 정직하게 찍힌다. 그래서 여기서는 선언·읽기·**생성자
전달**을 따로 본다. slam.py 는 import-time 에 rclpy·gtsam·cv_bridge 를 끌어와
path-load 가 닿지 않으므로(CONVENTIONS §2.8) 소스를 AST 로 읽는다.
"""
import ast
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PARAMS = ("fft_localization.use_roi", "fft_localization.roi_threshold")
KWARGS = {"use_roi": "fft_use_roi", "roi_threshold": "fft_roi_threshold"}


def _slam_tree():
    rel = "core/slam.py"
    return ast.parse((ROOT / rel).read_text(encoding="utf-8"), filename=rel)


def _string_args(tree, attr):
    return {n.args[0].value for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == attr and n.args
            and isinstance(n.args[0], ast.Constant)}


def _fft_localizer_call(tree):
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) \
                and n.func.id == "FFTLocalizer":
            return n
    raise AssertionError("slam.py 에 FFTLocalizer(...) 호출이 없다")


def test_both_parameters_are_declared():
    """선언이 없으면 노드가 ParameterNotDeclaredException 으로 죽는다."""
    declared = _string_args(_slam_tree(), "declare_parameter")
    for p in PARAMS:
        assert p in declared, f"slam.py 가 {p} 을 선언하지 않는다"


def test_both_parameters_are_read():
    read = _string_args(_slam_tree(), "get_parameter")
    for p in PARAMS:
        assert p in read, f"slam.py 가 {p} 을 읽지 않는다"


def test_the_values_reach_the_constructor():
    """읽기만으로는 부족하다 — 생성자에 안 넘기면 ROI 는 기본값 False 로 남는다."""
    call = _fft_localizer_call(_slam_tree())
    passed = {k.arg: k.value for k in call.keywords}
    for kwarg, expected_name in KWARGS.items():
        assert kwarg in passed, f"FFTLocalizer(...) 가 {kwarg} 를 안 받는다"
        value = passed[kwarg]
        assert isinstance(value, ast.Name) and value.id == expected_name, \
            f"{kwarg}= 의 오른쪽이 {expected_name} 이 아니다"


def test_the_localizer_accepts_both_and_defaults_to_off():
    """생성자 시그니처가 두 인자를 갖고, 기본이 꺼짐인 것까지 본다.

    기본이 켜짐으로 바뀌면 이건 실험 토글이 아니라 동작 변경이므로 PR 이 갈린다.
    """
    rel = "core/localization_fft.py"
    tree = ast.parse((ROOT / rel).read_text(encoding="utf-8"), filename=rel)
    init = next(n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef) and n.name == "__init__")
    names = [a.arg for a in init.args.args]
    for kwarg in KWARGS:
        assert kwarg in names, f"FFTLocalizer.__init__ 에 {kwarg} 가 없다"
    # 기본값은 args 뒤쪽에 정렬돼 붙는다.
    defaults = dict(zip(names[len(names) - len(init.args.defaults):],
                        init.args.defaults))
    assert defaults["use_roi"].value is False, "use_roi 기본값이 꺼짐이 아니다"


def test_yaml_ships_both_keys_with_roi_off():
    """yaml 에 없으면 실험자가 override 할 손잡이가 없다."""
    cfg = yaml.safe_load((ROOT.parent / "config/slam.yaml").read_text(encoding="utf-8"))
    fft = cfg["/**"]["ros__parameters"]["fft_localization"]
    assert fft["use_roi"] is False
    assert isinstance(fft["roi_threshold"], float)
