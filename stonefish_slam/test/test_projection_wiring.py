# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""투영 규약이 ICP 점군과 FFT 직교영상 **양쪽에** 배선돼 있는지 AST 로 고정한다.

`sonar.projection` 은 두 소비자가 있다. FeatureExtraction 은 파라미터를 직접
읽지만, FFTLocalizer 는 노드를 모르고 `OculusProperty` 만 들고 있어서 slam.py 가
값을 넘겨줘야 한다. 실제로 그 한 줄을 빼먹은 채 실험 런을 한 번 태웠다 — 점군만
투영되고 FFT 는 legacy 로 남아 시드와 점군이 다른 좌표계였는데, 아무것도 실패하지
않았고 로그는 `proj=altitude` 라고 정직하게 찍혔다. 조용히 반쪽만 도는 배선이다.

slam.py 는 import-time 에 rclpy·gtsam·cv_bridge 를 끌어와 path-load 가 닿지
않으므로(CONVENTIONS §2.8) 여기서는 소스를 AST 로 읽어 배선의 존재만 본다.
"""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PARAM = "sonar.projection"


def _tree(rel):
    return ast.parse((ROOT / rel).read_text(encoding="utf-8"), filename=rel)


def _declared_params(tree):
    return {n.args[0].value for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == "declare_parameter" and n.args
            and isinstance(n.args[0], ast.Constant)}


def _get_parameter_names(tree):
    return {n.args[0].value for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == "get_parameter" and n.args
            and isinstance(n.args[0], ast.Constant)}


def test_the_parameter_is_declared_by_both_nodes():
    """선언이 없으면 노드가 ParameterNotDeclaredException 으로 죽는다."""
    for rel in ("core/slam.py", "nodes/feature_extraction_node.py"):
        assert PARAM in _declared_params(_tree(rel)), f"{rel} 이 {PARAM} 을 선언하지 않는다"


def test_feature_extraction_reads_the_parameter():
    assert PARAM in _get_parameter_names(_tree("core/feature_extraction.py"))


def test_slam_hands_the_projection_to_the_fft_localizer():
    """`oculus.projection = get_parameter('sonar.projection')` 이 있어야 한다.

    읽기만으로는 부족하다 — 파라미터를 읽어 로그로 흘리고 oculus 에 안 넣으면
    FFT 는 legacy 로 남는다. 대입의 오른쪽이 이 파라미터인 것까지 확인한다.
    """
    wired = False
    for node in ast.walk(_tree("core/slam.py")):
        if not isinstance(node, ast.Assign):
            continue
        target = node.targets[0]
        if not (isinstance(target, ast.Attribute) and target.attr == "projection"
                and isinstance(target.value, ast.Attribute)
                and target.value.attr == "oculus"):
            continue
        if PARAM in {c.args[0].value for c in ast.walk(node.value)
                     if isinstance(c, ast.Call) and isinstance(c.func, ast.Attribute)
                     and c.func.attr == "get_parameter" and c.args
                     and isinstance(c.args[0], ast.Constant)}:
            wired = True
    assert wired, (
        "slam.py 가 oculus.projection 에 sonar.projection 을 대입하지 않는다 — "
        "점군만 투영되고 FFT 직교영상은 legacy 로 남는다")


def test_the_oculus_property_carries_a_default():
    """slam 이 아닌 소비자(standalone·테스트)도 속성이 있어야 한다."""
    src = (ROOT / "utils/sonar.py").read_text(encoding="utf-8")
    assert "self.projection" in src and "self.altitude_m" in src
