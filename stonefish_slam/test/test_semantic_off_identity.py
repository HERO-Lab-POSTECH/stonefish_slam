# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""`semantic.enable: false` 가 semantic 이전의 노드와 같은 것을 내보내는지 고정한다.

A/B 검증의 기준선은 "off 런 = 오늘의 런"이라는 전제 위에 서 있다. 그 전제가
조용히 깨지면 on/off 비교가 통째로 무의미해지므로, off 에서 달라질 수 있는
세 가지를 여기서 못 박는다: **`/slam/cloud` 의 PointField 스키마**, **새 토픽·
구독의 존재**, **`[INSTR]` 줄의 형식**.

런타임이 아니라 AST 로 본다. `slam.py`·`conversions.py` 는 import-time 에
rclpy·gtsam·cv_bridge 를 끌어오므로 `load_module` 로도 안 뜨고, ROS 없는 CI
러너(`.github/workflows/ci.yml`)에서는 `importorskip` 이 곧 무검증이다.
"""
import ast
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]

# n2r("PointCloudXYZI") 가 오늘 내보내는 필드. 이 표가 바뀌면 기존 소비자의
# 파싱이 깨지므로, semantic 이 켜졌든 아니든 이쪽은 움직이면 안 된다.
GOLDEN_XYZI = [("x", 0), ("y", 4), ("z", 8), ("i", 12)]
GOLDEN_XYZIL = GOLDEN_XYZI + [("label", 16)]


def _tree(relpath):
    src = (PKG / relpath).read_text(encoding="utf-8")
    return ast.parse(src, filename=relpath), src


def _parents(tree):
    """자식 → 부모 맵. `ast` 는 부모 링크를 안 주므로 직접 만든다."""
    out = {}
    for node in ast.walk(tree):
        for child in ast.iter_child_nodes(node):
            out[child] = node
    return out


def _guarded_by(node, parents, needle):
    """`node` 가 `needle` 을 검사하는 `if` 의 **본문** 안에 있는가."""
    child, cur = node, parents.get(node)
    while cur is not None:
        if isinstance(cur, ast.If) and needle in ast.dump(cur.test):
            # orelse(=off 경로)에 있으면 게이트가 반대로 걸린 것이다.
            if any(child is s or child in ast.walk(s) for s in cur.body):
                return True
        child, cur = cur, parents.get(cur)
    return False


def _pointfields_in_branch(tree, cloud_type):
    """`msg == "<cloud_type>"` 분기가 만드는 PointField 의 (name, offset) 목록."""
    parents = _parents(tree)
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Compare)
                and isinstance(node.comparators[0], ast.Constant)
                and node.comparators[0].value == cloud_type):
            continue
        # elif 사슬이라 `orelse` 에는 다음 분기가 들어 있다. 이 비교식이 test 인
        # If 를 찾아 그 **body 만** 본다 — walk(branch) 면 뒤 분기까지 딸려 온다.
        branch = node
        while branch is not None and not (
                isinstance(branch, ast.If) and node in ast.walk(branch.test)):
            branch = parents.get(branch)
        assert branch is not None
        fields = []
        for stmt in branch.body:
            for call in ast.walk(stmt):
                if not (isinstance(call, ast.Call)
                        and isinstance(call.func, ast.Name)
                        and call.func.id == "PointField"):
                    continue
                kw = {k.arg: k.value for k in call.keywords}
                assert isinstance(kw["datatype"], ast.Attribute), "필드 타입 표기가 바뀌었다"
                assert kw["datatype"].attr == "FLOAT32", "필드 타입이 FLOAT32 가 아니다"
                assert kw["count"].value == 1
                fields.append((kw["name"].value, kw["offset"].value))
        return fields
    raise AssertionError(f'n2r 에 "{cloud_type}" 분기가 없다')


def test_xyzi_schema_is_frozen():
    """off 런이 내보내는 4필드가 그대로여야 한다 — 소비자의 파싱 전제다."""
    tree, _ = _tree("utils/conversions.py")
    assert _pointfields_in_branch(tree, "PointCloudXYZI") == GOLDEN_XYZI


def test_xyzil_adds_exactly_one_field_after_the_xyzi_four():
    """on 스키마는 off 스키마의 확장이어야 한다 — 앞 4필드가 같은 자리."""
    tree, _ = _tree("utils/conversions.py")
    assert _pointfields_in_branch(tree, "PointCloudXYZIL") == GOLDEN_XYZIL


def test_the_labelled_cloud_type_is_only_reachable_when_semantic_is_on():
    """XYZIL 을 무조건 쓰면 off 런의 /slam/cloud 스키마가 바뀐다."""
    tree, _ = _tree("core/slam.py")
    parents = _parents(tree)
    uses = [n for n in ast.walk(tree)
            if isinstance(n, ast.Constant) and n.value == "PointCloudXYZIL"]
    assert uses, 'slam.py 가 "PointCloudXYZIL" 을 쓰지 않는다 — 테스트 전제가 깨졌다'
    for node in uses:
        assert _guarded_by(node, parents, "semantic_enable"), (
            "PointCloudXYZIL 이 semantic_enable 분기 밖에 있다 — off 런의 "
            "/slam/cloud 가 5필드로 바뀐다"
        )


def test_the_semantic_instr_line_is_only_emitted_when_semantic_is_on():
    """`[INSTR] counters` 줄 형식은 A/B 기준선이라 off 에서 늘면 안 된다."""
    tree, _ = _tree("core/slam.py")
    parents = _parents(tree)
    lines = [n for n in ast.walk(tree)
             if isinstance(n, ast.Constant) and isinstance(n.value, str)
             and n.value.startswith("[INSTR] semantic")]
    assert lines, "[INSTR] semantic 줄이 없다 — 테스트 전제가 깨졌다"
    for node in lines:
        assert _guarded_by(node, parents, "semantic_enable")


def test_detection_subscription_lives_behind_the_enable_early_return():
    """구독·vision_msgs import 가 off 런에 존재하면 안 된다.

    `ros2 topic list` 에 새 구독이 잡히는 것도 차이지만, 더 중요한 것은
    `vision_msgs` 가 없는 머신에서도 off 런이 그대로 떠야 한다는 것이다 —
    그래서 import 까지 함수 안에 있다.
    """
    tree, src = _tree("core/slam.py")
    func = next((n for n in ast.walk(tree)
                 if isinstance(n, ast.FunctionDef) and n.name == "_init_semantic"), None)
    assert func is not None, "_init_semantic 을 찾지 못했다"

    # 모듈 최상단에 vision_msgs import 가 없어야 한다.
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            names = getattr(node, "module", "") or ""
            names += " ".join(a.name for a in node.names)
            assert "vision_msgs" not in names, (
                "vision_msgs 를 모듈 최상단에서 import 한다 — 그 패키지가 없는 "
                "머신에서는 semantic off 런조차 못 뜬다"
            )

    # _init_semantic 의 조기 return 이 import 보다 앞에 있어야 한다.
    early_return = next((s for s in func.body
                         if isinstance(s, ast.If) and isinstance(s.test, ast.UnaryOp)
                         and "semantic_enable" in ast.dump(s.test)
                         and any(isinstance(b, ast.Return) for b in s.body)), None)
    assert early_return is not None, "not self.semantic_enable 조기 return 이 없다"

    imports = [n for n in ast.walk(func)
               if isinstance(n, ast.ImportFrom) and (n.module or "").startswith("vision_msgs")]
    assert imports, "_init_semantic 안에 vision_msgs import 가 없다"
    assert all(i.lineno > early_return.lineno for i in imports)

    subs = [n for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == "create_subscription"
            and any(isinstance(a, ast.Name) and a.id == "Detection2DArray" for a in n.args)]
    assert len(subs) == 1, "검출 구독이 없거나 둘 이상이다"
    assert any(s is subs[0] or subs[0] in ast.walk(s) for s in func.body), \
        "검출 구독이 _init_semantic 밖에 있다"


def test_semantic_defaults_to_off_in_code_and_config():
    """선언 기본값과 배포 yaml 이 모두 off 여야 A/B 의 기준선이 성립한다."""
    tree, _ = _tree("core/slam.py")
    decls = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "declare_parameter" and len(node.args) == 2
                and isinstance(node.args[0], ast.Constant)
                and str(node.args[0].value).startswith("semantic.")):
            decls[node.args[0].value] = getattr(node.args[1], "value", None)
    assert decls.get("semantic.enable") is False, "semantic.enable 기본값이 False 가 아니다"

    yaml_text = (PKG.parent / "config" / "slam.yaml").read_text(encoding="utf-8")
    assert "semantic:" in yaml_text, "config/slam.yaml 에 semantic 블록이 없다"
    block = yaml_text.split("semantic:", 1)[1]
    assert "enable: false" in block.split("# ====", 1)[0]
