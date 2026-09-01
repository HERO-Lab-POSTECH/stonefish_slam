# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""계측(I1~I11)이 실제로 배선돼 있는지 AST 로 고정한다.

`slam.py` 는 import-time 에 rclpy·gtsam·cv_bridge 를 끌어오므로 `load_module`
fixture 로도 뜨지 않는다(CONVENTIONS §2.8 의 path-load 가 닿지 않는 부류).
그래서 여기서는 소스를 AST 로 읽어 **배선의 존재**만 검사한다 — 값이 맞는지는
GPU 머신 런타임이 판정할 몫이고, 이 테스트가 막으려는 것은 그 이전 단계의 실패,
즉 계측이 조용히 끊겨 로그가 영영 안 나오는 상황이다.

가장 중요한 것은 `fft_is_dr_fallback` 이다. 이 플래그는 계측 전까지 **write-only**
였고, 그 탓에 DR fallback 시드에도 `[FFT_SEED]` 태그가 붙어 로그가 거짓을 말했다.
읽는 코드가 사라지면 같은 상태로 되돌아가므로 여기서 못 박는다.
"""
import ast
from pathlib import Path

CORE = Path(__file__).resolve().parents[1] / "core"


def _tree(name):
    return ast.parse((CORE / name).read_text(encoding="utf-8"), filename=name)


def _self_attr_keys(tree, attr):
    """`self.<attr>['key']` 형태로 접근되는 문자열 키를 모은다."""
    keys = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Subscript):
            continue
        v = node.value
        if (isinstance(v, ast.Attribute) and v.attr == attr
                and isinstance(v.value, ast.Name) and v.value.id == "self"
                and isinstance(node.slice, ast.Constant)
                and isinstance(node.slice.value, str)):
            keys.add(node.slice.value)
    return keys


def test_every_declared_counter_is_incremented_somewhere():
    """선언한 카운터 키와 실제로 갱신되는 키가 일치해야 한다.

    오타 하나면 카운터가 조용히 0 으로 남고, 그러면 "분모 없는 비율 보고" 라는
    계측의 존재 이유 자체가 무너진다. 그래서 갱신은 전부 상수 키로 쓴다 —
    `self.instr[f'reject_{reason}']` 같은 동적 키는 AST 로 볼 수 없어 이 검사를
    통째로 우회하고, 선언에 없는 사유가 추가되면 KeyError 로 죽는다(적대 검증 NIT).
    """
    tree = _tree("slam.py")

    declared = None
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Attribute)
                and node.targets[0].attr == "instr"
                and isinstance(node.value, ast.Dict)):
            declared = {k.value for k in node.value.keys}
            break
    assert declared is not None, "self.instr 선언을 찾지 못했다"

    touched = _self_attr_keys(tree, "instr")
    missing = declared - touched
    assert not missing, f"선언만 되고 아무도 갱신하지 않는 카운터: {sorted(missing)}"

    unknown = touched - declared
    assert not unknown, f"선언에 없는 카운터를 갱신한다(오타 의심): {sorted(unknown)}"


def test_dr_fallback_flag_is_read_not_only_written():
    """I4 — `fft_is_dr_fallback` 에 읽기 접근이 있어야 한다.

    쓰기만 남으면 시드 출처를 구분할 수 없고 태그 분기도 거짓이 된다.
    """
    src = (CORE / "slam.py").read_text(encoding="utf-8")
    tree = ast.parse(src, filename="slam.py")

    written = reads = 0
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and node.attr == "fft_is_dr_fallback":
            if isinstance(node.ctx, ast.Store):
                written += 1
            else:
                reads += 1
    # getattr(keyframe, 'fft_is_dr_fallback', False) 형태도 읽기로 센다
    reads += src.count("'fft_is_dr_fallback'") + src.count('"fft_is_dr_fallback"')

    assert written > 0, "플래그를 쓰는 곳이 사라졌다 — 계측 대상 자체가 없어졌다"
    assert reads > 0, (
        "fft_is_dr_fallback 이 다시 write-only 다 — DR fallback 시드에도 "
        "[FFT_SEED] 가 붙어 로그가 거짓을 말하게 된다"
    )


def test_seed_tag_is_branched():
    """I5 — 시드 태그가 출처별로 갈려 있어야 한다."""
    src = (CORE / "slam.py").read_text(encoding="utf-8")
    assert "[FFT_SEED]" in src
    assert "[DR_SEED]" in src, "DR 시드 태그가 없다 — 두 경로가 같은 태그로 찍힌다"


def test_ssm_disabled_counter_exists_and_increments():
    """I1 — SSM 조기 반환 경로가 세어져야 한다.

    이 값이 키프레임 총수와 같으면 "icp 0%" 의 원인이 알고리즘이 아니라
    `ssm.enable: false` 라는 설정임이 확정된다. 계측의 출발점이다.
    """
    tree = _tree("localization.py")

    init = incr = False
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Attribute)
                and node.targets[0].attr == "ssm_disabled_count"):
            init = True
        if (isinstance(node, ast.AugAssign)
                and isinstance(node.target, ast.Attribute)
                and node.target.attr == "ssm_disabled_count"):
            incr = True

    assert init, "ssm_disabled_count 초기화가 없다"
    assert incr, "ssm_disabled_count 를 증가시키는 곳이 없다 — 영영 0 으로 보고된다"


def test_every_exit_path_of_ssm_logs_the_summary():
    """`add_sequential_scan_matching` 의 **모든** 종료 경로가 요약을 내야 한다.

    조기 return 이 요약을 건너뛰면, 그 경로로만 빠지는 구성에서는 카운터가 영영
    보이지 않는다. 실제로 그런 구멍이 있었다 — `ssm.enable: false` 면 초기화가
    실패해 매 키프레임 조기 return 으로 빠지는데, 하필 그 상황이 I1
    (`ssm_disabled_count`)이 답해야 할 바로 그 질문이었다. 계측의 출발점이
    자기 질문에서만 침묵하던 셈이다.
    """
    tree = _tree("slam.py")

    func = next(
        (n for n in ast.walk(tree)
         if isinstance(n, ast.FunctionDef) and n.name == "add_sequential_scan_matching"),
        None,
    )
    assert func is not None, "add_sequential_scan_matching 을 찾지 못했다"

    def _is_log_call(stmt):
        return (isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call)
                and isinstance(stmt.value.func, ast.Attribute)
                and stmt.value.func.attr == "_log_instrumentation")

    def _check(body, guarded):
        """블록을 훑으며 각 return 이 **같은 블록 안에서** 요약 뒤에 오는지 본다.

        라인 번호만 비교하면 앞선 블록의 호출 하나가 뒤따르는 모든 조기 return 을
        가려 준다 — 두 번째 조기 return 에서 호출을 빼도 통과한다(적대 검증 NIT).
        그래서 실제 블록 구조를 따라간다: `guarded` 는 이 블록에 들어오기까지
        요약이 이미 불렸는지이고, 블록 안에서 호출을 만나면 그 이후로 참이 된다.
        """
        seen = guarded
        for stmt in body:
            if _is_log_call(stmt):
                seen = True
            elif isinstance(stmt, ast.Return):
                assert seen, (
                    f"slam.py:{stmt.lineno} 의 return 앞에 _log_instrumentation() 이 "
                    "없다 — 이 경로로 빠지는 구성에서는 계측이 통째로 침묵한다"
                )
            for attr in ("body", "orelse", "finalbody"):
                nested = getattr(stmt, attr, None)
                if nested:
                    # 분기 안에서 불린 호출은 그 분기에서만 유효하므로 seen 을
                    # 물려주되 되돌려 받지는 않는다.
                    _check(nested, seen)
        return seen

    returns = [n for n in ast.walk(func) if isinstance(n, ast.Return)]
    assert returns, "조기 return 이 하나도 없다 — 테스트 전제가 깨졌다"

    tail_seen = _check(func.body, False)
    assert tail_seen, "정상 종료 경로(함수 끝)에 요약 호출이 없다"


def test_fft_return_dict_carries_the_values_instrumentation_consumes():
    """I8·I9·I10 — slam.py 가 읽는 키가 FFT 반환 dict 에 실제로 있어야 한다.

    이 값들은 원래부터 계산돼 dict 에 실려 있었고 아무도 읽지 않았을 뿐이다.
    계측은 소비만 추가했으므로, 키가 사라지면 로그가 통째로 NaN 이 된다.
    """
    tree = _tree("localization_fft.py")

    consumed = {"rot_peak", "trans_peak", "covariance", "rotation_fft",
                "rotation_override_used", "rotation"}
    produced = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            produced |= {k.value for k in node.keys
                         if isinstance(k, ast.Constant) and isinstance(k.value, str)}

    missing = consumed - produced
    assert not missing, f"계측이 읽는데 FFT 가 안 내보내는 키: {sorted(missing)}"
