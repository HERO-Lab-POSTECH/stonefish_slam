"""정적 import 검증 도구 (slam P3 안전망, rclpy/gtsam 부재 환경용).

이 환경엔 rclpy/gtsam/cv2가 없어 대부분의 core 모듈을 실제 import(exec_module)하면
import-time에 죽는다(conversions.py:4 `import gtsam` 등). 따라서 "모듈 실제 로드"
import-smoke가 불가능하다. 대신 **정적 분석**(ast.parse + py_compile + 파일경로 해석)으로
wildcard 변환·dead 삭제·순수함수 추출의 동작보존 불변식을 검증한다.

slam ≠ sim 차이 반영:
  - slam은 **절대 import 표준**(`from stonefish_slam.X import ...`)이 norm(sim은 상대).
  - **단일 패키지**(stonefish_slam) — sim의 다중 ament_python 패키지와 다름.
  - 빌드 = CMakeLists install PROGRAMS(setup.py console_scripts 아님) — 노드 엔트리는 CMake가 관리.
  - P3 초점 = **wildcard import 17곳 변환**(dead 삭제 7 + live 명시화 5 + types 추가분)의 안전.

게이트 (ralplan 합의, O6=단순 grep로 확정 — 부재 서드파티 재귀 해석 불요):
  W1  wildcard source export-set 정적 계산 (own top-level defs + transitive `import *` 1차)
  W2  consumer 실사용 심볼 추출 (Name 노드 - 자기 정의/파라미터/import 제외, 근사)
  W3  import target-set 동결 (변환 전후 repo-내부 import 집합 비교)
  W4  transitive-only 참조 0건 단언 (B3: consumer가 transitive 체인으로만 닿는 이름 미참조)
  G5  py_compile (실행 안 함, rclpy-safe)
  G6  모듈 top-level 부작용 0 (이동/추출이 import-time 동작에 무영향)

⚠️ 한계(정직하게 명시 — sim 게이트 docstring과 동일): 이 게이트들은 경로/타겟집합/심볼집합
불변을 **정적으로** 증명할 뿐, 런타임 심볼 binding(동명이클래스, __init__ re-export 섀도잉,
메서드→함수 binding 변경의 외부 동적 호출자)은 증명하지 못한다 — 그건 colcon build +
ros2 launch가 필요하며 P4 sign-off로 미룬다. W2는 Name 노드 근사라 getattr/문자열 디스패치는
별도 grep으로 확인한다(이 도구는 그 grep의 보조).
"""
import ast
import py_compile
from pathlib import Path

# repo root = test/ 의 부모
REPO_ROOT = Path(__file__).resolve().parent.parent

# 단일 패키지: (패키지명, src 루트) — 절대 점표기 ↔ 파일경로 해석에 사용
_PKG = 'stonefish_slam'
_PKG_SRC = REPO_ROOT / 'stonefish_slam'

# 서드파티/외부 + stdlib — 절대 import여도 repo 내부 파일로 해석하지 않음
_EXTERNAL_TOP = {
    'rclpy', 'numpy', 'np', 'scipy', 'gtsam', 'cv2', 'cv_bridge', 'sklearn',
    'shapely', 'matplotlib', 'std_msgs', 'nav_msgs', 'geometry_msgs',
    'sensor_msgs', 'visualization_msgs', 'octomap_msgs', 'builtin_interfaces',
    'tf2_ros', 'tf2_geometry_msgs', 'message_filters', 'sensor_msgs_py',
    'tf_transformations', 'ament_index_python', 'math', 'sys', 'os', 'struct',
    'time', 'traceback', 'collections', 'typing', 'dataclasses', 'enum', 'abc',
    'copy', 'functools', 'itertools', 'warnings',
}


def module_to_file(dotted):
    """`stonefish_slam.sub.module` 점표기 → 실제 .py 파일 Path. 해석 불가면 None."""
    parts = dotted.split('.')
    if parts[0] != _PKG:
        return None
    # _PKG_SRC 자체가 stonefish_slam/ 이므로 parts[0] 이후를 이어붙임
    sub = parts[1:]
    candidate = _PKG_SRC.joinpath(*sub).with_suffix('.py')
    if candidate.exists():
        return candidate
    pkg_init = _PKG_SRC.joinpath(*sub) / '__init__.py'
    if pkg_init.exists():
        return pkg_init
    return None


def _import_targets_for_file(pyfile):
    """파일 내 ImportFrom 노드를 (kind, module, names) 튜플 집합으로 정규화.

    절대(slam 표준)/상대 어느 형태든 같은 repo 모듈을 가리키면 동일 튜플이 나오도록 —
    wildcard 변환·이동 전후 target-set diff의 핵심. 외부 import는 EXTERNAL 마커.
    """
    pyfile = Path(pyfile).resolve()
    # 파일이 속한 패키지 내부 점표기 prefix (상대 import 해석용)
    try:
        rel = pyfile.relative_to(_PKG_SRC.resolve())
        pkg_prefix = [_PKG] + list(rel.parts[:-1])
    except ValueError:
        pkg_prefix = None
    targets = set()
    tree = ast.parse(pyfile.read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        names = tuple(sorted(a.name for a in node.names))
        if node.level == 0:
            mod = node.module or ''
            top = mod.split('.')[0]
            if top == _PKG:
                targets.add(('REPO', mod, names))
            else:
                targets.add(('EXTERNAL', mod, names))
        else:
            if pkg_prefix is None:
                targets.add(('UNRESOLVED', f'.{node.level}:{node.module}', names))
                continue
            up = node.level - 1
            base = pkg_prefix[:len(pkg_prefix) - up] if up <= len(pkg_prefix) else None
            if base is None:
                targets.add(('UNRESOLVED', f'.{node.level}:{node.module}', names))
                continue
            mod_parts = base + ([node.module] if node.module else [])
            targets.add(('REPO', '.'.join(mod_parts), names))
    return targets


def repo_import_targets(pyfile):
    """파일의 repo-내부 import 타겟 집합(외부 제외). wildcard 변환/이동 전후 비교용."""
    return {t for t in _import_targets_for_file(pyfile) if t[0] == 'REPO'}


def module_exported_names(pyfile, _seen=None):
    """모듈이 `from X import *`로 노출하는 top-level 이름 집합 (W1).

    __all__이 있으면 그것을, 없으면 밑줄 안 붙은 모든 top-level 바인딩(def/class/할당/
    import 별칭)을 반환. 단 transitive `from .Y import *`는 1차로 Y의 export-set을 합친다
    (slam은 전부 repo-internal이라 정적 계산 가능 — ralplan O6 확인). 순환 방지로 _seen 추적.
    """
    pyfile = Path(pyfile).resolve()
    if _seen is None:
        _seen = set()
    if pyfile in _seen:
        return set()
    _seen.add(pyfile)
    tree = ast.parse(pyfile.read_text())
    # __all__ 우선
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id == '__all__':
                    if isinstance(node.value, (ast.List, ast.Tuple)):
                        return {e.value for e in node.value.elts
                                if isinstance(e, ast.Constant) and isinstance(e.value, str)}
    names = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            if not node.name.startswith('_'):
                names.add(node.name)
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name) and not t.id.startswith('_'):
                    names.add(t.id)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            # import * → transitive 해석; 일반 import는 별칭 이름 노출
            for a in node.names:
                if a.name == '*' and isinstance(node, ast.ImportFrom):
                    sub = module_to_file(node.module) if node.level == 0 else None
                    if sub:
                        names |= module_exported_names(sub, _seen)
                else:
                    bound = a.asname or a.name.split('.')[0]
                    if not bound.startswith('_'):
                        names.add(bound)
    return names


def used_names(pyfile):
    """파일에서 참조되는 모든 Name(ast.Load) 식별자 집합 (W2, 근사).

    consumer가 실제로 어떤 이름을 쓰는지 — wildcard를 명시 import로 좁힐 때 어떤 심볼이
    필요한지 판정용. docstring/주석 텍스트는 Name 노드가 아니므로 자동 제외(이게 핵심:
    'X-axis' 같은 docstring 'X'는 안 잡힘 → feature_extraction dead 판정 근거).
    """
    tree = ast.parse(Path(pyfile).read_text())
    used = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load):
            used.add(node.id)
        elif isinstance(node, ast.Attribute):
            # attribute base가 Name이면 base만 (a.b.c → a)
            pass
    return used


def direct_bound_names(pyfile):
    """consumer가 wildcard 외 경로로 직접 바인딩하는 이름.

    = 자기 def/class + 일반 import 별칭 + named import + 할당 + 함수 파라미터.
    wildcard로부터 오는 이름과 구분하기 위함 — consumer가 `import numpy as np`로 직접
    np를 끌어오면, conversions wildcard가 np를 re-export해도 그건 wildcard 의존이 아니다.
    """
    tree = ast.parse(Path(pyfile).read_text())
    bound = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            bound.add(node.name)
        elif isinstance(node, ast.Import):
            for a in node.names:
                bound.add(a.asname or a.name.split('.')[0])
        elif isinstance(node, ast.ImportFrom):
            for a in node.names:
                if a.name != '*':
                    bound.add(a.asname or a.name)
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name):
                    bound.add(t.id)
        elif isinstance(node, ast.arg):
            bound.add(node.arg)
    return bound


def wildcard_consumed_symbols(consumer_file, source_file):
    """consumer가 source의 wildcard로부터 **실제로 의존하는** 심볼 집합 (W2 정밀).

    = module_exported_names(source) ∩ used_names(consumer) − direct_bound_names(consumer).
    consumer가 직접 import/정의/할당으로 바인딩한 이름은 wildcard 의존이 아니므로 차감한다
    (예: feature_extraction이 `import numpy as np`하면 conversions의 np re-export는 무관).
    빈 집합이면 그 wildcard는 **dead**(삭제 안전). 비면 명시 import 대상 목록.
    """
    exported = module_exported_names(source_file)
    used = used_names(consumer_file)
    direct = direct_bound_names(consumer_file)
    return (exported & used) - direct


def py_compiles(pyfile):
    """py_compile로 바이트컴파일(실행 안 함). 성공 True. rclpy-safe."""
    try:
        py_compile.compile(str(pyfile), doraise=True)
        return True
    except py_compile.PyCompileError:
        return False


def module_toplevel_dynamic_refs(pyfile):
    """모듈 top-level(함수/클래스 body 제외)에 동적 실행문이 있으면 설명 리스트 반환.

    빈 리스트 = 깨끗(이동/추출이 import-time 동작에 무영향).
    import / def / class / docstring / __all__·상수 할당 / if __name__ 가드는 허용.
    """
    tree = ast.parse(Path(pyfile).read_text())
    offenders = []
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom,
                             ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant):
            continue  # 모듈 docstring
        if isinstance(node, ast.Assign):
            src = ast.dump(node)
            if '__file__' in src:
                offenders.append(f'L{node.lineno}: dynamic assign (__file__)')
            continue
        if isinstance(node, ast.If):
            if '__name__' in ast.dump(node.test):
                continue
            offenders.append(f'L{node.lineno}: top-level if (non-__name__ guard)')
            continue
        offenders.append(f'L{node.lineno}: {type(node).__name__}')
    return offenders


def function_body_ast(pyfile, func_name):
    """모듈 top-level 또는 클래스 내 `func_name`의 본문 AST dump 반환 (추출 전후 동등 비교용).

    메서드→모듈함수 추출(T5)에서 본문이 바이트 동일하게 보존됐는지 증명.
    self 파라미터 제거를 감안해 **본문 statement들만** dump(시그니처 제외).
    """
    tree = ast.parse(Path(pyfile).read_text())
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == func_name:
            return '\n'.join(ast.dump(s) for s in node.body)
    return None
