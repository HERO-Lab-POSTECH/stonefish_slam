"""Wildcard import 정리 회귀 가드 (slam P3 T4a/T4b 완료 상태 동결).

P3 시작 시 우리 소스엔 17곳의 `from X import *`가 있었다. T4a에서 dead 10곳을 삭제하고,
T4b에서 live 7곳을 명시 import로 좁혔다 → **우리 소스 wildcard 0개**가 최종 상태다.

이 테스트는 그 최종 상태를 동결한다:
  (1) 우리 소스(core/utils/nodes, pybind11·test 제외)에 `from X import *`가 0개.
  (2) T4b가 만든 7개 명시 import 문이 골든 마스터 심볼 집합과 정확히 일치(누락·추가 0).
  (3) 그 심볼들이 실제로 source 모듈에 존재(유령 심볼 0).

이 환경엔 rclpy/gtsam/cv2가 없어 모듈 실제 import가 불가능하므로(conversions.py:4
`import gtsam`), test/static_import_gate.py의 AST 정적 분석으로 판정한다.

⚠️ code-reviewer 지적 반영: live 검증은 `used − direct` 재유도(= same-name 로컬 섀도잉에
속을 수 있음) 대신 **명시 ImportFrom 문 자체를 파싱**해 그 `names`를 골든 집합과 비교한다.
이것이 "심볼을 드롭했는데 같은 이름 로컬이 가려 게이트가 못 잡는" 맹점을 막는다.

⚠️ 한계: 정적 판정이라 런타임 binding은 미증명(static_import_gate.py docstring 참조).
"""
import ast
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / 'test'))

from static_import_gate import module_to_file, module_exported_names  # noqa: E402


# T4b가 만든 7개 명시 import: (consumer, source 점표기) → 명시해야 할 심볼 집합.
# 골든 마스터(2026-06-24 게이트 산출, code-reviewer 함수별 전수 확인).
_EXPLICIT_IMPORTS = {
    ('stonefish_slam/core/kalman.py', 'stonefish_slam.utils.conversions'): {'g2r'},
    ('stonefish_slam/core/dead_reckoning.py', 'stonefish_slam.utils.conversions'):
        {'g2n', 'g2r', 'n2g', 'r2g'},
    ('stonefish_slam/core/types.py', 'stonefish_slam.utils.conversions'):
        {'g2n', 'n2g', 'pose322'},
    ('stonefish_slam/core/slam.py', 'stonefish_slam.utils.io'): {'CodeTimer'},
    ('stonefish_slam/core/slam.py', 'stonefish_slam.utils.conversions'):
        {'X', 'g2n', 'g2r', 'n2g', 'n2r', 'r2g'},
    ('stonefish_slam/core/slam.py', 'stonefish_slam.utils.visualization'):
        {'ros_colorline_trajectory', 'ros_constraints'},
    ('stonefish_slam/core/slam.py', 'stonefish_slam.utils.topics'): {
        'LOCALIZATION_ODOM_TOPIC', 'SLAM_CLOUD_TOPIC', 'SLAM_CONSTRAINT_TOPIC',
        'SLAM_NS', 'SLAM_ODOM_TOPIC', 'SLAM_POSE_TOPIC', 'SLAM_TRAJ_TOPIC'},
}


def _importfrom_names(consumer_rel, source_dotted):
    """consumer 파일의 `from <source_dotted> import a, b, ...` 문에서 가져오는 이름 집합.

    명시 ImportFrom 문 자체를 AST로 파싱 — wildcard('*')면 빈 집합이 아니라 {'*'} 반환.
    """
    tree = ast.parse((REPO_ROOT / consumer_rel).read_text())
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == source_dotted and node.level == 0:
            for a in node.names:
                names.add(a.name)
    return names


def test_no_wildcard_imports_in_our_source():
    """우리 소스(core/utils/nodes, pybind11·test 제외)에 `from X import *`가 0개."""
    out = []
    for py in REPO_ROOT.joinpath('stonefish_slam').rglob('*.py'):
        s = str(py)
        if 'pybind11' in s or '/test/' in s:
            continue
        for i, line in enumerate(py.read_text().splitlines(), 1):
            if re.match(r'^\s*from\s+[\w.]+\s+import\s+\*\s*$', line):
                out.append(f'{py.relative_to(REPO_ROOT)}:{i}')
    assert out == [], f'정리 후에도 wildcard import 잔존: {out}'


def test_explicit_imports_match_frozen_symbol_sets():
    """T4b가 만든 7개 명시 import 문의 심볼이 골든 마스터와 정확히 일치(누락·추가 0).

    ImportFrom 문 자체를 파싱하므로 same-name 로컬 섀도잉에 속지 않는다(code-reviewer 지적).
    """
    for (consumer, src_dotted), expected in _EXPLICIT_IMPORTS.items():
        actual = _importfrom_names(consumer, src_dotted)
        assert '*' not in actual, f'{consumer} ← {src_dotted}: wildcard 미변환 잔존'
        assert actual == expected, (
            f'{consumer} ← {src_dotted} 명시 심볼 불일치:\n'
            f'  기대 {sorted(expected)}\n  실제 {sorted(actual)}\n'
            f'  누락 {sorted(expected - actual)} / 추가 {sorted(actual - expected)}')


def test_explicit_symbols_exist_in_source_modules():
    """명시한 모든 심볼이 실제 source 모듈에 존재(유령/오타 심볼 0)."""
    for (consumer, src_dotted), expected in _EXPLICIT_IMPORTS.items():
        sfile = module_to_file(src_dotted)
        assert sfile is not None, f'source 해석 실패: {src_dotted}'
        exported = module_exported_names(sfile)
        missing = expected - exported
        assert missing == set(), (
            f'{consumer} ← {src_dotted}: source에 없는 심볼을 명시 {sorted(missing)}')
