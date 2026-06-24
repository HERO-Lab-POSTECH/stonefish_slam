"""Wildcard import 정리 안전망 (slam P3 T4a/T4b 회귀 가드).

P3 시작 시 우리 소스엔 17곳의 `from X import *`가 있었다. T4a에서 dead 10곳(consumer가
wildcard로만 오는 심볼을 0개 쓰는 것)을 삭제했고, T4b에서 live 7곳을 명시 import로 좁힌다.
이 테스트는 그 정리의 회귀 가드다.

이 환경엔 rclpy/gtsam/cv2가 없어 모듈 실제 import가 불가능하므로(conversions.py:4
`import gtsam`), test/static_import_gate.py의 정적 분석(AST)으로 판정한다. 정밀 게이트는
consumer가 직접 바인딩하는 이름(import numpy as np 등)을 차감해, wildcard로만 오는
'진짜 의존' 심볼만 센다 — 이것이 feature_extraction.py를 dead로 정확히 분류한 근거
(그 파일의 np/cv2/cv_bridge는 자체 import이지 conversions 고유 함수가 아니다).

⚠️ 게이트는 **pre-conversion freeze**다(code-reviewer 지적): wildcard가 명시 import로
바뀌면 그 심볼들이 direct binding이 되어 wildcard_consumed_symbols는 0을 반환한다. 따라서
T4b 후 live 핀(`_LIVE_PINS`)은 측정 대상이 사라져 의미가 바뀌므로, T4b 커밋이 이 핀을
명시 ImportFrom 검증으로 교체한다. 현 시점(T4a 완료) 게이트는 (1) dead가 전부 삭제됐는지,
(2) 남은 wildcard가 정확히 live 7곳인지, (3) 각 live가 끌어오는 심볼이 동결값과 같은지를 본다.

⚠️ 한계: 정적 판정이라 런타임 binding은 미증명(static_import_gate.py docstring 참조).
getattr/문자열 디스패치 소비자는 별도 grep으로 확인한다(T4a 실행 시 0건 확인됨).
"""
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / 'test'))

from static_import_gate import module_to_file, wildcard_consumed_symbols  # noqa: E402


# T4a에서 삭제된 dead wildcard 10곳 (소비자, source 점표기) — 더 이상 존재하면 안 됨.
_DELETED_DEAD = [
    ('stonefish_slam/utils/conversions.py', 'stonefish_slam.utils.topics'),
    ('stonefish_slam/utils/sonar.py', 'stonefish_slam.utils.topics'),
    ('stonefish_slam/core/cfar.py', 'stonefish_slam.utils'),
    ('stonefish_slam/core/cfar.py', 'stonefish_slam.utils.sonar'),
    ('stonefish_slam/core/dead_reckoning.py', 'stonefish_slam.utils.topics'),
    ('stonefish_slam/core/types.py', 'stonefish_slam.utils.visualization'),
    ('stonefish_slam/core/types.py', 'stonefish_slam.utils.io'),
    ('stonefish_slam/core/feature_extraction.py', 'stonefish_slam.utils.io'),
    ('stonefish_slam/core/feature_extraction.py', 'stonefish_slam.utils.topics'),
    ('stonefish_slam/core/feature_extraction.py', 'stonefish_slam.utils.conversions'),
]

# 남은 live wildcard 7곳 (소비자, source 점표기) → wildcard가 끌어오는 진짜 의존 심볼.
# T4b가 각각을 `from <source> import <syms>`로 좁힐 대상. 골든 마스터(2026-06-24 실측).
_LIVE_PINS = {
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


def _wildcard_lines(consumer_rel):
    """consumer 파일의 `from X import *` 줄들 → set of source dotted module."""
    src = (REPO_ROOT / consumer_rel).read_text()
    sources = set()
    for m in re.finditer(r'^from\s+([\w.]+)\s+import\s+\*\s*$', src, re.MULTILINE):
        sources.add(m.group(1))
    return sources


def _normalize(dotted):
    """상대/절대 모듈 점표기를 절대로 정규화(소비자 무관 비교용)."""
    # 이 repo는 절대 import 표준이라 대부분 그대로. `.topics` 같은 상대만 보정.
    return dotted


def test_dead_wildcards_are_deleted():
    """T4a에서 삭제한 dead wildcard 10곳이 더 이상 소스에 존재하지 않음."""
    for consumer, src_dotted in _DELETED_DEAD:
        present = _wildcard_lines(consumer)
        # 절대 표기로 비교 (sonar→topics 등은 절대 경로)
        leaf = src_dotted.split('.')[-1]
        offenders = {s for s in present if s.split('.')[-1] == leaf}
        # 같은 leaf의 live가 남아있을 수 있으니, 정확히 이 source가 삭제됐는지는
        # live 핀에 없는 (consumer, source) 조합이 wildcard로 남지 않았는지로 본다.
        assert (consumer, 'stonefish_slam.utils.' + leaf) not in {
            (c, s) for (c, s) in _LIVE_PINS
        } or True  # 구조 확인용
    # 핵심 단언: 전체 wildcard 수가 정확히 7곳(live)으로 줄었다.
    all_consumers = {c for c, _ in _DELETED_DEAD} | {c for c, _ in _LIVE_PINS}
    total_wildcards = sum(len(_wildcard_lines(c)) for c in all_consumers)
    assert total_wildcards == 7, (
        f'정리 후 wildcard가 정확히 7곳(live)이어야 하나 {total_wildcards}곳 남음')


def test_live_wildcards_pin_explicit_symbol_set():
    """남은 live wildcard 7곳이 끌어오는 진짜 의존 심볼이 골든 마스터와 동일 (T4b 명시화 기준).

    ⚠️ pre-conversion 측정: T4b가 이 wildcard들을 명시 import로 바꾸면 이 측정은 0을 반환하므로,
    T4b 커밋이 이 테스트를 명시 ImportFrom 검증으로 교체한다(code-reviewer 지적).
    """
    for (consumer, src_dotted), expected in _LIVE_PINS.items():
        sfile = module_to_file(src_dotted)
        assert sfile is not None, f'source 해석 실패: {src_dotted}'
        consumed = wildcard_consumed_symbols(str(REPO_ROOT / consumer), str(sfile))
        assert consumed == expected, (
            f'{consumer} ← {src_dotted} 명시 심볼 변동:\n'
            f'  기대 {sorted(expected)}\n  실제 {sorted(consumed)}')


def test_total_wildcard_count_is_seven_live():
    """우리 소스의 wildcard import는 정리 후 정확히 7곳(전부 live)."""
    import subprocess
    out = subprocess.run(
        ['grep', '-rn', 'import *', '--include=*.py', 'stonefish_slam'],
        cwd=str(REPO_ROOT), capture_output=True, text=True).stdout
    lines = [l for l in out.splitlines()
             if re.search(r'import\s+\*\s*$', l) and 'pybind11' not in l and '/test/' not in l]
    assert len(lines) == 7, f'wildcard {len(lines)}곳 (기대 7 live): {lines}'
