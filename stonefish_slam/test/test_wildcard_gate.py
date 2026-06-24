"""Wildcard import 변환 안전망 (slam P3 T4a/T4b 회귀 가드).

17곳의 `from X import *`를 dead 삭제(T4a) + live 명시화(T4b)로 정리하기 전에,
각 wildcard의 dead/live 판정과 live의 '명시할 심볼 집합'을 골든 마스터로 동결한다.
변환 후 이 분류가 바이트 동일하게 보존돼야 GREEN — 누가 dead를 live로 잘못 분류하거나
명시 심볼을 누락하면 RED.

이 환경엔 rclpy/gtsam/cv2가 없어 모듈 실제 import가 불가능하므로(conversions.py:4
`import gtsam`), test/static_import_gate.py의 정적 분석(AST)으로 판정한다. 정밀 게이트는
consumer가 직접 바인딩하는 이름(import numpy as np 등)을 차감해, wildcard로만 오는
'진짜 의존' 심볼만 센다 — 이것이 feature_extraction.py:9를 dead로 정확히 분류하는 근거
(그 파일의 np/cv2/cv_bridge는 자체 import이지 conversions 고유 함수가 아님).

⚠️ 한계: 정적 판정이라 런타임 binding은 미증명(static_import_gate.py docstring 참조).
getattr/문자열 디스패치 소비자는 W2가 못 잡으므로 T4 실행 시 별도 grep으로 확인한다.
"""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / 'test'))

from static_import_gate import module_to_file, wildcard_consumed_symbols  # noqa: E402


# 17곳 wildcard: (consumer 상대경로, source 점표기, 줄번호)
_WILDCARDS = [
    ('stonefish_slam/core/kalman.py', 'stonefish_slam.utils.conversions', 15),
    ('stonefish_slam/utils/conversions.py', 'stonefish_slam.utils.topics', 20),
    ('stonefish_slam/utils/sonar.py', 'stonefish_slam.utils.topics', 6),
    ('stonefish_slam/core/cfar.py', 'stonefish_slam.utils', 5),
    ('stonefish_slam/core/cfar.py', 'stonefish_slam.utils.sonar', 6),
    ('stonefish_slam/core/dead_reckoning.py', 'stonefish_slam.utils.conversions', 21),
    ('stonefish_slam/core/dead_reckoning.py', 'stonefish_slam.utils.topics', 23),
    ('stonefish_slam/core/types.py', 'stonefish_slam.utils.conversions', 10),
    ('stonefish_slam/core/types.py', 'stonefish_slam.utils.visualization', 11),
    ('stonefish_slam/core/types.py', 'stonefish_slam.utils.io', 12),
    ('stonefish_slam/core/feature_extraction.py', 'stonefish_slam.utils.io', 7),
    ('stonefish_slam/core/feature_extraction.py', 'stonefish_slam.utils.topics', 8),
    ('stonefish_slam/core/feature_extraction.py', 'stonefish_slam.utils.conversions', 9),
    ('stonefish_slam/core/slam.py', 'stonefish_slam.utils.io', 21),
    ('stonefish_slam/core/slam.py', 'stonefish_slam.utils.conversions', 22),
    ('stonefish_slam/core/slam.py', 'stonefish_slam.utils.visualization', 23),
    ('stonefish_slam/core/slam.py', 'stonefish_slam.utils.topics', 32),
]

# 골든 마스터: 정밀 게이트가 산출한 dead/live 분류 (2026-06-24 실측 동결).
# DEAD = wildcard로만 오는 의존 심볼 0개 → 삭제 안전(T4a).
_DEAD = {
    ('stonefish_slam/utils/conversions.py', 20),
    ('stonefish_slam/utils/sonar.py', 6),
    ('stonefish_slam/core/cfar.py', 5),
    ('stonefish_slam/core/cfar.py', 6),
    ('stonefish_slam/core/dead_reckoning.py', 23),
    ('stonefish_slam/core/types.py', 11),
    ('stonefish_slam/core/types.py', 12),
    ('stonefish_slam/core/feature_extraction.py', 7),
    ('stonefish_slam/core/feature_extraction.py', 8),
    ('stonefish_slam/core/feature_extraction.py', 9),
}
# LIVE = (consumer, line) → 명시 import할 심볼 집합(T4b).
_LIVE = {
    ('stonefish_slam/core/kalman.py', 15): {'g2r'},
    ('stonefish_slam/core/dead_reckoning.py', 21): {'g2n', 'g2r', 'n2g', 'r2g'},
    ('stonefish_slam/core/types.py', 10): {'g2n', 'n2g', 'pose322'},
    ('stonefish_slam/core/slam.py', 21): {'CodeTimer'},
    ('stonefish_slam/core/slam.py', 22): {'X', 'g2n', 'g2r', 'n2g', 'n2r', 'r2g'},
    ('stonefish_slam/core/slam.py', 23): {'ros_colorline_trajectory', 'ros_constraints'},
    ('stonefish_slam/core/slam.py', 32): {
        'LOCALIZATION_ODOM_TOPIC', 'SLAM_CLOUD_TOPIC', 'SLAM_CONSTRAINT_TOPIC',
        'SLAM_NS', 'SLAM_ODOM_TOPIC', 'SLAM_POSE_TOPIC', 'SLAM_TRAJ_TOPIC'},
}


def _consumed(consumer_rel, source_dotted):
    cfile = REPO_ROOT / consumer_rel
    sfile = module_to_file(source_dotted)
    assert sfile is not None, f'source 해석 실패: {source_dotted}'
    return wildcard_consumed_symbols(str(cfile), str(sfile))


def test_wildcard_count_is_seventeen():
    """우리 소스의 wildcard import는 정확히 17곳(P4_FLAGS 동기화 기준)."""
    assert len(_WILDCARDS) == 17
    assert len(_DEAD) + len(_LIVE) == 17, 'dead+live 합이 17이어야 함'


def test_dead_wildcards_have_zero_consumed_symbols():
    """DEAD로 분류된 wildcard는 consumer가 wildcard로만 오는 심볼을 0개 사용 (T4a 삭제 안전)."""
    for consumer, src, line in _WILDCARDS:
        if (consumer, line) in _DEAD:
            consumed = _consumed(consumer, src)
            assert consumed == set(), (
                f'{consumer}:{line} ← {src} 가 dead로 동결됐으나 '
                f'사용 심볼 발견: {sorted(consumed)} (변환이 분류를 깨뜨림)')


def test_live_wildcards_pin_explicit_symbol_set():
    """LIVE wildcard의 '명시할 심볼 집합'이 골든 마스터와 동일 (T4b 명시화 정확성)."""
    for consumer, src, line in _WILDCARDS:
        if (consumer, line) in _LIVE:
            consumed = _consumed(consumer, src)
            expected = _LIVE[(consumer, line)]
            assert consumed == expected, (
                f'{consumer}:{line} ← {src} 명시 심볼 변동:\n'
                f'  기대 {sorted(expected)}\n  실제 {sorted(consumed)}')


def test_every_wildcard_is_classified():
    """17곳 전부 dead 또는 live로 분류됨(미분류 0)."""
    for consumer, src, line in _WILDCARDS:
        key = (consumer, line)
        assert key in _DEAD or key in _LIVE, f'미분류 wildcard: {consumer}:{line}'
