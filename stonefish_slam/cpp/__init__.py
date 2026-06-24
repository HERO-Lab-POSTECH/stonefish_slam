"""
C++ 확장 모듈 패키지

이 패키지는 pybind11로 빌드된 C++ 확장 모듈을 포함합니다.
.so 파일은 빌드 시 상위 stonefish_slam 패키지에 설치됩니다.

각 C++ 모듈을 상위 패키지에서 import 시도하고, 실패하면(미빌드 등) 어느
모듈이 빠졌는지 경고를 남긴다. pcl은 .so가 없으면 같은 디렉토리의 순수
Python 구현(pcl.py)으로 대체되는데 — 그 fallback은 동작이 다른(덜 정밀한)
ICP estimator이므로, 누락을 silent로 두지 않고 어떤 구현이 활성화됐는지
로그로 드러낸다.
"""
import logging

logger = logging.getLogger(__name__)

# C++ 모듈을 상위 패키지에서 import (빌드 시 .so가 상위에 설치됨).
# 실패는 _missing에 모아 import 끝에 한 번 경고한다(silent pass 금지).
_missing = []

try:
    from stonefish_slam import cfar
except ImportError:
    _missing.append("cfar")

try:
    from stonefish_slam import dda_traversal
except ImportError:
    _missing.append("dda_traversal")

try:
    from stonefish_slam import octree_mapping
except ImportError:
    _missing.append("octree_mapping")

try:
    from stonefish_slam import ray_processor
except ImportError:
    _missing.append("ray_processor")

try:
    from stonefish_slam import pcl
except ImportError:
    _missing.append("pcl")

if _missing:
    logger.warning(
        "C++ extension(s) not available: %s. "
        "Affected features fall back to pure-Python where one exists "
        "(e.g. pcl -> pcl.py, a less precise ICP) or are disabled. "
        "Build the package (colcon build) to use the C++ implementations.",
        ", ".join(_missing),
    )

__all__ = ['cfar', 'dda_traversal', 'octree_mapping', 'ray_processor', 'pcl']
