"""
C++ 확장 모듈 패키지

이 패키지는 pybind11로 빌드된 C++ 확장 모듈을 포함합니다.
"""

# cfar 모듈은 상위 패키지에 설치되므로 상대 import 사용
try:
    from .. import cfar
except ImportError as e:
    import warnings
    warnings.warn(f"cfar C++ module not available: {e}")

__all__ = ['cfar']
