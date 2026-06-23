import importlib.util
import sys
from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).parent

@pytest.fixture
def load_module():
    """대상 .py를 파일 경로로 직접 로드 — 패키지 __init__/형제 모듈(ROS·gtsam) 오염 우회."""
    loaded = []
    def _load(relpath, name):
        path = REPO_ROOT / relpath
        spec = importlib.util.spec_from_file_location(name, str(path))
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
        loaded.append(name)
        return mod
    yield _load
    for name in loaded:
        sys.modules.pop(name, None)
