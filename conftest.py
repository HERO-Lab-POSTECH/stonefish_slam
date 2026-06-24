import importlib.util
import sys
import types
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


@pytest.fixture
def load_factor_graph():
    """factor_graph.py를 clean env(rclpy/cv_bridge 부재)에서 로드.

    factor_graph.py는 import-time에 형제 모듈(core.types → rclpy.time, utils.conversions
    → cv_bridge/cv2/ROS msgs)을 끌어오는데, 이들은 verify_pcm·noise-model 함수에는 쓰이지
    않는다(그 함수들은 순수 gtsam+numpy). 그래서 ROS 의존만 stub으로 채우고 패키지 __init__을
    우회하면 clean env에서도 FactorGraph 클래스에 도달한다. gtsam은 실제로 import된다.

    load_module(단일 파일)로는 factor_graph가 형제 절대 import에 걸려 못 뜨므로 별도 fixture.
    """
    preexisting = set(sys.modules)  # teardown에서 이 집합에 없는 키만 제거

    # ROS/cv 의존 stub (import-time 크래시 회피용 — 함수 본체는 안 씀).
    stub_specs = {
        "cv_bridge": {"CvBridge": object},
        "cv2": {},
        "sensor_msgs_py": {},
        "sensor_msgs_py.point_cloud2": {},
        "rclpy": {},
        "rclpy.time": {"Time": object},
        "geometry_msgs": {},
        "geometry_msgs.msg": {"Pose": object},
        "std_msgs": {},
        "std_msgs.msg": {"Header": object},
    }
    for name, attrs in stub_specs.items():
        if name not in sys.modules:
            mod = types.ModuleType(name)
            for k, v in attrs.items():
                setattr(mod, k, v)
            sys.modules[name] = mod
    # 서브모듈 속성 연결 (예: sensor_msgs_py.point_cloud2).
    sys.modules["sensor_msgs_py"].point_cloud2 = sys.modules["sensor_msgs_py.point_cloud2"]

    # stonefish_slam 패키지 __init__ 우회 — 빈 네임스페이스 패키지로 등록.
    for pkg in ("stonefish_slam", "stonefish_slam.core", "stonefish_slam.utils"):
        if pkg not in sys.modules:
            m = types.ModuleType(pkg)
            m.__path__ = [str(REPO_ROOT / pkg.replace(".", "/"))]
            sys.modules[pkg] = m

    def _load(relpath, name):
        spec = importlib.util.spec_from_file_location(name, str(REPO_ROOT / relpath))
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
        return mod

    # factor_graph가 from-import하는 형제를 정확한 패키지 이름으로 먼저 로드.
    _load("stonefish_slam/utils/conversions.py", "stonefish_slam.utils.conversions")
    _load("stonefish_slam/core/types.py", "stonefish_slam.core.types")
    fg = _load("stonefish_slam/core/factor_graph.py", "stonefish_slam.core.factor_graph")

    yield fg

    # teardown: 이 fixture가 들여온 키만 제거(기존 키는 보존).
    for name in list(sys.modules):
        if name not in preexisting:
            sys.modules.pop(name, None)


@pytest.fixture
def load_mapping_3d():
    """mapping_3d.py를 clean env(cv_bridge 부재)에서 로드.

    mapping_3d.py 자체는 cv_bridge를 안 쓰지만(numpy·scipy·profiler·octree만),
    `from stonefish_slam.utils.profiler import ...`가 패키지 `utils/__init__`을
    실행시키고 그게 sonar→conversions→cv_bridge를 끌어와 import-time에 크래시한다.
    그래서 패키지 __init__을 빈 네임스페이스로 우회하고, mapping_3d가 실제로 쓰는
    형제(profiler·octree, 둘 다 순수)만 직접 로드하면 clean env에서도 도달한다.
    C++ 확장(ray_processor:25)은 try/except 가드라 clean env에서 자연히 fallback.

    [[stonefish-p4-test-load-constraint]] — load_factor_graph와 동일 패턴.
    """
    preexisting = set(sys.modules)

    # 패키지 __init__ 우회 — 빈 네임스페이스 패키지로 등록(utils/__init__의 sonar→
    # conversions→cv_bridge 체인 차단).
    for pkg in ("stonefish_slam", "stonefish_slam.core", "stonefish_slam.utils"):
        if pkg not in sys.modules:
            m = types.ModuleType(pkg)
            m.__path__ = [str(REPO_ROOT / pkg.replace(".", "/"))]
            sys.modules[pkg] = m

    def _load(relpath, name):
        spec = importlib.util.spec_from_file_location(name, str(REPO_ROOT / relpath))
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
        return mod

    # mapping_3d가 import-time에 from-import하는 형제(순수)를 정확한 이름으로 먼저.
    _load("stonefish_slam/utils/profiler.py", "stonefish_slam.utils.profiler")
    _load("stonefish_slam/core/octree.py", "stonefish_slam.core.octree")
    m3d = _load("stonefish_slam/core/mapping_3d.py", "stonefish_slam.core.mapping_3d")

    yield m3d

    for name in list(sys.modules):
        if name not in preexisting:
            sys.modules.pop(name, None)
