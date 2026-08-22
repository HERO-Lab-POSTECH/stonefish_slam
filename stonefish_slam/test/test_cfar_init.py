"""CFAR 생성자 회귀 테스트.

과거 detector2 dict가 컴파일 확장(cfar)에 존재하지 않는 ca2/soca2/goca2/os2를
참조해 slam_node가 기동 즉시 AttributeError로 죽었다 — 기존 스위트는 CFAR를
인스턴스화하지 않아 전부 통과한 채 잠복했던 결함.

CI는 .so를 빌드하지 않으므로 실제 확장 대신 cpp/cfar.cpp의 m.def(...) 선언에서
파싱한 실재 export 집합으로 바인딩 stub을 깔고 path-load한다(conftest의 stub
패턴과 동일). __init__이 export에 없는 심볼을 참조하면 stub에서도 똑같이
AttributeError가 나므로 회귀가 CI에서 잡힌다.

평범한 `from stonefish_slam.core.cfar import ...`는 core/__init__ 경유로
cv_bridge를 끌어와 clean env 수집을 깨므로 금지(conftest.load_module 설계).
"""
import re
import sys
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def test_cfar_init_uses_only_existing_bindings(load_module, monkeypatch):
    src = (REPO_ROOT / "stonefish_slam" / "cpp" / "cfar.cpp").read_text()
    exports = set(re.findall(r'm\.def\("(\w+)"', src))
    assert exports, "cfar.cpp에서 m.def export를 파싱하지 못함 — 정규식/파일 확인"

    stub = types.ModuleType("stonefish_slam.cpp.cfar")
    for name in exports:
        setattr(stub, name, lambda *a, **k: None)
    cpp_pkg = types.ModuleType("stonefish_slam.cpp")
    cpp_pkg.cfar = stub
    monkeypatch.setitem(sys.modules, "stonefish_slam", types.ModuleType("stonefish_slam"))
    monkeypatch.setitem(sys.modules, "stonefish_slam.cpp", cpp_pkg)
    monkeypatch.setitem(sys.modules, "stonefish_slam.cpp.cfar", stub)

    cfar_mod = load_module("stonefish_slam/core/cfar.py", "cfar_under_test")

    # 버그 코드(detector2)는 stub에 없는 ca2 접근 시점에 AttributeError로 죽는다.
    det = cfar_mod.CFAR(Ntc=40, Ngc=20, Pfa=0.01)
    assert set(det.detector) == {"CA", "SOCA", "GOCA", "OS"}
    assert set(det.detector) == {k.upper() for k in exports}
