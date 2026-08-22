"""CFAR 생성자 회귀 테스트.

CFAR.__init__이 컴파일 확장(stonefish_slam.cfar)의 실재 심볼만 참조하는지 검증.
과거 detector2 dict가 존재하지 않는 ca2/soca2/goca2/os2를 참조해 slam_node가
기동 즉시 AttributeError로 죽었다 — 기존 스위트는 CFAR를 인스턴스화하지 않아
70개 테스트가 전부 통과한 채 잠복했던 결함. cfar.py는 ROS 무의존(numpy/scipy)
이라 path-load fixture 없이 직접 import한다(.so 스테이징은 기존 전제 그대로).
"""
from stonefish_slam.core.cfar import CFAR


def test_cfar_init_uses_only_existing_bindings():
    det = CFAR(Ntc=40, Ngc=20, Pfa=0.01)
    assert set(det.detector) == {"CA", "SOCA", "GOCA", "OS"}
    assert all(callable(f) for f in det.detector.values())
