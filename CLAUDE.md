# stonefish_slam

ROS2 Humble 수중 SLAM 패키지 — Python SLAM 코어(`core/`)와 pybind11 C++ 확장(`cpp/`)으로 구성된 단일 패키지 repo.
`core/`(알고리즘), `nodes/`(ROS2 진입점), `utils/`(헬퍼), `cpp/`(C++ 바인딩+순수파이썬 fallback)로 관심사를 분리한다.

## 작업 전 필독

**모든 코드 작업 전에 [`docs/CONVENTIONS.md`](docs/CONVENTIONS.md)를 먼저 읽을 것.** 그 문서가 이 repo의 단일 진실(SSOT)이다:
- **코딩 컨벤션** — 명명·디렉토리 구조·import·docstring·ROS2 노드 패턴·config·테스트·C++/pybind11 바인딩. 외부 표준(REP 144/103/105, PEP 8/257, Google Style)과 대조하고 강제(normative)/관행(practice)을 구분한다.
- **작업 프로세스 게이트** — 비자명한 변경은 ①정독+의존성추적(C++ `.so` 경계 포함) → ②자료조사 → ③설계 → ④구현 → ⑤검토를 순서대로 거치고 산출물을 남긴다.

## 핵심 사실
- 라이선스 **GPL-3.0**, 메인테이너 Seungmin Kim <luckkim123@gmail.com>.
- 빌드: `CMakeLists.txt`(ament_cmake + Python). C++ pybind11 확장 때문에 setup.py가 없다 — C++/Python 혼합 패키지의 정당한 형태(`docs/CONVENTIONS.md` §2.0).
- 좌표계 **ENU**(`map`/`odom`/`base_link`, REP 105 부합). 단 3D 매핑 경로는 `world_ned`(NED) 레거시를 일부 혼용(`core/slam.py:824`·`mapping_3d.py` — 통일은 P4, `docs/CONVENTIONS.md` §2.0). ⚠️ sim은 전역 NED(`world_ned`)라 두 repo 통합 시 TF 변환 필요.
- 테스트: 루트 `conftest.py`의 `load_module` fixture로 모듈 직접 로드(import-time rclpy/gtsam 오염 회피), `pytest -v`. vendored pybind11은 discovery 배제. CI는 `.github/workflows/ci.yml`(Python 3.10).
- 미해결 수치/명명 이슈는 [`P4_FLAGS.md`](P4_FLAGS.md)에 모인다(현재: ICP 수렴·노드명 3중 충돌·wildcard import 등) — 새 코드는 거기 적힌 안티패턴을 답습하지 않는다.
