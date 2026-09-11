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
- 좌표계: **전역 = `world_ned`(NED)로 통일**(P4d 결정 2026-06-24 — Stonefish sim이 전역을 NED로 발행해 정합. 의도적 REP-103/105 비순응), **로컬 TF 체인 `odom→base_link`는 프레임 이름만 REP-105를 빌렸고 데이터는 NED(z-down)**(`core/dead_reckoning.py` — 2026-08-21 소비자 전수 추적으로 이전의 "ENU 유지" 기술이 반증됨. 구독자 0·TF identity라 체인 전체가 NED로 자기정합). TF는 identity라 좌표 변환 없이 frame_id 이름만 정합. 상세·근거는 `docs/CONVENTIONS.md` §2.0. ⚠️ sim도 전역 NED라 전역 프레임은 양 repo 일치(체인 전체가 NED — 통합 시 좌표 변환 불필요, 이름 층위만 정합).
- 테스트: 루트 `conftest.py`의 `load_module` fixture로 모듈 직접 로드(import-time rclpy/gtsam 오염 회피), 이 repo 루트에서 `python3 -m pytest`. vendored pybind11은 discovery 배제. CI는 `.github/workflows/ci.yml`(Python 3.10).
- **`.so` 스테이징이 테스트의 선행 조건입니다.** pybind11 확장 5개(`cfar`·`dda_traversal`·`octree_mapping`·`ray_processor`·`pcl`)는 gitignore 대상이라 빌드 산출물을 소스 트리로 복사해야 수집이 됩니다 — 워크스페이스 루트에서 `colcon build --merge-install --packages-select stonefish_slam && cp build/stonefish_slam/*.so src/stonefish_slam/stonefish_slam/`. 안 하면 collection 단계에서 실패하는데 이는 정상 동작이지 깨진 체크아웃이 아닙니다.
- 코드 그래프는 **이 repo 루트 `.graphify/`**에 있습니다(2026-09-11 실측 1,300노드). 탐색은 Grep/Read보다
  `graphify query "<식별자>"`를 먼저 — 조회는 반드시 이 디렉터리 안에서 합니다(워크스페이스
  루트에는 그래프가 없습니다). 신뢰 규칙 정본은 `/workspace/.claude/rules/code-graph.md`.
- 거버넌스 스토어 `.hq/`는 **이 repo에 없습니다** — 워크스페이스 루트 `/workspace/.hq/`
  (앵커 `id: stonefish_ws`) 하나가 세 repo를 함께 관장합니다. 구조·명명 규칙은
  `/workspace/.hq/config/project/rules.json`이 정본입니다.
- 미해결 수치/명명 이슈는 [`P4_FLAGS.md`](P4_FLAGS.md)에 모인다(현재: fusion.ema_fusion의 observation_count 미사용·localization.yaml icp_config 하드코딩 절대경로(dead fallback 확정, 제거 미실행)·dead_reckoning.py docstring 내부 탭 등) — 새 코드는 거기 적힌 안티패턴을 답습하지 않는다. (ICP 수렴·노드명 공유·wildcard import는 P3/P4에서 이미 해결/반증됨 — P4_FLAGS.md 참조.)
