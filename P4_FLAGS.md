# P4 Investigation Flags

## 압력→깊이 변환 Pa/kPa 1000배 단위 버그 (P3.2에서 확정) ★HIGH

- **파일**: `stonefish_slam/core/kalman_filter.py::pressure_to_depth` (원래 `core/kalman.py` pressure_callback). `core/dead_reckoning.py:173-174`에도 동일 식 주석 존재.
- **발견일**: 2026-06-23 (P3.2 콜백 추출의 자료조사+의존성추적에서 확정)
- **증상**: `curr_depth = -((fluid_pressure / 101.325) - 1) * 10`. ROS `sensor_msgs/FluidPressure.fluid_pressure`는 **Pa** 단위인데, `101.325`는 1 atm을 **kPa**로 표현한 값 → **1000배 단위 불일치**.
- **증거체인(추측 아님, 실측)**:
  1. ROS 스펙: FluidPressure.fluid_pressure = "Absolute pressure reading in Pascals". 출처: [docs.ros2.org FluidPressure](https://docs.ros2.org/latest/api/sensor_msgs/msg/FluidPressure.html).
  2. sim publisher가 변환 없이 raw Pa publish: `stonefish_ros2/src/stonefish_ros2/ROS2Interface.cpp:185` `msg.fluid_pressure = s.getValue(0)`.
  3. 환경 압력이 Pa: `stonefish_description/data/worlds/common/environment.scn:15` `pressure="101300.0"`.
  4. **검산**: 표면(101300 Pa) → 코드 depth `-9990 m`. 해수 10m(≈201818 Pa) → `-19910 m`. 물리적 불가능.
- **부차 이슈**: `* 10`(m/atm)도 해수 정확값 10.08(rho=1025) 대신 10 → ~0.8% 스케일 오차. `offset_z=2.5`(kalman.py TODO)가 이 오류를 부분 은폐한 것으로 보임.
- **정답식(P4 수정용)**: `depth = (P_abs − 101325) / (rho·g)`, 해수 rho=1025·g=9.80665 → `depth = -(fluid_pressure − 101325.0)/(1025·9.80665) − offset_z`(Z-up 부호). 출처: 해양물리(talleylab.ucsd.edu, WHOI), [표준중력 wiki].
- **현재 처리**: P3.2는 동작 보존 추출이라 **식 무변경**(`pressure_to_depth`가 버그를 그대로 보존, docstring에 경고). `test_kalman.py`가 현재 동작을 특성화 → **P4 수정 시 회귀 안전망**. 수정하면 특성화 테스트 기대값을 정답으로 교체.

## ICP 수렴 실패 — test_icp_recovers_known_translation

- **파일**: `stonefish_slam/test/test_pcl.py::test_icp_recovers_known_translation`
- **발견일**: 2026-06-23
- **증상**: shift=[0.3, -0.2] 평행이동 복원에서 max abs diff 0.138 (atol=0.05 초과). ICP가 40회 반복 후에도 tolerance 내 수렴 미달.
- **현재 처리**: `@pytest.mark.xfail(reason="ICP 수렴 tolerance 미달, P4 조사", strict=True)` — strict이라 P4에서 수렴이 고쳐지면 XPASS가 FAILED로 떠서 "xfail 마크를 제거하라"는 신호가 됨.
- **유력 근본 원인(P2 검토자가 소스에서 식별, 우선 조사 권고)**:
  - `pcl.py` ICP.compute의 `T_delta = np.eye(3, dtype=np.float32)` — float64 R·t가 float32로 다운캐스트된 뒤 `T = T_delta @ T`로 누적되어, 순수 평행이동(첫 iteration에 정답이 나와야 하는 가장 쉬운 케이스)에서도 40회 반복 중 float32 반올림이 증폭됨. **`dtype=np.float64`로 변경이 근본 수정 후보.**
  - `outlier_ratio=0.8`이 7점 점군에서 2점을 버리는데, 순수 shift라 모든 correspondence 거리가 동일 → sort 순서가 비결정적이고 float32 오차로 매 iteration 선택 5점 집합이 흔들려 centroid 추정이 불안정.
- **추가 조사 사항**:
  - ICP `max_iterations`(현재 40) 증가 또는 `tolerance`(현재 0.01) 조정 필요성
  - `outlier_ratio`(0.8) + `max_correspondence_distance`(3.0) 파라미터 튜닝
  - 소규모 점군(7점)에서의 ICP 수렴 특성 확인
  - atol 완화(0.15) 또는 ICP 파라미터 개선 중 방향 결정 필요

## fusion.ema_fusion — observation_count 인자 미사용 (의도 검증 필요)

- **파일**: `stonefish_slam/utils/fusion.py::ema_fusion`
- **발견일**: 2026-06-23 (P2 최종 whole-branch 검토에서 식별)
- **증상**: 시그니처·docstring은 `observation_count`로 첫 관측을 판정한다는 의도를 시사하나, 본문은 `observation_count`를 전혀 쓰지 않고 first-observation 판정을 `old_map <= threshold`로 한다. 테스트 `test_first_observation_uses_new_value`는 old[0]=0.0이 우연히 count[0]=0과 일치해 green이 됨.
- **현재 처리**: 코드 무수정(P2 0변경 원칙). 기록만.
- **조사 필요**: docstring 의도(count 기반 판정)와 실제 코드(threshold 기반)의 괴리가 버그인지 설계 변경 흔적인지 확인. 버그면 판정 로직 수정 또는 시그니처에서 미사용 인자 제거.

## kalman_predict/correct — P2 제외 사유 (추적용 기록)

- **파일**: `stonefish_slam`의 kalman 모듈
- **사유**: kalman.py가 module-top에서 `rclpy`+`gtsam`을 import해 importlib 파일 직접 로드 시점에 import 자체가 크래시함(P2 테스트 전략으로 격리 불가). spec 1차 후보였으나 의도적으로 P2 제외 → 지연 import 또는 메서드 추출(P3 모듈화) 후 테스트 가능.

## 노드명 3중 충돌 — standalone mapping 노드 (P3.0 컨벤션 조사에서 발견)

- **파일**: `stonefish_slam/nodes/mapping_2d_standalone_node.py:28`, `stonefish_slam/nodes/mapping_3d_standalone_node.py:30`
- **발견일**: 2026-06-23 (P3.0 명명·구조 외부 표준 대조 중)
- **증상**: 두 standalone 노드가 모두 `super().__init__('slam_node')`로 초기화되며, `core/slam.py:41`(`Node.__init__(self, 'slam_node')`)도 `'slam_node'`다. 셋 중 둘 이상이 한 ROS 그래프에 동시에 뜨면 ROS2 고유 노드명 요구(RMW)를 위반해 노드 등록 충돌·진단 혼선이 발생한다.
- **근거 표준**: 노드명 고유성은 RMW 강제 사항. [rmw validate_node_name.c](https://github.com/ros2/rmw/blob/master/rmw/src/validate_node_name.c).
- **수정안**: standalone 노드를 각자 고유 이름(`mapping_2d_node`·`mapping_3d_node`)으로 초기화. 동작 변경(노드명 토픽 네임스페이스에 영향 가능)이라 P4에서 처리.

## kalman 공분산 업데이트 — Joseph form 수치 안정성 (P3.1 자료조사에서 식별)

- **파일**: `stonefish_slam/core/kalman_filter.py::kalman_correct`
- **발견일**: 2026-06-23 (P3.1 kalman 추출의 자료조사 단계)
- **증상**: 공분산 보정에 단순형 `P = (I − KH)P⁻`(코드: `P − K@H@P`)를 쓴다. 이는 **최적 칼만 게인에서만** 정확하며, 부동소수점 누적으로 P의 대칭성·양정치성이 시간에 따라 깨질 수 있다(특히 장시간 필터링).
- **표준 근거**: 수치 안정형은 Joseph form `P = (I−KH)P⁻(I−KH)ᵀ + KRKᵀ` — 두 항이 각각 양반정치라 반올림으로도 indefinite가 되지 않는다. 출처: [kalman-filter.com/joseph-form](https://kalman-filter.com/joseph-form/), Welch & Bishop TR 95-041.
- **현재 처리**: P3.1은 **동작 보존**이 원칙이라 단순형 그대로 추출(식 무변경). 기록만.
- **수정안**: P4 수치 고도화에서 Joseph form 전환 검토. 전환 시 기존 테스트(최적 게인 가정)는 동등하게 통과해야 하고, 장시간 시퀀스에서 P 대칭성 유지를 추가 검증.

## ✅ wildcard import — PEP 8 위반 → **P3에서 해소(2026-06-24)**

- **발견일**: 2026-06-23 (P3.0 명명·구조 외부 표준 대조 중). 정확히 **17곳**(P4_FLAGS 초기 목록은 6모듈만 열거했으나 slam.py 4곳·dead_reckoning.py 2곳·cfar.py 2번째 등 누락분 있었음).
- **증상**: `from <module> import *` 사용. PEP 8은 "Wildcard imports should be avoided", Google Style은 모듈 단위 import만 허용해 둘 다 위반.
- **해소(P3 T4a/T4b)**: 정적 게이트(`test/static_import_gate.py`)로 17곳을 dead/live 분류 — 각 consumer가 wildcard로만 오는 의존 심볼을 세고, consumer 직접 바인딩은 차감. **dead 10곳 삭제**(conversions:20·sonar:6·cfar:5/6·dead_reckoning:23·types:11/12·feature_extraction:7/8/9), **live 7곳 명시화**(kalman:15·dead_reckoning:21·types:10·slam의 io/conversions/visualization/topics). 우리 소스 wildcard **0개** 달성. `test_wildcard_gate.py`가 명시 ImportFrom을 파싱해 골든 심볼 집합과 비교(동결). code-reviewer 독립검토로 dead 삭제 런타임 안전성·live 심볼 완전성 확인.
- **잔여 P4**: source 5모듈(conversions/topics/io/sonar/visualization)의 `__all__` 추가는 게이트 baseline 정합 위해 P4로 미룸(Open Q O7). C++ `.so` 상대 import(`__init__.py`)는 의도된 §2.2 예외라 불변(Open Q O8).

## dead_reckoning.py:175 — curr_depth 깊이 무력화 (P3 재감사에서 발견)

- **파일**: `stonefish_slam/core/dead_reckoning.py:175`(`curr_depth = 0.0`), :173-174(주석 처리된 압력→깊이식).
- **발견일**: 2026-06-24 (P3 동작보존 위험 지도 분석 중).
- **증상**: dead_reckoning 콜백이 압력→깊이 변환식을 주석 처리하고 `curr_depth = 0.0`을 하드코딩 → DR이 깊이를 항상 0으로 쓴다. 의도된 비활성화인지 미완성인지 불명. 주석 처리된 :173-174 식은 `kalman_filter.py:85`의 1000배 압력 버그(위 HIGH 플래그)와 동일식이라, 주석 부활 시 그 버그도 따라온다.
- **현재 처리**: P3는 동작 보존이라 현 동작(=0)을 그대로 둠. 현재 출력이 이미 상수 0이라 P3에서 건드릴 것 없음.
- **수정안**: P4에서 깊이를 실제 압력 변환으로 복원할지 결정(복원 시 pressure 1000배 버그를 정답식으로 동시 수정). pressure→depth 플래그와 함께 처리.

## dead_reckoning.py docstring 내부 탭 (P3 T2 잔여)

- **파일**: `stonefish_slam/core/dead_reckoning.py` — 4개 docstring(클래스 + 메서드 Args 블록)의 내부 들여쓰기가 여전히 탭.
- **발견일**: 2026-06-24 (P3 T2 탭→4space 변환 중).
- **증상**: T2에서 코드 들여쓰기 탭은 4-space로 변환했으나, docstring 내부 탭은 문자열 *데이터*(`__doc__` 값)라 변환하면 `__doc__`의 raw 값이 바뀐다(동작 변경). 따라서 코드 들여쓰기만 변환하고 docstring 내부 탭은 보존했다(`ast.dump` before==after 유지).
- **현재 처리**: 보존(동작 보존). 코드 들여쓰기는 4-space로 통일됨.
- **수정안**: P4에서 docstring 내부 탭→space 정규화 시 `__doc__` raw 값 변경을 명시 수용하고(`inspect.getdoc` dedent 결과는 불변임을 확인 후), 다른 모듈의 docstring 스타일과 함께 일괄 정리.

## kalman.py — 미해결 외부 의존성 `uuv_sensor_ros_plugins_msgs` (P3 재감사에서 발견)

- **파일**: `stonefish_slam/core/kalman.py:11`(`from uuv_sensor_ros_plugins_msgs.msg import DVL`, 무가드 모듈-top import).
- **발견일**: 2026-06-24 (P3 재감사 build-packaging 차원, 적대 검증 통과).
- **증상**: `kalman_node`(CMakeLists install PROGRAMS로 등록됨)를 실행하면 `kalman.py:11`이 import되는데, `uuv_sensor_ros_plugins_msgs` 패키지가 워크스페이스에도 시스템(`/opt/ros/*/share/`)에도 **존재하지 않는다**(실측 확인). try/except fallback도 없어 `ImportError`로 노드가 죽는다. 형제 노드 `dead_reckoning.py`는 동일 DVL 메시지를 sim repo에 실존하는 `stonefish_msgs`에서 가져오는데(이번 P3에서 package.xml에 선언 보강), `kalman.py`만 외부 UUV Simulator 패키지를 가리킨다.
- **근거 표준**: [REP 149](https://www.ros.org/reps/rep-0149.html) — `find_package`/import하는 런타임 의존성은 package.xml에 선언되어 rosdep이 설치하도록 해야 한다.
- **현재 처리**: 보류. 존재하지 않는 rosdep 키를 package.xml에 넣으면 빌드/rosdep이 깨질 수 있어 추가하지 **않았다**. `stonefish_msgs`(실존)만 이번 P3에서 선언 보강했다.
- **수정안**: P4에서 둘 중 하나 — (a) `kalman.py`가 실제 사용되는 노드면 DVL 출처를 `stonefish_msgs`로 통일(`dead_reckoning.py`와 일치, 동작 변경)하거나, (b) UUV Simulator 외부 패키지 배포를 전제로 `uuv_sensor_ros_plugins_msgs`를 rosdep 키로 선언. `kalman.py`가 legacy dead 노드인지 먼저 확인 필요.

---

## P3 작업 요약 (2026-06-24, 동작 보존 — 참고)

P3(기업표준 구조·명명 통일·재구조화·모듈화)에서 **동작 보존이 가능한 것만** 처리했다. 베이스라인 pytest 16 passed+1 xfailed → 20 passed+1 xfailed(신규 안전망 테스트 포함, live 동작 불변).
- **PREREQ**: AST 정적 게이트(`test/static_import_gate.py`) + wildcard 분류 동결(`test_wildcard_gate.py`) + octree adaptive 0.3 config 커버갭(`test_octree.py`).
- **T1**: CONVENTIONS 줄번호 drift 정정(§2.3·§2.5)·world_ned 혼용 기록(§2.0)·예외 3노드 일반화(§2.1)·P3 안전망 절(§2.8)·P4 백로그(§3).
- **T2**: dead_reckoning.py 코드 들여쓰기 탭→4-space(ast.dump 동등·diff -w=0).
- **T3**: slam.py dead `pointcloud2_to_xyz_array`(+orphan `import struct`) 삭제(사용자 승인).
- **T4a/T4b**: wildcard 17곳 정리(dead 10 삭제 + live 7 명시화 → 0).
- **T5**: octree 커버갭 + mapping_3d 무상태 메서드 2개(`create_transform_matrix`·`pose_msg_to_transform`)를 모듈 함수로 추출(실행 statement AST 동등·외부 호출자 0).
- **P4 격리**: 노드명 3중충돌·standalone 구조·god-method 분해·수치버그(pressure/ICP/fusion/Joseph)·polar_to_cartesian 통합·rename군·PascalCase 파라미터·frame_id world_ned·`__all__` 추가·docstring 탭·depth 복원 — 전부 런타임/수치 변경이라 P4.

---

## 구조 전수 감사 후속 정리 (2026-06-24, 동작 보존)

이전 P3가 예시 폴더만 보고 "구조 재정리 불필요"로 일반화한 오류를 바로잡고자, 두 repo 전체 트리를 ROS2/Python 표준에 전수 대조(41 에이전트, 122 관찰, 적대 검증). 그 결과 **동작보존인데 P3에서 빠진 구조 결함**을 처리했다. 시스템 pybind11(`/usr/lib/cmake/pybind11/`)이 `find_package`로 잡히고 vendored 경로는 CMakeLists 어디서도 참조 안 됨을 실측 확인 → 제거해도 .so 빌드 불변(pytest 20 passed+1xfailed 유지).
- **vendored pybind11 제거**: `stonefish_slam/cpp/pybind11/` 346파일 `git rm`(시스템 pybind11 사용, 빌드 dead tree였음). 무의미해진 `pytest.ini`의 `--ignore`/`cpp/pybind11` 규칙도 정리(일반 `*/pybind11/*` 방어는 유지).
- **orphan 진단스크립트 명시화**: `scripts/diagnose_center_dip.py`(178줄, install 미등록 ad-hoc 진단노드)는 `nodes/`(배포 진입점)가 아니라 `scripts/`가 정당한 자리임을 `scripts/README.md`로 문서화(이동 아님 — `nodes/`로 옮기면 오히려 배포 노드로 오인). 감사의 "nodes/로 이동" 제안은 과교정이라 기각.
- **(sim repo 대칭 정리)**: `stonefish_thruster_manager/launch/{build,install,log}` 커밋된 colcon 산출물 21파일 `git rm` + `.gitignore`에 `**/build|install|log` 보강. `stonefish_ros2/package.xml`의 가짜 rosidl 2줄(`rosidl_default_generators`·`member_of_group rosidl_interface_packages` — `rosidl_generate_interfaces()` 없음) 제거.

---

## localization.yaml — icp_config 하드코딩 절대경로 (P3 재감사에서 발견)

- **파일**: `config/localization.yaml:29`(`icp_config: '/workspace/colcon_ws/src/stonefish_slam/config/icp.yaml'`).
- **발견일**: 2026-06-24 (P3 재감사 config-launch 차원).
- **증상**: `slam.py:96-98`이 `os.path.join(...)`로 패키지 상대 기본값을 계산해 `declare_parameter('icp_config', ...)`로 선언하지만, `localization.yaml:29`가 이를 특정 머신 절대경로(`/workspace/colcon_ws/...`)로 오버라이드한다. `slam.py:528-530`이 그 값으로 `self.localization.icp.loadFromYaml(icp_config)`를 실제 호출 → 이 경로의 파일을 런타임에 로드한다. 현재 워크스페이스는 `/workspace/src/`라 `/workspace/colcon_ws/` 경로는 존재하지 않으므로, 이 오버라이드가 적용되면 ICP 설정 로드가 실패하거나 빈 설정으로 동작한다(머신 의존·재현 불가).
- **근거 표준**: ROS2 launch는 `ament_index`(`get_package_share_directory`)로 패키지 상대 config 경로를 런타임 해석하는 게 표준 — 절대경로 하드코딩은 비표준([ROS2 Launch substitutions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html)).
- **현재 처리**: 보류. 이 값을 바꾸면 `loadFromYaml`이 로드하는 파일이 달라져 런타임 거동이 변하므로(동작보존 불가) P3에서 손대지 않았다.
- **수정안**: P4에서 `localization.yaml:29`의 절대경로를 제거(코드의 패키지 상대 기본값에 위임)하거나 launch substitution으로 대체. ICP 설정 로드 거동 변화를 명시 수용 후 처리.
