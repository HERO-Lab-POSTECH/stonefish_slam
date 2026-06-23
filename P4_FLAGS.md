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

- **파일**: `stonefish_slam/nodes/mapping_2d_standalone_node.py:29`, `stonefish_slam/nodes/mapping_3d_standalone_node.py:30`
- **발견일**: 2026-06-23 (P3.0 명명·구조 외부 표준 대조 중)
- **증상**: 두 standalone 노드가 모두 `super().__init__('slam_node')`로 초기화되며, `core/slam.py:94`(`Node.__init__(self, 'slam_node')`)도 `'slam_node'`다. 셋 중 둘 이상이 한 ROS 그래프에 동시에 뜨면 ROS2 고유 노드명 요구(RMW)를 위반해 노드 등록 충돌·진단 혼선이 발생한다.
- **근거 표준**: 노드명 고유성은 RMW 강제 사항. [rmw validate_node_name.c](https://github.com/ros2/rmw/blob/master/rmw/src/validate_node_name.c).
- **수정안**: standalone 노드를 각자 고유 이름(`mapping_2d_node`·`mapping_3d_node`)으로 초기화. 동작 변경(노드명 토픽 네임스페이스에 영향 가능)이라 P4에서 처리.

## kalman 공분산 업데이트 — Joseph form 수치 안정성 (P3.1 자료조사에서 식별)

- **파일**: `stonefish_slam/core/kalman_filter.py::kalman_correct`
- **발견일**: 2026-06-23 (P3.1 kalman 추출의 자료조사 단계)
- **증상**: 공분산 보정에 단순형 `P = (I − KH)P⁻`(코드: `P − K@H@P`)를 쓴다. 이는 **최적 칼만 게인에서만** 정확하며, 부동소수점 누적으로 P의 대칭성·양정치성이 시간에 따라 깨질 수 있다(특히 장시간 필터링).
- **표준 근거**: 수치 안정형은 Joseph form `P = (I−KH)P⁻(I−KH)ᵀ + KRKᵀ` — 두 항이 각각 양반정치라 반올림으로도 indefinite가 되지 않는다. 출처: [kalman-filter.com/joseph-form](https://kalman-filter.com/joseph-form/), Welch & Bishop TR 95-041.
- **현재 처리**: P3.1은 **동작 보존**이 원칙이라 단순형 그대로 추출(식 무변경). 기록만.
- **수정안**: P4 수치 고도화에서 Joseph form 전환 검토. 전환 시 기존 테스트(최적 게인 가정)는 동등하게 통과해야 하고, 장시간 시퀀스에서 P 대칭성 유지를 추가 검증.

## wildcard import — PEP 8 위반 (P3.0 컨벤션 조사에서 발견)

- **파일**: 여러 모듈 — `utils/conversions.py:20`(`from .topics import *`), `utils/sonar.py:6`, `core/types.py:10-12`(conversions·visualization·io), `core/kalman.py:15`, `core/feature_extraction.py:7-9`, `core/cfar.py:5` 등.
- **발견일**: 2026-06-23 (P3.0 명명·구조 외부 표준 대조 중)
- **증상**: `from <module> import *` 사용. PEP 8은 "Wildcard imports should be avoided", Google Style은 모듈 단위 import만 허용해 둘 다 위반. 네임스페이스에 어떤 이름이 들어오는지 불투명해 정적 분석·가독성을 해친다.
- **현재 처리**: 코드 무수정(P3.0은 문서 작업). 기록만 — 새 코드는 wildcard 금지(CONVENTIONS.md §2.2).
- **수정안**: 명시적 import로 전환. 레거시라 P4(또는 해당 모듈 리팩토링 시) 처리.
