# P4 Investigation Flags

> **2026-09-01 갱신 패스.** 양 repo 코드 건강 감사(37건 적대 검증)에 맞춰 이 파일의 **유령 백로그를
> 정리**했다 — 삭제된 파일을 가리키던 항목, 이미 분해된 god-method, 반증된 가설에 각각 종결·정정
> 배너를 달았다(원 서술은 이력으로 보존). **이번 감사에서 새로 확정된 결함은 이 파일이 아니라
> `.hq/community/posts/finding/008-2026-09-01-code-health-audit.md`가 SSOT**이며, 원 보고 전문은
> `.hq/work/project/audit-2026-09-01/`에 있다. 두 곳에 같은 항목을 중복 기재하지 말 것.


## ✅ 압력→깊이 변환 Pa/kPa 1000배 단위 버그 — **P4b 해결(`683ea4a`, 2026-06-24)**

> **해결**: `core/depth.py` 순수함수 `pressure_to_depth()`로 정답식 구현 — `(P − 101325) / (1025 · 9.80665)`, NED z-down(수면→0, 10m→+10). `dead_reckoning.py`가 이를 사용(`curr_depth=0.0` 하드코딩 제거), `offset=2.5` fudge 제거. 출처 `kalman_filter.py::pressure_to_depth`는 kalman 모듈과 함께 삭제됨(`3e2a2b4`). 아래는 발견 당시 기록(보존).

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

## ✅ ICP 수렴 실패 — test_icp_recovers_known_translation — **P4a 해결(`1b037ca`, 2026-06-24)**

> **해결 + 근본원인 정정**: float32 다운캐스트 가설은 **실증 반박**됨(float32/float64 트레이스 바이트 동일). 실제 근본원인은 고정 `outlier_ratio=0.8`이 perfect-overlap에서도 대응점 20%를 잘라 Kabsch centroid를 편향시킨 것(TrICP, Chetverikov 2002는 trim 비율 = 실제 overlap 요구). `pcl.py` `outlier_ratio 0.8→1.0`으로 수정(`max_correspondence_distance=3.0`이 실제 outlier 거부). xfail 제거 + atol 1e-6 정밀 테스트 추가. C++ 경로(libpointmatcher, 런타임 live)는 0.8이 옳아 불변. 아래 "float32 다운캐스트가 근본 후보"는 **틀린 가설**로 판명(보존, 교훈용).

- **파일**: `stonefish_slam/test/test_pcl.py::test_icp_recovers_known_translation`
- **발견일**: 2026-06-23
- **증상**: shift=[0.3, -0.2] 평행이동 복원에서 max abs diff 0.138 (atol=0.05 초과). ICP가 40회 반복 후에도 tolerance 내 수렴 미달.
- **~~유력 근본 원인(P2 검토자가 소스에서 식별)~~ — float32 가설은 P4에서 반박됨**:
  - ~~`pcl.py` ICP.compute의 `T_delta = np.eye(3, dtype=np.float32)` 다운캐스트가 근본 수정 후보~~ → **반박**: float32/float64 동일 결과. 실제 원인은 `outlier_ratio=0.8`의 비대칭 trim(위 해결 노트).
  - `outlier_ratio=0.8`이 7점 점군에서 2점을 버리는데, 순수 shift라 → centroid 추정 불안정. **이것이 진짜 근본원인이었음**(0.8→1.0으로 해결).

## `fix/loc-critical`의 GPU 머신 sign-off 대상 (2026-09-01)

> 이 브랜치의 수정은 전부 정적 테스트로 고정했으나, 아래 4건은 **닫힌루프 실기에서만
> 관측 가능한 효과**를 갖는다. 결함 SSOT는
> `.hq/community/posts/finding/008-2026-09-01-code-health-audit.md`이고 여기는 sign-off
> 항목만 등재한다(중복 기재 금지).

- **P0-4 WEIGHTED_AVERAGE delta 변환** — 3D 맵 확률값이 바뀐다. 단위 테스트는 수학만
  고정하고 맵 품질은 못 본다. 확인: `config/mapping/method_weighted_avg.yaml`로
  `slam.launch.py`를 돌려 OctoMap이 clamp 상·하한(0.97/0.03)에 몰리지 않고 강도에
  비례해 분포하는지. ⚠️ shipped 기본값은 IWLO라 이 경로는 명시 선택 시에만 탄다.
- **P0-3 ISAM2 실패 후 저하 경로** — 실데이터에서 이 예외가 실제로 나는지 자체가
  미측정이다. 확인: 실해역 bag 재생 중
  `[FactorGraph] ISAM2 update failed` / `marginalCovariance(...) failed` 로그가 뜨는지,
  뜬다면 그 이후 궤적이 이어지는지. 잃는 것은 그 틱의 제약 하나다(실측 근거는
  `test_isam_recovers_from_a_real_indeterminate_system` 주석).
- **P1-14 NSSM 큐 aging** — `update_graph`의 pose 갱신이 O(N) 전체 경로로 되돌아가는
  빈도가 줄어든다. 확인: 장기 주행에서 키프레임당 `update_graph` 소요가 N에 비례해
  증가하지 않는지(계측은 다음 브랜치 `feat/loc-instrumentation` I1~I3).
- **P0-1 순수 Python ICP fallback** — 프로덕션 이미지는 `.so`를 포함하므로 이 경로를
  타지 않는다. `.so` 없는 환경에서만 동작이 바뀌며, 그 환경이 실기에 쓰이면
  `max_correspondence_distance`·`max_iterations`가 이제 `icp.yaml`을 따른다는 점을
  확인한다.

## fusion.ema_fusion — observation_count 인자 미사용 — **2026-09-01 판정: 실버그 아님, 순수 정리 건**

> **적대 검증 결과(SLAM-M2, PARTIAL)**: "실버그로 승격"은 성립하지 않는다. 두 판정이 갈리려면
> `count>0 ∧ old_map<=0.0`, 즉 어떤 셀이 **정확히 0.0인 관측값으로** 한 번 쓰여야 하는데,
> `mapping_2d.py:573`의 정규화가 `raw > threshold`(strict, uint8)를 항상 >0으로 보내므로 그 상태를
> 만드는 코드 경로가 없다. 노이즈 저감이 "다수 셀에서 무너진다"는 서술은 반증됐다.
> 다만 이 동치성은 (a) 그 strict `>` 필터와 (b) 호출부가 `threshold=0.0`을 넘긴다는 **두 외부 불변식에
> 전적으로 의존**한다 — 필터를 `>=`로 바꾸거나 다른 호출자가 `threshold>0`을 넘기면 즉시 갈린다.
> **처방은 판정 로직 수정이 아니라 시그니처에서 `observation_count`를 제거**(또는 count 기반으로 통일)해
> 이 암묵 의존을 없애는 쪽이다. 현행 동작 불변 = 회귀 위험 0.
> 부수 관찰(조치 권하지 않음): `mapping_2d.py:465`의 `global_map_count`가 float64인데 용도는 정수 카운트,
> `test/test_mapping_2d_accumulation.py:85`는 int32로 스텁한다.


- **파일**: `stonefish_slam/utils/fusion.py::ema_fusion`
- **발견일**: 2026-06-23 (P2 최종 whole-branch 검토에서 식별)
- **증상**: 시그니처·docstring은 `observation_count`로 첫 관측을 판정한다는 의도를 시사하나, 본문은 `observation_count`를 전혀 쓰지 않고 first-observation 판정을 `old_map <= threshold`로 한다. 테스트 `test_first_observation_uses_new_value`는 old[0]=0.0이 우연히 count[0]=0과 일치해 green이 됨.
- **현재 처리**: 코드 무수정(P2 0변경 원칙). 기록만.
- **조사 필요**: docstring 의도(count 기반 판정)와 실제 코드(threshold 기반)의 괴리가 버그인지 설계 변경 흔적인지 확인. 버그면 판정 로직 수정 또는 시그니처에서 미사용 인자 제거.

## `fix/map-and-metrics`의 GPU 머신 sign-off 대상 (2026-09-01)

> 정적 테스트가 수식은 고정하지만 아래 3건은 **닫힌루프 실기에서만 판정 가능한 효과**를
> 갖는다. 결함 SSOT는 `.hq/community/posts/finding/008-2026-09-01-code-health-audit.md`이고
> 여기는 sign-off 항목만 등재한다(중복 기재 금지).

- **P1-1 2D 맵 `range_min` 복원** — 전 맵 점이 0.5 m(수평 0.433 m) 바깥으로 이동한다.
  확인: 같은 bag을 수정 전후로 돌려 벽·구조물 반사가 GT 지오메트리에 더 가까워지는지.
  ⚠️ pose는 불변이므로 ATE로는 안 보인다 — 맵 이미지를 직접 비교해야 한다.
- **P1-7 `gaussian_sigma_factor` 배선** — 프로덕션 3D 경로의 수직 개구 가중이 3.0에서
  2.5(또는 오퍼레이터 지정값)로 바뀐다. 확인: `method_iwlo.yaml`로 돌려 OctoMap의
  수직 퍼짐이 좁아지는지, `mapping_3d.gaussian_sigma_factor`를 바꿨을 때 실제로
  반응하는지(지금까지는 반응하지 않았다).
- **P1-17 드리프트 지표 재정의** — `drift_window`가 실기에서 의미 있는 값을 내는지 자체가
  미측정이다. 확인: 실해역 bag 재생 중 값이 0%나 100%에 고착하지 않고 궤적 품질을 따라
  움직이는지. 특히 **정지 호버링 구간**에서 GT 경로 길이가 0에 가까워지면
  `distance_epsilon`이 분모를 지배해 값이 폭주할 수 있다 — 실기에서 그 구간을 확인한다.

## ✅ kalman_predict/correct — P2 제외 사유 — **무효화: kalman 모듈 제거(`3e2a2b4`, 2026-06-24)**

> **무효**: kalman 모듈 전체가 legacy-dead로 제거됨(아래 uuv 의존성 항목 참조). 더 이상 테스트 대상 아님.

- **파일**: `stonefish_slam`의 kalman 모듈
- **사유**: kalman.py가 module-top에서 `rclpy`+`gtsam`을 import해 importlib 파일 직접 로드 시점에 import 자체가 크래시함(P2 테스트 전략으로 격리 불가). spec 1차 후보였으나 의도적으로 P2 제외 → 지연 import 또는 메서드 추출(P3 모듈화) 후 테스트 가능.

## ✅ 노드명 'slam_node' 공유 — **P4d 측정 결과 런타임 충돌 아님**(2026-06-24)

- **파일**: `stonefish_slam/nodes/mapping_2d_standalone_node.py:28`, `stonefish_slam/nodes/mapping_3d_standalone_node.py:30`, `core/slam.py:41` — 셋 다 노드명 `'slam_node'`.
- **최초 가설(P3.0, 2026-06-23)**: "셋 중 둘 이상이 한 ROS 그래프에 동시에 뜨면 RMW 고유 노드명 요구 위반 → 충돌".
- **P4d 측정으로 가설 반증(2026-06-24, 정적 grep + launch 전수 확인)**:
  1. `slam.launch.py`는 standalone 노드를 include하지 않음 — `slam_node` 1개만 뜸.
  2. `mapping_2d_standalone.launch.py`·`mapping_3d_standalone.launch.py`는 각자 별도 executable을 **단독** 실행 — 각 그래프에 `slam_node` 1개.
  3. 유일하게 둘을 함께 띄우는 `mapping_combined_standalone.launch.py`는 **각각을 `PushRosNamespace('mapping_2d')`/`PushRosNamespace('mapping_3d')`로 분리** → 완전 노드 경로가 `/mapping_2d/slam_node`·`/mapping_3d/slam_node`로 **고유**. RMW 충돌 없음.
  4. 이름 공유는 **의도적 설계**: standalone launch 주석 `# Must match yaml namespace (slam_node.ros__parameters)` — config YAML의 `slam_node.ros__parameters` 네임스페이스와 일치시켜 같은 mapping.yaml을 재사용하기 위함.
- **결론**: 모든 실행 경로에서 노드명 충돌이 실재하지 않는다(combined 경로조차 네임스페이스로 분리). **버그 아님 → P4 수정 대상에서 제외.** 코드 명확성 차원에서 standalone에 고유 이름을 줄 수는 있으나, 그러면 config YAML 네임스페이스도 함께 바꿔야 하고 동작상 이득이 없어 **불변 유지**가 정당하다. (교훈: "동시에 뜨면 충돌"은 이론적으로 맞으나 그런 실행 경로가 없거나 네임스페이스로 분리됨을 확인하지 않은 단정이었다.)
- **근거 표준**: 노드명 고유성은 `namespace + node_name` 전체 경로에 적용된다 — [rmw validate_node_name.c](https://github.com/ros2/rmw/blob/master/rmw/src/validate_node_name.c), [ROS2 Node naming](https://design.ros2.org/articles/topic_and_service_names.html).

## ✅ kalman 공분산 업데이트 — Joseph form — **무효화: kalman 모듈 제거(`3e2a2b4`, 2026-06-24)**

> **무효**: `kalman_filter.py`가 kalman 모듈과 함께 삭제됨(legacy-dead). Joseph form 전환은 더 이상 대상 아님. (수중 SLAM에서 칼만 필터를 살릴 경우 이 노트가 출발점으로 유효하나, 현 코드베이스엔 칼만 필터가 없음 — 위치추정은 sim odometry 직접 구독.)

- **파일**: `stonefish_slam/core/kalman_filter.py::kalman_correct`
- **발견일**: 2026-06-23 (P3.1 kalman 추출의 자료조사 단계)
- **증상**: 공분산 보정에 단순형 `P = (I − KH)P⁻`(코드: `P − K@H@P`)를 쓴다. 이는 **최적 칼만 게인에서만** 정확하며, 부동소수점 누적으로 P의 대칭성·양정치성이 시간에 따라 깨질 수 있다(특히 장시간 필터링).
- **표준 근거**: 수치 안정형은 Joseph form `P = (I−KH)P⁻(I−KH)ᵀ + KRKᵀ` — 두 항이 각각 양반정치라 반올림으로도 indefinite가 되지 않는다. 출처: [kalman-filter.com/joseph-form](https://kalman-filter.com/joseph-form/), Welch & Bishop TR 95-041.
- **현재 처리**: P3.1은 **동작 보존**이 원칙이라 단순형 그대로 추출(식 무변경). 기록만.
- **수정안**: P4 수치 고도화에서 Joseph form 전환 검토. 전환 시 기존 테스트(최적 게인 가정)는 동등하게 통과해야 하고, 장시간 시퀀스에서 P 대칭성 유지를 추가 검증.

## ✅ wildcard import — PEP 8 위반 → **P3에서 해소(2026-06-24)**

- **발견일**: 2026-06-23 (P3.0 명명·구조 외부 표준 대조 중). 정확히 **17곳**(P4_FLAGS 초기 목록은 6모듈만 열거했으나 slam.py 4곳·dead_reckoning.py 2곳·cfar.py 2번째 등 누락분 있었음).
- **증상**: `from <module> import *` 사용. PEP 8은 "Wildcard imports should be avoided", Google Style은 모듈 단위 import만 허용해 둘 다 위반.
- **해소(P3 T4a/T4b)**: 정적 게이트(`test/static_import_gate.py`)로 17곳을 dead/live 분류 — 각 consumer가 wildcard로만 오는 의존 심볼을 세고, consumer 직접 바인딩은 차감. **dead 10곳 삭제**(conversions:20·sonar:6·cfar:5/6·dead_reckoning:23·types:11/12·feature_extraction:7/8/9), **live 7곳 명시화**(kalman:15·dead_reckoning:21·types:10·slam의 io/conversions/visualization/topics). 우리 소스 wildcard **0개** 달성. `test_wildcard_gate.py`가 명시 ImportFrom을 파싱해 골든 심볼 집합과 비교(동결). code-reviewer 독립검토로 dead 삭제 런타임 안전성·live 심볼 완전성 확인. (이는 **P3 시점 기록**이다. P4에서 `kalman.py` 모듈 제거(`3e2a2b4`)로 `kalman:15` live wildcard가 사라져 현재 live는 6모듈이며, `test_wildcard_gate.py` 골든 집합도 그 커밋에서 함께 갱신됨.)
- **잔여 P4**: source 5모듈(conversions/topics/io/sonar/visualization)의 `__all__` 추가는 게이트 baseline 정합 위해 P4로 미룸(Open Q O7). **[2026-09-01 재측정]** 실제로 `__all__`이 없는 `utils/` 파일은 5개가 아니라 **7개 전부**다 — 위 5개에 `fusion.py`·`profiler.py`가 추가된다(`utils/__init__.py`만 보유). C++ `.so` 상대 import(`__init__.py`)는 의도된 §2.2 예외라 불변(Open Q O8).

## ✅ dead_reckoning.py — curr_depth 깊이 무력화 — **P4b 해결(`683ea4a`, 2026-06-24)**

> **해결**: `curr_depth = 0.0` 하드코딩 제거, `core/depth.py::pressure_to_depth`로 실제 깊이 계산(위 압력 1000배 버그 항목과 동일 수정). NED z-down 정답식, `offset=2.5` fudge 제거. 단 SLAM은 현재 sim odometry를 직접 구독하므로 이 수정은 DR 노드 자체 출력만 바꾸고 맵은 DR 재배선 전까지 latent.

- **파일**: `stonefish_slam/core/dead_reckoning.py:175`(`curr_depth = 0.0`), :173-174(주석 처리된 압력→깊이식).
- **발견일**: 2026-06-24 (P3 동작보존 위험 지도 분석 중).
- **증상**: dead_reckoning 콜백이 압력→깊이 변환식을 주석 처리하고 `curr_depth = 0.0`을 하드코딩 → DR이 깊이를 항상 0으로 쓴다. 의도된 비활성화인지 미완성인지 불명. 주석 처리된 :173-174 식은 `kalman_filter.py:85`의 1000배 압력 버그(위 HIGH 플래그)와 동일식이라, 주석 부활 시 그 버그도 따라온다.
- **현재 처리**: P3는 동작 보존이라 현 동작(=0)을 그대로 둠. 현재 출력이 이미 상수 0이라 P3에서 건드릴 것 없음.
- **수정안**: P4에서 깊이를 실제 압력 변환으로 복원할지 결정(복원 시 pressure 1000배 버그를 정답식으로 동시 수정). pressure→depth 플래그와 함께 처리.

## dead_reckoning 깊이 부호 — **문서 결함으로 판정(2026-08-21), 실기 검증 이월**

- **파일**: `core/depth.py::pressure_to_depth`(NED 반환) → `core/dead_reckoning.py:165`(`curr_depth`) → `send_odometry`의 pose z → `/dead_reck/odom`·`odom→base_link` TF.
- **제기된 의심**: NED 깊이(+ = 깊음)가 문서상 REP-105 ENU(z-up)로 기술된 `odom→base_link` 체인에 `+z`로 그대로 발행된다 → 부호 오류인가?
- **소비자 전수 추적 결과 — 코드 무결함, 문서 결함**:
  1. dead_reckoning이 발행하는 `/dead_reck/odom`·`/dead_reck/path`·`/dead_reck/key_traj`를 **repo 내에서 구독하는 곳이 0개**(전수 grep, `.py`·`.yaml`·`.rviz`).
  2. SLAM은 `LOCALIZATION_ODOM_TOPIC = "/bluerov2/odometry"` — 시뮬레이터의 NED odometry를 직접 구독(`utils/topics.py:18`의 명시 결정 주석).
  3. `launch/slam.launch.py`는 dead_reckoning 노드를 include하지 않음.
  4. `rviz/stonefish_slam.rviz`에 `/dead_reck` 참조 없음.
  5. 이 체인의 TF는 모두 identity → 어디에도 회전 변환 없음.
  - 따라서 체인 전체가 **NED로 자기정합**이고 하위 보상 부호반전도 없다. 부호를 뒤집으면 발행 깊이가 오히려 틀려진다.
- **처리**: 코드 무변경. `docs/CONVENTIONS.md` §2.0과 `CLAUDE.md`의 "로컬 체인은 REP-105 ENU" 기술을 "이름만 REP-105, 데이터는 NED"로 정정. `test/test_dead_reckoning_depth_frame.py`가 (a) 깊이 positive-down, (b) `curr_depth`에 부호반전 없음, (c) `/dead_reck` 구독자 0 — 이 추적의 전제 3개를 동결한다. 특히 (c)가 깨지면 새 소비자가 규약을 결정하므로 부호 문제를 **재개**해야 한다.
- **이월(실기 sign-off 필요)**: 이 판정은 정적 추적이다. 실제 pressure 센서를 단 차량에서 하강 시 `/dead_reck/odom`의 `pose.position.z`가 **증가**하는지(NED), RViz에서 `odom→base_link` TF가 수면 아래로 내려가는지 GPU 머신 실기로 확인한다(`docs/RUN_TEST.md` 절차). dead_reckoning을 SLAM 입력으로 재배선하는 작업이 생기면 그 시점에 반드시 함께 검증한다.

## dead_reckoning.py docstring 내부 탭 (P3 T2 잔여)

- **파일**: `stonefish_slam/core/dead_reckoning.py` — 4개 docstring(클래스 + 메서드 Args 블록)의 내부 들여쓰기가 여전히 탭.
- **발견일**: 2026-06-24 (P3 T2 탭→4space 변환 중).
- **증상**: T2에서 코드 들여쓰기 탭은 4-space로 변환했으나, docstring 내부 탭은 문자열 *데이터*(`__doc__` 값)라 변환하면 `__doc__`의 raw 값이 바뀐다(동작 변경). 따라서 코드 들여쓰기만 변환하고 docstring 내부 탭은 보존했다(`ast.dump` before==after 유지).
- **현재 처리**: 보존(동작 보존). 코드 들여쓰기는 4-space로 통일됨.
- **수정안**: P4에서 docstring 내부 탭→space 정규화 시 `__doc__` raw 값 변경을 명시 수용하고(`inspect.getdoc` dedent 결과는 불변임을 확인 후), 다른 모듈의 docstring 스타일과 함께 일괄 정리.

## ✅ kalman.py — 외부 의존성 `uuv_sensor_ros_plugins_msgs` — **P4a 해결(`3e2a2b4`, 2026-06-24)**

> **해결(O1 결정 = kalman_node 제거)**: 수정안 (a)·(b) 대신 제3안 — kalman_node가 **legacy-dead**임이 4중 증거로 확정돼 모듈 통째 제거. 증거: ①launch 참조 0 ②uuv import `ModuleNotFoundError` 실측 ③출력 토픽(`kalman_odom`/`kalman_path`) 구독처 0 ④`topics.py:18` "sim odom 직접 사용" 결정 주석. `kalman_filter.py` 4함수 전부 kalman.py 전용(grep 확정) → 파일·노드·CMakeLists·test_kalman 일괄 삭제. dead_reckoning은 live(`stonefish_msgs.DVL`)라 불변. 부재 rosdep 키를 package.xml에 넣지 않은 P3 판단이 옳았음.

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

## ✅ localization.yaml — icp_config 하드코딩 절대경로 — **해소(`chore/dead-code-cleanup`, 2026-09-01)**

> 계획 §4.3 으로 `config/localization.yaml` 에서 이 키를 삭제했다. 이제 `slam.py:105-107` 의
> package-share 기본값에 위임된다. 아래 서술은 이력으로 보존한다.

> **P4d 측정(O9)**: `localization.yaml:29`의 하드코딩 경로 `/workspace/colcon_ws/...`는 파일이 **존재하지 않고**, `launch.py:30,46`이 이미 패키지 상대 경로로 오버라이드하므로 **dead fallback**. 즉 런타임에 이 값은 실제로 안 쓰임 → 안전 제거 가능. 단 코드 동작에 영향 없는 cleanup이라 P4 핵심 수정에선 제외(미실행). 향후 cleanup pass 또는 sim 통합 단계에서 제거.
>
> **[2026-09-01 재측정 — 줄번호 정정 + 범위 한정]** 실제 위치는 `localization.yaml:29`가 아니라 **`:30`**이다.
> 그리고 위험 범위는 아래 본문 서술보다 좁다: `slam.launch.py:57`이 `param_dict['icp_config']`에 올바른
> package-share 경로를 넣고 이 dict가 파라미터 리스트 **마지막**(`:87`)에 오므로 **launch 경로에서는 항상
> 올바른 값이 이긴다.** 잘못된 경로가 실제로 로드되는 것은 `ros2 run stonefish_slam slam_node`로 노드를
> **직접** 띄울 때뿐이다. 아래 "이 오버라이드가 적용되면 로드가 실패한다"는 서술은 그 한 경우로 한정해 읽을 것.
> 또한 `pcl.so`가 live라 C++ `loadFromYaml`이 실제로 호출된다(순수 파이썬 fallback의 `cpp/pcl.py:134-140`은
> `print`만 하는 no-op stub이라 무해했을 것 — 그 no-op 자체가 별건의 HIGH 결함이다, finding/008 #1).

- **파일**: `config/localization.yaml:29`(`icp_config: '/workspace/colcon_ws/src/stonefish_slam/config/icp.yaml'`).
- **발견일**: 2026-06-24 (P3 재감사 config-launch 차원).
- **증상**: `slam.py:96-98`이 `os.path.join(...)`로 패키지 상대 기본값을 계산해 `declare_parameter('icp_config', ...)`로 선언하지만, `localization.yaml:29`가 이를 특정 머신 절대경로(`/workspace/colcon_ws/...`)로 오버라이드한다. `slam.py:528-530`이 그 값으로 `self.localization.icp.loadFromYaml(icp_config)`를 실제 호출 → 이 경로의 파일을 런타임에 로드한다. 현재 워크스페이스는 `/workspace/src/`라 `/workspace/colcon_ws/` 경로는 존재하지 않으므로, 이 오버라이드가 적용되면 ICP 설정 로드가 실패하거나 빈 설정으로 동작한다(머신 의존·재현 불가).
- **근거 표준**: ROS2 launch는 `ament_index`(`get_package_share_directory`)로 패키지 상대 config 경로를 런타임 해석하는 게 표준 — 절대경로 하드코딩은 비표준([ROS2 Launch substitutions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html)).
- **현재 처리**: 보류. 이 값을 바꾸면 `loadFromYaml`이 로드하는 파일이 달라져 런타임 거동이 변하므로(동작보존 불가) P3에서 손대지 않았다.
- **수정안**: P4에서 `localization.yaml:29`의 절대경로를 제거(코드의 패키지 상대 기본값에 위임)하거나 launch substitution으로 대체. ICP 설정 로드 거동 변화를 명시 수용 후 처리.

## Phase 3 `chore/dead-code-cleanup` 의 의도적 미포함 (2026-09-01)

- **✅ ~~`profiling_enabled` 가 게이트하는 `perf_counter` 계측 체인~~ — `feat/loc-instrumentation` 에서 판단 완료**: `MappingProfiler` 는 걷어냈고(빈 CSV 부작용 제거, 사용자 결정), **`perf_counter` 체인은 남긴다**. 이유는
  이연 당시와 같다 — `t_ray_total`·`t_octree_total` 이 헬퍼 3개의 반환
  시그니처에 박혀 있어 떼어내는 것이 시그니처 수술이고 그건 §4.4(god-method
  분해)의 범위다. 남기는 이유는 이제 `mapping_3d.py` 주석에도 있다.
  소비자를 잃은 `enable_profiling` ROS 파라미터도 함께 제거했다.
  아래는 이연 당시 기록(보존).

- **(이력) `profiling_enabled` 가 게이트하는 `perf_counter` 계측 체인** — P1-8 이 두 통계
  딕셔너리(`performance_stats`·`profiling_data`)를 지우면서 이 계측의 **소비자가
  사라졌다**. 그런데 `t_ray_total`·`t_octree_total` 은 `_process_bearing_rays` 등
  헬퍼 3개의 **반환 시그니처에 박혀** 있어 제거가 시그니처 수술이 되고, 그건 계획
  §4.4(god-method 분해)의 범위다. 게다가 바로 다음 브랜치
  `feat/loc-instrumentation`(계획 §5 의 I1~I11)이 이 층을 재설계하므로, 지금
  걷어내면 곧 다시 세워야 한다. **그 브랜치에서 함께 판단한다** — 재사용하든
  걷어내든 결정을 미루는 것이지 잊는 것이 아니다.

  같은 이유로 `MappingProfiler` 도 남겼다. `mapping_3d.py:328` 이 `start()`,
  `:1396` 이 `close()` 를 부르지만 `record_frame()` 의 호출자는 이제 0이라
  `/tmp/mapping_profiling.csv` 에 **헤더만 있는 빈 파일**이 남는다. ⚠️ 이건
  P1-8 이 만든 상태가 아니다 — `record_frame` 은 `_print_profiling_stats` 안에
  있었고 그 함수의 호출부는 **main 에서 이미 주석 처리**돼 있어서
  (`mapping_3d.py:1285-1290`) 삭제 전에도 실행된 적이 없다. 달라진 것은 죽어
  있던 사실이 드러난 것뿐이다.

- **`utils/topics.py` 의 미참조 상수 15/22** — 계획이 명시적으로 범위 밖에 뒀다.
  `test_wildcard_gate.py` 가 골든 심볼 집합을 동결하므로 삭제하면 그 골든도 함께
  갱신해야 하는데, 상수는 향후 배선 지점을 이름으로 예약하는 성격이 있어 삭제
  이득이 회귀 위험보다 크지 않다. 이번에 `__all__` 을 추가하면서 22개 전부를
  넣었다 — 노출 범위를 명시했을 뿐 어느 것도 삭제하지 않았다.

- **`query_cell` 과 `RayProcessorConfig::bearing_resolution`** — 계획의 dead 목록에
  있었으나 이 트리에서 **살아 있음이 확인돼** 삭제하지 않았다. 전자는 PR #18 이
  추가한 `test_cpp_extensions.py` 가 3곳에서 쓰고, 후자는 `mapping_3d.py:235` 가
  설정한다. 계획의 "호출자 0" 은 #18·#19 머지 이전 기준이었다.

## Phase 3 `feat/loc-instrumentation` 의 실기 sign-off 대상

계측은 **값을 세기만 하고 판정을 바꾸지 않는다.** 따라서 정적 게이트로 확인할 수
있는 것은 배선의 존재뿐이고(테스트 7개가 AST 로 고정), 계측이 답하려던 질문 자체는
전부 GPU 머신 런타임에서 판정된다.

- **[I1] `ssm_disabled_count` 가 `키프레임 총수 - 1` 과 같은가** — 같으면 "icp 0%" 의
  원인이 알고리즘이 아니라 `ssm.enable: false` 라는 설정임이 확정된다. **1 을 빼는
  이유**는 첫 키프레임이 prior factor 만 넣고 SSM 을 아예 부르지 않기 때문이다
  (`slam.py:880`) — 총수와 그냥 비교하면 항상 하나 모자라 보인다. **다른 모든 계측의
  전제**이므로 이것부터 1회 실행으로 확인한다.
- **[I2·I3] `icp_rate` 와 factor 구성비** — `ssm.enable:=true` 로 켠 뒤 분모 있는
  비율을 처음으로 얻는다.
- **[I6] `reject_rot` 이 0 인가** — `use_dr_rotation: true` 에서 회전 오차는 항등 0
  이므로 0 이어야 한다. 0 이 아니면 회전 게이트 사문화 가설이 틀린 것이다.
- **[I7] 기각 사례가 `|dr_ty|` 큰 구간에 몰리는가** — 몰리면 ty 부호 오류, 균등하면
  부호는 정상이고 게이트가 단순히 빡빡한 것이다.
- **[I8·I9] `rot_peak`·`trans_peak`·공분산의 분포** — P1-5 품질 게이트의 임계값은
  이 분포 없이는 정할 수 없다. 공분산이 DR 불일치를 예측하면 게이트를 공분산 기반으로
  설계할 수 있다(U2).
- **[I10] FFT 회전이 쓸 만한가** — `use_dr_rotation` 을 끌 수 있는지 판단할 유일한
  근거다. 끄기 전에 `max_rotation_error` 5° 를 먼저 재검토해야 한다(P1-6 참조 —
  주석만 고쳤고 값은 owner 결정으로 남겼다).
- **[I11] 병진 스케일 비율의 중앙값** — 1 에서 유의하게 벗어나면 점군 척도가 어긋나
  있다는 뜻이다. **방향은 예단하지 않는다** — LOC-3 교정에 따라 압축일 수도 팽창일
  수도 있어 값만 남겼다. 이 판정이 `feature_extraction.py` 투영 수정의 근거가 된다.

### 숫자를 읽을 때의 함정 (적대 검증에서 나온 것)

- **I11 의 표본은 조용히 걸러진다.** `init_norm <= 1e-6`(정지·순수 회전 키프레임)은
  누락 카운터도 로그도 없이 제외되고, 반대로 스케일 로그는 large-transform·overlap
  최종 검사보다 **먼저** 나가므로 나중에 odometry 로 fallback 되는 ICP 결과까지
  포함한다. 중앙값은 그래서 "채택된 ICP 의 스케일" 이 아니라 "시도된 ICP 의
  스케일" 이다. 누락률이 없어 대표성은 판단할 수 없으니, 표본 수를
  `icp_attempted` 와 대조해서 읽는다.
- **`validate_with_odom: false` 면 I8~I10 이 통째로 침묵한다.** `[INSTR] gate` 로그는
  `validate_fft_with_odom()` 안에만 있어서, 검증을 끄면 FFT transform 은 그대로
  채택되는데 분포도 기각 카운터도 안 나온다. 기본값은 `true` 이므로 기본 설정
  sign-off 에서는 문제가 없지만, 프로파일 yaml 로 끈 채 돌리면 **로그 부재를
  "기각이 없었다" 로 읽으면 안 된다.**

### 계측이 답하지 않는 것
`[INSTR]` 로그는 **경로와 분포**를 준다. 궤적 정확도 자체는 I13(`traj_2d_error_
accumulator` 의 기존 `mean_err`)이 수정 전후 비교로 답한다 — 신규 코드가 없으므로
이 브랜치는 그 지표를 건드리지 않는다.


## 🔎 `max_frames` 리셋이 C++ 옥트리는 안 지운다 — 미해결 (2026-09-02 발견)

- **파일**: `stonefish_slam/core/mapping_3d.py` `process_sonar_image()` 의 프레임 상한 분기.
- **발견 경위**: semantic 복셀 라벨의 수명 규칙(라벨은 지도가 비면 같이 비워야 한다)을
  걸 자리를 찾다가 확인했다. 라벨 쪽은 이 자리에 같이 걸어 뒀다.
- **증상**: 리셋 분기는 `self.octree.clear()`(**순수 파이썬 백엔드**)만 부른다.
  `use_cpp_backend: true`(배포 기본값)면 `self.cpp_octree` 는 안 지워지므로
  `frame_count` 만 0 으로 돌아가고 지도는 계속 누적된다. 즉 **기본 설정에서
  `max_frames` 는 사실상 동작하지 않는다.**
- **영향 범위**: 장시간 런의 메모리·`get_occupied_cells` 비용. 정확도에는 직접
  영향이 없어 지금까지 드러나지 않았다. semantic 라벨은 "현재 점유 복셀과의
  교집합만 출력" 규칙 때문에 이 결함에 안 물린다.
- **미수정 이유**: C++ 옥트리를 언제 비울지는 매핑 정책 결정이고(전체 clear 대신
  sliding window·decay 가 원래 TODO 로 적혀 있다) semantic 작업의 범위 밖이다.
- **수정 시 확인할 것**: `OctreeMapping.clear()` 바인딩은 이미 있다
  (`cpp/octree_mapping.cpp`). 지우면 `voxel_labels` 도 같이 지워야 한다 —
  파이썬 경로에는 이미 그렇게 걸려 있다.

