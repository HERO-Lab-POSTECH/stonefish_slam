# P4 Investigation Flags

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
