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
