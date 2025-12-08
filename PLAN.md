# IWLO Shadow Region 처리 계획

## 작업: First Hit 이후 Shadow 영역의 잘못된 Free Space 업데이트 방지

### 분석

**목표**: First hit 이후 그림자(shadow) 영역이 free space로 업데이트되는 버그 수정

**영향받는 패키지**: stonefish_slam

**복잡도**: Medium

### 전제 조건
- [x] research-analyst 분석 완료
- [x] 근본 원인 파악 완료
- [ ] 사용자 승인 대기

### 실행 계획

#### Phase 1: 코드 분석 및 구조 파악
**목표**: 현재 구현 검증 및 최적 해결 방안 선정

1. **현재 동작 분석** → Agent: `직접 수행` (완료)
   - ray_processor.cpp의 `process_single_ray_internal()` 로직 확인
   - Line 307-450: First hit 탐지 → Free space DDA → Occupied voxel 처리
   - Line 437-442: First hit 이후 모든 고강도 픽셀을 occupied로 처리
   - 문제: 저강도 픽셀 (≤ threshold)은 필터링되지만, free space DDA에서 이미 shadow 영역이 포함됨

2. **Shadow 검증 메커니즘 확인** → Agent: `직접 수행` (완료)
   - Line 387-390: `is_voxel_in_shadow()` 함수가 이미 존재
   - Free space DDA 경로에서만 호출됨
   - Occupied voxel 경로 (Line 455-541)에서는 호출 안 됨 (의도된 설계)

3. **해결 방안 선정** → Agent: `직접 수행`
   - Option 1 (채택): Shadow validation 강화
     - 장점: 최소 변경, 기존 메커니즘 활용
     - 방법: Free space DDA가 first hit을 정확히 피하도록 range 조정 검증
   - Option 2: VoxelUpdate에 플래그 추가
     - 단점: 구조 변경 필요, 오버헤드 증가

#### Phase 2: 문제 재현 및 검증
**목표**: 버그가 실제로 발생하는지 검증

1. **로깅 추가** → Agent: `code-writer`
   - 파일: `/workspace/colcon_ws/src/stonefish_slam/stonefish_slam/cpp/ray_processor.cpp`
   - 작업:
     - Line 426: Free space 업데이트 전 shadow 검증 결과 카운트
     - 통계 출력: shadow로 필터링된 voxel 수 vs 업데이트된 voxel 수

2. **테스트 실행** → Agent: `build-manager`
   - colcon build --packages-select stonefish_slam
   - 시뮬레이션 실행: `ros2 launch stonefish_slam slam_sim.launch.py`
   - 로그 확인: Shadow voxel 필터링 통계

#### Phase 3: 버그 수정 (필요시)
**목표**: Shadow 검증 로직 강화 또는 range 계산 수정

**경우 1: Shadow 검증이 작동하지 않는 경우**

1. **Shadow 검증 디버깅** → Agent: `code-writer`
   - `is_voxel_in_shadow()` 함수 검증 (Line 651-723)
   - First hit map 정확도 검증
   - Shadow cone width 검증 (현재: vertical_fov 사용)

2. **수정 구현** → Agent: `code-writer`
   - 필요시 shadow cone 각도 조정
   - 또는 range 비교 로직 강화

**경우 2: Free space DDA가 first hit을 침범하는 경우**

1. **Range 계산 검증** → Agent: `code-writer`
   - Line 321: `range_to_first_hit` 계산 검증
   - Line 342: `safe_range` 적용 검증 (현재: -1 voxel)
   - 필요시 margin 증가 (1 → 2 voxel)

2. **수정 구현** → Agent: `code-writer`
   - Safe margin 조정
   - 또는 DDA traversal에서 end voxel 제외 강화 (Line 380-382)

#### Phase 4: 빌드 및 테스트
**목표**: 수정 사항 검증

1. **빌드** → Agent: `build-manager`
   - `colcon build --packages-select stonefish_slam`
   - 빌드 에러 확인

2. **시뮬레이션 테스트** → Agent: `직접 수행` (사용자 실행)
   - 명령: `ros2 launch stonefish_slam slam_sim.launch.py`
   - 검증: Shadow 영역이 unknown으로 유지되는지 확인
   - RViz에서 시각적 확인

#### Phase 5: 문서화 및 커밋
**목표**: 변경 사항 기록

1. **CHANGELOG 업데이트** → Agent: `documentation-recorder`
   - 파일: `/workspace/colcon_ws/src/stonefish_slam/CHANGELOG.md`
   - 내용: "Fix: IWLO shadow region incorrectly updated as free space"

2. **Git 커밋** → Agent: `git-manager`
   - 메시지: "Fix IWLO shadow region free space bug"
   - 영향받는 파일: ray_processor.cpp, (필요시 ray_processor.h)

### 검증 기준
- [ ] Free space DDA가 first hit 이전에서만 동작
- [ ] Shadow 영역 voxel이 업데이트에서 제외됨
- [ ] 로그에서 shadow 필터링 통계 확인 가능
- [ ] RViz에서 shadow 영역이 검은색(unknown)으로 표시됨

### 롤백 전략
- Git revert로 이전 커밋 복원
- 빌드 재실행: `colcon build --packages-select stonefish_slam`
