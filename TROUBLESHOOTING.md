# Stonefish SLAM Troubleshooting Guide

## [2025-11-24] OctoMap API 사용 오류

**Category**: Bug Fix

**Situation**:
C++ OctoMap backend에서 `setLogOdds()` 직접 호출로 수동 log-odds 설정. 결과적으로 pruning이 미수행되고 메모리 비효율 발생.

**Cause**:
OctoMap의 공식 API `updateNode()`를 사용하지 않음. `setLogOdds()`는 internal 메서드로 octree 구조 정보 미갱신 → identical children merge 불가능 → 메모리 누적.

**Solution**:
1. octree_mapping.cpp의 node update 로직 변경:
   ```cpp
   // 수정 전: 자동 pruning 없음
   node->setLogOdds(log_odds_update);

   // 수정 후: 자동 pruning + merge
   tree_->updateNode(key, log_odds_update);
   ```

2. Log-odds 반환 추가 (Nx4 array):
   ```cpp
   // Nx3 → Nx4 (x, y, z, log_odds) 포함
   coords_array.push_back(coord.x());
   coords_array.push_back(coord.y());
   coords_array.push_back(coord.z());
   coords_array.push_back(node->getLogOdds());
   ```

3. Log-odds 범위 정규화 (mapping_3d.py):
   ```python
   # -2.0 ~ 3.5 (OctoMap 기본값)로 clamping
   log_odds = np.clip(log_odds, -2.0, 3.5)
   ```

**Files**:
- `/workspace/colcon_ws/src/stonefish_slam/stonefish_slam/cpp/octree_mapping.cpp` (updateNode 적용)
- `/workspace/colcon_ws/src/stonefish_slam/stonefish_slam/core/mapping_3d.py` (범위 정규화)

**Note**:
- updateNode() 사용으로 메모리 효율성 향상, identical children 자동 merge
- Log-odds 값 정확히 저장/반환되므로 Python에서 occupancy 확률 계산 가능
- 테스트: 1.5 log-odds → prob = 0.818 (정확함)

---

## [2025-11-23] 3D Mapping 성능 병목 식별

**Category**: Performance

**Situation**:
3D sonar mapping이 0.1 FPS로 매우 느리게 실행됨. DDA (Amanatides-Woo) 최적화가 완료되었지만 전체 성능 개선이 미흡함.

**Cause**:
Python 기반 Octree 연산이 전체 프레임 처리 시간의 69.5% (6602.8ms) 차지. DDA 최적화는 0.7%만 사용하므로 추가 최적화의 여지가 없음. 점유 복셀 처리가 22.1% 차지하는 것도 Python 기반 연산의 결과.

**Solution**:
1. `/workspace/colcon_ws/src/stonefish_slam/stonefish_slam/core/mapping_3d.py` 에서 프로파일링 실행 (이미 구현됨)
```bash
cd /workspace/colcon_ws/src/stonefish_slam
ros2 launch stonefish_slam slam_sim.launch.py
# 콘솔에 매 10프레임마다 성능 리포트 출력됨
```

2. 프로파일링 결과 분석:
   - DDA: 0.7% ← 이미 최적화됨
   - Octree: 69.5% ← C++ 마이그레이션 필수
   - 점유 처리: 22.1% ← C++로 이동 필요

3. **P0.3 C++ Core Migration** (예정):
   - HierarchicalOctree를 C++로 변환
   - Occupied voxel processing을 C++로 변환
   - Pybind11로 Python 바인딩
   - 예상 성능: 95-100배 향상 (9.5초 → 100ms)

**Files**:
- `/workspace/colcon_ws/src/stonefish_slam/stonefish_slam/core/mapping_3d.py` (프로파일링 구현)
- `/workspace/3d_mapping_optimization_plan.md` (상세 분석)
- `/workspace/colcon_ws/src/stonefish_slam/CHANGELOG.md` (구현 기록)

**Note**:
- 프로파일링은 매 10프레임 자동 출력되고 후속 프레임에서는 롤링 윈도우로 업데이트됨
- `enable_profiling=True`로 설정하면 `/tmp/mapping_profiling_stats.txt`에 저장 가능 (구현 예정)
- DDA 최적화는 이미 완료되었으므로 추가 개선 불필요

---

## [2025-11-20] DDA 배치 처리 성능 최적화

**Category**: Performance

**Situation**:
DDA (Digital Differential Analyzer) C++ 모듈 통합 후 성능 저하 (2-3 Hz → 1-2 Hz). 예상과 반대로 성능이 악화됨.

**Cause**:
Vertical loop (각 ray당 10번)을 Python에서 실행하면서 Python-C++ 경계 교차가 320회/프레임 발생. 각 교차마다 Python 객체 생성 및 메모리 복사로 인한 오버헤드.

**Solution**:
1. Vertical loop를 C++로 이동 (DDA 내부에서 처리)
2. 배치 처리 API로 변경:
   ```python
   # 이전 (320회 호출)
   for v_step in vertical_aperture:
       dda_result = self.dda_traverser.process_single_ray(...)

   # 이후 (32회 호출)
   voxel_updates = self.dda_traverser.process_free_space_ray(...)
   ```
3. 구조체를 통한 파라미터 전달 (Pybind11):
   - `SonarRayConfig`: 모든 ray 파라미터를 한 번에 전달
   - `VoxelUpdate`: 누적된 결과를 구조체로 반환

**Files**:
- `/workspace/stonefish/Library/dda_traversal.cpp` (배치 처리 구현)
- `/workspace/colcon_ws/src/stonefish_slam/stonefish_slam/core/mapping_3d.py` (API 변경)

**Note**:
- 배치 처리로 경계 교차 90% 감소
- Python ↔ C++ 경계 오버헤드 제거가 성능 최적화의 핵심
- 유사한 최적화는 occupied voxel processing에도 적용 가능
