# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added
- **FFT Localization - Periodic Decomposition** (2025-12-10): Moisan 2011 기반 spectral leakage 제거
  - 기능: `_periodic_decomposition()` 메서드로 Moisan periodic-plus-smooth decomposition 구현
  - 효과: Border effect 제거로 FFT 수행 전 spectral leakage 감소
  - 변경: `compute_phase_correlation()`에 `apply_periodic_decomp` 파라미터 추가
  - 새 파라미터: `slam.yaml`에 `periodic_decomposition.enable` (기본값: true)
  - 참고: Moisan, L. "Periodic Plus Smooth Image Decomposition", J Math Imaging Vis 39, 161-179 (2011)
  - 파일: `localization_fft.py`, `config/slam.yaml`
- **DFT Subpixel Refinement 구현** (2025-12-10): Guizar-Sicairos 2008 기반 국소 최적화
  - 기능: DFT를 이용한 FFT correlation peak의 subpixel 정밀 위치 계산
  - 효과: subpixel 정밀도 0.1 pixel → 0.01 pixel 향상 (10배 개선)
  - 새 파라미터: `dft_refinement.enable`, `dft_refinement.upsample_factor`
  - 파일: `localization_fft.py`, `config/slam.yaml`
- **Python 버전 호환성 개선** (2025-12-08): 동적 Python 버전 감지로 모든 Python 버전 지원
  - 변경: CMakeLists.txt에서 하드코딩된 python3.10 → `${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}` 사용
  - 효과: .so 파일 설치 경로 자동 결정
  - 파일: `CMakeLists.txt`
- **IWLO Free Space Carving 최적화** (2025-12-08): Range-weighted log_odds 직접 사용
  - 새 API: `insert_point_cloud_with_intensity_and_logodds()` 추가
  - 개선: ray_processor에서 계산한 range-weighted log_odds를 octree에 직접 삽입
  - 효과: Free space carving 성능을 log odds 방법과 동등하게 개선
  - 파일: `octree_mapping.h`, `octree_mapping.cpp`, `ray_processor.cpp`
- **IWLO 강도 가중치 구현** (2025-12-08): Sigmoid 함수 기반 intensity 가중치
  - 새 함수: `compute_intensity_weight()` 추가
  - 기능: 높은 강도(>127) → occupied 강한 업데이트, 낮은 강도(<127) → 약한 업데이트
  - 파일: `core/ray_processor.cpp` (라인 530, 599)

### Changed
- **파라미터 이름 통일** (2025-12-08): YAML 설정값 정상 로드 확보
  - 변경: `intensity_threshold` → `mapping_3d.intensity_threshold`
  - 파일: `mapping_3d_standalone_node.py`
- **Standalone 매핑 노드 개선** (2025-12-08): 설정 파일에서 업데이트 메소드 읽기
  - 변경: 하드코딩된 update_method → mapping.yaml에서 동적 로드
  - 파일: `launch/mapping_3d_standalone.launch.py`
- **IWLO 파라미터 최적화** (2025-12-08): 성능 및 안정성 개선
  - 파일: `config/mapping.yaml`, `config/method_iwlo.yaml`

### Fixed
- **Occupied/Free Space 처리 통합** (2025-12-08): 높은 물체 윗면 업데이트 문제 해결
  - 문제: 높은 물체의 윗면이 한 프레임에 업데이트되지 않음
  - 원인: Free space와 Occupied 처리 방식 불일치
    * Free space: 각 voxel마다 픽셀 확인 O
    * Occupied: vertical fan 전체를 무조건 occupied (픽셀 확인 X)
  - 수정: DDA 기반 통합 처리
    * DDA 범위를 first_hit 포함하도록 확장
    * 각 voxel에서 (range, bearing) 픽셀 직접 확인
    * hit 픽셀 → occupied 업데이트
    * no-hit + range < first_hit → free 업데이트
    * no-hit + range >= first_hit → shadow (업데이트 안함)
    * 기존 separate occupied 처리 제거
  - 효과: 모든 voxel에 동일한 픽셀 검증 로직 적용
  - 파일: `ray_processor.cpp`
- **Free Space에서 실제 픽셀 확인** (2025-12-08): Shadow 영역이 free space로 업데이트되는 버그 수정
  - 문제: first_hit_map만 확인하고 실제 픽셀을 확인하지 않아 shadow 영역이 free로 업데이트됨
  - 원인: first_hit 앞이면 무조건 free로 가정 (실제 픽셀의 hit/no-hit 확인 안함)
  - 수정: 각 voxel의 (range, bearing)에 해당하는 polar 이미지 픽셀을 직접 확인
    * hit 픽셀 → skip (occupied 처리에서 담당)
    * no-hit + range < first_hit → free
    * no-hit + range >= first_hit → shadow (업데이트 안함)
  - 파일: `ray_processor.cpp`, `ray_processor.h`
- **Occupied 처리 Shadow Check 추가** (2025-12-08): Multi-hit 상황에서 shadow 영역 hit 무시
  - 문제: Multi-hit bearing에서 가장 먼 hit도 occupied로 처리됨
  - 원인: find_first_hit()가 FAR→NEAR 스캔으로 가장 먼 hit 반환, 모든 threshold 이상 hit 처리
  - 수정: 각 hit의 range를 first_hit_map[bearing_idx]와 비교하여 shadow 영역 hit skip
  - 효과: first hit (가장 가까운 hit) 근처의 hit만 occupied로 처리
  - 파일: `ray_processor.cpp` (라인 461-472)
- **섀도우 판정 거리 타입 수정** (2025-12-08): 수평 거리 → 슬랜트 거리 (3D 거리)
  - 문제: floor/ceiling voxel이 자유 공간으로 오표기됨
  - 원인: 3D 실제 거리가 아닌 2D 수평 거리 비교
  - 수정: 3D Euclidean distance 사용
  - 파일: `core/ray_processor.cpp` (라인 439)
- **find_first_hit() 반복 방향 수정** (2025-12-08): FAR→NEAR → NEAR→FAR
  - 문제: 가장 가까운 hit를 먼저 찾지 않음
  - 원인: 거리가 먼 것부터 검사하는 역순 반복
  - 수정: NEAR→FAR로 변경 (compute_first_hit_map과 일관성 유지)
  - 파일: `core/ray_processor.cpp` (라인 571)
- **Occupied 처리 루프 방향 수정** (2025-12-08): first_hit→size() → 0→first_hit
  - 문제: first_hit 이후의 hit들이 처리되지 않음
  - 원인: 루프 범위가 first_hit부터 끝까지인데, first_hit 자체와 그 이상 거리의 hit들을 누락
  - 수정: 0부터 first_hit까지만 반복 (first_hit 포함, 그 이상은 shadow로 처리)
  - 파일: `core/ray_processor.cpp` (라인 461)
- **IWLO 섀도우 영역 오탐 수정** (2025-12-08): 고정 bearing width로 인한 섀도우 감지 실패 해결
  - 문제: Multi-hit 광선에서 HIT1과 HIT2 사이의 섀도우 영역이 자유 공간으로 잘못 업데이트됨
  - 원인: `is_voxel_in_shadow()` 함수가 고정 `bearing_half_width` (약 5도)를 사용하여 인접 bearing만 감지
  - 수정: 거리 기반 동적 각도 범위 계산 (`voxel_angular_extent = config_.voxel_resolution / voxel_range`)
    * 모든 bearing 내에서 최소 first_hit 확인
    * `voxel_range >= min_first_hit`이면 섀도우로 표시
  - 파일: `ray_processor.cpp`
- **디버그 로그 제거** (2025-12-08): 프로덕션 환경 최적화
  - 파일: `ray_processor.cpp`, `octree_mapping.cpp`
- **IWLO Free Space Carving 버그** (2025-12-08): Free space intensity 및 alpha 중복 적용 수정
  - 문제: IWLO에서 free space carving이 동작하지 않음
  - 원인: Free space intensity가 0.0으로 설정되어 있고, alpha가 이중으로 적용됨
  - 수정:
    * `ray_processor.cpp`: free space intensity를 0.0 → 1.0으로 변경
    * `octree_mapping.cpp`: alpha 이중 감쇠 제거 (log odds 방식과 일관성 유지)
  - 효과: IWLO 방식에서 free space carving이 log odds와 동등하게 동작
  - 파일: `ray_processor.cpp`, `octree_mapping.cpp`

### Cleanup
- **디버그 코드 제거 및 주석 정리** (2025-12-08): 약 64줄의 디버그 통계 및 cerr 출력 제거
  - 제거 항목: 통계 변수, 성능 프로파일링 cerr 메시지
  - 간소화: "CRITICAL FIX" 주석 정리
  - 수정: 미사용 파라미터 경고 처리
  - 파일: `core/ray_processor.cpp`

---

## [0.3.1] - 2025-12-08

### Fixed
- **Occupied Voxel Elevation 계산 오류** (2025-12-07): bearing 0도 포인트 깊이 불일치 수정
  - 문제: Occupied voxel 처리 시 불필요한 perspective correction 적용으로 이중 보정 발생
  - 원인: Stonefish는 이미 3D Euclidean distance를 제공하므로 추가 보정 불필요
  - 증상: Bearing 0도(정면)에서 생성 포인트가 다른 bearing보다 얕게 표시
  - 수정: `actual_elevation = nominal_vertical_angle` 로 변경 (perspective correction 제거)
  - 파일: `stonefish_slam/core/mapping_3d.py`
- **FLS Perspective Projection 보정** (2025-12-07): 3D 매핑 곡면 왜곡 수정
  - 문제: Stonefish FLS는 perspective projection 사용, 같은 vertical sample이 곡면 형태
  - 기존: 모든 bearing에서 동일한 vertical_angle 적용 (평면 가정)
  - 수정: bearing에 따른 실제 elevation 계산 (`arcsin(tan(v)/sqrt(tan²(b)+tan²(v)+1))`)
  - 효과: 가장자리(±65°)에서 Z 오차 0.5m(57%) → 0, 회전 시 갈고리 왜곡 제거
  - 파일: `ray_processor.cpp`, `ray_processor.h`, `mapping_3d.py`
- **Adaptive Protection 로직 복원** (2025-12-04): `updateNode()` 전에 확률 확인 필수 (알고리즘 정확성)

### Added
- **3가지 확률 업데이트 방법** (2025-12-04): `log_odds`, `weighted_avg`, `iwlo` 지원
- **C++ RayProcessor IWLO 지원** (2025-12-04): intensity 기반 가중 업데이트
- **FFT 기반 로컬라이제이션** (2025-12-01): polar sonar image로 rotation/translation 추정

### Changed
- **3D 매핑 성능 최적화** (2025-12-04):
  - Phase 2: Search radius 캐싱 (5-10ms 개선)
  - Phase 3: NumPy 경계 제거, C++ 네이티브 경로 (10-15ms 개선)
  - Phase 4: 중복 계산 제거 (<5ms 개선)
- **C++ 백엔드 55x 성능 개선** (2025-12-04): Python 22s → C++ 340ms
- **exp() LUT 최적화** (2025-12-04): 256-entry lookup table

### Fixed (2025-12-01)
- FFT rotation center 버그 (top → bottom center)
- FFT range resolution 계산 오류 (`range_max/rows`)
- FFT rotation 부호 오류

---

## [0.3.0] - 2025-11-30

### Added
- **C++ 백엔드**: OctoMap 기반 3D 매핑, pybind11 바인딩
- **OpenMP 병렬화**: 28 스레드 지원
- **Adaptive protection**: 저확률 voxel 보호

### Changed
- **Config 구조 모듈화**: sonar.yaml, feature.yaml, localization.yaml 등 분리
- **Launch 파일 개선**: OpaqueFunction 패턴으로 조건부 파라미터 처리

---

## [0.2.0] - 2025-11-15

### Added
- **ICP 기반 scan matching**: Point-to-point, point-to-plane 지원
- **Factor graph SLAM**: GTSAM 기반 pose graph 최적화
- **Loop closure (NSSM)**: Scan context 기반 장소 인식

### Changed
- **CFAR 특징 추출**: CA-CFAR, GOCA-CFAR 알고리즘 C++ 구현

---

## [0.1.0] - 2025-10-01

### Added
- **기본 SLAM 구조**: Dead reckoning, keyframe 관리
- **2D 매핑**: Occupancy grid 기반
- **ROS2 통합**: Humble 지원, launch 파일
