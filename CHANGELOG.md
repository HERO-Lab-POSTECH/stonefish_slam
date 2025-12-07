# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Fixed
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
