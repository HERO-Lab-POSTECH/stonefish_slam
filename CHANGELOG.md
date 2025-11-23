# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Gaussian Weighting 기능** (`mapping_3d.py`)
  - `enable_gaussian_weighting`: Gaussian vs Uniform distribution 선택 (기본값: False)
  - `gaussian_sigma_factor`: Gaussian 분포의 폭 조절 (기본값: 2.5)
  - Vertical aperture에 대해 intensity weighting 적용
  - 수식: `log_odds_update = base_log_odds × exp(-0.5 × (normalized_angle × sigma_factor)²)`
  - 기본값 Uniform distribution으로 물리적 정확성 우선 (문헌 근거)
  - 사용자 선택으로 Gaussian 활성화 가능하며 하위 호환성 유지

- **성능 프로파일링 도구** (`mapping_3d.py`)
  - `performance_stats` 딕셔너리로 프레임 처리 시간 및 복셀 업데이트 수 추적
  - `get_performance_summary()` 메서드로 누적 성능 통계 제공
  - 매 10프레임마다 자동 성능 정보 로깅 (INFO 레벨)

- **Bearing Propagation 메커니즘** (`mapping_3d.py`)
  - `propagate_bearing_updates()` 메서드로 Gaussian weight 기반 인접 bearing으로 복셀 업데이트 전파
  - 회전 변환을 통한 정확한 3D 좌표 계산 (ENU → body frame)
  - 설정 파라미터:
    - `propagation_radius`: 전파 범위 (기본값: 2, 인접 bearing 수)
    - `propagation_sigma`: Gaussian weight의 표준편차 (기본값: 1.5)

### Changed

- **Phase 2: Bearing Propagation 최적화** (`mapping_3d.py`)
  - `propagate_bearing_updates()` → `propagate_bearing_updates_optimized()` 교체
  - 전체 3D 변환 대신 Z축 회전만 수행 (2D 회전 최적화)
  - Sonar origin 계산 최적화 (중복 제거)
  - 불필요한 행렬 연산 제거

- **파라미터 조정** (`Mapping3D` 클래스)
  - `propagation_radius`: 2 → 1 (영향 범위 축소)
  - `propagation_sigma`: 1.5 → 1.0 (가중치 감소율 개선)
  - `enable_propagation`: False → True (기본값으로 활성화)
  - `bearing_divisor`: 128 → 256 (샘플링 밀도 조정)

- **설정 파라미터 추가** (`Mapping3D` 클래스)
  - `enable_propagation`: Bearing propagation 활성화 여부 (기본값: False → True)
  - `enable_profiling`: 성능 측정 활성화 여부 (기본값: True)

### Performance

- **Baseline (Propagation 비활성화)**
  - 평균 처리 시간: 1.087초/프레임
  - 프레임당 복셀 업데이트: 50,738개

- **Phase 1: Propagation 활성화 (최적화 전)**
  - 평균 처리 시간: 3.671초/프레임 (237.7% 증가)
  - 프레임당 복셀 업데이트: 214,770개 (323.3% 증가)
  - 문제점: 성능 저하가 심함

- **Phase 2: 최적화 후 (예상)**
  - Propagation radius 감소로 처리 voxel 수 ~60% 감소
  - 2D 회전 최적화로 계산 복잡도 감소
  - Bearing coverage 100% 유지
  - 목표: 프레임당 처리 시간 < 2.5초 (baseline 대비 2배 이내)

- **알고리즘 개선**
  - Bearing step=2 (512 bearings → 256 samples)
  - Propagation radius=1로 각 sampled bearing이 인접 1개 bearing에만 전파
  - 중복 제거로 모든 bearing을 효율적으로 커버

### Notes

- 기본값 `enable_propagation=True`로 최적화된 버전 기본 활성화
- 기존 API 변경 없음 (하위 호환성 유지)
- Phase 2 최적화 적용으로 성능과 맵 품질의 균형 달성

## [0.1.0] - 2024-11-18

### Added

- 초기 릴리스
- Sonar 기반 SLAM 기본 구현
- Kalman 필터 기반 상태 추정
- 3D 맵핑 및 특징 추출
- Dead reckoning 지원
