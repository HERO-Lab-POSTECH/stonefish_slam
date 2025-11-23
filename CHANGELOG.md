# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

- **설정 파라미터 추가** (`Mapping3D` 클래스)
  - `enable_propagation`: Bearing propagation 활성화 여부 (기본값: False)
  - `enable_profiling`: 성능 측정 활성화 여부 (기본값: True)

### Performance

- **Baseline (Propagation 비활성화)**
  - 평균 처리 시간: 1.087초/프레임
  - 프레임당 복셀 업데이트: 50,738개

- **Propagation 활성화 시**
  - 평균 처리 시간: 3.671초/프레임 (237.7% 증가)
  - 프레임당 복셀 업데이트: 214,770개 (323.3% 증가)
  - 권장: 맵 품질 개선 필요 시에만 활성화

### Notes

- 기본값 `enable_propagation=False`로 기존 동작 유지
- 기존 API 변경 없음 (하위 호환성 유지)
- 차후 파라미터 최적화로 성능 개선 예정

## [0.1.0] - 2024-11-18

### Added

- 초기 릴리스
- Sonar 기반 SLAM 기본 구현
- Kalman 필터 기반 상태 추정
- 3D 맵핑 및 특징 추출
- Dead reckoning 지원
