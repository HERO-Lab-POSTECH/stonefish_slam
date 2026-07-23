# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

- README Docker 설치 안내 — 전용 배포 repo
  [`stonefish_bringup`](https://github.com/HERO-Lab-POSTECH/stonefish_bringup) 참조
  (sim과 동일 이미지에 slam 포함 — C++ 의존성·`.so` 확장까지 이미지 빌드에서 완결)
- 협업 규칙 `CONTRIBUTING.md` + PR 템플릿 (발효 2026-07-23)
- README Testing 섹션 (pytest용 pybind11 `.so` 스테이징 절차)

### Removed

- `fft_localization.min_ppr`·`reject_on_failure` 파라미터 제거 — 선언·로드만 되고
  검증 경로 어디서도 미적용이던 dead knobs (참조 전수 추적으로 확인, 동작 불변)

### Changed

- `fft_localization_node`(standalone) 기본값 4개(verbose/erosion/sigma/truncate)를
  slam.yaml 튜닝값과 정렬 — slam_node FFT와 동일 조건으로 비교 가능

### Fixed

- README 설치 blocker: `libpointmatcher-dev`(jammy에 없음) → `ros-humble-libpointmatcher`
- README Quick Start: `path_following.launch.py` → 실존하는 `path.launch.py`
- README 토픽 표: `/{vehicle}/dvl_sim` → 실제 발행 토픽 `/{vehicle}/dvl`
- `fft_localization_node` 주석의 거짓 주장 정정 (기본값이 slam.yaml과 다름 — 동작 불변)

## [0.4.0] - 2026-06-24

**P4 알고리즘·수치 정확성 + 의도적 동작 변경.** P3까지의 동작 보존 철칙이 끝나는 단계 — 수치 버그 수정·표준 정합·ROS 그래프 정렬·god-method 분해를 위해 런타임/수치/frame_id를 의도적으로 바꿨다. 검증 기준이 "이전과 동일"에서 "의도대로 올바른가"로 전환됨. 9개 모듈 통째 정독 + 외부 표준 조사 + 적대 검증(0 refuted) + ralplan 합의를 거친 87개 진단에서 도출. 모든 변경은 executor 작업 + code-reviewer 독립 검증(전건 APPROVE, 0 blocker).

### Removed
- **`kalman_node` 일괄 제거** (P4a T-A1): legacy-dead 노드 — `kalman.py`가 부재 패키지 `uuv_sensor_ros_plugins_msgs`를 import해 실행 시 크래시, launch 참조 0, 출력 토픽(`kalman_odom`/`kalman_path`) 구독처 0, H 행렬이 0으로 하드코딩돼 필터 자체가 무기능. 4중 증거로 dead 확정. `core/kalman.py`·`core/kalman_filter.py`(전적으로 kalman.py 전용)·`nodes/kalman_node.py`·`test/test_kalman.py`·CMakeLists install 블록 제거. live한 `dead_reckoning`은 불변(`stonefish_msgs.DVL` 사용, 공유 코드 없음).

### Fixed
- **octree leaf size = 2×resolution 버그** (P4a T-A3, `octree.py`): `_update_recursive`가 `node.size <= resolution*2`에서 재귀를 멈춰 모든 leaf voxel이 `2*resolution` 큐브(의도 부피의 8배)였음. 생성자 계약·OctoMap 표준(Hornung 2013)·`world_to_key` 양자화와 모순. `<= resolution + 1e-9`로 수정(epsilon 가드).
- **DDA free-space voxel corner bias** (P4a T-A4 part1, `mapping_3d.py`): `voxel_center = key * resolution`이 cell 모서리를 가리켜 shadow 검증에 half-voxel 편향. `key`는 floor 양자화 인덱스(C++ `dda_traversal.cpp` `world_to_key` 확인)이므로 cell 중심은 `(key + 0.5) * resolution`. Python fallback 경로와도 정합. (free 영역 반전 + first-hit 정의는 FLS 이미지 row 규약 + live-sim 검증 필요라 시뮬레이터 단계로 연기 — part2 #37.)
- **Python ICP fallback wrong-transform** (P4a T-A5, `cpp/pcl.py`): perfect-overlap cloud에서 잘못된 transform으로 수렴. P4_FLAGS의 float32 가설은 실증 반박(float32/float64 바이트 동일). 실제 원인은 고정 `outlier_ratio=0.8`이 100% 겹침에서도 대응점 20%를 잘라 Kabsch centroid를 편향시킨 것(TrICP, Chetverikov 2002는 trim 비율 = 실제 overlap 요구). `0.8 → 1.0`으로 수정(`max_correspondence_distance=3.0`이 실제 outlier 거부). C++ 경로(libpointmatcher, 런타임 live)는 0.8이 옳아 불변. xfail 제거 + atol 1e-6 정밀 테스트 추가.
- **dead_reckoning depth 하드코딩 0.0** (P4b T-B1/T-B2, `dead_reckoning.py` + 신규 `core/depth.py`): `curr_depth`가 항상 0이라 z 추정 불능. 주석 처리됐던 식은 1000배 단위 오류(절대 Pa를 kPa 상수로 나눔) + Z-up 부호 오류로 수면에서 -9990m 반환. 순수 함수 `pressure_to_depth()`를 `core/depth.py`로 추출 — `h = (P - 101325) / (1025 * 9.80665)`, NED z-down(수면→0, 10m→+10, P4_FLAGS의 Z-up 식과 부호 반대). `offset=2.5` fudge 제거. (SLAM은 현재 sim의 odometry를 직접 구독하므로 이 수정은 DR 노드 자체 출력만 바꾸고 맵은 DR 재배선 전까지 불변.)

### Changed
- **robust Cauchy loss를 loop closure에 연결 + PCM 수치 안정** (P4c T-C2/T-C3, `factor_graph.py`): `create_robust_full_noise_model`이 dead code(caller 0)였고 loop closure 팩터가 non-robust Gaussian을 써 outlier loop에 무방비였음. `add_loop_closure`(NSSM)에만 robust=True 연결, SSM·odometry는 non-robust 유지(표준 관행: odometry reliable, loop만 robust). Cauchy `c=1.0 → 3.0`(3σ에서 weight 0.5, 보수적; config `slam_loop_robust_c`로 조정). 근거: AEROS 2022, Mangelson et al. ICRA 2018, GTSAM robust noise model 문서. `verify_pcm`은 `np.linalg.inv(cov)` → `np.linalg.solve(cov, error)`(near-singular에서 더 안정, well-conditioned 동치). PCM 임계값 11.34 = chi2.ppf(0.99,3)는 표준 부합이라 무변경, 주석에 출처 명시. (궤적 개선 효과는 시뮬레이터 검증 필요 — 코드는 배선·스케일의 올바름까지만 증명.)
- **전역 frame_id를 `world_ned`로 통일** (P4d T-D1, `slam.py` 8곳 + `visualization.py` 1곳): SLAM 출력 메시지가 전역 프레임에 `"map"`/`"{rov}_map"`(ENU 명명)을 쓰는데 3D 경로·시뮬레이터는 이미 `world_ned`(NED)를 씀. Stonefish가 전역을 NED로 발행하므로 single/multi-ROV 양 경로를 `world_ned`로 통일. REP-105 `odom→base_link` TF 체인과 child_frame_id body 프레임은 의도적 보존(결정: "전역 프레임만"). TF는 identity라 좌표 변환 없이 이름만 정합. 의도적 REP-103/105 비순응(근거 `docs/CONVENTIONS.md` §2.0).
- **콜백 `SLAM_callback_integrated` → `slam_callback_integrated`** (P4d, `slam.py`): PEP 8 snake_case, 3곳(정의·등록·주석).
- **C++ 확장 누락을 침묵하지 않고 경고** (P4a T-A5, `cpp/__init__.py`): `except ImportError: pass`가 모든 `.so` import 실패를 삼켜, 확장 없는 빌드가 pure-Python fallback(특히 덜 정밀한 `pcl.py` ICP)을 표시 없이 실행. 누락 모듈명과 degrade 내용을 명시하는 경고 1건을 emit. `.so` 존재 시 동작은 불변.

### Refactored
- **god-method 분해 — 동작 보존** (P4e T-E1/T-E2, `mapping_3d.py`): `process_sonar_ray`(274줄)를 29줄 조율자 + 3 헬퍼(`_detect_hits`·`_update_free_space_voxels`·`_update_occupied_voxels`)로, `process_sonar_image`(240줄)를 75줄 조율자 + 3 헬퍼(`_prepare_image_frame`·`_process_all_rays`·`_apply_octree_updates`)로 분해. public 시그니처 불변, 순수 코드 이동(code-reviewer가 라인 단위 문자 동등 확인). characterization 테스트를 분해 **전** 작성(oracle), mutation testing으로 비공허성 증명. 인프라: `conftest.py`에 `load_factor_graph`·`load_mapping_3d` fixture 추가(패키지 `__init__`의 cv_bridge import 체인 우회로 clean env 로드).

### Verification
- clean env(`env -i /usr/bin/python3 -m pytest -q`) 베이스라인 **37 passed, 0 xfail**(P4 시작 13+1xfail → 37 passed로 단조 증가).
- 모든 동작 변경은 TDD(수치 버그는 RED→GREEN) 또는 characterization(refactor) + code-reviewer 독립 검증(전건 APPROVE, 0 blocker).
- `package.xml` version 0.3.1 → 0.4.0(SemVer minor — 의도적 동작 변경).

### Notes
- **노드명 `slam_node` 충돌 가정 반증**(측정): 모든 launch 경로가 단일 `slam_node`를 실행하고, 둘을 함께 띄우는 유일한 경로(`mapping_combined_standalone`)는 `PushRosNamespace`로 분리(`/mapping_2d/slam_node` vs `/mapping_3d/slam_node`)해 전체 노드 경로가 고유. 이름 공유는 config YAML 네임스페이스 재사용 목적의 의도적 설계 — 버그 아님, P4 수정 범위에서 제외.
- **토픽 rename 불필요**(측정): SLAM 발행 토픽을 sim이 구독하는 것 0개(완전 단방향). 양방향 계약은 sim→slam의 `/{vehicle}/odometry|fls/image|dvl|imu|pressure`뿐(vehicle_name 파라미터 동기화) — 불변.
- **알려진 한계**: `process_sonar_image` characterization은 identity pose만 써 transform 합성 *순서*를 검증 못 함(분해는 라인 동등으로 이미 증명). 순서 고정 테스트가 전체 실행 시 flaky라(원인 미규명) 거짓 안전망 대신 docstring에 한계 명시 — 향후 합성 수정 시 isolation-stable 테스트 작성.
- **연기**: T-A4 part2(free 영역 반전 + first-hit 정의)는 FLS 이미지 row 규약 확정 + live-sim 검증 필요라 시뮬레이터 단계로 이월(#37).

## [0.3.1] - 2026-06-24

### Changed
- **P3 기업표준 재구조화 — 동작 보존** (2026-06-24): 명명·구조 컨벤션 통일 + import 정리 + 모듈화. 런타임/수치/토픽그래프 변경 없음(pytest 16→20 passed + 1 xfailed, live 동작 불변).
  - **import 정리**: wildcard import 17곳 제거 — 정적 게이트로 dead/live 분류 후 dead 10곳 삭제 + live 7곳 명시 import화. 우리 소스 `from X import *` 0개. (CONVENTIONS §2.2 절대 import 표준 준수)
  - **dead code 제거**: `core/slam.py`의 미사용 `pointcloud2_to_xyz_array`(+orphan `import struct`) 삭제.
  - **포맷 통일**: `core/dead_reckoning.py` 코드 들여쓰기 탭→4-space(PEP 8, AST 동등 보존).
  - **모듈화**: `core/mapping_3d.py`의 무상태 변환 함수 `create_transform_matrix`·`pose_msg_to_transform`를 클래스 메서드에서 모듈 레벨 함수로 추출(실행 statement AST 동등).
  - **메타데이터**: `package.xml` version 0.1.0→0.3.1(CHANGELOG 정렬). CONVENTIONS 줄번호 drift 정정·P3 안전망 절·P4 백로그 추가. P4_FLAGS wildcard 17곳 동기화 + 신규 후보(depth 무력화·docstring 탭) 기록.
  - **안전망 추가**: `test/static_import_gate.py`(AST 정적 게이트), `test/test_wildcard_gate.py`(wildcard 분류 동결), `test/test_octree.py` adaptive 0.3 config 커버갭. rclpy/gtsam 부재 환경 대응(런타임 binding은 P4 sign-off).
  - **P4 격리**: 노드명 'slam_node' 3중충돌·standalone 노드 구조·god-method 분해·수치버그(pressure 1000배·ICP float32·fusion·Joseph)·frame_id world_ned 통일 등은 동작 변경이라 P4(`P4_FLAGS.md`).

### Added
- **Combined Mapping Standalone Launch** (2025-12-11): 3D/2D 매핑 동시 실행 지원
  - 파일: `launch/mapping_combined_standalone.launch.py`
  - 기능: GroupAction + PushRosNamespace로 3D 매핑과 2D 매핑을 독립적 네임스페이스에서 동시 실행
  - 효과: 단일 실행으로 3D/2D 통합 매핑 가능
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
- **FFT Phase Correlation fftshift 순서 오류** (2025-12-10): 위상 정보 왜곡 수정
  - 문제: `compute_phase_correlation()`에서 fft2 직후 fftshift 적용으로 위상 정보 왜곡
  - 원인: fftshift를 잘못된 시점에 적용하여 표준 phase correlation 구현에서 벗어남
  - 수정: fftshift를 ifft2 이후에만 적용 (표준 FFT phase correlation 구현에 따름)
  - 효과: 회전/병진 추정 정확도 향상
  - 파일: `localization_fft.py` (라인 420-431)
- **DFT Upsampling 좌표 계산 버그** (2025-12-10): Subpixel refinement 정밀도 향상
  - 문제: `_upsampled_dft()`에서 row_center/col_center를 계산하고 사용하지 않음
  - 원인: DFT가 항상 원점 근처를 샘플링하여 subpixel 위치 정확도 저하
  - 수정: Guizar-Sicairos 2008 논문에 따라 초기 offset 중심으로 DFT 샘플링하도록 수정
    * row_center, col_center를 이용한 올바른 좌표 변환 적용
    * DFT 샘플링 범위를 offset 주변으로 정확하게 계산
  - 효과: Subpixel refinement 정확도 향상 (회전/병진 모두)
  - 파일: `localization_fft.py` (라인 533-551)
  - 참고: Guizar-Sicairos, M., Thurman, S. T., & Sinclair, G. (2008) "Efficient subpixel image registration algorithms", Opt. Lett.
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
