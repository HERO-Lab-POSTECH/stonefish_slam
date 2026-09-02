# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Changed

- **config 6 → 1, launch 10 → 8, launch 인자 17 → 10** (`refactor/config-consolidation`):
  `sonar.yaml`·`feature.yaml`·`localization.yaml`·`factor_graph.yaml`·`mapping.yaml`·
  `slam.yaml` 은 전부 `slam_node.ros__parameters` 하나였고 launch 가 항상 여섯을 같이
  로드했다 — `config/slam.yaml` 한 파일의 섹션으로 합친다(값은 전부 보존, 코드 기본값에만
  있던 `sonar_topic`·`odom_topic`·`sonar_compressed`·`keyframe_duration_max`·
  `nssm.try_interval`·`publish_point_cloud` 도 적어 파일이 파라미터 카탈로그가 되게).
  `localization.launch.py`·`mapping.launch.py` 는 `slam.launch.py` 에 `mode` 와 플래그만
  넣던 래퍼라 삭제하고 `slam.launch.py mode:=slam|localization|mapping` preset 으로
  흡수(노드의 `mode` 파라미터는 그대로 — launch 만 바뀐다). `ssm_enable`/`nssm_enable`
  인자는 preset 과 yaml 이 대신하고, 평가 관측기 인자 6개는 `evaluate:=true|false`
  하나로, `enable_2d/3d_mapping` 은 빈 값이 yaml 을 뜻하도록(전에는 launch 기본 `true`
  가 yaml 값을 항상 덮었다). 우선순위는 launch docstring 에 한 줄로 고정: slam.yaml <
  method_*.yaml < override_config < mode preset < 명시 인자. 래퍼와 동등하게 mapping preset
  은 2D on/3D off, localization·mapping 모드는 rviz 기본 off(`rviz:=` 빈 값 = 모드별 기본)
- **yaml 루트 키를 `slam_node:` 에서 `/**:` 로** — combined standalone 이 노드를
  `/mapping_2d/slam_node`·`/mapping_3d/slam_node` 로 띄우는데 `slam_node:` 선택자는
  네임스페이스 노드에 **0개** 매칭돼 그 하니스는 지금까지 코드 기본값으로 돌았다
  (codex 검증). `/**` 는 어느 노드든 자기가 선언한 키만 받으므로 feature/FFT standalone
  도 같은 파일을 읽을 수 있게 됐다(전엔 노드명이 달라 dict 로만 받았다). 2D standalone 은
  항상 쓰던 0.1 m/px 를 launch 오버라이드로 유지(slam_node 의 0.2 는 메시지 크기 상한).
  3D standalone launch 의 bare `update_method` 키(선언 안 돼 조용히 버려짐)도 정정
- **거짓말하던 설정 두 건을 살린다**: `mapping_2d.map_2d_resolution` 은 yaml 이 0.1 을
  광고했지만 아무도 읽지 않았고 `slam.py` 가 0.2 를 하드코딩했다 — 이제 파라미터를
  읽고 yaml 값은 실제 값 0.2(동작 불변). `mapping_3d.propagation_radius/sigma` 는 선언만
  되고 mapper 로 전달되지 않아 `config.get` 기본값(2·1.5)으로 돌았다 — 전달한다(같은 값,
  동작 불변). `mapping_3d_standalone` 은 같은 키를 접두사 없이 선언해 yaml 에서 한 번도
  로드되지 않았다 — `mapping_3d.` 접두사로 정합
- 자동 계산은 못 넓혔다: `sonar.*` 중 이미지 메시지에서 나오는 것은 `num_beams`·
  `num_bins`(512×500) 뿐인데 세 모듈이 첫 프레임 전에 각도 테이블을 미리 계산하므로
  지연 초기화 없이는 못 뺀다(N7 god-method 분해와 같이 볼 것). 틸트·FOV·range 는
  메시지에도 TF 에도 없다(시뮬은 `world_ned→base_link` 만 발행) — 크로스 repo 가드
  `test_sonar_tilt_matches_sim_scenario.py` 가 그 자리를 맡는다

### Added

- **목표물 검출 라벨 채널 (`feat/semantic-labels`, Stage 2/4)**: sim 의
  `stonefish_sonar_yolo` 가 내는 `vision_msgs/Detection2DArray` 를 구독해
  CFAR 피크 픽셀마다 라벨(=클래스 인덱스+1)을 붙이고, 그 라벨을 `/slam/cloud`
  에 다섯 번째 필드로 실어 보낸다(요청의 "PCL XYZI 처럼 3D 좌표에 index 를
  추가"에 해당). 새 ROS-free 모듈 `core/semantic.py` 가 순수 로직을 갖는다 —
  `labels_from_detections`(bbox 안 픽셀에 라벨, 겹치면 나중 것이 이김),
  `detection_rows`(중심+크기 → 모서리, 신뢰도·비정수 class_id 필터),
  `pixel_to_bearing_range`/`sonar_to_pixel`/`slant_range_bearing`(좌표 변환),
  `PendingSemantic`(검출↔키프레임 stamp 짝짓기 큐).
  `feature_extraction` 안에 두지 않은 이유는 그 모듈이 상단에서 `cv_bridge`·
  `cfar` 를 끌어와 `load_module` 로 못 열기 때문이다 — ROS 없는 CI 에서 테스트가
  통째로 빠진다. 대신 `extract_features_with_pixels()` 를 더해 점과 그 점이 나온
  픽셀을 같이 돌려준다(`extract_features` 는 이를 호출하는 얇은 껍데기).

  **검출은 그 키프레임보다 늦게 온다.** YOLO 는 추론 중 프레임을 드랍하므로
  `ApproximateTimeSynchronizer` 로는 멈춘다. 대신 stamp 로 색인한 미결 큐를 두고
  검출 콜백·slam 콜백 **양쪽에서** 짝을 찾아 정확히 한 번 소비한다. 짝을 못 찾은
  항목은 `semantic.pending_timeout`(기본 3 s = 키프레임 간격 × 3) 워터마크를
  넘길 때 만료되며, 그 전까지는 "아직 안 옴"이지 유실이 아니다.

  계측은 `[INSTR] semantic` 한 줄로 따로 나간다(카운터 9종:
  `det_received`·`det_empty`·`det_below_conf`·`det_bad_class`·`det_duplicate`·
  `det_missing`·`det_expired`·`det_matched`·`det_no_labeled_peaks`).

  **`semantic.enable: false` 는 semantic 이전과 같은 산출물을 낸다** — 구독도,
  새 토픽도, 새 `[INSTR]` 줄도 생기지 않고 `/slam/cloud` 는 4필드(XYZI) 스키마를
  유지한다. `vision_msgs` import 조차 `_init_semantic()` 의 조기 return 뒤에
  있어, 그 패키지가 없는 머신에서도 off 런은 그대로 뜬다. (프로세스가 바이트
  단위로 같다는 뜻은 아니다: `semantic.*` 파라미터 5개가 선언되므로
  `ros2 param list` 는 달라지고 `Keyframe` 에 필드 둘이 는다.) 이 동일성이
  A/B 기준선이므로 `test_semantic_off_identity.py` 가 AST 로 못 박는다 —
  PointField 골든 표, `PointCloudXYZIL`·`PointCloudXYZPL` 게이트,
  `[INSTR] semantic` 게이트, 구독·발행자 위치, 기본값, 그리고 **게이트 자신이
  부정 조건에 속지 않는지**(`if not semantic_enable:` 로 뒤집어도 통과하면
  회귀 테스트가 아무것도 안 지킨다).

  검증용 주입기 `scripts/fake_detection_publisher.py` 를 함께 둔다 — 시뮬 씬에
  학습된 클래스(sofa) 자산이 없어 진짜 YOLO 로는 소비 경로를 검증할 수 없다.
  이미지 header 를 그대로 복사해 되쏘므로 `det_missing`/`det_expired` 가 0 이
  아니면 그건 타이밍이 아니라 큐 버그다.

  크로스 repo: 발행자는 `stonefish_sim` 의 `feat/sonar-yolo-detection2d`
  (상대 PR 상호 링크는 본 PR 본문 — `CONTRIBUTING.md` §5).
  `package.xml` 에 `vision_msgs` 를 `exec_depend` 로 추가했다.

- **검출을 pose graph 가 소비한다 — 랜드마크 factor (Stage 4)**: 검출 하나마다
  `BearingRangeFactor2D(X(k), L(j))` 를 ICP·odometry factor 와 **같은 그래프**에
  넣는다. 이것이 "검출 결과가 위치 인식에 사용된다"의 실제 코드 경로이고, 증거는
  `[INSTR] semantic` 의 `landmark_factors_added` 다.

  측정값은 **bbox 중심 픽셀**이다(라벨이 붙은 CFAR 점의 centroid 가 아니라).
  centroid 를 쓰면 bbox 안에 CFAR 피크가 하나도 없는 프레임에서 factor 가 아예
  안 생겨, "검출이 쓰였는가"의 답이 검출기 성능에 의존한다. bbox 중심이면
  **검출 1건 ⇒ factor 1건이 구성상 보장**되고 factor 가 0 이 되는 원인은 검출
  부재·stamp 불일치·ISAM2 실패 셋뿐이라 카운터로 갈린다.

  - `utils/conversions.py` 에 `L(j)` 심볼 추가.
  - `FactorGraph.add_landmark_factor()` — 같은 클래스가 `assoc_radius` 안이면
    재사용, 아니면 새 변수. 한 tick 에 두 검출이 같은 새 랜드마크로 연관돼도
    `values.insert` 는 한 번만 한다(두 번이면 gtsam 이 죽는다).
  - `create_landmark_noise_model()` 신설 — `create_robust_full_noise_model` 은
    covariance 를 받고 `robust_loop_c` 를 고정으로 쓰므로 재사용할 수 없다.
    시그마는 `[bearing_rad, range_m]` 순서(BearingRange2D 의 측정 순서)이고,
    Cauchy c 는 loop closure 와 **독립 파라미터**다 — 오검출률과 오루프율은
    다른 값이다.
  - **원자성**: 새 랜드마크 메타데이터는 `pending_landmarks` 에 두었다가
    `isam.update` 가 성공한 뒤에만 `landmarks` 로 옮긴다. 실패한 tick 의
    id 가 남으면 다음 관측이 존재하지 않는 변수에 연관된다. id 는 재사용하지
    않는다.
  - **`update_graph` 의 키프레임 개수**: `values.size()` → `len(self.keyframes)`.
    랜드마크가 pose 와 같은 `Values` 에 들어가므로 `size()` 는 더 이상 키프레임
    수가 아니고, 검출이 **한 번이라도** 쓰이는 순간
    `atPose2(X(size-1))` 이 존재하지 않는 키를 물어 노드가 죽는다. 포즈 갱신
    루프에는 `values.exists(X(x))` 가드를 둬서, 실패한 tick 때문에 변수가 없는
    키프레임은 dead-reckoning pose 를 유지하고 넘어간다.
  - 랜드마크 위치는 매 tick 최적화 결과로 갱신해 연관 반경이 현재 추정치를
    기준으로 재도록 한다.
  - `semantic.landmark.enable: false` 면 라벨 채널만 돌고 factor 는 안 생긴다.
    `mapping-only` 모드에서도 안 생긴다(factor graph 자체가 안 돈다).

- **3D 복원에 라벨을 얹는다 — 복셀 역투영과 `mapping/cloud_3d` (Stage 3)**:
  점유 복셀을 검출이 그려진 이미지로 **되쏘아** bbox 안에 떨어지는 것에 클래스를
  붙이고, `[x, y, z, prob, label]` PointCloud2 로 발행한다. 요청의 "3D 복원 후
  그 정보를 위치 인식에 사용"에서 3D 산출물에 해당한다.

  C++ 경로를 안 건드리는 이유: `ray_processor` 는 픽셀 인덱스를 C++ 밖으로
  내보내지 않고 OctoMap 노드에는 라벨 슬롯이 없다. 갱신 시점에 라벨을 붙이려면
  `VoxelUpdate`·옥트리 노드·pybind 세 층을 같이 고쳐야 하는데, 역투영은 파이썬
  40여 줄로 같은 결과를 낸다.

  - **경사거리로 역산한다.** `ray_processor.cpp` 는 복셀을
    `x=r·cos(v)·cos(b), y=r·cos(v)·sin(b), z=r·sin(v)` 로 놓으므로 `|P|=r` 이다.
    `mapping_3d._voxel_to_sonar_coords` 는 `sqrt(x²+y²)`(수평거리)를 내므로
    앙각이 0 이 아닌 복셀에서 range bin 이 어긋난다 — 그래서 쓰지 않는다.
    C++ 이 실제로 그 자리에 놓는지는 `test_cpp_extensions.py` 의 왕복 케이스가
    스테이징된 `.so` 로 확인한다.
  - **복셀 중심에는 정확한 픽셀 역산이 없다**(격자에 스냅된 자리라서). 허용
    오차는 복셀 반대각선(`voxel_resolution·√3/2`)을 픽셀로 환산한 pad 다.
  - **수명**: `max_frames` 리셋으로 지도가 비면 라벨도 같이 비운다. 출력은 항상
    **현재 점유 복셀과의 교집합**이라, 점유가 풀린 셀의 라벨은 소비자에게
    도달하지 않는다. 충돌은 나중 관측이 이긴다.
  - **재투영 큐**: `keyframes[last_map_update_kf:]` 는 늦게 검출이 붙은
    키프레임을 다시 돌려주지 않는다. 그래서 검출이 확정된 키프레임을
    `pending_semantic_map` 에 모아 두었다가 map tick 에서 비운다.
  - `⚠️ max_frames` 리셋이 **C++ 옥트리는 안 지운다**는 기존 결함을 이 자리에서
    발견했다 — 배포 기본값(`use_cpp_backend: true`)에서 `max_frames` 가 사실상
    동작하지 않는다. 매핑 정책 결정이라 범위 밖으로 두고 `P4_FLAGS.md` 에 적었다.

- **2차 적대 검증 지적 6건 반영** (agy oracle, ground 4 — codex 는 사용량 한도로
  중단돼 다른 벤더로 다시 돌렸다):
  - **MAJOR**: `semantic.enable: true` + `enable_3d_mapping: false` 조합에서
    `pending_semantic_map` 이 영원히 안 비워져 검출이 붙은 모든 키프레임(점군·
    이미지 포함)이 쌓였다 — 큐를 비우는 곳이 3D map tick 안에만 있었기 때문이다.
    `label_3d` 를 `enable_3d_mapping` 과 묶었다. 라벨링 중 예외가 나도 큐가
    막히지 않도록 `clear()` 를 `finally` 로 옮겼다(안 그러면 다음 틱마다 같은
    예외가 재발한다).
  - **MINOR**: 한 map tick 에서 키프레임이 M 개면 C++ 옥트리를 M+1 번 걸었다 →
    점유 복셀을 한 번만 읽어 넘긴다(`label_voxels_from_keyframe(points=...)`,
    `labels_for_points()` 분리).
  - **MINOR**: 콜백이 `update_graph` 에 닿기 전에 예외로 끊기면 그 키프레임은
    append 되지 않고 다음 키프레임이 같은 X 인덱스를 받는다. 그 상태로 늦은
    검출이 오면 **엉뚱한 pose** 를 조용히 구속했다 → 랜드마크 factor 를 붙이기
    전에 `landmark_pose_key_is_valid()` 로 확인하고, 어긋나면
    `pose_key_mismatch` 로 센다. ⚠️ 이 가드의 첫 판이 **정상 경로까지 막았다**:
    검출이 먼저 와 있으면 키프레임은 아직 append 전이라
    `pose_key == len(keyframes)` 가 맞는데, 동일성만 보면 전부 걸러진다. bag
    재생에서 `landmark_factors_added=0` · `pose_key_mismatch=19` 로 드러났고,
    판정을 순수 함수로 떼어 네 케이스를 테스트로 고정했다 — 단위 테스트 195건이
    이 구멍을 못 잡았다는 뜻이라 그 자체를 기록으로 남긴다.
  - **NIT**: `add_landmark_factor` 가 검출마다 `calculateEstimate()` 로 전체
    Bayes tree 를 다시 풀었다 → `isam.valueExists()`(O(1) theta_ 조회).
  - **NIT**: `config/slam.yaml` 의 `semantic:` 블록에 `label_3d` 가 빠져 있었다
    (선언·문서에는 있는데 파라미터 카탈로그에만 없었다) → 추가하고, 블록이
    선언된 키를 다 담는지 테스트로 고정했다.
  - **NIT**: off 동일성 docstring 의 "파라미터 5개" 를 실제 11개로 정정.

- **`cpp/pcl.py` fallback 의 descriptor 집계를 C++ 과 맞췄다**: 순수 파이썬
  `downsample` 은 descriptor 를 **평균**냈다. 이 채널로 keyframe index 와
  semantic 라벨이 나가므로, 한 복셀에 라벨 1·2 가 섞이면 1.5 가 되고 소비자가
  정수로 자르면 경고 없이 클래스 1 이 된다. C++ 경로(libpointmatcher OctreeGrid,
  `samplingMethod=3`)는 medoid 를 고르므로 정수가 보존되는데, fallback 만 달라서
  `.so` 유무로 라벨이 갈렸다 — 이제 대표점(centroid 최근접)의 descriptor 를
  그대로 쓴다. 점 좌표는 종전대로 centroid 라 ICP 동작은 그대로다
  (`docs/CONVENTIONS.md` §2.9 "C++ 동작을 바꾸면 fallback 도 동기화한다").

- **적대 검증 지적 9건 반영** (codex oracle, ground 4):
  - **BLOCKER**: mapping-only + semantic on 에서 두 번째 키프레임부터 점군 발행이
    `ValueError` 로 죽었다. `frame.update()` 가 점을 넣기 **전에** 불려
    `transf_points` 가 (0,2) 로 굳는데 `labels` 는 N 이라 `np.c_` 가 터진다.
    `aligned_labels()` 로 길이를 `transf_points` 에 맞춘다(ISAM2 실패 tick 도 같은
    불일치를 만든다).
  - **MAJOR**: 워터마크를 키프레임 콜백에서만 돌려, 키프레임이 안 생기는
    구간(정지·특징 없음)에서 큐가 무한히 자랐다 → 검출 콜백에서도 돌린다.
    `/clock` 이 크게 뒤로 뛰면(bag 루프) cutoff 가 후퇴해 이전 epoch 항목이
    영원히 안 만료되므로, 그때는 큐를 통째로 비운다.
  - **MAJOR**: 짝짓기 동률에서 dict 순회 순서가 결과를 갈랐다 → 엄격한 개선일
    때만 갱신해 **먼저 들어온 쪽**이 이기게 했다. 같은 stamp 의 키프레임이
    덮어써지는 사건은 `kf_stamp_collision` 으로 센다.
  - **MAJOR**: 늦게 온 검출이 계측 로그에 안 나타났다 → 매칭 성공 시 검출
    콜백에서도 요약을 낸다. (점군은 다음 키프레임 발행에 실린다 — 전체를 매번
    다시 만드는 함수라 검출마다 부르면 비용이 감당이 안 된다.)
  - **MAJOR**: A/B 절차의 `-p semantic.enable:=true` 는 launch 문법이 아니었다 →
    `slam.launch.py` 에 `semantic:=true|false` 인자를 넣었다. 주입기 문서의
    "`det_expired` 가 0 이 아니면 큐 버그" 도 **거짓**이었다 — SLAM 은
    `filter.skip` 을 통과해 키프레임이 된 프레임만 큐에 넣으므로 나머지 검출은
    정상적으로 만료된다. 판정 기준을 `det_missing == 0` 과
    `landmark_factors_added ≈ det_matched` 로 고쳤다.
  - **MINOR**: off 동일성 주장의 범위를 산출물(알고리즘·토픽·스키마·로그)로
    좁혔다 — 파라미터 5개는 off 에서도 선언되고 `Keyframe` 필드도 는다.
  - **MINOR**: AST 게이트가 `if not semantic_enable:` 로 뒤집어도 통과했다 →
    부정 조건을 거부하고, 그 성질 자체를 테스트로 고정했다.
  - **MINOR**: `class_id` 가 `-1`(라벨 0 과 구분 불가)·`255`(uint8 wrap)여도
    통과했다 → 0..254 범위 밖은 `det_bad_class` 로 버린다.
  - **MINOR**: `det_empty` 가 필터 **뒤** 값을 세어 "검출기가 빈 결과를 냈다"로
    읽을 수 없었다 → 발행자가 보낸 탐지 0건만 센다.

- **위치 추정 파이프라인 계측 I1~I11** (`feat/loc-instrumentation`): "icp 0%" 도
  "DR seed 17%" 도 **분모가 없는 보고**였다 — 어느 경로를 몇 번 탔는지 세는 곳이
  하나도 없었다. 판정은 그대로 두고 세기만 한다. `Localization.ssm_disabled_count`
  (I1, 이 값이 **키프레임 총수 -1** 과 같으면 원인이 알고리즘이 아니라
  `ssm.enable: false` 라는 **설정**임이 확정된다 — 첫 키프레임은 prior 만 넣고 SSM 을
  부르지 않으므로 분모가 하나 작다), `icp_attempted`/`icp_converged`(I2, 비율의 분모),
  `icp_factor_added`/`odom_factor_fallback`/`ssm_init_failed`(I3, factor graph 실제
  구성비 — **odometry factor 수는 뒤 둘의 합**이다. 초기화 실패는 `ssm_init_failed`,
  ICP 를 돌리고 실패한 것만 `odom_factor_fallback` 으로 갈린다),
  seed 출처 분해(I4·I5) · 기각 사유 분리(I6) · `dr_ty` 동시 기록(I7) ·
  FFT `rot_peak`/`trans_peak`/`covariance`/`rotation_fft` 소비(I8·I9·I10) ·
  병진 스케일 비율(I11). 로그 태그는 `[INSTR]`.
  **I4 는 버그 수정이기도 하다** — `fft_is_dr_fallback` 이 write-only 라 DR
  fallback 시드에도 `[FFT_SEED]` 가 붙어 **로그가 거짓을 말했다**. 읽어서
  `[DR_SEED]` 로 분기한다. I8·I9·I10 이 읽는 값은 FFT 가 이미 계산해 반환 dict 에
  실어 보내던 것이라 `localization_fft.py` 는 주석 외에 한 줄도 안 바뀐다.
  I12(tilt A/B 이중 점군)는 오프라인 bag 전용이라 제외했고 I11 이 같은 질문에 더
  싸게 답한다. I13 은 기존 `mean_err` 재사용이라 코드 0줄
- 계측 배선 회귀 테스트 7개 — `slam.py` 는 import-time 에 rclpy·gtsam·cv_bridge 를
  끌어와 path-load 가 닿지 않으므로 AST 로 배선의 **존재**를 고정한다. 막으려는 것은
  값의 오류가 아니라 그 이전 단계, 즉 계측이 조용히 끊겨 로그가 영영 안 나오는
  상황이다. 특히 `fft_is_dr_fallback` 이 다시 write-only 가 되는 회귀를 못 박는다

- **실해역 bag 재생 지원** (김민종 colcon_ws2 통합): `sonar_topic`/`odom_topic`/
  `sonar_compressed` 파라미터로 데이터 소스 전환(기본값은 기존 시뮬 토픽 그대로),
  CompressedImage 디코드(BGR→GRAY 포함), `config/real_bag_overrides.yaml` +
  `config/icp_real_bag.yaml` 프로파일, `launch/slam_real_bag.launch.py`
  (odom→TF 브리지 + real-bag rviz 포함), `rviz/real_bag.rviz`
- **평가 노드 2종 + TF 브리지**: `slam_accuracy_monitor`(GT 대비 ATE/Acc@r/경로 오차),
  `traj_2d_error_accumulator`(SLAM pose vs GT 2D 오차 누적), `odom_tf_bridge`
  (bag의 Odometry를 TF로 재발행). slam.launch.py에서 launch 인자로 on/off
  (`enable_accuracy_monitor`·`enable_traj_error_accumulator`, 기본 on)
- `keyframe_duration_max` 파라미터 — 초과 시 이동량 무관 keyframe 강제(저속 구간
  cadence 유지). 기본 0.0(비활성)으로 기존 동작 불변
- slam.launch.py에 `override_config`(프로파일 yaml 후순위 로드)·`icp_config_file` 인자 추가

### Removed

- **`MappingProfiler` — 헤더만 있는 빈 CSV 만 남기던 프로파일러**
  (`feat/loc-instrumentation`): `start()`·`close()` 는 불리는데 `record_frame()` 의
  호출자가 0이라 매 실행 `/tmp/mapping_profiling.csv` 에 행이 하나도 없는 파일이
  생겼다. `record_frame` 은 `_print_profiling_stats` 안에 있었고 그 호출부는 main
  에서 이미 주석 처리돼 있어 **삭제 전에도 실행된 적이 없다** — 죽어 있던 사실이
  드러난 것뿐이다. 클래스·재수출·배선과 함께, 소비자를 잃은 `enable_profiling`
  ROS 파라미터도 제거했다(효과 없는 노브를 오퍼레이터에게 보여주느니 없애는 편이
  정직하다). **`profiling_enabled` 가 게이트하는 `perf_counter` 체인은 남긴다** —
  `t_ray_total`·`t_octree_total` 이 헬퍼 3개의 반환 시그니처에 박혀 있어 떼어내는
  것이 시그니처 수술이고, 그건 god-method 분해의 범위다
- **호출자 없는 코드 954줄**(`chore/dead-code-cleanup`): Python 메서드 14개
  (`feature_extraction.polar_to_cartesian` 106줄로 `polar_to_cartesian` 이 3벌→2벌 ·
  `FactorGraph` 4개 · `SonarMapping2D` 4개 · `SonarMapping3D` 4개 ·
  `ICP.getCovariance` · `HierarchicalOctree.get_voxel_key`) · `OculusProperty` 의 실
  Oculus 하드웨어 드라이버 군(`configure`·`remap`·72줄 psf 커널·`noise`·부품번호 표,
  216→64줄, 클래스 자체는 살아 있다) · C++ 6개 함수와 헤더 선언
  (`insert_voxels_batch_native` 115줄 · `set_log_odds_thresholds` 바인딩 포함 ·
  `compute_ray_direction` · `find_first_hit` · `world_to_voxel_key` ·
  `compute_intensity_weight`) · write-only 배선 `all_slam_keyframes`(속성·파라미터·
  호출부 전부, 2D 는 저장만 하고 3D 는 docstring 이 "not used in 3D" 라고 적고 있었다) ·
  단일 줄 7건(tf2 Buffer/Listener — `/tf`·`/tf_static` 를 30초 버퍼로 실제 구독하는데
  참조 0이었다 · 중복 `CVbridge` · 미사용 `pose` · `mapping_stats['maps_published']` ·
  미사용 `dt` · `source_pose_info` — `np.linalg.inv` 결과를 안 써서 shgo 의
  prior-covariance 가중이 조용히 무효였다 · `log_odds_threshold`).
  **판정은 이 트리에서 재측정했다** — 계획의 "호출자 0" 은 PR #18·#19 이전 기준이라
  두 건이 이미 틀렸다: `query_cell`(#18 의 `test_cpp_extensions.py` 가 3곳에서 사용)과
  `RayProcessorConfig::bearing_resolution`(`mapping_3d.py:235` 가 설정)은 **삭제하지
  않았다**
- **`config/localization.yaml` 의 절대경로 `icp_config`**: 값이
  `/workspace/colcon_ws/...` 로 이 컨테이너에도 없는 경로다. launch 경로에서는
  `slam.launch.py` 가 이겨 무해했지만 `ros2 run` 직접 실행 시 없는 파일을 읽었다.
  `slam.py:105-107` 의 package-share 기본값에 위임한다

### Changed

- **3D 매퍼의 두 무한 성장 통계 딕셔너리 제거**(P1-8, 메모리 누수): `performance_stats`
  (5개 리스트)와 `profiling_data`(9개 리스트) 모두 상한이 없고, trim 하던 호출부는
  주석 처리돼 있었다. 두 딕셔너리를 읽던 `get_performance_summary`·
  `_print_profiling_stats` 는 호출자가 0이라 같은 커밋에서 함께 삭제했다 —
  나눠 하면 한쪽이 다른 쪽을 깬다. `enable_profiling` 플래그는 남는다
  (`MappingProfiler` CSV 시작이 여전히 읽는다). ⚠️ `profiling_enabled` 가 게이트하는
  `perf_counter` 계측 체인은 **의도적으로 남겼다** — 소비자를 잃었지만 헬퍼 3개의 반환
  시그니처에 박혀 있고 `feat/loc-instrumentation` 이 이 층을 재설계한다
- **`utils/fusion.py::ema_fusion` 시그니처에서 `observation_count` 제거**: 본문이 한
  번도 읽지 않는 파라미터였다. 판정 로직은 불변
- **`utils/` 7개 모듈에 `__all__` 추가**: P4_FLAGS 는 5개로 적었으나 실측 7개 전부다
  (`fusion.py`·`profiler.py` 누락). wildcard 게이트가 `__all__` 을 export set 으로
  읽으므로 각 모듈의 public 심볼 전부를 넣었다
- **주석·docstring 의 사실 오류 4건**: `localization_fft` 의 `# Left` →
  `# Starboard (+y, NED body)`(코드는 정상, 주석만 반대였다) · `CodeTimer` docstring 의
  "Disable output by setting silent = False" → `True` · `conversions.r2g` 의
  `-> gtsam.Pose3` 힌트 제거(Quaternion 분기는 `Rot3` 를 반환한다) · `mapping_3d` 모듈
  docstring 에 C++/Python 두 경로와 갱신법 3종 명시. `dead_reckoning.py` 의 탭 34개도
  공백으로
- **SSM에서 FFT 성공 시 ICP 대체 → ICP 초기해 시드로 변경** (김민종 통합): FFT 변환이
  ICP를 건너뛰던 경로를 제거하고 FFT를 초기 추정으로 넘겨 ICP가 회전·병진을 정밀화.
  검증(LARGE_TRANSFORMATION·overlap)이 FFT 경로에도 항상 적용됨. status에
  `[FFT_SEED]` 표기
- `FactorGraph.update_graph`가 매 갱신마다 전체 keyframe 대신 최근 10개 윈도우만
  재갱신(루프 클로저 대기열이 있으면 전체 갱신) — 장기 주행에서 O(N) 비용 상한
- `nssm.try_interval` 파라미터(기본 1=기존 동작): N keyframe마다만 루프 클로저 시도
- `publish_point_cloud` 파라미터(기본 true=기존 동작): 실데이터에서 콜백 스레드를
  ~30s 블로킹(N=45 실측)하던 클라우드 발행을 끌 수 있게 게이트
- SLAM odom 출력에 pose covariance 복사(기존엔 0으로 발행)
- `mapping_combined_standalone.launch.py`에 `use_sim_time` 인자 전파

- **메인 파이프라인 3D 갱신법이 설정 의도대로 IWLO로 동작** (`fix/audit-bugs`,
  런타임 수치 변경): `slam_node`가 `mapping_3d.{update_method,sharpness,decay_rate,
  min_alpha,use_cpp_ray_processor}`를 선언·전달하지 않아 `config/mapping.yaml`의
  `update_method: 'iwlo'`가 조용히 무시되고 항상 log_odds로 동작하던 3중 단절을 배선.
  3D intensity threshold도 2D 값(10) 대신 `mapping_3d.intensity_threshold`(35)를 사용.
  launch 기본값은 하드코딩 대신 mapping.yaml에서 해석(standalone launch와 동일 패턴).
  **닫힌루프 맵 품질은 RTX4070 실기 sign-off 이월**(P4_FLAGS). method YAML의
  무소비 키(`intensity_max`·`occupied_threshold`)는 제거(하드코딩 실측 근거 주석화)

### Added

- README Docker 설치 안내 — 전용 배포 repo
  [`stonefish_bringup`](https://github.com/HERO-Lab-POSTECH/stonefish_bringup) 참조
  (sim과 동일 이미지에 slam 포함 — C++ 의존성·`.so` 확장까지 이미지 빌드에서 완결)
- 협업 규칙 `CONTRIBUTING.md` + PR 템플릿 (발효 2026-07-23)
- README Testing 섹션 (pytest용 pybind11 `.so` 스테이징 절차)

### Removed

- `fft_localization.min_ppr`·`reject_on_failure` 파라미터 제거 — 선언·로드만 되고
  검증 경로 어디서도 미적용이던 dead knobs (참조 전수 추적으로 확인, 동작 불변)
- **동작 불변 dead code 일괄 정리** (grep 재확증 후 삭제, `.so` 재빌드·37 passed 불변 확인):
  - `utils/conversions.py::pose223` (0 호출자; `pose322`·`build_rgb_cloud`/`n2r`는 live 유지)
  - `core/cfar.py::CFAR.detect2`/`self.detector2` + `cpp/cfar.cpp`의 `ca2`/`soca2`/`goca2`/`os2`
    C++ 함수 4종과 PYBIND11 바인딩 (`.detect`/`.detector`만 실사용; `calc_WGN_pfa_*`는 threshold
    계산에서 호출돼 live 유지)
  - `cpp/pcl.py`(Python fallback)·`cpp/pcl.cpp`(pointmatcher/PCL 백엔드) 양쪽의
    `remove_outlier`·`density_filter` — 비대칭 삭제 방지를 위해 양쪽 호출자 0 동시 확증
    (`pcl.ICP`/`match`/`downsample`은 live). 부수적으로 orphan된
    `sklearn.neighbors.NearestNeighbors` import(pcl.py)와
    `pcl/filters/radius_outlier_removal.h`/`pcl/point_types.h` include(pcl.cpp) 정리
  - `utils/sonar.py::OculusProperty.deconvolve`/`.polygon`/`.plot`/`.adjust_gamma`
    (`remap`/`configure`/`__str__`은 live 유지)
  - `utils/visualization.py`의 `apply_custom_colormap`·`colorline`·`make_segments`·
    `plot_cov_ellipse`·`ros_colorline`·`plot_polygon` (`make_color_rgba`는 모듈 로드시
    `colors` dict 구성에 6회 호출되는 live로 확증 후 유지; `ros_colorline_trajectory`·
    `ros_constraints`도 live)

### Fixed

- **`ssm.enable`·`nssm.enable`·`fft_localization.enable` 을 다시 켠다**
  (`fix/localization-config-restore`): `fc84203`(IWLO 매핑 수정)이 "설정 파일
  최적화" 한 줄 아래 세 플래그를 껐고 — 로컬라이제이션 결정이 아니었다 — 그 뒤로
  기본 launch 는 ICP·FFT 에 **도달조차 못 했다**(`initialize_sequential_scan_matching`
  이 status=False 를 돌려 odometry factor 만 남김. 계측: 336 키프레임에
  `icp_attempted=0`). 켠 뒤 lawnmower 424 s bag 재생에서 `icp_attempted=216
  icp_converged=216 seed_fft=188 seed_dr=28 reject_pos=32`, 예외 0. 시뮬 odometry 가
  무노이즈 GT 라 ICP 를 켜면 `2D err` 가 0→~1 m 로 커지는 것은 정상이다(이월 큐 N2 종결)
- **`sonar.*` 코드 기본값을 `sonar.yaml` 에 맞춘다** — 이탈점은 `core/slam.py` 하나
  (`sonar_tilt_deg` 10→30, `range_max` 30→40). `SonarMapping2D` 생성자 기본값과
  docstring 도 정합(기준면은 **수평면 아래 각** — 코드가 `cos(tilt)` 를 수평거리 계수로
  쓴다). `test_standalone_node_defaults` 가 `sonar.*` 를 선언하는 다섯 파일 전부를 yaml 에 묶는다
- **소나 틸트 크로스 repo 가드** `test_sonar_tilt_matches_sim_scenario.py`:
  `bluerov2.scn` 의 `<origin rpy>` 를 Rz·Ry·Rx 로 합성해(Stonefish
  `ScenarioParser.cpp:4211`) 센서 +Z 시선의 하향각을 구하고 `sonar_tilt_deg` 와 비교한다.
  .scn 의 roll 은 **연직 기준**(roll 0 = 바로 아래)이라 하향각 = 90° − roll 이고, 현재
  시뮬은 10°(roll 80°) vs config 30° 로 어긋나 있다 — sim `54636c6` 이 roll 을 틸트로
  오독한 결과. 정본을 정할 때(A2)까지 `xfail(strict)`; 맞춰지는 순간 XPASS 로 실패해
  마커 제거를 강제한다. 형제 sim 체크아웃이 없으면 skip
- **WEIGHTED_AVERAGE 3D 갱신법이 목표 확률에서 발산** (`fix/loc-critical`, 런타임 수치
  변경): `cpp/octree_mapping.cpp`가 절대 log-odds를 OctoMap의 **가산** API
  `updateNode(key, float, bool)`에 넘겨, 같은 관측을 반복하면 확률이 유지되지 않고
  clamp 상·하한으로 밀려났다(실측: 강도 255는 0.8176→0.97, 강도 40은 0.1928→0.03).
  인접 IWLO 분기와 같이 delta로 변환. `config/mapping/method_weighted_avg.yaml`이 이
  경로를 직접 선택하므로 사용자 도달 가능
- **순수 Python ICP fallback이 `config/icp.yaml`을 무시** (`cpp/pcl.py`): `loadFromYaml`이
  print 한 줄짜리 no-op이라 libpointmatcher 없는 환경에서 오퍼레이터 튜닝이 조용히
  버려졌다. 이 fallback이 실제 구현하는 두 값(`MaxDistOutlierFilter.maxDist`,
  `CounterTransformationChecker.maxIterationCount`)만 반영하고 나머지 키는 이름을 찍어
  경고한다. `TrimmedDistOutlierFilter.ratio`는 **의도적으로 미반영** — 옮기면 P4a에서
  고친 centroid 편향 버그가 되살아난다
- **ISAM2 예외가 SLAM 노드를 미션 중간에 종료**: `isam.update()`·`marginalCovariance()`가
  무가드였다. 가드하면서 대기 중인 `graph`/`values`를 `finally`로 반드시 비운다 —
  안 비우면 오염된 factor가 큐에 남아 매 키프레임 재push되고 ISAM2가 노드 수명 내내
  영구 실패한다. `marginalCovariance` 실패 시 odometry noise model의 공분산을 대입해
  `verify_pcm`의 2차 크래시를 막는다(`RuntimeError`·`IndexError` 양쪽 포착)
- **NSSM 큐가 루프 없는 구역에서 영구 고착**: `nssm_queue`가 새 성공적 loop closure로만
  trim돼, 차량이 루프 없는 구역으로 나가면 큐가 비지 않아 `update_graph`의 pose 갱신이
  남은 미션 내내 O(N) 전체 이력 경로로 되돌아갔다. `update_graph`에서도 같은 기준으로
  aging
- **잠복 크래시·계약 결함 6건**: `verify_pcm`이 `cov=None` 후보에서 LinAlgError
  (`nssm.cov_samples: 0` 조합) · `MinCovDet.fit`의 `LinAlgError`가 `except ValueError`를
  빠져나가 ROS2 콜백 밖으로 전파 · `fft_localization_node`의 무가드
  `np.frombuffer().reshape()` · `set_adaptive_params`만 입력 검증 부재
  (`adaptive_threshold`는 6곳 나눗셈 분모, `0.0`이면 NaN이 옥트리에 영구 기록) ·
  `RayProcessor` 바인딩의 `py::keep_alive<1,2>()` 누락(non-owning raw pointer의
  use-after-free) · `slam.py:1254`가 `ret2` 대신 `ret`에 대입(형제 정답 `:1158`)
- `ray_processor.cpp`의 intensity 임계 비교를 strict `>`로 통일 — 생성 시점(`:314`)과
  merge 시점(`:179`)이 갈려 있었다. `final_log_odds = free_sum + occupied_sum`이라 합은
  동일하므로 동작 보존적 일관성 정리이고, 나머지 전 경로가 이미 strict `>`
- **2D 맵 좌표에서 `range_min` 항 누락** (`fix/map-and-metrics`, 맵 출력 변경):
  `_accumulate_keyframe_into_map`의 `local_x_raw`가 `(fan_h - yy - 0.5) *
  range_resolution`뿐이라, `polar_to_cartesian_image`가 행을 `x = range_max -
  range_resolution * YY`(`range_resolution = (range_max - range_min)/rows`)로 만드는
  것과 어긋났다. 전 맵 점이 `range_min`만큼(config 0.5 m, tilt 30°에서 수평 0.433 m)
  차량 쪽으로 밀렸다. 위치추정 경로는 이 함수를 타지 않아 pose는 불변
- **`gaussian_sigma_factor`가 프로덕션 3D 경로에 전달되지 않음** (런타임 수치 변경):
  `mapping_3d.py`가 C++ config를 둘 만드는데 `dda_config`에만 넘기고
  `RayProcessorConfig` 빌드에는 빠뜨려, `slam_node`가 실제로 쓰는 경로에서 오퍼레이터
  노브가 무효였다. 기본값도 갈려 있었다 — Python 셋은 모두 2.5인데
  `cpp/ray_processor.h`만 3.0이라 같은 노브가 경로마다 다른 의미였다. 헤더를 2.5로
  통일. 어떤 YAML도 이 키를 설정하지 않으므로 지금까지는 항상 기본값이 이겼다
- **드리프트 지표가 윈도 RMSE를 누적 거리로 나눔** (`slam_accuracy_monitor`):
  분자는 롤링 윈도(최근 500쌍, 매 호출 SE(2) 재정렬)인데 분모 `total_distance`는
  리셋이 없어, 주행이 길어질수록 실제 드리프트와 무관하게 `drift_total`이 0%로
  `acc_total`이 100%로 수렴했다 — 해상시험 평가 목적과 정반대. 분모를 같은 윈도의 GT
  경로 길이로 바꾸고 이름을 `drift_window`·`acc_window`로 정정(로그 전용 지표라 토픽
  영향 없음). `total_distance`는 `dist_total=`로 같은 줄에 유지
- **standalone 노드 소나 기본값이 `sonar.yaml`과 모순**: 두 노드 모두 "defaults match
  sonar.yaml" 주석 아래 `range_max=15.0`·`sonar_tilt_deg=10.0`을 선언했으나 실제
  yaml은 40.0·30.0이다. 파라미터 파일 없이 돌리는 standalone 실행에서 2.7배 range
  오차와 20° 틸트 오차가 조용히 들어갔다. (config 30° vs 시뮬 실물 80° 문제는 별건으로
  계측 후 판단 — 여기서는 기본값을 config에 맞출 뿐)
- `feature_extraction_node`의 feature cloud가 `header.frame_id`는 소나 메시지에서
  가져오면서 `header.stamp`만 벽시계를 썼다. 한 헤더의 두 반쪽이 다른 출처라 시간 동기가
  불가능하다(형제 `fft_localization_node.py:161`이 정답). 현재 구독자 0이라 미래
  소비자를 위한 정합
- `stonefish_slam/__init__.py`의 `__all__`에서 누락된 `pcl` 모듈 추가 —
  `cpp/__init__.py`(cfar·dda_traversal·octree_mapping·ray_processor·pcl 5종 모두 import+export)와
  정합
- `docs/CONVENTIONS.md` §2.9의 `CMakeLists.txt` add_library 줄번호 인용 drift 정정
  (cfar 115→124, dda_traversal 149→158, octree_mapping 205→214, ray_processor 250→259,
  pcl_module 297→306, octree_mapping_core STATIC 183→192 — grep 재측정)
- `docs/RUN_TEST.md`의 `core/slam.py` Subscriber 줄번호 인용 drift 정정 (465-466 → 463-464)

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
