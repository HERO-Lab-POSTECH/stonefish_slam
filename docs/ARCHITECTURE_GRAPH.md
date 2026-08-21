# stonefish_slam — 아키텍처 그래프 맵

> code-review-graph 측정 스냅샷. **커밋 `34a0d0a` (브랜치 `main`)** 기준이며
> `head_matches_build: true` 상태에서 조회했다.
>
> 크로스 repo 관찰(sim↔slam 비교, 토픽 경계)은 워크스페이스
> `.omp/wiki/architecture-2026-08-21-graph-map.md`에 있다. 이 문서는 slam 내부만 다룬다.

---

## 1. 규모

| 항목 | 값 |
|:--|--:|
| 파일 | 58 |
| 노드 | 468 |
| 엣지 | 4,558 |
| 언어 | python, cpp, c |

노드 구성: Function 336 · File 58 · Class 37 · Test 37.
엣지 구성: CALLS 3,514 · CONTAINS 421 · IMPORTS_FROM 340 · TESTED_BY 233 · INHERITS 15 · REFERENCES 35.

엣지/노드 비율 9.7로 sim(8.3)보다 조밀하다. CONVENTIONS.md §1이 경고하는
"한 줄 변경의 파급이 넓다"가 측정으로 뒷받침된다.

## 2. 커뮤니티 (= 디렉토리)

커뮤니티는 Leiden 클러스터가 아니라 **디렉토리 기반**이다(`description` 필드로 확인).

| 커뮤니티 | 디렉토리 | 노드 | cohesion | 주 언어 |
|:--|:--|--:|--:|:--|
| core-map | `stonefish_slam/core` | 172 | 0.175 | python |
| cpp-compute | `stonefish_slam/cpp` | 90 | 0.112 | cpp |
| test-identity | `stonefish_slam/test` | 58 | 0.254 | python |
| utils-plot | `stonefish_slam/utils` | 42 | 0.069 | python |
| nodes-node | `stonefish_slam/nodes` | 19 | 0.049 | python |
| launch-launch | `launch` | 10 | 0.000 | python |
| test-module | `test` | 10 | 0.053 | python |
| scripts-callback | `scripts` | 5 | 0.053 | python |
| stonefish-slam-load | `conftest` | 4 | 0.143 | python |

`core/` 172노드(37%)와 `cpp/` 90노드(19%)가 이 repo의 실질이다. `nodes/`가 19노드뿐인
것은 **ROS 진입점이 core의 얇은 래퍼**라는 CLAUDE.md 서술과 정확히 일치한다.

### 2.1 core 드릴다운 (172노드)

| 노드 | 파일 |
|--:|:--|
| 27 | `core/mapping_3d.py` |
| 23 | `core/octree.py` |
| 19 | `core/localization_fft.py` |
| 18 | `core/factor_graph.py` |
| 16 | `core/types.py` |
| 15 | `core/slam.py` |
| 14 | `core/mapping_2d.py` |
| 13 | `core/cfar.py` |

3D 매핑(`mapping_3d` + `octree` = 50노드)이 core에서 가장 무겁다. `slam.py`는 15노드로
작지만 뒤에 나오듯 **허브·flow 양쪽에서 지배적**이다 — 노드 수가 적다고 가벼운 파일이
아니라는 반례다.

### 2.2 cpp 드릴다운 (90노드)

| 노드 | 파일 |
|--:|:--|
| 26 | `cpp/ray_processor.cpp` |
| 25 | `cpp/octree_mapping.cpp` |

pybind11 확장 5개(`cfar`·`dda_traversal`·`octree_mapping`·`ray_processor`·`pcl`) 중
레이 처리와 옥트리 매핑 둘이 C++ 쪽 무게의 절반 이상이다. 이 `.so`들은 gitignore되므로
테스트 수집 전에 스테이징이 선행돼야 한다(CLAUDE.md 테스트 절 참조).

### 2.3 utils cohesion 0.069는 실제 신호다

`utils/` 42노드에 cohesion 0.069. sim의 C++ 브리지(0.007)와 달리 여기는 **파이썬이고
같은 파서가 잘 잡는 언어**다. 즉 이 낮은 값은 측정의 얕음이 아니라 "응집된 유틸이 아니라
잡동사니"라는 실제 신호로 읽을 수 있다. 정리 후보를 찾는다면 여기다.

## 3. 결합 경고 2건 — 결함이 아니라 레이어다

그래프가 경고를 2건 올린다.

| 출발 | 도착 | 엣지 | 종류 |
|:--|:--|--:|:--|
| core-map | utils-plot | 52 | CALLS |
| core-map | nodes-node | 13 | CALLS |
| nodes-node | utils-plot | 1 | CALLS |

**둘 다 설계대로다.** core가 utils를 호출하고(52), core와 nodes가 얽히는(13) 것은
CONVENTIONS.md가 규정한 core/utils/nodes 3층 구조가 동작한다는 증거에 가깝다.
커뮤니티가 디렉토리 기반이므로 이 "경고"는 아키텍처 레이어 위반이 아니라
**디렉토리 간 호출 횟수**를 말할 뿐이다 — 결함 신호로 읽으면 오독이다.

주목할 점은 오히려 `nodes → utils`가 1건뿐이라는 것이다. ROS 래퍼가 유틸을 거의 직접
쓰지 않고 core를 통해서만 접근한다는 뜻으로, 래퍼가 얇게 유지되고 있다.

## 4. 허브 노드 — `slam.py`가 지배한다

| 노드 | 파일 | out | 총 degree |
|:--|:--|--:|--:|
| `SLAMNode.init_node` | `core/slam.py` | 146 | 148 |
| `Mapping3DStandaloneNode.__init__` | `nodes/mapping_3d_standalone_node.py` | 105 | 107 |
| `SLAMNode.__init__` | `core/slam.py` | 103 | 104 |
| `SLAMNode.slam_callback_integrated` | `core/slam.py` | 87 | 88 |
| `SonarMapping2D._process_keyframes_to_map` | `core/mapping_2d.py` | 77 | 80 |
| `CenterDipDiagnostic.analyze_callback` | `scripts/diagnose_center_dip.py` | 73 | 74 |
| `DeadReckoningNode.init_node_params` | `core/dead_reckoning.py` | 50 | 52 |
| `FeatureExtraction.init_params` | `core/feature_extraction.py` | 48 | 50 |
| `FFTLocalizer.estimate_rotation` | `core/localization_fft.py` | 47 | 49 |
| `SonarMapping3D.__init__` | `core/mapping_3d.py` | 48 | 49 |

상위 10개 중 **3개가 `core/slam.py`** 하나에서 나온다(148·104·88). 이 파일은 노드 15개로
작지만 그래프상 이 repo의 중심이다.

**읽는 법**: sim과 마찬가지로 out_degree가 크고 in_degree는 1~3이다 — 많이 호출당하는
공용 함수가 아니라 **많이 호출하는 조립 지점**이다. `init_node`(out 146)는 초기화가
그만큼 많은 것을 엮는다는 뜻이고, 이는 P4에서 god-method를 분해했음에도 초기화 경로는
여전히 넓다는 신호로 볼 수 있다.

`scripts/diagnose_center_dip.py`의 out 73은 진단 스크립트가 core를 광범위하게 끌어쓴다는
뜻이다. 배포 경로가 아니므로 위험은 아니지만, core API가 바뀌면 조용히 깨질 자리다.

## 5. 실행 흐름 — 단일 深 파이프라인

| flow | criticality | 노드 |
|:--|--:|--:|
| `slam_callback_integrated` | 0.73 | 96 |
| `__init__` | 0.68 | 10 |
| `sonar_callback` | 0.63 | 21 |
| `sync_callback` | 0.62 | 11 |
| `callback` | 0.61 | 9 |
| `sync_callback` | 0.58 | 26 |
| `get_states` | 0.52 | 7 |
| `sonar_callback` | 0.52 | 3 |
| `subroutine` | 0.51 | 4 |
| `__init__` | 0.51 | 6 |

`slam_callback_integrated` 하나가 **96노드를 관통**한다. 468노드 repo에서 flow 하나가
20%를 덮는다는 뜻이고, 이것이 slam의 성격을 규정한다: **단일 深 파이프라인**.

sim(최고 0.48, flow당 6~12노드)과 정반대이므로 회귀 전략이 다르다:
- slam은 `slam_callback_integrated` 회귀 테스트 하나가 넓은 면적을 덮는다.
- 대신 그 콜백 안의 변경은 무엇이든 96노드에 영향을 줄 수 있으므로, 여기를 만질 때
  CONVENTIONS.md §1의 5단계 게이트를 생략하면 안 된다.

## 6. 이 그래프가 못 보는 것

- **ROS 토픽 결합**: sim이 발행하는 `/{vehicle}/fls/image`·`odometry`·`imu`·`dvl`·
  `pressure` 구독은 엣지가 아니다. `ros2 topic info -v`로 검증할 것.
- **`stonefish_msgs` 의존**: sim repo에 있으므로 이 그래프에 없다. msg/srv가 바뀌면
  slam이 조용히 깨지는데 그래프는 아무것도 보고하지 않는다.
- **`config/slam.yaml` 등 YAML**: 노드를 만들지 않는다. FFT 튜닝처럼 config로 동작이
  바뀌는 변경은 그래프가 항상 빈 답을 준다.
- **Python↔C++ `.so` 경계**: pybind11 바인딩 너머의 호출은 CALLS 엣지로 이어지지 않는다.

## 7. 조회

```
repo_root: /workspace/src/stonefish_slam
```

생략하면 오류가 아니라 `status: "ok"`에 0건이 돌아온다. 신뢰 규칙 정본은 워크스페이스
`.claude/rules/code-review-graph.md`.
