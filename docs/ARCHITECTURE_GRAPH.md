# stonefish_slam — 아키텍처 그래프 맵

> **2026-08-31 graphify로 전면 재측정.** 이전 판의 수치는 은퇴한
> code-review-graph(CRG) 스냅샷이었고 §1~§5를 전부 교체했다. 측정 대상은
> `.graphify/graph.json` — 빌드 커밋 `af8fed6`, 현 브랜치 HEAD 대비 1커밋 뒤이고 그
> 1커밋은 문서·ignore 파일만 건드리므로 코드 수치에는 영향이 없다.
>
> **CRG 수치와 직접 비교하지 말 것 — 세는 대상이 다르다.** CRG는 tree-sitter로 코드만
> 봤고, graphify는 의미 추출 패스를 돌려 산문도 노드로 갖는다. 이 repo는 1,284노드 중
> 코드가 715개이고 나머지 569개가 문서·근거·개념 노드다. 노드 수가 468→1,284로 "늘어난"
> 게 아니라 코퍼스가 달라진 것이다.
>
> **§5는 지표 자체가 바뀌었다.** CRG의 flow criticality는 graphify에 대응물이 없어
> `graphify query`의 BFS 반경으로 대체했다. 묻는 것("어느 진입점이 얼마나 넓게 닿나")은
> 같지만 숫자의 정의가 다르므로 이전 판의 criticality 값과는 비교 불가다.
> CRG의 cohesion 지표도 대응물이 없어 §2.3의 근거를 실제 측정 가능한 것으로 교체했다.
>
> 크로스 repo 관찰(sim↔slam 비교, 토픽 경계)은 워크스페이스
> `.omp/wiki/architecture-2026-08-21-graph-map.md`에 있다. 이 문서는 slam 내부만 다룬다.

---

## 1. 규모

| 항목 | 값 |
|:--|--:|
| 노드 | 1,284 |
| 링크 | 1,893 |
| 하이퍼엣지 | 3 |
| 인덱싱된 파일 | 96 |
| 커뮤니티 | 114 |

노드 종류: code 715 · rationale 359 · document 146 · concept 58 · paper 6.
출처: AST(무료·오프라인) 1,141 · semantic(유료 LLM 패스) 141.
확장자별: `.py` 788 · `.md` 224 · `.h` 117 · `.cpp` 28 · `.yaml` 21 · `.txt` 8.

링크 종류: contains 372 · calls 368 · rationale_for 327 · references 266 · method 214 ·
defines 106 · imports 105 · imports_from 39 · conceptually_related_to 30 · uses 28 ·
inherits 18 · re_exports 12 · semantically_similar_to 3 · cites 2 · shares_data_with 2 ·
indirect_call 1.

**sim과의 대비가 CRG 때와 반대 방향으로 나온다.** CRG는 slam의 엣지/노드 비율이 9.7로
sim(8.3)보다 조밀하다고 봤다. graphify에서는 slam 1.47, sim 1.32로 여전히 slam이 높지만
차이가 훨씬 작다. 이유는 CRG가 CALLS를 3,514개까지 전개했기 때문이다(graphify는 368).
**"한 줄 변경의 파급이 넓다"는 `CONVENTIONS.md` §1의 경고는 이 비율이 아니라 §5의 BFS
반경으로 뒷받침된다** — 그쪽이 훨씬 강한 증거다.

calls 368이 contains 372와 거의 같다는 점은 sim(445 대 767)과 다르다. slam 쪽 그래프는
호출 지도로서의 밀도가 상대적으로 높다.

## 2. 노드 분포

**graphify의 커뮤니티는 디렉토리가 아니다.** 114개 Leiden 클러스터(노드당 평균 11개)라
심볼 단위에 가깝고 패키지 지도로 쓸 수 없다. 아래는 **코드 노드만** 센 것이다(산문 제외).

| 최상위 경로 | 코드 노드 |
|:--|--:|
| `stonefish_slam` | 568 |
| _(source_file 없음 — 외부 심볼)_ | 98 |
| `launch` | 21 |
| `test` | 11 |
| _(repo 루트 파일)_ | 11 |
| `scripts` | 6 |

`stonefish_slam` 568의 내부 분포:

| 하위 디렉토리 | 코드 노드 |
|:--|--:|
| `core/` | 224 |
| `cpp/` | 154 |
| `utils/` | 37 |
| `nodes/` | 32 |

`core/`(39%)와 `cpp/`(27%)가 이 repo의 실질이다. **`nodes/`가 32노드뿐인 것은 ROS
진입점이 core의 얇은 래퍼라는 CLAUDE.md 서술과 정확히 일치한다** — CRG 때와 같은 결론이고,
§3의 레이어 간 엣지가 이를 다시 확인해 준다.

`source_file`이 없는 98개는 이 repo에 정의가 없는 **외부 심볼**이다(rclpy·gtsam·std 등).

### 2.1 core 드릴다운 (코드 노드 224)

| 노드 | 파일 |
|--:|:--|
| 28 | `mapping_3d.py` |
| 22 | `types.py` |
| 22 | `slam.py` |
| 21 | `octree.py` |
| 20 | `localization_fft.py` |
| 19 | `factor_graph.py` |
| 18 | `mapping_2d.py` |
| 16 | `slam_accuracy_monitor.py` |
| 13 | `cfar.py` |
| 11 | `localization.py` |
| 10 | `dead_reckoning.py` |
| 9 | `traj_2d_error_accumulator.py` |
| 7 | `feature_extraction.py` |
| 5 | `odom_tf_bridge.py` |
| 2 | `depth.py` |

3D 매핑(`mapping_3d` + `octree` = 49노드)이 core에서 가장 무겁다. `slam.py`는 22노드로
중간이지만 §4·§5 양쪽에서 지배적이다 — **노드 수가 적다고 가벼운 파일이 아니라는 반례**로
CRG 때 지적한 그대로다.

`types.py` 22노드가 2위라는 점은 새로 눈에 띈다. §4에서 `Keyframe`(in 19)이 허브로
올라오는 것과 같은 얘기다.

### 2.2 cpp 드릴다운 (코드 노드 154)

| 노드 | 파일 |
|--:|:--|
| 53 | `ray_processor.h` |
| 43 | `octree_mapping.h` |
| 21 | `dda_traversal.h` |
| 11 | `ray_processor.cpp` |
| 8 | `pcl.py` |
| 6 | `cfar.cpp` |
| 5 | `pcl.cpp` |
| 3 | `dda_traversal.cpp` |
| 3 | `octree_mapping.cpp` |

**헤더가 소스보다 압도적으로 크다**(`.h` 117노드 대 `.cpp` 28노드). C++ 노드는 대부분
선언에서 나오므로 여기서 "노드 = 로직의 양"으로 읽으면 틀린다. 레이 처리와 옥트리 매핑이
C++ 쪽 무게의 대부분인 것은 CRG 때와 같다.

pybind11 확장 5개(`cfar`·`dda_traversal`·`octree_mapping`·`ray_processor`·`pcl`)의
`.so`는 gitignore되므로 테스트 수집 전에 스테이징이 선행돼야 한다(CLAUDE.md 테스트 절).

### 2.3 utils는 "잡동사니"가 아니다 — 판정 근거가 바뀌었다

이전 판은 CRG의 cohesion 0.069를 근거로 `utils/`가 응집된 유틸이 아니라 잡동사니라고
읽고 정리 후보로 지목했다. **graphify에는 cohesion 지표가 없으므로 그 판정은 재확인
불가다.** 대신 측정 가능한 것으로 바꾸면 그림이 다르게 보인다:

- `utils/` 코드 노드 37개(8파일: `conversions` 10 · `profiler` 7 · `sonar` 6 · `io` 6 ·
  `visualization` 4 · `fusion` 2 · `topics` 1).
- **`core/` → `utils/` 엣지가 81개**(§3). 노드당 2.2개꼴로 들어온다.
- `n2g()`(`utils/conversions.py`)는 **in 16**으로 이 repo에서 가장 많이 의존되는 유틸
  함수다(§4).

즉 utils는 core가 실제로 광범위하게 쓰는 층이다. 정리 후보로 다루기 전에 `n2g()`부터
호출자를 확인할 것 — 시그니처를 바꾸면 core 전역에 파급된다.

## 3. 레이어 간 결합 — 결함이 아니라 설계다

`calls`/`imports`/`imports_from`/`method` 엣지를 디렉토리 층으로 집계한 것이다.

| 출발 | 도착 | 엣지 |
|:--|:--|--:|
| `core` | `utils` | 81 |
| `nodes` | `core` | 27 |
| `test` | `core` | 9 |
| `nodes` | `utils` | 3 |
| `utils` | `test` | 1 |

**CONVENTIONS.md가 규정한 core/utils/nodes 3층 구조가 그대로 측정된다.** core가 utils를
호출하고(81), nodes가 core를 통해 들어간다(27). CRG는 같은 관계를 52/13/1로 봤는데 —
숫자는 다르지만 모양은 동일하다.

주목할 점 두 가지:

- **`nodes` → `utils`가 3건뿐이다.** ROS 래퍼가 유틸을 거의 직접 쓰지 않고 core를 통해서만
  접근한다는 뜻으로, 래퍼가 얇게 유지되고 있다.
- **`core` → `cpp`가 0건이다.** pybind11 `.so` 경계를 그래프가 못 넘는다는 증거다(§6).
  C++ 노드 154개와 파이썬 노드가 같은 그래프에 있으면서 서로 이어지지 않는다.

## 4. 허브 노드 (`graphify god-nodes`)

`god-nodes`는 `_callable` 노드만 센다 — 파일 노드(`ray_processor.cpp` 24 ·
`core/__init__.py` 21 등)는 집계에서 빠진다. in/out은 링크에 기록된 방향이며, 그래프
자체는 `directed=false`다.

| 노드 | 파일 | degree | in / out |
|:--|:--|--:|:--|
| `OctreeMapping` | `cpp/octree_mapping.h` | 43 | 1 / 42 |
| `SonarMapping3D` | `core/mapping_3d.py` | 35 | 9 / 26 |
| `SLAMNode` | `core/slam.py` | 34 | 4 / 30 |
| `RayProcessorConfig` | `cpp/ray_processor.h` | 28 | 5 / 23 |
| `FactorGraph` | `core/factor_graph.py` | 28 | 9 / 19 |
| `FFTLocalizer` | `core/localization_fft.py` | 27 | 8 / 19 |
| `RayProcessor` | `cpp/ray_processor.h` | 27 | 1 / 26 |
| `SonarMapping2D` | `core/mapping_2d.py` | 25 | 9 / 16 |
| `Keyframe` | `core/types.py` | 24 | 19 / 5 |
| `Localization` | `core/localization.py` | 22 | 6 / 16 |
| `HierarchicalOctree` | `core/octree.py` | 20 | 6 / 14 |
| `CFAR` | `core/cfar.py` | 18 | 6 / 12 |
| `n2g()` | `utils/conversions.py` | 18 | 16 / 2 |
| `OculusProperty` | `utils/sonar.py` | 16 | 11 / 5 |

**CRG의 "`slam.py`가 지배한다"는 제목은 더 이상 정확하지 않다.** CRG는 메서드 단위로
세어 상위 10개 중 3개가 `core/slam.py`에서 나왔지만, graphify는 클래스 단위 허브를
집계하고 거기서 `SLAMNode`는 3위(34)다. `slam.py`가 이 repo의 중심이라는 판정 자체는
§5의 BFS 반경(216노드)이 훨씬 강하게 뒷받침한다 — 근거를 그쪽으로 옮겨 읽어야 한다.

**in-degree가 큰 셋이 진짜 위험 지점이다.** 대부분의 허브는 out이 커서 바꿔도 자기만
깨지지만, 다음 셋은 파급이 바깥으로 간다:

- `Keyframe`(in 19) — `core/types.py`의 자료형. SLAM 파이프라인 전체가 이 타입을 주고받는다.
- `n2g()`(in 16) — numpy↔gtsam 변환 유틸. core 전역에서 부른다.
- `OculusProperty`(in 11) — 소나 속성. 센서 파라미터를 만지는 모든 경로가 지난다.

`OctreeMapping`(1/42)·`RayProcessor`(1/26)는 반대로 순수한 조립 지점이다 — C++ 헤더의
선언 묶음이라 out만 크다.

## 5. 진입점 BFS 반경 (`graphify query`, depth 2) — 단일 深 파이프라인

CRG의 flow criticality를 대체하는 측정이다. 각 진입점 식별자에서 깊이 2 BFS로 닿는
노드 수다.

| 진입점 | 닿는 노드 | 전체 대비 |
|:--|--:|--:|
| `SLAMNode` | 216 | 16.8% |
| `SonarMapping3D` | 148 | 11.5% |
| `FFTLocalizer` | 118 | 9.2% |
| `FFTLocalizationNode` | 46 | 3.6% |
| `Mapping3DStandaloneNode` | 41 | 3.2% |
| `DeadReckoningNode` | 35 | 2.7% |
| `Mapping2DStandaloneNode` | 32 | 2.5% |
| `FactorGraph` | 21 | 1.6% |
| `FeatureExtractionNode` | 20 | 1.6% |

**`SLAMNode` 하나가 216노드에 닿는다 — 1,284노드 그래프의 17%다.** 도구를 바꿔도
slam의 성격 판정은 그대로다: **단일 深 파이프라인.** sim은 최대 진입점이 4%에
그친다(그쪽 문서 §4).

회귀 전략도 그대로다:

- slam은 `SLAMNode` 경로를 덮는 회귀 테스트 하나가 넓은 면적을 덮는다.
- 대신 그 경로 안의 변경은 무엇이든 200노드 규모에 영향을 줄 수 있으므로, 여기를 만질 때
  `CONVENTIONS.md` §1의 5단계 게이트를 생략하면 안 된다.

`nodes/`의 standalone 진입점들이 32~46에 그치는 것도 §3의 "얇은 래퍼" 결론과 맞는다.
반면 알고리즘 클래스(`SonarMapping3D` 148 · `FFTLocalizer` 118)가 자기 래퍼보다 3~4배
넓다 — 무게는 core에 있다.

## 6. 이 그래프가 못 보는 것

- **ROS 토픽 결합**: sim이 발행하는 `/{vehicle}/fls/image`·`odometry`·`imu`·`dvl`·
  `pressure` 구독은 엣지가 아니다. `ros2 topic info -v`로 검증할 것.
- **`stonefish_msgs` 의존**: sim repo에 있으므로 이 그래프에 없다. msg/srv가 바뀌면
  slam이 조용히 깨지는데 그래프는 아무것도 보고하지 않는다.
- **Python↔C++ `.so` 경계**: §3에서 측정했듯 `core` → `cpp` 엣지가 **0건**이다.
  C++ 노드 154개가 그래프에 있지만 파이썬 쪽과 연결되지 않으므로, 바인딩 너머의 영향
  범위는 그래프로 답할 수 없다.
- **YAML의 *값***: `.yaml`은 21노드로 그래프에 있지만 19개가 semantic 패스가 만든 개념
  노드다. 즉 그래프가 아는 것은 "이 config가 무엇에 관한 것인가"이지 게인 숫자가 아니다.
  **FFT 튜닝처럼 값을 바꾼 변경은 그래프에 나타나지 않는다.** 이전 판의 "YAML은 노드를
  만들지 않는다"는 서술은 graphify에서 반증됐지만, 실용적 결론(값 비교는 `grep`)은 같다.

## 7. 조회

```bash
cd /workspace/src/stonefish_slam
graphify query "FactorGraph"     # 심볼·개념에서 BFS
graphify god-nodes               # 가장 많이 연결된 허브
graphify update .                # 편집 후 갱신 (AST 패스는 무료·오프라인)
```

**루트(`/workspace`)에서 물으면 안 된다** — meta-repo가 `src/`를 gitignore하므로 루트엔
그래프가 없다. 여러 단어로 물으면 매칭이 안 되니 식별자 하나로 좁힐 것. 신뢰 규칙 정본은
워크스페이스 `.claude/rules/code-graph.md`.
