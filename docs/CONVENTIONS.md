# stonefish_slam — 코딩 컨벤션 & 작업 프로세스

> **작업 전 필독.** 이 문서는 `stonefish_slam`(단일 ROS2 패키지: SLAM 코어 + pybind11 C++ 확장)에서
> 코드를 추가·수정·리팩토링할 때 따라야 할 두 가지를 규정한다: **(1) 코딩 컨벤션**(이 repo의
> *실제 코드*에서 귀납한 규칙, 추측 아님) + **(2) 작업 프로세스 게이트**(구현 전 분석·조사 의무).
>
> 이 규칙은 P3(2026-06)부터 적용한다. 컨벤션은 이 repo에 *이미 존재하는 다수 패턴*에서 근거를
> 인용해 도출했다 — 새 규칙을 발명한 것이 아니라 집을 짓던 방식을 명문화한 것이다.

---

## 1. 작업 프로세스 게이트 (먼저 읽을 것)

P1·P2에서 테스트·CI 안전망을 깔았지만 본 코드는 거의 손대지 않았다. P3부터는 **실제 동작을
바꾸는 변경**이 시작되므로, 모든 비자명한 작업은 아래 5단계 게이트를 **순서대로** 통과한다.
"간단해 보여서 바로 고침"이 이 프로젝트에서 가장 위험한 안티패턴이다. SLAM 코어는 gtsam
factor graph·C++ 확장과 얽혀 있어 한 줄 변경의 파급이 넓다.

| 단계 | 무엇을 | 산출물(증거를 남길 것) |
|:---|:---|:---|
| ① 정독 + 의존성 추적 | 대상 코드를 줄 단위로 읽고, 그것을 import/호출하는 곳·그것이 의존하는 곳(C++ `.so` 경계 포함)을 모두 추적 | "누가 이걸 쓰나" 목록 (grep/LSP 근거) |
| ② 자료 조사 | ROS2 Humble 관행·gtsam API·numpy/scipy/sklearn·pybind11·SLAM 도메인 관행을 공식 문서로 확인 (기억 금지) | 인용 URL/문서 근거 |
| ③ 설계 | 변경 범위·인터페이스·하위호환·Python/C++ 경계 영향·테스트 전략을 먼저 글로 적음 | 설계 노트 (.sp/ 또는 작업 브리프) |
| ④ 구현 | 설계대로 최소 변경. 기존 스타일을 따름(§2) | diff |
| ⑤ 검토 | 별도 패스(자가승인 금지)로 정합성·테스트·회귀 확인 | 테스트 결과 + 리뷰 |

**게이트 운용 원칙**
- **추측 금지·증거 우선.** 파일 경로·함수 시그니처·동작은 *읽어서* 확인한다. "아마 이럴 것"으로 코드를 쓰지 않는다.
- **단계별 산출물을 남긴다.** 분석·조사 결과를 `.sp/`(scratch)나 작업 브리프에 기록해, 검토자가 *왜* 그렇게 고쳤는지 추적할 수 있게 한다.
- **authoring과 review는 분리한다.** 같은 컨텍스트에서 자기 작업을 승인하지 않는다. 검토는 별도 에이전트/패스로 한다.
- **C++ 경계는 양쪽을 본다.** `cpp/` 바인딩이나 `.so` 모듈 동작을 바꾸면 Python wrapper·순수파이썬 fallback(§2.9)·CMake 빌드를 함께 확인한다.
- **규모에 맞게 조절.** 3–4줄 자명한 수정에 5단계 전부를 강요하지 않는다. 로직·수치·인터페이스·다중파일·C++ 경계를 건드리면 전체 게이트, 오타·주석은 ④⑤만.
- **3-Strike / 15분 규칙.** 같은 접근이 3번 실패하거나 한 문제에 15분 막히면 방법을 바꾼다.

---

## 2. 코딩 컨벤션 (이 repo의 실제 패턴)

### 2.0 명명·구조 표준 — 외부 근거와 대조

이 절은 §2.1~2.3의 명명·구조 규칙이 *왜* 맞는지를 권위 있는 외부 표준으로 뒷받침한다.
표준은 **강제(normative — ROS RMW/RCL 코드나 REP가 실제로 거부)**와 **관행(practice — 커뮤니티 norm,
린터 수준)**으로 나뉜다. slam은 단일 패키지 + pybind11 C++ 확장이라 sim과 빌드·import가 다르며, 어긋나는 지점은 명시한다.

| 항목 | 외부 표준 | 등급 | 이 repo 현황 | 출처 |
|:---|:---|:---|:---|:---|
| 패키지명 | 소문자 영숫자+`_`, 알파벳 시작, 2자↑, `__`·하이픈·대문자 금지 | **강제** | `stonefish_slam` 준수 | [REP 144](https://ros.org/reps/rep-0144.html) |
| 빌드 시스템 | 순수 Python은 ament_python(setup.py) | 사실상 강제(툴체인) | ⚠️ **setup.py 없음** — `CMakeLists.txt`(ament_cmake+Python). C++ pybind11 확장 때문에 의도적 선택. 표준 ament_python과 다르나 C++/Python 혼합 패키지의 정당한 형태 | [design.ros2.org/ament](https://design.ros2.org/articles/ament.html) |
| 내부 서브디렉토리 | `core/`·`nodes/`·`utils/`·`cpp/` 세부 레이아웃 | **신뢰할 출처 없음**(공식 규정 없음) | 자유롭게 분리 — 표준 위반 아님 | (none) |
| 노드명 문자 | 영숫자+`_`만, 숫자시작·`/` 금지 | **강제(RMW)** | 문자 규칙 준수 | [rmw validate_node_name.c](https://github.com/ros2/rmw/blob/master/rmw/src/validate_node_name.c) |
| 노드명 고유성 | 동시 실행 노드는 고유 이름 필요 | **강제** | ⚠️ **위반**(아래 발견 참조) | RMW |
| 토픽/서비스명 문자 | `[a-zA-Z0-9_/~{}]`, `//`·`__` 금지, `~/foo`(≠`~foo`) | **강제(RCL)** | 준수 | [design.ros2.org/topic names](https://design.ros2.org/articles/topic_and_service_names.html) |
| 토픽 상수 | — | — | 중앙 `utils/topics.py`에 `UPPER_SNAKE_CASE` 상수로 집약(좋은 패턴, 권장) | (이 repo 관행) |
| 파라미터 dot notation | `sonar.horizontal_fov`·`offset.x` 중첩 표기 | 관행(정식 char spec 없음) | dot notation 사용 | [design.ros2.org/parameters](https://design.ros2.org/articles/ros_parameters.html) |
| 함수·변수 | lower_snake_case / 클래스 `CapWords` / 상수 `UPPER_WITH_UNDER` / 비공개 `_` | **강제(PEP 8)** | 대체로 준수(비공개 `_` 접두는 비일관 — 새 코드는 일관 적용) | [PEP 8](https://peps.python.org/pep-0008/) |
| import — wildcard | "Wildcard imports (`from <module> import *`) should be avoided" | PEP 8 권고 / Google 암묵 금지 | ⚠️ **위반**(일부 모듈 wildcard, 레거시 — 새 코드 금지) | [PEP 8](https://peps.python.org/pep-0008/) / [Google](https://google.github.io/styleguide/pyguide.html) |
| docstring | 모든 public 모듈/함수/클래스 + Google Args/Returns/Raises | PEP 257 권고 | 커버리지 불균일 — 새 코드는 채움 | [PEP 257](https://peps.python.org/pep-0257/) |

**좌표계 — REP 103/105 (★중요, sim과 다름)**
- ROS 표준(REP 103)은 world에 **ENU**, body에 **FLU**를 1차로, NED는 `_ned` 접미사 secondary frame으로 허용한다. REP 105는 표준 프레임 `earth→map→odom→base_link`를 규정(`map`은 Z-up=ENU 정렬).
- **전역 프레임 = `world_ned`(NED)로 통일**(P4d 결정, 2026-06-24). 근거: Stonefish 시뮬레이터가 전역 프레임을 NED로 발행하며(`core/slam.py:904` 주석 "Simulator provides world_ned → base_link_frd directly"), slam의 모든 출력 메시지가 그 NED 전역 위에서 동작하므로 전역 frame_id를 `world_ned`로 맞추는 것이 sim과 정합한다. 이전엔 SLAMNode가 같은 노드 안에서 `"map"`(2D·pose·traj·cloud)과 `'world_ned'`(3D octomap)를 **혼용**했으나, P4d에서 전역 frame_id 9곳(`core/slam.py` single/multi-ROV 8곳 + `utils/visualization.py:161`)을 `world_ned`/`{rov}_world_ned`로 통일했다. **이는 의도적 REP-103/105 비순응이다** — 수중 로봇 도메인 표준이 NED이고 sim이 NED를 강제하므로 `map`(ENU)을 전역에 쓰지 않기로 결정. TF는 모두 identity(회전 0)라 좌표 변환은 없고 frame_id **이름만** 정합한 것이다.
- **로봇-로컬 TF 체인 `odom→base_link`는 REP-105 표준대로 유지**(`core/dead_reckoning.py:299/305/318/319`). 전역만 NED로 통일하고 dead_reckoning의 ENU TF 체인과 `child_frame_id`(`base_link`/`{rov}_base_link`, body 프레임)는 보존한다 — "전역만 통일" 결정의 경계.
- ⚠️ **sim 통합 시**: sim도 전역 `world_ned`(NED)라 이제 전역 프레임은 양 repo가 일치한다. 단 dead_reckoning의 로컬 ENU TF 체인을 NED 전역 트리에 붙일 때는 여전히 NED↔ENU 변환 경계가 `launch`의 `world_ned→map` static TF(현재 identity)에 위치한다. 출처: [REP 103](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst), [REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst).

**이 repo에서 발견된 명명 위반(P4_FLAGS 후보, 고칠 때까지 새 코드는 답습 금지)**
- ⚠️ **CRITICAL — 노드명 3중 충돌**: `nodes/mapping_2d_standalone_node.py:28`와 `nodes/mapping_3d_standalone_node.py:30`이 둘 다 `super().__init__('slam_node')`로 초기화되며, `core/slam.py:41`(`Node.__init__(self, 'slam_node')`)도 `'slam_node'`다. 셋 중 둘 이상이 동시에 뜨면 ROS2 고유 노드명 요구를 위반한다. 표준 standalone 노드는 `mapping_2d_node`·`mapping_3d_node`처럼 고유 이름을 써야 한다.

---

### 2.1 파일·디렉토리 구조
- 모든 Python 파일은 **snake_case**. 노드 실행 진입점은 **`*_node.py`** 접미사(`nodes/slam_node.py`, `dead_reckoning_node.py`, `kalman_node.py`).
- **단일 패키지 레이아웃** `stonefish_slam/`의 서브모듈로 관심사 분리:
  - `core/` — 알고리즘 구현(`factor_graph.py`, `mapping_2d.py`, `feature_extraction.py`, `slam.py`). 대부분 ROS Node가 아니다.
  - `nodes/` — ROS2 실행 진입점(얇은 wrapper, `core`의 `main()`을 import).
  - `utils/` — 헬퍼(`conversions.py`, `profiler.py`, `visualization.py`, `topics.py`).
  - `cpp/` — C++ 바인딩 wrapper + 순수파이썬 fallback.
  - `test/` — 테스트.
- ⚠️ 의도된 예외: `core/`의 ROS Node 진입 클래스(`SLAMNode`=`core/slam.py:35`, `KalmanNode`=`core/kalman.py:24`, `DeadReckoningNode`=`core/dead_reckoning.py:22`)는 알고리즘 계층인 `core/`에 두고, `nodes/*_node.py`(각 ~10줄)는 그 `main()`을 import하는 얇은 진입점이다. ROS2 라이프사이클과 알고리즘을 분리하기 위함이며, 세 노드 모두 동일 패턴이다 — 이 분리를 깨지 않는다.
- launch는 `*.launch.py`, config는 `config/` 하위 계층, C++ 빌드는 루트 `CMakeLists.txt`.

### 2.2 import
- 순서: **(1) stdlib → (2) 서드파티(numpy, scipy, cv2, rclpy, gtsam, sklearn) → (3) 로컬 절대 import** `from stonefish_slam.X import ...`.
- ⚠️ sim과 다름: **slam은 절대 import가 표준**이다(sim은 상대 import). 새 코드는 이 절대 경로 방식을 따른다.
- C++ `.so` 모듈은 `__init__.py`에서 **`try/except ImportError`**로 감싼다(빌드 안 된 환경에서도 import 가능하게). 근거: `stonefish_slam/__init__.py:2-22`, `cpp/__init__.py`.
- ⚠️ 기존 일부 모듈이 wildcard import(`from utils.conversions import *`)를 쓰지만 이는 PEP 8 위반이다. **새 코드는 wildcard import를 쓰지 않는다**(grandfathered).

### 2.3 명명
- 변수·함수·파라미터: **snake_case**. 비공개는 `_`/`__` 접두.
- 클래스: **PascalCase**(`Keyframe`, `FactorGraph`, `SLAMNode`, `OculusProperty`). 토픽 등 모듈 상수: **UPPER_SNAKE_CASE**(`IMU_TOPIC`, `SLAM_NS` — `utils/topics.py`).
- ROS 파라미터는 **dot notation 계층**(`offset.x`, `ssm.enable`, `nssm.enable`). 근거: `core/kalman.py:66-68`(`declare_parameter('offset.x'/'offset.y'/'offset.z')`).
- **단위는 변수명에 일관되게 넣지 않는다**(`_m`, `_rad`, `_deg` 접미 없음). 단위는 YAML·코드 주석으로 문서화. (이는 "권장"이지 절대 규칙은 아님 — repo 전반에 일관 적용되어 있지 않다.)

### 2.4 docstring·타입힌트
- **Google-style docstring**(`Args:`, `Returns:`, `Raises:`, `Examples:`). 모듈·주요 클래스 docstring은 대체로 존재, 함수는 공개 API 위주. 언어는 **영어**. 커버리지는 균일하지 않다 — 새 코드는 공개 API에 docstring을 채운다.
- **타입힌트는 중간 수준**(강제 아님). 코어 알고리즘·`conversions.py`·콜백 시그니처(ROS 메시지 타입)에 사용. 새 공개 함수에는 붙이는 것을 권장.

### 2.5 에러 처리·로깅
- 미지원 변환 등은 명시적 `NotImplementedError`(서술적 메시지, `Raises:` docstring). 선택적 C++ import는 `try/except ImportError`. `main()` 루프는 `try/except KeyboardInterrupt`.
- ⚠️ 일부 critical 구역(예: `core/mapping_2d.py`)에 catch-all `except Exception`이 있다. 이는 견고성을 위한 기존 패턴이나, **새 코드에서 catch-all은 최소화**하고 잡은 예외는 로깅한다.
- **로깅 — 컨텍스트로 갈린다(의도된 분기)**:
  - ROS2 노드 내부 → **`self.get_logger()`**(`.info()`/`.debug()`/`.warning()`). 근거: `core/kalman.py:137`(`self.get_logger().info(...)`; :130은 `TransformBroadcaster`).
  - 비-Node 코어 모듈 → **`logging.getLogger('<name>')`**(`core/mapping_2d.py:128` — `logging.getLogger('SonarMapping2D')`).
  - 타이밍/디버그 → `CodeTimer`가 `print('[DEBUG] ...')` 사용(`utils/io.py`). 새 코드의 일반 디버그는 `logger.debug()` 선호.

### 2.6 ROS2 노드 패턴
- 노드는 `rclpy.node.Node` 상속, `super().__init__('node_name')`.
- 파라미터는 `declare_parameter()` → `get_parameter().value`로 init에서 읽음. QoS 프로파일(`ReliabilityPolicy`, `HistoryPolicy`) 사용, `TransformBroadcaster`로 TF 발행. 근거: `core/slam.py:88-99`, `core/kalman.py:38-65`.
- launch에서 여러 YAML config를 로드하고 런타임 오버라이드를 param dict로 노드에 전달.

### 2.7 config·상수·단위·좌표계
- **YAML 기반 config**가 `config/`에 모듈별로(`sonar.yaml`, `slam.yaml`, `factor_graph.yaml`, `localization.yaml`, `mapping.yaml`, `feature.yaml`, `icp.yaml`). 계층 네임스페이스(`ros__parameters`, `sonar.*`, `ssm.*`).
- 센서 상수는 클래스 속성(예: `OculusProperty.OCULUS_VERTICAL_APERTURE = {1: np.deg2rad(20), ...}` — `utils/sonar.py`)이나 YAML.
- ⚠️ frame_id(전역 `'world_ned'`, 로컬 `'odom'`·`'base_link'`)가 노드 코드에 **하드코딩**되어 있다(sim의 `world_ned` 중앙화와 대조). 새 코드는 가능하면 상수/파라미터로 중앙화하는 것을 권장하되, 기존 값과 일관성을 우선한다. (전역 frame_id 통일 경위는 §2.0 좌표계 항목 참조.)

### 2.8 테스트 (P2에서 정립)
- 테스트는 `stonefish_slam/test/`에 `test_*.py`, 함수는 `test_*`.
- **import-time 오염(rclpy/gtsam)을 피하려고 루트 `conftest.py`의 `load_module` fixture**(`importlib.util.spec_from_file_location` + `sys.modules` 정리)로 `.py` 파일을 직접 로드한다. 근거: `conftest.py:1-22`, `test/test_fusion.py:1-7`.
- `pytest.ini`: `testpaths = stonefish_slam`, vendored pybind11 디렉토리는 discovery에서 배제(이중 격리).
- sklearn 등 선택 의존성은 `pytest.importorskip('sklearn')`로 가드. 알려진 실패는 `@pytest.mark.xfail(strict=True)` + 사유.
- 기대값은 **수학적 정답**으로 작성한다. 코드 출력과 불일치하면 코드 버그로 보고 `P4_FLAGS.md`에 기록(테스트를 코드에 맞추지 않는다). 현재 등록된 것: ICP 수렴(근본원인 `cpp/pcl.py`의 `T_delta` float32 다운캐스트)·fusion `observation_count` 미사용·kalman 제외사유.
- 테스트 추가 시 **live 코드 0줄 변경** 원칙(P2). 코드 변경이 필요한 테스트(kalman·guidance는 module-top import가 rclpy/gtsam을 끌어와 import 자체가 크래시)는 P3 추출 리팩토링 후 다룬다.
- CI: `.github/workflows/ci.yml`(Python 3.10, transforms3d 미포함).

**P3 동작보존 안전망 (rclpy/gtsam/cv2 부재 환경 대응)**
- 이 환경엔 rclpy·gtsam·cv2가 없어(numpy/scipy만 있음) 대부분의 `core/` 모듈은 import-time에 크래시한다(`conversions.py:4 import gtsam`이 이를 import하는 모든 모듈로 전파). 따라서 "모듈 실제 로드" import-smoke가 불가능하고, P3 동작보존은 **검증 가능성에 따라 두 갈래**로 증명한다:
  - **numpy-only 모듈(`core/octree.py`)** → `conftest.py`의 `load_module` fixture로 직접 path-load해 **수학적 골든 마스터**를 잠근다(예: `test_octree.py`의 adaptive 0.3 config 경로 = 0.45). 실측 verifiable.
  - **rclpy/gtsam/cv2 오염 모듈(slam·kalman·dead_reckoning·mapping_3d·feature_extraction·localization_fft 등)** → path-load 불가 → **정적 AST 게이트**(`test/static_import_gate.py`)로만 검증한다: `py_compile` + repo-내부 import target-set diff(변환 전후 동결) + wildcard dead/live 분류(consumer 직접 바인딩을 차감해 wildcard로만 오는 의존 심볼 집계) + 모듈 top-level 부작용 0. `test_wildcard_gate.py`가 17곳 wildcard의 분류(10 dead / 7 live)와 live의 명시 심볼 집합을 골든 마스터로 동결한다.
- ⚠️ **한계(정직하게)**: AST 게이트는 경로·타겟집합·심볼집합 불변을 *정적으로* 증명할 뿐, 런타임 심볼 binding(동명이클래스, `__init__` re-export 섀도잉, 메서드→함수 추출의 외부 동적 호출자)은 증명하지 못한다. `getattr`/문자열 디스패치는 별도 grep으로 확인한다. 런타임 검증(`colcon build` + `ros2 launch` 스모크)은 **P4 sign-off**로 미룬다.

### 2.9 C++ / pybind11 확장
- `CMakeLists.txt`가 확장별로 별도 **pybind11 MODULE 타겟**을 정의(`cfar`, `dda_traversal`, `octree_mapping`, `ray_processor`, `pcl_module`). `.so` 네이밍은 `SET_TARGET_PROPERTIES`로 지정. 재사용 코어는 static lib로 빼서 다른 모듈이 링크(`octree_mapping_core`, `CMakeLists.txt:183` STATIC). 근거: `CMakeLists.txt`의 MODULE 타겟 — cfar(115)·dda_traversal(149)·octree_mapping(205)·ray_processor(250)·pcl_module(297).
- 설치 경로: `local/lib/python3.X/dist-packages/stonefish_slam/`.
- C++/Python 경계: `PYBIND11_MODULE()`로 노출. **각 C++ 모듈은 `__init__.py`에서 `try/except ImportError`로 감싸고**, 가능한 경우 **순수파이썬 fallback**을 제공한다(예: `cpp/pcl.py`가 `remove_outlier`/`density_filter`/`downsample`의 순수 Python 구현 제공). C++ 동작을 바꾸면 fallback도 동기화한다.

---

## 3. 적용 메모
- 이 repo의 SSOT는 이 문서다. `CLAUDE.md`가 생기면 "작업 전 `docs/CONVENTIONS.md` 필독" 포인터 1줄을 둔다(CLAUDE.md만 자동 로드되므로).
- 라이선스는 **GPL-3.0**(루트 `LICENSE` 기준). 메인테이너: Seungmin Kim <luckkim123@gmail.com>.
- `P4_FLAGS.md`에 모인 수치/알고리즘 이슈는 P4에서 소화한다. **P4 진행(2026-06-24)**: kalman 제거(legacy-dead)·octree leaf·DDA corner·ICP outlier·dead_reckoning depth(NED)·robust Cauchy loop closure 연결·PCM 수치안정·`SLAM_callback_integrated`→snake_case·전역 frame_id `world_ned` 통일 — **완료**. 노드명 'slam_node' 공유는 측정 결과 **충돌 아님으로 반증**(combined launch가 PushRosNamespace로 분리, `P4_FLAGS.md` 참조). 잔여: god-method 분해(P4e), `polar_to_cartesian` 통합, PascalCase 파라미터, `__all__` 추가, free-region 반전(live-sim 필요). **P3는 동작 보존이 가능한 것만 처리했다** — 순수 이동·dead 제거·dedup·import 정리·메타데이터·문서·테스트 추가. 런타임 동작(수치 출력·토픽 그래프·노드명 네임스페이스)을 바꾸는 것은 전부 P4로 격리했다.
- P3 안전망 도구: `test/static_import_gate.py`(AST 정적 게이트), `test/test_wildcard_gate.py`(wildcard 분류 동결), `test/test_octree.py`(numpy 실측 골든). §2.8 'P3 동작보존 안전망' 참조.
