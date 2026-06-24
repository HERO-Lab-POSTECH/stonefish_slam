# 시스템 구조

이 페이지는 `stonefish_slam` 패키지의 디렉토리 계층(`core`/`nodes`/`cpp`/`utils`/`test`), `core` 알고리즘 모듈의 역할, C++↔Python(pybind11) 연결 방식, 그리고 외부 의존성을 정리한다. 노드·토픽 세부와 좌표계 정책은 별도 페이지에서 다룬다.

## 개요

`stonefish_slam`은 버전 0.4.0, GPL-3.0 라이선스의 ROS2 패키지다. C++ pybind11 확장을 포함하므로 빌드 시스템은 `setup.py` 대신 `ament_cmake`를 사용한다. 메인테이너는 Seungmin Kim이다.

설계의 핵심은 알고리즘 코드(`core/`)를 ROS에 독립적으로 유지하고, ROS2 진입점(`nodes/`)을 얇은 wrapper로 분리한 점이다. 성능이 중요한 연산(CFAR, ray traversal, ICP 등)은 `cpp/`의 pybind11 확장으로 구현하되, 확장이 빌드되지 않은 환경에서도 동작하도록 순수 Python fallback을 제공한다.

## 디렉토리 트리

```
stonefish_slam/
├── stonefish_slam/
│   ├── core/         # 알고리즘(ROS-independent)
│   │   ├── slam.py            SLAMNode(rclpy.Node) 메인 통합
│   │   ├── factor_graph.py    GTSAM 그래프+루프클로저(NSSM/PCM)
│   │   ├── localization.py    ICP 스캔매칭(SSM/NSSM)
│   │   ├── localization_fft.py FFT 위치추정(선택)
│   │   ├── mapping_2d.py      2D 점유그리드(polar→cartesian, overlay)
│   │   ├── mapping_3d.py      3D OctoMap 확률(ray→voxel, 3 업데이트법)
│   │   ├── feature_extraction.py CFAR 피처추출→점군
│   │   ├── dead_reckoning.py  DeadReckoningNode DVL/IMU 위치추정
│   │   ├── cfar.py            CFAR wrapper(CA/SOCA/GOCA/OS)
│   │   ├── octree.py          레거시 Python Octree(C++로 대체)
│   │   ├── depth.py           압력→깊이 순수 numpy
│   │   └── types.py           Keyframe, STATUS enum
│   ├── nodes/        # ROS2 진입점(얇은 wrapper ~10줄)
│   │   ├── slam_node.py, dead_reckoning_node.py
│   │   ├── mapping_2d_standalone_node.py, mapping_3d_standalone_node.py
│   │   ├── feature_extraction_node.py(P4 신규), fft_localization_node.py(P4 신규)
│   ├── cpp/          # pybind11 C++ 확장 5개 .so
│   │   ├── cfar.cpp           CFAR(CA/SOCA/GOCA/OS)
│   │   ├── dda_traversal.cpp  DDA ray traversal
│   │   ├── octree_mapping.cpp OctoMap 래퍼
│   │   ├── ray_processor.cpp  소나 ray 처리(multi-hit, intensity 가중)
│   │   ├── pcl.cpp            libpointmatcher ICP 래퍼
│   │   ├── pcl.py             ICP fallback(순수 Python numpy/scipy)
│   │   └── __init__.py        try/except ImportError로 확장 감쌈
│   ├── utils/        # topics.py, conversions.py, visualization.py, sonar.py, fusion.py, profiler.py, io.py
│   └── test/         # test_*.py + static_import_gate.py + test_wildcard_gate.py(AST 게이트)
├── launch/   # 9개 launch
├── config/   # 8 + mapping/3 = 11개 YAML
├── rviz/, CMakeLists.txt, package.xml, README, CHANGELOG, CLAUDE.md, P4_FLAGS.md
└── docs/CONVENTIONS.md(SSOT), docs/RUN_TEST.md
```

## 계층 구조와 디렉토리 역할

패키지는 다섯 개의 최상위 디렉토리로 책임을 분리한다.

| 디렉토리 | 역할 |
|---------|------|
| `core/` | ROS에 독립적인 알고리즘 구현. SLAM 통합, factor graph, localization, mapping, feature extraction, dead reckoning 등이 모두 여기에 있다. |
| `nodes/` | ROS2 진입점. 약 10줄 수준의 얇은 wrapper로, `core`의 클래스를 `rclpy`로 띄우는 역할만 한다. |
| `cpp/` | pybind11로 빌드되는 5개 C++ 확장(`.so`)과 각 확장의 순수 Python fallback. |
| `utils/` | 토픽 정의(`topics.py`), 메시지 변환(`conversions.py`), 시각화(`visualization.py`), 소나 속성(`sonar.py`의 `OculusProperty`), 융합(`fusion.py`의 EMA), 프로파일러(`profiler.py`), I/O(`io.py`) 등 공용 유틸. |
| `test/` | `test_*.py` 단위 테스트와 AST 기반 정적 게이트(`static_import_gate.py`, `test_wildcard_gate.py`). |

!!! note "core와 nodes 분리 의도"
    `core/`의 모듈은 ROS 노드 클래스를 포함하더라도(`slam.py`의 `SLAMNode`, `dead_reckoning.py`의 `DeadReckoningNode`) 알고리즘 로직 자체는 분리되어 있고, `nodes/`는 이를 실행 진입점으로 감싸기만 한다. 노드·토픽의 구체적인 정의는 [노드·토픽](nodes-topics.md)을 참조하라.

## core 모듈 역할

`core/`의 각 모듈은 SLAM 파이프라인의 한 단계를 담당한다.

| 모듈 | 역할 |
|------|------|
| `slam.py` | `SLAMNode(rclpy.Node)`. 전체 파이프라인을 통합하는 메인 노드. `slam.py:44-154`에서 파라미터를 총 82회 `declare_parameter`한다. |
| `factor_graph.py` | GTSAM factor graph와 루프클로저(NSSM) 및 PCM(Pairwise Consistency Maximization) 관리. |
| `localization.py` | ICP 스캔매칭. 연속 스캔매칭(SSM)과 루프클로저용 매칭(NSSM)을 수행한다. |
| `localization_fft.py` | FFT 기반 위치추정(선택적). 극좌표·cartesian phase correlation과 subpixel refinement. |
| `mapping_2d.py` | 2D 점유그리드. polar→cartesian 변환 후 overlay 누적. |
| `mapping_3d.py` | 3D OctoMap 확률 격자. ray→voxel 처리와 3가지 업데이트법 지원. |
| `feature_extraction.py` | CFAR 기반 피처 추출 후 점군 생성. |
| `dead_reckoning.py` | `DeadReckoningNode`. DVL/IMU/압력 기반 위치추정(병렬). |
| `cfar.py` | CFAR wrapper(CA/SOCA/GOCA/OS 알고리즘). |
| `octree.py` | 레거시 Python Octree. C++ 확장으로 대체되었다. |
| `depth.py` | 압력→깊이 변환. 순수 numpy. `P_ATM_PA`(`depth.py:8`), `RHO_SEAWATER`(`depth.py:9`), `G`(`depth.py:10`) 상수 정의. |
| `types.py` | `Keyframe` 자료구조와 `STATUS` enum. |

## C++↔Python 연결 (pybind11)

`cpp/`에는 pybind11로 빌드되는 5개의 C++ 확장 모듈이 있다. `CMakeLists.txt:120-339`에 정의되며 C++17로 컴파일된다. Python 버전은 빌드 시 동적으로 감지하고(`CMakeLists.txt:115-118`), 산출된 `.so` 파일은 `local/lib/pythonX.Y/dist-packages/stonefish_slam/`에 설치된다.

| C++ 모듈 | 역할 | Python fallback |
|----------|------|-----------------|
| `cfar.cpp` | CFAR(CA/SOCA/GOCA/OS) 임계값 연산 | — |
| `dda_traversal.cpp` | DDA ray traversal(free space) | — |
| `octree_mapping.cpp` | OctoMap 래퍼 | — |
| `ray_processor.cpp` | 소나 ray 처리(multi-hit, intensity 가중) | — |
| `pcl.cpp` | libpointmatcher ICP 래퍼 | `pcl.py`(순수 Python numpy/scipy) |

### try/except ImportError fallback 패턴

`cpp/__init__.py`는 각 확장의 import를 `try/except ImportError`로 감싼다. 빌드되지 않은 확장은 누락 목록에 모았다가 import 끝에서 한 번 warning으로 알리고(silent pass 금지), 확장이 빌드되지 않은 환경에서는 순수 Python fallback이 동작한다. 특히 ICP는 `pcl.cpp`(libpointmatcher Point-to-Point)가 없으면 `pcl.py`의 Kabsch+SVD 구현으로 대체된다.

```python
try:
    from stonefish_slam import cfar
except ImportError:
    _missing.append("cfar")
```

!!! warning "C++ 변경 시 fallback 동기화"
    C++ 확장의 동작을 바꾸면 대응하는 Python fallback도 함께 맞춰야 한다(`CONVENTIONS §2.9`). 두 경로가 어긋나면 확장 빌드 여부에 따라 결과가 달라진다. v0.4.0에서 Python ICP fallback의 outlier ratio를 `0.8`→`1.0`으로 정정해 perfect overlap을 복원한 것이 한 예다.

!!! tip "확장 미빌드 시 동작"
    확장이 빌드되지 않으면 해당 import는 실패하고, fallback이 있는 기능(ICP→`pcl.py`)은 순수 Python으로 대체되며 fallback이 없는 기능은 비활성화된다. v0.4.0부터는 C++ 확장이 빠진 경우 조용히 넘어가지 않고 warning을 출력한다. 설치된 `.so`는 `install/local/lib/python3.10/dist-packages/stonefish_slam/*.so`에서 확인할 수 있다.

## 의존성

빌드와 실행에 필요한 의존성은 ROS, Python, C++ 세 갈래로 나뉜다.

### ROS

`rclpy`/`rclcpp`, `std_msgs`/`sensor_msgs`/`geometry_msgs`/`nav_msgs`/`visualization_msgs`, `octomap_msgs`, `stonefish_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `cv_bridge`, `message_filters`, `sensor_msgs_py`, `tf_transformations`.

### Python

`numpy`, `scipy`, `opencv-python`, `yaml`, `scikit-learn`, `shapely`, `matplotlib`, 그리고 `gtsam`.

### C++

`pybind11`, `Eigen3`, `octomap`, 그리고 선택적(QUIET) 의존성인 `libpointmatcher`와 `PCL`. `libpointmatcher`/`PCL`이 없으면 ICP는 Python fallback으로 동작한다.

| 핵심 라이브러리 | 용도 |
|----------------|------|
| `gtsam` | factor graph 최적화(ISAM2). |
| `libpointmatcher` | C++ ICP 스캔매칭(Point-to-Point). 선택적. |
| `octomap` | 3D 확률 점유 격자. |

!!! warning "gtsam 설치 주의"
    `apt`로 설치하는 `ros-humble-gtsam`은 C++ 라이브러리만 제공하므로 Python에서 `import gtsam`이 불가능하다. Python 바인딩이 필요하므로 `pip install gtsam`을 권장한다.

## 관련 페이지

- [노드·토픽](nodes-topics.md) — 노드 목록, 구독/발행 토픽, 서비스 정의.
- [좌표계(frames)](frames.md) — `world_ned` 전역 frame과 `odom`→`base_link` ENU 로컬 TF 정책.
