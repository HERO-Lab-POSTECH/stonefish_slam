# 동작 테스트 실행 가이드

stonefish_sim(시뮬레이터) + stonefish_slam(SLAM)을 함께 띄워 전체 파이프라인을 테스트하는 절차.
모든 launch 파일을 코드에서 측정해 정리함(2026-06-24). 빌드 검증 완료(3패키지 Finished, 새 노드 2개 install 확인).

## 0. 매 터미널 선행 (필수)

모든 터미널에서 먼저 환경을 source 해야 한다. **안 하면 `ament_package`/`rclpy`조차 import 안 돼 launch가 실패**한다(빌드 첫 시도 실패의 원인이 바로 이것).

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash   # merge-install 레이아웃
```

## 1. 빌드 (이미 완료, 재빌드 시 참고)

```bash
cd /workspace
source /opt/ros/humble/setup.bash
colcon build --merge-install --packages-up-to stonefish_slam stonefish_ros2
```

- `--merge-install` 필수(기존 install이 merged 레이아웃).
- ⚠️ **merge-install 함정**: 소스를 삭제·이름변경했으면 옛 install 산출물이 자동 제거 안 됨. 깨끗하게 하려면 `rm -rf build install` 후 재빌드.
- 의존: pip `gtsam`(apt `ros-humble-gtsam`은 C++만이라 Python import 안 됨), pybind11, shapely. ICP C++은 `ros-humble-libpointmatcher`(없으면 순수 Python fallback). 상세 [[stonefish-build-deps]].

## 2. 토픽 계약 (왜 이 순서로 띄우나)

slam은 sim이 발행하는 두 토픽을 **2-way 시간동기**로 구독한다(`core/slam.py:463-464`). 따라서 **sim이 먼저, slam이 나중**.

| 토픽 | 발행 | 구독 | 비고 |
|---|---|---|---|
| `/bluerov2/fls/image` | sim (FLS 소나) | slam, 새 노드 2개 | BEST_EFFORT QoS |
| `/bluerov2/odometry` | sim | slam | 2-way sync 짝 |

`vehicle_name` 파라미터로 네임스페이스가 정해진다(양쪽 기본 `bluerov2`).

## 3. 전체 SLAM 테스트 (메인 경로)

**터미널 A — 시뮬레이터 + 차량:**
```bash
ros2 launch stonefish_ros2 bluerov2.launch.py
```
→ Stonefish 시뮬레이터(GPU 렌더, RViz와 별개 창) + thruster manager + odom 발행.
헤드리스면 `gpu:=false`(nogpu 빌드 사용). `start_thruster_manager:=false`로 thruster 끌 수 있음.

**터미널 B — SLAM:**
```bash
ros2 launch stonefish_slam slam.launch.py
```
기본값: `mode:=slam`, `vehicle_name:=bluerov2`, `enable_2d_mapping:=true`, `enable_3d_mapping:=true`, `rviz:=true`.
→ slam_node + world_ned→map static TF + RViz(rviz=true).

**(선택) 터미널 C — 차량을 움직여 데이터 생성:**
정지 상태면 소나/odom이 변하지 않아 맵이 안 쌓인다. 궤적/제어로 움직여야 한다.
```bash
ros2 launch stonefish_control control.launch.py        # 제어기
# 또는 trajectory manager (path.launch.py)
```

## 4. 모드별 변형 (slam.launch.py 인자)

```bash
# localization-only (맵 안 쌓고 위치추정만)
ros2 launch stonefish_slam slam.launch.py mode:=localization

# mapping-only (SSM/NSSM 끄고 매핑만, SLAM 콜백은 거침)
ros2 launch stonefish_slam slam.launch.py mode:=mapping

# RViz 끄고 (헤드리스/원격)
ros2 launch stonefish_slam slam.launch.py rviz:=false

# 2D만
ros2 launch stonefish_slam slam.launch.py enable_3d_mapping:=false
```

## 4b. semantic(검출 라벨) A/B — bag 재생

검출 파이프라인은 시뮬을 다시 띄우지 않고 bag 재생으로 판정한다. 씬에 학습된
클래스(sofa) 자산이 없으므로 `scripts/fake_detection_publisher.py` 가 소나 프레임
header 를 그대로 복사한 결정적 검출을 되쏜다 — 검출기 성능은 "검출 결과가 위치
인식에 쓰였는가"의 전제가 아니다.

```bash
# --- A: off (기준선) ---
ros2 launch stonefish_slam slam.launch.py vehicle_name:=bluerov2 \
    use_sim_time:=true rviz:=false
ros2 bag play data/bags/2026-09-02-bluerov2-lawnmower-tilt10 --clock

# --- B: on ---
ros2 launch stonefish_slam slam.launch.py vehicle_name:=bluerov2 \
    use_sim_time:=true rviz:=false semantic:=true
python3 scripts/fake_detection_publisher.py --ros-args \
    -p use_sim_time:=true                    # every_n 기본 1 = 전 프레임 (아래 참조)
ros2 bag play data/bags/2026-09-02-bluerov2-lawnmower-tilt10 --clock
```

판정은 `[INSTR] semantic` 한 줄로 한다.

| 지표 | 기대 | 왜 |
|:--|:--|:--|
| `landmark_factors_added` | > 0, 그리고 `det_matched` 와 **같다** | "검출이 위치 추정에 쓰였다"의 유일한 증거. bbox 중심을 측정값으로 쓰므로 검출 1건 ⇒ factor 1건이 구성상 보장된다 |
| `pose_key_mismatch` | 0 | > 0 이면 늦은 검출이 엉뚱한 pose 에 붙을 뻔한 것을 막은 것 |
| `det_expired` | **크다(정상)** | 주입기는 모든 이미지에 검출을 내지만 SLAM 은 `filter.skip` 을 통과해 키프레임이 된 프레임만 큐에 넣는다. 큐 버그의 신호가 아니다 |
| `det_missing` | 작을수록 좋다 | 키프레임은 생겼는데 검출이 워터마크까지 안 왔다. **`every_n` 을 `filter.skip` 과 맞춰서 줄이려 하지 말 것** — 주입기와 SLAM 이 각자의 첫 수신 프레임부터 세므로 위상이 맞는다는 보장이 없고, 한 프레임만 어긋나면 매칭이 0 이 된다. 전 프레임 발행(`every_n:=1`)이 정합을 보장하는 유일한 설정이다 |
| `voxels_labeled` | > 0 | 3D 역투영이 돌았다 |
| `[FactorGraph] ISAM2 update failed` | 0건 | |

off 런은 semantic 이전과 같아야 한다. 확인 3종:

```bash
ros2 topic list | grep -E "cloud_3d|sonar_yolo"          # off 면 아무것도 없어야 한다
ros2 topic echo /stonefish_slam/slam/cloud --once --field fields   # off: 4필드(x,y,z,i)
grep -c "INSTR. semantic" <slam 로그>                     # off: 0
```

⚠️ **절대 개수는 런마다 흔들린다.** 소나 구독이 BEST_EFFORT(depth 20)라 재생
속도와 CPU 부하에 따라 프레임이 떨어진다 — `icp_attempted` 를 off/on 사이에서
같은 값으로 기대하지 말고 **수렴률(`icp_rate`)**과 위 카운터들로 판정한다.
`--rate` 를 올리면 이 흔들림이 커진다.

## 5. 독립 노드 테스트 (이번에 추가한 것 + 기존 standalone)

이들은 **SLAM 파이프라인을 우회**하고 sim의 raw 토픽만으로 각 모듈을 격리 검증한다. sim(터미널 A)이 떠 있어야 한다.

```bash
# [신규] feature 추출만 → /feature_extraction/points (PointCloud2)
ros2 launch stonefish_slam feature_extraction_standalone.launch.py

# [신규] FFT 위치추정만 → /fft_localization/transform (PoseWithCovarianceStamped)
ros2 launch stonefish_slam fft_localization_standalone.launch.py

# 2D 매핑만 (odom 원본 pose 사용, loop closure 없음)
ros2 launch stonefish_slam mapping_2d_standalone.launch.py

# 3D 매핑만
ros2 launch stonefish_slam mapping_3d_standalone.launch.py

# 2D+3D 동시 (각각 mapping_2d/mapping_3d 네임스페이스로 분리)
ros2 launch stonefish_slam mapping_combined_standalone.launch.py

# dead reckoning만
ros2 launch stonefish_slam dead_reckoning.launch.py
```

신규 두 노드는 모두 **발행만** 한다(slam_node가 구독 안 함, 통합 경로 불변). `vehicle_name:=X`로 차량 변경 가능.

## 6. 동작 확인 명령 (별도 터미널)

```bash
ros2 node list                                  # 노드 떴나
ros2 topic list                                 # 토픽 그래프
ros2 topic hz /bluerov2/fls/image               # sim이 소나 발행 중인가
ros2 topic echo /feature_extraction/points --once   # 신규 feature 노드 출력
ros2 topic echo /fft_localization/transform --once  # 신규 fft 노드 출력
ros2 topic hz /bluerov2/odometry                # odom 흐르나
```

⚠️ **소나가 BEST_EFFORT라** `ros2 topic echo`가 기본 RELIABLE이면 안 보일 수 있다 — `--qos-reliability best_effort` 추가.

## 7. 흔한 실패 진단

| 증상 | 원인 | 처치 |
|---|---|---|
| `package not found` | install source 안 함 | `source /workspace/install/setup.bash` |
| `ament_package` ModuleNotFoundError (빌드) | ROS source 안 함 | `source /opt/ros/humble/setup.bash` 후 빌드 |
| 새 노드가 콜백 안 받음 | QoS 불일치 | 신규 노드는 BEST_EFFORT로 수정됨(해결). 직접 echo 시 `--qos-reliability best_effort` |
| 맵이 안 쌓임 | 차량 정지 | 제어/궤적으로 이동(터미널 C) |
| RViz 창 안 뜸 | DISPLAY 없음/rviz2 미설치 | `rviz:=false` 또는 `ros-humble-rviz2` 설치 |
