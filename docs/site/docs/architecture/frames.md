# 좌표계와 TF

이 페이지는 stonefish_slam의 좌표계 정책(`CONVENTIONS.md` §2.0, P4d 결정)을 설명한다. 전역 출력은 모두 `world_ned`(NED)로 통일하고, 로컬 TF 체인(`odom`→`base_link`)은 frame **이름**만 REP-105에서 빌렸을 뿐 **데이터는 NED(z-down)**이며, 두 경계 사이의 TF는 회전 없는 identity로 둔다(2026-08-21 소비자 전수 추적으로 이전의 "로컬 ENU 유지" 기술이 반증됨 — 근거는 아래).

## 정책 요약

| 구분 | frame | 컨벤션 | 적용 대상 |
|-----|-------|--------|----------|
| 전역 frame_id | `world_ned` | NED | SLAM 출력 전부(pose, odom, traj, map 등) |
| 로컬 TF 체인 | `odom`→`base_link` | 이름만 REP-105, 데이터는 NED(z-down) | `dead_reckoning_node` |
| 경계 변환 | — | identity(회전 없음) | 전역·로컬 모두 NED — 이름 층위만 상이 |

## 전역 frame_id = `world_ned` (NED)

SLAM 노드가 발행하는 모든 토픽의 `frame_id`는 `world_ned`로 통일되어 있다(`CONVENTIONS.md` §2.0, P4d). 이 문자열은 `slam.py`의 각 발행 메서드(pose/odom/constraint/traj/cloud)에 `"world_ned"`로 직접 하드코딩되어 있다.

통일의 근거는 sim 측에 있다. Stonefish sim이 전역 좌표를 NED로 발행하므로, SLAM 출력을 동일한 NED 전역으로 맞춰야 sim의 ground truth(`/{v}/odometry`, RELIABLE QoS, `world_ned` 기준)와 일관되게 비교·정렬된다.

이는 P4d에서 정정된 결정이다. 이전 코드는 일부 출력에 ENU 기준의 `"map"` frame을 혼용하고 있었고, P4d에서 이 혼용을 제거하여 `frame_id`를 `world_ned` 한 가지로 통일했다(v0.4.0 변경: `frame_id world_ned` 통일 8+1곳).

!!! warning "REP-103/REP-105 의도적 비순응"
    ROS 표준 REP-103은 전역 frame을 ENU(`x` 동쪽, `z` 위)로, REP-105는 `map`/`odom`/`base_link` 명명을 권장한다. stonefish_slam의 전역 `world_ned`(NED, `z` 아래)는 이 권장을 **의도적으로 따르지 않는다**. 이유는 sim이 NED 전역을 발행하기 때문이며, sim과의 정합이 ROS 표준 순응보다 우선한다. 표준을 기대하고 외부 도구나 노드를 연결할 때는 이 비순응을 전제로 frame을 다루어야 한다.

## 로컬 TF 체인 — 이름만 REP-105, 데이터는 NED

`dead_reckoning_node`가 발행하는 로컬 TF `odom`→`base_link`와 출력 토픽 `/dead_reck/odom`은 frame **이름**을 REP-105(`odom`/`base_link`)에서 빌렸을 뿐, 싣는 데이터는 **NED(z-down)**다. 압력 센서의 깊이는 양수-아래(`depth.py`)로 계산되어 부호 반전 없이 z에 그대로 실린다(`dead_reckoning.py`).

이는 2026-08-21 소비자 전수 추적으로 확정된 사실이다: `/dead_reck/*` 토픽의 repo 내 구독자는 0이고, 체인의 모든 TF는 identity이며, NED↔ENU 변환은 코드 어디에도 존재하지 않는다 — 체인 전체가 NED로 자기정합한다. 이전 문서의 "로컬 ENU 유지" 기술은 이 추적으로 반증되었고, 세 전제(양수-아래 깊이·무반전·구독자 0)는 `test/test_dead_reckoning_depth_frame.py`가 고정한다. DR을 SLAM 입력으로 재배선하는 순간 이 질문은 다시 열린다.

`dead_reckoning.py`가 직접 브로드캐스트하는 TF는 `odom`→`base_link` 하나뿐이다(`dead_reckoning.py:318-319`). 전역 `world_ned`와 로컬 체인을 잇는 변환은 launch의 static TF(`world_ned`→`{v}_map`, identity)로 별도 발행된다.

## TF는 identity (회전 없음)

경계 TF는 identity로 둔다. 실제 회전 변환 없이 `frame_id` 이름만 정합시킨다(`CONVENTIONS.md` §2.0).

!!! note "왜 identity가 정당한가"
    전역도 로컬도 데이터가 모두 NED이므로 경계에 좌표축 회전이 필요 없다. 좌표계 층위의 차이는 존재하지 않고, frame **이름** 층위의 차이(REP-105식 명명 vs `world_ned`)만 있으므로 이름 정합만으로 체인이 연결된다.

## TF 체인

전역 `world_ned`에서 차량별(`{v}`) frame으로 가는 변환은 launch의 static TF가, 로컬 `odom`→`base_link`는 `dead_reckoning_node`가 각각 발행한다.

```mermaid
flowchart TD
    A["world_ned (전역 NED)"] -->|"launch static TF (identity)"| B["{v}_map"]
    C["odom (이름만 REP-105, 데이터 NED)"] -->|"dead_reckoning_node broadcast"| D["base_link"]
```

## RViz 설정

RViz에서 Fixed Frame은 `world_ned`로 설정한다. 이 frame을 기준으로 pose(공분산 ellipsoid), constraint(loop closure 빨강 line), traj(초록), octomap(청록), `map_2d_image`가 표시된다.

## 관련 하드코딩 상수

frame 문자열이 노드 코드에 직접 정의되어 있다(`CONVENTIONS.md` §2.0).

| 상수 | 값 | 위치 |
|-----|-----|------|
| 전역 frame | `"world_ned"` | `slam.py`의 발행 메서드(예: `:826`, `:893`, `:961`, `:1011`) |
| 로컬 odom frame | `"odom"` | `dead_reckoning.py:299`, `:318` |
| 로컬 body frame | `"base_link"` | `dead_reckoning.py:305`, `:319` |
