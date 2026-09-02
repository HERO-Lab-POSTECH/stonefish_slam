# scripts/

배포 대상이 아닌 **진단·디버깅용 ad-hoc 스크립트** 모음. `nodes/`(배포되는 ROS2 진입점, `CMakeLists.txt`의 `install(PROGRAMS ...)`로 설치)와 구분된다 — 여기 스크립트는 install되지 않고, 특정 이슈를 일회성으로 조사할 때 `python3 scripts/<name>.py`로 직접 실행한다.

| 파일 | 용도 |
|:---|:---|
| `diagnose_center_dip.py` | 소나 3D 매핑의 "중앙 dip" 현상 조사 — bearing 0° vs 65°의 좌표 분포 비교. `core/mapping_3d.py` 디버깅용. |
| `fake_detection_publisher.py` | bag 재생 A/B 용 결정적 검출 주입기 — 소나 프레임의 header 를 그대로 복사한 `vision_msgs/Detection2DArray` 를 되쏜다. 시뮬 씬에 학습 클래스 자산이 없어 진짜 YOLO 로는 semantic 소비 경로를 검증할 수 없기 때문이다. |

새 진단 스크립트를 추가할 때: 배포 노드면 `nodes/`로, 일회성 조사 도구면 여기로.
