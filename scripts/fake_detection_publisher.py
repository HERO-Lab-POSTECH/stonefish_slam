#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Seungmin Kim
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""소나 프레임마다 결정적인 가짜 검출을 되쏘는 주입기 (bag 재생 검증용).

**왜 있는가.** 시뮬 씬에 학습된 클래스(sofa) 자산이 없어서 진짜 YOLO 검출로는
"검출 결과가 위치 인식에 쓰였다"를 보일 수가 없다. 검출기의 성능은 그 질문의
전제가 아니므로, 검출을 상수로 고정해 **소비 경로만** 먼저 검증한다.

구독한 이미지의 header 를 그대로 복사해 발행하므로, 그 프레임이 키프레임이
되기만 하면 `semantic.max_stamp_delta` 안에서 반드시 매칭된다.

**판정 기준을 잘못 읽지 말 것**: SLAM 은 `filter.skip` 을 통과하고 키프레임으로
뽑힌 프레임만 큐에 넣는다. 주입기는 모든 이미지에 검출을 내므로 나머지는 짝지을
키프레임이 없어 **정상적으로** `det_expired` 가 된다 — 그 값이 큰 것은 버그가
아니다. 큐 버그의 신호는 `det_missing`(키프레임은 생겼는데 검출이 끝내 안 옴)
이 0 이 아닌 것과, `landmark_factors_added` 가 `det_matched` 보다 작은 것이다.

bbox 는 프레임에서 가장 밝은 영역에 놓는다. 랜드마크 factor 는 bbox 중심만
쓰므로 위치와 무관하게 생기지만, 3D 복셀 라벨 경로는 bbox 안에 실제 반사가
있어야 라벨이 붙기 때문이다.

```bash
# 터미널 A — SLAM (semantic on). launch 인자다 — `--ros-args -p` 가 아니다.
ros2 launch stonefish_slam slam.launch.py vehicle_name:=bluerov2 \
    use_sim_time:=true rviz:=false semantic:=true
# 터미널 B — 주입기
python3 scripts/fake_detection_publisher.py --ros-args -p use_sim_time:=true
# 터미널 C — bag
ros2 bag play data/bags/2026-09-02-bluerov2-lawnmower-tilt10 --clock
```
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class FakeDetectionPublisher(Node):
    """이미지 한 장당 가짜 검출 한 건을 같은 stamp 로 발행한다."""

    def __init__(self):
        """Class constructor. Declares parameters and wires the topics."""
        super().__init__('fake_detection_publisher')

        self.declare_parameter('image_topic', '/bluerov2/fls/image')
        self.declare_parameter('detection_topic', '/sonar_yolo/detections')
        self.declare_parameter('class_id', 0)
        self.declare_parameter('score', 0.9)
        self.declare_parameter('box_size', [64, 64])   # [width_px, height_px]
        # 1 로 두면 SLAM 이 `filter.skip`(기본 5) 로 건너뛴 프레임의 검출은
        # 짝을 못 찾고 `det_expired` 로 만료된다 — 정상이고, 이 값을 올려
        # 줄이려 하면 안 된다. 두 카운터가 각자의 첫 수신 프레임부터 세므로
        # 위상이 맞는다는 보장이 없어, `filter.skip` 과 같은 값을 넣으면
        # 오히려 어긋난 프레임만 골라 쏘게 될 수 있다. 전 프레임 발행이
        # 정합을 보장하는 유일한 설정이다.
        self.declare_parameter('every_n', 1)           # N 프레임마다 한 번

        self.class_id = int(self.get_parameter('class_id').value)
        self.score = float(self.get_parameter('score').value)
        self.box_w, self.box_h = [int(v) for v in self.get_parameter('box_size').value]
        self.every_n = max(1, int(self.get_parameter('every_n').value))
        self.frame_count = 0
        self.published = 0

        self.bridge = CvBridge()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        image_topic = self.get_parameter('image_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        self.pub = self.create_publisher(Detection2DArray, detection_topic, 10)
        self.sub = self.create_subscription(Image, image_topic, self.cb_image, qos)
        self.get_logger().info(
            f"fake detections: {image_topic} -> {detection_topic} "
            f"(class_id={self.class_id}, box={self.box_w}x{self.box_h}, "
            f"every_n={self.every_n})")

    def cb_image(self, msg: Image) -> None:
        """Publishes one fake detection carrying this image's header.

        Args:
            msg (sensor_msgs.msg.Image): the polar sonar frame.
        """
        self.frame_count += 1
        if self.frame_count % self.every_n != 0:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:                       # noqa: BLE001 - 진단 도구
            self.get_logger().error(f"image conversion failed: {e}")
            return

        cx, cy = self._brightest_center(img)
        det = Detection2D()
        det.header = msg.header
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(self.class_id)
        hyp.hypothesis.score = self.score
        det.results = [hyp]
        det.bbox.center.position.x = float(cx)
        det.bbox.center.position.y = float(cy)
        det.bbox.size_x = float(self.box_w)
        det.bbox.size_y = float(self.box_h)

        out = Detection2DArray()
        out.header = msg.header
        out.detections = [det]
        self.pub.publish(out)
        # 발행 수를 남긴다: SLAM 의 `det_received` 와 대조해야 "검출이
        # 안 왔다"와 "보냈는데 큐에서 떨어졌다"가 갈린다.
        self.published += 1
        self.get_logger().info(f"published={self.published}",
                               throttle_duration_sec=10.0)

    def _brightest_center(self, img: np.ndarray):
        """Centre of the box-sized window with the most energy.

        Args:
            img (np.ndarray): mono8 polar image (rows = range bins).

        Returns:
            tuple: `(col, row)` pixel centre, clamped so the box stays inside
            the image.
        """
        smooth = cv2.blur(img, (self.box_w, self.box_h))
        row, col = np.unravel_index(int(np.argmax(smooth)), smooth.shape)
        half_w, half_h = self.box_w // 2, self.box_h // 2
        col = int(np.clip(col, half_w, img.shape[1] - half_w - 1))
        row = int(np.clip(row, half_h, img.shape[0] - half_h - 1))
        return col, row


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = FakeDetectionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
