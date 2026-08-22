import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class Traj2DErrorAccumulator(Node):
    """
    /stonefish_slam/slam/pose 와 /{vehicle}/odometry 를 각각 구독한 뒤,
    stamp가 가장 가까운 쌍을 (|dt| <= sync_slop)로 매칭해서 2D error를 누적(sum) 출력.

    - message_filters의 ApproxSync가 "페어를 못 만드는" 구간에서도
      왜 못 만드는지(수신은 되는지, dt가 큰지)를 확인하기 쉽도록 상태 로그 포함.
    """

    def __init__(self) -> None:
        super().__init__("traj_2d_error_accumulator")

        self.declare_parameter("vehicle_name", "bluerov2")
        self.declare_parameter("slam_pose_topic", "/stonefish_slam/slam/pose")
        self.declare_parameter("ground_truth_topic", "")  # empty => /{vehicle}/odometry

        self.declare_parameter("sync_slop", 0.1)          # seconds
        self.declare_parameter("queue_size", 100)         # buffer size per stream
        self.declare_parameter("log_every_n", 1)
        self.declare_parameter("status_period", 1.0)      # seconds

        vehicle_name = str(self.get_parameter("vehicle_name").value)
        slam_pose_topic = str(self.get_parameter("slam_pose_topic").value)

        ground_truth_topic = str(self.get_parameter("ground_truth_topic").value).strip()
        if not ground_truth_topic:
            ground_truth_topic = f"/{vehicle_name}/odometry"

        self.sync_slop = float(self.get_parameter("sync_slop").value)
        self.queue_size = int(self.get_parameter("queue_size").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)
        self.status_period = float(self.get_parameter("status_period").value)

        if self.sync_slop <= 0.0:
            self.sync_slop = 0.01
        if self.queue_size <= 0:
            self.queue_size = 100
        if self.log_every_n <= 0:
            self.log_every_n = 0
        if self.status_period <= 0.0:
            self.status_period = 1.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Buffers: (t, x, y, frame_id)
        self.slam_buf = deque(maxlen=self.queue_size)
        self.gt_buf = deque(maxlen=self.queue_size)

        # Counters / accumulators
        self.sum_error = 0.0
        self.count_pairs = 0
        self.rx_slam = 0
        self.rx_gt = 0
        self.last_pair_dt = None
        self.warned_frame_mismatch = False

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, slam_pose_topic, self._slam_cb, qos)
        self.create_subscription(Odometry, ground_truth_topic, self._gt_cb, qos)

        # Status timer
        self.create_timer(self.status_period, self._status_timer)

        self.get_logger().info(
            f"Started. slam='{slam_pose_topic}', gt='{ground_truth_topic}', "
            f"sync_slop={self.sync_slop:.3f}s, buf={self.queue_size}, log_every_n={self.log_every_n}"
        )

    def _slam_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self.rx_slam += 1
        t = stamp_to_sec(msg.header.stamp)
        p = msg.pose.pose.position
        self.slam_buf.append((t, float(p.x), float(p.y), msg.header.frame_id))
        self._try_match(from_stream="slam")

    def _gt_cb(self, msg: Odometry) -> None:
        self.rx_gt += 1
        t = stamp_to_sec(msg.header.stamp)
        p = msg.pose.pose.position
        self.gt_buf.append((t, float(p.x), float(p.y), msg.header.frame_id))
        self._try_match(from_stream="gt")

    def _try_match(self, from_stream: str) -> None:
        if not self.slam_buf or not self.gt_buf:
            return

        # 기준은 "방금 들어온 쪽의 최신 샘플"
        if from_stream == "slam":
            t_ref, sx, sy, sframe = self.slam_buf[-1]
            other = self.gt_buf
        else:
            t_ref, gx, gy, gframe = self.gt_buf[-1]
            other = self.slam_buf

        # other 버퍼에서 가장 가까운 stamp 찾기
        best_i = None
        best_dt = None
        for i, (t, x, y, frame) in enumerate(other):
            dt = t - t_ref
            adt = abs(dt)
            if best_dt is None or adt < best_dt:
                best_dt = adt
                best_i = i

        if best_i is None or best_dt is None:
            return

        if best_dt > self.sync_slop:
            # 아직 매칭할 만큼 가깝지 않음
            return

        # 매칭 성립: ref 최신 샘플 + other의 best_i 샘플
        if from_stream == "slam":
            t_ref, sx, sy, sframe = self.slam_buf.pop()  # 최신 slam 사용 후 제거
            # other(gt)에서 best_i까지 popleft해서 best를 얻음
            for _ in range(best_i):
                other.popleft()
            t_o, gx, gy, gframe = other.popleft()
        else:
            t_ref, gx, gy, gframe = self.gt_buf.pop()
            for _ in range(best_i):
                other.popleft()
            t_o, sx, sy, sframe = other.popleft()

        if not self.warned_frame_mismatch and sframe and gframe and sframe != gframe:
            self.get_logger().warn(
                f"Frame mismatch: slam='{sframe}', gt='{gframe}'. Using raw positions without TF."
            )
            self.warned_frame_mismatch = True

        dt = (t_o - t_ref) if from_stream == "slam" else (t_ref - t_o)
        self.last_pair_dt = float(dt)

        err = math.hypot(sx - gx, sy - gy)
        self.sum_error += err
        self.count_pairs += 1

        if self.log_every_n and (self.count_pairs % self.log_every_n == 0):
            avg = self.sum_error / self.count_pairs
            self.get_logger().info(
                f"2D err={err:.3f} m | cum_err={self.sum_error:.3f} m | mean_err={avg:.3f} m | "
                f"pairs={self.count_pairs} | last_dt={self.last_pair_dt:+.3f}s"
            )

    def _status_timer(self) -> None:
        # "왜 안 찍히나" 확인용 상태 로그
        # pair_rate = (self.count_pairs / max(1.0, self.get_clock().now().nanoseconds * 1e-9))  # 대략값
        # self.get_logger().info(
        #     f"[status] rx_slam={self.rx_slam}, rx_gt={self.rx_gt}, pairs={self.count_pairs}, "
        #     f"slam_buf={len(self.slam_buf)}, gt_buf={len(self.gt_buf)}, "
        #     f"last_dt={self.last_pair_dt if self.last_pair_dt is not None else 'None'}"
        # )
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Traj2DErrorAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.count_pairs > 0:
            node.get_logger().info(
                f"Final: cum_err={node.sum_error:.3f} m | mean_err={node.sum_error/node.count_pairs:.3f} m | pairs={node.count_pairs}"
            )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
