import math
from collections import deque
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Path

import tf2_ros


# -------------------------
# Math helpers
# -------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def umeyama_se2(src_xy: np.ndarray, dst_xy: np.ndarray):
    n = src_xy.shape[0]
    if n == 0:
        return np.eye(2, dtype=np.float64), np.zeros(2, dtype=np.float64)
    if n == 1:
        return np.eye(2, dtype=np.float64), (dst_xy[0] - src_xy[0]).astype(np.float64)

    mu_src = src_xy.mean(axis=0)
    mu_dst = dst_xy.mean(axis=0)

    src_c = src_xy - mu_src
    dst_c = dst_xy - mu_dst

    cov = (dst_c.T @ src_c) / float(n)

    U, _, Vt = np.linalg.svd(cov)
    D = np.eye(2, dtype=np.float64)

    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[1, 1] = -1.0

    R = U @ D @ Vt
    t = mu_dst - (R @ mu_src)
    return R, t


def point_to_polyline_min_dist_2d(p_xy: np.ndarray, A: np.ndarray, V: np.ndarray, VV: np.ndarray) -> float:
    w = p_xy.reshape(1, 2) - A
    t = (w[:, 0] * V[:, 0] + w[:, 1] * V[:, 1]) / VV
    t = np.clip(t, 0.0, 1.0)
    proj = A + V * t.reshape(-1, 1)
    d = p_xy.reshape(1, 2) - proj
    dist2 = d[:, 0] * d[:, 0] + d[:, 1] * d[:, 1]
    return float(np.sqrt(np.min(dist2)))


class TrajPathGtAteMonitor(Node):
    def __init__(self):
        super().__init__("traj_path_gt_ate_monitor")

        # Topics
        self.declare_parameter("traj_topic", "/stonefish_slam/slam/traj")
        self.declare_parameter("path_topic", "/path_generator_node/path")

        # GT TF frame
        self.declare_parameter("vehicle_name", "bluerov2")
        self.declare_parameter("gt_frame", "")

        # Thresholds
        self.declare_parameter("r_gt_m", 1.0)
        self.declare_parameter("r_path_m", 0.5)

        # ✅ 추가: 0.24m 임계치
        self.declare_parameter("r_gt_024_m", 0.24)

        # Logging
        self.declare_parameter("log_every_n", 1)
        self.declare_parameter("keyframe_stride", 1)

        # TF
        self.declare_parameter("tf_timeout_sec", 0.05)

        # ATE window
        self.declare_parameter("ate_window_size", 500)
        self.declare_parameter("ate_min_samples", 20)
        self.declare_parameter("distance_epsilon", 1e-6)
        self.declare_parameter("ate_use_se2_align", True)

        self.traj_topic = str(self.get_parameter("traj_topic").value)
        self.path_topic = str(self.get_parameter("path_topic").value)

        vehicle = str(self.get_parameter("vehicle_name").value)
        gt_frame = str(self.get_parameter("gt_frame").value).strip()
        if not gt_frame:
            gt_frame = f"{vehicle}/base_link_frd"
        self.gt_frame = gt_frame

        self.r_gt = float(self.get_parameter("r_gt_m").value)
        if self.r_gt <= 0.0:
            self.r_gt = 1.0

        self.r_path = float(self.get_parameter("r_path_m").value)
        if self.r_path <= 0.0:
            self.r_path = 1.0

        # ✅ 0.24m 임계치
        self.r_gt_024 = float(self.get_parameter("r_gt_024_m").value)
        if self.r_gt_024 <= 0.0:
            self.r_gt_024 = 0.24

        self.log_every_n = int(self.get_parameter("log_every_n").value)
        if self.log_every_n <= 0:
            self.log_every_n = 0

        self.keyframe_stride = int(self.get_parameter("keyframe_stride").value)
        if self.keyframe_stride <= 0:
            self.keyframe_stride = 1

        self.tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_sec").value))

        self.ate_window_size = int(self.get_parameter("ate_window_size").value)
        if self.ate_window_size < 0:
            self.ate_window_size = 0
        self.ate_min_samples = int(self.get_parameter("ate_min_samples").value)
        if self.ate_min_samples < 2:
            self.ate_min_samples = 2

        self.distance_epsilon = float(self.get_parameter("distance_epsilon").value)
        if self.distance_epsilon <= 0.0:
            self.distance_epsilon = 1e-9

        self.ate_use_se2_align = bool(self.get_parameter("ate_use_se2_align").value)

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Path cache
        self.path_frame: Optional[str] = None
        self.path_xy: Optional[np.ndarray] = None
        self.seg_A: Optional[np.ndarray] = None
        self.seg_V: Optional[np.ndarray] = None
        self.seg_VV: Optional[np.ndarray] = None

        # GT stats
        self.gt_samples = 0
        self.gt_within = 0
        # ✅ 0.24m 이내 카운터
        self.gt_within_024 = 0

        self.last_err_gt = float("nan")

        # ATE buffers
        maxlen = self.ate_window_size if self.ate_window_size > 0 else None
        self.est_buf = deque(maxlen=maxlen)
        self.gt_buf = deque(maxlen=maxlen)

        # traveled distance
        self.total_distance = 0.0
        self.last_gt_xy_global: Optional[Tuple[float, float]] = None

        # Counters
        self.traj_msgs = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        self.create_subscription(Path, self.path_topic, self._path_cb, qos)
        self.create_subscription(PointCloud2, self.traj_topic, self._traj_cb, qos)

    def _path_cb(self, msg: Path):
        if not msg.poses:
            self.path_frame = None
            self.path_xy = None
            self.seg_A = self.seg_V = self.seg_VV = None
            return

        self.path_frame = msg.header.frame_id.strip() if msg.header.frame_id else ""
        pts = []
        for ps in msg.poses:
            p = ps.pose.position
            pts.append((float(p.x), float(p.y)))
        path_xy = np.asarray(pts, dtype=np.float64)
        self.path_xy = path_xy

        if path_xy.shape[0] >= 2:
            A = path_xy[:-1, :]
            B = path_xy[1:, :]
            V = B - A
            VV = np.sum(V * V, axis=1)
            VV = np.maximum(VV, 1e-12)
            self.seg_A, self.seg_V, self.seg_VV = A, V, VV
        else:
            self.seg_A = self.seg_V = self.seg_VV = None

    def _traj_cb(self, msg: PointCloud2):
        self.traj_msgs += 1

        traj_frame = msg.header.frame_id.strip() if msg.header.frame_id else ""
        if not traj_frame:
            return

        pts_xyz, last_xyz = self._extract_points_and_last(msg, stride=self.keyframe_stride)
        if last_xyz is None:
            return

        gt_xyz = self._lookup_gt_in_traj_frame(msg, traj_frame)
        if gt_xyz is None:
            return

        # ---- (A) GT instant error + Acc@r_gt ----
        err_gt = self._compute_err(last_xyz, gt_xyz, dim="xy")
        self.last_err_gt = err_gt
        self.gt_samples += 1

        if err_gt <= self.r_gt:
            self.gt_within += 1
        acc_gt = 100.0 * (self.gt_within / self.gt_samples)

        # ✅ 0.24m 이내 비율
        if err_gt <= self.r_gt_024:
            self.gt_within_024 += 1
        acc_gt_024 = 100.0 * (self.gt_within_024 / self.gt_samples)

        # ---- (B) ATE update ----
        est_xy = (float(last_xyz[0]), float(last_xyz[1]))
        gt_xy = (float(gt_xyz[0]), float(gt_xyz[1]))
        self._accumulate_total_distance(gt_xy)
        self.est_buf.append(est_xy)
        self.gt_buf.append(gt_xy)

        ate_rmse, drift_total, acc_total = self._compute_ate_and_accuracy()

        # ---- (C) Path metric ----
        mean_err_path, acc_path = self._compute_path_metric(traj_frame, msg, pts_xyz)

        if self.log_every_n and (self.traj_msgs % self.log_every_n == 0):
            self.get_logger().info(
                f"err_gt={err_gt:.3f}m | "
                f"Acc_gt@{self.r_gt:.3f}m={acc_gt:.2f}% | "
                f"Acc_gt@{self.r_gt_024:.3f}m={acc_gt_024:.2f}% | "
                f"ATE_RMSE={ate_rmse:.3f}m | drift_total={drift_total:.3f}% | acc_total={acc_total:.3f}% | "
                f"mean_err_path={mean_err_path:.3f}m | Acc_path@{self.r_path:.3f}m={acc_path:.2f}%"
            )

    def _lookup_gt_in_traj_frame(self, msg: PointCloud2, traj_frame: str) -> Optional[Tuple[float, float, float]]:
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            tf_time = rclpy.time.Time()
        else:
            tf_time = rclpy.time.Time.from_msg(stamp)

        try:
            tf = self.tf_buffer.lookup_transform(
                traj_frame,
                self.gt_frame,
                tf_time,
                timeout=self.tf_timeout,
            )
            gx = float(tf.transform.translation.x)
            gy = float(tf.transform.translation.y)
            gz = float(tf.transform.translation.z)
            return (gx, gy, gz)
        except Exception:
            return None

    def _compute_err(self, a_xyz, b_xyz, dim: str = "xy") -> float:
        ax, ay, az = float(a_xyz[0]), float(a_xyz[1]), float(a_xyz[2])
        bx, by, bz = float(b_xyz[0]), float(b_xyz[1]), float(b_xyz[2])
        dx, dy, dz = ax - bx, ay - by, az - bz
        if dim == "xyz":
            return float(math.sqrt(dx * dx + dy * dy + dz * dz))
        return float(math.hypot(dx, dy))

    def _accumulate_total_distance(self, gt_xy: Tuple[float, float]):
        if self.last_gt_xy_global is None:
            self.last_gt_xy_global = gt_xy
            return
        dx = gt_xy[0] - self.last_gt_xy_global[0]
        dy = gt_xy[1] - self.last_gt_xy_global[1]
        self.total_distance += math.hypot(dx, dy)
        self.last_gt_xy_global = gt_xy

    def _compute_ate_and_accuracy(self) -> Tuple[float, float, float]:
        n = len(self.est_buf)
        if n < self.ate_min_samples:
            return float("nan"), float("nan"), float("nan")

        src = np.asarray(self.est_buf, dtype=np.float64)
        dst = np.asarray(self.gt_buf, dtype=np.float64)

        if self.ate_use_se2_align:
            R, t = umeyama_se2(src, dst)
            aligned = src @ R.T + t.reshape(1, 2)
        else:
            aligned = src

        errs = np.linalg.norm(aligned - dst, axis=1)
        rmse = float(np.sqrt(np.mean(errs * errs)))

        denom = max(self.total_distance, self.distance_epsilon)
        drift_total = 100.0 * (rmse / denom)
        acc_total = clamp(100.0 - drift_total, 0.0, 100.0)
        return rmse, float(drift_total), float(acc_total)

    def _compute_path_metric(self, traj_frame: str, msg: PointCloud2, pts_xyz: np.ndarray) -> Tuple[float, float]:
        if self.path_xy is None or self.path_xy.shape[0] < 1:
            return float("nan"), float("nan")
        if pts_xyz is None or pts_xyz.shape[0] == 0:
            return float("nan"), float("nan")

        pts_xy = self._transform_pts_to_path_frame(traj_frame, msg, pts_xyz)
        if pts_xy is None or pts_xy.shape[0] == 0:
            return float("nan"), float("nan")
        pts_xy = pts_xy[:, :2]

        if self.path_xy.shape[0] == 1:
            p0 = self.path_xy.reshape(1, 2)
            d = pts_xy - p0
            errs = np.sqrt(np.sum(d * d, axis=1))
        else:
            if self.seg_A is None or self.seg_V is None or self.seg_VV is None:
                return float("nan"), float("nan")
            errs = np.empty((pts_xy.shape[0],), dtype=np.float64)
            for i in range(pts_xy.shape[0]):
                errs[i] = point_to_polyline_min_dist_2d(pts_xy[i], self.seg_A, self.seg_V, self.seg_VV)

        mean_err = float(np.mean(errs))
        acc = 100.0 * float(np.mean(errs <= self.r_path))
        return mean_err, float(acc)

    def _transform_pts_to_path_frame(self, traj_frame: str, msg: PointCloud2, pts_xyz: np.ndarray) -> Optional[np.ndarray]:
        if not self.path_frame or not traj_frame:
            return pts_xyz
        if traj_frame == self.path_frame:
            return pts_xyz

        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            tf_time = rclpy.time.Time()
        else:
            tf_time = rclpy.time.Time.from_msg(stamp)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.path_frame,
                traj_frame,
                tf_time,
                timeout=self.tf_timeout,
            )
        except Exception:
            return None

        t = tf.transform.translation
        r = tf.transform.rotation
        x, y, z, w = float(r.x), float(r.y), float(r.z), float(r.w)
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        R = np.array(
            [
                [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
                [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
                [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
            ],
            dtype=np.float64,
        )
        trans = np.array([t.x, t.y, t.z], dtype=np.float64).reshape(1, 3)
        return (pts_xyz @ R.T) + trans

    def _extract_points_and_last(self, msg: PointCloud2, stride: int = 1):
        pts = []
        last = None
        try:
            i = 0
            for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                last = (x, y, z)
                if (i % stride) == 0:
                    pts.append((x, y, z))
                i += 1
        except Exception:
            i = 0
            for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
                x, y = float(p[0]), float(p[1])
                last = (x, y, 0.0)
                if (i % stride) == 0:
                    pts.append((x, y, 0.0))
                i += 1

        if last is None:
            return None, None
        pts_xyz = np.asarray(pts, dtype=np.float64) if pts else np.zeros((0, 3), dtype=np.float64)
        last_xyz = np.asarray(last, dtype=np.float64)
        return pts_xyz, last_xyz


def main(args=None):
    rclpy.init(args=args)
    node = TrajPathGtAteMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
