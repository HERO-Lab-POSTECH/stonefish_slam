import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBridge(Node):
    """Republish a nav_msgs/Odometry as a TF.

    Useful when a bag streams the vehicle pose only as Odometry (header.frame_id ->
    child_frame_id) so that RViz views with a Target Frame anchored at the vehicle
    (e.g. girona500/base_link) can resolve the transform.
    """

    def __init__(self):
        super().__init__('odom_tf_bridge')

        self.declare_parameter('odom_topic', '/vehicle_synced/dynamics/odometry')
        self.declare_parameter('parent_frame', '')   # '' => use header.frame_id
        self.declare_parameter('child_frame', 'girona500/base_link')  # '' => use msg.child_frame_id
        self.declare_parameter('reliability', 'reliable')  # 'reliable' | 'best_effort'

        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_override = self.get_parameter('parent_frame').value
        self.child_override = self.get_parameter('child_frame').value
        reliability = self.get_parameter('reliability').value

        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=(ReliabilityPolicy.BEST_EFFORT
                         if reliability == 'best_effort'
                         else ReliabilityPolicy.RELIABLE),
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, qos)

        self._last_stamp_ns = -1
        self.get_logger().info(
            f"odom_tf_bridge: '{self.odom_topic}' -> TF "
            f"[parent='{self.parent_override or '<msg.frame_id>'}', "
            f"child='{self.child_override or '<msg.child_frame_id>'}', "
            f"reliability={reliability}]"
        )

    def _on_odom(self, msg: Odometry):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns == self._last_stamp_ns:
            # tf2 rejects duplicate timestamps with a warning; drop silently.
            return
        self._last_stamp_ns = stamp_ns

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_override or msg.header.frame_id
        t.child_frame_id = self.child_override or msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0
