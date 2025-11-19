# python imports
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import gtsam
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL
from tf_transformations import euler_from_quaternion

# import stonefish_slam
from stonefish_slam.utils.conversions import *


class KalmanNode(Node):
    '''A class to support Kalman filtering using DVL, IMU and Depth readings.
    '''

    def __init__(self):
        super().__init__('kalman_node')

        self.state_vector = np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])
        self.cov_matrix = np.diag([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
        self.imu_yaw0 = None
        self.offset_z = 2.5  # TODO GW_i don't know why there is error between the measurement from a pressure sensor and real depth

        self.init_node()

    def init_node(self, ns=""):
        """Init the node, fetch all paramaters.

        Args:
            ns (str, optional): The namespace of the node. Defaults to "~".
        """
        # Declare all parameters
        self.declare_parameter('state_vector', [0.0] * 12)
        self.declare_parameter('cov_matrix', [0.0] * 12)

        # dvl
        self.declare_parameter('R_dvl', [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]])
        self.declare_parameter('dt_dvl', 0.1)
        self.declare_parameter('H_dvl', [[0] * 12] * 3)

        # imu
        self.declare_parameter('A_imu', [[1.0 if i == j else 0.0 for j in range(12)] for i in range(12)])
        self.declare_parameter('R_imu', [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]])
        self.declare_parameter('dt_imu', 0.01)
        self.declare_parameter('H_imu', [[0] * 12] * 3)

        # depth
        self.declare_parameter('H_depth', [[0] * 12] * 3)
        self.declare_parameter('R_depth', [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]])
        self.declare_parameter('dt_depth', 0.1)

        # MISC
        self.declare_parameter('Q', [[0.01 if i == j else 0.0 for j in range(12)] for i in range(12)])
        self.declare_parameter('offset.x', 0.0)
        self.declare_parameter('offset.y', 0.0)
        self.declare_parameter('offset.z', 0.0)
        self.declare_parameter('dvl_max_velocity', 5.0)
        self.declare_parameter('imu_offset', 0.0)

        # Get parameters
        self.state_vector = np.array(self.get_parameter('state_vector').value).reshape(-1, 1)
        self.cov_matrix = np.array(self.get_parameter('cov_matrix').value)
        if self.cov_matrix.ndim == 1:
            self.cov_matrix = np.diag(self.cov_matrix)

        # dvl
        self.R_dvl = np.array(self.get_parameter('R_dvl').value)
        self.dt_dvl = self.get_parameter('dt_dvl').value
        self.H_dvl = np.array(self.get_parameter('H_dvl').value)

        # imu
        self.A_imu = np.array(self.get_parameter('A_imu').value)
        self.R_imu = np.array(self.get_parameter('R_imu').value)
        self.dt_imu = self.get_parameter('dt_imu').value
        self.H_imu = np.array(self.get_parameter('H_imu').value)

        # depth
        self.H_depth = np.array(self.get_parameter('H_depth').value)
        self.R_depth = np.array(self.get_parameter('R_depth').value)
        self.dt_depth = self.get_parameter('dt_depth').value

        # MISC
        self.Q = np.array(self.get_parameter('Q').value)
        x = self.get_parameter('offset.x').value
        y = self.get_parameter('offset.y').value
        z = self.get_parameter('offset.z').value
        self.offset_matrix = Rotation.from_euler("xyz", [x, y, z], degrees=True).as_matrix()
        self.dvl_max_velocity = self.get_parameter('dvl_max_velocity').value
        self.imu_offset = np.radians(self.get_parameter('imu_offset').value)

        # define the subscribers - BlueROV2 topics
        self.imu_sub = self.create_subscription(
            Imu,
            "bluerov2/imu",
            self.imu_callback,
            10
        )
        self.dvl_sub = self.create_subscription(
            DVL,
            "bluerov2/dvl_sim",
            self.dvl_callback,
            10
        )
        self.pressure_sub = self.create_subscription(
            FluidPressure,
            "bluerov2/pressure",
            self.pressure_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "kalman_odom", 10)
        self.path_pub = self.create_publisher(Path, "kalman_path", 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        self.tf1 = TransformBroadcaster(self)

        # define the initial pose, all zeros
        R_init = gtsam.Rot3.Ypr(0., 0., 0.)
        self.pose = gtsam.Pose3(R_init, gtsam.Point3(0, 0, 0))

        # log at the roslevel that we are done with init
        self.get_logger().info("Kalman Node is initialized")

    def kalman_predict(self, previous_x: np.array, previous_P: np.array, A: np.array):
        """Propagate the state and the error covariance ahead.

        Args:
            previous_x (np.array): value of the previous state vector
            previous_P (np.array): value of the previous covariance matrix
            A (np.array): State Transition Matrix

        Returns:
            predicted_x (np.array): predicted estimation
            predicted_P (np.array): predicted covariance matrix
        """

        A = np.array(A)
        predicted_x = A @ previous_x
        predicted_P = A @ previous_P @ A.T + self.Q

        return predicted_x, predicted_P

    def kalman_correct(self, predicted_x: np.array, predicted_P: np.array, z: np.array, H: np.array, R: np.array):
        """Measurement Update.

        Args:
            predicted_x (np.array): predicted state vector with kalman_predict()
            predicted_P (np.array): predicted covariance matrix with kalman_predict()
            z (np.array): Output Vector (measurement)
            H (np.array): Observation Matrix (H_dvl, H_imu, H_gyro, H_depth)
            R (np.array): Measurement Uncertainty (R_dvl, R_imu, R_gyro, R_depth)

        Returns:
            corrected_x (np.array): corrected estimation
            corrected_P (np.array): corrected covariance matrix

        """

        K = predicted_P @ H.T @ np.linalg.inv(H @ predicted_P @ H.T + R)
        corrected_x = predicted_x + K @ (z - H @ predicted_x)
        corrected_P = predicted_P - K @ H @ predicted_P

        return corrected_x, corrected_P

    def dvl_callback(self, dvl_msg: DVL) -> None:
        """Handle the Kalman Filter using the DVL only.

        Args:
            dvl_msg (DVL): the message from the DVL
        """

        # parse the dvl velocites
        dvl_measurement = np.array([[dvl_msg.velocity.x], [dvl_msg.velocity.y], [dvl_msg.velocity.z]])

        # We do not do a kalman correction if the speed is high.
        if np.any(np.abs(dvl_measurement) > self.dvl_max_velocity):
            return
        else:
            self.state_vector, self.cov_matrix = self.kalman_correct(self.state_vector, self.cov_matrix, dvl_measurement, self.H_dvl, self.R_dvl)

    def imu_callback(self, imu_msg: Imu) -> None:
        """Handle the Kalman Filter using the VN100 only. Publish the state vector.

        Args:
            imu_msg (Imu): the message from VN100
        """
        # Kalman prediction
        predicted_x, predicted_P = self.kalman_predict(self.state_vector, self.cov_matrix, self.A_imu)

        # measurement_z preparation
        # parse the IMU measurnment
        roll_x, pitch_y, yaw_z = euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
        euler_angle = np.array([[self.imu_offset + roll_x], [pitch_y], [yaw_z]])

        # if we have no yaw yet, set this one as zero
        if self.imu_yaw0 is None:
            self.imu_yaw0 = yaw_z

        # make yaw relative to the first meas
        euler_angle[2] -= self.imu_yaw0

        # Kalman correction
        self.state_vector, self.cov_matrix = self.kalman_correct(predicted_x, predicted_P, euler_angle, self.H_imu, self.R_imu)

        # Use filtered velocity to update our x and y estimates
        trans_x = self.state_vector[6][0] * self.dt_imu  # x update
        trans_y = self.state_vector[7][0] * self.dt_imu  # y update
        local_point = gtsam.Point2(trans_x, trans_y)

        # check if we are using the FOG
        R = gtsam.Rot3.Ypr(self.state_vector[5][0], self.state_vector[4][0], self.state_vector[3][0])
        pose2 = gtsam.Pose2(self.pose.x(), self.pose.y(), self.pose.rotation().yaw())

        # GW_current_depth
        curr_z = self.state_vector[2][0]

        # update our pose estimate and send out the odometry message
        point = pose2.transformFrom(local_point)
        self.pose = gtsam.Pose3(R, gtsam.Point3(point[0], point[1], curr_z))
        self.send_odometry(imu_msg.header.stamp)
        self.send_path(imu_msg.header.stamp)

    def pressure_callback(self, pressure_msg: FluidPressure) -> None:
        """Handle the Kalman Filter using the Depth.
        Args:
            pressure_msg (FluidPressure): pressure
        """
        curr_depth = -((pressure_msg.fluid_pressure / 101.325) - 1) * 10
        curr_depth -= self.offset_z

        depth = np.array([[curr_depth], [0], [0]])  # We need the shape(3,1) for the correction
        self.state_vector, self.cov_matrix = self.kalman_correct(self.state_vector, self.cov_matrix, depth, self.H_depth, self.R_depth)

    def send_odometry(self, t):
        """Publish the pose.
        Args:
            t: time from imu_msg
        """

        odom_msg = Odometry()
        odom_msg.header.stamp = t
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose = g2r(self.pose)
        odom_msg.child_frame_id = "base_link"
        odom_msg.twist.twist.linear.x = 0.
        odom_msg.twist.twist.linear.y = 0.
        odom_msg.twist.twist.linear.z = 0.
        odom_msg.twist.twist.angular.x = 0.
        odom_msg.twist.twist.angular.y = 0.
        odom_msg.twist.twist.angular.z = 0.
        self.odom_pub.publish(odom_msg)

        # ROS2 TF2 broadcast
        transform = TransformStamped()
        transform.header.stamp = t
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = odom_msg.pose.pose.position.x
        transform.transform.translation.y = odom_msg.pose.pose.position.y
        transform.transform.translation.z = odom_msg.pose.pose.position.z
        transform.transform.rotation.x = odom_msg.pose.pose.orientation.x
        transform.transform.rotation.y = odom_msg.pose.pose.orientation.y
        transform.transform.rotation.z = odom_msg.pose.pose.orientation.z
        transform.transform.rotation.w = odom_msg.pose.pose.orientation.w
        self.tf1.sendTransform(transform)

    def send_path(self, t):
        """Publish the path.
        Args:
            t: time from imu_msg
        """
        # send path topic
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = t
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose = g2r(self.pose)
        self.path_msg.poses.append(pose_stamped)

        self.path_msg.header.stamp = t
        self.path_pub.publish(self.path_msg)


def main(args=None):
    """Main function for Kalman node"""
    rclpy.init(args=args)

    node = KalmanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    main()
