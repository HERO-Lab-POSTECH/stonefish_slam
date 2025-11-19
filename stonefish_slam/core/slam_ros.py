# python imports
import threading
import numpy as np
import struct
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster
import cv_bridge
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped

# stonefish_slam imports
from stonefish_slam.utils.io import *
from stonefish_slam.utils.conversions import *
from stonefish_slam.utils.visualization import *
from stonefish_slam.core.slam import SLAM, Keyframe
from stonefish_slam.core.mapping_2d import Mapping2D
from stonefish_slam.cpp import pcl
from stonefish_slam.utils.topics import *


def pointcloud2_to_xyz_array(cloud_msg):
    """
    Convert PointCloud2 message to numpy array of XYZ points

    Args:
        cloud_msg: sensor_msgs.msg.PointCloud2

    Returns:
        numpy array of shape (N, 3) with xyz coordinates
    """
    # Get point step and create dtype
    point_step = cloud_msg.point_step

    # Parse fields to find x, y, z offsets
    field_names = [field.name for field in cloud_msg.fields]

    # Find offsets for x, y, z
    x_offset = None
    y_offset = None
    z_offset = None

    for field in cloud_msg.fields:
        if field.name == 'x':
            x_offset = field.offset
        elif field.name == 'y':
            y_offset = field.offset
        elif field.name == 'z':
            z_offset = field.offset

    if x_offset is None or y_offset is None or z_offset is None:
        raise ValueError("PointCloud2 must have x, y, z fields")

    # Convert data to numpy array
    num_points = cloud_msg.width * cloud_msg.height
    points = []

    for i in range(num_points):
        offset = i * point_step

        # Unpack x, y, z as floats
        x = struct.unpack('f', cloud_msg.data[offset + x_offset:offset + x_offset + 4])[0]
        y = struct.unpack('f', cloud_msg.data[offset + y_offset:offset + y_offset + 4])[0]
        z = struct.unpack('f', cloud_msg.data[offset + z_offset:offset + z_offset + 4])[0]

        points.append([x, y, z])

    # Ensure we return a 2D array even when empty
    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    return np.array(points, dtype=np.float32)


class SLAMNode(SLAM, Node):
    """This class takes the functionality from slam.py and implments it in the ROS2
    environment.
    """

    def __init__(self):
        # Initialize ROS2 Node first
        Node.__init__(self, 'slam_node')

        # Then initialize SLAM parent class
        SLAM.__init__(self)

        # the threading lock
        self.lock = threading.RLock()

        # Mapping initialization (configured in init_node)
        self.mapper = None
        self.enable_2d_mapping = False
        self.map_update_interval = 5  # 5 키프레임마다 업데이트 (성능 최적화)
        self.last_map_update_kf = 0
        self.bridge = cv_bridge.CvBridge()

        # Initialize node parameters and subscribers/publishers
        self.init_node()

    def init_node(self, ns="~") -> None:
        """Configures the SLAM node

        Args:
            ns (str, optional): The namespace of the node. Defaults to "~".
        """
        # keyframe paramters, how often to add them
        self.declare_parameter('keyframe_duration', 1.0)
        self.declare_parameter('keyframe_translation', 3.0)
        self.declare_parameter('keyframe_rotation', 0.5236)  # 30 degrees in radians

        keyframe_duration_sec = self.get_parameter('keyframe_duration').value
        self.keyframe_duration = Duration(seconds=keyframe_duration_sec)
        self.keyframe_translation = self.get_parameter('keyframe_translation').value
        self.keyframe_rotation = self.get_parameter('keyframe_rotation').value

        # SLAM paramter, are we using SLAM or just dead reckoning
        self.declare_parameter('enable_slam', True)
        self.enable_slam = self.get_parameter('enable_slam').value
        self.get_logger().info(f"SLAM STATUS: {self.enable_slam}")

        # noise models
        self.declare_parameter('prior_sigmas', [0.1, 0.1, 0.01])
        self.declare_parameter('odom_sigmas', [0.2, 0.2, 0.02])
        self.declare_parameter('icp_odom_sigmas', [0.1, 0.1, 0.01])

        self.prior_sigmas = self.get_parameter('prior_sigmas').value
        self.odom_sigmas = self.get_parameter('odom_sigmas').value
        self.icp_odom_sigmas = self.get_parameter('icp_odom_sigmas').value

        # resultion for map downsampling
        self.declare_parameter('point_resolution', 0.5)
        self.point_resolution = self.get_parameter('point_resolution').value

        # sequential scan matching parameters (SSM)
        self.declare_parameter('ssm.enable', True)
        self.declare_parameter('ssm.min_points', 50)
        self.declare_parameter('ssm.max_translation', 3.0)
        self.declare_parameter('ssm.max_rotation', 0.5236)  # 30 degrees
        self.declare_parameter('ssm.target_frames', 3)

        self.ssm_params.enable = self.get_parameter('ssm.enable').value
        self.ssm_params.min_points = self.get_parameter('ssm.min_points').value
        self.ssm_params.max_translation = self.get_parameter('ssm.max_translation').value
        self.ssm_params.max_rotation = self.get_parameter('ssm.max_rotation').value
        self.ssm_params.target_frames = self.get_parameter('ssm.target_frames').value
        self.get_logger().info(f"SSM: {self.ssm_params.enable}")

        # non sequential scan matching parameters (NSSM) aka loop closures
        self.declare_parameter('nssm.enable', True)
        self.declare_parameter('nssm.min_st_sep', 8)
        self.declare_parameter('nssm.min_points', 50)
        self.declare_parameter('nssm.max_translation', 10.0)
        self.declare_parameter('nssm.max_rotation', 1.0472)  # 60 degrees
        self.declare_parameter('nssm.source_frames', 5)
        self.declare_parameter('nssm.cov_samples', 30)

        self.nssm_params.enable = self.get_parameter('nssm.enable').value
        self.nssm_params.min_st_sep = self.get_parameter('nssm.min_st_sep').value
        self.nssm_params.min_points = self.get_parameter('nssm.min_points').value
        self.nssm_params.max_translation = self.get_parameter('nssm.max_translation').value
        self.nssm_params.max_rotation = self.get_parameter('nssm.max_rotation').value
        self.nssm_params.source_frames = self.get_parameter('nssm.source_frames').value
        self.nssm_params.cov_samples = self.get_parameter('nssm.cov_samples').value
        self.get_logger().info(f"NSSM: {self.nssm_params.enable}")

        # pairwise consistency maximization parameters for loop closure
        # outliar rejection
        self.declare_parameter('pcm_queue_size', 5)
        self.declare_parameter('min_pcm', 2)

        self.pcm_queue_size = self.get_parameter('pcm_queue_size').value
        self.min_pcm = self.get_parameter('min_pcm').value

        # 2D mapping parameters
        self.declare_parameter('enable_2d_mapping', True)
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('map_size', [4000, 4000])  # 원본과 동일 (slam_2d.py Line 1019)
        self.declare_parameter('sonar_range', 20.0)
        self.declare_parameter('sonar_fov', 130.0)
        self.declare_parameter('map_update_interval', 5)  # 5 키프레임마다 업데이트 (성능 최적화)

        self.enable_2d_mapping = self.get_parameter('enable_2d_mapping').value
        self.map_update_interval = self.get_parameter('map_update_interval').value

        if self.enable_2d_mapping:
            map_resolution = self.get_parameter('map_resolution').value
            map_size = tuple(self.get_parameter('map_size').value)
            sonar_range = self.get_parameter('sonar_range').value
            sonar_fov = self.get_parameter('sonar_fov').value

            self.mapper = Mapping2D(
                map_resolution=map_resolution,
                map_size=map_size,
                sonar_range=sonar_range,
                sonar_fov=sonar_fov
            )
            self.get_logger().info(f"2D Mapping enabled: resolution={map_resolution}m/px")

        # max delay between an incoming point cloud and dead reckoning
        self.feature_odom_sync_max_delay = 0.5

        # QoS profile for subscriptions (matching simulator's BEST_EFFORT)
        qos_sub_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        # QoS profile for publishers (RELIABLE for RViz compatibility)
        qos_pub_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # define the subsrcibing topics
        self.feature_sub = Subscriber(self, PointCloud2, SONAR_FEATURE_TOPIC, qos_profile=qos_sub_profile)
        self.odom_sub = Subscriber(self, Odometry, LOCALIZATION_ODOM_TOPIC, qos_profile=qos_sub_profile)

        # Add debug prints for topic names
        self.get_logger().info(f"Subscribing to feature topic: {SONAR_FEATURE_TOPIC}")
        self.get_logger().info(f"Subscribing to odom topic: {LOCALIZATION_ODOM_TOPIC}")

        # Sonar image subscriber for 2D mapping
        if self.enable_2d_mapping:
            self.sonar_sub = Subscriber(self, Image, '/bluerov2/fls/image', qos_profile=qos_sub_profile)
            self.get_logger().info("Subscribing to sonar image: /bluerov2/fls/image")

        # define the sync policy
        if self.enable_2d_mapping:
            self.time_sync = ApproximateTimeSynchronizer(
                [self.feature_sub, self.odom_sub, self.sonar_sub],
                20,
                self.feature_odom_sync_max_delay
            )
            self.time_sync.registerCallback(self.SLAM_callback_with_mapping)
            self.get_logger().info("Using 3-way synchronizer (feature + odom + sonar)")
        else:
            self.time_sync = ApproximateTimeSynchronizer(
                [self.feature_sub, self.odom_sub],
                20,
                self.feature_odom_sync_max_delay
            )
            self.time_sync.registerCallback(self.SLAM_callback)
            self.get_logger().info("Using 2-way synchronizer (feature + odom)")

        self.get_logger().info(f"Created time synchronizer with max delay: {self.feature_odom_sync_max_delay}")

        # pose publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, SLAM_POSE_TOPIC, 10)

        # dead reckoning topic
        self.odom_pub = self.create_publisher(Odometry, SLAM_ODOM_TOPIC, 10)

        # SLAM trajectory topic
        self.traj_pub = self.create_publisher(
            PointCloud2, SLAM_TRAJ_TOPIC, qos_pub_profile)

        # constraints between poses
        self.constraint_pub = self.create_publisher(
            Marker, SLAM_CONSTRAINT_TOPIC, qos_pub_profile)

        # point cloud publisher topic
        self.cloud_pub = self.create_publisher(
            PointCloud2, SLAM_CLOUD_TOPIC, qos_pub_profile)

        # 2D map publisher
        if self.enable_2d_mapping:
            self.map_2d_pub = self.create_publisher(
                Image,
                SLAM_NS + 'mapping/map_2d_image',
                qos_profile=qos_pub_profile
            )
            self.get_logger().info(f"Publishing 2D map to: {SLAM_NS}mapping/map_2d_image")

        # tf broadcaster to show pose
        self.tf = TransformBroadcaster(self)

        # cv bridge object
        self.CVbridge = cv_bridge.CvBridge()

        # get the ICP configuration from the yaml file
        self.declare_parameter('icp_config', '')
        icp_config = self.get_parameter('icp_config').value
        if icp_config:
            self.icp.loadFromYaml(icp_config)

        # define the robot ID this is not used here, extended in multi-robot SLAM
        self.rov_id = ""

        # call the configure function
        self.configure()
        self.get_logger().info("SLAM node is initialized")

    def SLAM_callback(self, feature_msg: PointCloud2, odom_msg: Odometry) -> None:
        """SLAM call back. Subscibes to the feature msg point cloud and odom msg
            Handles the whole SLAM system and publishes map, poses and constraints

        Args:
            feature_msg (PointCloud2): the incoming sonar point cloud
            odom_msg (Odometry): the incoming DVL/IMU state estimate
        """

        # aquire the lock
        self.lock.acquire()
        self.get_logger().info("SLAM_callback", throttle_duration_sec=1.0)

        # get rostime from the point cloud
        time = feature_msg.header.stamp

        # get the dead reckoning pose from the odom msg, GTSAM pose object
        dr_pose3 = r2g(odom_msg.pose.pose)

        # init a new key frame
        frame = Keyframe(False, time, dr_pose3)

        # convert the point cloud message to a numpy array of 2D
        points = pointcloud2_to_xyz_array(feature_msg)
        # Extract [x, y] from FRD frame point cloud (original Oculus used [x, -z])
        points = np.c_[points[:, 0], points[:, 1]]

        # In case feature extraction is skipped in this frame
        if len(points) and np.isnan(points[0, 0]):
            frame.status = False
        else:
            frame.status = self.is_keyframe(frame)

        # set the frames twist
        frame.twist = odom_msg.twist.twist

        # update the keyframe with pose information from dead reckoning
        if self.keyframes:
            dr_odom = self.current_keyframe.dr_pose.between(frame.dr_pose)
            pose = self.current_keyframe.pose.compose(dr_odom)
            frame.update(pose)

        # check frame staus, are we actually adding a keyframe?
        if frame.status:

            # add the point cloud to the frame
            frame.points = points

            # perform seqential scan matching
            # if this is the first frame do not
            if not self.keyframes:
                self.add_prior(frame)
            else:
                self.add_sequential_scan_matching(frame)

            # update the factor graph with the new frame
            self.update_factor_graph(frame)

            # if loop closures are enabled
            # nonsequential scan matching is True (a loop closure occured) update graph again
            if self.nssm_params.enable and self.add_nonsequential_scan_matching():
                self.update_factor_graph()

        # update current time step and publish the topics
        self.current_frame = frame
        self.publish_all()
        self.lock.release()

    def SLAM_callback_with_mapping(self, feature_msg: PointCloud2, odom_msg: Odometry, sonar_msg: Image) -> None:
        """SLAM callback with 2D mapping support (3-way synchronization).

        Integrates sonar image acquisition into the SLAM pipeline for 2D map generation.

        Args:
            feature_msg (PointCloud2): the incoming sonar point cloud
            odom_msg (Odometry): the incoming DVL/IMU state estimate
            sonar_msg (Image): the incoming sonar image
        """
        with self.lock:
            # 1. Convert sonar image
            try:
                sonar_image = self.bridge.imgmsg_to_cv2(sonar_msg, desired_encoding="mono8")
            except Exception as e:
                self.get_logger().error(f"Failed to convert sonar image: {e}")
                sonar_image = None

            # 2. Standard SLAM processing
            time = feature_msg.header.stamp
            dr_pose3 = r2g(odom_msg.pose.pose)
            frame = Keyframe(False, time, dr_pose3)

            # Convert point cloud
            points = pointcloud2_to_xyz_array(feature_msg)
            points = np.c_[points[:, 0], points[:, 1]]

            # Check if valid points
            if len(points) and np.isnan(points[0, 0]):
                frame.status = False
            else:
                frame.status = self.is_keyframe(frame)

            # Set frame twist
            frame.twist = odom_msg.twist.twist

            # Update keyframe pose from dead reckoning
            if self.keyframes:
                dr_odom = self.current_keyframe.dr_pose.between(frame.dr_pose)
                pose = self.current_keyframe.pose.compose(dr_odom)
                frame.update(pose)

            # 3. Process keyframe
            if frame.status:
                # Add points
                frame.points = points

                # Add sonar image to frame
                if sonar_image is not None:
                    frame.image = sonar_image

                # Sequential scan matching
                if not self.keyframes:
                    self.add_prior(frame)
                else:
                    self.add_sequential_scan_matching(frame)

                # Update factor graph
                self.update_factor_graph(frame)

                # Loop closure
                if self.nssm_params.enable and self.add_nonsequential_scan_matching():
                    self.update_factor_graph()

                # 4. Update 2D map periodically
                if (len(self.keyframes) - self.last_map_update_kf >= self.map_update_interval):
                    self.publish_2d_map()
                    self.last_map_update_kf = len(self.keyframes)

            # Update current frame and publish
            self.current_frame = frame
            self.publish_all()

    def publish_all(self) -> None:
        """Publish to all ouput topics
            trajectory, contraints, point cloud and the full GTSAM instance
        """
        if not self.keyframes:
            return

        self.publish_pose()
        if self.current_frame.status:
            self.publish_trajectory()
            self.publish_constraint()
            self.publish_point_cloud()

    def publish_pose(self) -> None:
        """Append dead reckoning from Localization to SLAM estimate to achieve realtime TF.
        """

        # define a pose with covariance message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.current_frame.time
        if self.rov_id == "":
            pose_msg.header.frame_id = "map"
        else:
            pose_msg.header.frame_id = self.rov_id + "_map"
        pose_msg.pose.pose = g2r(self.current_frame.pose3)

        cov = 1e-4 * np.identity(6, np.float32)
        # FIXME Use cov in current_frame
        cov[np.ix_((0, 1, 5), (0, 1, 5))] = self.current_keyframe.transf_cov
        pose_msg.pose.covariance = cov.ravel().tolist()
        self.pose_pub.publish(pose_msg)

        o2m = self.current_frame.pose3.compose(self.current_frame.dr_pose3.inverse())
        o2m = g2r(o2m)
        p = o2m.position
        q = o2m.orientation

        # ROS2 TF2 broadcast
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.stamp = self.current_frame.time
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self.tf.sendTransform(t)

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.pose.pose = pose_msg.pose.pose
        if self.rov_id == "":
            odom_msg.child_frame_id = "base_link"
        else:
            odom_msg.child_frame_id = self.rov_id + "_base_link"
        odom_msg.twist.twist = self.current_frame.twist
        self.odom_pub.publish(odom_msg)

    def publish_constraint(self) -> None:
        """Publish constraints between poses in the factor graph,
        either sequential or non-sequential.
        """

        # define a list of all the constraints
        links = []

        # iterate over all the keframes
        for x, kf in enumerate(self.keyframes[1:], 1):

            # append each SSM factor in blue
            p1 = self.keyframes[x - 1].pose3.x(), self.keyframes[x - 1].pose3.y(), self.keyframes[x - 1].dr_pose3.z()
            p2 = self.keyframes[x].pose3.x(), self.keyframes[x].pose3.y(), self.keyframes[x].dr_pose3.z()
            links.append((p1, p2, "blue"))

            # loop over all loop closures in this keyframe and append them in red
            for k, _ in self.keyframes[x].constraints:
                p0 = self.keyframes[k].pose3.x(), self.keyframes[k].pose3.y(), self.keyframes[k].dr_pose3.z()
                links.append((p0, p2, "red"))

        # if nothing, do nothing
        if links:

            # conver this list to a series of multi-colored lines and publish
            link_msg = ros_constraints(links)
            link_msg.header.stamp = self.current_keyframe.time
            if self.rov_id != "":
                link_msg.header.frame_id = self.rov_id + "_map"
            self.constraint_pub.publish(link_msg)

    def publish_trajectory(self) -> None:
        """Publish 3D trajectory as point cloud in [x, y, z, roll, pitch, yaw, index] format.
        """

        # get all the poses from each keyframe
        poses = np.array([g2n(kf.pose3) for kf in self.keyframes])

        # convert to a ros color line
        traj_msg = ros_colorline_trajectory(poses)
        traj_msg.header.stamp = self.current_keyframe.time
        if self.rov_id == "":
            traj_msg.header.frame_id = "map"
        else:
            traj_msg.header.frame_id = self.rov_id + "_map"
        self.traj_pub.publish(traj_msg)

    def publish_point_cloud(self) -> None:
        """Publish downsampled 3D point cloud with z = 0.
        The last column represents keyframe index at which the point is observed.
        """
        # 1. Collect point clouds from all keyframes
        all_points = [np.zeros((0, 2), np.float32)]

        # List of keyframe ids
        all_keys = []

        # 2. Transform each keyframe's points to global coordinate system
        for key in range(len(self.keyframes)):

            # get the pose
            pose = self.keyframes[key].pose

            # get the registered point cloud
            transf_points = self.keyframes[key].transf_points

            # append
            all_points.append(transf_points)
            all_keys.append(key * np.ones((len(transf_points), 1)))

        # 3. Merge point clouds
        all_points = np.concatenate(all_points)
        all_keys = np.concatenate(all_keys)

        # 4. Downsample point cloud using PCL
        sampled_points, sampled_keys = pcl.downsample(
            all_points, all_keys, self.point_resolution
        )

        # 5. Convert to ROS message and publish
        sampled_xyzi = np.c_[sampled_points, np.zeros_like(sampled_keys), sampled_keys]

        # if there are no points return and do nothing
        if len(sampled_xyzi) == 0:
            return

        # convert the point cloud to a ros message and publish
        cloud_msg = n2r(sampled_xyzi, "PointCloudXYZI")
        cloud_msg.header.stamp = self.current_keyframe.time
        if self.rov_id == "":
            cloud_msg.header.frame_id = "map"
        else:
            cloud_msg.header.frame_id = self.rov_id + "_map"
        self.cloud_pub.publish(cloud_msg)

    def publish_2d_map(self) -> None:
        """Generate and publish 2D sonar mosaic map.

        Uses the Mapping2D module to create a global 2D map from SLAM keyframes
        with sonar images. The map is published as a ROS Image message.
        """
        if not self.enable_2d_mapping or not self.mapper:
            return

        if not self.keyframes:
            return

        # Update global map from SLAM keyframes
        self.mapper.update_global_map_from_slam(self.keyframes)

        # Get map image
        map_image = self.mapper.get_map_image()
        if map_image is None or map_image.size == 0:
            return

        # Convert to ROS Image message
        try:
            image_msg = self.bridge.cv2_to_imgmsg(map_image, encoding="mono8")
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = "map"
            self.map_2d_pub.publish(image_msg)

            self.get_logger().info(
                f"Published 2D map: {map_image.shape[1]}x{map_image.shape[0]} pixels, "
                f"{len(self.keyframes)} keyframes",
                throttle_duration_sec=5.0
            )
        except Exception as e:
            self.get_logger().error(f"Failed to publish map: {e}")


def main(args=None):
    """Main function for SLAM node"""
    rclpy.init(args=args)

    node = SLAMNode()

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
