# python imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import gtsam
import numpy as np
import os
import sys
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

# ros2-python imports
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, Imu, FluidPressure
from message_filters import ApproximateTimeSynchronizer, Subscriber
from stonefish_msgs.msg import DVL
import geometry_msgs.msg

# stonefish_slam imports
from stonefish_slam.utils.conversions import *
from stonefish_slam.utils.visualization import ros_colorline_trajectory
from stonefish_slam.utils.topics import *

import math
from std_msgs.msg import String, Float32

class DeadReckoningNode(Node):
	'''A class to support dead reckoning using DVL and IMU readings
	'''
	def __init__(self):
		super().__init__('dead_reckoning_node')
		self.get_logger().info("Dead reckoning node initializing...")

		self.pose = None  # vehicle pose
		self.prev_time = None  # previous reading time
		self.prev_vel = None  # previous reading velocity
		self.keyframes = []  # keyframe list

		# Force yaw at origin to be aligned with x axis
		self.imu_yaw0 = None
		self.imu_pose = [0, 0, 0, 0, 0, 0]  # x,y,z,roll,pitch,yaw
		self.imu_rot = None
		self.dvl_max_velocity = 0.3

		# Create a new key pose when
		# - |ti - tj| > min_duration and
		# - |xi - xj| > max_translation or
		# - |ri - rj| > max_rotation
		self.keyframe_duration = None
		self.keyframe_translation = None
		self.keyframe_rotation = None
		self.dvl_error_timer = 0.0

		# TODO pressure to depth has error(need to be modify)
		self.offset = 2.5

		# Latest pressure message
		self.latest_pressure_msg = None

		# Initialize node
		self.init_node_params()


	def init_node_params(self):
		"""Initialize ROS2 parameters and create publishers/subscribers"""

		# Declare parameters with default values
		self.declare_parameter('vehicle_name', 'bluerov2')
		self.declare_parameter('imu_pose', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
		self.declare_parameter('dvl_max_velocity', 0.3)
		self.declare_parameter('keyframe_duration', 1.0)
		self.declare_parameter('keyframe_translation', 1.0)
		self.declare_parameter('keyframe_rotation', 0.1)

		# Get parameters
		self.vehicle_name = self.get_parameter('vehicle_name').get_parameter_value().string_value
		imu_pose_list = self.get_parameter('imu_pose').get_parameter_value().double_array_value
		self.imu_pose = n2g(imu_pose_list, "Pose3")
		self.imu_rot = self.imu_pose.rotation()
		self.dvl_max_velocity = self.get_parameter('dvl_max_velocity').get_parameter_value().double_value
		self.keyframe_duration = self.get_parameter('keyframe_duration').get_parameter_value().double_value
		self.keyframe_translation = self.get_parameter('keyframe_translation').get_parameter_value().double_value
		self.keyframe_rotation = self.get_parameter('keyframe_rotation').get_parameter_value().double_value

		# QoS profile
		qos = QoSProfile(
			reliability=ReliabilityPolicy.BEST_EFFORT,
			history=HistoryPolicy.KEEP_LAST,
			depth=10
		)

		# Build topic names with vehicle_name
		dvl_topic = f'/{self.vehicle_name}/dvl_sim'
		imu_topic = f'/{self.vehicle_name}/imu'
		pressure_topic = f'/{self.vehicle_name}/pressure'
		odometry_gt_topic = f'/{self.vehicle_name}/odometry'

		self.get_logger().info(f'Subscribing to topics for vehicle: {self.vehicle_name}')
		self.get_logger().info(f'  DVL: {dvl_topic}')
		self.get_logger().info(f'  IMU: {imu_topic}')
		self.get_logger().info(f'  Pressure: {pressure_topic}')
		self.get_logger().info(f'  Odometry GT: {odometry_gt_topic}')

		# Subscribers using message_filters for synchronization
		self.dvl_sub = Subscriber(self, DVL, dvl_topic, qos_profile=qos)
		self.imu_sub = Subscriber(self, Imu, imu_topic, qos_profile=qos)

		# Pressure subscriber (simple subscription for caching)
		self.pressure_sub = self.create_subscription(
			FluidPressure,
			pressure_topic,
			self.pressure_callback,
			qos
		)

		# Ground truth odometry subscriber for comparison
		self.odometry_gt_sub = self.create_subscription(
			Odometry,
			odometry_gt_topic,
			self.odometry_gt_callback,
			qos
		)
		self.latest_odometry_gt = None

		# Approximate time synchronizer for IMU and DVL
		self.ts = ApproximateTimeSynchronizer(
			[self.imu_sub, self.dvl_sub],
			queue_size=200,
			slop=0.1
		)
		self.ts.registerCallback(self.callback)

		# Publishers - simplified topic names
		self.traj_pub = self.create_publisher(
			PointCloud2, '/dead_reck/key_traj', 10)

		self.odom_pub = self.create_publisher(
			Odometry, '/dead_reck/odom', 10)

		self.path_pub = self.create_publisher(
			Path, '/dead_reck/path', 10)

		self.path_msg = Path()
		self.path_msg.header.frame_id = "odom"

		# TF broadcaster
		self.tf_broadcaster = TransformBroadcaster(self)

		self.get_logger().info("Dead reckoning node initialized")

	def pressure_callback(self, msg):
		"""Cache the latest pressure message"""
		self.latest_pressure_msg = msg

	def odometry_gt_callback(self, msg):
		"""Cache the latest ground truth odometry message"""
		self.latest_odometry_gt = msg

	def callback(self, imu_msg: Imu, dvl_msg: DVL) -> None:
		"""Handle the dead reckoning using the IMU and DVL only. Fuse and publish an odometry message.

		Args:
			imu_msg (Imu): the message from IMU
			dvl_msg (DVL): the message from the DVL
		"""
		# get the previous pressure msg
		pressure_msg = self.latest_pressure_msg
		# if there is no depth msg, then skip this time step
		if pressure_msg is None:
			return

		# curr_depth = -((pressure_msg.fluid_pressure / 101.325) - 1) * 10
		# curr_depth -= self.offset
		curr_depth = 0.0

		# Calculate time difference
		dvl_time_sec = dvl_msg.header.stamp.sec + dvl_msg.header.stamp.nanosec / 1e9
		pressure_time_sec = pressure_msg.header.stamp.sec + pressure_msg.header.stamp.nanosec / 1e9
		pd_delay = pressure_time_sec - dvl_time_sec

		if abs(pd_delay) > 1.0:
			self.get_logger().debug(f"Missing pressure msg for {pd_delay}")

		# convert the imu message from msg to gtsam rotation object
		rot = r2g(imu_msg.orientation)

		# if we have no yaw yet, set this one as zero
		if self.imu_yaw0 is None:
			self.imu_yaw0 = rot.yaw()

		# Get a rotation matrix
		rot = gtsam.Rot3.Ypr(rot.yaw()-self.imu_yaw0, rot.pitch(), np.radians(90)+rot.roll())

		# parse the DVL message into an array of velocities
		vel = np.array([dvl_msg.velocity.x, dvl_msg.velocity.y, dvl_msg.velocity.z])

		# package the odom message and publish it
		self.send_odometry(vel, rot, dvl_msg.header.stamp, curr_depth)


	def send_odometry(self, vel: np.array, rot: gtsam.Rot3, dvl_time, depth: float) -> None:
		"""Package the odometry given all the DVL, rotation matrix, and depth

		Args:
			vel (np.array): a numpy array (1D) of the DVL velocities
			rot (gtsam.Rot3): the rotation matrix of the vehicle
			dvl_time: the time stamp for the DVL message (builtin_interfaces.msg.Time)
			depth (float): vehicle depth
		"""

		# Convert ROS2 time to seconds
		current_time_sec = dvl_time.sec + dvl_time.nanosec / 1e9

		# if the DVL message has any velocity above the max threshold do some error handling
		if np.any(np.abs(vel) > self.dvl_max_velocity):
			if self.pose:
				if self.prev_time is not None:
					prev_time_sec = self.prev_time.sec + self.prev_time.nanosec / 1e9
					self.dvl_error_timer += (current_time_sec - prev_time_sec)

				if self.dvl_error_timer > 5.0:
					self.get_logger().warning(
						f"DVL velocity ({vel[0]:.1f}, {vel[1]:.1f}, {vel[2]:.1f}) "
						f"exceeds max velocity {self.dvl_max_velocity:.1f} for {self.dvl_error_timer:.1f} secs."
					)
				vel = self.prev_vel
			else:
				return
		else:
			self.dvl_error_timer = 0.0

		if self.pose:
			# figure out how far we moved in the body frame using the DVL message
			prev_time_sec = self.prev_time.sec + self.prev_time.nanosec / 1e9
			dt = current_time_sec - prev_time_sec
			dv = (vel + self.prev_vel) * 0.5
			trans = dv * dt

			# get a rotation matrix with only roll and pitch
			rotation_flat = gtsam.Rot3.Ypr(0, rot.pitch(), rot.roll())

			# propagate our movement forward using the GTSAM utilities
			local_point = gtsam.Point2(trans[0], trans[1])

			pose2 = gtsam.Pose2(
				self.pose.x(), self.pose.y(), self.pose.rotation().yaw()
			)
			point = pose2.transformFrom(local_point)

			self.pose = gtsam.Pose3(
				rot, gtsam.Point3(point[0], point[1], depth)
			)

		else:
			# init the pose
			self.pose = gtsam.Pose3(rot, gtsam.Point3(0, 0, depth))

		# log this timestep's messages for next time
		self.prev_time = dvl_time
		self.prev_vel = vel

		# Check for new keyframe
		new_keyframe = False
		if not self.keyframes:
			new_keyframe = True
		else:
			prev_kf_time_sec = self.keyframes[-1][0]
			duration = current_time_sec - prev_kf_time_sec
			if duration > self.keyframe_duration:
				odom = self.keyframes[-1][1].between(self.pose)
				odom = g2n(odom)
				translation = np.linalg.norm(odom[:3])
				rotation = abs(odom[-1])

				if (translation > self.keyframe_translation or
					rotation > self.keyframe_rotation):
					new_keyframe = True

		if new_keyframe:
			self.keyframes.append((current_time_sec, self.pose))

		self.publish_pose(new_keyframe)

		# send path topic
		pose_stamped = geometry_msgs.msg.PoseStamped()
		pose_stamped.header.stamp = self.prev_time
		pose_stamped.header.frame_id = "odom"
		pose_stamped.pose = g2r(self.pose)
		self.path_msg.poses.append(pose_stamped)

		self.path_msg.header.stamp = self.prev_time
		self.path_pub.publish(self.path_msg)


	def publish_pose(self, publish_traj: bool = False) -> None:
		"""Publish the pose

		Args:
			publish_traj (bool, optional): Are we publishing the whole set of keyframes?. Defaults to False.
		"""
		if self.pose is None:
			return

		# Create header
		odom_msg = Odometry()
		odom_msg.header.stamp = self.prev_time
		odom_msg.header.frame_id = "odom"

		# pose in odom frame
		odom_msg.pose.pose = g2r(self.pose)

		# twist in local frame
		odom_msg.child_frame_id = "base_link"
		odom_msg.twist.twist.linear.x = 0.0
		odom_msg.twist.twist.linear.y = 0.0
		odom_msg.twist.twist.linear.z = 0.0
		odom_msg.twist.twist.angular.x = 0.0
		odom_msg.twist.twist.angular.y = 0.0
		odom_msg.twist.twist.angular.z = 0.0

		self.odom_pub.publish(odom_msg)

		# Broadcast TF
		t = TransformStamped()
		t.header.stamp = self.prev_time
		t.header.frame_id = 'odom'
		t.child_frame_id = 'base_link'

		p = odom_msg.pose.pose.position
		q = odom_msg.pose.pose.orientation

		t.transform.translation.x = p.x
		t.transform.translation.y = p.y
		t.transform.translation.z = p.z
		t.transform.rotation.x = q.x
		t.transform.rotation.y = q.y
		t.transform.rotation.z = q.z
		t.transform.rotation.w = q.w

		self.tf_broadcaster.sendTransform(t)

		# Compare with ground truth if available
		if self.latest_odometry_gt is not None:
			gt_pos = self.latest_odometry_gt.pose.pose.position
			dr_pos = odom_msg.pose.pose.position

			# Calculate position error
			error_x = dr_pos.x - gt_pos.x
			error_y = dr_pos.y - gt_pos.y
			error_z = dr_pos.z - gt_pos.z
			error_2d = np.sqrt(error_x**2 + error_y**2)
			error_3d = np.sqrt(error_x**2 + error_y**2 + error_z**2)

			# Log error periodically (every 50 updates to avoid spam)
			if not hasattr(self, 'error_log_counter'):
				self.error_log_counter = 0

			self.error_log_counter += 1
			if self.error_log_counter >= 50:
				self.get_logger().info(
					f'Dead Reckoning Error - 2D: {error_2d:.3f}m, 3D: {error_3d:.3f}m '
					f'(X: {error_x:.3f}, Y: {error_y:.3f}, Z: {error_z:.3f})'
				)
				self.error_log_counter = 0

		if publish_traj:
			traj = np.array([g2n(pose) for _, pose in self.keyframes])
			traj_msg = ros_colorline_trajectory(traj)
			traj_msg.header.stamp = self.prev_time
			traj_msg.header.frame_id = "odom"
			self.traj_pub.publish(traj_msg)


def main(args=None):
	rclpy.init(args=args)
	node = DeadReckoningNode()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
