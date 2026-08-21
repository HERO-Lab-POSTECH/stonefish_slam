import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2  # ROS2
from std_msgs.msg import Header

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def ros_colorline_trajectory(traj):
    # ROS2: Create PointField objects properly
    from sensor_msgs.msg import PointField
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="roll", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="pitch", offset=16, datatype=PointField.FLOAT32, count=1),
        PointField(name="yaw", offset=20, datatype=PointField.FLOAT32, count=1),
        PointField(name="i", offset=24, datatype=PointField.FLOAT32, count=1),
    ]

    traji = np.c_[traj, np.mgrid[0 : len(traj)]]

    header = Header()
    return pc2.create_cloud(header, fields, traji)


def make_color_rgba(r, g, b, a=1.0):
    """Helper function to create ColorRGBA for ROS2"""
    color = ColorRGBA()
    color.r = r
    color.g = g
    color.b = b
    color.a = a
    return color

colors = {
    "red": make_color_rgba(1.0, 0.0, 0.0, 1.0),      # Pure red, opaque (loop closures)
    "blue": make_color_rgba(0.0, 0.5, 1.0, 1.0),     # Sky blue, opaque (SSM)
    "green": make_color_rgba(0.2, 0.8, 0.5, 0.6),    # Cyan-green, semi-transparent
    "white": make_color_rgba(1.0, 1.0, 1.0, 1.0),
    "yellow": make_color_rgba(1.0, 1.0, 0.0, 1.0),
    "light_blue": make_color_rgba(0.44, 0.62, 0.8118, 1.0)
}


def ros_constraints(links):
    marker = Marker()
    marker.header.frame_id = "world_ned"
    marker.type = Marker.LINE_LIST
    marker.ns = "constraints"
    marker.scale.x = 0.03  # Thinner lines (3cm instead of 20cm)
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0  # Fully opaque

    for point1, point2, color in links:
        # ROS2: Point() has no args, set fields directly
        p1 = Point()
        p1.x = float(point1[0])
        p1.y = float(point1[1])
        p1.z = float(point1[2])

        p2 = Point()
        p2.x = float(point2[0])
        p2.y = float(point2[1])
        p2.z = float(point2[2])

        marker.points.append(p1)
        marker.points.append(p2)
        marker.colors.append(colors[color])
        marker.colors.append(colors[color])

    return marker
