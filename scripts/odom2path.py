#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped


class Odom2Path():
    def __init__(self):
        rospy.init_node("Odom2Path", anonymous = True)
        rospy.Subscriber("rexrov2/pose_gt", Odometry, self.callback)
        self.path_pub = rospy.Publisher("rexrov2/gt_path", Path, queue_size = 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        rospy.spin()

    def callback(self, data):
        odom_position = data.pose.pose.position
        odom_orientation = data.pose.pose.orientation

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = data.header.stamp
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose.position = odom_position
        pose_stamped.pose.orientation = odom_orientation

        self.path_msg.poses.append(pose_stamped)
        self.path_pub.publish(self.path_msg)

        


        

        



if __name__ == "__main__":
    odom2path = Odom2Path()
    
    


# rospy.init_node("pose_to_path", log_level=rospy.INFO)

# rospy.Subscriber("rexrov2/pose_gt", Odometry, callback)
# rospy.spin()

    
