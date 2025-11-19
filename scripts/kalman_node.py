#!/usr/bin/env python3

# python imports
import rospy

# pull in the dead reckoning code
# from bruce_slam.utils.io import *
from hero_slam.kalman import KalmanNode

if __name__ == "__main__":
    rospy.init_node("localization", log_level=rospy.INFO)

    node = KalmanNode()
    node.init_node()
    rospy.spin()