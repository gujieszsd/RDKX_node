#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Empty
import std_srvs.srv   # 新增这一行

class OdomResetNode:
    def __init__(self):
        rospy.init_node('odom_reset_node', anonymous=True)
        self.sub = rospy.Subscriber('/reset_odom', Empty, self.callback)
        rospy.loginfo("Odom reset node ready (no control messages published)")

    def callback(self, msg):
        rospy.loginfo("Received odom reset request")
        try:
            reset_srv = rospy.ServiceProxy('/reset_odometry', std_srvs.srv.Empty)
            reset_srv()
            rospy.loginfo("Reset service called successfully")
        except rospy.ServiceException as e:
            rospy.logwarn(f"Reset service not available: {e}")
        except Exception as e:
            rospy.logwarn(f"Could not reset odometry: {e}")

if __name__ == '__main__':
    try:
        node = OdomResetNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass