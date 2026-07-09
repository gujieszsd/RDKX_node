#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import subprocess
import os

class MapSaverNode:
    def __init__(self):
        rospy.init_node('map_saver_node', anonymous=True)
        self.map_script = rospy.get_param('~map_script_path', 
                                          '/home/fsrobot/catkin_ws/src/fsrobot/maps/map.sh')
        rospy.loginfo(f"Map save script: {self.map_script}")
        self.sub = rospy.Subscriber('/save_map', String, self.callback)

    def callback(self, msg):
        if msg.data.lower() != 'save':
            return
        rospy.loginfo("Received save map command")
        if not os.path.exists(self.map_script):
            rospy.logerr(f"Script not found: {self.map_script}")
            return
        try:
            subprocess.run([self.map_script], check=True, cwd=os.path.dirname(self.map_script))
            rospy.loginfo("Map saved successfully")
        except Exception as e:
            rospy.logerr(f"Failed to save map: {e}")

if __name__ == '__main__':
    try:
        node = MapSaverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass