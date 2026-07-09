#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import signal
import os
import threading
from std_msgs.msg import String

class LaunchManager:
    def __init__(self):
        rospy.init_node('launch_manager_node', anonymous=True)
        self.processes = {}  # {name: subprocess.Popen}
        self.lock = threading.Lock()

        rospy.Subscriber('/start_slam', String, self.cb_start_slam)
        rospy.Subscriber('/stop_slam', String, self.cb_stop_slam)
        rospy.Subscriber('/start_navigation', String, self.cb_start_nav)
        rospy.Subscriber('/stop_navigation', String, self.cb_stop_nav)

        rospy.loginfo("Launch Manager Node ready.")

    def _launch(self, name, pkg, launch_file, extra_args=""):
        full_cmd = f"roslaunch {pkg} {launch_file} {extra_args}".strip()
        rospy.loginfo(f"Starting {name}: {full_cmd}")
        with self.lock:
            if name in self.processes and self.processes[name].poll() is None:
                rospy.logwarn(f"{name} already running.")
                return False
            try:
                proc = subprocess.Popen(full_cmd, shell=True, preexec_fn=os.setsid)
                self.processes[name] = proc
                rospy.loginfo(f"{name} started (PID: {proc.pid})")
                return True
            except Exception as e:
                rospy.logerr(f"Failed to start {name}: {e}")
                return False

    def _stop(self, name):
        with self.lock:
            proc = self.processes.get(name)
            if proc is None or proc.poll() is not None:
                rospy.logwarn(f"{name} is not running.")
                return False
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                rospy.loginfo(f"{name} stopped (SIGINT sent).")
                del self.processes[name]
                return True
            except Exception as e:
                rospy.logerr(f"Failed to stop {name}: {e}")
                return False

    def cb_start_slam(self, msg):
        if msg.data.lower() == 'start':
            self._launch('slam', 'fsrobot', 'lidar_slam.launch')
        else:
            rospy.logwarn("start_slam expects 'start' data")

    def cb_stop_slam(self, msg):
        if msg.data.lower() == 'stop':
            self._stop('slam')
        else:
            rospy.logwarn("stop_slam expects 'stop' data")

    def cb_start_nav(self, msg):
        if msg.data.lower() == 'start':
            self._launch('navigation', 'fsrobot', 'navigate.launch')
        else:
            rospy.logwarn("start_navigation expects 'start' data")

    def cb_stop_nav(self, msg):
        if msg.data.lower() == 'stop':
            self._stop('navigation')
        else:
            rospy.logwarn("stop_navigation expects 'stop' data")

if __name__ == '__main__':
    try:
        lm = LaunchManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass