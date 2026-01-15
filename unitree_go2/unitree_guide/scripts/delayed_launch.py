#!/usr/bin/env python
import rospy
import sys
import time
import subprocess

if __name__ == '__main__':
    if len(sys.argv) < 2:
        rospy.logerr("Usage: delayed_launch.py <delay_seconds> <roslaunch_args...>")
        sys.exit(1)
    
    delay = float(sys.argv[1])
    rospy.loginfo("延迟 %.1f 秒后启动..." % delay)
    time.sleep(delay)
    
    roslaunch_args = sys.argv[2:]
    rospy.loginfo("执行: roslaunch %s" % ' '.join(roslaunch_args))
    
    cmd = ['roslaunch'] + roslaunch_args
    subprocess.call(cmd)
