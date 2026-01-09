#!/usr/bin/env python3
import math
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class SetInitialPoseFromGazebo:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "go2_gazebo")
        self.initialpose_topic = rospy.get_param("~initialpose_topic", "/initialpose")
        self.frame_id = rospy.get_param("~frame_id", "map")

        # 可选：等一会儿再取真值（比如等起立结束）
        self.delay = float(rospy.get_param("~delay", 0.0))

        # 多发几次，避免AMCL订阅尚未建立
        self.repeat = int(rospy.get_param("~repeat", 3))
        self.repeat_dt = float(rospy.get_param("~repeat_dt", 0.1))

        self._done = False
        self._pub = rospy.Publisher(self.initialpose_topic, PoseWithCovarianceStamped, queue_size=1, latch=True)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._cb, queue_size=1)
        rospy.loginfo("Waiting for /gazebo/model_states, model_name=%s, will publish to %s",
                      self.model_name, self.initialpose_topic)

    def _cb(self, msg: ModelStates):
        if self._done:
            return
        if self.model_name not in msg.name:
            return

        if self.delay > 0:
            rospy.sleep(self.delay)

        i = msg.name.index(self.model_name)
        pose = msg.pose[i]

        q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _, _, yaw = euler_from_quaternion(q)

        out = PoseWithCovarianceStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.frame_id
        out.pose.pose.position.x = pose.position.x
        out.pose.pose.position.y = pose.position.y
        out.pose.pose.position.z = 0.0  # 2D导航一般用0
        out.pose.pose.orientation = pose.orientation

        cov = [0.0] * 36
        cov[0]  = 0.04    # σ=0.2 m
        cov[7]  = 0.04
        cov[35] = 0.02    # σ≈8°

        out.pose.covariance = cov

        for _ in range(max(1, self.repeat)):
            self._pub.publish(out)
            rospy.sleep(self.repeat_dt)

        rospy.loginfo("Published initialpose from Gazebo: model=%s x=%.3f y=%.3f yaw=%.3f to %s",
                      self.model_name, pose.position.x, pose.position.y, yaw, self.initialpose_topic)

        self._done = True

if __name__ == "__main__":
    rospy.init_node("set_initialpose_from_gazebo")
    SetInitialPoseFromGazebo()
    rospy.spin()
