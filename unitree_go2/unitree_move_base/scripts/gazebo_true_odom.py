#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
import math

def pose_to_mat(p):
    q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    t = [p.position.x, p.position.y, p.position.z]
    M = tft.quaternion_matrix(q)
    M[0, 3], M[1, 3], M[2, 3] = t
    return M

def tf_to_mat(trans, rot):
    q = [rot.x, rot.y, rot.z, rot.w]
    t = [trans.x, trans.y, trans.z]
    M = tft.quaternion_matrix(q)
    M[0, 3], M[1, 3], M[2, 3] = t
    return M

def mat_to_tf(M):
    t = (M[0, 3], M[1, 3], M[2, 3])
    q = tft.quaternion_from_matrix(M)
    return t, q

def yaw_from_quat(q):
    return tft.euler_from_quaternion(q)[2]

class GazeboTruthStaticMapOdom:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "go2_gazebo")
        self.map_frame  = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.rate = float(rospy.get_param("~rate", 10.0))

        # 变化阈值：超过才更新静态 TF
        self.trans_eps = float(rospy.get_param("~trans_eps", 0.01))  # 1cm
        self.yaw_eps   = float(rospy.get_param("~yaw_eps", 0.01))    # ~0.57deg

        self.truth_pose = None
        self._last_t = None
        self._last_yaw = None
        self._initial_odom_base = None  # 记录初始 odom->base 变换
        self._map_odom_rotation_fixed = None  # 固定的 map->odom 旋转
        self._map_odom_initialized = False  # 标记是否已初始化

        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.br_static = tf2_ros.StaticTransformBroadcaster()

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._cb, queue_size=1)

        rospy.loginfo("static_map_odom: model=%s map=%s odom=%s base=%s rate=%.1f",
                      self.model_name, self.map_frame, self.odom_frame, self.base_frame, self.rate)

    def _cb(self, msg: ModelStates):
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        self.truth_pose = msg.pose[idx]

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.truth_pose is None:
                r.sleep()
                continue

            # 获取最新 odom->base
            try:
                tf_ob = self.tf_buf.lookup_transform(self.odom_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.2))
            except Exception as e:
                r.sleep()
                continue

            # 如果是第一次，记录初始 odom->base 变换，并计算初始 map->odom（包括旋转和平移）
            if not self._map_odom_initialized:
                self._initial_odom_base = tf_to_mat(tf_ob.transform.translation, tf_ob.transform.rotation)
                T_map_base = pose_to_mat(self.truth_pose)
                T_map_odom = T_map_base.dot(tft.inverse_matrix(self._initial_odom_base))
                t_init, q_init = mat_to_tf(T_map_odom)
                self._map_odom_rotation_fixed = q_init
                self._last_t = t_init
                
                # 发布初始 map->odom TF（包括旋转和平移，传递初始朝向）
                out = TransformStamped()
                out.header.stamp = rospy.Time.now()
                out.header.frame_id = self.map_frame
                out.child_frame_id = self.odom_frame
                out.transform.translation.x = t_init[0]
                out.transform.translation.y = t_init[1]
                out.transform.translation.z = t_init[2]
                out.transform.rotation.x = q_init[0]
                out.transform.rotation.y = q_init[1]
                out.transform.rotation.z = q_init[2]
                out.transform.rotation.w = q_init[3]
                
                self.br_static.sendTransform(out)
                self._map_odom_initialized = True
                
                yaw_init = yaw_from_quat(q_init)
                rospy.loginfo("Initialized map->odom TF from Gazebo: x=%.3f y=%.3f yaw=%.3f", 
                             t_init[0], t_init[1], yaw_init)
                r.sleep()
                continue

            # 之后只更新平移部分，保持旋转固定（避免 costmap 旋转）
            T_map_base = pose_to_mat(self.truth_pose)
            T_odom_base = tf_to_mat(tf_ob.transform.translation, tf_ob.transform.rotation)
            T_map_odom = T_map_base.dot(tft.inverse_matrix(T_odom_base))
            t, _ = mat_to_tf(T_map_odom)

            # 阈值判断：变化小就不更新
            if self._last_t is not None:
                dx = t[0] - self._last_t[0]
                dy = t[1] - self._last_t[1]
                dist = math.hypot(dx, dy)
                if dist < self.trans_eps:
                    r.sleep()
                    continue

            self._last_t = t

            # 发布 map->odom，使用固定的旋转
            out = TransformStamped()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = self.map_frame
            out.child_frame_id = self.odom_frame
            out.transform.translation.x = t[0]
            out.transform.translation.y = t[1]
            out.transform.translation.z = t[2]
            out.transform.rotation.x = self._map_odom_rotation_fixed[0]
            out.transform.rotation.y = self._map_odom_rotation_fixed[1]
            out.transform.rotation.z = self._map_odom_rotation_fixed[2]
            out.transform.rotation.w = self._map_odom_rotation_fixed[3]

            self.br_static.sendTransform(out)
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("gazebo_true_static_map_odom")
    GazeboTruthStaticMapOdom().spin()
