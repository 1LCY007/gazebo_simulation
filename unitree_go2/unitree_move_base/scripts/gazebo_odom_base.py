#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 Gazebo ground truth 发布 odom -> base TF

功能：
- 订阅 /gazebo/model_states 获取机器人实际位置
- 使用固定的 map -> odom TF（由静态 TF 发布器设置）
- 计算并发布 odom -> base TF

这样 TF 树就是：
- map -> odom：静态 TF（固定）
- odom -> base：从 Gazebo ground truth 计算
"""

import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
import math

def pose_to_mat(p):
    """将 Pose 转换为 4x4 变换矩阵"""
    q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
    t = [p.position.x, p.position.y, p.position.z]
    M = tft.quaternion_matrix(q)
    M[0, 3], M[1, 3], M[2, 3] = t
    return M

def mat_to_tf(M):
    """将 4x4 变换矩阵转换为平移和四元数"""
    t = (M[0, 3], M[1, 3], M[2, 3])
    q = tft.quaternion_from_matrix(M)
    return t, q

class GazeboOdomBase:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "go2_gazebo")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.rate = float(rospy.get_param("~rate", 100.0))

        self.truth_pose = None

        self.tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)
        self.br = tf2_ros.TransformBroadcaster()

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._cb, queue_size=1)

        rospy.loginfo("gazebo_odom_base: model=%s map=%s odom=%s base=%s rate=%.1f",
                      self.model_name, self.map_frame, self.odom_frame, self.base_frame, self.rate)

    def _cb(self, msg: ModelStates):
        """处理 gazebo model states 消息"""
        try:
            idx = msg.name.index(self.model_name)
        except ValueError:
            return
        self.truth_pose = msg.pose[idx]

    def spin(self):
        """主循环：计算并发布 odom -> base TF"""
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.truth_pose is None:
                r.sleep()
                continue

            # 获取固定的 map -> odom TF（由静态 TF 发布器设置）
            try:
                tf_map_odom = self.tf_buf.lookup_transform(
                    self.map_frame, self.odom_frame, rospy.Time(0), rospy.Duration(0.2))
            except Exception as e:
                rospy.logdebug_throttle(5.0, "Waiting for TF %s -> %s: %s", 
                                       self.map_frame, self.odom_frame, str(e))
                r.sleep()
                continue

            # Gazebo ground truth 给出 map -> base（world 坐标系中的位置）
            T_map_base = pose_to_mat(self.truth_pose)

            # 将 map -> odom TF 转换为矩阵
            q_map_odom = [
                tf_map_odom.transform.rotation.x,
                tf_map_odom.transform.rotation.y,
                tf_map_odom.transform.rotation.z,
                tf_map_odom.transform.rotation.w
            ]
            t_map_odom = [
                tf_map_odom.transform.translation.x,
                tf_map_odom.transform.translation.y,
                tf_map_odom.transform.translation.z
            ]
            T_map_odom = tft.quaternion_matrix(q_map_odom)
            T_map_odom[0, 3], T_map_odom[1, 3], T_map_odom[2, 3] = t_map_odom

            # 计算 odom -> base = inv(map -> odom) * (map -> base)
            T_odom_base = tft.inverse_matrix(T_map_odom).dot(T_map_base)
            t_odom_base, q_odom_base = mat_to_tf(T_odom_base)

            # 发布 odom -> base TF
            out = TransformStamped()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = self.odom_frame
            out.child_frame_id = self.base_frame
            out.transform.translation.x = t_odom_base[0]
            out.transform.translation.y = t_odom_base[1]
            out.transform.translation.z = t_odom_base[2]
            out.transform.rotation.x = q_odom_base[0]
            out.transform.rotation.y = q_odom_base[1]
            out.transform.rotation.z = q_odom_base[2]
            out.transform.rotation.w = q_odom_base[3]

            self.br.sendTransform(out)
            
            # 调试日志（每5秒输出一次）
            rospy.logdebug_throttle(5.0, 
                "Published odom->base TF: x=%.3f y=%.3f z=%.3f (from Gazebo ground truth)",
                t_odom_base[0], t_odom_base[1], t_odom_base[2])
            
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("gazebo_odom_base")
    GazeboOdomBase().spin()

