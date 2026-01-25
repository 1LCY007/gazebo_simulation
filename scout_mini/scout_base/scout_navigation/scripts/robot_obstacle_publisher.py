#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人障碍物点云发布器

功能：
- 订阅 /gazebo/model_states 获取所有机器人位置
- 计算其他机器人与当前机器人的距离
- 当距离较近时，发布 PointCloud2 消息，包含机器人 footprint 的点
- 将点云发布到 local_costmap 的 observation source

使用方法：
rosrun scout_navigation robot_obstacle_publisher.py _robot_namespace:=robot_1 _distance_threshold:=3.0
"""

import rospy
import math
import threading
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point
import tf2_ros
import tf2_geometry_msgs

class RobotObstaclePublisher:
    def __init__(self):
        rospy.init_node('robot_obstacle_publisher', anonymous=True)
        
        # 获取参数
        self.robot_namespace = rospy.get_param('~robot_namespace', 'robot_1')
        # 确保命名空间格式正确（去掉前导斜杠，因为gazebo模型名称通常不带斜杠）
        if self.robot_namespace.startswith('/'):
            self.robot_namespace = self.robot_namespace[1:]
        
        self.distance_threshold = rospy.get_param('~distance_threshold', 3.0)  # 距离阈值（米）
        self.robot_radius = rospy.get_param('~robot_radius', 0.4)  # 机器人半径（米）
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # 发布频率（Hz）
        
        # 机器人footprint（从costmap_common_params.yaml获取，默认scout的footprint）
        # footprint: [[-0.32, -0.31], [-0.32, 0.31], [0.32, 0.31], [0.32, -0.31]]
        self.footprint_points = rospy.get_param('~footprint', 
            [[-0.32, -0.31], [-0.32, 0.31], [0.32, 0.31], [0.32, -0.31]])
        
        # 获取所有机器人命名空间（用于过滤）
        robot_namespaces_str = rospy.get_param('~robot_namespaces', 
            'robot_1,robot_2,robot_3,robot_4,robot_5,robot_6')
        self.robot_namespaces = [ns.strip() for ns in robot_namespaces_str.split(',')]
        # 移除当前机器人
        if self.robot_namespace in self.robot_namespaces:
            self.robot_namespaces.remove(self.robot_namespace)
        
        # 获取每个机器人的 footprint 配置
        # 格式：robot_footprints: "{robot_1: [[-0.32, -0.31], [-0.32, 0.31], [0.32, 0.31], [0.32, -0.31]], robot_5: [[0.3, 0.15], [0.3, -0.15], [-0.35, -0.15], [-0.35, 0.15]]}"
        # 或者通过参数直接配置字典
        self.robot_footprints = {}
        
        # 尝试从参数获取 robot_footprints 字典
        try:
            robot_footprints_param = rospy.get_param('~robot_footprints', {})
            if isinstance(robot_footprints_param, dict):
                self.robot_footprints = robot_footprints_param
        except:
            pass
        
        # 如果没有配置，使用默认值：根据命名空间判断
        # robot_1-4 通常是 Scout，robot_5-6 通常是 Go2
        if not self.robot_footprints:
            # Scout footprint: [[-0.32, -0.31], [-0.32, 0.31], [0.32, 0.31], [0.32, -0.31]]
            scout_footprint = [[-0.32, -0.31], [-0.32, 0.31], [0.32, 0.31], [0.32, -0.31]]
            # Go2 footprint: [[0.3, 0.15], [0.3, -0.15], [-0.35, -0.15], [-0.35, 0.15]]
            go2_footprint = [[0.3, 0.15], [0.3, -0.15], [-0.35, -0.15], [-0.35, 0.15]]
            
            for ns in self.robot_namespaces + [self.robot_namespace]:
                # 根据命名空间判断：robot_5 和 robot_6 通常是 Go2
                if ns in ['robot_5', 'robot_6']:
                    self.robot_footprints[ns] = go2_footprint
                else:
                    # 默认使用 Scout footprint
                    self.robot_footprints[ns] = scout_footprint
        
        rospy.loginfo(f"Robot footprints: {self.robot_footprints}")
        
        rospy.loginfo(f"Robot Obstacle Publisher initialized for {self.robot_namespace}")
        rospy.loginfo(f"Monitoring robots: {self.robot_namespaces}")
        rospy.loginfo(f"Distance threshold: {self.distance_threshold} m")
        rospy.loginfo(f"Publish rate: {self.publish_rate} Hz")
        rospy.loginfo(f"Publishing to local costmap topic: {rospy.resolve_name('robot_obstacles')}")
        rospy.loginfo(f"Publishing to global costmap topic: {rospy.resolve_name('robot_obstacles_global')}")
        
        # TF buffer 和 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 订阅 gazebo model states
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', 
                                                 ModelStates, 
                                                 self.model_states_callback)
        
        # 发布 PointCloud2（用于local_costmap，odom坐标系）
        self.pointcloud_pub_local = rospy.Publisher('robot_obstacles', 
                                                     PointCloud2, 
                                                     queue_size=1)
        # 发布 PointCloud2（用于global_costmap，map坐标系）
        self.pointcloud_pub_global = rospy.Publisher('robot_obstacles_global', 
                                                     PointCloud2, 
                                                     queue_size=1)
        
        # 存储最新的模型状态
        self.model_states = None
        self.model_states_lock = threading.Lock()
        
        # 定时发布点云
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), 
                                 self.publish_obstacles)
        
        rospy.loginfo("Robot Obstacle Publisher started")
    
    def model_states_callback(self, msg):
        """处理 gazebo model states 消息"""
        with self.model_states_lock:
            self.model_states = msg
    
    def get_robot_pose_from_model_states(self, robot_name):
        """从 model_states 中获取指定机器人的位姿"""
        if self.model_states is None:
            return None
        
        try:
            # 查找机器人模型（gazebo中的模型名称可能包含命名空间）
            # 例如：robot_1, robot_1_gazebo 等
            robot_index = None
            for i, name in enumerate(self.model_states.name):
                if name == robot_name or name.startswith(robot_name + '_'):
                    robot_index = i
                    break
            
            if robot_index is None:
                return None
            
            return self.model_states.pose[robot_index]
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Error getting pose for {robot_name}: {e}")
            return None
    
    def calculate_distance(self, pose1, pose2):
        """计算两个位姿之间的2D距离"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def footprint_to_pointcloud(self, pose, footprint_points):
        """将机器人footprint转换为点云点（在odom坐标系中）"""
        points = []
        
        # 获取机器人的位置和朝向
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        
        # 计算朝向角度（从四元数）
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        
        # 转换为欧拉角（只需要yaw）
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 
                         1.0 - 2.0 * (qy * qy + qz * qz))
        
        # 旋转和平移footprint点
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        for fp_point in footprint_points:
            # 旋转
            rx = fp_point[0] * cos_yaw - fp_point[1] * sin_yaw
            ry = fp_point[0] * sin_yaw + fp_point[1] * cos_yaw
            
            # 平移
            px = x + rx
            py = y + ry
            pz = z  # 保持z坐标
            
            points.append([px, py, pz])
        
        return points
    
    def create_pointcloud2(self, points, frame_id='odom'):
        """创建 PointCloud2 消息"""
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        
        # 创建 PointCloud2
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        
        # 定义字段
        cloud.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        cloud.point_step = 12  # 3 * 4 bytes
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        
        # 填充点数据
        cloud.data = []
        for point in points:
            # 将点转换为字节
            import struct
            cloud.data += struct.pack('fff', point[0], point[1], point[2])
        
        return cloud
    
    def publish_obstacles(self, event):
        """定时发布障碍物点云"""
        if self.model_states is None:
            return
        
        # 获取当前机器人的位姿
        current_pose = None
        with self.model_states_lock:
            current_pose = self.get_robot_pose_from_model_states(self.robot_namespace)
        
        if current_pose is None:
            rospy.logwarn_throttle(5.0, f"Could not find pose for {self.robot_namespace}")
            return
        
        # 收集附近机器人的点
        all_obstacle_points = []
        nearby_robots_info = []
        
        with self.model_states_lock:
            for other_robot_ns in self.robot_namespaces:
                other_pose = self.get_robot_pose_from_model_states(other_robot_ns)
                
                if other_pose is None:
                    rospy.logdebug_throttle(5.0, f"Could not find pose for robot {other_robot_ns} in model_states")
                    continue
                
                # 计算距离
                distance = self.calculate_distance(current_pose, other_pose)
                
                if distance <= self.distance_threshold:
                    # 根据机器人类型使用不同的 footprint
                    other_footprint = self.robot_footprints.get(
                        other_robot_ns, 
                        [[-0.5, -0.5], [-0.5, 0.5], [0.5, 0.5], [0.5, -0.5]]  # 默认 1m x 1m
                    )
                    
                    # 将其他机器人的footprint转换为点云点
                    obstacle_points = self.footprint_to_pointcloud(other_pose, other_footprint)
                    all_obstacle_points.extend(obstacle_points)
                    nearby_robots_info.append((other_robot_ns, distance, len(obstacle_points)))
                    
                    rospy.loginfo_throttle(1.0, 
                        f"[{self.robot_namespace}] Robot {other_robot_ns} is {distance:.2f}m away, using footprint with {len(other_footprint)} points, added {len(obstacle_points)} points to costmap")
        
        # 只有当检测到附近机器人时才发布点云
        if len(all_obstacle_points) > 0:
            # 获取frame_id（tf2要求frame_id不能以'/'开头）
            if self.robot_namespace and self.robot_namespace != '/' and self.robot_namespace != '':
                ns = self.robot_namespace.lstrip('/')
                odom_frame_id = f"{ns}/odom"
            else:
                odom_frame_id = 'odom'
            
            # 发布到local_costmap（odom坐标系）
            cloud_local = self.create_pointcloud2(all_obstacle_points, odom_frame_id)
            self.pointcloud_pub_local.publish(cloud_local)
            
            # 发布到global_costmap（map坐标系）
            # 注意：model_states中的位姿是在world坐标系中的，我们需要转换到map坐标系
            # 如果map和world是同一个坐标系，可以直接使用map
            map_frame_id = 'map'
            cloud_global = self.create_pointcloud2(all_obstacle_points, map_frame_id)
            self.pointcloud_pub_global.publish(cloud_global)
            
            # 记录附近机器人信息
            nearby_info = [f"{info[0]}({info[1]:.2f}m, {info[2]}pts)" for info in nearby_robots_info]
            
            rospy.loginfo_throttle(2.0, 
                f"[{self.robot_namespace}] Published {len(all_obstacle_points)} obstacle points to local and global costmaps from {len(nearby_robots_info)} nearby robot(s): {nearby_info}")
        # 如果没有附近机器人，不发布点云（按需运行）

if __name__ == '__main__':
    try:
        publisher = RobotObstaclePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

