#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多机器人基于医院房间位置的目标点发送脚本
从 hospital.yaml 文件中读取房间位置，随机选择房间作为目标点
使用 roadmap 生成的路径图进行路径规划，然后通过 move_base_flex 的 ExePath action 执行路径

注意: 此脚本仅支持 move_base_flex，需要确保所有机器人都已启动 move_base_flex

使用方法:
    python3 multi_robot_room_goals.py
    
参数:
    --robot_list: 机器人命名空间列表，用逗号分隔，例如: robot_0,robot_1
    --roadmap_topic: Roadmap topic 名称 (默认: /roadmap)
    --hospital_yaml: hospital.yaml 文件路径 (默认: config/hospital.yaml)
    --interval: 发送目标点的间隔时间（秒，默认: 30.0）
    --random_yaw: 是否随机生成朝向角度 (默认: True)
    --publish_path: 是否发布 roadmap 规划的路径 (默认: True)
    --room_types: 房间类型过滤，用逗号分隔，例如: room_patient,room_surgery (默认: 所有房间)
"""

import rospy
import sys
import os
import math
import random
import argparse
import time
import yaml
import networkx as nx
import actionlib
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Path, Odometry
from gazebo_msgs.msg import ModelStates
from tf.transformations import quaternion_from_euler
import tf.transformations

from roadmap_builder.msg import RoadMap, RoadMapNode, RoadMapEdge

# 必须导入 mbf_msgs，如果不存在则退出
try:
    from mbf_msgs.msg import ExePathAction, ExePathGoal
except ImportError:
    print("错误: mbf_msgs 不可用！此脚本仅支持 move_base_flex。")
    print("请确保已安装 move_base_flex 包。")
    sys.exit(1)


def distance(ps, pt):
    """计算两点之间的欧几里得距离"""
    return round(math.sqrt((pt[0]-ps[0])**2 + (pt[1]-ps[1])**2), 3)


class MultiRobotRoomGoalSender:
    def __init__(self, robot_list=None, roadmap_topic='/roadmap', 
                 hospital_yaml=None, random_yaw=True, publish_path=True,
                 room_types=None):
        """
        初始化多机器人基于房间位置的目标点发送器（仅使用 move_base_flex）
        
        参数:
            robot_list: 机器人命名空间列表，如果为None则自动检测
            roadmap_topic: Roadmap topic 名称
            hospital_yaml: hospital.yaml 文件路径
            random_yaw: 是否随机生成朝向角度
            publish_path: 是否发布 roadmap 规划的路径
            room_types: 房间类型过滤列表，如果为None则使用所有房间
        """
        rospy.init_node('multi_robot_room_goals', anonymous=True)
        
        self.random_yaw = random_yaw
        self.publish_path = publish_path
        
        # 加载房间位置
        if hospital_yaml is None:
            # 默认路径：相对于脚本所在目录的 config/hospital.yaml
            script_dir = os.path.dirname(os.path.abspath(__file__))
            hospital_yaml = os.path.join(script_dir, '..', 'config', 'hospital.yaml')
        
        self.room_positions = self._load_room_positions(hospital_yaml, room_types)
        if not self.room_positions:
            rospy.logerr("未找到任何房间位置！请检查 hospital.yaml 文件路径和格式。")
            sys.exit(1)
        
        rospy.loginfo("成功加载 %d 个房间位置: %s", 
                     len(self.room_positions), 
                     ', '.join(self.room_positions.keys()))
        
        # Roadmap 相关
        self.roadmap_msg = None
        self.roadmap_graph = None  # networkx Graph
        self.roadmap_available = False
        
        # TF 监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 订阅 roadmap
        rospy.loginfo("等待 roadmap 消息 (topic: %s)...", roadmap_topic)
        rospy.Subscriber(roadmap_topic, RoadMap, self.roadmap_callback)
        
        # 等待 roadmap 消息
        rospy.loginfo("等待 roadmap 消息...")
        timeout = rospy.Duration(10.0)
        start_time = rospy.Time.now()
        while self.roadmap_msg is None and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)
        
        if self.roadmap_msg is None:
            rospy.logerr("未收到 roadmap 消息！请确保 roadmap_builder 正在运行并发布到 %s", roadmap_topic)
            sys.exit(1)
        
        rospy.loginfo("成功接收到 roadmap 消息，包含 %d 个节点", len(self.roadmap_msg.nodes))
        
        # 检测机器人
        if robot_list is None:
            self.robot_list = self._detect_robots()
        else:
            self.robot_list = robot_list
        
        if not self.robot_list:
            rospy.logerr("未检测到任何机器人！请确保机器人已启动。")
            sys.exit(1)
        
        rospy.loginfo("检测到 %d 个机器人: %s", len(self.robot_list), ', '.join(self.robot_list))
        
        # 创建发布者和 action clients
        self.path_publishers = {}
        self.mbf_clients = {}  # move_base_flex ExePath action clients
        self.robot_positions = {}  # 存储每个机器人的当前位置（从 Gazebo 获取）
        
        # 订阅 Gazebo model_states 获取机器人真实位置
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_model_states_callback, queue_size=1)
        
        for robot_ns in self.robot_list:
            # move_base_flex ExePath action client（必需）
            action_topic = '/' + robot_ns + '/move_base_flex/exe_path'
            client = actionlib.SimpleActionClient(action_topic, ExePathAction)
            self.mbf_clients[robot_ns] = client
            rospy.loginfo("等待 MBF ExePath server: %s", action_topic)
            if not client.wait_for_server(rospy.Duration(10.0)):
                rospy.logerr("机器人 %s 的 MBF ExePath server 不可用！请确保 move_base_flex 已启动。", robot_ns)
                rospy.logerr("此脚本仅支持 move_base_flex，无法继续运行。")
                sys.exit(1)
            else:
                rospy.loginfo("已连接到机器人 %s 的 MBF ExePath server", robot_ns)
            
            # 路径发布者（用于可视化）
            if self.publish_path:
                path_topic = '/' + robot_ns + '/room_path'
                self.path_publishers[robot_ns] = rospy.Publisher(path_topic, Path, queue_size=1)
        
        rospy.loginfo("使用 move_base_flex ExePath action 执行 roadmap 路径")
        rospy.sleep(1.0)
    
    def _load_room_positions(self, yaml_path, room_types=None):
        """
        从 hospital.yaml 文件中加载房间位置
        
        参数:
            yaml_path: hospital.yaml 文件路径
            room_types: 房间类型过滤列表，如果为None则使用所有房间
        
        返回:
            dict: {房间名称: [x, y]} 的字典
        """
        room_positions = {}
        
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            if 'Regions' not in data:
                rospy.logerr("hospital.yaml 文件中未找到 'Regions' 部分")
                return room_positions
            
            regions = data['Regions']
            for room_name, room_data in regions.items():
                if 'pos' not in room_data:
                    continue
                
                # 如果指定了房间类型过滤，检查房间类型
                if room_types is not None:
                    labels = room_data.get('labels', [])
                    # 房间名称到类型的映射
                    room_type_map = {
                        'rp': 'room_patient',      # 病房
                        'ri': 'room_icu',           # ICU
                        'rc': 'room_consulation',   # 诊室
                        'rs': 'room_surgery',       # 手术室
                        'rl': 'room_lab',           # 化验室
                        'rw': 'room_waste',         # 废弃物处理室
                        'reception': 'reception',   # 挂号窗口
                        'exit': 'exit',             # 入口
                        'repo': 'repo',             # 仓库
                    }
                    
                    # 检查房间类型
                    matched = False
                    room_type_str = None
                    
                    # 根据房间名称前缀判断类型
                    for prefix, rtype in room_type_map.items():
                        if room_name.startswith(prefix):
                            room_type_str = rtype
                            break
                    
                    # 检查是否匹配指定的房间类型
                    if room_type_str:
                        for room_type in room_types:
                            if room_type == room_type_str or room_type in labels:
                                matched = True
                                break
                    else:
                        # 如果没有匹配的前缀，检查标签
                        for room_type in room_types:
                            if room_type in labels:
                                matched = True
                                break
                    
                    if not matched:
                        continue
                
                pos = room_data['pos']
                if isinstance(pos, list) and len(pos) >= 2:
                    room_positions[room_name] = [float(pos[0]), float(pos[1])]
                    rospy.loginfo("加载房间 %s: 位置 (%.2f, %.2f), 标签: %s", 
                                 room_name, pos[0], pos[1], room_data.get('labels', []))
        
        except FileNotFoundError:
            rospy.logerr("未找到 hospital.yaml 文件: %s", yaml_path)
        except yaml.YAMLError as e:
            rospy.logerr("解析 hospital.yaml 文件时出错: %s", str(e))
        except Exception as e:
            rospy.logerr("加载房间位置时出错: %s", str(e))
            import traceback
            traceback.print_exc()
        
        return room_positions
    
    def roadmap_callback(self, msg):
        """Roadmap 消息回调"""
        self.roadmap_msg = msg
        if not self.roadmap_available:
            try:
                self.roadmap_graph = self._gen_roadmap_graph(self.roadmap_msg)
                self.roadmap_available = True
                rospy.loginfo("Roadmap 已加载，包含 %d 个节点，%d 条边", 
                            len(self.roadmap_msg.nodes), len(self.roadmap_msg.edges))
            except Exception as e:
                rospy.logerr("创建 Roadmap Graph 失败: %s", str(e))
                import traceback
                traceback.print_exc()
    
    def _gen_roadmap_graph(self, roadmap_msg):
        """将 RoadMap 消息转换为 networkx Graph"""
        roadmap = nx.Graph()
        for node in roadmap_msg.nodes:
            roadmap.add_node((node.point.x, node.point.y))
        for edge in roadmap_msg.edges:
            roadmap.add_edge(
                (edge.source.x, edge.source.y),
                (edge.target.x, edge.target.y),
                dist=edge.dist,
            )
        return roadmap
    
    def _find_nearest_node(self, pos):
        """找到 roadmap 中距离给定位置最近的节点"""
        if self.roadmap_graph is None:
            return None, float('inf')
        min_dist = float('inf')
        nearest_node = None
        for node in self.roadmap_graph.nodes:
            dist = distance(pos, node)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node, min_dist
    
    def _get_shortest_path(self, curr_pos, goal_pos):
        """
        使用 roadmap 计算从当前位置到目标位置的最短路径
        
        返回:
            path_msg: Path 消息，包含规划的路径
            goal_msg: ExePathGoal 消息（如果使用 MBF），否则为 None
        """
        if self.roadmap_graph is None or len(self.roadmap_graph.nodes) == 0:
            rospy.logwarn("Roadmap graph 不可用，返回空路径")
            return None, None
        
        curr_node, curr_dist = self._find_nearest_node(curr_pos)
        goal_node, goal_dist = self._find_nearest_node(goal_pos)
        
        if curr_node is None or goal_node is None:
            rospy.logwarn("无法找到最近的节点，返回空路径")
            return None, None
        
        try:
            # 计算最短路径（只包含中间节点，不包含起点和终点）
            path_nodes = nx.shortest_path(self.roadmap_graph, curr_node, goal_node, weight='dist')
            
            # 生成 Path 消息
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = rospy.Time.now()
            
            # 添加起始点
            start_pose = PoseStamped()
            start_pose.header.frame_id = "map"
            start_pose.header.stamp = rospy.Time.now()
            start_pose.pose.position.x = curr_pos[0]
            start_pose.pose.position.y = curr_pos[1]
            start_pose.pose.orientation.w = 1
            path_msg.poses.append(start_pose)
            
            # 添加中间路径点（roadmap 节点）
            for node in path_nodes:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = node[0]
                pose.pose.position.y = node[1]
                pose.pose.orientation.w = 1
                path_msg.poses.append(pose)
            
            # 添加终点
            end_pose = PoseStamped()
            end_pose.header.frame_id = "map"
            end_pose.header.stamp = rospy.Time.now()
            end_pose.pose.position.x = goal_pos[0]
            end_pose.pose.position.y = goal_pos[1]
            
            # 计算终点的朝向
            if len(path_msg.poses) > 1:
                # 获取倒数第二个点
                last_pose = path_msg.poses[-1].pose
                dx = goal_pos[0] - last_pose.position.x
                dy = goal_pos[1] - last_pose.position.y
                # 计算朝向角度（弧度）
                yaw = math.atan2(dy, dx)
                # 将欧拉角转换为四元数
                quaternion = quaternion_from_euler(0, 0, yaw)
                end_pose.pose.orientation.x = quaternion[0]
                end_pose.pose.orientation.y = quaternion[1]
                end_pose.pose.orientation.z = quaternion[2]
                end_pose.pose.orientation.w = quaternion[3]
            else:
                # 如果没有路径点，默认朝向
                end_pose.pose.orientation.w = 1
            
            path_msg.poses.append(end_pose)
            
            # 构造 ExePathGoal 消息
            goal_msg = ExePathGoal()
            goal_msg.path = path_msg
            goal_msg.tolerance_from_action = True
            goal_msg.dist_tolerance = 0.3
            goal_msg.angle_tolerance = 3.14 / 18
            
            return path_msg, goal_msg
            
        except nx.NetworkXNoPath:
            rospy.logwarn("无法找到从 %s 到 %s 的路径", curr_node, goal_node)
            return None, None
        except Exception as e:
            rospy.logwarn("计算路径时出错: %s", str(e))
            return None, None
    
    def gazebo_model_states_callback(self, msg):
        """
        从 Gazebo model_states 获取机器人真实位置
        更新所有机器人的位置信息
        """
        for robot_ns in self.robot_list:
            # 模型名称通常是 robot_ns（例如 robot_0）
            model_name = robot_ns
            try:
                idx = msg.name.index(model_name)
                pose = msg.pose[idx]
                # 存储机器人在 map frame 中的真实位置（Gazebo 中的位置）
                self.robot_positions[robot_ns] = (
                    pose.position.x,
                    pose.position.y
                )
            except ValueError:
                # 模型名称不存在，跳过
                continue
    
    def get_robot_position(self, robot_ns):
        """
        获取机器人当前位置
        直接从 Gazebo model_states 获取真实位置
        """
        # 从 Gazebo 获取的位置（已存储在 robot_positions 中）
        if robot_ns in self.robot_positions:
            return self.robot_positions[robot_ns]
        
        # 如果 Gazebo 数据不可用，返回 None
        rospy.logwarn("无法从 Gazebo 获取机器人 %s 的位置", robot_ns)
        return None
    
    def _detect_robots(self):
        """自动检测所有可用的机器人"""
        robot_list = []
        rospy.loginfo("正在检测可用的机器人...")
        
        try:
            topics = [topic[0] for topic in rospy.get_published_topics()]
            
            # 方法1: 检查 move_base_simple/goal topic
            for topic in topics:
                if '/move_base_simple/goal' in topic:
                    parts = topic.split('/')
                    if len(parts) >= 3 and parts[1].startswith('robot_'):
                        robot_ns = parts[1]
                        if robot_ns not in robot_list:
                            robot_list.append(robot_ns)
                            rospy.loginfo("检测到机器人: %s (通过 move_base_simple/goal: %s)", robot_ns, topic)
            
            # 方法2: 检查 odom topic
            for topic in topics:
                if '/odom' in topic and topic.endswith('/odom'):
                    parts = topic.split('/')
                    if len(parts) >= 3 and parts[1].startswith('robot_'):
                        robot_ns = parts[1]
                        if robot_ns not in robot_list:
                            robot_list.append(robot_ns)
                            rospy.loginfo("检测到机器人: %s (通过 odom: %s)", robot_ns, topic)
            
            # 方法3: 检查 hokuyo/scan topic
            for topic in topics:
                if '/hokuyo/scan' in topic:
                    parts = topic.split('/')
                    if len(parts) >= 3 and parts[1].startswith('robot_'):
                        robot_ns = parts[1]
                        if robot_ns not in robot_list:
                            robot_list.append(robot_ns)
                            rospy.loginfo("检测到机器人: %s (通过 hokuyo/scan: %s)", robot_ns, topic)
                            
        except Exception as e:
            rospy.logwarn("检测topic时出错: %s", str(e))
        
        if not robot_list:
            rospy.logwarn("未通过topic检测到机器人，尝试使用默认列表")
            default_robots = ['robot_0', 'robot_1', 'robot_2', 'robot_3', 'robot_4', 'robot_5']
            for ns in default_robots:
                robot_list.append(ns)
                rospy.loginfo("使用默认机器人: %s", ns)
        
        return sorted(robot_list)
    
    def generate_goal_from_rooms(self):
        """
        从房间位置中随机选择一个目标点
        
        返回:
            (x, y, yaw, room_name): 目标坐标、朝向角度和房间名称
        """
        if not self.room_positions:
            rospy.logerr("没有可用的房间位置")
            return None
        
        # 随机选择一个房间
        room_name = random.choice(list(self.room_positions.keys()))
        pos = self.room_positions[room_name]
        x, y = pos[0], pos[1]
        
        rospy.loginfo("从房间位置中选择目标点: %s (%.2f, %.2f)", room_name, x, y)
        
        if self.random_yaw:
            yaw = random.uniform(-math.pi, math.pi)
        else:
            yaw = 0.0
        
        return (x, y, yaw, room_name)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    
    def send_goal_to_robot(self, robot_ns, x, y, yaw=0.0, frame_id='map', room_name=None):
        """
        向指定机器人发送目标点，使用 roadmap 规划路径并通过 move_base_flex 执行
        
        参数:
            robot_ns: 机器人命名空间
            x, y: 目标坐标
            yaw: 目标朝向角度（弧度）
            frame_id: 坐标系ID
            room_name: 房间名称（用于日志）
        """
        if robot_ns not in self.mbf_clients:
            rospy.logerr("机器人 %s 的 MBF client 不存在", robot_ns)
            return False
        
        # 获取机器人当前位置
        curr_pos = self.get_robot_position(robot_ns)
        if curr_pos is None:
            rospy.logwarn("无法获取机器人 %s 的当前位置，跳过", robot_ns)
            return False
        
        goal_pos = (x, y)
        
        # 使用 roadmap 规划路径
        if self.roadmap_available and self.roadmap_graph is not None:
            try:
                path_msg, goal_msg = self._get_shortest_path(curr_pos, goal_pos)
                
                if path_msg and goal_msg:
                    # 发布路径用于可视化
                    if self.publish_path and robot_ns in self.path_publishers:
                        self.path_publishers[robot_ns].publish(path_msg)
                        rospy.loginfo("[%s] 发布 roadmap 规划路径，包含 %d 个点", 
                                    robot_ns, len(path_msg.poses))
                    
                    room_info = f" (房间: {room_name})" if room_name else ""
                    rospy.loginfo("[%s] 使用 roadmap 规划路径: 从 (%.2f, %.2f) 到 (%.2f, %.2f)%s, 路径包含 %d 个点",
                                robot_ns, curr_pos[0], curr_pos[1], x, y, room_info, len(path_msg.poses))
                    
                    # 发送 ExePathGoal 给 move_base_flex ExePath action
                    try:
                        self.mbf_clients[robot_ns].send_goal(goal_msg)
                        rospy.loginfo("[%s] 通过 MBF ExePath 发送 roadmap 路径到房间 %s，机器人将沿着此路径移动", 
                                     robot_ns, room_name if room_name else "目标点")
                        return True
                    except Exception as e:
                        rospy.logerr("[%s] MBF ExePath 发送失败: %s", robot_ns, str(e))
                        return False
                else:
                    rospy.logerr("[%s] Roadmap 路径规划失败，无法发送目标点", robot_ns)
                    return False
            except Exception as e:
                rospy.logerr("[%s] Roadmap 路径规划失败: %s", robot_ns, str(e))
                return False
        else:
            rospy.logerr("[%s] Roadmap 不可用，无法发送目标点", robot_ns)
            return False
    
    def send_room_goals_to_all(self):
        """为所有机器人发送房间目标点（基于 hospital.yaml）"""
        for robot_ns in self.robot_list:
            goal = self.generate_goal_from_rooms()
            if goal:
                x, y, yaw, room_name = goal
                self.send_goal_to_robot(robot_ns, x, y, yaw, room_name=room_name)
    
    def run_continuous(self, interval=30.0):
        """
        持续运行，定期为所有机器人发送房间目标点
        
        参数:
            interval: 发送间隔（秒）
        """
        rospy.loginfo("开始持续发送基于房间位置的目标点，间隔: %.1f 秒", interval)
        rospy.loginfo("可用房间数量: %d", len(self.room_positions))
        rospy.loginfo("房间列表: %s", ', '.join(self.room_positions.keys()))
        
        count = 0
        rate = rospy.Rate(1.0 / interval)
        
        while not rospy.is_shutdown():
            count += 1
            rospy.loginfo("\n=== 第 %d 轮发送基于房间位置的目标点 ===", count)
            self.send_room_goals_to_all()
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(description='多机器人基于医院房间位置的目标点发送脚本')
    parser.add_argument('--robot_list', type=str, default=None,
                       help='机器人命名空间列表，用逗号分隔，例如: robot_0,robot_1')
    parser.add_argument('--roadmap_topic', type=str, default='/roadmap',
                       help='Roadmap topic 名称 (默认: /roadmap)')
    parser.add_argument('--hospital_yaml', type=str, default=None,
                       help='hospital.yaml 文件路径 (默认: config/hospital.yaml)')
    parser.add_argument('--interval', type=float, default=30.0,
                       help='发送目标点的间隔时间 (秒，默认: 30.0)')
    parser.add_argument('--random_yaw', type=bool, default=True,
                       help='是否随机生成朝向角度 (默认: True)')
    parser.add_argument('--publish_path', type=bool, default=True,
                       help='是否发布 roadmap 规划的路径 (默认: True)')
    parser.add_argument('--room_types', type=str, default=None,
                       help='房间类型过滤，用逗号分隔，例如: room_patient,room_surgery (默认: 所有房间)')
    parser.add_argument('--once', action='store_true',
                       help='只发送一次目标点，不持续运行')
    
    args = parser.parse_args()
    
    robot_list = None
    if args.robot_list:
        robot_list = [ns.strip() for ns in args.robot_list.split(',')]
    
    room_types = None
    if args.room_types:
        room_types = [rt.strip() for rt in args.room_types.split(',')]
    
    try:
        sender = MultiRobotRoomGoalSender(
            robot_list=robot_list,
            roadmap_topic=args.roadmap_topic,
            hospital_yaml=args.hospital_yaml,
            random_yaw=args.random_yaw,
            publish_path=args.publish_path,
            room_types=room_types
        )
        
        if args.once:
            rospy.loginfo("发送一次基于房间位置的目标点...")
            sender.send_room_goals_to_all()
            rospy.loginfo("目标点已发送完成")
        else:
            sender.run_continuous(interval=args.interval)
            
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except KeyboardInterrupt:
        rospy.loginfo("用户中断程序")
    except Exception as e:
        rospy.logerr("发生错误: %s", str(e))
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

