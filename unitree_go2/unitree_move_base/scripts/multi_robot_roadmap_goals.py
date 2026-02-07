#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多机器人基于 Roadmap 的目标点发送脚本
使用 roadmap 生成的路径图进行路径规划，然后通过 move_base_flex 的 ExePath action 执行路径

注意: 此脚本仅支持 move_base_flex，需要确保所有机器人都已启动 move_base_flex

使用方法:
    python3 multi_robot_roadmap_goals.py
    
参数:
    --robot_list: 机器人命名空间列表，用逗号分隔，例如: robot_0,robot_1
    --roadmap_topic: Roadmap topic 名称 (默认: /roadmap)
    --min_x, --max_x: X坐标范围 (默认: -25, 25)
    --min_y, --max_y: Y坐标范围 (默认: -15, 15)
    --interval: 发送目标点的间隔时间（秒，默认: 30.0）
    --random_yaw: 是否随机生成朝向角度 (默认: True)
    --use_roadmap_nodes: 是否从 roadmap 节点中选择目标点 (默认: True)
    --publish_path: 是否发布 roadmap 规划的路径 (默认: True)
"""

import rospy
import sys
import os
import math
import random
import argparse
import time
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


class MultiRobotRoadmapGoalSender:
    def __init__(self, robot_list=None, roadmap_topic='/roadmap', 
                 min_x=-10.0, max_x=10.0, min_y=-15.0, max_y=15.0, 
                 random_yaw=True, use_roadmap_nodes=True, publish_path=True):
        """
        初始化多机器人基于 Roadmap 的目标点发送器（仅使用 move_base_flex）
        
        参数:
            robot_list: 机器人命名空间列表，如果为None则自动检测
            roadmap_topic: Roadmap topic 名称
            min_x, max_x: X坐标范围
            min_y, max_y: Y坐标范围
            random_yaw: 是否随机生成朝向角度
            use_roadmap_nodes: 是否从 roadmap 节点中选择目标点
            publish_path: 是否发布 roadmap 规划的路径
        """
        rospy.init_node('multi_robot_roadmap_goals', anonymous=True)
        
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.random_yaw = random_yaw
        self.use_roadmap_nodes = use_roadmap_nodes
        self.publish_path = publish_path
        
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
                path_topic = '/' + robot_ns + '/roadmap_path'
                self.path_publishers[robot_ns] = rospy.Publisher(path_topic, Path, queue_size=1)
        
        rospy.loginfo("使用 move_base_flex ExePath action 执行 roadmap 路径")
        rospy.sleep(1.0)
    
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
        参考 motion_planner.py 的实现
        
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
            
            # 生成 Path 消息（参考 motion_planner.py 的 gen_path_goal_msg）
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
            
            # 计算终点的朝向（参考 motion_planner.py）
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
            
            # 构造 ExePathGoal 消息（参考 motion_planner.py）
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
    
    def generate_goal_from_roadmap(self):
        """
        从 roadmap 节点中随机选择一个目标点
        如果 use_roadmap_nodes=False，则随机生成目标点
        """
        if self.use_roadmap_nodes and self.roadmap_msg and len(self.roadmap_msg.nodes) > 0:
            # 从 roadmap 节点中随机选择
            node = random.choice(self.roadmap_msg.nodes)
            x = node.point.x
            y = node.point.y
            rospy.loginfo("从 roadmap 节点中选择目标点: (%.2f, %.2f)", x, y)
        else:
            # 随机生成目标点
            x = random.uniform(self.min_x, self.max_x)
            y = random.uniform(self.min_y, self.max_y)
            rospy.loginfo("随机生成目标点: (%.2f, %.2f)", x, y)
        
        if self.random_yaw:
            yaw = random.uniform(-math.pi, math.pi)
        else:
            yaw = 0.0
        
        return (x, y, yaw)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    
    def send_goal_to_robot(self, robot_ns, x, y, yaw=0.0, frame_id='map'):
        """
        向指定机器人发送目标点，使用 roadmap 规划路径并通过 move_base_flex 执行
        
        参数:
            robot_ns: 机器人命名空间
            x, y: 目标坐标
            yaw: 目标朝向角度（弧度）
            frame_id: 坐标系ID
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
        
        # 使用 roadmap 规划路径（参考 robot_controller_node.py 的实现）
        if self.roadmap_available and self.roadmap_graph is not None:
            try:
                path_msg, goal_msg = self._get_shortest_path(curr_pos, goal_pos)
                
                if path_msg and goal_msg:
                    # 发布路径用于可视化（参考 robot_controller_node.py）
                    if self.publish_path and robot_ns in self.path_publishers:
                        self.path_publishers[robot_ns].publish(path_msg)
                        rospy.loginfo("[%s] 发布 roadmap 规划路径，包含 %d 个点", 
                                    robot_ns, len(path_msg.poses))
                    
                    rospy.loginfo("[%s] 使用 roadmap 规划路径: 从 (%.2f, %.2f) 到 (%.2f, %.2f), 路径包含 %d 个点",
                                robot_ns, curr_pos[0], curr_pos[1], x, y, len(path_msg.poses))
                    
                    # ========== 以下代码用于在终端输出规划的路径，可以随时注释掉 ==========
                    print("\n" + "="*80)
                    print(f"[{robot_ns}] Roadmap 规划路径详情:")
                    print(f"  起点: ({curr_pos[0]:.3f}, {curr_pos[1]:.3f})")
                    print(f"  终点: ({x:.3f}, {y:.3f})")
                    print(f"  路径点数: {len(path_msg.poses)}")
                    print(f"  路径点列表:")
                    for i, pose_stamped in enumerate(path_msg.poses):
                        pos = pose_stamped.pose.position
                        ori = pose_stamped.pose.orientation
                        yaw_angle = math.atan2(2*(ori.w*ori.z + ori.x*ori.y), 1-2*(ori.y*ori.y + ori.z*ori.z))
                        print(f"    [{i:2d}] ({pos.x:7.3f}, {pos.y:7.3f}) yaw={math.degrees(yaw_angle):6.1f}°")
                    print("="*80 + "\n")
                    # ========== 以上代码用于在终端输出规划的路径，可以随时注释掉 ==========
                    
                    # 发送 ExePathGoal 给 move_base_flex ExePath action（参考 robot_controller_node.py）
                    try:
                        # goal_msg 已经在 _get_shortest_path 中构造好了（参考 motion_planner.py）
                        self.mbf_clients[robot_ns].send_goal(goal_msg)
                        rospy.loginfo("[%s] 通过 MBF ExePath 发送 roadmap 路径，机器人将沿着此路径移动", robot_ns)
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
    
    def send_random_goals_to_all(self):
        """为所有机器人发送随机目标点（基于 roadmap）"""
        for robot_ns in self.robot_list:
            x, y, yaw = self.generate_goal_from_roadmap()
            self.send_goal_to_robot(robot_ns, x, y, yaw)
    
    def run_continuous(self, interval=30.0):
        """
        持续运行，定期为所有机器人发送随机目标点
        
        参数:
            interval: 发送间隔（秒）
        """
        rospy.loginfo("开始持续发送基于 roadmap 的随机目标点，间隔: %.1f 秒", interval)
        rospy.loginfo("地图范围: x [%.1f, %.1f], y [%.1f, %.1f]", 
                     self.min_x, self.max_x, self.min_y, self.max_y)
        rospy.loginfo("使用 roadmap 节点: %s", "是" if self.use_roadmap_nodes else "否")
        
        count = 0
        rate = rospy.Rate(1.0 / interval)
        
        while not rospy.is_shutdown():
            count += 1
            rospy.loginfo("\n=== 第 %d 轮发送基于 roadmap 的目标点 ===", count)
            self.send_random_goals_to_all()
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(description='多机器人基于 Roadmap 的目标点发送脚本')
    parser.add_argument('--robot_list', type=str, default=None,
                       help='机器人命名空间列表，用逗号分隔，例如: robot_0,robot_1')
    parser.add_argument('--roadmap_topic', type=str, default='/roadmap',
                       help='Roadmap topic 名称 (默认: /roadmap)')
    parser.add_argument('--min_x', type=float, default=-10.0,
                       help='X坐标最小值 (默认: -10.0)')
    parser.add_argument('--max_x', type=float, default=10.0,
                       help='X坐标最大值 (默认: 10.0)')
    parser.add_argument('--min_y', type=float, default=-15.0,
                       help='Y坐标最小值 (默认: -15.0)')
    parser.add_argument('--max_y', type=float, default=15.0,
                       help='Y坐标最大值 (默认: 15.0)')
    parser.add_argument('--interval', type=float, default=30.0,
                       help='发送目标点的间隔时间 (秒，默认: 30.0)')
    parser.add_argument('--random_yaw', type=bool, default=True,
                       help='是否随机生成朝向角度 (默认: True)')
    parser.add_argument('--use_roadmap_nodes', type=bool, default=True,
                       help='是否从 roadmap 节点中选择目标点 (默认: True)')
    parser.add_argument('--publish_path', type=bool, default=True,
                       help='是否发布 roadmap 规划的路径 (默认: True)')
    parser.add_argument('--once', action='store_true',
                       help='只发送一次目标点，不持续运行')
    
    args = parser.parse_args()
    
    robot_list = None
    if args.robot_list:
        robot_list = [ns.strip() for ns in args.robot_list.split(',')]
    
    try:
        sender = MultiRobotRoadmapGoalSender(
            robot_list=robot_list,
            roadmap_topic=args.roadmap_topic,
            min_x=args.min_x,
            max_x=args.max_x,
            min_y=args.min_y,
            max_y=args.max_y,
            random_yaw=args.random_yaw,
            use_roadmap_nodes=args.use_roadmap_nodes,
            publish_path=args.publish_path
        )
        
        if args.once:
            rospy.loginfo("发送一次基于 roadmap 的目标点...")
            sender.send_random_goals_to_all()
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

