#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多机器人随机目标点发送脚本
自动检测所有机器人，并为每个机器人随机发送目标点

使用方法:
    python3 multi_robot_random_goals.py
    
参数:
    --robot_list: 机器人命名空间列表，用逗号分隔，例如: robot_1,robot_2
    --min_x, --max_x: X坐标范围 (默认: -25, 25)
    --min_y, --max_y: Y坐标范围 (默认: -15, 15)
    --interval: 发送目标点的间隔时间（秒，默认: 30.0）
    --random_yaw: 是否随机生成朝向角度 (默认: True)
"""

import rospy
import sys
import math
import random
import argparse
import time
from geometry_msgs.msg import PoseStamped, Quaternion
import tf.transformations


class MultiRobotRandomGoalSender:
    def __init__(self, robot_list=None, min_x=-10.0, max_x=10.0, 
                 min_y=-15.0, max_y=15.0, random_yaw=True):
        """
        初始化多机器人随机目标点发送器
        
        参数:
            robot_list: 机器人命名空间列表，如果为None则自动检测
            min_x, max_x: X坐标范围
            min_y, max_y: Y坐标范围
            random_yaw: 是否随机生成朝向角度
        """
        rospy.init_node('multi_robot_random_goals', anonymous=True)
        
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.random_yaw = random_yaw
        
        if robot_list is None:
            self.robot_list = self._detect_robots()
        else:
            self.robot_list = robot_list
        
        if not self.robot_list:
            rospy.logerr("未检测到任何机器人！请确保机器人已启动。")
            sys.exit(1)
        
        rospy.loginfo("检测到 %d 个机器人: %s", len(self.robot_list), ', '.join(self.robot_list))
        
        self.publishers = {}
        for robot_ns in self.robot_list:
            goal_topic = '/' + robot_ns + '/move_base_simple/goal'
            pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)
            self.publishers[robot_ns] = pub
            rospy.loginfo("为机器人 %s 创建发布者: %s", robot_ns, goal_topic)
        
        rospy.sleep(1.0)
    
    def _detect_robots(self):
        """
        自动检测所有可用的机器人
        通过检查 /robot_X/move_base_simple/goal topic 是否存在
        """
        robot_list = []
        rospy.loginfo("正在检测可用的机器人...")
        
        try:
            topics = [topic[0] for topic in rospy.get_published_topics()]
            
            for topic in topics:
                if '/move_base_simple/goal' in topic:
                    parts = topic.split('/')
                    if len(parts) >= 3 and parts[1].startswith('robot_'):
                        robot_ns = parts[1]
                        if robot_ns not in robot_list:
                            robot_list.append(robot_ns)
                            rospy.loginfo("检测到机器人: %s (topic: %s)", robot_ns, topic)
        except Exception as e:
            rospy.logwarn("检测topic时出错: %s", str(e))
        
        if not robot_list:
            rospy.logwarn("未通过topic检测到机器人，尝试使用默认列表: robot_1, robot_2")
            default_robots = ['robot_1', 'robot_2']
            for ns in default_robots:
                robot_list.append(ns)
                rospy.loginfo("使用默认机器人: %s", ns)
        
        return sorted(robot_list)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    
    def generate_random_goal(self):
        """
        生成随机目标点
        
        返回:
            (x, y, yaw) 元组
        """
        x = random.uniform(self.min_x, self.max_x)
        y = random.uniform(self.min_y, self.max_y)
        
        if self.random_yaw:
            yaw = random.uniform(-math.pi, math.pi)
        else:
            yaw = 0.0
        
        return (x, y, yaw)
    
    def send_goal_to_robot(self, robot_ns, x, y, yaw=0.0, frame_id='map'):
        """
        向指定机器人发送目标点
        
        参数:
            robot_ns: 机器人命名空间
            x, y: 目标坐标
            yaw: 目标朝向角度（弧度）
            frame_id: 坐标系ID
        """
        if robot_ns not in self.publishers:
            rospy.logwarn("机器人 %s 的发布者不存在", robot_ns)
            return False
        
        if not frame_id or frame_id.strip() == '':
            frame_id = 'map'
        
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation = self.euler_to_quaternion(0, 0, yaw)
        
        self.publishers[robot_ns].publish(goal)
        
        rospy.loginfo("[%s] 发送目标点: x=%.2f, y=%.2f, yaw=%.1f°", 
                     robot_ns, x, y, math.degrees(yaw))
        
        return True
    
    def send_random_goals_to_all(self):
        """为所有机器人发送随机目标点"""
        for robot_ns in self.robot_list:
            x, y, yaw = self.generate_random_goal()
            self.send_goal_to_robot(robot_ns, x, y, yaw)
    
    def run_continuous(self, interval=30.0):
        """
        持续运行，定期为所有机器人发送随机目标点
        
        参数:
            interval: 发送间隔（秒）
        """
        rospy.loginfo("开始持续发送随机目标点，间隔: %.1f 秒", interval)
        rospy.loginfo("地图范围: x [%.1f, %.1f], y [%.1f, %.1f]", 
                     self.min_x, self.max_x, self.min_y, self.max_y)
        
        count = 0
        rate = rospy.Rate(1.0 / interval)
        
        while not rospy.is_shutdown():
            count += 1
            rospy.loginfo("\n=== 第 %d 轮发送随机目标点 ===", count)
            self.send_random_goals_to_all()
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(description='多机器人随机目标点发送脚本')
    parser.add_argument('--robot_list', type=str, default=None,
                       help='机器人命名空间列表，用逗号分隔，例如: robot_1,robot_2')
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
    parser.add_argument('--once', action='store_true',
                       help='只发送一次目标点，不持续运行')
    
    args = parser.parse_args()
    
    robot_list = None
    if args.robot_list:
        robot_list = [ns.strip() for ns in args.robot_list.split(',')]
    
    try:
        sender = MultiRobotRandomGoalSender(
            robot_list=robot_list,
            min_x=args.min_x,
            max_x=args.max_x,
            min_y=args.min_y,
            max_y=args.max_y,
            random_yaw=args.random_yaw
        )
        
        if args.once:
            rospy.loginfo("发送一次随机目标点...")
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
