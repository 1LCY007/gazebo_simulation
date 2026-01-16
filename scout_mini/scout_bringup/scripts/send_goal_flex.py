#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
发送目标点给go2机器狗，使用move_base_flex进行路径规划
使用方法:
    python3 send_goal_flex.py x y yaw
    例如: python3 send_goal_flex.py 5.0 3.0 0.0
    
或者直接运行，会提示输入坐标:
    python3 send_goal_flex.py
"""

import rospy
import sys
import math
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import Quaternion
import tf.transformations


class MoveBaseFlexGoalSender:
    def __init__(self):
        """初始化move_base_flex action client"""
        rospy.init_node('send_goal_node', anonymous=True)
        
        # 输出当前使用的规划器信息
        self.print_planner_info()
        
        # 创建action client
        self.client = SimpleActionClient('robot_0/move_base_flex/move_base', MoveBaseAction)
        rospy.loginfo("等待move_base_flex服务器连接...")
        
        # 等待服务器连接，最多等待5秒
        if not self.client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("无法连接到move_base_flex服务器！请确保move_base_flex正在运行。")
            sys.exit(1)
        
        rospy.loginfo("已连接到move_base_flex服务器")
    
    def print_planner_info(self):
        """输出当前使用的规划器信息"""
        try:
            # 获取全局规划器
            global_planner = rospy.get_param('/robot_0/move_base_flex/base_global_planner', 'navfn/NavfnROS')
            # 获取局部规划器
            local_planner = rospy.get_param('/robot_0/move_base_flex/base_local_planner', 'teb_local_planner/TebLocalPlannerROS')
            
            print("\n" + "="*50)
            print("当前使用的规划器配置:")
            print("="*50)
            print("全局规划器 (Global Planner): %s" % global_planner)
            print("局部规划器 (Local Planner):  %s" % local_planner)
            print("="*50 + "\n")
            
            # 如果是navfn，输出额外信息
            if 'navfn' in global_planner:
                try:
                    use_dijkstra = rospy.get_param('/robot_0/move_base_flex/NavfnROS/use_dijkstra', True)
                    use_quadratic = rospy.get_param('/robot_0/move_base_flex/NavfnROS/use_quadratic', True)
                    print("Navfn配置:")
                    print("  - 使用Dijkstra算法: %s" % use_dijkstra)
                    print("  - 使用二次插值: %s" % use_quadratic)
                except:
                    pass
            
            # 如果是TEB，输出额外信息
            if 'teb' in local_planner.lower():
                try:
                    max_vel_x = rospy.get_param('/robot_0/move_base_flex/TebLocalPlannerROS/max_vel_x', 'N/A')
                    max_vel_theta = rospy.get_param('/robot_0/move_base_flex/TebLocalPlannerROS/max_vel_theta', 'N/A')
                    print("TEB局部规划器配置:")
                    print("  - 最大线速度: %s m/s" % max_vel_x)
                    print("  - 最大角速度: %s rad/s" % max_vel_theta)
                except:
                    pass
            
            print("="*50 + "\n")
            
        except Exception as e:
            rospy.logwarn("无法获取规划器信息: %s" % str(e))
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """将欧拉角转换为四元数"""
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
    
    def send_goal(self, x, y, yaw=0.0, frame_id='map'):
        """
        发送目标点
        
        参数:
            x: 目标x坐标 (米)
            y: 目标y坐标 (米)
            yaw: 目标朝向角度 (弧度，默认0.0)
            frame_id: 坐标系ID (默认'map')
            
        注意: 地图范围约为 x: [-35, 35], y: [-35, 35]
        """
        # 检查目标点是否在地图范围内（粗略检查）
        if x < -40 or x > 40 or y < -40 or y > 40:
            rospy.logwarn("警告: 目标点可能超出地图范围！地图范围约为 x: [-35, 35], y: [-35, 35]")
        # 创建目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置位置
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        # 设置朝向（将yaw转换为四元数）
        goal.target_pose.pose.orientation = self.euler_to_quaternion(0, 0, yaw)
        
        rospy.loginfo("发送目标点: x=%.2f, y=%.2f, yaw=%.2f (度)" % (x, y, math.degrees(yaw)))
        
        # 使用默认规划器/控制器（空字符串会由move_base_flex选择默认）
        goal.planner = ""
        goal.controller = ""
        goal.recovery_behaviors = []

        # 发送目标
        self.client.send_goal(goal)
        
        # 等待结果
        rospy.loginfo("等待机器人到达目标点...")
        wait = self.client.wait_for_result(rospy.Duration(240.0))  # 最多等待240秒
        
        if not wait:
            rospy.logerr("动作执行超时！")
            self.client.cancel_goal()
            return False
        else:
            state = self.client.get_state()
            if state == 3:  # SUCCEEDED
                rospy.loginfo("成功到达目标点！")
                return True
            else:
                rospy.logwarn("未能到达目标点，状态码: %d" % state)
                return False
    
    def send_simple_goal(self, x, y, yaw=0.0, frame_id='map'):
        """
        使用简单的Publisher方式发送目标点（move_base_flex不支持PoseStamped goal）
        
        参数:
            x: 目标x坐标 (米)
            y: 目标y坐标 (米)
            yaw: 目标朝向角度 (弧度，默认0.0)
            frame_id: 坐标系ID (默认'map')
            
        注意: 地图范围约为 x: [-35, 35], y: [-35, 35]
        """
        # 检查目标点是否在地图范围内（粗略检查）
        if x < -40 or x > 40 or y < -40 or y > 40:
            rospy.logwarn("警告: 目标点可能超出地图范围！地图范围约为 x: [-35, 35], y: [-35, 35]")
        rospy.logwarn("move_base_flex不提供PoseStamped的simple goal接口，请使用action方式。")


def main():
    try:
        sender = MoveBaseFlexGoalSender()
        
        # 检查命令行参数
        if len(sys.argv) == 4:
            # 从命令行参数获取坐标
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            yaw_deg = float(sys.argv[3])  # 输入的是度数
            yaw = math.radians(yaw_deg)
            
            # 使用action方式发送（会等待结果）
            sender.send_goal(x, y, yaw)
            
        elif len(sys.argv) == 3:
            # 只有x和y，yaw默认为0
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            sender.send_goal(x, y, 0.0)
            
        else:
            # 交互式输入
            print("\n=== Go2机器狗目标点发送程序 ===")
            print("地图范围: x: [-35, 35] 米, y: [-35, 35] 米")
            print("请输入目标点坐标（按Enter使用默认值）")
            
            try:
                x_str = input("X坐标 (米) [默认: 5.0]: ").strip()
                x = float(x_str) if x_str else 5.0
                
                y_str = input("Y坐标 (米) [默认: 3.0]: ").strip()
                y = float(y_str) if y_str else 3.0
                
                yaw_str = input("朝向角度 (度) [默认: 0.0]: ").strip()
                yaw_deg = float(yaw_str) if yaw_str else 0.0
                yaw = math.radians(yaw_deg)
                
                # 发送目标点
                sender.send_goal(x, y, yaw)
                    
            except ValueError:
                rospy.logerr("输入格式错误！请输入数字。")
            except KeyboardInterrupt:
                rospy.loginfo("用户中断")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr("发生错误: %s" % str(e))


if __name__ == '__main__':
    main()
