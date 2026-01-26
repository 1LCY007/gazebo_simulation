#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人速度监控节点

功能：
- 每次只监控一个机器人的速度信息（cmd_vel 和 odom）
- 使用字母键 z/x/c/v/b/n 切换要监控的机器人（对应 robot_1 到 robot_6）
- 简洁显示当前监控机器人的速度信息

使用方法：
rosrun scout_bringup robot_velocity_monitor.py _robot_namespaces:="robot_1,robot_2,robot_3,robot_4,robot_5,robot_6"
"""

import rospy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import time

# 机器人切换键映射：z/x/c/v/b/n 对应 robot_1 到 robot_6
robotSwitchKeys = {
    'z': 0,  # robot_1
    'x': 1,  # robot_2
    'c': 2,  # robot_3
    'v': 3,  # robot_4
    'b': 4,  # robot_5
    'n': 5,  # robot_6
}


class RobotVelocityMonitor:
    def __init__(self, robot_namespaces):
        self.robot_namespaces = robot_namespaces
        self.current_robot_idx = 0
        self.current_robot_ns = robot_namespaces[self.current_robot_idx] if robot_namespaces else None
        
        # 当前监控机器人的速度信息
        self.cmd_vel = Twist()
        self.odom_vel = Twist()
        self.cmd_vel_received = False
        self.odom_received = False
        
        # 订阅者（只订阅当前监控的机器人）
        self.subscriber_cmd_vel = None
        self.subscriber_odom = None
        
        # 订阅当前机器人
        self._subscribe_current_robot()
    
    def _subscribe_current_robot(self):
        """订阅当前监控的机器人"""
        # 取消之前的订阅
        if self.subscriber_cmd_vel:
            self.subscriber_cmd_vel.unregister()
        if self.subscriber_odom:
            self.subscriber_odom.unregister()
        
        # 重置状态
        self.cmd_vel_received = False
        self.odom_received = False
        
        if not self.current_robot_ns:
            return
        
        # 订阅 cmd_vel
        cmd_vel_topic = self.current_robot_ns + '/cmd_vel'
        self.subscriber_cmd_vel = rospy.Subscriber(
            cmd_vel_topic, Twist, 
            self.cmd_vel_callback,
            queue_size=1
        )
        
        # 订阅 odom
        odom_topic = self.current_robot_ns + '/odom'
        self.subscriber_odom = rospy.Subscriber(
            odom_topic, Odometry,
            self.odom_callback,
            queue_size=1
        )
        
        rospy.loginfo(f"Monitoring robot: {self.current_robot_ns}")
    
    def cmd_vel_callback(self, msg):
        """cmd_vel 回调函数"""
        self.cmd_vel = msg
        self.cmd_vel_received = True
    
    def odom_callback(self, msg):
        """odom 回调函数"""
        self.odom_vel = msg.twist.twist
        self.odom_received = True
    
    def get_key(self, settings):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def display_velocity_info(self):
        """显示当前监控机器人的速度信息"""
        if not self.current_robot_ns:
            return
        
        # 清屏并显示信息
        sys.stdout.write('\033[2J\033[H')  # 清屏并移动光标到左上角
        
        # 标题
        sys.stdout.write(f"Robot: {self.current_robot_ns}\n")
        sys.stdout.write("-" * 50 + "\n\n")
        
        # 命令速度
        cmd_status = "✓" if self.cmd_vel_received else "✗"
        sys.stdout.write(f"Command (cmd_vel) [{cmd_status}]:\n")
        sys.stdout.write(f"  Linear:  {self.cmd_vel.linear.x:6.3f} m/s\n")
        sys.stdout.write(f"  Angular: {self.cmd_vel.angular.z:6.3f} rad/s\n\n")
        
        # 实际速度
        odom_status = "✓" if self.odom_received else "✗"
        sys.stdout.write(f"Actual (odom) [{odom_status}]:\n")
        sys.stdout.write(f"  Linear:  {self.odom_vel.linear.x:6.3f} m/s\n")
        sys.stdout.write(f"  Angular: {self.odom_vel.angular.z:6.3f} rad/s\n\n")
        
        # 速度大小
        cmd_speed = abs(self.cmd_vel.linear.x)
        odom_speed = abs(self.odom_vel.linear.x)
        sys.stdout.write(f"Speed: cmd={cmd_speed:.3f} m/s, actual={odom_speed:.3f} m/s\n")
        sys.stdout.write("-" * 50 + "\n")
        sys.stdout.write("Press z/x/c/v/b/n to switch, CTRL-C to quit\n")
        sys.stdout.flush()
    
    def switch_robot(self, idx):
        """切换监控的机器人"""
        if 0 <= idx < len(self.robot_namespaces):
            self.current_robot_idx = idx
            self.current_robot_ns = self.robot_namespaces[self.current_robot_idx]
            self._subscribe_current_robot()
            return True
        return False
    
    def run(self):
        """主运行循环"""
        settings = termios.tcgetattr(sys.stdin)
        
        print("Robot Velocity Monitor")
        print("Press z/x/c/v/b/n to switch robot, CTRL-C to quit\n")
        
        # 启动显示线程（降低频率到2Hz）
        display_thread = threading.Thread(target=self._display_loop, daemon=True)
        display_thread.start()
        
        try:
            while not rospy.is_shutdown():
                # 检查键盘输入
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = self.get_key(settings)
                    
                    # 切换机器人
                    if key in robotSwitchKeys.keys():
                        idx = robotSwitchKeys[key]
                        if self.switch_robot(idx):
                            print(f"\nSwitched to: {self.current_robot_ns}\n")
                    elif key == '\x03':  # CTRL-C
                        break
                
                rospy.sleep(1)  # 降低主循环频率
        
        except Exception as e:
            rospy.logerr(f"Error: {e}")
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print("\nExiting...")
    
    def _display_loop(self):
        """显示循环（在单独线程中运行，2Hz更新）"""
        while not rospy.is_shutdown():
            self.display_velocity_info()
            time.sleep(0.5)  # 2Hz更新频率


if __name__ == "__main__":
    # 获取机器人命名空间列表
    robot_namespaces_str = rospy.get_param('~robot_namespaces', 
                                          'robot_1,robot_2,robot_3,robot_4,robot_5,robot_6')
    robot_namespaces_raw = [ns.strip() for ns in robot_namespaces_str.split(',')]
    
    rospy.init_node('robot_velocity_monitor', anonymous=True)
    
    # 处理命名空间（确保以 / 开头）
    robot_namespaces = []
    for ns_raw in robot_namespaces_raw:
        if not ns_raw.startswith('/'):
            ns = '/' + ns_raw
        else:
            ns = ns_raw
        robot_namespaces.append(ns)
    
    if len(robot_namespaces) == 0:
        rospy.logerr("No robot namespaces specified!")
        sys.exit(1)
    
    # 创建监控器并运行
    monitor = RobotVelocityMonitor(robot_namespaces)
    monitor.run()
