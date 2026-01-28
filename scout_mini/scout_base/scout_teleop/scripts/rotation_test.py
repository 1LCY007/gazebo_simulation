#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人旋转运动测试脚本

功能：
- 让机器人执行旋转运动，用于检测角速度
- 使用字母键 z/x/c/v/b/n 选择要测试的机器人（对应 robot_1 到 robot_6）
- 监控选中机器人的角速度（cmd_vel 和 odom）
- 可以设置旋转速度、持续时间等参数

使用方法：
rosrun scout_teleop rotation_test.py
"""

import rospy
import sys
import time
import math
import select
import termios
import tty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading

# 机器人切换键映射：z/x/c/v/b/n 对应 robot_1 到 robot_6
robotSwitchKeys = {
    'z': 0,  # robot_1
    'x': 1,  # robot_2
    'c': 2,  # robot_3
    'v': 3,  # robot_4
    'b': 4,  # robot_5
    'n': 5,  # robot_6
}

class RotationTest:
    def __init__(self, robot_namespaces=None):
        """
        初始化旋转测试
        
        参数:
            robot_namespaces: 机器人命名空间列表，如果为None则使用默认列表
        """
        rospy.init_node('rotation_test', anonymous=True)
        
        # 默认机器人列表
        if robot_namespaces is None:
            robot_namespaces = ['/robot_1', '/robot_2', '/robot_3', '/robot_4', '/robot_5', '/robot_6']
        
        self.robot_namespaces = robot_namespaces
        self.current_robot_idx = 0
        self.current_robot_ns = self.robot_namespaces[self.current_robot_idx] if self.robot_namespaces else None
        
        # 存储所有机器人的速度数据
        self.robot_data = {}
        for ns in self.robot_namespaces:
            self.robot_data[ns] = {
                'cmd_vel': Twist(),
                'odom_vel': Twist(),
                'cmd_received': False,
                'odom_received': False,
                'publisher': rospy.Publisher(ns + '/cmd_vel', Twist, queue_size=1),
            }
            # 订阅 odom 和 cmd_vel
            rospy.Subscriber(ns + '/odom', Odometry, self._odom_callback, callback_args=ns, queue_size=1)
            rospy.Subscriber(ns + '/cmd_vel', Twist, self._cmd_callback, callback_args=ns, queue_size=1)
        
        self.test_running = False
        self.test_angular_vel = 0.0
        
        rospy.loginfo(f"Rotation test initialized for {len(self.robot_namespaces)} robots")
        rospy.loginfo(f"Available robots: {', '.join(self.robot_namespaces)}")
        rospy.sleep(1.0)  # 等待发布者和订阅者建立连接
    
    def _cmd_callback(self, msg, robot_ns):
        """cmd_vel 回调"""
        if robot_ns in self.robot_data:
            self.robot_data[robot_ns]['cmd_vel'] = msg
            self.robot_data[robot_ns]['cmd_received'] = True
    
    def _odom_callback(self, msg, robot_ns):
        """odom 回调"""
        if robot_ns in self.robot_data:
            self.robot_data[robot_ns]['odom_vel'] = msg.twist.twist
            self.robot_data[robot_ns]['odom_received'] = True
    
    def get_key(self, settings):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def display_info(self):
        """显示信息"""
        sys.stdout.write('\033[2J\033[H')  # 清屏
        
        sys.stdout.write("=" * 70 + "\n")
        sys.stdout.write("Robot Rotation Test - Angular Velocity Monitor\n")
        sys.stdout.write("=" * 70 + "\n\n")
        
        # 显示当前选中的机器人信息
        if self.current_robot_ns:
            robot_data = self.robot_data[self.current_robot_ns]
            
            sys.stdout.write(f"Current Robot: {self.current_robot_ns}\n")
            sys.stdout.write("-" * 70 + "\n")
            
            # 测试状态
            if self.test_running:
                sys.stdout.write(f"Test Status: RUNNING (angular_vel={self.test_angular_vel:.4f} rad/s)\n\n")
            else:
                sys.stdout.write("Test Status: IDLE\n")
                sys.stdout.write("Press 's' to start rotation test, 'z/x/c/v/b/n' to switch robot\n\n")
            
            # 命令速度
            cmd_status = "✓" if robot_data['cmd_received'] else "✗"
            sys.stdout.write(f"Command (cmd_vel) [{cmd_status}]:\n")
            sys.stdout.write(f"  Angular Z: {robot_data['cmd_vel'].angular.z:8.4f} rad/s ({math.degrees(robot_data['cmd_vel'].angular.z):7.2f} deg/s)\n")
            
            # 实际速度
            odom_status = "✓" if robot_data['odom_received'] else "✗"
            sys.stdout.write(f"Actual (odom) [{odom_status}]:\n")
            sys.stdout.write(f"  Angular Z: {robot_data['odom_vel'].angular.z:8.4f} rad/s ({math.degrees(robot_data['odom_vel'].angular.z):7.2f} deg/s)\n")
            
            # 误差
            error = abs(robot_data['cmd_vel'].angular.z - robot_data['odom_vel'].angular.z)
            sys.stdout.write(f"Error: {error:8.4f} rad/s ({math.degrees(error):7.2f} deg/s)\n\n")
        
        # 显示所有机器人列表
        sys.stdout.write("Available Robots:\n")
        for i, ns in enumerate(self.robot_namespaces):
            marker = " <--" if i == self.current_robot_idx else ""
            key = list(robotSwitchKeys.keys())[list(robotSwitchKeys.values()).index(i)] if i in robotSwitchKeys.values() else "?"
            sys.stdout.write(f"  [{key}] {ns}{marker}\n")
        
        sys.stdout.write("\n" + "=" * 70 + "\n")
        sys.stdout.write("Controls: z/x/c/v/b/n=switch robot, s=start test, space=stop, CTRL-C=quit\n")
        sys.stdout.flush()
    
    def switch_robot(self, idx):
        """切换机器人"""
        if 0 <= idx < len(self.robot_namespaces):
            # 停止当前测试
            self.stop_test()
            self.current_robot_idx = idx
            self.current_robot_ns = self.robot_namespaces[self.current_robot_idx]
            rospy.loginfo(f"Switched to robot: {self.current_robot_ns}")
            return True
        return False
    
    def start_test(self, angular_velocity=1.0, direction='left'):
        """开始旋转测试"""
        if self.current_robot_ns is None:
            rospy.logwarn("No robot selected")
            return
        
        # 确定角速度符号
        if direction == 'left':
            angular_vel = angular_velocity  # 正值为逆时针（左转）
        else:
            angular_vel = -angular_velocity  # 负值为顺时针（右转）
        
        self.test_angular_vel = angular_vel
        self.test_running = True
        
        # 创建速度命令
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_vel
        
        # 发布速度命令
        self.robot_data[self.current_robot_ns]['publisher'].publish(twist)
        rospy.loginfo(f"Started rotation test for {self.current_robot_ns}: angular_vel={angular_vel:.4f} rad/s")
    
    def stop_test(self):
        """停止测试"""
        if self.current_robot_ns and self.test_running:
            stop_twist = Twist()
            self.robot_data[self.current_robot_ns]['publisher'].publish(stop_twist)
            self.test_running = False
            rospy.loginfo(f"Stopped rotation test for {self.current_robot_ns}")
    
    def run_interactive(self, default_angular_velocity=1.0, default_direction='left'):
        """交互式运行"""
        settings = termios.tcgetattr(sys.stdin)
        
        print("Robot Rotation Test - Interactive Mode")
        print("Press z/x/c/v/b/n to select robot, s to start test, space to stop, CTRL-C to quit\n")
        
        # 启动显示线程
        display_thread = threading.Thread(target=self._display_loop, daemon=True)
        display_thread.start()
        
        # 启动命令发布线程
        command_thread = threading.Thread(target=self._command_loop, daemon=True)
        command_thread.start()
        
        try:
            while not rospy.is_shutdown():
                # 检查键盘输入
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = self.get_key(settings)
                    
                    # 切换机器人
                    if key in robotSwitchKeys.keys():
                        idx = robotSwitchKeys[key]
                        self.switch_robot(idx)
                    # 开始测试
                    elif key == 's':
                        self.start_test(default_angular_velocity, default_direction)
                    # 停止测试
                    elif key == ' ':
                        self.stop_test()
                    # 退出
                    elif key == '\x03':  # CTRL-C
                        break
                
                rospy.sleep(0.1)
        
        except Exception as e:
            rospy.logerr(f"Error: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            self.stop_test()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print("\nExiting...")
    
    def run_auto_test(self, angular_velocity=1.0, duration=10.0, direction='left'):
        """自动运行测试（指定持续时间）"""
        rospy.loginfo(f"Starting auto rotation test: angular_velocity={angular_velocity} rad/s, duration={duration}s, direction={direction}")
        
        # 启动显示线程
        display_thread = threading.Thread(target=self._display_loop, daemon=True)
        display_thread.start()
        
        # 开始测试
        self.start_test(angular_velocity, direction)
        
        # 等待指定时间
        start_time = time.time()
        rate = rospy.Rate(50)  # 50Hz 发布频率
        
        try:
            while not rospy.is_shutdown() and (time.time() - start_time) < duration:
                # 持续发布速度命令
                if self.test_running and self.current_robot_ns:
                    twist = Twist()
                    twist.angular.z = self.test_angular_vel
                    self.robot_data[self.current_robot_ns]['publisher'].publish(twist)
                rate.sleep()
            
            # 停止测试
            self.stop_test()
            rospy.loginfo("Auto rotation test completed")
            
            # 等待一下让数据显示
            rospy.sleep(2.0)
        
        except KeyboardInterrupt:
            rospy.loginfo("Test interrupted by user")
            self.stop_test()
    
    def _display_loop(self):
        """显示循环（在单独线程中运行，5Hz更新）"""
        while not rospy.is_shutdown():
            self.display_info()
            time.sleep(0.2)  # 5Hz更新频率
    
    def _command_loop(self):
        """命令发布循环（在单独线程中运行，50Hz）"""
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.test_running and self.current_robot_ns:
                twist = Twist()
                twist.angular.z = self.test_angular_vel
                self.robot_data[self.current_robot_ns]['publisher'].publish(twist)
            rate.sleep()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='机器人旋转运动测试脚本')
    parser.add_argument('--angular_velocity', type=float, default=1.0,
                       help='角速度（rad/s，默认: 1.0）')
    parser.add_argument('--duration', type=float, default=None,
                       help='持续时间（秒，如果指定则自动运行，否则交互式运行）')
    parser.add_argument('--direction', type=str, default='left', choices=['left', 'right'],
                       help='旋转方向（left 或 right，默认: left）')
    parser.add_argument('--robot_list', type=str, default=None,
                       help='机器人命名空间列表，用逗号分隔（默认: robot_1,robot_2,robot_3,robot_4,robot_5,robot_6）')
    
    args = parser.parse_args()
    
    # 处理机器人列表
    robot_namespaces = None
    if args.robot_list:
        robot_namespaces_raw = [ns.strip() for ns in args.robot_list.split(',')]
        robot_namespaces = []
        for ns_raw in robot_namespaces_raw:
            if not ns_raw.startswith('/'):
                ns = '/' + ns_raw
            else:
                ns = ns_raw
            robot_namespaces.append(ns)
    
    try:
        test = RotationTest(robot_namespaces=robot_namespaces)
        
        if args.duration is not None:
            # 自动运行模式
            test.run_auto_test(
                angular_velocity=args.angular_velocity,
                duration=args.duration,
                direction=args.direction
            )
        else:
            # 交互式模式
            test.run_interactive(
                default_angular_velocity=args.angular_velocity,
                default_direction=args.direction
            )
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()
