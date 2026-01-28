#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Go2 双机器人键盘遥控节点（robot_5 和 robot_6）

功能：
- 同时控制 robot_5 和 robot_6 两个 Go2 机器人
- robot_5: wsad 前后左右平移，qe 左转右转
- robot_6: ikjl 前后左右平移，uo 左转右转
- 支持平移功能（左右移动）

使用方法：
rosrun scout_teleop go2_dual_teleop_keyboard.py
"""

import rospy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist

# 控制键映射说明
msg = """
Reading from the keyboard and controlling robot_5 and robot_6 simultaneously!
================================================================================
Robot 5 (Go2) Controls:
   w
a  s  d
   q    e
w: forward, s: backward, a: left strafe, d: right strafe
q: turn left, e: turn right

Robot 6 (Go2) Controls:
   i
j  k  l
   u    o
i: forward, k: backward, j: left strafe, l: right strafe
u: turn left, o: turn right

SPACE : stop both robots
f : stop robot_5
h : stop robot_6
CTRL-C to quit
================================================================================
"""

# robot_5 控制键映射 (x, y, z, th)
# x: 前后移动, y: 左右平移, z: 上下移动, th: 旋转
robot5_bindings = {
    'w': (1, 0, 0, 0),   # 前进
    's': (-1, 0, 0, 0),  # 后退
    'a': (0, 1, 0, 0),   # 左平移
    'd': (0, -1, 0, 0),  # 右平移
    'q': (0, 0, 0, 1),   # 左转
    'e': (0, 0, 0, -1),  # 右转
}

# robot_6 控制键映射 (x, y, z, th)
robot6_bindings = {
    'i': (1, 0, 0, 0),   # 前进
    'k': (-1, 0, 0, 0),  # 后退
    'j': (0, 1, 0, 0),   # 左平移
    'l': (0, -1, 0, 0),  # 右平移
    'u': (0, 0, 0, 1),   # 左转
    'o': (0, 0, 0, -1),  # 右转
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn, strafe_speed):
    return "currently:\tspeed %s\tturn %s\tstrafe %s " % (speed, turn, strafe_speed)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('go2_dual_teleop_keyboard')
    
    # 固定控制 robot_5 和 robot_6
    robot5_ns = '/robot_5'
    robot6_ns = '/robot_6'
    
    # 创建发布者
    robot5_pub = rospy.Publisher(robot5_ns + '/cmd_vel', Twist, queue_size=1)
    robot6_pub = rospy.Publisher(robot6_ns + '/cmd_vel', Twist, queue_size=1)
    
    rospy.loginfo(f"Created publisher for {robot5_ns}/cmd_vel")
    rospy.loginfo(f"Created publisher for {robot6_ns}/cmd_vel")
    
    # 速度参数
    speed = rospy.get_param("~speed", 0.5)          # 前后移动速度
    turn = rospy.get_param("~turn", 1.0)            # 旋转速度
    strafe_speed = rospy.get_param("~strafe_speed", 0.5)  # 平移速度
    
    # robot_5 的速度状态
    robot5_x = 0
    robot5_y = 0
    robot5_z = 0
    robot5_th = 0
    
    # robot_6 的速度状态
    robot6_x = 0
    robot6_y = 0
    robot6_z = 0
    robot6_th = 0
    
    status = 0

    try:
        print(msg)
        print(f"\n{'='*50}")
        print(f"Controlling: {robot5_ns} and {robot6_ns}")
        print(f"{'='*50}")
        print(vels(speed, turn, strafe_speed))
        print("\nPress keys to control robots...")
        
        while True:
            key = getKey()
            
            # 处理 robot_5 的控制
            if key in robot5_bindings.keys():
                robot5_x = robot5_bindings[key][0]
                robot5_y = robot5_bindings[key][1]
                robot5_z = robot5_bindings[key][2]
                robot5_th = robot5_bindings[key][3]
            # 处理 robot_6 的控制
            elif key in robot6_bindings.keys():
                robot6_x = robot6_bindings[key][0]
                robot6_y = robot6_bindings[key][1]
                robot6_z = robot6_bindings[key][2]
                robot6_th = robot6_bindings[key][3]
            # 停止 robot_5
            elif key == 'f':
                robot5_x = 0
                robot5_y = 0
                robot5_z = 0
                robot5_th = 0
                print(f"[{robot5_ns}] Stopped")
            # 停止 robot_6
            elif key == 'h':
                robot6_x = 0
                robot6_y = 0
                robot6_z = 0
                robot6_th = 0
                print(f"[{robot6_ns}] Stopped")
            # 停止所有机器人
            elif key == ' ':
                robot5_x = 0
                robot5_y = 0
                robot5_z = 0
                robot5_th = 0
                robot6_x = 0
                robot6_y = 0
                robot6_z = 0
                robot6_th = 0
            # 退出
            elif key == '\x03':  # CTRL-C
                break
            else:
                # 如果按键不是控制键，不改变速度（保持当前速度）
                pass

            # 创建并发布 robot_5 的速度命令
            robot5_twist = Twist()
            robot5_twist.linear.x = robot5_x * speed
            robot5_twist.linear.y = robot5_y * strafe_speed  # 平移速度
            robot5_twist.linear.z = robot5_z * speed
            robot5_twist.angular.x = 0
            robot5_twist.angular.y = 0
            robot5_twist.angular.z = robot5_th * turn
            robot5_pub.publish(robot5_twist)
            
            # 创建并发布 robot_6 的速度命令
            robot6_twist = Twist()
            robot6_twist.linear.x = robot6_x * speed
            robot6_twist.linear.y = robot6_y * strafe_speed  # 平移速度
            robot6_twist.linear.z = robot6_z * speed
            robot6_twist.angular.x = 0
            robot6_twist.angular.y = 0
            robot6_twist.angular.z = robot6_th * turn
            robot6_pub.publish(robot6_twist)
            
            # 显示当前状态（减少输出频率）
            if robot5_x != 0 or robot5_y != 0 or robot5_z != 0 or robot5_th != 0 or \
               robot6_x != 0 or robot6_y != 0 or robot6_z != 0 or robot6_th != 0:
                if status % 20 == 0:  # 每20次输出一次
                    print(f"[{robot5_ns}] vx={robot5_twist.linear.x:.2f}, vy={robot5_twist.linear.y:.2f}, vth={robot5_twist.angular.z:.2f} | "
                          f"[{robot6_ns}] vx={robot6_twist.linear.x:.2f}, vy={robot6_twist.linear.y:.2f}, vth={robot6_twist.angular.z:.2f}")
            status = (status + 1) % 100

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # 停止所有机器人
        robot5_twist = Twist()
        robot6_twist = Twist()
        robot5_pub.publish(robot5_twist)
        robot6_pub.publish(robot6_twist)
        rospy.loginfo("Stopped both robots")
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

