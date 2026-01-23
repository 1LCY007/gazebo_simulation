#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多机器人键盘遥控节点

功能：
- 可以通过键盘控制多个机器人
- 使用字母键 z/x/c/v/b/n 切换要控制的机器人（对应 robot_1 到 robot_6）
- 使用 WASD 控制移动，JL 控制旋转
- 显示当前控制的机器人信息

使用方法：
rosrun scout_teleop multi_robot_teleop_keyboard.py _robot_namespaces:="robot_1,robot_2,robot_3,robot_4,robot_5,robot_6"
"""

import rospy
import sys
import select
import termios
import tty
from geometry_msgs.msg import Twist

# 控制键映射
msg = """
Reading from the keyboard and publishing to cmd_vel!
---------------------------
Moving around:
   w
a  s  d

   j    l
turn left/right

Switch robot:
z/x/c/v/b/n : Switch robot (z=robot_1, x=robot_2, c=robot_3, v=robot_4, b=robot_5, n=robot_6)

SPACE : stop robot
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    '[': (-1, 0, 0, 0),
    ']': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    'k': (-1, 0, 0, 1),
    ';': (1, 0, 0, -1),
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    # 注意：x, b 键已用于切换机器人，不再用于移动控制
    # 使用 's' 键代替后退，'t' 键已移除（与切换键冲突）
}

# 机器人切换键映射：z/x/c/v/b/n 对应 robot_1 到 robot_6
robotSwitchKeys = {
    'z': 0,  # robot_1
    'x': 1,  # robot_2
    'c': 2,  # robot_3
    'v': 3,  # robot_4
    'b': 4,  # robot_5
    'n': 5,  # robot_6
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # 获取机器人命名空间列表
    robot_namespaces_str = rospy.get_param('~robot_namespaces', 'robot_1,robot_2,robot_3,robot_4,robot_5,robot_6')
    robot_namespaces_raw = [ns.strip() for ns in robot_namespaces_str.split(',')]
    
    rospy.init_node('multi_robot_teleop_keyboard')
    
    # 为每个机器人创建发布者
    publishers = {}
    robot_namespaces = []  # 存储处理后的命名空间（带 / 前缀）
    
    for ns_raw in robot_namespaces_raw:
        # 确保命名空间以 / 开头
        if not ns_raw.startswith('/'):
            ns = '/' + ns_raw
        else:
            ns = ns_raw
        topic = ns + '/cmd_vel'
        publishers[ns] = rospy.Publisher(topic, Twist, queue_size=1)
        robot_namespaces.append(ns)
        rospy.loginfo(f"Created publisher for {topic}")
    
    if len(robot_namespaces) == 0:
        rospy.logerr("No robot namespaces specified!")
        sys.exit(1)
    
    # 当前控制的机器人索引（0-5）
    current_robot_idx = 0
    current_robot_ns = robot_namespaces[current_robot_idx]
    
    # 速度参数
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(f"\n{'='*50}")
        print(f"Current robot: {current_robot_ns}")
        print(f"Available robots: {', '.join(robot_namespaces)}")
        print(f"Press z/x/c/v/b/n to switch robot")
        print(f"{'='*50}")
        print(vels(speed, turn))
        
        while True:
            key = getKey()
            
            # 切换机器人 (z/x/c/v/b/n)
            if key in robotSwitchKeys.keys():
                idx = robotSwitchKeys[key]
                if idx < len(robot_namespaces):
                    # 停止之前的机器人
                    if current_robot_idx < len(robot_namespaces):
                        old_twist = Twist()
                        publishers[robot_namespaces[current_robot_idx]].publish(old_twist)
                    
                    current_robot_idx = idx
                    current_robot_ns = robot_namespaces[current_robot_idx]
                    print(f"\n[Switched to robot: {current_robot_ns}] (key: {key})")
                    print(vels(speed, turn))
                else:
                    print(f"\n[Warning] Robot {key} not available (only {len(robot_namespaces)} robots)")
                continue
            
            # 移动控制
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key == ' ':
                x = 0
                y = 0
                z = 0
                th = 0
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            
            # 发布到当前选中的机器人
            publishers[current_robot_ns].publish(twist)
            
            # 显示当前状态（减少输出频率）
            if x != 0 or y != 0 or z != 0 or th != 0:
                if status % 20 == 0:  # 每20次输出一次
                    print(f"[{current_robot_ns}] vx={twist.linear.x:.2f}, vy={twist.linear.y:.2f}, vz={twist.linear.z:.2f}, vth={twist.angular.z:.2f}")
            status = (status + 1) % 100

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # 停止所有机器人
        twist = Twist()
        for ns in robot_namespaces:
            publishers[ns].publish(twist)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

