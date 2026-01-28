

# Gazebo_Simulation

This project provides a Gazebo-based simulation environment for testing
**multi-type robots** using ROS.

Currently, the simulation supports **Unitree Go2**.

## Install Dependencies

```bash
sudo apt-get install liblcm-dev
sudo apt-get install ros-noetic-controller-interface  ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller
```

Then

```
mkdir -p test_ws/src
cd test_ws
catkin_make
source devel/setup.bash
```



## 1. Single Robot Simulation (Go2)

Launch the Gazebo simulation:

```bash
roslaunch unitree_guide gazeboSim.launch
```

Then start **RViz** and the navigation stack:

```bash
roslaunch unitree_move_base rvizMoveBase.launch
```


## Notes

- go2 的配置在unitree_ros/robots/go2_description中
- unitree_move_base设置move_base的相关参数与rviz启动，要集成到spawn_go2中
- unitree_guide中的launch是启动文件，spawn_go2是为了多机器狗做的集成配置文件


## Scout_mini
### 单机
```bash
roslaunch scout_gazebo_sim hospital_single_mini.launch 
roslaunch scout_navigation hospital_navigation.launch

```

## 多机器人仿真 (Go2与Scout结合)

本仿真环境支持多机器人协同导航，包含：
- **4个 Scout Mini 机器人**
- **2个 Unitree Go2 机器人**

### 启动多机器人仿真

```bash
roslaunch scout_bringup multi_robots_gazeboSim.launch
```

该启动文件会同时启动：
- Gazebo 仿真环境
- 4个 Scout Mini 机器人的导航节点（AMCL、move_base等）
- 2个 Unitree Go2 机器人的导航节点

### 发送随机目标点

启动仿真后，在另一个终端运行以下命令为所有机器人发送随机目标点：

```bash
rosrun unitree_move_base multi_robot_random_goals.py
# 下面是使用roadmap进行规划的
rosrun unitree_move_base multi_robot_roadmap_goals.py
```

该脚本会为每个机器人随机生成目标点，机器人将自动进行路径规划和避障导航。

### 注意事项

- 确保所有机器人的命名空间正确配置
- 多机器人避障依赖于 costmap 的实时更新，建议调整 costmap 更新频率以改善避障性能
- 可以通过 RViz 可视化所有机器人的状态和路径

## 键盘操控

多机器人仿真环境支持通过键盘同时控制多个机器人，可以在机器人之间快速切换。

### 启动键盘操控

在启动多机器人仿真后，在另一个终端运行：

```bash
roslaunch scout_teleop multi_robot_teleop_keyboard.launch
```

或者直接运行节点：

```bash
rosrun scout_teleop multi_robot_teleop_keyboard.py _robot_namespaces:="robot_1,robot_2,robot_3,robot_4,robot_5,robot_6"
```

### 控制说明

**移动控制：**
- `w` - 前进
- `s` - 后退
- `a` - 左转
- `d` - 右转
- `空格` - 停止当前机器人

**切换机器人：**
- `z` - 切换到 robot_1
- `x` - 切换到 robot_2
- `c` - 切换到 robot_3
- `v` - 切换到 robot_4
- `b` - 切换到 robot_5
- `n` - 切换到 robot_6

**退出：**
- `CTRL-C` - 退出程序并停止所有机器人

### 参数配置

可以通过 launch 文件参数配置：

- `robot_namespaces` - 机器人命名空间列表（默认：`robot_1,robot_2,robot_3,robot_4,robot_5,robot_6`）
- `speed` - 线速度（默认：`0.5` m/s）
- `turn` - 角速度（默认：`1.0` rad/s）

示例：

```bash
roslaunch scout_teleop multi_robot_teleop_keyboard.launch speed:=0.8 turn:=1.5
```

