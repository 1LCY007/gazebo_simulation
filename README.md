

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

### 发送目标点

启动仿真后，在另一个终端运行以下命令为所有机器人发送目标点：

#### 1. 随机目标点

```bash
rosrun unitree_move_base multi_robot_random_goals.py
```

该脚本会为每个机器人在指定范围内随机生成目标点，机器人将自动进行路径规划和避障导航。

#### 2. 基于 Roadmap 的目标点

```bash
rosrun unitree_move_base multi_robot_roadmap_goals.py
```

该脚本使用 roadmap 进行路径规划，从 roadmap 节点中随机选择目标点，然后通过 move_base_flex 的 ExePath action 执行路径。

**参数配置：**
- `--robot_list` - 机器人命名空间列表（默认：自动检测）
- `--roadmap_topic` - Roadmap topic 名称（默认：`/roadmap`）
- `--min_x, --max_x` - X坐标范围（默认：-10.0, 10.0）
- `--min_y, --max_y` - Y坐标范围（默认：-15.0, 15.0）
- `--interval` - 发送目标点的间隔时间（秒，默认：30.0）
- `--once` - 只发送一次目标点，不持续运行

**使用示例：**
```bash
# 自定义坐标范围和发送间隔
rosrun unitree_move_base multi_robot_roadmap_goals.py --min_x -15 --max_x 15 --min_y -20 --max_y 20 --interval 60.0

# 只发送一次目标点
rosrun unitree_move_base multi_robot_roadmap_goals.py --once
```

#### 3. 基于 Hospital 房间位置的目标点

```bash
rosrun unitree_move_base multi_robot_room_goals.py
```

该脚本从 `hospital.yaml` 文件中读取房间位置，随机选择房间作为目标点，然后使用 roadmap 进行路径规划。

**功能特点：**
- 从 hospital.yaml 的 `Regions` 部分读取所有房间位置
- 支持房间类型过滤（如只选择病房、手术室等）
- 使用 roadmap 规划路径，确保路径可达
- 通过 move_base_flex 执行路径

**参数配置：**
- `--robot_list` - 机器人命名空间列表（默认：自动检测）
- `--roadmap_topic` - Roadmap topic 名称（默认：`/roadmap`）
- `--hospital_yaml` - hospital.yaml 文件路径（默认：`config/hospital.yaml`）
- `--interval` - 发送目标点的间隔时间（秒，默认：30.0）
- `--room_types` - 房间类型过滤，用逗号分隔（默认：所有房间）
- `--once` - 只发送一次目标点，不持续运行

**房间类型：**
- `room_patient` - 病房（rp1-rp4）
- `room_icu` - ICU（ri5-ri8）
- `room_consulation` - 诊室（rc1-rc2）
- `room_surgery` - 手术室（rs1-rs2）
- `room_lab` - 化验室（rl）
- `room_waste` - 废弃物处理室（rw）
- `reception` - 挂号窗口
- `exit` - 入口
- `repo` - 仓库（repo1-repo2）

**使用示例：**
```bash
# 使用所有房间
rosrun unitree_move_base multi_robot_room_goals.py

# 只使用病房和手术室
rosrun unitree_move_base multi_robot_room_goals.py --room_types="room_patient,room_surgery"

# 自定义 hospital.yaml 路径和发送间隔
rosrun unitree_move_base multi_robot_room_goals.py --hospital_yaml="/path/to/hospital.yaml" --interval=60.0

# 只发送一次目标点
rosrun unitree_move_base multi_robot_room_goals.py --once
```

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

## Go2 双机器人键盘控制

专门用于同时控制 robot_5 和 robot_6 两个 Go2 机器人的键盘控制脚本。

### 启动 Go2 双机器人键盘控制

```bash
roslaunch scout_teleop go2_dual_teleop_keyboard.launch
```

或者直接运行节点：

```bash
rosrun scout_teleop go2_dual_teleop_keyboard.py
```

### 控制说明

**Robot 5 (Go2) 控制：**
- `w` - 前进
- `s` - 后退
- `a` - 左平移
- `d` - 右平移
- `q` - 左转（逆时针）
- `e` - 右转（顺时针）
- `f` - 停止 robot_5

**Robot 6 (Go2) 控制：**
- `i` - 前进
- `k` - 后退
- `j` - 左平移
- `l` - 右平移
- `u` - 左转（逆时针）
- `o` - 右转（顺时针）
- `h` - 停止 robot_6

**通用控制：**
- `空格` - 停止两个机器人
- `CTRL-C` - 退出程序

### 参数配置

可以通过 launch 文件参数配置：

- `speed` - 线速度（默认：`0.5` m/s）
- `turn` - 角速度（默认：`1.0` rad/s）
- `strafe_speed` - 平移速度（默认：`0.5` m/s）

示例：

```bash
roslaunch scout_teleop go2_dual_teleop_keyboard.launch speed:=0.8 turn:=1.5 strafe_speed:=0.6
```

## 机器人速度监控

用于监控机器人速度信息的工具，可以实时查看命令速度（cmd_vel）和实际速度（odom）的对比。

### 启动速度监控

```bash
rosrun scout_bringup robot_velocity_monitor.py _robot_namespaces:="robot_1,robot_2,robot_3,robot_4,robot_5,robot_6"
```

### 使用方法

- 使用字母键 `z/x/c/v/b/n` 切换要监控的机器人（对应 robot_1 到 robot_6）
- 实时显示当前监控机器人的：
  - 命令速度（cmd_vel）：线速度和角速度
  - 实际速度（odom）：线速度和角速度
  - 速度大小对比
- 按 `CTRL-C` 退出

### 显示信息

监控界面会显示：
- 命令速度（cmd_vel）：发送给机器人的速度命令
- 实际速度（odom）：机器人实际执行的速度
- 速度大小：命令速度与实际速度的对比

## 旋转运动测试

用于测试机器人角速度的脚本，可以通过键盘选择要测试的机器人，并实时监控角速度。

### 启动旋转测试

```bash
rosrun scout_teleop rotation_test.py
```

### 交互式控制

脚本启动后进入交互式模式：

**选择机器人：**
- `z` - 选择 robot_1
- `x` - 选择 robot_2
- `c` - 选择 robot_3
- `v` - 选择 robot_4
- `b` - 选择 robot_5
- `n` - 选择 robot_6

**控制测试：**
- `s` - 开始旋转测试（使用默认参数）
- `空格` - 停止当前测试
- `CTRL-C` - 退出程序

### 参数配置

可以通过命令行参数配置：

- `--angular_velocity` - 角速度（rad/s，默认：`1.0`）
- `--duration` - 持续时间（秒，如果指定则自动运行，否则交互式运行）
- `--direction` - 旋转方向（`left` 或 `right`，默认：`left`）
- `--robot_list` - 机器人命名空间列表，用逗号分隔（默认：所有6个机器人）

### 使用示例

```bash
# 交互式模式（默认）
rosrun scout_teleop rotation_test.py

# 交互式模式，自定义默认角速度
rosrun scout_teleop rotation_test.py --angular_velocity 1.5

# 自动运行模式：左转 1.0 rad/s，持续 10 秒
rosrun scout_teleop rotation_test.py --duration 10

# 自动运行模式：右转 0.8 rad/s，持续 20 秒
rosrun scout_teleop rotation_test.py --direction right --angular_velocity 0.8 --duration 20

# 只测试指定的机器人
rosrun scout_teleop rotation_test.py --robot_list="robot_5,robot_6"
```

### 显示信息

测试过程中会实时显示：
- 当前选中的机器人
- 测试状态（运行中/空闲）
- 命令角速度（cmd_vel）和实际角速度（odom）
- 角速度以 rad/s 和 deg/s 两种单位显示
- 命令速度与实际速度的误差
- 所有可用机器人列表

### 应用场景

- 检测机器人的角速度响应性能
- 对比命令角速度与实际角速度的差异
- 测试不同角速度下的机器人行为
- 验证角速度控制精度
- 支持多机器人切换测试

