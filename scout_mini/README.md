# Scout_mini_ROS1_Navigation
Implementation of Autonomous driving of scout_mini in gazebo simulator using SLAM+EKF+Point_Cloud+GMAPPING

### **1. Installation**
```
1. mkdir -p ~/scout_ws/src
2. cd ~/scout_ws/src
3. Clone this repository in your src as
   git clone
4. chmod +x install_packages.sh && ./setup.sh
```
   mapping, Navigation, Robot_localization packages will be downloaded
```
5. rosdep install --from-paths src --ignore-src -r -y
6. catkin_make
7. source devel/setup.bash
```
   **rosdep install** command will automatically install the required dependencies for the packages in the workspace. The dependencies are listed in CMakeLists.txt file in the packages.

### **2. Implementation**

1. **Display platform description in RVIZ**
    ```
    In terminal 1: 
    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_description display_scout_mini.launch 
    ```
    This will show you default vehicle platform without additional sensors.
![rviz](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/af6bc104-013a-4f52-b224-ee08d63f186c)

2. **Launch gazebo simulator and teleop control**

    a. Launch gazebo simulator
    ```
    In terminal 1:
    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_gazebo_sim scout_mini_empty_world.launch
    ```
![gazebo](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/0420164b-19c9-41ae-ae81-878f9d0f2172)

    b. Run teleop controller (move: w, a, x, d / stop: s)

    ```
    //Open another terminal
    In Terminal 2:
    
    cd ~/scout_ws
    source devel/setup.bash
    roslaunch scout_teleop scout_teleop_key.launch 
    ```

### **3. 2D SLAM**

Before running any command below, source devel/setup.bash

0. **Run Simulator**

    ```
    In Terminal 1:
    roslaunch scout_gazebo_sim scout_mini_playpen.launch
    ```
![gazebo_slam_scout_mini](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/02a92a1f-ff7e-4cd8-9950-e92fe1fe0fd5)

1. **Odometry & Kalman Filter Localization**
    ```
    In Terminal 2:
    roslaunch scout_base scout_mini_base.launch
    ```
    ```
    In Terminal 3:
    roslaunch scout_filter ekf_filter_cmd.launch
    ```

2. **SLAM mapping**

    a. Gmapping

    ```
    In Terminal 4:
    // Run gmapping slam
    roslaunch scout_slam scout_slam.launch
    ```
![rviz_slam_scout_mini](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/c5a88abc-d917-483a-8324-e82d4580c1b4)

    b. Drive & mapping

    ```
    // Drive via teleop
    In terminal 5:
    roslaunch scout_teleop scout_teleop_key.launch
    ```

    c. Save map

    ```
    // Save map
    In Terminal 6:
    roslaunch scout_slam gmapping_save.launch
    ```
    - default map file name: map1
    - map file will be saved in "scout_bringup/scout_slam/maps"
    - you can change saved map file name in the launch file
    - or you can set file name
        ```
        roslaunch scout_slam gmapping_save.launch map_file:=(file name)
        ```
![slam_mymap](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/af47398a-2c7d-4257-824d-7703aca189a9)


Now close everything.

### 3. **2D Navigation**

Before running any command below, source devel/setup.bash

0. **Run Simulator**

    ```
    In terminal 1: 
    roslaunch scout_gazebo_sim scout_mini_playpen.launch
    ```
![gazebo_scout_mini](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/250f8a87-3ecb-4974-a600-f804028d9036)

1. **Odometry & Kalman Filter Localization**

    ```
    In Terminal 2:
    roslaunch scout_base scout_mini_base.launch
    ```

    ```
    In Terminal 3:
    roslaunch scout_filter ekf_filter_cmd.launch
    ```

2. Navigation

    ```
    // Run navigation
    In Terminal 4:
    roslaunch scout_navigation scout_navigation.launch
    ```
    you can set the destination via "2D Nav Goal" button
   
![rviz_nav_scout_mini](https://github.com/Kazimbalti/Scout_mini_ROS1_Navigation/assets/32608321/4c783656-85c2-4d21-b080-800404d18f69)

### **4. 多机器人键盘遥控**

多机器人键盘遥控功能允许通过键盘同时控制多个机器人，支持在机器人之间快速切换。

#### **使用方法**

1. **启动多机器人仿真**
   ```
   In terminal 1:
   roslaunch scout_bringup multi_robots_gazeboSim.launch
   ```
   这将启动包含 6 个机器人的 Gazebo 仿真环境。

2. **启动多机器人键盘遥控**
   ```
   In terminal 2:
   roslaunch scout_teleop multi_robot_teleop_keyboard.launch
   ```
   或者直接运行节点：
   ```
   rosrun scout_teleop multi_robot_teleop_keyboard.py _robot_namespaces:="robot_1,robot_2,robot_3,robot_4,robot_5,robot_6"
   ```

#### **控制说明**

**移动控制：**
- `w` - 前进
- `s` - 后退
- `a` - 左转
- `d` - 右转
- `j` - 逆时针旋转
- `l` - 顺时针旋转
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

#### **参数配置**

可以通过 launch 文件或命令行参数配置：

- `robot_namespaces` - 机器人命名空间列表（默认：`robot_1,robot_2,robot_3,robot_4,robot_5,robot_6`）
- `speed` - 线速度（默认：`0.5` m/s）
- `turn` - 角速度（默认：`1.0` rad/s）

示例：
```xml
<launch>
  <node pkg="scout_teleop" type="multi_robot_teleop_keyboard.py" name="multi_robot_teleop_keyboard" output="screen">
    <param name="robot_namespaces" value="robot_1,robot_2,robot_3"/>
    <param name="speed" value="0.5"/>
    <param name="turn" value="1.0"/>
  </node>
</launch>
```

### **5. 动态机器人障碍物发布器（Robot Obstacle Publisher）**

**robot_obstacle_publisher** 是一个 Python 节点，用于实时发布其他机器人的位置信息作为障碍物点云数据到本地和全局 costmap，从而实现**多机器人动态碰撞回避**。

#### **功能说明**

1. **订阅 Gazebo 机器人状态**：从 `/gazebo/model_states` 获取所有机器人的真实位置和速度
2. **计算相邻机器人**：检查其他机器人是否在设定的距离阈值内
3. **发布点云数据**：将相邻机器人的轮廓转换为 PointCloud2 格式，发布给 costmap
4. **支持多坐标系**：
   - **local costmap**：相对于机器人的 `odom` 坐标系（话题：`robot_obstacles`）
   - **global costmap**：相对于全局 `map` 坐标系（话题：`robot_obstacles_global`）

#### **工作原理**

```
Gazebo /gazebo/model_states
    ↓
robot_obstacle_publisher.py (获取其他机器人位置)
    ↓
    ├─→ 发布 PointCloud2 (local costmap)  → /robot_X/robot_obstacles
    └─→ 发布 PointCloud2 (global costmap) → /robot_X/robot_obstacles_global
        ↓
     obstacle_layer (costmap_2d)
        ↓
     move_base_flex (规划器和控制器获得最新的障碍物信息)
```

#### **配置参数**

在 launch 文件中配置以下参数：

```xml
<node pkg="scout_navigation" type="robot_obstacle_publisher.py" name="robot_obstacle_publisher" output="screen" ns="$(arg robot_namespace)">
  <!-- 机器人当前命名空间 -->
  <param name="robot_namespace" value="$(arg robot_namespace)"/>
  
  <!-- 相邻机器人检测距离阈值（米），超过此距离的机器人不会被发布 -->
  <param name="distance_threshold" value="3.0"/>
  
  <!-- 机器人碰撞半径（米） -->
  <param name="robot_radius" value="0.4"/>
  
  <!-- 发布频率（Hz） -->
  <param name="publish_rate" value="30.0"/>
  
  <!-- 机器人足迹（矩形四个角的坐标，相对于机器人中心） -->
  <param name="footprint" value="[[-0.32, -0.31], [-0.32, 0.31], [0.32, 0.31], [0.32, -0.31]]"/>
  
  <!-- 系统中所有机器人的命名空间列表 -->
  <param name="robot_namespaces" value="robot_1,robot_2,robot_3,robot_4,robot_5,robot_6"/>
</node>
```

#### **参数说明**

| 参数 | 说明 | 默认值 | 备注 |
|------|------|--------|------|
| `robot_namespace` | 当前机器人的命名空间 | "/" | 用于 TF 坐标系的前缀 |
| `distance_threshold` | 相邻机器人的检测距离（米） | 3.0 | 超过此距离的机器人不会被发布 |
| `robot_radius` | 机器人碰撞半径（米） | 0.4 | 用于计算碰撞检测 |
| `publish_rate` | 发布频率（Hz） | 30.0 | 数值越大越流畅，但 CPU 占用更高 |
| `footprint` | 机器人足迹（米） | `[[-0.32, -0.31], [-0.32, 0.31], [0.32, 0.31], [0.32, -0.31]]` | Scout 标准足迹（1×0.62 m） |
| `robot_namespaces` | 所有机器人的命名空间列表 | `robot_1,...,robot_6` | 逗号分隔，无空格 |

#### **Costmap 配置**

该节点发布的数据需要在 costmap 配置中添加 `robot_obstacles` 观测源：

**Local Costmap:**
```yaml
obstacle_layer:
  observation_sources: scan robot_obstacles
  robot_obstacles:
    topic: robot_obstacles              # 本地话题（相对命名空间）
    data_type: PointCloud2
    sensor_frame: base_footprint        # 相对于机器人基座
    marking: true                       # 标记障碍物
    clearing: false                     # 不清除
    expected_update_rate: 0.0           # 按需发布
    observation_persistence: 0.5        # 0.5秒后过期
```

**Global Costmap:**
```yaml
obstacle_layer:
  observation_sources: scan robot_obstacles_global
  robot_obstacles_global:
    topic: robot_obstacles_global       # 全局话题
    data_type: PointCloud2
    sensor_frame: base_footprint        # 相对于机器人基座
    marking: true                       # 标记障碍物
    clearing: false                     # 不清除
    expected_update_rate: 0.0           # 按需发布
    observation_persistence: 0.5        # 0.5秒后过期
```

#### **RVO2 碰撞回避整合**

该节点与 **RVO2 速度融合节点** 配合使用，实现多机器人最优倒易碰撞回避（ORCA）：

1. **Local Planner**（move_base_flex）使用 costmap 信息生成安全的目标速度
2. **RVO2 节点** 进一步优化速度，确保不与其他机器人发生碰撞
3. 最终安全的速度命令发送到 `/cmd_vel`

#### **调试建议**

```bash
# 1. 查看发布的点云数据
rosopic echo /robot_1/robot_obstacles

# 2. 在 RVIZ 中可视化点云
# 添加 PointCloud2 显示：
# - Topic: /robot_1/robot_obstacles（本地坐标系）
# - Topic: /robot_1/robot_obstacles_global（全局坐标系）

# 3. 查看 costmap 中的障碍物
# 添加 Costmap 显示，观察其他机器人是否被正确标记为障碍物

# 4. 检查日志
rosnode info /robot_1/robot_obstacle_publisher
```
