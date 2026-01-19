

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

------

## 2. Multi-Robot Simulation (TODO)

Multi-robot simulation support is under development.

```bash
roslaunch unitree_guide multi_gazeboSim.launch
```

------

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

### 多机scout
为了方便，目前两个主文件均在hospital_simulation文件夹下

1. 启动文件
```bash
roslaunch hospital_simulation hospital_multi_mini.launch
```
2. 单机配置文件

    **hospital_simulation** 文件夹下**spawn_mini_simple.launch**
3. 其他配置
   
    **Scout_mini_ROS1_Navigation/scout_bringup/scout_navigation/launch**文件下下，amcl与move_base

    **amcl_multi.launch** 和 **move_base_multi.launch**


4. move_base发点测试
```bash
rosrun hospital_simulation send_goal.py 
```
5. spawn_mini_simple.launch 

   前者是amcl，来获取odom-map

   后者是直接获取gazebo中base_footprint到map的位置，进而反推odom-map。

   都可以尝试，但二者都有问题，且问题基本相似。
```bash
        <include file="$(find scout_navigation)/launch/amcl_multi.launch">
            <arg name="robot_namespace" value="$(arg robot_namespace)" />
            <arg name="scan_topic" value="$(arg scan_topic)" />
            <arg name="initial_pose_x" value="$(arg x)" />
            <arg name="initial_pose_y" value="$(arg y)" />
            <arg name="initial_pose_a" value="$(arg yaw)" />
            <arg name="odom_model_type" value="$(arg odom_model_type)"/>
            <arg name="odom_alpha1" value="$(arg odom_alpha1)"/>
            <arg name="odom_alpha2" value="$(arg odom_alpha2)"/>
            <arg name="odom_alpha3" value="$(arg odom_alpha3)"/>
            <arg name="odom_alpha4" value="$(arg odom_alpha4)"/>
        </include>

        <!-- <node pkg="scout_navigation" type="gazebo_true_odom.py" name="gazebo_true_odom">
            <param name="model_name" value="$(arg robot_namespace)"/>
            <param name="base_frame" value="$(arg robot_namespace)/$(arg base_frame)"/>
            <param name="odom_frame" value="$(arg robot_namespace)/$(arg odom_frame)"/>
            <param name="map_frame" value="/map"/>
            <param name="rate" value="10.0"/>
        </node> -->
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

