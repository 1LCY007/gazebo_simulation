

# Gazebo_Simulation

This project provides a Gazebo-based simulation environment for testing
**multi-type robots** using ROS.

Currently, the simulation supports **Unitree Go2**.

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