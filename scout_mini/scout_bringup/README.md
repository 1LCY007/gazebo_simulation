# Scout

TODO:
1. 优化一下scout的输入

    考虑一下，
2. multi_scout启动


2025.1.10
这是现在的启动方式
```bash
roslaunch scout_gazebo_sim hospital_single_mini.launch 
roslaunch scout_navigation hospital_navigation.launch

```
1.在这个包里进行优化。
在move_base加入了teb规划器。现在使用的是teb规划器。（总共有DWA和TEB）


2025.1.12
scout位置的问题
1. amcl是否可以用
2. gazebo直接定位是否可以
