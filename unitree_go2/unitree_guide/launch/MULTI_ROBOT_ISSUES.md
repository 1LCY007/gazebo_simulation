# 多机器人 Launch 文件问题分析

## 发现的问题

### 1. **Gazebo 插件 robotNamespace 硬编码** ❌
**位置：** `go2_description/xacro/gazebo.xacro` 第6行
```xml
<robotNamespace>/go2_gazebo</robotNamespace>
```
**问题：** 所有机器人都会使用相同的 `/go2_gazebo` namespace，导致冲突

### 2. **控制器配置 namespace 不匹配** ❌
**位置：** `go2_bringup.launch` 第24行和第31行
- `robot_control.yaml` 顶层是 `go2_gazebo:`
- `controller_spawner` 在 `robot_0/go2` namespace 下运行
- 参数路径应该是 `robot_0/go2/go2_gazebo/...` 但实际是 `go2_gazebo/...`

### 3. **spawn_model 的 model 名称格式错误** ❌
**位置：** `go2_bringup.launch` 第27行
```xml
-model $(arg ns)/go2_gazebo
```
**问题：** Gazebo 模型名称不能包含 `/`，应该使用下划线或直接使用名称

### 4. **robot_state_publisher 缺少 joint_states remap** ❌
**位置：** `go2_bringup.launch` 第38-41行
**问题：** 控制器在 `robot_0/go2` namespace 下，但 `robot_state_publisher` 在 `robot_0` namespace 下，需要 remap

### 5. **多机器人启动时缺少 namespace 隔离** ⚠️
**位置：** `multi_go2_gazeboSim.launch`
- go2_1 和 go2_2 的启动代码被注释掉了
- 如果启用，需要确保每个机器人有独立的 namespace

## 正确的多机器人配置应该：

1. **每个机器人有独立的 Gazebo 模型名称**：`robot_0_go2_gazebo`, `robot_1_go2_gazebo`
2. **每个机器人有独立的控制器 namespace**：`robot_0/go2`, `robot_1/go2`
3. **每个机器人有独立的 topic**：`robot_0/go2/FL_calf_controller/command`
4. **每个机器人有独立的参数**：`robot_0/go2/go2_gazebo/FL_calf_controller/...`
5. **每个机器人有独立的 TF 前缀**：`robot_0/trunk`, `robot_1/trunk`
