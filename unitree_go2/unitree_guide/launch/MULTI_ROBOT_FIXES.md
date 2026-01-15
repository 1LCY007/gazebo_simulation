# 多机器人 Launch 文件修复总结

## 已修复的问题

### 1. ✅ Gazebo 插件 robotNamespace 改为可配置
**文件：** `go2_description/xacro/gazebo.xacro`
- 从硬编码 `/go2_gazebo` 改为使用 `$(arg robot_namespace)` 参数
- 现在可以通过 xacro 参数传递不同的 namespace

### 2. ✅ 控制器配置 namespace 匹配
**文件：** `go2_bringup.launch`
- 使用 `robot_control_ns.yaml`（无顶层 namespace）
- 在 `<group ns="$(arg robot_gazebo_ns)">` 下加载，确保参数路径正确
- 控制器在 `robot_0/go2_gazebo` namespace 下运行

### 3. ✅ spawn_model 的 model 名称修复
**文件：** `go2_bringup.launch` 第33行
- 从 `$(arg ns)/go2_gazebo` 改为 `$(arg ns)_go2_gazebo`
- Gazebo 模型名称不能包含 `/`，使用下划线替代

### 4. ✅ robot_state_publisher 添加 joint_states remap
**文件：** `go2_bringup.launch` 第46行
- 添加了 `<remap from="joint_states" to="$(arg robot_gazebo_ns)/joint_states"/>`
- 确保能正确订阅控制器发布的 joint_states

### 5. ✅ robot_name 参数设置
**文件：** `go2_bringup.launch` 第50行
- 设置为 `$(arg ns)/go2`（例如 `robot_0/go2`）
- `IOROS.cpp` 会提取 `go2`，然后构建 `/robot_0/go2_gazebo/...` topic
- 注意：`IOROS.cpp` 使用绝对路径，所以需要包含完整的 namespace

## 当前配置结构

### 对于 robot_0：
- **Gazebo 模型名称：** `robot_0_go2_gazebo`
- **Gazebo 插件 namespace：** `robot_0/go2_gazebo`
- **控制器 namespace：** `robot_0/go2_gazebo`
- **控制器参数路径：** `robot_0/go2_gazebo/FL_calf_controller/...`
- **Topic 路径：** `/robot_0/go2_gazebo/FL_calf_controller/command`
- **TF 前缀：** `robot_0/trunk`, `robot_0/FL_hip`, etc.
- **节点 namespace：** `robot_0/junior_ctrl`, `robot_0/robot_state_publisher`

### 对于 robot_1（如果启用）：
- **Gazebo 模型名称：** `robot_1_go2_gazebo`
- **Gazebo 插件 namespace：** `robot_1/go2_gazebo`
- **控制器 namespace：** `robot_1/go2_gazebo`
- **控制器参数路径：** `robot_1/go2_gazebo/FL_calf_controller/...`
- **Topic 路径：** `/robot_1/go2_gazebo/FL_calf_controller/command`
- **TF 前缀：** `robot_1/trunk`, `robot_1/FL_hip`, etc.
- **节点 namespace：** `robot_1/junior_ctrl`, `robot_1/robot_state_publisher`

## 注意事项

1. **IOROS.cpp 使用绝对路径：** 代码中使用 `"/" + _robot_name + "_gazebo/..."`，所以 topic 是全局的，不依赖节点 namespace
2. **需要确保 robot_name 包含完整路径：** 例如 `robot_0/go2`，这样提取后是 `go2`，构建的 topic 是 `/robot_0/go2_gazebo/...`
3. **多机器人时每个需要独立的 namespace：** 确保 `ns` 参数不同（`robot_0`, `robot_1`, `robot_2`）

## 待验证

1. 测试多机器人启动是否正常工作
2. 验证 topic 路径是否正确匹配
3. 验证 TF tree 是否正确隔离
4. 验证控制器是否能正确接收命令
