# 多机器人随机目标点发送脚本

## 简介

`multi_robot_random_goals.py` 是一个用于测试多机器人导航系统的工具脚本。它可以自动检测所有可用的机器人，并为每个机器人定期发送随机生成的目标点，用于测试多机器人在移动过程中的避障、路径规划等效果。

## 功能特性

- ✅ **自动检测机器人**：自动扫描 ROS 系统中的所有机器人（通过检测 `/robot_X/move_base_simple/goal` topic）
- ✅ **随机目标点生成**：为每个机器人生成随机坐标和朝向角度
- ✅ **持续运行模式**：定期为所有机器人发送新的随机目标点
- ✅ **单次发送模式**：支持只发送一次目标点后退出
- ✅ **灵活配置**：支持自定义地图范围、发送间隔、机器人列表等参数
- ✅ **多机器人支持**：同时为多个机器人发送不同的随机目标点

## 依赖要求

- ROS Noetic（或其他 ROS 版本）
- Python 3
- 以下 ROS 包：
  - `rospy`
  - `geometry_msgs`
  - `tf` (用于四元数转换)

## 使用方法

### 基本使用

最简单的使用方式，脚本会自动检测所有机器人，每30秒发送一次随机目标点：

```bash
python3 multi_robot_random_goals.py
```

### 指定机器人列表

如果自动检测失败，或者只想为特定机器人发送目标点：

```bash
python3 multi_robot_random_goals.py --robot_list robot_1,robot_2
```

### 自定义地图范围

根据实际地图大小调整目标点的生成范围：

```bash
python3 multi_robot_random_goals.py --min_x -20 --max_x 20 --min_y -10 --max_y 10
```

### 调整发送间隔

改变发送目标点的频率（单位：秒）：

```bash
# 每20秒发送一次
python3 multi_robot_random_goals.py --interval 20.0

# 每60秒发送一次
python3 multi_robot_random_goals.py --interval 60.0
```

### 只发送一次

如果只需要发送一次目标点，不持续运行：

```bash
python3 multi_robot_random_goals.py --once
```

### 禁用随机朝向

如果不需要随机生成朝向角度（yaw始终为0）：

```bash
python3 multi_robot_random_goals.py --random_yaw False
```

### 组合使用

组合多个参数以满足特定需求：

```bash
python3 multi_robot_random_goals.py \
    --robot_list robot_1,robot_2 \
    --min_x -15 --max_x 15 \
    --min_y -10 --max_y 10 \
    --interval 25.0 \
    --random_yaw True
```

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--robot_list` | string | None | 机器人命名空间列表，用逗号分隔。如果未指定，脚本会自动检测所有可用的机器人 |
| `--min_x` | float | -25.0 | X坐标的最小值（米） |
| `--max_x` | float | 25.0 | X坐标的最大值（米） |
| `--min_y` | float | -15.0 | Y坐标的最小值（米） |
| `--max_y` | float | 15.0 | Y坐标的最大值（米） |
| `--interval` | float | 30.0 | 发送目标点的间隔时间（秒）。仅在持续运行模式下有效 |
| `--random_yaw` | bool | True | 是否随机生成朝向角度。如果为 False，yaw 始终为 0 |
| `--once` | flag | False | 如果设置，只发送一次目标点后退出，不持续运行 |

## 使用示例

### 示例 1：测试两个机器人的基本导航

```bash
# 启动多机器人仿真
roslaunch unitree_guide multi_go2_gazeboSim.launch

# 在另一个终端运行脚本
python3 multi_robot_random_goals.py --robot_list robot_1,robot_2 --interval 30.0
```

### 示例 2：快速测试（短间隔，小范围）

```bash
python3 multi_robot_random_goals.py \
    --min_x -5 --max_x 5 \
    --min_y -5 --max_y 5 \
    --interval 10.0
```

### 示例 3：大范围测试（覆盖整个地图）

```bash
python3 multi_robot_random_goals.py \
    --min_x -30 --max_x 30 \
    --min_y -20 --max_y 20 \
    --interval 45.0
```

### 示例 4：单次发送测试

```bash
# 只发送一次目标点，用于快速验证
python3 multi_robot_random_goals.py --once --robot_list robot_1,robot_2
```

## 工作原理

1. **机器人检测**：
   - 脚本启动后，会扫描 ROS 系统中所有已发布的 topic
   - 查找包含 `/move_base_simple/goal` 的 topic
   - 提取命名空间（如 `robot_1`, `robot_2`）
   - 如果自动检测失败，会使用默认列表 `['robot_1', 'robot_2']`

2. **目标点生成**：
   - 在指定的地图范围内随机生成 X、Y 坐标
   - 如果启用随机朝向，yaw 角度在 [-π, π] 范围内随机生成
   - 否则 yaw 为 0

3. **目标点发送**：
   - 为每个机器人创建 `PoseStamped` 消息
   - 发布到对应的 `/robot_X/move_base_simple/goal` topic
   - `move_base` 节点会接收这些目标点并进行路径规划

4. **持续运行**：
   - 在持续运行模式下，脚本会按照设定的间隔时间定期发送新的随机目标点
   - 每次发送都会为所有机器人生成新的随机目标点

## 输出信息

脚本运行时会输出以下信息：

```
[INFO] 正在检测可用的机器人...
[INFO] 检测到机器人: robot_1 (topic: /robot_1/move_base_simple/goal)
[INFO] 检测到机器人: robot_2 (topic: /robot_2/move_base_simple/goal)
[INFO] 检测到 2 个机器人: robot_1, robot_2
[INFO] 为机器人 robot_1 创建发布者: /robot_1/move_base_simple/goal
[INFO] 为机器人 robot_2 创建发布者: /robot_2/move_base_simple/goal
[INFO] 开始持续发送随机目标点，间隔: 30.0 秒
[INFO] 地图范围: x [-25.0, 25.0], y [-15.0, 15.0]

=== 第 1 轮发送随机目标点 ===
[INFO] [robot_1] 发送目标点: x=12.34, y=-8.56, yaw=45.2°
[INFO] [robot_2] 发送目标点: x=-15.78, y=9.23, yaw=-120.5°
```

## 注意事项

1. **地图范围**：
   - 确保设置的地图范围与实际地图大小匹配
   - 如果目标点超出地图范围，`move_base` 可能会规划失败
   - 建议先查看地图文件（如 `hospital.yaml`）了解实际地图范围

2. **机器人命名空间**：
   - 脚本假设机器人命名空间格式为 `robot_X`（如 `robot_1`, `robot_2`）
   - 如果使用不同的命名空间格式，请使用 `--robot_list` 参数手动指定

3. **move_base 状态**：
   - 确保所有机器人的 `move_base` 节点都已启动
   - 如果某个机器人的 `move_base` 未运行，该机器人的目标点会被忽略

4. **发送间隔**：
   - 建议间隔时间不要过短（至少 10 秒），给机器人足够时间到达目标点
   - 如果机器人移动较慢，可以增加间隔时间

5. **坐标系**：
   - 目标点使用 `map` 坐标系
   - 确保 `map` 到 `odom` 的 TF 变换已正确发布

## 故障排除

### 问题 1：未检测到任何机器人

**错误信息**：
```
[ERROR] 未检测到任何机器人！请确保机器人已启动。
```

**解决方案**：
1. 确保所有机器人的 `move_base` 节点已启动
2. 检查机器人命名空间是否正确（应为 `robot_1`, `robot_2` 等）
3. 使用 `rostopic list` 命令查看是否存在 `/robot_X/move_base_simple/goal` topic
4. 使用 `--robot_list` 参数手动指定机器人列表

### 问题 2：目标点发送失败

**可能原因**：
- `move_base` 节点未运行
- topic 名称不正确
- 机器人命名空间不匹配

**解决方案**：
1. 使用 `rostopic echo /robot_X/move_base_simple/goal` 检查 topic 是否存在
2. 使用 `rosnode list` 检查 `move_base` 节点是否运行
3. 查看脚本输出的日志信息，确认发布者创建成功

### 问题 3：机器人不移动

**可能原因**：
- 目标点超出地图范围
- 目标点不可达（被障碍物阻挡）
- `move_base` 配置问题

**解决方案**：
1. 检查目标点是否在地图范围内
2. 在 RViz 中查看全局路径规划是否成功
3. 检查 `move_base` 的日志输出
4. 尝试手动发送一个已知可达的目标点进行测试

### 问题 4：脚本运行缓慢

**可能原因**：
- 机器人数量过多
- 系统资源不足

**解决方案**：
1. 减少同时测试的机器人数量
2. 增加发送间隔时间
3. 检查系统资源使用情况

## 相关文件

- `send_goal.py`：单机器人目标点发送脚本
- `move_base.launch`：move_base 启动文件
- `multi_go2_gazeboSim.launch`：多机器人仿真启动文件

## 许可证

本脚本是 Unitree Go2 机器人仿真项目的一部分。

## 更新日志

- **v1.0** (2024)
  - 初始版本
  - 支持自动检测机器人
  - 支持随机目标点生成
  - 支持持续运行和单次发送模式
